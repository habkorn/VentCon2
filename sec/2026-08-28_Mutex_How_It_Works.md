# How the Settings Mutex Works in VentCon2

Date: 2026-08-28

---

## Part 1 — What is a Mutex? (Basics)

### The problem: shared data

A modern embedded system like VentCon2 runs several pieces of code "at the same time"
(or close enough that they appear simultaneous). These are called **tasks**. VentCon2 has:

- **ControlTask** — runs on CPU core 1 at high priority (1000 Hz). Reads the pressure
  sensor, computes the PID output, and drives the valve PWM.
- **NetworkTask** — runs on CPU core 0 at lower priority. Handles incoming HTTP requests
  from the browser (the web interface).
- **loop()** — the Arduino main loop, also on core 1. Processes serial commands typed into
  a terminal.

All three tasks share one object: `SettingsHandler settings`. This object holds every
tunable value: setpoint, PID gains, sensor calibration, etc.

When one task **reads** a value while another task is **writing** it, the result is
undefined. On a 32-bit processor, a 64-bit value like `double setpoint` requires two
32-bit machine instructions to write. If the CPU core 0 writes the upper 32 bits of
`setpoint` and then the core 1 reads `setpoint` before the lower 32 bits are written,
the reader gets half the old value and half the new one. This is called a **torn read**
and can produce wildly wrong numbers that look valid but are garbage.

```
Core 0 (NetworkTask, writing setpoint = 5.0)   Core 1 (ControlTask, reading setpoint)
─────────────────────────────────────────────   ──────────────────────────────────────
write upper 32 bits of 5.0  ──────────────────────────────────────────────────────────▶
                                                 ← read upper 32 bits of 5.0 (new)
write lower 32 bits of 5.0                       ← read lower 32 bits of ??? (old)
```

The reader now has a value that is neither the old setpoint nor 5.0. It is random noise.
For a pressure control system this is dangerous.

### The solution: a mutex

A **mutex** (short for *mutual exclusion*) is a flag that enforces a rule:

> Only one task at a time may be inside the protected region.

Before entering the protected region, a task must **take** the mutex. If another task
already holds it, the requesting task is put to sleep until the mutex is released.
After the protected region is done, the task **gives** (releases) the mutex.

```
Task A wants to write           Task B wants to read
──────────────────              ──────────────────────
take mutex  ───OK               take mutex  ───BLOCKED (A holds it)
  write field(s)                  (sleeping...)
give mutex  ───                 take mutex  ───OK (woken up)
                                  read field(s)
                                give mutex
```

This prevents any overlap. The read is always consistent.

### Mutex vs semaphore

A **semaphore** is the general concept. It is a counter. A **binary semaphore** can be
0 or 1. A **mutex** is a binary semaphore with one extra feature: it records which task
owns it, which allows the OS to prevent *priority inversion* (a low-priority task
accidentally blocking a high-priority one indefinitely). For protecting shared data, a
mutex is always the right choice.

---

## Part 2 — FreeRTOS Mutexes on the ESP32

The ESP32 Arduino framework uses **FreeRTOS** as its real-time operating system.
FreeRTOS provides the following mutex API:

```cpp
// Create a new mutex. Returns a handle (an opaque pointer).
SemaphoreHandle_t xSemaphoreCreateMutex();

// Take the mutex (lock it). Blocks until it is available.
// portMAX_DELAY means "wait forever".
xSemaphoreTake(handle, portMAX_DELAY);

// Give back the mutex (unlock it).
xSemaphoreGive(handle);

// Destroy the mutex and free its memory.
vSemaphoreDelete(handle);
```

`SemaphoreHandle_t` is just a pointer to a small internal FreeRTOS structure. The actual
locking/unlocking is done in a handful of CPU instructions and takes roughly 1–2 µs on
the ESP32-S3 when there is no contention (no other task waiting).

---

## Part 3 — How VentCon2 Uses the Mutex

### Where the mutex lives

The mutex is a private member of `SettingsHandler` (`src/SettingsHandler.h`):

```cpp
private:
    SemaphoreHandle_t _mutex;
```

It is created in the constructor and deleted in the destructor:

```cpp
SettingsHandler::SettingsHandler()
    : _mutex(xSemaphoreCreateMutex())
{ ... }

SettingsHandler::~SettingsHandler()
{
    if (_mutex) { vSemaphoreDelete(_mutex); _mutex = nullptr; }
}
```

Because the mutex lives inside `SettingsHandler`, it cannot be copied. The class marks
its copy constructor and copy-assignment operator as `= delete` to enforce this.

### The three public access patterns

Rather than exposing `xSemaphoreTake`/`xSemaphoreGive` everywhere, `SettingsHandler`
offers three clean methods. Call sites never see a mutex handle.

---

#### Pattern 1 — `update(fn)` — writing one or more fields

```cpp
template<typename Fn>
void update(Fn fn) {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    fn(*this);
    xSemaphoreGive(_mutex);
}
```

`update` takes a **lambda** (an inline function). It locks the mutex, calls the lambda
with a reference to the settings object, then unlocks. The caller writes fields via the
reference `s`:

```cpp
// Single field — filter strength
settings->update([v](SettingsHandler& s){ s.filter_strength = v; });

// Multiple fields atomically — all three PID gains change together
settings->update([&](SettingsHandler& s){
    s.Kp = newKp; s.Ki = newKi; s.Kd = newKd;
});

// Write AND read back in one lock — needed when the caller wants the new values
// immediately (e.g. to pass to pid->SetTunings)
double kp, ki, kd;
settings->update([&](SettingsHandler& s){
    s.Kp = val;           // write
    kp = s.Kp;            // read back all three while still locked
    ki = s.Ki;
    kd = s.Kd;
});
pid->SetTunings(kp, ki, kd);   // use the captured values outside the lock
```

**Reading the first line piece by piece:**

```
settings->update( [v](SettingsHandler& s){ s.filter_strength = v; } );
//        ──────  ─────────────────────────────────────────────────
//        method  one argument: a lambda (inline anonymous function)
```

**`settings->update(...)`** — calls the `update` method, which takes the mutex, calls
its argument, then releases the mutex. The argument is always a lambda.

**`[v]` — the capture clause** answers: "which outer variables does the lambda need?"
The lambda is defined inside the surrounding function (e.g. `handleSignalProcessingCommands`)
but *called* inside `update`, so it has no automatic access to the outer function's local
variables. The capture clause bridges that gap.

`[v]` means: copy the current value of `v` *into* the lambda at the moment it is created —
before `update` even starts:

```cpp
float v = 0.42f;           // local variable in the outer function
settings->update(
    [v](...){ ... }        // lambda gets its own private copy: v = 0.42
);
v = 999.0f;                // changing the original does NOT affect the copy inside the lambda
```

This matters for thread safety: the mutex is taken *inside* `update`, which happens a
microsecond after the lambda is created. Using `[v]` (copy) instead of `[&v]` (reference)
guarantees the lambda always uses the value that was validated in the outer function, even
if another task were to modify `v` in that tiny gap.

`[&]` (seen in the multi-field examples above) captures *all* local variables by reference —
used when multiple locals need to be written back out of the lambda after it runs
(e.g. `kp`, `ki`, `kd` in the read-back example).

**`(SettingsHandler& s)` — the parameter** — receives `*this`, passed by `update` as
`fn(*this)`. Writing `s.filter_strength` is identical to `settings->filter_strength`,
but it executes while the mutex is held.

**`{ s.filter_strength = v; }` — the body** — the actual work: assign the captured copy
of `v` to the field.

---

#### Pattern 2 — `withLock(fn)` — protected reads of 64-bit fields

```cpp
template<typename Fn>
void withLock(Fn fn) {
    xSemaphoreTake(_mutex, portMAX_DELAY);
    fn();
    xSemaphoreGive(_mutex);
}
```

`withLock` locks the mutex and calls a void lambda (no argument). It is used in two
specific places:

**1. `pid->Compute()` in ControlSystem:**

```cpp
settings->withLock([&]{ pid->Compute(); });
```

Why is this needed? The PID library holds a raw C++ pointer `double* mySetpoint` that
points directly into `settings.setpoint`. Every time `Compute()` runs it reads
`*mySetpoint`, which is `settings.setpoint`. Because `setpoint` is a `double` (64 bits)
and the CPU is 32-bit, this read takes two instructions. Without the lock, the NetworkTask
could write the upper 32 bits of a new setpoint value between those two instructions,
causing the PID to compute with a garbage setpoint.

Holding the settings mutex during `Compute()` prevents the NetworkTask from writing
`setpoint` at the same time.

**2. Reading `setpoint` in AutoTuner::process():**

```cpp
double sp;
settings->withLock([&]{ sp = settings->setpoint; });
```

`process()` runs from the ControlTask and compares the current pressure against the
setpoint. The same torn-read risk applies, so the single 64-bit read is protected.

---

#### Pattern 3 — `snapshotTo(out)` — bulk consistent reads for rendering

```cpp
void SettingsHandler::snapshotTo(SettingsSnapshot& out)
{
    xSemaphoreTake(_mutex, portMAX_DELAY);
    out.Kp = Kp; out.Ki = Ki; out.Kd = Kd;
    out.setpoint = setpoint;
    // ... all other fields ...
    xSemaphoreGive(_mutex);
}
```

`SettingsSnapshot` is a plain struct (no mutex) that mirrors every field of
`SettingsHandler`. `snapshotTo` copies all fields in one locked operation. The result is
a consistent, point-in-time copy.

Used in all HTTP GET handlers that render data:

```cpp
SettingsSnapshot cfg;
settings->snapshotTo(cfg);
// Render the page using cfg.Kp, cfg.setpoint, etc.
// The mutex is long released; no lock is held during the potentially slow page send.
```

Why not just take the lock for the whole page render? Because sending an HTTP response
can take milliseconds. Holding the mutex that long would block the ControlTask every time
a browser loads the page.

---

### `save()` — serialize under lock, write outside

`save()` must read all fields to build a JSON string, then write that string to flash.
Flash I/O is slow (typically 5–20 ms). If the mutex were held during the whole operation,
the ControlTask would stall for up to 20 ms every time the web interface clicks "Apply".

Solution: serialize the JSON into a `String` buffer under the lock (fast, microseconds),
release the lock, then write the buffer to flash:

```cpp
bool SettingsHandler::save()
{
    String jsonBuffer;
    xSemaphoreTake(_mutex, portMAX_DELAY);
    JsonDocument doc;
    doc["Kp"] = Kp;
    // ... populate all fields ...
    size_t len = serializeJson(doc, jsonBuffer);   // fast in-memory operation
    xSemaphoreGive(_mutex);                         // release BEFORE flash write

    // --- mutex is free here ---
    File file = LittleFS.open(SETTINGS_FILE_PATH, "w");
    file.print(jsonBuffer);   // slow flash write, no mutex held
    file.close();
    return true;
}
```

### `load()` — read file outside lock, assign fields under lock

`load()` reads the JSON file from flash (slow), parses it (medium), then assigns all
fields. Only the assignment step needs the lock, because that is the only point where
another task could see a partial state:

```cpp
bool SettingsHandler::load()
{
    // File read and JSON parse happen outside the lock (slow I/O)
    File file = LittleFS.open(SETTINGS_FILE_PATH, "r");
    JsonDocument doc;
    deserializeJson(doc, file);
    file.close();

    // All field assignments happen atomically under lock
    xSemaphoreTake(_mutex, portMAX_DELAY);
    Kp = doc["Kp"] | SettingsDefaults::KP;
    Ki = doc["Ki"] | SettingsDefaults::KI;
    // ... all other fields ...
    xSemaphoreGive(_mutex);
    return true;
}
```

---

## Part 4 — Which Fields Need Protection?

Not every field needs the same care. On the 32-bit Xtensa LX7 (ESP32-S3), reads and
writes of 32-bit values are **atomic** — they complete in a single instruction, so they
can never be torn. Only 64-bit values are at risk.

| Field type                       | Width  | Atomic?      | Protection required |
| -------------------------------- | ------ | ------------ | ------------------- |
| `bool`, `int8_t`             | 8 bit  | Yes          | None                |
| `float`, `int`, `uint32_t` | 32 bit | Yes          | None                |
| `double`, `int64_t`          | 64 bit | **No** | Must be under lock  |

In `SettingsHandler` the only 64-bit fields are `Kp`, `Ki`, `Kd`, and `setpoint` (all
`double`). All writes go through `update()`. The two critical read paths (`pid->Compute`
and AutoTuner setpoint comparison) go through `withLock`. Multi-field reads for rendering
go through `snapshotTo`.

Fields like `filter_strength` (`float`), `antiWindup` (`bool`), and `control_freq_hz`
(`int`) are read directly without locking throughout `ControlSystem` and `SensorManager`
because a single-instruction atomic read cannot be torn.

---

## Part 5 — Summary Diagram

```
NetworkTask (Core 0)              ControlTask (Core 1)
─────────────────────────         ────────────────────────────────────
Browser sends /set?kp=1.2         pid->Compute() runs at 1000 Hz
  │                                 │
  ▼                                 ▼
settings->update([&](s){          settings->withLock([&]{
    s.Kp = 1.2;                       pid->Compute();   // reads setpoint
});                               });
  │                                 │
  Mutex taken for ~2 µs             Mutex taken for ~5 µs
  (field write only)                (PID arithmetic only)
  │                                 │
settings->save()                  applyHysteresisCompensation()
  │                                 │  (reads hysteresis, hystAmount — floats, no lock)
  Mutex taken for ~10 µs            │
  (JSON serialization only)
  │
  Flash write (~10 ms, NO mutex)
```

The two tasks can run at full speed. The only time one waits for the other is during
the brief moments when a field is being written or `Compute()` is reading the setpoint.
