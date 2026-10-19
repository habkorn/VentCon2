# Changelog

All notable changes to VentCon2 are documented in this file.

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).
Versioning: [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

---

## [2.7.5] - 2026-10-19

### Added
- `tools/shrink_manual_pdf.py` regenerates the downsampled copy of the user manual that is
  served from LittleFS (`data/VentCon_User_Manual.pdf`).

### Documentation
- User manual, Section 2.1: `PV+` and `PV-` now describe their function (supply potential and
  PWM-switched terminal) and give the mapping for valve connectors marked 1 and 2
  (pin 1 to `PV+`, pin 2 to `PV-`).
- User manual, Section 2.1: new note that the supplied VENTREX coil has no integrated diode and
  works in either polarity, that third-party valves may be polarity sensitive, and that the valve
  must not be connected or disconnected while the unit is powered.
- User manual, Section 2.1: the callout about familiarising oneself with the interface first was
  folded into the intro sentence, so the section still fits on a single page; the terminal block
  figure is scaled to 0.78 of the text width for the same reason.
- Technical documentation, Section 3.1: added "Electrical Connection" describing the low-side
  switching, the default 2 kHz PWM frequency and the 1/2 terminal mapping.

---

## [2.7.3] — 2026-08-28

### Added
- FreeRTOS mutex inside `SettingsHandler` protecting all concurrent access from
  NetworkTask (Core 0) and ControlTask (Core 1).
- `SettingsHandler::update(fn)` — executes a lambda under the settings mutex;
  single entry point for all field writes. C++11-compatible explicit parameter type.
- `SettingsHandler::withLock(fn)` — executes a void lambda under the mutex; used
  for 64-bit `double` reads (`setpoint` in `pid->Compute()` and `AutoTuner::process()`).
- `SettingsHandler::snapshotTo(SettingsSnapshot&)` — atomically copies all fields
  into a POD struct for consistent multi-field reads without holding the lock during
  the (potentially slow) HTTP response send.
- `SettingsSnapshot` POD struct mirroring all `SettingsHandler` fields.
- Sensor voltage-to-pressure conversion diagram added to `resources/`. *(525f68c)*

### Changed
- `save()` serialises to a `String` buffer under the mutex, then writes the buffer
  to LittleFS outside the lock (flash I/O no longer blocks the ControlTask).
- `load()` parses JSON outside the mutex, assigns all fields atomically under the
  mutex (ControlTask never sees a partially-written state).
- `WebHandlers::handleSet()` wraps all parameter mutations in one `update()` call,
  making the entire "Apply Changes" form submission atomic.
- All serial command handlers in `CommandProcessor` use `update()` for field writes.
- HTTP read-only handlers (`handleRoot`, `handleValues`, slider-limits GET,
  chart-settings GET) use `snapshotTo()` for consistent rendering.
- `pid->Compute()` in `ControlSystem` wrapped in `withLock()` to protect the raw
  `double* setpoint` pointer against concurrent NetworkTask writes.
- `AutoTuner::start()` and `stop()` update setpoint and PID gains via `update()`.
- `SettingsHandler` copy constructor and copy-assignment marked `= delete` (mutex
  must not be copied).
- `CHANGELOG.md` moved from `LaTeX/CHANGELOG.md` to project root.

### Fixed
- Torn read of `setpoint` (`double`, 64-bit, non-atomic on 32-bit Xtensa LX7) when
  NetworkTask writes while ControlTask reads inside `pid->Compute()`.

### Security
- Web interface security analysis conducted; full report in
  `sec/2026-08-27_Webinterface_Sicherheitsanalyse.md`. Issues identified:
  no HTTP authentication, hardcoded AP password, sensor calibration writable via
  unauthenticated HTTP (can suppress emergency over-pressure shutdown), CSRF on GET
  endpoints, arbitrary LittleFS file read, unbounded PID gain ranges, flash-wear DoS.

### Documentation
- `sec/2026-08-27_Webinterface_Sicherheitsanalyse.md` — security analysis and
  remediation plan (German).
- `sec/2026-08-27_Mutex_Settings_Zugriff_Plan.md` — mutex implementation plan
  and verification protocol (German).
- `sec/2026-08-28_Mutex_How_It_Works.md` — mutex and semaphore primer for this
  codebase (English).

---

## [2.7.2] — 2026-07-23

### Added
- Explicit client-disconnect guard in `handleFileRead()`: returns immediately when
  the TCP connection drops before the handler runs, preventing `ERR_EMPTY_RESPONSE`.
- Serial warning on LittleFS mount failure with instruction to re-upload the
  filesystem image.
- `plans/2026-07-18_Bottom_Nav_Tab_Architecture.md` — architecture plan for a
  tab-based navigation UI (not yet implemented).

### Changed
- `handleFileRead()` rewritten to use `sendContent()` chunked streaming (the same
  pattern as `handleRoot()`), replacing raw `client().write()` calls that bypassed
  the WebServer library's connection tracking.
- File streaming chunk size 1 KB → 4 KB (reduces PDF loop iterations from ~320 to
  ~80 for the ~320 KB manual).
- `delay(1)` between chunks replaced with `yield()` (lower latency, still sufficient
  for WiFi watchdog).
- LittleFS mounted with `begin(false)` — prevents silent auto-format on corruption
  (previously destroyed all stored files, including PDF and settings, without warning).
- `docs/Upload_Procedure.md` and `docs/Flashing_Virgin_Arduino_Nano_ESP32.md` merged
  into `docs/Upload_Guide.md`; BOOT button labels corrected throughout (B0/B1 → BOOT).
- User manual updated.
- Web UI cleanup: 21 changes across 5 phases — bug fixes, dead-code removal,
  performance improvements. *(4bebd29)*

### Fixed
- PDF manual failing to open with `ERR_EMPTY_RESPONSE`, reliably reproducible when
  the `/values` polling was active concurrently.
- Large file streaming silently dropping bytes under TCP backpressure.
- LittleFS silent auto-format on mount failure destroying filesystem without warning.
- BOOT button label inconsistency (B0/B1) in upload documentation.

---

## [2.7.1] — 2026-07-07

### Changed
- Reverted sensor calibration mapping to 0.5 V → 0 bar(g) and 4.5 V → 10 bar(g).
- `MAX_OPERATING_BAR` reduced from 20 bar to 10 bar, cascading to all UI defaults
  (slider, chart, display range).
- `HARD_SP_MAX_BAR` reduced from 20 bar to 10 bar.
- Safety threshold (`isPressureSafe()`) now triggers at 11 bar (10 × 1.1).

### Documentation
- LaTeX documentation updated to reflect 0–10 bar sensor range.

---

## [2.7.0] — 2026-07-07

### Added
- Configurable display range for the pressure bar visualisation, decoupled from the
  sensor calibration range.
- Firmware-enforced hard setpoint cap (`HARD_SP_MAX_BAR = 20 bar`) independent of
  the UI slider configuration.
- `OperatingDefaults` namespace in `Constants.h` centralising operating-range
  constants.

### Changed
- `SENSOR_MAX_BAR` updated from 10 bar to 100 bar to match the physical sensor.
- Setpoint slider default max reduced from 100 bar to 20 bar (operating range).
- Chart Y-axis default max reduced from 10 bar to 20 bar.
- Auto-tuner max test setpoint capped at 20 bar.
- `isPressureSafe()` now uses the operating-range-based limit (22 bar) instead of
  tracking the full sensor range.

### Fixed
- Pressure bar and setpoint marker scaling now use the configurable display range,
  preventing near-invisible bar fills when operating in a narrow pressure range.
- `SliderDefaults::SP_MAX` decoupled from `SENSOR_MAX_BAR` so changing one no longer
  inadvertently changes the other.

### Documentation
- LaTeX documentation updated to reflect new sensor range (0–100 bar) and operating
  range (0–20 bar).

---

<!-- Pre-2.7.0 entries are reconstructed from commit messages only.
     To fill gaps: git log --oneline <hash-before-16cbb38> -->

## [Pre-2.7.0]

### Changed
- Footer brand title style updated in `WebContentFooter.h`. *(2344fd2)*

### Added
- Agent definition files added to `.github/agents/` for AI-assisted development
  workflow. *(7bdff2d)*

### Other
- Build tooling and command-line configuration updates. *(6e4094b)*
