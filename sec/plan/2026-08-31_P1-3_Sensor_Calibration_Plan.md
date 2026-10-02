# Plan: Protect Sensor Calibration Against Safety Bypass (P1.3)

Rolle: Umsetzungsplan (Detaildokument)
Grundlage: `../CRA_Compliance_Checklist.md`
Maßnahme: P1.3
CRA-Bezug: Annex I, Teil I, 1.7 (Pflicht-Nr. 9)
Risikobezug: R3
Risikoanalyse: `../analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md`
Datum: 2026-08-31
Note: P1.1 (authentication) removes the unauthenticated attack path; this plan
is defense-in-depth that limits damage even if authentication is bypassed.

## 1. Objective

Hard-cap the sensor calibration values so that no combination of `sensor_maxP`,
`sensor_maxV`, `sensor_minP`, and `sensor_minV` can cause the firmware to silently
underestimate physical pressure and thus suppress the emergency shutdown.

Finding 3 from the security analysis: the emergency shutdown compares the
*reported* pressure (derived from the calibration values) against a safety limit
that is also derived from those same calibration values. An attacker who can write
calibration values via `/set` can craft a combination that keeps reported pressure
below the shutdown threshold while the actual physical pressure exceeds it.

Confirmed decisions:

- `sensor_max_pressure` is capped at `SensorConfigDefaults::SENSOR_MAX_BAR`
  (10.0 bar). Values above 10.0 bar are rejected.
- `sensor_max_voltage` is capped at `SensorConfigDefaults::SENSOR_MAX_VOLTAGE`
  (4.50 V, the datasheet max for the fitted BOSCH MPS1 sensor), not at the ADC
  hardware limit of 5.0 V. Raising maxV above the sensor's physical maximum is the
  primary mechanism that enables an underestimation attack (see section 2.2).
- The same caps are enforced in `SettingsHandler::load()` so that a manually
  edited or corrupted `settings.json` cannot inject out-of-range values on boot.
- Cap constants are placed in `SensorConfigDefaults` in `src/Constants.h`
  (constants already exist; some need addition).
- Calibration changes remain possible through the web interface; authentication
  (P1.1) is the access gate. P1.3 is a value-range constraint, not an access
  constraint.
- Documents in English; code comments in German (project convention).

## 2. Current State

### 2.1 Validation in `handleSet()` (`src/WebHandlers.cpp`, line 555)

The current check is:

```cpp
if (newMinP < newMaxP && newMinV >= 0.0f && newMinV < newMaxV && newMaxV <= 5.0f)
{
    s.sensor_min_pressure = newMinP;
    s.sensor_max_pressure = newMaxP;
    ...
}
```

What is missing:
- No upper bound on `newMaxP`. An attacker can set it to 1000 bar.
- The cap on `newMaxV` uses the ADC hardware limit (5.0 V), not the sensor's
  physical output maximum (4.50 V). This gap is the attack surface described in
  section 2.2.
- No lower bound on `newMaxP` (could be set below minP bypass is caught, but
  a value like 0.01 bar produces a dangerously tight safety window).

### 2.2 The underestimation attack

`calculatePressure()` (`src/SensorManager.cpp`, line 100) uses a linear
interpolation:

```
reported = minP + (maxP - minP) / (maxV - minV) * (voltage - minV)
```

The sensor outputs a physical voltage proportional to the actual pressure.
If `maxV` is set above the sensor's true maximum output voltage (4.50 V), the
slope of the interpolation decreases. The firmware therefore reports a lower
pressure than the actual physical pressure.

Example:
- Sensor outputs 4.50 V at actual 10.0 bar (sensor rated maximum).
- Normal calibration: maxV = 4.50 V, maxP = 10.0 bar → slope = 2.5 bar/V →
  reported = 10.0 bar.
- Attacked calibration: maxV = 4.99 V, maxP = 10.0 bar → slope = 2.22 bar/V →
  reported = 8.9 bar at the same voltage.

`isPressureSafe()` (`src/SensorManager.cpp`, line 160):

```cpp
float operatingLimit = MAX_OPERATING_BAR * SAFETY_MARGIN_FACTOR;  // 11.0 bar
float sensorLimit    = sensor_max_pressure * SAFETY_MARGIN_FACTOR; // 11.0 bar (default)
return filtered_pressure <= min(operatingLimit, sensorLimit);
```

With normal calibration: 10.0 bar (reported) vs. 11.0 bar limit → OK (shutdown
not triggered). But the actual pressure is also 10.0 bar, so this is correct.

With attacked calibration (maxV = 4.99 V): 8.9 bar (reported) vs. 11.0 bar
limit → no shutdown. But actual pressure is 10.0 bar. The safety system is silent.

The `operatingLimit` based on `MAX_OPERATING_BAR` prevents an attacker from
disabling the safety check by inflating `sensor_max_pressure` alone (raising maxP
raises reported pressure proportionally, so the check still trips). The voltage
axis is the soft underbelly.

### 2.3 Load-time enforcement in `SettingsHandler::load()`

`load()` (`src/SettingsHandler.cpp`, line 142) assigns values directly from JSON
with a default fallback:

```cpp
sensor_max_pressure = sensor_lim["maxP"] | SensorConfigDefaults::SENSOR_MAX_BAR;
```

No range clamping is applied after assignment. A manually edited `settings.json`
or a backup file with out-of-range values silently corrupts the calibration at boot.

## 3. Design

### 3.1 New constant: `SENSOR_MAX_VOLTAGE`

`SensorConfigDefaults::SENSOR_MAX_VOLTAGE` already exists (4.50 V, line 76 of
`Constants.h`) and is the per-sensor datasheet maximum for the fitted BOSCH MPS1.
No new constant is needed; the cap just needs to be wired in.

`SensorConfigDefaults::SENSOR_MAX_BAR` (10.0 bar) similarly already exists.

### 3.2 Tightened validation in `handleSet()`

Replace the current sensor calibration block with:

```
newMinP >= SENSOR_MIN_BAR (0.0)
newMaxP <= SENSOR_MAX_BAR (10.0)       ← new upper cap
newMinP < newMaxP
newMinV >= 0.0f
newMinV < newMaxV
newMaxV <= SENSOR_MAX_VOLTAGE (4.50)   ← tightened from 5.0 to 4.50
```

All six conditions must hold; any single failure rejects the entire update.

Note: Tightening `newMaxV` to 4.50 V means a user cannot calibrate a different
sensor with a higher output voltage through the web UI. That is intentional: this
firmware is built for the BOSCH MPS1. If a different sensor is ever fitted,
`SENSOR_MAX_VOLTAGE` must be updated in `Constants.h` and the firmware rebuilt.
This tradeoff is noted in the comment.

### 3.3 Clamping in `SettingsHandler::load()`

After reading the four sensor calibration fields from JSON, apply
`constrain()`/`max()`/`min()` to enforce the same bounds:

```
sensor_min_pressure = max(SensorConfigDefaults::SENSOR_MIN_BAR, loaded_value)
sensor_max_pressure = min(SensorConfigDefaults::SENSOR_MAX_BAR, loaded_value)
sensor_min_voltage  = max(0.0f, loaded_value)
sensor_max_voltage  = min(SensorConfigDefaults::SENSOR_MAX_VOLTAGE, loaded_value)
```

Additionally: if after clamping `sensor_min_pressure >= sensor_max_pressure` or
`sensor_min_voltage >= sensor_max_voltage`, reset the entire calibration block to
factory defaults and log a warning (`LOG_W(CAT_SETTINGS, ...)`). This handles
corrupt or adversarially crafted files.

### 3.4 Relationship to `isPressureSafe()`

`isPressureSafe()` already has a correct dual-threshold design that prevents
inflated `sensor_max_pressure` from disabling the safety check. After this plan
is applied, the voltage-axis attack is also closed. No change to `isPressureSafe()`
itself is needed.

## 4. Implementation Steps

Phase A - `src/WebHandlers.cpp`:
1. In the sensor calibration validation block (line 555), replace `newMaxV <= 5.0f`
   with `newMaxV <= SensorConfigDefaults::SENSOR_MAX_VOLTAGE` and add
   `newMaxP <= SensorConfigDefaults::SENSOR_MAX_BAR`.
2. Add a short comment explaining why 4.50 V is used instead of the ADC limit.

Phase B - `src/SettingsHandler.cpp`:
3. After the four sensor calibration assignments in `load()`, add clamping and
   coherence check as described in section 3.3.
4. Log a `LOG_W` if any clamp fires.

Phase C - Data and version:
5. Verify `data/settings.json` and `data/default.json`: `"maxP": 10.0` and
   `"maxV": 4.5` (both already within limits; no data change expected).
6. `CHANGELOG.md` entry; version bump (can ship as a patch on top of P1.1/P1.2
   or independently, TBD).

## 5. Verification

### 5.1 Static

- `pio run` without warnings.
- Grep: no `<= 5.0f` or `<= 5.0` in the sensor calibration validation block.
- `load()` contains clamping for all four sensor fields.

### 5.2 Functional (curl against 192.168.4.1 after P1.1 is active)

Authentication required for all these tests.

| Check | Expected result |
|---|---|
| POST `/set?sensor_maxP=50.0&sensor_maxV=4.5` | 200 but value not applied (maxP > 10.0 rejected) |
| POST `/set?sensor_maxV=4.99&sensor_maxP=10.0` | 200 but value not applied (maxV > 4.50 rejected) |
| POST `/set?sensor_maxP=9.0&sensor_maxV=4.3` | 200, value applied (within bounds) |
| POST `/set?sensor_minP=5.0&sensor_maxP=3.0` | 200 but value not applied (minP > maxP) |
| Craft `settings.json` with `"maxP": 50.0`, reboot | value clamped to 10.0 at load; LOG_W visible on serial |
| Craft `settings.json` with `"maxV": 4.99`, reboot | value clamped to 4.50 at load |
| Craft `settings.json` with `"minP": 9.0`, `"maxP": 1.0`, reboot | coherence check fails; factory defaults restored; LOG_W |

### 5.3 Safety regression

With the attack from section 2.2 attempted after this fix:
- `POST /set?sensor_maxV=4.99` is rejected (maxV > 4.50).
- Manual `settings.json` edit with maxV=4.99: clamped to 4.50 at boot.
- Reported pressure at 10.0 bar actual = 10.0 bar; `isPressureSafe()` triggers
  at reported > 11.0 bar as intended.

### 5.4 Normal operation regression

- Existing `settings.json` with `"maxV": 4.5` and `"maxP": 10.0`: no clamping,
  no change in behavior.
- Save and reload from web UI within valid range: round-trips correctly.

## 6. Risks

- **Sensor replacement**: tightening maxV to 4.50 V breaks calibration for sensors
  with a different output range. This is documented as a `Constants.h` change
  requirement. Consider adding a compile-time `static_assert` that
  `SENSOR_MAX_VOLTAGE <= SENSOR_MAX_VOLTAGE_LIMIT` (the ADC cap) to prevent
  accidental misconfiguration.
- **Firmware update on a device with maxV already set to a value between 4.50 and
  5.0 V**: on the next boot, `load()` will clamp maxV to 4.50 V and log a warning.
  This is the correct behavior (the value was outside the safe range) but operators
  should be informed in release notes.
- **`LOG_W` category**: confirm that `CAT_SETTINGS` is the correct log category
  for settings load events; adjust to whichever category `load()` already uses.
