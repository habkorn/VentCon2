# Plan: Hard Upper Limits for PID Gains (P1.4)

Rolle: Umsetzungsplan (Detaildokument, Skelett)
Grundlage: `../CRA_Compliance_Checklist.md`
Maßnahme: P1.4
CRA-Bezug: Annex I, Teil I, 1.7 (Pflicht-Nr. 4)
Risikobezug: R6
Risikoanalyse: `../analyse/2026-08-27_Webinterface_Sicherheitsanalyse.md`
Abhängigkeiten: P1.1 (Authentifizierung) verkleinert die Angriffsfläche, ersetzt aber keine harten Grenzen
Datum: 2026-09-28
Hinweis: Skelett. Abschnitt 2 ist verifiziert, die Abschnitte 3 bis 6 sind zu füllen.

## 1. Objective

Bound the PID gains and their slider limits with hard coded ceilings, so that no combination of
web requests, serial commands or a hand-edited `settings.json` can produce a controller that
oscillates or hammers the valve. The setpoint already has such a ceiling, the gains do not.

## 2. Current State (verified 2026-09-28)

- `handleSet()` validates each gain against `settings->kp_limits`, `ki_limits` and `kd_limits`
  (`src/WebHandlers.cpp`, gain block around line 502).
- Those limits are themselves writable through `POST /api/slider-limits`
  (`src/WebHandlers.cpp`, `handleSliderLimits`) and are only checked for `max > min` and `step > 0`.
  Any ceiling can therefore be raised without bound.
- The serial commands KP, KI and KD use the same limit fields
  (`src/CommandProcessor.cpp`, `handlePIDCommands`).
- Defaults come from `SliderDefaults` (`src/SettingsHandler.cpp`, `initialize()`; `src/Constants.h`).
- The setpoint is already protected by the pattern to be mirrored here:
  `min({s.sp_limits.max, s.sensor_max_pressure, OperatingDefaults::HARD_SP_MAX_BAR})`.

## 3. Design (TODO)

- Add hard ceiling constants next to `OperatingDefaults::HARD_SP_MAX_BAR`.
- Enforce the ceiling in three places: request validation (`handleSet`), slider limit update
  (`handleSliderLimits`), and load-time clamping (`SettingsHandler::load`) so that a manipulated
  `settings.json` cannot bypass it.
- Decide whether the auto tuner writes through the same clamp (`src/AutoTuner.cpp`, tuning setters).

## 4. Implementation Steps (TODO)

## 5. Verification (TODO)

- Set a slider limit far above the ceiling through `/api/slider-limits` and confirm rejection.
- Edit `settings.json` by hand, reboot, and confirm the value is clamped on load.
- Confirm a legitimate tuning session is not restricted by the ceiling.

## 6. Open Items

- The numeric ceilings need control engineering judgement (10 bar range, valve type, sampling rate).
- Relationship to the bounds used by `AutoTuneConfig`.
