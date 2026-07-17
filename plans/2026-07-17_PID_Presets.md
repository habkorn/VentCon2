# Plan: PID Presets — Save & Load Named Gains

**Date:** 2026-07-17

## Decisions

- **Q1 (max presets):** A — 5 presets maximum
- **Q2 (name collision):** A — Overwrite silently
- **Q3 (save workflow):** A — "Save Current" snapshot (one-click)
- **Q4 (default presets):** C — Ship one "Factory Default" preset with current default.json Kp/Ki/Kd
- **Q5 (UI location):** C — Modal dialog (like Chart Settings), opened from a "Presets" button
- **Snackbar:** Loading a preset fills sliders and shows "Apply Changes" snackbar — gains don't take effect until user clicks Apply

---

## Data Structure

### settings.json — new `presets` key

```json
"presets": [
  { "name": "Factory Default", "Kp": 9000.0, "Ki": 10000.0, "Kd": 0.0 }
]
```

Array of up to 5 objects. Name is unique (used as key for overwrite).

### C++ struct

```cpp
struct Preset { char name[32]; double Kp; double Ki; double Kd; };
Preset presets[5];  // MAX_PRESETS = 5
uint8_t preset_count;  // track how many are filled
```

Load/save via ArduinoJson `JsonArray` iteration — same pattern as `chart_settings`.

### Defaults

- `Constants.h`: `PRESET_DEFAULTS` with the single "Factory Default" preset using current `SettingsDefaults::KP/KI/KD`
- `default.json`: ships with the same single preset

---

## API Endpoints

| Route | Method | Body (form-encoded) | Response |
|---|---|---|---|
| `/api/presets` | GET | — | `{"presets":[{"name":"...","Kp":...,"Ki":...,"Kd":...},...]}` |
| `/api/presets/save` | POST | `name`, `Kp`, `Ki`, `Kd` | `{"success":true}` or `{"success":false,"message":"Preset list full"}` |
| `/api/presets/load` | POST | `name` | `{"success":true,"Kp":...,"Ki":...,"Kd":...}` |
| `/api/presets/delete` | POST | `name` | `{"success":true}` |

---

## UI — Modal Dialog (Q5=C)

### Trigger

A "Presets" button added to the PID section header row (alongside the existing "Reset PID" button) in `WebContentInputs.h`.

### Modal (added to `WebContentBody.h`)

Structure following the Chart Settings modal pattern:

```
┌─ Presets ───────────────────────── × ─┐
│                                        │
│  Save Current Gains as Preset:         │
│  [__________name___________] [Save]    │
│                                        │
│  ── Saved Presets ──────────────────  │
│  Factory Default  Kp:9000 Ki:10000    │
│  Kd:0             [Load] [✕ Delete]   │
│  ───────────────────────────────────  │
│  Custom Tuning    Kp:15000 Ki:8000    │
│  Kd:100           [Load] [✕ Delete]   │
│  ───────────────────────────────────  │
│                                        │
│              [Close]                   │
└────────────────────────────────────────┘
```

### JavaScript flow

1. **Open modal**: `openPresetsModal()` — fetches `/api/presets` (GET), renders list
2. **Save**: `savePreset()` — reads name input + current slider values, POSTs to `/api/presets/save`, re-renders list
3. **Load**: `loadPreset(name)` — POSTs `/api/presets/load`, gets Kp/Ki/Kd, sets slider/text values via `cachedElements`, calls `showSaveSnackbar('kp', ...)` etc. for all three params so "Apply Changes" appears
4. **Delete**: `deletePreset(name)` — POSTs `/api/presets/delete`, re-renders list
5. **Close modal**: `closePresetsModal()`

---

## Implementation Phases

### Phase 1 — C++ Data Layer

1. Add `Preset` struct and `MAX_PRESETS` to `Constants.h`
2. Add `presets[]` array + `preset_count` to `SettingsHandler.h`
3. Implement load/save for `presets` JSON array in `SettingsHandler.cpp`
4. Add default "Factory Default" preset initialization
5. Update `default.json` with the single preset

### Phase 2 — C++ API Routes

6. Add `/api/presets` GET handler — serialize presets to JSON
7. Add `/api/presets/save` POST handler — create/overwrite preset
8. Add `/api/presets/load` POST handler — return one preset
9. Add `/api/presets/delete` POST handler — remove by name
10. Register all four routes in `setupRoutes()`

### Phase 3 — HTML & CSS

11. Add "Presets" button in PID section header (`WebContentInputs.h`)
12. Add Presets modal HTML (`WebContentBody.h`)
13. Add minimal CSS for preset list rows (`styles.h`)

### Phase 4 — JavaScript

14. Add `openPresetsModal()` — fetch presets, render list dynamically
15. Add `savePreset()` — snapshot sliders, POST save
16. Add `loadPreset(name)` — POST load, set sliders, show snackbar
17. Add `deletePreset(name)` — POST delete, re-render
18. Add `closePresetsModal()`
19. Cache modal elements in `cacheElements()`
20. Wire Presets button click handler in `setupEventListeners()`

### Phase 5 — Verify

21. Compile, flash, test: save/load/delete presets, verify snackbar flow, verify settings.json persistence

---

## Relevant Files

- `src/Constants.h` — `MAX_PRESETS`, default preset values
- `src/SettingsHandler.h` — `Preset` struct, array member
- `src/SettingsHandler.cpp` — load/save JSON array
- `src/WebHandlers.cpp` — 4 new API routes
- `src/WebContentInputs.h` — "Presets" button
- `src/WebContentBody.h` — Presets modal HTML
- `src/styles.h` — Preset list CSS
- `src/WebContentScript.h` — Modal open/save/load/delete JS
- `data/default.json` — Factory Default preset
- `data/settings.json` — Runtime presets (managed by server)
