# Plan: PID Presets — Save & Load Named Gains

**Date:** 2026-07-17

## Decisions

- **Q1 (max presets):** A — 5 presets maximum
- **Q2 (name collision):** A — Overwrite silently (same name = update values)
- **Q3 (save workflow):** A — "Save Current" snapshot (one-click, reads current slider values)
- **Q4 (default presets):** C — Ship one "Factory Default" preset with current default.json Kp/Ki/Kd (9000/10000/0)
- **Q5 (UI layout):** A+C — Option A vertical list inside modal, with scrollable preset area
- **Snackbar:** Loading a preset fills sliders and shows "Apply Changes" snackbar — gains don't take effect until user clicks Apply

---

## Data Structure

### settings.json — new `presets` key

```json
"presets": [
  { "name": "Factory Default", "Kp": 9000.0, "Ki": 10000.0, "Kd": 0.0 }
]
```

Array of up to 5 objects. Name is unique (used as key for overwrite). Empty array `[]` if no user presets beyond Factory Default.

### C++ struct (Constants.h)

```cpp
#define MAX_PRESETS 5

struct Preset {
    char name[32];   // null-terminated, max 31 usable chars
    double Kp;
    double Ki;
    double Kd;
};
```

### SettingsHandler members (SettingsHandler.h)

```cpp
Preset presets[MAX_PRESETS];
uint8_t preset_count;   // 0..5, tracks how many slots are filled
```

### ArduinoJson load pattern (SettingsHandler.cpp)

```cpp
// Load presets array; gracefully handles missing key (empty array default)
JsonArray arr = doc["presets"];
preset_count = 0;
for (JsonObject p : arr) {
    if (preset_count >= MAX_PRESETS) break;
    const char* name = p["name"] | "";
    if (name[0] == '\0') continue;  // skip empty names
    strncpy(presets[preset_count].name, name, 31);
    presets[preset_count].name[31] = '\0';
    presets[preset_count].Kp = p["Kp"] | SettingsDefaults::KP;
    presets[preset_count].Ki = p["Ki"] | SettingsDefaults::KI;
    presets[preset_count].Kd = p["Kd"] | SettingsDefaults::KD;
    preset_count++;
}
```

### ArduinoJson save pattern (SettingsHandler.cpp)

```cpp
// Save presets as JSON array
JsonArray arr = doc["presets"].to<JsonArray>();
for (int i = 0; i < preset_count; i++) {
    JsonObject p = arr.add<JsonObject>();
    p["name"] = presets[i].name;
    p["Kp"] = presets[i].Kp;
    p["Ki"] = presets[i].Ki;
    p["Kd"] = presets[i].Kd;
}
```

### Defaults

- `Constants.h`: `PresetDefaults` namespace with the single "Factory Default" preset using `SettingsDefaults::KP/KI/KD`
- `data/default.json`: ships with the same single preset in `"presets"` array
- Empty `"presets"` array is valid — `load()` handles it gracefully (loop over zero items)

---

## API Endpoints

| Route | Method | Body (form-encoded) | Success Response | Error Response |
|---|---|---|---|---|
| `/api/presets` | GET | — | `{"presets":[...]}` | — |
| `/api/presets/save` | POST | `name`, `Kp`, `Ki`, `Kd` | `{"success":true}` | `{"success":false,"message":"Preset list full"}` or `"Name cannot be empty"` |
| `/api/presets/load` | POST | `name` | `{"success":true,"Kp":...,"Ki":...,"Kd":...,"clamped":false}` | `{"success":false,"message":"Preset not found"}` |
| `/api/presets/delete` | POST | `name` | `{"success":true}` | `{"success":false,"message":"Preset not found"}` |

### Validation rules (server-side)

- **Save**: name trimmed, non-empty, max 31 chars. Kp/Ki/Kd clamped to current slider limits. If name matches existing preset, overwrite silently. If `preset_count == MAX_PRESETS` and no match, reject "Preset list full".
- **Load**: find preset by exact name match. If found, clamp Kp/Ki/Kd to current slider limits. Return `"clamped":true` if any value was clamped. If not found, return 400 "Preset not found".
- **Delete**: find by exact name match, shift remaining presets down, decrement count. If not found, return 400 "Preset not found".

---

## UI — Modal Dialog (Option A with scrollable list)

### Trigger

A **"Presets" button** added to the PID section header row (alongside the existing "Reset PID" button) in `WebContentInputs.h`:

```html
<div class="section-title-row">
  <span class="section-title"><span>PID</span><div></div></span>
  <button id="resetPidBtn" class="reset-btn">Reset PID</button>
  <button id="presetsBtn" class="reset-btn">Presets</button>   <!-- NEW -->
</div>
```

### Modal ASCII

Save row and Close button always visible; preset list is scrollable:

```
┌─ Presets ──────────────────────────── × ─┐
│                                           │
│  Save Current Gains as Preset:            │  ← always visible
│  ┌─────────────────────────┐ ┌──────────┐│
│  │ Preset name...          │ │   Save   ││
│  └─────────────────────────┘ └──────────┘│
│                                           │
│  ─────────── Saved Presets ───────────── │
│  ┌─────────────────────────────────────┐ ┐│
│  │ Factory Default           [✕]       │ ││
│  │ Kp: 9000  Ki: 10000  Kd: 0         │ ││
│  │           [Load]                    │ ││ ← scrollable area
│  ├─────────────────────────────────────┤ ││   (max-height: 220px)
│  │ Fast Response             [✕]       │ ││
│  │ Kp: 15000 Ki: 8000  Kd: 100        │ ││
│  │           [Load]                    │ ││
│  └─────────────────────────────────────┘ ┘│
│                                           │
│                    [Close]                │  ← always visible
└───────────────────────────────────────────┘
```

### HTML structure

```html
<div id="presetsModal" class="modal-overlay">
  <div class="modal-content">
    <div class="modal-header">
      <h3>PID Presets</h3>
      <button class="modal-close" onclick="closePresetsModal()">&times;</button>
    </div>
    <div class="modal-body">
      <!-- Save row — always visible -->
      <div class="modal-row">
        <label for="presetName">Save Current Gains as Preset</label>
        <input type="text" id="presetName" maxlength="31" placeholder="Preset name...">
        <button class="modal-btn modal-btn-save" onclick="savePreset()">Save</button>
      </div>
      <hr>
      <!-- Preset list — scrollable -->
      <div id="presetList" class="preset-list">
        <!-- Dynamically populated by JS -->
      </div>
    </div>
    <div class="modal-footer">
      <button class="modal-btn modal-btn-cancel" onclick="closePresetsModal()">Close</button>
    </div>
  </div>
</div>
```

### CSS (styles.h)

```css
/* Scrollable preset list area */
.preset-list {
  max-height: 220px;
  overflow-y: auto;
  margin-top: 0.5rem;
}

/* Single preset row */
.preset-row {
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  justify-content: space-between;
  padding: 0.5rem 0;
  border-bottom: 1px solid var(--border);
}
.preset-row:last-child {
  border-bottom: none;
}

/* Preset name (bold, left side) */
.preset-name {
  font-weight: 600;
  font-size: 0.85rem;
  min-width: 40%;
}

/* Preset values (Kp/Ki/Kd, compact) */
.preset-values {
  font-size: 0.75rem;
  color: var(--text-light);
  display: flex;
  gap: 0.5rem;
}

/* Preset action buttons */
.preset-actions {
  display: flex;
  gap: 0.375rem;
  margin-top: 0.25rem;
  width: 100%;
  justify-content: flex-end;
}

.preset-load-btn {
  background-color: var(--primary);
  color: white;
  border: none;
  border-radius: 0.25rem;
  padding: 0.2rem 0.75rem;
  font-size: 0.75rem;
  font-weight: 600;
  cursor: pointer;
  min-width: 0;
}
.preset-load-btn:hover {
  background-color: var(--primary-dark);
}

.preset-delete-btn {
  background: none;
  border: none;
  color: var(--danger);
  cursor: pointer;
  font-size: 1rem;
  padding: 0 0.25rem;
  min-width: 0;
  opacity: 0.6;
}
.preset-delete-btn:hover {
  opacity: 1;
}

/* Empty state text */
.preset-empty {
  text-align: center;
  color: var(--text-light);
  font-size: 0.85rem;
  padding: 1rem 0;
}
```

### JavaScript flow

1. **Open modal** (`openPresetsModal`): fetch `GET /api/presets`, render list dynamically into `#presetList`. Show empty-state text if zero presets. Reset name input. Show modal.

2. **Save** (`savePreset`): read `#presetName` value (trimmed), validate non-empty. Read current Kp/Ki/Kd from slider values via `cachedElements.texts.kp.value` etc. POST to `/api/presets/save`. On success, clear name input, re-fetch and re-render list. On error, show alert with server message.

3. **Load** (`loadPreset(name)`): POST `/api/presets/load`. On success, set slider/text values via `cachedElements.sliders.kp.value = data.Kp` etc., then call `showSaveSnackbar('kp', data.Kp)`, `showSaveSnackbar('ki', data.Ki)`, `showSaveSnackbar('kd', data.Kd)` — this triggers the "Apply Changes" snackbar. Close modal. If `data.clamped` is true, the clamped values are shown in the sliders.

4. **Delete** (`deletePreset(name)`): POST `/api/presets/delete`. On success, re-fetch and re-render list. On error, show alert.

5. **Close modal** (`closePresetsModal`): hide `#presetsModal`.

6. **Overlay click-to-close**: same pattern as existing modals — click on `.modal-overlay` background closes.

### Cache elements (cacheElements addition)

```js
presetsModal: document.getElementById('presetsModal'),
presetName: document.getElementById('presetName'),
presetList: document.getElementById('presetList')
```

### Event wiring (setupEventListeners addition)

```js
var presetsBtn = document.getElementById('presetsBtn');
if (presetsBtn) presetsBtn.addEventListener('click', openPresetsModal);
```

---

## Implementation Phases

### Phase 1 — C++ Data Layer

1. Add `Preset` struct and `MAX_PRESETS` to `Constants.h`
2. Add `presets[]` array + `preset_count` to `SettingsHandler.h`
3. Implement load/save for `presets` JSON array in `SettingsHandler.cpp` (see pseudocode above)
4. Add default "Factory Default" preset initialization
5. Update `default.json` with the single preset

### Phase 2 — C++ API Routes

6. Add `/api/presets` GET handler — serialize presets to JSON
7. Add `/api/presets/save` POST handler — create/overwrite preset (with validation)
8. Add `/api/presets/load` POST handler — return one preset (with slider-limit clamping)
9. Add `/api/presets/delete` POST handler — remove by name
10. Register all four routes in `setupRoutes()`

### Phase 3 — HTML & CSS

11. Add "Presets" button in PID section header (`WebContentInputs.h`)
12. Add Presets modal HTML (`WebContentBody.h`) — see HTML structure above
13. Add preset list CSS rules (`styles.h`) — see CSS above

### Phase 4 — JavaScript

14. Add `openPresetsModal()` — fetch presets, render list dynamically
15. Add `savePreset()` — snapshot sliders, POST save
16. Add `loadPreset(name)` — POST load, set sliders, show snackbar ×3
17. Add `deletePreset(name)` — POST delete, re-render
18. Add `closePresetsModal()`
19. Cache modal elements in `cacheElements()` — see addition above
20. Wire Presets button click handler in `setupEventListeners()` — see addition above

### Phase 5 — Verify

21. Compile, flash, test: save/load/delete presets, verify snackbar flow, verify settings.json persistence

---

## Relevant Files

- `src/Constants.h` — `MAX_PRESETS`, default preset values
- `src/SettingsHandler.h` — `Preset` struct, array member
- `src/SettingsHandler.cpp` — load/save JSON array (ArduinoJson pseudocode provided)
- `src/WebHandlers.cpp` — 4 new API routes + validation
- `src/WebContentInputs.h` — "Presets" button in PID section header
- `src/WebContentBody.h` — Presets modal HTML (structure provided)
- `src/styles.h` — Preset list CSS (rules provided)
- `src/WebContentScript.h` — Modal open/save/load/delete JS
- `data/default.json` — Factory Default preset
- `data/settings.json` — Runtime presets (managed by server)
