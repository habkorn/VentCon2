## Plan: Bottom Navigation Tab Architecture for VentCon2 Web UI

**Status**: Approved -- ready for implementation (2026-07-18)

### TL;DR

Replace the 4 `<details class="card">` accordion sections with a flat, tab-swapping architecture driven by a fixed Bottom Navigation Bar. The gauges and live chart remain a persistent header at the top. The viewport is NOT fixed -- the page scrolls naturally, with content height adapting to whichever tab is active. Zero new dependencies; all changes confined to existing vanilla HTML/CSS/JS in PROGMEM.

---

### Architecture Decisions (from user input)

| Decision | Choice |
|---|---|
| Dashboard (gauges + chart) | Persistent header, always visible at top |
| Tab count | 4 tabs: **Control**, **Auxiliary**, **Sensors**, **System** |
| Reset PID & Reset to Default | Both relocated to the **Auxiliary** tab |
| Bottom nav | `position: fixed` at viewport bottom |
| Card visual style | **Retained** -- all `.view-section` divs carry `class="card"` for the same white background, border-radius, box-shadow, and padding as the current accordions |
| Nav icons | Unicode BMP-safe monochrome glyphs (zero PROGMEM cost, no SVG/emoji) |

### Bottom Nav Icon Set

| Tab | Unicode | Glyph | Name | Rationale |
|---|---|---|---|---|
| **Control** | `U+25CE` | &#9672; | BULLSEYE | Represents aiming at the pressure setpoint. Visually distinct from the &#9870; gear already used for slider/chart settings buttons. |
| **Auxiliary** | `U+2692` | &#9882; | HAMMER AND PICK | Tools / maintenance. Fits the reset buttons, PWM config, and filter tuning housed here. |
| **Sensors** | `U+223F` | &#8767; | SINE WAVE | Analog signal waveform -- the pressure sensor produces a voltage signal. |
| **System** | `U+2139` | &#8505; | INFORMATION SOURCE | Info / about. Network status, PDF manual, device version. |

All four glyphs are in the Basic Multilingual Plane and render consistently as monochrome text across Segoe UI (Windows), San Francisco (macOS/iOS), Roboto (Android), and DejaVu (Linux). No additional PROGMEM payload.

---

### Files to Modify

| File | Type of change | Complexity |
|---|---|---|
| `src/WebContentBody.h` | Add `<nav id="bottom-nav">` markup before `</main>`; restructure accordions into tab panels | High |
| `src/WebContentInputs.h` | Remove `<details>/<summary>` wrappers; wrap each section in `<div class="view-section" id="view-xxx">`; add `#resetDefaultsBtn` to Auxiliary tab; add hidden class to all but Control | High |
| `src/WebContentFooter.h` | Remove `reset-defaults-container` div and `#resetDefaultsBtn` (moved to Auxiliary tab) | Low |
| `src/styles.h` | Add styles for `#bottom-nav`, `.nav-btn`, `.view-section`, `.hidden`; remove or neuter `details.card summary` styles; add `body` `padding-bottom` for fixed nav clearance | Medium |
| `src/WebContentScript.h` | Add tab-switching click handler for `.nav-btn`; no other JS logic changes needed (all existing handlers reference IDs that stay the same) | Low |
| `src/WebHandlers.cpp` | No change needed (handleRoot() streams HTML_INPUTS as before, placeholder substitution unchanged) | None |

### Files NOT Modified

| File | Reason |
|---|---|
| `src/WebContentHead.h` | Unchanged (meta tags, Chart.js include) |
| `src/WebHandlers.h` | Unchanged (no API changes) |
| `src/WebHandlers.cpp` | Unchanged (streaming order stays the same; placeholder substitution logic works on the same content) |
| `src/Constants.h` | Unchanged |
| `data/` | Unchanged (chart.min.js, settings.json, default.json) |

---

### Implementation Steps

#### Phase 1: HTML Structure Refactor (WebContentBody.h + WebContentInputs.h + WebContentFooter.h)

**Step 1.1**: In `WebContentBody.h` -- Add `<nav id="bottom-nav">` with 4 buttons inside `<main>`, after the modals and snackbar but before the tab content. Remove the `<section class="card">` wrapper from gauges/chart (keep the gauges and chart as-is, just unwrap from the card). Add the `.view-section` container opening. The nav button markup:

```html
<nav id="bottom-nav">
  <button class="nav-btn active" data-target="view-control">
    <span class="nav-icon">&#9672;</span>Control
  </button>
  <button class="nav-btn" data-target="view-aux">
    <span class="nav-icon">&#9882;</span>Auxiliary
  </button>
  <button class="nav-btn" data-target="view-sensors">
    <span class="nav-icon">&#8767;</span>Sensors
  </button>
  <button class="nav-btn" data-target="view-system">
    <span class="nav-icon">&#8505;</span>System
  </button>
</nav>
```

_Reference_: WebContentBody.h lines 27-94 (current body: gauges, chart, snackbar, modals)

**Step 1.2**: In `WebContentInputs.h` -- Flatten all 4 `<details class="card">` sections into `<div class="view-section" id="view-xxx">` blocks:
- `#view-control` (currently: Control Parameters `<details class="card" open>`) -- Setpoint, Kp/Ki/Kd sliders
- `#view-aux` (currently: Auxiliary Settings `<details class="card">`) -- Filter, PWM freq/res, PID sample time, PLUS Reset PID button AND Reset to Default button (both relocated here per user decision)
- `#view-sensors` (currently: Sensor Settings `<details class="card">`) -- Pressure/voltage calibration
- `#view-system` (currently: System Information `<details class="card" open>`) -- Network status, PDF link

Remove ALL `<details>`, `<summary>`, `<div class="collapsible-content">` wrappers. Preserve all `id` attributes on inputs, sliders, buttons, and display elements. **Every `<div class="view-section">` also carries `class="card"`** so the visual style (white background, rounded corners, box-shadow, 0.75rem padding) is identical to the current accordion cards -- only the collapsible behavior is removed. Add `class="hidden"` to `#view-aux`, `#view-sensors`, and `#view-system` (only `#view-control` visible by default).

**Step 1.3**: In `WebContentInputs.h` -- Move `#resetPidBtn` from its current `.section-title-row` in Control Parameters to the Auxiliary tab (top of the Auxiliary section). Move `#resetDefaultsBtn` from `WebContentFooter.h` to the Auxiliary tab (bottom of the Auxiliary section, after the PID Sample Time row).

**Step 1.4**: In `WebContentFooter.h` -- Remove the entire `<div class="reset-defaults-container">` block (lines 173-175 in current file). The button now lives in `WebContentInputs.h`. Everything else in the footer stays unchanged (logo, brand title, version, easter egg).

#### Phase 2: CSS Refactor (styles.h)

**Step 2.1**: Add bottom navigation bar styles:
```css
#bottom-nav {
  position: fixed;
  bottom: 0;
  left: 0;
  width: 100%;
  display: flex;
  justify-content: space-around;
  align-items: center;
  background: var(--card);
  border-top: 1px solid var(--border);
  padding: 0.5rem 0;
  padding-bottom: calc(0.5rem + env(safe-area-inset-bottom, 0px));
  z-index: 500;
  box-shadow: 0 -2px 8px var(--shadow);
}
.nav-btn {
  background: none;
  border: none;
  min-width: unset;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.2rem;
  padding: 0.25rem 1rem;
  font-size: 0.7rem;
  font-weight: 600;
  color: var(--text-light);
  cursor: pointer;
  transition: color 0.2s;
}
.nav-btn.active {
  color: var(--primary);
}
.nav-btn .nav-icon {
  font-size: 1.3rem;
  line-height: 1;
}
```

**Step 2.2**: Add view toggling utilities:
```css
.view-section { }
.view-section.hidden { display: none !important; }
```

**Step 2.3**: Add body padding-bottom to prevent content occlusion by the fixed nav:
```css
body {
  /* existing styles */
  padding-bottom: 80px; /* matches #bottom-nav height */
}
```
Place this in the existing `body` rule (styles.h lines 32-42). Keep the existing `padding: 0 0.5rem 0.5rem 0.5rem` and add `padding-bottom: 80px` to override the bottom padding.

**Step 2.4**: Neuter the `<details>`-specific CSS rules since `<details>` elements are removed:
- `details.card` (line 54-57): Remove
- `details.card summary` (line 59-67): Remove
- `details.card summary::-webkit-details-marker` (line 69-71): Remove
- `details.card summary::after` (line 73-76): Remove
- `details.card[open] summary::after` (line 78-80): Remove
- `details.card .collapsible-content` (line 82-85): Remove

**Step 2.5**: Remove `.reset-defaults-container` and `.reset-defaults-btn` styles (moved to Aux tab). Add a compact inline button group style for the two reset buttons in the Auxiliary tab -- side by side with reduced padding to prevent pushing PID Sample Time below the fold on narrow screens:

```css
.reset-btn-group {
  display: flex;
  gap: 0.5rem;
  margin: 0.5rem 0;
}
.reset-btn-group .reset-btn,
.reset-btn-group .reset-defaults-btn {
  flex: 1;
  min-width: unset;
  padding: 0.4rem 0.5rem;
  font-size: 0.75rem;
  margin: 0;
}
```

Also remove the old `.reset-defaults-container` and `.reset-defaults-btn` standalone styles.

**Step 2.6**: Adjust the snackbar bottom offset to clear the bottom nav. The current snackbar CSS has `bottom: 24px`. Change to `bottom: calc(24px + 70px)` where 70px accounts for the nav height + safe area. Also update the VisualViewport repositioning JS to factor in the nav height.

**Step 2.7**: Verify the `.card` class still works on the new `.view-section` divs (currently `.card` styles apply to both `<section class="card">` and `<details class="card">`). The `.view-section` divs carry both classes: `<div class="view-section card">`. The `.card` class provides the familiar white background, `border-radius: 0.5rem`, `box-shadow`, and `padding: 0.75rem`, ensuring each tab section is visually indistinguishable from the current accordion cards.

#### Phase 3: JavaScript (WebContentScript.h)

**Step 3.1**: Add tab-switching click handler. Insert after `cacheElements()` or at the end of `setupEventListeners()`:

```javascript
// Bottom navigation tab switching
document.querySelectorAll('.nav-btn').forEach(function(btn) {
  btn.addEventListener('click', function() {
    var targetId = this.getAttribute('data-target');
    // Deactivate all buttons and hide all views
    document.querySelectorAll('.nav-btn').forEach(function(b) { b.classList.remove('active'); });
    document.querySelectorAll('.view-section').forEach(function(v) { v.classList.add('hidden'); });
    // Activate clicked button and show target view
    this.classList.add('active');
    var target = document.getElementById(targetId);
    if (target) {
      target.classList.remove('hidden');
      // Scroll to top of view-section for clean tab transition
      target.scrollIntoView({ behavior: 'smooth', block: 'start' });
    }
  });
});
```

**Step 3.2**: No other JS changes required. All existing event listeners bind by element ID (`#sp_slider`, `#resetPidBtn`, etc.) which are preserved in their new DOM positions. The `input.closest('details.card')` call in the sensor focus handler (line ~600) must be updated to `input.closest('.view-section')` -- this is the only JS line that breaks from the DOM structure change.

**Step 3.3**: Update the VisualViewport snackbar repositioning to account for the bottom nav height. Current code resets `bottom` to `'24px'` on keyboard close. Change to account for nav:
- Constant at top of script: `var BOTTOM_NAV_HEIGHT = 66; // px`
- In `repositionSnackbar()`: `saveSnackbar.style.bottom = Math.max(24 + BOTTOM_NAV_HEIGHT, 24 + BOTTOM_NAV_HEIGHT + offset) + 'px';`
- In the `focusout` reset: `saveSnackbar.style.bottom = (24 + BOTTOM_NAV_HEIGHT) + 'px';` instead of `'24px'`

**Step 3.4**: Verify that `setupScrollHandler()` still works after removing `<details>` elements. This handler disables slider thumbs during scroll to prevent accidental value changes. It should be unaffected since it works on `input[type="range"]` elements directly.

---

### Verification Plan

#### Automated / Compile-Time Checks

1. **PlatformIO build**: `pio run` -- must compile without errors. The C++ PROGMEM strings must be syntactically valid.
2. **PROGMEM size check**: Compare `WebContent.h` total size before and after. Expected net change: under +500 bytes (nav markup ~400 bytes added, details/summary markup ~300 bytes removed, CSS changes ~+200 bytes, JS changes ~+150 bytes).

#### Manual / Visual Checks

3. **Initial page load**: Gauges and chart visible at top. Control tab active by default. Bottom nav visible with 4 buttons (Control highlighted). Page scrollable.
4. **Tab switching**: Click each nav button. Verify:
   - Only the corresponding section is visible
   - Active button highlights in primary color
   - Page scrolls to bring the active section into view
   - Chart and gauges remain visible at top regardless of active tab
   - All sliders, text inputs, and buttons function identically to before
5. **Reset PID button** (now in Auxiliary tab): Click -- verify PID reset works (green flash, server response)
6. **Reset to Default button** (now in Auxiliary tab): Click -- verify multi-step default restoration works (confirm dialog, slider limits reset, chart settings reset, values reset, PID reset)
7. **Snackbar positioning**: 
   - Modify a parameter value -- snackbar appears above the bottom nav (not behind it)
   - On mobile, focus a text input -- snackbar repositions above keyboard AND above nav
   - On blur, snackbar returns to position above nav
8. **Slider scroll protection**: On mobile, scroll through a tab -- verify sliders don't accidentally change values
9. **Easter egg**: Click footer logo 5 times -- verify developer panel still appears
10. **Chart toggle**: Toggle chart on/off -- works as before
11. **All modals**: Slider settings modal and chart settings modal open, function, and close correctly

#### Edge Case Checks

12. **Rapid tab switching**: Click tabs quickly in succession -- no flicker, no JS errors, correct tab active
13. **Unsaved changes survive tab switch**: Modify a slider value (snackbar appears), switch tabs, switch back -- value still shown in text field, snackbar still visible
14. **Network status updates across tabs**: Network indicator continues updating even when System tab is not active (the polling loop runs independently of tab visibility)
15. **Chart data collection continues**: Chart buffers keep collecting data even if chart is hidden via toggle

---

### Design Decisions & Rationale

- **Persistent gauges/chart**: The user confirmed gauges and chart stay at top. This makes sense because they show real-time state independent of which settings tab is active.
- **4 tabs instead of 3**: The user preferred keeping Control and Auxiliary separate. This avoids a long scrollable Control tab that mixes PID tuning with PWM hardware settings. Each tab has a clear, focused purpose.
- **Both reset buttons in Auxiliary**: The user's choice. Auxiliary becomes the "maintenance" tab (filter, PWM config, reset actions). This is logical since Reset PID and Reset to Default are infrequent operations, not part of the primary Control workflow.
- **Fixed bottom nav (not sticky)**: The user chose fixed positioning. This is the standard mobile pattern and ensures the nav is always accessible without scrolling. The `padding-bottom` on body ensures content isn't permanently hidden behind it.
- **`.hidden` uses `display: none !important`**: No layout thrashing when switching tabs. The browser doesn't render hidden sections. Chart.js and the polling loop are unaffected since they don't depend on DOM visibility of the parameter sections.

---

### What Is Deliberately Excluded

    No SVG icons in nav buttons -- Unicode BMP-safe monochrome glyphs: &#9672; BULLSEYE (CTRL), &#9882; HAMMER AND PICK (AUX), &#8767; SINE WAVE (SEN), &#8505; INFORMATION SOURCE (SYS)
- **Nav label typography**: Bahnschrift (DIN 1451) font stack with `font-weight: 500`, `letter-spacing: 0.04em`, `text-transform: uppercase` -- condensed industrial geometric face that ships with every Windows 10/11 installation; falls back to DIN Alternate, Segoe UI, then system-ui. No web font download required. Short 3-4 character labels (CTRL/AUX/SEN/SYS) for balanced proportions at small sizes.
- No CSS transitions/animations on tab swap -- instant display switch keeps complexity low and avoids repaint interaction with Chart.js
- No changes to the loader overlay, modals, or Easter egg
- No changes to any API endpoint or server-side logic
- No changes to `handleRoot()` streaming order in WebHandlers.cpp
- **Reset buttons (in Aux tab)**: Compact inline styling -- side by side, smaller padding, to prevent pushing PID Sample Time below the fold on narrow mobile screens (~375px)
