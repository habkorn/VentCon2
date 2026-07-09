# Changelog

## [2.7.1] — 2026-07-07

### Changed
- Reverted sensor calibration mapping to 0.5V → 0 bar(g), 4.5V → 10 bar(g).
- `MAX_OPERATING_BAR` reduced from 20 bar to 10 bar, cascading to all UI defaults (slider, chart, display range).
- `HARD_SP_MAX_BAR` reduced from 20 bar to 10 bar.
- Safety threshold (`isPressureSafe()`) now triggers at 11 bar (10 × 1.1).

### Documentation
- Updated LaTeX documentation to reflect 0-10 bar sensor range.

---

## [2.7.0] — 2026-07-07

### Added
- Configurable display range setting for the pressure bar visualization, decoupled from sensor calibration range.
- Firmware-enforced hard setpoint cap (`HARD_SP_MAX_BAR = 20 bar`) for safety, independent of UI slider configuration.
- New `OperatingDefaults` namespace in `Constants.h` centralizing operating-range constants.

### Changed
- `SENSOR_MAX_BAR` updated from 10 bar to 100 bar to match the physical pressure sensor.
- Setpoint slider default max reduced from 100 bar to 20 bar (operating range).
- Chart Y-axis default max reduced from 10 bar to 20 bar.
- Auto-tuner max test setpoint capped at 20 bar (operating range).
- Safety pressure threshold (`isPressureSafe()`) now uses operating-range-based limit (22 bar) instead of tracking the full sensor range.

### Fixed
- Pressure bar and setpoint marker scaling now use the configurable display range, preventing tiny bar fills when operating in a narrow range.
- `SliderDefaults::SP_MAX` decoupled from `SENSOR_MAX_BAR` so changing one doesn't inadvertently change the other.

### Documentation
- Updated LaTeX documentation to reflect new sensor range (0-100 bar) and operating range (0-20 bar).
