# VentCon2 - AI Coding Agent Instructions

ESP32-S3 PID pressure control system for solenoid valves using PlatformIO/Arduino.

## Architecture Overview

**Control Flow**: `main.cpp` initializes all components → `TaskManager` creates FreeRTOS tasks → `ControlSystem` runs PID loop at configurable frequency (default 1kHz) → PWM output to solenoid valve.

**Component Hierarchy** (dependency injection pattern):
```
SettingsHandler (standalone, LittleFS persistence)
    ↓
SensorManager (depends on SettingsHandler)
    ↓
AutoTuner (depends on SettingsHandler, PID)
    ↓
ControlSystem (depends on all above + PID)
WebHandler, CommandProcessor (parallel, depend on SettingsHandler + PID)
    ↓
TaskManager (orchestrates ControlSystem + WebHandler as FreeRTOS tasks)
```

## Build & Upload Commands

```bash
pio run                    # Build firmware
pio run --target upload    # Upload via esptool (default)
pio run --target uploadfs  # Upload LittleFS filesystem
python switch_protocol.py dfu     # Switch to DFU upload mode
python switch_protocol.py esptool # Switch to esptool upload mode
```

Use `upload_filesystem.bat` to upload static web resources (handles protocol switching automatically).

## Key Conventions

### Constants Organization
All hardware/system constants are in `Constants.h` using namespaces:
- `NetworkConfig::` - WiFi AP settings
- `HardwareConfig::` - GPIO pins, PWM channels
- `SensorConfig::` - ADC configuration, pressure range (0-10 bar)
- `ValveConfig::` - `VALVE_MIN_DUTY` (50%), `VALVE_MAX_DUTY` (90%)

### PWM/Valve Mapping
The valve operates in a restricted duty cycle range (50-90%). The `mapPwmToValve()` function in `ControlSystem.cpp` maps PID output (0-100%) to this effective range. When modifying valve control:
- `pwmFullScaleRaw` = maximum raw PWM value based on resolution (e.g., 16383 for 14-bit)
- Duty cycle < 1% keeps valve closed (returns 0)
- Anti-windup resets PID when output is outside effective valve range

### Rate-Limited Debug Output Pattern
Use this pattern for debug output in control loops:
```cpp
static unsigned long lastDebugTime = 0;
if (*continousValueOutput && (millis() - lastDebugTime >= SERIAL_OUTPUT_INTERVAL)) {
    Serial.println("Debug message");
    lastDebugTime = millis();
}
```

### Settings Persistence
`SettingsHandler` manages all configurable parameters. Changes persist to `/settings.json` on LittleFS. Always call `settings.save()` after modifications.

## Serial Command Interface

Commands are processed by `CommandProcessor`. Key commands:
- `sp=X.X` - Set pressure setpoint (bar)
- `kp=X.X`, `ki=X.X`, `kd=X.X` - Set PID gains
- `co` - Toggle continuous output
- `tune` - Start auto-tuning
- `status` - Show current state
- `save` - Persist settings

## Web Interface

WiFi AP mode (SSID: `VENTCON_AP`, password in `Constants.h`). Captive portal serves HTML from `WebContent.h` (generated from `test/webcontent.html`). Static resources (Chart.js) stored compressed in `data/` folder.

## FreeRTOS Task Structure

- **Control Task**: High priority, runs PID loop with `vTaskDelayUntil()` for precise timing
- **Network Task**: Lower priority, handles web server and DNS

## Documentation

LaTeX documentation in `LaTeX/` folder. Build with `make` in that directory. Keep code snippets in documentation synchronized with actual implementation.

## Testing

Manual testing via:
1. Serial commands through `pio device monitor`
2. Web interface at `http://192.168.4.1` when connected to AP
3. `test/webcontent.html` for web UI development (uses CDN resources)
