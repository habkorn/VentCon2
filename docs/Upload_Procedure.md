# VentCon2 Upload Procedure

Last updated: 2026-07-09 | Firmware: v2.7.1

## One-Time Prerequisites

### 1. USB Reset Helper (Admin, once per machine)

Run **as Administrator**:

```
install_usb_reset_helper.bat
```

Creates a Windows Scheduled Task that lets PlatformIO reset the ESP32-S3 USB port without admin prompts.

### 2. Confirm Board Detection

```
pio device list
```

Should show a device with VID `2341` and PID `0070` (Arduino Nano ESP32) on a COM port.

---

## Normal Upload (DFU)

**Use this for everyday firmware changes that don't touch the `data/` folder.**

```
pio run --target upload
```

| Step                        | What happens                                                         |
| --------------------------- | -------------------------------------------------------------------- |
| `inject_version.py`       | Reads`VERSION` file, injects `VENTCON_VERSION_NUMBER` build flag |
| `usb_reset_pre_upload.py` | Resets USB port for DFU handshake                                    |
| Compile + flash             | Firmware binary uploaded via DFU at 921600 baud                      |
| Auto-reset                  | Board reboots into new firmware                                      |

**No button pressing needed.** DFU handshake works automatically.

---

## Full Upload (Firmware + Filesystem)

**Use this when `data/` folder contents have changed** (settings.json, default.json, chart.min.js, Logo.svg, user manual PDF).

### Step 1: Firmware

```
pio run --target upload
```

(Done in seconds, no button press needed.)

### Step 2: Filesystem

```
upload_filesystem.bat
```

This script:

1. Temporarily switches `upload_protocol` from `dfu` to `esptool`
2. Runs `pio run --target uploadfs`
3. Switches `upload_protocol` back to `dfu`

### When esptool Can't Auto-Connect

Because `monitor_rts = 0` and `monitor_dtr = 0` are set in `platformio.ini` (to prevent accidental resets during normal operation), esptool cannot auto-trigger ROM bootloader mode.

**Manual bootloader entry (B0+RESET):**

1. **Hold** the B0 (BOOT) button
2. Press and release **RESET**
3. **Release** B0

The RGB LED will turn **purple**, indicating the ESP32-S3 is in ROM download mode. esptool will then connect and upload the filesystem image.

---

## Flashing a Brand-New (Virgin) Board

For a factory-fresh Arduino Nano ESP32 that has never been flashed:

### Step 1: Switch to esptool Protocol

```
python switch_protocol.py esptool
```

### Step 2: Upload Firmware

```
pio run --target upload
```

If the upload fails with a connection error, use the B0+RESET sequence (see above) to force download mode, then retry.

### Step 3: Restore DFU Protocol

```
python switch_protocol.py dfu
```

### Step 4: Upload Filesystem

```
upload_filesystem.bat
```

(May need B0+RESET again.)

---

## What Gets Flashed

| Target                      | Content                                                                                                                   | Size    |
| --------------------------- | ------------------------------------------------------------------------------------------------------------------------- | ------- |
| Firmware (`firmware.bin`) | Compiled C++ application (Control system, PID, web server, WiFi AP)                                                       | ~1.0 MB |
| LittleFS image              | `data/default.json`, `data/settings.json`, `data/chart.min.js`, `data/Logo.svg`, `data/VentCon_User_Manual.pdf` | ~220 KB |

---

## Verify After Upload

1. Connect to WiFi: **SSID** `VENTCON_AP`, **Password** `ventcon12!`
2. Open `http://192.168.4.1` or `http://ventcon.local`
3. Footer shows version **2.7.1**
4. "Outlet Pressure Sensor Settings" card shows `0.0 - 10.0 bar`, `0.50 - 4.50 V`
5. "Display Range" shows `10.0 bar(g)`
6. Setpoint slider range is `0.00 - 10.00 bar`, step `0.25`

## Troubleshooting

| Symptom                                       | Fix                                                                                            |
| --------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| COM port disappears after upload              | Run`install_usb_reset_helper.bat` as Admin                                                   |
| Filesystem upload fails with connection error | Use B0+RESET sequence (hold BOOT, press RESET, release BOOT)                                   |
| LED stays purple after upload                 | Power cycle the board (unplug USB, plug back in)                                               |
| Device not creating WiFi AP                   | Power cycle; verify`monitor_rts = 0` and `monitor_dtr = 0` in `platformio.ini`           |
| Virgin board not detected                     | Try a different USB cable (must be data-capable, not charge-only); try direct motherboard port |
| Web UI shows old values after uploadfs        | Hard-refresh browser (Ctrl+F5); clear cache                                                    |
