# VentCon2 Upload Guide

Last updated: 2026-07-23 | Firmware: v2.7.2

This guide covers all firmware and filesystem upload scenarios for VentCon2 on the Arduino Nano ESP32 (ESP32-S3).

---

## Prerequisites

### Software

- **PlatformIO** (CLI or VS Code extension) installed and working
- **Python 3** (comes with PlatformIO, but must be on PATH)
- **USB drivers**: The Arduino Nano ESP32 uses an ESP32-S3 with native USB CDC. Windows 10/11 typically recognizes it automatically. If not, install the drivers via the [Arduino IDE](https://www.arduino.cc/en/software) or [Zadig](https://zadig.akeo.ie/).

### Hardware

- Arduino Nano ESP32 board (ESP32-S3 based)
- USB-C cable (**must be data-capable, not charge-only**)

> **Tip:** If no COM port appears when you plug in the board, try a different USB cable — cheap cables are often charge-only. Also try different USB ports (prefer direct motherboard ports over hubs).

---

## One-Time Setup

### 1. USB Reset Helper (Admin, once per machine)

Run **as Administrator**:

```
.\install_usb_reset_helper.bat
```

Creates a Windows Scheduled Task (`VentCon2_USB_Reset`) that lets PlatformIO reset the ESP32-S3 USB port without admin prompts. The ESP32-S3 native USB CDC has a known issue where the COM port becomes inaccessible after the first upload session. This helper fixes it automatically.

### 2. Confirm Board Detection

```
pio device list
```

Should show a device with VID `2341` and PID `0070` (Arduino Nano ESP32) on a COM port (e.g., `COM3`).

---

## Which Procedure Do I Need?

| Starting point                                                                  | Procedure                                                                                             |
| ------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| Factory-fresh board, never flashed                                              | [Initial Setup or Recovery](#initial-setup-or-recovery-full-system-flash) — skip the erase step       |
| Board in unknown or corrupted state (DIR fails, settings lost, files missing)   | [Initial Setup or Recovery](#initial-setup-or-recovery-full-system-flash) — **with** erase step |
| Already flashed with VentCon2, updating firmware only                           | [Firmware Update Only](#firmware-update-only-everyday-upload)                                          |
| Already flashed with VentCon2, updating firmware + static files (PDF, JS, logo) | [Firmware + Filesystem Update](#firmware--filesystem-update-full-upload)                               |
| Filesystem upload succeeded but DIR shows only 1-2 files                        | [Initial Setup or Recovery](#initial-setup-or-recovery-full-system-flash) — **with** erase step |

---

## Initial Setup or Recovery (Full System Flash)

**This is the standard provisioning procedure.** It writes the bootloader, partition table, firmware, and filesystem — a complete factory image. Use it for first-time flashing, recovery, or whenever the board state is uncertain.

### Step 0: Enter Bootloader Mode

Enter bootloader mode **before** running any commands below: short B1 to GND and press RESET (or hold BOOT + press RESET). The LED turns purple. Stay in bootloader mode through all the following steps — the board will handle each esptool connection automatically.

### Step 1: Switch to esptool

```
python switch_protocol.py esptool
```

This modifies `platformio.ini` to set `upload_protocol = esptool`.

### Step 2: Erase Flash

> **Skip this step for factory-fresh virgin boards.** Run it for any board that has been previously flashed or is in an unknown state.

```
pio run --target erase
```

This wipes the entire 16 MB flash — all partitions, firmware, and filesystem data. Takes ~12 seconds. The board remains in bootloader mode afterward.

### Step 3: Upload Full Firmware

```
pio run --target upload
```

esptool writes the bootloader, partition table, and firmware. Connects immediately since the board is already in bootloader mode.

### Step 4: Upload Filesystem

```
.\upload_filesystem.bat
```

Connects immediately (board already in bootloader mode). Switches to esptool, uploads, then restores DFU automatically — no separate DFU switch step needed.

### Step 5: Power-Cycle and Verify

Power-cycle the board (unplug USB, plug back in) to exit bootloader mode, then follow the [Verifying a Successful Upload](#verifying-a-successful-upload) checklist.

---

## Firmware Update Only (Everyday Upload)

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

**No button pressing needed.** DFU handshake works automatically. Settings and filesystem are preserved.

---

## Firmware + Filesystem Update (Full Upload)

**Use this when `data/` folder contents have changed** (settings.json, default.json, chart.min.js, Logo.svg, user manual PDF) — but the board is already provisioned and working.

### Step 1: Firmware

```
pio run --target upload
```

(Done in seconds via DFU, no button press needed.)

### Step 2: Filesystem

```
.\upload_filesystem.bat
```

Enter bootloader mode before running (B1-GND + RESET, or BOOT+RESET — LED turns purple). This script:

1. Temporarily switches `upload_protocol` from `dfu` to `esptool`
2. Runs `pio run --target uploadfs`
3. Switches `upload_protocol` back to `dfu`

See [What Gets Flashed](#what-gets-flashed) for the file listing.

> **WARNING**: Uploading the filesystem overwrites the LittleFS partition, including runtime `settings.json`. Any settings changed via the web UI or serial console will be lost. After upload, re-apply your settings or restore from a backup.

---

## Entering Bootloader Mode

`monitor_rts = 0` and `monitor_dtr = 0` are set in `platformio.ini` to prevent accidental resets during normal operation. As a result, esptool **cannot** auto-trigger ROM bootloader mode — you must always enter it manually before any esptool upload.

**Method 1 — BOOT+RESET buttons:**

1. **Hold** the BOOT button
2. Press and release **RESET**
3. **Release** BOOT

**Method 2 — Short B1 to GND (Arduino Nano ESP32):**

1. Use a jumper wire to **short the B1 pin to GND**
2. Press and release **RESET**
3. **Remove** the jumper

The RGB LED will turn **purple**, indicating the ESP32-S3 is in ROM download mode. esptool will then connect and upload.

---

## What Gets Flashed

| Target                      | Content                                                                                                                   | Size                                   |
| --------------------------- | ------------------------------------------------------------------------------------------------------------------------- | -------------------------------------- |
| Firmware (`firmware.bin`) | Compiled C++ application (control system, PID, web server, WiFi AP)                                                       | ~1.0 MB                                |
| LittleFS image              | `data/default.json`, `data/settings.json`, `data/chart.min.js`, `data/Logo.svg`, `data/VentCon_User_Manual.pdf` | ~320 KB compressed (~3.4 MB partition) |

---

## Quick Reference

### Initial Setup or Recovery (Full System Flash)

```bash
# Step 0: Enter bootloader mode (B1-GND + RESET — LED purple) — stay there for all steps
python switch_protocol.py esptool
pio run --target erase          # skip for virgin boards
pio run --target upload
.\upload_filesystem.bat         # also restores DFU automatically
# Power-cycle the board to exit bootloader mode
```

### Firmware Update Only

```bash
pio run --target upload
```

### Firmware + Filesystem Update

```bash
pio run --target upload
.\upload_filesystem.bat         # enter bootloader mode (B1-GND + RESET)
```

---

## Verifying a Successful Upload

1. Open the serial monitor: `pio device monitor`
2. Confirm the VentCon2 boot message appears with the firmware version
3. Boot log shows `LittleFS mounted. Files:` listing 5 files
4. Type `DIR` and press Enter — lists all 5 LittleFS files with sizes (~3456 KB total, ~14% used)
5. Type `READ` — shows loaded settings from flash
6. Type `STATUS` — confirms system is running
7. Connect to WiFi: **SSID** `VENTCON_AP`, **Password** `ventcon12!`
8. Open `http://192.168.4.1` or `http://ventcon.local`
9. Footer shows version **2.7.2**
10. "Outlet Pressure Sensor Settings" card shows `0.0 - 10.0 bar`, `0.50 - 4.50 V`
11. "Display Range" shows `10.0 bar(g)`
12. Setpoint slider range is `0.00 - 10.00 bar`, step `0.25`

---

## Troubleshooting

| Symptom                                                   | Fix                                                                                                                              |
| --------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------- |
| No COM port detected                                      | Try a different USB-C cable (must be data-capable). Try a different USB port.                                                    |
| Upload fails with connection error                        | Enter bootloader mode (B1-GND + RESET — LED purple), then retry.                                                                |
| COM port disappears after upload                          | Run`install_usb_reset_helper.bat` as Admin (one-time).                                                                         |
| LED stays purple after upload                             | Power cycle the board (unplug USB, plug back in).                                                                                |
| Device not creating WiFi AP                               | Power cycle; verify`monitor_rts = 0` and `monitor_dtr = 0` in `platformio.ini`.                                            |
| Board stuck / unresponsive                                | Double-tap RESET quickly to enter Arduino bootloader mode, then flash with esptool.                                              |
| Web interface not loading                                 | Ensure filesystem was uploaded. Check serial for`LittleFS mounted` at boot.                                                    |
| Web UI shows old values after uploadfs                    | Hard-refresh browser (Ctrl+F5); clear cache.                                                                                     |
| DIR fails, settings lost, or files missing after uploadfs | Partition table or filesystem corrupted. Run[Initial Setup or Recovery](#initial-setup-or-recovery-full-system-flash) with erase. |

---

## Reference Notes

- The board has a **16 MB flash** chip partitioned via `default_16MB.csv`.
- Firmware upload baud rate is **921600**.
- The `inject_version.py` pre-build script reads the `VERSION` file and injects it into the firmware build automatically.
- The `switch_protocol.py` script toggles `upload_protocol` between `dfu` and `esptool` in `platformio.ini`.
- Manual bootloader entry (B1-GND + RESET, or BOOT+RESET) pulls **GPIO0** low during reset, which the ESP32-S3 ROM bootloader reads to enter download mode. The physical BOOT button on the Nano ESP32 silkscreen connects to GPIO0.
