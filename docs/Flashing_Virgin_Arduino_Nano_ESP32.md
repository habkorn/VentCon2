# Flashing a Virgin Arduino Nano ESP32 for VentCon2

This guide covers how to flash the VentCon2 firmware onto a brand-new (factory-fresh) Arduino Nano ESP32 board.

## Prerequisites

### Software
- **PlatformIO** (CLI or VS Code extension) installed and working
- **Python 3** (comes with PlatformIO, but must be on PATH)
- **USB drivers**: The Arduino Nano ESP32 uses an ESP32-S3 with native USB CDC. Windows 10/11 typically recognizes it automatically. If not, install the drivers via the [Arduino IDE](https://www.arduino.cc/en/software) or [Zadig](https://zadig.akeo.ie/).

### Hardware
- Arduino Nano ESP32 board (ESP32-S3 based)
- USB-C cable (data-capable, not charge-only)

## Step-by-Step Instructions

### 1. Connect the Board

Plug the Arduino Nano ESP32 into your PC via USB-C. A new COM port should appear in Device Manager.

> **Tip:** If no COM port appears, try a different USB cable — cheap cables are often charge-only. Also try different USB ports (prefer direct motherboard ports over hubs).

### 2. Verify the Board Is Detected

Open a terminal in the project root and run:

```bash
pio device list
```

You should see a device with VID `2341` and PID `0070` (Arduino Nano ESP32). Note the COM port number (e.g., `COM3`).

### 3. Initial Flash — Use esptool Protocol

A virgin Arduino Nano ESP32 ships with the Arduino bootloader, which expects **esptool** for the first upload. The project's default `upload_protocol` is `dfu`, but a factory-fresh board has not yet been configured for DFU mode.

Switch the upload protocol to esptool:

```bash
python switch_protocol.py esptool
```

This modifies `platformio.ini` to set `upload_protocol = esptool`.

### 4. Build and Upload Firmware

```bash
pio run --target upload
```

PlatformIO will compile the firmware and flash it via esptool. You should see output like:

```
Writing at 0x00010000... (100%)
Wrote 1234567 bytes ...
Hard resetting via RTS pin...
```

> **Troubleshooting:** If the upload fails with a connection error, hold the **BOOT** button (B1) on the board, press and release **RESET**, then release **BOOT**. This forces the ESP32-S3 into download mode. Then retry the upload command.

### 5. Upload the Filesystem (LittleFS)

VentCon2 stores web resources (Chart.js) and default settings on a LittleFS partition. Upload the `data/` folder contents:

```bash
pio run --target uploadfs
```

Alternatively, use the convenience script which handles protocol switching automatically:

```bash
upload_filesystem.bat
```

The filesystem contains:
- `chart.min.js` — Chart.js library for the web interface
- `default.json` — Factory-default settings
- `settings.json` — Active settings (initially a copy of defaults)

### 6. Switch to DFU Protocol for Future Uploads

Once the VentCon2 firmware is running, the board supports DFU uploads (faster and more reliable for subsequent flashes). Switch back:

```bash
python switch_protocol.py dfu
```

All future uploads via `pio run --target upload` will now use DFU.

### 7. Install the USB Reset Helper (One-Time, Optional but Recommended)

The ESP32-S3 native USB CDC has a known issue where the COM port becomes inaccessible after the first upload session. The USB reset helper fixes this automatically.

Run **as Administrator** (one-time setup):

```bash
install_usb_reset_helper.bat
```

This creates a Windows Scheduled Task (`VentCon2_USB_Reset`) that the pre-upload script calls automatically to reset the USB device when the COM port is stuck.

### 8. Verify the Flash

After uploading, open the serial monitor:

```bash
pio device monitor
```

You should see the VentCon2 boot message with the firmware version. Type `status` and press Enter to confirm the system is running.

Connect to the WiFi Access Point to verify the web interface:
- **SSID:** `VENTCON_AP`
- **Password:** `ventcon12!`
- **URL:** `http://192.168.4.1`

## Quick Reference — Complete First-Flash Sequence

```bash
# 1. Switch to esptool for virgin board
python switch_protocol.py esptool

# 2. Flash firmware
pio run --target upload

# 3. Flash filesystem
pio run --target uploadfs

# 4. Switch to DFU for future uploads
python switch_protocol.py dfu

# 5. (One-time, as Admin) Install USB reset helper
install_usb_reset_helper.bat

# 6. Verify
pio device monitor
```

## Troubleshooting

| Problem | Solution |
|---|---|
| No COM port detected | Try a different USB-C cable (must be data-capable). Try a different USB port. |
| Upload fails with "No serial data received" | Hold **BOOT**, tap **RESET**, release **BOOT**, then retry upload. |
| COM port disappears after first upload | Run `install_usb_reset_helper.bat` as Admin (one-time). The pre-upload script will handle resets automatically. |
| `esptool` can't connect after DFU upload | Switch back to esptool: `python switch_protocol.py esptool`, then retry. |
| Web interface not loading | Ensure filesystem was uploaded (`pio run --target uploadfs`). Check serial monitor for LittleFS mount errors. |
| Board stuck / unresponsive | Double-tap the **RESET** button quickly to enter bootloader mode, then flash with esptool. |

## Notes

- The board has a **16 MB flash** chip partitioned via `default_16MB.csv`.
- Firmware baud rate for upload is `921600`.
- The `inject_version.py` pre-build script reads the `VERSION` file and injects it into the firmware build automatically.
