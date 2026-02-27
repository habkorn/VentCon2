@echo off
REM =====================================================================
REM  VENTCON Initial Flash Tool — One-time setup for NEW Arduino Nano ESP32
REM =====================================================================
REM
REM  PURPOSE:
REM    The Arduino Nano ESP32 ships with a factory partition table
REM    (app3M_fat9M_fact512k_16MB.csv) that does NOT contain a "spiffs"
REM    partition.  VentCon2 uses default_16MB.csv which defines a LittleFS
REM    partition at 0xC90000.  DFU uploads never write the partition table,
REM    so a new board must be flashed ONCE via esptool to establish:
REM
REM      1. Bootloader          (0x0000)
REM      2. Partition table     (0x8000)
REM      3. Application         (0x10000)
REM      4. LittleFS filesystem (0xC90000)
REM
REM  WHEN TO USE:
REM    - Brand-new Arduino Nano ESP32 (never flashed with VentCon2)
REM    - After an "erase_flash" or if LittleFS shows 0 bytes
REM    - After changing the partition table (board_build.partitions)
REM
REM  AFTER THIS SCRIPT:
REM    Normal DFU uploads (pio run --target upload) will work for
REM    firmware updates.  The partition table persists across DFU uploads.
REM    Use upload_filesystem.bat only when data/ files change.
REM
REM  PREREQUISITES:
REM    - PlatformIO CLI (pio) in PATH
REM    - Python in PATH
REM    - Board connected via USB
REM
REM  B1-GND BOOT MODE (Arduino Nano ESP32):
REM    esptool requires the ESP32-S3 to be in USB download mode.
REM    To enter download mode:
REM      1. Short the B1 pin to GND (e.g., with a jumper wire)
REM      2. Press the RESET button (or unplug/replug USB)
REM      3. The board enumerates as "ESP32-S3" (VID:PID = 303A:1001)
REM    Keep B1-GND shorted during ALL esptool flash steps.
REM    Remove the short only when prompted at the end.
REM =====================================================================

echo.
echo ========================================================
echo   VENTCON Initial Flash (One-Time Setup)
echo ========================================================
echo   This script flashes bootloader, partition table,
echo   firmware, and filesystem via esptool.
echo   Required ONCE per new Arduino Nano ESP32.
echo ========================================================
echo.

REM ===================================================================
REM  PHASE 1: Build (no special hardware setup needed)
REM ===================================================================

echo [1/6] Building firmware...
call pio run
if errorlevel 1 (
    echo.
    echo ERROR: Build failed. Fix build errors and try again.
    pause
    exit /b 1
)
echo       Build OK.

echo.
echo [2/6] Switching upload protocol to esptool...
python switch_protocol.py esptool
if errorlevel 1 (
    echo ERROR: Could not switch protocol.
    pause
    exit /b 1
)

REM ===================================================================
REM  PHASE 2: Enter download mode (requires B1-GND short)
REM ===================================================================

echo.
echo ========================================================
echo   ACTION REQUIRED: Put board into DOWNLOAD MODE
echo ========================================================
echo.
echo   1. Short pin B1 to GND (jumper wire or tweezers)
echo   2. Press the RESET button once
echo   3. The board should now show as "ESP32-S3" in
echo      Device Manager (VID:PID = 303A:1001)
echo.
echo   Keep B1-GND shorted until prompted to remove it!
echo.
pause

REM --- Show detected ports so user can verify download mode ---
echo.
echo   Detected USB devices:
pio device list
echo.

REM ===================================================================
REM  PHASE 3: Flash via esptool (B1-GND must remain shorted)
REM ===================================================================

echo [3/6] Flashing bootloader + partition table + firmware...
echo       (The COM port number may differ from normal — this is OK)
echo.
pio run --target upload
if errorlevel 1 (
    echo.
    echo ERROR: Firmware flash failed.
    echo   - Verify B1 is still shorted to GND
    echo   - Close any Serial Monitor that may hold the port
    echo   - Press RESET while B1-GND is shorted, then retry
    echo.
    echo Restoring DFU protocol...
    python switch_protocol.py dfu
    pause
    exit /b 1
)
echo       Firmware flash OK.

echo.
echo [4/6] Uploading LittleFS filesystem (data/ folder)...
echo       (B1-GND must still be shorted)
echo.
pio run --target uploadfs
if errorlevel 1 (
    echo.
    echo ERROR: Filesystem upload failed.
    echo   - The board may have changed COM port after firmware flash.
    echo   - Press RESET while B1-GND is shorted, then try again
    echo     with: pio run --target uploadfs
    echo.
    echo Restoring DFU protocol...
    python switch_protocol.py dfu
    pause
    exit /b 1
)
echo       Filesystem upload OK.

REM ===================================================================
REM  PHASE 4: Exit download mode (remove B1-GND short)
REM ===================================================================

echo.
echo ========================================================
echo   ACTION REQUIRED: Exit DOWNLOAD MODE
echo ========================================================
echo.
echo   1. REMOVE the B1-GND short (disconnect jumper wire)
echo   2. Press the RESET button once
echo   3. The board should now show as "Arduino" in
echo      Device Manager (VID:PID = 2341:0070)
echo.
pause

REM --- Restore DFU protocol for normal development ---
echo.
echo [5/6] Switching upload protocol back to DFU...
python switch_protocol.py dfu

REM --- Verify the board is back in normal mode ---
echo.
echo [6/6] Verifying board is in normal mode...
pio device list
echo.

echo ========================================================
echo   SUCCESS — Initial flash complete!
echo.
echo   Flashed:
echo     Bootloader          at 0x0000
echo     Partition table     at 0x8000  (default_16MB.csv)
echo     Firmware            at 0x10000
echo     LittleFS filesystem at 0xC90000 (data/ folder)
echo.
echo   You can now use DFU for normal firmware uploads.
echo   Use upload_filesystem.bat when data/ files change.
echo ========================================================
pause
