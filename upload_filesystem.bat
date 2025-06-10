@echo off
echo ===== VENTCON Filesystem Upload Tool =====
echo This script will temporarily switch to esptool upload protocol
echo for uploading the filesystem, then switch back to DFU.

echo.
echo 1. Switching protocol to esptool...
python switch_protocol.py esptool

echo.
echo 2. Uploading filesystem...
pio run --target uploadfs

echo.
echo 3. Switching protocol back to DFU...
python switch_protocol.py dfu

echo.
echo Done! You can now continue using DFU for normal uploads.
pause
