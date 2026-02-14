@echo off
echo ===== VentCon2 USB Reset Helper Setup =====
echo.
echo This script creates a Windows Scheduled Task that can reset the Arduino
echo USB device without requiring admin rights each time.
echo.
echo You only need to run this ONCE as Administrator.
echo.

:: Check for admin rights
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo ERROR: This script must be run as Administrator!
    echo Right-click this file and select "Run as administrator"
    pause
    exit /b 1
)

:: Create the USB reset PowerShell script in a known location
set SCRIPT_DIR=%USERPROFILE%\.ventcon
if not exist "%SCRIPT_DIR%" mkdir "%SCRIPT_DIR%"

:: Write the reset script
(
echo # VentCon2 Arduino USB Reset Script
echo $devices = Get-PnpDevice ^| Where-Object { $_.InstanceId -match 'USB\\VID_2341^&PID_0070' -and $_.Status -eq 'OK' -and $_.Class -eq 'USB' }
echo foreach ^($dev in $devices^) {
echo     Write-Host "Restarting: $($dev.FriendlyName) [$($dev.InstanceId)]"
echo     pnputil /restart-device $dev.InstanceId
echo }
echo Start-Sleep -Seconds 3
echo Write-Host "USB reset complete"
) > "%SCRIPT_DIR%\reset_arduino_usb.ps1"

:: Create scheduled task that runs with highest privileges
schtasks /create /tn "VentCon2_USB_Reset" /tr "powershell.exe -ExecutionPolicy Bypass -File \"%SCRIPT_DIR%\reset_arduino_usb.ps1\"" /sc once /st 00:00 /rl highest /f /ru "%USERNAME%"

if %errorLevel% equ 0 (
    echo.
    echo SUCCESS! USB Reset helper installed.
    echo.
    echo The pre-upload script will now automatically reset the USB device
    echo when the COM port is stuck. No more restarts needed!
    echo.
    echo To manually reset: schtasks /run /tn "VentCon2_USB_Reset"
    echo To uninstall:      schtasks /delete /tn "VentCon2_USB_Reset" /f
) else (
    echo.
    echo ERROR: Failed to create scheduled task.
)

echo.
pause
