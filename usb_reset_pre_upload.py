"""
Pre-upload script for PlatformIO that resets the Arduino Nano ESP32 USB device
before uploading. This fixes the ESP32-S3 USB CDC issue where the COM port
becomes inaccessible after the first upload.

Requires one-time admin setup: run install_usb_reset_helper.bat as Administrator.
This creates a scheduled task that can reset USB without admin on each upload.
"""
import ctypes
from ctypes import wintypes
import subprocess
import time
import sys
import serial
import serial.tools.list_ports

Import("env")

# Arduino Nano ESP32 USB identifiers
ARDUINO_VID = 0x2341
ARDUINO_PID = 0x0070
SCHEDULED_TASK_NAME = "VentCon2_USB_Reset"


def find_arduino_port():
    """Find the Arduino Nano ESP32 COM port."""
    for port in serial.tools.list_ports.comports():
        if port.vid == ARDUINO_VID and port.pid == ARDUINO_PID:
            return port.device, port.hwid
    return None, None


def try_1200_baud_touch(port_name):
    """Try the Arduino 1200-baud touch to trigger bootloader reset."""
    try:
        print(f"  Attempting 1200-baud touch on {port_name}...")
        ser = serial.Serial(port_name, 1200)
        ser.setDTR(False)
        time.sleep(0.5)
        ser.close()
        time.sleep(2)  # Wait for re-enumeration
        print(f"  1200-baud touch successful!")
        return True
    except Exception as e:
        print(f"  1200-baud touch failed: {e}")
        return False


def reset_usb_via_scheduled_task():
    """
    Trigger the pre-installed scheduled task to reset the USB device.
    The task runs with elevated privileges (set up once via install_usb_reset_helper.bat).
    """
    try:
        print(f"  Triggering scheduled task '{SCHEDULED_TASK_NAME}'...")
        result = subprocess.run(
            ["schtasks", "/run", "/tn", SCHEDULED_TASK_NAME],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            print("  USB reset task triggered - waiting for device re-enumeration...")
            time.sleep(5)  # Wait for USB device to reset and re-enumerate
            return True
        else:
            stderr = result.stderr.strip()
            print(f"  Scheduled task failed: {stderr}")
            if "cannot be found" in stderr.lower() or "nicht gefunden" in stderr.lower() or "not found" in stderr.lower():
                print("  --> Run 'install_usb_reset_helper.bat' as Administrator first!")
            return False
    except FileNotFoundError:
        print("  schtasks not found")
        return False
    except Exception as e:
        print(f"  Scheduled task error: {e}")
        return False


def test_port_accessible(port_name):
    """Test if a COM port can actually be opened."""
    try:
        ser = serial.Serial(port_name, 115200, timeout=1)
        ser.close()
        return True
    except Exception:
        return False


def before_upload(source, target, env):
    """PlatformIO pre-upload hook."""
    print("\n" + "=" * 50)
    print("VentCon2 Pre-Upload USB Check")
    print("=" * 50)
    
    port_name, hwid = find_arduino_port()
    
    if not port_name:
        print("  No Arduino Nano ESP32 found on any COM port.")
        print("  Please check USB connection or double-tap RESET on the board.")
        return
    
    print(f"  Arduino detected on {port_name}")
    
    # Test if the port is actually accessible
    if test_port_accessible(port_name):
        print(f"  {port_name} is accessible - good to upload!")
        return
    
    print(f"  {port_name} is NOT accessible - USB CDC is in a bad state")
    
    # Try the scheduled task approach (requires one-time admin setup)
    print("  Attempting automated USB reset...")
    if reset_usb_via_scheduled_task():
        # Check if port is now accessible
        time.sleep(2)
        new_port, _ = find_arduino_port()
        if new_port and test_port_accessible(new_port):
            print(f"  Success! {new_port} is now accessible")
            return
        elif new_port:
            # Port found but not yet accessible, try 1200-baud touch
            if try_1200_baud_touch(new_port):
                return
            print(f"  Port {new_port} found but still not accessible after reset")
        else:
            # Wait a bit more for re-enumeration
            print("  Waiting for device re-enumeration...")
            for i in range(5):
                time.sleep(2)
                new_port, _ = find_arduino_port()
                if new_port and test_port_accessible(new_port):
                    print(f"  Success! {new_port} is now accessible")
                    return
            print("  Device did not re-enumerate after USB reset")
    
    print("\n  *** MANUAL ACTION REQUIRED ***")
    print("  The USB CDC port is stuck. Try one of these:")
    print("  1. Double-tap the RESET button on the Arduino Nano ESP32")
    print("  2. Unplug and re-plug the USB cable")
    print("  3. First-time setup: run 'install_usb_reset_helper.bat' as Admin")
    print("")


# Register the pre-upload hook
env.AddPreAction("upload", before_upload)
