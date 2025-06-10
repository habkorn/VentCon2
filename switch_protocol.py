import os
import sys

# Path to platformio.ini
INI_PATH = "platformio.ini"

def set_upload_protocol(protocol):
    """Replace the upload_protocol line in platformio.ini"""
    with open(INI_PATH, 'r') as file:
        lines = file.readlines()
    
    with open(INI_PATH, 'w') as file:
        for line in lines:
            if line.strip().startswith("upload_protocol"):
                file.write(f"upload_protocol = {protocol}\n")
            else:
                file.write(line)
    print(f"Changed upload protocol to: {protocol}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python switch_protocol.py [protocol]")
        print("Example: python switch_protocol.py esptool")
        print("Example: python switch_protocol.py dfu")
        sys.exit(1)
    
    protocol = sys.argv[1]
    set_upload_protocol(protocol)
