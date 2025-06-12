# LittleFS Migration Guide

## Overview

VentCon2 has been migrated from SPIFFS to LittleFS filesystem for improved reliability and features.

## What Changed

### Code Changes
- All `#include <SPIFFS.h>` changed to `#include <LittleFS.h>`
- All `SPIFFS.` calls replaced with `LittleFS.`
- Function names updated (e.g., `showSettingsFromSPIFFS()` → `showSettingsFromLittleFS()`)
- Comments and documentation updated

### Files Modified
- `src/main.cpp` - Core filesystem operations
- `src/WebHandlers.cpp` - Web file serving
- `src/WebContent.h` - Documentation comments

## Benefits of LittleFS over SPIFFS

### 1. **Better Directory Support**
- True hierarchical directories (not just virtual paths)
- Can create, list, and manipulate directories
- Better file organization capabilities

### 2. **Improved Reliability**
- More robust error handling
- Better power-loss protection
- Wear leveling improvements

### 3. **Enhanced Performance**
- Faster file operations
- More efficient space utilization
- Better memory usage

### 4. **Future-Proof**
- SPIFFS is deprecated in newer ESP32 frameworks
- LittleFS is actively maintained and supported
- Better compatibility with modern toolchains

## Migration Impact

### Data Compatibility
- **IMPORTANT**: Existing SPIFFS data will NOT be automatically migrated
- Settings stored in `/settings.json` will need to be reconfigured
- Web assets in `/data` folder will need to be re-uploaded

### Upload Process
The upload process remains the same:
```bash
# Upload filesystem data
platformio run --target uploadfs
```

### File Structure
The file structure in `/data` folder remains unchanged:
```
data/
├── chart.min.js
├── chartjs-adapter-moment.min.js
├── Logo.svg
├── moment.min.js
└── settings.json
```

## Post-Migration Steps

1. **Re-upload filesystem data**:
   ```bash
   platformio run --target uploadfs
   ```

2. **Reconfigure settings** via web interface or serial commands

3. **Verify functionality**:
   - Check web interface loads correctly
   - Verify settings persistence
   - Test file operations

## Troubleshooting

### Common Issues

1. **"LittleFS error!" on startup**
   - Run `platformio run --target uploadfs` to initialize filesystem
   - Check that `data/` folder contains required files

2. **Settings not persisting**
   - Verify LittleFS initialization successful
   - Check serial output for filesystem errors
   - Use `LISTFILES` command to verify `/settings.json` exists

3. **Web assets not loading**
   - Ensure `data/` folder uploaded successfully
   - Check file paths match exactly (case-sensitive)
   - Verify file permissions

### Serial Commands for Debugging
```
LISTFILES   - List all files in LittleFS
READ        - Display current settings from filesystem
STATUS      - Show system status including filesystem usage
```

## Development Notes

### File Operations
LittleFS uses the same API as SPIFFS for basic operations:
```cpp
// File operations remain the same
File file = LittleFS.open("/settings.json", "r");
LittleFS.exists("/settings.json");
file.close();
```

### Directory Operations (New Capability)
```cpp
// Now possible with LittleFS
LittleFS.mkdir("/logs");
LittleFS.rmdir("/temp");
```

### Size and Usage
```cpp
// Filesystem information
LittleFS.totalBytes();
LittleFS.usedBytes();
```

## Backward Compatibility

This migration breaks compatibility with existing SPIFFS-based firmware. If reverting is necessary:

1. Restore SPIFFS includes and function calls
2. Re-upload SPIFFS filesystem data
3. Reconfigure settings

However, we recommend staying with LittleFS for future development due to its superior features and ongoing support.
