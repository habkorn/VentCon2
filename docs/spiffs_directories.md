# SPIFFS Directory Support

## Key Limitations

SPIFFS does **not** support true hierarchical directories in the traditional sense. However, it does support what are called "virtual directories" through the file naming scheme:

- You can use paths with slashes (e.g., `/js/chart.min.js`)
- These are stored internally as flat filenames with the path as part of the name
- You cannot create or manipulate empty directories
- Directory-specific operations like listing only files in a specific directory are not supported natively

## Practical Implementation

For your VentCon2 system, this means:

1. **File Organization**: 
   - You can organize files with paths like `/js/chart.min.js`
   - These will work correctly when served by the web server

2. **Limitations**:
   - When listing files with your `listFiles()` function, they appear in a flat structure
   - Cannot create empty directories
   - Cannot perform directory operations (mkdir, rmdir)

3. **Recommended Approach**:
   - Continue using path-like filenames (e.g., `/js/chart.min.js`)
   - Your current code in `WebHandlers.cpp` correctly handles these paths
   - When uploading files to SPIFFS, maintain the intended directory structure

## Alternatives

If true directory support is needed, consider:

1. LittleFS - A newer filesystem with better directory support (available for ESP32)
2. FATFS - For SD cards, with full directory support

For the current implementation, SPIFFS with virtual directories should be sufficient for organizing your web assets.
