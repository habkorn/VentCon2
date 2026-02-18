# Web Resource Compression for ESP32

This project uses gzipped web resources to reduce bandwidth and improve loading times on the ESP32.

## Files to Download

Before running compression, download these files:

1. **Chart.js** (latest version)
   - Download from: https://cdn.jsdelivr.net/npm/chart.js
   - Save as: `chart.min.js`

2. **Logo.svg** 
   - Use your existing VENTREX logo file

## Compression Process

### Option 1: Automatic (Recommended)
```bash
python compress_resources.py
```

### Option 2: Manual using gzip command line
```bash
gzip -9 -k chart.min.js
gzip -9 -k Logo.svg
```

## Expected Compression Results

| File | Original Size | Compressed Size | Savings |
|------|---------------|-----------------|---------|
| chart.min.js | ~200KB | ~60KB | ~70% |
| Logo.svg | Variable | Variable | ~60-80% |

## Upload to ESP32

1. Use Arduino IDE > Tools > ESP32 Sketch Data Upload
2. Upload the compressed files in the `data/` folder to LittleFS
3. The ESP32 web server automatically serves .gz files when browsers support gzip

## Benefits

- **Reduced bandwidth**: 60-70% smaller file sizes
- **Faster loading**: Less data to transfer over WiFi
- **Better user experience**: Faster page loads
- **ESP32 friendly**: Less flash storage usage
