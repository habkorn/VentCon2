#pragma once
#include <Arduino.h>

// ============================================================================
// Section 1: HTML Head (scripts, meta tags)
// ============================================================================

// PROGMEM stores this string in flash memory instead of RAM.
// On ESP32, flash is much larger than RAM (~4MB vs ~320KB), so large
// constant strings like HTML content should use PROGMEM to avoid
// exhausting the heap. Access requires pgm_read_byte() or helper
// functions, but Arduino String and many ESP32 APIs handle this
// transparently.


//    <!-- Load JS libraries from LittleFS root with deferred loading:
//        The 'defer' attribute downloads scripts in parallel during HTML parsing
//        but executes them in order only after the DOM is fully parsed.
//        This improves page load performance while maintaining script order. -->


const char HTML_HEAD[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <title>VentCon Pressure Control</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <!-- Load JS libraries from LittleFS root with deferred loading.-->
  <script src="/chart.min.js" defer></script>

)rawliteral";
