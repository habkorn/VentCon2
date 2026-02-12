#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include "styles.h" // Include the CSS styles

// ============================================================================
// HTML CONTENT - Split into separate header files for better organization
// ============================================================================
// File naming follows HTML document structure:
//   WebContentHead.h   - <head> section (meta tags, scripts)
//   WebContentBody.h   - <body> start (static HTML content)
//   WebContentInputs.h - Dynamic form inputs with placeholders
//   WebContentFooter.h - <footer> section
//   WebContentScript.h - <script> section (~25KB JavaScript)
// The "WebContent" prefix groups files in the browser; suffix identifies
// the HTML section, making large HTML easier to navigate and maintain.

#include "WebContentHead.h"    // Section 1: HTML Head (scripts, meta tags)
#include "WebContentBody.h"    // Section 2: Body start (static content)
#include "WebContentInputs.h"  // Section 3: Dynamic input fields with placeholders
#include "WebContentFooter.h"  // Section 4: Footer with version placeholder
#include "WebContentScript.h"  // Section 5: JavaScript (~25KB)
