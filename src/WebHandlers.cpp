#include <WebServer.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <SPIFFS.h>
#include "WebContent.h"
#include "Constants.h"  // Add this include
#include <PID_v2.h> // Changed from PID_v1 to PID_v2

/*
 * ====== VentCon2 Web Server Implementation ======
 *
 * This file implements a responsive web interface for the VentCon2 pressure control system.
 * The ESP32 acts as both an access point (AP) and a web server, allowing users to monitor 
 * and control the system through any modern web browser without additional software.
 *
 * === Server Architecture ===
 *
 * 1. Network Layer:
 *    - Creates a WiFi Access Point named "VENTCON_AP"
 *    - Assigns static IP 192.168.4.1
 *    - Implements a DNS server for captive portal functionality
 *    - Resolves all DNS queries to the ESP32's IP, allowing "ventcon.at" domain access
 *
 * 2. Web Server Layer (port 80):
 *    - Serves HTML/CSS/JS content for the main interface
 *    - Handles API endpoints for data exchange
 *    - Serves static files from SPIFFS (JS libraries, SVG logo)
 *    - Implements a 404 handler for unmatched routes
 *
 * 3. API Endpoints:
 *    - GET /         : Main web interface with real-time monitoring and controls
 *    - GET /values   : JSON API returning current system state (pressure, settings, status)
 *    - GET /set?...  : Accepts parameter updates (PID values, setpoint, etc.)
 *    - GET /[file]   : Serves static files from SPIFFS filesystem
 *
 * 4. Data Flow:
 *    - Browser → ESP32: Parameter updates via /set API with URL parameters
 *    - ESP32 → Browser: System state via /values JSON API (polled every 300ms)
 *    - Data displayed through gauges, charts, and status indicators
 *
 * 5. Resources Management:
 *    - Uses PROGMEM for storing HTML content to conserve RAM
 *    - Assembles responses dynamically to insert current values
 *    - Serves static files (JS libraries, SVG) from SPIFFS filesystem
 *    - Minimizes JSON payload sizes for efficient communication
 *
 * 6. Responsive Design:
 *    - Interface adapts to different screen sizes (phones, tablets, desktops)
 *    - Chart.js provides responsive, interactive pressure visualization
 *    - Gauges provide clear visual feedback on system state
 *
 * 7. Performance Considerations:
 *    - Main loop continues execution during server handling (non-blocking)
 *    - Web interface updates optimize for minimal data transfer
 *    - Chart animation disabled for better performance on resource-constrained devices
 *    - Toggle for enabling/disabling chart to reduce CPU load if needed
 *
 * 8. Security:
 *    - Basic WiFi password protection (WPA2)
 *    - No encryption for data in transit (HTTP, not HTTPS)
 *    - No user authentication beyond WiFi access
 *    - Intended for closed industrial systems, not public networks
 *
 * This implementation creates a modern, responsive monitoring and control system
 * that requires no additional software beyond a web browser, making it accessible
 * from any device including phones, tablets, and computers.
 */

// External declarations from main.cpp



extern WebServer server;  // Web server instance
extern DNSServer dnsServer;
extern struct Settings {
  double Kp, Ki, Kd;
  float filter_strength;
  double setpoint;
  int pwm_freq;
  int pwm_res;
  bool antiWindup;
  bool hysteresis;
  float hystAmount;
} settings;

extern double pressureInput, pwmOutput;
extern PID pid;
extern void saveSettings();
extern void updatePWM();
extern bool ads_found;
extern int pwm_max_value; // Make sure to access the global pwm_max_value

// Add external declarations
extern int connectedClients;
// No need to declare MAX_CLIENTS since it's available from Constants.h

// 1. Data Flow from Web to Arduino:

// When a user adjusts values on the web interface, the browser sends an HTTP request to the /set endpoint with parameters (e.g., /set?sp=3.0&kp=1.5)
// The handleSet function processes these parameters using server.hasArg() and server.arg() methods
// The Arduino settings are updated accordingly, and the changes are saved to SPIFFS
// The PID controller is reconfigured when relevant parameters change

// 2. Data Flow from Arduino to Web:

// The web interface periodically requests current values using the /values endpoint
// The handleValues function creates a JSON response with all current values and status information
// This data is sent back to the browser where JavaScript updates the UI elements

// 3. Main Web Interface:

// The handleRoot function serves the HTML content defined in WebContent.h
// This content includes the user interface with sliders, inputs, and status displays

// This two-way communication allows for real-time monitoring and control of the 
// pressure control system through a web browser, which is particularly useful for tuning the 
// PID controller and monitoring system performance.

// Helper function to get content type based on file extension
String getContentType(String filename) {
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".json")) return "application/json";
  else if (filename.endsWith(".svg")) return "image/svg+xml";  // Add SVG MIME type
  else if (filename.endsWith(".png")) return "image/png";
  else if (filename.endsWith(".jpg")) return "image/jpeg";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  return "text/plain";
}

// Handler for serving files from SPIFFS
bool handleFileRead(String path) {
  if (path.endsWith("/")) path += "index.html"; // Default to index.html if path ends with a slash
  String contentType = getContentType(path);
  
  if (SPIFFS.exists(path)) {
    File file = SPIFFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleRoot() 
{
  String page = getFullHtmlContent();
  
  // Replace placeholders with current values
  page.replace("%SP%", String(settings.setpoint, 2));
  page.replace("%KP%", String(settings.Kp, 2));
  page.replace("%KI%", String(settings.Ki, 2));
  page.replace("%KD%", String(settings.Kd, 2));
  page.replace("%FLT%", String(settings.filter_strength, 2));
  page.replace("%FREQ%", String(settings.pwm_freq));
  page.replace("%RES%", String(settings.pwm_res));
  page.replace("%VERSION%", VENTCON_VERSION);  // Add this line to replace version from Constants.h

  server.send(200, "text/html", page);
}

void handleSet() 
{
  // Process parameter updates
  if (server.hasArg("sp")) 
    settings.setpoint = server.arg("sp").toFloat();
  
  if (server.hasArg("kp")) 
    settings.Kp = server.arg("kp").toFloat();
  
  if (server.hasArg("ki")) 
    settings.Ki = server.arg("ki").toFloat();
  
  if (server.hasArg("kd")) 
    settings.Kd = server.arg("kd").toFloat();
  
  if (server.hasArg("flt")) 
    settings.filter_strength = server.arg("flt").toFloat();
  
  if (server.hasArg("freq")) 
  {
    settings.pwm_freq = server.arg("freq").toInt();
    updatePWM();
  }
  
  if (server.hasArg("res")) 
  {
    int old_res = settings.pwm_res;
    int new_res = server.arg("res").toInt();
    
    // Only process if resolution actually changed
    if (old_res != new_res) {
      // Store current duty cycle percentage before changing resolution
      float current_duty_percent = (pwmOutput / (float)pwm_max_value) * 100.0;
      
      // Update resolution
      settings.pwm_res = new_res;
      int new_max_value = (1 << settings.pwm_res) - 1;
      
      // Scale pwmOutput to maintain the same duty cycle
      pwmOutput = (current_duty_percent / 100.0) * new_max_value;
      
      // Update max value and PID limits
      pwm_max_value = new_max_value;
      pid.SetOutputLimits(0, pwm_max_value);
      updatePWM();
    }
  }

  // Update PID and save settings
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd); // Updated for PID_v2
  saveSettings();
  server.send(200, "text/plain", "OK");
}

void handleValues() 
{
  const int max_pwm = (1 << settings.pwm_res) - 1;
  const float pwm_percent = max_pwm > 0 ? (pwmOutput / max_pwm) * 100.0 : 0;
  const char* adc_status = ads_found ? "100" : "000";
  
  char json[320]; // Increased buffer size for additional fields
  snprintf(json, sizeof(json),
    "{\"sp\":%.2f,\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f,\"flt\":%.2f,"
    "\"freq\":%d,\"res\":%d,\"client_count\":%d,\"max_clients\":%d,"
    "\"pressure\":%.2f,\"pwm\":%.2f,\"adc_status\":\"%s\"}",
    settings.setpoint,
    settings.Kp,
    settings.Ki,
    settings.Kd,
    settings.filter_strength,
    settings.pwm_freq,
    settings.pwm_res,
    connectedClients,
    NetworkConfig::MAX_CLIENTS,  // Use constant from header
    pressureInput,
    pwm_percent,
    adc_status
  );
  
  server.send(200, "application/json", json);
}

// Setup function to register all web handlers
void setupWebHandlers() 
{
  // Register main page and API endpoints
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/values", handleValues);
  
  // Register handlers for JavaScript files at root level
  server.on("/chart.min.js", [](){
    handleFileRead("/chart.min.js");
  });
  
  server.on("/moment.min.js", [](){
    handleFileRead("/moment.min.js");
  });
  
  server.on("/chartjs-adapter-moment.min.js", [](){
    handleFileRead("/chartjs-adapter-moment.min.js");
  });
  
  // Add handler for the SVG logo
  server.on("/Logo.svg", [](){
    handleFileRead("/Logo.svg");
  });
  
  // Fallback handler for other static files
  server.onNotFound([]() {
    if (!handleFileRead(server.uri())) {
      server.send(404, "text/plain", "404: Not Found");
    }
  });
  
  server.begin();
}