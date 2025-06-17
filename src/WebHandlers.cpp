#include <WebServer.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <LittleFS.h>
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
 *    - Serves static files from LittleFS (JS libraries, SVG logo)
 *    - Implements a 404 handler for unmatched routes
 *
 * 3. API Endpoints:
 *    - GET /         : Main web interface with real-time monitoring and controls
 *    - GET /values   : JSON API returning current system state (pressure, settings, status)
 *    - GET /set?...  : Accepts parameter updates (PID values, setpoint, etc.)
 *    - GET /[file]   : Serves static files from LittleFS filesystem
 *
 * 4. Data Flow:
 *    - Browser → ESP32: Parameter updates via /set API with URL parameters
 *    - ESP32 → Browser: System state via /values JSON API (polled every 300ms)
 *    - Data displayed through gauges, charts, and status indicators
 *
 * 5. Resources Management:
 *    - Uses PROGMEM for storing HTML content to conserve RAM
 *    - Assembles responses dynamically to insert current values
 *    - Serves static files (JS libraries, SVG) from LittleFS filesystem
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
  int pid_sample_time; // Sample time for PID control in milliseconds
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
extern float last_filtered_pressure; // Add declaration for last_filtered_pressure

// Add external declarations
extern int connectedClients;
// No need to declare MAX_CLIENTS since it's available from Constants.h

// 1. Data Flow from Web to Arduino:

// When a user adjusts values on the web interface, the browser sends an HTTP request to the /set endpoint with parameters (e.g., /set?sp=3.0&kp=1.5)
// The handleSet function processes these parameters using server.hasArg() and server.arg() methods
// The Arduino settings are updated accordingly, and the changes are saved to LittleFS
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

// Handler for serving files from LittleFS
bool handleFileRead(String path) {
  if (path.endsWith("/")) path += "index.html"; // Default to index.html if path ends with a slash
  String contentType = getContentType(path);
  
  if (LittleFS.exists(path)) {
    File file = LittleFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

/*
 * handleRoot() - Main Web Interface Handler
 * 
 * This function serves the primary web interface when users navigate to the ESP32's IP address.
 * It's the core of the web-based control system, providing a complete HTML page with embedded
 * CSS and JavaScript for real-time monitoring and control.
 * 
 * Operation Flow:
 * 1. Retrieves the complete HTML template from getFullHtmlContent() (defined in WebContent.h)
 * 2. Performs dynamic placeholder replacement to inject current system values into the HTML
 * 3. Sends the customized HTML page to the client's browser
 * 
 * Placeholder Replacement:
 * - %SP%     → Current pressure setpoint value
 * - %KP%     → PID proportional gain
 * - %KI%     → PID integral gain  
 * - %KD%     → PID derivative gain
 * - %FLT%    → Pressure filter strength
 * - %FREQ%   → PWM frequency setting
 * - %RES%    → PWM resolution (bits)
 * - %PSAMT%  → PID sample time (milliseconds)
 * - %VERSION% → Software version from Constants.h
 * 
 * This approach allows the HTML template to be static (stored in PROGMEM) while still
 * displaying current system values when the page loads. The browser receives a fully
 * populated HTML page that immediately shows the current state without requiring
 * additional AJAX calls for initial values.
 * 
 * Response: Sends HTTP 200 with content-type "text/html" containing the complete interface
 */
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
  page.replace("%PSAMT%", String(settings.pid_sample_time));

  page.replace("%VERSION%", VENTCON_VERSION);  // Add this line to replace version from Constants.h

  server.send(200, "text/html", page);
}

/*
 * handleSet() - Parameter Update Handler
 * 
 * This function processes HTTP GET requests to the /set endpoint, allowing the web interface
 * to update system parameters in real-time. It's called when users adjust sliders, input
 * fields, or other controls on the web interface.
 * 
 * Operation Flow:
 * 1. Receives HTTP GET request with URL parameters (e.g., /set?sp=3.0&kp=1.5&ki=0.2)
 * 2. Checks for each possible parameter using server.hasArg()
 * 3. Updates corresponding settings values using server.arg().toFloat() or .toInt()
 * 4. Applies special handling for PWM frequency/resolution changes
 * 5. Updates PID controller with new tuning parameters
 * 6. Saves all settings to persistent storage (LittleFS)
 * 7. Sends "OK" response to confirm successful update
 * 
 * Supported Parameters:
 * - sp    → Pressure setpoint (double)
 * - kp    → PID proportional gain (double)
 * - ki    → PID integral gain (double)
 * - kd    → PID derivative gain (double)
 * - flt   → Pressure filter strength (float)
 * - freq  → PWM frequency in Hz (int) - triggers updatePWM()
 * - res   → PWM resolution in bits (int) - maintains duty cycle percentage
 * - psamt → PID sample time in milliseconds (int, constrained 1-1000ms)
 * 
 * Special Handling:
 * - PWM Resolution: Calculates current duty cycle percentage before changing resolution,
 *   then scales pwmOutput to maintain the same duty cycle with new resolution
 * - PID Sample Time: Constrains value between 1-1000ms for stability
 * - All changes are immediately applied to the running PID controller
 * 
 * Response: HTTP 200 with "text/plain" content type and "OK" message
 */
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
  
  if (server.hasArg("psamt")) 
  {
    int newSampleTime = server.arg("psamt").toInt();
    newSampleTime = constrain(newSampleTime, 1, 1000);
    settings.pid_sample_time = newSampleTime;
    pid.SetSampleTime(newSampleTime);
  }

  // Update PID and save settings
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd); // Updated for PID_v2
  saveSettings();
  server.send(200, "text/plain", "OK");
}

/*
 * handleValues() - Real-time System Data API
 * 
 * This function provides the core data feed for the web interface's real-time monitoring.
 * It's called periodically (every 250ms) by JavaScript to fetch current system state
 * and update all gauges, displays, and charts without requiring a page refresh.
 * 
 * Operation Flow:
 * 1. Calculates current PWM duty cycle percentage from raw pwmOutput value
 * 2. Determines ADC status based on ADS1115 availability
 * 3. Assembles all system data into a compact JSON response
 * 4. Sends JSON data to the requesting web client
 * 
 * Data Included:
 * - Settings: sp, kp, ki, kd, flt, freq, res, psamt (current configuration)
 * - Status: client_count, max_clients, adc_status (system health)
 * - Real-time: pressure, pwm (current measurements and output)
 * 
 * JSON Response Format:
 * {
 *   "sp": 3.00,           // Setpoint pressure
 *   "kp": 1.50,           // PID proportional gain
 *   "ki": 0.20,           // PID integral gain
 *   "kd": 0.05,           // PID derivative gain
 *   "flt": 0.80,          // Filter strength
 *   "freq": 1000,         // PWM frequency (Hz)
 *   "res": 12,            // PWM resolution (bits)
 *   "psamt": 100,         // PID sample time (ms)
 *   "client_count": 2,    // Currently connected clients
 *   "max_clients": 4,     // Maximum allowed clients
 *   "pressure": 2.85,     // Current pressure reading
 *   "pwm": 65.432,        // PWM output percentage
 *   "adc_status": "100"   // ADC status ("100"=OK, "000"=Error)
 * }
 * 
 * Performance Considerations:
 * - Uses fixed-size buffer (330 chars) for predictable memory usage
 * - snprintf() prevents buffer overflows
 * - Minimal JSON structure reduces network overhead
 * - Called frequently, so must execute quickly
 * 
 * Response: HTTP 200 with "application/json" content type
 */
void handleValues() // Send current values as JSON  
{
  const int max_pwm = (1 << settings.pwm_res) - 1;
  const float pwm_percent = max_pwm > 0 ? (pwmOutput / max_pwm) * 100.0 : 0;
  const char* adc_status = ads_found ? "100" : "000";
  
  char json[330]; // Increased buffer size for additional fields
  snprintf(json, sizeof(json),
    "{\"sp\":%.2f,\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f,\"flt\":%.2f,"
    "\"freq\":%d,\"res\":%d,\"psamt\":%d,\"client_count\":%d,\"max_clients\":%d,"
    "\"pressure\":%.2f,\"pwm\":%.3f,\"adc_status\":\"%s\"}",
    settings.setpoint,
    settings.Kp,
    settings.Ki,
    settings.Kd,
    settings.filter_strength,
    settings.pwm_freq,
    settings.pwm_res,
    settings.pid_sample_time,
    connectedClients,
    NetworkConfig::MAX_CLIENTS,  // Use constant from header
    pressureInput,
    pwm_percent,
    adc_status
  );
    server.send(200, "application/json", json); // why 200? Because this is a successful response
}
// Handler for PID controller reset
void handleResetPID() {
  // Temporarily set to manual mode
  pid.SetMode(PID::Manual);
  pwmOutput = 0;
  ledcWrite(HardwareConfig::PWM_CHANNEL_MOSFET, 0);
  
  // Clear any internal state
  last_filtered_pressure = 0;
  
  // Reset internal state (by re-initializing the PID controller)
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
  pid.SetOutputLimits(0, pwm_max_value);
  
  // Set back to automatic mode
  pid.SetMode(PID::Automatic);
  
  // Send success response
  server.send(200, "application/json", "{\"success\":true,\"message\":\"PID controller reset successfully\"}");
}

// Setup function to register all web handlers
void setupWebHandlers()
{  // Register main page and API endpoints
  // describe API endpoints:  
  // - GET /         : Main web interface (HTML page)
  // - GET /set      : Update parameters (sp, kp, ki, kd, flt, freq, res, psamt)
  // - GET /values   : Get current system values as JSON
  // - GET /resetPID : Reset PID controller to initial state
  // - GET /[file]   : Serve static files from LittleFS (e.g., JS libraries, SVG logo)
  // - GET /Logo.svg : Serve SVG logo file
  // - GET /chart.min.js, /moment.min.js, /chartjs-adapter-moment.min.js : Serve JS libraries
  // - Fallback for other static files with 404 handling
  // - GET /favicon.ico : Serve favicon if requested (not implemented yet)

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/values", handleValues);
  server.on("/resetPID", handleResetPID);
  
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