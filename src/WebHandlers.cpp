#include <WebServer.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include "WebContent.h"
#include "Constants.h"
#include "Logger.h"
#include "SettingsHandler.h"   // Add SettingsHandler class include
#include "WebHandlers.h" // Include our class definition
#include <PID_v2.h> // Changed from PID_v1 to PID_v2

// Helper: derive number of decimal places from a step value
static int decimalsFromStep(float step) {
  if (step <= 0) return 2;
  int d = 0;
  // Multiply by 10 repeatedly until the fractional part vanishes (max 6)
  float v = step;
  while (d < 6 && (v - (int)v) > 0.0001f) { v *= 10.0f; d++; }
  return d;
}

/*
 * ====== VentCon2 Web Server Implementation ======
 *
 * This file implements a responsive web interface for the VentCon2 pressure control system
 * using Object-Oriented Programming principles. The WebHandler class encapsulates all
 * web server functionality with proper dependency injection.
 */

// WebHandler Class Implementation

// Static instance pointer initialization
// This static pointer allows WiFi event callback functions (which must be static) 
// to access the current WebHandler instance methods. It's used in the WiFi event
// system where ESP32 WiFi events trigger static callback functions that need to
// call instance-specific methods for handling client connections/disconnections.
WebHandler* WebHandler::instance = nullptr;

WebHandler::WebHandler(SettingsHandler* settings,
                       PID* pid,
                       double* pressureInput,
                       double* pwmPIDoutput,
                       uint32_t* actualPwm,
                       bool* ads_found,
                       int* pwmFullScaleRaw,
                       float* last_filtered_pressure)
    : webServer(NetworkConfig::WEB_PORT),
      webDnsServer(),
      settings(settings),
      pid(pid),
      pressureInput(pressureInput),
      pwmPIDoutput(pwmPIDoutput),
      actualPwm(actualPwm),
      ads_found(ads_found),
      pwmFullScaleRaw(pwmFullScaleRaw),
      last_filtered_pressure(last_filtered_pressure),
      connectedClients(0),
      webServerEnabled(true),
      ap_ssid(NetworkConfig::AP_SSID),
      ap_password(NetworkConfig::AP_PASSWORD),
      ap_ip(NetworkConfig::AP_IP[0], NetworkConfig::AP_IP[1], NetworkConfig::AP_IP[2], NetworkConfig::AP_IP[3]),
      ap_gateway(NetworkConfig::AP_GATEWAY[0], NetworkConfig::AP_GATEWAY[1], NetworkConfig::AP_GATEWAY[2], NetworkConfig::AP_GATEWAY[3]),
      ap_subnet(NetworkConfig::AP_SUBNET[0], NetworkConfig::AP_SUBNET[1], NetworkConfig::AP_SUBNET[2], NetworkConfig::AP_SUBNET[3])
{
    // Set static instance for WiFi event callbacks
    instance = this;
    
    // Initialize connected MACs array
    for (int i = 0; i < NetworkConfig::MAX_CLIENTS; i++)
    {
        connectedMACs[i] = "";
    }
}

WebHandler::~WebHandler()
{
    // Clear static instance
    if (instance == this)
    {
        instance = nullptr;
    }
}

// WiFi Event Handler Methods
void WebHandler::setupWiFiEvents()
{
    WiFi.onEvent(onWiFiEventWrapper);
}

// Static wrapper function for WiFi event callback
void WebHandler::onWiFiEventWrapper(WiFiEvent_t event, WiFiEventInfo_t info)
{
    if (instance != nullptr)
    {
        instance->onWiFiEvent(event, info);
    }
}

// WiFi event handler instance method
void WebHandler::onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    char macStr[18];
    
    // Serial.printf("WiFi event: %d\n", event); // Debug output to see which events are actually firing
    
    switch(event)
    {
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED: // Updated event name for newer ESP32 cores
            // A device has connected to the AP
            snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                     info.wifi_ap_staconnected.mac[0], info.wifi_ap_staconnected.mac[1],
                     info.wifi_ap_staconnected.mac[2], info.wifi_ap_staconnected.mac[3],
                     info.wifi_ap_staconnected.mac[4], info.wifi_ap_staconnected.mac[5]);
            
            // Serial.println("AP client connect event triggered!");
            
            if (connectedClients < NetworkConfig::MAX_CLIENTS)
            {
                // We have room for this client
                connectedMACs[connectedClients] = String(macStr);
                connectedClients++;
                
                LOG_I(CAT_NETWORK, "Device connected to AP (%d/%d clients)", 
                              connectedClients, NetworkConfig::MAX_CLIENTS);
                LOG_D(CAT_NETWORK, "MAC address: %s", macStr);
            } else {
                // Too many clients, disconnect this one
                LOG_W(CAT_NETWORK, "Maximum client limit reached - disconnecting new client");
                LOG_D(CAT_NETWORK, "MAC address: %s", macStr);
                  // Force disconnection - this requires a brief reset of the AP
                WiFi.softAPdisconnect(false);  // Disconnect all clients but keep AP running
                delay(10); // Brief delay
                
                // Reconfigure AP using class member variables
                WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet); // Reconfigure AP
                WiFi.softAP(ap_ssid, ap_password);  // Restart AP with same settings
            }
            break;
            
        case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED: // Updated event name for newer ESP32 cores
            // A device has disconnected from the AP
            snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                    info.wifi_ap_stadisconnected.mac[0], info.wifi_ap_stadisconnected.mac[1],
                    info.wifi_ap_stadisconnected.mac[2], info.wifi_ap_stadisconnected.mac[3],
                    info.wifi_ap_stadisconnected.mac[4], info.wifi_ap_stadisconnected.mac[5]);
                    
            // Serial.println("AP client disconnect event triggered!");
            // Find and remove the client from our list
            String disconnectedMAC = String(macStr);
            for (int i = 0; i < NetworkConfig::MAX_CLIENTS; i++)
            {
                if (connectedMACs[i] == disconnectedMAC)
                {
                    // Shift remaining clients down
                    for (int j = i; j < NetworkConfig::MAX_CLIENTS - 1; j++)
                    {
                        connectedMACs[j] = connectedMACs[j + 1];
                    }
                    connectedMACs[NetworkConfig::MAX_CLIENTS - 1] = ""; // Clear the last slot
                    connectedClients--;
                    break;
                }
            }
              LOG_I(CAT_NETWORK, "Clients remaining: %d/%d", connectedClients, NetworkConfig::MAX_CLIENTS);
            break;
    }
}

// Initialize WiFi Access Point
void WebHandler::initializeWiFiAP()
{
    // Force clean WiFi state - fixes issue where AP doesn't appear after flash
    WiFi.disconnect(true, true);  // Disconnect and erase stored credentials
    WiFi.mode(WIFI_OFF);          // Turn off WiFi completely
    delay(TimingConfig::WIFI_RESET_DELAY_MS);                   // Allow peripheral to fully reset
    
    // Set WiFi mode to Access Point
    WiFi.mode(WIFI_AP);
    
    // Configure WiFi AP
    WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
    WiFi.softAP(ap_ssid, ap_password, 1, 0, NetworkConfig::MAX_CLIENTS);
    
    // Start DNS server for captive portal
    webDnsServer.start(NetworkConfig::DNS_PORT, "*", ap_ip);
    
    // Start mDNS responder (accessible as http://ventcon.local)
    if (MDNS.begin(NetworkConfig::MDNS_HOSTNAME))
    {
        MDNS.addService("http", "tcp", NetworkConfig::WEB_PORT);
        LOG_I(CAT_NETWORK, "mDNS started - http://%s.local", NetworkConfig::MDNS_HOSTNAME);
    }
    else
    {
        LOG_E(CAT_NETWORK, "mDNS failed to start");
    }
    
    LOG_I(CAT_NETWORK, "WiFi AP initialized - SSID: %s, IP: %s", ap_ssid, ap_ip.toString().c_str());
    LOG_D(CAT_NETWORK, "Gateway: %s, Subnet: %s", ap_gateway.toString().c_str(), ap_subnet.toString().c_str());
}

void WebHandler::updatePWM()
{
    ledcSetup(HardwareConfig::PWM_CHANNEL_MOSFET, settings->pwm_freq, settings->pwm_res);
    ledcWrite(HardwareConfig::PWM_CHANNEL_MOSFET, *pwmPIDoutput);
}

// Helper function to get content type based on file extension
String WebHandler::getContentType(String filename)
{
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".json")) return "application/json";
  else if (filename.endsWith(".svg")) return "image/svg+xml";  // Add SVG MIME type
  else if (filename.endsWith(".png")) return "image/png";
  else if (filename.endsWith(".jpg")) return "image/jpeg";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  else if (filename.endsWith(".pdf")) return "application/pdf";
  return "text/plain";
}

// Handler for serving files from LittleFS with chunked transfer for large files
bool WebHandler::handleFileRead(String path)
{
  if (path.endsWith("/")) path += "index.html";
  String contentType = getContentType(path);
  
  LOG_D(CAT_NETWORK, "File request: %s", path.c_str());
  
  // Check if a gzipped version exists and serve it transparently
  bool isGzipped = false;
  String gzPath = path + ".gz";
  if (LittleFS.exists(gzPath))
  {
    path = gzPath;
    isGzipped = true;
    LOG_D(CAT_NETWORK, "Using gzipped version: %s", gzPath.c_str());
  }

  if (LittleFS.exists(path))
  {
    File file = LittleFS.open(path, "r");
    if (file)
    {
      size_t fileSize = file.size();
      LOG_D(CAT_NETWORK, "Serving: %s (%d bytes, type: %s, gzip: %s)", path.c_str(), fileSize, contentType.c_str(), isGzipped ? "yes" : "no");
      
      if (fileSize == 0)
      {
        LOG_E(CAT_NETWORK, "File is empty: %s", path.c_str());
        file.close();
        webServer.send(500, "text/plain", "File is empty");
        return true;
      }
      
      // Cache static assets (JS libraries, images) for 1 week; skip for dynamic content like JSON
      if (contentType == "application/javascript" || contentType == "image/svg+xml" || 
          contentType == "image/png" || contentType == "image/jpeg")
      {
        webServer.sendHeader("Cache-Control", "public, max-age=604800, immutable");
      }
      
      // Send headers with content length and gzip encoding if applicable
      if (isGzipped)
      {
        webServer.sendHeader("Content-Encoding", "gzip");
      }
      webServer.setContentLength(fileSize);
      webServer.send(200, contentType, "");
      
      // Stream file in chunks to avoid WiFi buffer issues
      const size_t chunkSize = NetworkConfig::CHUNK_SIZE;  // 1KB chunks
      uint8_t buffer[chunkSize];
      size_t bytesRemaining = fileSize;
      size_t bytesSent = 0;
      
      while (bytesRemaining > 0)
      {
        size_t toRead = (bytesRemaining < chunkSize) ? bytesRemaining : chunkSize;
        size_t bytesRead = file.read(buffer, toRead);
        
        if (bytesRead == 0)
        {
          LOG_E(CAT_NETWORK, "Read failed at byte %d for %s", bytesSent, path.c_str());
          break;
        }
        
        webServer.client().write(buffer, bytesRead);
        bytesSent += bytesRead;
        bytesRemaining -= bytesRead;
        
        // Yield to allow WiFi stack to process - critical for large files
        yield();
        delay(1);  // Small delay to prevent WiFi buffer overflow
      }
      
      file.close();
      LOG_D(CAT_NETWORK, "Sent %d/%d bytes for %s", bytesSent, fileSize, path.c_str());
      return true;
    } else {
      LOG_E(CAT_NETWORK, "Could not open file: %s", path.c_str());
    }
  }
  LOG_W(CAT_NETWORK, "File not found: %s", path.c_str());
  return false;
}

/*
 * handleRoot() - Main Web Interface Handler
 * 
 * This function serves the primary web interface when users navigate to the ESP32's IP address.
 * It's the core of the web-based control system, providing a complete HTML page with embedded
 * CSS and JavaScript for real-time monitoring and control.
 */
/*
 * handleRoot() - Memory-Efficient Main Page Handler
 * 
 * This function serves the main web interface using chunked streaming to minimize
 * memory usage. Instead of building the entire ~55KB HTML page in RAM, it:
 * 1. Streams static sections directly from PROGMEM
 * 2. Only processes small sections (~1.5KB) that contain placeholders
 * 
 * Memory usage reduced from ~110KB peak to ~3KB for placeholder processing.
 */
void WebHandler::handleRoot() 
{
  // Set response headers for chunked transfer
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(200, "text/html", "");
  
  // 1. Stream HTML head directly from PROGMEM
  webServer.sendContent_P(HTML_HEAD);
  yield();
  
  // 2. Stream CSS styles directly from PROGMEM
  webServer.sendContent_P(CSS_STYLES);
  yield();
  
  // 3. Stream body start directly from PROGMEM  
  webServer.sendContent_P(HTML_BODY_START);
  yield();
  
  // 4. Process HTML_INPUTS section - this is the only section with placeholders (~1.5KB)
  String inputs = FPSTR(HTML_INPUTS);
  inputs.replace("%SP%", String(settings->setpoint, decimalsFromStep(settings->sp_limits.step)));
  inputs.replace("%KP%", String(settings->Kp, decimalsFromStep(settings->kp_limits.step)));
  inputs.replace("%KI%", String(settings->Ki, decimalsFromStep(settings->ki_limits.step)));
  inputs.replace("%KD%", String(settings->Kd, decimalsFromStep(settings->kd_limits.step)));
  inputs.replace("%FLT%", String(settings->filter_strength, 2));
  inputs.replace("%FREQ%", String(settings->pwm_freq));
  inputs.replace("%RES%", String(settings->pwm_res));
  
  // Replace slider limit placeholders
  inputs.replace("%SP_MIN%", String(settings->sp_limits.min, decimalsFromStep(settings->sp_limits.step)));
  inputs.replace("%SP_MAX%", String(settings->sp_limits.max, decimalsFromStep(settings->sp_limits.step)));
  inputs.replace("%SP_STEP%", String(settings->sp_limits.step, decimalsFromStep(settings->sp_limits.step)));
  inputs.replace("%KP_MIN%", String(settings->kp_limits.min, decimalsFromStep(settings->kp_limits.step)));
  inputs.replace("%KP_MAX%", String(settings->kp_limits.max, decimalsFromStep(settings->kp_limits.step)));
  inputs.replace("%KP_STEP%", String(settings->kp_limits.step, decimalsFromStep(settings->kp_limits.step)));
  inputs.replace("%KI_MIN%", String(settings->ki_limits.min, decimalsFromStep(settings->ki_limits.step)));
  inputs.replace("%KI_MAX%", String(settings->ki_limits.max, decimalsFromStep(settings->ki_limits.step)));
  inputs.replace("%KI_STEP%", String(settings->ki_limits.step, decimalsFromStep(settings->ki_limits.step)));
  inputs.replace("%KD_MIN%", String(settings->kd_limits.min, decimalsFromStep(settings->kd_limits.step)));
  inputs.replace("%KD_MAX%", String(settings->kd_limits.max, decimalsFromStep(settings->kd_limits.step)));
  inputs.replace("%KD_STEP%", String(settings->kd_limits.step, decimalsFromStep(settings->kd_limits.step)));
  
  webServer.sendContent(inputs);
  yield();
  
  // 5. Process footer section - small section with version placeholder
  String footer = FPSTR(HTML_FOOTER);
  footer.replace("%VERSION%", VENTCON_VERSION);
  webServer.sendContent(footer);
  yield();
  
  // 6. Stream JavaScript directly from PROGMEM
  webServer.sendContent_P(HTML_SCRIPT);
  
  // End chunked response
  webServer.sendContent("");
}

/*
 * handleSet() - Parameter Update Handler
 */
void WebHandler::handleSet() 
{
  // Process parameter updates
  if (webServer.hasArg("sp")) 
    settings->setpoint = webServer.arg("sp").toFloat();
  
  if (webServer.hasArg("kp")) 
    settings->Kp = webServer.arg("kp").toFloat();
  
  if (webServer.hasArg("ki")) 
    settings->Ki = webServer.arg("ki").toFloat();
  
  if (webServer.hasArg("kd")) 
    settings->Kd = webServer.arg("kd").toFloat();
  
  if (webServer.hasArg("flt")) 
    settings->filter_strength = webServer.arg("flt").toFloat();
  
  if (webServer.hasArg("freq")) 
  {
    settings->pwm_freq = webServer.arg("freq").toInt();
    updatePWM();
  }
  
  if (webServer.hasArg("res")) 
  {
    int old_res = settings->pwm_res;
    int new_res = webServer.arg("res").toInt();
    
    // Only process if resolution actually changed
    if (old_res != new_res)
    {
      // Store current duty cycle percentage before changing resolution
      float current_duty_percent = (*pwmPIDoutput / (float)*pwmFullScaleRaw) * 100.0;
      
      // Update resolution
      settings->pwm_res = new_res;
      int new_max_value = (1 << settings->pwm_res) - 1;
      
      // Scale pwmPIDoutput to maintain the same duty cycle
      *pwmPIDoutput = (current_duty_percent / 100.0) * new_max_value;
      
      // Update max value and PID limits
      *pwmFullScaleRaw = new_max_value;
      pid->SetOutputLimits(0, *pwmFullScaleRaw);
      updatePWM();
    }
  }
  


  // Update PID and save settings
  pid->SetTunings(settings->Kp, settings->Ki, settings->Kd); // Updated for PID_v2 
  settings->save();
  webServer.send(200, "text/plain", "OK");
}

/*
 * handleValues() - Real-time System Data API
 */
void WebHandler::handleValues() // Send current values as JSON  
{
  // Calculate maximum PWM value based on resolution (e.g., 16383 for 14-bit: 2^14 - 1)
  const int max_output_pwm = (1 << settings->pwm_res) - 1; 

  // Calculate actual valve duty cycle percentage from the mapped PWM value
  const float pwm_actual_percent = max_output_pwm > 0 ? (*actualPwm / (float)max_output_pwm) * 100.0 : 0;
  const char* adc_status = *ads_found ? "100" : "000";
  
  char json[200]; 
  snprintf(json, sizeof(json),
    "{\"sp\":%.2f,\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f,\"flt\":%.2f,"
    "\"freq\":%d,\"res\":%d,"
    "\"pressure\":%.2f,\"pwm\":%.3f,\"adc_status\":\"%s\"}",
    settings->setpoint,
    settings->Kp,
    settings->Ki,
    settings->Kd,
    settings->filter_strength,
    settings->pwm_freq,
    settings->pwm_res,
    *pressureInput,
    pwm_actual_percent,
    adc_status
  );
  webServer.send(200, "application/json", json); // why 200? Because this is a successful response
}

// Handler for PID controller reset
void WebHandler::handleResetPID()
{
  // Temporarily set to manual mode
  pid->SetMode(PID::Manual);
  *pwmPIDoutput = 0;
  ledcWrite(HardwareConfig::PWM_CHANNEL_MOSFET, 0);
  
  // Clear any internal state
  *last_filtered_pressure = 0;
  
  // Reset internal state (by re-initializing the PID controller)
  pid->SetTunings(settings->Kp, settings->Ki, settings->Kd);
  pid->SetOutputLimits(0, *pwmFullScaleRaw);
  
  // Set back to automatic mode
  pid->SetMode(PID::Automatic);
  
  // Send success response
  webServer.send(200, "application/json", "{\"success\":true,\"message\":\"PID controller reset successfully\"}");
}

// Handler for slider limits API (GET/POST)
void WebHandler::handleSliderLimits()
{
  if (webServer.method() == HTTP_GET)
  {
    // Return current slider limits as JSON
    char json[400];
    snprintf(json, sizeof(json),
      "{\"sp\":{\"min\":%.2f,\"max\":%.2f,\"step\":%.3f},"
      "\"kp\":{\"min\":%.2f,\"max\":%.2f,\"step\":%.3f},"
      "\"ki\":{\"min\":%.2f,\"max\":%.2f,\"step\":%.3f},"
      "\"kd\":{\"min\":%.2f,\"max\":%.2f,\"step\":%.3f}}",
      settings->sp_limits.min, settings->sp_limits.max, settings->sp_limits.step,
      settings->kp_limits.min, settings->kp_limits.max, settings->kp_limits.step,
      settings->ki_limits.min, settings->ki_limits.max, settings->ki_limits.step,
      settings->kd_limits.min, settings->kd_limits.max, settings->kd_limits.step
    );
    webServer.send(200, "application/json", json);
  }
  else if (webServer.method() == HTTP_POST)
  {
    // Update slider limits from POST parameters
    String param = webServer.arg("param");
    
    if (param == "sp")
    {
      if (webServer.hasArg("min")) settings->sp_limits.min = webServer.arg("min").toFloat();
      if (webServer.hasArg("max")) settings->sp_limits.max = webServer.arg("max").toFloat();
      if (webServer.hasArg("step")) settings->sp_limits.step = webServer.arg("step").toFloat();
    }
    else if (param == "kp")
    {
      if (webServer.hasArg("min")) settings->kp_limits.min = webServer.arg("min").toFloat();
      if (webServer.hasArg("max")) settings->kp_limits.max = webServer.arg("max").toFloat();
      if (webServer.hasArg("step")) settings->kp_limits.step = webServer.arg("step").toFloat();
    }
    else if (param == "ki")
    {
      if (webServer.hasArg("min")) settings->ki_limits.min = webServer.arg("min").toFloat();
      if (webServer.hasArg("max")) settings->ki_limits.max = webServer.arg("max").toFloat();
      if (webServer.hasArg("step")) settings->ki_limits.step = webServer.arg("step").toFloat();
    }
    else if (param == "kd")
    {
      if (webServer.hasArg("min")) settings->kd_limits.min = webServer.arg("min").toFloat();
      if (webServer.hasArg("max")) settings->kd_limits.max = webServer.arg("max").toFloat();
      if (webServer.hasArg("step")) settings->kd_limits.step = webServer.arg("step").toFloat();
    } else {
      webServer.send(400, "application/json", "{\"success\":false,\"message\":\"Invalid param\"}");
      return;
    }
    
    // Save to LittleFS
    settings->save();
    webServer.send(200, "application/json", "{\"success\":true}");
  }
}

// ============================================================================
// setupRoutes() - Web Server Route Configuration
// ============================================================================
//
// HOW WEB SERVERS WORK:
// A web server listens for HTTP requests from browsers. When you type a URL
// like "http://192.168.4.1/values", the browser sends an HTTP request to the
// ESP32. The server must know what to do for each URL path.
//
// WHAT ARE ROUTES?
// Routes are URL-to-function mappings. Each webServer.on() call says:
// "When someone requests THIS path, call THIS function."
//
// Example:  webServer.on("/values", [this](){ this->handleValues(); });
//           └── path ──┘           └──── function to call ────────┘
//
// WHAT IS A LAMBDA?
// The [this](){ ... } syntax is a "lambda" - an inline anonymous function.
// [this] captures the class instance so we can call this->handleValues().
// It's shorthand for creating a separate callback function.
//
// HTTP METHODS (GET vs POST):
// - GET: Retrieve data (e.g., load a page, fetch sensor values)
// - POST: Send data to server (e.g., submit form, change settings)
// If not specified, the route accepts any method.
//
// SPECIAL ROUTES:
// - "/" is the root/home page (what loads when you first connect)
// - onNotFound() catches any URL not explicitly registered
// - webServer.begin() starts the server after all routes are defined
//
// EXAMPLE: HOW /set IS TRIGGERED
// 1. User adjusts slider in browser → JavaScript stores new value
//    → See WebContentScript.h showSaveSnackbar(): pendingChanges[param] = value
// 2. User clicks "Apply Changes" button
//    → See WebContentScript.h handleSaveClick()
// 3. JavaScript builds URL: "/set?sp=5.0&kp=1.2&ki=0.5&kd=0.1"
//    → See WebContentScript.h: Object.entries(pendingChanges).map(...)
// 4. Browser sends HTTP GET request to ESP32 with that URL
//    → See WebContentScript.h: fetch("/set?" + params)
// 5. ESP32 web server receives request, sees path "/set"
//    → Internal to WebServer library (handles TCP/HTTP parsing)
// 6. Server looks up "/set" in route table → finds handleSet()
//    → See setupRoutes() below: webServer.on("/set", ...)
// 7. handleSet() parses URL parameters (?sp=5.0&kp=...) and updates settings
//    → See handleSet() in this file: webServer.arg("sp").toFloat()
// 8. Server sends response back to browser (success/error)
//    → See handleSet(): webServer.send(200, "text/plain", "OK")
//
// ============================================================================

void WebHandler::setupRoutes()
{  
  // Register main page handler (memory-efficient streaming)
  webServer.on("/", [this](){
    this->handleRoot();
  });
  
  webServer.on("/set", [this](){
    this->handleSet();
  });
  
  webServer.on("/values", [this](){
    this->handleValues();
  });
  
  webServer.on("/resetPID", [this](){
    this->handleResetPID();
  });
  
  webServer.on("/api/slider-limits", HTTP_GET, [this](){
    this->handleSliderLimits();
  });
  
  webServer.on("/api/slider-limits", HTTP_POST, [this](){
    this->handleSliderLimits();
  });
  
  // Register handlers for JavaScript files at root level
  webServer.on("/chart.min.js", [this](){
    this->handleFileRead("/chart.min.js");
  });
  
  webServer.on("/moment.min.js", [this](){
    this->handleFileRead("/moment.min.js");
  });
  
  webServer.on("/chartjs-adapter-moment.min.js", [this](){
    this->handleFileRead("/chartjs-adapter-moment.min.js");
  });
  
  // Add handler for the SVG logo
  webServer.on("/Logo.svg", [this](){
    this->handleFileRead("/Logo.svg");
  });
  
  // Fallback handler for other static files
  webServer.onNotFound([this]()
  {
    if (!this->handleFileRead(webServer.uri()))
    {
      webServer.send(404, "text/plain", "404: Not Found");
    }
  });
    webServer.begin();
}

/*
 * scanWiFiNetworks() - WiFi Network Scanner
 * 
 * This function scans for available WiFi networks and displays them with their
 * signal strength (RSSI), encryption type, and channel information. This is useful
 * for debugging WiFi connectivity issues and understanding the RF environment.
 * 
 * Operation:
 * 1. Initiates a WiFi scan without disconnecting from current AP mode
 * 2. Displays all found networks sorted by signal strength (strongest first)
 * 3. Shows SSID, RSSI (dBm), encryption type, and channel for each network
 * 4. Provides statistics on channel usage and signal quality
 * 
 * Output Format:
 * - Network name, signal strength in dBm, encryption status, channel
 * - Signal quality indicators: Excellent (>-50), Good (-50 to -60), Fair (-60 to -70), Poor (<-70)
 * - Channel distribution summary to help identify interference
 */
void WebHandler::scanWiFiNetworks()
{
    Serial.println("\n=== WiFi Network Scan ===");
    Serial.println("Scanning for available networks...");
    
    // Perform WiFi scan
    int networkCount = WiFi.scanNetworks();
    
    if (networkCount == 0)
    {
        Serial.println("No networks found.");
        return;
    }
    
    Serial.printf("Found %d networks:\n\n", networkCount);
    Serial.println("SSID                          RSSI  Quality    Encryption  Channel");
    Serial.println("-----------------------------------------------------------------------");
    
    // Arrays to track channel usage and signal quality distribution
    int channelCount[14] = {0}; // Channels 1-13 (index 0 unused)
    int signalQuality[4] = {0}; // Excellent, Good, Fair, Poor
    
    // Display networks sorted by signal strength (WiFi.scanNetworks() already sorts by RSSI)
    for (int i = 0; i < networkCount; i++)
    {
        String ssid = WiFi.SSID(i);
        int32_t rssi = WiFi.RSSI(i);
        wifi_auth_mode_t encryption = WiFi.encryptionType(i);
        int32_t channel = WiFi.channel(i);
        
        // Determine signal quality
        String quality;
        int qualityIndex;
        if (rssi > -50)
        {
            quality = "Excellent";
            qualityIndex = 0;
        }
        else if (rssi > -60)
        {
            quality = "Good     ";
            qualityIndex = 1;
        }
        else if (rssi > -70)
        {
            quality = "Fair     ";
            qualityIndex = 2;
        }
        else
        {
            quality = "Poor     ";
            qualityIndex = 3;
        }
        signalQuality[qualityIndex];
        
        // Count channel usage
        if (channel >= 1 && channel <= 13)
        {
            channelCount[channel]++;
        }
        
        // Determine encryption type
        String encType;
        switch (encryption)
        {
            case WIFI_AUTH_OPEN:
                encType = "Open    ";
                break;
            case WIFI_AUTH_WEP:
                encType = "WEP     ";
                break;
            case WIFI_AUTH_WPA_PSK:
                encType = "WPA     ";
                break;
            case WIFI_AUTH_WPA2_PSK:
                encType = "WPA2    ";
                break;
            case WIFI_AUTH_WPA_WPA2_PSK:
                encType = "WPA/WPA2";
                break;
            case WIFI_AUTH_WPA2_ENTERPRISE:
                encType = "WPA2-Ent";
                break;
            case WIFI_AUTH_WPA3_PSK:
                encType = "WPA3    ";
                break;
            default:
                encType = "Unknown ";
                break;
        }
        
        // Truncate long SSIDs for formatting
        if (ssid.length() > 28)
        {
            ssid = ssid.substring(0, 25) + "...";
        }
        
        // Print network information
        Serial.printf("%-30s %4d  %-10s %-9s %7d\n", 
                     ssid.c_str(), rssi, quality.c_str(), encType.c_str(), channel);
    }
    
    // Display summary statistics
    Serial.println("-----------------------------------------------------------------------");
    Serial.printf("Total Networks: %d\n", networkCount);
    
    Serial.println("\nSignal Quality Distribution:");
    Serial.printf("  Excellent (>-50 dBm): %d networks\n", signalQuality[0]);
    Serial.printf("  Good (-50 to -60 dBm): %d networks\n", signalQuality[1]);
    Serial.printf("  Fair (-60 to -70 dBm): %d networks\n", signalQuality[2]);
    Serial.printf("  Poor (<-70 dBm): %d networks\n", signalQuality[3]);
    
    Serial.println("\nChannel Usage (2.4GHz):");
    for (int ch = 1; ch <= 13; ch++)
    {
        if (channelCount[ch] > 0)
        {
            Serial.printf("  Channel %2d: %d network(s)", ch, channelCount[ch]);
            
            // Add interference warning for overlapping channels
            if (ch >= 1 && ch <= 3 && (channelCount[1] + channelCount[2] + channelCount[3]) > 3)
            {
                Serial.print(" [High interference area]");
            }
            else if (ch >= 6 && ch <= 8 && (channelCount[6] + channelCount[7] + channelCount[8]) > 3)
            {
                Serial.print(" [High interference area]");
            }
            else if (ch >= 11 && ch <= 13 && (channelCount[11] + channelCount[12] + channelCount[13]) > 3)
            {
                Serial.print(" [High interference area]");
            }
            Serial.println();
        }
    }
    
    // Provide recommendations
    Serial.println("\nRecommendations:");
    if (signalQuality[0] + signalQuality[1] > networkCount * 0.7)
    {
        Serial.println("  • RF environment: Good - Multiple strong signals available");
    }
    else if (signalQuality[2] + signalQuality[3] > networkCount * 0.7)
    {
        Serial.println("  • RF environment: Poor - Consider relocating for better reception");
    } else {
        Serial.println("  • RF environment: Mixed - Adequate for most applications");
    }
    
    // Find least congested channels
    int minCount = static_cast<int>(AutoTuneConfig::INITIAL_MIN_PRESSURE);
    String bestChannels = "";
    for (int ch = 1; ch <= 13; ch += 5) { // Check channels 1, 6, 11 (non-overlapping)
        if (channelCount[ch] < minCount)
        {
            minCount = channelCount[ch];
            bestChannels = "Channel " + String(ch);
        }
        else if (channelCount[ch] == minCount && minCount < 3)
        {
            bestChannels += ", " + String(ch);
        }
    }
    
    if (minCount < 3)
    {
        Serial.printf("  • Least congested channels: %s (%d networks)\n", bestChannels.c_str(), minCount);
    } else {
        Serial.println("  • All standard channels (1, 6, 11) are congested");
    }
    
    Serial.println("\nScan complete.\n");
}

/*
 * changeWiFiChannel() - WiFi Channel Changer
 * 
 * This function changes the WiFi Access Point channel to optimize performance
 * and avoid interference from other networks. The ESP32 will restart the AP
 * on the new channel while maintaining all client connections.
 * 
 * Operation:
 * 1. Validates the requested channel (1-13 for 2.4GHz)
 * 2. Saves current AP configuration
 * 3. Stops the current Access Point
 * 4. Restarts the AP on the new channel
 * 5. Restarts the DNS server for captive portal functionality
 * 6. Provides feedback on the channel change
 * 
 * Parameters:
 * - channel: WiFi channel number (1-13 for 2.4GHz band)
 * 
 * Notes:
 * - Connected clients will be temporarily disconnected during the switch
 * - Clients should automatically reconnect to the new channel
 * - Channel change takes effect immediately
 * - Use SCAN WIFI first to identify the best channel
 */
void WebHandler::changeWiFiChannel(int channel)
{
    // Validate channel range (2.4GHz channels 1-13)
    if (channel < 1 || channel > 13)
    {
        Serial.printf("Error: Invalid channel %d. Valid range: 1-13\n", channel);
        return;
    }    Serial.printf("\n=== Changing WiFi Channel to %d ===\n", channel);
    
    // Get current AP configuration
    String currentSSID = WiFi.softAPSSID();
    // Note: ESP32 doesn't support retrieving AP password, so we'll use the constant
    String currentPassword = NetworkConfig::AP_PASSWORD;
    IPAddress currentIP = WiFi.softAPIP();
    IPAddress currentGateway = WiFi.gatewayIP();
    IPAddress currentSubnet = WiFi.subnetMask();
    
    Serial.printf("Current AP: %s\n", currentSSID.c_str());
    Serial.printf("Current IP: %s\n", currentIP.toString().c_str());
    Serial.println("Stopping current Access Point...");
      // Stop current services
    webServer.stop();
    webDnsServer.stop();
    
    // Disconnect all clients and stop AP
    WiFi.softAPdisconnect(true);
    delay(TimingConfig::WIFI_RESET_DELAY_MS); // Brief delay to ensure clean shutdown
    
    Serial.printf("Starting Access Point on channel %d...\n", channel);
    
    // Configure AP with new channel
    WiFi.softAPConfig(currentIP, currentGateway, currentSubnet);
    
    // Start AP on new channel - the channel parameter is the 4th argument
    if (WiFi.softAP(currentSSID.c_str(), currentPassword.c_str(), channel))
    {
        Serial.printf("✓ Access Point started successfully on channel %d\n", channel);
        Serial.printf("  SSID: %s\n", currentSSID.c_str());
        Serial.printf("  Channel: %d\n", WiFi.channel());
        Serial.printf("  IP: %s\n", WiFi.softAPIP().toString().c_str());
    } else {
        Serial.printf("✗ Failed to start Access Point on channel %d\n", channel);
        Serial.println("Attempting to restore previous configuration...");
        
        // Try to restore original configuration
        WiFi.softAPConfig(currentIP, currentGateway, currentSubnet);
        WiFi.softAP(currentSSID.c_str(), currentPassword.c_str());
        Serial.println("Access Point restored to automatic channel selection");
    }
      // Restart DNS server for captive portal
    webDnsServer.start(NetworkConfig::DNS_PORT, "*", WiFi.softAPIP());
    
    // Restart mDNS responder
    MDNS.end();
    if (MDNS.begin(NetworkConfig::MDNS_HOSTNAME))
    {
        MDNS.addService("http", "tcp", NetworkConfig::WEB_PORT);
        Serial.printf("mDNS restarted - http://%s.local\n", NetworkConfig::MDNS_HOSTNAME);
    }
    
    // Restart web server
    webServer.begin();
    
    Serial.println("DNS server and web server restarted");
      // Reset connected clients counter since all clients were disconnected
    connectedClients = 0;
    Serial.println("Client counter reset - clients will reconnect automatically");
    
    // Provide channel analysis
    Serial.println("\nChannel Information:");
    Serial.printf("  New Channel: %d\n", WiFi.channel());
    Serial.printf("  Frequency: %.1f MHz\n", NetworkConfig::WIFI_CH1_FREQ_MHZ + (WiFi.channel() - 1) * NetworkConfig::WIFI_CHANNEL_STEP_MHZ);
    
    // Suggest monitoring
    Serial.println("\nRecommendations:");
    Serial.println("  • Monitor client reconnection in the next 30 seconds");
    Serial.println("  • Use 'SCAN WIFI' to verify reduced interference");
    Serial.println("  • Use 'STATUS' to check connected client count");
    
    if (channel == 1 || channel == 6 || channel == 11)
    {
        Serial.printf("  • Channel %d is a standard non-overlapping channel (good choice)\n", channel);
    } else {
        Serial.printf("  • Channel %d may overlap with adjacent channels - monitor for interference\n", channel);
        Serial.println("  • Consider using channels 1, 6, or 11 for best performance");
    }
    
    Serial.println("\nChannel change complete.\n");
}

// WiFi information getter methods
int WebHandler::getWiFiChannel() const {
    return WiFi.channel();
}
