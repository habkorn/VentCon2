#include <WebServer.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include "WebContent.h"
#include "Constants.h"  // Add this include
#include "Settings.h"   // Add Settings class include
#include "WebHandlers.h" // Include our class definition
#include <PID_v2.h> // Changed from PID_v1 to PID_v2

/*
 * ====== VentCon2 Web Server Implementation ======
 *
 * This file implements a responsive web interface for the VentCon2 pressure control system
 * using Object-Oriented Programming principles. The WebHandler class encapsulates all
 * web server functionality with proper dependency injection.
 */

// WebHandler Class Implementation

WebHandler::WebHandler(WebServer* webServer, 
                       DNSServer* dnsServer,
                       Settings* settings,
                       PID* pid,
                       double* pressureInput,
                       double* pwmOutput,
                       bool* ads_found,
                       int* pwm_max_value,
                       float* last_filtered_pressure,
                       int* connectedClients)
    : server(webServer),
      dnsServer(dnsServer),
      settings(settings),
      pid(pid),
      pressureInput(pressureInput),
      pwmOutput(pwmOutput),
      ads_found(ads_found),
      pwm_max_value(pwm_max_value),
      last_filtered_pressure(last_filtered_pressure),
      connectedClients(connectedClients) {
    // Constructor - all initialization done via member initializer list
}

WebHandler::~WebHandler() {
    // Destructor - no dynamic memory to clean up
}

void WebHandler::updatePWM() {
    ledcSetup(HardwareConfig::PWM_CHANNEL_MOSFET, settings->pwm_freq, settings->pwm_res);
    ledcWrite(HardwareConfig::PWM_CHANNEL_MOSFET, *pwmOutput);
}

// Helper function to get content type based on file extension
String WebHandler::getContentType(String filename) {
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
bool WebHandler::handleFileRead(String path) {
  if (path.endsWith("/")) path += "index.html"; // Default to index.html if path ends with a slash
  String contentType = getContentType(path);
  
  if (LittleFS.exists(path)) {
    File file = LittleFS.open(path, "r");
    server->streamFile(file, contentType);
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
 */
void WebHandler::handleRoot() 
{
  String page = getFullHtmlContent();
  
  // Replace placeholders with current values
  page.replace("%SP%", String(settings->setpoint, 2));
  page.replace("%KP%", String(settings->Kp, 2));
  page.replace("%KI%", String(settings->Ki, 2));
  page.replace("%KD%", String(settings->Kd, 2));
  page.replace("%FLT%", String(settings->filter_strength, 2));
  page.replace("%FREQ%", String(settings->pwm_freq));
  page.replace("%RES%", String(settings->pwm_res));
  page.replace("%PSAMT%", String(settings->pid_sample_time));
  page.replace("%VERSION%", VENTCON_VERSION);  // Add this line to replace version from Constants.h

  server->send(200, "text/html", page);
}

/*
 * handleSet() - Parameter Update Handler
 */
void WebHandler::handleSet() 
{
  // Process parameter updates
  if (server->hasArg("sp")) 
    settings->setpoint = server->arg("sp").toFloat();
  
  if (server->hasArg("kp")) 
    settings->Kp = server->arg("kp").toFloat();
  
  if (server->hasArg("ki")) 
    settings->Ki = server->arg("ki").toFloat();
  
  if (server->hasArg("kd")) 
    settings->Kd = server->arg("kd").toFloat();
  
  if (server->hasArg("flt")) 
    settings->filter_strength = server->arg("flt").toFloat();
  
  if (server->hasArg("freq")) 
  {
    settings->pwm_freq = server->arg("freq").toInt();
    updatePWM();
  }
  
  if (server->hasArg("res")) 
  {
    int old_res = settings->pwm_res;
    int new_res = server->arg("res").toInt();
    
    // Only process if resolution actually changed
    if (old_res != new_res) {
      // Store current duty cycle percentage before changing resolution
      float current_duty_percent = (*pwmOutput / (float)*pwm_max_value) * 100.0;
      
      // Update resolution
      settings->pwm_res = new_res;
      int new_max_value = (1 << settings->pwm_res) - 1;
      
      // Scale pwmOutput to maintain the same duty cycle
      *pwmOutput = (current_duty_percent / 100.0) * new_max_value;
      
      // Update max value and PID limits
      *pwm_max_value = new_max_value;
      pid->SetOutputLimits(0, *pwm_max_value);
      updatePWM();
    }
  }
  
  if (server->hasArg("psamt")) 
  {
    int newSampleTime = server->arg("psamt").toInt();
    newSampleTime = constrain(newSampleTime, 1, 1000);
    settings->pid_sample_time = newSampleTime;
    pid->SetSampleTime(newSampleTime);
  }

  // Update PID and save settings
  pid->SetTunings(settings->Kp, settings->Ki, settings->Kd); // Updated for PID_v2
  settings->save();
  server->send(200, "text/plain", "OK");
}

/*
 * handleValues() - Real-time System Data API
 */
void WebHandler::handleValues() // Send current values as JSON  
{
  const int max_pwm = (1 << settings->pwm_res) - 1;
  const float pwm_percent = max_pwm > 0 ? (*pwmOutput / max_pwm) * 100.0 : 0;
  const char* adc_status = *ads_found ? "100" : "000";
  
  char json[330]; // Increased buffer size for additional fields
  snprintf(json, sizeof(json),
    "{\"sp\":%.2f,\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f,\"flt\":%.2f,"
    "\"freq\":%d,\"res\":%d,\"psamt\":%d,\"client_count\":%d,\"max_clients\":%d,"
    "\"pressure\":%.2f,\"pwm\":%.3f,\"adc_status\":\"%s\"}",
    settings->setpoint,
    settings->Kp,
    settings->Ki,
    settings->Kd,
    settings->filter_strength,
    settings->pwm_freq,
    settings->pwm_res,
    settings->pid_sample_time,
    *connectedClients,
    NetworkConfig::MAX_CLIENTS,  // Use constant from header
    *pressureInput,
    pwm_percent,
    adc_status
  );
  server->send(200, "application/json", json); // why 200? Because this is a successful response
}

// Handler for PID controller reset
void WebHandler::handleResetPID() {
  // Temporarily set to manual mode
  pid->SetMode(PID::Manual);
  *pwmOutput = 0;
  ledcWrite(HardwareConfig::PWM_CHANNEL_MOSFET, 0);
  
  // Clear any internal state
  *last_filtered_pressure = 0;
  
  // Reset internal state (by re-initializing the PID controller)
  pid->SetTunings(settings->Kp, settings->Ki, settings->Kd);
  pid->SetOutputLimits(0, *pwm_max_value);
  
  // Set back to automatic mode
  pid->SetMode(PID::Automatic);
  
  // Send success response
  server->send(200, "application/json", "{\"success\":true,\"message\":\"PID controller reset successfully\"}");
}

// Setup function to register all web handlers
void WebHandler::setupRoutes()
{  
  // Register main page and API endpoints using lambda functions to capture 'this'
  server->on("/", [this](){
    this->handleRoot();
  });
  
  server->on("/set", [this](){
    this->handleSet();
  });
  
  server->on("/values", [this](){
    this->handleValues();
  });
  
  server->on("/resetPID", [this](){
    this->handleResetPID();
  });
  
  // Register handlers for JavaScript files at root level
  server->on("/chart.min.js", [this](){
    this->handleFileRead("/chart.min.js");
  });
  
  server->on("/moment.min.js", [this](){
    this->handleFileRead("/moment.min.js");
  });
  
  server->on("/chartjs-adapter-moment.min.js", [this](){
    this->handleFileRead("/chartjs-adapter-moment.min.js");
  });
  
  // Add handler for the SVG logo
  server->on("/Logo.svg", [this](){
    this->handleFileRead("/Logo.svg");
  });
  
  // Fallback handler for other static files
  server->onNotFound([this]() {
    if (!this->handleFileRead(server->uri())) {
      server->send(404, "text/plain", "404: Not Found");
    }
  });
    server->begin();
}