#include <WebServer.h>
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include "WebContent.h"
#include "Constants.h"  // Add this include
#include "SettingsHandler.h"   // Add SettingsHandler class include
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

// Static instance pointer initialization
// This static pointer allows WiFi event callback functions (which must be static) 
// to access the current WebHandler instance methods. It's used in the WiFi event
// system where ESP32 WiFi events trigger static callback functions that need to
// call instance-specific methods for handling client connections/disconnections.
WebHandler* WebHandler::instance = nullptr;

WebHandler::WebHandler(SettingsHandler* settings,
                       PID* pid,
                       double* pressureInput,
                       double* pwmOutput,
                       bool* ads_found,
                       int* pwm_max_value,
                       float* last_filtered_pressure)
    : webServer(80),
      webDnsServer(),
      settings(settings),
      pid(pid),
      pressureInput(pressureInput),
      pwmOutput(pwmOutput),
      ads_found(ads_found),
      pwm_max_value(pwm_max_value),
      last_filtered_pressure(last_filtered_pressure),
      connectedClients(0),
      webServerEnabled(true),
      ap_ssid(NetworkConfig::AP_SSID),
      ap_password(NetworkConfig::AP_PASSWORD),
      ap_ip(192, 168, 4, 1),
      ap_gateway(192, 168, 4, 1),
      ap_subnet(255, 255, 255, 0) {
    // Set static instance for WiFi event callbacks
    instance = this;
    
    // Initialize connected MACs array
    for (int i = 0; i < NetworkConfig::MAX_CLIENTS; i++) {
        connectedMACs[i] = "";
    }
}

WebHandler::~WebHandler() {
    // Clear static instance
    if (instance == this) {
        instance = nullptr;
    }
}

// WiFi Event Handler Methods
void WebHandler::setupWiFiEvents() {
    WiFi.onEvent(onWiFiEventWrapper);
}

// Static wrapper function for WiFi event callback
void WebHandler::onWiFiEventWrapper(WiFiEvent_t event, WiFiEventInfo_t info) {
    if (instance != nullptr) {
        instance->onWiFiEvent(event, info);
    }
}

// WiFi event handler instance method
void WebHandler::onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
    char macStr[18];
    
    // Serial.printf("WiFi event: %d\n", event); // Debug output to see which events are actually firing
    
    switch(event) {
        case ARDUINO_EVENT_WIFI_AP_STACONNECTED: // Updated event name for newer ESP32 cores
            // A device has connected to the AP
            snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                     info.wifi_ap_staconnected.mac[0], info.wifi_ap_staconnected.mac[1],
                     info.wifi_ap_staconnected.mac[2], info.wifi_ap_staconnected.mac[3],
                     info.wifi_ap_staconnected.mac[4], info.wifi_ap_staconnected.mac[5]);
            
            // Serial.println("AP client connect event triggered!");
            
            if (connectedClients < NetworkConfig::MAX_CLIENTS) {
                // We have room for this client
                connectedMACs[connectedClients] = String(macStr);
                connectedClients++;
                
                Serial.printf("Device connected to AP (%d/%d clients)\n", 
                              connectedClients, NetworkConfig::MAX_CLIENTS);
                Serial.printf("MAC address: %s\n", macStr);
            } else {
                // Too many clients, disconnect this one
                Serial.println("Maximum client limit reached - disconnecting new client");
                Serial.printf("MAC address: %s\n", macStr);
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
            for (int i = 0; i < NetworkConfig::MAX_CLIENTS; i++) {
                if (connectedMACs[i] == disconnectedMAC) {
                    // Shift remaining clients down
                    for (int j = i; j < NetworkConfig::MAX_CLIENTS - 1; j++) {
                        connectedMACs[j] = connectedMACs[j + 1];
                    }
                    connectedMACs[NetworkConfig::MAX_CLIENTS - 1] = ""; // Clear the last slot
                    connectedClients--;
                    break;
                }
            }
              Serial.printf("Clients remaining: %d/%d\n", connectedClients, NetworkConfig::MAX_CLIENTS);
            break;
    }
}

// Initialize WiFi Access Point
void WebHandler::initializeWiFiAP() {
    // Configure WiFi AP
    WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
    WiFi.softAP(ap_ssid, ap_password);
    
    // Start DNS server for captive portal
    webDnsServer.start(53, "*", ap_ip);
    
    Serial.println("WiFi AP initialized:");
    Serial.printf("  SSID: %s\n", ap_ssid);
    Serial.printf("  IP: %s\n", ap_ip.toString().c_str());
    Serial.printf("  Gateway: %s\n", ap_gateway.toString().c_str());
    Serial.printf("  Subnet: %s\n", ap_subnet.toString().c_str());
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
    webServer.streamFile(file, contentType);
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
  page.replace("%VERSION%", VENTCON_VERSION);  // Add this line to replace version from Constants.h

  webServer.send(200, "text/html", page);
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
  const int max_pwm = (1 << settings->pwm_res) - 1;
  const float pwm_percent = max_pwm > 0 ? (*pwmOutput / max_pwm) * 100.0 : 0;
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
    pwm_percent,
    adc_status
  );
  webServer.send(200, "application/json", json); // why 200? Because this is a successful response
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
  webServer.send(200, "application/json", "{\"success\":true,\"message\":\"PID controller reset successfully\"}");
}

// Setup function to register all web handlers
void WebHandler::setupRoutes()
{  
  // Register main page and API endpoints using lambda functions to capture 'this'
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
  webServer.onNotFound([this]() {
    if (!this->handleFileRead(webServer.uri())) {
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
void WebHandler::scanWiFiNetworks() {
    Serial.println("\n=== WiFi Network Scan ===");
    Serial.println("Scanning for available networks...");
    
    // Perform WiFi scan
    int networkCount = WiFi.scanNetworks();
    
    if (networkCount == 0) {
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
    for (int i = 0; i < networkCount; i++) {
        String ssid = WiFi.SSID(i);
        int32_t rssi = WiFi.RSSI(i);
        wifi_auth_mode_t encryption = WiFi.encryptionType(i);
        int32_t channel = WiFi.channel(i);
        
        // Determine signal quality
        String quality;
        int qualityIndex;
        if (rssi > -50) {
            quality = "Excellent";
            qualityIndex = 0;
        } else if (rssi > -60) {
            quality = "Good     ";
            qualityIndex = 1;
        } else if (rssi > -70) {
            quality = "Fair     ";
            qualityIndex = 2;
        } else {
            quality = "Poor     ";
            qualityIndex = 3;
        }
        signalQuality[qualityIndex];
        
        // Count channel usage
        if (channel >= 1 && channel <= 13) {
            channelCount[channel]++;
        }
        
        // Determine encryption type
        String encType;
        switch (encryption) {
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
        if (ssid.length() > 28) {
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
    for (int ch = 1; ch <= 13; ch++) {
        if (channelCount[ch] > 0) {
            Serial.printf("  Channel %2d: %d network(s)", ch, channelCount[ch]);
            
            // Add interference warning for overlapping channels
            if (ch >= 1 && ch <= 3 && (channelCount[1] + channelCount[2] + channelCount[3]) > 3) {
                Serial.print(" [High interference area]");
            } else if (ch >= 6 && ch <= 8 && (channelCount[6] + channelCount[7] + channelCount[8]) > 3) {
                Serial.print(" [High interference area]");
            } else if (ch >= 11 && ch <= 13 && (channelCount[11] + channelCount[12] + channelCount[13]) > 3) {
                Serial.print(" [High interference area]");
            }
            Serial.println();
        }
    }
    
    // Provide recommendations
    Serial.println("\nRecommendations:");
    if (signalQuality[0] + signalQuality[1] > networkCount * 0.7) {
        Serial.println("  • RF environment: Good - Multiple strong signals available");
    } else if (signalQuality[2] + signalQuality[3] > networkCount * 0.7) {
        Serial.println("  • RF environment: Poor - Consider relocating for better reception");
    } else {
        Serial.println("  • RF environment: Mixed - Adequate for most applications");
    }
    
    // Find least congested channels
    int minCount = 999;
    String bestChannels = "";
    for (int ch = 1; ch <= 13; ch += 5) { // Check channels 1, 6, 11 (non-overlapping)
        if (channelCount[ch] < minCount) {
            minCount = channelCount[ch];
            bestChannels = "Channel " + String(ch);
        } else if (channelCount[ch] == minCount && minCount < 3) {
            bestChannels += ", " + String(ch);
        }
    }
    
    if (minCount < 3) {
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
void WebHandler::changeWiFiChannel(int channel) {
    // Validate channel range (2.4GHz channels 1-13)
    if (channel < 1 || channel > 13) {
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
    delay(100); // Brief delay to ensure clean shutdown
    
    Serial.printf("Starting Access Point on channel %d...\n", channel);
    
    // Configure AP with new channel
    WiFi.softAPConfig(currentIP, currentGateway, currentSubnet);
    
    // Start AP on new channel - the channel parameter is the 4th argument
    if (WiFi.softAP(currentSSID.c_str(), currentPassword.c_str(), channel)) {
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
    webDnsServer.start(53, "*", WiFi.softAPIP());
    
    // Restart web server
    webServer.begin();
    
    Serial.println("DNS server and web server restarted");
      // Reset connected clients counter since all clients were disconnected
    connectedClients = 0;
    Serial.println("Client counter reset - clients will reconnect automatically");
    
    // Provide channel analysis
    Serial.println("\nChannel Information:");
    Serial.printf("  New Channel: %d\n", WiFi.channel());
    Serial.printf("  Frequency: %.1f MHz\n", 2412.0 + (WiFi.channel() - 1) * 5.0);
    
    // Suggest monitoring
    Serial.println("\nRecommendations:");
    Serial.println("  • Monitor client reconnection in the next 30 seconds");
    Serial.println("  • Use 'SCAN WIFI' to verify reduced interference");
    Serial.println("  • Use 'STATUS' to check connected client count");
    
    if (channel == 1 || channel == 6 || channel == 11) {
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
