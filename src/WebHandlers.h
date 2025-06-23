#pragma once

#include <WebServer.h>
#include <DNSServer.h>
#include <WiFi.h>
#include <PID_v2.h>
#include "Settings.h"
#include "Constants.h"

/**
 * WebHandler Class
 * 
 * Provides web server route handlers and functionality for the VentCon2 system.
 * This class handles all HTTP requests, serves static files, and provides
 * API endpoints for system control and monitoring using proper OOP principles.
 */
class WebHandler {
private:
    // Static instance pointer for WiFi event callback
    static WebHandler* instance;
    
    // Owned network components
    WebServer webServer;
    DNSServer webDnsServer;
    
    // References to system components (dependency injection)
    Settings* settings;
    PID* pid;
      // References to system variables
    double* pressureInput;
    double* pwmOutput;
    bool* ads_found;
    int* pwm_max_value;
    float* last_filtered_pressure;
      // WiFi connection management variables
    int connectedClients;
    String connectedMACs[NetworkConfig::MAX_CLIENTS]; // Store MAC addresses of connected clients
    bool webServerEnabled; // Flag to enable/disable web server processing
    
    // WiFi AP configuration constants
    const char* ap_ssid;
    const char* ap_password;
    IPAddress ap_ip;
    IPAddress ap_gateway;
    IPAddress ap_subnet;
      // Private helper methods
    String getContentType(String filename);
    bool handleFileRead(String path);
    
    // WiFi event handler (static wrapper for class method)
    static void onWiFiEventWrapper(WiFiEvent_t event, WiFiEventInfo_t info);
    void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
    
    // Route handlers
    void handleRoot();
    void handleSet();
    void handleValues();
    void handleResetPID();

public:    // Constructor with dependency injection
    WebHandler(Settings* settings,
               PID* pid,
               double* pressureInput,
               double* pwmOutput,
               bool* ads_found,
               int* pwm_max_value,
               float* last_filtered_pressure);
      // Destructor
    ~WebHandler();
    
    // Public interface
    void setupRoutes();    void updatePWM();
    void scanWiFiNetworks();
    void changeWiFiChannel(int channel);
    void setupWiFiEvents();
    void initializeWiFiAP(); // Initialize WiFi Access Point    // WiFi connection management getters
    int getConnectedClients() const { return connectedClients; }
    bool isWebServerEnabled() const { return webServerEnabled; }
    void setWebServerEnabled(bool enabled) { webServerEnabled = enabled; }
    const char* getAPSSID() const { return ap_ssid; }
    const char* getAPPassword() const { return ap_password; }
    IPAddress getAPIP() const { return ap_ip; }
    int getWiFiChannel() const;
    
    // Server access methods
    WebServer& getWebServer() { return webServer; }
    DNSServer& getDNSServer() { return webDnsServer; }
};
