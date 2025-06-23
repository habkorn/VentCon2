#pragma once

#include <WebServer.h>
#include <DNSServer.h>
#include <PID_v2.h>
#include "Settings.h"

/**
 * WebHandler Class
 * 
 * Provides web server route handlers and functionality for the VentCon2 system.
 * This class handles all HTTP requests, serves static files, and provides
 * API endpoints for system control and monitoring using proper OOP principles.
 */
class WebHandler {
private:
    // References to system components (dependency injection)
    WebServer* server;
    DNSServer* dnsServer;
    Settings* settings;
    PID* pid;
    
    // References to system variables
    double* pressureInput;
    double* pwmOutput;
    bool* ads_found;
    int* pwm_max_value;
    float* last_filtered_pressure;
    int* connectedClients;
    
    // Private helper methods
    String getContentType(String filename);
    bool handleFileRead(String path);
    
    // Route handlers
    void handleRoot();
    void handleSet();
    void handleValues();
    void handleResetPID();

public:
    // Constructor with dependency injection
    WebHandler(WebServer* webServer, 
               DNSServer* dnsServer,
               Settings* settings,
               PID* pid,
               double* pressureInput,
               double* pwmOutput,
               bool* ads_found,
               int* pwm_max_value,
               float* last_filtered_pressure,
               int* connectedClients);
    
    // Destructor
    ~WebHandler();
    
    // Public interface
    void setupRoutes();
    void updatePWM();
};
