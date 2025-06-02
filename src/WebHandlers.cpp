#include <WebServer.h>
#include <ArduinoJson.h>
#include <DNSServer.h> 
#include "WebContent.h"
#include <PID_v1.h>

// External declarations from main.cpp
extern WebServer server;
extern DNSServer dnsServer;
extern struct Settings {
  double Kp, Ki, Kd;
  float filter_strength;
  double setpoint;
  int pwm_freq;
  int pwm_res;
} settings;

extern double pressureInput, pwmOutput;
extern PID pid;
extern void saveSettings();
extern void updatePWM();
extern bool ads_found;

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
    settings.pwm_res = server.arg("res").toInt();
    updatePWM();
    pid.SetOutputLimits(0, (1 << settings.pwm_res) - 1);
  }

  // Update PID and save settings
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
  saveSettings();
  server.send(200, "text/plain", "OK");
}

void handleValues() 
{
  const int max_pwm = (1 << settings.pwm_res) - 1;
  const float pwm_percent = max_pwm > 0 ? (pwmOutput / max_pwm) * 100.0 : 0;
  const char* adc_status = ads_found ? "100" : "000";
  
  char json[256];
  snprintf(json, sizeof(json),
    "{\"sp\":%.2f,\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f,\"flt\":%.2f,"
    "\"freq\":%d,\"res\":%d,"
    "\"pressure\":%.2f,\"pwm\":%.2f,\"adc_status\":\"%s\"}",
    settings.setpoint,
    settings.Kp,
    settings.Ki,
    settings.Kd,
    settings.filter_strength,
    settings.pwm_freq,
    settings.pwm_res,
    pressureInput,
    pwm_percent,
    adc_status
  );
  
  server.send(200, "application/json", json);
}