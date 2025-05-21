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

extern double Input, Output;
extern PID pid;
extern void saveSettings();
extern void updatePWM();
extern bool ads_found;

void handleRoot() {
  if (!ads_found) {
    // Serve a simple error page if ADS1015 is missing
    server.send(200, "text/html",
      "<!DOCTYPE html><html><head><title>ADS1015 Error</title></head>"
      "<body style='font-family:sans-serif;background:#fee;padding:2em;'>"
      "<h1 style='color:#b00;'>ERROR: ADS1015 Not Found</h1>"
      "<p>The pressure sensor ADC (ADS1015) was not detected.<br>"
      "The control algorithm is disabled.<br>"
      "Please check wiring and power, then reset the device.</p>"
      "</body></html>"
    );
    return;
  }

  String page = FPSTR(HTML_CONTENT);
  
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

void handleSet() {
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
  
  if (server.hasArg("freq")) {
    settings.pwm_freq = server.arg("freq").toInt();
    updatePWM();
  }
  
  if (server.hasArg("res")) {
    settings.pwm_res = server.arg("res").toInt();
    updatePWM();
    pid.SetOutputLimits(0, (1 << settings.pwm_res) - 1);
  }

  // Update PID and save settings
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
  saveSettings();
  server.send(200, "text/plain", "OK");
}

void handleValues() {
  const int max_pwm = (1 << settings.pwm_res) - 1;
  const float pwm_percent = max_pwm > 0 ? (Output / max_pwm) * 100.0 : 0;
  
  char json[128];
  snprintf(json, sizeof(json),
    "{\"sp\":%.2f,\"pressure\":%.2f,\"pwm\":%.2f,\"freq\":%d,\"res\":%d}",
    settings.setpoint,
    Input,
    pwm_percent,
    settings.pwm_freq,
    settings.pwm_res
  );
  
  server.send(200, "application/json", json);
}