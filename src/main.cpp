#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <Adafruit_ADS1X15.h>
#include "WebContent.h"

// ====== WiFi Access Point Configuration ======
const char* ap_ssid = "VENTCON_AP";
const char* ap_password = "ventcon12!";
const IPAddress ap_ip(192, 168, 4, 1);
const IPAddress ap_gateway(192, 168, 4, 1);
const IPAddress ap_subnet(255, 255, 255, 0);

// ====== Hardware Configuration ======
const int SOLENOID_PIN = 7;      // PWM output pin for solenoid valve
const int PWM_CHANNEL = 0;       // PWM channel for ESP32 LEDC
Adafruit_ADS1015 ads;            // ADC for pressure sensor
DNSServer dnsServer;             // DNS server for captive portal
WebServer server(80);            // Web server on port 80

// Add fallback analog pin for pressure if ADS1015 is not found
const int FALLBACK_ANALOG_PIN = A0;

// ====== ADS1015 Status Flag ======
bool ads_found = true;           // True if ADS1015 is detected

// ====== PID and System Settings ======
struct Settings
{
  double Kp;
  double Ki;
  double Kd;
  float filter_strength;
  double setpoint;
  int pwm_freq;
  int pwm_res;
};

// ====== Default Settings ======
Settings settings = {
  .Kp = 0.5,
  .Ki = 0.05,
  .Kd = 0.01,
  .filter_strength = 0.1,
  .setpoint = 3.0,
  .pwm_freq = 1000,
  .pwm_res = 8
};

// ====== PID and Sensor Variables ======
double Input, Output; // PID input (pressure) and output (PWM)
float filtered_pressure = 0;

// --- Custom PID variables ---
double pid_integral = 0;
double pid_last_error = 0;
unsigned long pid_last_time = 0;

// ====== Sensor Configuration ======
const int ADC_CHANNEL = 0;
const float SENSOR_MIN_BAR = 0.0;
const float SENSOR_MAX_BAR = 10.0;
const int ADC_MIN = 0;
const int ADC_MAX = 2047;

// ====== PWM Update Function ======
void updatePWM()
{
  ledcSetup(PWM_CHANNEL, settings.pwm_freq, settings.pwm_res);
  ledcWrite(PWM_CHANNEL, Output);
}

// ====== Save Settings to SPIFFS ======
void saveSettings()
{
  File file = SPIFFS.open("/settings.json", "w");
  JsonDocument doc;
  doc["kp"] = settings.Kp;
  doc["ki"] = settings.Ki;
  doc["kd"] = settings.Kd;
  doc["flt"] = settings.filter_strength;
  doc["sp"] = settings.setpoint;
  doc["freq"] = settings.pwm_freq;
  doc["res"] = settings.pwm_res;
  serializeJson(doc, file);
  file.close();
}

// ====== Load Settings from SPIFFS ======
void loadSettings()
{
  if (SPIFFS.exists("/settings.json"))
  {
    File file = SPIFFS.open("/settings.json", "r");
    JsonDocument doc;
    deserializeJson(doc, file);
    settings.Kp = doc["kp"] | 0.5;
    settings.Ki = doc["ki"] | 0.05;
    settings.Kd = doc["kd"] | 0.01;
    settings.filter_strength = doc["flt"] | 0.1;
    settings.setpoint = doc["sp"] | 3.0;
    settings.pwm_freq = doc["freq"] | 1000;
    settings.pwm_res = doc["res"] | 8;
    file.close();
  }
}

// ====== Serial Command Parser ======
void parseSerialCommand(String cmd)
{
  cmd.trim();
  cmd.toUpperCase();

  // PID and system parameter commands
  if (cmd.startsWith("KP="))
  {
    settings.Kp = cmd.substring(3).toFloat();
    Serial.printf("Proportional gain set to: %.2f\n", settings.Kp);
    saveSettings();
  }
  else if (cmd.startsWith("KI="))
  {
    settings.Ki = cmd.substring(3).toFloat();
    Serial.printf("Integral gain set to: %.2f\n", settings.Ki);
    saveSettings();
  }
  else if (cmd.startsWith("KD="))
  {
    settings.Kd = cmd.substring(3).toFloat();
    Serial.printf("Derivative gain set to: %.2f\n", settings.Kd);
    saveSettings();
  }
  else if (cmd.startsWith("FLT="))
  {
    float new_flt = cmd.substring(4).toFloat();
    settings.filter_strength = constrain(new_flt, 0.0, 1.0);
    Serial.printf("Filter strength set to: %.2f\n", settings.filter_strength);
    saveSettings();
  }
  else if (cmd.startsWith("SP="))
  {
    settings.setpoint = cmd.substring(3).toFloat();
    Serial.printf("Setpoint updated to: %.2f bar\n", settings.setpoint);
    saveSettings();
  }
  else if (cmd.startsWith("FREQ="))
  {
    int new_freq = cmd.substring(5).toInt();
    settings.pwm_freq = constrain(new_freq, 100, 10000);
    updatePWM();
    Serial.printf("PWM frequency updated to: %d Hz\n", settings.pwm_freq);
    saveSettings();
  }
  else if (cmd.startsWith("RES="))
  {
    int new_res = cmd.substring(4).toInt();
    settings.pwm_res = constrain(new_res, 1, 16);
    pid.SetOutputLimits(0, (1 << settings.pwm_res) - 1);
    updatePWM();
    Serial.printf("PWM resolution updated to: %d bits\n", settings.pwm_res);
    saveSettings();
  }
  // Help and status commands
  else if (cmd == "HELP")
  {
    Serial.println(
      "\n=== Serial Command Help ==="
      "\nKP=0.5    Set proportional gain"
      "\nKI=0.1    Set integral gain"
      "\nKD=0.01   Set derivative gain"
      "\nFLT=0.2   Set filter strength (0.0-1.0)"
      "\nSP=3.0    Set pressure setpoint (bar)"
      "\nFREQ=1000 Set PWM frequency (100-10000Hz)"
      "\nRES=8     Set PWM resolution (1-16 bits)"
      "\nSTATUS    Show current parameters"
      "\nSAVE      Force save settings to flash"
      "\nHELP      Show this help message"
    );
  }
  else if (cmd == "STATUS")
  {
    char status[256];
    snprintf(status, sizeof(status),
      "\n=== System Status ===\n"
      "Setpoint: %.2f bar\n"
      "Pressure: %.2f bar\n"
      "PWM: %d/%d (%.1f%%)\n"
      "PID Gains: Kp=%.2f, Ki=%.2f, Kd=%.2f\n"
      "Filter α: %.2f\n"
      "PWM Config: %dHz, %d-bit\n"
      "IP: %s",
      settings.setpoint, 
      Input,
      (int)Output, 
      (1 << settings.pwm_res) - 1,
      (Output / float((1 << settings.pwm_res) - 1)) * 100.0,
      settings.Kp, 
      settings.Ki, 
      settings.Kd,
      settings.filter_strength,
      settings.pwm_freq, 
      settings.pwm_res,
      WiFi.softAPIP().toString().c_str()
    );
    Serial.println(status);
  }
  else if (cmd == "SAVE")
  {
    saveSettings();
    Serial.println("Settings saved to persistent storage");
  }
  else
  {
    Serial.println("Invalid command. Type 'HELP' for options.");
  }
}

// ====== Arduino Setup Function ======
void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Ventcon System Starting...");
  
  // Try to initialize ADS1015 for up to 2 seconds
  unsigned long ads_start = millis();
  ads_found = false;
  while (millis() - ads_start < 2000)
  {
    if (ads.begin(0x48))
    {
      ads_found = true;
      break;
    }
    delay(100);
  }
  if (!ads_found)
  {
    Serial.println("ERROR: ADS1015 not found! Using analogRead(A0) as fallback for pressure input.");
    // Optionally configure A0 as input (usually not needed on ESP32)
    pinMode(FALLBACK_ANALOG_PIN, INPUT);
  }
  // Only attach PWM if ADS is found
  if (ads_found)
  {
    ledcSetup(PWM_CHANNEL, settings.pwm_freq, settings.pwm_res);
    ledcAttachPin(SOLENOID_PIN, PWM_CHANNEL);
  }
  
  // Initialize SPIFFS for settings storage
  if (!SPIFFS.begin(true))
    Serial.println("SPIFFS error!");
  loadSettings();
  
  // Start WiFi AP and DNS server for captive portal
  WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
  WiFi.softAP(ap_ssid, ap_password);
  dnsServer.start(53, "*", ap_ip);
  
  // Register web server routes
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/values", handleValues);
  server.begin();

  Serial.println("\nSystem Ready - AP Mode");
  Serial.printf("Connect to: %s\nPassword: %s\n", ap_ssid, ap_password);
  Serial.printf("Access via: http://www.ventcon.at\n");
  Serial.print("Direct IP access: http://");
  Serial.println(WiFi.softAPIP());
}

// ====== Arduino Main Loop ======
void loop()
{
  dnsServer.processNextRequest();
  server.handleClient();

  // If ADS1015 not found, skip control loop
  if (!ads_found)
  {
    delay(100);
    return;
  }

  // ====== Sensor Reading ======
  int adc_value;
  if (ads_found)
  {
    int16_t adc_raw = ads.readADC_SingleEnded(ADC_CHANNEL);
    adc_value = adc_raw / 16;
  }
  else
  {
    // Use ESP32 built-in ADC as fallback
    adc_value = analogRead(FALLBACK_ANALOG_PIN);
    // Map ESP32 ADC (0-4095) to expected 0-2047 range
    adc_value = map(adc_value, 0, 4095, 0, 2047);
  }
  float raw_pressure = SENSOR_MIN_BAR + (adc_value - ADC_MIN) * 
                      (SENSOR_MAX_BAR - SENSOR_MIN_BAR) / (ADC_MAX - ADC_MIN);

  // Exponential filter for pressure reading
  filtered_pressure = settings.filter_strength * raw_pressure + 
                     (1 - settings.filter_strength) * filtered_pressure;
  Input = filtered_pressure;

  // ====== Custom PID Calculation ======
  double error = settings.setpoint - Input;
  unsigned long now = millis();
  double dt = (pid_last_time > 0) ? (now - pid_last_time) / 1000.0 : 0.01;
  pid_last_time = now;

  pid_integral += error * dt;
  double derivative = (dt > 0) ? (error - pid_last_error) / dt : 0;
  pid_last_error = error;

  double out = settings.Kp * error + settings.Ki * pid_integral + settings.Kd * derivative;

  // Clamp output to PWM range
  int max_pwm = (1 << settings.pwm_res) - 1;
  if (out < 0) out = 0;
  if (out > max_pwm) out = max_pwm;
  Output = out;

  ledcWrite(PWM_CHANNEL, (uint32_t)Output);

  // ====== Emergency Shutdown ======
  if (Input > SENSOR_MAX_BAR * 1.1)
  {
    ledcWrite(PWM_CHANNEL, 0);
    while(1)
      Serial.println("EMERGENCY SHUTDOWN!");
  }

  // ====== Serial Command Processing ======
  static String serialBuffer;
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n')
    {
      parseSerialCommand(serialBuffer);
      serialBuffer = "";
    }
    else if (c != '\r')
    {
      serialBuffer += c;
    }
  }
}