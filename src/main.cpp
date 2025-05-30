#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <PID_v1.h>
#include <Adafruit_ADS1X15.h>
#include "WebContent.h"

// ====== WiFi Access Point Configuration ======
const char* ap_ssid = "VENTCON_AP";
const char* ap_password = "ventcon12!";
const IPAddress ap_ip(192, 168, 4, 1);
const IPAddress ap_gateway(192, 168, 4, 1);
const IPAddress ap_subnet(255, 255, 255, 0);

// ====== Hardware Configuration ======
const int SOLENOID_PIN = 5;      // PWM output pin for solenoid valve
const int PWM_CHANNEL = 0;       // PWM channel for ESP32 LEDC
Adafruit_ADS1015 ads;            // ADC for pressure sensor
DNSServer dnsServer;             // DNS server for captive portal
WebServer server(80);            // Web server on port 80

// Add fallback analog pin for pressure if ADS1015 is not found
const int FALLBACK_ANALOG_PIN = A0;

// ====== ADS1015 Status Flag ======
bool ads_found = false;           // False until ADS1015 is detected
long currentMillis = 0; // Current time in milliseconds

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
  .Kp = 1.5,
  .Ki = 0.0,
  .Kd = 0.0,
  .filter_strength = 0.1,
  .setpoint = 3.0,
  .pwm_freq = 2000,
  .pwm_res = 12
};

// ====== PID and Sensor Variables ======
double pressureInput, pwmOutput; // PID input (pressure) and output (PWM)
PID pid(&pressureInput, &pwmOutput, &settings.setpoint, settings.Kp, settings.Ki, settings.Kd, DIRECT);
float voltage;
float filtered_pressure = 0;
float raw_pressure = 0;

// ====== Sensor Configuration ======
const int ADC_CHANNEL = 0;
const float MIN_VOLTAGE = 0.5; // Minimum voltage for pressure sensor
const float MAX_VOLTAGE = 4.5; // Minimum voltage for pressure sensor

const float SENSOR_MIN_BAR = 0.;
const float SENSOR_MAX_BAR = 10.0;

// Low-pass filter variables
const float ALPHA = 0.2;          // Filter coefficient (0-1): lower = smoother, higher = more responsive
float last_filtered_pressure = 0;  // Previous filtered value

// ====== ADS1015 Configuration ======
const uint8_t ADS1015_I2C_ADDRESS = 0x48; // Default I2C address for ADS1015
const adsGain_t ADS1015_GAIN = GAIN_ONE;  // +/-4.096V range (adjust as needed)
const int ADS1015_DATA_RATE = 1600;       // 1600 samples per second (default)

// ====== Global Variables ======
bool continousValueOutput = false; // Flag for Serial output
long lastContinousValueOutputTime=0; // Last time output was sent to Serial
int PWM_MAX_VALUE = (1 << settings.pwm_res) - 1; // Maximum PWM value based on resolution
bool manualPWMMode = false; // Flag to track manual PWM control mode

// ====== PWM Update Function ======
void updatePWM()
{
  ledcSetup(PWM_CHANNEL, settings.pwm_freq, settings.pwm_res);
  ledcWrite(PWM_CHANNEL, pwmOutput);
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
    DeserializationError error = deserializeJson(doc, file);
    
    if (error) 
    {
      Serial.println("Failed to parse settings.json");
    } else 
    {
      // Alternative approach using isNull() to check if keys exist
      settings.Kp = !doc["kp"].isNull() ? doc["kp"].as<double>() : 0.5;
      settings.Ki = !doc["ki"].isNull() ? doc["ki"].as<double>() : 0.05;
      settings.Kd = !doc["kd"].isNull() ? doc["kd"].as<double>() : 0.01;
      settings.filter_strength = !doc["flt"].isNull() ? doc["flt"].as<float>() : 0.1;
      settings.setpoint = !doc["sp"].isNull() ? doc["sp"].as<double>() : 3.0;
      settings.pwm_freq = !doc["freq"].isNull() ? doc["freq"].as<int>() : 2000;
      settings.pwm_res = !doc["res"].isNull() ? doc["res"].as<int>() : 12;
      
      // Update PWM_MAX_VALUE to match the loaded resolution
      PWM_MAX_VALUE = (1 << settings.pwm_res) - 1;
      
    }
    file.close();
  }
}

void showSettingsFromSPIFFS()
{
    // Read and display settings from SPIFFS without modifying current settings
    if (SPIFFS.exists("/settings.json"))
    {
      File file = SPIFFS.open("/settings.json", "r");
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, file);
      
      if (error) {
        Serial.println("\n=== Error Reading Settings File ===");
        Serial.printf("deserializeJson() failed: %s\n", error.c_str());
      } else {
        Serial.println("\n=== Settings Stored in SPIFFS ===");
        Serial.printf("PID Parameters:    Kp=%.2f, Ki=%.2f, Kd=%.2f\n", 
                    doc["kp"].as<double>(), doc["ki"].as<double>(), doc["kd"].as<double>());
        Serial.printf("Filter Strength:   %.2f\n", doc["flt"].as<float>());
        Serial.printf("Pressure Setpoint: %.2f bar\n", doc["sp"].as<double>());
        Serial.printf("PWM Configuration: %d Hz, %d-bit (max value: %d)\n", 
                    doc["freq"].as<int>(), doc["res"].as<int>(), 
                    (1 << doc["res"].as<int>()) - 1);
      }
      file.close();
    }
    else
    {
      Serial.println("No settings file found in SPIFFS");
    }
}

// ====== Serial Command Parser ======
void parseSerialCommand(String cmd)
{
  cmd.trim();
  cmd.toUpperCase();

  // PID and system parameter commands
  if (cmd.startsWith("KP "))
  {
    settings.Kp = cmd.substring(3).toFloat();
    pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
    Serial.printf("Proportional gain set to: %.2f\n", settings.Kp);
    saveSettings();
  }
  else if (cmd.startsWith("KI "))
  {
    settings.Ki = cmd.substring(3).toFloat();
    pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
    Serial.printf("Integral gain set to: %.2f\n", settings.Ki);
    saveSettings();
  }
  else if (cmd.startsWith("KD "))
  {
    settings.Kd = cmd.substring(3).toFloat();
    pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
    Serial.printf("Derivative gain set to: %.2f\n", settings.Kd);
    saveSettings();
  }
  else if (cmd.startsWith("FLT "))
  {
    float new_flt = cmd.substring(4).toFloat();
    settings.filter_strength = constrain(new_flt, 0.0, 1.0);
    Serial.printf("Filter strength set to: %.2f\n", settings.filter_strength);
    saveSettings();
  }
  else if (cmd.startsWith("SP "))
  {
    settings.setpoint = cmd.substring(3).toFloat();
    Serial.printf("Setpoint updated to: %.2f bar\n", settings.setpoint);
    saveSettings();
  }
  else if (cmd.startsWith("FREQ "))
  {
    int new_freq = cmd.substring(5).toInt();
    settings.pwm_freq = constrain(new_freq, 100, 10000);
    updatePWM();
    Serial.printf("PWM frequency updated to: %d Hz\n", settings.pwm_freq);
    saveSettings();
  }
  else if (cmd.startsWith("RES "))
  {
    int new_res = cmd.substring(4).toInt();
    settings.pwm_res = constrain(new_res, 1, 16);
    PWM_MAX_VALUE = (1 << settings.pwm_res) - 1; // Update max value when resolution changes
    pid.SetOutputLimits(0, PWM_MAX_VALUE);
    updatePWM();
    Serial.printf("PWM resolution updated to: %d bits\n", settings.pwm_res);
    saveSettings();
  }
  else if (cmd.startsWith("PWM "))
  {
    // Force PWM duty cycle for testing (overrides PID)
    float duty_percent = cmd.substring(4).toFloat();
    // Constrain to valid percentage range
    duty_percent = constrain(duty_percent, 0.0, 100.0);
    // Convert percentage to absolute PWM value
    pwmOutput = (duty_percent / 100.0) * PWM_MAX_VALUE;
    // Apply PWM value
    ledcWrite(PWM_CHANNEL, (uint32_t)pwmOutput);
    Serial.printf("PWM manually set to: %.1f%% (%d/%d)\n", 
                 duty_percent, (int)pwmOutput, PWM_MAX_VALUE);
                 
    // Set manual mode flag
    manualPWMMode = true;
    Serial.println("WARNING: Manual PWM control active - PID control suspended");
    Serial.println("         Use RESUME command to return to PID control");
  }
  else if (cmd == "RESUME")
  {
    // Resume PID control
    manualPWMMode = false;
    Serial.println("PID control resumed");
  }
  else if (cmd == "READ")
  {
    showSettingsFromSPIFFS();
  }
  else if (cmd == "HELP")
  {
    Serial.println(
      "\n=== Serial Command Help ==="
      "\nKP 0.5    Set proportional gain"
      "\nKI 0.1    Set integral gain"
      "\nKD 0.01   Set derivative gain"
      "\nFLT 0.2   Set filter strength (0.0-1.0)"
      "\nSP 3.0    Set pressure setpoint (bar)"
      "\nFREQ 1000 Set PWM frequency (100-10000Hz)"
      "\nRES 8     Set PWM resolution (1-16 bits)"
      "\nPWM 25    Force PWM duty cycle (0-100%) for testing"
      "\nRESUME    Resume normal PID control after manual PWM control"
      "\nREAD      Read and display settings stored in flash"
      "\nSTATUS    Show current parameters"
      "\nSAVE      Force save settings to flash"
      "\nSTARTCD   Start continuous data output for plotting"
      "\nSTOPCD    Stop continuous data output"
      "\nHELP      Show this help message"
    );
  }
  else if (cmd == "STATUS")
  {
    // Break up status into multiple smaller messages instead of one large buffer
    Serial.println("\n=== System Status ===");
    
    // Pressure Control section
    Serial.println("Pressure Control:");
    Serial.printf("  Current Pressure: %.2f bar\n", pressureInput);
    Serial.printf("  Setpoint: %.2f bar\n", settings.setpoint);
    
    // PWM Output section
    Serial.println("\nPWM Output:");
    Serial.printf("  Value: %d/%d (%.1f%%)\n", 
                 (int)pwmOutput, PWM_MAX_VALUE, 
                 (pwmOutput / float(PWM_MAX_VALUE)) * 100.0);
    Serial.printf("  Resolution: %d-bit\n", settings.pwm_res);
    Serial.printf("  Frequency: %d Hz\n", settings.pwm_freq);
    Serial.printf("  Control Mode: %s\n", manualPWMMode ? "MANUAL" : "PID");
    Serial.printf("  Solenoid Pin: %d\n", SOLENOID_PIN);
    
    // PID Configuration section
    Serial.println("\nPID Configuration:");
    Serial.printf("  Kp=%.2f, Ki=%.2f, Kd=%.2f\n", 
                 settings.Kp, settings.Ki, settings.Kd);
    Serial.printf("  Filter Strength: %.2f\n", settings.filter_strength);
    
    // Network section
    Serial.println("\nNetwork:");
    Serial.printf("  IP: %s\n", WiFi.softAPIP().toString().c_str());
  }
  else if (cmd == "SAVE")
  {
    saveSettings();
    Serial.println("Settings saved to persistent storage");
  }
  else if (cmd == "STARTCD")
  {
    Serial.println("Starting output for Continuous Data");
    continousValueOutput = true;
  }
  else if (cmd == "STOPCD")
  {
    Serial.println("Stopping output for Continuous Data");
    continousValueOutput = false;
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
  Serial.println();
  Serial.println("====================================================");
  Serial.println("Ventcon System Starting...");
  Serial.println("====================================================");
  
  // Try to initialize ADS1015 for up to 2 seconds
  unsigned long ads_start = millis();
  ads_found = false;
  while (millis() - ads_start < 2000)
  {
    if (ads.begin(ADS1015_I2C_ADDRESS))
    {
      ads_found = true;
      // Set ADS1015 gain and data rate after detection
      ads.setGain(ADS1015_GAIN);
      ads.setDataRate(ADS1015_DATA_RATE);
      break;
    }
    delay(100);
  }

  if (!ads_found)
  {
    Serial.println("WARNING: ADS1015 not found! Using analogRead(A0) as fallback for pressure input.");
    // Optionally configure A0 as input (usually not needed on ESP32)
    pinMode(FALLBACK_ANALOG_PIN, INPUT);
  }
  // Only attach PWM if ADS is found
  if (ads_found)
  {
    Serial.println("ADS1015 found! ");

  }

  ledcSetup(PWM_CHANNEL, settings.pwm_freq, settings.pwm_res);
  ledcAttachPin(SOLENOID_PIN, PWM_CHANNEL);
  
  // Initialize SPIFFS for settings storage
  if (!SPIFFS.begin(true))
    Serial.println("SPIFFS error!");

  loadSettings();
  showSettingsFromSPIFFS(); // Show loaded settings on startup
  
  // Start WiFi AP and DNS server for captive portal
  WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
  WiFi.softAP(ap_ssid, ap_password);
  dnsServer.start(53, "*", ap_ip);
  
  // Initialize PID 
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0, PWM_MAX_VALUE);  // Use the calculated value directly

  // Register web server routes (endpoints)
  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/values", handleValues);
  server.begin();  // Uncomment this line to enable the web server

  Serial.println("\nSystem Ready - AP Mode");
  Serial.println("\nType HELP for command options");
  Serial.printf("Connect to: %s\nPassword: %s\n", ap_ssid, ap_password);
  Serial.printf("Access via: http://www.ventcon.at\n");
  Serial.print("Direct IP access: http://");
  Serial.println(WiFi.softAPIP());
}

// Low-pass filter function
float lowPassFilter(float measurement) 
{
  // Apply exponential low-pass filter
  // new_filtered = alpha * measurement + (1-alpha) * last_filtered
  float filtered = settings.filter_strength * measurement + (1 - settings.filter_strength) * last_filtered_pressure;
  
  // Store current result for next iteration
  last_filtered_pressure = filtered;
  
  // Return the filtered value
  return filtered;
}

float calculatePressure(float voltage) 
{
  // Handle invalid voltages
  if (voltage < MIN_VOLTAGE) return 0.0;
  return (SENSOR_MAX_BAR-SENSOR_MIN_BAR)/(MAX_VOLTAGE-MIN_VOLTAGE) * (voltage - MIN_VOLTAGE) + SENSOR_MIN_BAR;
}

// ====== Arduino Main Loop ======
void loop()
{
  // ====== Update Current Time ======
  currentMillis = millis();
  
  // ====== DNS and Web Server Processing ======
  dnsServer.processNextRequest();
  server.handleClient();

  // ====== Sensor Reading and PID Update ======
  int adc_value;

  if (ads_found)
  {
    int16_t adc_raw = ads.readADC_SingleEnded(ADC_CHANNEL);
    // Remove redundant second reading that isn't used
    voltage = ads.computeVolts(adc_raw);
  }
  else
  {
    // Use ESP32 built-in ADC as fallback
    adc_value = analogRead(FALLBACK_ANALOG_PIN);

    voltage = adc_value * (3.3 / 4095.0); // Assuming 3.3V reference
  }

  // Calculate pressure from voltage
  raw_pressure = calculatePressure(voltage);
 
  // Apply low-pass filter
  filtered_pressure = lowPassFilter(raw_pressure);
  
  // 3. Update Input value for PID before computation
  pressureInput = filtered_pressure;
  
  // PID calculation and PWM output
  if (!manualPWMMode) 
  {
    pid.Compute();
    pwmOutput = constrain(pwmOutput, 0, PWM_MAX_VALUE); // Constrain output to valid range
    ledcWrite(PWM_CHANNEL, (uint32_t)pwmOutput);
  }
  // We don't set PWM here if in manual mode since it was set directly in the command handler

  // ====== Emergency Shutdown ======
  if (pressureInput > SENSOR_MAX_BAR * 1.1)
  {
    // Stop PWM output
    ledcWrite(PWM_CHANNEL, 0);
    
    // Use non-blocking approach instead of delay

    static unsigned long lastEmergencyMsgTime = 0;
    if (millis() - lastEmergencyMsgTime >= 1000) {
      Serial.println("EMERGENCY SHUTDOWN! Pressure exceeds safe limit.");
      lastEmergencyMsgTime = millis();
    }
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
    

  if(continousValueOutput == true && (millis() - lastContinousValueOutputTime >= 250))
  {
    // Output in format suitable for data plotting
    Serial.print("voltage=");
    Serial.print(voltage, 3);
    Serial.print(", ");

    Serial.print("press=");
    Serial.print(pressureInput, 1);
    Serial.print(", ");

    Serial.print("setPress=");
    Serial.print(settings.setpoint, 1); // Fix: print setpoint, not Input again
    Serial.print(", ");
 
    Serial.print("PWM=");
    Serial.println((pwmOutput/PWM_MAX_VALUE)*100.0); // Fix: use floating point, add newline
    
    lastContinousValueOutputTime = millis();
  }
}