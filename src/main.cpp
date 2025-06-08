#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <PID_v2.h>   // Changed from PID_v1 to PID_v2
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
const int ANALOG_PRESS_PIN = 10;      // Analog output pin for pressure sensor signal 
const int PWM_CHANNEL_MOSFET = 0;       // PWM channel for ESP32 LEDC to switch MOSFET
const int PWM_CHANNEL_ANALOG_PRESS = 1;       // PWM channel for ESP32 LEDC to output analog pressure signal
Adafruit_ADS1015 ads;            // ADC for pressure sensor
DNSServer dnsServer;             // DNS server for captive portal
WebServer server(80);            // Web server on port 80

// Add fallback analog pin for pressure if ADS1015 is not found
const int FALLBACK_ANALOG_PIN = A0;


// ====== ADS1015 Status Flag ======
bool ads_found = false;           // False until ADS1015 is detected


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
  bool antiWindup;  // Flag to enable/disable anti-windup for deadband
  bool hysteresis;  // Flag to enable/disable hysteresis compensation
  float hystAmount; // Amount of hysteresis compensation (percentage points)
};

// ====== Default Settings ======
Settings settings = {
  .Kp = 1.5,
  .Ki = 0.0,
  .Kd = 0.0,
  .filter_strength = 0.1,
  .setpoint = 3.0,
  .pwm_freq = 2000,
  .pwm_res = 12,
  .antiWindup = false,
  .hysteresis = false,
  .hystAmount = 5.0  // Default compensation of 5 percentage points
};

// ====== PID and Sensor Variables ======
double pressureInput, pwmOutput; // PID input (pressure) and output (PWM)
PID pid(&pressureInput, &pwmOutput, &settings.setpoint, settings.Kp, settings.Ki, settings.Kd, DIRECT);
float voltage;
float filtered_pressure = 0;
float raw_pressure = 0;

// ====== Sensor Configuration ======
const int ADC_CHANNEL = 0;
const float MIN_VOLTAGE = 2; // Minimum voltage for pressure sensor
const float MAX_VOLTAGE = 4.09; // Minimum voltage for pressure sensor

const float SENSOR_MIN_BAR = 0.;
const float SENSOR_MAX_BAR = 7.5;

// ====== Valve Configuration ======
const float VALVE_MIN_DUTY = 60.0; // Minimum effective duty cycle for valve (%)
const float VALVE_MAX_DUTY = 90.0; // Maximum effective duty cycle for valve (%)

// Low-pass filter variables
float last_filtered_pressure = 0;  // Previous filtered value

// ====== ADS1015 Configuration ======
const uint8_t ADS1015_I2C_ADDRESS = 0x48; // Default I2C address for ADS1015
const adsGain_t ADS1015_GAIN = GAIN_ONE;  // +/-4.096V range (adjust as needed)
const int ADS1015_DATA_RATE = 1600;       // 1600 samples per second (default)

// ====== Global Variables ======
bool continousValueOutput = false; // Flag for Serial output
long lastMainLoopTime=0; // Last time output was sent to Serial
long lastcontinousValueOutputTime=0; // Last time output was sent to Serial
long lastAnalogOutPressureSignalTime=0; // Last time output was sent to Serial
int PWM_MAX_VALUE = (1 << settings.pwm_res) - 1; // Maximum PWM value based on resolution
bool manualPWMMode = false; // Flag to track manual PWM control mode
int SERIAL_OUTPUT_INTERVAL  = 100; // Interval for continuous serial output in milliseconds
int pwm_analog_pressure_signal_freq=5000; // Frequency for analog pressure signal output
int pwm_analog_pressure_signal_pwm_res=12; // Resolution for analog pressure signal output
int SENSOR_MAX_VALUE = (1 << pwm_analog_pressure_signal_pwm_res) - 1; 

long deltaTimeMainLoop = 0; // Time difference for main loop execution
long deltaTimecontinousValueOutput = 0; // Time difference for continuous value output

// Track previous pressure direction for hysteresis compensation
bool pressureIncreasing = false;
float lastPressure = 0.0;

// ====== PWM Update Function ======
void updatePWM()
{
  ledcSetup(PWM_CHANNEL_MOSFET, settings.pwm_freq, settings.pwm_res);
  ledcWrite(PWM_CHANNEL_MOSFET, pwmOutput);
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
  doc["aw"] = settings.antiWindup;
  doc["hyst"] = settings.hysteresis;  // Save hysteresis setting
  doc["hystamt"] = settings.hystAmount;  // Save hysteresis amount
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
      settings.antiWindup = !doc["aw"].isNull() ? doc["aw"].as<bool>() : true;
      settings.hysteresis = !doc["hyst"].isNull() ? doc["hyst"].as<bool>() : false;
      settings.hystAmount = !doc["hystamt"].isNull() ? doc["hystamt"].as<float>() : 5.0;
      
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
        Serial.printf("Anti-Windup:       %s\n", doc["aw"].as<bool>() ? "Enabled" : "Disabled");
        Serial.printf("Hysteresis Comp:   %s (%.1f%%)\n", 
                    doc["hyst"].as<bool>() ? "Enabled" : "Disabled", 
                    doc["hystamt"].as<float>());
      }
      file.close();
    }
    else
    {
      Serial.println("No settings file found in SPIFFS");
    }
}

// Function to map PID output to effective valve range
uint32_t mapPwmToValve(double pidOutput, int maxPwmValue) 
{
  // Calculate the percentage within PID range (0-100%)
  float pidPercent = (pidOutput / maxPwmValue) * 100.0;
  
  // If below minimum threshold, keep valve closed
  if (pidPercent < 1.0) {
    return 0;
  }
  
  // Apply hysteresis compensation if enabled
  if (settings.hysteresis) {
    // Determine if pressure is currently increasing or decreasing
    bool currentlyIncreasing = pressureInput > lastPressure;
    
    // Apply compensation only when direction changes
    if (currentlyIncreasing != pressureIncreasing) {
      // When changing from increasing to decreasing, add compensation
      if (!currentlyIncreasing) {
        pidPercent += settings.hystAmount;
      }
      // When changing from decreasing to increasing, subtract compensation
      else {
        pidPercent -= settings.hystAmount;
      }
      
      // Update direction flag
      pressureIncreasing = currentlyIncreasing;
    }
  }
  
  // Map PID's 0-100% to valve's effective range
  float mappedPercent = VALVE_MIN_DUTY + (pidPercent / 100.0) * (VALVE_MAX_DUTY - VALVE_MIN_DUTY);
  
  // Constrain to valid range
  mappedPercent = constrain(mappedPercent, 0.0, 100.0);
  
  // Convert percentage back to absolute PWM value
  return (uint32_t)((mappedPercent / 100.0) * maxPwmValue);
}


// ====== Version Information ======
const char* getVersionString() {
  static char versionString[50];
  sprintf(versionString, "2.0.0 (Build: %s %s)", __DATE__, __TIME__);
  return versionString;
}

// ====== Auto-Tuning Variables ======
bool autoTuneRunning = false;
unsigned long autoTuneStartTime = 0;
unsigned long lastTransitionTime = 0;
float autoTuneOutputValue = 0;
float autoTuneSetpoint = 0;
bool autoTuneState = false;
const unsigned long AUTO_TUNE_TIMEOUT = 180000; // 3 minutes timeout
const unsigned long MIN_CYCLE_TIME = 1000; // Minimum cycle time in ms to prevent false transitions
const int AUTO_TUNE_CYCLES = 10; // Number of cycles to collect
int currentCycle = 0;
unsigned long cycleTimes[10]; // Store cycle times
float cycleAmplitudes[10]; // Store amplitudes

// Auto-tune constants for relay method
const float AUTOTUNE_RELAY_STEP = 75.0; // PWM step size in % for relay test
const float AUTOTUNE_TEST_SETPOINT = 3.0; // Target pressure for auto-tuning
const float AUTOTUNE_NOISE_BAND = 0.1; // Deadband to prevent noise-triggered oscillations

// ====== PID Auto-Tuning Function ======
void startAutoTune()
{
  // Save current settings
  autoTuneSetpoint = settings.setpoint;
  
  // Set auto-tune setpoint and prepare system
  settings.setpoint = AUTOTUNE_TEST_SETPOINT;
  
  // Initialize auto-tune variables
  autoTuneRunning = true;
  autoTuneStartTime = millis();
  lastTransitionTime = 0;
  autoTuneState = false;
  currentCycle = 0;
  
  // Set initial output state
  autoTuneOutputValue = AUTOTUNE_RELAY_STEP;
  
  // Switch to manual mode
  manualPWMMode = true;
  
  // Apply initial output
  float pwmValue = (autoTuneOutputValue / 100.0) * PWM_MAX_VALUE;
  ledcWrite(PWM_CHANNEL_MOSFET, mapPwmToValve(pwmValue, PWM_MAX_VALUE));
  
  Serial.println("\n=== PID Auto-Tuning Started ===");
  Serial.printf("Target Setpoint: %.2f bar\n", settings.setpoint);
  Serial.printf("Relay Output: %.1f%%\n", AUTOTUNE_RELAY_STEP);
  Serial.println("Auto-tuning will run for up to 3 minutes or 10 complete cycles");
  Serial.println("Keep system stable and avoid disturbances during tuning");
}

void stopAutoTune(bool calculateParameters = false)
{
  if (calculateParameters && currentCycle >= 3) // Need at least 3 cycles for valid data
  {
    // Calculate average period and amplitude
    unsigned long totalTime = 0;
    float totalAmplitude = 0;
    
    for (int i = 0; i < currentCycle; i++)
    {
      totalTime += cycleTimes[i];
      totalAmplitude += cycleAmplitudes[i];
    }
    
    float avgPeriod = (float)totalTime / currentCycle / 1000.0; // Convert to seconds
    float avgAmplitude = totalAmplitude / currentCycle;
    
    // Calculate ultimate gain Ku and period Tu
    float Ku = (4.0 * AUTOTUNE_RELAY_STEP) / (3.14159 * avgAmplitude);
    float Tu = avgPeriod;
    
    // Calculate PID parameters using Ziegler-Nichols method
    float newKp = 0.6 * Ku;
    float newKi = 1.2 * Ku / Tu;
    float newKd = 0.075 * Ku * Tu;
    
    Serial.println("\n=== Auto-Tuning Results ===");
    Serial.printf("Average Period: %.2f seconds\n", avgPeriod);
    Serial.printf("Average Amplitude: %.2f bar\n", avgAmplitude);
    Serial.printf("Ultimate Gain (Ku): %.2f\n", Ku);
    Serial.printf("Ultimate Period (Tu): %.2f s\n", Tu);
    
    Serial.println("\nCalculated PID Parameters (Ziegler-Nichols):");
    Serial.printf("Kp: %.2f (previously %.2f)\n", newKp, settings.Kp);
    Serial.printf("Ki: %.2f (previously %.2f)\n", newKi, settings.Ki);
    Serial.printf("Kd: %.2f (previously %.2f)\n", newKd, settings.Kd);
    
    // Prompt user to accept or reject values
    Serial.println("\nTo accept these values, type 'TUNE ACCEPT'");
    Serial.println("To reject and keep current values, type 'TUNE REJECT'");
    
    // Store calculated values for later acceptance
    settings.Kp = newKp;
    settings.Ki = newKi;
    settings.Kd = newKd;
  }
  else
  {
    Serial.println("\n=== Auto-Tuning Cancelled ===");
    Serial.println("Not enough cycles collected for reliable tuning.");
  }
  
  // Restore previous settings
  settings.setpoint = autoTuneSetpoint;
  
  // Reset state
  autoTuneRunning = false;
  manualPWMMode = false;
  
  // Reset PID controller
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
  pid.SetMode(PID::Automatic);
}

void processAutoTune()
{
  // Check if timeout occurred
  if (millis() - autoTuneStartTime > AUTO_TUNE_TIMEOUT)
  {
    Serial.println("Auto-tuning timeout reached!");
    stopAutoTune(true); // Calculate with available data
    return;
  }
  
  // Check if we've collected enough cycles
  if (currentCycle >= AUTO_TUNE_CYCLES)
  {
    Serial.println("Auto-tuning complete!");
    stopAutoTune(true);
    return;
  }
  
  // Relay oscillation logic
  if (!autoTuneState && pressureInput > settings.setpoint)
  {
    // Transition from low to high
    unsigned long now = millis();
    
    // Debounce transitions to prevent noise
    if (now - lastTransitionTime > MIN_CYCLE_TIME)
    {
      // Record cycle data if this isn't the first transition
      if (lastTransitionTime > 0)
      {
        cycleTimes[currentCycle] = now - lastTransitionTime;
        cycleAmplitudes[currentCycle] = pressureInput - (settings.setpoint - cycleAmplitudes[currentCycle]);
        currentCycle++;
        
        Serial.printf("Cycle %d: Period=%.2fs, Amplitude=%.2f bar\n", 
                      currentCycle, (now - lastTransitionTime)/1000.0, 
                      cycleAmplitudes[currentCycle-1]);
      }
      else
      {
        // First transition, just store amplitude
        cycleAmplitudes[0] = pressureInput - settings.setpoint;
      }
      
      lastTransitionTime = now;
      autoTuneState = true;
      autoTuneOutputValue = 0; // Set output low
      
      // Apply the output
      float pwmValue = (autoTuneOutputValue / 100.0) * PWM_MAX_VALUE;
      ledcWrite(PWM_CHANNEL_MOSFET, mapPwmToValve(pwmValue, PWM_MAX_VALUE));
    }
  }
  else if (autoTuneState && pressureInput < settings.setpoint - AUTOTUNE_NOISE_BAND)
  {
    // Transition from high to low
    unsigned long now = millis();
    
    // Debounce transitions to prevent noise
    if (now - lastTransitionTime > MIN_CYCLE_TIME)
    {
      lastTransitionTime = now;
      autoTuneState = false;
      autoTuneOutputValue = AUTOTUNE_RELAY_STEP; // Set output high
      
      // Apply the output
      float pwmValue = (autoTuneOutputValue / 100.0) * PWM_MAX_VALUE;
      ledcWrite(PWM_CHANNEL_MOSFET, mapPwmToValve(pwmValue, PWM_MAX_VALUE));
    }
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
    pid.SetOutputLimits(0, PWM_MAX_VALUE);  // No change needed in method signature
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
    
    // Apply PWM value directly (without mapping for manual control)
    ledcWrite(PWM_CHANNEL_MOSFET, (uint32_t)pwmOutput);
    
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
  else if (cmd == "AW ON" || cmd == "AWON")
  {
    settings.antiWindup = true;
    Serial.println("Anti-windup for deadband enabled");
    saveSettings();
  }
  else if (cmd == "AW OFF" || cmd == "AWOFF")
  {
    settings.antiWindup = false;
    Serial.println("Anti-windup for deadband disabled");
    saveSettings();
  }
  else if (cmd == "HYST ON" || cmd == "HYSTON")
  {
    settings.hysteresis = true;
    Serial.println("Hysteresis compensation enabled");
    saveSettings();
  }
  else if (cmd == "HYST OFF" || cmd == "HYSTOFF")
  {
    settings.hysteresis = false;
    Serial.println("Hysteresis compensation disabled");
    saveSettings();
  }
  else if (cmd.startsWith("HYSTAMT "))
  {
    settings.hystAmount = cmd.substring(8).toFloat();
    settings.hystAmount = constrain(settings.hystAmount, 0.0, 20.0);
    Serial.printf("Hysteresis compensation amount set to: %.1f%%\n", settings.hystAmount);
    saveSettings();
  }
  else if (cmd == "HELP")
  {
    Serial.println(
      "\n=== Serial Command Help ==="
      "\nKP 0.5     Set proportional gain"
      "\nKI 0.1     Set integral gain"
      "\nKD 0.01    Set derivative gain"
      "\nFLT 0.2    Set filter strength (0.0-1.0)"
      "\nSP 3.0     Set pressure setpoint (bar)"
      "\nFREQ 1000  Set PWM frequency (100-10000Hz)"
      "\nRES 8      Set PWM resolution (1-16 bits)"
      "\nAW ON      Enable anti-windup for deadband"
      "\nAW OFF     Disable anti-windup for deadband"
      "\nHYST ON    Enable hysteresis compensation"
      "\nHYST OFF   Disable hysteresis compensation"
      "\nHYSTAMT 5  Set hysteresis compensation amount (%)"
      "\nTUNE START Start PID auto-tuning process"
      "\nTUNE STOP  Cancel auto-tuning process"
      "\nTUNE ACCEPT Accept auto-tuned PID parameters"
      "\nTUNE REJECT Reject auto-tuned PID parameters"
      "\nPWM 25     Force PWM duty cycle (0-100%) for testing"
      "\nRESUME     Resume normal PID control after manual PWM control"
      "\nRESET      Reset PID controller (clear integral windup and state)"
      "\nREAD       Read and display settings stored in flash"
      "\nSTATUS     Show current parameters"
      "\nSAVE       Force save settings to flash"
      "\nSTARTCD    Start continuous data output for plotting"
      "\nSTOPCD     Stop continuous data output"
      "\nVER        Display firmware version and build timestamp"
      "\nHELP       Show this help message"
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
    Serial.printf("  Anti-Windup: %s\n", settings.antiWindup ? "Enabled" : "Disabled");
    Serial.printf("  Hysteresis Comp: %s (%.1f%%)\n", 
                 settings.hysteresis ? "Enabled" : "Disabled", settings.hystAmount);
    
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
  else if (cmd == "VER")
  {
    // Display firmware version and build timestamp
    Serial.printf("Firmware Version: %s\n", getVersionString());
  }
  else if (cmd == "RESET")
  {
    // Reset PID controller
    Serial.println("Resetting PID controller...");
    
    // Reset the PID controller by re-initializing it
    pid.SetMode(MANUAL); // 
    pwmOutput = 0;
    ledcWrite(PWM_CHANNEL_MOSFET, 0);
    
    // Clear any internal state
    last_filtered_pressure = 0;
    
    // Re-initialize PID with current settings
    pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
    pid.SetOutputLimits(0, PWM_MAX_VALUE);  // No change needed in method signature
    pid.SetMode(AUTOMATIC);
    
    // Reset manual mode if it was enabled
    manualPWMMode = false;
    
    Serial.println("PID controller reset complete");
  }
  else if (cmd == "TUNE START")
  {
    if (!autoTuneRunning)
    {
      startAutoTune();
    }
    else
    {
      Serial.println("Auto-tuning already running!");
    }
  }
  else if (cmd == "TUNE STOP" || cmd == "TUNE CANCEL")
  {
    if (autoTuneRunning)
    {
      Serial.println("Auto-tuning cancelled by user.");
      stopAutoTune(false);
    }
    else
    {
      Serial.println("No auto-tuning process is running.");
    }
  }
  else if (cmd == "TUNE ACCEPT")
  {
    Serial.println("New PID parameters accepted!");
    saveSettings();
    pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
  }
  else if (cmd == "TUNE REJECT")
  {
    Serial.println("New PID parameters rejected. Reverting to previous values.");
    loadSettings(); // Reload previous settings
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

  ledcSetup(PWM_CHANNEL_ANALOG_PRESS, pwm_analog_pressure_signal_freq, pwm_analog_pressure_signal_pwm_res);
  ledcAttachPin(ANALOG_PRESS_PIN, PWM_CHANNEL_ANALOG_PRESS);

  ledcSetup(PWM_CHANNEL_MOSFET, settings.pwm_freq, settings.pwm_res);
  ledcAttachPin(SOLENOID_PIN, PWM_CHANNEL_MOSFET);
  
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
  pid.SetMode(PID::Automatic);
  pid.SetOutputLimits(0, PWM_MAX_VALUE);  // No change needed in method signature
  pid.SetSampleTime(20); // Set PID sample time to 20ms (50Hz update rate)

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
  // new_filtered = (1-alpha) * measurement + (alpha) * last_filtered
  float filtered = (1-settings.filter_strength) * measurement + (settings.filter_strength) * last_filtered_pressure;
  
  // Store current result for next iteration
  last_filtered_pressure = filtered;
  
  // Return the filtered value
  return filtered;
}

float calculatePressure(float voltage) 
{
  // Handle invalid voltages
  if (voltage < MIN_VOLTAGE) return 0.0;
  return (SENSOR_MAX_BAR-SENSOR_MIN_BAR)/(MAX_VOLTAGE-MIN_VOLTAGE) * (voltage - MIN_VOLTAGE) ;
}





// ====== Arduino Main Loop ======
void loop()
{

  
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
  
  // Save last pressure before updating
  lastPressure = pressureInput;
  
  // Update Input value for PID before computation
  pressureInput = filtered_pressure;


  // ====== Analog Pressure Signal Output ======'
  if (millis()-lastAnalogOutPressureSignalTime >= 10)
  { 
    // Output analog pressure signal to ANALOG_PRESS_PIN
    // Scale pressureInput to PWM range
    // Use SENSOR_MAX_VALUE to ensure it fits within the PWM resolution
    
    ledcWrite(PWM_CHANNEL_ANALOG_PRESS, (uint32_t)(pressureInput/SENSOR_MAX_VALUE*pwm_analog_pressure_signal_pwm_res));   
    lastAnalogOutPressureSignalTime = millis();
  }



  
  // PID calculation and PWM output
  if (!manualPWMMode) 
  {
    // Store previous output for anti-windup check
    double previousOutput = pwmOutput;
    
    // Compute PID output
    pid.Compute();
    
    // Constrain output to valid range
    pwmOutput = constrain(pwmOutput, 0, PWM_MAX_VALUE);
    
    // Anti-windup for deadband: 
    // If enabled, check if we're in the deadband and prevent integral accumulation
    if (settings.antiWindup) 
    {
      float pidPercent = (pwmOutput / PWM_MAX_VALUE) * 100.0;
      
      // If we're below the valve's minimum effective duty cycle and trying to increase output
      if (pidPercent < VALVE_MIN_DUTY && pwmOutput > previousOutput) 
      {
        // Reset the PID to prevent integral accumulation
        pid.SetMode(PID::Manual);
        pid.SetMode(PID::Automatic);
      }
    }
    
    // Use the mapping function to get actual PWM value to apply
    uint32_t actualPwm = mapPwmToValve(pwmOutput, PWM_MAX_VALUE);
    ledcWrite(PWM_CHANNEL_MOSFET, actualPwm);
  }



  // We don't set PWM here if in manual mode since it was set directly in the command handler

  // ====== Emergency Shutdown ======
  if (pressureInput > SENSOR_MAX_BAR * 1.1)
  {
    // Stop PWM output
    ledcWrite(PWM_CHANNEL_MOSFET, 0);
    
    // Use non-blocking approach instead of delay

    static unsigned long lastEmergencyMsgTime = 0;
    if (millis() - lastEmergencyMsgTime >= 1000) 
    {
      Serial.println("EMERGENCY SHUTDOWN! Pressure exceeds safe limit.");
      lastEmergencyMsgTime = millis();
    }
  }

  // ====== Serial Command Processing ======
  static String serialBuffer;
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n') // newline
    {
      parseSerialCommand(serialBuffer);
      serialBuffer = "";
    }
    else if (c != '\r') // Ignore carriage return
    {
      serialBuffer += c; // Append character to buffer
    }
  }
    
  // ====== Continuous Serial Output ======
  deltaTimeMainLoop = millis() - lastMainLoopTime;
  lastMainLoopTime = millis();


  deltaTimecontinousValueOutput = millis() - lastcontinousValueOutputTime;

  
  if(continousValueOutput == true && (deltaTimecontinousValueOutput >= SERIAL_OUTPUT_INTERVAL))
  {
    // Format all data at once using a single buffer
    char buffer[170]; // Buffer size to fit all data
    
    int len = snprintf(buffer, sizeof(buffer),
      "voltage=%.3f, press=%.3f, setPress=%.3f, PWM%%=%.1f, Time (s)=%.2f, deltaTime=%ld%s\r\n",
      voltage,
      pressureInput,
      settings.setpoint,
      (autoTuneRunning ? autoTuneOutputValue : (pwmOutput/PWM_MAX_VALUE)*100.0),
      lastMainLoopTime / 1000.,
      deltaTimeMainLoop,
      autoTuneRunning ? ", TUNING" : "");

    // Send the entire buffer in one operation
    Serial.write(buffer, len);

    lastcontinousValueOutputTime = millis();
  }
  
}