#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <PID_v2.h>
#include <Adafruit_ADS1X15.h>
#include "WebContent.h"
#include "Constants.h"  // Add this include

// External function declarations
extern void setupWebHandlers();

// ====== WiFi Access Point Configuration ======
const char* ap_ssid = NetworkConfig::AP_SSID;
const char* ap_password = NetworkConfig::AP_PASSWORD;
const IPAddress ap_ip(192, 168, 4, 1);
const IPAddress ap_gateway(192, 168, 4, 1);
const IPAddress ap_subnet(255, 255, 255, 0);

const int MAX_CLIENTS = NetworkConfig::MAX_CLIENTS; // Maximum number of clients allowed

// ====== Hardware Configuration ======
const int SOLENOID_PIN = HardwareConfig::SOLENOID_PIN;
const int ANALOG_PRESS_PIN = HardwareConfig::ANALOG_PRESS_PIN;
const int PWM_CHANNEL_MOSFET = HardwareConfig::PWM_CHANNEL_MOSFET;
const int PWM_CHANNEL_ANALOG_PRESS = HardwareConfig::PWM_CHANNEL_ANALOG_PRESS;
Adafruit_ADS1015 ads;            // ADC for pressure sensor
DNSServer dnsServer;             // DNS server for captive portal
WebServer server(80);            // Web server on port 80

// Add fallback analog pin for pressure if ADS1015 is not found
const int FALLBACK_ANALOG_PIN = HardwareConfig::FALLBACK_ANALOG_PIN; // ESP32 internal ADC pin

// ====== Valve Configuration ======

const float VALVE_MIN_DUTY = ValveConfig::VALVE_MIN_DUTY; 
const float VALVE_MAX_DUTY = ValveConfig::VALVE_MAX_DUTY;

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
  int pid_sample_time; // Sample time for PID control in milliseconds
  int control_freq_hz; // Control loop frequency in Hz
  bool antiWindup;  // Flag to enable/disable anti-windup for deadband
  bool hysteresis;  // Flag to enable/disable hysteresis compensation
  float hystAmount; // Amount of hysteresis compensation (percentage points)
};

// ====== Default Settings ======
Settings DEFAULT_SETTINGS = 
{
  .Kp = 0.0,
  .Ki = 0.0,
  .Kd = 0.0,
  .filter_strength = 0.,
  .setpoint = 3.0,
  .pwm_freq = 2000,
  .pwm_res = 14,
  .pid_sample_time = 10, // Sample time in milliseconds
  .control_freq_hz = 1000, // Default 1000Hz control loop
  .antiWindup = false,
  .hysteresis = false,
  .hystAmount = 5.0  // Default compensation of 5 percentage points
};

// ====== Global Settings Variable ======
Settings settings =   
{
  .Kp = DEFAULT_SETTINGS.Kp,
  .Ki = DEFAULT_SETTINGS.Ki,
  .Kd = DEFAULT_SETTINGS.Kd,
  .filter_strength = DEFAULT_SETTINGS.filter_strength,
  .setpoint = DEFAULT_SETTINGS.setpoint,
  .pwm_freq = DEFAULT_SETTINGS.pwm_freq,
  .pwm_res = DEFAULT_SETTINGS.pwm_res,
  .pid_sample_time = DEFAULT_SETTINGS.pid_sample_time,
  .control_freq_hz = DEFAULT_SETTINGS.control_freq_hz,
  .antiWindup = DEFAULT_SETTINGS.antiWindup,
  .hysteresis = DEFAULT_SETTINGS.hysteresis,
  .hystAmount = DEFAULT_SETTINGS.hystAmount  // Default compensation of 5 percentage points
};



// ====== WiFi Connection Management ======
int connectedClients = 0;
String connectedMACs[NetworkConfig::MAX_CLIENTS]; // Store MAC addresses of connected clients
bool webServerEnabled = true; // Flag to enable/disable web server processing

// ====== PID and Sensor Variables ======
double pressureInput, pwmOutput; // PID input (pressure) and output (PWM)
PID pid(&pressureInput, &pwmOutput, &settings.setpoint, settings.Kp, settings.Ki, settings.Kd, DIRECT);
float voltage;
float filtered_pressure = 0;
float raw_pressure = 0;

// ====== Sensor Configuration ======
const int ADC_CHANNEL = SensorConfig::ADC_CHANNEL; // ADC channel for pressure sensor
const float MIN_VOLTAGE = SensorConfig::MIN_VOLTAGE; // Minimum voltage for pressure sensor
const float MAX_VOLTAGE = SensorConfig::MAX_VOLTAGE; // Minimum voltage for pressure sensor

const float SENSOR_MIN_BAR = SensorConfig::SENSOR_MIN_BAR; // Minimum pressure in bar
const float SENSOR_MAX_BAR = SensorConfig::SENSOR_MAX_BAR; // Maximum pressure in bar


// Low-pass filter variables
float last_filtered_pressure = 0;  // Previous filtered value

// ====== ADS1015 Configuration ======
const uint8_t ADS1015_I2C_ADDRESS = 0x48; // Default I2C address for ADS1015
const adsGain_t ADS1015_GAIN = GAIN_TWOTHIRDS;  // +/-6.144V range (for 0-5V signals)

const uint16_t ADS1015_DATA_RATE = RATE_ADS1015_1600SPS;   //  Data rate for ADS1015 (Default)


// ====== Global Variables ======
bool continousValueOutput = false; // Flag for Serial output
long lastcontinousValueOutputTime=0; // Last time output was sent to Serial
long lastAnalogOutPressureSignalTime=0; // Last time output was sent to Serial
int pwm_max_value = (1 << settings.pwm_res) - 1; // Maximum PWM value based on resolution
bool manualPWMMode = false; // Flag to track manual PWM control mode
int SERIAL_OUTPUT_INTERVAL  = 100; // Interval for continuous serial output in milliseconds
int pwm_analog_pressure_signal_freq=5000; // Frequency for analog pressure signal output
int pwm_analog_pressure_signal_pwm_res=12; // Resolution for analog pressure signal output
int SENSOR_MAX_VALUE = (1 << pwm_analog_pressure_signal_pwm_res) - 1;
TaskHandle_t networkTaskHandle = NULL;
TaskHandle_t controlTaskHandle = NULL;

int16_t adc_value;  // Variable to store ADC value 

// Track previous pressure direction for hysteresis compensation
bool pressureIncreasing = false;
float lastPressure = 0.0;



// ====== PWM Update Function ======
void updatePWM()
{
  ledcSetup(PWM_CHANNEL_MOSFET, settings.pwm_freq, settings.pwm_res);
  ledcWrite(PWM_CHANNEL_MOSFET, pwmOutput);
}

// ====== Save Settings to LittleFS ======
void saveSettings()
{
  // Save implementation remains the same
  File file = LittleFS.open("/settings.json", "w");
  if (file)
  {
    JsonDocument doc;
    doc["Kp"] = settings.Kp;
    doc["Ki"] = settings.Ki;
    doc["Kd"] = settings.Kd;
    doc["filter_strength"] = settings.filter_strength;
    doc["setpoint"] = settings.setpoint;
    doc["pwm_freq"] = settings.pwm_freq;
    doc["pwm_res"] = settings.pwm_res;
    doc["pid_sample_time"] = settings.pid_sample_time;
    doc["control_freq_hz"] = settings.control_freq_hz;
    doc["antiWindup"] = settings.antiWindup;
    doc["hysteresis"] = settings.hysteresis;
    doc["hystAmount"] = settings.hystAmount;
    
    serializeJson(doc, file);
    file.close();
    Serial.println("Settings saved to LittleFS");
  }
  else
  {
    Serial.println("Error: Unable to save settings!");
  }
}

// ====== Load Settings from LittleFS ======
void loadSettings()
{
  if (LittleFS.exists("/settings.json"))
  {
    File file = LittleFS.open("/settings.json", "r");
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    
    if (error) 
    {
      Serial.println("Failed to parse settings.json");
    } else 
    {
      // Alternative approach using isNull() to check if keys exist      
      settings.Kp = !doc["Kp"].isNull() ? doc["Kp"].as<double>() : DEFAULT_SETTINGS.Kp;
      settings.Ki = !doc["Ki"].isNull() ? doc["Ki"].as<double>() : DEFAULT_SETTINGS.Ki;
      settings.Kd = !doc["Kd"].isNull() ? doc["Kd"].as<double>() : DEFAULT_SETTINGS.Kd;
      settings.filter_strength = !doc["filter_strength"].isNull() ? doc["filter_strength"].as<float>() : DEFAULT_SETTINGS.filter_strength;
      settings.setpoint = !doc["setpoint"].isNull() ? doc["setpoint"].as<double>() : DEFAULT_SETTINGS.setpoint;
      settings.pwm_freq = !doc["pwm_freq"].isNull() ? doc["pwm_freq"].as<int>() : DEFAULT_SETTINGS.pwm_freq;
      settings.pwm_res = !doc["pwm_res"].isNull() ? doc["pwm_res"].as<int>() : DEFAULT_SETTINGS.pwm_res;
      settings.pid_sample_time = !doc["pid_sample_time"].isNull() ? doc["pid_sample_time"].as<int>() : DEFAULT_SETTINGS.pid_sample_time;
      settings.control_freq_hz = !doc["control_freq_hz"].isNull() ? doc["control_freq_hz"].as<int>() : DEFAULT_SETTINGS.control_freq_hz;
      settings.antiWindup = !doc["antiWindup"].isNull() ? doc["antiWindup"].as<bool>() : DEFAULT_SETTINGS.antiWindup;
      settings.hysteresis = !doc["hysteresis"].isNull() ? doc["hysteresis"].as<bool>() : DEFAULT_SETTINGS.hysteresis;
      settings.hystAmount = !doc["hystAmount"].isNull() ? doc["hystAmount"].as<float>() : DEFAULT_SETTINGS.hystAmount;
      
      // Update PWM_MAX_VALUE to match the loaded resolution
      pwm_max_value = (1 << settings.pwm_res) - 1;
    }
    file.close();
  }
}

void showSettingsFromLittleFS()
{
    // Read and display settings from LittleFS without modifying current settings
    if (LittleFS.exists("/settings.json"))
    {
      File file = LittleFS.open("/settings.json", "r");
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, file);
      
      if (error) {
        Serial.println("\n=== Error Reading Settings File ===");
        Serial.printf("deserializeJson() failed: %s\n", error.c_str());
      } else {
        Serial.println("\n=== Settings Stored in LittleFS ===");
        Serial.printf("PID Parameters:    Kp=%.2f, Ki=%.2f, Kd=%.2f\n", 
                    doc["Kp"].as<double>(), doc["Ki"].as<double>(), doc["Kd"].as<double>());
        Serial.printf("Filter Strength:   %.2f\n", doc["filter_strength"].as<float>());
        Serial.printf("Pressure Setpoint: %.2f bar\n", doc["setpoint"].as<double>());

        Serial.printf("PWM Configuration: %d Hz, %d-bit (max value: %d)\n", 
                    doc["pwm_freq"].as<int>(), doc["pwm_res"].as<int>(), 
                    (1 << doc["pwm_res"].as<int>()) - 1);
        Serial.printf("Control Loop Freq: %d Hz\n", doc["control_freq_hz"].as<int>());
        Serial.printf("Anti-Windup:       %s\n", doc["antiWindup"].as<bool>() ? "Enabled" : "Disabled");
        Serial.printf("PID Sample Time:   %d ms\n", doc["pid_sample_time"].as<int>());
        Serial.printf("Hysteresis Comp:   %s (%.1f%%)\n", 
                    doc["hysteresis"].as<bool>() ? "Enabled" : "Disabled", 
                    doc["hystAmount"].as<float>());
      }
      file.close();

    }

    else
    {
      Serial.println("No settings file found in LittleFS");
    }
}

// Function to map PID output to effective valve range
uint32_t mapPwmToValve(double pidOutput, int maxPwmValue) 
{
  // Calculate the percentage within PID range (0-100%)
  float pidPercent = (pidOutput / maxPwmValue) * 100.0;
  
  // If below minimum threshold, keep valve closed
  if (pidPercent < 1.0) 
  {
    return 0;
  }
  
  // Apply hysteresis compensation if enabled
  if (settings.hysteresis) 
  {
    // Determine if pressure is currently increasing or decreasing
    bool currentlyIncreasing = pressureInput > lastPressure;
    
    // Apply compensation only when direction changes
    if (currentlyIncreasing != pressureIncreasing) 
    {
      // When changing from increasing to decreasing, add compensation
      if (!currentlyIncreasing) 
      {
        pidPercent += settings.hystAmount;
      }
      // When changing from decreasing to increasing, subtract compensation
      else 
      {
        pidPercent -= settings.hystAmount;
      }
      
      // Update direction flag
      pressureIncreasing = currentlyIncreasing;
    }
  }
  
  // Map PID's 0-100% to valve's effective range
  float mappedPercent = VALVE_MIN_DUTY + (pidPercent / 100.0) * 
                       (VALVE_MAX_DUTY - VALVE_MIN_DUTY);
  
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
unsigned long min_cycle_time = 100; // Minimum cycle time in ms to prevent false transitions
const int AUTO_TUNE_CYCLES = 20; // Number of cycles to collect
int currentCycle = 0;
unsigned long cycleTimes[20]; // Store cycle times
float cycleAmplitudes[20]; // Store amplitudes

// Additional variables for proper amplitude tracking
float maxPressure = 0;
float minPressure = 999;
bool firstCycleComplete = false;

// Auto-tune constants for relay method
// Use the valve's effective range for better oscillation amplitude
const float AUTOTUNE_RELAY_HIGH = ValveConfig::VALVE_MAX_DUTY; // Use maximum effective valve opening
const float AUTOTUNE_RELAY_LOW = ValveConfig::VALVE_MIN_DUTY;   // Use minimum effective valve opening (changed from 0.0)

float min_auto_tune_cycle_pwm_value= 65.; // Minimum PWM value to trigger auto-tuning (e.g. 65%)
float max_auto_tune_cycle_pwm_value= 85.; // Maximum PWM value to trigger auto-tuning (e.g. 85%)

float autotune_effective_amplitude = max_auto_tune_cycle_pwm_value - min_auto_tune_cycle_pwm_value; // e.g. 20% effective range

float AUTOTUNE_TEST_SETPOINT = 5.0; // Target pressure for auto-tuning (now variable)
const float AUTOTUNE_NOISE_BAND = 0.1; // Deadband to prevent noise-triggered oscillations

// New auto-tuning rules and control
enum TuningRule 
{
  ZIEGLER_NICHOLS_CLASSIC = 0,   // Standard Z-N (balanced response)
  ZIEGLER_NICHOLS_AGGRESSIVE = 1, // More aggressive Z-N for faster response
  TYREUS_LUYBEN = 2,             // More conservative rule with less overshoot
  PESSEN_INTEGRAL = 3            // Aggressive integral action for setpoint tracking
};

TuningRule currentTuningRule = ZIEGLER_NICHOLS_AGGRESSIVE; // Default to more aggressive tuning
float tuningAggressiveness = 2.0; // Higher multiplier for more aggressive response (was 1.2)

// ====== PID Auto-Tuning Implementation ======
/*
 * Theory and Implementation:
 * This auto-tuning functionality implements the relay method based on Ziegler-Nichols frequency response method.
 * Instead of increasing gain until oscillation (classic Z-N), this uses a relay (on/off) controller to
 * force controlled oscillation around the setpoint, which is safer and more reliable.
 * 
 * The process:
 * 1. The system applies a square wave output (relay) that switches between 50% and 90% valve duty cycle
 * 2. This causes the pressure to oscillate around the setpoint
 * 3. The code measures:
 *    - The period of oscillation (Tu)
 *    - The amplitude of oscillation (A)
 * 4. From these measurements, it calculates the "ultimate gain" (Ku) using the formula:
 *    Ku = (4 * AUTOTUNE_EFFECTIVE_AMPLITUDE) / (π * A)
 * 5. It then applies the selected tuning rules to calculate PID parameters with valve range compensation
 * 
 * Advantages:
 * - No need to drive the system to instability, safer than traditional Z-N
 * - Works well for many types of processes, especially first and second-order systems
 * - Fully automated, requiring minimal user intervention
 * - Adapts to the specific system dynamics and valve characteristics
 * - Uses only the valve's effective operating range (50-90%) for accurate system characterization
 * 
 * Limitations:
 * - May not work well with highly nonlinear systems
 * - Sensitive to noise (mitigated with AUTOTUNE_NOISE_BAND)
 * - Z-N tuning tends to produce aggressive control (high overshoot), may need manual refinement
 * - Assumes symmetric response (opening/closing behavior is similar)
 * 
 * For best results:
 * 1. Ensure the system is at a stable starting point
 * 2. Choose a setpoint that's in the middle of your operating range
 * 3. Minimize external disturbances during tuning
 * 4. The relay amplitude (40% effective range) should be high enough to cause measurable oscillation
 * 5. After auto-tuning, you may want to reduce Ki slightly to reduce overshoot
 * 
 * Implementation details:
 * - AUTO_TUNE_CYCLES (20): Number of oscillation cycles to measure for averaging
 * - AUTO_TUNE_TIMEOUT (180s): Safety timeout to prevent indefinite tuning
 * - MIN_CYCLE_TIME (100ms): Prevents false triggers from noise or sensor jitter
 * - AUTOTUNE_NOISE_BAND (0.1): Deadband around setpoint to improve stability
 * - AUTOTUNE_EFFECTIVE_AMPLITUDE (40%): Difference between max and min valve positions
 * 
 * After tuning, you can accept the parameters with "TUNE ACCEPT" or reject with "TUNE REJECT"
 */

/*
 * ====== Auto-Tuning Serial Command Sequence ======
 * 
 * The auto-tuning process is controlled through the following serial commands:
 * 
 * 1. "TUNE START" - Initiates the auto-tuning process:
 *    - Saves the current setpoint and temporarily sets to AUTOTUNE_TEST_SETPOINT (configurable)
 *    - Switches to manual control mode
 *    - Begins applying relay outputs (alternating between 50% and 90% valve duty)
 *    - Collects data on oscillation periods and amplitudes
 *    - Progress is reported in real-time via serial output
 * 
 * 2. During auto-tuning:
 *    - The tuning runs automatically for up to 3 minutes or 20 cycles
 *    - The continuous data output will show "TUNING" if enabled with "STARTCD"
 *    - You can monitor the process through the pressure readings
 * 
 * 3. "TUNE STOP" or "TUNE CANCEL" - Stops the auto-tuning process immediately:
 *    - Restores the original setpoint
 *    - Returns to PID control
 *    - No PID parameters are changed
 * 
 * 4. After auto-tuning completes (or reaches timeout):
 *    - The system calculates optimal PID parameters
 *    - Displays the results (periods, amplitudes, Ku, Tu)
 *    - Applies valve range compensation (2.5x for 50-90% range)
 *    - Proposes new Kp, Ki, and Kd values
 *    - Prompts for acceptance or rejection
 * 
 * 5. "TUNE ACCEPT" - Confirms and saves the new PID parameters:
 *    - Updates the PID controller with the new values
 *    - Saves the values to persistent storage
 * 
 * 6. "TUNE REJECT" - Discards the calculated PID parameters:
 *    - Reloads the previous PID parameters
 *    - No changes are saved
 * 
 * Additional commands:
 * - "TUNE SP x.x" - Set the auto-tuning test setpoint (0.5-10.0 bar)
 * - "TUNE RULE n" - Select tuning rule (0-3)
 * - "TUNE AGGR x.x" - Set aggressiveness factor (0.5-2.0)
 * 
 * Example sequence for a complete auto-tuning process:
 *   > TUNE SP 3.5           (optional: set test setpoint)
 *   > TUNE RULE 1           (optional: select aggressive Z-N rule)
 *   > STARTCD               (optional: start continuous data output)
 *   > TUNE START            (start auto-tuning)
 *   ... system collects data and displays progress ...
 *   ... when complete, system shows calculated parameters ...
 *   > TUNE ACCEPT           (accept the new parameters)
 *   > STOPCD                (optional: stop continuous data output)
 */

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
  autoTuneState = false;  // Start with relay off
  currentCycle = 0;
  
  // Initialize amplitude tracking
  maxPressure = pressureInput;
  minPressure = pressureInput;
  firstCycleComplete = false;
  // Set initial output state - Use high relay value for clarity in reporting
  autoTuneOutputValue = max_auto_tune_cycle_pwm_value;
  
  // Switch to manual mode for auto-tuning
  manualPWMMode = true;
  
  // Apply initial output - map to valve's effective range
  uint32_t pwmValue = (uint32_t)((autoTuneOutputValue / 100.0) * pwm_max_value);

  ledcWrite(PWM_CHANNEL_MOSFET, pwmValue);

  Serial.println("\n=== PID Auto-Tuning Started ===");
  Serial.printf("Target Setpoint: %.2f bar\n", settings.setpoint);
  Serial.printf("Relay Output: %.1f%% - %.1f%% (valve effective range)\n", min_auto_tune_cycle_pwm_value, max_auto_tune_cycle_pwm_value);
  Serial.printf("Valve operating range: %.1f%% - %.1f%%\n", VALVE_MIN_DUTY, VALVE_MAX_DUTY);
  Serial.printf("Effective relay amplitude: %.1f%%\n", autotune_effective_amplitude);
  Serial.println("Auto-tuning will run for up to 3 minutes or 20 complete cycles");
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
    float avgAmplitude = totalAmplitude / currentCycle;    // Calculate ultimate gain Ku and period Tu
    // Use the effective amplitude for calculations - the actual pressure swing amplitude
    float effectiveAmplitude = max_auto_tune_cycle_pwm_value - min_auto_tune_cycle_pwm_value;
    float Ku = (4.0 * effectiveAmplitude) / (3.141592 * avgAmplitude);
    float Tu = avgPeriod;
      // Calculate PID parameters based on selected tuning rule
    float newKp = 0.0, newKi = 0.0, newKd = 0.0;
    const char* ruleName = "";
    
    // Apply a scaling factor to compensate for limited valve range (50-90% instead of 0-100%)
    float valveRangeCompensation = 100.0 / (VALVE_MAX_DUTY - VALVE_MIN_DUTY); // 100/40 = 2.5x
    
    switch (currentTuningRule) 
    {
      case ZIEGLER_NICHOLS_CLASSIC:
        // Classic Ziegler-Nichols - balanced response, scaled for valve range
        newKp = 1.5 * Ku * valveRangeCompensation;
        newKi = 3.0 * Ku / Tu * valveRangeCompensation;
        newKd = 0.18 * Ku * Tu;
        ruleName = "Ziegler-Nichols Classic";
        break;
        
      case ZIEGLER_NICHOLS_AGGRESSIVE:
        // Modified Ziegler-Nichols for faster response, scaled for valve range
        newKp = 2.0 * Ku * tuningAggressiveness * valveRangeCompensation; // Much higher proportional gain
        newKi = 4.5 * Ku / Tu * valveRangeCompensation;                  // Much stronger integral action
        newKd = 0.25 * Ku * Tu;                                          // More derivative
        ruleName = "Ziegler-Nichols Aggressive";
        break;
        
      case TYREUS_LUYBEN:
        // Tyreus-Luyben - more conservative but still scaled for valve range
        newKp = 1.2 * Ku * valveRangeCompensation;
        newKi = 1.0 * Ku / Tu * valveRangeCompensation; // Higher integral gain than before
        newKd = 0.35 * Ku * Tu; // Higher derivative gain = more damping
        ruleName = "Tyreus-Luyben";
        break;
        
      case PESSEN_INTEGRAL:
        // Pessen Integral Rule - very aggressive for setpoint tracking
        newKp = 2.2 * Ku * valveRangeCompensation;
        newKi = 5.0 * Ku / Tu * valveRangeCompensation; // Very high integral action for fast setpoint tracking
        newKd = 0.3 * Ku * Tu;
        ruleName = "Pessen Integral";
        break;
    }
      Serial.println("\n=== Auto-Tuning Results ===");
    Serial.printf("Average Period: %.2f seconds\n", avgPeriod);
    Serial.printf("Average Amplitude: %.2f bar\n", avgAmplitude);
    Serial.printf("Valve Range Compensation: %.2fx\n", valveRangeCompensation);
    Serial.printf("Ultimate Gain (Ku): %.2f\n", Ku);
    Serial.printf("Ultimate Period (Tu): %.2f s\n", Tu);
    Serial.printf("Tuning Rule: %s\n", ruleName);
    Serial.printf("Aggressiveness Factor: %.1f\n", tuningAggressiveness);
    
    Serial.println("\nCalculated PID Parameters:");
    Serial.printf("Kp: %.2f (previously %.2f)\n", newKp, settings.Kp);
    Serial.printf("Ki: %.2f (previously %.2f)\n", newKi, settings.Ki);
    Serial.printf("Kd: %.2f (previously %.2f)\n", newKd, settings.Kd);
    
    // Prompt user to accept or reject values
    Serial.println("\nTo accept these values, type 'TUNE ACCEPT'");
    Serial.println("To reject and keep current values, type 'TUNE REJECT'");
    Serial.println("For faster response, try 'TUNE RULE 1' before starting auto-tune");
    Serial.println("For even faster response, try 'TUNE AGGR 1.5' to increase aggressiveness");
    
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

void performAutoTune()
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
    // Transition from high to low
    unsigned long now = millis();
    
    // Debounce transitions to prevent noise
    if (now - lastTransitionTime > min_cycle_time)
    {
      // Record cycle data if this isn't the first transition
      if (lastTransitionTime > 0)
      {
        cycleTimes[currentCycle] = now - lastTransitionTime;
        cycleAmplitudes[currentCycle] = maxPressure - minPressure;
        currentCycle++;
        
        Serial.printf("Cycle %d: Period=%.2fs, Amplitude=%.2f bar (Max=%.2f, Min=%.2f)\n", 
                      currentCycle, (now - lastTransitionTime)/1000.0, 
                      cycleAmplitudes[currentCycle-1], maxPressure, minPressure);
      } 
      
      lastTransitionTime = now;
      autoTuneState = true;
      autoTuneOutputValue = min_auto_tune_cycle_pwm_value; // Set output to low effective range
      
      // Reset amplitude tracking for next cycle
      maxPressure = pressureInput;
      minPressure = pressureInput;
      
      uint32_t pwmValue = (uint32_t)((autoTuneOutputValue / 100.0) * pwm_max_value);
      ledcWrite(PWM_CHANNEL_MOSFET, pwmValue);
    }
  }
  else if (autoTuneState && pressureInput < settings.setpoint - AUTOTUNE_NOISE_BAND)
  {
    // Transition from low to high
    unsigned long now = millis();
    
    // Debounce transitions to prevent noise
    if (now - lastTransitionTime > min_cycle_time)
    {
      lastTransitionTime = now;
      autoTuneState = false;
      autoTuneOutputValue = max_auto_tune_cycle_pwm_value; // Set output high - use raw percentage
      
      // Apply the output - calculate the direct PWM value for the desired valve opening (90%)

      uint32_t pwmValue = (uint32_t)((autoTuneOutputValue / 100.0) * pwm_max_value);
      ledcWrite(PWM_CHANNEL_MOSFET, pwmValue);
    }
  }
  
  // Track max and min pressures during oscillation
  if (autoTuneRunning) 
  {
    if (pressureInput > maxPressure) maxPressure = pressureInput;
    if (pressureInput < minPressure) minPressure = pressureInput;
  }
}

// Helper function to list all files in LittleFS
void listFiles() {
  Serial.println("\n=== Files in Flash Memory ===");
  
  File root = LittleFS.open("/");
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }
  
  int fileCount = 0;
  size_t totalSize = 0;
  
  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      // Print file name and size in KB
      Serial.printf("%-20s %8.2f KB\n", file.name(), file.size() / 1024.0);
      fileCount++;
      totalSize += file.size();
    }
    file = root.openNextFile();
  }
  
  Serial.println("------------------------------");
  Serial.printf("Total: %d files, %.2f KB\n", fileCount, totalSize / 1024.0);
  Serial.printf("Flash usage: %.1f%% (of %.2f KB)\n", 
                (totalSize * 100.0) / (LittleFS.totalBytes()), 
                LittleFS.totalBytes() / 1024.0);
  Serial.printf("Free space: %.2f KB\n", (LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024.0);
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
    // Store current duty cycle percentage before changing resolution
    float current_duty_percent = (pwmOutput / (float)pwm_max_value) * 100.0;
    
    // Update resolution and max value
    settings.pwm_res = constrain(new_res, 1, 16);
    int new_max_value = (1 << settings.pwm_res) - 1;
    
    // Scale pwmOutput to maintain the same duty cycle
    pwmOutput = (current_duty_percent / 100.0) * new_max_value;
    
    // Update max value and PID limits
    pwm_max_value = new_max_value;
    pid.SetOutputLimits(0, pwm_max_value);
    updatePWM();
    
    Serial.printf("PWM resolution updated to: %d bits (max: %d)\n", settings.pwm_res, pwm_max_value);
    Serial.printf("Duty cycle maintained at: %.1f%%\n", current_duty_percent);
    saveSettings();
  }
  else if (cmd.startsWith("PWM "))
  {
    // Force PWM duty cycle for testing (overrides PID)
    float duty_percent = cmd.substring(4).toFloat();
    // Constrain to valid percentage range
    duty_percent = constrain(duty_percent, 0.0, 100.0);
    // Convert percentage to absolute PWM value
    pwmOutput = (duty_percent / 100.0) * pwm_max_value;
    
    // Apply PWM value directly (without mapping for manual control)
    ledcWrite(PWM_CHANNEL_MOSFET, (uint32_t)pwmOutput);
    
    Serial.printf("PWM manually set to: %.1f%% (%d/%d)\n", 
                 duty_percent, (int)pwmOutput, pwm_max_value);
                 
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
    showSettingsFromLittleFS();
  }
  else if (cmd == "DIR" )
  {
    listFiles();
  }
  else if (cmd == "AW ON" )
  {
    settings.antiWindup = true;
    Serial.println("Anti-windup for deadband enabled");
    saveSettings();
  }
  else if (cmd == "AW OFF" )
  {
    settings.antiWindup = false;
    Serial.println("Anti-windup for deadband disabled");
    saveSettings();
  }
  else if (cmd == "HYST ON" )
  {
    settings.hysteresis = true;
    Serial.println("Hysteresis compensation enabled");
    saveSettings();
  }
  else if (cmd == "HYST OFF" )
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
  else if (cmd.startsWith("TUNE SP "))
  {
    float newSetpoint = cmd.substring(8).toFloat();
    AUTOTUNE_TEST_SETPOINT = constrain(newSetpoint, 0.5, 10.0);
    Serial.printf("Auto-tuning test setpoint set to: %.1f bar\n", AUTOTUNE_TEST_SETPOINT);
  }    else if (cmd.startsWith("SAMPLE "))
  {
    int newSampleTime = cmd.substring(7).toInt();
    newSampleTime = constrain(newSampleTime, 1, 1000);
    settings.pid_sample_time = newSampleTime; // Update the settings structure
    pid.SetSampleTime(settings.pid_sample_time);
    Serial.printf("PID sample time set to: %d ms (%.1f Hz)\n", settings.pid_sample_time, 1000.0/settings.pid_sample_time);
    
    // Check timing relationship and provide feedback
    float controlPeriod = 1000.0 / settings.control_freq_hz;
    float pidPeriod = settings.pid_sample_time;
    if (controlPeriod > pidPeriod) {
      Serial.printf("WARNING: Control period (%.1f ms) > PID period (%.1f ms) - PID limited to %.0f Hz\n", 
                   controlPeriod, pidPeriod, settings.control_freq_hz);
    } else if (controlPeriod < pidPeriod / 2) {
      Serial.printf("INFO: Control freq much faster than PID - good for sensor resolution\n");
    } else {
      Serial.printf("OK: Control/PID timing balanced\n");
    }
    
    saveSettings();
  }
  else if (cmd.startsWith("TUNE MIN "))
  {
    float newMin = cmd.substring(9).toFloat();
    min_auto_tune_cycle_pwm_value = constrain(newMin, ValveConfig::VALVE_MIN_DUTY, max_auto_tune_cycle_pwm_value);
    autotune_effective_amplitude = max_auto_tune_cycle_pwm_value - min_auto_tune_cycle_pwm_value;
    // Update effective amplitude
    Serial.printf("Auto-tuning minimum PWM set to: %.1f%%\n", min_auto_tune_cycle_pwm_value);
    Serial.printf("Effective amplitude: %.1f%%\n", max_auto_tune_cycle_pwm_value - min_auto_tune_cycle_pwm_value);
  }
  else if (cmd.startsWith("TUNE MAX "))
  {
    float newMax = cmd.substring(9).toFloat();
    max_auto_tune_cycle_pwm_value = constrain(newMax, min_auto_tune_cycle_pwm_value, ValveConfig::VALVE_MAX_DUTY);
    autotune_effective_amplitude = max_auto_tune_cycle_pwm_value - min_auto_tune_cycle_pwm_value;
    // Update effective amplitude
    Serial.printf("Auto-tuning maximum PWM set to: %.1f%%\n", max_auto_tune_cycle_pwm_value);
    Serial.printf("Effective amplitude: %.1f%%\n", max_auto_tune_cycle_pwm_value - min_auto_tune_cycle_pwm_value);
  }  else if (cmd.startsWith("TUNE CYCLE "))
  {
    unsigned long newCycleTime = cmd.substring(11).toInt();
    min_cycle_time = constrain(newCycleTime, 50UL, 2000UL);
    Serial.printf("Auto-tuning minimum cycle time set to: %lu ms\n", min_cycle_time);
    Serial.printf("This prevents noise-triggered transitions during auto-tuning\n");
  }  else if (cmd.startsWith("CONTROL FREQ "))
  {
    int new_freq = cmd.substring(13).toInt();
    settings.control_freq_hz = constrain(new_freq, 10, 1000); // Max 1000 Hz
    Serial.printf("Control loop frequency updated to: %d Hz (period: %.1f ms)\n", 
                 settings.control_freq_hz, 1000.0/settings.control_freq_hz);
    
    // Check timing relationship and provide feedback
    float controlPeriod = 1000.0 / settings.control_freq_hz;
    float pidPeriod = settings.pid_sample_time;
    if (controlPeriod > pidPeriod) {
      Serial.printf("WARNING: Control period (%.1f ms) > PID period (%.1f ms) - PID limited to %.0f Hz\n", 
                   controlPeriod, pidPeriod, settings.control_freq_hz);
    } else if (controlPeriod < pidPeriod / 2) {
      Serial.printf("INFO: Control freq much faster than PID - good for sensor resolution\n");
    } else {
      Serial.printf("OK: Control/PID timing balanced\n");
    }
    
    saveSettings();
  }
  else if (cmd == "HELP")
  {
    Serial.println(
      "\n=== Serial Command Help === All commands are case-insensitive."
      "\n--- PID Control ---"
      "\nKP 0.5     Set proportional gain"
      "\nKI 0.1     Set integral gain"
      "\nKD 0.01    Set derivative gain"
      "\nSP 3.0     Set pressure setpoint (bar)"
      "\nSAMPLE 10  Set PID sample time (1-1000 ms)"
      "\nRESET      Reset PID controller (clear integral windup and state)"
      "\n"
      "\n--- Signal Processing ---"
      "\nFLT 0.2    Set filter strength (0.0-1.0)"
      "\nAW ON/OFF  Enable/disable anti-windup for deadband"
      "\nHYST ON/OFF Enable/disable hysteresis compensation"
      "\nHYSTAMT 5  Set hysteresis compensation amount (%)"
      "\n"
      "\n--- PWM Control ---"
      "\nFREQ 1000  Set PWM frequency (100-10000Hz)"
      "\nRES 8      Set PWM resolution (1-16 bits)"
      "\nPWM 25     Force PWM duty cycle (0-100%) for testing"
      "\nRESUME     Resume normal PID control after manual PWM"
      "\n"
      "\n--- Control Loop ---"
      "\nCONTROL FREQ 1000 Set control loop frequency (10-1000 Hz)"
      "\n"
      "\n--- Auto-Tuning ---"
      "\nTUNE START Start PID auto-tuning process"
      "\nTUNE STOP  Cancel auto-tuning process"
      "\nTUNE ACCEPT Accept auto-tuned PID parameters"
      "\nTUNE REJECT Reject auto-tuned PID parameters"
      "\nTUNE SP 3.0 Set auto-tuning test setpoint (0.5-10.0 bar)"
      "\nTUNE MIN 65 Set auto-tuning minimum PWM (50-90%)"
      "\nTUNE MAX 85 Set auto-tuning maximum PWM (60-95%)"
      "\nTUNE CYCLE 100 Set min cycle time for auto-tuning (50-2000ms)"
      "\nTUNE RULE n Select auto-tuning rule (0-3, see TUNE RULES)"
      "\nTUNE AGGR x Set tuning aggressiveness (0.5-2.0)"
      "\nTUNE RULES  Show available tuning rules"
      "\n"
      "\n--- System & Data ---"
      "\nSTATUS     Show current parameters"
      "\nSAVE       Force save settings to flash"
      "\nREAD       Read settings stored in flash"
      "\nSTARTCD    Start continuous data output for plotting"
      "\nSTOPCD     Stop continuous data output"
      "\nPAGE ON     Enable web server processing"
      "\nPAGE OFF    Disable web server processing"
      "\nDIR        List all files in flash memory with sizes"
      "\nVER        Display firmware version and build info"
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
    Serial.printf("  Control Loop Freq: %d Hz (period: %.1f ms)\n", 
                 settings.control_freq_hz, 1000.0/settings.control_freq_hz);
    Serial.printf("  Auto-Tune Test Setpoint: %.2f bar\n", AUTOTUNE_TEST_SETPOINT);
    Serial.printf("  Auto-Tune PWM Range: %.1f%% - %.1f%% (amplitude: %.1f%%)\n", 
                 min_auto_tune_cycle_pwm_value, max_auto_tune_cycle_pwm_value,
                 max_auto_tune_cycle_pwm_value - min_auto_tune_cycle_pwm_value);
    Serial.printf("  Auto-Tune Min Cycle Time: %lu ms\n", min_cycle_time);
    
    // Sensor Information section
    Serial.println("\nSensor Information:");
    Serial.printf("  ADC Source: %s\n", ads_found ? "ADS1015" : "ESP32 Internal");
    if (ads_found) {
      Serial.printf("  ADS1015 Address: 0x%02X\n", ADS1015_I2C_ADDRESS);
      Serial.printf("  ADS1015 Channel: %d\n", ADC_CHANNEL);
      const char* gainStr = (ADS1015_GAIN == GAIN_TWOTHIRDS) ? "GAIN_TWOTHIRDS (+/-6.144V)" :
                           (ADS1015_GAIN == GAIN_ONE) ? "GAIN_ONE (+/-4.096V)" :
                           (ADS1015_GAIN == GAIN_TWO) ? "GAIN_TWO (+/-2.048V)" :
                           (ADS1015_GAIN == GAIN_FOUR) ? "GAIN_FOUR (+/-1.024V)" :
                           (ADS1015_GAIN == GAIN_EIGHT) ? "GAIN_EIGHT (+/-0.512V)" :
                           (ADS1015_GAIN == GAIN_SIXTEEN) ? "GAIN_SIXTEEN (+/-0.256V)" : "Unknown";
      Serial.printf("  Gain Setting: %s\n", gainStr);
    } else {
      Serial.printf("  Fallback Pin: %d\n", FALLBACK_ANALOG_PIN);
    }
    Serial.printf("  Raw ADC Value: %d\n", adc_value);
    Serial.printf("  Voltage: %.3f V\n", voltage);
    Serial.printf("  Raw Pressure: %.3f bar\n", raw_pressure);
    Serial.printf("  Filtered Pressure: %.3f bar\n", filtered_pressure);
    Serial.printf("  Voltage Range: %.1fV - %.1fV\n", MIN_VOLTAGE, MAX_VOLTAGE);
    Serial.printf("  Pressure Range: %.1f - %.1f bar\n", SENSOR_MIN_BAR, SENSOR_MAX_BAR);
    
    // PWM Output section
    Serial.println("\nPWM Output:");
    Serial.printf("  Value: %d/%d (%.3f%%)\n", 
                 (int)pwmOutput, pwm_max_value, 
                 (pwmOutput / float(pwm_max_value)) * 100.0);
    Serial.printf("  Resolution: %d-bit\n", settings.pwm_res);
    Serial.printf("  Frequency: %d Hz\n", settings.pwm_freq);
    Serial.printf("  Control Mode: %s\n", manualPWMMode ? "MANUAL" : "PID");
    Serial.printf("  Solenoid Pin: %d\n", SOLENOID_PIN);
      // PID Configuration section
    Serial.println("\nPID Configuration:");
    Serial.printf("  Kp=%.2f, Ki=%.2f, Kd=%.2f\n", 
                 settings.Kp, settings.Ki, settings.Kd);
    Serial.printf("  Sample Time: %d ms (%.1f Hz)\n", settings.pid_sample_time, 1000.0/settings.pid_sample_time);
    Serial.printf("  Filter Strength: %.2f\n", settings.filter_strength);
    Serial.printf("  Anti-Windup: %s\n", settings.antiWindup ? "Enabled" : "Disabled");
    Serial.printf("  Hysteresis Comp: %s (%.1f%%)\n", 
                 settings.hysteresis ? "Enabled" : "Disabled", settings.hystAmount);
    
    // Network section
    Serial.println("\nNetwork:");
    Serial.printf("  IP: %s\n", WiFi.softAPIP().toString().c_str());
    Serial.printf("  Web Server: %s\n", webServerEnabled ? "Enabled" : "Disabled");
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
  else if (cmd == "PAGE ON")
  {
    webServerEnabled = true;
    Serial.println("Web server processing enabled");
  }
  else if (cmd == "PAGE OFF")
  {
    webServerEnabled = false;
    Serial.println("Web server processing disabled");
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
    pid.SetOutputLimits(0, pwm_max_value);  // No change needed in method signature
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
  else if (cmd.startsWith("TUNE RULE "))
  {
    int rule = cmd.substring(10).toInt();
    if (rule >= 0 && rule <= 3) {
      currentTuningRule = (TuningRule)rule;
      Serial.println("\n=== Auto-Tuning Rule Updated ===");
      Serial.println("Selected Rule: ");
      switch (currentTuningRule) {
        case ZIEGLER_NICHOLS_CLASSIC:
          Serial.println("0 - Ziegler-Nichols Classic (Balanced)");
          break;
        case ZIEGLER_NICHOLS_AGGRESSIVE:
          Serial.println("1 - Ziegler-Nichols Aggressive (Faster Response)");
          break;
        case TYREUS_LUYBEN:
          Serial.println("2 - Tyreus-Luyben (Less Overshoot)");
          break;
        case PESSEN_INTEGRAL:
          Serial.println("3 - Pessen Integral (Fast Setpoint Tracking)");
          break;
      }
    } else {
      Serial.println("Invalid rule number. Valid options: 0-3");
    }
  }
  else if (cmd.startsWith("TUNE AGGR "))
  {
    float aggr = cmd.substring(10).toFloat();
    if (aggr >= 0.5 && aggr <= 2.0) {
      tuningAggressiveness = aggr;
      Serial.printf("Tuning aggressiveness set to: %.1f\n", tuningAggressiveness);
    } else {
      Serial.println("Invalid aggressiveness value. Valid range: 0.5-2.0");
    }
  }
  else if (cmd == "TUNE RULES")
  {
    Serial.println("\n=== Available Auto-Tuning Rules ===");
    Serial.println("0 - Ziegler-Nichols Classic: Balanced response");
    Serial.println("1 - Ziegler-Nichols Aggressive: Faster response with more overshoot");
    Serial.println("2 - Tyreus-Luyben: Less overshoot, slower recovery");
    Serial.println("3 - Pessen Integral: Fast setpoint tracking");
    Serial.printf("\nCurrent Rule: %d, Aggressiveness: %.1f\n", currentTuningRule, tuningAggressiveness);
    Serial.printf("Auto-Tune Test Setpoint: %.1f bar\n", AUTOTUNE_TEST_SETPOINT);
    Serial.printf("Auto-Tune PWM Range: %.1f%% - %.1f%% (amplitude: %.1f%%)\n", 
                 min_auto_tune_cycle_pwm_value, max_auto_tune_cycle_pwm_value,
                 max_auto_tune_cycle_pwm_value - min_auto_tune_cycle_pwm_value);
    Serial.printf("Auto-Tune Min Cycle Time: %lu ms\n", min_cycle_time);
  }
  else
  {
    Serial.println("Invalid command. Type 'HELP' for options.");
  }
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



// WiFi event handler function
void onWiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) 
{
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
      
      if (connectedClients < MAX_CLIENTS) 
      {
        // We have room for this client
        connectedMACs[connectedClients] = String(macStr);
        connectedClients++;
        
        Serial.printf("Device connected to AP (%d/%d clients)\n", 
                      connectedClients, MAX_CLIENTS);
        Serial.printf("MAC address: %s\n", macStr);
      } else 
      {
        // Too many clients, disconnect this one
        Serial.println("Maximum client limit reached - disconnecting new client");
        Serial.printf("MAC address: %s\n", macStr);
        
        // Force disconnection - this requires a brief reset of the AP
        WiFi.softAPdisconnect(false);  // Disconnect all clients but keep AP running
        delay(10); // Brief delay
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
      for (int i = 0; i < MAX_CLIENTS; i++) {
        if (connectedMACs[i] == disconnectedMAC) {
          // Shift remaining clients down
          for (int j = i; j < MAX_CLIENTS - 1; j++) {
            connectedMACs[j] = connectedMACs[j + 1];
          }
          connectedMACs[MAX_CLIENTS - 1] = ""; // Clear the last slot
          connectedClients--;
          break;
        }
      }
      
      Serial.printf("Clients remaining: %d/%d\n", connectedClients, MAX_CLIENTS);
      break;
  }
}

// Create the control task function
void controlTask(void* parameter) 
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    // Use configurable frequency
    TickType_t frequency = pdMS_TO_TICKS(1000 / settings.control_freq_hz);
    
    static unsigned long lastCycleEnd = 0;
    
    while(true) 
    {
        // Update frequency if settings changed
        frequency = pdMS_TO_TICKS(1000 / settings.control_freq_hz);
        
        unsigned long taskStartTime = micros();
        
        // ====== Sensor Reading and PID Update ======
        if (ads_found)
        {
            adc_value = ads.readADC_SingleEnded(ADC_CHANNEL);
            voltage = ads.computeVolts(adc_value);
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
        
        // Save last pressure before updating, used for hysteresis compensation
        lastPressure = pressureInput;
        
        // Update Input value for PID before computation
        pressureInput = filtered_pressure;

        // ====== Analog Pressure Signal Output =====
        static unsigned long lastAnalogOutTime = 0;
        if (millis() - lastAnalogOutTime >= 10)
        { 
            // Output analog pressure signal to ANALOG_PRESS_PIN
            ledcWrite(PWM_CHANNEL_ANALOG_PRESS, (uint32_t)(pressureInput/SENSOR_MAX_VALUE*pwm_analog_pressure_signal_pwm_res));   
            lastAnalogOutTime = millis();
        }

        // Process auto-tuning if active
        if (autoTuneRunning)
        {
            performAutoTune();
        }
        // PID calculation and PWM output (only if not in auto-tune mode)
        else if (!manualPWMMode) 
        {
            // Store previous output for anti-windup check
            double previousOutput = pwmOutput;
            
            // Constrain output to valid range
            pwmOutput = constrain(pwmOutput, 0, pwm_max_value);

            // Compute PID output
            pid.Compute();
            
            // Anti-windup for deadband and saturation
            if (settings.antiWindup) 
            {
                float pidPercent = (pwmOutput / pwm_max_value) * 100.0;
                
                if ((pidPercent < VALVE_MIN_DUTY && pwmOutput > previousOutput) ||
                    (pidPercent > VALVE_MAX_DUTY && pwmOutput > previousOutput)) 
                {
                    // Reset the PID to prevent integral accumulation
                    pid.SetMode(PID::Manual);
                    pid.SetMode(PID::Automatic);
                    
                    // Optional debug output if continuous output is enabled
                    static unsigned long lastDebugTime = 0;
                    if (continousValueOutput && (millis() - lastDebugTime >= SERIAL_OUTPUT_INTERVAL)) {
                        if (pidPercent < VALVE_MIN_DUTY) 
                        {
                            Serial.println("Anti-windup: Below min duty cycle");
                        } else 
                        {
                            Serial.println("Anti-windup: Above max duty cycle");
                        }
                        lastDebugTime = millis();
                    }
                }
            }

            // Use the mapping function to get actual PWM value to apply
            uint32_t actualPwm = mapPwmToValve(pwmOutput, pwm_max_value);
            ledcWrite(PWM_CHANNEL_MOSFET, actualPwm);
        }

        // ====== Emergency Shutdown ======
        if (pressureInput > SENSOR_MAX_BAR * 1.1)
        {
            // Stop PWM output
            ledcWrite(PWM_CHANNEL_MOSFET, 0);
            
            static unsigned long lastEmergencyMsgTime = 0;
            if (millis() - lastEmergencyMsgTime >= 1000) 
            {
                Serial.println("EMERGENCY SHUTDOWN! Pressure exceeds safe limit.");
                lastEmergencyMsgTime = millis();
            }
        }

        // ====== Continuous Data Output ======
        static unsigned long lastContinuousOutputTime = 0;
        unsigned long deltaTimeContinuous = millis() - lastContinuousOutputTime;
        
        if(continousValueOutput == true && (deltaTimeContinuous >= SERIAL_OUTPUT_INTERVAL))
        {
            // Calculate execution time for this task iteration
            unsigned long taskExecTime = micros() - taskStartTime;
            
            // Calculate total time including wait time from previous cycle
            
            unsigned long totalCycleTime = 0;
            if (lastCycleEnd > 0) 
            {
                totalCycleTime = taskStartTime - lastCycleEnd;
            }
            
            
            // Format all data at once using a single buffer
            char buffer[120];
            
            int len = snprintf(buffer, sizeof(buffer),
              "voltage=%.3f, error=%.3f, press=%.3f, setPress=%.3f, PWM%%=%.3f, t=%.2f, exec=%lu, total=%lu, task=CTRL%s\r\n",
              voltage,
              settings.setpoint - pressureInput,
              pressureInput,
              settings.setpoint,
              (autoTuneRunning ? autoTuneOutputValue : (pwmOutput/pwm_max_value)*100.0),
              xTaskGetTickCount() * portTICK_PERIOD_MS / 1000.0, // Convert ticks to seconds
              taskExecTime, // Task execution time in microseconds
              totalCycleTime, // Total cycle time in microseconds (execution + wait)
              autoTuneRunning ? ", TUNING" : "");

            // Send the entire buffer in one operation
            Serial.write(buffer, len);

            lastContinuousOutputTime = millis();
        }
        
        // Wait for next cycle - this properly yields to other tasks
        vTaskDelayUntil(&lastWakeTime, frequency);
        lastCycleEnd = taskStartTime; // Update last cycle start time
    }
}

// Create the network task function
void networkTask(void* parameter) 
{
    while(true) 
    {
        if (webServerEnabled) 
        {
            unsigned long netStart = micros();
            
            dnsServer.processNextRequest();
            server.handleClient();
            
            unsigned long netTime = micros() - netStart;
            if (netTime > 20000) 
            { // Log if >20ms
                // Serial.printf("Network delay: %lu us (Core 0)\n", netTime);
            }
        }
        
        // Small delay to prevent watchdog timeout and allow other tasks
        vTaskDelay(pdMS_TO_TICKS(2)); // 2ms delay = ~500Hz update rate
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
  
  // Set I2C bus speed to 100 kHz for ADS1015
  Wire.setClock(100000);
  
  // Try to initialize ADS1015 for up to 2 seconds
  unsigned long ads_start = millis();
  ads_found = false;

  while (millis() - ads_start < 2000)
  {
    if (ads.begin(ADS1015_I2C_ADDRESS,&Wire))
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
  // Initialize LittleFS for settings storage
  if (!LittleFS.begin(true))
    Serial.println("LittleFS error!");
  loadSettings();
  showSettingsFromLittleFS(); // Show loaded settings on startup

  ledcSetup(PWM_CHANNEL_ANALOG_PRESS, pwm_analog_pressure_signal_freq, pwm_analog_pressure_signal_pwm_res);
  ledcAttachPin(ANALOG_PRESS_PIN, PWM_CHANNEL_ANALOG_PRESS);

  ledcSetup(PWM_CHANNEL_MOSFET, settings.pwm_freq, settings.pwm_res);
  ledcAttachPin(SOLENOID_PIN, PWM_CHANNEL_MOSFET);
  
  // Set up WiFi event handler before starting AP
  WiFi.onEvent(onWiFiEvent);
  
  // Start WiFi AP and DNS server for captive portal
  WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
  WiFi.softAP(ap_ssid, ap_password);
  dnsServer.start(53, "*", ap_ip);  // Create network task on Core 0
  xTaskCreatePinnedToCore(
        networkTask,           // Task function
        "NetworkTask",         // Task name
        4096,                  // Stack size (bytes)
        NULL,                  // Parameter passed to task
        1,                     // Task priority (1 = low, higher number = higher priority)
        &networkTaskHandle,    // Task handle
        0                      // Core 0
  );

  // Create control task on Core 1
  xTaskCreatePinnedToCore(
        controlTask,           // Task function
        "ControlTask",         // Task name
        4096,                  // Stack size (bytes)
        NULL,                  // Parameter passed to task
        2,                     // Task priority (2 = higher than network)
        &controlTaskHandle,    // Task handle
        1                      // Core 1
  );

  // Initialize PID 
  pid.SetMode(PID::Automatic);
  pid.SetOutputLimits(0, pwm_max_value);
  pid.SetSampleTime(settings.pid_sample_time); // Use settings value for consistency

  // Use the new setup function for web handlers
  setupWebHandlers();

  Serial.println("\nSystem Ready - AP Mode");
  Serial.println("\nType HELP for command options");
  Serial.printf("Connect to: %s\nPassword: %s\n", ap_ssid, ap_password);
  Serial.printf("Access via: http://www.ventcon.local\n");
  Serial.print("Direct IP access: http://");
  Serial.println(WiFi.softAPIP());
}



// ====== Arduino Main Loop ======
void loop()
{
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
  
  // Small delay to prevent watchdog timeout and allow other tasks
  vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay - serial commands don't need high frequency
}



