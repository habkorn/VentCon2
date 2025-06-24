#include <WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <PID_v2.h>
// #include <Adafruit_ADS1X15.h>
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "WebContent.h"
#include "Constants.h"  // Add this include
#include "Settings.h"   // Add Settings class include
#include "WebHandlers.h" // Add WebHandlers include
#include "SensorManager.h" // Add SensorManager include

// ====== Hardware Configuration ======
const int SOLENOID_PIN = HardwareConfig::SOLENOID_PIN;
const int ANALOG_PRESS_PIN = HardwareConfig::ANALOG_PRESS_PIN;
const int PWM_CHANNEL_MOSFET = HardwareConfig::PWM_CHANNEL_MOSFET;
const int PWM_CHANNEL_ANALOG_PRESS = HardwareConfig::PWM_CHANNEL_ANALOG_PRESS;

// ====== Settings and Sensor Instances ======
Settings settings; // Use Settings class instance
SensorManager* sensorManager = nullptr; // Will be initialized in setup()

// ====== Valve Configuration ======
const float VALVE_MIN_DUTY = ValveConfig::VALVE_MIN_DUTY; 
const float VALVE_MAX_DUTY = ValveConfig::VALVE_MAX_DUTY;

// ====== PID and Control Variables ======
double pressureInput, pwmOutput; // PID input (pressure) and output (PWM)
PID pid(&pressureInput, &pwmOutput, &settings.setpoint, settings.Kp, settings.Ki, settings.Kd, DIRECT);

// ====== WebHandler Instance ======
// Will be initialized in setup() after all dependencies are ready
WebHandler* webHandler = nullptr;


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

// Track previous pressure direction for hysteresis compensation
bool pressureIncreasing = false;
float lastPressure = 0.0;



// ====== PWM Update Function ======
void updatePWM()
{
  ledcSetup(PWM_CHANNEL_MOSFET, settings.pwm_freq, settings.pwm_res);
  ledcWrite(PWM_CHANNEL_MOSFET, pwmOutput);
}

void showSettingsFromLittleFS()
{
    // Use the Settings class method to display stored settings
    settings.printStoredSettings();
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
  sprintf(versionString, VENTCON_VERSION);
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
  {    settings.Kp = cmd.substring(3).toFloat();
    pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
    Serial.printf("Proportional gain set to: %.2f\n", settings.Kp);
    settings.save();
  }
  else if (cmd.startsWith("KI "))
  {
    settings.Ki = cmd.substring(3).toFloat();
    pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
    Serial.printf("Integral gain set to: %.2f\n", settings.Ki);
    settings.save();
  }
  else if (cmd.startsWith("KD "))
  {
    settings.Kd = cmd.substring(3).toFloat();
    pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
    Serial.printf("Derivative gain set to: %.2f\n", settings.Kd);
    settings.save();
  }
  else if (cmd.startsWith("FLT "))
  {
    float new_flt = cmd.substring(4).toFloat();
    settings.filter_strength = constrain(new_flt, 0.0, 1.0);
    Serial.printf("Filter strength set to: %.2f\n", settings.filter_strength);
    settings.save();
  }
  else if (cmd.startsWith("SP "))
  {
    settings.setpoint = cmd.substring(3).toFloat();
    Serial.printf("Setpoint updated to: %.2f bar\n", settings.setpoint);
    settings.save();
  }
  else if (cmd.startsWith("FREQ "))
  {
    int new_freq = cmd.substring(5).toInt();
    settings.pwm_freq = constrain(new_freq, 100, 10000);
    updatePWM();
    Serial.printf("PWM frequency updated to: %d Hz\n", settings.pwm_freq);
    settings.save();
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
    settings.save();
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
  else if (cmd == "MEM")
  {
    Serial.println("\n=== Memory Information ===");
    
    // Heap memory information
    size_t freeHeap = ESP.getFreeHeap();
    size_t totalHeap = ESP.getHeapSize();
    size_t usedHeap = totalHeap - freeHeap;
    size_t minFreeHeap = ESP.getMinFreeHeap();
    size_t maxAllocHeap = ESP.getMaxAllocHeap();
    
    Serial.printf("Heap Memory:\n");
    Serial.printf("  Total: %u bytes (%.1f KB)\n", totalHeap, totalHeap / 1024.0);
    Serial.printf("  Used:  %u bytes (%.1f KB, %.1f%%)\n", 
                 usedHeap, usedHeap / 1024.0, (usedHeap * 100.0) / totalHeap);
    Serial.printf("  Free:  %u bytes (%.1f KB, %.1f%%)\n", 
                 freeHeap, freeHeap / 1024.0, (freeHeap * 100.0) / totalHeap);
    Serial.printf("  Min Free: %u bytes (%.1f KB)\n", minFreeHeap, minFreeHeap / 1024.0);
    Serial.printf("  Max Alloc: %u bytes (%.1f KB)\n", maxAllocHeap, maxAllocHeap / 1024.0);
    
    // Flash memory information (LittleFS)
    size_t totalFS = LittleFS.totalBytes();
    size_t usedFS = LittleFS.usedBytes();
    size_t freeFS = totalFS - usedFS;
    
    Serial.printf("\nFlash Storage (LittleFS):\n");
    Serial.printf("  Total: %u bytes (%.1f KB)\n", totalFS, totalFS / 1024.0);
    Serial.printf("  Used:  %u bytes (%.1f KB, %.1f%%)\n", 
                 usedFS, usedFS / 1024.0, (usedFS * 100.0) / totalFS);
    Serial.printf("  Free:  %u bytes (%.1f KB, %.1f%%)\n", 
                 freeFS, freeFS / 1024.0, (freeFS * 100.0) / totalFS);
    
    // Task information
    Serial.printf("\nTask Information:\n");
    Serial.printf("  Control Task Stack: %u bytes free\n", uxTaskGetStackHighWaterMark(controlTaskHandle));
    Serial.printf("  Network Task Stack: %u bytes free\n", uxTaskGetStackHighWaterMark(networkTaskHandle));
    Serial.printf("  Main Loop Stack: %u bytes free\n", uxTaskGetStackHighWaterMark(NULL));
    
    // Additional system info
    Serial.printf("\nSystem:\n");
    Serial.printf("  Flash Size: %u MB\n", ESP.getFlashChipSize() / (1024 * 1024));
    Serial.printf("  Chip Model: %s\n", ESP.getChipModel());
    Serial.printf("  CPU Freq: %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("  SDK Version: %s\n", ESP.getSdkVersion());
  }
  else if (cmd == "AW ON" )
  {
    settings.antiWindup = true;
    Serial.println("Anti-windup for deadband enabled");
    settings.save();
  }
  else if (cmd == "AW OFF" )
  {
    settings.antiWindup = false;
    Serial.println("Anti-windup for deadband disabled");
    settings.save();
  }
  else if (cmd == "HYST ON" )
  {
    settings.hysteresis = true;
    Serial.println("Hysteresis compensation enabled");
    settings.save();
  }
  else if (cmd == "HYST OFF" )
  {
    settings.hysteresis = false;
    Serial.println("Hysteresis compensation disabled");
    settings.save();
  }
  else if (cmd.startsWith("HYSTAMT "))
  {
    settings.hystAmount = cmd.substring(8).toFloat();
    settings.hystAmount = constrain(settings.hystAmount, 0.0, 20.0);
    Serial.printf("Hysteresis compensation amount set to: %.1f%%\n", settings.hystAmount);
    settings.save();
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
    
    settings.save();
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
    
    settings.save();
  }
  else if (cmd == "HELP")
  {    Serial.println(
      "\n╔═══════════════════════════════════════════════════════════════════════════════╗"
      "\n║                          VENTCON2 COMMAND REFERENCE                           ║"
      "\n║                        All commands are case-insensitive                      ║"
      "\n╚═══════════════════════════════════════════════════════════════════════════════╝"
      "\n"
      "\n┌─ PID CONTROL ─────────────────────────────────────────────────────────────────┐"
      "\n│ KP <value>      │ Set proportional gain (e.g., KP 0.5)                        │"
      "\n│ KI <value>      │ Set integral gain (e.g., KI 0.1)                            │"
      "\n│ KD <value>      │ Set derivative gain (e.g., KD 0.01)                         │"
      "\n│ SP <value>      │ Set pressure setpoint in bar (e.g., SP 3.0)                 │"
      "\n│ SAMPLE <ms>     │ Set PID sample time, 1-1000ms (e.g., SAMPLE 10)             │"
      "\n│ RESET           │ Reset PID controller (clear windup & state)                 │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"
      "\n┌─ SIGNAL PROCESSING ───────────────────────────────────────────────────────────┐"
      "\n│ FLT <value>     │ Set filter strength, 0.0-1.0 (e.g., FLT 0.2)                │"
      "\n│ AW ON/OFF       │ Enable/disable anti-windup for deadband                     │"
      "\n│ HYST ON/OFF     │ Enable/disable hysteresis compensation                      │"
      "\n│ HYSTAMT <value> │ Set hysteresis amount in % (e.g., HYSTAMT 5)                │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"
      "\n┌─ PWM & VALVE CONTROL ─────────────────────────────────────────────────────────┐"
      "\n│ FREQ <hz>       │ Set PWM frequency, 100-10000Hz (e.g., FREQ 1000)            │"
      "\n│ RES <bits>      │ Set PWM resolution, 1-16 bits (e.g., RES 8)                 │"
      "\n│ PWM <percent>   │ Force PWM duty cycle, 0-100% (e.g., PWM 25)                 │"
      "\n│ RESUME          │ Resume normal PID control after manual PWM                  │"
      "\n│ CONTROL FREQ <> │ Set control loop frequency, 10-1000Hz                       │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"
      "\n┌─ AUTO-TUNING ─────────────────────────────────────────────────────────────────┐"
      "\n│ TUNE START      │ Start PID auto-tuning process                               │"
      "\n│ TUNE STOP       │ Cancel auto-tuning process                                  │"
      "\n│ TUNE ACCEPT     │ Accept calculated PID parameters                            │"
      "\n│ TUNE REJECT     │ Reject and keep current PID parameters                      │"
      "\n│ TUNE SP <bar>   │ Set auto-tune test setpoint, 0.5-10.0 bar                   │"
      "\n│ TUNE MIN <pct>  │ Set auto-tune minimum PWM, 50-90%                           │"
      "\n│ TUNE MAX <pct>  │ Set auto-tune maximum PWM, 60-95%                           │"
      "\n│ TUNE CYCLE <ms> │ Set min cycle time, 50-2000ms                               │"
      "\n│ TUNE RULE <0-3> │ Select tuning rule (see TUNE RULES)                         │"
      "\n│ TUNE AGGR <val> │ Set aggressiveness factor, 0.5-2.0                          │"
      "\n│ TUNE RULES      │ Show available tuning rules with descriptions               │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"
      "\n┌─ NETWORK & WIFI ──────────────────────────────────────────────────────────────┐"
      "\n│ SCAN WIFI       │ Scan and list WiFi networks with signal strength            │"
      "\n│ WIFI CHANNEL <> │ Set WiFi AP channel, 1-13 (interference avoidance)          │"
      "\n│ PAGE ON/OFF     │ Enable/disable web server processing                        │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"      "\n┌─ DATA & MONITORING ───────────────────────────────────────────────────────────┐"
      "\n│ STATUS          │ Show comprehensive system status                            │"
      "\n│ SENSOR          │ Test SensorManager and show detailed sensor readings       │"
      "\n│ STARTCD         │ Start continuous data output for plotting                   │"
      "\n│ STOPCD          │ Stop continuous data output                                 │"
      "\n│ MEM             │ Show memory usage and system information                    │"
      "\n│ VER             │ Display firmware version and build info                     │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"
      "\n┌─ FILE SYSTEM & SETTINGS ──────────────────────────────────────────────────────┐"
      "\n│ SAVE            │ Force save current settings to flash memory                 │"
      "\n│ READ            │ Read and display settings stored in flash                   │"
      "\n│ DIR             │ List all files in flash memory with sizes                   │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"
      "\n┌─ QUICK REFERENCE ─────────────────────────────────────────────────────────────┐"
      "\n│ HELP            │ Show this command reference                                 │"
      "\n│                 │                                                             │"
      "\n│ Example workflow:                                                             │"
      "\n│   STATUS        → Check current system state                                  │"
      "\n│   TUNE START    → Begin auto-tuning for optimal PID                           │"
      "\n│   TUNE ACCEPT   → Accept calculated parameters                                │"
      "\n│   STARTCD       → Monitor real-time data                                      │"
      "\n│   SAVE          → Persist settings to flash                                   │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘");
  }  else if (cmd == "SCAN WIFI")
  {
    // Call the WiFi scanning function from WebHandler
    if (webHandler) {
      webHandler->scanWiFiNetworks();
    } else {
      Serial.println("Error: WebHandler not initialized");
    }
  }
  else if (cmd.startsWith("WIFI CHANNEL "))
  {
    // Change WiFi AP channel
    int channel = cmd.substring(13).toInt();
    if (webHandler) {
      webHandler->changeWiFiChannel(channel);
    } else {
      Serial.println("Error: WebHandler not initialized");
    }
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
    if (sensorManager) {
      Serial.printf("  ADC Source: %s\n", sensorManager->isADSFound() ? "ADS1015" : "ESP32 Internal");
      if (sensorManager->isADSFound()) {
        Serial.printf("  ADS1015 Address: 0x48\n");
        Serial.printf("  ADS1015 Channel: %d\n", SensorConfig::ADC_CHANNEL);
        Serial.printf("  Gain Setting: GAIN_TWOTHIRDS (+/-6.144V)\n");
      } else {
        Serial.printf("  Fallback Pin: %d\n", HardwareConfig::FALLBACK_ANALOG_PIN);
      }
      Serial.printf("  Raw ADC Value: %d\n", sensorManager->getADCValue());
      Serial.printf("  Voltage: %.3f V\n", sensorManager->getVoltage());
      Serial.printf("  Raw Pressure: %.3f bar\n", sensorManager->getRawPressure());
      Serial.printf("  Filtered Pressure: %.3f bar\n", sensorManager->getPressure());
      Serial.printf("  Voltage Range: %.1fV - %.1fV\n", SensorConfig::MIN_VOLTAGE, SensorConfig::MAX_VOLTAGE);
      Serial.printf("  Pressure Range: %.1f - %.1f bar\n", SensorConfig::SENSOR_MIN_BAR, SensorConfig::SENSOR_MAX_BAR);
    } else {
      Serial.println("  ERROR: SensorManager not initialized");
    }
    
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
    Serial.printf("  Hysteresis Comp: %s (%.1f%%)\n",                 settings.hysteresis ? "Enabled" : "Disabled", settings.hystAmount);    // Network section
    Serial.println("\nNetwork:");
    Serial.printf("  SSID: %s\n", webHandler ? webHandler->getAPSSID() : "Not Initialized");
    Serial.printf("  IP: %s\n", webHandler ? webHandler->getAPIP().toString().c_str() : "Not Initialized");
    Serial.printf("  Channel: %d (%.1f MHz)\n", webHandler ? webHandler->getWiFiChannel() : 0, webHandler ? 2412.0 + (webHandler->getWiFiChannel() - 1) * 5.0 : 0.0);
    Serial.printf("  Connected Clients: %d/%d\n", webHandler ? webHandler->getConnectedClients() : 0, NetworkConfig::MAX_CLIENTS);
    Serial.printf("  Web Server: %s\n", webHandler ? (webHandler->isWebServerEnabled() ? "Enabled" : "Disabled") : "Not Initialized");
  }
  else if (cmd == "SAVE")
  {
    settings.save();
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
  }  else if (cmd == "PAGE ON")
  {
    if (webHandler) {
      webHandler->setWebServerEnabled(true);
      Serial.println("Web server processing enabled");
    } else {
      Serial.println("Error: WebHandler not initialized");
    }
  }
  else if (cmd == "PAGE OFF")
  {
    if (webHandler) {
      webHandler->setWebServerEnabled(false);
      Serial.println("Web server processing disabled");
    } else {
      Serial.println("Error: WebHandler not initialized");
    }
  }
  else if (cmd == "VER")
  {
    // Display firmware version and build timestamp
    Serial.printf("Firmware Version: %s\n", getVersionString());
  }
  else if (cmd == "RESET")
  {    // Reset PID controller
    Serial.println("Resetting PID controller...");
    
    // Reset the PID controller by re-initializing it
    pid.SetMode(MANUAL); // 
    pwmOutput = 0;
    ledcWrite(PWM_CHANNEL_MOSFET, 0);
    
    // Note: Reset SensorManager filter state
    if (sensorManager) {
        sensorManager->resetFilterState();
    }
    
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
    settings.save();
    pid.SetTunings(settings.Kp, settings.Ki, settings.Kd);
  }
  else if (cmd == "TUNE REJECT")
  {
    Serial.println("New PID parameters rejected. Reverting to previous values.");
    settings.load(); // Reload previous settings
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

  else if (cmd == "SENSOR")
  {
    // Test SensorManager functionality
    if (sensorManager) {
      Serial.println("\n=== SensorManager Test ===");
      sensorManager->printSensorStatus();
      
      // Take a fresh reading
      sensorManager->readSensor();
      
      Serial.println("\nCurrent Readings:");
      Serial.printf("  ADC Value: %d\n", sensorManager->getADCValue());
      Serial.printf("  Voltage: %.3f V\n", sensorManager->getVoltage());
      Serial.printf("  Raw Pressure: %.3f bar\n", sensorManager->getRawPressure());
      Serial.printf("  Filtered Pressure: %.3f bar\n", sensorManager->getPressure());
      Serial.printf("  Pressure Safe: %s\n", sensorManager->isPressureSafe() ? "Yes" : "NO - EMERGENCY!");
      Serial.printf("  Filter Strength: %.2f\n", settings.filter_strength);
    } else {
      Serial.println("ERROR: SensorManager not initialized");
    }
  }
  else
  {
    Serial.println("Invalid command. Type 'HELP' for options.");
  }
}



// Low-pass filter function - DEPRECATED: Now handled by SensorManager
// Kept for reference during transition
/*
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
*/



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
        
        // ====== Sensor Reading using SensorManager ======
        sensorManager->readSensor();
        
        // Save last pressure before updating, used for hysteresis compensation
        lastPressure = pressureInput;
        
        // Update Input value for PID before computation
        pressureInput = sensorManager->getPressure();        // ====== Analog Pressure Signal Output =====
        static unsigned long lastAnalogOutTime = 0;
        if (millis() - lastAnalogOutTime >= 10)
        { 
            // Output analog pressure signal to ANALOG_PRESS_PIN
            float pressurePercent = pressureInput / SensorConfig::SENSOR_MAX_BAR;
            uint32_t analogOutValue = (uint32_t)(pressurePercent * ((1 << pwm_analog_pressure_signal_pwm_res) - 1));
            ledcWrite(PWM_CHANNEL_ANALOG_PRESS, analogOutValue);   
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
        }        // ====== Emergency Shutdown ======
        if (!sensorManager->isPressureSafe())
        {
            // Stop PWM output
            ledcWrite(PWM_CHANNEL_MOSFET, 0);
            
            static unsigned long lastEmergencyMsgTime = 0;
            if (millis() - lastEmergencyMsgTime >= 1000) 
            {
                Serial.printf("EMERGENCY SHUTDOWN! Pressure %.2f bar exceeds safe limit.\n", pressureInput);
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
              sensorManager->getVoltage(),
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
{    while(true) 
    {
        if (webHandler && webHandler->isWebServerEnabled()) 
        {
            unsigned long netStart = micros();
            
            webHandler->getDNSServer().processNextRequest();
            webHandler->getWebServer().handleClient();
            
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
{  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("====================================================");
  Serial.println("Ventcon System Starting...");
  Serial.println("====================================================");
  
  // Initialize LittleFS for settings storage
  if (!LittleFS.begin(true))
    Serial.println("LittleFS error!");
  settings.load();
  
  // Update PWM max value after loading settings
  pwm_max_value = (1 << settings.pwm_res) - 1;
  showSettingsFromLittleFS(); // Show loaded settings on startup

  // Initialize SensorManager
  sensorManager = new SensorManager(&settings);
  if (!sensorManager->initialize()) {
    Serial.println("ERROR: Failed to initialize SensorManager!");
    // Continue anyway as SensorManager has fallback options
  }

  ledcSetup(PWM_CHANNEL_ANALOG_PRESS, pwm_analog_pressure_signal_freq, pwm_analog_pressure_signal_pwm_res);
  ledcAttachPin(ANALOG_PRESS_PIN, PWM_CHANNEL_ANALOG_PRESS);  ledcSetup(PWM_CHANNEL_MOSFET, settings.pwm_freq, settings.pwm_res);
  ledcAttachPin(SOLENOID_PIN, PWM_CHANNEL_MOSFET);
  // Initialize WebHandler with dependency injection
  // Note: We need to create a persistent bool pointer for ads_found status
  static bool ads_found_status = sensorManager->isADSFound();
  webHandler = new WebHandler(&settings, &pid, 
                             &pressureInput, &pwmOutput, &ads_found_status, 
                             &pwm_max_value, sensorManager->getLastFilteredPressurePtr());
  
  // Initialize WiFi AP and DNS server
  webHandler->initializeWiFiAP();
  
  // Setup WiFi event handler
  webHandler->setupWiFiEvents();

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
  );  // Initialize PID 
  pid.SetMode(PID::Automatic);
  pid.SetOutputLimits(0, pwm_max_value);
  pid.SetSampleTime(settings.pid_sample_time); // Use settings value for consistency

  // Setup all web routes
  webHandler->setupRoutes();
  Serial.println("\nSystem Ready - AP Mode");
  Serial.println("\nType HELP for command options");  Serial.printf("Connect to: %s\nPassword: %s\n", webHandler->getAPSSID(), webHandler->getAPPassword());
  Serial.printf("Access via: http://www.ventcon.local\n");
  Serial.print("Direct IP access: http://");
  Serial.println(webHandler->getAPIP());
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



