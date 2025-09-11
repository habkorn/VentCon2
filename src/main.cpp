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
#include "SettingsHandler.h"   // Add SettingsHandler class include
#include "WebHandlers.h" // Add WebHandlers include
#include "SensorManager.h" // Add SensorManager include
#include "AutoTuner.h" // Add AutoTuner include
#include "CommandProcessor.h" // Add CommandProcessor include
#include "ControlSystem.h" // Add ControlSystem include
#include "TaskManager.h" // Add TaskManager include

// ====== Hardware Configuration ======
const int SOLENOID_PIN = HardwareConfig::SOLENOID_PIN;
const int ANALOG_PRESS_PIN = HardwareConfig::ANALOG_PRESS_PIN;
const int PWM_CHANNEL_MOSFET = HardwareConfig::PWM_CHANNEL_MOSFET;
const int PWM_CHANNEL_ANALOG_PRESS = HardwareConfig::PWM_CHANNEL_ANALOG_PRESS;

// ====== Settings and Sensor Instances ======
SettingsHandler settings; // Use SettingsHandler class instance
SensorManager* sensorManager = nullptr; // Will be initialized in setup()
AutoTuner* autoTuner = nullptr; // Will be initialized in setup()

// ====== Valve Configuration ======
const float VALVE_MIN_DUTY = ValveConfig::VALVE_MIN_DUTY; 
const float VALVE_MAX_DUTY = ValveConfig::VALVE_MAX_DUTY;

// ====== PID and Control Variables ======
double pressureInput, pwmOutput; // PID input (pressure) and output (PWM)
PID pid(&pressureInput, &pwmOutput, &settings.setpoint, settings.Kp, settings.Ki, settings.Kd, DIRECT);

// ====== System Management Instances ======
// Will be initialized in setup() after all dependencies are ready
WebHandler* webHandler = nullptr;
CommandProcessor* commandProcessor = nullptr;
ControlSystem* controlSystem = nullptr;
TaskManager* taskManager = nullptr;


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

void showSettingsFromLittleFS()
{
    // Use the SettingsHandler class method to display stored settings
    settings.printStoredSettings();
}


// ====== Version Information ======
const char* getVersionString() 
{
  static char versionString[50];
  sprintf(versionString, VENTCON_VERSION);
  return versionString;
}



// ====== Serial Command Parser ======
void parseSerialCommand(String cmd)
{
  if (commandProcessor) 
  {
    commandProcessor->processCommand(cmd);
  } 
  else 
  {
    Serial.println("ERROR: CommandProcessor not initialized");
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
  
  // Initialize LittleFS for settings storage
  if (!LittleFS.begin(true))
    Serial.println("LittleFS error!");
  settings.load();
  
  // Update PWM max value after loading settings
  pwm_max_value = (1 << settings.pwm_res) - 1;
  showSettingsFromLittleFS(); // Show loaded settings on startup
  // Initialize SensorManager
  sensorManager = new SensorManager(&settings);
  if (!sensorManager->initialize()) 
  {
    Serial.println("ERROR: Failed to initialize SensorManager!");
    // Continue anyway as SensorManager has fallback options
  }

  // Initialize AutoTuner
  autoTuner = new AutoTuner(&settings, &pid, &pressureInput, &pwm_max_value);
  Serial.println("AutoTuner initialized successfully!");
  // Initialize CommandProcessor (now with TaskManager reference)
  commandProcessor = new CommandProcessor(&settings, sensorManager, autoTuner, 
                                        webHandler, &pid, &pressureInput, &pwmOutput,
                                        &pwm_max_value, &manualPWMMode, &continousValueOutput,
                                        taskManager);
  Serial.println("CommandProcessor initialized successfully!");

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
  // Initialize ControlSystem
  controlSystem = new ControlSystem(&settings, sensorManager, autoTuner, &pid,
                                   &pressureInput, &pwmOutput, &pwm_max_value,
                                   &manualPWMMode, &continousValueOutput);
  controlSystem->initializeControlSystem();

  // Initialize TaskManager
  taskManager = new TaskManager(controlSystem, webHandler);
  
  // Create and start tasks
  if (!taskManager->createTasks()) 
  {
    Serial.println("ERROR: Failed to create one or more tasks!");
  }

  // Initialize PID 
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
void loop() // this runs on core 1, like the web server and command processor  
{
  // ====== Serial Command Processing ======
  if (commandProcessor) 
  {
    commandProcessor->processSerialInput();
  }
  
  // Small delay to prevent watchdog timeout and allow other tasks
  vTaskDelay(pdMS_TO_TICKS(10)); // 10ms delay - serial commands don't need high frequency
}



