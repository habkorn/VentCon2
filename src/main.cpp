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
#include "Logger.h" // Add Logger include

// ====== Hardware Configuration ======
const int SOLENOID_PIN = HardwareConfig::SOLENOID_PIN;
const int ANALOG_PRESS_PIN = HardwareConfig::ANALOG_PRESS_PIN;
const int PWM_CHANNEL_MOSFET = HardwareConfig::PWM_CHANNEL_MOSFET;

// ====== Settings and Sensor Instances ======
SettingsHandler settings; // Use SettingsHandler class instance
SensorManager* sensorManager = nullptr; // Will be initialized in setup()
AutoTuner* autoTuner = nullptr; // Will be initialized in setup()

// ====== PID and Control Variables ======
double pressureInput = 0, pwmPIDoutput = 0; // PID input (pressure) and output (PWM), zero-initialized for safe PID startup
uint32_t actualPwm = 0; // Mapped valve PWM value (after PID→valve mapping), shared with WebHandler
PID pid(&pressureInput, &pwmPIDoutput, &settings.setpoint, settings.Kp, settings.Ki, settings.Kd, DIRECT);

// ====== System Management Instances ======
// Will be initialized in setup() after all dependencies are ready
WebHandler* webHandler = nullptr;
CommandProcessor* commandProcessor = nullptr;
ControlSystem* controlSystem = nullptr;
TaskManager* taskManager = nullptr;


// ====== Global Variables ======
bool continousValueOutput = false; // Flag for Serial output
long lastcontinousValueOutputTime=0; // Last time output was sent to Serial
int pwmFullScaleRaw = (1 << settings.pwm_res) - 1; // Maximum PWM value based on resolution
bool manualPWMMode = false; // Flag to track manual PWM control mode
int SERIAL_OUTPUT_INTERVAL  = TimingConfig::SERIAL_OUTPUT_INTERVAL_MS; // Interval for continuous serial output in milliseconds

void showSettingsFromLittleFS()
{
    // Use the SettingsHandler class method to display stored settings
    settings.printStoredSettings();
}


// ====== Arduino Setup Function ======
void setup()
{
  Serial.begin(UartConfig::BAUD_RATE);
  Logger::init();  // Initialize logging system
  delay(TimingConfig::STARTUP_DELAY_MS);
  Serial.println();
  Serial.println("====================================================");
  Serial.println("Ventcon System Starting...");
  Serial.println("====================================================");
  
  // Initialize LittleFS for settings storage
  if (!LittleFS.begin(true))
  {
    Serial.println("LittleFS error!");
  }
  else
  {
    Serial.println("LittleFS mounted. Files:");
    File root = LittleFS.open("/");
    File file = root.openNextFile();
    while(file)
    {
      Serial.printf("  /%s (%d bytes)\n", file.name(), file.size());
      file = root.openNextFile();
    }
  }
  settings.load();
  
  // Update PWM max value after loading settings
  pwmFullScaleRaw = (1 << settings.pwm_res) - 1;
  showSettingsFromLittleFS(); // Show loaded settings on startup
  // Initialize SensorManager
  sensorManager = new SensorManager(&settings);
  if (!sensorManager->initialize()) 
  {
    Serial.println("ERROR: Failed to initialize SensorManager!");
    // Continue anyway as SensorManager has fallback options
  }

  // Initialize AutoTuner
  autoTuner = new AutoTuner(&settings, &pid, &pressureInput, &pwmFullScaleRaw);
  Serial.println("AutoTuner initialized successfully!");

  // Setup PWM for solenoid valve control
  ledcSetup(PWM_CHANNEL_MOSFET, settings.pwm_freq, settings.pwm_res);
  // ledcAttachPin(pin, channel): Assigns the defined PWM channel to the output GPIO pin
  // 
  // 1. PIN (SOLENOID_PIN):
  //    - On ESP32-S3, the GPIO Matrix allows mapping PWM signal to almost any output GPIO.
  //    - Restrictions: Do not use input-only GPIOS or strapping pins during boot.
  //
  // 2. CHANNEL (PWM_CHANNEL_MOSFET):
  //    - ESP32-S3 has 8 LEDC channels (0-7), unlike original ESP32's 16.
  //    - Features: Hardware-controlled duty cycle, automatic fading support.
  //    - Limitations:
  //      a) Resolution trade-off: Higher frequencies reduce max available resolution (bits).
  //         Formula: Max_Freq = APB_CLK (80MHz) / 2^Resolution_Bits (approx).
  //      b) Timer sharing: 4 hardware timers exist. Multiple channels can share a timer,
  //         but all channels sharing a timer must use the same frequency.
  ledcAttachPin(SOLENOID_PIN, PWM_CHANNEL_MOSFET);
  
  // Initialize WebHandler with dependency injection
  // Note: We need to create a persistent bool pointer for ads_found status
  static bool ads_found_status = sensorManager->isADSFound();
  webHandler = new WebHandler(&settings, &pid, 
                             &pressureInput, &pwmPIDoutput, &actualPwm, &ads_found_status, 
                             &pwmFullScaleRaw, sensorManager->getLastFilteredPressurePtr());
  
  // Initialize WiFi AP and DNS server
  webHandler->initializeWiFiAP();
  
  // Setup WiFi event handler
  webHandler->setupWiFiEvents();
  // Initialize ControlSystem
  controlSystem = new ControlSystem(&settings, sensorManager, autoTuner, &pid,
                                   &pressureInput, &pwmPIDoutput, &actualPwm, &pwmFullScaleRaw,
                                   &manualPWMMode, &continousValueOutput);
  controlSystem->initializeControlSystem();

  // Initialize PID fully BEFORE starting tasks to prevent race condition.
  // Order matters: SetOutputLimits and SetSampleTime must precede SetMode(Automatic)
  // because SetMode triggers Initialize() which clamps outputSum to current limits,
  // and SetTunings scales gains by the current sample time.
  pid.SetOutputLimits(0, pwmFullScaleRaw);
  pid.SetSampleTime(settings.pid_sample_time);
  pid.SetTunings(settings.Kp, settings.Ki, settings.Kd); // Apply gains loaded from flash (constructor used pre-load defaults)

  // Read sensor once so Initialize() captures real pressure in lastInput,
  // preventing a derivative kick on the first Compute() cycle.
  sensorManager->readSensor();
  pressureInput = sensorManager->getPressure();

  pid.SetMode(PID::Automatic);

  // Initialize TaskManager
  taskManager = new TaskManager(controlSystem, webHandler);
  
  // Initialize CommandProcessor (after webHandler and taskManager are ready)
  commandProcessor = new CommandProcessor(&settings, sensorManager, autoTuner, 
                                        webHandler, &pid, &pressureInput, &pwmPIDoutput,
                                        &actualPwm, &pwmFullScaleRaw, &manualPWMMode, &continousValueOutput,
                                        taskManager);
  Serial.println("CommandProcessor initialized successfully!");

  // Create and start tasks
  if (!taskManager->createTasks()) 
  {
    Serial.println("ERROR: Failed to create one or more tasks!");
  }

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
  vTaskDelay(pdMS_TO_TICKS(TaskConfig::NETWORK_TASK_DELAY_MS)); // 10ms delay - serial commands don't need high frequency
}



