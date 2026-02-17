#pragma once

// System version string including build timestamp for identification and debugging
#define VENTCON_VERSION "2.2.0 (Build: " __DATE__ " " __TIME__ ")"

// Network Configuration
namespace NetworkConfig 
{
    // SSID (network name) for the WiFi Access Point created by the device
    constexpr char AP_SSID[] = "VENTCON_AP";
    
    // Password required to connect to the WiFi Access Point (must be at least 8 characters)
    constexpr char AP_PASSWORD[] = "ventcon12!";
    
    // Maximum number of simultaneous client connections allowed to the Access Point
    constexpr int MAX_CLIENTS = 2;
    
    // Access Point IP address configuration (192.168.4.1)
    constexpr uint8_t AP_IP[4] = {192, 168, 4, 1};
    
    // Access Point gateway address (same as IP for AP mode)
    constexpr uint8_t AP_GATEWAY[4] = {192, 168, 4, 1};
    
    // Access Point subnet mask (255.255.255.0 = /24 network)
    constexpr uint8_t AP_SUBNET[4] = {255, 255, 255, 0};
    
    // DNS server port for captive portal
    constexpr int DNS_PORT = 53;
    
    // Captive portal domain name (redirects to AP_IP)
    constexpr char CAPTIVE_PORTAL_DOMAIN[] = "www.ventcon.at";
    
    // mDNS hostname (accessible as http://ventcon.local)
    constexpr char MDNS_HOSTNAME[] = "ventcon";
    
    // Web server port
    constexpr int WEB_PORT = 80;
    
    // File streaming chunk size in bytes
    constexpr size_t CHUNK_SIZE = 1024;
    
    // WiFi channel 1 frequency (MHz) for frequency calculation
    constexpr float WIFI_CH1_FREQ_MHZ = 2412.0f;
    
    // WiFi channel step (MHz between channels)
    constexpr float WIFI_CHANNEL_STEP_MHZ = 5.0f;
}

// Hardware Configuration
namespace HardwareConfig 
{
    // GPIO pin number for controlling the solenoid valve (digital output)
    constexpr int SOLENOID_PIN = 5;
    
    // GPIO pin number for analog pressure sensor input
    constexpr int ANALOG_PRESS_PIN = 10;
    
    // PWM channel assignment for MOSFET control (ESP32 has 16 PWM channels: 0-15)
    constexpr int PWM_CHANNEL_MOSFET = 0;
    
    // Fallback analog input pin using ESP32's internal ADC when primary pin fails
    constexpr int FALLBACK_ANALOG_PIN = A0;  // ESP32 internal ADC pin
}

// Sensor Configuration — Factory Defaults
// These values are compile-time fallbacks used when settings.json is missing or
// corrupt (e.g. first boot, flash erase). At runtime, the actual sensor
// calibration is read from SettingsHandler (loaded from settings.json).
// To change defaults for the web UI "Reset to Default" button, also update
// data/default.json to keep both sources in sync.
namespace SensorConfigDefaults 
{
    // ADC channel number for pressure sensor readings (ESP32 ADC1 channels: 0-7)
    constexpr int ADC_CHANNEL_PRESS_SENS = 0;
    
    // Minimum voltage output from pressure sensor (corresponds to minimum pressure)
    constexpr float SENSOR_MIN_VOLTAGE = 0.5f;
    
    // Maximum voltage output from pressure sensor (corresponds to maximum pressure)
    constexpr float SENSOR_MAX_VOLTAGE = 4.50f;
    
    // Minimum pressure reading in bar that the sensor can measure
    constexpr float SENSOR_MIN_BAR = 0.0f;
    
    // Maximum pressure reading in bar that the sensor can measure (10 bar range)
    constexpr float SENSOR_MAX_BAR = 10.f;
}

// Valve Configuration
// The solenoid valve has a non-linear response: it doesn't move at low duty cycles
// and saturates before 100%. These constants define the effective operating range.
//
// NOTE: Web interface displays UNMAPPED PID output (0-100%), not the actual valve
// duty cycle. Example: Web shows 50% → actual valve receives 70% duty cycle.
// See WebHandlers.cpp handleValues() for the displayed calculation.
namespace ValveConfig 
{
    // Minimum PID output percentage threshold - below this, valve stays closed.
    // Prevents tiny PID outputs from wastefully energizing the valve coil.
    // Used in: mapPwmToValve() to return 0 when output is negligible.
    constexpr float PID_MIN_OUTPUT_PERCENT = 1.0f;
    
    // Minimum PWM duty cycle where valve physically begins to respond.
    // Below ~50%, the solenoid doesn't generate enough force to move.
    // PID's 0-100% output is mapped to this as the lower bound.
    // Example: PID 0% → Valve 50%, PID 50% → Valve 70%
    // Used in: mapPwmToValve() for range mapping, anti-windup detection,
    //          and AutoTuner for gain compensation.
    constexpr float VALVE_MIN_DUTY = 50.0f;
    
    // Maximum PWM duty cycle for valve operation.
    // Above ~90%, additional current generates heat without more force.
    // PID's 0-100% output is mapped to this as the upper bound.
    // Example: PID 100% → Valve 90%
    // Used in: mapPwmToValve() for range mapping and AutoTuner.
    constexpr float VALVE_MAX_DUTY = 90.0f;
    
    // Hysteresis compensation range (percentage points).
    // Compensates for mechanical friction causing different open/close thresholds.
    constexpr float HYST_MIN = 0.0f;
    constexpr float HYST_MAX = 20.0f;
}

// UART Configuration
namespace UartConfig
{
    // Serial communication baud rate
    constexpr unsigned long BAUD_RATE = 115200;
}

// Timing Configuration
namespace TimingConfig
{
    // Emergency message interval in milliseconds
    constexpr unsigned long EMERGENCY_MSG_INTERVAL_MS = 1000;
    
    // Serial output interval in milliseconds
    constexpr int SERIAL_OUTPUT_INTERVAL_MS = 100;
    
    // Startup delay in milliseconds
    constexpr unsigned long STARTUP_DELAY_MS = 1000;
    
    // WiFi peripheral reset delay in milliseconds
    constexpr unsigned long WIFI_RESET_DELAY_MS = 100;
    
    // I2C initialization delay in milliseconds
    constexpr unsigned long I2C_INIT_DELAY_MS = 100;
}

// FreeRTOS Task Configuration
namespace TaskConfig
{
    // Stack size for FreeRTOS tasks in bytes
    constexpr uint32_t STACK_SIZE = 4096;
    
    // Network task delay in milliseconds
    constexpr int NETWORK_TASK_DELAY_MS = 10;
    
    // Network delay warning threshold in microseconds
    constexpr unsigned long NETWORK_DELAY_WARNING_US = 20000;
}

// PWM Configuration
namespace PwmConfig
{
    // Minimum PWM frequency in Hz
    constexpr int MIN_FREQ_HZ = 100;
    
    // Maximum PWM frequency in Hz
    constexpr int MAX_FREQ_HZ = 10000;
    
    // PWM frequency slider step size in Hz
    constexpr int FREQ_STEP_HZ = 100;
    
    // Minimum PWM resolution in bits (ESP32 PWM < 8 bits is impractical)
    constexpr int MIN_RES_BITS = 8;
    
    // Maximum PWM resolution in bits
    constexpr int MAX_RES_BITS = 16;
    
    // PWM resolution slider step size in bits
    constexpr int RES_STEP_BITS = 1;
}

// Filter Configuration
namespace FilterConfig
{
    // Minimum low-pass filter strength (no filtering)
    constexpr float MIN_STRENGTH = 0.0f;
    
    // Maximum low-pass filter strength (maximum smoothing)
    constexpr float MAX_STRENGTH = 1.0f;
    
    // Filter strength slider step size
    constexpr float STRENGTH_STEP = 0.01f;
}

// Control Loop Configuration
namespace ControlConfig
{
    // Minimum control loop frequency in Hz
    constexpr int MIN_FREQ_HZ = 10;
    
    // Maximum control loop frequency in Hz
    constexpr int MAX_FREQ_HZ = 1000;
    
    // Minimum PID sample time in milliseconds
    constexpr int MIN_SAMPLE_TIME_MS = 1;
    
    // Maximum PID sample time in milliseconds
    constexpr int MAX_SAMPLE_TIME_MS = 1000;
}

// Auto-Tuning Configuration
namespace AutoTuneConfig
{
    // Minimum PWM output during auto-tuning (percentage)
    constexpr float MIN_PWM_PERCENT = 65.0f;
    
    // Maximum PWM output during auto-tuning (percentage)
    constexpr float MAX_PWM_PERCENT = 85.0f;
    
    // Minimum cycle time for valid oscillation detection (ms)
    constexpr unsigned long MIN_CYCLE_TIME_MS = 100;
    
    // Initial max value for min tracking (sentinel value)
    constexpr float INITIAL_MIN_PRESSURE = 999.0f;
    
    // Default test setpoint for auto-tuning (bar)
    constexpr float DEFAULT_TEST_SETPOINT = 5.0f;
    
    // Test setpoint range limits (bar)
    constexpr float MIN_TEST_SETPOINT = 0.5f;
    constexpr float MAX_TEST_SETPOINT = SensorConfigDefaults::SENSOR_MAX_BAR;
    
    // Tuning aggressiveness defaults and range
    constexpr float DEFAULT_AGGRESSIVENESS = 2.0f;
    constexpr float MIN_AGGRESSIVENESS = 0.5f;
    constexpr float MAX_AGGRESSIVENESS = 2.0f;
    
    // Minimum cycle time range for setMinCycleTime() (ms)
    constexpr unsigned long CYCLE_TIME_LOWER_BOUND = 50UL;
    constexpr unsigned long CYCLE_TIME_UPPER_BOUND = 2000UL;
    
    // Auto-tune timeout (ms) - 3 minutes
    constexpr unsigned long TIMEOUT_MS = 180000UL;
    
    // Deadband around setpoint (bar)
    constexpr float NOISE_BAND = 0.1f;
}

// ESP32 ADC Configuration
namespace Esp32AdcConfig
{
    // ESP32 internal ADC reference voltage
    constexpr float VREF = 3.3f;
    
    // ESP32 12-bit ADC maximum value
    constexpr int ADC_MAX = 4095;
}

// Slider Limits Defaults (for web UI)
namespace SliderDefaults
{
    // Setpoint slider limits (bar)
    constexpr float SP_MIN = SensorConfigDefaults::SENSOR_MIN_BAR;
    constexpr float SP_MAX = SensorConfigDefaults::SENSOR_MAX_BAR;
    constexpr float SP_STEP = 0.1f;
    
    // Proportional gain slider limits
    constexpr float KP_MIN = 0.0f;
    constexpr float KP_MAX = 3000.0f;
    constexpr float KP_STEP = 1.0f;
    
    // Integral gain slider limits
    constexpr float KI_MIN = 0.0f;
    constexpr float KI_MAX = 5000.0f;
    constexpr float KI_STEP = 1.0f;
    
    // Derivative gain slider limits
    constexpr float KD_MIN = 0.0f;
    constexpr float KD_MAX = 1000.0f;
    constexpr float KD_STEP = 1.0f;
}
