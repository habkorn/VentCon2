#pragma once

// System version string including build timestamp for identification and debugging
#define VENTCON_VERSION "2.1.6 (Build: " __DATE__ " " __TIME__ ")"

// Network Configuration
namespace NetworkConfig 
{
    // SSID (network name) for the WiFi Access Point created by the device
    constexpr char AP_SSID[] = "VENTCON_AP";
    
    // Password required to connect to the WiFi Access Point (must be at least 8 characters)
    constexpr char AP_PASSWORD[] = "ventcon12!";
    
    // Maximum number of simultaneous client connections allowed to the Access Point
    constexpr int MAX_CLIENTS = 2;
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
    
    // PWM channel assignment for analog pressure control (must be different from MOSFET channel)
    constexpr int PWM_CHANNEL_ANALOG_PRESS = 1;
    
    // Fallback analog input pin using ESP32's internal ADC when primary pin fails
    constexpr int FALLBACK_ANALOG_PIN = A0;  // ESP32 internal ADC pin
}

// Sensor Configuration
namespace SensorConfig 
{
    // ADC channel number for pressure sensor readings (ESP32 ADC1 channels: 0-7)
    constexpr int ADC_CHANNEL = 0;
    
    // Minimum voltage output from pressure sensor (corresponds to minimum pressure)
    constexpr float MIN_VOLTAGE = 0.5f;
    
    // Maximum voltage output from pressure sensor (corresponds to maximum pressure)
    constexpr float MAX_VOLTAGE = 4.50f;
    
    // Minimum pressure reading in bar that the sensor can measure
    constexpr float SENSOR_MIN_BAR = 0.0f;
    
    // Maximum pressure reading in bar that the sensor can measure (10 bar range)
    constexpr float SENSOR_MAX_BAR = 10.f;
}

// Valve Configuration
namespace ValveConfig 
{
    // Minimum PWM duty cycle percentage for valve operation (below this valve may not respond)
    constexpr float VALVE_MIN_DUTY = 50.0f;
    
    // Maximum PWM duty cycle percentage for valve operation (above this may damage valve)
    constexpr float VALVE_MAX_DUTY = 90.0f;
}
