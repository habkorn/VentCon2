#pragma once

// System Version
#define VENTCON_VERSION "2.0.5"

// Network Configuration
namespace NetworkConfig 
{
    constexpr char AP_SSID[] = "VENTCON_AP";
    constexpr char AP_PASSWORD[] = "ventcon12!";
    constexpr int MAX_CLIENTS = 2;
}

// Hardware Configuration
namespace HardwareConfig 
{
    constexpr int SOLENOID_PIN = 5;
    constexpr int ANALOG_PRESS_PIN = 10;
    constexpr int PWM_CHANNEL_MOSFET = 0;
    constexpr int PWM_CHANNEL_ANALOG_PRESS = 1;
    constexpr int FALLBACK_ANALOG_PIN = A0;  // ESP32 internal ADC pin
}

// Sensor Configuration
namespace SensorConfig 
{
    constexpr int ADC_CHANNEL = 0;
    constexpr float MIN_VOLTAGE = 2.0f;
    constexpr float MAX_VOLTAGE = 4.09f;
    constexpr float SENSOR_MIN_BAR = 0.0f;
    constexpr float SENSOR_MAX_BAR = 7.5f;
}

// Valve Configuration
namespace ValveConfig 
{
    constexpr float VALVE_MIN_DUTY = 50.0f;
    constexpr float VALVE_MAX_DUTY = 90.0f;
}
