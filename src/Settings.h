#pragma once

#include <ArduinoJson.h>
#include <LittleFS.h>

/**
 * Settings Class
 * 
 * Encapsulates all system configuration parameters for the VentCon2 system.
 * Provides methods for loading, saving, and managing configuration data
 * with automatic persistence to LittleFS.
 * 
 * Features:
 * - Automatic load/save to flash storage
 * - Default value management
 * - Validation and constraints
 * - JSON serialization/deserialization
 * 
 * Usage:
 * 1. Create Settings instance
 * 2. Call load() to read from flash
 * 3. Access parameters via public members
 * 4. Call save() to persist changes
 */
class Settings {
public:
    // PID Parameters
    double Kp;
    double Ki;
    double Kd;
    
    // System Parameters
    float filter_strength;
    double setpoint;
    int pwm_freq;
    int pwm_res;
    int pid_sample_time;        // Sample time for PID control in milliseconds
    int control_freq_hz;        // Control loop frequency in Hz
    
    // Advanced Features
    bool antiWindup;            // Flag to enable/disable anti-windup for deadband
    bool hysteresis;            // Flag to enable/disable hysteresis compensation
    float hystAmount;           // Amount of hysteresis compensation (percentage points)
    
    // Constructor with default values
    Settings();
    
    // Destructor
    ~Settings();
    
    // Load settings from LittleFS
    bool load();
    
    // Save settings to LittleFS
    bool save();
    
    // Reset to default values
    void resetToDefaults();
    
    // Display current settings to Serial
    void printSettings();
    
    // Display settings stored in LittleFS to Serial
    void printStoredSettings();

private:
    // Default values
    static constexpr double DEFAULT_KP = 0.0;
    static constexpr double DEFAULT_KI = 0.0;
    static constexpr double DEFAULT_KD = 0.0;
    static constexpr float DEFAULT_FILTER_STRENGTH = 0.0;
    static constexpr double DEFAULT_SETPOINT = 3.0;
    static constexpr int DEFAULT_PWM_FREQ = 2000;
    static constexpr int DEFAULT_PWM_RES = 14;
    static constexpr int DEFAULT_PID_SAMPLE_TIME = 10;
    static constexpr int DEFAULT_CONTROL_FREQ_HZ = 1000;
    static constexpr bool DEFAULT_ANTI_WINDUP = false;
    static constexpr bool DEFAULT_HYSTERESIS = false;
    static constexpr float DEFAULT_HYST_AMOUNT = 5.0;
    
    static const char* SETTINGS_FILE_PATH;
};
