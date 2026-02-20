#pragma once

#include <ArduinoJson.h>
#include <LittleFS.h>
#include "Constants.h"

/**
 * SliderLimits Structure
 * 
 * Defines configurable min/max/step values for UI sliders.
 */
struct SliderLimits {
    float min;
    float max;
    float step;
};

/**
 * ChartSettings Structure
 * 
 * Defines configurable axis limits and time window for the live chart.
 */
struct ChartSettings {
    float y_min;        // Pressure axis minimum (bar)
    float y_max;        // Pressure axis maximum (bar)
    float pwm_min;      // Duty-cycle axis minimum (%)
    float pwm_max;      // Duty-cycle axis maximum (%)
    int time_window;    // X-axis rolling window (seconds)
    int time_grid;      // X-axis grid/tick interval (seconds)
};

/**
 * SettingsHandler Class
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
 * 1. Create SettingsHandler instance
 * 2. Call load() to read from flash
 * 3. Access parameters via public members
 * 4. Call save() to persist changes
 */
class SettingsHandler 
{
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
    
    // Slider Limits (user-configurable min/max/step for UI)
    SliderLimits sp_limits;     // Setpoint slider limits
    SliderLimits kp_limits;     // Kp slider limits
    SliderLimits ki_limits;     // Ki slider limits
    SliderLimits kd_limits;     // Kd slider limits
    
    // Sensor Calibration (user-configurable pressure/voltage mapping)
    float sensor_min_pressure;  // Minimum pressure reading (bar)
    float sensor_max_pressure;  // Maximum pressure reading (bar)
    float sensor_min_voltage;   // Sensor output at min pressure (V)
    float sensor_max_voltage;   // Sensor output at max pressure (V)
    
    // Chart Axis Settings (user-configurable limits for live chart)
    ChartSettings chart_settings;
    
    // Constructor with default values
    SettingsHandler();
    
    // Destructor
    ~SettingsHandler();
    
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
    static const char* SETTINGS_FILE_PATH;
};
