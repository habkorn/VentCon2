#include "SettingsHandler.h"
#include "Logger.h"
#include "Constants.h"

const char* SettingsHandler::SETTINGS_FILE_PATH = "/settings.json";

SettingsHandler::SettingsHandler() 
{
    resetToDefaults();
}

SettingsHandler::~SettingsHandler() 
{
    // Destructor - no specific cleanup needed
}

void SettingsHandler::resetToDefaults() 
{
    Kp = SettingsDefaults::KP;
    Ki = SettingsDefaults::KI;
    Kd = SettingsDefaults::KD;
    filter_strength = SettingsDefaults::FILTER_STRENGTH;
    setpoint = SettingsDefaults::SETPOINT;
    pwm_freq = SettingsDefaults::PWM_FREQ;
    pwm_res = SettingsDefaults::PWM_RES;
    pid_sample_time = SettingsDefaults::PID_SAMPLE_TIME;
    control_freq_hz = SettingsDefaults::CONTROL_FREQ_HZ;
    antiWindup = SettingsDefaults::ANTI_WINDUP;
    hysteresis = SettingsDefaults::HYSTERESIS;
    hystAmount = SettingsDefaults::HYST_AMOUNT;
    
    // Default slider limits
    sp_limits = {SliderDefaults::SP_MIN, SliderDefaults::SP_MAX, SliderDefaults::SP_STEP};
    kp_limits = {SliderDefaults::KP_MIN, SliderDefaults::KP_MAX, SliderDefaults::KP_STEP};
    ki_limits = {SliderDefaults::KI_MIN, SliderDefaults::KI_MAX, SliderDefaults::KI_STEP};
    kd_limits = {SliderDefaults::KD_MIN, SliderDefaults::KD_MAX, SliderDefaults::KD_STEP};
    
    // Default sensor calibration
    sensor_min_pressure = SensorConfigDefaults::SENSOR_MIN_BAR;
    sensor_max_pressure = SensorConfigDefaults::SENSOR_MAX_BAR;
    sensor_min_voltage = SensorConfigDefaults::SENSOR_MIN_VOLTAGE;
    sensor_max_voltage = SensorConfigDefaults::SENSOR_MAX_VOLTAGE;
    
    // Default chart axis settings
    chart_settings = {
        ChartDefaults::Y_MIN, ChartDefaults::Y_MAX,
        ChartDefaults::PWM_MIN, ChartDefaults::PWM_MAX,
        ChartDefaults::TIME_WINDOW, ChartDefaults::TIME_GRID
    };
}

bool SettingsHandler::load() 
{
    if (!LittleFS.exists(SETTINGS_FILE_PATH))
    {
        LOG_I(CAT_SYSTEM, "Settings file not found, using defaults");
        return false;
    }
    
    File file = LittleFS.open(SETTINGS_FILE_PATH, "r");
    if (!file)
    {
        LOG_E(CAT_SYSTEM, "Failed to open settings file for reading");
        return false;
    }
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error)
    {
        LOG_E(CAT_SYSTEM, "Failed to parse settings.json: %s", error.c_str());
        return false;
    }
    
    // Load values with defaults as fallback
    Kp = doc["Kp"] | SettingsDefaults::KP;
    Ki = doc["Ki"] | SettingsDefaults::KI;
    Kd = doc["Kd"] | SettingsDefaults::KD;
    filter_strength = doc["filter_strength"] | SettingsDefaults::FILTER_STRENGTH;
    setpoint = doc["setpoint"] | SettingsDefaults::SETPOINT;
    pwm_freq = doc["pwm_freq"] | SettingsDefaults::PWM_FREQ;
    pwm_res = doc["pwm_res"] | SettingsDefaults::PWM_RES;
    pid_sample_time = doc["pid_sample_time"] | SettingsDefaults::PID_SAMPLE_TIME;
    control_freq_hz = doc["control_freq_hz"] | SettingsDefaults::CONTROL_FREQ_HZ;
    antiWindup = doc["antiWindup"] | SettingsDefaults::ANTI_WINDUP;
    hysteresis = doc["hysteresis"] | SettingsDefaults::HYSTERESIS;
    hystAmount = doc["hystAmount"] | SettingsDefaults::HYST_AMOUNT;
    
    // Load slider limits with defaults from Constants.h
    JsonObject sp_lim = doc["sp_limits"];
    sp_limits.min = sp_lim["min"] | SliderDefaults::SP_MIN;
    sp_limits.max = sp_lim["max"] | SliderDefaults::SP_MAX;
    sp_limits.step = sp_lim["step"] | SliderDefaults::SP_STEP;
    
    JsonObject kp_lim = doc["kp_limits"];
    kp_limits.min = kp_lim["min"] | SliderDefaults::KP_MIN;
    kp_limits.max = kp_lim["max"] | SliderDefaults::KP_MAX;
    kp_limits.step = kp_lim["step"] | SliderDefaults::KP_STEP;
    
    JsonObject ki_lim = doc["ki_limits"];
    ki_limits.min = ki_lim["min"] | SliderDefaults::KI_MIN;
    ki_limits.max = ki_lim["max"] | SliderDefaults::KI_MAX;
    ki_limits.step = ki_lim["step"] | SliderDefaults::KI_STEP;
    
    JsonObject kd_lim = doc["kd_limits"];
    kd_limits.min = kd_lim["min"] | SliderDefaults::KD_MIN;
    kd_limits.max = kd_lim["max"] | SliderDefaults::KD_MAX;
    kd_limits.step = kd_lim["step"] | SliderDefaults::KD_STEP;
    
    // Load sensor calibration with defaults
    JsonObject sensor_lim = doc["sensor_limits"];
    sensor_min_pressure = sensor_lim["minP"] | SensorConfigDefaults::SENSOR_MIN_BAR;
    sensor_max_pressure = sensor_lim["maxP"] | SensorConfigDefaults::SENSOR_MAX_BAR;
    sensor_min_voltage  = sensor_lim["minV"] | SensorConfigDefaults::SENSOR_MIN_VOLTAGE;
    sensor_max_voltage  = sensor_lim["maxV"] | SensorConfigDefaults::SENSOR_MAX_VOLTAGE;
    
    // Load chart axis settings with defaults
    JsonObject chart = doc["chart_settings"];
    chart_settings.y_min      = chart["y_min"]      | ChartDefaults::Y_MIN;
    chart_settings.y_max      = chart["y_max"]      | ChartDefaults::Y_MAX;
    chart_settings.pwm_min    = chart["pwm_min"]     | ChartDefaults::PWM_MIN;
    chart_settings.pwm_max    = chart["pwm_max"]     | ChartDefaults::PWM_MAX;
    chart_settings.time_window = chart["time_window"] | ChartDefaults::TIME_WINDOW;
    chart_settings.time_grid    = chart["time_grid"]    | ChartDefaults::TIME_GRID;
    
    LOG_I(CAT_SYSTEM, "Settings loaded successfully from LittleFS");
    return true;
}

bool SettingsHandler::save() 
{
    File file = LittleFS.open(SETTINGS_FILE_PATH, "w");
    if (!file)
    {
        LOG_E(CAT_SYSTEM, "Failed to open settings file for writing");
        return false;
    }
    
    JsonDocument doc;
    doc["Kp"] = Kp;
    doc["Ki"] = Ki;
    doc["Kd"] = Kd;
    doc["filter_strength"] = filter_strength;
    doc["setpoint"] = setpoint;
    doc["pwm_freq"] = pwm_freq;
    doc["pwm_res"] = pwm_res;
    doc["pid_sample_time"] = pid_sample_time;
    doc["control_freq_hz"] = control_freq_hz;
    doc["antiWindup"] = antiWindup;
    doc["hysteresis"] = hysteresis;
    doc["hystAmount"] = hystAmount;
    
    // Save slider limits as nested objects
    JsonObject sp_lim = doc["sp_limits"].to<JsonObject>();
    sp_lim["min"] = sp_limits.min;
    sp_lim["max"] = sp_limits.max;
    sp_lim["step"] = sp_limits.step;
    
    JsonObject kp_lim = doc["kp_limits"].to<JsonObject>();
    kp_lim["min"] = kp_limits.min;
    kp_lim["max"] = kp_limits.max;
    kp_lim["step"] = kp_limits.step;
    
    JsonObject ki_lim = doc["ki_limits"].to<JsonObject>();
    ki_lim["min"] = ki_limits.min;
    ki_lim["max"] = ki_limits.max;
    ki_lim["step"] = ki_limits.step;
    
    JsonObject kd_lim = doc["kd_limits"].to<JsonObject>();
    kd_lim["min"] = kd_limits.min;
    kd_lim["max"] = kd_limits.max;
    kd_lim["step"] = kd_limits.step;
    
    // Save sensor calibration as nested object
    JsonObject sensor_lim = doc["sensor_limits"].to<JsonObject>();
    sensor_lim["minP"] = sensor_min_pressure;
    sensor_lim["maxP"] = sensor_max_pressure;
    sensor_lim["minV"] = sensor_min_voltage;
    sensor_lim["maxV"] = sensor_max_voltage;
    
    // Save chart axis settings as nested object
    JsonObject chart = doc["chart_settings"].to<JsonObject>();
    chart["y_min"]       = chart_settings.y_min;
    chart["y_max"]       = chart_settings.y_max;
    chart["pwm_min"]     = chart_settings.pwm_min;
    chart["pwm_max"]     = chart_settings.pwm_max;
    chart["time_window"] = chart_settings.time_window;
    chart["time_grid"]    = chart_settings.time_grid;
    
    if (serializeJson(doc, file) == 0)
    {
        LOG_E(CAT_SYSTEM, "Failed to write settings to file");
        file.close();
        return false;
    }
    
    file.close();
    LOG_I(CAT_SYSTEM, "Settings saved to LittleFS");
    return true;
}

void SettingsHandler::printSettings() 
{
    Serial.println("\n=== Current Settings ===");
    Serial.printf("PID Parameters:    Kp=%.2f, Ki=%.2f, Kd=%.2f\n", Kp, Ki, Kd);
    Serial.printf("Filter Strength:   %.2f\n", filter_strength);
    Serial.printf("Pressure Setpoint: %.2f bar\n", setpoint);
    Serial.printf("PWM Configuration: %d Hz, %d-bit (max value: %d)\n", 
                 pwm_freq, pwm_res, (1 << pwm_res) - 1);
    Serial.printf("Control Loop Freq: %d Hz\n", control_freq_hz);
    Serial.printf("PID Sample Time:   %d ms\n", pid_sample_time);
    Serial.printf("Anti-Windup:       %s\n", antiWindup ? "Enabled" : "Disabled");
    Serial.printf("Hysteresis Comp:   %s (%.1f%%)\n", 
                 hysteresis ? "Enabled" : "Disabled", hystAmount);
    Serial.printf("Sensor Pressure:   %.1f - %.1f bar\n", sensor_min_pressure, sensor_max_pressure);
    Serial.printf("Sensor Voltage:    %.2f - %.2f V\n", sensor_min_voltage, sensor_max_voltage);
    Serial.printf("Chart Axes:        Y[%.1f-%.1f] PWM[%.0f-%.0f] T=%ds grid=%ds\n",
                 chart_settings.y_min, chart_settings.y_max,
                 chart_settings.pwm_min, chart_settings.pwm_max,
                 chart_settings.time_window, chart_settings.time_grid);
}

void SettingsHandler::printStoredSettings() 
{
    if (!LittleFS.exists(SETTINGS_FILE_PATH))
    {
        Serial.println("No settings file found in LittleFS");
        return;
    }
    
    File file = LittleFS.open(SETTINGS_FILE_PATH, "r");
    if (!file)
    {
        Serial.println("Failed to open settings file for reading");
        return;
    }
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error)
    {
        Serial.println("\n=== Error Reading Settings File ===");
        Serial.printf("deserializeJson() failed: %s\n", error.c_str());
        return;
    }
    
    Serial.println("\n=== Settings Stored in LittleFS ===");
    Serial.printf("PID Parameters:    Kp=%.2f, Ki=%.2f, Kd=%.2f\n", 
                 doc["Kp"].as<double>(), doc["Ki"].as<double>(), doc["Kd"].as<double>());
    Serial.printf("Filter Strength:   %.2f\n", doc["filter_strength"].as<float>());
    Serial.printf("Pressure Setpoint: %.2f bar\n", doc["setpoint"].as<double>());
    Serial.printf("PWM Configuration: %d Hz, %d-bit (max value: %d)\n", 
                 doc["pwm_freq"].as<int>(), doc["pwm_res"].as<int>(), 
                 (1 << doc["pwm_res"].as<int>()) - 1);
    Serial.printf("Control Loop Freq: %d Hz\n", doc["control_freq_hz"].as<int>());
    Serial.printf("PID Sample Time:   %d ms\n", doc["pid_sample_time"].as<int>());
    Serial.printf("Anti-Windup:       %s\n", doc["antiWindup"].as<bool>() ? "Enabled" : "Disabled");
    Serial.printf("Hysteresis Comp:   %s (%.1f%%)\n", 
                 doc["hysteresis"].as<bool>() ? "Enabled" : "Disabled", 
                 doc["hystAmount"].as<float>());
}
