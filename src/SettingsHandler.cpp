#include "SettingsHandler.h"
#include "Logger.h"
#include "Constants.h"

// Define static const members
constexpr double SettingsHandler::DEFAULT_KP;
constexpr double SettingsHandler::DEFAULT_KI;
constexpr double SettingsHandler::DEFAULT_KD;
constexpr float SettingsHandler::DEFAULT_FILTER_STRENGTH;
constexpr double SettingsHandler::DEFAULT_SETPOINT;
constexpr int SettingsHandler::DEFAULT_PWM_FREQ;
constexpr int SettingsHandler::DEFAULT_PWM_RES;
constexpr int SettingsHandler::DEFAULT_PID_SAMPLE_TIME;
constexpr int SettingsHandler::DEFAULT_CONTROL_FREQ_HZ;
constexpr bool SettingsHandler::DEFAULT_ANTI_WINDUP;
constexpr bool SettingsHandler::DEFAULT_HYSTERESIS;
constexpr float SettingsHandler::DEFAULT_HYST_AMOUNT;

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
    Kp = DEFAULT_KP;
    Ki = DEFAULT_KI;
    Kd = DEFAULT_KD;
    filter_strength = DEFAULT_FILTER_STRENGTH;
    setpoint = DEFAULT_SETPOINT;
    pwm_freq = DEFAULT_PWM_FREQ;
    pwm_res = DEFAULT_PWM_RES;
    pid_sample_time = DEFAULT_PID_SAMPLE_TIME;
    control_freq_hz = DEFAULT_CONTROL_FREQ_HZ;
    antiWindup = DEFAULT_ANTI_WINDUP;
    hysteresis = DEFAULT_HYSTERESIS;
    hystAmount = DEFAULT_HYST_AMOUNT;
    
    // Default slider limits (from Constants.h)
    sp_limits = {SliderDefaults::SP_MIN, SliderDefaults::SP_MAX, SliderDefaults::SP_STEP};
    kp_limits = {SliderDefaults::KP_MIN, SliderDefaults::KP_MAX, SliderDefaults::KP_STEP};
    ki_limits = {SliderDefaults::KI_MIN, SliderDefaults::KI_MAX, SliderDefaults::KI_STEP};
    kd_limits = {SliderDefaults::KD_MIN, SliderDefaults::KD_MAX, SliderDefaults::KD_STEP};
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
    Kp = doc["Kp"] | DEFAULT_KP;
    Ki = doc["Ki"] | DEFAULT_KI;
    Kd = doc["Kd"] | DEFAULT_KD;
    filter_strength = doc["filter_strength"] | DEFAULT_FILTER_STRENGTH;
    setpoint = doc["setpoint"] | DEFAULT_SETPOINT;
    pwm_freq = doc["pwm_freq"] | DEFAULT_PWM_FREQ;
    pwm_res = doc["pwm_res"] | DEFAULT_PWM_RES;
    pid_sample_time = doc["pid_sample_time"] | DEFAULT_PID_SAMPLE_TIME;
    control_freq_hz = doc["control_freq_hz"] | DEFAULT_CONTROL_FREQ_HZ;
    antiWindup = doc["antiWindup"] | DEFAULT_ANTI_WINDUP;
    hysteresis = doc["hysteresis"] | DEFAULT_HYSTERESIS;
    hystAmount = doc["hystAmount"] | DEFAULT_HYST_AMOUNT;
    
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
