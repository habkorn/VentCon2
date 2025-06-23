#include "Settings.h"

// Define static const members
constexpr double Settings::DEFAULT_KP;
constexpr double Settings::DEFAULT_KI;
constexpr double Settings::DEFAULT_KD;
constexpr float Settings::DEFAULT_FILTER_STRENGTH;
constexpr double Settings::DEFAULT_SETPOINT;
constexpr int Settings::DEFAULT_PWM_FREQ;
constexpr int Settings::DEFAULT_PWM_RES;
constexpr int Settings::DEFAULT_PID_SAMPLE_TIME;
constexpr int Settings::DEFAULT_CONTROL_FREQ_HZ;
constexpr bool Settings::DEFAULT_ANTI_WINDUP;
constexpr bool Settings::DEFAULT_HYSTERESIS;
constexpr float Settings::DEFAULT_HYST_AMOUNT;

const char* Settings::SETTINGS_FILE_PATH = "/settings.json";

Settings::Settings() {
    resetToDefaults();
}

Settings::~Settings() {
    // Destructor - no specific cleanup needed
}

void Settings::resetToDefaults() {
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
}

bool Settings::load() {
    if (!LittleFS.exists(SETTINGS_FILE_PATH)) {
        Serial.println("Settings file not found, using defaults");
        return false;
    }
    
    File file = LittleFS.open(SETTINGS_FILE_PATH, "r");
    if (!file) {
        Serial.println("Failed to open settings file for reading");
        return false;
    }
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
        Serial.print("Failed to parse settings.json: ");
        Serial.println(error.c_str());
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
    
    Serial.println("Settings loaded successfully from LittleFS");
    return true;
}

bool Settings::save() {
    File file = LittleFS.open(SETTINGS_FILE_PATH, "w");
    if (!file) {
        Serial.println("Failed to open settings file for writing");
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
    
    if (serializeJson(doc, file) == 0) {
        Serial.println("Failed to write settings to file");
        file.close();
        return false;
    }
    
    file.close();
    Serial.println("Settings saved to LittleFS");
    return true;
}

void Settings::printSettings() {
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

void Settings::printStoredSettings() {
    if (!LittleFS.exists(SETTINGS_FILE_PATH)) {
        Serial.println("No settings file found in LittleFS");
        return;
    }
    
    File file = LittleFS.open(SETTINGS_FILE_PATH, "r");
    if (!file) {
        Serial.println("Failed to open settings file for reading");
        return;
    }
    
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();
    
    if (error) {
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
