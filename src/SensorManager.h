#pragma once

#include <Adafruit_ADS1X15.h>
#include "SettingsHandler.h"
#include "Constants.h"

/**
 * SensorManager Class
 * 
 * Handles all sensor-related operations including:
 * - ADS1015 ADC initialization and communication
 * - Fallback to ESP32 internal ADC
 * - Pressure calculation from voltage
 * - Low-pass filtering for noise reduction
 * - Sensor status monitoring
 */
class SensorManager 
{
private:
    // Hardware components
    Adafruit_ADS1015 ads;
    
    // Sensor status and data
    bool ads_found;
    int16_t adc_value;
    float voltage;
    float raw_pressure;
    float filtered_pressure;
    float last_filtered_pressure;
    
    // Configuration references
    SettingsHandler* settings;
    
    // Constants from configuration
    static constexpr int ADC_CHANNEL = SensorConfig::ADC_CHANNEL_PRESS_SENS;
    static constexpr float MIN_VOLTAGE = SensorConfig::MIN_VOLTAGE;
    static constexpr float MAX_VOLTAGE = SensorConfig::MAX_VOLTAGE;
    static constexpr float SENSOR_MIN_BAR = SensorConfig::SENSOR_MIN_BAR;
    static constexpr float SENSOR_MAX_BAR = SensorConfig::SENSOR_MAX_BAR;
    static constexpr int FALLBACK_ANALOG_PIN = HardwareConfig::FALLBACK_ANALOG_PIN;
    static constexpr uint8_t ADS1015_I2C_ADDRESS = 0x48;
    static constexpr adsGain_t ADS1015_GAIN = GAIN_TWOTHIRDS;
    static constexpr uint16_t ADS1015_DATA_RATE = RATE_ADS1015_1600SPS;
    
public:
    /**
     * Constructor
     * @param settings Pointer to SettingsHandler instance for filter configuration
     */
    SensorManager(SettingsHandler* settings);
    
    /**
     * Initialize the sensor system
     * Attempts to connect to ADS1015, falls back to ESP32 ADC if not found
     * @return true if initialization successful (either ADS1015 or fallback)
     */
    bool initialize();
    
    /**
     * Read sensor data and update all values
     * This should be called regularly from the control loop
     */
    void readSensor();
    
    /**
     * Get the current filtered pressure reading
     * @return Filtered pressure in bar
     */
    float getPressure() const { return filtered_pressure; }
    
    /**
     * Get the raw (unfiltered) pressure reading
     * @return Raw pressure in bar
     */
    float getRawPressure() const { return raw_pressure; }
    
    /**
     * Get the current voltage reading
     * @return Voltage in volts
     */
    float getVoltage() const { return voltage; }
    
    /**
     * Get the raw ADC value
     * @return Raw ADC reading (16-bit for ADS1015, 12-bit for ESP32)
     */
    int16_t getADCValue() const { return adc_value; }
    
    /**
     * Check if ADS1015 was successfully detected
     * @return true if ADS1015 is being used, false if using ESP32 fallback
     */
    bool isADSFound() const { return ads_found; }
    
    /**
     * Get pointer to last filtered pressure (for WebHandler compatibility)
     * @return Pointer to last_filtered_pressure member
     */
    float* getLastFilteredPressurePtr() { return &last_filtered_pressure; }
    
    /**
     * Get sensor configuration information for status display
     */
    void printSensorStatus() const;
    
    /**
     * Check if pressure is within safe operating limits
     * @return true if pressure is safe, false if emergency shutdown needed
     */
    bool isPressureSafe() const;
    
    /**
     * Reset sensor filter state (useful for PID reset operations)
     */
    void resetFilterState()
    { 
        last_filtered_pressure = 0.0f; 
        filtered_pressure = 0.0f;
    }
    
private:
    /**
     * Apply low-pass filter to pressure measurement
     * @param measurement Raw pressure measurement
     * @return Filtered pressure value
     */
    float lowPassFilter(float measurement);
    
    /**
     * Calculate pressure from voltage reading
     * @param voltage Input voltage from sensor
     * @return Pressure in bar
     */
    float calculatePressure(float voltage);
    
    /**
     * Attempt to initialize ADS1015 with timeout
     * @param timeout_ms Maximum time to spend trying to connect
     * @return true if ADS1015 successfully initialized
     */
    bool initializeADS1015(unsigned long timeout_ms = 2000);
};
