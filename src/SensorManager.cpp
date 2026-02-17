#include "SensorManager.h"
#include "Logger.h"
#include <Arduino.h>
#include <Wire.h>

SensorManager::SensorManager(SettingsHandler* settings)
    : ads(),
      ads_found(false),
      adc_value(0),
      voltage(0.0),
      raw_pressure(0.0),
      filtered_pressure(0.0),
      last_filtered_pressure(0.0),
      settings(settings) 
{
}

bool SensorManager::initialize() 
{
    LOG_I(CAT_SENSOR, "Initializing SensorManager...");
    
    // Try to initialize ADS1015
    if (initializeADS1015()) 
    {
        ads_found = true;
        LOG_I(CAT_SENSOR, "ADS1015 found and configured successfully!");
        printSensorStatus();
        return true;
    }
    
    // Fallback to ESP32 internal ADC
    ads_found = false;
    pinMode(FALLBACK_ANALOG_PIN, INPUT);
    LOG_W(CAT_SENSOR, "ADS1015 not found! Using ESP32 internal ADC as fallback.");
    printSensorStatus();
    
    return true; // Always return true since we have a fallback
}

bool SensorManager::initializeADS1015(unsigned long timeout_ms) 
{
    unsigned long start_time = millis();
    
    while (millis() - start_time < timeout_ms) 
    {
        if (ads.begin(ADS1015_I2C_ADDRESS, &Wire)) 
        {
            // Configure ADS1015 settings
            ads.setGain(ADS1015_GAIN);
            ads.setDataRate(ADS1015_DATA_RATE);
            
            // Test read to verify communication
            int16_t test_reading = ads.readADC_SingleEnded(ADC_CHANNEL);
            if (test_reading != 0 || ads.computeVolts(test_reading) >= 0) 
            {
                return true;
            }
        }
        delay(TimingConfig::I2C_INIT_DELAY_MS);
    }
    
    return false;
}

void SensorManager::readSensor() 
{
    // Read ADC value based on available hardware
    if (ads_found) 
    {
        adc_value = ads.readADC_SingleEnded(ADC_CHANNEL);
        voltage = ads.computeVolts(adc_value);
    } 
    else 
    {
        // Use ESP32 built-in ADC as fallback
        adc_value = analogRead(FALLBACK_ANALOG_PIN);
        voltage = adc_value * (Esp32AdcConfig::VREF / Esp32AdcConfig::ADC_MAX); // Assuming 3.3V reference
    }
    
    // Calculate pressure from voltage
    raw_pressure = calculatePressure(voltage);
    
    // Apply low-pass filter
    filtered_pressure = lowPassFilter(raw_pressure);
}

float SensorManager::lowPassFilter(float measurement) 
{
    // Apply exponential low-pass filter
    // new_filtered = (1-alpha) * measurement + (alpha) * last_filtered
    float filtered = (1.0f - settings->filter_strength) * measurement + 
                    (settings->filter_strength) * last_filtered_pressure;
    
    // Store current result for next iteration
    last_filtered_pressure = filtered;
    
    return filtered;
}

float SensorManager::calculatePressure(float voltage) 
{
    // Read calibration from settings (runtime-configurable)
    const float minV = settings->sensor_min_voltage;
    const float maxV = settings->sensor_max_voltage;
    const float minP = settings->sensor_min_pressure;
    const float maxP = settings->sensor_max_pressure;
    
    // Handle invalid voltages
    if (voltage < minV) 
    {
        return 0.0;
    }
    
    // Linear interpolation: pressure = (voltage - min_voltage) * scale + min_pressure
    float voltage_range = maxV - minV;
    float pressure_range = maxP - minP;
    
    return minP + (pressure_range / voltage_range) * (voltage - minV);
}

void SensorManager::printSensorStatus() const 
{
    Serial.println("\n=== Sensor Configuration ===");
    Serial.printf("ADC Source: %s\n", ads_found ? "ADS1015" : "ESP32 Internal");
    
    if (ads_found) 
    {
        Serial.printf("ADS1015 Address: 0x%02X\n", ADS1015_I2C_ADDRESS);
        Serial.printf("ADS1015 Channel: %d\n", ADC_CHANNEL);
        
        const char* gainStr;
        switch(ADS1015_GAIN) 
        {
            case GAIN_TWOTHIRDS: gainStr = "GAIN_TWOTHIRDS (+/-6.144V)"; break;
            case GAIN_ONE: gainStr = "GAIN_ONE (+/-4.096V)"; break;
            case GAIN_TWO: gainStr = "GAIN_TWO (+/-2.048V)"; break;
            case GAIN_FOUR: gainStr = "GAIN_FOUR (+/-1.024V)"; break;
            case GAIN_EIGHT: gainStr = "GAIN_EIGHT (+/-0.512V)"; break;
            case GAIN_SIXTEEN: gainStr = "GAIN_SIXTEEN (+/-0.256V)"; break;
            default: gainStr = "Unknown"; break;
        }
        Serial.printf("Gain Setting: %s\n", gainStr);
        Serial.printf("Data Rate: %d SPS\n", ADS1015_DATA_RATE);
    } 
    else 
    {
        Serial.printf("Fallback Pin: %d\n", FALLBACK_ANALOG_PIN);
        Serial.printf("Reference Voltage: %.1fV\n", Esp32AdcConfig::VREF);
        Serial.printf("Resolution: 12-bit (0-%d)\n", Esp32AdcConfig::ADC_MAX);
    }

    Serial.printf("Filter Strength: %.2f\n", settings->filter_strength);
    
    Serial.println("\n=== Pressure Sensor Configuration ===");
    Serial.printf("Voltage Range: %.1fV - %.1fV\n", settings->sensor_min_voltage, settings->sensor_max_voltage);
    Serial.printf("Pressure Range: %.1f - %.1f bar\n", settings->sensor_min_pressure, settings->sensor_max_pressure);
    Serial.println("=============================");
}

bool SensorManager::isPressureSafe() const 
{
    // Check if pressure exceeds safe limit (10% above max rated pressure)
    return filtered_pressure <= (settings->sensor_max_pressure * 1.1f);
}
