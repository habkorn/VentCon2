#ifndef CONTROLSYSTEM_H
#define CONTROLSYSTEM_H

#include <Arduino.h>
#include <PID_v2.h>
#include "Settings.h"
#include "SensorManager.h"
#include "AutoTuner.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class ControlSystem 
{
private:
    // Dependencies
    Settings* settings;
    SensorManager* sensorManager;
    AutoTuner* autoTuner;
    PID* pid;
    
    // System state pointers
    double* pressureInput;
    double* pwmOutput;
    int* pwm_max_value;
    bool* manualPWMMode;
    bool* continousValueOutput;
    
    // Internal state variables
    float lastPressure;
    bool pressureIncreasing;
    int SERIAL_OUTPUT_INTERVAL;
    int pwm_analog_pressure_signal_pwm_res;
    
    // Helper methods
    uint32_t mapPwmToValve(double pidOutput, int maxPwmValue);
    void handleEmergencyShutdown();
    void handleContinuousDataOutput(unsigned long taskStartTime, unsigned long lastCycleEnd);
    void handleAnalogPressureOutput();

public:
    ControlSystem(Settings* settings, SensorManager* sensorManager, AutoTuner* autoTuner,
                 PID* pid, double* pressureInput, double* pwmOutput, int* pwm_max_value,
                 bool* manualPWMMode, bool* continousValueOutput);
    
    void processControlLoop();
    void initializeControlSystem();
    
    // Static task wrapper
    static void controlTaskWrapper(void* parameter);
};

#endif // CONTROLSYSTEM_H
