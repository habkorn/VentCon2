#ifndef CONTROLSYSTEM_H
#define CONTROLSYSTEM_H

#include <Arduino.h>
#include <PID_v2.h>
#include "SettingsHandler.h"
#include "SensorManager.h"
#include "AutoTuner.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class ControlSystem 
{
private:
    // Dependencies
    SettingsHandler* settings;
    SensorManager* sensorManager;
    AutoTuner* autoTuner;
    PID* pid;
    
    // System state pointers
    double* pressureInput;
    double* pwmPIDoutput;
    uint32_t* actualPwm;  // Mapped valve PWM value written each control cycle
    int* pwmFullScaleRaw;
    bool* manualPWMMode;
    bool* continousValueOutput;
    
    // Internal state variables
    float lastPressure;
    bool pressureIncreasing;
    int SERIAL_OUTPUT_INTERVAL;
    
    // Helper methods
    uint32_t mapPIDoutputToPwmValve(double pidOutput, int maxPwmFullScaleRaw);
    void applyHysteresisCompensation(double& output);
    void handleEmergencyShutdown();
    void handleContinuousDataOutput(unsigned long taskStartTime, unsigned long lastCycleEnd);

public:
    ControlSystem(SettingsHandler* settings, SensorManager* sensorManager, AutoTuner* autoTuner,
                 PID* pid, double* pressureInput, double* pwmPIDoutput, uint32_t* actualPwm, int* pwmFullScaleRaw,
                 bool* manualPWMMode, bool* continousValueOutput);
    
    void processControlLoop();
    void initializeControlSystem();
    
    // Static task wrapper
    static void controlTaskWrapper(void* parameter);
};

#endif // CONTROLSYSTEM_H
