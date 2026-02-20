#include "ControlSystem.h"
#include "Constants.h"
#include "Logger.h"

ControlSystem::ControlSystem(SettingsHandler* settings, SensorManager* sensorManager, 
                           AutoTuner* autoTuner, PID* pid, double* pressureInput, 
                           double* pwmPIDoutput, uint32_t* actualPwm, int* pwmFullScaleRaw, bool* manualPWMMode,
                           bool* continousValueOutput)
    : settings(settings), sensorManager(sensorManager), autoTuner(autoTuner), 
      pid(pid), pressureInput(pressureInput), pwmPIDoutput(pwmPIDoutput), 
      actualPwm(actualPwm), pwmFullScaleRaw(pwmFullScaleRaw), manualPWMMode(manualPWMMode),
      continousValueOutput(continousValueOutput), lastPressure(0.0), 
      pressureIncreasing(false), SERIAL_OUTPUT_INTERVAL(TimingConfig::SERIAL_OUTPUT_INTERVAL_MS) 
{
}

uint32_t ControlSystem::mapPIDoutputToPwmValve(double pidOutput, int maxPwmFullScaleRaw) 
{
    // Convert PID output to percentage (0-100%)
    float pidPercent = (pidOutput / maxPwmFullScaleRaw) * 100.0;
    
    // If below minimum threshold, keep valve closed
    if (pidPercent < ValveConfig::PID_MIN_OUTPUT_PERCENT) 
    {
        return 0;
    }
    
    // Map PID's 0-100% to valve's effective range
    float mappedPercent = ValveConfig::VALVE_MIN_DUTY + (pidPercent / 100.0) * 
                         (ValveConfig::VALVE_MAX_DUTY - ValveConfig::VALVE_MIN_DUTY);
    
    // Constrain to valid range
    mappedPercent = constrain(mappedPercent, 0.0, 100.0);
    
    // Convert percentage back to absolute PWM value
    return (uint32_t)((mappedPercent / 100.0) * maxPwmFullScaleRaw);
}

void ControlSystem::applyHysteresisCompensation(double& output) 
{
    if (!settings->hysteresis) 
    {
        return;
    }
    
    // Determine if pressure is currently increasing or decreasing
    bool currentlyIncreasing = *pressureInput > lastPressure;
    
    // Apply compensation only when direction changes
    if (currentlyIncreasing != pressureIncreasing) 
    {
        // Calculate compensation as fraction of max PWM value
        double compensation = (*pwmFullScaleRaw * settings->hystAmount) / 100.0;
        
        // When changing from increasing to decreasing, add compensation
        if (!currentlyIncreasing) 
        {
            output += compensation;
        }
        // When changing from decreasing to increasing, subtract compensation
        else 
        {
            output -= compensation;
        }
        
        // Constrain to valid range
        output = constrain(output, 0.0, (double)*pwmFullScaleRaw);
        
        // Update direction flag
        pressureIncreasing = currentlyIncreasing;
    }
}

void ControlSystem::handleEmergencyShutdown() 
{
    if (!sensorManager->isPressureSafe()) 
    {
        // Stop PWM output
        ledcWrite(HardwareConfig::PWM_CHANNEL_MOSFET, 0);
        
        // Log error (always logged, rate-limited by LOG_E behavior)
        static unsigned long lastEmergencyMsgTime = 0;
        if (millis() - lastEmergencyMsgTime >= TimingConfig::EMERGENCY_MSG_INTERVAL_MS) 
        {
            LOG_E(CAT_CONTROL, "EMERGENCY SHUTDOWN! Pressure %.2f bar exceeds safe limit", *pressureInput);
            lastEmergencyMsgTime = millis();
        }
    }
}

void ControlSystem::handleContinuousDataOutput(unsigned long taskStartTime, unsigned long lastCycleEnd) 
{
    static unsigned long lastContinuousOutputTime = 0;
    unsigned long deltaTimeContinuous = millis() - lastContinuousOutputTime;
    
    if (*continousValueOutput && (deltaTimeContinuous >= SERIAL_OUTPUT_INTERVAL)) 
    {
        // Calculate execution time for this task iteration
        unsigned long taskExecTime = micros() - taskStartTime;
        
        // Calculate total time including wait time from previous cycle
        unsigned long totalCycleTime = 0;
        if (lastCycleEnd > 0) 
        {
            totalCycleTime = taskStartTime - lastCycleEnd;
        }
        
        // Format all data at once using a single buffer
        char buffer[120];
        int len = snprintf(buffer, sizeof(buffer),
          "voltage=%.3f, error=%.3f, press=%.3f, setPress=%.3f, PWM%%=%.3f, t=%.2f, exec=%lu, total=%lu, task=CTRL%s\r\n",
          sensorManager->getVoltage(),
          settings->setpoint - *pressureInput,
          *pressureInput,
          settings->setpoint,
          (autoTuner && autoTuner->isRunning() ? autoTuner->getOutputValue() : (*pwmPIDoutput / *pwmFullScaleRaw) * 100.0),
          xTaskGetTickCount() * portTICK_PERIOD_MS / 1000.0, // Convert ticks to seconds
          taskExecTime, // Task execution time in microseconds
          totalCycleTime, // Total cycle time in microseconds (execution + wait)
          autoTuner && autoTuner->isRunning() ? ", TUNING" : "");

        // Send the entire buffer in one operation
        Serial.write(buffer, len);

        lastContinuousOutputTime = millis();
    }
}

void ControlSystem::processControlLoop() 
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    // Use configurable frequency
    TickType_t frequency = pdMS_TO_TICKS(1000 / settings->control_freq_hz);
    
    static unsigned long lastCycleEnd = 0;
    
    while(true) 
    {
        // Update frequency if settings changed
        frequency = pdMS_TO_TICKS(1000 / settings->control_freq_hz);
        unsigned long taskStartTime = micros();
        
        // ====== Sensor Reading using SensorManager ======
        sensorManager->readSensor();
        
        // Save last pressure before updating, used for hysteresis compensation
        lastPressure = *pressureInput;
        
        // Update Input value for PID before computation
        *pressureInput = sensorManager->getPressure();
        
        // Process auto-tuning if active
        if (autoTuner && autoTuner->isRunning()) 
        {
            autoTuner->process();
        }
        // PID calculation and PWM output (only if not in auto-tune mode)
        else if (!*manualPWMMode) 
        {
            // Store previous output for anti-windup check
            double previousOutput = *pwmPIDoutput;

            // Compute PID output
            pid->Compute();

            
            
            // Constrain output to valid range (safety check, SetOutputLimits should handle this)
            *pwmPIDoutput = constrain(*pwmPIDoutput, 0, *pwmFullScaleRaw);
            
            // Anti-windup for deadband and saturation
            if (settings->antiWindup) 
            {
                float pidPercent = (*pwmPIDoutput / *pwmFullScaleRaw) * 100.0;
                
                // Below dead zone and trying to decrease further (valve already closed)
                // Above full scale and trying to increase further (valve already at max duty)
                if ((pidPercent < ValveConfig::PID_MIN_OUTPUT_PERCENT && *pwmPIDoutput < previousOutput) ||
                    (pidPercent >= 100.0f && *pwmPIDoutput > previousOutput)) 
                {
                    // Reset the PID to prevent integral accumulation
                    pid->SetMode(PID::Manual);
                    pid->SetMode(PID::Automatic);
                    
                    // Debug output using Logger (rate-limited automatically)
                    if (pidPercent < ValveConfig::PID_MIN_OUTPUT_PERCENT) 
                    {
                        LOG_D(CAT_CONTROL, "Anti-windup: Below min duty (%.1f%%), valve closed", pidPercent);
                    } 
                    else 
                    {
                        LOG_D(CAT_CONTROL, "Anti-windup: Above max duty (%.1f%%), valve saturated", pidPercent);
                    }
                }
            }

            // Apply hysteresis compensation to PID output
            applyHysteresisCompensation(*pwmPIDoutput);

            // Use the mapping function to get actual PWM value to apply
            // *actualPwm range: 0 to pwmFullScaleRaw (e.g., 0-16383 for 14-bit)
            // This is the mapped value: PID 0-100% → valve 50-90% duty cycle
            *actualPwm = mapPIDoutputToPwmValve(*pwmPIDoutput, *pwmFullScaleRaw);

            ledcWrite(HardwareConfig::PWM_CHANNEL_MOSFET, *actualPwm);
        }
        
        // ====== Emergency Shutdown ======
        handleEmergencyShutdown();

        // ====== Continuous Data Output ======
        handleContinuousDataOutput(taskStartTime, lastCycleEnd);
        
        // Wait for next cycle - this properly yields to other tasks
        vTaskDelayUntil(&lastWakeTime, frequency);
        lastCycleEnd = taskStartTime; // Update last cycle start time
    }
}

void ControlSystem::controlTaskWrapper(void* parameter) 
{
    ControlSystem* controlSystem = static_cast<ControlSystem*>(parameter);
    controlSystem->processControlLoop();
}

void ControlSystem::initializeControlSystem() 
{
    // Any initialization specific to the control system can go here
    Serial.println("ControlSystem initialized successfully!");
}
