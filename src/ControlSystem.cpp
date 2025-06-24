#include "ControlSystem.h"
#include "Constants.h"

ControlSystem::ControlSystem(Settings* settings, SensorManager* sensorManager, 
                           AutoTuner* autoTuner, PID* pid, double* pressureInput, 
                           double* pwmOutput, int* pwm_max_value, bool* manualPWMMode,
                           bool* continousValueOutput)
    : settings(settings), sensorManager(sensorManager), autoTuner(autoTuner), 
      pid(pid), pressureInput(pressureInput), pwmOutput(pwmOutput), 
      pwm_max_value(pwm_max_value), manualPWMMode(manualPWMMode),
      continousValueOutput(continousValueOutput), lastPressure(0.0), 
      pressureIncreasing(false), SERIAL_OUTPUT_INTERVAL(100),
      pwm_analog_pressure_signal_pwm_res(12) 
{
}

uint32_t ControlSystem::mapPwmToValve(double pidOutput, int maxPwmValue) 
{
    // Calculate the percentage within PID range (0-100%)
    float pidPercent = (pidOutput / maxPwmValue) * 100.0;
    
    // If below minimum threshold, keep valve closed
    if (pidPercent < 1.0) 
    {
        return 0;
    }
    
    // Apply hysteresis compensation if enabled
    if (settings->hysteresis) 
    {
        // Determine if pressure is currently increasing or decreasing
        bool currentlyIncreasing = *pressureInput > lastPressure;
        
        // Apply compensation only when direction changes
        if (currentlyIncreasing != pressureIncreasing) 
        {
            // When changing from increasing to decreasing, add compensation
            if (!currentlyIncreasing) 
            {
                pidPercent += settings->hystAmount;
            }
            // When changing from decreasing to increasing, subtract compensation
            else 
            {
                pidPercent -= settings->hystAmount;
            }
            
            // Update direction flag
            pressureIncreasing = currentlyIncreasing;
        }
    }
    
    // Map PID's 0-100% to valve's effective range
    float mappedPercent = ValveConfig::VALVE_MIN_DUTY + (pidPercent / 100.0) * 
                         (ValveConfig::VALVE_MAX_DUTY - ValveConfig::VALVE_MIN_DUTY);
    
    // Constrain to valid range
    mappedPercent = constrain(mappedPercent, 0.0, 100.0);
    
    // Convert percentage back to absolute PWM value
    return (uint32_t)((mappedPercent / 100.0) * maxPwmValue);
}

void ControlSystem::handleAnalogPressureOutput() 
{
    static unsigned long lastAnalogOutTime = 0;
    if (millis() - lastAnalogOutTime >= 10) 
    { 
        // Output analog pressure signal to ANALOG_PRESS_PIN
        float pressurePercent = *pressureInput / SensorConfig::SENSOR_MAX_BAR;
        uint32_t analogOutValue = (uint32_t)(pressurePercent * ((1 << pwm_analog_pressure_signal_pwm_res) - 1));
        ledcWrite(HardwareConfig::PWM_CHANNEL_ANALOG_PRESS, analogOutValue);   
        lastAnalogOutTime = millis();
    }
}

void ControlSystem::handleEmergencyShutdown() 
{
    if (!sensorManager->isPressureSafe()) 
    {
        // Stop PWM output
        ledcWrite(HardwareConfig::PWM_CHANNEL_MOSFET, 0);
        
        static unsigned long lastEmergencyMsgTime = 0;
        if (millis() - lastEmergencyMsgTime >= 1000) 
        {
            Serial.printf("EMERGENCY SHUTDOWN! Pressure %.2f bar exceeds safe limit.\n", *pressureInput);
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
          (autoTuner && autoTuner->isRunning() ? autoTuner->getOutputValue() : (*pwmOutput / *pwm_max_value) * 100.0),
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
        
        // ====== Analog Pressure Signal Output =====
        handleAnalogPressureOutput();
        
        // Process auto-tuning if active
        if (autoTuner && autoTuner->isRunning()) 
        {
            autoTuner->process();
        }
        // PID calculation and PWM output (only if not in auto-tune mode)
        else if (!*manualPWMMode) 
        {
            // Store previous output for anti-windup check
            double previousOutput = *pwmOutput;
            
            // Constrain output to valid range
            *pwmOutput = constrain(*pwmOutput, 0, *pwm_max_value);

            // Compute PID output
            pid->Compute();
            
            // Anti-windup for deadband and saturation
            if (settings->antiWindup) 
            {
                float pidPercent = (*pwmOutput / *pwm_max_value) * 100.0;
                
                if ((pidPercent < ValveConfig::VALVE_MIN_DUTY && *pwmOutput > previousOutput) ||
                    (pidPercent > ValveConfig::VALVE_MAX_DUTY && *pwmOutput > previousOutput)) 
                {
                    // Reset the PID to prevent integral accumulation
                    pid->SetMode(PID::Manual);
                    pid->SetMode(PID::Automatic);
                    
                    // Optional debug output if continuous output is enabled
                    static unsigned long lastDebugTime = 0;
                    if (*continousValueOutput && (millis() - lastDebugTime >= SERIAL_OUTPUT_INTERVAL)) 
                    {
                        if (pidPercent < ValveConfig::VALVE_MIN_DUTY) 
                        {
                            Serial.println("Anti-windup: Below min duty cycle");
                        } 
                        else 
                        {
                            Serial.println("Anti-windup: Above max duty cycle");
                        }
                        lastDebugTime = millis();
                    }
                }
            }

            // Use the mapping function to get actual PWM value to apply
            uint32_t actualPwm = mapPwmToValve(*pwmOutput, *pwm_max_value);
            ledcWrite(HardwareConfig::PWM_CHANNEL_MOSFET, actualPwm);
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
