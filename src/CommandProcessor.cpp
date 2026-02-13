#include "CommandProcessor.h"
#include "Constants.h"
#include "TaskManager.h"
#include "Logger.h"
#include <LittleFS.h>
#include <esp_system.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

CommandProcessor::CommandProcessor(SettingsHandler* settings, SensorManager* sensorManager, 
                                 AutoTuner* autoTuner, WebHandler* webHandler, PID* pid,
                                 double* pressureInput, double* pwmOutput, int* pwmFullScaleRaw,
                                 bool* manualPWMMode, bool* continousValueOutput, 
                                 TaskManager* taskManager)
    : settings(settings), sensorManager(sensorManager), autoTuner(autoTuner), 
      webHandler(webHandler), pid(pid), pressureInput(pressureInput), 
      pwmOutput(pwmOutput), pwmFullScaleRaw(pwmFullScaleRaw), 
      manualPWMMode(manualPWMMode), continousValueOutput(continousValueOutput),
      taskManager(taskManager) 
{
}

void CommandProcessor::processCommand(const String& cmd) 
{
    String trimmedCmd = cmd;
    trimmedCmd.trim();
    trimmedCmd.toUpperCase();

    // Route commands to appropriate handlers
    if (trimmedCmd.startsWith("KP ") || trimmedCmd.startsWith("KI ") || 
        trimmedCmd.startsWith("KD ") || trimmedCmd.startsWith("SP ") ||
        trimmedCmd.startsWith("SAMPLE ") || trimmedCmd == "RESET") 
    {
        handlePIDCommands(trimmedCmd);
    }
    else if (trimmedCmd.startsWith("FLT ") || trimmedCmd == "AW ON" || 
             trimmedCmd == "AW OFF" || trimmedCmd == "HYST ON" || 
             trimmedCmd == "HYST OFF" || trimmedCmd.startsWith("HYSTAMT ")) 
    {
        handleSignalProcessingCommands(trimmedCmd);
    }
    else if (trimmedCmd.startsWith("FREQ ") || trimmedCmd.startsWith("RES ") ||
             trimmedCmd.startsWith("PWM ") || trimmedCmd == "RESUME" ||
             trimmedCmd.startsWith("CONTROL FREQ ")) 
    {
        handlePWMCommands(trimmedCmd);
    }
    else if (trimmedCmd.startsWith("TUNE ") || trimmedCmd == "AUTOTUNE") 
    {
        handleAutoTuningCommands(trimmedCmd);
    }
    else if (trimmedCmd == "SCAN WIFI" || trimmedCmd.startsWith("WIFI CHANNEL ") ||
             trimmedCmd == "PAGE ON" || trimmedCmd == "PAGE OFF") 
    {
        handleNetworkCommands(trimmedCmd);
    }
    else if (trimmedCmd == "STATUS" || trimmedCmd == "MEM" || trimmedCmd == "VER" ||
             trimmedCmd == "STARTCD" || trimmedCmd == "STOPCD" || trimmedCmd == "SENSOR") 
    {
        handleDiagnosticCommands(trimmedCmd);
    }
    else if (trimmedCmd == "READ" || trimmedCmd == "DIR" || trimmedCmd == "SAVE") 
    {
        handleFileSystemCommands(trimmedCmd);
    }
    else if (trimmedCmd == "HELP") 
    {
        showHelp();
    }
    else 
    {
        Serial.println("Invalid command. Type 'HELP' for options.");
    }
}

void CommandProcessor::handlePIDCommands(const String& cmd) 
{
    if (cmd.startsWith("KP ")) 
    {
        settings->Kp = cmd.substring(3).toFloat();
        pid->SetTunings(settings->Kp, settings->Ki, settings->Kd);
        Serial.printf("Proportional gain set to: %.2f\n", settings->Kp);
        settings->save();
    }
    else if (cmd.startsWith("KI ")) 
    {
        settings->Ki = cmd.substring(3).toFloat();
        pid->SetTunings(settings->Kp, settings->Ki, settings->Kd);
        Serial.printf("Integral gain set to: %.2f\n", settings->Ki);
        settings->save();
    }
    else if (cmd.startsWith("KD ")) 
    {
        settings->Kd = cmd.substring(3).toFloat();
        pid->SetTunings(settings->Kp, settings->Ki, settings->Kd);
        Serial.printf("Derivative gain set to: %.2f\n", settings->Kd);
        settings->save();
    }
    else if (cmd.startsWith("SP ")) 
    {
        settings->setpoint = cmd.substring(3).toFloat();
        Serial.printf("Setpoint updated to: %.2f bar\n", settings->setpoint);
        settings->save();
    }
    else if (cmd.startsWith("SAMPLE ")) 
    {
        int newSampleTime = cmd.substring(7).toInt();
        newSampleTime = constrain(newSampleTime, ControlConfig::MIN_SAMPLE_TIME_MS, ControlConfig::MAX_SAMPLE_TIME_MS);
        settings->pid_sample_time = newSampleTime;
        pid->SetSampleTime(settings->pid_sample_time);
        Serial.printf("PID sample time set to: %d ms (%.1f Hz)\n", 
                     settings->pid_sample_time, 1000.0/settings->pid_sample_time);
        
        // Check timing relationship and provide feedback
        float controlPeriod = 1000.0 / settings->control_freq_hz;
        float pidPeriod = settings->pid_sample_time;
        if (controlPeriod > pidPeriod) 
        {
            Serial.printf("WARNING: Control period (%.1f ms) > PID period (%.1f ms) - PID limited to %.0f Hz\n", 
                         controlPeriod, pidPeriod, settings->control_freq_hz);
        } 
        else if (controlPeriod < pidPeriod / 2) 
        {
            Serial.printf("INFO: Control freq much faster than PID - good for sensor resolution\n");
        } 
        else 
        {
            Serial.printf("OK: Control/PID timing balanced\n");
        }
        settings->save();
    }
    else if (cmd == "RESET") 
    {
        // Reset PID controller
        Serial.println("Resetting PID controller...");
        
        pid->SetMode(MANUAL);
        *pwmOutput = 0;
        ledcWrite(HardwareConfig::PWM_CHANNEL_MOSFET, 0);
        
        // Reset SensorManager filter state
        if (sensorManager) 
        {
            sensorManager->resetFilterState();
        }
        
        // Re-initialize PID with current settings
        pid->SetTunings(settings->Kp, settings->Ki, settings->Kd);
        pid->SetOutputLimits(0, *pwmFullScaleRaw);
        pid->SetMode(AUTOMATIC);
        
        // Reset manual mode if it was enabled
        *manualPWMMode = false;
        
        Serial.println("PID controller reset complete");
    }
}

void CommandProcessor::handleSignalProcessingCommands(const String& cmd) 
{
    if (cmd.startsWith("FLT ")) 
    {
        float new_flt = cmd.substring(4).toFloat();
        settings->filter_strength = constrain(new_flt, 0.0, 1.0);
        Serial.printf("Filter strength set to: %.2f\n", settings->filter_strength);
        settings->save();
    }
    else if (cmd == "AW ON") 
    {
        settings->antiWindup = true;
        Serial.println("Anti-windup for deadband enabled");
        settings->save();
    }
    else if (cmd == "AW OFF") 
    {
        settings->antiWindup = false;
        Serial.println("Anti-windup for deadband disabled");
        settings->save();
    }
    else if (cmd == "HYST ON") 
    {
        settings->hysteresis = true;
        Serial.println("Hysteresis compensation enabled");
        settings->save();
    }
    else if (cmd == "HYST OFF") 
    {
        settings->hysteresis = false;
        Serial.println("Hysteresis compensation disabled");
        settings->save();
    }
    else if (cmd.startsWith("HYSTAMT ")) 
    {
        settings->hystAmount = cmd.substring(8).toFloat();
        settings->hystAmount = constrain(settings->hystAmount, ValveConfig::HYST_MIN, ValveConfig::HYST_MAX);
        Serial.printf("Hysteresis compensation amount set to: %.1f%%\n", settings->hystAmount);
        settings->save();
    }
}

void CommandProcessor::handlePWMCommands(const String& cmd) 
{
    if (cmd.startsWith("FREQ ")) 
    {
        int new_freq = cmd.substring(5).toInt();
        settings->pwm_freq = constrain(new_freq, PwmConfig::MIN_FREQ_HZ, PwmConfig::MAX_FREQ_HZ);
        updatePWM();
        Serial.printf("PWM frequency updated to: %d Hz\n", settings->pwm_freq);
        settings->save();
    }
    else if (cmd.startsWith("RES ")) 
    {
        int new_res = cmd.substring(4).toInt();
        // Store current duty cycle percentage before changing resolution
        float current_duty_percent = (*pwmOutput / (float)*pwmFullScaleRaw) * 100.0;
        
        // Update resolution and max value
        settings->pwm_res = constrain(new_res, PwmConfig::MIN_RES_BITS, PwmConfig::MAX_RES_BITS);
        int new_max_value = (1 << settings->pwm_res) - 1;
        
        // Scale pwmOutput to maintain the same duty cycle
        *pwmOutput = (current_duty_percent / 100.0) * new_max_value;
        
        // Update max value and PID limits
        *pwmFullScaleRaw = new_max_value;
        pid->SetOutputLimits(0, *pwmFullScaleRaw);
        updatePWM();
        
        Serial.printf("PWM resolution updated to: %d bits (max: %d)\n", settings->pwm_res, *pwmFullScaleRaw);
        Serial.printf("Duty cycle maintained at: %.1f%%\n", current_duty_percent);
        settings->save();
    }
    else if (cmd.startsWith("PWM ")) 
    {
        // Force PWM duty cycle for testing (overrides PID)
        float duty_percent = cmd.substring(4).toFloat();
        duty_percent = constrain(duty_percent, 0.0, 100.0);
        *pwmOutput = (duty_percent / 100.0) * *pwmFullScaleRaw;
        
        // Apply PWM value directly (without mapping for manual control)
        ledcWrite(HardwareConfig::PWM_CHANNEL_MOSFET, (uint32_t)*pwmOutput);
        
        Serial.printf("PWM manually set to: %.1f%% (%d/%d)\n", 
                     duty_percent, (int)*pwmOutput, *pwmFullScaleRaw);
                     
        // Set manual mode flag
        *manualPWMMode = true;
        Serial.println("WARNING: Manual PWM control active - PID control suspended");
        Serial.println("         Use RESUME command to return to PID control");
    }
    else if (cmd == "RESUME") 
    {
        // Resume PID control
        *manualPWMMode = false;
        Serial.println("PID control resumed");
    }
    else if (cmd.startsWith("CONTROL FREQ ")) 
    {
        int new_freq = cmd.substring(13).toInt();
        settings->control_freq_hz = constrain(new_freq, ControlConfig::MIN_FREQ_HZ, ControlConfig::MAX_FREQ_HZ);
        Serial.printf("Control loop frequency updated to: %d Hz (period: %.1f ms)\n", 
                     settings->control_freq_hz, 1000.0/settings->control_freq_hz);
        
        // Check timing relationship and provide feedback
        float controlPeriod = 1000.0 / settings->control_freq_hz;
        float pidPeriod = settings->pid_sample_time;
        if (controlPeriod > pidPeriod) 
        {
            Serial.printf("WARNING: Control period (%.1f ms) > PID period (%.1f ms) - PID limited to %.0f Hz\n", 
                         controlPeriod, pidPeriod, settings->control_freq_hz);
        } 
        else if (controlPeriod < pidPeriod / 2) 
        {
            Serial.printf("INFO: Control freq much faster than PID - good for sensor resolution\n");
        } 
        else 
        {
            Serial.printf("OK: Control/PID timing balanced\n");
        }
        settings->save();
    }
}

void CommandProcessor::handleAutoTuningCommands(const String& cmd) 
{
    if (cmd == "TUNE START") 
    {
        if (autoTuner) 
        {
            if (!autoTuner->isRunning()) 
            {
                // Switch to manual mode for auto-tuning
                *manualPWMMode = true;
                autoTuner->start();
            } 
            else 
            {
                Serial.println("Auto-tuning already running!");
            }
        } 
        else 
        {
            Serial.println("ERROR: AutoTuner not initialized");
        }
    }
    else if (cmd == "TUNE STOP" || cmd == "TUNE CANCEL") 
    {
        if (autoTuner) 
        {
            if (autoTuner->isRunning()) 
            {
                Serial.println("Auto-tuning cancelled by user.");
                autoTuner->stop(false);
                // Return to PID control
                *manualPWMMode = false;
            } 
            else 
            {
                Serial.println("No auto-tuning process is running.");
            }
        } 
        else 
        {
            Serial.println("ERROR: AutoTuner not initialized");
        }
    }
    else if (cmd == "TUNE ACCEPT") 
    {
        if (autoTuner) 
        {
            autoTuner->acceptParameters();
        } 
        else 
        {
            Serial.println("ERROR: AutoTuner not initialized");
        }
    }
    else if (cmd == "TUNE REJECT") 
    {
        if (autoTuner) 
        {
            autoTuner->rejectParameters();
        } 
        else 
        {
            Serial.println("ERROR: AutoTuner not initialized");
        }
    }
    else if (cmd.startsWith("TUNE SP ")) 
    {
        float newSetpoint = cmd.substring(8).toFloat();
        if (autoTuner) 
        {
            autoTuner->setTestSetpoint(newSetpoint);
        } 
        else 
        {
            Serial.println("ERROR: AutoTuner not initialized");
        }
    }
    else if (cmd.startsWith("TUNE MIN ")) 
    {
        float newMin = cmd.substring(9).toFloat();
        float currentMax = autoTuner ? autoTuner->getMaxPWM() : 85.0f;
        if (autoTuner) 
        {
            autoTuner->setMinMaxPWM(newMin, currentMax);
        } 
        else 
        {
            Serial.println("ERROR: AutoTuner not initialized");
        }
    }
    else if (cmd.startsWith("TUNE MAX ")) 
    {
        float newMax = cmd.substring(9).toFloat();
        float currentMin = autoTuner ? autoTuner->getMinPWM() : 65.0f;
        if (autoTuner) 
        {
            autoTuner->setMinMaxPWM(currentMin, newMax);
        } 
        else 
        {
            Serial.println("ERROR: AutoTuner not initialized");
        }
    }
    else if (cmd.startsWith("TUNE CYCLE ")) 
    {
        unsigned long newCycleTime = cmd.substring(11).toInt();
        if (autoTuner) 
        {
            autoTuner->setMinCycleTime(newCycleTime);
        } 
        else 
        {
            Serial.println("ERROR: AutoTuner not initialized");
        }
    }
    else if (cmd.startsWith("TUNE RULE ")) 
    {
        int rule = cmd.substring(10).toInt();
        if (autoTuner) 
        {
            if (rule >= 0 && rule <= 3) 
            {
                autoTuner->setTuningRule((TuningRule)rule);
            } 
            else 
            {
                Serial.println("Invalid rule number. Valid options: 0-3");
            }
        } 
        else 
        {
            Serial.println("ERROR: AutoTuner not initialized");
        }
    }
    else if (cmd.startsWith("TUNE AGGR ")) 
    {
        float aggr = cmd.substring(10).toFloat();
        if (autoTuner) 
        {
            autoTuner->setAggressiveness(aggr);
        } 
        else 
        {
            Serial.println("ERROR: AutoTuner not initialized");
        }
    }
    else if (cmd == "TUNE RULES") 
    {
        if (autoTuner) 
        {
            autoTuner->printTuningRules();
            autoTuner->printConfiguration();
        } 
        else 
        {
            Serial.println("ERROR: AutoTuner not initialized");
        }
    }
    else if (cmd == "AUTOTUNE") 
    {
        // Test AutoTuner functionality  
        if (autoTuner) 
        {
            Serial.println("\n=== AutoTuner Test ===");
            autoTuner->printConfiguration();
            Serial.printf("Currently running: %s\n", autoTuner->isRunning() ? "Yes" : "No");
            if (autoTuner->isRunning()) 
            {
                Serial.printf("Output value: %.1f%%\n", autoTuner->getOutputValue());
            }
        } 
        else 
        {
            Serial.println("ERROR: AutoTuner not initialized");
        }
    }
}

void CommandProcessor::handleNetworkCommands(const String& cmd) 
{
    if (cmd == "SCAN WIFI") 
    {
        // Call the WiFi scanning function from WebHandler
        if (webHandler) 
        {
            webHandler->scanWiFiNetworks();
        } 
        else 
        {
            Serial.println("Error: WebHandler not initialized");
        }
    }
    else if (cmd.startsWith("WIFI CHANNEL ")) 
    {
        // Change WiFi AP channel
        int channel = cmd.substring(13).toInt();
        if (webHandler) 
        {
            webHandler->changeWiFiChannel(channel);
        } 
        else 
        {
            Serial.println("Error: WebHandler not initialized");
        }
    }
    else if (cmd == "PAGE ON") 
    {
        if (webHandler) 
        {
            webHandler->setWebServerEnabled(true);
            Serial.println("Web server processing enabled");
        } 
        else 
        {
            Serial.println("Error: WebHandler not initialized");
        }
    }
    else if (cmd == "PAGE OFF") 
    {
        if (webHandler) 
        {
            webHandler->setWebServerEnabled(false);
            Serial.println("Web server processing disabled");
        } 
        else 
        {
            Serial.println("Error: WebHandler not initialized");
        }
    }
}

void CommandProcessor::handleDiagnosticCommands(const String& cmd) 
{
    if (cmd == "STATUS") 
    {
        showStatus();
    }
    else if (cmd == "MEM") 
    {
        showMemoryInfo();
    }
    else if (cmd == "VER") 
    {
        // Display firmware version and build timestamp
        Serial.printf("Firmware Version: %s\n", getVersionString());
    }
    else if (cmd == "STARTCD") 
    {
        Serial.println("Starting output for Continuous Data");
        *continousValueOutput = true;
        Logger::setEnabled(true);  // Sync Logger with continuous output flag
    }
    else if (cmd == "STOPCD") 
    {
        Serial.println("Stopping output for Continuous Data");
        *continousValueOutput = false;
        Logger::setEnabled(false);  // Sync Logger with continuous output flag
    }
    else if (cmd == "SENSOR") 
    {
        // Test SensorManager functionality
        if (sensorManager) 
        {
            Serial.println("\n=== SensorManager Test ===");
            sensorManager->printSensorStatus();
            
            // Take a fresh reading
            sensorManager->readSensor();
            
            Serial.println("\nCurrent Readings:");
            Serial.printf("  ADC Value: %d\n", sensorManager->getADCValue());
            Serial.printf("  Voltage: %.3f V\n", sensorManager->getVoltage());
            Serial.printf("  Raw Pressure: %.5f bar\n", sensorManager->getRawPressure());
            Serial.printf("  Filtered Pressure: %.5f bar\n", sensorManager->getPressure());
            Serial.printf("  Pressure Safe: %s\n", sensorManager->isPressureSafe() ? "Yes" : "NO - EMERGENCY!");
            Serial.printf("  Filter Strength: %.2f\n", settings->filter_strength);
        } 
        else 
        {
            Serial.println("ERROR: SensorManager not initialized");
        }
    }
}

void CommandProcessor::handleFileSystemCommands(const String& cmd) 
{
    if (cmd == "READ") 
    {
        settings->printStoredSettings();
    }
    else if (cmd == "DIR") 
    {
        listFiles();
    }
    else if (cmd == "SAVE") 
    {
        settings->save();
        Serial.println("Settings saved to persistent storage");
    }
}

void CommandProcessor::updatePWM() 
{
    ledcSetup(HardwareConfig::PWM_CHANNEL_MOSFET, settings->pwm_freq, settings->pwm_res);
    ledcWrite(HardwareConfig::PWM_CHANNEL_MOSFET, *pwmOutput);
}

void CommandProcessor::showHelp() 
{
    Serial.println(
      "\n╔═══════════════════════════════════════════════════════════════════════════════╗"
      "\n║                          VENTCON2 COMMAND REFERENCE                           ║"
      "\n║                        All commands are case-insensitive                      ║"
      "\n╚═══════════════════════════════════════════════════════════════════════════════╝"
      "\n"
      "\n┌─ PID CONTROL ─────────────────────────────────────────────────────────────────┐"
      "\n│ KP <value>      │ Set proportional gain (e.g., KP 0.5)                        │"
      "\n│ KI <value>      │ Set integral gain (e.g., KI 0.1)                            │"
      "\n│ KD <value>      │ Set derivative gain (e.g., KD 0.01)                         │"
      "\n│ SP <value>      │ Set pressure setpoint in bar (e.g., SP 3.0)                 │"
      "\n│ SAMPLE <ms>     │ Set PID sample time, 1-1000ms (e.g., SAMPLE 10)             │"
      "\n│ RESET           │ Reset PID controller (clear windup & state)                 │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"
      "\n┌─ SIGNAL PROCESSING ───────────────────────────────────────────────────────────┐"
      "\n│ FLT <value>     │ Set filter strength, 0.0-1.0 (e.g., FLT 0.2)                │"
      "\n│ AW ON/OFF       │ Enable/disable anti-windup for deadband                     │"
      "\n│ HYST ON/OFF     │ Enable/disable hysteresis compensation                      │"
      "\n│ HYSTAMT <value> │ Set hysteresis amount in % (e.g., HYSTAMT 5)                │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"
      "\n┌─ PWM & VALVE CONTROL ─────────────────────────────────────────────────────────┐"
      "\n│ FREQ <hz>       │ Set PWM frequency, 100-10000Hz (e.g., FREQ 1000)            │"
      "\n│ RES <bits>      │ Set PWM resolution, 1-16 bits (e.g., RES 8)                 │"
      "\n│ PWM <percent>   │ Force PWM duty cycle, 0-100% (e.g., PWM 25)                 │"
      "\n│ RESUME          │ Resume normal PID control after manual PWM                  │"
      "\n│ CONTROL FREQ <> │ Set control loop frequency, 10-1000Hz                       │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"      "\n┌─ AUTO-TUNING ─────────────────────────────────────────────────────────────────┐"
      "\n│ AUTOTUNE        │ Test AutoTuner and show detailed configuration          │"
      "\n│ TUNE START      │ Start PID auto-tuning process                               │"
      "\n│ TUNE STOP       │ Cancel auto-tuning process                                  │"
      "\n│ TUNE ACCEPT     │ Accept calculated PID parameters                            │"
      "\n│ TUNE REJECT     │ Reject and keep current PID parameters                      │"
      "\n│ TUNE SP <bar>   │ Set auto-tune test setpoint, 0.5-10.0 bar                   │"
      "\n│ TUNE MIN <pct>  │ Set auto-tune minimum PWM, 50-90%                           │"
      "\n│ TUNE MAX <pct>  │ Set auto-tune maximum PWM, 60-95%                           │"
      "\n│ TUNE CYCLE <ms> │ Set min cycle time, 50-2000ms                               │"
      "\n│ TUNE RULE <0-3> │ Select tuning rule (see TUNE RULES)                         │"
      "\n│ TUNE AGGR <val> │ Set aggressiveness factor, 0.5-2.0                          │"
      "\n│ TUNE RULES      │ Show available tuning rules with descriptions               │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"
      "\n┌─ NETWORK & WIFI ──────────────────────────────────────────────────────────────┐"
      "\n│ SCAN WIFI       │ Scan and list WiFi networks with signal strength            │"
      "\n│ WIFI CHANNEL <> │ Set WiFi AP channel, 1-13 (interference avoidance)          │"
      "\n│ PAGE ON/OFF     │ Enable/disable web server processing                        │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"      "\n┌─ DATA & MONITORING ───────────────────────────────────────────────────────────┐"
      "\n│ STATUS          │ Show comprehensive system status                            │"
      "\n│ SENSOR          │ Test SensorManager and show detailed sensor readings       │"
      "\n│ STARTCD         │ Start continuous data output for plotting                   │"
      "\n│ STOPCD          │ Stop continuous data output                                 │"
      "\n│ MEM             │ Show memory usage and system information                    │"
      "\n│ VER             │ Display firmware version and build info                     │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"
      "\n┌─ FILE SYSTEM & SETTINGS ──────────────────────────────────────────────────────┐"
      "\n│ SAVE            │ Force save current settings to flash memory                 │"
      "\n│ READ            │ Read and display settings stored in flash                   │"
      "\n│ DIR             │ List all files in flash memory with sizes                   │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘"
      "\n"
      "\n┌─ QUICK REFERENCE ─────────────────────────────────────────────────────────────┐"
      "\n│ HELP            │ Show this command reference                                 │"
      "\n│                 │                                                             │"
      "\n│ Example workflow:                                                             │"
      "\n│   STATUS        → Check current system state                                  │"
      "\n│   TUNE START    → Begin auto-tuning for optimal PID                           │"
      "\n│   TUNE ACCEPT   → Accept calculated parameters                                │"
      "\n│   STARTCD       → Monitor real-time data                                      │"
      "\n│   SAVE          → Persist settings to flash                                   │"
      "\n└───────────────────────────────────────────────────────────────────────────────┘");
}

void CommandProcessor::showStatus() 
{
    // Break up status into multiple smaller messages instead of one large buffer
    Serial.println("\n=== System Status ===");
    
    // Pressure Control section
    Serial.println("Pressure Control:");
    Serial.printf("  Current Pressure: %.2f bar\n", *pressureInput);
    Serial.printf("  Setpoint: %.2f bar\n", settings->setpoint);
    Serial.printf("  Control Loop Freq: %d Hz (period: %.1f ms)\n", 
                 settings->control_freq_hz, 1000.0/settings->control_freq_hz);
    if (autoTuner)
    {
        Serial.printf("  Auto-Tune Test Setpoint: %.2f bar\n", autoTuner->getTestSetpoint());
        Serial.printf("  Auto-Tune PWM Range: %.1f%% - %.1f%% (amplitude: %.1f%%)\n", 
                     autoTuner->getMinPWM(), autoTuner->getMaxPWM(), autoTuner->getEffectiveAmplitude());
        Serial.printf("  Auto-Tune Min Cycle Time: %lu ms\n", autoTuner->getMinCycleTime());
    }
    
    // Sensor Information section
    Serial.println("\nSensor Information:");
    if (sensorManager)
    {
        Serial.printf("  ADC Source: %s\n", sensorManager->isADSFound() ? "ADS1015" : "ESP32 Internal");
        if (sensorManager->isADSFound())
        {
            Serial.printf("  ADS1015 Address: 0x48\n");
            Serial.printf("  ADS1015 Channel: %d\n", SensorConfig::ADC_CHANNEL_PRESS_SENS);
            Serial.printf("  Gain Setting: GAIN_TWOTHIRDS (+/-6.144V)\n");
        }
        else
        {
            Serial.printf("  Fallback Pin: %d\n", HardwareConfig::FALLBACK_ANALOG_PIN);
        }
        Serial.printf("  Raw ADC Value: %d\n", sensorManager->getADCValue());
        Serial.printf("  Voltage: %.3f V\n", sensorManager->getVoltage());
        Serial.printf("  Raw Pressure: %.3f bar\n", sensorManager->getRawPressure());
        Serial.printf("  Filtered Pressure: %.3f bar\n", sensorManager->getPressure());
        Serial.printf("  Voltage Range: %.1fV - %.1fV\n", SensorConfig::MIN_VOLTAGE, SensorConfig::MAX_VOLTAGE);
        Serial.printf("  Pressure Range: %.1f - %.1f bar\n", SensorConfig::SENSOR_MIN_BAR, SensorConfig::SENSOR_MAX_BAR);
    } else {
        Serial.println("  ERROR: SensorManager not initialized");
    }
    
    // PWM Output section
    Serial.println("\nPWM Output:");
    Serial.printf("  Value: %d/%d (%.3f%%)\n", 
                 (int)*pwmOutput, *pwmFullScaleRaw, 
                 (*pwmOutput / float(*pwmFullScaleRaw)) * 100.0);
    Serial.printf("  Resolution: %d-bit\n", settings->pwm_res);
    Serial.printf("  Frequency: %d Hz\n", settings->pwm_freq);
    Serial.printf("  Control Mode: %s\n", *manualPWMMode ? "MANUAL" : "PID");
    Serial.printf("  Solenoid Pin: %d\n", HardwareConfig::SOLENOID_PIN);
    
    // PID Configuration section
    Serial.println("\nPID Configuration:");
    Serial.printf("  Kp=%.2f, Ki=%.2f, Kd=%.2f\n", 
                 settings->Kp, settings->Ki, settings->Kd);
    Serial.printf("  Sample Time: %d ms (%.1f Hz)\n", settings->pid_sample_time, 1000.0/settings->pid_sample_time);
    Serial.printf("  Filter Strength: %.2f\n", settings->filter_strength);
    Serial.printf("  Anti-Windup: %s\n", settings->antiWindup ? "Enabled" : "Disabled");
    Serial.printf("  Hysteresis Comp: %s (%.1f%%)\n", 
                 settings->hysteresis ? "Enabled" : "Disabled", settings->hystAmount);
    
    // Network section
    Serial.println("\nNetwork:");
    Serial.printf("  SSID: %s\n", webHandler ? webHandler->getAPSSID() : "Not Initialized");
    Serial.printf("  IP: %s\n", webHandler ? webHandler->getAPIP().toString().c_str() : "Not Initialized");
    Serial.printf("  Channel: %d (%.1f MHz)\n", webHandler ? webHandler->getWiFiChannel() : 0, 
                 webHandler ? NetworkConfig::WIFI_CH1_FREQ_MHZ + (webHandler->getWiFiChannel() - 1) * NetworkConfig::WIFI_CHANNEL_STEP_MHZ : 0.0);
    Serial.printf("  Connected Clients: %d/%d\n", webHandler ? webHandler->getConnectedClients() : 0, NetworkConfig::MAX_CLIENTS);
    Serial.printf("  Web Server: %s\n", webHandler ? (webHandler->isWebServerEnabled() ? "Enabled" : "Disabled") : "Not Initialized");
}

void CommandProcessor::showMemoryInfo() 
{
    Serial.println("\n=== Memory Information ===");
    
    // Heap memory information
    size_t freeHeap = ESP.getFreeHeap();
    size_t totalHeap = ESP.getHeapSize();
    size_t usedHeap = totalHeap - freeHeap;
    size_t minFreeHeap = ESP.getMinFreeHeap();
    size_t maxAllocHeap = ESP.getMaxAllocHeap();
    
    Serial.printf("Heap Memory:\n");
    Serial.printf("  Total: %u bytes (%.1f KB)\n", totalHeap, totalHeap / 1024.0);
    Serial.printf("  Used:  %u bytes (%.1f KB, %.1f%%)\n", 
                 usedHeap, usedHeap / 1024.0, (usedHeap * 100.0) / totalHeap);
    Serial.printf("  Free:  %u bytes (%.1f KB, %.1f%%)\n", 
                 freeHeap, freeHeap / 1024.0, (freeHeap * 100.0) / totalHeap);
    Serial.printf("  Min Free: %u bytes (%.1f KB)\n", minFreeHeap, minFreeHeap / 1024.0);
    Serial.printf("  Max Alloc: %u bytes (%.1f KB)\n", maxAllocHeap, maxAllocHeap / 1024.0);
    
    // Flash memory information (LittleFS)
    size_t totalFS = LittleFS.totalBytes();
    size_t usedFS = LittleFS.usedBytes();
    size_t freeFS = totalFS - usedFS;
    
    Serial.printf("\nFlash Storage (LittleFS):\n");
    Serial.printf("  Total: %u bytes (%.1f KB)\n", totalFS, totalFS / 1024.0);
    Serial.printf("  Used:  %u bytes (%.1f KB, %.1f%%)\n", 
                 usedFS, usedFS / 1024.0, (usedFS * 100.0) / totalFS);
    Serial.printf("  Free:  %u bytes (%.1f KB, %.1f%%)\n", 
                 freeFS, freeFS / 1024.0, (freeFS * 100.0) / totalFS);
      // Task information
    Serial.printf("\nTask Information:\n");
    if (taskManager) 
    {
        taskManager->printTaskInfo();
    } 
    else 
    {
        Serial.printf("  TaskManager: Not initialized\n");
    }
    
    // Additional system info
    Serial.printf("\nSystem:\n");
    Serial.printf("  Flash Size: %u MB\n", ESP.getFlashChipSize() / (1024 * 1024));
    Serial.printf("  Chip Model: %s\n", ESP.getChipModel());
    Serial.printf("  CPU Freq: %u MHz\n", ESP.getCpuFreqMHz());
    Serial.printf("  SDK Version: %s\n", ESP.getSdkVersion());
}

void CommandProcessor::listFiles() 
{
    Serial.println("\n=== Files in Flash Memory ===");
    
    File root = LittleFS.open("/");
    if (!root) 
    {
        Serial.println("Failed to open directory");
        return;
    }
    
    if (!root.isDirectory()) 
    {
        Serial.println("Not a directory");
        return;
    }
    
    int fileCount = 0;
    size_t totalSize = 0;
    
    File file = root.openNextFile();
    while (file) 
    {
        if (!file.isDirectory()) 
        {
            // Print file name and size in KB
            Serial.printf("%-20s %8.2f KB\n", file.name(), file.size() / 1024.0);
            fileCount++;
            totalSize += file.size();
        }
        file = root.openNextFile();
    }
    
    Serial.println("------------------------------");
    Serial.printf("Total: %d files, %.2f KB\n", fileCount, totalSize / 1024.0);
    Serial.printf("Flash usage: %.1f%% (of %.2f KB)\n", 
                  (totalSize * 100.0) / (LittleFS.totalBytes()), 
                  LittleFS.totalBytes() / 1024.0);
    Serial.printf("Free space: %.2f KB\n", (LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024.0);
}

const char* CommandProcessor::getVersionString() 
{
    static char versionString[50];
    sprintf(versionString, VENTCON_VERSION);
    return versionString;
}

void CommandProcessor::processSerialInput() 
{
    static String serialBuffer;
    
    while (Serial.available()) 
    {
        char c = Serial.read();
        if (c == '\n') 
        { // newline
            processCommand(serialBuffer);
            serialBuffer = "";
        }
        else if (c != '\r') 
        { // Ignore carriage return
            serialBuffer += c; // Append character to buffer
        }
    }
}
