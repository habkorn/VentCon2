#include "AutoTuner.h"
#include "Logger.h"
#include <Arduino.h>
#include <cmath> // for M_PI

AutoTuner::AutoTuner(SettingsHandler* settings, PID* pid, double* pressureInput, int* pwmMaxValue)
    : autoTuneRunning(false),
      autoTuneStartTime(0),
      lastTransitionTime(0),
      autoTuneOutputValue(0.0f),
      autoTuneSetpoint(0.0f),
      autoTuneState(false),
      currentCycle(0),
      maxPressure(0.0f),
      minPressure(AutoTuneConfig::INITIAL_MIN_PRESSURE),
      firstCycleComplete(false),
      testSetpoint(AutoTuneConfig::DEFAULT_TEST_SETPOINT),
      minPwmValue(AutoTuneConfig::MIN_PWM_PERCENT),
      maxPwmValue(AutoTuneConfig::MAX_PWM_PERCENT),
      minCycleTime(AutoTuneConfig::MIN_CYCLE_TIME_MS),
      currentTuningRule(ZIEGLER_NICHOLS_AGGRESSIVE),
      tuningAggressiveness(AutoTuneConfig::DEFAULT_AGGRESSIVENESS),
      settings(settings),
      pid(pid),
      pressureInput(pressureInput),
      pwmMaxValue(pwmMaxValue)
{
    
    // Initialize cycle arrays
    for (int i = 0; i < MAX_CYCLES; i++) 
    {
        cycleTimes[i] = 0;
        cycleAmplitudes[i] = 0.0f;
    }
}

void AutoTuner::start() 
{
    if (autoTuneRunning) 
    {
        Serial.println("Auto-tuning already running!");
        return;
    }
    
    // Save current setpoint and install test setpoint atomically.
    settings->update([this](SettingsHandler& s){ autoTuneSetpoint = s.setpoint; s.setpoint = testSetpoint; });
    
    // Initialize auto-tune variables
    autoTuneRunning = true;
    autoTuneStartTime = millis();
    lastTransitionTime = 0;
    autoTuneState = false;  // Start with relay off
    currentCycle = 0;
    
    // Initialize amplitude tracking
    maxPressure = *pressureInput;
    minPressure = *pressureInput;
    firstCycleComplete = false;
    
    // Set initial output state - Use high relay value for clarity in reporting
    autoTuneOutputValue = maxPwmValue;
    
    // Apply initial output
    applyPWMOutput(autoTuneOutputValue);
    
    Serial.println("\n=== PID Auto-Tuning Started ===");
    Serial.printf("Target Setpoint: %.2f bar\n", testSetpoint);
    Serial.printf("Relay Output: %.1f%% - %.1f%% (valve effective range)\n", minPwmValue, maxPwmValue);
    Serial.printf("Valve operating range: %.1f%% - %.1f%%\n", ValveConfig::VALVE_MIN_DUTY, ValveConfig::VALVE_MAX_DUTY);
    Serial.printf("Effective relay amplitude: %.1f%%\n", getEffectiveAmplitude());
    Serial.printf("Tuning Rule: %s\n", getTuningRuleName());
    Serial.printf("Aggressiveness: %.1f\n", tuningAggressiveness);
    Serial.println("Auto-tuning will run for up to 3 minutes or 20 complete cycles");
    Serial.println("Keep system stable and avoid disturbances during tuning");
}

void AutoTuner::stop(bool calculateParameters) 
{
    if (!autoTuneRunning) 
    {
        Serial.println("No auto-tuning process is running.");
        return;
    }
    
    if (calculateParameters && currentCycle >= 3) 
    { // Need at least 3 cycles for valid data
        float newKp, newKi, newKd;
        if (calculatePIDParameters(newKp, newKi, newKd)) 
        {
            Serial.println("\n=== Auto-Tuning Results ===");
            Serial.printf("Cycles Collected: %d\n", currentCycle);
            Serial.printf("Tuning Rule: %s\n", getTuningRuleName());
            Serial.printf("Aggressiveness Factor: %.1f\n", tuningAggressiveness);
            
            Serial.println("\nCalculated PID Parameters:");
            // Read old values then write new ones in one atomic update.
            double oldKp, oldKi, oldKd;
            settings->update([&](SettingsHandler& s) {
                oldKp = s.Kp; oldKi = s.Ki; oldKd = s.Kd;
                s.Kp = newKp;  s.Ki = newKi;  s.Kd = newKd;
            });
            Serial.printf("Kp: %.2f (previously %.2f)\n", newKp, oldKp);
            Serial.printf("Ki: %.2f (previously %.2f)\n", newKi, oldKi);
            Serial.printf("Kd: %.2f (previously %.2f)\n", newKd, oldKd);
        } 
        else 
        {
            Serial.println("\n=== Auto-Tuning Failed ===");
            Serial.println("Could not calculate reliable PID parameters from collected data.");
        }
    } 
    else 
    {
        Serial.println("\n=== Auto-Tuning Cancelled ===");
        Serial.println("Not enough cycles collected for reliable tuning.");
    }
    
    // Restore previous setpoint.
    settings->update([this](SettingsHandler& s){ s.setpoint = autoTuneSetpoint; });
    
    // Reset state
    autoTuneRunning = false;
    
    // Reset PID controller — force Manual→Automatic transition so
    // Initialize() runs and clears stale outputSum from tuning.
    double kp, ki, kd;
    settings->update([&](SettingsHandler& s){ kp = s.Kp; ki = s.Ki; kd = s.Kd; });
    pid->SetMode(PID::Manual);
    pid->SetTunings(kp, ki, kd);
    pid->SetMode(PID::Automatic);
}

void AutoTuner::process() 
{
    if (!autoTuneRunning) 
    {
        return;
    }
    
    // Check if timeout occurred
    if (millis() - autoTuneStartTime > AUTO_TUNE_TIMEOUT) 
    {
        Serial.println("Auto-tuning timeout reached!");
        stop(true); // Calculate with available data
        return;
    }
    
    // Check if we've collected enough cycles
    if (currentCycle >= MAX_CYCLES) 
    {
        Serial.println("Auto-tuning complete!");
        stop(true);
        return;
    }
    
    // setpoint is a double (64-bit); read it atomically before comparing.
    double sp;
    settings->withLock([&]{ sp = settings->setpoint; });

    // Relay oscillation logic
    if (!autoTuneState && *pressureInput > sp) 
    {
        // Transition from high to low
        unsigned long now = millis();
        
        // Debounce transitions to prevent noise
        if (now - lastTransitionTime > minCycleTime) 
        {
            // Record cycle data if this isn't the first transition
            if (lastTransitionTime > 0) 
            {
                cycleTimes[currentCycle] = now - lastTransitionTime;
                cycleAmplitudes[currentCycle] = maxPressure - minPressure;
                currentCycle++;
                
                LOG_D(CAT_AUTOTUNE, "Cycle %d: Period=%.2fs, Amplitude=%.2f bar (Max=%.2f, Min=%.2f)", 
                             currentCycle, (now - lastTransitionTime)/1000.0, 
                             cycleAmplitudes[currentCycle-1], maxPressure, minPressure);
            }
            
            lastTransitionTime = now;
            autoTuneState = true;
            autoTuneOutputValue = minPwmValue; // Set output to low effective range
            
            // Reset amplitude tracking for next cycle
            maxPressure = *pressureInput;
            minPressure = *pressureInput;
            
            applyPWMOutput(autoTuneOutputValue);
        }
    } 
    else if (autoTuneState && *pressureInput < sp - AUTOTUNE_NOISE_BAND) 
    {
        // Transition from low to high
        unsigned long now = millis();
        
        // Debounce transitions to prevent noise
        if (now - lastTransitionTime > minCycleTime) 
        {
            lastTransitionTime = now;
            autoTuneState = false;
            autoTuneOutputValue = maxPwmValue; // Set output high
            
            applyPWMOutput(autoTuneOutputValue);
        }
    }
    
    // Track max and min pressures during oscillation
    if (*pressureInput > maxPressure) maxPressure = *pressureInput;
    if (*pressureInput < minPressure) minPressure = *pressureInput;
}

void AutoTuner::acceptParameters() 
{
    Serial.println("New PID parameters accepted!");
    double kp, ki, kd;
    settings->update([&](SettingsHandler& s){ kp = s.Kp; ki = s.Ki; kd = s.Kd; });
    settings->save();
    pid->SetTunings(kp, ki, kd);
}

void AutoTuner::rejectParameters() 
{
    Serial.println("New PID parameters rejected. Reverting to previous values.");
    settings->load();
    double kp, ki, kd;
    settings->update([&](SettingsHandler& s){ kp = s.Kp; ki = s.Ki; kd = s.Kd; });
    pid->SetTunings(kp, ki, kd);
}

void AutoTuner::setTestSetpoint(float setpoint) 
{
    testSetpoint = constrain(setpoint, AutoTuneConfig::MIN_TEST_SETPOINT, AutoTuneConfig::MAX_TEST_SETPOINT);
    Serial.printf("Auto-tuning test setpoint set to: %.1f bar\n", testSetpoint);
}

void AutoTuner::setTuningRule(TuningRule rule) 
{
    currentTuningRule = rule;
    Serial.println("\n=== Auto-Tuning Rule Updated ===");
    Serial.printf("Selected Rule: %s\n", getTuningRuleName());
}

void AutoTuner::setAggressiveness(float aggr) 
{
    tuningAggressiveness = constrain(aggr, AutoTuneConfig::MIN_AGGRESSIVENESS, AutoTuneConfig::MAX_AGGRESSIVENESS);
    Serial.printf("Tuning aggressiveness set to: %.1f\n", tuningAggressiveness);
}

void AutoTuner::setMinMaxPWM(float minPwm, float maxPwm) 
{
    minPwmValue = constrain(minPwm, ValveConfig::VALVE_MIN_DUTY, maxPwm);
    maxPwmValue = constrain(maxPwm, minPwmValue, ValveConfig::VALVE_MAX_DUTY);
    Serial.printf("Auto-tuning PWM range set to: %.1f%% - %.1f%%\n", minPwmValue, maxPwmValue);
    Serial.printf("Effective amplitude: %.1f%%\n", getEffectiveAmplitude());
}

void AutoTuner::setMinCycleTime(unsigned long cycleTime) 
{
    minCycleTime = constrain(cycleTime, AutoTuneConfig::CYCLE_TIME_LOWER_BOUND, AutoTuneConfig::CYCLE_TIME_UPPER_BOUND);
    Serial.printf("Auto-tuning minimum cycle time set to: %lu ms\n", minCycleTime);
    Serial.printf("This prevents noise-triggered transitions during auto-tuning\n");
}

void AutoTuner::printTuningRules() const 
{
    Serial.println("\n=== Available Auto-Tuning Rules ===");
    Serial.println("0 - Ziegler-Nichols Classic: Balanced response");
    Serial.println("1 - Ziegler-Nichols Aggressive: Faster response with more overshoot");
    Serial.println("2 - Tyreus-Luyben: Less overshoot, slower recovery");
    Serial.println("3 - Pessen Integral: Fast setpoint tracking");
    Serial.printf("\nCurrent Rule: %d (%s)\n", currentTuningRule, getTuningRuleName());
    Serial.printf("Aggressiveness: %.1f\n", tuningAggressiveness);
}

void AutoTuner::printConfiguration() const 
{
    Serial.println("\n=== Auto-Tuning Configuration ===");
    Serial.printf("Test Setpoint: %.1f bar\n", testSetpoint);
    Serial.printf("PWM Range: %.1f%% - %.1f%% (amplitude: %.1f%%)\n", 
                 minPwmValue, maxPwmValue, getEffectiveAmplitude());
    Serial.printf("Min Cycle Time: %lu ms\n", minCycleTime);
    Serial.printf("Tuning Rule: %s\n", getTuningRuleName());
    Serial.printf("Aggressiveness: %.1f\n", tuningAggressiveness);
    Serial.printf("Timeout: %lu seconds\n", AUTO_TUNE_TIMEOUT / 1000);
    Serial.printf("Max Cycles: %d\n", MAX_CYCLES);
}

bool AutoTuner::calculatePIDParameters(float& newKp, float& newKi, float& newKd) 
{
    if (currentCycle < 3) 
    {
        return false; // Need at least 3 cycles
    }
    
    // Calculate average period and amplitude
    unsigned long totalTime = 0;
    float totalAmplitude = 0.0f;
    
    for (int i = 0; i < currentCycle; i++) 
    {
        totalTime += cycleTimes[i];
        totalAmplitude += cycleAmplitudes[i];
    }
    
    float avgPeriod = (float)totalTime / currentCycle / 1000.0f; // Convert to seconds
    float avgAmplitude = totalAmplitude / currentCycle;
    
    // Calculate ultimate gain Ku and period Tu
    float effectiveAmplitude = getEffectiveAmplitude();
    float Ku = (4.0f * effectiveAmplitude) / (static_cast<float>(M_PI) * avgAmplitude);
    float Tu = avgPeriod;
    
    // Apply a scaling factor to compensate for limited valve range
    float valveRangeCompensation = 100.0f / (ValveConfig::VALVE_MAX_DUTY - ValveConfig::VALVE_MIN_DUTY);
    
    // Calculate PID parameters based on selected tuning rule
    switch (currentTuningRule) 
    {
        case ZIEGLER_NICHOLS_CLASSIC:
            // Classic Ziegler-Nichols - balanced response, scaled for valve range
            newKp = 1.5f * Ku * valveRangeCompensation;
            newKi = 3.0f * Ku / Tu * valveRangeCompensation;
            newKd = 0.18f * Ku * Tu;
            break;
            
        case ZIEGLER_NICHOLS_AGGRESSIVE:
            // Modified Ziegler-Nichols for faster response, scaled for valve range
            newKp = 2.0f * Ku * tuningAggressiveness * valveRangeCompensation;
            newKi = 4.5f * Ku / Tu * valveRangeCompensation;
            newKd = 0.25f * Ku * Tu;
            break;
            
        case TYREUS_LUYBEN:
            // Tyreus-Luyben - more conservative but still scaled for valve range
            newKp = 1.2f * Ku * valveRangeCompensation;
            newKi = 1.0f * Ku / Tu * valveRangeCompensation;
            newKd = 0.35f * Ku * Tu;
            break;
            
        case PESSEN_INTEGRAL:
            // Pessen Integral Rule - very aggressive for setpoint tracking
            newKp = 2.2f * Ku * valveRangeCompensation;
            newKi = 5.0f * Ku / Tu * valveRangeCompensation;
            newKd = 0.3f * Ku * Tu;
            break;
            
        default:
            return false;
    }
    
    // Print calculation details
    LOG_D(CAT_AUTOTUNE, "Average Period: %.2f seconds", avgPeriod);
    LOG_D(CAT_AUTOTUNE, "Average Amplitude: %.2f bar", avgAmplitude);
    LOG_D(CAT_AUTOTUNE, "Valve Range Compensation: %.2fx", valveRangeCompensation);
    LOG_D(CAT_AUTOTUNE, "Ultimate Gain (Ku): %.2f", Ku);
    LOG_D(CAT_AUTOTUNE, "Ultimate Period (Tu): %.2f s", Tu);
    
    return true;
}

const char* AutoTuner::getTuningRuleName() const 
{
    switch (currentTuningRule) 
    {
        case ZIEGLER_NICHOLS_CLASSIC: return "Ziegler-Nichols Classic";
        case ZIEGLER_NICHOLS_AGGRESSIVE: return "Ziegler-Nichols Aggressive";
        case TYREUS_LUYBEN: return "Tyreus-Luyben";
        case PESSEN_INTEGRAL: return "Pessen Integral";
        default: return "Unknown";
    }
}

void AutoTuner::applyPWMOutput(float pwmPercent) 
{
    // Apply PWM output using the external PWM control
    uint32_t pwmValue = (uint32_t)((pwmPercent / 100.0f) * (*pwmMaxValue));
    ledcWrite(HardwareConfig::PWM_CHANNEL_MOSFET, pwmValue);
}
