#pragma once

#include <PID_v2.h>
#include "Settings.h"
#include "Constants.h"

/**
 * TuningRule enum
 * 
 * Defines different auto-tuning algorithms available for PID parameter calculation
 */
enum TuningRule 
{
    ZIEGLER_NICHOLS_CLASSIC = 0,   // Standard Z-N (balanced response)
    ZIEGLER_NICHOLS_AGGRESSIVE = 1, // More aggressive Z-N for faster response
    TYREUS_LUYBEN = 2,             // More conservative rule with less overshoot
    PESSEN_INTEGRAL = 3            // Aggressive integral action for setpoint tracking
};

/**
 * AutoTuner Class
 * 
 * Implements relay-based auto-tuning using the Ziegler-Nichols frequency response method.
 * This class encapsulates all auto-tuning functionality including:
 * - Relay oscillation control
 * - Cycle detection and measurement
 * - PID parameter calculation using various tuning rules
 * - Safety timeouts and validation
 * 
 * Theory:
 * Instead of increasing gain until oscillation (classic Z-N), this uses a relay (on/off) 
 * controller to force controlled oscillation around the setpoint, which is safer and 
 * more reliable. The system measures oscillation period and amplitude to calculate 
 * optimal PID parameters.
 */
class AutoTuner 
{
private:
    // Auto-tuning state variables
    bool autoTuneRunning;
    unsigned long autoTuneStartTime;
    unsigned long lastTransitionTime;
    float autoTuneOutputValue;
    float autoTuneSetpoint;
    bool autoTuneState;
    int currentCycle;
    
    // Cycle data collection
    static constexpr int MAX_CYCLES = 20;
    unsigned long cycleTimes[MAX_CYCLES];
    float cycleAmplitudes[MAX_CYCLES];
    
    // Amplitude tracking
    float maxPressure;
    float minPressure;
    bool firstCycleComplete;
    
    // Configuration parameters
    float testSetpoint;
    float minPwmValue;
    float maxPwmValue;
    unsigned long minCycleTime;
    TuningRule currentTuningRule;
    float tuningAggressiveness;
    
    // System references
    Settings* settings;
    PID* pid;
    double* pressureInput;
    int* pwmMaxValue;
    
    // Constants
    static constexpr unsigned long AUTO_TUNE_TIMEOUT = 180000; // 3 minutes
    static constexpr float AUTOTUNE_NOISE_BAND = 0.1f; // Deadband around setpoint
    
public:
    /**
     * Constructor
     * @param settings Pointer to Settings instance
     * @param pid Pointer to PID controller instance
     * @param pressureInput Pointer to current pressure reading
     * @param pwmMaxValue Pointer to maximum PWM value
     */
    AutoTuner(Settings* settings, PID* pid, double* pressureInput, int* pwmMaxValue);
    
    /**
     * Start the auto-tuning process
     * Initializes all variables and begins relay oscillation
     */
    void start();
    
    /**
     * Stop the auto-tuning process
     * @param calculateParameters If true, calculate PID parameters from collected data
     */
    void stop(bool calculateParameters = false);
    
    /**
     * Process auto-tuning logic (call regularly from control loop)
     * Handles relay switching and cycle detection
     */
    void process();
    
    /**
     * Check if auto-tuning is currently running
     * @return true if auto-tuning is active
     */
    bool isRunning() const { return autoTuneRunning; }
    
    /**
     * Get current auto-tuning output value
     * @return Current PWM output percentage for auto-tuning
     */
    float getOutputValue() const { return autoTuneOutputValue; }
    
    /**
     * Accept calculated PID parameters and apply them
     * Updates the PID controller and saves settings
     */
    void acceptParameters();
    
    /**
     * Reject calculated PID parameters
     * Reverts to previous PID settings
     */
    void rejectParameters();
    
    // Configuration methods
    
    /**
     * Set the test setpoint for auto-tuning
     * @param setpoint Target pressure during auto-tuning (0.5-10.0 bar)
     */
    void setTestSetpoint(float setpoint);
    
    /**
     * Set the tuning rule algorithm
     * @param rule Tuning rule to use for parameter calculation
     */
    void setTuningRule(TuningRule rule);
    
    /**
     * Set the aggressiveness factor
     * @param aggr Aggressiveness multiplier (0.5-2.0)
     */
    void setAggressiveness(float aggr);
    
    /**
     * Set the PWM range for relay oscillation
     * @param minPwm Minimum PWM percentage for low relay state
     * @param maxPwm Maximum PWM percentage for high relay state
     */
    void setMinMaxPWM(float minPwm, float maxPwm);
    
    /**
     * Set minimum cycle time to prevent noise-triggered transitions
     * @param cycleTime Minimum time between transitions in milliseconds
     */
    void setMinCycleTime(unsigned long cycleTime);
    
    /**
     * Get current test setpoint
     * @return Current auto-tuning test setpoint
     */
    float getTestSetpoint() const { return testSetpoint; }
    
    /**
     * Get current tuning rule
     * @return Current tuning rule enum
     */
    TuningRule getTuningRule() const { return currentTuningRule; }
    
    /**
     * Get current aggressiveness factor
     * @return Current aggressiveness multiplier
     */
    float getAggressiveness() const { return tuningAggressiveness; }
    
    /**
     * Get PWM range information
     */
    float getMinPWM() const { return minPwmValue; }
    float getMaxPWM() const { return maxPwmValue; }
    float getEffectiveAmplitude() const { return maxPwmValue - minPwmValue; }
    
    /**
     * Get minimum cycle time
     * @return Minimum cycle time in milliseconds
     */
    unsigned long getMinCycleTime() const { return minCycleTime; }
    
    /**
     * Display available tuning rules with descriptions
     */
    void printTuningRules() const;
    
    /**
     * Display current auto-tuning configuration
     */
    void printConfiguration() const;
    
private:
    /**
     * Calculate PID parameters from collected cycle data
     * @param newKp Reference to store calculated Kp value
     * @param newKi Reference to store calculated Ki value
     * @param newKd Reference to store calculated Kd value
     * @return true if calculation successful
     */
    bool calculatePIDParameters(float& newKp, float& newKi, float& newKd);
    
    /**
     * Get the name of the current tuning rule
     * @return String description of current tuning rule
     */
    const char* getTuningRuleName() const;
    
    /**
     * Apply PWM output for auto-tuning
     * @param pwmPercent PWM percentage to apply
     */
    void applyPWMOutput(float pwmPercent);
};
