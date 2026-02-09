#ifndef COMMANDPROCESSOR_H
#define COMMANDPROCESSOR_H

#include <Arduino.h>
#include <PID_v2.h>
#include "SettingsHandler.h"
#include "SensorManager.h"
#include "AutoTuner.h"
#include "WebHandlers.h"

// Forward declaration to avoid circular dependency
class TaskManager;

class CommandProcessor 
{
private:    // Dependencies
    SettingsHandler* settings;
    SensorManager* sensorManager;
    AutoTuner* autoTuner;
    WebHandler* webHandler;
    PID* pid;
    TaskManager* taskManager;
    
    // System state pointers
    double* pressureInput;
    double* pwmOutput;
    int* pwmFullScaleRaw;
    bool* manualPWMMode;
    bool* continousValueOutput;
    
    // Helper methods for specific command categories
    void handlePIDCommands(const String& cmd);
    void handleSignalProcessingCommands(const String& cmd);
    void handlePWMCommands(const String& cmd);
    void handleAutoTuningCommands(const String& cmd);
    void handleNetworkCommands(const String& cmd);
    void handleSystemCommands(const String& cmd);
    void handleFileSystemCommands(const String& cmd);
    void handleDiagnosticCommands(const String& cmd);
    
    // Helper methods
    void updatePWM();
    void showHelp();
    void showStatus();
    void showMemoryInfo();
    void listFiles();
    const char* getVersionString();

public:
    CommandProcessor(SettingsHandler* settings, SensorManager* sensorManager, AutoTuner* autoTuner, 
                    WebHandler* webHandler, PID* pid, double* pressureInput, double* pwmOutput,
                    int* pwmFullScaleRaw, bool* manualPWMMode, bool* continousValueOutput,
                    TaskManager* taskManager = nullptr);
    
    void processCommand(const String& cmd);
    void processSerialInput();
};

#endif // COMMANDPROCESSOR_H
