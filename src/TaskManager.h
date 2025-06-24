#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ControlSystem.h"
#include "WebHandlers.h"

class TaskManager 
{
private:
    TaskHandle_t networkTaskHandle;
    TaskHandle_t controlTaskHandle;
    
    ControlSystem* controlSystem;
    WebHandler* webHandler;
    
    // Static task wrappers
    static void networkTaskWrapper(void* parameter);

public:
    TaskManager(ControlSystem* controlSystem, WebHandler* webHandler);
    
    bool createTasks();
    void printTaskInfo();
    
    // Getters for task handles (for external monitoring)
    TaskHandle_t getNetworkTaskHandle() const { return networkTaskHandle; }
    TaskHandle_t getControlTaskHandle() const { return controlTaskHandle; }
};

#endif // TASKMANAGER_H
