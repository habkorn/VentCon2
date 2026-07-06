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
};

#endif // TASKMANAGER_H
