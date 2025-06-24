#include "TaskManager.h"

TaskManager::TaskManager(ControlSystem* controlSystem, WebHandler* webHandler)
    : controlSystem(controlSystem), webHandler(webHandler), 
      networkTaskHandle(NULL), controlTaskHandle(NULL) 
{
}

void TaskManager::networkTaskWrapper(void* parameter) 
{
    TaskManager* taskManager = static_cast<TaskManager*>(parameter);
    WebHandler* webHandler = taskManager->webHandler;
    
    while(true) 
    {
        if (webHandler && webHandler->isWebServerEnabled()) 
        {
            unsigned long netStart = micros();
            
            webHandler->getDNSServer().processNextRequest();
            webHandler->getWebServer().handleClient();
            
            unsigned long netTime = micros() - netStart;
            if (netTime > 20000) 
            { // Log if >20ms
                // Serial.printf("Network delay: %lu us (Core 0)\n", netTime);
            }
        }
        
        // Small delay to prevent watchdog timeout and allow other tasks
        vTaskDelay(pdMS_TO_TICKS(2)); // 2ms delay = ~500Hz update rate
    }
}

bool TaskManager::createTasks() 
{
    bool success = true;
    
    // Create network task on Core 0
    BaseType_t networkResult = xTaskCreatePinnedToCore(
        networkTaskWrapper,        // Task function
        "NetworkTask",            // Task name
        4096,                     // Stack size (bytes)
        this,                     // Parameter passed to task (this TaskManager instance)
        1,                        // Task priority (1 = low, higher number = higher priority)
        &networkTaskHandle,       // Task handle
        0                         // Core 0
    );
    
    if (networkResult != pdPASS) 
    {
        Serial.println("ERROR: Failed to create NetworkTask");
        success = false;
    } 
    else 
    {
        Serial.println("NetworkTask created successfully on Core 0");
    }

    // Create control task on Core 1
    BaseType_t controlResult = xTaskCreatePinnedToCore(
        ControlSystem::controlTaskWrapper,  // Task function
        "ControlTask",                      // Task name
        4096,                              // Stack size (bytes)
        controlSystem,                     // Parameter passed to task (ControlSystem instance)
        2,                                 // Task priority (2 = higher than network)
        &controlTaskHandle,                // Task handle
        1                                  // Core 1
    );
    
    if (controlResult != pdPASS) 
    {
        Serial.println("ERROR: Failed to create ControlTask");
        success = false;
    } 
    else 
    {
        Serial.println("ControlTask created successfully on Core 1");
    }
    
    return success;
}

void TaskManager::printTaskInfo() 
{
    Serial.println("\n=== Task Information ===");
    
    if (controlTaskHandle) 
    {
        Serial.printf("  Control Task Stack: %u bytes free\n", uxTaskGetStackHighWaterMark(controlTaskHandle));
    } 
    else 
    {
        Serial.println("  Control Task: Not created");
    }
    
    if (networkTaskHandle) 
    {
        Serial.printf("  Network Task Stack: %u bytes free\n", uxTaskGetStackHighWaterMark(networkTaskHandle));
    } 
    else 
    {
        Serial.println("  Network Task: Not created");
    }
    
    Serial.printf("  Main Loop Stack: %u bytes free\n", uxTaskGetStackHighWaterMark(NULL));
}
