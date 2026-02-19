#include "TaskManager.h"
#include "Constants.h"
#include "Logger.h"

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
            
            webHandler->getWebServer().handleClient();
            
            unsigned long netTime = micros() - netStart;
            if (netTime > TaskConfig::NETWORK_DELAY_WARNING_US)
            { // Log if >20ms
                // Serial.printf("Network delay: %lu us (Core 0)\n", netTime);
            }
        }
        
        // Delay to prevent watchdog timeout - matches web polling rate
        vTaskDelay(pdMS_TO_TICKS(TaskConfig::NETWORK_TASK_DELAY_MS)); // 10ms delay = 100Hz, matches /values poll rate
    }
}

bool TaskManager::createTasks() 
{
    bool success = true;
    
    // Create network task on Core 0
    BaseType_t networkResult = xTaskCreatePinnedToCore(
        networkTaskWrapper,        // Task function
        "NetworkTask",            // Task name
        TaskConfig::STACK_SIZE,                     // Stack size (bytes)
        this,                     // Parameter passed to task (this TaskManager instance)
        1,                        // Task priority (1 = low, higher number = higher priority)
        &networkTaskHandle,       // Task handle
        0                         // Core 0
    );
    
    if (networkResult != pdPASS) 
    {
        LOG_E(CAT_SYSTEM, "Failed to create NetworkTask");
        success = false;
    } 
    else 
    {
        LOG_I(CAT_SYSTEM, "NetworkTask created successfully on Core 0");
    }

    // Create control task on Core 1
    BaseType_t controlResult = xTaskCreatePinnedToCore(
        ControlSystem::controlTaskWrapper,  // Task function
        "ControlTask",                      // Task name
        TaskConfig::STACK_SIZE,                              // Stack size (bytes)
        controlSystem,                     // Parameter passed to task (ControlSystem instance)
        2,                                 // Task priority (2 = higher than network)
        &controlTaskHandle,                // Task handle
        1                                  // Core 1
    );
    
    if (controlResult != pdPASS) 
    {
        LOG_E(CAT_SYSTEM, "Failed to create ControlTask");
        success = false;
    } 
    else 
    {
        LOG_I(CAT_SYSTEM, "ControlTask created successfully on Core 1");
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
