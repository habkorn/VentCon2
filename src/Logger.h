#pragma once

#include <Arduino.h>
#include "Constants.h"

/**
 * Log Levels - higher value = more verbose
 * Set via build_flags: -DLOG_LEVEL=LOG_DEBUG
 */
enum LogLevel {
    LOG_NONE  = 0,
    LOG_ERROR = 1,
    LOG_WARN  = 2,
    LOG_INFO  = 3,
    LOG_DEBUG = 4
};

/**
 * Log Categories - each has independent rate limiting
 */
enum LogCategory {
    CAT_CONTROL,    // Control loop, PID, valve
    CAT_NETWORK,    // WiFi, web server, DNS
    CAT_SENSOR,     // Pressure sensor, ADC
    CAT_AUTOTUNE,   // Auto-tuning process
    CAT_SYSTEM,     // General system messages
    CAT_COUNT       // Must be last - used for array sizing
};

// Default log level if not set via build flags
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_INFO
#endif

/**
 * Logger Class
 * 
 * Provides centralized, rate-limited logging with:
 * - Per-category rate limiting (prevents flooding)
 * - Compile-time level filtering (zero overhead when disabled)
 * - Runtime enable/disable (ties into existing continousValueOutput)
 * - printf-style formatting
 */
class Logger {
private:
    static unsigned long lastLogTime[CAT_COUNT];
    static bool enabled;
    static unsigned long rateLimit;  // ms between logs per category

public:
    /**
     * Initialize the logger (call in setup())
     */
    static void init();
    
    /**
     * Enable/disable logging output
     * @param en true to enable, false to disable
     */
    static void setEnabled(bool en);
    
    /**
     * Check if logging is currently enabled
     */
    static bool isEnabled();
    
    /**
     * Set rate limit interval for all categories
     * @param intervalMs milliseconds between allowed logs per category
     */
    static void setRateLimit(unsigned long intervalMs);
    
    /**
     * Check if a category should log (rate limiting check)
     * @param cat the log category
     * @return true if enough time has passed since last log
     */
    static bool shouldLog(LogCategory cat);
    
    /**
     * Force reset rate limit timer for a category
     * @param cat the log category to reset
     */
    static void resetTimer(LogCategory cat);
    
    /**
     * Get category name for output prefix
     */
    static const char* getCategoryName(LogCategory cat);
};

// ============== Logging Macros ==============

// Helper macro for formatted output with category and level prefix
#define LOG_PRINT(level, cat, fmt, ...) \
    Serial.printf("[%s][%s] " fmt "\n", level, Logger::getCategoryName(cat), ##__VA_ARGS__)

// ERROR - Always logged (not rate-limited, not compile-filtered)
#define LOG_E(cat, fmt, ...) \
    LOG_PRINT("E", cat, fmt, ##__VA_ARGS__)

// WARNING - Logged if LOG_LEVEL >= LOG_WARN
#if LOG_LEVEL >= LOG_WARN
#define LOG_W(cat, fmt, ...) \
    if (Logger::isEnabled()) LOG_PRINT("W", cat, fmt, ##__VA_ARGS__)
#else
#define LOG_W(cat, fmt, ...) ((void)0)
#endif

// INFO - Logged if LOG_LEVEL >= LOG_INFO, rate-limited
#if LOG_LEVEL >= LOG_INFO
#define LOG_I(cat, fmt, ...) \
    if (Logger::shouldLog(cat)) LOG_PRINT("I", cat, fmt, ##__VA_ARGS__)
#else
#define LOG_I(cat, fmt, ...) ((void)0)
#endif

// DEBUG - Logged if LOG_LEVEL >= LOG_DEBUG, rate-limited
#if LOG_LEVEL >= LOG_DEBUG
#define LOG_D(cat, fmt, ...) \
    if (Logger::shouldLog(cat)) LOG_PRINT("D", cat, fmt, ##__VA_ARGS__)
#else
#define LOG_D(cat, fmt, ...) ((void)0)
#endif
