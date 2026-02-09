#include "Logger.h"

// Static member initialization
unsigned long Logger::lastLogTime[CAT_COUNT] = {0};
bool Logger::enabled = false;
unsigned long Logger::rateLimit = TimingConfig::SERIAL_OUTPUT_INTERVAL_MS;

void Logger::init() {
    enabled = false;
    rateLimit = TimingConfig::SERIAL_OUTPUT_INTERVAL_MS;
    for (int i = 0; i < CAT_COUNT; i++) {
        lastLogTime[i] = 0;
    }
}

void Logger::setEnabled(bool en) {
    enabled = en;
}

bool Logger::isEnabled() {
    return enabled;
}

void Logger::setRateLimit(unsigned long intervalMs) {
    rateLimit = intervalMs;
}

bool Logger::shouldLog(LogCategory cat) {
    if (!enabled) {
        return false;
    }
    
    unsigned long now = millis();
    if (now - lastLogTime[cat] >= rateLimit) {
        lastLogTime[cat] = now;
        return true;
    }
    return false;
}

void Logger::resetTimer(LogCategory cat) {
    if (cat < CAT_COUNT) {
        lastLogTime[cat] = 0;
    }
}

const char* Logger::getCategoryName(LogCategory cat) {
    switch (cat) {
        case CAT_CONTROL:  return "CTRL";
        case CAT_NETWORK:  return "NET ";
        case CAT_SENSOR:   return "SENS";
        case CAT_AUTOTUNE: return "TUNE";
        case CAT_SYSTEM:   return "SYS ";
        default:           return "??? ";
    }
}
