// FlowMeter.h
#pragma once
#include <Arduino.h>
#include <vector>
#include "freertos/portmacro.h"

class FlowMeter {
public:
    // makximum debounce time about 70 minutes (as this is in microseconds)
    FlowMeter(int interruptPin, float pulsesPerLiter, uint32_t debounceTime);

    void IRAM_ATTR handleInterrupt();

    float getLiters() const;
    
    inline void reset() { 
        portENTER_CRITICAL(&mux);
        pulseCount = 0;
        portEXIT_CRITICAL(&mux);
    }

    uint32_t getPulseCount() const {
        portENTER_CRITICAL(&mux);
        uint32_t count = pulseCount;
        portEXIT_CRITICAL(&mux);
        return count;
    }

    int getInterruptPin() const;
    void __begin();

private:
    int interruptPin;
    volatile uint32_t pulseCount;
    uint32_t lastInterruptTime;
    float pulsesPerLiter;

    uint32_t debounceTime; // in microseconds
    mutable portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
};

#define CREATE_FLOWMETER(name, interruptPin, pulsesPerLiter) \
    FlowMeter name(interruptPin, pulsesPerLiter); \
    void IRAM_ATTR name##_interruptHandler() { \
        name.handleInterrupt(); \
    }

#define BEGIN_FLOWMETER(name) \
    name.__begin(); \
    attachInterrupt(digitalPinToInterrupt(name.getInterruptPin()), name##_interruptHandler, FALLING)

