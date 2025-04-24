
// FlowMeter.cpp
#include "FlowMeter.h"

FlowMeter::FlowMeter(int interruptPin, float pulsesPerLiter, uint32_t debounceTime)
    : interruptPin(interruptPin), pulsesPerLiter(pulsesPerLiter), 
    pulseCount(0), lastInterruptTime(0), debounceTime(debounceTime) {}

void FlowMeter::__begin() {
    pinMode(interruptPin, INPUT_PULLUP);
}

void FlowMeter::handleInterrupt() {
    uint32_t now = micros();
    if (now - lastInterruptTime > debounceTime) {
        lastInterruptTime = now;
        portENTER_CRITICAL_ISR(&mux);
        pulseCount++;
        portEXIT_CRITICAL_ISR(&mux);
    }
}

float FlowMeter::getLiters() const {
    return (float)getPulseCount() / pulsesPerLiter;
}

int FlowMeter::getInterruptPin() const {
    return interruptPin;
}
