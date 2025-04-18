#ifndef PUMP_H
#define PUMP_H

#include <Arduino.h>
#include <vector>


class Pump {
private:
    int enablePin;             // Pin to enable or disable the pump
    int interruptPin;          // Pin connected to the interrupt signal
    volatile int16_t counter;           // Counter to track pulses
    uint32_t lastInterruptTime; // Time of the last interrupt (for debouncing)
    float mlPerPulse;          // Milliliters per pulse (calibrated value)
    static constexpr unsigned long debounceTime = 400; // Static debounce time (microseconds)

    //monitoring
    uint32_t startTime;       // Time when the pump was started
    uint32_t runTime;
    int16_t pulses;           // Number of pulses

public:
    // Constructor
    Pump(int interruptPin, int enablePin, float mlPerPulse);

    // Initialize pins
    void __begin();

    // Interrupt Service Routine (ISR)
    void IRAM_ATTR handleInterrupt();

    // Start the pump
    void start();

    // Stop the pump
    void stop();

    // Run the pump for a specified amount of milliliters
    void runForMl(float milliliters);

    // Run the pump for a specified number of pulses, this is for calibration mostly
    void runForPulses(int pulses);

    // Check if the pump is currently running
    bool isBusy() const {
        return counter > 0;
    }

    // Getter for the interrupt pin
    int getInterruptPin() const;

    //monitoring
    float getPulsesPerSecond() const {
        return (float)pulses / ((float)runTime / 1000.0);
    }
    int getLastPulses() const {return pulses;}

    // Debugging the hardware
    std::vector<uint32_t> runAndGetInterruptTimes();
};

// Macro to define a Pump instance and its static interrupt handler
#define CREATE_PUMP(name, interruptPin, enablePin, mlPerPulse) \
    Pump name(interruptPin, enablePin, mlPerPulse);                         \
    void IRAM_ATTR name##_interruptHandler() {                              \
        name.handleInterrupt();                                             \
    }                                                                     

#define BEGIN_PUMP(name) \
    name.__begin(); \
    attachInterrupt(digitalPinToInterrupt(name.getInterruptPin()), name##_interruptHandler, RISING)

#endif // PUMP_H