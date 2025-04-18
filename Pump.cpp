// Pump.cpp
#include "Pump.h"


// Constructor
Pump::Pump(int interruptPin, int enablePin, float mlPerPulse)
    : interruptPin(interruptPin), enablePin(enablePin), mlPerPulse(mlPerPulse), counter(0), lastInterruptTime(0) {}

// Initialize pins
void Pump::__begin() {
    pinMode(interruptPin, INPUT_PULLUP); // Set interrupt pin as input with pull-up resistor
    pinMode(enablePin, OUTPUT);         // Set enable pin as output
    digitalWrite(enablePin, LOW);       // Ensure pump is initially off
}

// Interrupt Service Routine (ISR) - no debounce needed with comparator edge
void IRAM_ATTR Pump::handleInterrupt() {
    if(counter<0) return; // Prevent counting if not running
    uint32_t currentTime = micros(); // Get the current time
    if (currentTime - lastInterruptTime > debounceTime) { // Check if debounce time has passed
        lastInterruptTime = currentTime; // Update last interrupt time
        if (--counter <= 0) { // Decrement counter and check if target is reached
            stop();           // Stop the pump if target is reached
            counter=-1;
        }
    }
}

// Start the pump
void Pump::start() {
    startTime = millis();          // Store the start time
    lastInterruptTime = micros() - debounceTime; // Initialize the first interrupt time, make sure first interrupt is triggered
    pulses = counter;              // Store the target count
    digitalWrite(enablePin, HIGH); // Enable the pump
}

// Stop the pump
void Pump::stop() {
    digitalWrite(enablePin, LOW); // Disable the pump
    counter = -1; // Set the counter to -1 to indicate the pump is stopped
    runTime = millis() - startTime; // Calculate the run time
}

// Run the pump for a specified amount of milliliters
void Pump::runForMl(float milliliters) {
    if (isBusy()) return; // Prevent starting if already running
    counter = static_cast<int16_t>(milliliters / mlPerPulse); // Calculate the target count from milliliters
    if(counter <= 0) return; // Prevent starting if target is invalid
    start();                                              // Start the pump
}

void Pump::runForPulses(int pulses) {
    if (isBusy()) return; // Prevent starting if already running
    counter = pulses;     // Set the target count
    start();              // Start the pump
}

// Getter for the interrupt pin
int Pump::getInterruptPin() const {
    return interruptPin;
}


std::vector<uint32_t> Pump::runAndGetInterruptTimes() {
    if(isBusy()) return {};
    int pulses = 300;
    std::vector<uint32_t> times(pulses,0);
    
    //use interrupts here
    runForPulses(pulses);//then count counter changes (interrupts on)
    int prev_counter = counter;
    uint32_t timeout = millis() + 15000;
    while(isBusy()){
        //counter counts down an is then set to -1, so need safeguard
        if(prev_counter != counter && pulses - counter < times.size() && pulses - counter >= 0){
            times[pulses - counter] = micros();
            prev_counter = counter; 
        }
        //timeout, consider that it can overflow
        if(millis() > timeout){
            stop();
            return {};
        }
    }
    //remove first 
    times.erase(times.begin());
    std::vector<uint32_t> times_diff(times.size()-1,0);
    //only consider difference between times
    for(int i = 0; i < times_diff.size(); i++){
        times_diff[i] = times[i+1] - times[i];
    }
    //remove first
    return times_diff;
}
