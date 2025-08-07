
#ifndef ASYNCH_MONITORED_PUMP_H
#define ASYNCH_MONITORED_PUMP_H

#include "MonitoredPump.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>

template <std::size_t Lookahead>
class AsyncMonitoredPump : public MonitoredPump<Lookahead> {
public:
    AsyncMonitoredPump(uint8_t enablePin, uint8_t touchPin, float pulsesPerMl,
                        size_t approxSamplesPerPulse = 0)
        : MonitoredPump<Lookahead>(enablePin, touchPin, pulsesPerMl, approxSamplesPerPulse),
          taskHandle_(nullptr), pulseTarget_(0), doFullDiagnostics_(false), 
          running_(false) {}

    AsyncMonitoredPump(AsyncMonitoredPump&&) = default; //for move semantics

    bool runForMl(float ml, bool fullDiagnostics = false) override;

    // Kick off a background run, overrides runForPulses in MonitoredPump
    bool runForPulses(uint32_t pulses, bool fullDiagnostics = false) override;

    // Check if the task has completed
    bool isFinished() const override {
        return !running_.load();
    }

    // compatibility to other pump classes
    bool isBusy() const override {
        return running_.load();
    }

    void stop() override;

private:
    static void taskFunc(void* param) {
        AsyncMonitoredPump* self = static_cast<AsyncMonitoredPump*>(param);
        // not needed esp_task_wdt_add(NULL);  // Register with watchdog
        self->running_.store(true);
        self->MonitoredPump<Lookahead>::runForPulses(self->pulseTarget_, self->doFullDiagnostics_, &self->abort_);//feed watchdog
        self->running_.store(false);
        // not needed esp_task_wdt_delete(NULL);//unregister
        taskENTER_CRITICAL(&(self->localMux_));
        self->taskHandle_ = nullptr;
        taskEXIT_CRITICAL(&(self->localMux_));
        vTaskDelete(NULL);
    }

    TaskHandle_t taskHandle_;
    uint32_t pulseTarget_;
    bool doFullDiagnostics_;
    mutable std::atomic<bool> abort_{false}; 
    mutable std::atomic<bool> running_{false};
    portMUX_TYPE localMux_ = portMUX_INITIALIZER_UNLOCKED;

};


template <std::size_t Lookahead>
bool AsyncMonitoredPump<Lookahead>::runForMl(float ml, bool fullDiagnostics) {
    if (isBusy()) return false; // already running
    if (!this->volumeSupported(ml)) return false; // not enough pulses
    //get no of pulses
    float pulsesNeeded = ml * this->pulsesPerMl();
    return runForPulses(pulsesNeeded, fullDiagnostics);
}

template <std::size_t Lookahead>
bool AsyncMonitoredPump<Lookahead>::runForPulses(uint32_t pulses, bool fullDiagnostics) {
    if (isBusy()) return false; // already running
    running_.store(true);//set the flag here
    
    pulseTarget_ = pulses;
    doFullDiagnostics_ = fullDiagnostics;

    BaseType_t result = xTaskCreatePinnedToCore(
        taskFunc,               // Function
        nullptr , // Name, 
        8192,                   // Stack size in bytes
        this,                   // Pass this pointer
        1,                      // Priority
        &taskHandle_,           // Task handle out
        0                       // Core 0, arduino core uses core 1
    );

    if (result != pdPASS) {
        return false;
    }
    return true;//all good
}

template <std::size_t Lookahead>
void AsyncMonitoredPump<Lookahead>::stop() {
     if (!running_.load()) return;           // nothing to do

    abort_.store(true, std::memory_order_release); // ask worker to exit

    TaskHandle_t h;
    taskENTER_CRITICAL(&localMux_);
    h = taskHandle_;                        // copy under lock
    taskEXIT_CRITICAL(&localMux_);

    /*  Wait (max 1 s) until the task sets taskHandle_ = nullptr in
        its epilogue.  The loop also feeds the WDT.                   */
    uint32_t deadline = millis() + 1000;
    while (h && millis() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(10));
        taskENTER_CRITICAL(&localMux_);
        h = taskHandle_;
        taskEXIT_CRITICAL(&localMux_);
    }

    /*  If the worker has *already* deleted itself, h is nullptr and
        we’re done.  If it is still alive (timeout), fall back to a
        forced delete but only when the state is not eDeleted yet.   */
    if (h && eTaskGetState(h) != eDeleted) {
        vTaskDelete(h);
    }
    running_.store(false);
}


#endif
