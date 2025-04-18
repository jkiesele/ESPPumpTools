
#ifndef ASYNCH_MONITORED_PUMP_H
#define ASYNCH_MONITORED_PUMP_H

#include "MonitoredPump.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <atomic>

class AsyncMonitoredPump : public MonitoredPump {
public:
    AsyncMonitoredPump(uint8_t enablePin, uint8_t touchPin,
                        size_t lookahead, float pulsesPerMl,
                        size_t approxSamplesPerPulse = 0)
        : MonitoredPump(enablePin, touchPin, lookahead, pulsesPerMl, approxSamplesPerPulse),
          taskHandle_(nullptr), pulseTarget_(0), doFullDiagnostics_(false), 
          running_(false) {}

    bool runForMl(float ml, bool fullDiagnostics = false, bool blocking = false); ;

    // Kick off a background run, overrides runForPulses in MonitoredPump
    bool runForPulses(uint32_t pulses, bool fullDiagnostics = false, bool blocking = false);

    // Check if the task has completed
    bool isFinished() const {
        return !running_.load();
    }

    // compatibility to other pump classes
    bool isBusy() const {
        return running_.load();
    }

    void stop();

private:
    static void taskFunc(void* param) {
        AsyncMonitoredPump* self = static_cast<AsyncMonitoredPump*>(param);
        // not needed esp_task_wdt_add(NULL);  // Register with watchdog
        self->running_.store(true);
        self->MonitoredPump::runForPulses(self->pulseTarget_, self->doFullDiagnostics_);//feed watchdog
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
    mutable std::atomic<bool> running_{false};
    portMUX_TYPE localMux_ = portMUX_INITIALIZER_UNLOCKED;

};

#endif
