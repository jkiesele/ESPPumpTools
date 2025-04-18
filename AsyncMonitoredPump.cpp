
#include "AsyncMonitoredPump.h"

bool AsyncMonitoredPump::runForMl(float ml, bool fullDiagnostics, bool blocking) {
    if (isBusy()) return false; // already running
    if (!volumeSupported(ml)) return false; // not enough pulses
    //get no of pulses
    float pulsesNeeded = ml * pulsesPerMl();
    return runForPulses(pulsesNeeded, fullDiagnostics, blocking);
}

bool AsyncMonitoredPump::runForPulses(uint32_t pulses, bool fullDiagnostics, bool blocking) {
    if (isBusy()) return false; // already running
    running_.store(true);//set the flag here
    if(blocking){
        //run in blocking mode
        bool ret = MonitoredPump::runForPulses(pulses, fullDiagnostics);
        running_.store(false);//reset the flag also in blocking mode
        return ret;
    }

    pulseTarget_ = pulses;
    doFullDiagnostics_ = fullDiagnostics;

    BaseType_t result = xTaskCreatePinnedToCore(
        taskFunc,               // Function
        ("PumpTask_"+String((uint32_t)this)).c_str() , // Name, add this pointer as hex
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

void AsyncMonitoredPump::stop() {
    if(!running_.load()) return;
    taskENTER_CRITICAL(&localMux_);
    TaskHandle_t handle = taskHandle_;
    taskHandle_ = nullptr;
    taskEXIT_CRITICAL(&localMux_);

    if (handle != nullptr) {
        vTaskDelete(handle);
    }
    running_.store(false);
}
