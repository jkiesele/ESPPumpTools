
#ifndef MONITORED_PUMP_H
#define MONITORED_PUMP_H
extern "C" {
    #include "driver/touch_pad.h"
  }
#include <Arduino.h>
#include <vector>
#include <PulseLookaheadDetector.h>
#include "esp_task_wdt.h"



// little helper class for diagnostic information
class PumpDiagnostics {
public:
    // convencience method to check if there are any pulses
    // the other vectors are always filled if there are pulses
    bool hasFullShape(){return fullShape.size() > 0;}
    float averagePulseTime() const;
    float timeDeviation() const;
    float averageAmplitude() const;
    float amplitudeDeviation() const;

    void clear() {
        pulseTimes.clear();
        isPulse.clear();
        valuesAtPulses.clear();
        fullShape.clear();
        //baseline = 0; //don't clear baseline
    }

    String summary() const {
        return "Average pulse time: " + String(averagePulseTime()) + " +- " + String(timeDeviation()) + " µs; Average amplitude: " 
        + String(averageAmplitude()) + " +- " + String(amplitudeDeviation()) + "; Baseline: " + String(baseline);
    }

    std::vector<unsigned long> pulseTimes;
    std::vector<bool> isPulse;
    std::vector<int32_t> valuesAtPulses;
    std::vector<int32_t> fullShape;
    unsigned long baseline=0;
};

class MonitoredPump {
    /*
    * MonitoredPump class
    * This class is used to monitor the pump and detect pulses, as well as
    * to calculate the flow rate of the pump, and give other diagnostic information.
    * It needs an enable pin and a touch pin, as well as a lookahead window size, as 
    * well as the pulses per ml of pumped fluid.
    * 
    * There will be two pulse detectors, one operating on the peaks of the signal
    * and the other operating on the troughs of the signal. The difference between
    * the two detectors will be used to calculate the amplitude, which can be used
    * subsequently to check if the pump is transferring fluid or air (emty tank).
    * 
    * The flow rate will be calculated by counting the number of pulses in a given
    * time window, and dividing by the volume of the pump.
    * 
    * Also the times of each pulse will be stored in a vector, so that the time,
    * such that the time between pulses can be calculated. This can help detect
    * issues with the pulse detection (very large pulse variation) or the pump
    * -very large time between pulses or suddenly very short time between pulses.
    * 
    * The latter might happen if the hose in the pump is broken and provides less
    * resistance to the pump, or if the pump is clogged and provides more resistance.
    */
private:
    const uint8_t enablePin_;
    const uint8_t touchPin_;
    const size_t lookahead_;
    const float pulsesPerMl_;
    // the next two will be created dynamically when the pump runs
    // the pump will always run in blocking mode
    // PulseLookaheadDetector<uint32_t> pulseDetector;
    // PulseLookaheadDetector<uint32_t> troughDetector;
    
    mutable uint32_t approxSamplesPerPulse_;
    uint32_t capBaseline_=0;

    PulseLookaheadDetector<int32_t> peakDetector;
    PulseLookaheadDetector<int32_t> troughDetector;

    PumpDiagnostics diagnostics_;

public:
//constructor
    MonitoredPump(uint8_t enablePin, uint8_t touchPin, 
        size_t lookahead, float pulsesPerMl, size_t approxSamplesPerPulse=0)
        : enablePin_(enablePin), touchPin_(touchPin), lookahead_(lookahead), 
        pulsesPerMl_(pulsesPerMl), approxSamplesPerPulse_(approxSamplesPerPulse),
        peakDetector(lookahead), troughDetector(lookahead, true) {
        
    }
    void begin() {
        pinMode(enablePin_, OUTPUT);
        digitalWrite(enablePin_, LOW);
        pinMode(touchPin_, INPUT);
        
       // Set FSM to timer mode (more predictable)
        touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
      
        // Reduce measurement and sleep cycles (faster but noisier)
        touch_pad_set_meas_time(24, 300);//in 8MHz clock sycles: total of 1.5ms
    }
//destructor
    ~MonitoredPump() {
        // nothing to do here
    }
//methods
    bool runForPulses(uint32_t pulses, bool fulldiagnostics=false)  ;
    bool volumeSupported(float ml) const {
        return ml * pulsesPerMl_ > 5;
    }
    float pulsesPerMl() const {
        return pulsesPerMl_;
    }
    uint32_t getApproxSamplesPerPulse() const {
        return approxSamplesPerPulse_;
    }
    bool runForMl(float ml, bool fulldiagnostics=false) ;

    const PumpDiagnostics& getDiagnostics() const {
        return diagnostics_;
    }
    void clearDiagnostics() {
        diagnostics_.clear();
    }
    
};

#endif // MONITORED_PUMP_H