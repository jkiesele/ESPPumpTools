
#ifndef MONITORED_PUMP_H
#define MONITORED_PUMP_H
extern "C" {
    #include "driver/touch_pad.h"
  }
#include <Arduino.h>
#include <vector>
#include <PulseLookaheadDetector.h>
#include "esp_task_wdt.h"
#include "LoggingBase.h"



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

class MonitoredPumpBase {
    // virtual base class for MonitoredPump
public:
    virtual ~MonitoredPumpBase() = default;
    virtual bool runForMl(float ml, bool fulldiagnostics=false) = 0;
    virtual bool runForPulses(uint32_t pulses, bool fulldiagnostics=false) = 0;
    virtual void stop(){}
    virtual bool isBusy() const{return false;} //should be overridden for async pumps
    virtual bool isFinished() const{return true;} //should be overridden for async pumps
    virtual const PumpDiagnostics& getDiagnostics() const = 0;
    virtual void clearDiagnostics() = 0;
    virtual float pulsesPerMl() const = 0;
    virtual bool volumeSupported(float ml) const = 0;
    virtual uint32_t getApproxSamplesPerPulse() const = 0;
};

template<std::size_t  Lookahead>
class MonitoredPump : public MonitoredPumpBase {
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
    const float pulsesPerMl_;
    
    mutable uint32_t approxSamplesPerPulse_;
    uint32_t capBaseline_=0;

    PulseLookaheadDetector<int32_t,Lookahead> peakDetector;
    PulseLookaheadDetector<int32_t,Lookahead> troughDetector;

    PumpDiagnostics diagnostics_;

public:
//constructor
    MonitoredPump(uint8_t enablePin, uint8_t touchPin, float pulsesPerMl, size_t approxSamplesPerPulse=0)
        : enablePin_(enablePin), touchPin_(touchPin), 
        pulsesPerMl_(pulsesPerMl), approxSamplesPerPulse_(approxSamplesPerPulse),
        peakDetector(), troughDetector(true) {}
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
    bool runForPulses(uint32_t pulses, bool fulldiagnostics=false)  override;
    bool volumeSupported(float ml) const override {
        gLogger->println("Checking volume support for " + String(ml) + " ml with pulsesPerMl: " + String(pulsesPerMl_));
        return ml * pulsesPerMl_ > 5;
    }
    float pulsesPerMl() const {
        return pulsesPerMl_;
    }
    uint32_t getApproxSamplesPerPulse() const {
        return approxSamplesPerPulse_;
    }
    bool runForMl(float ml, bool fulldiagnostics=false) override;

    const PumpDiagnostics& getDiagnostics() const {
        return diagnostics_;
    }
    void clearDiagnostics() {
        diagnostics_.clear();
    }
    
};


template<std::size_t  Lookahead>
bool MonitoredPump<Lookahead>::runForPulses(uint32_t pulses, bool fulldiagnostics)  {
    if(pulses == 0){
        return false;
    }
    //set up the detectors
    peakDetector.clear();
    troughDetector.clear();
    //set up the diagnostics
    //read the baseline
    diagnostics_.clear();
    diagnostics_.baseline = touchRead(touchPin_);
    if(capBaseline_ == 0){
        capBaseline_ = diagnostics_.baseline;
    }
    diagnostics_.pulseTimes.reserve(pulses+1);
    diagnostics_.valuesAtPulses.reserve(pulses+1);
    if(fulldiagnostics && approxSamplesPerPulse_ > 0){
        //add some extra space to the full shape vector
        diagnostics_.fullShape.reserve(approxSamplesPerPulse_ * (pulses+10));
        diagnostics_.isPulse.reserve(approxSamplesPerPulse_ * (pulses+10));
    }

    //pre-fill the lookahead buffers
    for(size_t i=0; i<Lookahead; ++i){
        peakDetector.addSample(capBaseline_);
        troughDetector.addSample(capBaseline_);
    }
    unsigned long totalSamples = 0;
    //run the pump
    digitalWrite(enablePin_, HIGH);
    const unsigned long intervalMs = 2;
    float raw_average = 0.0;
    while(pulses > 0){
        //read the current value
        unsigned long raw_value = touchRead(touchPin_);
        raw_average += raw_value;
        int32_t value = raw_value - capBaseline_;//can subtract directly
        //add the sample to the detectors, raw here as it needs to be >0
        bool peak = peakDetector.addSample(raw_value);
        bool trough = troughDetector.addSample(raw_value);
        // if either is true
        if(peak || trough){
            //a pulse was detected
            diagnostics_.pulseTimes.push_back(micros() - (peakDetector.centerOffset() * intervalMs * 1000));//roughly
            
            int32_t valAtPulse = peakDetector.getCenterValue();
            if(trough)
                valAtPulse = troughDetector.getCenterValue();
            diagnostics_.valuesAtPulses.push_back(valAtPulse);
            
            --pulses;
        }
        if(fulldiagnostics){
            diagnostics_.fullShape.push_back(value);
            diagnostics_.isPulse.push_back(false);
            //now if this was a pulse, set the entry in the past defined by lookahead to true
            if(peak){
                int index = diagnostics_.isPulse.size() - peakDetector.centerOffset();
                if(index >= 0 && index < diagnostics_.isPulse.size())
                    diagnostics_.isPulse[index] = true;
            }
            if(trough){
                int index = diagnostics_.isPulse.size() - troughDetector.centerOffset();
                if(index >= 0 && index < diagnostics_.isPulse.size())
                    diagnostics_.isPulse[index] = true;
            }
        }
        ++totalSamples;
        delay(intervalMs);//this will call vTaskDelay under the hood - ok for watchdog; 
        // delayMicroseconds does not
    }
    //update the approxSamplesPerPulse
    approxSamplesPerPulse_ = totalSamples / diagnostics_.pulseTimes.size();
    raw_average /= totalSamples;
    capBaseline_ = raw_average;
    //stop the pump
    digitalWrite(enablePin_, LOW);
    //return the diagnostics
    return true;
}

template<std::size_t  Lookahead>
bool MonitoredPump<Lookahead>::runForMl(float ml, bool fulldiagnostics)  {
    //calculate the number of pulses needed
    float pulsesNeeded = ml * pulsesPerMl_;
    //if this is too low, return an empty diagnostics object
    if(!volumeSupported(ml)){
        return false;
    }
    return runForPulses(pulsesNeeded, fulldiagnostics);
}

#endif // MONITORED_PUMP_H