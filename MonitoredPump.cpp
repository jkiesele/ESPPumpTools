#include "MonitoredPump.h"


float PumpDiagnostics::averagePulseTime() const{
    if(pulseTimes.size() < 2){
        return 0.0;
    }
    float sum = 0.0;
    for(size_t i=1; i<pulseTimes.size(); ++i){
        sum += pulseTimes[i] - pulseTimes[i-1];
    }
    return sum / (pulseTimes.size() - 1);
}
float PumpDiagnostics::timeDeviation() const{
    if(pulseTimes.size() < 2){
        return 0.0;
    }
    float sum = 0.0;
    float avg = averagePulseTime();
    for(size_t i=1; i<pulseTimes.size(); ++i){
        sum += sq(pulseTimes[i] - pulseTimes[i-1] - avg);
    }
    return sqrt(sum / (pulseTimes.size() - 1));
}
float PumpDiagnostics::averageAmplitude() const{
    // we have pulses at minima and maxima, so the amplitude is the difference between the two
    if(valuesAtPulses.size() < 2){
        return 0.0;
    }
    float sum = 0.0;
    for(size_t i=1; i<valuesAtPulses.size(); ++i){
        sum += abs(valuesAtPulses[i] - valuesAtPulses[i-1]);
    }
    return sum / ((valuesAtPulses.size() - 1));//each counts twice
}
float PumpDiagnostics::amplitudeDeviation() const{
    
    if(valuesAtPulses.size() < 1){
        return 0.0;
    }
    float sum = 0.0;
    float avg = averageAmplitude(); //this is 1/2 peak to peak
    for(size_t i=1; i<valuesAtPulses.size(); ++i){
        sum += sq(abs(valuesAtPulses[i] - valuesAtPulses[i-1]) - avg);
    }
    return sqrt(sum / ((valuesAtPulses.size() - 1)));
}


bool MonitoredPump::runForPulses(uint32_t pulses, bool fulldiagnostics)  {
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
    for(size_t i=0; i<lookahead_; ++i){
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
            diagnostics_.pulseTimes.push_back(micros() - (peakDetector.getCenterIndexOffset() * intervalMs * 1000));//roughly
            
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
                int index = diagnostics_.isPulse.size() - peakDetector.getCenterIndexOffset();
                if(index >= 0 && index < diagnostics_.isPulse.size())
                    diagnostics_.isPulse[index] = true;
            }
            if(trough){
                int index = diagnostics_.isPulse.size() - troughDetector.getCenterIndexOffset();
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

bool MonitoredPump::runForMl(float ml, bool fulldiagnostics)  {
    //calculate the number of pulses needed
    float pulsesNeeded = ml * pulsesPerMl_;
    //if this is too low, return an empty diagnostics object
    if(!volumeSupported(ml)){
        return false;
    }
    return runForPulses(pulsesNeeded, fulldiagnostics);
}