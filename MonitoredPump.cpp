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

