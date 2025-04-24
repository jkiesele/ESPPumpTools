#ifndef PULSE_LOOKAHEAD_DETECTOR_H
#define PULSE_LOOKAHEAD_DETECTOR_H

#include "RingBuffer.h"

template<typename T, std::size_t Lookahead>
class PulseLookaheadDetector {
public:
    static constexpr std::size_t Capacity = 2 * Lookahead + 1;
    // The constructor sets up the ring buffer with a fixed capacity:
    // capacity = 2 * lookahead + 1.
    // this is a PEAK detector
    explicit PulseLookaheadDetector(bool invert=false)
        : buffer_(), invert(invert) {}

    // addSample:
    // Adds a new sample to the ring buffer. Once the buffer is full,
    // the center sample is always at index 'lookahead_' in the linearized view.
    // Returns true if a pulse is detected.
    bool addSample(T sample) {
        buffer_.push_back(sample);
        
        // wait until buffer is full
        if (buffer_.size() < Capacity) return false;
        
        // In the linearized view, the center sample is at index lookahead_
        constexpr std::size_t centerIndex = Lookahead;
        T centerValue = buffer_[centerIndex];
        
        // BACKWARD: Check that every sample before the center is not greater than centerValue.
        // (This mimics Python's: for j in range(i - lookahead, i + 1))
        for (size_t j = 0; j < centerIndex; ++j) {
            if(invert){
                if (buffer_[j] < centerValue)
                    return false;
            }else{
                if (buffer_[j] > centerValue)
                    return false;
            }
        }
        
        // FORWARD: Check that every sample after the center is strictly smaller than centerValue.
        // (This mimics Python's: for j in range(i + 1, i + lookahead + 1))
        for (size_t j = centerIndex + 1; j < buffer_.capacity(); ++j) {
            if(invert){
                if (buffer_[j] <= centerValue)
                    return false;
            }else{
                if (buffer_[j] >= centerValue)
                    return false;
            }
        }
        
        return true;
    }

    // Once addSample() returned true, you can fetch the center value:
    T getCenterValue() const {
        return buffer_[Lookahead];
    }
    
    // Since the center is always at index 'lookahead_' when the buffer is full,
    // the offset (in number of samples) from the end of the buffer is:
    // capacity - lookahead, which equals (2*lookahead + 1 - lookahead) = lookahead + 1.
    constexpr std::size_t centerOffset() const {
        return Lookahead;
    }

    // Clears the buffer.
    void clear() {
        buffer_.clear();
    }

private:
    RingBuffer<T, Capacity> buffer_;
    bool invert;
};

#endif // PULSE_LOOKAHEAD_DETECTOR_H