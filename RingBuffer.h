#pragma once
#include <array>
#include <cstddef>
#include <stdexcept>

template<typename T, std::size_t Capacity>
class RingBuffer {
public:
    // Constructs an empty ring buffer.
    RingBuffer() : start_(0), count_(0) {}

    // Adds an element. When full, overwrites the oldest.
    void push_back(const T& value) {
        if (count_ < Capacity) {
            buffer_[(start_ + count_) % Capacity] = value;
            ++count_;
        } else {
            buffer_[start_] = value;
            start_ = (start_ + 1) % Capacity;
        }
    }

    // Clears the buffer.
    void clear() {
        start_ = 0;
        count_ = 0;
    }

    // Number of stored elements.
    std::size_t size() const { return count_; }

    // Maximum capacity.
    constexpr std::size_t capacity() const { return Capacity; }

    // Random-access in logical order (0 = oldest).
    T& operator[](std::size_t i) {
        if (i >= count_) throw std::out_of_range("RingBuffer index out of range");
        return buffer_[(start_ + i) % Capacity];
    }
    const T& operator[](std::size_t i) const {
        if (i >= count_) throw std::out_of_range("RingBuffer index out of range");
        return buffer_[(start_ + i) % Capacity];
    }
    T& back() {
        return (*this)[count_ - 1];
    }
    const T& back() const {
        return (*this)[count_ - 1];
    }

    // Copy out in linear order (oldest→newest).
    template<std::size_t N = Capacity>
    void getLinearized(std::array<T, N>& out) const {
        static_assert(N >= Capacity, "Output array too small");
        for (std::size_t i = 0; i < count_; ++i) {
            out[i] = (*this)[i];
        }
    }

private:
    std::array<T, Capacity> buffer_;
    std::size_t             start_;  // index of oldest
    std::size_t             count_;  // how many stored
};