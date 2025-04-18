
template<typename T>
class RingBuffer {
public:
    // Constructs a ring buffer with fixed capacity.
    explicit RingBuffer(size_t capacity)
        : capacity_(capacity), buffer_(capacity), start_(0), count_(0) {}

    // Adds an element to the buffer. When full, overwrites the oldest element.
    void push_back(const T& value) {
        if (count_ < capacity_) {
            buffer_[count_++] = value;
        } else {
            buffer_[start_] = value;
            start_ = (start_ + 1) % capacity_;
        }
    }

    // Clears the buffer.
    void clear() {
        start_ = 0;
        count_ = 0;
    }

    // Returns the current number of stored elements.
    size_t size() const { return count_; }

    // Returns the fixed capacity of the buffer.
    size_t capacity() const { return capacity_; }

    // Random-access operator in "logical" order:
    // Index 0 is the oldest element, index size()-1 is the most recent.
    T& operator[](size_t index) {
        if (index >= count_) {
            throw std::out_of_range("RingBuffer index out of range");
        }
        return buffer_[(start_ + index) % capacity_];
    }

    const T& operator[](size_t index) const {
        if (index >= count_) {
            throw std::out_of_range("RingBuffer index out of range");
        }
        return buffer_[(start_ + index) % capacity_];
    }

    // Optionally, retrieve a linearized copy of the current contents.
    void getLinearized(std::vector<T>& out) const {
        out.resize(count_);
        for (size_t i = 0; i < count_; ++i) {
            out[i] = (*this)[i];
        }
    }

private:
    size_t capacity_;
    std::vector<T> buffer_;
    size_t start_;  // Index of the oldest element
    size_t count_;  // Number of elements currently stored
};