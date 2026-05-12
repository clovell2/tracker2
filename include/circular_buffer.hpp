#pragma once
#include <vector>
#include <stdexcept>
#include <mutex>

template <typename T>
class CircularBuffer {
public:
    explicit CircularBuffer(size_t capacity)
        : buffer(capacity), head(0), tail(0), size_(0), capacity_(capacity) {}

    // Add element to buffer (overwrite oldest if full)
    void push(const T& item) {
        const std::lock_guard<std::mutex> lock(buffer_mutex);
        buffer[tail] = item;
        tail = (tail + 1) % capacity_;

        if (size_ == capacity_) {
            // overwrite: move head forward
            head = (head + 1) % capacity_;
        } else {
            ++size_;
        }
    }

    // Remove and return oldest element
    T pop() {
        const std::lock_guard<std::mutex> lock(buffer_mutex);
        if (empty()) {
            throw std::out_of_range("Buffer is empty");
        }

        T item = buffer[head];
        head = (head + 1) % capacity_;
        --size_;
        return item;
    }

    // Access oldest element without removing
    const T& front() const {
        if (empty()) {
            throw std::out_of_range("Buffer is empty");
        }
        return buffer[head];
    }

    // Access newest element
    const T& back() const {
        if (empty()) {
            throw std::out_of_range("Buffer is empty");
        }
        size_t index = (tail + capacity_ - 1) % capacity_;
        return buffer[index];
    }

    bool empty() const {
        return size_ == 0;
    }

    bool full() const {
        return size_ == capacity_;
    }

    size_t size() const {
        return size_;
    }

    size_t capacity() const {
        return capacity_;
    }

    void clear() {
        head = tail = size_ = 0;
    }

private:
    std::vector<T> buffer;
    size_t head;     // points to oldest element
    size_t tail;     // points to next write position
    size_t size_;
    size_t capacity_;
    std::mutex buffer_mutex;
};