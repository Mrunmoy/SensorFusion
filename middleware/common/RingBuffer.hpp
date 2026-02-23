#pragma once

#include <cstddef>

namespace sf {

// Lock-free single-producer single-consumer ring buffer.
// Uses N+1 slots internally so that full vs empty is distinguishable
// without a separate counter, allowing SPSC use without locks.
template<typename T, size_t N>
class RingBuffer {
    static_assert(N > 0, "RingBuffer capacity must be > 0");

public:
    bool push(const T& item) {
        size_t nextHead = advance(head_);
        if (nextHead == tail_) return false;  // full
        buf_[head_] = item;
        head_ = nextHead;
        return true;
    }

    bool pop(T& item) {
        if (tail_ == head_) return false;  // empty
        item = buf_[tail_];
        tail_ = advance(tail_);
        return true;
    }

    size_t count() const {
        if (head_ >= tail_)
            return head_ - tail_;
        return (N + 1) - tail_ + head_;
    }

    bool isFull() const {
        return advance(head_) == tail_;
    }

    bool isEmpty() const {
        return head_ == tail_;
    }

    size_t capacity() const {
        return N;
    }

private:
    T buf_[N + 1]{};
    size_t head_ = 0;
    size_t tail_ = 0;

    static size_t advance(size_t idx) {
        return (idx + 1) % (N + 1);
    }
};

} // namespace sf
