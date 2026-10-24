#pragma once
// Fixed-capacity event queue (ring buffer). No heap: safe to use every frame.

#include <cstddef>

namespace core {

enum class EventType { Quit, WindowResized, FocusLost, FocusGained };

struct Event {
    EventType type;
    int width = 0, height = 0;  // WindowResized: new drawable size in pixels
};

template <typename T, size_t Capacity>
class RingQueue {
public:
    // Returns false (and drops the item) when full.
    bool push(const T& item) {
        if (count_ == Capacity) return false;
        items_[(head_ + count_) % Capacity] = item;
        ++count_;
        return true;
    }
    bool pop(T& out) {
        if (count_ == 0) return false;
        out = items_[head_];
        head_ = (head_ + 1) % Capacity;
        --count_;
        return true;
    }
    size_t size() const { return count_; }
    bool empty() const { return count_ == 0; }

private:
    T items_[Capacity] = {};
    size_t head_ = 0, count_ = 0;
};

using EventQueue = RingQueue<Event, 64>;

} // namespace core
