// Lockfree ring buffer
#pragma once
#include <stddef.h>
#include <array>
#include <atomic>

enum class PacketSource : uint8_t {
    UDP    = 0,
    Serial = 1
};

struct PacketEnvelope { // type erased since conflict with packet.hpp #TODO change name
    std::array<uint8_t, 256> data;
    size_t length;
    PacketSource source;
};

// Ring buffer template class
template <typename T, size_t Capacity>
class RingBuffer {
    private:
        std::array<T, Capacity> buffer;
        std::atomic<size_t> head{0};
        std::atomic<size_t> tail{0};

    public:
        bool push(const T& item) {
            size_t current_head = head.load(std::memory_order_relaxed);
            size_t next_head    = (current_head + 1) % Capacity;

            if (next_head == tail.load(std::memory_order_acquire)) {
                return false; // Buffer is full
            }

            buffer[current_head] = item;
            std::atomic_thread_fence(std::memory_order_release);
            head.store(next_head, std::memory_order_relaxed);


            return true;
        }

        bool pop(T& item) {
            size_t current_tail = tail.load(std::memory_order_relaxed);

            if (current_tail == head.load(std::memory_order_acquire)) {
                return false; // Buffer is empty
            }

            item = buffer[current_tail];
            tail.store((current_tail + 1) % Capacity, std::memory_order_release);

            return true;
        }

        // Helpers
        size_t count() const {
            size_t current_head = head.load(std::memory_order_acquire);
            size_t current_tail = tail.load(std::memory_order_acquire);
            if (current_head >= current_tail) {
                return current_head - current_tail;
            } else {
                return Capacity - (current_tail - current_head);
            }
        }

        bool isEmpty() const {
            return head.load(std::memory_order_acquire) == tail.load(std::memory_order_acquire);
        }

        bool isFull() const {
            return (head.load(std::memory_order_acquire) + 1) % Capacity == tail.load(std::memory_order_acquire);
        }
};
