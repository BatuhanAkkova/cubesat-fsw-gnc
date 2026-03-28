#pragma once

#include <atomic>
#include <cstddef>
#include <memory>
#include <vector>

namespace fsw {
namespace core {

/**
 * @brief A high-performance, lock-free Single-Producer Single-Consumer (SPSC) queue.
 *
 * Uses a fixed-size circular buffer and atomic memory ordering (acquire/release)
 * for maximum performance. Cache-line padding is used to prevent false sharing.
 *
 * @tparam T The type of message to store in the queue.
 * @tparam Capacity The maximum number of elements in the queue (must be power of 2 for optimization).
 */
template <typename T, size_t Capacity = 1024>
class SPSCQueue {
   public:
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of 2");

    SPSCQueue() : head_(0), tail_(0) {}

    // Prevent copies and moves for safety in multi-threaded contexts
    SPSCQueue(const SPSCQueue&) = delete;
    SPSCQueue& operator=(const SPSCQueue&) = delete;

    /**
     * @brief Push an item into the queue.
     * @return true if successful, false if queue is full.
     */
    bool push(const T& item) {
        const size_t current_tail = tail_.load(std::memory_order_relaxed);
        const size_t next_tail = (current_tail + 1) & (Capacity - 1);

        if (next_tail == head_.load(std::memory_order_acquire)) {
            return false;  // Queue full
        }

        buffer_[current_tail] = item;
        tail_.store(next_tail, std::memory_order_release);
        return true;
    }

    /**
     * @brief Pop an item from the queue.
     * @return true if successful, false if queue is empty.
     */
    bool pop(T& out_item) {
        const size_t current_head = head_.load(std::memory_order_relaxed);

        if (current_head == tail_.load(std::memory_order_acquire)) {
            return false;  // Queue empty
        }

        out_item = buffer_[current_head];
        head_.store((current_head + 1) & (Capacity - 1), std::memory_order_release);
        return true;
    }

    /**
     * @brief Check if the queue is empty.
     */
    bool empty() const {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    /**
     * @brief Returns the approximate number of elements in the queue.
     */
    size_t size() const {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        if (tail >= head) return tail - head;
        return Capacity - (head - tail);
    }

   private:
    // Alignment to prevent false sharing (typical cache line is 64 bytes)
    static constexpr size_t kCacheLineSize = 64;

    alignas(kCacheLineSize) std::atomic<size_t> head_;
    alignas(kCacheLineSize) std::atomic<size_t> tail_;

    // Buffer is also aligned for SIMD friendliness if T is a vector type
    alignas(kCacheLineSize) T buffer_[Capacity];
};

}  // namespace core
}  // namespace fsw
