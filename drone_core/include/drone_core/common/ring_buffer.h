/**
 * @file ring_buffer.h
 * @brief Lock-free Single-Producer Single-Consumer (SPSC) ring buffer.
 *
 * Design rationale:
 *   - No mutex, no condition variable → no blocking, no priority inversion.
 *   - Uses std::atomic with acquire/release memory ordering for ARM multi-core.
 *   - Cache-line aligned (64 bytes) head/tail to prevent false sharing.
 *   - N must be a power of 2 → bitmask replaces modulo division (faster).
 *
 * Thread safety:
 *   - Exactly ONE writer thread may call push().
 *   - Exactly ONE reader thread may call pop() / peek().
 *   - DO NOT share across more than one producer or more than one consumer.
 *
 * Usage:
 *   @code
 *   RingBuffer<ImuMeasurement, 4096> imu_buf;
 *
 *   // Producer thread (IMU driver):
 *   imu_buf.push(measurement);
 *
 *   // Consumer thread (VIO engine):
 *   ImuMeasurement m;
 *   if (imu_buf.peek(m) && m.timestamp_ns <= frame_ts) {
 *       imu_buf.pop(m);
 *       process(m);
 *   }
 *   @endcode
 */
#pragma once
#include <array>
#include <atomic>
#include <cstddef>

namespace drone {

template <typename T, std::size_t N>
class RingBuffer {
    static_assert(N >= 2,           "N must be at least 2");
    static_assert((N & (N - 1)) == 0, "N must be a power of 2");

public:
    RingBuffer() : head_(0), tail_(0) {}

    // Non-copyable, non-movable (atomics cannot be copied)
    RingBuffer(const RingBuffer&)            = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;

    // -------------------------------------------------------------------------
    // Producer API (call only from the single writer thread)
    // -------------------------------------------------------------------------

    /**
     * @brief Push one item into the buffer.
     * @return true on success, false if the buffer is full (item is dropped).
     *
     * On drop: the oldest item is NOT overwritten — the new item is discarded.
     * This preserves data integrity at the cost of dropping the newest sample.
     */
    bool push(const T& item) noexcept {
        const std::size_t h    = head_.load(std::memory_order_relaxed);
        const std::size_t next = (h + 1) & MASK;

        if (next == tail_.load(std::memory_order_acquire)) {
            return false; // Buffer full — drop
        }

        buf_[h] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    // -------------------------------------------------------------------------
    // Consumer API (call only from the single reader thread)
    // -------------------------------------------------------------------------

    /**
     * @brief Peek at the oldest item without consuming it.
     * @param[out] item Receives the item value if available.
     * @return true if an item was available, false if buffer is empty.
     *
     * Use peek() + pop() together when you need to decide based on the value
     * whether to consume it (e.g., IMU timestamp boundary check in VIO).
     */
    bool peek(T& item) const noexcept {
        const std::size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) {
            return false; // Empty
        }
        item = buf_[t];
        return true;
    }

    /**
     * @brief Consume (remove) the oldest item from the buffer.
     * @param[out] item Receives the item value.
     * @return true on success, false if buffer is empty.
     */
    bool pop(T& item) noexcept {
        const std::size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) {
            return false; // Empty
        }
        item = buf_[t];
        tail_.store((t + 1) & MASK, std::memory_order_release);
        return true;
    }

    /** @brief Number of items currently in the buffer. */
    std::size_t size() const noexcept {
        const std::size_t h = head_.load(std::memory_order_acquire);
        const std::size_t t = tail_.load(std::memory_order_acquire);
        return (h - t) & MASK;
    }

    bool empty() const noexcept { return size() == 0; }
    bool full()  const noexcept { return size() == N - 1; }

private:
    static constexpr std::size_t MASK = N - 1;

    alignas(64) std::array<T, N>      buf_;   // Data storage
    alignas(64) std::atomic<std::size_t> head_; // Write pointer (producer owns)
    alignas(64) std::atomic<std::size_t> tail_; // Read  pointer (consumer owns)
};

} // namespace drone
