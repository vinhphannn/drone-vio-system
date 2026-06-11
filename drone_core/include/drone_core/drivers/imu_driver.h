/**
 * @file imu_driver.h
 * @brief IMU driver — reads BMI160 data from STM32 bridge over USB CDC.
 *
 * Protocol: 18-byte binary frames (StmRawFrame) at up to 1600 Hz.
 * Thread: runs on CPU core 1, pinned via pthread affinity.
 */
#pragma once
#include "drone_core/common/types.h"
#include "drone_core/common/ring_buffer.h"

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

namespace drone {

// 1600 Hz × 4 seconds of buffering, rounded to next power of 2
using ImuRingBuffer = RingBuffer<ImuMeasurement, 8192>;

/** @brief Runtime configuration for ImuDriver. */
struct ImuDriverConfig {
    std::string device  = "/dev/ttyACM0"; ///< USB CDC device (STM32 bridge)
    int         cpu_core = 1;             ///< CPU core affinity (-1 = no pinning)
};

/**
 * @brief Reads binary IMU frames from STM32 USB CDC, converts to SI units,
 *        anchors hardware timestamps to system clock, and pushes to ring buffer.
 *
 * Key design decisions:
 *  - Non-blocking serial read + 100μs sleep prevents busy-spinning.
 *  - Header sync (0xAA55) recovers automatically from USB framing errors.
 *  - Hardware timestamp offset computed once on first packet, then tracked
 *    purely via the STM32 counter — avoids OS scheduling jitter on subsequent samples.
 *  - Unit conversion (LSB → m/s², rad/s) done here once, not in VIO hot loop.
 */
class ImuDriver {
public:
    explicit ImuDriver(const ImuDriverConfig& cfg, ImuRingBuffer& out_ring);
    ~ImuDriver();

    void start(); ///< Open serial port and launch driver thread
    void stop();  ///< Signal thread to exit and join

    uint64_t received() const { return received_; }
    uint64_t dropped()  const { return dropped_;  }

private:
    int  open_serial() const;
    void run_loop();
    ImuMeasurement convert(const StmRawFrame& raw) const;

    ImuDriverConfig cfg_;
    ImuRingBuffer&  ring_;

    int         fd_      = -1;
    std::thread thread_;
    std::atomic<bool>     running_{false};
    std::atomic<uint64_t> received_{0};
    std::atomic<uint64_t> dropped_{0};

    mutable bool     ts_synced_    = false;
    mutable int64_t  ts_offset_ns_ = 0; ///< sys_ns - hw_ns at first packet
    mutable uint32_t last_hw_us_   = 0;
    mutable uint64_t hw_ns_upper_  = 0;
    mutable bool     hw_ts_init_   = false;
};

} // namespace drone
