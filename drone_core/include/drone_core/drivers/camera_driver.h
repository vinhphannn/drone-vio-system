/**
 * @file camera_driver.h
 * @brief Camera driver — captures grayscale frames from V4L2 USB camera.
 *
 * Thread: runs on CPU core 2, pinned via pthread affinity.
 * Output: mono8 CameraFrame pushed to CamRingBuffer at configured FPS.
 */
#pragma once
#include "drone_core/common/types.h"
#include "drone_core/common/ring_buffer.h"

#include <atomic>
#include <string>
#include <thread>
#include <opencv2/videoio.hpp>

namespace drone {

// Small buffer: VIO processes at camera FPS, so 4 slots is enough.
// Larger buffer = more latency, NOT better performance.
using CamRingBuffer = RingBuffer<CameraFrame, 4>;

/** @brief Runtime configuration for CameraDriver. */
struct CameraDriverConfig {
    int    device_index = 0;    ///< V4L2 device index (/dev/video<N>)
    int    width        = 640;  ///< Requested capture width  [pixels]
    int    height       = 480;  ///< Requested capture height [pixels]
    double fps          = 30.0; ///< Requested frame rate [Hz]
    int    cpu_core     = 2;    ///< CPU core affinity (-1 = no pinning)
    double time_offset_ms = 0.0;///< Time compensation: true_time = stamp - offset
};

/**
 * @brief Captures frames via V4L2 (MJPEG), converts to grayscale,
 *        stamps with steady_clock, and pushes to ring buffer.
 *
 * Key design decisions:
 *  - cap.read() is naturally blocking — no explicit sleep needed, prevents CPU waste.
 *  - Timestamp captured immediately after cap.read() returns — closest achievable
 *    approximation to hardware shutter time in software-only capture.
 *  - V4L2 internal buffer limited to 2 frames (CAP_PROP_BUFFERSIZE=2) to reduce
 *    the "stale frame" latency that V4L2 default buffering can cause.
 *  - Converts BGR→Grayscale immediately: VIO only needs luminance.
 */
class CameraDriver {
public:
    explicit CameraDriver(const CameraDriverConfig& cfg, CamRingBuffer& out_ring);
    ~CameraDriver();

    void start(); ///< Open camera and launch capture thread
    void stop();  ///< Signal thread to exit and join

    uint64_t captured() const { return captured_; }
    uint64_t dropped()  const { return dropped_;  }

private:
    void run_loop();

    CameraDriverConfig cfg_;
    CamRingBuffer&     ring_;

    cv::VideoCapture      cap_;
    std::thread           thread_;
    std::atomic<bool>     running_{false};
    std::atomic<uint64_t> captured_{0};
    std::atomic<uint64_t> dropped_{0};
};

} // namespace drone
