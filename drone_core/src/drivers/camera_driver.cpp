// =============================================================================
// camera_driver.cpp
// Captures grayscale frames from USB camera via V4L2 and pushes to ring buffer.
// =============================================================================
#include "drone_core/drivers/camera_driver.h"

#include <chrono>
#include <iostream>
#include <pthread.h>
#include <stdexcept>
#include <time.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace drone {

CameraDriver::CameraDriver(const CameraDriverConfig& cfg, CamRingBuffer& out_ring)
    : cfg_(cfg), ring_(out_ring) {}

CameraDriver::~CameraDriver() { stop(); }

// -----------------------------------------------------------------------------
// Public interface
// -----------------------------------------------------------------------------

void CameraDriver::start() {
    // Open device with V4L2 backend (more reliable FOURCC/FPS control than GStreamer)
    cap_.open(cfg_.device_index, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        throw std::runtime_error(
            "[CameraDriver] Cannot open /dev/video" + std::to_string(cfg_.device_index) +
            "\n  Hint: check 'ls /dev/video*', ensure no other process holds the device"
        );
    }

    // Request MJPEG compression: ~10x less USB bandwidth vs raw YUYV
    cap_.set(cv::CAP_PROP_FOURCC,     cv::VideoWriter::fourcc('M','J','P','G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH,  cfg_.width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.height);
    cap_.set(cv::CAP_PROP_FPS,          cfg_.fps);

    // Limit V4L2 internal buffer to 2 frames.
    // Default is 4 which adds up to ~133ms extra latency at 30fps!
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 2);

    // Disable Auto-Exposure to prevent severe frame queueing (buffer bloat) in low light
    // Note: V4L2 backend expects 1 for Manual, 3 for Auto.
    cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap_.set(cv::CAP_PROP_EXPOSURE, 156); // Set a fixed moderate exposure time

    int actual_w = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
    int actual_h = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

    if (actual_w != cfg_.width || actual_h != cfg_.height) {
        std::cerr << "[CameraDriver] WARNING: Requested " << cfg_.width << "x" << cfg_.height
                  << " but got " << actual_w << "x" << actual_h
                  << ". Update camera calibration accordingly.\n";
    }
    std::cout << "[CameraDriver] Opened device " << cfg_.device_index
              << " at " << actual_w << "x" << actual_h
              << " @ " << cfg_.fps << " fps (MJPEG, V4L2)\n";

    running_ = true;
    thread_  = std::thread(&CameraDriver::run_loop, this);
}

void CameraDriver::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (cap_.isOpened()) cap_.release();
}

// -----------------------------------------------------------------------------
// Capture thread
// -----------------------------------------------------------------------------

void CameraDriver::run_loop() {
    if (cfg_.cpu_core >= 0) {
#ifdef __linux__
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(static_cast<unsigned>(cfg_.cpu_core), &cpuset);
        pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
#endif
    }

    cv::Mat bgr, gray;
    int consecutive_failures = 0;

    while (running_) {
        // cap.read() blocks until a frame is available — natural rate limiting at camera FPS
        const bool ok = cap_.read(bgr);

        // Timestamp immediately after read() returns — closest to actual shutter time
        uint64_t ts_ns = static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count()
        );
        const uint64_t offset_ns = static_cast<uint64_t>(cfg_.time_offset_ms * 1e6);
        ts_ns = (ts_ns > offset_ns) ? (ts_ns - offset_ns) : ts_ns;

        if (!ok || bgr.empty()) {
            ++consecutive_failures;
            if (consecutive_failures == 5) {
                std::cerr << "[CameraDriver] WARNING: Frame read failed 5 times in a row\n";
            } else if (consecutive_failures >= 30) {
                std::cerr << "[CameraDriver] ERROR: Camera may have disconnected ("
                          << consecutive_failures << " failures)\n";
                consecutive_failures = 0; // Reset to avoid log spam
            }
            continue;
        }
        consecutive_failures = 0;

        // Convert BGR→Grayscale: VIO uses luminance only (saves ~66% memory/bandwidth)
        cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

        CameraFrame frame;
        frame.timestamp_ns = ts_ns;
        frame.width        = gray.cols;
        frame.height       = gray.rows;
        frame.data.assign(gray.datastart, gray.dataend);

        if (!ring_.push(frame)) {
            ++dropped_;
            // Warn every 100 drops — means VIO is slower than camera FPS
            if (dropped_ % 100 == 0) {
                std::cerr << "[CameraDriver] VIO can't keep up — "
                          << dropped_ << " frames dropped total\n";
            }
        }
        ++captured_;
    }
}

} // namespace drone
