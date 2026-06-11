// =============================================================================
// imu_driver.cpp
// Reads binary IMU frames from STM32 via USB CDC and pushes to ring buffer.
// =============================================================================
#include "drone_core/drivers/imu_driver.h"

#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <pthread.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

namespace drone {

ImuDriver::ImuDriver(const ImuDriverConfig& cfg, ImuRingBuffer& out_ring)
    : cfg_(cfg), ring_(out_ring) {}

ImuDriver::~ImuDriver() { stop(); }

// -----------------------------------------------------------------------------
// Public interface
// -----------------------------------------------------------------------------

void ImuDriver::start() {
    fd_      = open_serial();
    running_ = true;
    thread_  = std::thread(&ImuDriver::run_loop, this);
}

void ImuDriver::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (fd_ >= 0) { close(fd_); fd_ = -1; }
}

// -----------------------------------------------------------------------------
// Serial port setup
// -----------------------------------------------------------------------------

int ImuDriver::open_serial() const {
    int fd = ::open(cfg_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        throw std::runtime_error(
            "[ImuDriver] Cannot open " + cfg_.device + ": " + strerror(errno) +
            "\n  Hint: check USB cable, run 'ls /dev/ttyACM*', add user to dialout group"
        );
    }

    // Raw mode — no line processing, non-blocking reads
    struct termios tty{};
    tcgetattr(fd, &tty);
    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cc[VMIN]  = 0;  // Return immediately even if 0 bytes
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIFLUSH);

    // Assert DTR + RTS — USB CDC on STM32 uses DTR to detect that the host
    // application is ready to receive. Without this, the STM32 CDC stack may
    // stop calling CDC_TransmitCplt after the first packet (leaving TxState=1).
    int modem_bits = TIOCM_DTR | TIOCM_RTS;
    if (ioctl(fd, TIOCMBIS, &modem_bits) < 0) {
        // Non-fatal: some virtual COM ports ignore DTR
        std::cerr << "[ImuDriver] WARN: Cannot assert DTR/RTS: " << strerror(errno) << "\n";
    }

    std::cout << "[ImuDriver] Opened " << cfg_.device << " (USB CDC, binary 18B frames, DTR asserted)\n";
    return fd;
}

// -----------------------------------------------------------------------------
// Driver thread
// -----------------------------------------------------------------------------

void ImuDriver::run_loop() {
    // Pin to dedicated CPU core to reduce scheduling jitter
    if (cfg_.cpu_core >= 0) {
#ifdef __linux__
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(static_cast<unsigned>(cfg_.cpu_core), &cpuset);
        pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
#endif
    }

    constexpr std::size_t FRAME_SZ = sizeof(StmRawFrame);
    uint8_t  buf[4096] = {};  // Large buffer to instantly drain OS USB buffer
    std::size_t buf_len = 0;

    auto    last_data_time = std::chrono::steady_clock::now();
    bool    watchdog_warned = false;

    while (running_) {

        // ── Read available bytes (non-blocking) ──────────────────────────────
        ssize_t n = ::read(fd_, buf + buf_len, sizeof(buf) - buf_len);

        if (n > 0) {
            buf_len += static_cast<std::size_t>(n);
            last_data_time  = std::chrono::steady_clock::now();
            watchdog_warned = false;
        } else {
            // No data — sleep 100μs to avoid busy-spinning (saves ~30% CPU)
            struct timespec ts { 0, 100'000L };
            nanosleep(&ts, nullptr);

            // Watchdog: warn if silent for > 2 seconds
            auto silent_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - last_data_time).count();
            if (!watchdog_warned && silent_ms > 2000) {
                std::cerr << "[ImuDriver] WARNING: No data for 2s. "
                          << "Check STM32 USB connection and firmware.\n";
                watchdog_warned = true;
            }
            continue;
        }

        // ── Frame sync + parse ───────────────────────────────────────────────
        // Look for header bytes 0x55 0xAA (little-endian 0xAA55) in the buffer.
        // This handles USB CDC framing errors and reconnects gracefully.
        while (buf_len >= FRAME_SZ) {

            // Search for header start
            std::size_t sync = buf_len; // "not found" sentinel
            for (std::size_t i = 0; i + 1 < buf_len; ++i) {
                if (buf[i] == 0x55u && buf[i + 1] == 0xAAu) {
                    sync = i;
                    break;
                }
            }

            if (sync == buf_len) {
                // No header found — discard all but the last byte
                if (buf_len > 1) { buf[0] = buf[buf_len - 1]; buf_len = 1; }
                break;
            }

            if (sync > 0) {
                // Garbage before header — skip it
                std::memmove(buf, buf + sync, buf_len - sync);
                buf_len -= sync;
                continue;
            }

            // Not enough bytes yet for a full frame
            if (buf_len < FRAME_SZ) break;

            // Copy and parse
            StmRawFrame raw;
            std::memcpy(&raw, buf, FRAME_SZ);

            if (raw.header == 0xAA55u) {
                ImuMeasurement m = convert(raw);
                if (!ring_.push(m)) {
                    ++dropped_;
                    if (dropped_ % 1000 == 0) {
                        std::cerr << "[ImuDriver] Ring buffer full — "
                                  << dropped_ << " packets dropped total\n";
                    }
                }
                ++received_;
            }

            // Consume the frame bytes from the buffer
            std::memmove(buf, buf + FRAME_SZ, buf_len - FRAME_SZ);
            buf_len -= FRAME_SZ;
        }
    }
}

// -----------------------------------------------------------------------------
// Unit conversion: raw int16 → SI float
// -----------------------------------------------------------------------------

ImuMeasurement ImuDriver::convert(const StmRawFrame& raw) const {
    // 1. Handle 32-bit hardware microsecond counter wrap-around
    if (!hw_ts_init_) {
        last_hw_us_  = raw.timestamp_us;
        hw_ts_init_  = true;
    }
    if (raw.timestamp_us < last_hw_us_) {
        // Wraps every ~71.5 minutes
        hw_ns_upper_ += (1ULL << 32);
    }
    last_hw_us_ = raw.timestamp_us;
    uint64_t hw_ns = (hw_ns_upper_ | raw.timestamp_us) * 1000ULL;

    // 2. Compute current offset (sys - hw)
    uint64_t sys_ns = static_cast<uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    int64_t current_offset = static_cast<int64_t>(sys_ns) - static_cast<int64_t>(hw_ns);

    if (!ts_synced_ || current_offset < ts_offset_ns_) {
        ts_offset_ns_ = current_offset;
        ts_synced_ = true;
    }

    ImuMeasurement m;
    m.timestamp_ns = static_cast<uint64_t>(static_cast<int64_t>(hw_ns) + ts_offset_ns_);

    // 1. Convert to raw floats
    float raw_ax = static_cast<float>(raw.ax) * IMU_ACCEL_SCALE;
    float raw_ay = static_cast<float>(raw.ay) * IMU_ACCEL_SCALE;
    float raw_az = static_cast<float>(raw.az) * IMU_ACCEL_SCALE;

    float raw_gx = static_cast<float>(raw.gx) * IMU_GYRO_SCALE;
    float raw_gy = static_cast<float>(raw.gy) * IMU_GYRO_SCALE;
    float raw_gz = static_cast<float>(raw.gz) * IMU_GYRO_SCALE;
    // 2. MAPPING RAW IMU AXES TO ROS FLU (Forward-Left-Up) FRAME
    // Correct mapping based on raw output of detect_axes.py:
    // Test 1: On desk -> raw_ax = +9.8. Since gravity normal force is UP, raw_ax points UP.
    // Test 2: Nose down -> raw_az = +9.8. Since nose is down, UP is BACKWARD. So raw_az points BACKWARD.
    // Therefore, physical chip orientation is: X=Up, Z=Backward, Y=Left.
    // To map to ROS FLU (Forward-Left-Up):
    // FLU X (Forward) = -Z (Forward) = -raw_az
    // FLU Y (Left) = Y = raw_ay
    // FLU Z (Up) = X = raw_ax
    // This perfectly satisfies Right Hand Rule: (-raw_az) x (raw_ay) = raw_ax.
    m.accel_x = -raw_az;
    m.accel_y = raw_ay;
    m.accel_z = raw_ax;

    m.gyro_x = -raw_gz;
    m.gyro_y = raw_gy;
    m.gyro_z = raw_gx;

    return m;
}

} // namespace drone
