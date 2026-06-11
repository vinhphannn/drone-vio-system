/**
 * @file types.h
 * @brief Shared data types for the drone_core VIO pipeline.
 *
 * Data flow:
 *   STM32 (USB CDC, binary) ──► ImuDriver ──► ImuPacket ──► ImuRingBuffer ──► VioEngine
 *   Camera (V4L2)           ──► CameraDriver ──► CameraFrame ──► CamRingBuffer ──► VioEngine
 *                                                                               └──► Pose (output)
 */
#pragma once
#include <cstdint>
#include <vector>

namespace drone {

// =============================================================================
// IMU Types
// =============================================================================

/**
 * @brief Binary frame layout transmitted by STM32 over USB CDC.
 *
 * 18 bytes, packed (no padding). Header is 0xAA55 (little-endian: 0x55, 0xAA).
 * Raw int16 values — unit conversion is done in ImuDriver, NOT on the MCU.
 *
 * BMI160 register layout (burst read from 0x0C, 12 bytes):
 *   [0x0C..0x11] = Gyro  X,Y,Z  (GYR_DATA)
 *   [0x12..0x17] = Accel X,Y,Z  (ACC_DATA)
 */
#pragma pack(push, 1)
struct StmRawFrame {
    uint16_t header;       ///< Always 0xAA55 — used for stream sync
    uint32_t timestamp_us; ///< STM32 hardware timer, wraps at ~4295s (uint32 max μs)
    int16_t  gx;           ///< Gyro  X [LSB], scale: 16.4 LSB/(°/s) at ±2000°/s range
    int16_t  gy;           ///< Gyro  Y [LSB]
    int16_t  gz;           ///< Gyro  Z [LSB]
    int16_t  ax;           ///< Accel X [LSB], scale: 8192 LSB/g    at ±4g range
    int16_t  ay;           ///< Accel Y [LSB]
    int16_t  az;           ///< Accel Z [LSB]
};
#pragma pack(pop)
static_assert(sizeof(StmRawFrame) == 18, "StmRawFrame size mismatch");

/** BMI160 scale factors — do NOT change without updating firmware register config */
static constexpr float IMU_ACCEL_SCALE = (9.80665f / 8192.0f) * 1.06136f; ///< [m/s² per LSB]
static constexpr float IMU_GYRO_SCALE  = (3.14159265f / 180.0f) / 16.4f;  ///< [rad/s per LSB]

/**
 * @brief Processed IMU measurement ready for VIO consumption.
 *
 * All values are in SI units. Timestamp is anchored to steady_clock
 * on first received packet and tracked via hardware counter thereafter.
 */
struct ImuMeasurement {
    uint64_t timestamp_ns; ///< Nanoseconds since steady_clock epoch

    float accel_x;  ///< [m/s²]  Body-frame X
    float accel_y;  ///< [m/s²]  Body-frame Y
    float accel_z;  ///< [m/s²]  Body-frame Z

    float gyro_x;   ///< [rad/s] Body-frame X (roll rate)
    float gyro_y;   ///< [rad/s] Body-frame Y (pitch rate)
    float gyro_z;   ///< [rad/s] Body-frame Z (yaw rate)
};

// =============================================================================
// Camera Types
// =============================================================================

/**
 * @brief A single grayscale camera frame with hardware-aligned timestamp.
 *
 * Timestamp is captured immediately after cap.read() returns, minimizing
 * the gap between shutter and software timestamp (~0.5ms typical).
 *
 * VIO REQUIRES this timestamp to correctly bound the IMU integration window.
 * Without it, IMU preintegration will accumulate significant error.
 */
struct CameraFrame {
    uint64_t             timestamp_ns; ///< Capture time [ns], from steady_clock
    int                  width;        ///< Image width  [pixels]
    int                  height;       ///< Image height [pixels]
    std::vector<uint8_t> data;         ///< Row-major, mono8 (1 byte/pixel)
};

// =============================================================================
// VIO Output Types
// =============================================================================

/**
 * @brief 6-DOF pose output from the VIO engine.
 *
 * Coordinate convention: NED (North-East-Down) body frame relative to
 * world origin at first keyframe. Quaternion uses Hamilton convention [w,x,y,z].
 *
 * `valid` is false during VIO initialization or when tracking is lost.
 * Consumers MUST check `valid` before using position/orientation values.
 */
struct Pose {
    uint64_t timestamp_ns; ///< Corresponds to the camera frame timestamp

    double px, py, pz;     ///< Position [m] in world frame
    double qw, qx, qy, qz; ///< Orientation quaternion [Hamilton, unit quaternion]

    bool valid;            ///< False during init or tracking failure — do NOT use pose if false
};

} // namespace drone
