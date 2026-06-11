<div align="center">
  <h1>🚁 Drone Monocular-Inertial Navigation System (VIO)</h1>
  <p>
    <strong>A robust, real-time Visual-Inertial Odometry pipeline for autonomous UAVs using ORB-SLAM3 and ROS 2.</strong>
  </p>
  
  [![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/index.html)
  [![C++17](https://img.shields.io/badge/C++-17-00599C?logo=c%2B%2B)](https://en.cppreference.com/w/cpp/17)
  [![ORB-SLAM3](https://img.shields.io/badge/SLAM-ORB--SLAM3-orange)]()
  [![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
  [![Build Status](https://github.com/zynh/drone-vio-system/actions/workflows/ci.yml/badge.svg)](https://github.com/zynh/drone-vio-system/actions)

  <br />
</div>

## 📖 Overview

This repository contains a complete, low-latency Visual-Inertial Odometry (VIO) system designed for autonomous drone navigation in GPS-denied environments. By tightly coupling a custom STM32-based BMI160 IMU driver with the ORB-SLAM3 state-of-the-art Monocular-Inertial pipeline running on ROS 2 Humble, the system achieves drift-free, real-world metric scale localization.

**Key Achievements:**
- **Custom Hardware Synchronization:** Developed a multi-threaded C++ driver (`drone_core`) to fetch 1.6kHz IMU data over USB CDC with sub-millisecond hardware timestamp synchronization.
- **Robust VIO Fusion:** Overcame critical IMU/Camera extrinsic calibration (`T_b_c1`) and rotation coordinate frame (ROS FLU vs. OpenCV) mismatches to achieve continuous tracking without map resets for >600 frames.
- **RTOS-Grade Latency Control:** Implemented lock-free ring buffers and core affinity pinning to guarantee real-time performance on Cortex-M7 and companion computers.

---

## 🛠️ System Architecture

1. **Hardware Layer:**
   - **Camera:** USB Monocular Camera (640x480 @ 30fps)
   - **IMU:** Bosch BMI160 running on an STM32 MCU
2. **Driver Layer (`drone_core`):**
   - Reads binary IMU frames, maps to **ROS FLU** (Forward-Left-Up) frame.
   - Interpolates hardware timestamps to match the system clock, preventing OS scheduling jitter.
3. **SLAM Layer (`orbslam3`):**
   - Employs **ORB-SLAM3** in Monocular-Inertial mode.
   - Computes pre-integrated IMU measurements combined with ORB feature matching via Local Mapping and IMU Initialization logic.

---

## 🚀 Getting Started

### Prerequisites

- **OS:** Ubuntu 22.04 LTS
- **Middleware:** ROS 2 Humble
- **Dependencies:** OpenCV 4, Eigen3, Pangolin, Boost

### Installation & Build

1. **Clone the repository:**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/drone-vio-system.git .
   ```

2. **Install ROS 2 dependencies:**
   ```bash
   cd ~/ros2_ws
   rosdep install -i --from-path src --rosdistro humble -y
   ```

3. **Build the workspace:**
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

### Running the Pipeline

To launch the complete Monocular-Inertial pipeline with real-time visualization:

```bash
# Ensure your USB camera and STM32 IMU are connected
./run_slam3.sh
```

*Note: For the IMU to initialize correctly, you must perform a slow "figure-8" motion with the device (amplitude > 20cm) for at least 10 seconds immediately after startup to allow the visual-inertial bundle adjustment (VIBA) to converge.*

---

## 🧠 Engineering & Debugging Highlights

Throughout the development of this project, several complex issues typical of VIO systems were identified and resolved:

- **Map Reset Anomaly Resolved:** Diagnosed a persistent issue where `PoseInertialOptimization` caused a 90% drop in inliers (48 → 5). Injected custom diagnostic logging into `Tracking.cc` and isolated the root cause to a `[0, 0, 0]` translation matrix in the extrinsic calibration.
- **Coordinate Frame Alignment:** Corrected the rotation matrix mapping the Camera (OpenCV: X-Right, Y-Down, Z-Forward) to the IMU Body (ROS FLU: X-Forward, Y-Left, Z-Up).
- **Read the full debugging write-up:** [debug_imu_slam3_kinh_nghiem.md](./debug_imu_slam3_kinh_nghiem.md)

---

## 📝 License

This project is licensed under the MIT License - see the LICENSE file for details. Note that ORB-SLAM3 is released under GPLv3.

---
*Built with ❤️ for autonomous flight.*
