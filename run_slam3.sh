#!/bin/bash
# ==============================================================================
# run_slam3.sh - Trình khởi chạy hệ thống ORB-SLAM3 (Camera + IMU)
# ==============================================================================

# Dừng hệ thống an toàn nếu ấn Ctrl+C
trap 'echo -e "\nĐang dọn dẹp hệ thống..."; kill $DRONE_PID 2>/dev/null; exit 0' SIGINT SIGTERM EXIT

echo "================================================="
echo "   KÍCH HOẠT HỆ THỐNG ORB-SLAM3 & DRONE CORE     "
echo "================================================="

cd ~/ros2_ws
source install/setup.bash

echo "[1/2] Dọn dẹp tiến trình cũ và Khởi động Hardware Synchronizer..."
pkill -f drone_core 2>/dev/null
pkill -f mono-inertial 2>/dev/null
sleep 1
ros2 run drone_core drone_core > /dev/null 2>&1 &
DRONE_PID=$!

sleep 1

echo "[2/2] Khởi động ORB-SLAM3 (Mono-Inertial)... [CỬA SỔ 3D SẼ BẬT LÊN]"
# Chạy Foreground để xem tiến trình SLAM
ros2 run orbslam3 mono-inertial \
    src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    orb_slam3_config.yaml \
    false \
    false
