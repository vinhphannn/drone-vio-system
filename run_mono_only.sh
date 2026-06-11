#!/bin/bash
# ==============================================================================
# run_mono_only.sh - Test THUẦN Visual (không IMU) để kiểm tra T_b_c1
# MỤC ĐÍCH: Nếu Mono-only ổn định → lỗi nằm ở T_b_c1/IMU calibration
# ==============================================================================

trap 'echo -e "\nĐang dọn dẹp..."; kill $DRONE_PID 2>/dev/null; exit 0' SIGINT SIGTERM EXIT

echo "================================================="
echo "   TEST MONO-ONLY (KHÔNG IMU) — Chẩn đoán T_b_c1"
echo "================================================="
echo ""
echo "[!] Mục đích: Nếu tracking ổn định ở đây → T_b_c1 calibration sai"
echo "    Quan sát: inliers nên luôn > 30 khi di chuyển chậm"
echo ""

cd ~/ros2_ws
source install/setup.bash

echo "[1/2] Khởi động drone_core (camera driver)..."
pkill -f drone_core 2>/dev/null
pkill -f "orbslam3" 2>/dev/null
sleep 1
ros2 run drone_core drone_core > /dev/null 2>&1 &
DRONE_PID=$!
sleep 2

echo "[2/2] Khởi động ORB-SLAM3 Monocular (KHÔNG IMU)..."
echo "      Subscribe: /camera/image_raw"
echo "      Vocab: src/ORB_SLAM3/Vocabulary/ORBvoc.txt"
echo ""
ros2 run orbslam3 mono \
    src/ORB_SLAM3/Vocabulary/ORBvoc.txt \
    orb_slam3_config.yaml
