#!/bin/bash
# ==============================================================================
# run_vio.sh - Trình khởi chạy hệ thống VIO (Xuất Log trực tiếp)
# ==============================================================================

# Dừng hệ thống an toàn nếu ấn Ctrl+C
trap 'echo -e "\nĐang dọn dẹp hệ thống..."; kill $DRONE_PID 2>/dev/null; exit 0' SIGINT SIGTERM EXIT

echo "================================================="
echo "   KÍCH HOẠT HỆ THỐNG OPENVINS & DRONE CORE      "
echo "================================================="

cd ~/ros2_ws
source install/setup.bash

echo "[1/2] Khởi động Hardware Synchronizer (drone_core) ngầm..."
ros2 run drone_core drone_core > /dev/null 2>&1 &
DRONE_PID=$!

sleep 1

echo "[2/2] Khởi động OpenVINS (ov_msckf)... [LOGS SẼ HIỂN THỊ TẠI ĐÂY]"
# Chạy Foreground để xem Log
ros2 launch ov_msckf subscribe.launch.py config:=zynh_drone rviz_enable:=true max_cameras:=1 use_stereo:=false
