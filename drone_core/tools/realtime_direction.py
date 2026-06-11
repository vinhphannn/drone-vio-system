#!/usr/bin/env python3
import time
import os
import sys

LOG_FILE = "/tmp/drone_calib.csv"

def tail(file_path):
    try:
        with open(file_path, "r") as f:
            f.seek(0, os.SEEK_END)
            while True:
                line = f.readline()
                if not line:
                    time.sleep(0.05)
                    continue
                yield line
    except FileNotFoundError:
        print(f"[ERROR] Cannot find {file_path}")
        sys.exit(1)

def main():
    print("╔══════════════════════════════════════════════════════════╗")
    print("║          REAL-TIME SENSOR DIRECTION MONITOR              ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print("Waiting for data...\n")

    # Cân bằng lại ngưỡng (Threshold): 0.5 rad/s tương đương khoảng 8-10 pixels/frame
    GYRO_THRESH = 0.5    # rad/s
    FLOW_THRESH = 8.0    # pixels

    for line in tail(LOG_FILE):
        try:
            parts = line.strip().split(',')
            if len(parts) < 6 or parts[0] == "timestamp_ns":
                continue
            
            gx, gy, gz = float(parts[1]), float(parts[2]), float(parts[3])
            dx, dy = float(parts[4]), float(parts[5])

            imu_yaw, cam_yaw = "", ""
            imu_pitch, cam_pitch = "", ""
            imu_roll = ""

            # --- YAW ---
            if gz > GYRO_THRESH: imu_yaw = "Trái"
            elif gz < -GYRO_THRESH: imu_yaw = "Phải"

            if dx > FLOW_THRESH: cam_yaw = "Trái"
            elif dx < -FLOW_THRESH: cam_yaw = "Phải"

            # --- PITCH ---
            if gy > GYRO_THRESH: imu_pitch = "Lên"
            elif gy < -GYRO_THRESH: imu_pitch = "Xuống"

            if dy > FLOW_THRESH: cam_pitch = "Lên"
            elif dy < -FLOW_THRESH: cam_pitch = "Xuống"

            # --- ROLL ---
            if gx > GYRO_THRESH: imu_roll = "Nghiêng Phải"
            elif gx < -GYRO_THRESH: imu_roll = "Nghiêng Trái"

            # Build display strings
            imu_parts = []
            if imu_yaw: imu_parts.append(f"Xoay {imu_yaw}")
            if imu_pitch: imu_parts.append(f"Ngẩng {imu_pitch}" if imu_pitch == "Lên" else f"Chúi {imu_pitch}")
            if imu_roll and not imu_parts: imu_parts.append(imu_roll)
            
            cam_parts = []
            if cam_yaw: cam_parts.append(f"Xoay {cam_yaw}")
            if cam_pitch: cam_parts.append(f"Ngẩng {cam_pitch}" if cam_pitch == "Lên" else f"Chúi {cam_pitch}")

            imu_str = " + ".join(imu_parts) if imu_parts else "Đứng yên"
            cam_str = " + ".join(cam_parts) if cam_parts else "Đứng yên"

            # Logic báo LỆCH hay KHỚP (Chỉ so sánh các trục có chuyển động)
            match = True
            
            if not imu_yaw and not cam_yaw and not imu_pitch and not cam_pitch:
                continue # Bỏ qua nếu cả 2 đều đứng yên thật sự
                
            if imu_yaw and cam_yaw and imu_yaw != cam_yaw: match = False
            if imu_pitch and cam_pitch and imu_pitch != cam_pitch: match = False
            
            # Nếu 1 thằng báo có chuyển động mạnh (vượt threshold) mà thằng kia Đứng yên hoàn toàn -> Lệch
            if (imu_yaw or imu_pitch) and not (cam_yaw or cam_pitch): match = False
            if (cam_yaw or cam_pitch) and not (imu_yaw or imu_pitch): match = False

            status = "✅ KHỚP" if match else "❌ LỆCH"
            
            # Định dạng In ra cột cho đẹp
            print(f"IMU: {imu_str:<25} | CAM: {cam_str:<25} | {status}")

        except ValueError:
            continue
        except KeyboardInterrupt:
            break

if __name__ == "__main__":
    main()
