import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
import sys

class ImuCalibrator(Node):
    def __init__(self):
        super().__init__('imu_calibrator')
        self.sub = self.create_subscription(Imu, '/imu/data', self.callback, 10)
        self.last_msg = None

    def callback(self, msg):
        self.last_msg = msg

    def print_state(self):
        if not self.last_msg:
            return
        ax = self.last_msg.linear_acceleration.x
        ay = self.last_msg.linear_acceleration.y
        az = self.last_msg.linear_acceleration.z
        
        # Clear screen
        sys.stdout.write("\033[H\033[J")
        print("======================================================")
        print("          CÔNG CỤ XÁC ĐỊNH TRỤC IMU TRỰC TIẾP         ")
        print("======================================================\n")
        print(f"Gia tốc hiện tại (m/s²):")
        print(f"   X = {ax:6.2f}")
        print(f"   Y = {ay:6.2f}")
        print(f"   Z = {az:6.2f}\n")
        
        gx = self.last_msg.angular_velocity.x
        gy = self.last_msg.angular_velocity.y
        gz = self.last_msg.angular_velocity.z

        print("HƯỚNG DẪN KIỂM TRA QUAY (GYRO):")
        print("1. Hãy xoay vòng drone sang TRÁI (như cái đĩa quay vòng ngược chiều kim đồng hồ).")
        print("2. Nhìn xem Vận tốc góc Z (Yaw) báo Dương (+) hay Âm (-) ?\n")
        print(f"Vận tốc góc hiện tại (rad/s):")
        print(f"   X (Roll)  = {gx:6.2f}")
        print(f"   Y (Pitch) = {gy:6.2f}")
        print(f"   Z (Yaw)   = {gz:6.2f}\n")

        print("\nBấm Ctrl+C để thoát.")

def main():
    rclpy.init()
    node = ImuCalibrator()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.print_state()
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
