#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuTestNode(Node):
    def __init__(self):
        super().__init__('imu_test_node')
        self.sub = self.create_subscription(Imu, '/imu/data', self.cb, 10)
        self.count = 0

    def cb(self, msg):
        if self.count % 100 == 0:
            print(f"Accel: x={msg.linear_acceleration.x:5.2f}, y={msg.linear_acceleration.y:5.2f}, z={msg.linear_acceleration.z:5.2f}")
        self.count += 1

def main():
    rclpy.init()
    node = ImuTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
