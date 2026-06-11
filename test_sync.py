import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
import time

class SyncTest(Node):
    def __init__(self):
        super().__init__('sync_test')
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_cb, 10)
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.img_cb, 10)
        self.last_imu = 0
        self.last_img = 0

    def imu_cb(self, msg):
        self.last_imu = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def img_cb(self, msg):
        self.last_img = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_imu > 0:
            print(f"Cam: {self.last_img:.3f}, IMU: {self.last_imu:.3f}, Diff: {self.last_img - self.last_imu:.3f} sec")

rclpy.init()
node = SyncTest()
rclpy.spin(node)
