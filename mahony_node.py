#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class MahonyFilter:
    def __init__(self, Kp=5.0, Ki=0.1, sample_freq=1600.0):
        self.Kp = Kp
        self.Ki = Ki
        self.dt = 1.0 / sample_freq
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) # q0, q1, q2, q3
        self.eInt = np.array([0.0, 0.0, 0.0])
        
    def update(self, gx, gy, gz, ax, ay, az):
        q0, q1, q2, q3 = self.q
        
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0.0:
            return
        ax /= norm; ay /= norm; az /= norm
        
        vx = 2.0 * (q1 * q3 - q0 * q2)
        vy = 2.0 * (q0 * q1 + q2 * q3)
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
        
        ex = (ay * vz - az * vy)
        ey = (az * vx - ax * vz)
        ez = (ax * vy - ay * vx)
        
        if self.Ki > 0.0:
            self.eInt[0] += ex * self.dt
            self.eInt[1] += ey * self.dt
            self.eInt[2] += ez * self.dt
            
        gx += self.Kp * ex + self.Ki * self.eInt[0]
        gy += self.Kp * ey + self.Ki * self.eInt[1]
        gz += self.Kp * ez + self.Ki * self.eInt[2]
        
        gx *= (0.5 * self.dt); gy *= (0.5 * self.dt); gz *= (0.5 * self.dt)
        
        qa, qb, qc = q0, q1, q2
        q0 += (-qb * gx - qc * gy - q3 * gz)
        q1 += (qa * gx + qc * gz - q3 * gy)
        q2 += (qa * gy - qb * gz + q3 * gx)
        q3 += (qa * gz + qb * gy - qc * gx)
        
        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q = np.array([q0, q1, q2, q3]) / norm

class ImuMahonyNode(Node):
    def __init__(self):
        super().__init__('imu_mahony_node')
        self.subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback, 100)
        self.publisher = self.create_publisher(Imu, '/imu/data_mahony', 100)
        self.filter = MahonyFilter(Kp=5.0, Ki=0.1, sample_freq=1600.0)
        
    def imu_callback(self, msg):
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        
        self.filter.update(gx, gy, gz, ax, ay, az)
        q0, q1, q2, q3 = self.filter.q
        
        # Publish the same IMU message but with orientation filled!
        out_msg = Imu()
        out_msg.header = msg.header
        # rviz2 defaults to 'map' fixed frame. Use 'map' to prevent TF caching crash.
        out_msg.header.frame_id = "map"
        
        out_msg.linear_acceleration = msg.linear_acceleration
        out_msg.angular_velocity = msg.angular_velocity
        
        # ROS uses x, y, z, w quaternion format
        out_msg.orientation.x = q1
        out_msg.orientation.y = q2
        out_msg.orientation.z = q3
        out_msg.orientation.w = q0
        
        self.publisher.publish(out_msg)

def main():
    rclpy.init()
    node = ImuMahonyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
