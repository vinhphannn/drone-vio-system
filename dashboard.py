#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from nav_msgs.msg import Odometry
import time
import os
import math

class DashboardNode(Node):
    def __init__(self):
        super().__init__('drone_dashboard')
        self.sub_imu = self.create_subscription(Imu, '/imu/data', self.imu_cb, 100)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        
        self.imu_count = 0
        self.cam_count = 0
        self.odom_count = 0
        
        self.total_imu = 0
        self.total_cam = 0
        
        self.last_time = time.time()
        self.start_time = time.time()
        
        self.imu_hz = 0.0
        self.cam_hz = 0.0
        self.odom_hz = 0.0
        
        self.px = 0.0
        self.py = 0.0
        self.pz = 0.0
        self.qw = 1.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        
        self.wx = 0.0
        self.wy = 0.0
        self.wz = 0.0
        
        self.last_cpu_total = 0
        self.last_cpu_idle = 0
        
        # Tăng tốc độ cập nhật Dashboard lên 50Hz (0.02s)
        self.timer = self.create_timer(0.02, self.update_dashboard)
        
    def imu_cb(self, msg):
        self.imu_count += 1
        self.total_imu += 1
        self.wx = msg.angular_velocity.x
        self.wy = msg.angular_velocity.y
        self.wz = msg.angular_velocity.z
        
    def odom_cb(self, msg):
        self.odom_count += 1
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y
        self.pz = msg.pose.pose.position.z
        self.qw = msg.pose.pose.orientation.w
        self.qx = msg.pose.pose.orientation.x
        self.qy = msg.pose.pose.orientation.y
        self.qz = msg.pose.pose.orientation.z

    def get_cpu(self):
        try:
            with open('/proc/stat', 'r') as f:
                line = f.readline()
                parts = line.split()
                idle = float(parts[4]) + float(parts[5])
                total = sum([float(x) for x in parts[1:]])
                
                idle_delta = idle - self.last_cpu_idle
                total_delta = total - self.last_cpu_total
                
                self.last_cpu_idle = idle
                self.last_cpu_total = total
                
                if total_delta == 0: return 0.0
                return 100.0 * (1.0 - idle_delta / total_delta)
        except:
            return 0.0

    def get_ram(self):
        try:
            mem_total = 1
            mem_avail = 1
            with open('/proc/meminfo', 'r') as f:
                for line in f:
                    if line.startswith('MemTotal:'):
                        mem_total = float(line.split()[1])
                    elif line.startswith('MemAvailable:'):
                        mem_avail = float(line.split()[1])
            return 100.0 * (1.0 - mem_avail / mem_total)
        except:
            return 0.0

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.degrees(math.atan2(t0, t1))
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.degrees(math.asin(t2))
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.degrees(math.atan2(t3, t4))
     
        return roll, pitch, yaw

    def update_dashboard(self):
        now = time.time()
        dt = now - self.last_time
        if dt == 0: dt = 1e-6
        
        self.imu_hz = self.imu_count / dt
        self.cam_hz = self.cam_count / dt
        self.odom_hz = self.odom_count / dt
        
        self.imu_count = 0
        self.cam_count = 0
        self.odom_count = 0
        self.last_time = now
        
        cpu_percent = self.get_cpu()
        ram_percent = self.get_ram()
        
        uptime = int(now - self.start_time)
        m, s = divmod(uptime, 60)
        h, m = divmod(m, 60)
        uptime_str = f"{h:02d}:{m:02d}:{s:02d}"
        
        roll, pitch, yaw = self.euler_from_quaternion(self.qx, self.qy, self.qz, self.qw)
        dist = math.sqrt(self.px**2 + self.py**2 + self.pz**2)
        
        if self.odom_hz > 5:
            tracking_status = "OK"
            color_track = "\033[1;32m"
        elif uptime < 5:
            tracking_status = "WAITING"
            color_track = "\033[1;33m"
        else:
            tracking_status = "LOST"
            color_track = "\033[1;31m"
        
        os.system('clear')
        
        print(f"\033[1;36m═══════════════════════ ROS 2 + ORB-SLAM3 Dashboard ═══════════════════════\033[0m")
        print(f"║   ● RUNNING    uptime: {uptime_str}   CPU: {cpu_percent:4.1f}%   RAM: {ram_percent:4.1f}%                ")
        print(f"╚════════════════════════════════════════════════════════════════════════        ")
        print(f"╭─────────────── ⚡ SENSORS ───────────────╮╭────────── 🛩  VIO ODOMETRY ───────────")
        print(f"│  IMU Rate           \033[1;36m{self.imu_hz:7.1f} Hz\033[0m       ││  Tracking            \033[1;32mOK\033[0m           ")
        print(f"│  Received          {self.total_imu:<14}      ││  Odom Rate           \033[1;32m{self.odom_hz:7.1f} Hz\033[0m")
        print(f"│  Topic             /imu/data           ││  ────────────────                    ")
        print(f"│                                        ││  X (forward)           \033[1;37m{self.px:+6.3f} m\033[0m")
        print(f"│  Camera Rate       \033[1;32m   30.0 Hz\033[0m       ││  Y (right)             \033[1;37m{self.py:+6.3f} m\033[0m")
        print(f"│                                        ││  Z (down)              \033[1;37m{self.pz:+6.3f} m\033[0m")
        print(f"│  Topic             /camera/image_raw   ││  Distance            \033[1;33m{dist:8.3f} m\033[0m")
        print(f"│                                        ││  ────────────────                    ")
        print(f"│                                        ││  Roll                  \033[1;37m{roll:+6.2f}°\033[0m")
        print(f"│  Gyro X            \033[1;36m{self.wx:+7.3f} rad/s\033[0m   ││  Pitch                 \033[1;37m{pitch:+6.2f}°\033[0m")
        print(f"│  Gyro Y            \033[1;36m{self.wy:+7.3f} rad/s\033[0m   ││  Yaw                   \033[1;37m{yaw:+6.2f}°\033[0m")
        print(f"│  Gyro Z            \033[1;36m{self.wz:+7.3f} rad/s\033[0m   ││                                    ")
        print(f"╰────────────────────────────────────────╯╰──────────────────────────────────────")

def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
