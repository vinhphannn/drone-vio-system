import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

class MahonyFilter:
    def __init__(self, Kp=2.0, Ki=0.005, sample_freq=1600.0):
        self.Kp = Kp
        self.Ki = Ki
        self.dt = 1.0 / sample_freq
        self.q = np.array([1.0, 0.0, 0.0, 0.0]) # q0, q1, q2, q3
        self.eInt = np.array([0.0, 0.0, 0.0])
        
    def update(self, gx, gy, gz, ax, ay, az):
        q0, q1, q2, q3 = self.q
        
        # Normalize accelerometer measurement
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm == 0.0:
            return
        ax /= norm
        ay /= norm
        az /= norm
        
        # Estimated direction of gravity
        vx = 2.0 * (q1 * q3 - q0 * q2)
        vy = 2.0 * (q0 * q1 + q2 * q3)
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
        
        # Error is cross product between estimated direction and measured direction of gravity
        ex = (ay * vz - az * vy)
        ey = (az * vx - ax * vz)
        ez = (ax * vy - ay * vx)
        
        if self.Ki > 0.0:
            self.eInt[0] += ex * self.dt
            self.eInt[1] += ey * self.dt
            self.eInt[2] += ez * self.dt
        else:
            self.eInt = np.array([0.0, 0.0, 0.0])
            
        # Apply feedback terms
        gx += self.Kp * ex + self.Ki * self.eInt[0]
        gy += self.Kp * ey + self.Ki * self.eInt[1]
        gz += self.Kp * ez + self.Ki * self.eInt[2]
        
        # Integrate rate of change of quaternion
        gx *= (0.5 * self.dt)
        gy *= (0.5 * self.dt)
        gz *= (0.5 * self.dt)
        
        qa = q0
        qb = q1
        qc = q2
        q0 += (-qb * gx - qc * gy - q3 * gz)
        q1 += (qa * gx + qc * gz - q3 * gy)
        q2 += (qa * gy - qb * gz + q3 * gx)
        q3 += (qa * gz + qb * gy - qc * gx)
        
        # Normalize quaternion
        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q = np.array([q0, q1, q2, q3]) / norm

    def get_euler(self):
        # returns roll, pitch, yaw in degrees
        q0, q1, q2, q3 = self.q
        roll = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2))
        pitch = math.asin(2.0 * (q0 * q2 - q3 * q1))
        yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3))
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

class ImuVizNode(Node):
    def __init__(self):
        super().__init__('imu_mahony_viz')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            100) # High queue size for 1600Hz
        
        # Assuming IMU runs at ~1600Hz
        self.filter = MahonyFilter(Kp=5.0, Ki=0.1, sample_freq=1600.0)
        self.latest_euler = (0.0, 0.0, 0.0)
        self.latest_q = [1.0, 0.0, 0.0, 0.0]
        
    def imu_callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        
        self.filter.update(gx, gy, gz, ax, ay, az)
        self.latest_euler = self.filter.get_euler()
        self.latest_q = self.filter.q

# Global node reference
node = None

def ros_spin_thread():
    rclpy.spin(node)

def update_plot(frame, ax, lines, text_elem):
    if node is None:
        return lines + [text_elem]
        
    roll, pitch, yaw = node.latest_euler
    q0, q1, q2, q3 = node.latest_q
    
    # Rotation matrix from quaternion
    # R represents the orientation of the IMU in the world frame
    R11 = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
    R12 = 2.0 * (q1 * q2 - q0 * q3)
    R13 = 2.0 * (q1 * q3 + q0 * q2)
    
    R21 = 2.0 * (q1 * q2 + q0 * q3)
    R22 = 1.0 - 2.0 * (q1 * q1 + q3 * q3)
    R23 = 2.0 * (q2 * q3 - q0 * q1)
    
    R31 = 2.0 * (q1 * q3 - q0 * q2)
    R32 = 2.0 * (q2 * q3 + q0 * q1)
    R33 = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    
    # Local axes
    X_axis = np.array([R11, R21, R31])
    Y_axis = np.array([R12, R22, R32])
    Z_axis = np.array([R13, R23, R33])
    
    # Update XYZ axes lines (Red=X, Green=Y, Blue=Z)
    lines[0].set_data([0, X_axis[0]], [0, X_axis[1]])
    lines[0].set_3d_properties([0, X_axis[2]])
    
    lines[1].set_data([0, Y_axis[0]], [0, Y_axis[1]])
    lines[1].set_3d_properties([0, Y_axis[2]])
    
    lines[2].set_data([0, Z_axis[0]], [0, Z_axis[1]])
    lines[2].set_3d_properties([0, Z_axis[2]])
    
    # Drone 'X' frame arms (45 degrees off the X/Y axes)
    arm_len = 0.5
    m1 = (X_axis + Y_axis) * arm_len
    m2 = (X_axis - Y_axis) * arm_len
    m3 = (-X_axis - Y_axis) * arm_len
    m4 = (-X_axis + Y_axis) * arm_len
    
    # Arm 1 (Front-Left) to Arm 3 (Back-Right)
    lines[3].set_data([m1[0], m3[0]], [m1[1], m3[1]])
    lines[3].set_3d_properties([m1[2], m3[2]])
    
    # Arm 2 (Front-Right) to Arm 4 (Back-Left)
    lines[4].set_data([m2[0], m4[0]], [m2[1], m4[1]])
    lines[4].set_3d_properties([m2[2], m4[2]])
    
    # Update text
    text_elem.set_text(f"Roll (X):  {roll:7.2f}°\nPitch (Y): {pitch:7.2f}°\nYaw (Z):   {yaw:7.2f}°")
    
    return lines + [text_elem]

def main():
    global node
    rclpy.init()
    node = ImuVizNode()
    
    spin_thread = threading.Thread(target=ros_spin_thread)
    spin_thread.daemon = True
    spin_thread.start()
    
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    
    # Position text in the top left corner
    text_elem = fig.text(0.05, 0.95, "", transform=fig.transFigure, fontsize=14, 
                         verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
                         
    # Set static plot bounds
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_xlabel('X (Forward)')
    ax.set_ylabel('Y (Left)')
    ax.set_zlabel('Z (Up)')
    ax.set_title("Drone 3D Orientation (60 FPS)")
    
    # Draw base plane
    xx, yy = np.meshgrid([-1, 1], [-1, 1])
    zz = np.zeros_like(xx)
    ax.plot_surface(xx, yy, zz, alpha=0.1, color='gray')
    
    # Initialize fast Line3D objects
    lines = []
    lines.append(ax.plot([0, 1], [0, 0], [0, 0], color='red', linewidth=4)[0])   # X axis (Forward)
    lines.append(ax.plot([0, 0], [0, 1], [0, 0], color='green', linewidth=4)[0]) # Y axis (Left)
    lines.append(ax.plot([0, 0], [0, 0], [0, 1], color='blue', linewidth=4)[0])  # Z axis (Up)
    
    # Drone frame lines (gray)
    lines.append(ax.plot([0, 0], [0, 0], [0, 0], color='black', linewidth=6, linestyle='-')[0]) 
    lines.append(ax.plot([0, 0], [0, 0], [0, 0], color='black', linewidth=6, linestyle='-')[0])
    
    # Set up animation (60 FPS = 16ms interval)
    ani = FuncAnimation(fig, update_plot, fargs=(ax, lines, text_elem), interval=16, blit=False)
    
    plt.show()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
