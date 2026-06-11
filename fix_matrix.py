import numpy as np

# IMU vectors from user data
g2_imu = np.array([1.611, 0.044, 9.924])
g3_imu = np.array([-0.610, -11.073, 0.033])

# CORRECT Camera vectors (Specific force points UP)
# Stand straight: UP is -Y
g2_cam = np.array([0.0, -1.0, 0.0])
# Tilt left: UP is +X
g3_cam = np.array([1.0, 0.0, 0.0])

# Normalize
g2_imu /= np.linalg.norm(g2_imu)
g2_cam /= np.linalg.norm(g2_cam)
g3_imu /= np.linalg.norm(g3_imu)
g3_cam /= np.linalg.norm(g3_cam)

A = np.column_stack([g2_cam, g3_cam, np.cross(g2_cam, g3_cam)])
B = np.column_stack([g2_imu, g3_imu, np.cross(g2_imu, g3_imu)])
R_bc = B @ np.linalg.inv(A)

# Orthogonalize
U, S, Vt = np.linalg.svd(R_bc)
R_bc = U @ Vt

print("True R_bc:")
print(np.round(R_bc, 4))
