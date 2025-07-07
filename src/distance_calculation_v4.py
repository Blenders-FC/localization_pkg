import numpy as np
import math

camera_height = 0.46      # meters
tilt_deg = -10            # downwards
tilt_deg -= 7  # hip pitch
pan_deg = 0               # facing forward

# Translation vector (camera position in world, in camera coordinates)
t = np.array([[0],
              [camera_height],
              [0]])

# Angles in radians
tilt_rad = math.radians(tilt_deg)
pan_rad  = math.radians(pan_deg)

# Pan (yaw) rotation around Y
R_pan = np.array([
    [math.cos(pan_rad), 0, math.sin(pan_rad)],
    [0, 1, 0],
    [-math.sin(pan_rad), 0, math.cos(pan_rad)]
])

# Tilt (pitch) rotation around X
R_tilt = np.array([
    [1, 0, 0],
    [0, math.cos(tilt_rad), -math.sin(tilt_rad)],
    [0, math.sin(tilt_rad), math.cos(tilt_rad)]
])

# Camera intrinsic matrix
fx, fy = 644.2, 646.7
cx, cy = 320.0, 240.0
K = np.array([
    [fx, 0, cx],
    [0, fy, cy],
    [0,  0,  1]
])

# Camera rotation (R) and translation (t)
# Example: Identity (looking forward, at origin)
# Combined rotation matrix: first pan, then tilt
R = R_tilt @ R_pan
# t = np.array([[0],    # Replace with your translation vector (3x1)
#               [1],
#               [0]]) # Example: Camera 1m above the ground

# Pixel coordinates
u, v = 320, 240       # Replace with your pixel of interest

# === CODE ===

# 1. Camera center in world coordinates: C_g = -R^T @ t
C_g = -R.T @ t  # Shape: (3, 1)

# 2. Convert pixel to normalized camera coordinates (Q_c)
Q = np.array([[u], [v], [1]])  # shape (3,1)
K_inv = np.linalg.inv(K)
Q_c = K_inv @ Q   # Direction in camera frame (not normalized)

# 3. Ray direction in world coordinates
ray_dir = R.T @ Q_c  # (3,1)
ray_dir = ray_dir / np.linalg.norm(ray_dir)  # (optional: normalize)

# 4. Ray: P_g = C_g + λ * ray_dir
#    Find λ so that P_g[2] == 0 (Z = 0, ground plane)
lambda_ground = -C_g[2, 0] / ray_dir[2, 0]

# 5. Intersection point in world coordinates
P_g = C_g + lambda_ground * ray_dir  # (3,1)

# 6. Distance from camera center to ground intersection
distance = np.linalg.norm(P_g - C_g)

print("Camera center in world:", C_g.ravel())
print("Ground intersection:", P_g.ravel())
print("Distance to ground point:", distance)
