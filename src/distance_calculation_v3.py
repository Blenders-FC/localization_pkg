import numpy as np
import cv2
import math

# === Camera Parameters ===
fx, fy = 644.24495292, 646.70221114
cx, cy = 320.0, 240.0
camera_height = 0.46  # meters above ground
tilt_deg = -10.0      # camera tilt down from horizontal
pan_deg = 0.0         # camera pan (left/right, zero if looking forward)
tilt_rad = math.radians(tilt_deg)
pan_rad = math.radians(pan_deg)

# K = np.array([[fx, 0, cx],
#               [0, fy, cy],
#               [0,  0,  1]])
# dist_coeffs = np.array([0.05977197, -0.33388722, 0.00231276, 0.00264637, 0.47509537])

# Detected from image
u, v = 640, 0          # Center of ball in image (pixels)

# undistort the detected pixel
# pts = np.array([[[u, v]]], dtype=np.float32)
# undistorted_pts = cv2.undistortPoints(pts, K, dist_coeffs, P=K)
# u, v = undistorted_pts[0,0,0], undistorted_pts[0,0,1]

# Step 1: Back-project to (X_cam, Y_cam, Z_cam) in camera frame
X_cam = (u - cx) / fx
Y_cam = (v - cy) / fy
direction_camera  = np.array([X_cam, Y_cam, 1.0])
direction_camera = direction_camera / np.linalg.norm(direction_camera)

# Step 2: Camera frame to robot/world frame
# Rotation matrix for tilt (around camera X), then pan (around camera Z)
R_tilt = np.array([
    [1, 0, 0],
    [0, np.cos(tilt_rad), -np.sin(tilt_rad)],
    [0, np.sin(tilt_rad),  np.cos(tilt_rad)]
])
R_pan = np.array([
    [math.cos(pan_rad), 0, math.sin(pan_rad)],
    [0,                 1, 0],
    [-math.sin(pan_rad), 0, math.cos(pan_rad)]
])
# Full rotation (pan * tilt)
R = R_pan @ R_tilt
direction_world = R @ direction_camera

# The camera's position relative to robot base (assuming z = camera_height)
camera_pos = np.array([0, camera_height, 0])  # (X, Y, Z)

# Step 3: Transform ball vector to robot/world frame (intersection with ground plane)
t = -camera_height / direction_world[1]
point_ground = camera_pos + t * direction_world
print(point_ground)
print(direction_world)
print(camera_pos)
print(t * direction_world)

print(f"Ground intersection point: X={point_ground[0]:.3f} m, Y={point_ground[1]:.3f} m, Z={point_ground[2]:.3f} m")
# # 2D field coordinates (X, Z):
# print(f"Field coordinates: X={point_ground[0]:.3f} m, Z={point_ground[2]:.3f} m")

# Horizontal bearing in robot frame:
theta_rad = np.arctan2(point_ground[1], point_ground[0])
theta_deg = np.degrees(theta_rad)
print(f"Ball horizontal bearing in robot frame: {theta_deg:.2f} deg")
