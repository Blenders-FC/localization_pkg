import numpy as np
import cv2
import math

# === Camera Parameters ===
fx, fy = 644.24495292, 646.70221114
cx, cy = 320.0, 240.0
camera_height = 0.46  # meters above ground
tilt_deg = -10.0      # camera tilt down from horizontal
tilt_deg -= 7  # hip pitch
pan_deg = 0.0         # camera pan (left/right, zero if looking forward)
tilt_rad = math.radians(tilt_deg)
pan_rad = math.radians(pan_deg)

# K = np.array([[fx, 0, cx],
#               [0, fy, cy],
#               [0,  0,  1]])
# dist_coeffs = np.array([0.05977197, -0.33388722, 0.00231276, 0.00264637, 0.47509537])

# Soccer ball parameters
D = 0.12  # meters (FIFA size 1 ball)

# Detected from image
u, v = 320, 240          # Center of ball in image (pixels)
diameter = 36            # Diameter of ball in image (pixels)

# undistort the detected pixel
# pts = np.array([[[u, v]]], dtype=np.float32)
# undistorted_pts = cv2.undistortPoints(pts, K, dist_coeffs, P=K)
# u, v = undistorted_pts[0,0,0], undistorted_pts[0,0,1]

# Step 1: Calculate Z (depth from camera to ball center)
f = (fx + fy) / 2
Z_cam = f * D / diameter

# Step 2: Back-project to (X_cam, Y_cam, Z_cam) in camera frame
X_cam = (u - cx) * Z_cam / fx
Y_cam = (v - cy) * Z_cam / fy

# ---- Camera frame to robot/world frame ----
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

# The camera's position relative to robot base (assuming z = camera_height)
cam_pos_in_robot = np.array([camera_height, 0, 0])

# Transform ball vector to robot/world frame
ball_cam = np.array([Z_cam, X_cam, Y_cam])

ball_robot = R @ ball_cam + cam_pos_in_robot

# ----- Output -----
print(f"Ball position in camera frame: X={X_cam:.2f} m, Y={Y_cam:.2f} m, Z={Z_cam:.2f} m")
print(f"Ball position in robot frame:  X={ball_robot[0]:.2f} m, Y={ball_robot[1]:.2f} m, Z={ball_robot[2]:.2f} m")

# Horizontal bearing in robot frame:
theta_rad = np.arctan2(ball_robot[1], ball_robot[0])
theta_deg = np.degrees(theta_rad)
print(f"Ball horizontal bearing in robot frame: {theta_deg:.2f} deg")
