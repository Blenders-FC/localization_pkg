import numpy as np
import cv2
import math

# === Camera Parameters ===
fx, fy = 644.24495292, 646.70221114
cx, cy = 320.0, 240.0
camera_height = 0.46  # meters
tilt_deg = -10.0
pan_deg = 0.0
tilt_rad = math.radians(tilt_deg)

K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])

dist_coeffs = np.array([0.05977197, -0.33388722, 0.00231276, 0.00264637, 0.47509537])  # assume no distortion for now

# === Ball Detection (Example) ===
# Let's say you found the ball at pixel (bx, by) = (300, 280)
bx, by = 630, 150
pixel = np.array([[[bx, by]]], dtype=np.float32)

# === Step 1: Convert to normalized direction vector ===
undistorted = cv2.undistortPoints(pixel, K, dist_coeffs)
# direction_cam = np.append(undistorted[0][0], 1.0)  # (x, y, 1)
direction_cam = np.array([undistorted[0][0][0], undistorted[0][0][1], 1.0])
direction_cam /= np.linalg.norm(direction_cam)  # normalize

# === Step 2: Rotate based on camera tilt ===
# Rotation around x-axis (downward pitch)
Rx = np.array([[1, 0, 0],
               [0, math.cos(tilt_rad), -math.sin(tilt_rad)],
               [0, math.sin(tilt_rad),  math.cos(tilt_rad)]])

pan_rad = math.radians(pan_deg)
Ry = np.array([[ math.cos(pan_rad), 0, math.sin(pan_rad)],
               [0,                 1, 0],
               [-math.sin(pan_rad), 0, math.cos(pan_rad)]])

direction_world = Ry @ Rx @ direction_cam
print(f"direction_world: {direction_world}")
sign_direction_world = np.sign(direction_world[1])
# # if direction_world[1] >= 0:
# #     print("Ray is not pointing toward ground. No intersection.")
# #     exit()

# # === Step 3: Intersect with ground plane ===
# # We assume the ground is at y = 0 in world frame
# scale = camera_height / (sign_direction_world * direction_world[1])  # y component
# position_world = direction_world * scale  # 3D position of the ball
# Compute scale — where the ray intersects y = 0 (ground)
if direction_world[1] == 0:
    print("Ray is parallel to ground. No intersection.")
    exit()

scale = camera_height / (sign_direction_world * direction_world[1])
position_world = direction_world * scale
print(f"position_world: {position_world}")

# Check if point is in front of robot
if position_world[2] <= 0:
    print("Ground intersection is behind the camera. Invalid.")
    exit()

# === Step 4: Calculate horizontal distance ===
distance = np.linalg.norm([position_world[0], position_world[2]])  # x and z
angle_from_robot_center = math.atan2(position_world[0], position_world[2])  # in radians

# === Output ===
print(f"Estimated distance to ball: {distance:.2f} meters")
print(f"Estimated pan angle to ball: {math.degrees(angle_from_robot_center)} degrees")
