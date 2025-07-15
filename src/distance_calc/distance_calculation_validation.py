import numpy as np
import cv2
import math

# === Known camera intrinsics ===
fx, fy = 644.24495292, 646.70221114
cx, cy = 329.96326873, 236.89398176
camera_height = 0.46  # meters
tilt_deg = -10.0
tilt_deg -= 7  # hip pitch
pan_deg = 0.0
tilt_rad = math.radians(tilt_deg)
pan_rad = math.radians(pan_deg)

K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])

dist_coeffs = np.zeros(5)  # no distortion

# === Rotation matrices ===
Rx = np.array([[1, 0, 0],
               [0, math.cos(tilt_rad), -math.sin(tilt_rad)],
               [0, math.sin(tilt_rad),  math.cos(tilt_rad)]])
Ry = np.array([[ math.cos(pan_rad), 0, math.sin(pan_rad)],
               [0,                 1, 0],
               [-math.sin(pan_rad), 0, math.cos(pan_rad)]])
R = Rx @ Ry  # world-to-camera rotation

# === Test: Ground truth ball positions (X, Y, Z) ===
# Y is always 0 (on the ground)
test_positions = [
    [0.0, 0.0, 1.0],
    [0.2, 0.0, 1.5],
    [-0.2, 0.0, 1.5],
    [0.3, 0.0, 2.0],
    [-0.3, 0.0, 2.0],
]

print("\n==== VALIDATION RESULTS ====\n")
for gt_pos in test_positions:
    X, Y, Z = gt_pos
    # Step 1: Transform ball world position to camera frame
    # ball_cam = np.linalg.inv(R) @ np.array([X, Y - camera_height, Z])  # note: Y axis is height
    ball_cam = R.T @ np.array([X, Y - camera_height, Z])

    # Step 2: Project to pixel using intrinsics
    u = fx * (ball_cam[0] / ball_cam[2]) + cx
    v = fy * (ball_cam[1] / ball_cam[2]) + cy
    pixel = np.array([[[u, v]]], dtype=np.float32)

    # Step 3: Distance estimation using your method
    undistorted = cv2.undistortPoints(pixel, K, dist_coeffs)
    direction_cam = np.append(undistorted[0][0], 1.0)

    direction_world = Ry @ Rx @ direction_cam
    scale = camera_height / direction_world[1]
    position_world = direction_world * scale

    estimated_X = position_world[0]
    estimated_Z = position_world[2]
    estimated_distance = np.linalg.norm([estimated_X, estimated_Z])
    true_distance = np.linalg.norm([X, Z])
    error = estimated_distance - true_distance
    angle_deg = math.degrees(math.atan2(estimated_X, estimated_Z))

    print(f"GT: X={X:.2f} Z={Z:.2f} | Estimated: X={estimated_X:.2f} Z={estimated_Z:.2f} | "
          f"Error: {error:.3f} m | Angle: {angle_deg:.2f}°")
