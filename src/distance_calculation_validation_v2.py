import numpy as np

# === Camera Intrinsics ===
fx, fy = 644.24495292, 646.70221114
cx, cy = 330.0, 234.0
D = 0.12          # Ball diameter (m)
camera_height = 0.46

# Camera orientation
tilt_deg = -10.0  # Down tilt (negative = down)
pan_deg  = 25.0   # Pan right (positive = right)
tilt_rad = np.radians(tilt_deg)
pan_rad  = np.radians(pan_deg)

# Camera pose in robot/world frame
cam_pos_robot = np.array([0, 0, camera_height])

# Rotation: robot/world -> camera frame
R_tilt = np.array([
    [1, 0, 0],
    [0, np.cos(tilt_rad), -np.sin(tilt_rad)],
    [0, np.sin(tilt_rad),  np.cos(tilt_rad)]
])
R_pan = np.array([
    [np.cos(pan_rad), -np.sin(pan_rad), 0],
    [np.sin(pan_rad),  np.cos(pan_rad), 0],
    [0,               0,                1]
])
R = R_pan @ R_tilt

# ==== MULTIPLE BALL POSITIONS (ground truth, in robot/world frame) ====
ball_positions_robot = [
    [0.2,  0.0, 1.1],   # In front, right
    [0.0,  0.0, 1.2],   # Center, further
    [-0.2, 0.0, 1.0],   # Left, closer
    [0.3,  0.1, 1.3],   # Farther right and up
    [0.0, -0.1, 0.9],   # Lower and close
    [0.0, 0.0, 2.61],   # in front
]

print("\n=== Testing Multiple Ball Positions ===\n")
for idx, pos in enumerate(ball_positions_robot):
    print(f"--- Ball Position {idx+1} ---")
    Xr_gt, Yr_gt, Zr_gt = pos

    # Transform to camera frame
    ball_vec_robot = np.array([Xr_gt, Yr_gt, Zr_gt]) - cam_pos_robot
    ball_cam = R.T @ ball_vec_robot
    X_gt, Y_gt, Z_gt = ball_cam

    # Project to image
    u, v = 640, 234
    f = (fx + fy) / 2
    diameter = f * D / Z_gt
    print(diameter)

    # Estimate 3D position from image detection
    Z_est = f * D / diameter
    X_est = (u - cx) * Z_est / fx
    Y_est = (v - cy) * Z_est / fy
    ball_cam_est = np.array([X_est, Y_est, Z_est])

    # Rotate back to robot/world frame
    ball_robot_est = R @ ball_cam_est + cam_pos_robot

    # Print results
    print(f"  GT (robot/world): X={Xr_gt:.3f} Y={Yr_gt:.3f} Z={Zr_gt:.3f}")
    print(f"  Simulated image:  u={u:.1f} v={v:.1f} diameter={diameter:.1f}")
    print(f"  Est (robot/world): X={ball_robot_est[0]:.3f} Y={ball_robot_est[1]:.3f} Z={ball_robot_est[2]:.3f}")

    err = np.linalg.norm(ball_robot_est - np.array([Xr_gt, Yr_gt, Zr_gt]))
    print(f"  Estimation error: {err*100:.2f} cm\n")
