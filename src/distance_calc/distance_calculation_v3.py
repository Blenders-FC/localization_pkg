import numpy as np

# Helper: build rotation matrix from pan and tilt
def rotation_matrix_y(angle_deg):
    angle_rad = np.deg2rad(angle_deg)
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[c, 0, s],
                     [0, 1, 0],
                     [-s, 0, c]])

def rotation_matrix_x(angle_deg):
    angle_rad = np.deg2rad(angle_deg)
    c, s = np.cos(angle_rad), np.sin(angle_rad)
    return np.array([[1, 0, 0],
                     [0, c, -s],
                     [0, s, c]])

# ====== Step 0: Camera and Scene Parameters ======
# Intrinsic parameters (example values - use your calibration!)
fx, fy = 644.2, 646.7
cx, cy = 320.0, 240.0
K = np.array([[fx, 0, cx],
              [0, fy, cy],
              [0,  0,  1]])

# Camera pose (height and orientation)
camera_height = 0.46    # meters (Y_0)
pan_deg = 0          # degrees, yaw (rotation about Y)
tilt_deg = -10.0        # degrees, pitch (rotation about X)
tilt_deg -= 7  # hip pitch
# Assume camera is at (0, H, 0)
X0, Y0, Z0 = 0.0, camera_height, 0.0

# ====== Step 1: Convert pixel to normalized camera ray ======
u, v = 320, 240  # Example: bottom center of detected object in image

# -- Normalize pixel using intrinsic parameters
x_c = (u - cx) / fx
y_c = (v - cy) / fy

d_c = np.array([x_c, y_c, 1.0])  # Camera frame direction

# ====== Step 2: Rotate the ray to world coordinates ======
# Apply tilt first (X), then pan (Y)
R_tilt = rotation_matrix_x(tilt_deg)    # roll
R_pan = rotation_matrix_y(pan_deg)      # yaw
R = R_pan @ R_tilt  # Matrix multiplication: R = R_pan * R_tilt

# -- World-frame direction vector
d_w = R @ d_c  # Rotate camera ray into world coordinates

# ====== Step 3: Find intersection with ground plane (Y=0) ======
# Parametric: [X, Y, Z] = [X0, Y0, Z0] + t * [dx, dy, dz]
dx, dy, dz = d_w

if abs(dy) < 1e-6:
    raise ValueError("Ray is parallel to the ground!")

t = -Y0 / dy  # Step along the ray until Y=0

# ====== Step 4: Compute ground point coordinates ======
X_ground = X0 + t * dx
Y_ground = 0.0
Z_ground = Z0 + t * dz

print("Ground intersection point:")
print(f"X = {X_ground:.3f} m, Y = {Y_ground:.3f} m, Z = {Z_ground:.3f} m")

# (Optional) Ground distance from camera center to object
ground_dist = np.sqrt((X_ground - X0)**2 + (Z_ground - Z0)**2)
print(f"Ground (XZ) distance: {ground_dist:.3f} m")
