import cv2
import numpy as np
import glob

# Chessboard size
chessboard_size = (8, 6)
square_size = 0.025  # in meters

# Termination criteria for subpixel corner refinement
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 3D world points
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []  # 3D points in real world
imgpoints = []  # 2D points in image plane

images = glob.glob('../calibrationdata/*.jpg')  # folder with your images

if len(images) < 1:
    print("No images found")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    # ret, corners = cv2.findChessboardCorners(gray, chessboard_size, flags)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    # print(corners)
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        # cv2.imshow('img', img)
        # cv2.waitKey(100)
        print(f"successful img: {fname}")
    # else:
    #     cv2.imshow('fail', img)
    #     cv2.waitKey(0)  # pause so you can inspect

cv2.destroyAllWindows()
print(f"size of the image: {gray.shape[::-1]}")
print(f"total num of imgpoints: {len(imgpoints)}")

# Calibrate
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera matrix (K):\n", K)
print("Distortion coefficients:\n", dist)

total_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    total_error += error

print("Mean Reprojection Error: {:.4f} pixels".format(total_error / len(objpoints)))
