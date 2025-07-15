#!/usr/bin/env python3

import rospy
import numpy as np
import math
import cv2

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

class BallProjector:
    def __init__(self):
        # === Camera Parameters ===
        self.fx, self.fy = 644.24495292, 646.70221114
        self.f = (self.fx + self.fy) / 2
        self.cx, self.cy = 320.0, 240.0  # 329.96326873, 236.89398176
        self.camera_height = 0.46  # meters
        self.D = 0.12  # meters (FIFA size 1 ball)

        self.K = np.array([[self.fx, 0, self.cx],
                           [0, self.fy, self.cy],
                           [0,     0,     1]])
        # self.dist_coeffs = [0.05977197, -0.33388722, 0.00231276, 0.00264637, 0.47509537] # np.zeros(5)  # assuming no distortion

        # === Initialize Values ===
        self.pan_rad = 0.0
        self.tilt_rad = 0.0
        self.bx, self.by = None, None

        # === ROS Subscriptions ===
        self.robot_id = rospy.get_param('robot_id', 0)
        rospy.Subscriber(f'/robotis_{self.robot_id}/present_joint_states', JointState, self.angle_callback)
        rospy.Subscriber(f'/robotis_{self.robot_id}/ball_center', Point, self.pixel_callback)

    def angle_callback(self, msg):
        self.pan_rad, self.tilt_rad = msg.position[0], msg.position[1]
        self.tilt_rad -= 0.122173  # 7° hip pitch
        self.try_estimate()

    def pixel_callback(self, msg):
        self.bx, self.by, self.diameter = msg.x, msg.y, msg.z
        self.try_estimate()

    def try_estimate(self):
        # Step 1: Calculate Z (depth from camera to ball center)
        Z_cam = self.f * self.D / self.diameter

        # Step 2: Back-project to (X_cam, Y_cam, Z_cam) in camera frame
        X_cam = (self.bx - self.cx) * Z_cam / self.fx
        Y_cam = (self.by - self.cy) * Z_cam / self.fy

        # ---- Camera frame to robot/world frame ----
        # Rotation matrix for tilt (around camera X), then pan (around camera Z)
        R_tilt = np.array([
            [1, 0, 0],
            [0, np.cos(self.tilt_rad), -np.sin(self.tilt_rad)],
            [0, np.sin(self.tilt_rad),  np.cos(self.tilt_rad)]
        ])
        R_pan = np.array([
            [np.cos(self.pan_rad), -np.sin(self.pan_rad), 0],
            [np.sin(self.pan_rad),  np.cos(self.pan_rad), 0],
            [0,               0,                1]
        ])
        # Full rotation (pan * tilt)
        R = R_pan @ R_tilt

        # The camera's position relative to robot base (assuming z = camera_height)
        cam_pos_in_robot = np.array([self.camera_height, 0, 0])

        # Transform ball vector to robot/world frame
        ball_cam = np.array([Z_cam, X_cam, Y_cam])

        ball_robot = R @ ball_cam + cam_pos_in_robot

        # ----- Output -----
        print(f"Ball position in camera frame: X={X_cam:.2f} m, Y={Y_cam:.2f} m, Z={Z_cam:.2f} m")
        print(f"Ball position in robot frame:  X={ball_robot[0]:.2f} m, Y={ball_robot[1]:.2f} m, Z={ball_robot[2]:.2f} m")

        # Navigation: you can use (X, Y, Z) in the robot/world frame to plan your step!

        # Optionally, for horizontal bearing in robot frame:
        theta_rad = np.arctan2(ball_robot[1], ball_robot[0])
        theta_deg = np.degrees(theta_rad)
        print(f"Ball horizontal bearing in robot frame: {theta_deg:.2f} deg")


if __name__ == "__main__":
    rospy.init_node("ball_projection_node")
    BallProjector()
    rospy.spin()
