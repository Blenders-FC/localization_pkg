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
        self.cx, self.cy = 320.0, 240.0
        self.camera_height = 0.46  # meters

        self.K = np.array([[self.fx, 0, self.cx],
                           [0, self.fy, self.cy],
                           [0,     0,     1]])
        self.dist_coeffs = [0.05977197, -0.33388722, 0.00231276, 0.00264637, 0.47509537] # np.zeros(5)  # assuming no distortion

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
        self.bx, self.by = msg.x, msg.y
        self.try_estimate()

    def try_estimate(self):
        if self.bx is None or self.by is None:
            return

        pixel = np.array([[[self.bx, self.by]]], dtype=np.float32)
        undistorted = cv2.undistortPoints(pixel, self.K, self.dist_coeffs)
        direction_cam = np.array([undistorted[0][0][0], undistorted[0][0][1], 1.0])
        direction_cam /= np.linalg.norm(direction_cam)

        # === Apply camera tilt and pan ===
        Rx = np.array([[                      1,                       0,                        0],
                       [                      0, math.cos(self.tilt_rad), -math.sin(self.tilt_rad)],
                       [                      0, math.sin(self.tilt_rad),  math.cos(self.tilt_rad)]])
        
        Ry = np.array([[ math.cos(self.pan_rad),                       0,   math.sin(self.pan_rad)],
                       [                      0,                       1,                        0],
                       [-math.sin(self.pan_rad),                       0,   math.cos(self.pan_rad)]])

        direction_world = Ry @ Rx @ direction_cam
        sign_y = np.sign(direction_world[1])

        if direction_world[1] == 0:
            rospy.logwarn("Ray is parallel to ground. No intersection.")
            return

        scale = self.camera_height / (sign_y * direction_world[1])
        position_world = direction_world * scale

        if position_world[2] <= 0:
            rospy.logwarn("Ground intersection is behind the camera. Invalid.")
            return

        # === Compute distance and angle ===
        distance = np.linalg.norm([position_world[0], position_world[2]])
        angle_from_center = math.atan2(position_world[0], position_world[2])

        rospy.loginfo(f"Estimated ball position: {position_world}")
        rospy.loginfo(f"Distance: {distance:.2f} m | Pan angle: {math.degrees(angle_from_center):.2f}°")


if __name__ == "__main__":
    rospy.init_node("ball_projection_node")
    BallProjector()
    rospy.spin()
