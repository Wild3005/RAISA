#!/usr/bin/env python3
"""
Pozyx + Kalman 3D (x, y, theta) + Umeyama + ROS2 Pose2D Publisher
"""

import sys
import time
import csv
import termios
import tty
import select
import requests
from datetime import datetime
import numpy as np
import pandas as pd
from time import sleep

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32

from pypozyx import (
    PozyxSerial, get_first_pozyx_serial_port, PozyxConstants,
    Coordinates, DeviceCoordinates, SensorData, POZYX_SUCCESS, EulerAngles
)

# ==========================
# REGRESSION
# ==========================
termsX2_2 = [-1.2838861917401223e-001, 1.0450381308097056e+000]
def regressX(x):
    t, r = 1, 0
    for c in termsX2_2:
        r += c * t
        t *= x
    return r

termsY2_2 = [-3.9182911446895652e-001, 9.9020003655176714e-001]
def regressY(x):
    t, r = 1, 0
    for c in termsY2_2:
        r += c * t
        t *= x
    return r

# ==========================
# CONFIG
# ==========================
TAG_TARGET = 0x6800
CSV_FILE   = "position_log.csv"
LOOP_DT    = 0.5

OFFSET_X = 3325
OFFSET_Y = 831

ANCHORS = [
    DeviceCoordinates(0x6722, 1, Coordinates(0 - OFFSET_X, 0 - OFFSET_Y, 1109)),
    DeviceCoordinates(0x6772, 1, Coordinates(9210 - OFFSET_X, -1154 - OFFSET_Y, 1637)),
    DeviceCoordinates(0x6764, 1, Coordinates(11591 - OFFSET_X, 8201 - OFFSET_Y, 480)),
    DeviceCoordinates(0x671D, 1, Coordinates(604 - OFFSET_X, 8235 - OFFSET_Y, 1897)),
]

offset_theta = 0

# ==========================
# KALMAN FILTER 3D (x, y, theta)http://192.168.100.235/
# ==========================
class Kalman3D:
    """
    Extended Kalman Filter for x, y, and theta (heading)
    State vector: [x, y, theta, vx, vy, omega]
    where omega is angular velocity
    """
    def __init__(self, meas, q_pos=100.0, q_theta=0.1, r_pos=40000.0, r_theta=0.5):
        """
        meas: tuple (x, y, theta) for initial measurement
        q_pos: process noise for position
        q_theta: process noise for angle
        r_pos: measurement noise for position
        r_theta: measurement noise for angle
        """
        self.x = np.array([
            [meas[0]],    # x
            [meas[1]],    # y
            [meas[2]],    # theta
            [0.0],        # vx
            [0.0],        # vy
            [0.0]         # omega (angular velocity)
        ])
        
        # Initial covariance
        self.P = np.diag([500, 500, 1.0, 500, 500, 1.0])
        
        # Process noise parameters
        self.q_pos = q_pos
        self.q_theta = q_theta
        
        # Measurement noise parameters
        self.r_pos = r_pos
        self.r_theta = r_theta

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np. pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def predict(self, dt):
        """Predict step with constant velocity model"""
        # State transition matrix
        F = np. array([
            [1, 0, 0, dt, 0,  0],
            [0, 1, 0, 0,  dt, 0],
            [0, 0, 1, 0,  0,  dt],
            [0, 0, 0, 1,  0,  0],
            [0, 0, 0, 0,  1,  0],
            [0, 0, 0, 0,  0,  1]
        ])

        # Control input matrix (process noise)
        G = np.array([
            [0.5*dt*dt, 0,          0],
            [0,         0.5*dt*dt,  0],
            [0,         0,          0.5*dt*dt],
            [dt,        0,          0],
            [0,         dt,         0],
            [0,         0,          dt]
        ])

        # Process noise covariance
        Q = G @ np.diag([self.q_pos, self. q_pos, self.q_theta]) @ G.T
        
        # Predict
        self.x = F @ self.x
        
        # Normalize theta
        self.x[2, 0] = self.normalize_angle(self.x[2, 0])
        
        self.P = F @ self.P @ F.T + Q

    def update(self, meas):
        """
        Update step
        meas: tuple (x, y, theta)
        """
        # Measurement matrix (we measure x, y, theta directly)
        H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])

        # Measurement noise covariance
        R = np.diag([self.r_pos, self.r_pos, self. r_theta])
        
        # Measurement vector
        z = np.array([[meas[0]], [meas[1]], [meas[2]]])

        # Innovation (measurement residual)
        y = z - H @ self.x
        
        # Normalize angle difference
        y[2, 0] = self.normalize_angle(y[2, 0])

        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        self.x = self.x + K @ y
        
        # Normalize theta after update
        self.x[2, 0] = self.normalize_angle(self.x[2, 0])
        
        # Update covariance
        self. P = (np.eye(6) - K @ H) @ self.P

    def get_state(self):
        """Return filtered x, y, theta"""
        return float(self.x[0]), float(self.x[1]), float(self.x[2])

# ==========================
# ROS2 PUBLISHER
# ==========================
class UwbPose2DPublisher(Node):
    def __init__(self):
        super().__init__('uwb_pose2d_publisher')
        self.pub = self.create_publisher(Pose2D, 'uwb_pose2d', 1)
        self.sub_calibrate = self.create_subscription(Float32,'/uwb/calibrate',self.calibrate_callback,1)

    def publish(self, x, y, theta=0.0):
        msg = Pose2D()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(theta)
        self.pub.publish(msg)

    def calibrate_callback(self, msg):
        global offset_theta
        self.get_logger().info(f"Calibration command received.: {msg.data}")
        offset_theta = msg.data

# ==========================
# POZYX HELPERS
# ==========================
def get_position(po, rid):
    pos = Coordinates()
    ok = po.doPositioning(
        pos,
        PozyxConstants.DIMENSION_2D,
        1000,
        PozyxConstants. POSITIONING_ALGORITHM_UWB_ONLY,
        remote_id=rid
    )
    return (ok == POZYX_SUCCESS, pos)

def get_heading(po, rid):
    euler = EulerAngles()
    status = po.getEulerAngles_deg(euler, remote_id=rid)
    if status == POZYX_SUCCESS:
        return float(euler.heading)  # degrees
    return None

# ==========================
# UMEYAMA
# ==========================
def umeyama_alignment(src, dst):
    mu_src = src.mean(axis=1, keepdims=True)
    mu_dst = dst. mean(axis=1, keepdims=True)

    src_c = src - mu_src
    dst_c = dst - mu_dst

    Sigma = dst_c @ src_c.T / src.shape[1]
    U, D, Vt = np.linalg.svd(Sigma)

    S = np.eye(2)
    if np.linalg. det(U @ Vt) < 0:
        S[1,1] = -1

    Rm = U @ S @ Vt
    var_src = np.sum(src_c**2) / src.shape[1]
    s = np. sum(D * np.diag(S)) / var_src

    t = mu_dst - s * Rm @ mu_src
    return s, Rm, t

# ==========================
# LOAD TRANSFORM
# ==========================
df = pd.read_csv("log.csv")
uwb = df[['kal_x', 'kal_y']].to_numpy().T
odom = df[['odom_x', 'odom_y']]. to_numpy().T
s, Rm, t = umeyama_alignment(uwb, odom)

def uwb_to_odom(x, y):
    v = np.array([[x],[y]])
    return (s * Rm @ v + t).flatten()

# ==========================
# MAIN
# ==========================
def main():
    global offset_theta
    rclpy.init()
    ros_node = UwbPose2DPublisher()

    port = get_first_pozyx_serial_port()
    if not port:
        print("No Pozyx found.")
        return

    pozyx = PozyxSerial(port)
    print("Connected:", port)

    pozyx.clearDevices(TAG_TARGET)
    for a in ANCHORS:
        pozyx.addDevice(a, TAG_TARGET)

    kalman = None
    prev_heading_rad = 0.0

    try:
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.0)
            ok, pos = get_position(pozyx, TAG_TARGET)
            heading_deg = get_heading(pozyx, TAG_TARGET)

            if ok and heading_deg is not None: 
                raw_x = float(pos.x) / 1000
                raw_y = float(pos.y) / 1000
                heading_rad = np.deg2rad(heading_deg)
                
                # Apply your heading adjustments to the measurement
                heading_rad += np.pi
                # ros_node.get_logger().info(f"=== {heading_rad} offset_theta: {offset_theta}")
                
                # Normalize to [-pi, pi]
                while heading_rad > np.pi:
                    heading_rad -= 2 * np.pi
                while heading_rad < -np. pi:
                    heading_rad += 2 * np.pi

                meas = (raw_x, raw_y, heading_rad)

                # Initialize Kalman filter on first valid measurement
                if kalman is None:
                    kalman = Kalman3D(
                        meas, 
                        q_pos=4000,      # position process noise
                        q_theta=0.1,     # angle process noise (tune this)
                        r_pos=20000,    # position measurement noise
                        r_theta=0.1      # angle measurement noise (tune this)
                    )
                    ros_node.get_logger().info("Kalman 3D filter initialized")

                # Predict and update
                kalman.predict(LOOP_DT)
                kalman.update(meas)

                # Get filtered state
                fx, fy, ftheta_kalman = kalman.get_state()

                alpha = 0.5
                heading_rad_used = alpha * heading_rad + (1 - alpha) * prev_heading_rad


                pub_heading = heading_rad_used

                pub_heading += offset_theta

                while pub_heading > np.pi:
                    pub_heading -= 2 * np.pi
                while pub_heading < -np. pi:
                    pub_heading += 2 * np.pi

                # Transform to odometry frame
                ux, uy = uwb_to_odom(fx, fy)

                # Publish with filtered theta
                ros_node.publish(ux, uy, -pub_heading)
                prev_heading_rad = heading_rad_used

                # ros_node.get_logger().info(
                #     f"Filtered Pose → x:{ux:.3f} y:{uy:.3f} θ:{np.rad2deg(ftheta):.1f}°"
                # )

            sleep(LOOP_DT)

    except KeyboardInterrupt: 
        pass

    finally:
        rclpy.shutdown()
        print("\nShutdown complete.")

# ==========================
if __name__ == "__main__":
    main()