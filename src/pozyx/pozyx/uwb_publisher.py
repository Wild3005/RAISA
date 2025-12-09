# #!/usr/bin/env python3
"""
Pozyx + Kalman + Umeyama + ROS2 Pose2D Publisher
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

from pypozyx import (
    PozyxSerial, get_first_pozyx_serial_port, PozyxConstants,
    Coordinates, DeviceCoordinates, SensorData, POZYX_SUCCESS
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
LOOP_DT    = 0.3

OFFSET_X = 3325
OFFSET_Y = 831

ANCHORS = [
    DeviceCoordinates(0x6722, 1, Coordinates(0 - OFFSET_X, 0 - OFFSET_Y, 1109)),
    DeviceCoordinates(0x6772, 1, Coordinates(9210 - OFFSET_X, -1154 - OFFSET_Y, 1637)),
    DeviceCoordinates(0x6764, 1, Coordinates(11591 - OFFSET_X, 8201 - OFFSET_Y, 480)),
    DeviceCoordinates(0x671D, 1, Coordinates(604 - OFFSET_X, 8235 - OFFSET_Y, 1897)),
]

# ==========================
# KALMAN FILTER 2D
# ==========================
class Kalman2D:
    def __init__(self, meas, q=100.0, r=40000.0):
        self.x = np.array([[meas[0]],[meas[1]],[0.0],[0.0]])
        self.P = np.diag([500,500,500,500])
        self.q = q
        self.r = r

    def predict(self, dt):
        F = np.array([
            [1,0,dt,0],
            [0,1,0,dt],
            [0,0,1,0],
            [0,0,0,1]
        ])

        G = np.array([
            [0.5*dt*dt,0],
            [0,0.5*dt*dt],
            [dt,0],
            [0,dt]
        ])

        Q = G @ (np.eye(2)*self.q) @ G.T
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, meas):
        H = np.array([
            [1,0,0,0],
            [0,1,0,0]
        ])

        R = np.eye(2)*self.r
        z = np.array([[meas[0]],[meas[1]]])

        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

    def get_xy(self):
        return float(self.x[0]), float(self.x[1])

# ==========================
# ROS2 PUBLISHER
# ==========================
class UwbPose2DPublisher(Node):
    def __init__(self):
        super().__init__('uwb_pose2d_publisher')
        self.pub = self.create_publisher(Pose2D, 'uwb_pose2d', 10)

    def publish(self, x, y, theta=0.0):
        msg = Pose2D()
        msg.x = float(x)
        msg.y = float(y)
        msg.theta = float(theta)
        self.pub.publish(msg)

# ==========================
# POZYX HELPERS
# ==========================
def get_position(po, rid):
    pos = Coordinates()
    ok = po.doPositioning(
        pos,
        PozyxConstants.DIMENSION_2D,
        1000,
        PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY,
        remote_id=rid
    )
    return (ok == POZYX_SUCCESS, pos)


# ==========================
# UMEYAMA
# ==========================
def umeyama_alignment(src, dst):
    mu_src = src.mean(axis=1, keepdims=True)
    mu_dst = dst.mean(axis=1, keepdims=True)

    src_c = src - mu_src
    dst_c = dst - mu_dst

    Sigma = dst_c @ src_c.T / src.shape[1]
    U, D, Vt = np.linalg.svd(Sigma)

    S = np.eye(2)
    if np.linalg.det(U @ Vt) < 0:
        S[1,1] = -1

    Rm = U @ S @ Vt
    var_src = np.sum(src_c**2) / src.shape[1]
    s = np.sum(D * np.diag(S)) / var_src

    t = mu_dst - s * Rm @ mu_src
    return s, Rm, t

# ==========================
# LOAD TRANSFORM
# ==========================
df = pd.read_csv("log.csv")
uwb = df[['kal_x', 'kal_y']].to_numpy().T
odom = df[['odom_x', 'odom_y']].to_numpy().T
s, Rm, t = umeyama_alignment(uwb, odom)

def uwb_to_odom(x, y):
    v = np.array([[x],[y]])
    return (s * Rm @ v + t).flatten()

# ==========================
# MAIN
# ==========================
def main():
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

    try:
        while rclpy.ok():
            ok, pos = get_position(pozyx, TAG_TARGET)

            if ok:
                raw_x = float(pos.x) / 1000
                raw_y = float(pos.y) / 1000

                meas = (raw_x, raw_y)

                if kalman is None:
                    kalman = Kalman2D(meas, 4000, 50000)

                kalman.predict(LOOP_DT)
                kalman.update(meas)

                fx, fy = kalman.get_xy()
                ux, uy = uwb_to_odom(fx, fy)

                ros_node.publish(ux, uy, 0.0)
                # print(f"ROS2 Pose2D → x:{ux:.3f} y:{uy:.3f}")
                # use ros2 log info instead of print if needed
                # ros_node.get_logger().info(f"ROS2 Pose2D → x:{ux:.3f} y:{uy:.3f}")

            sleep(LOOP_DT)

    except KeyboardInterrupt:
        pass

    finally:
        rclpy.shutdown()
        print("\nShutdown complete.")

# ==========================
if __name__ == "__main__":
    main()
