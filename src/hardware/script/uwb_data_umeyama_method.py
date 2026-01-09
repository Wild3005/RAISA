#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import math

import sys, time, csv, termios, tty, select, requests
from datetime import datetime
import numpy as np
import pandas as pd
from time import sleep
from pypozyx import (
    PozyxSerial, get_first_pozyx_serial_port, PozyxConstants,
    Coordinates, DeviceCoordinates, SensorData, POZYX_SUCCESS
)

MANUAL_OFFSET = 90.0

# -----------------------------
# Fungsi pembacaan yaw mentah
# -----------------------------
def read_sensors(po, rid):
    s = SensorData()
    ok = po.getAllSensorData(s, rid)
    return s if ok == POZYX_SUCCESS else None

def get_raw_yaw(po, rid):
    s = read_sensors(po, rid)
    if s is None:
        return None

    try:
        yaw = float(s.euler_angles.heading)
        if yaw > 180:
            yaw -= 360
        return yaw
    except:
        return None


# -----------------------------
# Regres X Y seperti aslinya
# -----------------------------
termsX2_2 = [-0.12838861917401223, 1.0450381308097056]
termsY2_2 = [-0.39182911446895652, 0.99020003655176714]

def regressX(x):
  t=1; r=0
  for c in termsX2_2: r += c*t; t*=x
  return r

def regressY(x):
  t=1; r=0
  for c in termsY2_2: r += c*t; t*=x
  return r


TAG_TARGET = 0x6800
CSV_FILE   = "position_log.csv"
LOOP_DT    = 0.25


OFFSET_X = 3325
OFFSET_Y = 831
ANCHORS = [
    DeviceCoordinates(0x6722, 1, Coordinates(0-OFFSET_X,0-OFFSET_Y,1109)),
    DeviceCoordinates(0x6772, 1, Coordinates(9210-OFFSET_X,-1154-OFFSET_Y,1637)),
    DeviceCoordinates(0x6764, 1, Coordinates(11591-OFFSET_X,8201-OFFSET_Y,480)),
    DeviceCoordinates(0x671D, 1, Coordinates(604-OFFSET_X,8235-OFFSET_Y,1897)),
]


# KALMAN 2D seperti asli (TIDAK DIUBAH)
class Kalman2D:
    def __init__(self, m, q=100.0, r=40000.0):
        self.x=np.array([[m[0]],[m[1]],[0.0],[0.0]])
        self.P=np.diag([500,500,500,500])
        self.q=q; self.r=r

    def predict(self,dt):
        F=np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]])
        G=np.array([[0.5*dt*dt,0],[0,0.5*dt*dt],[dt,0],[0,dt]])
        Q=G@(np.eye(2)*self.q)@G.T
        self.x=F@self.x
        self.P=F@self.P@F.T+Q

    def update(self,m):
        H=np.array([[1,0,0,0],[0,1,0,0]])
        R=np.eye(2)*self.r
        z=np.array([[m[0]],[m[1]]])
        y=z-H@self.x
        S=H@self.P@H.T+R
        K=self.P@H.T@np.linalg.inv(S)
        self.x=self.x+K@y
        self.P=(np.eye(4)-K@H)@self.P

    def get_xy(self):
        return float(self.x[0]), float(self.x[1])



# ---------------------------------------------
def get_position(po,rid):
    pos=Coordinates()
    ok=po.doPositioning(pos,PozyxConstants.DIMENSION_2D,200,
                        PozyxConstants.POSITIONING_ALGORITHM_UWB_ONLY,
                        remote_id=rid)
    return ok==POZYX_SUCCESS,pos

# --------------------------------------------- MAIN
class UWBNode(Node):
    def __init__(self):
        super().__init__('uwb_node')

        self.pub = self.create_publisher(Pose2D, "/uwb/pose", 10)

        self.timer = self.create_timer(0.3, self.loop)

        # --- Inisialisasi UWB seperti kode Anda ---
        port = get_first_pozyx_serial_port()
        if not port:
            self.get_logger().error("No Pozyx detected.")
            return

        self.pozyx = PozyxSerial(port)
        self.get_logger().info(f"Connected: {port}")

        self.pozyx.clearDevices(TAG_TARGET)
        for a in ANCHORS:
            self.pozyx.addDevice(a, TAG_TARGET)

        self.kalman = None
        self.yaw_offset = None

    # ----------------------------------------------------
    def loop(self):
        ok, pos = get_position(self.pozyx, TAG_TARGET)
        if not ok:
            return

        # Kalman filtering
        raw_x = float(pos.x) / 1000
        raw_y = float(pos.y) / 1000
        meas = (raw_x, raw_y)

        if self.kalman is None:
            self.kalman = Kalman2D(meas, 2500, 5000)

        self.kalman.predict(0.25)
        self.kalman.update(meas)
        fx, fy = self.kalman.get_xy()

        # Yaw
        yaw = get_raw_yaw(self.pozyx, TAG_TARGET)
        if yaw is not None:
            if self.yaw_offset is None:
                self.yaw_offset = yaw
            yaw0 = yaw - self.yaw_offset
        else:
            yaw0 = 0.0

        # --------------------------
        # Publish ke ROS
        # --------------------------
        msg = Pose2D()
        msg.x = round(fx, 3)
        msg.y = round(fy, 3)
        yaw_rad = math.radians(yaw0)        # derajat -> radian
        msg.theta = round(yaw_rad, 6)

        self.pub.publish(msg)

        self.get_logger().info(
            f"x={fx:.3f}, y={fy:.3f}, yaw_deg={yaw0:.2f}, yaw_rad={yaw_rad:.4f}"
        )


# ----------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = UWBNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()