#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Int8

import csv
import os
from datetime import datetime


class UiCsvLogger(Node):
    def __init__(self):
        super().__init__('ui_csv_logger')

        # =========================
        # CSV SETUP
        # =========================
        log_dir = os.path.expanduser('~/ros2_logs')
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        
        self.csv_path = f'{log_dir}/zia_duduk_uwb_{ts}.csv'
        

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([
            'time',
            'robot_x', 'robot_y', 'robot_theta',
            # 'robot_mode',
            # 'robot_fsm_mode',
            # 'robot_following_mode',
            'human_x', 'human_y', 'human_theta',
            # 'human_linear', 'human_angular',
            'target_x', 'target_y', 'target_theta', 'human_detected',
        ])

        # =========================
        # DATA BUFFER
        # =========================
        self.data = {
            'robot_pose': Pose2D(),
            'robot_mode': 0,
            'robot_fsm_mode': 0,
            'robot_following_mode': 0,
            'human_pose': Pose2D(),
            'human_linear': 0.0,
            'human_angular': 0.0,
            'human_mode': 0,
            'target_pose': Pose2D()
        }

        # =========================
        # SUBSCRIBERS (MATCH EXACT TOPICS)
        # =========================
        self.create_subscription(Pose2D, '/ui/robot/pose2d', self.cb_robot_pose, 1)
        self.create_subscription(Int8,   '/ui/robot/mode', self.cb_robot_mode, 1)
        self.create_subscription(Int8,   '/ui/robot/fsm_mode', self.cb_robot_fsm, 1)
        self.create_subscription(Int8,   '/ui/robot/following_mode', self.cb_robot_following, 1)
        self.create_subscription(Pose2D, '/ui/target/nav', self.cb_nav_pose, 1)

        self.create_subscription(Pose2D, '/ui/human/pose2d', self.cb_human_pose, 1)
        self.create_subscription(Twist,  '/ui/human/velocity', self.cb_human_velocity, 1)
        self.create_subscription(Int8,   '/ui/human/mode', self.cb_human_mode, 1)

        # =========================
        # LOGGING TIMER (10 Hz)
        # =========================
        self.create_timer(0.5, self.write_csv)

        self.get_logger().info(f'Logging to CSV: {self.csv_path}')

    # =========================
    # CALLBACKS
    # =========================
    def cb_robot_pose(self, msg):
        self.data['robot_pose'] = msg

    def cb_robot_mode(self, msg):
        self.data['robot_mode'] = msg.data

    def cb_robot_fsm(self, msg):
        self.data['robot_fsm_mode'] = msg.data

    def cb_robot_following(self, msg):
        self.data['robot_following_mode'] = msg.data

    def cb_nav_pose(self, msg):
        self.data['target_pose'] = msg

    def cb_human_pose(self, msg):
        self.data['human_pose'] = msg

    def cb_human_velocity(self, msg):
        self.data['human_linear'] = msg.linear.x
        self.data['human_angular'] = msg.angular.z

    def cb_human_mode(self, msg):
        self.data['human_mode'] = msg.data

    def f2(self, v):
        return f'{v:.2f}'

    # =========================
    # CSV WRITE
    # =========================
    def write_csv(self):
        t = self.get_clock().now().nanoseconds * 1e-9

        self.writer.writerow([
            f'{t:.3f}',

            self.f2(self.data['robot_pose'].x),
            self.f2(self.data['robot_pose'].y),
            self.f2(self.data['robot_pose'].theta),

            self.f2(self.data['human_pose'].x),
            self.f2(self.data['human_pose'].y),
            self.f2(self.data['human_pose'].theta),

            self.f2(self.data['target_pose'].x),
            self.f2(self.data['target_pose'].y),
            self.f2(self.data['target_pose'].theta),

            self.data['human_mode'],
        ])

        self.csv_file.flush()


    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = UiCsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
