#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_dual_leg;

from std_msgs.msg import Float32MultiArray
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
        log_dir = os.path.expanduser('~/raisa_humanoid/logger_csv')
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        
        self.csv_path = f'{log_dir}/aslam_uwb_{ts}.csv'
        

        self.csv_file = open(self.csv_path, 'w', newline='')
        self.writer = csv.writer(self.csv_file)

        self.writer.writerow([
            'time',
            'robot_x', 'robot_y', 'robot_theta',
            'human_x', 'human_y', 'human_theta',
            'target_x', 'target_y', 'target_theta',
            'leg_l', 'leg_r',
            'human_moving', 'human_sitting', 'human_detected'
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
            'target_pose': Pose2D(),
            'leg_l': 0.0,
            'leg_r': 0.0,
            'human_mode': 0,
            'human_sitting': 0,
            'human_detected': 0,
        }

        # =========================
        # SUBSCRIBERS (MATCH EXACT TOPICS)
        # =========================
        self.create_subscription(Pose2D, '/ui/robot/pose2d', self.cb_robot_pose, 1)
        self.create_subscription(Int8,   '/ui/robot/mode', self.cb_robot_mode, 1)
        self.create_subscription(Int8,   '/ui/robot/fsm_mode', self.cb_robot_fsm, 1)
        self.create_subscription(Int8,   '/ui/robot/following_mode', self.cb_robot_following, 1)
        self.create_subscription(Pose2D, '/ui/target/nav', self.cb_nav_pose, 1)
        self.create_subscription(Float32MultiArray, '/dual_leg', self.cb_dual_leg, 1)
        

        self.create_subscription(Pose2D, '/ui/human/pose2d', self.cb_human_pose, 1)
        self.create_subscription(Twist,  '/ui/human/velocity', self.cb_human_velocity, 1)
        self.create_subscription(Int8,   '/ui/human/mode', self.cb_human_mode, 1)
        self.create_subscription(Int8,   '/ui/human/detected', self.cb_human_detected, 1)
        self.create_subscription(Int8,   '/ui/human/sitting', self.cb_human_sitting, 1)

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

    def cb_human_detected(self, msg):
        self.data['human_detected'] = msg.data  

    def cb_human_sitting(self, msg):
        self.data['human_sitting'] = msg.data

    def f2(self, v):
        return f'{v:.2f}'
    
    def cb_dual_leg(self, msg):

        if len(msg.data) >= 2:
            self.data['leg_l'] = msg.data[0]
            self.data['leg_r'] = msg.data[1]
        elif len(msg.data) == 1:
            self.data['leg_l'] = msg.data[0]
            self.data['leg_r'] = msg.data[0]
        else:
            self.data['leg_l'] = 0.0
            self.data['leg_r'] = 0.0
        
        pass

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

            self.f2(self.data['leg_l']),
            self.f2(self.data['leg_r']),

            self.data['human_mode'],
            self.data['human_sitting'],
            self.data['human_detected']
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
