#!/usr/bin/env python3
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import sys

# =======================
# MediaPipe Init
# =======================
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# =======================
# Config
# =======================
MAX_COOLDOWN_HAND_ACTION = 100
# CAMERA_INDEX = 0
CAMERA_INDEX = '/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_078E232F-video-index0'
# CAMERA_INDEX = '/dev/v4l/by-id/usb-046d_C922_Pro_Stream_Webcam_3BD7DCCF-video-index0' #path cam robt raisa
FPS = 30.0


class HandTrack(Node):
    def __init__(self):
        super().__init__('hand_track_local_debug_node')

        # =======================
        # CV & MediaPipe
        # =======================
        self.bridge = CvBridge()
        self.hands = mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.4,
            min_tracking_confidence=0.6
        )

        # Kamera
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error(f"Gagal membuka kamera index {CAMERA_INDEX}")
            self.cap = None
            return

        # =======================
        # QoS (WAJIB untuk web_video_server)
        # =======================
        qos_image = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        qos_default = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # =======================
        # ROS Publishers
        # =======================
        self.pub_hand_data = self.create_publisher(
            Int8, '/cmd/hand_stop', qos_default
        )

        self.pub_hand_action_ready = self.create_publisher(
            Int8, '/vision/hand_action_ready', qos_default
        )
        
        self.pub_frame = self.create_publisher(
            Image,
            '/vision/hand_frame',
            qos_image
        )


        # =======================
        # State
        # =======================
        self.pref_gesture = -1
        self.gesture = 0
        self.cooldown_hand_action = MAX_COOLDOWN_HAND_ACTION
        self.cntr_hand_detected = 0

        # =======================
        # Timer
        # =======================
        self.timer = self.create_timer(1.0 / FPS, self.timer_callback)

        self.get_logger().info(
            "HandTrack ACTIVE | DEBUG LOCAL + WEB_VIDEO_SERVER"
        )

    # =======================
    # Gesture Logic
    # =======================
    def is_hand_open(self, lm):
        return (
            lm[8].y < lm[6].y and
            lm[12].y < lm[10].y and
            lm[16].y < lm[14].y and
            lm[20].y < lm[18].y
        )

    # =======================
    # Timer Callback
    # =======================
    def timer_callback(self):
        if self.cap is None:
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Frame kamera gagal dibaca")
            return

        frame = cv2.flip(frame, 1)
        self.process_frame(frame)

    # =======================
    # Main Processing
    # =======================
    def process_frame(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        H, W, _ = frame.shape
        mid_y = H // 2
        self.gesture = 0

        # Visual guide
        cv2.line(frame, (0, mid_y), (W, mid_y), (0, 0, 255), 2)
        cv2.putText(
            frame, "DETECTION AREA (Y < H/2)",
            (10, mid_y - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (0, 0, 255), 2
        )

        max_area = 0
        selected = None
        bbox = None

        if results.multi_hand_landmarks:
            for hand in results.multi_hand_landmarks:
                if int(hand.landmark[9].y * H) > mid_y:
                    continue

                mp_drawing.draw_landmarks(
                    frame, hand, mp_hands.HAND_CONNECTIONS
                )

                x_min = min(int(l.x * W) for l in hand.landmark)
                x_max = max(int(l.x * W) for l in hand.landmark)
                y_min = min(int(l.y * H) for l in hand.landmark)
                y_max = max(int(l.y * H) for l in hand.landmark)

                area = (x_max - x_min) * (y_max - y_min)
                if area > max_area:
                    max_area = area
                    selected = hand
                    bbox = (x_min, y_min, x_max, y_max)

        if selected:
            self.cntr_hand_detected += 1
            if self.cntr_hand_detected > 5:
                lm = selected.landmark
                x_min, y_min, x_max, y_max = bbox

                if self.is_hand_open(lm):
                    self.gesture = 1  # STOP
                    cv2.rectangle(
                        frame, (x_min, y_min),
                        (x_max, y_max),
                        (0, 0, 255), 4
                    )
                else:
                    self.gesture = 0
                    cv2.rectangle(
                        frame, (x_min, y_min),
                        (x_max, y_max),
                        (0, 255, 0), 2
                    )
        else:
            self.cntr_hand_detected = 0

        # # Debug local
        # cv2.imshow("HandTrack DEBUG (LOCAL)", frame)
        # cv2.waitKey(1)

        # Publish
        self.publish_data(frame.copy())

    # =======================
    # Publish
    # =======================
    def publish_data(self, frame):
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub_frame.publish(img_msg)

        ready = Int8()
        ready.data = 1 if self.cntr_hand_detected > 0 else 0
        self.pub_hand_action_ready.publish(ready)

        if self.cooldown_hand_action > 0:
            self.cooldown_hand_action -= 1
        else:
            if self.gesture != self.pref_gesture:
                self.pref_gesture = self.gesture
                cmd = Int8()
                cmd.data = self.gesture  # 1=STOP, 0=RUN
                self.pub_hand_data.publish(cmd)
                self.cooldown_hand_action = MAX_COOLDOWN_HAND_ACTION

    # =======================
    # Cleanup
    # =======================
    def destroy_node(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = HandTrack()
    if node.cap is None:
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        rclpy.shutdown()

if __name__ == '__main__':
    main()