#!/usr/bin/env python3
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int16MultiArray
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import threading
import queue
import time

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

MAX_COOLDOWN_HAND_ACTION = 100

class HandTrack(Node):
    def __init__(self):
        super().__init__('hand_track_node')

        # Subscriptions
        self.sub_image = self.create_subscription(
            Image, '/vision/image_raw', self.image_callback, 10
        )

        # Publishers
        self.pub_hand_data = self.create_publisher(Int16MultiArray, '/vision/hand_data', 1)
        self.pub_hand_action_ready = self.create_publisher(Int8, '/vision/hand_action_ready', 1)
        self.pub_frame = self.create_publisher(Image, '/vision/hand_frame', 1)

        # ROS & CV utilities
        self.bridge = CvBridge()
        self.hands = mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.4,
            min_tracking_confidence=0.6
        )

        # Thread-safe queue for frames
        self.frame_queue = queue.Queue(maxsize=2)

        # State variables
        self.pref_gesture = -1
        self.gesture = -1
        self.cooldown_hand_action = MAX_COOLDOWN_HAND_ACTION
        self.cntr_hand_detected = 5;

        # Start background inference thread
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.inference_routine, daemon=True)
        self.thread.start()

        self.get_logger().info("HandTrack Node Active. Using threaded inference routine.")

    # === ROS CALLBACK ===
    def image_callback(self, msg):
        """ROS callback — only enqueue frame, don't process."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if not self.frame_queue.full():
                self.frame_queue.put(frame)
        except Exception as e:
            self.get_logger().error(f"Frame enqueue failed: {e}")

    # === INFERENCE ROUTINE ===
    def inference_routine(self):
        """Runs in a separate thread; handles MediaPipe inference."""
        while not self.stop_event.is_set():
            if self.frame_queue.empty():
                time.sleep(0.005)
                continue

            frame = self.frame_queue.get()
            self.process_frame(frame)

    # === MAIN PROCESSING ===
    def process_frame(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        mid_x = frame.shape[1] // 2
        mid_y = frame.shape[0] // 3
        hand_pos_x = 0
        self.gesture = -1

        # Visual guides
        cv2.line(frame, (0, mid_y), (frame.shape[1], mid_y), (0, 0, 255), 2)
        cv2.line(frame, (mid_x + 100, 0), (mid_x + 100, frame.shape[0]), (0, 0, 255), 2)
        cv2.line(frame, (mid_x - 100, 0), (mid_x - 100, frame.shape[0]), (0, 0, 255), 2)

        max_area = 0
        selected = None

        if results.multi_hand_landmarks:
            hands_upper = [
                h for h in results.multi_hand_landmarks
                if int(h.landmark[9].y * frame.shape[0]) < (mid_y)
            ]

            for hand in hands_upper:
                mp_drawing.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)
                x_min = min(int(l.x * frame.shape[1]) for l in hand.landmark)
                x_max = max(int(l.x * frame.shape[1]) for l in hand.landmark)
                y_min = min(int(l.y * frame.shape[0]) for l in hand.landmark)
                y_max = max(int(l.y * frame.shape[0]) for l in hand.landmark)
                area = (x_max - x_min) * (y_max - y_min)

                cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                if area > max_area:
                    max_area = area
                    selected = hand

            if selected:
                self.cntr_hand_detected += 1
                if self.cntr_hand_detected > 5:
                    lm = selected.landmark
                    px = int(lm[9].x * frame.shape[1])
                    py = int(lm[9].y * frame.shape[0])
                    hand_pos_x = px - mid_x
                    cv2.circle(frame, (px, py), 10, (255, 0, 0), -1)

                    if self.is_hand_open(lm):
                        self.gesture = 1
                    elif self.is_fist(lm):
                        self.gesture = 1

        # Publish data
        self.publish_data(frame, hand_pos_x)

    # === LOGIC HELPERS ===
    def is_hand_open(self, lm):
        return (lm[8].y < lm[6].y and lm[12].y < lm[10].y and
                lm[16].y < lm[14].y and lm[20].y < lm[18].y)

    def is_fist(self, lm):
        return (lm[8].y > lm[6].y and lm[12].y > lm[10].y and
                lm[16].y > lm[14].y and lm[20].y > lm[18].y)

    def publish_data(self, frame, hand_pos_x):
        # Action ready trigger
        if self.pref_gesture == -1 and self.gesture == 1 and self.cooldown_hand_action <= 0:
            # self.pub_hand_action_ready.publish(Int8(data=1))
            self.cooldown_hand_action = MAX_COOLDOWN_HAND_ACTION

        if self.cooldown_hand_action > 0:
            self.cooldown_hand_action -= 1

        self.pref_gesture = self.gesture

        # Publish combined data [gesture, pos_x]
        msg = Int16MultiArray()
        msg.data = [self.gesture, hand_pos_x]
        self.pub_hand_data.publish(msg)

        # Publish debug frame
        # img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        # self.pub_frame.publish(img_msg)

    # === CLEANUP ===
    def destroy_node(self):
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandTrack()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
