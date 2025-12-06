#!/usr/bin/env python3
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class FaceDetectorNode(Node):
    MAX_TIMER_OPEN_WINDOW = 5.0      # seconds
    MAX_FACE_DETECT_COUNT = 5
    TIMER_INTERVAL = 0.2             # seconds
    GREETING_COOLDOWN_TIME = 20.0    # seconds

    def __init__(self):
        super().__init__('face_detector_node')

        # === Publishers ===
        self.pub_detected = self.create_publisher(Int8, '/vision/face_detected', 1)
        self.pub_audio_greeting = self.create_publisher(Int8, '/vision/play_greeting', 1)
        self.pub_frame = self.create_publisher(Image, '/vision/face_frame', 1)

        # === Subscriptions ===
        self.create_subscription(Image, '/vision/image_raw', self.image_callback, 10)

        # === Tools ===
        self.bridge = CvBridge()
        self.mp_face = mp.solutions.face_detection.FaceDetection(model_selection=0, min_detection_confidence=0.6)
        self.mp_draw = mp.solutions.drawing_utils

        # === State ===
        self.last_frame = None
        self.open_window = False
        self.prev_open_window = False
        self.detecting = False
        self.timer_open_window = self.MAX_TIMER_OPEN_WINDOW
        self.face_detected_counter = 0
        self.greeting_cooldown = 0.0
        self.scale_factor = 0.05

        # === Timers ===
        self.create_timer(self.TIMER_INTERVAL, self.tick_timer)

        self.get_logger().info("✅ FaceDetectorNode active and listening on /vision/image_raw")

    # === ROS IMAGE CALLBACK ===
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.last_frame = frame

        # Skip detection if still in cooldown
        if not self.detecting:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.mp_face.process(rgb)

            if results.detections:
                for detection in results.detections:
                    self.mp_draw.draw_detection(frame, detection)
                    self.face_detected_counter += 1

                    bboxC = detection.location_data.relative_bounding_box
                    ih, iw, _ = frame.shape
                    bbox = (
                        int(bboxC.xmin * iw),
                        int(bboxC.ymin * ih),
                        int(bboxC.width * iw),
                        int(bboxC.height * ih),
                    )

                    area_bbox = bbox[2] * bbox[3]
                    area_frame = iw * ih

                    # Jika area wajah cukup besar dan sudah stabil (counter cukup)
                    if (area_bbox > self.scale_factor * area_frame) and (
                        self.face_detected_counter >= self.MAX_FACE_DETECT_COUNT
                    ):
                        self._open_window()

            else:
                self.face_detected_counter = 0

        # Jika tidak open window, tampilkan overlay hitam
        if not self.open_window:
            cv2.rectangle(frame, (0, 0), (frame.shape[1], frame.shape[0]), (0, 0, 0), -1)

        # Publish annotated frame
        # self._publish_frame(frame)

    # === TIMER ROUTINE ===
    def tick_timer(self):
        # Handle open window timer
        if self.open_window:
            self.timer_open_window -= self.TIMER_INTERVAL
            if self.timer_open_window <= 1.0:
                self.detecting = False
            if self.timer_open_window <= 0:
                self._close_window()

        # Trigger greeting sound
        if self.open_window and not self.prev_open_window and self.greeting_cooldown <= 0:
            self.pub_detected.publish(Int8(data=int(1)))
            self.greeting_cooldown = self.GREETING_COOLDOWN_TIME
            self.get_logger().info("👋 Greeting triggered!")

        # Adaptive sensitivity (supaya tidak terlalu sering trigger)
        self.scale_factor = 0.001 if self.open_window and self.prev_open_window else 0.05

        # Cooldown
        if self.greeting_cooldown > 0:
            self.greeting_cooldown -= self.TIMER_INTERVAL

        # Update previous state
        self.prev_open_window = self.open_window

        # Publish detection flag
        self.pub_detected.publish(Int8(data=int(self.open_window)))

    # === INTERNAL HELPERS ===
    def _open_window(self):
        self.open_window = True
        self.detecting = True
        self.timer_open_window = self.MAX_TIMER_OPEN_WINDOW
        self.face_detected_counter = 0

    def _close_window(self):
        self.open_window = False
        self.detecting = False
        self.timer_open_window = self.MAX_TIMER_OPEN_WINDOW

    def _publish_frame(self, frame):
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.pub_frame.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing frame: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🧹 Shutting down FaceDetectorNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
