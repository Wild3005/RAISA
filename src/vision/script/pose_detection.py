#!/usr/bin/env python3
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, String, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import math
import json
import os

class PoseDetectorNode(Node):
    # Detection parameters
    MIN_DETECTION_CONFIDENCE = 0.5
    MIN_TRACKING_CONFIDENCE = 0.5
    TIMER_INTERVAL = 0.1  
    
    # Posture detection thresholds
    SITTING_THRESHOLD = 0.3  
    HAND_RAISE_THRESHOLD = 0.15  
    WAVING_SPEED_THRESHOLD = 0.05 
    
    ZONE_MOST_LEFT_LIMIT = 0.20
    ZONE_LEFT_LIMIT = 0.40
    ZONE_MIDDLE_LIMIT = 0.60
    ZONE_RIGHT_LIMIT = 0.80
    
    # Default distance thresholds (shoulder width in pixels)
    DISTANCE_CLOSE_THRESHOLD = 200  # px
    DISTANCE_FAR_THRESHOLD = 80     # px
    
    # Camera centering threshold
    CENTER_THRESHOLD = 0.15  # How far from center before adjustment needed
    
    def __init__(self):
        super().__init__('pose_detector_node')
        
        # === Publishers ===
        self.pub_pose_detected = self.create_publisher(Int8, '/vision/pose_detected', 1)
        self.pub_position = self.create_publisher(String, '/vision/person_position', 1)
        self.pub_orientation = self.create_publisher(String, '/vision/person_orientation', 1)
        
        # === Subscriptions ===
        self.create_subscription(Image, '/vision/image_raw', self.image_callback, 1)
        
        # === Tools ===
        self.bridge = CvBridge()
        self.mp_holistic = mp.solutions.holistic
        self.holistic = self.mp_holistic.Holistic(
            min_detection_confidence=self.MIN_DETECTION_CONFIDENCE,
            min_tracking_confidence=self.MIN_TRACKING_CONFIDENCE,
            model_complexity=1, # 0=Lite (Cepat), 1=Full, 2=Heavy
            smooth_landmarks=True,
            refine_face_landmarks=False # Set True jika butuh deteksi iris mata (lebih berat)
        )
        self.mp_pose = mp.solutions.pose
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # === State ===
        self.last_frame = None
        self.current_posture = "unknown"
        self.current_gesture = "none"
        self. prev_hand_positions = []  # For wave detection
        self.pose_detected = False
        
        # === Distance & Position Tracking ===
        self.current_distance = "unknown"  # close, medium, far
        self.current_position = "middle"   # most_left, left, middle, right, most_right
        self.shoulder_width_pixels = 0
        
        # === Camera Adjustment & Orientation ===
        self. current_camera_adjustment = "centered"  # move_left, move_right, centered, align_with_person
        self. current_orientation = "unknown"  # facing_camera, turned_away, side_view
        
        # === Calibration System ===
        self.calibration_mode = False
        self.calibration_data = {
            'close_shoulder_width': self.DISTANCE_CLOSE_THRESHOLD,
            'far_shoulder_width': self.DISTANCE_FAR_THRESHOLD
        }

        self.config_path = os.path.expanduser('~/.ros/vision_calibration.json')
        self.load_calibration()
        
        # === Timer ===
        self.create_timer(self.TIMER_INTERVAL, self.tick_timer)
        
        self.get_logger().info("✅ PoseDetectorNode active and listening on /vision/image_raw")
        self.get_logger().info(f"📏 Calibration loaded: Close={self.calibration_data['close_shoulder_width']}px, Far={self.calibration_data['far_shoulder_width']}px")
    
    # === CALIBRATION SYSTEM ===
    def load_calibration(self):
        """Load calibration data from JSON file."""
        if os.path.exists(self. config_path):
            try:
                with open(self.config_path, 'r') as f:
                    self.calibration_data = json.load(f)
                self.get_logger().info(f"Calibration loaded from {self.config_path}")
            except json.JSONDecodeError:
                self.get_logger().warn(f"Invalid calibration file, using defaults")
        else:
            self.get_logger().info("No calibration file found, using defaults")
    
    def save_calibration(self):
        """Save calibration data to JSON file."""
        os.makedirs(os.path. dirname(self.config_path), exist_ok=True)
        with open(self.config_path, 'w') as f:
            json.dump(self.calibration_data, f, indent=2)
        self.get_logger().info(f"💾 Calibration saved to {self.config_path}")
    
    def calibrate_distance(self, distance_label, shoulder_width):
        """Record shoulder width at specific distance."""
        key = f"{distance_label}_shoulder_width"
        self.calibration_data[key] = shoulder_width
        self. save_calibration()
        self.get_logger().info(f"📏 Calibrated '{distance_label}':  {shoulder_width}px")
    
    # === ROS IMAGE CALLBACK ===
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.flip(frame, 1)
            self.last_frame = frame
            self.process_pose(frame)
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")
    
    # === MAIN POSE PROCESSING ===
    def process_pose(self, frame):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.holistic.process(rgb)
        
        if results.pose_landmarks:
            self.pose_detected = True
            
            # Draw skeleton on frame
            self.mp_draw.draw_landmarks(
                frame, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())
            
            if results.face_landmarks:
                self.mp_draw.draw_landmarks(
                    frame, results.face_landmarks, self.mp_holistic.FACEMESH_TESSELATION,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_tesselation_style())

            # self.mp_draw.draw_landmarks(frame, results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)
            # self.mp_draw.draw_landmarks(frame, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)
            
            landmarks = results.pose_landmarks.landmark
            h, w, _ = frame.shape

            self.detect_orientation_holistic(landmarks, results.face_landmarks)
            self.calculate_position(landmarks)
            # self.calculate_camera_adjustment(landmarks) 
            
        else:
            self.pose_detected = False
            self.current_posture = "no_person"
            self.current_gesture = "none"
            self.current_distance = "unknown"
            self.current_position = "middle"
            self.current_camera_adjustment = "centered"
            self.current_orientation = "unknown"
        
        self.draw_hud(frame)
    
    def detect_orientation_holistic(self, pose_landmarks, face_landmarks):
        ORIENTATION_THRESHOLD = 0.005
        
        left_shoulder = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        shoulder_vis = (left_shoulder.visibility + right_shoulder.visibility) / 2

        if shoulder_vis > 0.5 and face_landmarks is None:
            self.current_orientation = "turned_away"
            return
        
        elif face_landmarks is not None:
            self.current_orientation = "facing_camera"

        if face_landmarks is not None:
            nose = pose_landmarks[self.mp_pose.PoseLandmark.NOSE]
            left_eye = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_EYE_OUTER]
            right_eye = pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_EYE_OUTER]
            
            nose_x = nose.x
            avg_eye_x = (left_eye.x + right_eye.x) / 2
            diff = avg_eye_x - nose_x
            
            if diff > ORIENTATION_THRESHOLD:
                self.current_orientation = "facing_left"
            elif diff < -ORIENTATION_THRESHOLD:
                self.current_orientation = "facing_right"
            else:
                self.current_orientation = "facing_front"
                
        else:
            self.current_orientation = "unknown"

    # === DISTANCE ESTIMATION ===
    def calculate_distance(self, landmarks, w, h):
        """Estimate distance based on shoulder width in pixels."""
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = landmarks[self. mp_pose.PoseLandmark.RIGHT_SHOULDER]
        
        # Calculate shoulder width in pixels
        shoulder_x1 = int(left_shoulder.x * w)
        shoulder_x2 = int(right_shoulder.x * w)
        shoulder_y1 = int(left_shoulder.y * h)
        shoulder_y2 = int(right_shoulder.y * h)
        
        self.shoulder_width_pixels = math.sqrt(
            (shoulder_x2 - shoulder_x1)**2 + (shoulder_y2 - shoulder_y1)**2
        )
        
        # Classify distance
        close_threshold = self.calibration_data['close_shoulder_width']
        far_threshold = self.calibration_data['far_shoulder_width']
        
        if self.shoulder_width_pixels >= close_threshold:
            self. current_distance = "close"
        elif self.shoulder_width_pixels <= far_threshold:
            self. current_distance = "far"
        else:
            self.current_distance = "medium"
    
    # === POSITION TRACKING ===
    def calculate_position(self, landmarks):
        """Determine if person is in one of 5 equal zones."""
        # Use nose position as reference (more stable than hips)
        nose = landmarks[self.mp_pose.PoseLandmark.NOSE]
        x_pos = nose.x
        
        if x_pos < self.ZONE_MOST_LEFT_LIMIT:     
            self.current_position = "most_left"
        elif x_pos < self.ZONE_LEFT_LIMIT:        
            self.current_position = "left"
        elif x_pos < self.ZONE_MIDDLE_LIMIT:      
            self.current_position = "middle"
        elif x_pos < self.ZONE_RIGHT_LIMIT:       
            self.current_position = "right"
        else:                                     
            self.current_position = "most_right"
    
    # === VISUAL HUD ===
    def draw_hud(self, frame):
        """Draw distance, position, and 5-zone markers on frame."""
        h, w = frame.shape[:2]
        
        # Calculate X coordinates for the 4 dividers (creating 5 zones)
        x1 = int(w * self. ZONE_MOST_LEFT_LIMIT)
        x2 = int(w * self.ZONE_LEFT_LIMIT)
        x3 = int(w * self. ZONE_MIDDLE_LIMIT)
        x4 = int(w * self.ZONE_RIGHT_LIMIT)
        
        # Draw vertical zone lines
        color_lines = (100, 100, 100)
        cv2.line(frame, (x1, 0), (x1, h), color_lines, 2)
        cv2.line(frame, (x2, 0), (x2, h), color_lines, 2)
        cv2.line(frame, (x3, 0), (x3, h), color_lines, 2)
        cv2.line(frame, (x4, 0), (x4, h), color_lines, 2)
        
        # Zone labels (at bottom)
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.4
        font_color = (200, 200, 200)
        
        cv2.putText(frame, "MOST L", (5, h - 10), font, font_scale, font_color, 1)
        cv2.putText(frame, "LEFT", (x1 + 5, h - 10), font, font_scale, font_color, 1)
        cv2.putText(frame, "MID", (x2 + 5, h - 10), font, font_scale, font_color, 1)
        cv2.putText(frame, "RIGHT", (x3 + 5, h - 10), font, font_scale, font_color, 1)
        cv2.putText(frame, "MOST R", (x4 + 5, h - 10), font, font_scale, font_color, 1)
        
        if self.pose_detected:
            # Distance info with color coding
            distance_colors = {"close": (0, 0, 255), "medium": (0, 165, 255), "far": (0, 255, 0)}
            dist_color = distance_colors.get(self.current_distance, (255, 255, 255))
            
            # Orientation color coding
            orientation_colors = {
                "facing_camera": (0, 255, 0), 
                "turned_away":  (0, 0, 255), 
                "facing_left":  (0,0,255), 
                "facing_right": (0,0,255),
                "unknown": (128, 128, 128)
            }
            orient_color = orientation_colors.get(self. current_orientation, (255, 255, 255))
            
            # Camera adjustment color coding
            adjustment_colors = {
                "centered": (0, 255, 0), 
                "move_left":  (0, 165, 255), 
                "move_right": (255, 165, 0),
                "align_with_person": (255, 0, 255)
            }
            adjust_color = adjustment_colors.get(self.current_camera_adjustment, (255, 255, 255))
            
            # Top-left HUD Box (expanded to fit new info)
            cv2.rectangle(frame, (5, 5), (320, 160), (0, 0, 0), -1)  # Background
            cv2.rectangle(frame, (5, 5), (320, 160), (255, 255, 255), 2)  # Border
            
            cv2.putText(frame, f"Dist: {self.current_distance. upper()}", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, dist_color, 2)
            cv2.putText(frame, f"Pos:  {self.current_position.upper()}", (15, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            cv2.putText(frame, f"Width: {int(self.shoulder_width_pixels)}px", (15, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(frame, f"Orient: {self.current_orientation. upper()}", (15, 105),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, orient_color, 2)
            cv2.putText(frame, f"Camera: {self.current_camera_adjustment.upper()}", (15, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, adjust_color, 2)
            
            # Draw camera adjustment indicator
            center_x = w // 2
            arrow_y = 50
            
            if self.current_camera_adjustment == "move_left":
                cv2.arrowedLine(frame, (center_x + 50, arrow_y), (center_x - 50, arrow_y), 
                               (0, 165, 255), 3, tipLength=0.3)
            elif self.current_camera_adjustment == "move_right": 
                cv2.arrowedLine(frame, (center_x - 50, arrow_y), (center_x + 50, arrow_y), 
                               (255, 165, 0), 3, tipLength=0.3)
            elif self.current_camera_adjustment == "align_with_person":
                # Draw circular arrow for 180° rotation
                cv2.putText(frame, "TURN 180", (center_x - 60, arrow_y + 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
                cv2.ellipse(frame, (center_x, arrow_y), (40, 40), 0, 0, 270, (255, 0, 255), 3)
                cv2.arrowedLine(frame, (center_x - 40, arrow_y), (center_x - 50, arrow_y - 10),
                               (255, 0, 255), 3, tipLength=0.5)
            
            # Highlight current zone
            highlight_color = (0, 255, 255)
            thickness = 3
            
            if self.current_position == "most_left":
                cv2.rectangle(frame, (0, 0), (x1, h), highlight_color, thickness)
            elif self.current_position == "left":
                cv2.rectangle(frame, (x1, 0), (x2, h), highlight_color, thickness)
            elif self.current_position == "middle":
                cv2.rectangle(frame, (x2, 0), (x3, h), highlight_color, thickness)
            elif self.current_position == "right":
                cv2.rectangle(frame, (x3, 0), (x4, h), highlight_color, thickness)
            elif self.current_position == "most_right":
                cv2.rectangle(frame, (x4, 0), (w, h), highlight_color, thickness)
    
    # === TIMER ROUTINE PUBLISHING ===
    def tick_timer(self):
        self.pub_pose_detected.publish(Int8(data=int(self.pose_detected)))
        
        position_msg = String()
        position_msg.data = self.current_position
        self.pub_position.publish(position_msg)
        
        orientation_msg = String()
        orientation_msg.data = self.current_orientation
        self.pub_orientation.publish(orientation_msg)
    
    # === CLEANUP ===
    def destroy_node(self):
        """Release resources on shutdown."""
        self.pose. close()
        self.hands.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PoseDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down PoseDetectorNode...")
    finally:
        node. destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()