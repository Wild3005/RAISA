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
    
    def __init__(self):
        super().__init__('pose_detector_node')
        
        # === Publishers ===
        self.pub_pose_detected = self.create_publisher(Int8, '/vision/pose_detected', 1)
        self.pub_posture = self.create_publisher(String, '/vision/posture', 1)
        self.pub_gesture = self.create_publisher(String, '/vision/gesture', 1)
        self.pub_landmarks = self.create_publisher(Float32MultiArray, '/vision/pose_landmarks', 1)
        self.pub_frame = self.create_publisher(Image, '/vision/pose_frame', 1)
        self.pub_distance = self.create_publisher(String, '/vision/person_distance', 1)
        self.pub_position = self.create_publisher(String, '/vision/person_position', 1)
        
        # === Subscriptions ===
        self.create_subscription(Image, '/vision/image_raw', self.image_callback, 10)
        
        # === Tools ===
        self.bridge = CvBridge()
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(
            min_detection_confidence=self.MIN_DETECTION_CONFIDENCE,
            min_tracking_confidence=self.MIN_TRACKING_CONFIDENCE,
            model_complexity=1  # 0=lite, 1=full, 2=heavy
        )
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # === State ===
        self.last_frame = None
        self.current_posture = "unknown"
        self.current_gesture = "none"
        self.prev_hand_positions = []  # For wave detection
        self.pose_detected = False
        
        # === Distance & Position Tracking ===
        self.current_distance = "unknown"  # close, medium, far
        self.current_position = "middle"   # most_left, left, middle, right, most_right
        self.shoulder_width_pixels = 0
        
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
        if os.path.exists(self.config_path):
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
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        with open(self.config_path, 'w') as f:
            json.dump(self.calibration_data, f, indent=2)
        self.get_logger().info(f"💾 Calibration saved to {self.config_path}")
    
    def calibrate_distance(self, distance_label, shoulder_width):
        """Record shoulder width at specific distance."""
        key = f"{distance_label}_shoulder_width"
        self.calibration_data[key] = shoulder_width
        self.save_calibration()
        self.get_logger().info(f"📏 Calibrated '{distance_label}': {shoulder_width}px")
    
    # === ROS IMAGE CALLBACK ===
    def image_callback(self, msg):
        """Receives camera frames and processes pose detection."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.flip(frame, 1)
            self.last_frame = frame
            self.process_pose(frame)
        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")
    
    # === MAIN POSE PROCESSING ===
    def process_pose(self, frame):
        """Run MediaPipe Pose detection and extract landmarks."""
        # Convert BGR to RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pose_results = self.pose.process(rgb)
        hand_results = self.hands.process(rgb)
        
        if pose_results.pose_landmarks:
            self.pose_detected = True
            
            # Draw skeleton on frame
            self.mp_draw.draw_landmarks(
                frame,
                pose_results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
            )
            
            # Draw hands if detected
            if hand_results.multi_hand_landmarks:
                for hand_landmarks in hand_results.multi_hand_landmarks:
                    self.mp_draw.draw_landmarks(
                        frame,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS
                    )
            
            # Extract landmarks for analysis
            landmarks = pose_results.pose_landmarks.landmark
            h, w, _ = frame.shape
            
            # Calculate distance and position
            self.calculate_distance(landmarks, w, h)
            self.calculate_position(landmarks)
            
            # Analyze posture and gestures
            self.analyze_posture(landmarks, h, w, frame)
            self.analyze_gestures(landmarks, h, w, frame, hand_results)
            
            # Publish landmark data
            self.publish_landmarks(landmarks)
            
        else:
            self.pose_detected = False
            self.current_posture = "no_person"
            self.current_gesture = "none"
            self.current_distance = "unknown"
            self.current_position = "middle"
        
        # Draw visual HUD
        self.draw_hud(frame)
        
        # Publish annotated frame
        self.publish_frame(frame)
    
    # === DISTANCE ESTIMATION ===
    def calculate_distance(self, landmarks, w, h):
        """Estimate distance based on shoulder width in pixels."""
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        
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
            self.current_distance = "close"
        elif self.shoulder_width_pixels <= far_threshold:
            self.current_distance = "far"
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
        x1 = int(w * self.ZONE_MOST_LEFT_LIMIT)
        x2 = int(w * self.ZONE_LEFT_LIMIT)
        x3 = int(w * self.ZONE_MIDDLE_LIMIT)
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
            
            # Top-left HUD Box
            cv2.rectangle(frame, (5, 5), (280, 100), (0, 0, 0), -1)  # Background
            cv2.rectangle(frame, (5, 5), (280, 100), (255, 255, 255), 2)  # Border
            
            cv2.putText(frame, f"Dist: {self.current_distance.upper()}", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, dist_color, 2)
            cv2.putText(frame, f"Pos: {self.current_position.upper()}", (15, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(frame, f"Width: {int(self.shoulder_width_pixels)}px", (15, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
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
        
        # Calibration mode indicator
        if self.calibration_mode:
            cv2.rectangle(frame, (w - 250, 10), (w - 10, 80), (0, 0, 0), -1)
            cv2.rectangle(frame, (w - 250, 10), (w - 10, 80), (0, 255, 255), 3)
            cv2.putText(frame, "CALIBRATION MODE", (w - 240, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, "Press 'c' = close", (w - 240, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.putText(frame, "Press 'f' = far", (w - 240, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
    
    # === POSTURE ANALYSIS ===
    def analyze_posture(self, landmarks, h, w, frame):
        """Classify body posture: standing, sitting, lying, crouching."""
        # Get key landmarks
        nose = landmarks[self.mp_pose.PoseLandmark.NOSE]
        left_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
        right_hip = landmarks[self.mp_pose.PoseLandmark.RIGHT_HIP]
        left_knee = landmarks[self.mp_pose.PoseLandmark.LEFT_KNEE]
        right_knee = landmarks[self.mp_pose.PoseLandmark.RIGHT_KNEE]
        left_ankle = landmarks[self.mp_pose.PoseLandmark.LEFT_ANKLE]
        right_ankle = landmarks[self.mp_pose.PoseLandmark.RIGHT_ANKLE]
        
        # Calculate average hip and knee positions
        hip_y = (left_hip.y + right_hip.y) / 2
        knee_y = (left_knee.y + right_knee.y) / 2
        ankle_y = (left_ankle.y + right_ankle.y) / 2
        
        # Calculate body segment ratios
        torso_length = abs(nose.y - hip_y)
        thigh_length = abs(hip_y - knee_y)
        shin_length = abs(knee_y - ankle_y)
        
        # Posture classification logic
        if torso_length < 0.1:  # Very compressed torso
            posture = "lying"
        elif thigh_length < self.SITTING_THRESHOLD * torso_length:
            # Knees close to hips = sitting
            posture = "sitting"
        elif nose.y < 0.3:  # Head in upper part of frame
            posture = "standing"
        elif knee_y - hip_y < 0.1:  # Knees near hips
            posture = "crouching"
        else:
            posture = "standing"
        
        self.current_posture = posture
        
        # Draw posture label on frame
        cv2.putText(frame, f"Posture: {posture.upper()}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # === GESTURE ANALYSIS ===
    def analyze_gestures(self, landmarks, h, w, frame, hand_results):
        """Detect gestures: waving, hands raised, pointing, thumbs up, peace, OK sign, arms crossed."""
        left_wrist = landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST]
        right_wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST]
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        left_elbow = landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW]
        right_elbow = landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW]
        nose = landmarks[self.mp_pose.PoseLandmark.NOSE]
        
        gesture = "none"
        
        # Check for hand-based gestures first (higher priority)
        if hand_results.multi_hand_landmarks:
            hand_gesture = self.detect_hand_gestures(hand_results, frame)
            if hand_gesture != "none":
                gesture = hand_gesture
                self.current_gesture = gesture
                cv2.putText(frame, f"Gesture: {gesture.upper()}", (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
                return
        
        # Check if either hand is raised above shoulder
        left_hand_raised = left_wrist.y < left_shoulder.y - self.HAND_RAISE_THRESHOLD
        right_hand_raised = right_wrist.y < right_shoulder.y - self.HAND_RAISE_THRESHOLD
        
        # Check if hand is raised above head (asking attention)
        left_hand_above_head = left_wrist.y < nose.y - 0.1
        right_hand_above_head = right_wrist.y < nose.y - 0.1
        
        if left_hand_above_head or right_hand_above_head:
            gesture = "hand_raised"
        elif left_hand_raised and right_hand_raised:
            gesture = "both_hands_up"
        elif self.detect_waving(left_wrist, right_wrist):
            gesture = "waving"
        elif self.detect_arms_crossed(left_wrist, right_wrist, left_shoulder, right_shoulder):
            gesture = "arms_crossed"
        elif left_hand_raised:
            gesture = "left_hand_up"
        elif right_hand_raised:
            gesture = "right_hand_up"
        
        self.current_gesture = gesture
        
        # Draw gesture label on frame
        if gesture != "none":
            cv2.putText(frame, f"Gesture: {gesture.upper()}", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
    
    # === HAND GESTURE DETECTION ===
    def detect_hand_gestures(self, hand_results, frame):
        """Detect finger-based gestures using MediaPipe Hands."""
        for hand_landmarks, handedness in zip(hand_results.multi_hand_landmarks, 
                                                hand_results.multi_handedness):
            hand_label = handedness.classification[0].label  # "Left" or "Right"
            
            # Check for thumbs up
            if self.is_thumbs_up(hand_landmarks):
                return f"thumbs_up_{hand_label.lower()}"
            
            # Check for pointing
            if self.is_pointing(hand_landmarks):
                return f"pointing_{hand_label.lower()}"
            
            # Check for peace sign
            if self.is_peace_sign(hand_landmarks):
                return f"peace_sign_{hand_label.lower()}"
            
            # Check for OK sign
            if self.is_ok_sign(hand_landmarks):
                return f"ok_sign_{hand_label.lower()}"
        
        return "none"
    
    def is_thumbs_up(self, hand_landmarks):
        """Detect thumbs up gesture."""
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        thumb_ip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_IP]
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        ring_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        pinky_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        
        # Thumb extended upward
        thumb_up = thumb_tip.y < thumb_ip.y < wrist.y
        
        # Other fingers curled
        fingers_down = (index_tip.y > wrist.y and middle_tip.y > wrist.y and 
                        ring_tip.y > wrist.y and pinky_tip.y > wrist.y)
        
        return thumb_up and fingers_down
    
    def is_pointing(self, hand_landmarks):
        """Detect pointing gesture (index finger extended, others curled)."""
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        index_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP]
        middle_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        middle_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
        ring_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        ring_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP]
        pinky_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]
        pinky_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_PIP]
        
        # Index extended
        index_extended = index_tip.y < index_pip.y
        
        # Other fingers curled
        others_curled = (middle_tip.y > middle_pip.y and 
                         ring_tip.y > ring_pip.y and 
                         pinky_tip.y > pinky_pip.y)
        
        return index_extended and others_curled
    
    def is_peace_sign(self, hand_landmarks):
        """Detect peace sign (index and middle extended, others curled)."""
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        index_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP]
        middle_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        middle_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
        ring_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        ring_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP]
        pinky_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_TIP]
        pinky_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.PINKY_PIP]
        
        # Index and middle extended
        index_extended = index_tip.y < index_pip.y
        middle_extended = middle_tip.y < middle_pip.y
        
        # Ring and pinky curled
        others_curled = ring_tip.y > ring_pip.y and pinky_tip.y > pinky_pip.y
        
        return index_extended and middle_extended and others_curled
    
    def is_ok_sign(self, hand_landmarks):
        """Detect OK sign (thumb and index forming circle)."""
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.THUMB_TIP]
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        middle_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        middle_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
        
        # Thumb and index close together
        distance = math.sqrt((thumb_tip.x - index_tip.x)**2 + 
                            (thumb_tip.y - index_tip.y)**2)
        
        circle_formed = distance < 0.05
        
        # Other fingers extended
        other_fingers_up = middle_tip.y < middle_pip.y
        
        return circle_formed and other_fingers_up
    
    # === GESTURE DETECTION HELPERS ===
    def detect_waving(self, left_wrist, right_wrist):
        """Detect waving by tracking hand movement speed."""
        # Store current hand positions
        current_pos = (left_wrist.x, right_wrist.x)
        self.prev_hand_positions.append(current_pos)
        
        # Keep only last 10 positions
        if len(self.prev_hand_positions) > 10:
            self.prev_hand_positions.pop(0)
        
        # Need at least 5 frames to detect wave
        if len(self.prev_hand_positions) < 5:
            return False
        
        # Calculate horizontal movement variance
        left_movements = [abs(self.prev_hand_positions[i][0] - self.prev_hand_positions[i-1][0]) 
                          for i in range(1, len(self.prev_hand_positions))]
        right_movements = [abs(self.prev_hand_positions[i][1] - self.prev_hand_positions[i-1][1]) 
                           for i in range(1, len(self.prev_hand_positions))]
        
        # If either hand moves significantly and repeatedly
        left_waving = sum(left_movements) > self.WAVING_SPEED_THRESHOLD
        right_waving = sum(right_movements) > self.WAVING_SPEED_THRESHOLD
        
        return left_waving or right_waving
    
    def detect_arms_crossed(self, left_wrist, right_wrist, left_shoulder, right_shoulder):
        """Detect if arms are crossed in front of body."""
        # Check if wrists cross the midline
        left_wrist_crossed = left_wrist.x > right_shoulder.x
        right_wrist_crossed = right_wrist.x < left_shoulder.x
        
        # Both wrists should be near chest level
        chest_level = (left_shoulder.y + right_shoulder.y) / 2
        wrists_at_chest = (abs(left_wrist.y - chest_level) < 0.2 and 
                           abs(right_wrist.y - chest_level) < 0.2)
        
        return (left_wrist_crossed or right_wrist_crossed) and wrists_at_chest
    
    # === LANDMARK PUBLISHING ===
    def publish_landmarks(self, landmarks):
        """Publish all 33 pose landmarks as flat array [x1,y1,z1,vis1, x2,y2,z2,vis2, ...]"""
        msg = Float32MultiArray()
        landmark_data = []
        
        for lm in landmarks:
            landmark_data.extend([lm.x, lm.y, lm.z, lm.visibility])
        
        msg.data = landmark_data
        self.pub_landmarks.publish(msg)
    
    # === TIMER ROUTINE ===
    def tick_timer(self):
        """Periodic updates for pose detection state."""
        # Publish pose detection status
        self.pub_pose_detected.publish(Int8(data=int(self.pose_detected)))
        
        # Publish current posture
        posture_msg = String()
        posture_msg.data = self.current_posture
        self.pub_posture.publish(posture_msg)
        
        # Publish current gesture
        gesture_msg = String()
        gesture_msg.data = self.current_gesture
        self.pub_gesture.publish(gesture_msg)
        
        # Publish distance
        distance_msg = String()
        distance_msg.data = self.current_distance
        self.pub_distance.publish(distance_msg)
        
        # Publish position
        position_msg = String()
        position_msg.data = self.current_position
        self.pub_position.publish(position_msg)
        
        # Log significant events
        # if self.current_gesture != "none":
        #     self.get_logger().info(f"👋 Gesture: {self.current_gesture} | Distance: {self.current_distance} | Position: {self.current_position}")
    
    # === FRAME PUBLISHING ===
    def publish_frame(self, frame):
        """Publish annotated video frame."""
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.pub_frame.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing frame: {e}")
    
    # === CALIBRATION CONTROLS ===
    def handle_keyboard_input(self, key):
        """Handle keyboard commands for calibration."""
        if key == ord('t'):  # Toggle calibration mode
            self.calibration_mode = not self.calibration_mode
            status = "ENABLED" if self.calibration_mode else "DISABLED"
            self.get_logger().info(f"Calibration mode {status}")
        
        elif key == ord('c') and self.calibration_mode:  # Calibrate 'close'
            if self.pose_detected and self.shoulder_width_pixels > 0:
                self.calibrate_distance('close', int(self.shoulder_width_pixels))
            else:
                self.get_logger().warn("No person detected, can't calibrate")
        
        elif key == ord('f') and self.calibration_mode:  # Calibrate 'far'
            if self.pose_detected and self.shoulder_width_pixels > 0:
                self.calibrate_distance('far', int(self.shoulder_width_pixels))
            else:
                self.get_logger().warn("No person detected, can't calibrate")
        
        elif key == ord('m') and self.calibration_mode:  # Calibrate 'medium' (optional)
            if self.pose_detected and self.shoulder_width_pixels > 0:
                self.calibrate_distance('medium', int(self.shoulder_width_pixels))
            else:
                self.get_logger().warn("No person detected, can't calibrate")
        
        elif key == ord('s'):  # Show current calibration
            self.get_logger().info(f"Current calibration: {self.calibration_data}")
        
        elif key == ord('q'):  # Quit
            return False
        
        return True
    
    # === CLEANUP ===
    def destroy_node(self):
        """Release resources on shutdown."""
        self.pose.close()
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()