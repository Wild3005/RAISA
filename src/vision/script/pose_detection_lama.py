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
        self.pub_posture = self.create_publisher(String, '/vision/posture', 1)
        self.pub_gesture = self.create_publisher(String, '/vision/gesture', 1)
        self.pub_landmarks = self.create_publisher(Float32MultiArray, '/vision/pose_landmarks', 1)
        self.pub_frame = self.create_publisher(Image, '/vision/pose_frame', 1)
        self.pub_distance = self. create_publisher(String, '/vision/person_distance', 1)
        self.pub_position = self.create_publisher(String, '/vision/person_position', 1)
        self.pub_camera_adjustment = self.create_publisher(String, '/vision/camera_adjustment', 1)
        self.pub_orientation = self.create_publisher(String, '/vision/person_orientation', 1)
        
        # === Subscriptions ===
        self.create_subscription(Image, '/vision/image_raw', self.image_callback, 10)
        
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

            # Draw hands if detected
            self.mp_draw.draw_landmarks(frame, results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)
            self.mp_draw.draw_landmarks(frame, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)
            
            # Extract landmarks for analysis
            landmarks = results.pose_landmarks.landmark
            h, w, _ = frame.shape

            # Orientation Detection
            self.detect_orientation_holistic(landmarks, results.face_landmarks)
            
            # Calculate distance and position
            self.calculate_distance(landmarks, w, h)
            self.calculate_position(landmarks)
            self.calculate_camera_adjustment(landmarks) # Updated logic inside
            
            # Analyze posture and gestures
            self.analyze_posture(landmarks, h, w, frame)
            self.analyze_gestures_holistic(landmarks, results, frame) # Pass full results for hands
            
            # Publish landmark data
            self.publish_landmarks(landmarks)
            
        else:
            self.pose_detected = False
            self.current_posture = "no_person"
            self.current_gesture = "none"
            self.current_distance = "unknown"
            self.current_position = "middle"
            self.current_camera_adjustment = "centered"
            self.current_orientation = "unknown"
        
        # Draw visual HUD
        self.draw_hud(frame)
        
        # Publish annotated frame
        self.publish_frame(frame)
    
    # === ORIENTATION DETECTION (IMPROVED) ===
    def detect_orientation(self, landmarks):
        """Detect if person is facing camera, turned away, or side view."""
        # Get key landmarks
        nose = landmarks[self.mp_pose.PoseLandmark.NOSE]
        left_eye = landmarks[self.mp_pose.PoseLandmark.LEFT_EYE]
        right_eye = landmarks[self.mp_pose.PoseLandmark.RIGHT_EYE]
        left_ear = landmarks[self.mp_pose.PoseLandmark.LEFT_EAR]
        right_ear = landmarks[self.mp_pose.PoseLandmark.RIGHT_EAR]
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = landmarks[self. mp_pose.PoseLandmark.RIGHT_SHOULDER]
        left_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
        right_hip = landmarks[self. mp_pose.PoseLandmark.RIGHT_HIP]
        
        # === METHOD 1: Eye and nose visibility ===
        left_eye_visible = left_eye. visibility > 0.5
        right_eye_visible = right_eye.visibility > 0.5
        nose_visible = nose.visibility > 0.5
        both_eyes_visible = left_eye_visible and right_eye_visible
        
        # === METHOD 2: Shoulder width vs hip width (back view shows wider shoulders) ===
        shoulder_width = abs(left_shoulder.x - right_shoulder.x)
        hip_width = abs(left_hip.x - right_hip.x)
        shoulder_hip_ratio = shoulder_width / hip_width if hip_width > 0.01 else 1.0
        
        # === METHOD 3: Z-depth analysis (which is closer to camera) ===
        # Back view: shoulders are behind hips
        # Front view:  hips are behind shoulders
        shoulder_z_avg = (left_shoulder.z + right_shoulder.z) / 2
        hip_z_avg = (left_hip.z + right_hip.z) / 2
        z_diff = shoulder_z_avg - hip_z_avg
        
        # === METHOD 4: Ear visibility (side view detection) ===
        left_ear_visible = left_ear.visibility > 0.5
        right_ear_visible = right_ear.visibility > 0.5
        one_ear_visible = left_ear_visible != right_ear_visible  # XOR - only one ear visible
        
        # === DECISION LOGIC ===
        # Front view: both eyes visible, nose visible, shoulders narrower than hips
        if both_eyes_visible and nose_visible and shoulder_hip_ratio < 1.3:
            self.current_orientation = "facing_camera"
        
        # Back view: eyes not visible, shoulders wider than hips, or shoulders behind hips
        elif not both_eyes_visible and (shoulder_hip_ratio > 1.3 or z_diff < -0.05):
            self.current_orientation = "turned_away"
        
        # Side view: only one ear visible, moderate shoulder/hip ratio
        elif one_ear_visible and 0.8 < shoulder_hip_ratio < 1.5:
            self.current_orientation = "side_view"
        
        # Ambiguous: use visibility as fallback
        elif nose_visible: 
            self.current_orientation = "facing_camera"
        else:
            self.current_orientation = "turned_away"
    
    def detect_orientation_holistic(self, pose_landmarks, face_landmarks):
        """
        Menggunakan keberadaan Face Mesh dari Holistic untuk menentukan arah.
        """
        ORIENTATION_THRESHOLD = 0.002

        # Ambil landmark bahu dari Pose
        left_shoulder = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        
        # Hitung rata-rata visibility bahu
        shoulder_vis = (left_shoulder.visibility + right_shoulder.visibility) / 2
        
        # LOGIKA UTAMA:
        # Jika Bahu terlihat Jelas, TAPI Face Mesh bernilai None -> PASTI HADAP BELAKANG
        if shoulder_vis > 0.5 and face_landmarks is None:
            self.current_orientation = "turned_away"
            return
        
        # Jika Face Mesh Ada -> Hadap Depan (atau samping)
        elif face_landmarks is not None:
            self.current_orientation = "facing_camera"

        # --- 2. Cek Front / Left / Right (Logika Baru) ---
        if face_landmarks is not None:
            # Ambil koordinat Pose Landmark
            nose = pose_landmarks[self.mp_pose.PoseLandmark.NOSE]
            left_eye = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_EYE_OUTER]
            right_eye = pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_EYE_OUTER]
            
            # Hitung posisi X
            nose_x = nose.x
            # Hitung rata-rata posisi X kedua mata
            avg_eye_x = (left_eye.x + right_eye.x) / 2
            
            # Hitung selisih posisi
            diff = avg_eye_x - nose_x
            
            # --- LOGIKA PEMBANDING ---
            # Jika Mata lebih ke Kanan (X lebih besar) dari Hidung -> Menghadap Kiri
            if diff > ORIENTATION_THRESHOLD:
                self.current_orientation = "facing_left"
                
            # Jika Mata lebih ke Kiri (X lebih kecil) dari Hidung -> Menghadap Kanan
            elif diff < -ORIENTATION_THRESHOLD:
                self.current_orientation = "facing_right"
                
            # Jika selisih sangat kecil -> Menghadap Depan
            else:
                self.current_orientation = "facing_front"
                
        else:
            self.current_orientation = "unknown"


    # === CAMERA ADJUSTMENT LOGIC (UPDATED) ===
    def calculate_camera_adjustment(self, landmarks):
        """Calculate which direction camera should move to center the person."""
        nose = landmarks[self.mp_pose.PoseLandmark. NOSE]
        
        # Calculate offset from center (0. 5 = center of frame)
        x_offset = nose.x - 0.5
        
        # === SPECIAL CASE: Person is turned away AND robot is close ===
        # This means robot has arrived beside the person
        # Robot should turn 180° to align with person's facing direction
        if self.current_orientation == "turned_away" and self.current_distance == "close":
            self.current_camera_adjustment = "align_with_person"
            return
        
        # === NORMAL CASE: Center the person in frame ===
        if x_offset > self.CENTER_THRESHOLD:  # Person too far right
            self.current_camera_adjustment = "move_left"
        elif x_offset < -self.CENTER_THRESHOLD:   # Person too far left
            self.current_camera_adjustment = "move_right"
        else: 
            self.current_camera_adjustment = "centered"  # Person is centered
    
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
                "side_view":  (255, 165, 0),
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
        """Classify body posture:  standing, sitting, lying, crouching."""
        # Get key landmarks
        nose = landmarks[self. mp_pose.PoseLandmark.NOSE]
        left_hip = landmarks[self.mp_pose.PoseLandmark.LEFT_HIP]
        right_hip = landmarks[self.mp_pose.PoseLandmark. RIGHT_HIP]
        left_knee = landmarks[self.mp_pose.PoseLandmark.LEFT_KNEE]
        right_knee = landmarks[self.mp_pose.PoseLandmark. RIGHT_KNEE]
        left_ankle = landmarks[self.mp_pose.PoseLandmark.LEFT_ANKLE]
        right_ankle = landmarks[self. mp_pose.PoseLandmark.RIGHT_ANKLE]
        
        # Calculate average hip and knee positions
        hip_y = (left_hip.y + right_hip.y) / 2
        knee_y = (left_knee.y + right_knee.y) / 2
        ankle_y = (left_ankle. y + right_ankle.y) / 2
        
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
        """Detect gestures:  waving, hands raised, pointing, thumbs up, peace, OK sign, arms crossed."""
        left_wrist = landmarks[self.mp_pose.PoseLandmark. LEFT_WRIST]
        right_wrist = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST]
        left_shoulder = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        right_shoulder = landmarks[self.mp_pose.PoseLandmark. RIGHT_SHOULDER]
        left_elbow = landmarks[self. mp_pose.PoseLandmark.LEFT_ELBOW]
        right_elbow = landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW]
        nose = landmarks[self.mp_pose.PoseLandmark. NOSE]
        
        gesture = "none"
        
        # Check for hand-based gestures first (higher priority)
        if hand_results. multi_hand_landmarks:
            hand_gesture = self.detect_hand_gestures(hand_results, frame)
            if hand_gesture != "none":
                gesture = hand_gesture
                self.current_gesture = gesture
                cv2.putText(frame, f"Gesture: {gesture.upper()}", (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)
                return
        
        # Check if either hand is raised above shoulder
        left_hand_raised = left_wrist. y < left_shoulder.y - self.HAND_RAISE_THRESHOLD
        right_hand_raised = right_wrist. y < right_shoulder.y - self.HAND_RAISE_THRESHOLD
        
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
                return f"thumbs_up_{hand_label. lower()}"
            
            # Check for pointing
            if self.is_pointing(hand_landmarks):
                return f"pointing_{hand_label. lower()}"
            
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
        pinky_tip = hand_landmarks. landmark[self.mp_hands. HandLandmark.PINKY_TIP]
        wrist = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
        
        # Thumb extended upward
        thumb_up = thumb_tip.y < thumb_ip.y < wrist.y
        
        # Other fingers curled
        fingers_down = (index_tip.y > wrist.y and middle_tip.y > wrist. y and 
                        ring_tip.y > wrist.y and pinky_tip. y > wrist.y)
        
        return thumb_up and fingers_down
    
    def is_pointing(self, hand_landmarks):
        """Detect pointing gesture (index finger extended, others curled)."""
        index_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        index_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_PIP]
        middle_tip = hand_landmarks.landmark[self. mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
        middle_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
        ring_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_TIP]
        ring_pip = hand_landmarks.landmark[self.mp_hands.HandLandmark.RING_FINGER_PIP]
        pinky_tip = hand_landmarks. landmark[self.mp_hands. HandLandmark.PINKY_TIP]
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
        others_curled = ring_tip.y > ring_pip.y and pinky_tip. y > pinky_pip.y
        
        return index_extended and middle_extended and others_curled
    
    def is_ok_sign(self, hand_landmarks):
        """Detect OK sign (thumb and index forming circle)."""
        thumb_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark. THUMB_TIP]
        index_tip = hand_landmarks. landmark[self.mp_hands. HandLandmark.INDEX_FINGER_TIP]
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
        self.prev_hand_positions. append(current_pos)
        
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
        right_wrist_crossed = right_wrist. x < left_shoulder.x
        
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
        posture_msg.data = self. current_posture
        self. pub_posture.publish(posture_msg)
        
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
        
        # Publish camera adjustment
        camera_adjustment_msg = String()
        camera_adjustment_msg.data = self.current_camera_adjustment
        self. pub_camera_adjustment.publish(camera_adjustment_msg)
        
        # Publish orientation
        orientation_msg = String()
        orientation_msg.data = self.current_orientation
        self.pub_orientation.publish(orientation_msg)
        
        # Log significant events (uncomment for debugging)
        # if self.current_camera_adjustment == "align_with_person":
        #     self.get_logger().info(f"🔄 ALIGN MODE: Person turned away, distance={self.current_distance}")
    
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
                self. calibrate_distance('close', int(self.shoulder_width_pixels))
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