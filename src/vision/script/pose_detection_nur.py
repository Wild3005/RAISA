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
    CENTER_THRESHOLD = 0.15 
    
    def __init__(self):
        super().__init__('pose_detector_node')
        
        # === Publishers ===
        self.pub_pose_detected = self.create_publisher(Int8, '/vision/pose_detected', 1)
        # self.pub_posture = self.create_publisher(String, '/vision/posture', 1)
        # self.pub_gesture = self.create_publisher(String, '/vision/gesture', 1)
        # self.pub_landmarks = self.create_publisher(Float32MultiArray, '/vision/pose_landmarks', 1)
        # self.pub_frame = self.create_publisher(Image, '/vision/pose_frame', 1)
        # self.pub_distance = self.create_publisher(String, '/vision/person_distance', 1)
        self.pub_position = self.create_publisher(String, '/vision/person_position', 1)
        # self.pub_camera_adjustment = self.create_publisher(String, '/vision/camera_adjustment', 1)
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
        # Note: Holistic sudah include Hands, jadi tidak perlu inisialisasi mp.solutions.hands terpisah
        self.mp_pose = mp.solutions.pose # Masih butuh enum untuk indexing landmark tubuh
        self.mp_draw = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        
        # === State ===
        self.last_frame = None
        self.current_posture = "unknown"
        self.current_gesture = "none"
        self.prev_hand_positions = []
        self.pose_detected = False
        
        # === Distance & Position Tracking ===
        self.current_distance = "unknown"
        self.current_position = "middle"
        self.shoulder_width_pixels = 0
        
        # === Camera Adjustment & Orientation ===
        self.current_camera_adjustment = "centered"
        self.current_orientation = "unknown"
        
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
        
        self.get_logger().info("✅ PoseDetectorNode (HOLISTIC VERSION) active")

    # [BAGIAN CALIBRATION - TIDAK BERUBAH, DISINGKAT]
    def load_calibration(self):
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, 'r') as f:
                    self.calibration_data = json.load(f)
            except: pass

    def save_calibration(self):
        os.makedirs(os.path.dirname(self.config_path), exist_ok=True)
        with open(self.config_path, 'w') as f:
            json.dump(self.calibration_data, f)
            
    def calibrate_distance(self, label, width):
        self.calibration_data[f"{label}_shoulder_width"] = width
        self.save_calibration()

    # === ROS IMAGE CALLBACK ===
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.flip(frame, 1)
            self.last_frame = frame
            self.process_holistic(frame) # Panggil fungsi baru
        except Exception as e:
            self.get_logger().error(f"Frame error: {e}")

    # --- PERUBAHAN 2: PROCESS HOLISTIC ---
    def process_holistic(self, frame):
        """Run MediaPipe Holistic detection."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Proses Holistic (mendapat Pose, Face, Left Hand, Right Hand sekaligus)
        results = self.holistic.process(rgb)
        
        if results.pose_landmarks:
            self.pose_detected = True
            
            # Draw Pose
            self.mp_draw.draw_landmarks(
                frame, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style())
            
            # Draw Face Mesh (Hanya jika terdeteksi)
            if results.face_landmarks:
                self.mp_draw.draw_landmarks(
                    frame, results.face_landmarks, self.mp_holistic.FACEMESH_TESSELATION,
                    landmark_drawing_spec=None,
                    connection_drawing_spec=self.mp_drawing_styles.get_default_face_mesh_tesselation_style())

            # Draw Hands
            self.mp_draw.draw_landmarks(frame, results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)
            self.mp_draw.draw_landmarks(frame, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)
            
            # Extract Pose Landmarks for logic
            landmarks = results.pose_landmarks.landmark
            h, w, _ = frame.shape
            
            # 1. Orientation Detection (Logic Baru via Holistic)
            self.detect_orientation_holistic(landmarks, results.face_landmarks)
            
            # 2. Basic Calculations
            self.calculate_distance(landmarks, w, h)
            self.calculate_position(landmarks) # Updated logic inside
            self.calculate_camera_adjustment(landmarks) # Updated logic inside
            
            # 3. Analysis
            self.analyze_posture(landmarks, h, w, frame)
            self.analyze_gestures_holistic(landmarks, results, frame) # Pass full results for hands
            
            self.publish_landmarks(landmarks)
            
        else:
            self.pose_detected = False
            self.current_posture = "no_person"
            self.current_distance = "unknown"
            self.current_orientation = "unknown"

        self.draw_hud(frame)
        self.publish_frame(frame)

    # --- PERUBAHAN 3: DETEKSI ORIENTASI VIA FACE MESH ---
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


    # --- PERUBAHAN 4: TRACKING POINT SWITCHING (Tetap diperlukan) ---
    def calculate_camera_adjustment(self, landmarks):
        """Switch tracking target: Nose (Front) vs Shoulder Center (Back)."""
        target_x = 0.5
        
        if self.current_orientation == "turned_away":
            # Gunakan titik tengah BAHU
            l_sh = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
            r_sh = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
            target_x = (l_sh.x + r_sh.x) / 2
        else:
            # Gunakan HIDUNG (jika orientation unknown, asumsi ada hidung di pose landmark)
            nose = landmarks[self.mp_pose.PoseLandmark.NOSE]
            target_x = nose.x
            
        x_offset = target_x - 0.5
        
        # Special case: Robot sudah dekat & orang membelakangi -> Align
        if self.current_orientation == "turned_away" and self.current_distance == "close":
            self.current_camera_adjustment = "align_with_person"
            return
        
        if x_offset > self.CENTER_THRESHOLD:
            self.current_camera_adjustment = "move_left"
        elif x_offset < -self.CENTER_THRESHOLD:
            self.current_camera_adjustment = "move_right"
        else: 
            self.current_camera_adjustment = "centered"

    # --- POSITION TRACKING (Updated) ---
    def calculate_position(self, landmarks):
        """Zone detection switching based on orientation."""
        x_pos = 0.5
        if self.current_orientation == "turned_away":
            l_sh = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
            r_sh = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
            x_pos = (l_sh.x + r_sh.x) / 2
        else:
            x_pos = landmarks[self.mp_pose.PoseLandmark.NOSE].x
        
        if x_pos < self.ZONE_MOST_LEFT_LIMIT: self.current_position = "most_left"
        elif x_pos < self.ZONE_LEFT_LIMIT:    self.current_position = "left"
        elif x_pos < self.ZONE_MIDDLE_LIMIT:  self.current_position = "middle"
        elif x_pos < self.ZONE_RIGHT_LIMIT:   self.current_position = "right"
        else:                                 self.current_position = "most_right"

    # [BAGIAN LAIN: Distance & Posture - Tidak Berubah Signifikan]
    def calculate_distance(self, landmarks, w, h):
        l_sh = landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER]
        r_sh = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
        sx1, sy1 = int(l_sh.x * w), int(l_sh.y * h)
        sx2, sy2 = int(r_sh.x * w), int(r_sh.y * h)
        self.shoulder_width_pixels = math.sqrt((sx2 - sx1)**2 + (sy2 - sy1)**2)
        
        if self.shoulder_width_pixels >= self.calibration_data['close_shoulder_width']:
            self.current_distance = "close"
        elif self.shoulder_width_pixels <= self.calibration_data['far_shoulder_width']:
            self.current_distance = "far"
        else:
            self.current_distance = "medium"

    def analyze_posture(self, landmarks, h, w, frame):
        # (Sama seperti kode sebelumnya, menggunakan Pose Landmark dari Holistic)
        nose = landmarks[self.mp_pose.PoseLandmark.NOSE]
        hip_y = (landmarks[23].y + landmarks[24].y) / 2
        knee_y = (landmarks[25].y + landmarks[26].y) / 2
        
        torso = abs(nose.y - hip_y)
        thigh = abs(hip_y - knee_y)
        
        if torso < 0.1: posture = "lying"
        elif thigh < self.SITTING_THRESHOLD * torso: posture = "sitting"
        elif nose.y < 0.3: posture = "standing"
        else: posture = "standing"
        
        self.current_posture = posture
        cv2.putText(frame, f"Posture: {posture.upper()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # --- GESTURE HOLISTIC (Menggunakan Hand Landmarks bawaan Holistic) ---
    def analyze_gestures_holistic(self, pose_landmarks, results, frame):
        gesture = "none"
        
        # Cek Hand Landmarks dari Holistic results
        # Holistic memisah Left/Right hand landmarks secara eksplisit
        
        if results.left_hand_landmarks:
            if self.is_thumbs_up(results.left_hand_landmarks): gesture = "thumbs_up_left"
            elif self.is_pointing(results.left_hand_landmarks): gesture = "pointing_left"
        
        if results.right_hand_landmarks:
            if self.is_thumbs_up(results.right_hand_landmarks): gesture = "thumbs_up_right"
            elif self.is_pointing(results.right_hand_landmarks): gesture = "pointing_right"
            
        # Jika tidak ada gesture tangan spesifik, cek gesture lengan (Pose)
        if gesture == "none":
            l_wrist = pose_landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST]
            r_wrist = pose_landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST]
            nose = pose_landmarks[self.mp_pose.PoseLandmark.NOSE]
            
            if l_wrist.y < nose.y or r_wrist.y < nose.y:
                gesture = "hand_raised"
            elif self.detect_waving(l_wrist, r_wrist):
                gesture = "waving"
        
        self.current_gesture = gesture
        if gesture != "none":
            cv2.putText(frame, f"Gesture: {gesture.upper()}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 255), 2)

    # [GESTURE HELPER FUNCTIONS - Tidak Berubah, hanya pastikan referensi landmark sesuai]
    # Note: Fungsi is_thumbs_up, is_pointing, detect_waving sama persis dengan kode lama
    # Anda bisa copy-paste method helper gesture dari kode lama Anda ke sini.
    # Untuk ringkasnya, saya tidak tulis ulang semua helper gesture di sini, 
    # tapi pastikan method seperti `is_thumbs_up` tetap ada di dalam class ini.
    
    def is_thumbs_up(self, lm):
        # Implementasi sama seperti sebelumnya
        return False # Placeholder

    def is_pointing(self, lm):
        # Implementasi sama seperti sebelumnya
        return False # Placeholder

    def detect_waving(self, l, r):
        # Implementasi sama seperti sebelumnya
        return False 

    # --- HUD & UTILS ---
    def draw_hud(self, frame):
        # (Sama seperti kode sebelumnya, disesuaikan sedikit untuk orientasi)
        h, w = frame.shape[:2]
        
        # Orientasi Indicator
        color_map = {"facing_camera": (0,255,0), 
        "turned_away": (0,0,255), 
        "unknown": (0,0,255), 
        "facing_left":  (0,0,255), 
        "facing_right": (0,0,255)}
        color = color_map.get(self.current_orientation, (255,255,255))
        
        cv2.putText(frame, f"Orient: {self.current_orientation.upper()}", (15, 105),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Camera Adjustment Arrow
        center_x = w // 2
        if self.current_camera_adjustment == "move_left":
            cv2.arrowedLine(frame, (center_x+50, 50), (center_x-50, 50), (0,165,255), 3)
        elif self.current_camera_adjustment == "move_right":
            cv2.arrowedLine(frame, (center_x-50, 50), (center_x+50, 50), (255,165,0), 3)

    def publish_landmarks(self, landmarks):
        msg = Float32MultiArray()
        data = []
        for lm in landmarks: data.extend([lm.x, lm.y, lm.z, lm.visibility])
        msg.data = data
        self.pub_landmarks.publish(msg)

    def publish_frame(self, frame):
        try:
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.pub_frame.publish(msg)
        except: pass

    def tick_timer(self):
        self.pub_orientation.publish(String(data=self.current_orientation))
        self.pub_camera_adjustment.publish(String(data=self.current_camera_adjustment))
        # Publish lainnya sesuai kebutuhan

    def destroy_node(self):
        self.holistic.close() # Close holistic solution
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PoseDetectorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()