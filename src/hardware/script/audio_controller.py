#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8, Int8
import subprocess
import threading
from collections import deque

# Audio Files
AUDIO_GREETING = "/home/raisa/assets/sound/sound-greeting-v1.mp3"
AUDIO_ABOUT = "/home/raisa/assets/sound/sound-desc-v2.mp3"
AUDIO_ACTION_READY = "/home/raisa/assets/sound/sound-action.mp3"

class AudioController(Node):

    def __init__(self):
        super().__init__('audio_controller_node')

        # Subscribers
        self.create_subscription(Int8, '/vision/play_greeting', self.cb_face, 1)
        self.create_subscription(Int8, '/ui/about_me', self.cb_button, 1)
        self.create_subscription(Int8, '/vision/hand_action_ready', self.cb_hand, 1)

        # Audio queue and state
        self.queue = deque()  # (file_path, priority)
        self.is_playing = False
        self.current_priority = 0

        self.greeting_cooldown = 0
        self.about_cooldown = 0
        self.cooldown_time = 3.0  # detik
        self.about_cooldown_time = 3.0  # detik

        self.create_timer(1.0, self.tick)

        self.get_logger().info("Audio Controller Online with Queue Support")

    def tick(self):
        if self.greeting_cooldown > 0:
            self.greeting_cooldown -= 1
        if self.about_cooldown > 0:
            self.about_cooldown -= 1

    def play_next(self):
        if self.is_playing or not self.queue:
            return

        file_path, priority = self.queue[0]
        self.current_priority = priority
        self.is_playing = True

        self.get_logger().info(f"PLAY: {file_path} (p={priority})")

        def run():
            subprocess.call(["mpg123", "-q", file_path])
            self.is_playing = False
            self.current_priority = 0
            self.queue.popleft()
            self.play_next()  # continue next

        threading.Thread(target=run, daemon=True).start()

    def enqueue_audio(self, file_path, priority):
        # Jika audio yang sedang dimainkan sama -> abaikan
        if self.is_playing and self.queue and self.queue[0][0] == file_path:
            self.get_logger().info(f"Ignored duplicate request for: {file_path}")
            return

        self.get_logger().info(f"REQ: {file_path} (p={priority})")

        # Hapus dari queue semua audio dengan file yang sama (biar tidak dobel)
        self.queue = deque([(fp, pr) for fp, pr in self.queue if fp != file_path])

        # Jika ada audio yang sedang berjalan dengan prioritas lebih rendah, hentikan
        if self.is_playing and priority > self.current_priority:
            subprocess.call(["pkill", "mpg123"])
            self.is_playing = False
            self.current_priority = 0
            self.queue.clear()

        # Tambahkan ke queue
        self.queue.append((file_path, priority))
        self.play_next()


    # Event callbacks
    def cb_face(self, msg):
        if msg.data == 1 and self.greeting_cooldown <= 0:
            self.enqueue_audio(AUDIO_GREETING, priority=1)
            self.greeting_cooldown = self.cooldown_time

    def cb_button(self, msg):
        # logging
        self.get_logger().info(f"About Me button pressed (data={msg.data})")

        if msg.data == 1 and self.about_cooldown <= 0:
            self.enqueue_audio(AUDIO_ABOUT, priority=2)
            self.about_cooldown = self.about_cooldown_time

        
        elif msg.data == 0: #
            # tombol dilepas → hentikan semua audio
            if self.is_playing:
                self.get_logger().info("Button released -> stopping all audio playback")
                subprocess.call(["pkill", "mpg123"])
                self.is_playing = False
                self.current_priority = 0
            # Bersihkan semua antrian
            self.queue.clear()


    def cb_hand(self, msg):
        if msg.data == 1:
            self.enqueue_audio(AUDIO_ACTION_READY, priority=0)


def main(args=None):
    rclpy.init(args=args)
    node = AudioController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
