#!/usr/bin/env python3
# filepath: /home/ichbinwil/kode_kuliah/RAISA/src/hardware/script/dual_leg.py
"""
Dual UDP Receiver for AS5600 Sensor Data from ESP32
Receives angle data from LEFT LEG and RIGHT LEG via UDP
"""

import socket
import sys
import time
import struct
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# UDP settings - LEFT LEG
UDP_IP_LEFT = "0.0.0.0"
UDP_PORT_LEFT = 12345

# UDP settings - RIGHT LEG  
UDP_IP_RIGHT = "0.0.0.0"
UDP_PORT_RIGHT = 12346

# Global variables for command sending
left_leg_address = None
right_leg_address = None
left_command_socket = None
right_command_socket = None

# Statistics
left_packet_count = 0
right_packet_count = 0
start_time = time.time()

# Shared data for display
left_angle = 0.0
right_angle = 0.0
left_connected = False
right_connected = False
last_update = time.time()

def receive_left_leg():
    """Thread to receive data from LEFT LEG"""
    global left_leg_address, left_packet_count, left_angle, left_connected
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP_LEFT, UDP_PORT_LEFT))
    
    print(f"[LEFT LEG] Listening on port {UDP_PORT_LEFT}")
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            
            # Store address for sending commands back
            if left_leg_address != addr:
                left_leg_address = addr
                left_connected = True
                print(f"\n✓ [LEFT LEG] Connected from {addr[0]}:{addr[1]}\n")
            
            left_packet_count += 1
            
            # Decode binary data (4 bytes float)
            if len(data) == 4:
                left_angle = struct.unpack('f', data)[0]
                
        except Exception as e:
            print(f"\n[LEFT LEG] Error: {e}")
            break

def receive_right_leg():
    """Thread to receive data from RIGHT LEG"""
    global right_leg_address, right_packet_count, right_angle, right_connected
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP_RIGHT, UDP_PORT_RIGHT))
    
    print(f"[RIGHT LEG] Listening on port {UDP_PORT_RIGHT}")
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            
            # Store address for sending commands back
            if right_leg_address != addr:
                right_leg_address = addr
                right_connected = True
                print(f"\n✓ [RIGHT LEG] Connected from {addr[0]}:{addr[1]}\n")
            
            right_packet_count += 1
            
            # Decode binary data (4 bytes float)
            if len(data) == 4:
                right_angle = struct.unpack('f', data)[0]
                
        except Exception as e:
            print(f"\n[RIGHT LEG] Error: {e}")
            break

def display_thread():
    """Thread to display both angles side by side"""
    global left_angle, right_angle, left_packet_count, right_packet_count, last_update
    
    while True:
        try:
            elapsed = time.time() - start_time
            left_pps = left_packet_count / elapsed if elapsed > 0 else 0
            right_pps = right_packet_count / elapsed if elapsed > 0 else 0
            
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            # Status indicators
            left_status = "●" if left_connected else "○"
            right_status = "●" if right_connected else "○"
            
            # Display both legs side by side with status
            left_str = f"{left_status} LEFT: {left_angle:7.2f}° ({left_pps:5.1f} pkt/s)" if left_connected else f"{left_status} LEFT: WAITING..."
            right_str = f"{right_status} RIGHT: {right_angle:7.2f}° ({right_pps:5.1f} pkt/s)" if right_connected else f"{right_status} RIGHT: WAITING..."
            
            print(f"[{timestamp}] {left_str} | {right_str}    ", end='\r')
            
            time.sleep(0.05)  # Update display 20 times per second
            
        except Exception as e:
            print(f"\n[DISPLAY] Error: {e}")
            break

class publisher(Node):
    def __init__(self):
        super().__init__("dual_leg_publisher")
        self.pub_left = self.create_publisher(Float64, "dual_leg_l", 10)
        self.pub_right = self.create_publisher(Float64, "dual_leg_r", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.callback)

    def callback(self):
        msg_l = Float64()
        msg_r = Float64()
        msg_l.data = left_angle
        msg_r.data = right_angle
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)
        self.get_logger().info(f"Publishing: left: {msg_l.data} | right: {msg_r.data}")

def main(args=None):
    global left_command_socket, right_command_socket
    
    # Create UDP sockets for sending commands
    left_command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    right_command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    print("=" * 70)
    print("Dual UDP Receiver for AS5600 Sensors - LEFT & RIGHT LEG")
    print("=" * 70)
    print("Waiting for data from both ESP32 devices...")
    
    # Start receiver threads
    left_thread = threading.Thread(target=receive_left_leg, daemon=True)
    right_thread = threading.Thread(target=receive_right_leg, daemon=True)
    display = threading.Thread(target=display_thread, daemon=True)
    
    left_thread.start()
    right_thread.start()
    display.start()

    rclpy.init(args=args)
    dual_leg_pub = publisher()
    rclpy.spin(dual_leg_pub)
    
    try:
        # Keep main thread alive
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n\nStopping UDP receivers...")
        elapsed = time.time() - start_time
        print(f"\n" + "=" * 70)
        print("Statistics:")
        print(f"  LEFT LEG  - Total: {left_packet_count:,} packets | Avg: {left_packet_count/elapsed:.1f} pkt/s")
        print(f"  RIGHT LEG - Total: {right_packet_count:,} packets | Avg: {right_packet_count/elapsed:.1f} pkt/s")
        print(f"  Runtime: {elapsed:.1f} seconds")
        print("=" * 70)
        
    finally:
        if left_command_socket:
            left_command_socket.close()
        if right_command_socket:
            right_command_socket.close()
        print("Sockets closed")
    
    dual_leg_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
