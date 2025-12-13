#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String, Float32
import struct
import requests
import numpy as np
import math
from typing import List, Tuple, Optional


class LidarProcessorNode(Node):
    # Configuration
    REEMAN_BASE_URL = "http://10.7.101.165:8080/"
    LIDAR_ENDPOINT = "reeman/laser"  
    UPDATE_RATE_HZ = 10.0  # 10Hz update rate
    
    HRI_ZONE_CLOSE = 0.8    
    HRI_ZONE_MEDIUM = 1.5   
    HRI_ZONE_FAR = 3.0      
    
    # Obstacle detection zones
    OBSTACLE_FRONT_WIDTH = 0.4   # ±0.2m radius (center)
    OBSTACLE_FRONT_DEPTH = 1.5   # 1.5m ahead
    OBSTACLE_SIDE_WIDTH = 0.6    # 0.3 radius (center)
    
    def __init__(self):
        super().__init__('lidar_processor_node')
        
        # === Publishers ===
        self.pub_point_cloud = self.create_publisher(PointCloud2, '/lidar/point_cloud', 10)
        self.pub_obstacle_status = self.create_publisher(String, '/lidar/obstacle_status', 10)
        self.pub_obstacle_distance = self.create_publisher(Float32, '/lidar/obstacle_distance', 10)
        self.pub_hri_zone = self.create_publisher(String, '/lidar/hri_zone', 10)
        
        # === Subscriptions ===
        self.create_subscription(String, '/vision/person_position', self.vision_position_callback, 10)
        
        # === State ===
        self.vision_person_position = "middle"  
        self.last_obstacle_distance = float('inf')
        self.last_hri_zone = "unknown"
        
        # === Timer ===
        self.create_timer(1.0 / self.UPDATE_RATE_HZ, self.lidar_update_callback)
        
        self.get_logger().info("LidarProcessorNode started")
        self.get_logger().info(f"Fetching from: {self.REEMAN_BASE_URL}{self.LIDAR_ENDPOINT}")
    
    def vision_position_callback(self, msg):
        """Receive person position from vision system."""
        self.vision_person_position = msg.data
    
    def lidar_update_callback(self):
        """Main update loop - fetch and process lidar data."""
        try:
            # Fetch lidar data from REST API
            coords = self.fetch_lidar_data()
            if coords is None or len(coords) == 0:
                return
            
            # Publish point cloud
            self.publish_point_cloud(coords)
            
            # Analyze obstacles
            obstacle_info = self.detect_obstacles(coords)
            self.publish_obstacle_info(obstacle_info)
            
            # Analyze HRI zones (person detection area)
            hri_zone = self.analyze_hri_zone(coords)
            self.publish_hri_zone(hri_zone)
            
        except Exception as e:
            self.get_logger().error(f"Lidar update error: {e}")
    
    def fetch_lidar_data(self) -> Optional[List[Tuple[float, float]]]:
        """Fetch lidar coordinates from Reeman API."""
        try:
            url = f"{self.REEMAN_BASE_URL}{self.LIDAR_ENDPOINT}"
            response = requests.get(url, timeout=0.5)
            
            if response.status_code != 200:
                self.get_logger().warn(f"Lidar API returned {response.status_code}")
                return None
            
            data = response.json()
            if "coordinates" not in data:
                self.get_logger().warn("No 'coordinates' field in lidar response")
                return None
            
            coords = [(float(p[0]), float(p[1])) for p in data["coordinates"]]
            return coords
            
        except requests.exceptions.Timeout:
            self.get_logger().warn("Lidar API timeout")
            return None
        except Exception as e:
            self.get_logger().error(f"Failed to fetch lidar: {e}")
            return None
    
    def publish_point_cloud(self, coords: List[Tuple[float, float]]):
        """Convert coordinates to PointCloud2 and publish."""
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        # Define point fields (x, y, z)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.height = 1
        msg.width = len(coords)
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Pack points (x, y, z=0 for 2D lidar)
        buffer = []
        for x, y in coords:
            buffer.append(struct.pack('fff', x, y, 0.0))
        
        msg.data = b''.join(buffer)
        self.pub_point_cloud.publish(msg)
    
    def detect_obstacles(self, coords: List[Tuple[float, float]]) -> dict:
        """Detect obstacles in front of robot."""
        obstacle_info = {
            'status': 'clear',
            'distance': float('inf'),
            'angle': 0.0,
            'position': None
        }
        
        min_dist = float('inf')
        closest_point = None
        
        # Check front obstacle zone (rectangular area)
        for x, y in coords:
            # Front zone: x > 0 (forward), |y| < OBSTACLE_FRONT_WIDTH/2
            if x > 0 and x < self.OBSTACLE_FRONT_DEPTH:
                if abs(y) < self.OBSTACLE_FRONT_WIDTH / 2:
                    dist = math.sqrt(x**2 + y**2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_point = (x, y)
        
        if closest_point:
            obstacle_info['status'] = 'obstacle_detected'
            obstacle_info['distance'] = min_dist
            obstacle_info['angle'] = math.atan2(closest_point[1], closest_point[0])
            obstacle_info['position'] = closest_point
            
            # Classify severity
            if min_dist < 0.3:
                obstacle_info['status'] = 'critical'
            elif min_dist < 0.6:
                obstacle_info['status'] = 'warning'
            else:
                obstacle_info['status'] = 'detected'
        
        self.last_obstacle_distance = min_dist
        return obstacle_info
    
    def analyze_hri_zone(self, coords: List[Tuple[float, float]]) -> str:
        """Analyze HRI interaction zone based on lidar and vision."""
        # Find closest point in the direction of detected person (from vision)
        zone_angles = {
            'most_left': (-90, -54),   # degrees
            'left': (-54, -18),
            'middle': (-18, 18),
            'right': (18, 54),
            'most_right': (54, 90)
        }
        
        angle_min, angle_max = zone_angles.get(self.vision_person_position, (-18, 18))
        angle_min_rad = math.radians(angle_min)
        angle_max_rad = math.radians(angle_max)
        
        min_dist_in_zone = float('inf')
        
        for x, y in coords:
            angle = math.atan2(y, x)
            dist = math.sqrt(x**2 + y**2)
            
            # Check if point is in the person's zone
            if angle_min_rad <= angle <= angle_max_rad and x > 0:
                min_dist_in_zone = min(min_dist_in_zone, dist)
        
        # Classify HRI zone
        if min_dist_in_zone < self.HRI_ZONE_CLOSE:
            hri_zone = "close"
        elif min_dist_in_zone < self.HRI_ZONE_MEDIUM:
            hri_zone = "medium"
        elif min_dist_in_zone < self.HRI_ZONE_FAR:
            hri_zone = "far"
        else:
            hri_zone = "no_detection"
        
        self.last_hri_zone = hri_zone
        return hri_zone
    
    def publish_obstacle_info(self, info: dict):
        """Publish obstacle detection results."""
        # Publish status
        status_msg = String()
        status_msg.data = info['status']
        self.pub_obstacle_status.publish(status_msg)
        
        # Publish distance
        dist_msg = Float32()
        dist_msg.data = info['distance'] if info['distance'] != float('inf') else -1.0
        self.pub_obstacle_distance.publish(dist_msg)
        
        # Log warnings
        if info['status'] == 'critical':
            self.get_logger().warn(f"CRITICAL: Obstacle at {info['distance']:.2f}m!")
        elif info['status'] == 'warning':
            self.get_logger().info(f"Obstacle detected at {info['distance']:.2f}m")
    
    def publish_hri_zone(self, zone: str):
        """Publish HRI zone information."""
        msg = String()
        msg.data = zone
        self.pub_hri_zone.publish(msg)
        
        # Log zone changes
        if zone != self.last_hri_zone:
            self.get_logger().info(f"HRI Zone: {zone} (Vision: {self.vision_person_position})")


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LidarProcessorNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
