#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image as RosImage
from PIL import Image, ImageDraw
import numpy as np
import os
import sys
import yaml

# Try to import get_map_dir handling different execution contexts
try:
    from robot_navigation.create_map import get_map_dir
except ImportError:
    try:
        from .create_map import get_map_dir
    except ImportError:
        # Fallback if executing as script
        from create_map import get_map_dir

class MapNode(Node):
    def _load_map_metadata(self):
        yaml_path = os.path.join(self.map_dir, 'my_map.yaml')
        self.get_logger().info(f"Looking for map metadata at: {yaml_path}")
        if os.path.exists(yaml_path):
            try:
                with open(yaml_path, 'r') as f:
                    return yaml.safe_load(f) or {}
            except Exception as e:
                self.get_logger().error(f"Failed to read metadata yaml: {e}")
                sys.exit(1)
        else:
            self.get_logger().error(f"Map metadata file not found: {yaml_path}")
            sys.exit(1)

    def __init__(self):
        super().__init__('map_node')
        
        self.map_dir = get_map_dir()
        self.map_path = os.path.join(self.map_dir, 'map.pgm')
        
        # Load metadata from YAML (must exist and contain required fields)
        metadata = self._load_map_metadata()
        
        # Extract required parameters
        required_fields = ['map_size', 'origin', 'real_size']
        for field in required_fields:
            if field not in metadata:
                self.get_logger().error(f"Required field '{field}' missing in map metadata")
                sys.exit(1)
        
        self.width = metadata['map_size']
        self.height = self.width   # assuming square map
        self.resolution = metadata['real_size'] / metadata['map_size']
        
        origin = metadata['origin']
        if not isinstance(origin, (list, tuple)) or len(origin) < 2:
            self.get_logger().error("Origin must be a list/tuple with at least 2 elements")
            sys.exit(1)
        self.origin_x = float(origin[0])
        self.origin_y = float(origin[1])
        
        # Map size in meters (can be computed, but also available as real_size if needed)
        self.map_size_meters = self.width * self.resolution
        
        # Load map image
        try:
            self.base_map = Image.open(self.map_path).convert('L')
            if self.base_map.size != (self.width, self.height):
                self.get_logger().warn(
                    f"Map size mismatch! Loaded {self.base_map.size}, expected ({self.width}, {self.height}). Resizing."
                )
                self.base_map = self.base_map.resize((self.width, self.height))
        except FileNotFoundError:
            self.get_logger().error(f"Map file not found: {self.map_path}")
            sys.exit(1)
        
        self.map_pub = self.create_publisher(OccupancyGrid, '/map_defence', 10)
        self.image_pub = self.create_publisher(RosImage, '/map_debug_image', 10)
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/arena_perception_opponent_base_footprint_pose',
            self.pose_callback,
            10
        )
        
        self.opponent_pose = None
        self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info(
            f"Map Node started. Map: {self.map_path} | Size: {self.width}x{self.height} | "
            f"Resolution: {self.resolution} | Origin: ({self.origin_x}, {self.origin_y})"
        )

    def pose_callback(self, msg):
        self.opponent_pose = msg

    def world_to_map(self, x, y):
        # World to grid indices (bottom-left origin)
        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)
        
        # Clamp to map bounds
        ix = max(0, min(ix, self.width - 1))
        iy = max(0, min(iy, self.height - 1))
        
        # Convert to PIL coordinates (top-left origin)
        px = ix
        py = self.height - 1 - iy
        return px, py

    def timer_callback(self):
        # Copy base map and draw opponent position
        current_map = self.base_map.copy()
        draw = ImageDraw.Draw(current_map)
        
        if self.opponent_pose:
            px, py = self.world_to_map(
                self.opponent_pose.pose.position.x,
                self.opponent_pose.pose.position.y
            )
            # Draw a hole (black) with a white center
            r = 5
            draw.ellipse((px - r, py - r, px + r, py + r), fill=0)
            draw.ellipse((px - r*2, py - r*2, px + r*2, py + r*2), fill=255 // 2)
        
        # Convert to OccupancyGrid
        grid_msg = OccupancyGrid()
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        grid_msg.info.resolution = float(self.resolution)
        grid_msg.info.width = self.width
        grid_msg.info.height = self.height
        grid_msg.info.origin.position.x = float(self.origin_x)
        grid_msg.info.origin.position.y = float(self.origin_y)
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Convert PIL image (top-left) to numpy array and flip vertically (bottom-left for ROS)
        img_arr = np.array(current_map)
        img_arr = np.flipud(img_arr)
        
        # Threshold: 0-100 -> occupied (100), 200-255 -> free (0), else unknown (-1)
        data = np.full(img_arr.shape, -1, dtype=np.int8)
        data[img_arr <= 100] = 100
        data[img_arr >= 200] = 0
        grid_msg.data = data.flatten().tolist()
        
        self.map_pub.publish(grid_msg)
        
        # Publish debug image
        ros_image = RosImage()
        ros_image.header = grid_msg.header
        ros_image.height = self.height
        ros_image.width = self.width
        ros_image.encoding = "mono8"
        ros_image.is_bigendian = 0
        ros_image.step = self.width
        ros_image.data = np.array(current_map).tobytes()
        self.image_pub.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    mapper = MapNode()
    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        pass
    finally:
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()