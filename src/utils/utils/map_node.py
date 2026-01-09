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

# Try to import get_map_dir handling different execution contexts
try:
    from utils.create_map import get_map_dir
except ImportError:
    try:
        from .create_map import get_map_dir
    except ImportError:
        # Fallback if executing as script
        from create_map import get_map_dir

class MapNode(Node):
    def __init__(self):
        super().__init__('map_node')
        
        self.map_dir = get_map_dir()
        self.map_path = os.path.join(self.map_dir, 'map.pgm')
        
        # Constants
        self.width = 256
        self.height = 256
        self.map_size_meters = 1.5
        self.resolution = self.map_size_meters / float(self.width) # ~0.00586 meters/pixel
        self.origin_x = 0
        self.origin_y = 0
        
        # Load map
        try:
            self.base_map = Image.open(self.map_path).convert('L')
            if self.base_map.size != (self.width, self.height):
               self.get_logger().warn(f"Map size mismatch! Loaded {self.base_map.size}, expected ({self.width}, {self.height})")
               self.base_map = self.base_map.resize((self.width, self.height))
        except FileNotFoundError:
            self.get_logger().error(f"Map file not found: {self.map_path}")
            # Initialize blank white map
            self.base_map = Image.new('L', (self.width, self.height), 255)

        self.map_pub = self.create_publisher(OccupancyGrid, '/map_defence', 10)
        self.image_pub = self.create_publisher(RosImage, '/map_debug_image', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/opponent/pose_sim', self.pose_callback, 10)
        
        self.opponent_pose = None
        # Timer 1 Hz
        self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info(f"Map Node started. Reading map from {self.map_path}")

    def pose_callback(self, msg):
        self.opponent_pose = msg

    def world_to_map(self, x, y):
        # World to Grid indices
        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)
        
        # Normalize to borders (clamping)
        ix = max(0, min(ix, self.width - 1))
        iy = max(0, min(iy, self.height - 1))
        
        # Convert to PIL coordinates (Top-Left Origin)
        # ROS Grid is Bottom-Left Origin.
        # PIL_X = ix
        # PIL_Y = Height - 1 - iy
        
        px = ix
        py = self.height - 1 - iy
        
        return px, py

    def timer_callback(self):
        # Edit map
        current_map = self.base_map.copy()
        draw = ImageDraw.Draw(current_map)
        
        if self.opponent_pose:
            px, py = self.world_to_map(self.opponent_pose.pose.position.x, self.opponent_pose.pose.position.y)
            
            # Draw Hole
            r = 20 # User said "big black hole". 20px radius ~ 12cm radius.
            # Black = 0
            px = self.width - 1 - px # Convert to PIL coords
            draw.ellipse((py-r, px-r, py+r, px+r), fill=0)
            draw.ellipse((py-r*2, px-r*2, py+r*2, px+r*2), fill=255//2) # White center
            
        # Convert to OccupancyGrid msg
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
        
        # Image to Data
        # ROS Data is int8 [], row-major, bottom-up.
        # Image is top-down (PIL default).
        # Flip image array up-down to get bottom-up order for flattening
        img_arr = np.array(current_map)
        img_arr = np.flipud(img_arr)
        
        # Map values: 0->100, 255->0, else -1
        # Create int8 array
        data = np.full(img_arr.shape, -1, dtype=np.int8)
        
        # Thresholding
        data[img_arr <= 100] = 100 # Occupied (Black-ish)
        data[img_arr >= 200] = 0   # Free (White-ish)
        
        grid_msg.data = data.flatten().tolist()
        
        self.map_pub.publish(grid_msg)

        # Publish Debug Image for RQT
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



