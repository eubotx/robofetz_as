#!/usr/bin/env python3
import os
import sys
import numpy as np
import cv2
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Added OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid 
from cv_bridge import CvBridge

class MapNode(Node):
    """
    A ROS 2 node that maintains a dynamic occupancy grid map based on enemy position.
    
    It loads a static base map, updates it based on the enemy's position,
    and publishes the result as an image for visualization AND an OccupancyGrid for Nav2.
    """

    def __init__(self):
        super().__init__('map_node')

        # --- Parameters ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_path', 'src/utils/map/map_robofetz.npy'),
                ('enemy_pose_topic', '/camera/enemy/pose'),
                ('map_image_topic', '/map/gradient_map_image'),
                ('map_occupancy_topic', '/map'), # Standard Nav2 Topic
                ('arena_size_meters', 1.5),
                ('map_resolution_pixels', 256),
                ('darkening_radius_meters', 0.15),
                ('darkening_factor', 0.4), 
                ('publish_rate_hz', 10.0)
            ]
        )

        # --- Configuration ---
        self.arena_size = self.get_parameter('arena_size_meters').value
        self.map_res = self.get_parameter('map_resolution_pixels').value
        self.darkening_radius_m = self.get_parameter('darkening_radius_meters').value
        self.darkening_factor = self.get_parameter('darkening_factor').value
        
        # Calculate pixel radius once
        self.pixels_per_meter = self.map_res / self.arena_size
        self.darkening_radius_px = int(self.darkening_radius_m * self.pixels_per_meter)
        
        # Calculate physical resolution (meters per pixel) for Nav2
        self.resolution_m_per_px = self.arena_size / self.map_res

        # --- State ---
        self.bridge = CvBridge()
        self.base_map: Optional[np.ndarray] = None
        self.current_map: Optional[np.ndarray] = None
        
        # --- Initialization ---
        if not self._load_map():
            sys.exit(1)

        # --- Communication ---
        
        # QoS for sensor data (fast, volatile)
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for MAPS (Nav2 requires Transient Local to receive the map late)
        qos_map = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter('enemy_pose_topic').value,
            self.pose_callback,
            qos_sensor
        )

        # Publisher for Visualization (RVIZ Image)
        self.image_pub = self.create_publisher(
            Image,
            self.get_parameter('map_image_topic').value,
            10
        )
        
        # Publisher for Navigation (Nav2 Costmap)
        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            self.get_parameter('map_occupancy_topic').value,
            qos_map
        )

        # Timer for publishing the map
        timer_period = 1.0 / self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(timer_period, self.publish_map)

        self.get_logger().info(f'Map Node initialized. Arena: {self.arena_size}m, Res: {self.map_res}px')

    def _load_map(self) -> bool:
        """Loads the initial numpy map from disk."""
        relative_path = self.get_parameter('map_path').value
        workspace_root = os.getcwd()
        full_path = os.path.join(workspace_root, relative_path)

        if not os.path.exists(full_path):
            self.get_logger().error(f"Map file not found at: {full_path}")
            return False

        try:
            self.base_map = np.load(full_path)
            
            if self.base_map.shape != (self.map_res, self.map_res):
                self.get_logger().warn(
                    f"Map dimensions {self.base_map.shape} do not match config. Resizing."
                )
                self.base_map = cv2.resize(self.base_map, (self.map_res, self.map_res))

            self.base_map = self.base_map.astype(np.float32)
            self.current_map = self.base_map.copy()
            self.get_logger().info("Map loaded successfully.")
            return True

        except Exception as e:
            self.get_logger().error(f"Failed to load map: {e}")
            return False

    def _world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        """Converts world coordinates (meters) to pixel coordinates."""
        x = max(0.0, min(x, self.arena_size))
        y = max(0.0, min(y, self.arena_size))

        # Note: This logic implies (0,0) world is at (Max, Max) pixel or similar inversion
        px = int((self.arena_size - x) * self.pixels_per_meter)
        py = int((self.arena_size - y) * self.pixels_per_meter)
        
        return px, py

    def pose_callback(self, msg: PoseStamped):
        if self.current_map is None:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y

        px, py = self._world_to_pixel(x, y)
        self._apply_darkening(px, py)

    def _apply_darkening(self, center_x: int, center_y: int):
        y_grid, x_grid = np.ogrid[:self.map_res, :self.map_res]
        dist_sq = (x_grid - center_x)**2 + (y_grid - center_y)**2
        radius_sq = self.darkening_radius_px**2
        
        mask = dist_sq <= radius_sq
        
        self.current_map = self.base_map.copy() 
        temp_darkening_factor = np.sqrt(dist_sq) / self.darkening_radius_px
        temp_darkening_factor = np.clip(temp_darkening_factor, self.darkening_factor, 1.0)

        self.current_map[mask] = self.base_map.copy()[mask] * temp_darkening_factor[mask] 
        np.clip(self.current_map, 0.0, 1.0, out=self.current_map)

    def publish_map(self):
        """Publishes the current map state as both Image and OccupancyGrid."""
        if self.current_map is None:
            return

        current_time = self.get_clock().now().to_msg()
        
        # --- 1. Publish Image (Visual) ---
        try:
            # Swap axes for Image format if necessary (keeping your original logic)
            display_map = (np.swapaxes(self.current_map, 0, 1) * 255).astype(np.uint8)

            img_msg = self.bridge.cv2_to_imgmsg(display_map, encoding="mono8")
            img_msg.header.stamp = current_time
            img_msg.header.frame_id = "map"
            self.image_pub.publish(img_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing map image: {e}")

        # --- 2. Publish OccupancyGrid (Nav2) ---
        try:
            grid_msg = OccupancyGrid()
            grid_msg.header.stamp = current_time
            grid_msg.header.frame_id = "map"

            # Metadata
            grid_msg.info.resolution = self.resolution_m_per_px
            grid_msg.info.width = self.map_res
            grid_msg.info.height = self.map_res
            
            # Origin: (0,0,0) - We align the grid to standard ROS coordinates
            grid_msg.info.origin.position.x = 0.0
            grid_msg.info.origin.position.y = 0.0
            grid_msg.info.origin.position.z = 0.0
            grid_msg.info.origin.orientation.w = 1.0

            # Data Conversion Logic:
            # 1. Start with raw map (0.0 to 1.0)
            # 2. Swap axes to match the visual output logic (self.current_map is [y,x]?)
            #    Based on `swapaxes` in Image, let's assume `current_map` needs swapping/flipping to be row-major.
            #    However, due to _world_to_pixel having `arena_size - x`, the data in memory is inverted.
            #    ROS OccupancyGrid expects data[0] to be at (0,0) World.
            #    Your memory data[0] is at (Max, Max) World.
            #    So we must flip both axes to align with ROS (0,0) origin.
            
            # Flip logic to correct for (arena_size - x)
            nav_map = np.flip(self.current_map, axis=(0, 1))
            
            # Also need to swap axes if the original array was (col, row) instead of (row, col)
            nav_map = np.swapaxes(nav_map, 0, 1)

            # Value Mapping:
            # Image: 255 (White) = Free, 0 (Black) = Occupied
            # Nav2: 0 = Free, 100 = Occupied
            # Formula: 100 - (Value * 100)
            # Assuming current_map is 1.0 (White/Safe) and 0.0 (Black/Obstacle)
            
            # Create int8 array initialized to 0
            occ_data = np.zeros_like(nav_map, dtype=np.int8)
            
            # Scale 0.0-1.0 to 0-100 integers
            # If map value is 1.0 (Free) -> Result 0
            # If map value is 0.0 (Blocked) -> Result 100
            occ_data = (100 - (nav_map * 100)).astype(np.int8)
            
            # Flatten to list
            grid_msg.data = occ_data.flatten().tolist()

            self.grid_pub.publish(grid_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing occupancy grid: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = MapNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()