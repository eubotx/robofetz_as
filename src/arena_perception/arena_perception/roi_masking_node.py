#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import threading


class ROIMaskingNode(Node):
    """
    Node that applies a Region of Interest (ROI) mask to rectified images.
    The ROI is defined in world coordinates as a rectangular area.
    Parameters are loaded from the pipeline config file.
    """
    
    def __init__(self):
        super().__init__('roi_masking_node')
        
        # =================== PARAMETERS FROM CONFIG ===================
        # Use the parameter namespace from the pipeline config
        self.declare_parameters(
            namespace='',
            parameters=[
                # Global settings
                ('opponent_det_pipeline.global.camera_frame', 'arena_camera_optical'),
                ('opponent_det_pipeline.global.target_frame', 'world'),
                
                # ROI masking specific parameters
                ('opponent_det_pipeline.roi_masking.enabled', True),
                ('opponent_det_pipeline.roi_masking.input_topic', '/arena_camera/image_rect'),
                ('opponent_det_pipeline.roi_masking.output_topic', '/arena_camera/image_rect_masked'),
                ('opponent_det_pipeline.roi_masking.debug_topic_mask', '/debug/roi_mask'),
                ('opponent_det_pipeline.roi_masking.debug_topic_overlay', '/debug/roi_overlay'),
                
                # ROI definition
                ('opponent_det_pipeline.roi_masking.roi.min_x', 0.0),
                ('opponent_det_pipeline.roi_masking.roi.max_x', 1.5),
                ('opponent_det_pipeline.roi_masking.roi.min_y', 0.0),
                ('opponent_det_pipeline.roi_masking.roi.max_y', 1.5),
                ('opponent_det_pipeline.roi_masking.roi.min_z', 0.0),
                ('opponent_det_pipeline.roi_masking.roi.max_z', 0.0),
                ('opponent_det_pipeline.roi_masking.roi.border_meters', 0.1),
                
                # Other ROI parameters
                ('opponent_det_pipeline.roi_masking.mask_color', 0),
                ('opponent_det_pipeline.roi_masking.debug', True),
            ]
        )
        
        # Check if ROI masking is enabled
        self.enabled = self.get_parameter('opponent_det_pipeline.roi_masking.enabled').value
        if not self.enabled:
            self.get_logger().info("ROI masking is disabled in config, node will forward images without masking")
        
        # Get global frames
        self.camera_frame = self.get_parameter('opponent_det_pipeline.global.camera_frame').value
        self.world_frame = self.get_parameter('opponent_det_pipeline.global.target_frame').value
        
        # Get ROI parameters with border
        self.border = self.get_parameter('opponent_det_pipeline.roi_masking.roi.border_meters').value
        self.roi_min_x = self.get_parameter('opponent_det_pipeline.roi_masking.roi.min_x').value - self.border
        self.roi_max_x = self.get_parameter('opponent_det_pipeline.roi_masking.roi.max_x').value + self.border
        self.roi_min_y = self.get_parameter('opponent_det_pipeline.roi_masking.roi.min_y').value - self.border
        self.roi_max_y = self.get_parameter('opponent_det_pipeline.roi_masking.roi.max_y').value + self.border
        self.roi_min_z = self.get_parameter('opponent_det_pipeline.roi_masking.roi.min_z').value - self.border
        self.roi_max_z = self.get_parameter('opponent_det_pipeline.roi_masking.roi.max_z').value + self.border
        
        self.mask_color = self.get_parameter('opponent_det_pipeline.roi_masking.mask_color').value
        self.debug = self.get_parameter('opponent_det_pipeline.roi_masking.debug').value
        
        # Get topics
        self.input_topic = self.get_parameter('opponent_det_pipeline.roi_masking.input_topic').value
        self.output_topic = self.get_parameter('opponent_det_pipeline.roi_masking.output_topic').value
        self.debug_mask_topic = self.get_parameter('opponent_det_pipeline.roi_masking.debug_topic_mask').value
        self.debug_overlay_topic = self.get_parameter('opponent_det_pipeline.roi_masking.debug_topic_overlay').value
        
        # =================== INIT ===================
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Camera variables
        self.camera_matrix = None
        self.image_width = 0
        self.image_height = 0
        self.camera_info_received = False
        
        # ROI mask (will be computed once camera info is available)
        self.roi_mask = None
        self.mask_lock = threading.Lock()
        
        # =================== COMMS ===================
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.input_topic,
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/arena_camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.masked_image_pub = self.create_publisher(
            Image,
            self.output_topic,
            10
        )
        
        if self.debug:
            self.debug_mask_pub = self.create_publisher(
                Image,
                self.debug_mask_topic,
                10
            )
            self.debug_roi_overlay_pub = self.create_publisher(
                Image,
                self.debug_overlay_topic,
                10
            )
        
        self.get_logger().info(
            f"ROI Masking Node initialized with ROI: "
            f"X[{self.roi_min_x:.2f}, {self.roi_max_x:.2f}], "
            f"Y[{self.roi_min_y:.2f}, {self.roi_max_y:.2f}], "
            f"Z[{self.roi_min_z:.2f}, {self.roi_max_z:.2f}] (with {self.border:.2f}m border)"
        )
        self.get_logger().info(f"Input topic: {self.input_topic}, Output topic: {self.output_topic}")
        self.get_logger().info(f"Enabled: {self.enabled}")
    
    def camera_info_callback(self, msg):
        """Get camera info and compute ROI mask"""
        if self.camera_info_received:
            return
            
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.image_width = msg.width
        self.image_height = msg.height
        self.camera_info_received = True
        
        if self.enabled:
            self.get_logger().info("Camera info received, computing ROI mask...")
            self.compute_roi_mask()
        else:
            self.get_logger().info("ROI masking disabled, skipping mask computation")
    
    def compute_roi_mask(self):
        """Compute the ROI mask by projecting world ROI corners into image space"""
        try:
            # Get transform from world to camera
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                self.world_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"Could not get transform for ROI mask: {e}")
            # Schedule retry
            self.create_timer(1.0, self.compute_roi_mask)
            return
        
        # Define ROI corners in world coordinates (rectangle on ground plane)
        world_corners = [
            [self.roi_min_x, self.roi_min_y, self.roi_min_z],  # bottom-left
            [self.roi_max_x, self.roi_min_y, self.roi_min_z],  # bottom-right
            [self.roi_max_x, self.roi_max_y, self.roi_min_z],  # top-right
            [self.roi_min_x, self.roi_max_y, self.roi_min_z],  # top-left
        ]
        
        # Add intermediate points for better accuracy with lens distortion
        steps = 5
        for i in range(steps):
            alpha = i / steps
            # Bottom edge
            world_corners.append([
                self.roi_min_x + alpha * (self.roi_max_x - self.roi_min_x),
                self.roi_min_y,
                self.roi_min_z
            ])
            # Top edge
            world_corners.append([
                self.roi_min_x + alpha * (self.roi_max_x - self.roi_min_x),
                self.roi_max_y,
                self.roi_min_z
            ])
            # Left edge
            world_corners.append([
                self.roi_min_x,
                self.roi_min_y + alpha * (self.roi_max_y - self.roi_min_y),
                self.roi_min_z
            ])
            # Right edge
            world_corners.append([
                self.roi_max_x,
                self.roi_min_y + alpha * (self.roi_max_y - self.roi_min_y),
                self.roi_min_z
            ])
        
        # Project all corners to image space
        image_points = []
        
        for world_point in world_corners:
            # Create point stamped
            point_world = PointStamped()
            point_world.header.frame_id = self.world_frame
            point_world.point.x = world_point[0]
            point_world.point.y = world_point[1]
            point_world.point.z = world_point[2]
            
            try:
                # Transform to camera frame
                point_camera = do_transform_point(point_world, transform)
                
                # Project to image
                point_3d = np.array([
                    point_camera.point.x,
                    point_camera.point.y,
                    point_camera.point.z
                ])
                
                pixel_hom = self.camera_matrix @ point_3d
                
                if pixel_hom[2] > 0:  # Point is in front of camera
                    px = int(pixel_hom[0] / pixel_hom[2])
                    py = int(pixel_hom[1] / pixel_hom[2])
                    
                    # Check if within image bounds
                    if 0 <= px < self.image_width and 0 <= py < self.image_height:
                        image_points.append([px, py])
                        
            except Exception as e:
                self.get_logger().debug(f"Could not project point: {e}")
                continue
        
        if len(image_points) < 3:
            self.get_logger().warn("Could not project enough points for ROI mask")
            return
        
        # Create mask
        with self.mask_lock:
            # Create empty mask
            self.roi_mask = np.zeros((self.image_height, self.image_width), dtype=np.uint8)
            
            # Convert to numpy array and get convex hull
            points = np.array(image_points, dtype=np.int32)
            hull = cv2.convexHull(points)
            
            # Fill polygon
            cv2.fillPoly(self.roi_mask, [hull], 255)
            
            # Apply morphological operations to clean up mask
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            self.roi_mask = cv2.morphologyEx(self.roi_mask, cv2.MORPH_CLOSE, kernel)
            self.roi_mask = cv2.morphologyEx(self.roi_mask, cv2.MORPH_OPEN, kernel)
            
            self.get_logger().info(f"ROI mask created with {len(image_points)} points")
            
            # Publish debug mask if enabled
            if self.debug:
                self.publish_debug_mask(image_points)
    
    def publish_debug_mask(self, image_points):
        """Publish debug visualizations of the ROI mask"""
        # Create mask visualization (color for better visibility)
        mask_viz = cv2.cvtColor(self.roi_mask, cv2.COLOR_GRAY2BGR)
        mask_viz[self.roi_mask > 0] = [0, 255, 0]  # Green for ROI
        
        # Draw points
        for pt in image_points:
            cv2.circle(mask_viz, tuple(pt), 3, (255, 0, 0), -1)
        
        # Add text
        cv2.putText(mask_viz, f"ROI Mask ({len(image_points)} points)", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(mask_viz, f"World ROI: [{self.roi_min_x:.1f}, {self.roi_max_x:.1f}] x [{self.roi_min_y:.1f}, {self.roi_max_y:.1f}]",
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        out_msg = self.bridge.cv2_to_imgmsg(mask_viz, "bgr8")
        out_msg.header.frame_id = self.camera_frame
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_mask_pub.publish(out_msg)
    
    def image_callback(self, msg):
        """Apply ROI mask to incoming images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return
        
        # If disabled, just forward the image
        if not self.enabled:
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            out_msg.header = msg.header
            out_msg.header.frame_id = msg.header.frame_id
            self.masked_image_pub.publish(out_msg)
            return
        
        # Check if we have mask
        with self.mask_lock:
            if self.roi_mask is None:
                # No mask yet, try to compute it
                if self.camera_info_received:
                    self.compute_roi_mask()
                
                # Forward original image with warning overlay if in debug mode
                if self.debug:
                    cv_image_copy = cv_image.copy()
                    cv2.putText(cv_image_copy, "WAITING FOR ROI MASK...", (50, 50),
                               cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                    masked_image = cv_image_copy
                else:
                    # If not debug, just forward original (no mask yet)
                    masked_image = cv_image
            else:
                # Apply mask
                if len(cv_image.shape) == 3:
                    # Color image
                    mask_3channel = cv2.cvtColor(self.roi_mask, cv2.COLOR_GRAY2BGR)
                    masked_image = cv2.bitwise_and(cv_image, mask_3channel)
                else:
                    # Grayscale image
                    masked_image = cv2.bitwise_and(cv_image, self.roi_mask)
                
                # Debug: create overlay visualization
                if self.debug:
                    overlay = cv_image.copy()
                    # Draw ROI boundary in green
                    contours, _ = cv2.findContours(self.roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cv2.drawContours(overlay, contours, -1, (0, 255, 0), 2)
                    
                    # Add semi-transparent mask overlay
                    mask_color = np.zeros_like(overlay)
                    mask_color[self.roi_mask > 0] = [0, 255, 0]  # Green tint for ROI
                    overlay = cv2.addWeighted(overlay, 0.7, mask_color, 0.3, 0)
                    
                    cv2.putText(overlay, "ROI Overlay (Green = Keep)", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    
                    out_msg = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
                    out_msg.header = msg.header
                    self.debug_roi_overlay_pub.publish(out_msg)
        
        # Publish masked image
        out_msg = self.bridge.cv2_to_imgmsg(masked_image, "bgr8")
        out_msg.header = msg.header  # Keep original timestamp
        out_msg.header.frame_id = msg.header.frame_id
        self.masked_image_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ROIMaskingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()