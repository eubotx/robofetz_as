#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import threading
from scipy.spatial.transform import Rotation

class FrameDiffRobotDetector(Node):
    """
    Robust Opponent Detection using MOG2.
    Refactored to synchronous processing (in callback) to ensure stability.
    """

    def __init__(self):
        super().__init__('frame_diff_oponent_detector')

        # =================== PARAMETERS ===================
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_contour_area', 300),
                ('max_contour_area', 10000),
                ('background_history', 500),
                ('var_threshold', 16),
                ('ignore_radius_px', 100),
                ('shadow_expansion_factor', 1.2),
                ('debug', True),
                ('robot_base_frame', 'robot_base'),
                ('camera_optical_frame', 'arena_camera_optical'),
            ]
        )

        # =================== INIT ===================
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MOG2
        history = self.get_parameter('background_history').value
        var_threshold = self.get_parameter('var_threshold').value
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=history,
            varThreshold=var_threshold,
            detectShadows=True
        )

        # Variables
        self.camera_matrix = None
        self.camera_matrix_inv = None
        self.image_width = 0
        self.image_height = 0
        self.camera_frame_id = ""
        self.own_robot_position_px = None
        self.own_robot_last_update = None
        self.own_robot_base_radius_px = self.get_parameter('ignore_radius_px').value
        
        # Motion Tracking State
        self.prev_world_pos = None
        self.prev_pixel_pos = None
        
        self.current_world_orientation = 0.0
        self.current_pixel_orientation = 0.0
        
        self.declare_parameter('motion_threshold', 0.02) # 2cm movement required to update angle
        
        self.lock = threading.Lock()

        # =================== COMMS ===================
        # NOTE: Synchronous usage means we don't use a timer for processing anymore.
        self.image_sub = self.create_subscription(Image, '/arena_camera/image_rect', self.image_callback, 10)
        # Using raw camera info as rectified info seems silent in simulation
        self.camera_info_sub = self.create_subscription(CameraInfo, '/arena_camera/camera_info', self.camera_info_callback, 10)
        self.robot_pose_sub = self.create_subscription(PoseStamped, '/bot/pose', self.robot_pose_callback, 10)
        
        self.pose_publisher = self.create_publisher(PoseStamped, '/detected_robot/pose', 10)
        self.debug_publisher = self.create_publisher(Image, '/debug/detection_image', 10)

        # Timer only for cleanup
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_stale_data)

        self.get_logger().info("MOG2 Opponent Detector (Synchronous) initialized.")

    def camera_info_callback(self, msg):
        if self.camera_matrix is not None: return
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.image_width = msg.width
        self.image_height = msg.height
        try:
            self.camera_matrix_inv = np.linalg.inv(self.camera_matrix)
            self.get_logger().info("Camera info received.")
        except: pass

    def robot_pose_callback(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.get_parameter('camera_optical_frame').value,
                'world',
                rclpy.time.Time()
            )
            robot_point = PointStamped()
            robot_point.header = msg.header
            robot_point.header.frame_id = 'world'
            robot_point.point = msg.pose.position
            camera_point = do_transform_point(robot_point, transform)

            if self.camera_matrix is not None:
                point_3d = np.array([camera_point.point.x, camera_point.point.y, camera_point.point.z])
                pixel_hom = self.camera_matrix @ point_3d
                if pixel_hom[2] > 0:
                    px = int(pixel_hom[0] / pixel_hom[2])
                    py = int(pixel_hom[1] / pixel_hom[2])
                    if 0 <= px < self.image_width and 0 <= py < self.image_height:
                        with self.lock:
                            self.own_robot_position_px = (px, py)
                            self.own_robot_last_update = self.get_clock().now()
        except: pass

    def image_callback(self, msg):
        """Processes image immediately upon receipt."""
        try:
            # Reverting to the simple conversion that WORKED in Minimal Node
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.camera_frame_id = msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")
            return

        debug_frame = cv_image.copy() # Local copy for drawing
        
        # Check Camera Info
        if self.camera_matrix is None:
            cv2.putText(debug_frame, "WAITING FOR CAMERA INFO...", (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            self.publish_debug(debug_frame)
            return

        with self.lock:
            # IMPORTANT: Deep copy to ensure we own the memory.
            # cv_bridge might return a view of the ROS message, which gets recycled.
            self.latest_image = cv_image.copy()
            robot_pos = self.own_robot_position_px

        # === 1. MOG2 Processing ===
        proc_frame = debug_frame.copy() 
        fg_mask = self.bg_subtractor.apply(proc_frame)
        
        # 2. Threshold
        _, thresh = cv2.threshold(fg_mask, 250, 255, cv2.THRESH_BINARY)
        
        # 3. Morphology
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)
        
        # 4. Ego Robot Masking
        if robot_pos is not None:
            mask = np.zeros(thresh.shape, dtype=np.uint8)
            radius = int(self.own_robot_base_radius_px * self.get_parameter('shadow_expansion_factor').value)
            cv2.circle(mask, robot_pos, radius, 255, -1)
            thresh = cv2.bitwise_and(thresh, thresh, mask=cv2.bitwise_not(mask))

        # 5. Contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_contour = None
        max_score = 0
        min_area = self.get_parameter('min_contour_area').value
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > min_area:
                x,y,w,h = cv2.boundingRect(cnt)
                aspect = float(w)/h
                if 0.3 < aspect < 3.0:
                    if area > max_score:
                        max_score = area
                        best_contour = cnt

        # 6. Publish
        pixel_pos = None

        if best_contour is not None:
            pixel_pos = self.get_robot_centroid(best_contour)
            if pixel_pos:
                world_pos = self.pixel_to_world(pixel_pos[0], pixel_pos[1])
                if world_pos is not None:
                    # --- Motion Based Orientation Logic ---
                    threshold = self.get_parameter('motion_threshold').value
                    
                    # 1. World Frame Tracking (For Publishing)
                    if self.prev_world_pos is None:
                        self.prev_world_pos = world_pos
                    else:
                        dx = world_pos[0] - self.prev_world_pos[0]
                        dy = world_pos[1] - self.prev_world_pos[1]
                        dist = np.sqrt(dx*dx + dy*dy)
                        
                        if dist > threshold:
                            self.current_world_orientation = np.arctan2(dy, dx)
                            self.prev_world_pos = world_pos
                    
                    # 2. Pixel Frame Tracking (For Visual Debugging)
                    # Use a slightly larger threshold in pixels (20px ~= 2cm approx depending on height)
                    if self.prev_pixel_pos is None:
                        self.prev_pixel_pos = pixel_pos
                    else:
                        pdx = pixel_pos[0] - self.prev_pixel_pos[0]
                        pdy = pixel_pos[1] - self.prev_pixel_pos[1]
                        pdist = np.sqrt(pdx*pdx + pdy*pdy)
                        
                        if pdist > 10: # 10 pixels movement
                             # In image space, Y is down, so standard atan2 works for image coordinates
                            self.current_pixel_orientation = np.arctan2(pdy, pdx)
                            self.prev_pixel_pos = pixel_pos

                    self.publish_pose(world_pos, self.current_world_orientation)

        if self.get_parameter('debug').value:
             self.publish_debug(debug_frame, best_contour, pixel_pos, robot_pos, self.current_pixel_orientation)

    def publish_debug(self, frame, contour=None, pixel_pos=None, robot_pos=None, orientation=0.0):
        # Simplified Drawing
        debug = frame # Already copied in callback
        
        if robot_pos is not None:
            radius = int(self.own_robot_base_radius_px * self.get_parameter('shadow_expansion_factor').value)
            cv2.circle(debug, robot_pos, radius, (0, 255, 0), 2)

        if contour is not None and pixel_pos is not None:
            cv2.drawContours(debug, [contour], -1, (0, 0, 255), 3)
            # Heading visual
            length = 40
            end_x = int(pixel_pos[0] + length * np.cos(orientation))
            end_y = int(pixel_pos[1] + length * np.sin(orientation))
            cv2.line(debug, pixel_pos, (end_x, end_y), (255, 255, 0), 2)

        out_msg = self.bridge.cv2_to_imgmsg(debug, "bgr8")
        out_msg.header.frame_id = self.camera_frame_id
        out_msg.header.stamp = self.get_clock().now().to_msg()
        self.debug_publisher.publish(out_msg)

    def get_robot_centroid(self, contour):
        M = cv2.moments(contour)
        if M['m00'] == 0: return None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return (cx, cy)

    def pixel_to_world(self, u, v):
        try:
            transform = self.tf_buffer.lookup_transform('world', self.get_parameter('camera_optical_frame').value, rclpy.time.Time())
            trans = transform.transform.translation
            cam_pos = np.array([trans.x, trans.y, trans.z])
            rot = transform.transform.rotation
            R = Rotation.from_quat([rot.x, rot.y, rot.z, rot.w]).as_matrix()
            
            pixel_hom = np.array([u, v, 1.0])
            ray_cam = self.camera_matrix_inv @ pixel_hom
            ray_cam /= np.linalg.norm(ray_cam)
            ray_world = R @ ray_cam
            
            if abs(ray_world[2]) < 1e-6: return None
            t = -cam_pos[2] / ray_world[2]
            if t < 0: return None
            return cam_pos + t * ray_world
        except: return None

    def publish_pose(self, position, orientation):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = 0.0
        rot = Rotation.from_euler('z', orientation, degrees=False)
        q = rot.as_quat()
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        self.pose_publisher.publish(msg)

    def cleanup_stale_data(self):
        now = self.get_clock().now()
        with self.lock:
            if self.own_robot_last_update:
                if (now - self.own_robot_last_update).nanoseconds > 1.0e9:
                    self.own_robot_position_px = None

def main(args=None):
    rclpy.init(args=args)
    node = FrameDiffRobotDetector()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()