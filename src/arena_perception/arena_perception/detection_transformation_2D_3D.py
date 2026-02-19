#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs
from sensor_msgs.msg import CameraInfo
from vision_msgs.msg import Detection2DArray, Detection3D, Detection3DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Point, Pose, PointStamped, PoseArray
from std_msgs.msg import Header
import numpy as np


class Detection2DTo3DConverter(Node):
    """Convert 2D detections to 3D points using camera projection onto a fixed Z-plane."""
    
    def __init__(self):
        super().__init__('detection_2d_to_3d_node')
        
        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('target_frame', 'world'),
                ('camera_frame', 'arena_camera_optical'),
                ('z_plane', 0.0),
                ('confidence_threshold', 0.5),
                ('tf_timeout_sec', 0.1),
                ('publish_visualization', True),  # New parameter
            ]
        )
        
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.z_plane = self.get_parameter('z_plane').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.tf_timeout = rclpy.duration.Duration(seconds=self.get_parameter('tf_timeout_sec').value)
        self.publish_visualization = self.get_parameter('publish_visualization').value
        
        # State
        self.camera_info = None
        self.camera_matrix = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Main publisher for 3D detections (application use)
        self.publisher = self.create_publisher(Detection3DArray, 'detections_3d', 10)
        
        # Optional visualization publishers (for RViz2)
        if self.publish_visualization:
            self.viz_point_publisher = self.create_publisher(PointStamped, 'detections_3d_viz_point', 10)
            self.viz_pose_array_publisher = self.create_publisher(PoseArray, 'detections_3d_viz_poses', 10)
        
        # Subscribers with appropriate QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.create_subscription(CameraInfo, 'camera_info', self.camera_info_callback, qos)
        self.create_subscription(Detection2DArray, 'detections_2d', self.detections_callback, 10)
        
        self.get_logger().info(
            f'Node initialized\n'
            f'  Target frame: {self.target_frame}\n'
            f'  Camera frame: {self.camera_frame}\n'
            f'  Z-plane: {self.z_plane}\n'
            f'  Confidence threshold: {self.confidence_threshold}\n'
            f'  Publish visualization: {self.publish_visualization}'
        )

    def camera_info_callback(self, msg):
        """Store camera intrinsics."""
        if self.camera_info is None:
            self.camera_info = msg
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info('Camera info received')

    def pixel_to_camera_ray(self, u, v):
        """
        Convert pixel to ray in camera frame.
        
        Returns:
            tuple: (ray_origin, ray_direction) in camera frame
        """
        if self.camera_matrix is None:
            return None, None
            
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Ray origin is camera center (0,0,0) in camera frame
        origin = np.array([0.0, 0.0, 0.0])
        
        # Ray direction from pixel coordinates
        direction = np.array([
            (u - cx) / fx,
            (v - cy) / fy,
            1.0
        ])
        direction = direction / np.linalg.norm(direction)
        
        return origin, direction

    def project_to_plane(self, ray_origin, ray_direction, plane_z):
        """
        Project ray onto horizontal plane at given Z.
        
        Returns:
            np.array: 3D point or None if no intersection
        """
        # Avoid division by zero (ray parallel to plane)
        if abs(ray_direction[2]) < 1e-6:
            return None
            
        # Solve for t where ray_origin[2] + t * ray_direction[2] = plane_z
        t = (plane_z - ray_origin[2]) / ray_direction[2]
        
        if t < 0:  # Intersection behind camera
            return None
            
        point = ray_origin + t * ray_direction
        return point

    def detections_callback(self, msg):
        """Process incoming detections and publish 3D detection array."""
        if self.camera_info is None:
            self.get_logger().warn('Waiting for camera info...', throttle_duration_sec=5.0)
            return
        
        # Filter by confidence
        valid_detections = []
        for detection in msg.detections:
            if detection.results and detection.results[0].hypothesis.score >= self.confidence_threshold:
                valid_detections.append(detection)
        
        if not valid_detections:
            return
        
        # Get latest transform from camera to target frame
        try:
            # Use empty time to get the latest available transform
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.camera_frame,
                rclpy.time.Time(),  # Empty time = latest available transform
                self.tf_timeout
            )
        except TransformException as e:
            self.get_logger().warn(f'TF error getting latest transform: {e}', throttle_duration_sec=2.0)
            return
        
        # Prepare 3D detection array
        detection_array_3d = Detection3DArray()
        detection_array_3d.header = Header()
        detection_array_3d.header.stamp = self.get_clock().now().to_msg()
        detection_array_3d.header.frame_id = self.target_frame
        
        # Prepare visualization containers
        if self.publish_visualization:
            viz_pose_array = PoseArray()
            viz_pose_array.header = detection_array_3d.header
        
        points_converted = 0
        
        for detection in valid_detections:
            # Get pixel coordinates of bounding box center
            u = detection.bbox.center.position.x
            v = detection.bbox.center.position.y
            
            # Get ray in camera frame
            ray_origin_cam, ray_dir_cam = self.pixel_to_camera_ray(u, v)
            if ray_origin_cam is None:
                continue
            
            # Create points for transformation
            origin_cam = PointStamped()
            origin_cam.header.frame_id = self.camera_frame
            origin_cam.header.stamp = msg.header.stamp  # Keep original timestamp for reference
            origin_cam.point.x = ray_origin_cam[0]
            origin_cam.point.y = ray_origin_cam[1]
            origin_cam.point.z = ray_origin_cam[2]
            
            # Transform ray endpoint to get direction in target frame
            endpoint_cam = PointStamped()
            endpoint_cam.header = origin_cam.header
            endpoint_cam.point.x = ray_origin_cam[0] + ray_dir_cam[0]
            endpoint_cam.point.y = ray_origin_cam[1] + ray_dir_cam[1]
            endpoint_cam.point.z = ray_origin_cam[2] + ray_dir_cam[2]
            
            try:
                # Transform points to target frame using the latest transform
                origin_target = tf2_geometry_msgs.do_transform_point(origin_cam, transform)
                endpoint_target = tf2_geometry_msgs.do_transform_point(endpoint_cam, transform)
                
                # Get ray in target frame
                ray_origin = np.array([
                    origin_target.point.x,
                    origin_target.point.y,
                    origin_target.point.z
                ])
                
                ray_end = np.array([
                    endpoint_target.point.x,
                    endpoint_target.point.y,
                    endpoint_target.point.z
                ])
                
                ray_direction = ray_end - ray_origin
                ray_direction = ray_direction / np.linalg.norm(ray_direction)
                
                # Project onto Z-plane
                point_3d = self.project_to_plane(ray_origin, ray_direction, self.z_plane)
                
                if point_3d is not None:
                    points_converted += 1
                    
                    # ===== MAIN DETECTION3D OUTPUT =====
                    # Create 3D detection
                    detection_3d = Detection3D()
                    
                    # Set header (use original timestamp for tracking consistency)
                    detection_3d.header = Header()
                    detection_3d.header.stamp = detection.header.stamp
                    detection_3d.header.frame_id = self.target_frame
                    
                    # Set the 3D position
                    detection_3d.bbox.center.position.x = point_3d[0]
                    detection_3d.bbox.center.position.y = point_3d[1]
                    detection_3d.bbox.center.position.z = point_3d[2]
                    
                    # Set orientation (identity - no rotation information)
                    detection_3d.bbox.center.orientation.x = 0.0
                    detection_3d.bbox.center.orientation.y = 0.0
                    detection_3d.bbox.center.orientation.z = 0.0
                    detection_3d.bbox.center.orientation.w = 1.0
                    
                    # Set size (unknown, but we could potentially estimate from 2D bbox)
                    # For now, set to zeros or small default value
                    detection_3d.bbox.size.x = 0.0
                    detection_3d.bbox.size.y = 0.0
                    detection_3d.bbox.size.z = 0.0
                    
                    # Copy results/hypotheses from original detection
                    # This preserves class IDs, confidence scores, and any tracking IDs
                    for result in detection.results:
                        hypothesis_with_pose = ObjectHypothesisWithPose()
                        hypothesis_with_pose.hypothesis.class_id = result.hypothesis.class_id
                        hypothesis_with_pose.hypothesis.score = result.hypothesis.score
                        
                        # If the original detection had pose information, copy it
                        # Otherwise, leave as default
                        if hasattr(result, 'pose'):
                            hypothesis_with_pose.pose = result.pose
                        
                        detection_3d.results.append(hypothesis_with_pose)
                    
                    # Copy ID if present (for tracking)
                    if hasattr(detection, 'id'):
                        detection_3d.id = detection.id
                    
                    # Add to array
                    detection_array_3d.detections.append(detection_3d)
                    
                    # ===== VISUALIZATION OUTPUTS (for RViz2) =====
                    if self.publish_visualization:
                        # Add to PoseArray for visualization
                        pose = Pose()
                        pose.position.x = point_3d[0]
                        pose.position.y = point_3d[1]
                        pose.position.z = point_3d[2]
                        pose.orientation.w = 1.0
                        viz_pose_array.poses.append(pose)
                        
                        # Publish individual PointStamped for each detection
                        # (only if you want separate points, otherwise just use PoseArray)
                        point_msg = PointStamped()
                        point_msg.header.stamp = self.get_clock().now().to_msg()
                        point_msg.header.frame_id = self.target_frame
                        point_msg.point.x = point_3d[0]
                        point_msg.point.y = point_3d[1]
                        point_msg.point.z = point_3d[2]
                        self.viz_point_publisher.publish(point_msg)
                        
            except Exception as e:
                self.get_logger().debug(f'Failed to transform point: {e}')
                continue
        
        # Publish main detection array
        if detection_array_3d.detections:
            self.publisher.publish(detection_array_3d)
            self.get_logger().info(f'Published {len(detection_array_3d.detections)} 3D detections')
        
        # Publish visualization PoseArray
        if self.publish_visualization and viz_pose_array.poses:
            self.viz_pose_array_publisher.publish(viz_pose_array)
            self.get_logger().info(f'Published {len(viz_pose_array.poses)} visualization poses')
        elif points_converted > 0:
            self.get_logger().info(f'Converted {points_converted} detections')


def main(args=None):
    rclpy.init(args=args)
    node = Detection2DTo3DConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()