#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String, Header
import tf2_ros
import tf_transformations
import numpy as np

class RobotDetectionNode(Node):
    def __init__(self):
        super().__init__('robot_detection_node')
        
        # ========== parameter declaration ==========
        self.declare_parameters(
            namespace='',
            parameters=[
                # frame parameters
                ('camera_frame', 'arena_camera_optical'),
                ('base_frame', 'robot/base_footprint'),
                ('detection_base_frame', 'arena_perception/robot/base_footprint'),
                
                # topic parameters
                ('visible_marker_topic', 'arena_perception/robot/visible_marker'),
                ('robot_pose_topic', 'arena_perception/robot/pose'),
                
                # timing parameters
                ('update_rate', 60.0),  # hz (main update loop rate)
                ('transform_timeout', 0.2),  # seconds
                
                # marker configuration parameters - CHANGED TYPE TO BYTE_ARRAY
                ('detection_marker_frames', rclpy.Parameter.Type.STRING_ARRAY),  # Changed
                ('robot_marker_frames', rclpy.Parameter.Type.STRING_ARRAY),       # Changed
                ('marker_names', rclpy.Parameter.Type.STRING_ARRAY),              # Changed
                
                # logging
                ('log_level', 'INFO')
            ]
        )
        
        # ========== parameter loading ==========
        # frame parameters
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.detection_base_frame = self.get_parameter('detection_base_frame').value
        
        # topic parameters
        self.visible_marker_topic = self.get_parameter('visible_marker_topic').value
        self.robot_pose_topic = self.get_parameter('robot_pose_topic').value
        
        # timing parameters
        update_rate = self.get_parameter('update_rate').value
        self.update_rate = 1.0 / update_rate if update_rate > 0 else 1.0 / 60.0
        self.transform_timeout = self.get_parameter('transform_timeout').value
        
        # marker configuration - USE LISTS DIRECTLY
        self.detection_marker_frames = self.get_parameter('detection_marker_frames').value
        self.robot_marker_frames = self.get_parameter('robot_marker_frames').value
        self.marker_names = self.get_parameter('marker_names').value
        
        # Convert to lists and ensure string type
        if not isinstance(self.detection_marker_frames, list):
            self.detection_marker_frames = []
        if not isinstance(self.robot_marker_frames, list):
            self.robot_marker_frames = []
        if not isinstance(self.marker_names, list):
            self.marker_names = []
        
        # Validate marker configuration
        if not (len(self.detection_marker_frames) == len(self.robot_marker_frames) == len(self.marker_names)):
            self.get_logger().error(f"Marker lists have different lengths: "
                                  f"detection_frames={len(self.detection_marker_frames)}, "
                                  f"robot_frames={len(self.robot_marker_frames)}, "
                                  f"marker_names={len(self.marker_names)}")
        
        # set log level
        log_level_param = self.get_parameter('log_level').value
        log_level_map = {
            'DEBUG': rclpy.logging.LoggingSeverity.DEBUG,
            'INFO': rclpy.logging.LoggingSeverity.INFO,
            'WARN': rclpy.logging.LoggingSeverity.WARN,
            'ERROR': rclpy.logging.LoggingSeverity.ERROR,
            'FATAL': rclpy.logging.LoggingSeverity.FATAL
        }
        if log_level_param in log_level_map:
            self.get_logger().set_level(log_level_map[log_level_param])
        else:
            self.get_logger().warn(f"invalid log level '{log_level_param}', using INFO instead")
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        
        # ========== log parameters ==========
        self.get_logger().info("robot detection node started with parameters:")
        self.get_logger().info(f"  camera_frame: {self.camera_frame}")
        self.get_logger().info(f"  base_frame: {self.base_frame}")
        self.get_logger().info(f"  detection_base_frame: {self.detection_base_frame}")
        self.get_logger().info(f"  visible_marker_topic: {self.visible_marker_topic}")
        self.get_logger().info(f"  robot_pose_topic: {self.robot_pose_topic}")
        self.get_logger().info(f"  update_rate: {update_rate} hz")
        self.get_logger().info(f"  transform_timeout: {self.transform_timeout} s")
        self.get_logger().info(f"  configured_markers: {len(self.marker_names)}")
        for i in range(len(self.marker_names)):
            self.get_logger().info(f"    marker {i+1}: detection='{self.detection_marker_frames[i]}', "
                                 f"robot='{self.robot_marker_frames[i]}', name='{self.marker_names[i]}'")
        self.get_logger().info(f"  log_level: {log_level_param}")
        
        # ========== initialization ==========
        # tf buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # marker visibility publisher
        self.marker_publisher = self.create_publisher(
            String, 
            self.visible_marker_topic, 
            10
        )
        
        # robot pose publisher
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            self.robot_pose_topic,
            10
        )
        
        # store static transforms - USE LISTS DIRECTLY
        self.marker_to_base_transforms = {}
        
        # track current visible marker
        self.current_visible_marker = "none"
        self.last_published_marker = ""
        
        # load static transforms once
        self.load_static_transforms()
        
        # timer for transform updates
        self.timer = self.create_timer(self.update_rate, self.update_transform)
    
    def load_static_transforms(self):
        """load static transforms from urdf for all configured markers"""
        self.get_logger().info("loading static transforms...")
        
        if not self.marker_names:
            self.get_logger().warn("no markers configured! node will not function properly.")
            return
            
        for i, marker_name in enumerate(self.marker_names):
            robot_marker_frame = self.robot_marker_frames[i]
            detection_marker_frame = self.detection_marker_frames[i]
            
            self.get_logger().info(f"waiting for static transform: {robot_marker_frame} -> {self.base_frame}")
            
            loaded = False
            for attempt in range(20):  # try for 2 seconds
                try:
                    transform = self.tf_buffer.lookup_transform(
                        robot_marker_frame,
                        self.base_frame,
                        rclpy.time.Time()
                    )
                    self.marker_to_base_transforms[marker_name] = {
                        'transform': transform,
                        'robot_marker_frame': robot_marker_frame,
                        'detection_marker_frame': detection_marker_frame
                    }
                    self.get_logger().info(f"✓ loaded static transform for '{marker_name}' marker")
                    loaded = True
                    break
                except tf2_ros.TransformException as e:
                    if attempt == 19:  # last attempt
                        self.get_logger().warn(f"✗ failed to load transform for '{marker_name}': {e}")
                    rclpy.spin_once(self, timeout_sec=0.1)
            
            if not loaded:
                self.get_logger().error(f"could not load transform for marker '{marker_name}'. node may not function properly.")
    
    def compose_transforms(self, transform_a, transform_b):
        """compose two transforms: result = a * b (apply b then a)"""
        # convert to numpy
        t1 = np.array([transform_a.transform.translation.x,
                       transform_a.transform.translation.y,
                       transform_a.transform.translation.z])
        q1 = np.array([transform_a.transform.rotation.x,
                       transform_a.transform.rotation.y,
                       transform_a.transform.rotation.z,
                       transform_a.transform.rotation.w])
        
        t2 = np.array([transform_b.transform.translation.x,
                       transform_b.transform.translation.y,
                       transform_b.transform.translation.z])
        q2 = np.array([transform_b.transform.rotation.x,
                       transform_b.transform.rotation.y,
                       transform_b.transform.rotation.z,
                       transform_b.transform.rotation.w])
        
        # compose rotations
        q_result = tf_transformations.quaternion_multiply(q1, q2)
        
        # rotate t2 by q1 and add to t1
        rot_matrix = tf_transformations.quaternion_matrix(q1)[:3, :3]
        t_rotated = rot_matrix @ t2
        t_result = t1 + t_rotated
        
        # create result
        result = TransformStamped()
        result.transform.translation.x = t_result[0]
        result.transform.translation.y = t_result[1]
        result.transform.translation.z = t_result[2]
        result.transform.rotation.x = q_result[0]
        result.transform.rotation.y = q_result[1]
        result.transform.rotation.z = q_result[2]
        result.transform.rotation.w = q_result[3]
        
        return result
    
    def get_detected_transform(self, detection_marker_frame):
        """try to get a transform for a detected marker, checking its age"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                detection_marker_frame,
                rclpy.time.Time()
            )
            
            # check if transform is recent
            current_time = self.get_clock().now()
            transform_time = rclpy.time.Time.from_msg(transform.header.stamp)
            age_ns = (current_time - transform_time).nanoseconds
            
            return transform, age_ns
                
        except tf2_ros.TransformException:
            return None, None
    
    def find_best_transform(self):
        """find the best (most recent) transform among all available markers"""
        max_age_ns = int(self.transform_timeout * 1e9)
        
        best_transform = None
        best_marker_info = None
        best_marker_name = "none"
        best_age_ns = float('inf')
        
        # check all configured markers
        for marker_name in self.marker_names:
            if marker_name not in self.marker_to_base_transforms:
                continue
                
            marker_info = self.marker_to_base_transforms[marker_name]
            detection_marker_frame = marker_info['detection_marker_frame']
            
            transform, age_ns = self.get_detected_transform(detection_marker_frame)
            
            if transform is not None and age_ns <= max_age_ns:
                # this transform is valid and recent enough
                if age_ns < best_age_ns:
                    best_transform = transform
                    best_marker_info = marker_info
                    best_marker_name = marker_name
                    best_age_ns = age_ns
        
        return best_transform, best_marker_info, best_marker_name, best_age_ns
    
    def publish_visible_marker(self, marker_name):
        """publish the currently visible marker name to the topic"""
        # only publish if marker has changed
        if marker_name != self.last_published_marker:
            msg = String()
            msg.data = marker_name
            self.marker_publisher.publish(msg)
            self.last_published_marker = marker_name
            self.get_logger().debug(f"published visible marker: {marker_name}")
    
    def publish_robot_pose(self, transform):
        """publish the robot pose as a PoseStamped message"""
        # create PoseStamped message
        pose_msg = PoseStamped()
        
        # set header
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.camera_frame
        
        # set position
        pose_msg.pose.position = Point(
            x=transform.transform.translation.x,
            y=transform.transform.translation.y,
            z=transform.transform.translation.z
        )
        
        # set orientation (same as transform)
        pose_msg.pose.orientation = transform.transform.rotation
        
        # publish
        self.pose_publisher.publish(pose_msg)
        
        self.get_logger().debug(f"published robot pose: "
                              f"x={pose_msg.pose.position.x:.3f}, "
                              f"y={pose_msg.pose.position.y:.3f}, "
                              f"z={pose_msg.pose.position.z:.3f}")
    
    def update_transform(self):
        """main update loop to compute and publish camera->base transform"""
        
        # find the best available transform
        detected_transform, marker_info, marker_name, age_ns = self.find_best_transform()
        
        # update current visible marker
        self.current_visible_marker = marker_name
        
        # publish visible marker information (only when it changes)
        self.publish_visible_marker(marker_name)
        
        if detected_transform is None or marker_info is None:
            # no valid transforms available
            if marker_name == "none":
                self.get_logger().debug("no valid marker transforms available")
            return
        
        # get the static transform for this marker
        static_transform = marker_info['transform']
        
        self.get_logger().debug(f"using '{marker_name}' marker (age: {age_ns/1e9:.3f}s)")
        
        # compose transforms: camera -> marker -> base
        camera_to_base = self.compose_transforms(detected_transform, static_transform)
        
        # set header and publish tf
        camera_to_base.header.stamp = self.get_clock().now().to_msg()
        camera_to_base.header.frame_id = self.camera_frame
        camera_to_base.child_frame_id = self.detection_base_frame
        
        self.tf_broadcaster.sendTransform(camera_to_base)
        
        # publish robot pose
        self.publish_robot_pose(camera_to_base)

def main(args=None):
    rclpy.init(args=args)
    
    node = RobotDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()