#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
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
                
                # timing parameters
                ('update_rate', 60.0),  # hz (main update loop rate)
                ('transform_timeout', 0.1),  # seconds
                ('static_transform_retry_period', 1.0),  # seconds between retry attempts
                
                # marker configuration parameters
                ('detection_marker_frames', rclpy.Parameter.Type.STRING_ARRAY),
                ('robot_marker_frames', rclpy.Parameter.Type.STRING_ARRAY),
                ('marker_names', rclpy.Parameter.Type.STRING_ARRAY),
            ]
        )
        
        # ========== parameter loading ==========
        # frame parameters
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.detection_base_frame = self.get_parameter('detection_base_frame').value
        
        # topic parameters
        self.visible_marker_topic = self.get_parameter('visible_marker_topic').value
        
        # timing parameters
        update_rate = self.get_parameter('update_rate').value
        self.update_rate = 1.0 / update_rate if update_rate > 0 else 1.0 / 60.0
        self.transform_timeout = self.get_parameter('transform_timeout').value
        self.static_transform_retry_period = self.get_parameter('static_transform_retry_period').value
        
        # marker configuration
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
        
        # ========== log parameters ==========
        self.get_logger().info("robot detection node started with parameters:")
        self.get_logger().info(f"  camera_frame: {self.camera_frame}")
        self.get_logger().info(f"  base_frame: {self.base_frame}")
        self.get_logger().info(f"  detection_base_frame: {self.detection_base_frame}")
        self.get_logger().info(f"  visible_marker_topic: {self.visible_marker_topic}")
        self.get_logger().info(f"  update_rate: {update_rate} hz")
        self.get_logger().info(f"  transform_timeout: {self.transform_timeout} s")
        self.get_logger().info(f"  static_transform_retry_period: {self.static_transform_retry_period} s")
        self.get_logger().info(f"  configured_markers: {len(self.marker_names)}")
        for i in range(len(self.marker_names)):
            self.get_logger().info(f"    marker {i+1}: detection='{self.detection_marker_frames[i]}', "
                                 f"robot='{self.robot_marker_frames[i]}', name='{self.marker_names[i]}'")
        
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
        
        # store static transforms - USE LISTS DIRECTLY
        self.marker_to_base_transforms = {}
        
        # track loading status
        self.initialized = False
        self.loading_complete = False
        self.failed_markers = set(self.marker_names) if self.marker_names else set()
        
        # track current visible marker
        self.current_visible_marker = "none"
        self.last_published_marker = ""
        
        # start with loading static transforms
        self.load_static_transforms_once()
        
        # if not all loaded, start retry timer
        if not self.loading_complete:
            self.get_logger().warn("Not all static transforms loaded. Will retry periodically.")
            self.retry_timer = self.create_timer(self.static_transform_retry_period, self.retry_load_static_transforms)
        
        # timer for transform updates (only start after initialization)
        self.update_timer = None
        if self.loading_complete:
            self.start_update_timer()
    
    def start_update_timer(self):
        """start the main update timer"""
        if self.update_timer is None:
            self.update_timer = self.create_timer(self.update_rate, self.update_transform)
            self.get_logger().info("Started main update loop")
    
    def load_static_transforms_once(self):
        """attempt to load static transforms once for all configured markers"""
        self.get_logger().info("Attempting to load static transforms...")
        
        if not self.marker_names:
            self.get_logger().warn("No markers configured! Node will not function properly.")
            self.initialized = True
            self.loading_complete = True
            return True
        
        newly_loaded = 0
        still_failed = []
        
        for i, marker_name in enumerate(self.marker_names):
            # skip if already loaded
            if marker_name in self.marker_to_base_transforms:
                continue
                
            robot_marker_frame = self.robot_marker_frames[i]
            detection_marker_frame = self.detection_marker_frames[i]
            
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
                
                # remove from failed set
                if marker_name in self.failed_markers:
                    self.failed_markers.remove(marker_name)
                
                self.get_logger().info(f"✓ Successfully loaded static transform for '{marker_name}' marker")
                newly_loaded += 1
                
            except tf2_ros.TransformException as e:
                self.get_logger().debug(f"Transform not yet available for '{marker_name}': {e}")
                still_failed.append(marker_name)
                if marker_name not in self.failed_markers:
                    self.failed_markers.add(marker_name)
        
        # update loading status
        self.loading_complete = len(self.failed_markers) == 0
        self.initialized = self.loading_complete
        
        if newly_loaded > 0:
            self.get_logger().info(f"Loaded {newly_loaded} new transforms. "
                                 f"Total loaded: {len(self.marker_to_base_transforms)}/{len(self.marker_names)}")
        
        if still_failed:
            self.get_logger().info(f"Still waiting for transforms: {', '.join(still_failed)}")
        
        return self.loading_complete
    
    def retry_load_static_transforms(self):
        """retry loading static transforms for failed markers"""
        if self.loading_complete:
            # all loaded, we can stop retrying
            if hasattr(self, 'retry_timer'):
                self.retry_timer.cancel()
                delattr(self, 'retry_timer')
            
            # start the main update timer if not already started
            if self.update_timer is None:
                self.start_update_timer()
            return
        
        self.get_logger().info("Retrying to load static transforms...")
        success = self.load_static_transforms_once()
        
        if success:
            self.get_logger().info("All static transforms successfully loaded!")
            # stop retry timer
            if hasattr(self, 'retry_timer'):
                self.retry_timer.cancel()
                delattr(self, 'retry_timer')
            
            # start the main update timer
            self.start_update_timer()
    
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
    
    def get_detected_transform(self, parent_frame, child_frame):
        """try to get a transform for a detected marker, checking its age"""
        try:
            transform = self.tf_buffer.lookup_transform(
                parent_frame,
                child_frame,
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
            
            transform, age_ns = self.get_detected_transform(self.camera_frame, detection_marker_frame)
            
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
        # Only publish if marker has changed to reduce traffic
        if marker_name != self.last_published_marker:
            msg = String()
            msg.data = marker_name
            self.marker_publisher.publish(msg)
            self.last_published_marker = marker_name
            self.get_logger().debug(f"published visible marker: {marker_name}")
    
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