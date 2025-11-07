import cv2
import numpy as np
import json
import detection.frame_source
import detection.detector
from detection.math_funcs import *
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion (x, y, z, w)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


def quaternion_from_vectors(v1, v2):
    print(f'v1: {v1}, v2: {v2}')
    v1 = np.array(v1, dtype=np.float64).squeeze()
    v2 = np.array(v2, dtype=np.float64).squeeze()
    print(f'v1: {v1}, v2: {v2}')

    v1 /= np.linalg.norm(v1)
    v2 /= np.linalg.norm(v2)

    dot = np.dot(v1, v2)

    if np.allclose(dot, 1.0):
        # Vectors are the same
        return np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion
    elif np.allclose(dot, -1.0):
        # Vectors are opposite
        # Find an orthogonal vector to use as rotation axis
        orthogonal = np.array([1.0, 0.0, 0.0])
        if np.allclose(v1, orthogonal):
            orthogonal = np.array([0.0, 1.0, 0.0])
        axis = np.cross(v1, orthogonal)
        axis /= np.linalg.norm(axis)
        return np.concatenate([[0.0], axis])  # 180-degree rotation

    axis = np.cross(v1, v2)
    s = np.sqrt((1.0 + dot) * 2.0)
    invs = 1.0 / s

    q = np.array([
        axis[0] * invs,
        axis[1] * invs,
        axis[2] * invs,
        s * 0.5
    ])
    return q


class DetectionNode(Node):
    def __init__(self):
        super().__init__('detection_node')

        # Load transformation data
        self.src_points, self.matrix = self.load_transformation_data()
        self.arena_width = 450
        self.arena_height = 450

        self.options = {
            'camera_calibration_path': 'src/detection/data/USBGS720P02-L170_calibration.json',
            'publish_ros': True,
            'frame_path': 'src/detection/recordings/output_april_corner_movement.mp4',
            'webcam_id': 4,
            'webcam_save_stream_path': 'src/detection/recordings/testSeqxxx.mp4',
            # Tags
            'arena_tag': {'id': 12, 'family': 'tagStandard41h12', 'size': 0.125},   #arena tag war mal 2
            'robot_tags': { 'sizes' : 0.125, 'family': 'tagStandard41h12',
                'top_id': 12,
                'bottom_id': 31},
        }

        # Initialize detectors
        self.frame_source = detection.frame_source.GenericSource(detection.frame_source.SourceType.Webcam, options=self.options)
        self.robot_detector = detection.detector.RobotDetector(self.options, self.frame_source.get_calibration())
        self.enemy_detector = detection.detector.EnemyDetector(self.options)
        self.april_tag_detector = detection.detector.AprilTagDetector(self.frame_source.get_calibration())
        self.arena_detector = None
        self.cameraFromWorld = None

        # ROS publisher
        self.robot_pub = self.create_publisher(PoseStamped, '/camera/pose', 10)
        self.enemy_pub = self.create_publisher(PoseStamped, '/camera/enemy/pose', 10)

        # Run main loop
        self.timer = self.create_timer(0.03, self.process_frame)  # ~30 FPS

    def load_transformation_data(self):
        with open("src/detection/data/transformation_data.json", "r") as f:
            data = json.load(f)
        return np.float32(data["src_points"]), np.float32(data["matrix"])

    def draw_forward_direction_axis(self, frame, center, yaw, length=50):
        forward_direction = (math.cos(yaw), math.sin(yaw))
        forward_end = (
            int(center[0] + forward_direction[0] * length),
            int(center[1] - forward_direction[1] * length)
        )
        cv2.arrowedLine(frame, center, forward_end, (0, 255, 0), 5)

    def transform_coordinates(self, point):
        """Transform coordinates from image space (top-left origin) to arena space (bottom-right origin)"""
        return (self.arena_width - point[0], self.arena_height - point[1])

    def transform_yaw(self, yaw):
        """Transform yaw angle to match the flipped coordinate system"""
        return math.pi - yaw

    def process_frame(self):
        frame = self.frame_source.get_frame()
        if frame is None:
            self.get_logger().warn("No frame captured")
            return

        # transformed = cv2.warpPerspective(frame, self.matrix, (self.arena_width, self.arena_height))
        # gray = cv2.cvtColor(transformed, cv2.COLOR_BGR2GRAY)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.cameraFromWorld is None:
            april_tag_output = self.april_tag_detector.detect(gray, self.options['arena_tag']['size'], frame.copy())
            april_tag_detections = april_tag_output["aprilTags"]
            for detection in april_tag_detections:
                if (detection.tag_family.decode() == self.options['arena_tag']['family'] and
                    detection.tag_id == self.options['arena_tag']['id']):
                    self.cameraFromWorld = Pose(detection.pose_R, detection.pose_t)
                    print(
                        f"Detected arena tag: {self.options['arena_tag']['family']}-{self.options['arena_tag']['id']}. "
                        f"Defined cameraFromWorld:\n{self.cameraFromWorld}")
            if not self.cameraFromWorld:
                print(f"Could not find aarena tag: {self.options['arena_tag']['family']}-{self.options['arena_tag']['id']}. Retrying...")
                return

        ##################
        robot_detection = self.robot_detector.detect(gray, self.cameraFromWorld.inv(), frame.copy())
        if robot_detection is None:
            return
        # robot_detection['worldFromRobot'] = Pose(np.array([[1,0,0],[0,-1,0],[0,0,1]])) * robot_detection['worldFromRobot']
        print(f'Robot in world: {robot_detection['worldFromRobot'].t}')
        # Prepare PoseStamped message with arena coordinates
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(robot_detection['worldFromRobot'].t[0])
        pose_msg.pose.position.y = float(robot_detection['worldFromRobot'].t[1])
        # pose_msg.pose.position.z = float(robot_detection['worldFromRobot'].t[2])
        pose_msg.pose.position.z = float(0)

        # Convert arena yaw to quaternion
        # qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, arena_yaw)
        robotXInWorld = robot_detection['worldFromRobot'] * np.array([0, -1, 0])
        qx, qy, qz, qw = quaternion_from_vectors(robotXInWorld, np.array([1, 0, 0]))
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        self.robot_pub.publish(pose_msg)

        #####
        enemy_detection = self.enemy_detector.detect(gray, robot_detection, frame.copy())
        if enemy_detection is None:
            return
        enemyInCamera2D = enemy_detection['enemyInCamera2D']

        fx = 225.20511562
        fy = 225.04733124
        cx = 283.42090143
        cy = 271.26381353
        u = enemyInCamera2D[0]
        v = enemyInCamera2D[1]
        depth = robot_detection['cameraFromRobot'].t[2]
        if depth > 0:  # Valid depth
            X = (u - cx) * depth / fx
            Y = (v - cy) * depth / fy
            Z = depth
            print(f"3D Point: [{X}, {Y}, {Z}]")
        enemyInCamera3D = np.array([X,Y,Z])
        enemyInWorld3D = self.cameraFromWorld.inv() * enemyInCamera3D

        # Prepare PoseStamped message with arena coordinates
        enemy_msg = PoseStamped()
        enemy_msg.header.frame_id = "map"
        enemy_msg.header.stamp = self.get_clock().now().to_msg()
        enemy_msg.pose.position.x = float(enemyInWorld3D[0])
        enemy_msg.pose.position.y = float(-enemyInWorld3D[1])
        # enemy_msg.pose.position.z = float(robot_detection['worldFromRobot'].t[2])
        enemy_msg.pose.position.z = float(0)

        # Convert arena yaw to quaternion
        qx, qy, qz, qw = quaternion_from_vectors(np.array([1, 0, 0]), np.array([1, 0, 0]))
        enemy_msg.pose.orientation.x = float(qx)
        enemy_msg.pose.orientation.y = float(qy)
        enemy_msg.pose.orientation.z = float(qz)
        enemy_msg.pose.orientation.w = float(qw)
        self.enemy_pub.publish(enemy_msg)

        # Draw and debug (using image coordinates for visualization)
        arena_size_m = 3
        metersToPixels = 100
        robot_arena = np.zeros(np.array([metersToPixels * arena_size_m, metersToPixels * arena_size_m, 3]).astype(int))
        robotInWorld2D = (metersToPixels * robot_detection['worldFromRobot'].t[0:2]).astype(int).squeeze()
        robotXInWorld2D = (metersToPixels * robotXInWorld[0:2]).astype(int).squeeze()
        enemyInWorld2D = (metersToPixels * enemyInWorld3D[0:2]).astype(int).squeeze()
        print(f'robotInWorld2D: {robotInWorld2D}, robotXInWorld2D: {robotXInWorld2D}')
        cv2.circle(robot_arena, robotInWorld2D, 5, (0, 0, 255), -1)
        cv2.arrowedLine(robot_arena, robotInWorld2D, robotXInWorld2D, (0, 255, 0), 5)
        cv2.circle(robot_arena, enemyInWorld2D, 5, (255, 0, 0), -1)

        cv2.putText(robot_arena, f"Yaw: {robotXInWorld2D} rad", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(robot_arena, f"Arena Pos: {robotInWorld2D}", (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("ROS DATA", robot_arena)


        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Exiting...")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()