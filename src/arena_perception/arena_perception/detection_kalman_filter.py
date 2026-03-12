#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from filterpy.kalman import KalmanFilter
import threading
import os

from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from .param_loader import ParamLoader


class TrackedObject:

    def __init__(self, position, time):

        self.last_prediction_time = time
        self.last_update_time = time

        self.missed_detections = 0

        pos = np.array(position)

        self.kf = KalmanFilter(dim_x=6, dim_z=3)

        self.kf.x = np.array([
            pos[0],
            pos[1],
            pos[2],
            0.0,
            0.0,
            0.0
        ]).reshape(6, 1)

        self.kf.F = np.eye(6)

        self.kf.H = np.array([
            [1,0,0,0,0,0],
            [0,1,0,0,0,0],
            [0,0,1,0,0,0]
        ])

        self.kf.P = np.eye(6)

        self.kf.P[0:3,0:3] *= 0.2
        self.kf.P[3:6,3:6] *= 50.0

        self.kf.R = np.eye(3) * 0.05

        self.pos_noise = 0.05
        self.vel_noise = 0.5


    def predict(self, now):

        dt = (now - self.last_prediction_time).nanoseconds / 1e9

        if dt <= 0:
            return

        dt = min(dt, 0.2)

        self.kf.F = np.array([
            [1,0,0,dt,0,0],
            [0,1,0,0,dt,0],
            [0,0,1,0,0,dt],
            [0,0,0,1,0,0],
            [0,0,0,0,1,0],
            [0,0,0,0,0,1]
        ])

        self.kf.Q = np.diag([
            self.pos_noise * dt,
            self.pos_noise * dt,
            self.pos_noise * dt,
            self.vel_noise * dt,
            self.vel_noise * dt,
            self.vel_noise * dt
        ])

        self.kf.predict()

        self.last_prediction_time = now

        self.missed_detections += 1


    def update(self, measurement, now, noise):

        self.kf.R = np.eye(3) * noise

        z = measurement.reshape(3,1)

        self.kf.update(z)

        self.last_update_time = now
        self.missed_detections = 0


    def state(self):

        pos = self.kf.x[0:3].flatten()
        vel = self.kf.x[3:6].flatten()

        return pos, vel


class SingleOpponentKalmanFilter(Node):

    def __init__(self):

        super().__init__('detection_kalman_filter')

        package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(package_dir, 'config', 'opponent_det_pipeline_config.yaml')

        self.param_loader = ParamLoader(self, config_path)

        self.detection_sources = self.param_loader.get_param(
            'opponent_det_pipeline',
            'detection_sources',
            default={}
        )

        self.kf_params = self.param_loader.get_param(
            'opponent_det_pipeline',
            'kalman_filter',
            default={}
        )

        self.used_sources = self.kf_params.get("used_sources", [])

        self.measurement_noise = self.kf_params.get("measurement_noise", {})

        self.max_gate = 1.5
        self.max_missed = 15

        self.track = None

        self.lock = threading.Lock()

        self.target_frame = "world"
        self.child_frame = "opponent_0"

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscribers = []

        for src in self.used_sources:

            if src not in self.detection_sources:
                continue

            topic = self.detection_sources[src].get("topic_3d")

            if not topic:
                continue

            def make_cb(name):
                return lambda msg: self.detection_callback(msg, name)

            self.subscribers.append(
                self.create_subscription(
                    Detection3DArray,
                    topic,
                    make_cb(src),
                    10
                )
            )

            self.get_logger().info(f"Subscribed to {topic}")

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Kalman filter node started")


    def velocity_to_quaternion(self, vel):

        vx, vy, vz = vel

        speed = np.sqrt(vx*vx + vy*vy)

        if speed < 0.01:
            return (0.0,0.0,0.0,1.0)

        yaw = np.arctan2(vy, vx)

        qz = float(np.sin(yaw/2.0))
        qw = float(np.cos(yaw/2.0))

        return (0.0,0.0,qz,qw)


    def extract_position(self, detection):

        try:

            c = detection.bbox.center

            if hasattr(c,"pose"):
                p = c.pose.position
            else:
                p = c.position

            return np.array([p.x,p.y,p.z])

        except:
            return None


    def detection_callback(self, msg, source):

        if not msg.detections:
            return

        now = self.get_clock().now()

        with self.lock:

            best = None
            best_dist = 1e9

            for det in msg.detections:

                m = self.extract_position(det)

                if m is None:
                    continue

                if self.track:

                    pos,_ = self.track.state()

                    dist = np.linalg.norm(m-pos)

                    if dist < self.max_gate and dist < best_dist:

                        best_dist = dist
                        best = m

                else:

                    best = m
                    break


            if best is None:
                return

            noise = self.measurement_noise.get(
                source,
                self.measurement_noise.get("default",0.05)
            )

            if self.track:

                self.track.update(best, now, noise)

            else:

                self.track = TrackedObject(best, now)

                self.get_logger().info("Created new track")


    def timer_callback(self):

        now = self.get_clock().now()

        with self.lock:

            if not self.track:
                return

            self.track.predict(now)

            pos,vel = self.track.state()

            qx,qy,qz,qw = self.velocity_to_quaternion(vel)

            t = TransformStamped()

            t.header.stamp = now.to_msg()
            t.header.frame_id = self.target_frame
            t.child_frame_id = self.child_frame

            t.transform.translation.x = float(pos[0])
            t.transform.translation.y = float(pos[1])
            t.transform.translation.z = float(pos[2])

            t.transform.rotation.x = float(qx)
            t.transform.rotation.y = float(qy)
            t.transform.rotation.z = float(qz)
            t.transform.rotation.w = float(qw)

            self.tf_broadcaster.sendTransform(t)


def main(args=None):

    rclpy.init(args=args)

    node = SingleOpponentKalmanFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()