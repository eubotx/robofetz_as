#!/bin/bash
set -eo pipefail
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=91
for pkg in arena_perception combat_strategizer robot_navigation robot_bringup \
           robot_description robofetz_gazebo slider_publisher; do
    ros2 pkg prefix "$pkg" >/dev/null
done
test -x "$(ros2 pkg prefix slider_publisher)/lib/slider_publisher/slider_publisher"
test -r /usr/share/novnc/vnc.html
python3 - <<'PY'
import cv2
import numpy as np
from cv_bridge import CvBridge
from filterpy.kalman import KalmanFilter
from pyapriltags import Detector
from python_qt_binding import QtWidgets
from arena_perception import detection_kalman_filter
from robot_navigation import elf_navigator

image = np.zeros((64, 64), dtype=np.uint8)
assert CvBridge().cv2_to_imgmsg(image, encoding="mono8").width == 64
Detector(families="tagStandard41h12").detect(image)
PY
gz sim --versions
ros2 launch robot_bringup robofetz_as.launch.py --show-args >/dev/null
"$(ros2 pkg prefix robot_bringup)/lib/robot_bringup/cmd_vel_muxer" &
mux=$!
trap 'kill -TERM "$mux" 2>/dev/null || true; wait "$mux" 2>/dev/null || true' EXIT
timeout 30 ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --once
echo 'Smoke test passed: installed packages, AprilTag, Gazebo CLI and mux output.'
