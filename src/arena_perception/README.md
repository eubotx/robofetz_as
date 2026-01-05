# robofetz_2025

ros2 run camera_calibration cameracalibrator   --size=8x5   --square=0.024   --no-service-check   --fisheye-k-coefficients=4   --ros-args   -r image:=/arena_camera/image_raw   -p camera:=/arena_camera


ros2 run camera_calibration cameracalibrator   --size=8x5   --square=0.0254   --no-service-check --ros-args   -r image:=/arena_camera/image_raw   -p camera:=/arena_camera
