# arena_perception

The camera pipeline locates the robot from AprilTags and tracks the opponent
from color and motion. `robot_tf_to_pose` publishes `/robot/pose` and
`opponent_tf_to_pose` publishes `/opponent/pose`. Both poses use the `world`
frame.

## AprilTags and shared camera calibration

### [apriltag_detection_node](../../src/arena_perception/arena_perception/apriltag_detection_node.py)

Reads `/arena_camera/image_rect` (Image) and `/arena_camera/camera_info` (CameraInfo). Broadcasts each visible tag under `arena_camera_optical`. Optional debug Image on `/arena_perception/apriltag_detection/debug`. Image endpoints use best effort, depth 1.

| Tag ID | TF child | Detected edge |
| ---: | --- | ---: |
| 11 | arena_perception/robot_top_tag | 0.0522 m |
| 13 | arena_perception/robot_bottom_tag | 0.0522 m |
| 2 | arena_tag | 0.0833 m |

Configured family: `tagStandard41h12`. The detector uses the first CameraInfo P matrix and stamps transforms at processing time. [Tag configuration](../../src/arena_perception/config/arena_perception_config.yaml)

**Improve:** Preserve the image stamp and detect the family once per image before estimating pose by tag size.

### [find_camera_in_world_service](../../src/arena_perception/arena_perception/find_camera_in_world_service.py)

`/find_camera_in_world` (`std_srvs/srv/Trigger`) takes an empty request and returns `success` and `message`. Each call samples the detected arena tag. Ten samples determine camera pose from the known marker pose. Automatic calibration attempts run at 2 Hz.

- **TF in:** observed `arena_camera_optical → arena_tag`.
- **TF out:** static `world → arena_tag_calibrated` at (0.15, 0.15, 0) m with RPY (0, π, π/2). Dynamic `world → arena_camera_optical` at 15 Hz.
- **Consumers:** `odom_drift_correction_node` uses the camera pose to express the detected footprint in `world`. `roi_mask_node`, `opponent_det_ColorSingle`, and `opponent_det_MOG2single` use it to project the arena boundary or robot mask into the image. `detection_2d_to_3d_color` and `detection_2d_to_3d_mog2` use it to project opponent pixels onto the arena floor.
- **File:** `calibration_file` can supply `camera_in_world.position` (xyz) and `orientation` (RPY). The configured `world_to_camera_calibration.temp.yaml` is absent. The node reads calibration but never saves it.

**Improve:** Count distinct, recent samples. Report how many remain so callers can distinguish progress from a missing tag.

## Robot localization

### [robot_detection_node](../../src/arena_perception/arena_perception/robot_detection_node.py)

At 15 Hz, combines the newest top or bottom tag detection (maximum age 0.2 s) with its URDF tag-to-footprint mount. Broadcasts `arena_camera_optical → arena_perception/robot/base_footprint`. Publishes `top`, `bottom` or `none` on `/arena_perception/robot/visible_marker` (String) only when the value changes. A missing marker stops new footprint TF.

**Improve:** Carry the detection stamp into TF and deliver the current marker state to late subscribers.

### [odom_drift_correction_node](../../src/arena_perception/arena_perception/odom_drift_correction_node.py)

Publishes `map → odom` at 15 Hz from three transforms:

`T_map_odom = inverse(T_world_map) × T_world_detected_base × inverse(T_odom_base)`

`static_map_tf_publisher` supplies identity `world → map` and
`static_odom_tf_publisher` supplies identity `odom → robot/base_footprint`.
Robot motion therefore appears in the correction. The detected footprint must
be newer than 0.1 s. [Localization config](../../src/arena_perception/config/robot_localization_config.yaml)

**Improve:** Compute corrections at the observation time and report when detection expires.

## Opponent detection

### [roi_mask_node](../../src/arena_perception/arena_perception/roi_mask_node.py)

Reads `/arena_camera/image_rect` (Image), `/arena_camera/camera_info` (CameraInfo) and camera TF. Projects the arena bounds (x/y −0.1 to 1.6 m, z=0) and blacks out pixels outside them. Publishes `/arena_camera/image_rect_masked` (Image) with the input header. Image and CameraInfo use reliable QoS, depth 1. Missing calibration, TF or a valid polygon causes full-image passthrough.

**Improve:** Indicate whether the mask was applied so downstream detectors can distinguish passthrough frames.

### Color and motion detectors

Both read `/arena_camera/image_rect_masked` (Image), `/arena_camera/cropped/camera_info` (CameraInfo), `/robot/pose` (PoseStamped) and camera TF. Each masks the robot, selects the largest matching contour, and publishes a Detection2DArray containing zero or one box. Detections use processing time and a fixed score of 1.0.

| Node | Selection | Detection2DArray / debug Image | Improvement |
| --- | --- | --- | --- |
| [opponent_det_ColorSingle](../../src/arena_perception/arena_perception/opponent_det_ColorSingle.py) | Click to select an HSV color, then accept contour areas 200–8000 px². | `/detections_2d/color` / `/debug/color_point` | Load a saved HSV profile for headless runs. |
| [opponent_det_MOG2single](../../src/arena_perception/arena_perception/opponent_det_MOG2single.py) | MOG2 foreground, shadows rejected, area 300–10000 px², aspect ratio 0.3–3.0. | `/detections_2d/mog2` / `/debug/mog2_point` | Test how long a stationary opponent remains detectable. |

### detection_2d_to_3d_color and detection_2d_to_3d_mog2

Both nodes run the `detection_transformation_2D_3D_node` executable. They
project each box center onto `world` z=0 using the first CameraInfo K matrix
and latest camera TF. Scores below 0.5 are discarded.
[Node source](../../src/arena_perception/arena_perception/detection_transformation_2D_3D_node.py)

| Interface | Type | `detection_2d_to_3d_color` | `detection_2d_to_3d_mog2` |
| --- | --- | --- | --- |
| In | sensor_msgs/CameraInfo, best effort depth 10 | `/arena_camera/cropped/camera_info` | `/arena_camera/cropped/camera_info` |
| In | vision_msgs/Detection2DArray | `/detections_2d/color` | `/detections_2d/mog2` |
| Out | vision_msgs/Detection3DArray | `/detections_3d/color_array` | `/detections_3d/mog2_array` |
| Debug out | geometry_msgs/PointStamped | `/detections_3d/color_point` | `/detections_3d/mog2_point` |

Position is in `bbox.center`. Size is unset. Empty projections produce no array. Each Detection3D retains its input stamp, while the array is stamped at publication.

**Improve:** Project rectified pixels with matching intrinsics and look up TF at the image stamp.

### [detection_kalman_filter](../../src/arena_perception/arena_perception/detection_kalman_filter.py)

Fuses `/detections_3d/color_array` and `/detections_3d/mog2_array` (Detection3DArray) into TF `world → opponent` at 20 Hz. Tracks one position and velocity. The nearest detection within 1.5 m updates the track. Measurement noises are 0.08 for color and 0.04 for motion. Heading follows velocity and holds at low speed. After 1.5 s without a measurement the filter stops broadcasting. [Fusion settings](../../src/arena_perception/config/opponent_detection_config.yaml)

**Improve:** Update at measurement time and expose track age so consumers can reject cached TF.

## Optional nodes and examples

The detector variants below share `/arena_camera/image_rect` (Image), `/arena_camera/camera_info` (CameraInfo), `/robot/pose` (PoseStamped) and camera TF, except the incomplete image-only prototype.

| Detector | Behavior and outputs | Improvement |
| --- | --- | --- |
| [frame_diff_detector](../../src/arena_perception/arena_perception/frame_diff_detector.py) | MOG2 prototype. Publishes `/detections_2d` (Detection2DArray) and `/debug/detection_image` (Image). | Share self-masking with the active MOG2 detector. |
| [opponent_tracker_node](../../src/arena_perception/arena_perception/opponent_tracker_node.py) | Node `frame_diff_oponent_detector`. Acquires with MOG2, tracks with KCF or CSRT, predicts through loss for 1 s. Outputs `/detected_robot/pose` (world PoseStamped) and `/debug/detection_image` (Image). | Replace fixed 0.033 s prediction steps with elapsed image time. |
| [opponent_det_ContourSingle.py](../../src/arena_perception/arena_perception/opponent_det_ContourSingle.py) (source only) | Mouse-selected box tracked by CSRT or KCF. Outputs `/detections_2d` (Detection2DArray) and `/debug/tracker_image` (Image). Can repeat a lost box with score 0.5. | Mark held boxes with their observation age and apply the computed self-mask to tracker input. |
| [opponent_detection.py](../../src/arena_perception/arena_perception/opponent_detection.py) (source only) | Node `frame_diff_oponent_detector`. Projects MOG2 contours directly onto z=0. Outputs `/detected_robot/pose` (world PoseStamped) and `/debug/detection_image` (Image). | Enforce the configured maximum contour area. |
| [FrameDiffRobotDetector.py](../../src/arena_perception/arena_perception/FrameDiffRobotDetector.py) (incomplete) | Node `opponent_detection_node`. Reads `/arena_camera/image_rect`. Median background subtraction targets `/detected_robot/pose` and optional `/debug/detection_image`. Entry function and inverse projection are unfinished. | Repair entry, projection and debug indexing before running the example. |

| Adapter or helper | Interface | Improvement |
| --- | --- | --- |
| [tf_frame_relay](../../src/arena_perception/arena_perception/tf_frame_relay.py) | In fake perception, `robot_tf_relay` and `opponent_tf_relay` copy TF at 60 Hz and publish `/arena_perception_robot_base_footprint_pose` and `/arena_perception_opponent_base_footprint_pose`, respectively. | Preserve source stamps. |
| [filter_transform_node](../../src/arena_perception/arena_perception/filter_transform_node.py) | Filters `parent → input_frame` into `parent → output_frame`. Requires both frame parameters and `config_file`. Supplied YAML selects median at 30 Hz. | Only filter new samples. Report fallback to raw TF. |
| [image_format_converter](../../src/arena_perception/arena_perception/image_format_converter.py) | Converts YUYV from `/image` to `/image_rgb` when run with default parameters at the root namespace. Other arrays can be relabeled without color conversion. | Validate encoding conversions. |
| [param_loader](../../src/arena_perception/arena_perception/param_loader.py) | Python helper to read nested YAML and declare ROS parameters. | Expose it as a library in setup.py. |
| [minimal_tf_test](../../src/arena_perception/arena_perception/minimal_tf_test.py) | Generates `world → opponent_0/1/2` on circles at 20 Hz. Creates `/fused_tracks` and `/filtered_poses` publishers but sends no messages. | Isolate synthetic TF in a test namespace. |
| [static_tf_test](../../src/arena_perception/arena_perception/static_tf_test'.py) | Broadcasts `world → static_test` at (2, 1, 0.5). | Correct the filename to match its entry point. |
| [resize_apriltag.py](../../src/arena_perception/scripts/apriltags/resize_apriltag.py) | Offline CLI: input/output PNG paths, 2048×2048 nearest-neighbor tag texture. | Validate arguments and image writes. |

**Broken console entries:** `camera_rectification_node`, `static_tf_test`, `opponent_det_MOG2multiple`, `detection_transformation_2D_3D` and `detection_2d_to_3d_node` lack their declared module paths. `param_loader` lacks `main`. [setup.py](../../src/arena_perception/setup.py)

## Package wiring

| Launch path | Data flow |
| --- | --- |
| [Camera](../../src/arena_perception/launch/arena_camera_usb.launch.py) | `/arena_camera/usb_cam` supplies 1920×1200 images at 15 Hz. The [alternative camera_ros launch](../../src/arena_perception/launch/arena_camera_ros.launch.py) runs the `camera_node` executable at 800×600 MJPEG. Both supply `/arena_camera/image` and `/arena_camera/camera_info`. |
| [Robot localization](../../src/arena_perception/launch/arena_perception.launch.py) | `/arena_camera/rectify_node` publishes `/arena_camera/image_rect` for `apriltag_detection_node`. `find_camera_in_world_service`, `robot_detection_node`, and [`odom_drift_correction_node`](../../src/arena_perception/launch/robot_localization.launch.py) supply the TF chain that `robot_tf_to_pose` reads to publish `/robot/pose`. |
| [Opponent fusion](../../src/arena_perception/launch/opponent_Fusiondetection.launch.py) | `roi_mask_node` publishes `/arena_camera/image_rect_masked` for `opponent_det_ColorSingle` and `opponent_det_MOG2single`. Their 3D projections feed `detection_kalman_filter`. `opponent_tf_to_pose` converts its TF output to `/opponent/pose`. |
| [Color only](../../src/arena_perception/launch/opponent_detection_color.launch.py) / [MOG2 only](../../src/arena_perception/launch/opponent_detection_mog.launch.py) | `point_to_pose_remap` converts `/detections_3d/color_point` or `/detections_3d/mog2_point` to `/opponent/pose` with identity orientation. |
| [Fake perception](../../src/arena_perception/launch/fake_arena_perception.launch.py) | `robot_tf_relay` and `opponent_tf_relay` copy Gazebo ground-truth TF. `robot_tf_to_pose` still reads the localized robot footprint. `opponent_tf_to_pose` reads `arena_perception/opponent/base_footprint`. |

**Image geometry mismatch.** `/arena_camera/crop_decimate_node` publishes
`/arena_camera/cropped/image_rect_masked` and
`/arena_camera/cropped/camera_info` at half resolution. The launch remaps
`/arena_camera/image_rect`, while the detector YAML selects the full-size
`/arena_camera/image_rect_masked`. Both detectors and both 3D projection
nodes use `/arena_camera/cropped/camera_info`. This shifts the self-mask and
projected opponent position.

**Package improvements**

- Pass a matched image/CameraInfo pair to both detectors and projectors. Check a known arena point at each resolution.
- Carry the image stamp through detection, projection and fusion. Apply `use_sim_time` to all fusion nodes.
- Give each selectable perception path one publisher for `/opponent/pose`.
