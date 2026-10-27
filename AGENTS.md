# Working in ROBOFETZ_AS

## Scope and source of truth

This is a ROS 2 Jazzy workspace for an externally observed differential-drive
combat robot. A fixed arena camera localizes the robot and tracks an opponent;
host-side strategy and navigation produce commands for Gazebo or ESP32 hardware.

Start with `src/robot_bringup/launch/robofetz_as.launch.py`, then follow the launch
files it actually includes. The default autonomy is **Elf**, not Nav2. Distinguish
default runtime, optional alternatives, and experimental files in every change.
See [docs/architecture.md](docs/architecture.md) for diagrams, interfaces, and the
source-based assessment. README commands and comments can lag behind the code.

## Workspace map

| Location | Responsibility and starting points |
| --- | --- |
| `src/robot_bringup` | System composition, USB camera/hardware selection, teleop, weapon slider; `robot_bringup/transmitter.py` exposes `radio_bridge_node`. |
| `src/arena_perception` | Camera calibration, AprilTag self-localization, ROI masking, color/MOG2 opponent detection, plane projection, Kalman tracking, TF-based localization. Start with `launch/arena_perception.launch.py` and `launch/robot_localization.launch.py`. |
| `src/combat_strategizer` | Elf state machine, velocity arbitration, optional simple/Nav2 strategies, weapon controller, pose adapters. |
| `src/robot_navigation` | Elf and simple velocity controllers, optional Nav2 action wrapper/configuration, map generation and defense-map experiments. |
| `src/robot_description` | Robot URDF/xacro, tag mounting geometry, Gazebo plugins, robot state publisher. |
| `src/robofetz_gazebo` | Worlds, opponent model, robot spawning, camera and ground-truth TF bridges. |
| `micro_ros_esp32` | Arduino firmware variants: HC-12 radio drive, USB/radio bridge, direct micro-ROS drive, and tuning tools. These are outside the colcon build. |
| `src/slider_publisher` | Third-party submodule used by the weapon slider. |

There are six first-party ament Python packages. `first_package` is not present.
`.gitmodules` also mentions micro-ROS repositories that are not checked-in gitlink
entries in this revision; inspect the checkout before assuming they are available.
Treat third-party submodules and tag image assets as external dependencies.

## Default runtime

1. Bringup starts the robot state publisher, command mux, and weapon slider.
   Simulation selects Gazebo and opponent teleop; hardware selects USB camera,
   the serial radio bridge, and robot Xbox teleop.
2. Real perception rectifies images, detects AprilTags, calibrates
   `world -> arena_camera_optical`, and derives the robot footprint from the
   detected top/bottom tag and URDF mounting transforms.
3. Opponent perception masks the arena, defines a 2x decimation stage, runs color
   and MOG2 detectors, projects image detections onto the world ground plane, and
   fuses them into the `opponent` TF frame. The current image/calibration wiring
   is inconsistent; see the architecture assessment before modifying this path.
4. Localization publishes identity `world -> map`, camera-based `map -> odom`,
   and a temporary identity `odom -> robot/base_footprint`. This is not an active
   wheel/IMU EKF. TF adapters publish `/robot/pose` and `/opponent/pose`.
5. After a fixed 30-second delay, `elf_autonomy.launch.py` starts
   `elf_combat_strategizer` and `elf_navigator`. Autonomy initially remains off.
6. Elf emits goals, drive mode, and immediate commands. Its controller publishes
   `/cmd_vel_autonomous`; the mux selects recent teleop or cached autonomy,
   applies bottom-marker inversion, and publishes `/cmd_vel`.

Startup uses fixed timers, not readiness guarantees. Fake perception replaces
camera processing with ground-truth TF relays and pose adapters; localization
and control remain in the loop. The default launch starts neither Nav2 nor
`map_node` nor `weapon_control`.

## Interfaces to preserve

| Interface | Type | Producer → consumer |
| --- | --- | --- |
| `/arena_camera/image`, `/arena_camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` | USB/Gazebo camera → image processing |
| `/robot/pose` | `geometry_msgs/PoseStamped` | TF adapter, in `world` → Elf strategy/controller |
| `/opponent/pose` | `geometry_msgs/PoseStamped` | TF adapter, in `world` → Elf strategy |
| `/autonomy` | `std_msgs/Bool` | Operator → Elf strategy; defaults false |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Strategy → controller |
| `/drive_mode` | `std_msgs/String` | Elf strategy → Elf controller: `FORWARD` / `BACKWARD` |
| `/elf_navigator_cmd` | `std_msgs/String` | Elf strategy → controller, e.g. `STOP`, `BACKWARD:0.5` |
| `/cmd_vel_autonomous`, `/cmd_vel_teleop` | `geometry_msgs/Twist` | Controllers / teleop → mux |
| `/arena_perception/robot/visible_marker` | `std_msgs/String` | Robot detection → mux: `top`, `bottom`, `none` |
| `/cmd_vel` | `geometry_msgs/Twist` | Mux → Gazebo bridge or radio bridge |
| `/weapon/speed` | `std_msgs/Float32` | Default slider → radio bridge; separate from drive arbitration |

`world` and `map` currently coincide. Elf copies attack goals in `world`, creates
defense goals in `map`, and its controller does not transform between frames.
Changing the identity relationship requires updating these consumers.
An opponent's filter-derived orientation estimates motion direction, not an
independently measured chassis heading.

Hardware transport sends newline-delimited `CMD:linear,angular`, `WPN:speed`, and
`PID:p,i,d,f` text through USB serial to the HC-12 bridge. The checked-in radio
robot firmware parses CMD/WPN, but not PID. Do not assume the active transport is
micro-ROS merely because of the firmware directory name.

## Elf behavior and alternative stacks

Elf runs at 20 Hz with `IDLE`, `ATTACK`, `DEFENSE`, and `WALLRECOVERY` states.
Autonomy off or missing/timed-out robot-pose reception leads to IDLE/STOP.
Defaults: continuous proximity within 0.2 m for 5 s triggers defense; defense
selects the farthest of four positions around arena center and drives backward
until within 0.1 m. Facing outward within 0.2 m of a wall triggers timed reverse
recovery (1 m / 0.5 m/s), then ATTACK. Recovery currently has no turn phase.

Optional paths are separate compositions:

- `rhino_autonomy.launch.py`: `simple_attack` forwards opponent poses to goals;
  `simple_navigator` follows them; `weapon_control` is also started.
- `robot_navigation/launch/nav2.launch.py`: Nav2 servers, map server, lifecycle
  manager, and velocity relay. `nav2_attack` is started separately and still uses
  `/arena_perception_*_base_footprint_pose` topics. Fake TF relays publish these
  legacy topics; the default camera pipeline does not.
- `map_node`: publishes `/map_defence`, `/map_debug_image`, `/defense_position`;
  it is not part of Elf. The Nav2 strategy's defense subscription is commented out.
- `conditions/` defines `update(**kwargs)`, `is_triggered()`, `reset()`. However,
  `nav2_attack.py` shadows the imported proximity condition with a local
  `check(robot_pose, opponent_pose)` implementation. Inspect the actual caller
  before extending conditions; there is not one unified condition API in use.

Do not run competing strategy/controller stacks together without explicitly
routing their command outputs. The optional Nav2 path writes directly to
`/cmd_vel` and bypasses the default mux.

## Development and validation

Use Python 3 and ROS 2 Jazzy APIs. Follow nearby style and use type hints where
helpful. Do not add code comments unless requested. Keep work focused; preserve
unrelated user edits. Edit source packages, not generated `build/`, `install/`,
or `log/` content. Update launch files, YAML, entry points, and dependency
manifests together when changing a runtime interface.

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon list
colcon build --symlink-install
source install/local_setup.bash
```

For a focused change, use `colcon build --packages-select <package> --symlink-install`
when its dependencies are already built, or `--packages-up-to <package>` for
declared workspace dependencies. Some dependency manifests are incomplete.

```bash
ament_flake8 src/<package>
ament_pep257 src/<package>
colcon test --packages-select <package> --event-handlers console_direct+
colcon test-result --verbose
```

The first-party test suites currently contain linter scaffolding, not behavioral
coverage; copyright tests are skipped. Report pre-existing failures separately.
For behavior changes, test state transitions, stale data, frame handling, and
command ownership at the changed boundary. Validate launch argument propagation
and installed entry points as well as Python syntax. Do not equate a successful
build with a functioning ROS graph.

Simulation commands (the launch package is `robot_bringup`, not `robofetz_as`):

```bash
ros2 launch robot_bringup robofetz_as.launch.py use_sim:=true
ros2 launch robot_bringup robofetz_as.launch.py use_sim:=true use_fake_perception:=true
```

Hardware entry point: `ros2 launch robot_bringup robofetz_as.launch.py use_sim:=false`.
It opens serial/camera devices and launches the weapon slider. Do not start it,
enable autonomy, or command actuators merely to validate a documentation change.

## Known constraints when changing the system

- The mux expires teleop after 0.5 s but does not expire autonomous commands.
  Adding expiry also requires changing Elf's one-shot timed-reverse command to
  a renewable command or an explicitly bounded maneuver.
- `tf_to_pose` stamps cached transforms with current time; Elf checks robot
  reception time and has no opponent freshness timeout. Pose reception is not
  evidence of a fresh camera observation.
- Elf uses wall time for strategy timing. Elf launch and the fusion launch do
  not consistently pass `use_sim_time` to their nodes. `run_rectification` is
  passed by bringup but is not consumed by the perception launch.
- The radio firmware retains last drive/weapon setpoints without a receive
  watchdog. `/weapon/armed` belongs to the optional controller, not a universal
  gate on the slider/serial path.
- Several perception entry points reference absent modules; check `setup.py`
  rather than assuming every listed executable is usable.

These are existing limitations, not desired contracts. Update this guidance and
the architecture document when fixes change the behavior.
