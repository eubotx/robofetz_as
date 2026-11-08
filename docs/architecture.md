# System architecture

A fixed arena camera estimates the robot and opponent poses. Elf uses those poses to choose a maneuver. Its controller sends wheel-motion requests through `robot_cmd_vel_muxer` to Gazebo or the radio bridge. [Root launch](../src/robot_bringup/launch/robofetz_as.launch.py) · [Package guides](packages/README.md)

## How the system runs

| Mode | Perception | Drive |
| --- | --- | --- |
| Default simulation | Gazebo pinhole camera → AprilTags for robot pose, color and motion for opponent pose. | Elf drives the robot via `/cmd_vel`. The operator drives the simulated opponent with Xbox teleop via `/opponent/cmd_vel` to test the robot’s response. |
| Fake perception (`use_fake_perception:=true` in simulation) | Ground-truth TF replaces the camera pipeline. Robot localization and both pose adapters remain active. | Same Elf controller and `robot_cmd_vel_muxer`. Fake perception supplies no top/bottom marker for flip detection. |
| Hardware (`use_sim:=false`) | USB camera → same camera pipeline as simulation. | Robot Xbox teleop or Elf → `robot_cmd_vel_muxer` → USB serial → HC-12 → ESP32 wheel controllers. Weapon slider → radio → weapon PWM. [Firmware](firmware.md) |

Root bringup starts perception after 8 s in simulation or 3 s on hardware, localization after 20/15 s and Elf after 30 s. These delays do not check readiness. Autonomy starts disabled.

## Observation and command paths

```mermaid
flowchart LR
  Camera[Camera] --> Tags[AprilTags]
  Tags --> Self[Camera calibration and robot localization]
  Camera --> ROI[Arena mask]
  ROI --> Detectors[Color and MOG2]
  Detectors --> Proj[Two ground-plane projectors]
  Proj --> Track[Opponent Kalman filter]
  Self -->|robot pose| Strategy[Elf strategy]
  Track -->|opponent pose via TF adapter| Strategy
  Self -->|robot pose| Controller[Elf controller]
  Strategy -->|goal, mode, immediate command| Controller
  Controller -->|cmd_vel_autonomous| Mux["robot_cmd_vel_muxer"]
  Teleop[Robot teleop] -->|cmd_vel_teleop| Mux
  Tags -->|visible marker via robot detection| Mux
  Mux -->|cmd_vel| Drive[Gazebo or radio bridge]
```

| Interface | Producer → consumer | Meaning |
| --- | --- | --- |
| `/robot/pose`, `/opponent/pose` (PoseStamped) | TF adapters → strategy. Robot pose also feeds controller and detector self-masking. | World coordinates. Opponent heading follows motion direction. |
| `/autonomy` (Bool) | Operator → Elf | Enables strategy. |
| `/goal_pose` (PoseStamped), `/drive_mode`, `/elf_navigator_cmd` (String) | Strategy → controller | Attack copies the opponent goal in `world`. Defense sets a `map` goal. The controller assumes these frames coincide. |
| `/cmd_vel_autonomous`, `/cmd_vel_teleop` → `/cmd_vel` (Twist) | Controller or robot teleop → [robot_cmd_vel_muxer](packages/robot_bringup.md#robot_cmd_vel_muxer) → drive | Teleop takes priority for 0.5 s. Otherwise the node repeats the last autonomous command. A bottom tag reverses linear x. |
| `/weapon/speed` (Float32) | Weapon slider → radio | Normalized speed request [0,1]. |

## TF ownership

```mermaid
flowchart TD
    World["world"] -->|identity: localization launch| Map["map"]
    Map -->|camera correction: odom_drift_correction_node| Odom["odom"]
    Odom -->|temporary identity: localization launch| Base["robot/base_footprint"]
    Base -->|URDF: robot_state_publisher| Links["Robot links and tag mounts"]
    World -->|camera calibration| Camera["arena_camera_optical"]
    Camera -->|AprilTag detector| Tags["Detected tag frames"]
    Camera -->|robot_detection_node| Detected["arena_perception/robot/base_footprint"]
    World -->|Kalman tracker| Opponent["opponent"]
    World -. "Gazebo only" .-> GroundTruth["robot/base_footprint_sim and opponent/base_footprint_sim"]
```

The camera-based correction carries the robot’s motion in `map → odom` because `odom → robot/base_footprint` is identity. The default radio and Gazebo paths supply no ROS wheel odometry. In fake mode the opponent adapter reads `arena_perception/opponent/base_footprint`. The robot adapter reads `robot/base_footprint` in both modes. [Perception wiring](packages/arena_perception.md)

## Autonomy approaches

| Approach | Strategy | Motion |
| --- | --- | --- |
| [Original direct pursuit (Rhino)](../src/combat_strategizer/launch/simple_autonomy.launch.py) | Obsolete first iteration retained as a learning example. `simple_combat_strategizer` forwards opponent poses directly to `/goal_pose`. | `simple_navigator` drives toward the goal. |
| [Elf (default)](../src/combat_strategizer/launch/elf_autonomy.launch.py) | Pursues the opponent. Within 0.2 m for 5 s, selects a defense point and drives backward. Near a wall and facing outward, commands a timed reverse. | `elf_navigator` tracks the goal or executes immediate commands. `robot_cmd_vel_muxer` selects the drive source. |
| [Optional Nav2](packages/robot_navigation.md) | `nav2_attack` selects targets and defense points from legacy pose topics. | Aligns locally, then sends `NavigateToPose` to external Nav2 servers. Alignment and the smoothed-velocity relay publish directly to `/cmd_vel`. |

## System improvements

| Current behavior | Proposed change | Check |
| --- | --- | --- |
| Detectors use full-size masked pixels with half-resolution CameraInfo. | Use one matching image/calibration pair for self-masking and projection. | Project a known arena point at both resolutions. |
| TF adapters restamp cached poses. Elf checks robot reception time and retains the opponent indefinitely. | Carry measurement time and validity through perception and control. | Stop camera updates and verify that both poses expire. |
| `robot_cmd_vel_muxer` and the radio firmware retain commands. Elf wall recovery relies on a one-shot reverse. | Renew reverse commands for the maneuver duration. Expire drive and weapon requests independently at the actuator. | Interrupt the controller and the radio separately and measure time to zero. |
| The optional Nav2 path bypasses `robot_cmd_vel_muxer`. | Route each drive source through `robot_cmd_vel_muxer`. | Check drive publisher ownership and stop behavior in each profile. |
| Elf and fusion use mixed clocks. Fixed launch delays can elapse before camera or TF is ready. | Propagate simulation time and gate startup on camera, calibration and TF availability. | Pause Gazebo and repeat startup with a delayed camera. |
