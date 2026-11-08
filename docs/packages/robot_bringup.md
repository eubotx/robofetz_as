# robot_bringup

[Package index](README.md) · [System architecture](../architecture.md)

Starts the simulation or hardware stack. Provides `robot_cmd_vel_muxer`, the serial radio
bridge, and an optional keyboard driver for the simulated opponent.

## Nodes

### robot_cmd_vel_muxer

The `robot_cmd_vel_muxer` executable and node share the same name.
`cmd_vel_muxer` is a command alias. The node publishes `/cmd_vel` at 20 Hz.
A command on `/cmd_vel_teleop`,
including zero velocity, takes priority for 0.5 seconds by default.
Afterward, the node repeats the last `/cmd_vel_autonomous` message until a new
one arrives. It has no autonomous command timeout, so a one-shot wall
recovery command can persist if the strategizer stops before sending
`STOP`. The `bottom` marker on `/arena_perception/robot/visible_marker`
reverses linear x by default. A `none` marker preserves the last known
orientation.
[Node source](../../src/robot_bringup/robot_bringup/cmd_vel_muxer.py)

In simulation, `ros2 node info /robot_cmd_vel_muxer` should show both velocity
inputs and the `/cmd_vel` publisher.

#### Proposed improvement

- Expire autonomous commands and make timed reverse renewable.

### radio_bridge_node

[Source: RadioBridgeNode in transmitter.py](../../src/robot_bringup/robot_bringup/transmitter.py). Selected only by hardware bringup. Opens serial immediately during construction: `serial_port=/dev/ttyUSB0`, `baud_rate=115200`, read timeout 0.1 s. Requires Python serial support.

| Input topic | Type | Serial output |
| --- | --- | --- |
| /cmd_vel | geometry_msgs/Twist | `CMD:linear.x,angular.z\n` |
| /weapon/speed | std_msgs/Float32 | `WPN:data\n` |
| /drive/pid_tuning | std_msgs/Float64MultiArray | `PID:p,i,d,f\n` from first four elements; fewer than four ignored. |

All subscriptions use depth 10. Numbers are formatted to two decimal places. Drive fields are m/s and rad/s; weapon is the firmware's normalized request. No range/finite checks, ownership, arming, acknowledgement, resend timer or command expiry are implemented. A daemon receive thread logs text from ESP32; it publishes **no ROS telemetry or odometry**. Serial opening/writes are not recovered automatically. The radio firmware consumes CMD/WPN but does not implement the transmitted PID command; see [firmware](../firmware.md).

**Improve:** Validate serial fields and expose connection state.

### enemy_teleop_node

[Source: EnemyTeleop](../../src/robot_bringup/robot_bringup/keyboard_node.py). Executable and node `enemy_teleop_node`, with `keyboard_node` as a command alias. Publishes `geometry_msgs/Twist` on `/opponent/cmd_vel`, depth 10. A nominal 0.1 s timer polls terminal keys and republishes the retained command: arrows/WASD select ±0.5 m/s or ±1 rad/s, space stops, Q quits. Terminal reads can block for 0.1 s each, so the timer is not a strict output-rate guarantee. Shutdown attempts a zero command. No parameters, subscriptions, services or actions. Uses POSIX termios/tty and is not started by the default simulation launch.

**Improve:** Expire keyboard commands when input stops.

## Launched external components

| Launch/config | Interface and role |
| --- | --- |
| [xbox_teleop.launch.py](../../src/robot_bringup/launch/xbox_teleop.launch.py) | Starts `joy_node` and `teleop_twist_joy_node`. `teleop_twist_joy_node` publishes Twist messages on `/cmd_vel_teleop` for `robot_cmd_vel_muxer`. |
| [teleop_opponent.launch.py](../../src/robot_bringup/launch/teleop_opponent.launch.py) | Starts `joy_node` and `teleop_twist_joy_node` with the velocity output remapped to `/opponent/cmd_vel` for the Gazebo opponent. |
| [xbox_teleop.config.yaml](../../src/robot_bringup/config/xbox_teleop.config.yaml) | Linear x axis 1, yaw axis 0; normal/turbo scales both 1.2 m/s and 5 rad/s; enable button 2, turbo button 5. |
| [config.rviz](../../src/robot_bringup/config/config.rviz) | Optional RViz visualization configuration; not a control node. |

**Improve:** Make teleop selection explicit, publish operator enable and
ownership state, and add a headless option that also disables the perception
GUI.

## Package wiring

[robofetz_as.launch.py](../../src/robot_bringup/launch/robofetz_as.launch.py) is the system entry point. It orchestrates packages rather than providing its own control algorithm.

| Argument | Default / effect |
| --- | --- |
| use_sim | true; Gazebo versus serial radio + USB camera + robot Xbox teleop |
| use_fake_perception | false. Uses Gazebo ground-truth relays in place of camera perception. Requires `use_sim:=true`. |
| world | robofetz_arena_pinhole.world; also allows robofetz_arena_wideangle.world |
| launch_rviz | true; only controls RViz |
| rviz_config | config.rviz |
| arena_perception_config | arena_perception/config/arena_perception_config.yaml |
| robot_localization_config | arena_perception/config/robot_localization_config.yaml |

Bringup always starts `robot_state_publisher` with frame prefix `robot/` and
`robot_cmd_vel_muxer`. Simulation starts Gazebo and configures
`teleop_twist_joy_node` for `/opponent/cmd_vel`. Hardware starts
`radio_bridge_node` and `/arena_camera/usb_cam`, and configures
`teleop_twist_joy_node` for `/cmd_vel_teleop`. Simulation can use either
camera-based or fake perception. Localization runs in both modes.
`elf_autonomy.launch.py` starts `elf_combat_strategizer` and `elf_navigator`,
with autonomy initially disabled.

Root bringup validates the perception mode, then starts the selected
subsystems together. The Gazebo spawners wait for their inputs. Perception and
localization wait for the camera data and transforms they need to publish
poses. RViz can show the graph as those inputs become available.

```mermaid
flowchart TD
  Root[robofetz_as.launch.py] --> Description["robot_state_publisher"]
  Root --> Perception["arena_perception.launch.py or fake_arena_perception.launch.py"]
  Root --> Localization["robot_localization.launch.py"]
  Root --> Elf["elf_autonomy.launch.py"]
  Root --> Mux["robot_cmd_vel_muxer"]
  Root --> Sim["robofetz_gazebo_sim.launch.py and teleop_opponent.launch.py"]
  Root --> HW["arena_camera_usb.launch.py, radio_bridge_node, and xbox_teleop.launch.py"]
```

## Package improvements

- Provide explicit strategy, teleop, camera and headless profiles; propagate serial/camera settings and clocks.
- Declare the remaining runtime dependencies in `package.xml` and add
  launch-composition validation without opening hardware.
