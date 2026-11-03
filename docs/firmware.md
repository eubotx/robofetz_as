# ESP32 firmware

Default hardware path: ROS topics → [radio_bridge_node](packages/robot_bringup.md) → USB serial (115200 baud) → PC-side ESP32 → HC-12 (9600 baud) → robot ESP32. The sketches live outside the colcon workspace packages.

## Radio pair

[transciever_PC_side](../micro_ros_esp32/transciever_PC_side/transciever_PC_side.ino) forwards newline-delimited USB commands to HC-12 UART2 (RX 22, TX 21, SET 15). It prints outgoing and incoming lines to USB. The host logs these lines. **Improve:** Separate acknowledgements from logs and bound the wait for partial lines.

[diff_drive_bot_esp32_funk](../micro_ros_esp32/diff_drive_bot_esp32_funk/diff_drive_bot_esp32_funk.ino) applies commands on the robot:

| Serial line | Robot behavior |
| --- | --- |
| `CMD:linear,angular` | Sets linear speed in m/s and yaw rate in rad/s. |
| `WPN:speed` | Clamps the weapon request to [0,1] and maps it to ESC PWM. |
| `PID:p,i,d,f` | Host can send this line. The radio firmware ignores it. |

Either accepted command reapplies both retained setpoints. `RobotControl` converts v and ω to left/right wheel speeds using 0.069 m wheel diameter and 0.19 m separation. Each encoder feeds a motor PID through the gearbox conversion. The zero-speed sleep counter only advances once the requested speed is already zero. A broken radio link leaves the last request active.

**Improve:** Give drive and weapon separate receive deadlines. Reject malformed fields before applying commands.

## Direct micro-ROS alternative

[diff_drive_bot_esp32](../micro_ros_esp32/diff_drive_bot_esp32/diff_drive_bot_esp32.ino) connects node `diff_drive_bot_esp32` to a micro-ROS agent over Wi-Fi. It replaces the radio path.

| Direction | Topic | Type and payload |
| --- | --- | --- |
| In | `cmd_vel` | Twist, linear.x and angular.z |
| In | `weapon/speed` | Float32, normalized speed |
| In | `drive/pid_tuning` | Float64MultiArray, exactly [Kp, Ki, Kd, KiMax] |
| Out, 10 Hz best effort | `joint_states` | JointState, left/right wheel position (rad), velocity (rad/s), PWM duty in effort. Stamp uses MCU uptime. |
| Optional, 10 Hz best effort | `drive/left/debug`, `drive/right/debug` | Float64MultiArray, desired motor RPS, measured RPS, error, duty |
| Optional, 10 Hz best effort | `looptime_debug` | Float64MultiArray, median/minimum/maximum loop time (µs) |

**Improve:** Synchronize timestamps and keep executor waits outside the actuator update loop. The executor can wait up to 1 s before the next drive update.

## Bench variants

| Sketch | Interface and purpose | Improvement |
| --- | --- | --- |
| [receiver_PC_dummy](../micro_ros_esp32/receiver_PC_dummy/receiver_PC_dummy.ino) | Parses CMD/WPN from radio and prints values over USB. Accepts USB lines for transmission. Use in place of the actuating receiver. | Share parser cases with the robot firmware. |
| [pid_tuning_helper](../micro_ros_esp32/pid_tuning_helper/pid_tuning_helper.ino) | Runs one motor with a 100 RPS amplitude, 0.4 Hz sine target. Prints desired/measured values every 10 ms over USB at 115200 baud. | Set explicit run duration and reuse the production PID implementation. |

## Hardware improvements

- Share actuator logic across both robot sketches and reconcile wheel geometry with the URDF.
- Advertise supported protocol commands so the host can detect unsupported PID tuning.
- Report wheel feedback and link health upstream.
