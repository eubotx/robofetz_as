# slider_publisher

Third-party Qt node. Bringup launches it as `velocity` to publish `/weapon/speed` (Float32) at 10 Hz. The slider starts at 0 and ranges from 0 to 1. The radio bridge forwards each value to the robot. [Launch](../../src/robot_bringup/launch/weaponspeed_slider.launch) · [Slider config](../../src/robot_bringup/config/Float32.yaml)

## Node interfaces

| Input or mode | Behavior |
| --- | --- |
| Config | YAML file from the first CLI argument or `config` parameter. Each top-level key names a topic or service. |
| Topic mode (`package/msg/Type`) | Publishes every timer tick at `rate` (default 10 Hz), depth 10. Conventional stamp fields receive node time. |
| Service mode (`package/srv/Type`) | Waits for the server during startup. Sends an asynchronous request when its value changes and ignores the response. |
| Controls | Numeric slider, checkbox or text mapped to message fields using `to`. A single-field message maps automatically. Reset restores configured defaults. |

Closing the GUI stops publication without sending zero. **Improve:** Report service failures and bound the startup wait. [Pinned node source](https://github.com/oKermorgant/slider_publisher/blob/6c6099c012b166b5cb21ee17163a9cc4f0743c53/src/slider_publisher)

## Package wiring

Qt widgets hold control values. A ROS timer samples them and updates one publisher or service client per YAML entry. The weapon configuration creates one topic publisher. The default radio path accepts its output directly.

**Package improvements**

- Route the slider through the robot’s weapon arming and expiry policy.
- Make the GUI optional for headless simulation.
