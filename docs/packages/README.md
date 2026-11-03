# Package guides

[System architecture](../architecture.md) · [Firmware and radio protocol](../firmware.md)

| Package | Responsibility |
| --- | --- |
| [arena_perception](arena_perception.md) | Camera calibration, robot localization and opponent tracking |
| [combat_strategizer](combat_strategizer.md) | Strategy, pose adapters, drive mux and weapon policy |
| [robot_navigation](robot_navigation.md) | Reactive controllers, Nav2 client and map tools |
| [robot_description](robot_description.md) | Robot geometry, tag mounts and Gazebo plugins |
| [robofetz_gazebo](robofetz_gazebo.md) | Simulation worlds, spawning and ROS bridges |
| [robot_bringup](robot_bringup.md) | System launch, teleop and serial radio bridge |
| [slider_publisher](slider_publisher.md) | Third-party GUI used for weapon speed |

Source revision `b4c2542` on `rework_before_new_season`. Of 38 first-party console entries, 32 have a matching module and `main`. The six broken entries are listed in the [perception guide](arena_perception.md). Runtime behavior was reviewed from source.
