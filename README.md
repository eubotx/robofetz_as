# ROBOFETZ_AS - An Autonomous System for Battle Bot Tournaments / Combat Robotics

**ROBOFETZ_AS** is a hands-on learning project for combat robot autonomy using
ROS 2 Jazzy. It brings together perception, localization, strategy, navigation,
and robot control.

[Package guides](docs/packages/README.md) · [System architecture](docs/architecture.md) · [Contributor guide](AGENTS.md)

[VS Code Dev Container](docker/DEVCONTAINER.md) · [Standalone Docker simulation](docker/README.md)

The system can run:

* Fully in simulation (Gazebo)
* On real hardware (camera + ESP32 controlled robot)

## Learning goals

The project keeps earlier implementations and alternative node variants so you
can compare how they solve the same problem, what each approach assumes, and
where its trade-offs show up. The [autonomy guide](docs/autonomy.md)
compares Elf's default state-machine strategy with the original direct pursuit.
The [perception guide](docs/packages/arena_perception.md#package-wiring) shows how
color-only and motion-only opponent detection differ from the fused pipeline.

When an AI coding agent helps with a change, it should explain how the
affected parts fit together, why it chose the approach, and how you can check
the result yourself. For behavior changes, Gazebo and ROS 2 inspection tools
let you follow the data and see how the robot responds.

## Robot Hardware

| Elfbar prototype | CAD model |
| --- | --- |
| ![Assembled Elfbar combat robot with its tracking marker on the top cover](docs/images/elfbar.jpg) | ![CAD rendering of the Elfbar combat robot](docs/images/elfbar-cad.png) |

The assembled Elfbar robot and its CAD model.

---

# Installation Instructions

## Prerequisites

* Ubuntu 24.04 LTS (native installation recommended)
  *This project does not support Xbox controllers on WSL 2. The tested WSL kernel
  lacks the `xpad` driver. Use [keyboard or rqt steering in simulation](docker/README.md#drive-the-simulated-opponent).
  Camera and ESP32 access may also be limited.*

---

## 1. Clone the Repository

```bash
git clone https://github.com/eubotx/robofetz_as.git
cd robofetz_as
git submodule update --init --recursive
```

---

## 2. Install ROS 2 Jazzy

Follow the official installation guide:

[https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

Add ROS 2 to your shell:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 3. Install Required ROS 2 Packages

```bash
sudo apt update
sudo apt install ros-jazzy-teleop-twist-keyboard
sudo apt install ros-jazzy-teleop-twist-joy
sudo apt install ros-jazzy-rqt*
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-xacro
sudo apt install ros-${ROS_DISTRO}-usb-cam
sudo apt install ros-jazzy-camera-ros
sudo apt install python3-pydantic
sudo apt install ros-${ROS_DISTRO}-camera-calibration
sudo apt install ros-${ROS_DISTRO}-tf-transformations
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-topic-tools
sudo apt install v4l-utils
sudo apt install ros-jazzy-apriltag-ros
python3 -m pip install filterpy
pyapriltags
```

---

## 4. Install Gazebo

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-ros-gz
```

---

# Rebuilding Packages

After pulling updates or modifying code:

```bash
cd robofetz_as
colcon build
source install/local_setup.bash
```

---

# Running the Robot

```bash
cd robofetz_as
source install/local_setup.bash
ros2 launch robot_bringup robofetz_as.launch.py
```

---

# Launch Arguments

Launch arguments can be overridden like this:

```bash
ros2 launch robot_bringup robofetz_as.launch.py use_sim:=true
```

---

## Available Arguments

| Argument                  | Description                      | Default                      | Options                                                       | Notes                                                                     |
| ------------------------- | -------------------------------- | ---------------------------- | ------------------------------------------------------------- | ------------------------------------------------------------------------- |
| use_sim                   | Launch Gazebo simulation         | true                         | true / false                                                  | false = real robot mode                                                   |
| use_fake_perception       | Use fake arena perception        | false                        | true / false                                                  | Only relevant when use_sim=true. Wideangle world currently not supported. |
| world                     | Gazebo world file                | robofetz_arena_pinhole.world | robofetz_arena_wideangle.world / robofetz_arena_pinhole.world | Only used when use_sim=true                                               |
| launch_rviz               | Launch RViz2                     | true                         | true / false                                                  | Visualization toggle                                                      |
| rviz_config               | RViz config file                 | config.rviz                  | any .rviz file                                                | Must exist in robot_bringup/config/                                       |
| arena_perception_config   | Arena perception configuration   | package default              | path to YAML file                                             | Must be valid YAML                                                        |
| robot_localization_config | Robot localization configuration | package default              | path to YAML file                                             | Must be valid YAML                                                        |

---

# Example Launch Scenarios

Full simulation (default):

```bash
ros2 launch robot_bringup robofetz_as.launch.py
```

Simulation with wide-angle world (currently not supported):

```bash
ros2 launch robot_bringup robofetz_as.launch.py \
use_sim:=true \
world:=robofetz_arena_wideangle.world
```

Simulation with fake perception:

```bash
ros2 launch robot_bringup robofetz_as.launch.py \
use_sim:=true \
use_fake_perception:=true
```

Real robot mode:

```bash
ros2 launch robot_bringup robofetz_as.launch.py \
use_sim:=false
```

---


# Create Map

To create a custom map you can adjust the parameters in `src/robot_navigation/map/my_map.yaml` and execute:

```bash
ros2 run robot_navigation create_map
```

After this you can run `map_node` that maps the enemy position to the map and publishes the defence map.

```bash
ros2 run robot_navigation map_node
```

# Useful Commands

List active topics:

```bash
ros2 topic list
```

Echo topic messages:

```bash
ros2 topic echo /your_topic
```

Launch rqt:

```bash
rqt
```

Launch image viewer:

```bash
ros2 run rqt_image_view rqt_image_view
```

Launch RViz manually:

```bash
rviz2
```

Selective build with symlink install:

```bash
colcon build --packages-select your_package --symlink-install
```

Clean workspace:

```bash
rm -rf build install log
```

Check simulation time usage:

```bash
for node in $(ros2 node list); do
  echo "=== $node ==="
  ros2 param get $node use_sim_time 2>/dev/null || echo "No use_sim_time param"
done
```

Run the `tf_to_pose` executable with a standalone output topic. In standard
bringup, `robot_tf_to_pose` publishes `/robot/pose`.

```bash
ros2 run combat_strategizer tf_to_pose \
  --ros-args \
  -p source_frame:=robot/base_footprint \
  -p reference_frame:=world \
  -p pose_topic:=/robot_pose
```

| Parameter       | Description                  | Default | Required |
|-----------------|------------------------------|---------|----------|
| source_frame    | TF frame to publish as pose  | -       | yes      |
| reference_frame | Reference frame for lookup   | world   | no       |
| pose_topic      | Output PoseStamped topic     | pose    | no       |
| rate            | Publishing rate (Hz)         | 60.0    | no       |

---

# Notes

* Every new terminal must run:

```bash
source install/local_setup.bash
```
