# ROBOFETZ_AS - An Autonomous System for Battle Bot Tournaments / Combat Robotics

**ROBOFETZ_AS** is an autonomous system based on ROS2 (Jazzy) to make battle bots actual autonomous robots. It provides robot perception, robot localization, combat strategy infrastructure, robot navigation, robot controls, and all required ROS2 infrastructure including a full simulation environment.

The system can run:

* Fully in simulation (Gazebo)
* On real hardware (camera + ESP32 controlled robot)

---

# Installation Instructions

## Prerequisites

* Ubuntu 24.04 LTS (native installation recommended)
  *When running on WSL2, hardware-related features (camera, ESP32, gamepads) may be limited.*

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
sudo apt install python3-pydantic
sudo apt install ros-${ROS_DISTRO}-camera-calibration
sudo apt install ros-${ROS_DISTRO}-tf-transformations
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-topic-tools
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
ros2 launch robofetz_as robofetz_as.launch.py
```

---

# Launch Arguments

Launch arguments can be overridden like this:

```bash
ros2 launch robofetz_as robofetz_as.launch.py use_sim:=false
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
ros2 launch robofetz_as robofetz_as.launch.py
```

Simulation with wide-angle world (currently not supported):

```bash
ros2 launch robofetz_as robofetz_as.launch.py \
use_sim:=true \
world:=robofetz_arena_wideangle.world
```

Simulation with fake perception:

```bash
ros2 launch robofetz_as robofetz_as.launch.py \
use_sim:=true \
use_fake_perception:=true
```

Real robot mode:

```bash
ros2 launch robofetz_as robofetz_as.launch.py \
use_sim:=false
```

---

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

Arm weapon:

```bash
ros2 topic pub /weapon/armed std_msgs/Bool "{data: true}"
ros2 topic pub /weapon/armed std_msgs/Bool "{data: false}"
```

Check simulation time usage:

```bash
for node in $(ros2 node list); do
  echo "=== $node ==="
  ros2 param get $node use_sim_time 2>/dev/null || echo "No use_sim_time param"
done
```

---

# Notes

* Every new terminal must run:

```bash
source install/local_setup.bash
```
