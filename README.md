# ROBOFETZ_AS - Autonomous Battle Bot System Based on ROS2 Jazzy

Welcome to **ROBOFETZ_AS**, an autonomous system designed for a battle bot competition built on **ROS2 Jazzy**. This project aims to control a robot for competitive scenarios, utilizing ROS2, a flexible robotics middleware.

### Prerequisites
To run this repository, you'll need a system running **Ubuntu 24.04 LTS** as either a native installation or via **WSL2** (Windows Subsystem for Linux). Note that when running on WSL2, some hardware-related functionalities (such as interfacing with the ESP32, gamepads, or cameras) may be limited.

---

## Table of Contents
1. [Installation Instructions](#installation-instructions)
2. [Rebuilding Packages](#rebuilding-packages)
3. [Running the Robot](#running-the-robot)
   - [Real Robot](#real-robot)
   - [Simulation](#simulation)
4. [Useful Commands](#useful-commands)
5. [Notes](#notes)

---

## Installation Instructions

### 1. Clone the Repository
Start by cloning this repository to your local machine:

```bash
git clone https://github.com/eubotx/robofetz_as.git
cd robofetz_as
git submodule update --init --recursive
```

### 2. Install ROS 2 Jazzy

To install **ROS 2 Jazzy**, follow the official installation guide:

```bash
sudo apt update
```

Then, follow the steps outlined on the [ROS 2 Jazzy Installation Page](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

Once installed, add ROS 2 to your shell's environment variables to ensure it gets sourced automatically:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

For further setup details, check the [ROS 2 Environment Configuration Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#add-sourcing-to-your-shell-startup-script).

### 3. Install Additional ROS 2 Packages

Install the required ROS 2 packages:

```bash
sudo apt update
sudo apt-get install ros-jazzy-teleop-twist-keyboard
sudo apt-get install ros-jazzy-teleop-twist-joy
sudo apt install ros-jazzy-rqt*
```

### 4. Install Gazebo for Simulation

Gazebo is essential for simulating your robot in a virtual environment. Install it and the necessary packages:

```bash
sudo apt update
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

Check if Gazebo is correctly installed by running:

```bash
which gz            # Should show the location of gz
gz sim              # Should start the Gazebo simulation environment
```

Next, install additional Gazebo-related packages:

```bash
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-xacro
```

Verify that Gazebo-related packages are installed:

```bash
ros2 pkg list | grep gz
```

### 5. Final Setup

Restart your terminal application.

---

## Rebuilding Packages

After the initial setup, code change or if you pull updates from the repository, you may need to rebuild the packages.

### Steps:
1. Navigate to your workspace:

   ```bash
   cd robofetz_as
   ```

2. **Always source the environment** before rebuilding:

   ```bash
   source install/local_setup.bash
   ```

3. Build the packages:

   ```bash
   colcon build
   ```

---

## Running the Robot

### Real Robot (Remote Controlled)


1. **Open a new terminal tab** and navigate to the repository:

   ```bash
   cd robofetz_as
   ```

2. **Source the environment**:

   ```bash
   source install/local_setup.bash
   ```

3. **Launch the teleoperation control**:

   - For Xbox controller:

     ```bash
     ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
     ```

   - If you need to adjust robot limits, use a custom configuration:

     ```bash
     ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' config_filepath:=config/bot135smallWheel_teleop.config.yaml
     ```

   - If you don’t have a gamepad, you can use the graphical interface with:

     ```bash
     rqt_robot_steering
     ```

4. **Open another terminal tab** and navigate to the repository: 

   ```bash
   cd robofetz_as
   ```

5. **Source the environment**:

   ```bash
   source install/local_setup.bash
   ```

6. **Run the micro-ROS agent** by specifying the ESP32’s IP address::

   ```
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 --dev 192.168.8.135
   ```

    Replace `192.168.8.135` with the actual IP of your ESP32.
   


### Simulation (Remote Controlled)

1. **Open a new terminal tab** and navigate to the repository:

   ```bash
   cd robofetz_as
   ```

2. **Source the environment**:

   ```bash
   source install/local_setup.bash
   ```

3. **Launch the simulation in Gazebo**:

   ```bash
   ros2 launch robofetz_gazebo  gazebo_bot_with_opponent.launch.py world:=robofetz_arena_pinhole.world
   ```

4. **Open another terminal tab** and launch the teleoperation control (same as for the real robot):

   - For Xbox controller:

     ```bash
     cd robofetz_as
     source install/local_setup.bash
     ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox'
     ```

   - Use a custom configuration if needed:

     ```bash
     cd robofetz_as
     source install/local_setup.bash
     ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' config_filepath:=config/bot135smallWheel_teleop.config.yaml
     ```

   - If you don’t have a gamepad, use the graphical interface:

     ```bash
     rqt_robot_steering
     ```

---

## Useful Commands

Here are some helpful ROS 2 commands for interacting with the robot and troubleshooting:

- List all active topics:

  ```bash
  ros2 topic list
  ```

- View messages from a specific topic:

  ```bash
  ros2 topic echo /your_topic
  ```

- Launch `rqt` for graphical tools:

  ```bash
  rqt
  ```

- Launch `rqt image viewer` to view image streams:

  ```bash
  ros2 run rqt_image_view rqt_image_view
  ```

- Launch `rviz2` to do various things

  ```bash
  rviz2
  ```

- Building selective packages with symlink so you can make changes  without recompiling

  ```bash
  colcon build --packages-select your_package --symlink-install
  ```

- Cleaning compiled files

  ```bash
  rm -rf build install log
  ```

  
---

## Notes

- **Remember:** Every time you open a new terminal tab, you **must** source the environment by running:

  ```bash
  source install/local_setup.bash
  ```

- Ensure you have the necessary permissions to run these commands on your system.
- If you encounter any issues, refer to the [ROS 2 documentation](https://docs.ros.org/en/jazzy/) or explore relevant troubleshooting guides.

---
