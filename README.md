# ROBOFETZ_AS - An Autonomous System for Battle Bot Toournaments / Combat Robotics

**ROBOFETZ_AS** is an autonomous system based on ROS2 (Jazzy) to make battle bots actual autonomous robots. It provides robot perception, robot localization, combat strategy,  robot navigation and robot controls as well as all the needed ROS2 infrastructure and a simulation environment. To run the real thing outside of simulation you will need a network, a camera and a robot controlled by an esp32.

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

### Prerequisites
To run this repository, you'll need a system running **Ubuntu 24.04 LTS** as either a native installation or via **WSL2** (Windows Subsystem for Linux). Note that when running on WSL2, some hardware-related functionalities (such as interfacing with the ESP32, gamepads, or cameras) may be limited. Docker setup is on the roadmap.

### 1. Clone the Repository
Start by cloning this repository and all its submodules to your local machine:

```bash
git clone https://github.com/eubotx/robofetz_as.git
cd robofetz_as
git submodule update --init --recursive
```

### 2. Install ROS 2 Jazzy

Follow the steps outlined on the [ROS 2 Jazzy Installation Page](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

Once installed, add ROS 2 to your shell's environment variables to ensure it gets sourced automatically:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 3. Install Additional ROS 2 Packages

Install the required ROS 2 packages:

```bash
sudo apt update
sudo apt-get install ros-jazzy-teleop-twist-keyboard
sudo apt-get install ros-jazzy-teleop-twist-joy
sudo apt install ros-jazzy-rqt*
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-xacro
sudo apt install ros-${ROS_DISTRO}-usb-cam
sudo apt install python3-pydantic
ros2 run rqt_image_view rqt_image_view
sudo apt install ros-${ROS_DISTRO}-camera-calibration
sudo apt-get install ros-${ROS_DISTRO}-tf-transformations
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

### 4. Install Gazebo for Simulation

The simulation environment is based on Gazebo. Install it and the necessary packages:

```bash
sudo apt update
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

Check if Gazebo is correctly installed by running:

```bash
which gz            # Should show the location of gz
gz sim              # Should start the Gazebo simulation environment
ros2 pkg list | grep gz # Verify that Gazebo-related packages
```

### 5. Source ROS2

Open new terminal window or run

```bash
source /opt/ros/jazzy/setup.bash
```

---

## Rebuilding Packages

After installation, code change or if you pull updates from the repository, you may need to rebuild the packages.

### Steps:
1. Navigate to your workspace:

   ```bash
   cd robofetz_as
   ```

2. Build the packages:

   ```bash
   colcon build
   ```

3. **Always source the environment** after rebuilding:

   ```bash
   source install/local_setup.bash
   ```

---

## Running the Robot

### Real Robot (Remote Controlled)


1. **Navigate to the repository** if you're not already in it:

   ```bash
   cd robofetz_as
   ```

2. **Source the environment**:

   ```bash
   source install/local_setup.bash
   ```

3. **Launch robofetz_as**:

    ```bash
   ros2 launch robofetz_as.launch.py
   ```


<table>
  <tr>
    <th>argument</th>
    <th>options</th>
    <th>explanation</th>
    <th>condition</th>
    <th>note</th>
  </tr>

  <tr>
    <td rowspan="3">--world</td>
    <td>file.txt</td>
    <td>Pfad zur Eingabedatei</td>
    <td>muss existieren</td>
    <td>required</td>
  </tr>

  <tr>
    <td>stdin</td>
    <td>Liest Daten von der Standard‑Eingabe</td>
    <td>nur wenn kein file angegeben</td>
    <td>optional</td>
  </tr>

  <tr>
    <td>--verbose</td>
    <td>true / false</td>
    <td>gibt mehr Log‑Ausgaben aus</td>
    <td>keine</td>
    <td>optional</td>
  </tr>
</table>

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