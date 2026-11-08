# Installation and first run

[Project overview](../README.md)

Choose one setup path. The Dev Container and Docker Compose run the same Gazebo
simulation with dependencies supplied by the container image. Native Ubuntu
installs those dependencies on the host and also supports physical hardware.

- [VS Code Dev Container](#vs-code-dev-container) for editing, building, and testing.
- [Standalone Docker Compose](#standalone-docker-compose) for running a packaged simulator.
- [Native Ubuntu 24.04](#native-ubuntu-2404) for host development and better hardware access.

## Get the source

Install Git and clone the repository from an Ubuntu terminal. If you already
have a checkout, open that directory and continue with your chosen setup.

```bash
git clone https://github.com/eubotx/robofetz_as.git
cd robofetz_as
```

On Windows, clone inside the Ubuntu 24.04 WSL 2 filesystem for the container
workflows. The AprilTag image submodule is only needed for marker assets used
by the tag-printing tools. Fetch it with `git submodule update --init --recursive`.

## Container prerequisites

On Ubuntu 24.04, install Docker Engine. On Windows, enable Docker Desktop's
Linux engine and WSL integration for Ubuntu 24.04. Check that `docker info`
works in your Ubuntu terminal.
[Docker Engine on Ubuntu](https://docs.docker.com/engine/install/ubuntu/) ·
[Docker Desktop WSL integration](https://docs.docker.com/desktop/features/wsl/)

## VS Code Dev Container

Install VS Code and its **Dev Containers** extension. On Windows, also install
the **WSL** extension. From the checkout, run `code .`, then follow the
Dev Container guide to build and run the simulator.
[Dev Container guide](../docker/DEVCONTAINER.md) ·
[Optional Xbox controller setup](../docker/DEVCONTAINER.md#xbox-controller-passthrough)

## Standalone Docker Compose

Check that `docker compose version` works in your Ubuntu terminal. With Docker
Engine on Ubuntu, install the Compose plugin if needed. Then start the
simulator from the checkout. [Docker Compose guide](../docker/README.md)

## Native Ubuntu 24.04

Run these steps in Bash on Ubuntu 24.04 LTS. Use a graphical desktop session
for Gazebo, RViz, and the perception color selector.

### 1. Install ROS 2 Jazzy and workspace dependencies

Follow the Jazzy Ubuntu instructions through ROS repository setup and install
the **ROS Base** package (`ros-jazzy-ros-base`). The package list below adds
the tools and runtime components used by this workspace. It follows the
project's Docker dependency list, with an explicit list because the package
manifests currently omit some runtime dependencies and include legacy Gazebo
entries. [ROS 2 Jazzy installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) ·
[Project Dockerfile](../docker/Dockerfile)

```bash
sudo apt update
sudo apt install \
  build-essential cmake python3-colcon-common-extensions python3-pip python3-wheel \
  python3-opencv python3-numpy python3-scipy python3-matplotlib python3-pil \
  python3-yaml python3-bitstring python3-serial \
  ros-jazzy-ros-gz ros-jazzy-rviz2 ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  ros-jazzy-joy ros-jazzy-teleop-twist-joy \
  ros-jazzy-image-proc ros-jazzy-cv-bridge ros-jazzy-vision-msgs \
  ros-jazzy-tf-transformations ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-usb-cam ros-jazzy-camera-ros ros-jazzy-topic-tools \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup
```

Install the two Python libraries with the versions used by the container.
The separate target directory and `--no-deps` keep NumPy and OpenCV on the
Ubuntu versions used by ROS's `cv_bridge`.

```bash
/usr/bin/python3 -m pip install --upgrade --no-deps \
  --target "$HOME/.local/share/robofetz-as/python" \
  filterpy==1.4.5 pyapriltags==3.4.3.1
```

### 2. Build and load the workspace

From the repository root, load Jazzy, build the packages, and load the
workspace overlay so ROS can find them.

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="$HOME/.local/share/robofetz-as/python${PYTHONPATH:+:$PYTHONPATH}"
colcon build --symlink-install
source install/local_setup.bash
ros2 pkg prefix robot_bringup
```

The last command should print a path ending in `/install/robot_bringup`.
In each new Bash terminal, repeat the two `source` commands and the
`PYTHONPATH` export. Rerun the build after pulling updates or changing
package metadata, dependencies, or installed assets.

### 3. Start and verify the simulation

```bash
ros2 launch robot_bringup robofetz_as.launch.py use_sim:=true
```

Gazebo should open the pinhole arena and RViz should open the project's
visualization. Select the simulated opponent in the color selector or press
**q** in that window to use its defaults. Elf starts with autonomy disabled.

Open a second Bash terminal with the same environment and run:

```bash
ros2 topic echo /clock --once
ros2 topic echo /arena_camera/image --once --field header
ros2 topic echo /robot/pose --once
```

Expect a simulation clock message, an image header with a frame and timestamp,
and a robot pose once localization has the transforms it needs. To steer the
simulated opponent, run `ros2 run robot_bringup enemy_teleop_node` and use WASD
or the arrow keys. Press Space to stop the opponent and Q to quit teleop.
Press Ctrl+C in the launch terminal to stop the simulation.

For a comparison with Gazebo ground truth, stop the first launch and restart
with `use_sim:=true use_fake_perception:=true`. To see the available launch
arguments, run `ros2 launch robot_bringup robofetz_as.launch.py --show-args`.
Use the pinhole world for the first run. The launch also lists the older
wide-angle world, which retains legacy Gazebo sensor plugin names.
[Launch argument reference](packages/robot_bringup.md#package-wiring) ·
[Gazebo worlds](packages/robofetz_gazebo.md)

### 4. Bring up physical hardware

Use native Ubuntu for the camera, serial radio adapter, and Xbox controller.
Before launching, check the camera device and calibration and confirm serial
access. The default camera and radio devices are `/dev/video0` and
`/dev/ttyUSB0`, respectively. For camera inspection and calibration, install
`v4l-utils` and `ros-jazzy-camera-calibration`.
[Arena camera setup](../src/arena_perception/Arena_Camera_Setup.md) ·
[Radio bridge](packages/robot_bringup.md#radio_bridge_node) ·
[ESP32 firmware](firmware.md)

Once the hardware is ready for testing, switch the launch to hardware mode.
This selects the USB camera, serial radio bridge, and robot gamepad teleop.

```bash
ros2 launch robot_bringup robofetz_as.launch.py use_sim:=false
```
