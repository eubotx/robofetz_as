# Run the simulation in Docker

From the repository root, run the same commands in Ubuntu 24.04 or the Ubuntu 24.04 WSL2 terminal. Use Docker Engine with the Compose plugin on Linux. On Windows, start Docker Desktop in Linux container mode and enable **Settings → Resources → WSL Integration → Ubuntu-24.04**. Verify connectivity with `docker info`. [Ubuntu installation](https://docs.docker.com/engine/install/ubuntu/) · [WSL integration](https://docs.docker.com/desktop/features/wsl/)

```bash
docker compose -f docker/compose.yaml up --build
```

Open [the simulation desktop](http://localhost:6080/vnc.html?autoconnect=1&resize=scale). Gazebo, RViz, the weapon slider and the color selector share this window. Click the opponent in the color selector or press **q** there to accept the defaults. Elf starts after the launch delay with autonomy disabled.

The image includes ROS 2 Jazzy, Gazebo Harmonic, the six project packages and the pinned slider submodule. The first build needs internet and runs a headless smoke test. Rebuild after source changes. The desktop uses software rendering, so simulation speed depends on CPU capacity. [Gazebo compatibility](https://gazebosim.org/docs/harmonic/ros_installation/)

## Try autonomy in simulation

From a second terminal in the same checkout:

```bash
docker compose -f docker/compose.yaml exec sim ros-env ros2 topic echo /robot/pose --once
docker compose -f docker/compose.yaml exec sim ros-env ros2 topic pub --once /autonomy std_msgs/msg/Bool '{data: true}'
docker compose -f docker/compose.yaml exec sim ros-env ros2 run robot_bringup keyboard_node
```

Use WASD or arrows in that terminal to move the **opponent**, Space to stop and Q to quit. This works on both hosts without USB passthrough. The default profile exposes no host devices.

## Other commands

```bash
# Stop the running simulation.
docker compose -f docker/compose.yaml down

# Run Gazebo ground-truth perception instead of the camera detectors.
docker compose -f docker/compose.yaml run --rm --service-ports sim robofetz-sim use_fake_perception:=true

# Run the headless check independently.
docker compose -f docker/compose.yaml run --rm sim robofetz-smoke
```

Keep the desktop port bound to localhost. The viewer has no password. The smoke test checks ROS package discovery, AprilTag and cv_bridge, launch arguments and an actual /cmd_vel message from the mux. To check the full simulator after startup, run `docker compose -f docker/compose.yaml exec sim ros-env ros2 topic hz /arena_camera/image`.
