# Run the simulation in Docker

Use standalone Docker Compose to run a packaged snapshot of the simulator. Use
the [VS Code Dev Container](DEVCONTAINER.md) when you want to edit source, build
packages incrementally, or run package tests. Both use the same
ROS 2 Jazzy and Gazebo dependencies from the [Dockerfile](Dockerfile).

## Open the right terminal

Run Compose commands from the repository root in your **Ubuntu host terminal**,
outside the Dev Container. On Windows, start Docker Desktop in Linux container
mode and enable **Settings → Resources → WSL Integration → Ubuntu-24.04**.
Then open that distribution and enter the WSL checkout:

```bash
cd ~/projects/robofetz_as
docker info
docker compose version
```

On native Ubuntu 24.04, install Docker Engine with the Compose plugin, then run
the same Docker commands from your Linux checkout. The image supplies ROS;
you do not need to install ROS on the host for this workflow.
[Ubuntu installation](https://docs.docker.com/engine/install/ubuntu/) ·
[Docker Desktop WSL integration](https://docs.docker.com/desktop/features/wsl/)

## Build and start the simulator

Stop any running ROBOFETZ_AS Dev Container first: both workflows reserve
localhost port 6080, even when the Dev Container's simulation task is idle.

```bash
docker compose -f docker/compose.yaml up --build
```

The first build downloads dependencies, builds the six project packages and
the pinned `slider_publisher`, and runs a headless smoke check. Compose starts
the `sim` service from the resulting `robofetz-as:jazzy` image and streams its
logs. Source is copied into the image, so run `up --build` again after editing
it. Docker can reuse unchanged dependency layers. To start the already-built
image in the background, use `up -d` in place of `up --build`.

Open [the simulation desktop](http://localhost:6080/vnc.html?autoconnect=1&resize=scale)
in your host browser. Gazebo simulates the arena and sensors; RViz visualizes
ROS data. Both run on the container's virtual desktop, alongside the weapon
slider and color selector. Click the simulated opponent in the color selector,
or press **q** in that window to use its default settings. The launch starts
Elf after a delay, with autonomy disabled. The desktop uses software rendering,
so simulation speed depends on CPU capacity.

## Inspect the running simulation

Open a second **host** terminal in the same checkout. `exec sim` runs a command
inside the existing service container; `ros-env` loads Jazzy and the installed
workspace for that command.

```bash
docker compose -f docker/compose.yaml ps
docker compose -f docker/compose.yaml exec sim ros-env ros2 node list
docker compose -f docker/compose.yaml exec sim ros-env ros2 topic echo /arena_camera/image --once --field header
docker compose -f docker/compose.yaml exec sim ros-env ros2 topic echo /robot/pose --once
```

An image header checks that the camera is publishing and shows its frame and
timestamp. A pose message checks that localization is producing an estimate.
If an echo waits during startup, watch the logs and press Ctrl+C to cancel the
echo if needed. To inspect transforms or run several commands without the
Compose prefix, open a shell with the ROS environment loaded:

```bash
docker compose -f docker/compose.yaml exec sim ros-env bash
ros2 run tf2_ros tf2_echo world robot/base_footprint
```

Press Ctrl+C to stop `tf2_echo`, then type `exit` to leave that shell.

## Try Elf against a moving opponent

For the running **sim** service, wait for Elf to start, then run these from the
host checkout:

```bash
docker compose -f docker/compose.yaml exec sim ros-env ros2 topic pub --once /autonomy std_msgs/msg/Bool '{data: true}'
docker compose -f docker/compose.yaml exec sim ros-env ros2 run robot_bringup keyboard_node
```

Focus the keyboard node's terminal. Use WASD or the arrow keys to move the
**simulated opponent**, Space to stop it, and Q to quit the keyboard node.
To disable Elf in the simulation, publish the same message with
`'{data: false}'`. The Compose service passes through no host devices.

## Stop, restart or run a variant

In the terminal running foreground Compose, Ctrl+C stops the service. Use
`down` from the host checkout to remove the stopped service container and its
Compose network. The built image remains available for the next launch.

```bash
docker compose -f docker/compose.yaml down
```

To diagnose camera tracking, stop the normal service and launch this one-off
variant. It uses simulated ground truth instead of the camera detectors, so you
can inspect motion without depending on image detection:

```bash
docker compose -f docker/compose.yaml run --rm --service-ports sim robofetz-sim use_fake_perception:=true
```

Ctrl+C stops the variant; `--rm` removes its container. The `exec sim`
examples above target the service started with `up`.

## Smoke check and troubleshooting

To verify the packaged image without launching Gazebo's desktop, build the
image and run:

```bash
docker compose -f docker/compose.yaml build sim
docker compose -f docker/compose.yaml run --rm sim robofetz-smoke
```

This checks ROS package discovery, selected Python imports, AprilTag,
`cv_bridge`, Gazebo's CLI, launch argument parsing and receipt of a
`/cmd_vel` message. It does not exercise the full simulated robot.

| Symptom | What to check |
| --- | --- |
| Cannot reach Docker | Run `docker info` in the host terminal. On Windows, check Docker Desktop and integration for the Ubuntu distribution you opened. |
| Port 6080 is already in use | Check `docker ps` on the host. Stop the competing container, including any idle ROBOFETZ_AS Dev Container, before restarting Compose. |
| Browser cannot connect or a node exits | Check `docker compose -f docker/compose.yaml ps -a` and `docker compose -f docker/compose.yaml logs --tail=100 sim`. |
| Simulator still runs older code | Check `git status --short --branch` in the host checkout you built from, then run `up --build`. |

The noVNC viewer has no password and is bound to loopback in
[`compose.yaml`](compose.yaml). Keep that binding for local use. See the
[validation record](20261105_Devcontainer_Validation.md) for what was verified
on WSL and the observed RViz issue.
