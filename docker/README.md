# Run the simulator with Docker Compose

Compose builds a runnable image from your checkout. Rebuild it after editing
source. For an editable workspace, use the [Dev Container](DEVCONTAINER.md).
[Installation and host setup](../docs/installation.md#container-prerequisites)

## Start and stop

Start Compose from the repository root in the **Ubuntu host terminal**.

```bash
docker compose -f docker/compose.yaml up --build
```

When the logs report that the desktop is ready, open the viewer and click the
opponent in the color selector or press `q` to accept the defaults.
[Simulation desktop](http://localhost:6080/vnc.html?autoconnect=1&resize=scale)

Press Ctrl+C to stop the foreground service. Use `up -d` to run the existing
image in the background and `down` to remove the service container.

## Inspect and steer

From a second host terminal, open a shell in the running service. `ros-env`
loads Jazzy and the installed workspace.

```bash
docker compose -f docker/compose.yaml exec sim ros-env bash
ros2 topic echo /arena_camera/image --once --field header
ros2 topic echo /robot/pose --once
```

Expect an image frame and timestamp, followed by a pose once localization is
ready. Run `ros2 run robot_bringup enemy_teleop_node` in that shell to steer the
simulated opponent with WASD or the arrow keys. Space stops it and Q quits.
[Xbox passthrough in the Dev Container](DEVCONTAINER.md#xbox-controller-passthrough)

Elf starts with autonomy disabled. To try it against the simulated opponent,
run `ros2 topic pub --once /autonomy std_msgs/msg/Bool '{data: true}'` in the
container shell. Publish `'{data: false}'` to disable it.

## Diagnostics

- **Desktop unavailable**. Check `docker compose -f docker/compose.yaml logs --tail=100 sim`.
  Free localhost port 6080 if another service or VS Code forward is using it.
- **Image smoke check**. After building the image, run
  `docker compose -f docker/compose.yaml run --rm sim robofetz-smoke` to
  check package discovery, dependencies, and command output.
- **Compare with ground truth**. Stop the standard service, then run
  `docker compose -f docker/compose.yaml run --rm --service-ports sim robofetz-sim use_fake_perception:=true`
  to substitute Gazebo ground truth for camera perception.

The noVNC viewer uses a loopback binding for local access.
[Compose configuration](compose.yaml)
