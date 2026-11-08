# Develop ROS 2 Jazzy in VS Code

The Dev Container mounts your checkout at `/ws` and keeps `build/`, `install/`,
and `log/` in Docker volumes. VS Code terminals load Jazzy and the workspace
overlay. Run ROS commands in the same container as the simulation.
[Installation and editor setup](../docs/installation.md#vs-code-dev-container)

## Run the simulation

From the repository root in VS Code, choose **Dev Containers: Reopen in
Container** and wait for the initial build. Run **ROS: simulation desktop**.
In the **Ports** panel, open the forwarded address for port 6080 with
`/vnc.html?autoconnect=1&resize=scale` appended. Select the opponent color or
press `q` to accept the defaults. Stop the task with Ctrl+C.

In a second container terminal, `ros2 topic echo /robot/pose --once` should
show a pose once localization is ready. To steer the simulated opponent, run
`ros2 run robot_bringup enemy_teleop_node`. Use WASD or the arrow keys, Space
to stop, and Q to quit. For a gamepad, use the [Xbox profile](#xbox-controller-passthrough).

To compare with Gazebo ground truth, stop the task and run
`bash .devcontainer/workspace.sh sim use_fake_perception:=true`.

## Edit, build, and test

| When | Action |
| --- | --- |
| Edit an existing Python function | Restart the affected node. The symlink install picks up source edits. |
| Change package metadata, entry points, or installed assets | Run **ROS: build simulation workspace** and restart the affected launch or node. |
| Remove stale build artifacts | Stop nodes using the workspace and run **ROS: clean build**. It clears generated build and install files and keeps logs. |
| Check the workspace | Run **ROS: test simulation workspace** for package tests or **ROS: smoke test** for a headless runtime check. |
| Change the Dockerfile or Dev Container settings | Run **Dev Containers: Rebuild Container**. |

Use **Terminal: Run Task** to find the ROS tasks. Open a fresh terminal after
a rebuild or run `source install/local_setup.bash`. Verify the overlay with
`ros2 pkg prefix robot_bringup`, which should return `/ws/install/robot_bringup`.

## Xbox controller passthrough

The optional local Xbox profile passes one Linux input device into the
container. Under WSL 2, the controller must appear as a Linux event device
accessible to the Docker engine. For a wired controller, see Microsoft's
[USB attachment instructions](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).

### Configure the local profile

Find the controller's `eventN` handler in `/proc/bus/input/devices`. In the
Ubuntu host terminal, substitute its path below. Look for
`ID_INPUT_JOYSTICK=1` and note the udev path and numeric group.

```bash
device=/dev/input/event14
udevadm info --query=property --name="$device"
stat -Lc 'udev=/run/udev/data/c%Hr:%Lr group=%g permissions=%A' "$device"
```

Use `.devcontainer/xbox.tmp/devcontainer.json` if it exists. Otherwise, from
the repository root, copy the default profile into that Git-ignored directory.

```bash
mkdir -p .devcontainer/xbox.tmp
cp -n .devcontainer/devcontainer.json .devcontainer/xbox.tmp/devcontainer.json
```

Set `name` to `ROBOFETZ_AS · ROS 2 Jazzy + Xbox (local)`,
`build.dockerfile` to `../../docker/Dockerfile`, and `build.context` to
`../..`. Append these entries, substituting your controller's event path,
udev path, and group.

| Setting | Entries to append |
| --- | --- |
| `mounts` | `"type=bind,source=/run/udev/data/c13:78,target=/run/udev/data/c13:78,readonly"` |
| `runArgs` | `"--device=/dev/input/event14:/dev/input/event14:r"`, `"--group-add=995"` |

Choose the Xbox profile with **Dev Containers: Reopen in Container** or
**Dev Containers: Switch Container**. Rebuild the container after editing the
mappings. If you reconnect the controller, check whether its paths changed.

### Check the controller

In the Xbox container, `ros2 run joy joy_enumerate_devices` should list the
controller. If it is missing, check the device's read permissions, the
container user's groups (`id -G`), and the mounted udev record.

Start **ROS: simulation desktop**. In a second terminal, run
`ros2 topic echo /joy` and move the stick to check input. Then run
`ros2 topic echo /opponent/cmd_vel`, hold the configured enable button, and
move the left stick. Expect velocity messages and movement in Gazebo. Use one
opponent control source at a time because keyboard and Xbox steering publish
to the same topic.
[Opponent teleop launch](../src/robot_bringup/launch/teleop_opponent.launch.py) ·
[Xbox button and axis mapping](../src/robot_bringup/config/xbox_teleop.config.yaml)
