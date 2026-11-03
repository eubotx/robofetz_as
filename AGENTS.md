# Working in ROBOFETZ_AS

This ROS 2 Jazzy workspace is a learning repository for combat robot autonomy.
Elf is the default. Preserve earlier approaches so readers can compare direct
pursuit, Elf’s state machine and optional Nav2 planning.

## Finding the relevant code

- Trace the active graph from the [root launch file](src/robot_bringup/launch/robofetz_as.launch.py) through its includes, YAML and remaps. Check the node implementation for the interface it actually uses.
- Use the [package index](docs/packages/README.md) for node interfaces and variants, the [system architecture](docs/architecture.md) for cross-package behavior, and the [firmware guide](docs/firmware.md) for the ESP32 transport and actuator paths.

## Making changes

- Use Python 3 and ROS 2 Jazzy APIs. Follow nearby style and add code comments only when requested. Edit source packages and preserve unrelated user changes. Colcon generates `build/`, `install/` and `log/`.
- When changing a ROS interface, check both producers and consumers, including launch remaps, parameters, entry points and package dependencies. Update the affected package guide and any system-level contract in the architecture doc.
- Keep strategy and controller variants selectable separately. Check which node owns the final drive or weapon output before combining launch files.

## Validation

On a ROS 2 Jazzy host, run these commands from the workspace root with the affected package name substituted for `PACKAGE`.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to PACKAGE
source install/local_setup.bash
colcon test --packages-select PACKAGE
colcon test-result --verbose
```

The package tests mainly check style. For behavior changes, exercise the affected
launch path in simulation and check the relevant frames, timestamps, command
ownership or response to lost inputs. For documentation edits, verify statements
against source and check links. Reserve hardware bringup, autonomy enablement
and actuator commands for explicit hardware testing.
