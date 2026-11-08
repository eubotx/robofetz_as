# Working in ROBOFETZ_AS

This ROS 2 Jazzy workspace teaches combat robot autonomy through concrete
examples. Elf is the default strategy. Keep earlier implementations and
alternative node variants when they illustrate a distinct approach, so readers
can compare their assumptions, behavior, and trade-offs.

## Teaching while making changes

Completing the requested work and helping the user understand it are both part
of the task. Match the depth of the explanation to the change and the user's
familiarity with the system.

- Explain the relevant concept through this repository. For a ROS behavior
  change, show which nodes, topics, or launch settings are involved and point
  to the files the user can follow.
- Explain why you chose the approach and what trade-off mattered when there
  was a meaningful choice.
- When documenting a variant, explain what problem it solves, how it differs
  from the default, and when its limitations matter. Identify it as a default,
  an optional path, or a historical example. Where practical, show how to
  compare the approaches under the same conditions in simulation.
- Give the user a concrete way to check the result and describe what they
  should observe. For behavior changes, prefer a simulation check or a ROS 2
  inspection command. State which checks you actually ran.

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
