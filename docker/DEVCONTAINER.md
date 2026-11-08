# Develop ROS 2 Jazzy in VS Code

Use the Dev Container to edit, build, and test ROS packages from VS Code.
For a packaged copy of the simulator, use [standalone Docker Compose](README.md).

VS Code's window stays on your desktop while its terminal and tasks run in
the container. The container mounts your Linux checkout at `/ws`, so edits
are saved directly to the repository. The image supplies ROS 2 Jazzy,
Gazebo, RViz, Python, and build tools.

## Host setup

Use Ubuntu 24.04 with Docker Engine, or Ubuntu 24.04 under WSL2 with Docker
Desktop's Linux engine and WSL integration enabled for that distribution.
Install VS Code and its **Dev Containers** extension; also install **WSL** on
Windows. No host ROS installation is required. Run `docker info` in your
Ubuntu host terminal to check that it can reach Docker. Compose is used for
the standalone simulator; the Dev Container builds directly from the Dockerfile.

Keep the WSL checkout in the Linux filesystem, such as `~/projects/robofetz_as`, rather than `/mnt/c`. This improves bind-mount performance and Linux file watching. Open it from an Ubuntu-24.04 terminal:

```bash
cd ~/projects/robofetz_as
code .
```

On native Ubuntu, open your Linux checkout the same way. For a fresh clone:

```bash
mkdir -p ~/projects
git clone https://github.com/eubotx/robofetz_as.git ~/projects/robofetz_as
cd ~/projects/robofetz_as
git switch rework_before_new_season
code .
```

Stop the standalone Compose simulator if it is running, then run
**Dev Containers: Reopen in Container** from the Command Palette.
The first setup installs editor extensions and builds the workspace. The
first image build requires internet access. Wait for the initial build to
finish.

Look for **Dev Container: ROBOFETZ_AS · ROS 2 Jazzy** in the bottom-left
corner of VS Code. Open an integrated terminal and check:

```bash
pwd
whoami
echo "$ROS_DISTRO"
ros2 pkg prefix robot_bringup
```

Expect `/ws`, `robot`, `jazzy`, and `/ws/install/robot_bringup`.
This confirms that your terminal is in the container and can find your
built workspace. Opening the container does not start the simulator.

The migration on 2026-11-05 copied the existing checkout, Git history and untracked `handover.md` to `/home/eubotx/projects/robofetz_as` in **Ubuntu-24.04**, retaining `rework_before_new_season`. Generated host build directories and Python caches were excluded. The Windows and WSL checkouts are independent copies; use the WSL copy for subsequent development. The environment configuration is also present in the Windows checkout for review.

## Where files are stored

| Path in the container | What it contains | What happens when you rebuild the container |
| --- | --- | --- |
| `/ws/src` and `/ws/.git` | Your source and Git history in the Linux checkout | Your edits and commits remain in the checkout. |
| `/ws/build` | Colcon's intermediate build files and test results | A dedicated Docker volume retains them. |
| `/ws/install` | The installed ROS workspace, also called the overlay | A dedicated Docker volume retains it. |
| `/ws/log` | Colcon build and test logs | A dedicated Docker volume retains them. |

Use Git from the WSL checkout or the VS Code container terminal; both see
the same repository. The original Windows checkout is a separate copy.

VS Code's remote tools, terminals, and tasks run as `robot`. Dev Containers
matches its UID/GID to the Linux host user. The entrypoint adjusts ownership
of the three generated-output volumes and then drops privileges.

The volumes are keyed by the Dev Container ID and hide any host outputs at
the same paths. Avoid mixing `--merge-install` with this development build:
they use different install layouts. If a ROS distro, Python ABI, or build
layout change makes the cached outputs incompatible, stop the container,
remove only its three generated-output volumes in Docker, and reopen to
build fresh. This clears the build outputs and logs; source stays in WSL.
For lasting changes to system dependencies, edit the [Dockerfile](Dockerfile)
and rebuild the container.

The build searches only `arena_perception`, `combat_strategizer`,
`robot_navigation`, `robot_bringup`, `robot_description`, and
`robofetz_gazebo`. It excludes the micro-ROS hardware paths even if
initialized. The AprilTag image dataset submodule is optional test and
reference data. No hardware submodule or dataset is downloaded automatically.

## Edit, build and test

Interactive Bash terminals load Jazzy and, if present, the workspace overlay
from `/ws/install/local_setup.bash`. Jazzy supplies the base ROS tools. The
overlay tells ROS where to find the packages built from this repository.
Tasks load their required environment themselves.

Use **Terminal: Run Build Task** (`Ctrl+Shift+B`) or **Terminal: Run Task**:

| Task | Equivalent container command |
| --- | --- |
| ROS: build simulation workspace | `bash .devcontainer/workspace.sh build` |
| ROS: test simulation workspace | `bash .devcontainer/workspace.sh test` |
| ROS: smoke test | `bash .devcontainer/workspace.sh smoke` |
| ROS: simulation desktop | `bash .devcontainer/workspace.sh sim` |

The test task prints `colcon test-result --verbose` and returns failure if
tests fail. The WSL validation found 12 existing style failures and six
skips. The smoke test checks package discovery, Python imports, AprilTag,
Gazebo CLI, and mux output in isolated ROS domain 91. For behavior changes,
also run the relevant launch path in simulation and check its outputs.

For a focused rebuild:

```bash
bash .devcontainer/workspace.sh build --packages-select combat_strategizer
source install/local_setup.bash
ros2 pkg prefix combat_strategizer
```

| What changed? | What to do |
| --- | --- |
| Body of an existing Python function | Save and restart the affected node. With [symlink install](https://colcon.readthedocs.io/en/released/reference/verb/build.html), the installed module can use the edited source. |
| Package entry point, manifest, compiled code, launch file, or config | Run the build task, then restart the affected launch or node. Open a fresh terminal or source the overlay again. |
| Simulation desktop script (`docker/sim.sh`) | Stop and rerun **ROS: simulation desktop**. The task reads the script from the mounted checkout. |
| Docker dependencies, Dev Container settings, or scripts copied into the image | Run **Dev Containers: Rebuild Container**. |

Rebuilding ROS packages updates the overlay. Rebuilding the container
recreates the environment that runs them. Dependency layers are cached
independently of ordinary application source edits.

## Simulation and GUI

Run **ROS: simulation desktop**, then open [noVNC](http://localhost:6080/vnc.html?autoconnect=1&resize=scale). Gazebo and RViz use the included Xvfb desktop and software rendering on either host. Select the opponent color, or press **q** in the color selector to accept defaults. Autonomy remains disabled until explicitly enabled in simulation.

Xbox controllers are unsupported in this WSL 2 setup because the tested WSL
kernel lacks the `xpad` driver. To drive the simulated opponent, open a second
Dev Container terminal while the simulation is running and start
`enemy_teleop_node` using the `keyboard_node` executable:

```bash
ros2 run robot_bringup keyboard_node
```

Use WASD or the arrow keys to steer, Space to stop, and Q to quit. If you have
installed rqt Robot Steering, you can also use its
[graphical controls](README.md#drive-the-simulated-opponent).

```bash
ros2 topic echo /arena_camera/image --once --field header
ros2 topic echo /robot/pose --once
ros2 topic echo /clock --once
ros2 run tf2_ros tf2_echo world robot/base_footprint
```

Development nodes use domain 42 with localhost discovery inside the container. The default profile exposes no host devices or Docker socket. Run ROS tools inside this container. Only one desktop can own localhost port 6080: stop the standalone Compose simulation before opening the Dev Container, and stop the Dev Container before starting standalone Compose.

Stop the simulation with Ctrl+C in its task terminal. Closing the VS Code remote window stops the container; volumes remain. Reopening starts it again without launching the simulation automatically. A container stop also terminates simulator processes started from its terminals. Standalone startup/shutdown remains documented in [docker/README.md](README.md).

To compare camera perception with simulator ground truth, stop the default
simulation first, then run this in the container:

```bash
bash .devcontainer/workspace.sh sim use_fake_perception:=true
```

## Troubleshooting

| Symptom | What to check |
| --- | --- |
| VS Code says **WSL: Ubuntu-24.04** or the terminal is in `/home/...` | Use **Reopen in Container**. A Dev Container terminal opens in `/ws`. |
| `ros2` works but cannot find `robot_bringup` | Check the initial build or run the build task. Use a fresh terminal, or source `install/local_setup.bash`. |
| Source edits appear missing | Check the branch and checkout with `git status --short --branch`. The original Windows copy is independent. Restart the node after saving. |
| Port 6080 is in use | Run `docker ps` from the Ubuntu host terminal. Stop the competing container before reopening. |
| The browser cannot connect | Start **ROS: simulation desktop** and check its task terminal. The task starts its own X11 desktop for noVNC and clears inherited Wayland display variables. |

If RViz exits with `corrupted double-linked list`, retry it from a container
terminal while the desktop is running:

```bash
ros-env rviz2 -d /ws/install/robot_bringup/share/robot_bringup/config/config.rviz
```

This occurred once during WSL validation. The retry stayed running.

## Tooling references

- [VS Code Dev Containers](https://code.visualstudio.com/docs/devcontainers/create-dev-container)
- [Non-root users and UID matching](https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user)
- [Named volumes and performance](https://code.visualstudio.com/remote/advancedcontainers/improve-performance)
- [Docker Desktop WSL integration](https://docs.docker.com/desktop/features/wsl/)

The Dev Container builds directly from the `development` Dockerfile stage. This avoids the generated-Dockerfile filesystem entitlement failure observed with the installed Compose/Bake combination, while retaining Compose for standalone simulation.
