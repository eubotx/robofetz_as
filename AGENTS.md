# AGENTS.md

## Project Overview

**ROBOFETZ_AS** is an autonomous combat robotics system built on ROS 2 Jazzy. It provides full stack autonomy for battle bots including perception, localization, navigation, and combat strategy.

## Build Commands

```bash
colcon build
source install/local_setup.bash
```

Selective build with symlink (faster iteration):
```bash
colcon build --packages-select <package_name> --symlink-install
```

## Linting & Testing

Run linting for a specific package:
```bash
ament_flake8 src/<package_name>
ament_pep257 src/<package_name>
```

Example for combat_strategizer:
```bash
ament_flake8 src/combat_strategizer
ament_pep257 src/combat_strategizer
```

## Package Structure

```
src/
├── arena_perception/      # Camera, opponent detection, localization
├── combat_strategizer/    # Combat state machine, attack/defence logic
├── robot_bringup/         # Main launch files, teleop
├── robot_description/     # URDF/xacro robot description
├── robot_navigation/      # Nav2 integration, map_node
├── robofetz_gazebo/       # Gazebo simulation environment
└── first_package/         # Legacy/testing package
```

## Key Nodes & Topics

### Combat Strategizer (`combat_strategizer`)
- **nav2_attack.py** - Main combat state machine (ATTACK/DEFENCE/STANDBY)
- **simple_attack.py** - Basic attack behavior
- **weapon_control.py** - Weapon arming/control

### Robot Navigation (`robot_navigation`)
- **nav2_navigator.py** - Nav2 wrapper with `attack()` and `escape()` methods
- **map_node.py** - Publishes `/defense_position` (PointStamped)

### Arena Perception (`arena_perception`)
- **opponent_detection.py** - Detects opponent position
- **robot_detection_node.py** - Robot self-localization

### Key Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/arena_perception_opponent_base_footprint_pose` | PoseStamped | Opponent position |
| `/arena_perception_robot_base_footprint_pose` | PoseStamped | Robot position |
| `/defense_position` | PointStamped | Strategic retreat position |
| `/cmd_vel` | Twist | Velocity commands |

## Combat State Machine (nav2_attack.py)

```
┌─────────┐     proximity met     ┌─────────┐     3s timer     ┌─────────┐
│ ATTACK  │ ───────────────────▶ │ DEFENCE │ ───────────────▶ │ STANDBY │
└─────────┘                       └─────────┘                  └─────────┘
     ▲                                                           │
     └───────────────────────────────────────────────────────────┘
                            immediate
```

### Adding New Conditions

Conditions must implement:
```python
class YourCondition:
    def __init__(self, ...): ...
    def check(self, robot_pose, opponent_pose) -> bool: ...  # True = trigger retreat
    def reset(self): ...
```

Replace in `Nav2Attack.__init__`:
```python
self.condition = YourCondition(...)
```

## Code Conventions

- Python 3 with type hints (`robot_pose: PoseStamped | None`)
- No comments unless requested
- Follow existing patterns in each package
- ROS 2 Jazzy API

## Run Commands

Full simulation:
```bash
ros2 launch robofetz_as robofetz_as.launch.py
```

Real robot:
```bash
ros2 launch robofetz_as robofetz_as.launch.py use_sim:=false
```

Run single node:
```bash
ros2 run combat_strategizer nav2_attack
ros2 run robot_navigation map_node
```
