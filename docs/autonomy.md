# Autonomy variants

| Stack | Role | Behavior |
| --- | --- | --- |
| Simple autonomy stack | Direct-pursuit implementation of first robot iteration: RhinozeROS | Drives directly toward the other robot's latest reported position. |
| Elf autonomy stack | Statemachine enhanced implementation of second and third robot iteration: Elfbar | Uses a simple state machine to pursue the opponent, back away after staying close, and recover near arena walls. |

Both autonomy stacks have two parts. The **combat strategizer** lives in the
`combat_strategizer` package and makes a decision for a goal pose and driving
style. The **navigator** lives in the `robot_navigation` package and turns
that decision into linear and angular velocity commands on
`/cmd_vel_autonomous`.

## Simple autonomy stack

### Combat strategizer

`simple_combat_strategizer` makes each new
`/opponent/pose` the target by forwarding it to `/goal_pose`. This makes
Simple a baseline for studying direct pursuit. [Strategy source](../src/combat_strategizer/combat_strategizer/simple_combat_strategizer.py)

### Navigator

`simple_navigator` starts steering as soon as it has both
the robot's own `/robot/pose` and a `/goal_pose`. It slows as it approaches
the goal and commands zero velocity within the position tolerance. It keeps
the last goal if opponent updates stop. [Navigator source](../src/robot_navigation/robot_navigation/simple_navigator.py)

### Simple autonomy launch

`simple_autonomy.launch.py` starts `simple_combat_strategizer` and
`simple_navigator`. [Simple launch](../src/combat_strategizer/launch/simple_autonomy.launch.py)

## Elf autonomy stack

### Combat strategizer

`elf_combat_strategizer` uses `/robot/pose` and
`/opponent/pose` in its state machine to choose a maneuver. It sends a target
on `/goal_pose` and a direction on `/drive_mode`, or sends an immediate
command on `/elf_navigator_cmd`. [Strategy source](../src/combat_strategizer/combat_strategizer/elf_combat_strategizer.py)

#### State machine

| State | Behavior with default settings |
| --- | --- |
| `IDLE` | Commands a stop. Enters `ATTACK` when `/autonomy` is enabled and the robot pose is recent. Disabling autonomy or going 2 seconds without a robot pose message returns Elf to `IDLE`. |
| `ATTACK` | Drives forward toward the opponent. Staying within 0.2 m for 5 seconds triggers `DEFENSE`. Facing out of the arena near a wall can trigger `WALLRECOVERY`. |
| `DEFENSE` | Picks the point farthest from the opponent among four points near the arena center. Backs toward that point, then resumes `ATTACK`. |
| `WALLRECOVERY` | Commands a timed reverse, then resumes `ATTACK`. |

Elf adds reactions to sustained close contact and to facing an arena boundary.
Those decisions depend on fixed distance and timing thresholds and the configured
arena size. If opponent updates stop, Elf keeps using the last reported opponent
pose. [Strategizer interfaces](packages/combat_strategizer.md)

### Navigator

`elf_navigator` uses `/robot/pose` to steer toward `/goal_pose`. The
strategizer sends `FORWARD` or `BACKWARD` on `/drive_mode` to choose which
end of the robot leads while tracking the goal. Immediate commands arrive on
`/elf_navigator_cmd`. For example, `STOP` clears the current goal and
publishes zero velocity. [Navigator source](../src/robot_navigation/robot_navigation/elf_navigator.py)
