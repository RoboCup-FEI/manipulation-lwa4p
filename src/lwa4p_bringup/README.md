# lwa4p_bringup

Launch files and robot configuration of the manipulation skill server for the Schunk LWA 4P.
Maintainer: Dmitrii Ognev (RoboCup-FEI manipulation team).

## Overview

[`manipulation_skill_server`](../manipulation_skill_server/README.md) knows no robot; everything specific to the LWA 4P is here:
the MoveIt group, the named poses, and how to start the server with MoveIt on each backend.

| File | Contents |
| --- | --- |
| `launch/skill_server.launch.py` | A backend with MoveIt, and the skill server |
| `config/skill_server.yaml` | The MoveIt group and the named poses of the LWA 4P |

## Launch

```bash
ros2 launch lwa4p_bringup skill_server.launch.py                  # mock hardware with RViz
ros2 launch lwa4p_bringup skill_server.launch.py sim:=gazebo      # Gazebo Harmonic
ros2 launch lwa4p_bringup skill_server.launch.py sim:=isaac       # Isaac Sim, started before
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `sim` | `mock` | `mock`: Tomas's `demo.launch.py`; `gazebo`: Tomas's `gazebo.launch.py`; `isaac`: `isaac_moveit.launch.py` of `manipulation_isaac_sim` |
| `rviz` | `true` | RViz with MoveIt |

Arguments of the backend's launch file pass through, for example `gui:=false` for Gazebo or `world:=table` for Isaac Sim.
Isaac Sim itself starts separately: `ISAACSIM_PATH=<Isaac Sim directory> isaac_sim/run.sh` ([isaac_sim/README.md](../../isaac_sim/README.md)).

The launch file changes none of the backends' launch files.
It enables `move_group/ExecuteTaskSolutionCapability`, which MTC needs, through the `capabilities` argument of `demo.launch.py` or through a parameter set around the Gazebo and Isaac Sim launch files.
The skill server gets the MoveIt kinematics, joint limits and planning pipelines of `lwa4p_moveit_config`, and copies the URDF and SRDF from `move_group`, so it plans with the robot model of the running backend.

## Try it

```bash
ros2 action send_goal /go_to_position manipulation_interfaces/action/GoToPosition "{named_target: work1}"
ros2 action send_goal /go_to_position manipulation_interfaces/action/GoToPosition "{named_target: home}"
```

Press Ctrl-C in the `send_goal` terminal during a motion to cancel the goal: the arm stops where it is.

## Configuration

`config/skill_server.yaml` sets the MoveIt group (`arm`) and the named poses.
`home`, `work1` and `work2` equal the SRDF states of the same names in `lwa4p_moveit_config` on 2026-11-22; they are ours and may change independently of Tomas's SRDF.
Positions are radians, one per joint in `named_poses.joint_names`; write them as doubles (`0.0`).
The other parameters are described in the skill server's README.
