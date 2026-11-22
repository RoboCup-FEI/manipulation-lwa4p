# manipulation_skill_server

The skill server of the manipulation subsystem: it executes `GoToPosition` goals from [`manipulation_interfaces`](../manipulation_interfaces/README.md).
Maintainer: Dmitrii Ognev (RoboCup-FEI manipulation team).

**Status: `GoToPosition` works on mock hardware, Gazebo and Isaac Sim: named targets, and TCP candidates with `PTP` and `LIN`.**
`Grip` and `Release` follow after `GoToPosition`.
To run it on the LWA 4P, use [`lwa4p_bringup`](../lwa4p_bringup/README.md).

## Overview

The package has three libraries and one executable.
The core (`manipulation_skill_server_core`) is plain C++17 and does not include ROS, MoveIt or MTC headers.
The ROS layer (`manipulation_skill_server_ros`) converts messages and parameters for the core and holds no logic.
The MTC backend (`manipulation_skill_server_mtc`) plans and executes motions with the MoveIt Task Constructor (MTC).
`skill_server_node` connects them.
The core and the ROS layer are tested with a fake motion backend, so their tests run without MoveIt, a robot or a simulator.

| Header | Contents |
| --- | --- |
| `types.hpp` | Poses, `TcpCandidate`, `MotionProfile`, `JointTarget`, `OutcomeCode`, `Outcome` |
| `task.hpp` | `CancellationToken`, `TaskContext` (the link to the caller), `TaskInterface`, the `Task` base |
| `busy_task_runner.hpp` | `BusyTaskRunner`: one worker thread, no queue |
| `motion_backend.hpp` | `MotionBackend` interface, `PlannedMotion`, plan and execution results |
| `named_pose_table.hpp` | `NamedPoseTable`: joint-space poses by name |
| `go_to_position_task.hpp` | `GoToPositionRequest`, `GoToPositionResult`, `SelectionWeights`, `GoToPositionSettings`, `GoToPositionTask` |
| `conversions.hpp` (ROS) | Goal and result messages to and from the core; the only conversion of outcome codes to `Outcome.msg` |
| `goal_context.hpp` (ROS) | `GoalContext`: a `TaskContext` on an action goal handle |
| `skill_server_ros.hpp` (ROS) | `SkillServer`: parameters and the action server |
| `parameters.hpp` (ROS) | Reading parameters that may already be declared |
| `mtc_motion_backend.hpp` (MTC) | `MtcMotionBackend`: the `MotionBackend` on MTC and `move_group` |
| `task_solution_executor.hpp` (MTC) | `TaskSolutionExecutor`: executes MTC solutions through `move_group` and waits until the arm is at rest |
| `polling.hpp` | `wait_until()`: the deadline- and cancellation-aware wait of the adapters |

Every action result has the same shape: `Outcome outcome` (code and message) plus the action's own fields, such as `selected_candidate_id`.
The roles follow the Command pattern: the tasks are commands, `BusyTaskRunner` is the invoker, `MotionBackend` is the receiver, and `SkillServer` creates the tasks.

### BusyTaskRunner

- One worker thread runs one task at a time.
- A task submitted while another runs ends at once with `BUSY` and never runs; the running task is not affected.
- A task that throws ends with `INTERNAL_ERROR`, and the runner refuses every later task with `INTERNAL_ERROR`: the arm may be in an unknown state, so a person must restart the server.
- Every task the runner receives gets exactly one result.
- The worker is free before it delivers a result, so a caller may send its next goal (for example `Grip` after `GoToPosition`) as soon as it has the result.
- `TaskContext::finish()` must not throw: the ROS layer handles and logs its own delivery errors.
- The destructor waits for the running task; `SkillServer` cancels it first.

### GoToPositionTask

1. **Validate** the goal; on any error the result is `INVALID_GOAL` and the backend is not called.
   Rules: exactly one of `candidates` and `named_target`; a known profile; a named target only with `PTP` and present in the named-pose table; candidate ids non-empty and unique; `frame_id` set; a stamp not before the epoch; finite position; a unit quaternion (norm within 1e-3 of 1, normalized afterwards); quality finite or absent.
2. **Plan** with the backend: all candidates within one shared time budget, or the named pose.
3. **Check cancellation**: a cancelled goal ends with `CANCELED` before any motion.
4. **Choose** the plan with the lowest combined cost (below); of equal costs, the one the backend listed first.
   No feasible plan ends with `PLANNING_FAILED` and the backend's reason.
5. **Execute** only the chosen plan.
   A failed execution ends with `EXECUTION_FAILED` and is not retried with another candidate, because the arm may have stopped anywhere on the path.

`selected_candidate_id` names the candidate whose plan was chosen, also when its motion was cancelled, failed or never started.
It is empty for a named target, when no plan was chosen, and with `INTERNAL_ERROR`: the task threw, so it reports nothing but the outcome.

### Selection cost (placeholder)

`combined_cost = w_motion * motion_cost - w_quality * quality`, lower is better.
A candidate without a quality estimate gets no quality term.
Both weights default to 1.0; only their ratio changes which plan wins.
The formula is open (consolidation D02); the weights are parameters (below).

## ROS interface

`SkillServer` takes a node and a motion backend; it does not create its own node, so the MTC backend can share it.

### Action `go_to_position`

Type `manipulation_interfaces/action/GoToPosition`, a relative name in the node's namespace.

- Every goal is accepted, so every goal ends with a result that carries an `Outcome`; a goal that arrives while another action runs ends at once with `BUSY`.
- The goal status agrees with the outcome code: `SUCCEEDED` with `SUCCESS`, `CANCELED` with `CANCELED`, `ABORTED` with every other code.
  ROS allows the `CANCELED` status only after a cancel request, so a `CANCELED` outcome without one ends as `ABORTED`.
- A cancel request is always accepted; the task checks it during planning and before the motion, and the motion backend stops the arm during execution.
- When `SkillServer` is destroyed (for example on Ctrl-C), it cancels the running goal and waits until the task returns; that goal's caller gets no result.
- The server logs every goal when it starts and when it ends, with its goal id and outcome.

### Parameters

| Parameter | Type | Default | Meaning |
| --- | --- | --- | --- |
| `go_to_position.planning_budget_s` | double | 5.0 | Planning time of one goal, shared by all its candidates (MoveIt's default allowed planning time) |
| `go_to_position.selection_weights.motion_cost` | double | 1.0 | Weight of the motion cost in the selection cost |
| `go_to_position.selection_weights.quality` | double | 1.0 | Weight of the candidate quality in the selection cost |
| `named_poses.names` | string array | none | Names a goal can use as `named_target` |
| `named_poses.joint_names` | string array | none | The joints of every named pose, in this order |
| `named_poses.<name>` | double array | none | Positions of pose `<name>`, one per joint in `named_poses.joint_names` |

The server refuses to start (`std::invalid_argument`) if the budget is not positive, a weight is negative, a pose has a different number of positions than `named_poses.joint_names`, or a joint or pose name repeats.
Write positions as doubles (`0.0`, not `0`): ROS does not convert an integer array to a double array.
Robot-specific values, such as the poses of the LWA 4P, belong in the robot's bringup configuration, not in this package.

```yaml
skill_server:
  ros__parameters:
    go_to_position:
      planning_budget_s: 5.0
    named_poses:
      names: [ready]
      joint_names: [joint_1, joint_2]
      ready: [0.0, 0.5]
```

The server reads parameters that are already declared, too, so it works on a node created with `automatically_declare_parameters_from_overrides(true)`, which MTC needs.

## MTC motion backend

- **Robot model:** the URDF and SRDF of `move_group`, copied from its parameters `robot_description` and `robot_description_semantic` when the server starts, unless the server has its own.
  In Gazebo and Isaac Sim they differ from the MoveIt package's, and MTC must check collisions with the same geometry that `move_group` executes with.
- **Start state:** every task of a goal starts from one planning scene, fetched from `move_group` (`get_planning_scene`) when the goal starts.
  The wait for it counts in the planning budget and ends on a cancel request; without an answer the goal ends with `PLANNING_FAILED`.
  MTC's `CurrentState` stage is not used: it waits for `move_group`'s answer without a time limit, and it creates a node for every task.
- **Named target:** one MTC task: the start state, then a joint-space motion.
  It tries joint interpolation first, the shortest joint-space path, and the planning pipeline (OMPL) only if interpolation collides.
- **TCP candidates:** one MTC task per candidate, planned one after another in the order of the goal.
  1. Every pose is transformed into the model frame (`world` on the LWA 4P) through TF at its `header.stamp`; a zero stamp takes the latest transform.
     All candidates of a goal share one wait of at most 0.5 s for missing transforms, counted on the steady clock (a paused simulator would stop TF's own timeout).
     A candidate whose transform fails gets no plan, and the others are still planned.
  2. Each candidate gets an equal share of the budget left when its turn comes, so time that a quick candidate leaves goes to the later ones.
     MTC checks its time only between planning steps, so a goal can take somewhat longer than the budget.
  3. **`PTP`:** up to 8 inverse kinematics (IK) solutions of the pose, searched for at most 0.5 s and at most half of the candidate's share: the first IK attempt starts from the current state, the next ones from random states.
     To every solution: joint interpolation, and the planning pipeline where interpolation collides; the shortest of these motions is the candidate's plan.
     One IK solution alone is not enough: on the LWA 4P, KDL started from `work2` reaches the TCP pose of `work1` through an elbow flip of 6.4 rad instead of 1.3 rad.
  4. **`LIN`:** `CartesianPath` moves the TCP along a straight line in 1 cm steps; the whole line or no plan.
- **Motion cost:** the joint-space path length (`cost::PathLength`) of the chosen motion; the IK search adds nothing.
- **Collision checks:** joint interpolation checks every 0.01 rad (MTC's default 0.1 rad moved the fingers past thin obstacles, and `move_group` then refused the executed trajectory).
  The OMPL step comes from the planning pipeline's parameters; `lwa4p_bringup` sets it for the LWA 4P.
- **Known limits:** KDL does not converge reliably at a wrist singularity (joint 5 at 0).
  `LIN` cannot start there; `PTP` to a pose whose only collision-free IK solutions are singular may fail.
  The named poses `home`, `work1` and `work2` of the LWA 4P are such configurations.
  Random collision-free targets with joint 5 away from 0 (2026-11-22): on mock, `PTP` reached 24 of 24 and a 5 cm `LIN` step down afterwards succeeded 22 times; in Gazebo 11 of 12 and 11 of 11 (the failure was Gazebo's controller, below the planner); in Isaac Sim 8 of 8 and 8 of 8.
- **Speed:** the planners use `default_velocity_scaling_factor` and `default_acceleration_scaling_factor` of the MoveIt configuration; MTC alone would ignore them and move faster than MoveIt allows.
- **Execution** (`TaskSolutionExecutor`): the `execute_task_solution` action of `move_group`, which needs `move_group/ExecuteTaskSolutionCapability`.
  It sends its own goal instead of MTC's `Task::execute()`: after a cancel request it waits until `move_group` reports that the motion stopped, so `CANCELED` means the arm is at rest.
  - A cancel request while it waits for the action server (at most 5 s) or before the goal is sent ends with `CANCELED`, and the arm does not move.
  - A motion still running at its planned duration plus 30 s is stopped and ends with `EXECUTION_FAILED`; `move_group` aborts slow motions earlier itself, so this only catches a `move_group` that stopped reporting.
  - After a stop request, `move_group` has 5 s to report the stop, counted from the request.
    If it does not, or does not answer the goal at all, the server ends with `INTERNAL_ERROR`, because the arm may still move.
- **Planning cancellation:** checked while waiting for the planning scene and the transforms, between candidates, and between MTC planning steps.
  Neither MTC's time limit nor a cancel request interrupts a call that is already running; how long it takes depends on the call's own limit.
  The IK search stops after at most 0.5 s, a planning pipeline call after its allowed planning time (the candidate's share of the budget), and `CartesianPath` has no time limit but interpolates the line only once.
  The backend does not pass the cancel request to the planners themselves.
- **Messages:** `PLANNING_FAILED` names every candidate without a plan and the first MTC stage that failed for it, for example `candidate 'a': no plan (PLANNING_FAILED): inverse kinematics (0/1): no IK found`.
  The server logs the motion cost of every planned candidate.

| Parameter | Type | Default | Meaning |
| --- | --- | --- | --- |
| `mtc.arm_group` | string | required | MoveIt planning group of the arm |
| `mtc.tcp_frame` | string | required | The link whose pose a `TcpCandidate` gives; the IK solver of `mtc.arm_group` must be able to place it |
| `mtc.planning_pipeline` | string | `ompl` | Planning pipeline used when joint interpolation fails |
| `mtc.move_group_node` | string | `move_group` | Node whose robot description the server copies |
| `robot_description_planning.default_velocity_scaling_factor`, `..._acceleration_scaling_factor` | double | required | From the MoveIt configuration (`joint_limits.yaml`) |

The node also needs the MoveIt parameters `robot_description_kinematics`, `robot_description_planning` and the planning pipelines; `lwa4p_bringup` passes them.

## Executable `skill_server_node`

- Creates the node `skill_server` with `automatically_declare_parameters_from_overrides(true)`, which MoveIt and MTC need, and spins it in a `SingleThreadedExecutor`.
- Handles SIGINT and SIGTERM itself instead of rclcpp: on Ctrl-C it cancels the running goal while ROS still works, waits until `move_group` stopped the arm, and only then shuts ROS down.
- Exits with code 1 if a parameter is missing or invalid, or the robot model cannot be loaded.

## Build and test

```bash
colcon build --packages-select manipulation_interfaces manipulation_skill_server
colcon test --packages-select manipulation_skill_server
colcon test-result --verbose
```

`test_task_solution_executor` runs `TaskSolutionExecutor` against a fake `execute_task_solution` server: success, abort, rejection, missing `move_group`, cancel before and during the motion, the stop of an overlong motion, and a `move_group` that does not stop.
Planning with MTC, TF and the planning scene are checked only live on mock hardware and the simulators (`lwa4p_bringup`).
The tests also run the default `ament_lint_common` linters, including the MIT copyright check.
New C++ files need the header: `ament_copyright --add-missing "RoboCup-FEI" mit <files>`, then `.github/scripts/format_cpp.sh`.
