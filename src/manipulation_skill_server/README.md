# manipulation_skill_server

The skill server of the manipulation subsystem: it executes `GoToPosition` goals from [`manipulation_interfaces`](../manipulation_interfaces/README.md).
Maintainer: Dmitrii Ognev (RoboCup-FEI manipulation team).

**Status: no executable yet.**
This version contains the core without ROS, the ROS interface (`SkillServer`: parameters and the `go_to_position` action server) and their tests.
The motion backend on the MoveIt Task Constructor (MTC), the node executable and its launch file come next; until then there is nothing to launch.
`Grip` and `Release` follow after `GoToPosition`.

## Overview

The package has two libraries.
The core (`manipulation_skill_server_core`) is plain C++17 and does not include ROS, MoveIt or MTC headers.
The ROS layer (`manipulation_skill_server_ros`) converts messages and parameters for the core and holds no logic.
Both are tested with a fake motion backend, so they build and run without a robot or a simulator.

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
   Rules: exactly one of `candidates` and `named_target`; a known profile; a named target only with `PTP` and present in the named-pose table; candidate ids non-empty and unique; `frame_id` set; finite position; a unit quaternion (norm within 1e-3 of 1, normalized afterwards); quality finite or absent.
2. **Plan** with the backend: all candidates within one shared time budget, or the named pose.
3. **Check cancellation**: a cancelled goal ends with `CANCELED` before any motion.
4. **Choose** the plan with the lowest combined cost (below); of equal costs, the one the backend listed first.
   No feasible plan ends with `PLANNING_FAILED` and the backend's reason.
5. **Execute** only the chosen plan.
   A failed execution ends with `EXECUTION_FAILED` and is not retried with another candidate, because the arm may have stopped anywhere on the path.

`selected_candidate_id` is set once a motion to a candidate has started, also when it was cancelled or failed.

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

## Build and test

```bash
colcon build --packages-select manipulation_interfaces manipulation_skill_server
colcon test --packages-select manipulation_skill_server
colcon test-result --verbose
```

The tests also run the default `ament_lint_common` linters, including the MIT copyright check.
New C++ files need the header: `ament_copyright --add-missing "RoboCup-FEI" mit <files>`, then `.github/scripts/format_cpp.sh`.
