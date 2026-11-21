# manipulation_skill_server

The skill server of the manipulation subsystem: it executes `GoToPosition` goals from [`manipulation_interfaces`](../manipulation_interfaces/README.md).
Maintainer: Dmitrii Ognev (RoboCup-FEI manipulation team).

**Status: core only.**
This version contains the logic without ROS and its unit tests.
The ROS node, the MoveIt Task Constructor (MTC) motion backend, `Grip` and `Release` come in later pull requests.
There is nothing to launch yet.

## Overview

The core is plain C++17 and does not include ROS, MoveIt or MTC headers.
It is tested with a fake motion backend, so it builds and runs without a robot or a simulator.

| Header | Contents |
| --- | --- |
| `types.hpp` | Poses, `TcpCandidate`, `MotionProfile`, `JointTarget`, `OutcomeCode`, `Outcome` |
| `task.hpp` | `CancellationToken`, `TaskContext` (the link to the caller), `TaskInterface`, the `Task` base |
| `busy_task_runner.hpp` | `BusyTaskRunner`: one worker thread, no queue |
| `motion_backend.hpp` | `MotionBackend` interface, `PlannedMotion`, plan and execution results |
| `named_pose_table.hpp` | `NamedPoseTable`: joint-space poses by name |
| `go_to_position_task.hpp` | `GoToPositionRequest`, `GoToPositionResult`, `SelectionWeights`, `GoToPositionSettings`, `GoToPositionTask` |

Every action result has the same shape: `Outcome outcome` (code and message) plus the action's own fields, such as `selected_candidate_id`.
The roles follow the Command pattern: the tasks are commands, `BusyTaskRunner` is the invoker, `MotionBackend` is the receiver, and the ROS layer will create the tasks.

### BusyTaskRunner

- One worker thread runs one task at a time.
- A task submitted while another runs ends at once with `BUSY` and never runs; the running task is not affected.
- A task that throws ends with `INTERNAL_ERROR`, and the runner refuses every later task with `INTERNAL_ERROR`: the arm may be in an unknown state, so a person must restart the server.
- Every task the runner receives gets exactly one result.
- The worker is free before it delivers a result, so a caller may send its next goal (for example `Grip` after `GoToPosition`) as soon as it has the result.
- `TaskContext::finish()` must not throw: the ROS layer handles and logs its own delivery errors.
- The destructor waits for the running task; the ROS layer must cancel it first.

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
The formula is open (consolidation D02); the weights will come from the server configuration.

## Build and test

```bash
colcon build --packages-select manipulation_interfaces manipulation_skill_server
colcon test --packages-select manipulation_skill_server
colcon test-result --verbose
```

The tests also run the default `ament_lint_common` linters, including the MIT copyright check.
New C++ files need the header: `ament_copyright --add-missing "RoboCup-FEI" mit <files>`, then `.github/scripts/format_cpp.sh`.
