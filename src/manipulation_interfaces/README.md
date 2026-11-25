# manipulation_interfaces

Actions and messages of the manipulation skill server.
Maintainer: Dmitrii Ognev (RoboCup-FEI manipulation team).

**Status: draft.**
Field names and outcome codes may change until the team and the AI team agree on them.

## Overview

The package contains only interface definitions, so a client can build against it without MoveIt or a simulator.
The names are hardware-agnostic; robot specifics come from the server configuration.

| Interface | Kind | Purpose |
| --- | --- | --- |
| `GoToPosition` | action | Move the TCP to the best of several candidate poses, or the arm to a named pose |
| `TcpCandidate` | message | One candidate TCP pose with a quality estimate |
| `Outcome` | message | How an action ended: code and readable message |

`Grip` and `Release` will be added here.

## GoToPosition

A goal holds exactly one of these:

- `candidates`: TCP poses, each with a unique non-empty `id`, a frame in `header.frame_id` and a unit quaternion.
  The server transforms each pose with TF at `header.stamp`; a zero stamp means the latest transform.
  It plans to the candidates within one shared time budget from its configuration and moves to the best plan found.
  The best is chosen by a combined cost of motion convenience and the candidate's `quality`; the formula is not settled yet.
  `quality` is higher-is-better, and NaN means no estimate; set NaN explicitly, because an unset field is 0.0.
- `named_target`: the name of a joint-space pose stored in the server configuration.

Anything else ends with `INVALID_GOAL` and no motion.
The result reports the chosen candidate in `selected_candidate_id`, so the caller can send the matching follow-up pose, for example the grasp pose after its pre-grasp pose.
The action has no feedback in v1.

| Profile | Motion | Allowed with |
| --- | --- | --- |
| `PTP` (default) | joint-space motion | candidates, named target |
| `LIN` | straight TCP line, the whole line or a failure | candidates |

Every motion ends at rest.
The server never grips, releases or lifts as part of `GoToPosition`.

## Outcomes and action status

Every accepted goal ends with a result that carries an `Outcome`.
A goal that arrives while another action runs is accepted and immediately aborted with `BUSY`; it never runs, and the running action is not affected.
The action status agrees with the code: `SUCCEEDED` with `SUCCESS`, `CANCELED` with `CANCELED`, `ABORTED` with every other code.
Read the code from the result body also when the status is `ABORTED`.
`Outcome.message` is for people and logs; do not parse it.

## Build

```bash
colcon build --packages-select manipulation_interfaces
```
