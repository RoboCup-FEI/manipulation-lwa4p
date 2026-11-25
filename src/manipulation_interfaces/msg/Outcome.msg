# How an action ended. Every accepted goal gets one, whatever its action status.
# The status agrees with the code: SUCCEEDED with SUCCESS, CANCELED with CANCELED,
# ABORTED with every other code.
# Draft values (Viktor's proposal section 7.2, simplified); the team settles the final list.

uint8 SUCCESS=0

# The caller cancelled the goal; the arm stopped where it was. Not a failure of the skill.
uint8 CANCELED=1

# The goal breaks the action's rules; nothing moved. Do not retry it unchanged.
uint8 INVALID_GOAL=10

# Another action is running; it is not affected. This goal never runs.
uint8 BUSY=12

# The server found no trajectory to any candidate, or to the named target, within its
# planning budget; nothing moved. All candidates of the goal have been considered.
# `message` says why, e.g. no IK solution, a collision at the target or on the way,
# a singularity on a LIN line.
uint8 PLANNING_FAILED=20

# Grip only: the grip finished, but there is no evidence of an object between the fingers.
uint8 GRIP_FAILED=30

# The arm or the gripper failed while executing a command; the arm may be anywhere on the path.
uint8 EXECUTION_FAILED=40

# An unexpected error inside the server left it in an uncertain state.
# Every later goal ends with this code too, without motion, until a person restarts the server.
uint8 INTERNAL_ERROR=59

uint8 code

# A readable cause, for people and logs. Callers must not parse it.
string message
