# One target pose of the tool centre point (TCP), with the producer's quality estimate.

# Unique and non-empty within one goal; the result reports it back.
string id

# The pose of the server's TCP frame. header.frame_id is required, any TF frame.
# The server transforms it with TF at header.stamp; a zero stamp means the latest transform.
# The orientation is required: a unit quaternion, never a placeholder.
geometry_msgs/PoseStamped tcp_pose

# The producer's estimate, higher is better; NaN = no estimate.
# Set NaN explicitly: an unset field is 0.0, which counts as a real estimate.
float64 quality
