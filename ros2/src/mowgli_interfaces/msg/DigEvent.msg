# DigEvent — the robot's wheels were slipping in place ("digging a hole").
#
# Published by hardware_bridge_node when its wheel-slip detector
# (mowgli_hardware/dig_detector.hpp) finds that the encoders claimed real
# travel while the GNSS-anchored fused pose barely moved. The bridge has
# already hard-stopped and begun a bounded reverse by the time this is
# published; consumers act on the LOCATION, not the motion.
#
# Two consumers, neither of which may ever block the robot's own pose:
#  * map_server records a PENDING PROPOSAL (MapArea.proposed_obstacles,
#    SOURCE_DIG). Nothing is stamped into the keepout mask, the coverage
#    plan or areas.dat - a single inferred dig is a proposal, not a change to
#    the operator's map. They accept it with
#    map_server_node/promote_obstacle{pending_id} (applied + persisted) or
#    drop it with map_server_node/discard_obstacle. The evidence below is
#    folded into the proposal's name so the operator can judge it.
#  * behavior_tree_node keeps the session's dig points; FollowStrip cancels
#    the active coverage goal, resumes the same unit past the spot and skips
#    the path poses inside the dig skip zone for the rest of the session
#    (mowgli_behavior/dig_skip.hpp) - the anti re-dig protection of #500.

std_msgs/Header header

# Where the robot was when the dig latched, in the map frame (this is the
# fused-pose position — the same signal that detected the dig).
geometry_msgs/Point position

# Evidence, carried for logging/telemetry and so consumers can apply their
# own confidence policy.
float64 wheel_distance    # encoder-claimed travel over the detection window [m]
float64 map_distance      # fused-pose travel over the same window [m]
float64 position_sigma    # fused-pose 1-sigma position uncertainty at latch [m]
