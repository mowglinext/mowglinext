string name
geometry_msgs/Polygon area
geometry_msgs/Polygon[] obstacles
bool is_navigation_area

# Per-obstacle identity (name / provenance / pending flag), index-aligned
# with `obstacles`. Empty when the publisher predates the field, or when
# nothing but bare geometry is known - treat a missing entry as an unnamed
# user-drawn keepout. See MapObstacleInfo.msg.
MapObstacleInfo[] obstacle_info

# PENDING proposals for this area (today: wheel-slip dig reports, SOURCE_DIG).
# A proposal is NOT applied: it is absent from `obstacles`, from the keepout
# mask, from the classification layer and from areas.dat, so it can never
# block planning and never becomes a hole in a coverage plan. It exists only
# so the operator can review it: ~/promote_obstacle{pending_id} applies and
# persists it (it then moves to `obstacles`), ~/discard_obstacle drops it.
# `proposed_obstacle_info` is index-aligned with `proposed_obstacles`.
geometry_msgs/Polygon[] proposed_obstacles
MapObstacleInfo[] proposed_obstacle_info
