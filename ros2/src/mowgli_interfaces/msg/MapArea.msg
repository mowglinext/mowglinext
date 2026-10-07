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

# Stable, persistent area identifier (mowglinext#637). Survives reorders,
# edits or deletes of OTHER areas - unlike the positional index used by
# GetMowingArea/StartInArea, which the GUI's edit/delete flow (a full
# clear_map + add_area rebuild) can reassign for every area at once.
#
# 0 means "not yet assigned": a request built before this field existed, or
# (only ever transiently, in-memory) an entry not yet persisted. A genuine
# area coming back from map_server_node's ~/get_mowing_area always has a
# non-zero id - map_server_node mints one on first save if the on-disk
# areas.dat predates this field.
#
# Callers adding a NEW area should leave this 0 and let map_server_node
# mint one. The one exception is the GUI's rebuild flow: when re-adding an
# area the operator did not touch, round-trip the id it already had -
# never invent one client-side - so unrelated areas keep their identity
# across an edit to a different one. See area_manager.cpp's on_add_area().
uint32 id

# Per-area coverage lines: optional overrides of the robot-wide mow_angle_deg
# and mow_direction settings (the Map page's "mowing lines" preview edits them).
# Opt-in by design: a flag per value, so the zero value of every field means
# "use the robot-wide setting". That keeps a request or a persisted map built
# before these fields existed meaning exactly what it meant before, and keeps
# "0 degrees" and "planner default winding" expressible as real choices.
#
# has_mow_angle: false = use the robot-wide mow_angle_deg. true = mow_angle_deg
# applies to this area: degrees in [0, 180) (a swath has no sense of direction),
# or any NEGATIVE value for AUTO (the convention of mow_angle_deg everywhere) -
# which pins this area to auto even when the robot-wide angle is fixed.
bool has_mow_angle
float64 mow_angle_deg
# has_ring_direction: false = use the robot-wide ring_direction. true =
# ring_direction applies to this area: 0 planner default, 1 clockwise,
# 2 counter-clockwise (same values as coverage_server.ring_direction).
bool has_ring_direction
uint8 ring_direction

# Where the route starts, as a point (map-frame metres) near the OUTERMOST headland
# ring: the planner snaps it onto that ring, on a straight side, and the route
# always begins there, so the outermost ring is driven first. Opt-in like the two
# above: has_start_point false = the planner's own start (the midpoint of the
# longest side of the outermost ring). Meaningless with the headland rings off.
# Re-projected with the polygon when the datum changes.
bool has_start_point
float64 start_x
float64 start_y
