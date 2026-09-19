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
