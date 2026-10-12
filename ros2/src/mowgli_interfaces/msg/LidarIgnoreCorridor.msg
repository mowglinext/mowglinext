# LidarIgnoreCorridor - operator-drawn line along which LiDAR obstacle
# detection is suppressed within width_m/2, on BOTH the costmap and the
# collision_monitor path (costmap_scan_filter_node's corridor filter,
# mowgli_localization). Deliberately independent of MapArea geometry: the
# operator draws this wherever a real, physical, boundary-adjacent obstacle
# (e.g. a hedge the recorded boundary intentionally runs along) should stop
# being treated as something to avoid or stop for.
#
# SAFETY: this is the only map primitive that can make collision_monitor
# blind to something within its bounds - the drawn line's placement and
# width_m become the sole thing standing between the robot and whatever is
# physically there. See costmap_scan_filter_node.cpp's corridor filter.

# Free-text label shown to the operator. May be empty.
string name

# Polyline in the MAP frame, >= 2 points. Consecutive points define the
# segments the corridor filter measures perpendicular distance against - NOT
# a closed ring, despite the Polygon type: reused for consistency with every
# other persisted map geometry in this stack (same float32 precision, same
# map_server_node polygon_to_string/parse_polygon_string serialization) and
# because a Polygon is exactly "an ordered list of Point32" on the wire -
# nothing here relies on it being closed.
geometry_msgs/Polygon polyline

# Perpendicular ignore distance either side of the polyline, in metres.
# Clamped server-side by map_server_node (~/add_lidar_ignore_corridor) to a
# safety bound so a fat-fingered value cannot blind a large swath of the
# field - see area_manager's validation, not re-derived by any consumer.
float64 width_m

# Stable, persistent identifier, same convention as MapArea.id: 0 means
# "not yet assigned", map_server_node mints one on first save. Callers
# adding a NEW corridor leave this 0.
uint32 id
