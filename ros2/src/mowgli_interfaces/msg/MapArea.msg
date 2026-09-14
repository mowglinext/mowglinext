string name
geometry_msgs/Polygon area
geometry_msgs/Polygon[] obstacles
bool is_navigation_area

# Per-obstacle identity (name / provenance / pending flag), index-aligned
# with `obstacles`. Empty when the publisher predates the field, or when
# nothing but bare geometry is known - treat a missing entry as an unnamed
# user-drawn keepout. See MapObstacleInfo.msg.
MapObstacleInfo[] obstacle_info
