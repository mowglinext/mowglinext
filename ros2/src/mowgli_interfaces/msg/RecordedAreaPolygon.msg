# RecordedAreaPolygon - one recorded area's outer boundary ring, reduced to
# just what a consumer needs for an "is this point inside any recorded
# area?" test. Today's one consumer: the LiDAR-ignore-corridor filter's
# area-side restriction (costmap_scan_filter_node, mowgli_localization,
# see its file header comment item 3) - a corridor's width_m only ever
# suppresses LiDAR on the side of the drawn line that overlaps a recorded
# area, since that is the only side the robot is ever physically on.
#
# Deliberately NOT the full MapArea - no obstacles, no name, no proposals:
# those would only add coupling this consumer doesn't need. Obstacle holes
# are NOT subtracted from the ring either (a known, accepted simplification
# - see RecordedAreaPolygonArray.msg).

geometry_msgs/Polygon area
