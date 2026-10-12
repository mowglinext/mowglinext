# The FULL current set of recorded working + navigation area outer
# boundaries, republished on every area-list change (add/clear/load/datum
# migration) so a late-starting subscriber (costmap_scan_filter_node)
# always has the latest set without a service round-trip - same
# always-latest, transient_local shape as LidarIgnoreCorridorArray.
#
# Obstacle holes are NOT subtracted from each area's ring: a beam landing
# inside a mapped obstacle that also happens to be within a LiDAR-ignore
# corridor's reach is still treated as "on the area side". Getting this
# exactly right would need a point-in-polygon-with-holes test for a
# consumer that only needs a coarse area/not-area split - not worth the
# added complexity today.
std_msgs/Header header
RecordedAreaPolygon[] areas
