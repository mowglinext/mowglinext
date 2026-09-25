# The FULL current set of LidarIgnoreCorridor entries, republished on every
# add/clear/load so a late-starting subscriber (costmap_scan_filter_node)
# always has the latest list without a service round-trip - same
# always-latest, transient_local shape as /keepout_mask.
std_msgs/Header header
LidarIgnoreCorridor[] corridors
