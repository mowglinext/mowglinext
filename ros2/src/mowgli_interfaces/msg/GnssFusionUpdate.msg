# Facts about a GNSS factor committed into a published fusion_graph estimate.
# This is not motion permission or a bound on true position error.
# Receiver quality and physical observation identity still come from the
# original /gps/status association with applied_fix.

# Exact associated /odometry/filtered_map publication epoch in map.
std_msgs/Header header

# Fusion process identity, not a GNSS receiver/source incarnation.
string producer_incarnation
# Changes on reset/reseed/load, not ordinary sliding-window maintenance.
uint64 graph_epoch
uint64 estimate_revision

# True only after the associated odometry and owned TF have been sent.
# False invalidates publication support; startup/reset never reuse old facts.
bool pose_published
# Actual TF support epoch, which may be ahead of header.stamp due to TF lead.
builtin_interfaces/Time tf_support_stamp

# Original immutable NavSatFix whose factor successfully committed.
# Seed priors alone do not set this. Republishing never refreshes its stamp.
# Missing/evicted source data must leave has_applied_fix false.
bool has_applied_fix
sensor_msgs/NavSatFix applied_fix
uint64 applied_revision
