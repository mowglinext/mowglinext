# MapObstacleInfo - identity for ONE obstacle of a MapArea.
#
# Index-aligned with MapArea.obstacles: obstacle_info[j] describes
# obstacles[j]. The geometry is NOT repeated here so the two can never
# disagree. Publishers that predate this field leave obstacle_info empty;
# consumers must then fall back to name="", source=SOURCE_USER,
# pending=false, id=0 (a plain user-drawn keepout).

# Free-text label shown to the operator ("apple tree", or the auto-generated
# evidence line of a wheel-slip dig proposal). May be empty.
string name

# Provenance - the thing that makes a machine-generated keepout auditable.
uint8 SOURCE_USER=0        # drawn by the operator in the GUI / imported
uint8 SOURCE_TRACKER=1     # promoted from an /obstacle_tracker observation
uint8 SOURCE_DIG=2         # proposed by the wheel-slip dig detector
uint8 source

# True while this obstacle is only a PROPOSAL. A proposal is NOT applied: it
# is not in the keepout mask, not a NO_GO cell, not a coverage hole and not in
# areas.dat, and it does not survive a restart. It is listed in
# MapArea.proposed_obstacles (never in MapArea.obstacles) until the operator
# accepts it with ~/promote_obstacle{pending_id} - which applies AND persists
# it - or discards it with ~/discard_obstacle. Always false for an entry of
# MapArea.obstacle_info.
bool pending

# Session-scoped handle used by ~/promote_obstacle and ~/discard_obstacle.
# Stable only until map_server_node restarts. 0 for obstacles that were
# loaded from disk (nothing to accept or discard).
uint32 id
