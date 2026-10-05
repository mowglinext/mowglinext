# Garden-sized global planning grid

The global costmap stays fixed in `map` and covers the complete recorded garden,
including navigation areas and the dock body/approach corridor, with the map
server's existing 5 m padding. Its origin follows map edits, never the mower.
The local costmap remains rolling. Global resolution stays 0.08 m; the keepout
mask stays at its configured resolution (0.05 m by default).

`GardenKeepoutLayer` takes geometry and costs from one `/keepout_mask` snapshot.
It applies authorization after sensor inflation, preserving the existing soft
boundary bands and drawn-obstacle clearance. Startup, an empty mask, invalid
data, or an edit awaiting its settled rebuild closes the complete planning grid.
A geometry change also closes one update cycle while preceding sensor layers
resize; the following update rebuilds sensor obstacles and inflation before
authorization becomes current. Clearing recovery cannot erase the mask.
The standard `/global_costmap/keepout_filter/toggle_filter` service remains
available for explicit boundary recovery. Disabling boundary costs requires a
current valid mask, keeps geometry/readiness enforced, and cannot unlock an
unavailable or resizing grid. A new mask resets the exemption; re-enabling
closes the grid immediately until the next mask application.

Map loading/replacement and dock placement invalidate the latched mask before
rebuilding it. An empty mask means unavailable authorization, never a free map.
Map replacement still publishes only one complete raster after its 1.5 s settle
window. The map server now compares both size and center when fitting geometry,
so a same-size replacement at a different location recenters correctly.

Both geometry authorities enforce an explicit cell budget: map server
`max_grid_cells` and global `keepout_filter.max_cells`, each 4,000,000 by default.
The finer mask usually reaches its budget first. The map server keeps the
recorded polygons, reports the allocation error on the latched
`/map_server_node/planning_grid_error` string and ROS error log, and publishes
no traversable mask. Loading an oversized map reports failure. The global layer
also reports malformed or oversized masks in its error log and stays non-current
with lethal costs. Neither authority truncates the garden or changes resolution.
Advanced budget changes require validating actual available memory and latency.

Global `RecentObstacleLayer` rebuilds from current observation buffers each
update instead of keeping cells until the rolling window leaves them. LiDAR
marks persist for at most 2 s, with a 0.5 s stream update deadline; absent/stale
LiDAR prevents a current planning costmap. No-LiDAR fleet observations persist
for 1 s and are marking-only. Removed observations clear their marks and
inflation throughout the fixed extent; drawn mask obstacles remain lethal.
Local obstacle handling and collision-monitor settings are unchanged.

Map edits should be performed with autonomous motion stopped. This change gates
new planning against unavailable authorization; it does not cancel an already
dispatched controller path independently of the navigation behavior tree.

## Software checks

`test_garden_costmap` uses the real Nav2 layered costmap, inflation and
SmacPlanner2D. It checks distant goals in both directions, a detour beyond the
former 35 m axis limit, a 0.5 m navigation connection at unchanged resolution,
startup/resize/edit closure, recovery clearing, malformed/resource-limited
masks, and removed sensor marks including inflation. Marking/clearing and
marking-only cases represent the two global sensor configurations; they do not
constitute a sensor-driver or physical robot test.

Map-server tests cover dock-inclusive extents, same-size reload/recenter,
allocation refusal without coarsening, and closed masks during a burst of edits.
Configuration tests pin both overlays' fixed global/rolling local geometry and
bounded observation policy.

## Release hardware acceptance — HARDWARE_REQUIRED

No measurements from supported ARM hardware have been acquired. The weakest
supported board and unit must be identified before this can become
`HARDWARE_PENDING`; desktop tests do not establish ARM resource headroom.

Record the exact baseline: this branch's tested commit SHA, container image
digest, Nav2/grid-map source pins, all submodule gitlinks, firmware image/hash,
robot unit, SBC model/RAM/CPU governor, receiver/driver revision, installed robot
configuration, and checksum of the recorded area file. Use the least capable
supported production SBC, not an arbitrary desktop substitute.

With blades disabled, wheels physically inhibited, firmware e-stop available,
and a supervisor present, replay recorded map-frame poses and the matching scan
stream without commanding motion. Repeat with `use_lidar` true/false, the
representative largest real garden, a map just below the supported cell budget,
and one just above it. Request at least 30 `ComputePathToPose` plans in each
direction between dock and distant areas, including the longest authorized
corridor detour and narrowest connection. Replace/reload an area at a different
origin; remove a replayed obstacle/peer cloud and replan after its buffer expires.

Capture planner latency (median/p95/max), planner/costmap process CPU and peak
RSS, aggregate free memory/swap activity, costmap update duration, publication
message sizes/rate and ROS errors. Compare the same map with the previous image
where its goals fit. Pass criteria: every connected goal succeeds without an
out-of-bounds error; paths remain inside the keepout policy; masks/edits cannot
produce an enabled free bootstrap; removed marks/inflation disappear within
their 2 s/1 s buffer plus one global update; oversized geometry emits the explicit
budget error; planning finishes within configured `max_planning_time`; updates
keep up with configured frequency; no OOM or sustained swapping; costmap
publication stays at configured rate without starving navigation. Record actual
numbers and rejected cases before release. Physical driving/collision behavior
requires a separate supervised blade-disabled run on that exact baseline.
