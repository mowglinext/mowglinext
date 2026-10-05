# Transit authorization

Normal `GridBased` transit uses `mowgli_nav2_plugins/AuthorizedTransitPlanner`,
the pinned Smac 2D search with continuous edge checks against the closed union
of recorded mowing/navigation polygons and the explicit dock corridor.
It copies the live global costmap into a private view; shared outside body
slack, inner soft costs, local collision protection, coverage paths and pivot
geometry are retained. Drawn/promoted obstacles and the dock body are carried
in the geometry snapshot as well, so a newly rebuilt boundary cannot bypass
an obstacle while the costmap catches up.

`/map_server_node/transit_geometry` is a reliable, transient-local
`visualization_msgs/MarkerArray` containing one complete snapshot. Marker
namespaces `area` and `dock_corridor` are authorized outer rings; `obstacle`
rings are forbidden and `scale.x` is their explicit clearance margin in metres
(zero for the dock body). An empty array invalidates transit during map
replacement; the debounced keepout rebuild publishes the complete snapshot.
No timestamp is used as a freshness timeout or an observation identity.

Only start/goal terminal legs can leave the union: at most 0.15 m outside,
connected to an authorized grid anchor within 0.30 m of the endpoint. An
outside endpoint must have a uniquely nearest intended polygon, enter it once,
and remain in it until its anchor. Terminal legs never create search cells.
Starts/goals on polygon edges connect to an interior grid anchor even when
grid alignment puts the endpoint cell centre outside. Obstacles still block
these legs. A missing connection fails planning without unrestricted fallback.

Each search edge and every final path segment is checked against exact polygon
crossings and crossed obstacle-cost cells. Smoothing that leaves the union
falls back to the validated raw authorized path. A geometry change during
planning discards that result. Explicit BoundaryGuard recovery selects
`BoundaryRecovery` through `navigate_inside_boundary.xml`; its existing
keepout-disable/re-enable, timeout, cancellation and bounded BackUp paths
remain separate from ordinary transit.

Stop the mission before editing or replacing a map. Invalidation rejects new
plans; a controller already following a path may retain it until replanning.

## Physical acceptance

`HARDWARE_REQUIRED`: deterministic planner tests do not establish controller
tracking or robot/field safety. Before enabling blades or release, record the
exact repository SHA, image digest, firmware binary hash, submodule gitlinks,
robot unit, chassis/receiver/driver revisions, datum and saved map coordinates.
Use a supervised, flat test area with blades removed, emergency stop verified,
collision protection enabled, and an exclusion zone around the mower.

Create two separated authorized areas whose shared slack overlaps, joined by
a navigation detour. Command blade-off transit between the areas and record
`/plan`, localization and actual robot position. Pass: the planned centre path
stays inside the polygon union except for the stated bounded terminal legs,
the controller follows the detour, and no boundary emergency occurs. Remove
the navigation connection: planning must reject crossing. Repeat with boundary
starts/goals, a narrow navigation connection, dock staging and explicit
boundary recovery; verify keepout is re-enabled after recovery success,
failure and cancellation. Verify existing edge coverage and pivot geometry
on that same baseline. Stop immediately on unexpected motion or protection
failure. Neither hardware nor field acceptance has been performed here.
