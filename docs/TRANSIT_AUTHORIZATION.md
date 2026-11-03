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
rings are forbidden and `scale.x` is their explicit clearance margin in metres.
`dock_body` is forbidden in the same way, with the same margin, but is the one
ring a START may lie inside, see below; its `scale.y` says how far the dock pose
lies ahead of the ring's rear edge. An empty array invalidates transit during map
replacement; the debounced keepout rebuild publishes the complete snapshot.
No timestamp is used as a freshness timeout or an observation identity.
Replacing the area list retains the separately calibrated dock pose, corridor
and forbidden dock body; the GUI's clear/add/save sequence does not remove it.

Only start/goal terminal legs can leave the union: at most 0.15 m outside,
connected to an authorized grid anchor within 0.30 m of the endpoint. An
outside endpoint must have a uniquely nearest boundary location, enter the
union of rings sharing that location once, and remain in it until its anchor.
Adjoining or overlapping rings with the same closest boundary point represent
one intended boundary. Distinct equally near boundary points are ambiguous,
even when their rings connect elsewhere, and reject the terminal connection.
Terminal legs never create search cells.
Starts/goals on polygon edges connect to an interior grid anchor even when
grid alignment puts the endpoint cell centre outside. Obstacles still block
these legs. A missing connection fails planning without unrestricted fallback.

The dock body is the physical station: from the docked rear bumper to past the
docked nose (`robot_config_util.dock_body_extents`, -0.12 to +0.68 m along the
dock axis with the shipped chassis, 0.55 m wide), lethal in the keepout mask with
the obstacle body band. Until 2026-11 it lay on the lawn BEHIND the docked robot
(-0.80 to 0), so transit routed under the real station.

A start in the dock body or its margin is not refused: a failed docking stops
there, and refusing it strands the robot (field 2026-10-16: every transit
`START_OCCUPIED` until the operator pressed Play). Such a start is judged without
the dock body and without the mask's lethal cells for it, and connected to the
nearest clear, authorized, obstacle-free cell within 1.0 m by an exit leg that obeys every
other obstacle, the authorized union and the obstacle costs beyond the dock zone. With no such
cell the start stays `START_OCCUPIED`.

The exit never closes on the station head. The planner recovers the dock
frame from the body ring, the `dock_corridor` ring directly behind it (+x from
the corridor through the body toward the station) and `scale.y` (x = 0 is the
dock pose). With not exactly one body and one adjacent corridor in the snapshot
the start stays `START_OCCUPIED`. From anywhere up to the head the exit cell must
lie no further along +x than the start, so the leg goes backwards (toward the
corridor) or sideways; from beyond the head it must lie no nearer to it. The
transit controller may pivot in place before the leg, to a heading sampled along
the path rather than the leg, so on the approach axis (within the body's width,
short of its far end) the bound is heading-independent: the disc the footprint
sweeps around the start (radius = farthest footprint vertex) must end
`kStationClearance` (0.10 m) short of the head, taken to begin at the docked
robot's nose (footprint front reach ahead of the dock pose). With the shipped
footprint (front 0.53 m, sweep radius 0.597 m) that admits starts at least
0.167 m behind the dock pose; nearer starts facing the station stay
`START_OCCUPIED`. Neither docking nor undocking ever needs that plan, so the
refusal blocks nothing: within `dock_no_pivot_distance_m` (slightly wider than the
planner's limit) `DockRobot` skips the staging leg and drives straight in, with
docking_server reversing to staging for its retry, and the `BackOutOfStation`
branch reverses out on a start without charge. A robot with its rear to the
station has no pivot to make and leaves straight ahead. Goals in the dock zone,
the search grid and every path segment after the exit leg keep the whole ring and
its margin forbidden, so a transit can leave the dock zone but never enter or
cross it.

Other fleet members' stations arrive as `peer_dock` rings (`/fleet/peer_docks`, see
`docs/MULTI_ROBOT.md`), forbidden with the body margin even while their robot is away.
They can never block our own dock: a ring overlapping our dock body or corridor is
ignored with a warning, and one whose margin would reach our corridor keeps only the
object (`peer_dock_rings.hpp`). A start in a peer's margin leaves by the same bounded
leg, but the object itself stays blocked for it.

Coverage plans around the same ring: `coverage_server` reads `dock_body` from this
snapshot and grows it by `obstacle_margin`, as a hole when it lies inside the area
and a notch when it crosses the recorded line (an area with no overlap plans
bit-identically). It fails a plan rather than guess if no snapshot ever arrived.

Each search edge and every final path segment is checked against exact polygon
crossings and crossed obstacle-cost cells. Smoothing that leaves the union
falls back to the validated raw authorized path. A geometry change during
planning discards that result. Explicit BoundaryGuard recovery selects
`BoundaryRecovery` through `navigate_inside_boundary.xml`; its existing
keepout-disable/re-enable, timeout, cancellation and bounded BackUp paths
remain separate from ordinary transit.

Authorization preparation rasterizes polygon interiors by row and tests exact
boundary distances in the affected cell windows. The mask and crossing scratch
buffers belong to one plan and are never reused by later plans; the revision
check still discards a result if geometry changes during its plan. A negative
bounding-box connectivity preflight can reject separated authorized areas before
search. Overlapping boxes still require the exact search, including current
obstacle costs. These optimizations do not change the 8-second planning budget.
See [planner performance evidence](TRANSIT_PLANNER_PERFORMANCE.md) for reproducible
benchmarks and the remaining Raspberry Pi 4 validation.

Stop the mission before editing or replacing a map. Invalidation rejects new
plans; a controller already following a path may retain it until replanning.
Controller deviations, autonomous backoff and bounded recovery/docking
execution are tracked in [#924](https://github.com/mowglinext/mowglinext/issues/924).

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
