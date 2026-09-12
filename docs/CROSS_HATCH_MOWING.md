# Cross-hatch mowing

Settings → Mowing → **Cross-hatch mowing** saves `mow_cross_hatch` (default
`false`). Restart ROS2 after changing this setting. It works with both Auto
and a fixed Mow Angle. This rotates the stripe layout, not the blade motor.

Each area keeps its own orientation. Its first coverage session uses the normal
angle, its next uses that angle +90°, then it returns to normal. Fixed 25°
therefore gives 25°, 115°, 25°. Area-select mowing advances only that area;
an area that was not mowed keeps its next direction. A whole-lawn session can
therefore use different offsets in different areas.
Auto first resolves its normal angle for each interior polygon, then applies the
session's 0°/90° offset. It does not optimise the rotated angle back to the original.
Boundary passes, obstacle boundaries and clearance settings are unchanged.

The **Next stripe direction by area** panel shows each area's current and next
orientation. Change **Next mow** to choose its next orientation explicitly.
These choices save immediately in ROS2, without restarting it, and do not change
an active or paused plan. Fixed angles are shown in degrees; Auto is shown as
**Auto** or **Auto + 90°**, since its exact base angle is resolved per polygon
at planning time. Values use the running stack's base angle, not unsaved edits.
The cross-hatch enable toggle and base-angle setting still require save/restart.
Connection or persistence failures are displayed rather than reported as saved.

Each area's orientation and any next override are saved with `coverage_resume.txt` in the existing maps volume.
Pauses, detours, low-battery recharge, replanning and ROS2/host restarts retain it.
A real `EndSession` advances an area's phase if coverage was attempted, including a run later
abandoned and docked. This is latched before the blade-service availability check;
it records a coverage attempt, not a confirmed blade start or physical rotation.
Repeated session-end calls, failed plans, manual mowing and
sessions with cross-hatch disabled do not consume a phase. Resuming after STOP
continues the session. Clearing coverage progress alone preserves its orientation.
The first plan for an area latches the setting for that session, so changing the toggle does
not rotate an interrupted session when ROS2 restarts.

At session end, the old resume snapshot is removed **before** atomically writing
phase-only metadata. A failed write can lose phase history, but cannot leave the
old START snapshot behind after successful removal. Metadata cannot trigger
automatic startup. Users who never enabled cross-hatch keep no completed phase
history: their normal session end only removes the resume file. A temporary
base-direction record is saved with normal progress so enabling cross-hatch
cannot rotate an interrupted disabled session.

If persistence is disabled (`coverage_resume_path` empty), automatic orientation
state survives only in memory. Explicit **Next mow** edits fail and roll back:
the UI never reports an unsaved override as saved. First-plan and coverage-start
write failures log the area and path. Missing/deleted state starts with the normal angle again. Map or
base-angle edits can still change planned geometry; existing resume fingerprints
invalidate a cursor if its path changes.
Phase history uses ROS area indices, like the existing coverage resume state.
Deleting/reordering areas can reassign those indices; review the next directions
after restructuring the map. The GUI map stream carries the original ROS IDs
alongside its filtered mowing-area list, so navigation areas cannot shift a
direction edit or area-select mowing command to another lawn.
Obstacle promotion resolves through the same IDs, including its confirmation
label; missing ID metadata disables promotion rather than guessing an area.

Coverage-start bookkeeping is independent of availability of the blade service.
It means a coverage run was attempted, not that physical blade rotation was
verified. Repeated calls consume at most one phase per session. Failed plans
retain their direction and log the affected area and override advice. After
three distinct failed sessions, an additional warning identifies the retained
orientation. Retries within a session do not increase this count; coverage start
or an explicit override clears it. The count persists with phase metadata and
uses the same area-index identity (review it after deleting/reordering areas).

Orientation requests enter a bounded queue and are processed before BT ticks.
Both reads and writes validate the original index with the ROS map service;
missing/navigation-only areas and unavailable validation are rejected. Writes
reply successfully only after persistence succeeds. SD I/O remains synchronous
on the BT owner, like other resume writes; this is not a disk-worker redesign.

## Auto-angle correction

For interior cells over 400 m², Auto uses the longest boundary segment to avoid
an expensive search on the Pi. `atan2` can return a negative heading; previously
that was mistaken for the Auto sentinel and ran the search anyway. The edge
heading is now normalised modulo 180° before testing the sentinel. Smaller cells
continue to search in 5° steps. The longest-segment heuristic itself is unchanged.
Perpendicular generation uses the resolved fixed/large-Auto angle directly.
Exhaustive Auto derives its angle from the longest finite, non-degenerate swath;
an empty or tangent first clip cannot silently select absolute 90°. Unusable
directions and empty rotated output are reported through planning diagnostics.

Rotating a fixed angle by 90° and selecting that rotated angle manually produce
the same full driven path. Boundary definitions are unchanged, but row ends
meet different boundary edges, so both orientations exercise the continuous
path, clearance, swept-footprint and fragmentation guards. Those tests exposed
straight fallback joins exceeding the existing 120° sharp-turn check. Before retaining such a
join, the connector now checks the other Dubins candidates inside the **same**
boundary and radius limits. Ordinary right-angle pivot joins retain their prior
behaviour. Curved alternatives can add travel distance; no clearance tolerance
or test turn threshold was enlarged.

## Integration and verification

`PlanCoverage.action` adds `perpendicular` to its goal. The behavior tree and
coverage server must be rebuilt/upgraded together; ROS action type compatibility
changes. `CoverageOrientation.srv` adds read/set-next access on
`/behavior_tree_node/coverage_orientation`, exposed through the existing GUI
service proxy. Upgrade the GUI backend/frontend with ROS2 for the new controls
and area-ID metadata. Go service bindings and the virtual-map TypeScript type
are regenerated; the firmware protocol is unchanged.

Tests exercise negative edge headings, both polygon windings, perpendicular
fixed/small-Auto/large-Auto geometry, identical boundary rings, actual BT action
goals, selected-area isolation, mixed whole-lawn phases, session reset, persisted
restart/overrides, progress reset, failed/disabled sessions, service write failure,
launch injection and the GUI toggle. Browser tests cover the actual settings
controls, persistence across reload, unchanged active orientation, real area IDs
and unavailable ROS2. PR screenshots use mocked robot data. This changes physical mowing routes and
requires the usual monitored commissioning before deployment. No mower is flashed
or operated by this PR.

## Review validation and commissioning

Software baseline: `25e1a6688908591100c6d2c056e0811880d1eb74`, ROS2 Kilted,
Fields2Cover `884d895b59192882476e986ba44ea9143a06a6a9`, non-root Linux builds.
The 55 planner tests include both orientations of the recorded boundary,
swept-footprint, zero-slack, rings-off and fragmentation fixtures. The recorded
area uses 0.16 m operation width, 0.18 m headland/turn radius, 0.15 m inset/minimum
turn radius and 0.03 m sampling step. Both orientations produce one continuous
subpath, zero sampled points outside the planning boundary and a maximum local
turn of 91.05°. Driven length is 191.41 m base / 261.85 m perpendicular: safe
curved alternatives can cost extra travel, and field tracking is not established
by these geometry checks.

`HARDWARE_REQUIRED`: commissioning of the revised paths has not been performed.
Before running, record the deployed repository SHA, ROS/GUI image digests,
firmware source/image, submodule gitlinks, robot unit and receiver/driver revision.
Use a secured clear area, working safety interlocks and an operator with physical
stop access. Start with blades disabled. Test the recorded/concave and rings-off
cases in both orientations, then stop/resume and restart mid-session. Pass means
the current direction remains stable, only a real completed/abandoned session
advances used areas, and the driven footprint remains inside the authorised
boundary without unexplained fragmentation. Powered cutting requires a separate
supervised run. These hardware/image baselines are not selected by this PR.
