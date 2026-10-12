# Multi-robot: fleet view and coordinated mowing

Design and status of running several MowgliNext mowers at the same time. Two
capabilities, built in three phases, each usable on its own:

1. **Identity** — every robot has a `robot_name` (and a generated `robot_id`).
2. **Fleet view** — any robot's GUI lists the other mowers, shows their state
   and position, and sends Play / Pause / Home to each of them.
3. **Coordinated mowing** — several mowers share ONE map and mow it at the same
   time without working the same area or driving into each other.

## Constraints the design is built around

- **Each robot is a complete, independent stack** on its own Pi: ROS2 container,
  `foxglove_bridge` (`:8765`, LAN-open, `clientPublish` + `services` enabled,
  no auth), GUI backend (`:4006`, no auth, CORS `*`), optional MQTT broker.
- **Robots cannot see each other over DDS, and must not.** Cyclone DDS is pinned
  to loopback with multicast off (`install/config/cyclonedds.xml`, issue #418).
  Every cross-robot exchange therefore goes over the GUI HTTP/WS API, robot to
  robot, with each GUI backend acting as the gateway into its own ROS graph.
- **No central server.** Any robot's GUI can be the fleet console; the fleet
  state lives on every member (peer list + coordinator memory in each GUI DB).
- **"Mowed" is a BT session set, not the progress grid.** Coverage completion is
  `completed_areas` in `coverage_resume.txt`; `mow_progress` is display-only
  and cannot be imported. Fleet completion is therefore shared at the AREA
  level through the BT, never by merging grids.
- **Areas have no id.** An area is its index in `areas.dat`; the fleet relies on
  every member holding an identical `areas.dat` (same order, same datum).
- **Peers cannot be pushed in as temporary keepouts** (only permanent
  `promote_obstacle` or the dig proposal exist), and lethal cells in the GLOBAL
  costmap are masked from FTC's obstacle detection. Peer avoidance therefore
  uses (a) the LOCAL costmap and (b) a pause/resume yield rule, not keepouts.

## Phase 1 — Identity

| Where | What |
|-------|------|
| `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` | `robot_name: "mowgli"` template default (Invariant 15) |
| `install/config/mowgli/mowgli_robot.yaml` | seeded `robot_name: mowgli` (Bucket A, listed in `check_config_drift.py` `INSTALL_SEED`) |
| `gui/asserts/mower_config.schema.json` | `hardware_settings.robot_name`, default `mowgli` — parity test passes because the template carries the key |
| GUI Onboarding step 1 + Settings → Hardware | free-text name next to the model picker |
| `gui/pkg/providers/fleet_identity.go` | `robot_id`: UUID generated once into the GUI DB key `fleet.robot_id`; `robot_name` read from the yaml on every request |
| `GET /api/fleet/identity` | `{id, name, version, datum_lat, datum_lon, api_version}` |
| HomeKit | accessory name = `robot_name` |

The installer does not prompt for the name (like `mower_model`, it is an
onboarding/GUI choice); nothing in compose consumes it. Container names and
`COMPOSE_PROJECT_NAME` stay fixed — the GUI, updater and installer hardcode
them, and renaming the project orphans the maps volume.

## Phase 2 — Fleet view

### Backend (`gui/pkg/providers/fleet.go`, `gui/pkg/api/fleet.go`)

- **Peer registry**: DB key `fleet.peers` = JSON `[{id, name, address}]`
  (`address` is `host:port` of the peer's GUI). `POST /api/fleet/peers
  {address}` dials `http://<address>/api/fleet/identity`, stores the peer, then
  registers *us* on the peer (`POST /api/fleet/peers/register {id, name,
  port}`; the peer stores our source IP + port). Membership is symmetric, so
  the fleet looks the same from every robot. `DELETE /api/fleet/peers/:id`
  removes both directions.
- **Live cache**: one `peerClient` per peer keeps a WebSocket to the peer's
  `/api/mowglinext/multiplex` (a Go client sends no `Origin`, so the peer's
  same-host check passes), subscribes `highLevelStatus`, `status`, `power`,
  `pose`, `gps`, `gnssStatus`, `emergency`, decodes the msgpack frames and
  caches the last message per topic. Reconnect with capped backoff; a peer is
  `online` when the socket is up and `highLevelStatus` is under 10 s old.
- **Snapshot**: `GET /api/fleet/robots` returns `self` + peers, each
  `{identity, online, last_seen, high_level_status, power, pose, gps,
  gnss_status, emergency}`. Self is served from the local `RosProvider` cache.
- **Commands**: `POST /api/fleet/robots/:id/call/high_level_control` — proxied
  to the peer's `POST /api/mowglinext/call/high_level_control`, or executed
  locally for self. Only `high_level_control` and `emergency` are proxied; every
  other route stays local (settings, docker, firmware, calibration).
- A second `RosProvider` is deliberately NOT created for peers: its constructor
  binds the teleop relay to `localhost:8766`, so remote joystick commands would
  drive the wrong mower, and its session tracker would write into the local
  history.

### Frontend

- New page `/fleet` (`gui/web/src/pages/FleetPage.tsx`): one card per robot
  (name, state, battery, GPS fix, online/offline, Play / Pause / Home, "Open
  GUI" link) and a fleet map with one marker per robot, projected with each
  robot's own datum. Data comes from `GET /api/fleet/robots` polled every 2 s.
- Peer management (add by address, remove) lives on the same page.
- The Map page (`MapPage.tsx`, full and compact) also draws every online
  peer: this robot's URDF silhouette in purple (`FLEET_PEER_COLOR`) with the
  peer's name, at its fused pose (`pose` = `/odometry/filtered_map`),
  re-projected from the peer's datum into ours (`utils/fleetPeers.ts`).
  `useFleetPeers` polls `GET /api/fleet/robots` every 2 s while there is a
  peer and every 15 s while the robot is alone.

## Phase 3 — Coordinated mowing

Enabled per fleet with the DB flag `fleet.coordination.enabled` (default off).
While it is off, the coordinator only clears its local BT's fleet assignment.
It does this on disable and after GUI startup, retrying at the two-second tick
cadence until acknowledged, then remaining silent. This reconciles stale
exclusions if the BT outlives a GUI restart; it sends no START/STOP commands.

### 3a. Shared map

"Push map to fleet" on the Fleet page reads the local areas (the same
`get_mowing_area` probe the map poll already does) and replaces every peer's
map through the peer's map-replace route. It refuses when a peer's datum
differs from ours by more than 1e-8°: `map_server` re-projects a foreign-datum
`areas.dat` on load AND shifts the robot's own dock pose by the datum
difference, so fleet members must share one datum. Docks stay per robot
(`dock_pose_*` in each robot's own yaml).

### 3b. Area assignment (BT + coordinator)

- New BT service `~/set_fleet_assignment` (`mowgli_interfaces/srv/
  SetFleetAssignment`: `excluded_areas[]`, `preferred_start_index`). The
  handler defers the write to the tick thread (same pattern as
  `clear_coverage_resume`) into a new `BTContext::fleet_excluded_areas` +
  `fleet_preferred_start`. `GetNextUnmowedArea` skips excluded areas in both
  skip loops and rotates its ascending scan to start at the preferred index.
  A pass that ends because the area became excluded mid-mow is exempt from the
  no-progress budget, like a guard halt.
- New BT topic `~/coverage_session` (`mowgli_interfaces/msg/CoverageSession`:
  `session_active`, `current_area`, `completed_areas[]`, `attempted_areas[]`),
  published with `HighLevelStatus` at 1 Hz, so peers can see what this robot
  has finished this session (the resume file is not exposed otherwise).
- **Coordinator** (`gui/pkg/providers/fleet_coordinator.go`, runs on every
  robot for its own BT, leaderless): every 2 s it computes
  `excluded = ∪ peers.current_area (online, AUTONOMOUS) ∪ fleet_completed` and
  pushes it to the local BT. `fleet_completed` is the union of every member's
  `completed_areas` seen since the fleet session started, remembered in the DB
  (`fleet.session.completed`) so an area stays done after its robot docks and
  clears its own session. It resets on the Fleet page's "Start fresh" (which
  also calls `coverage_clear_resume` on every member) or after 12 h.
  `preferred_start_index` = this robot's rank among online members (sorted by
  `robot_id`), so idle robots do not all pick area 0.
- **Conflict rule**: if two robots report the same `current_area`, the one
  with the greater `robot_id` yields: the area is added to its exclusions, its
  `FollowStrip` halts with the cursor saved and `GetNextUnmowedArea` moves on.
- Limitation: partitioning is per AREA. A single-area lawn cannot be split
  between robots yet (sub-path splitting needs byte-identical plans on every
  member and is a later phase).

### 3c. Mutual avoidance

Two layers, and the first one is the primary protection since 2026-10-10:

**Each robot carries the others in its costmaps, always.** The `FleetProvider`
runs a *peer feed* (`gui/pkg/providers/fleet_peer_feed.go`) whenever at least
one peer is registered — independently of the coordination toggle — at 5 Hz:
it takes every online peer's FUSED pose (`/odometry/filtered_map` mirrored
over the peer's GUI, so the body centre with its heading, not the antenna
fix), re-projects it through the peer's datum into our map frame (the peer's
identity, refreshed every 60 s, carries its datum), drops any sample older
than 3 s, and publishes `geometry_msgs/PoseArray` on `/fleet/peers` through
foxglove `clientPublish`. A peer whose fused pose is stale but whose fix is
fresh is placed from the fix with an all-zero quaternion ("no heading").
`fleet_peer_obstacles.py` (`mowgli_bringup/scripts`, respawned by the launch)
turns each pose into a DENSE block of points covering the chassis footprint
(the same derived triple Nav2 uses for our own footprint, injected by
`full_system.launch.py`, plus a 0.15 m margin; a disc when the heading is
unknown) and publishes `sensor_msgs/PointCloud2` `/fleet/peer_obstacles` at
5 Hz continuously — an empty cloud when alone, so no costmap source ever
goes stale. Nav2 then treats a peer like any obstacle:

- LiDAR overlay: `fleet_peers` source on the LOCAL `obstacle_layer` (FTC's
  detection source — a peer on the swath is skirted or stopped for) AND on the
  GLOBAL `obstacle_layer` (Smac routes a transit around a peer stopped on the
  path instead of planning through it and waiting on the local collision
  check). The global placement was unsafe while FTC's zone mask treated
  global-lethal cells as non-obstacles; that mask is gone (root CLAUDE.md
  Invariant 5). The LiDAR itself also sees the other body, so collision_monitor
  keeps its scan source only.
- No-LiDAR overlay: a `fleet_layer` (`ObstacleLayer` under a distinct name) in
  BOTH costmaps, and — new — the consumers are ON like in the LiDAR variant:
  FTC `check_obstacles` + `enable_obstacle_deviation`, RPP
  `use_collision_detection`, and a collision_monitor whose only source is the
  fleet cloud (`pointcloud` type, an active velocity-projected approach gate,
  `source_timeout: 0.0` so a dead fleet node never halts a lone robot). On a
  lawn without peers the layer is empty and nothing changes.

The Fleet page shows the feed state ("Peers in costmap", count, rate, last
error) so an operator can tell whether the other mower actually reaches this
robot's obstacle map.

**Yield rule** (the coordinator, only while coordination is ON): when a peer
WITH PRIORITY (smaller `robot_id`) is autonomous within `yield_distance_m`
(3.0 m) of us while we are autonomous, we are sent `COMMAND_STOP`; once no
such peer has been within `resume_distance_m` (5.0 m) for 3 s we are sent
`COMMAND_START`, which resumes at the saved cursor. Distances use the same
map-frame positions as the feed (fused pose first, fix as the fallback,
3 s freshness; no datum → the rule stands down). Two bounds keep it from
deadlocking against the costmap avoidance above: a hold never lasts longer
than `yield_max_hold_s` (45 s — the priority robot may itself have stopped in
front of us on its own collision check, waiting for us to leave), and after
any resume no new yield is issued for `yield_cooldown_s` (20 s), so the
resumed robot gets to skirt the peer instead of stopping again at 3 m. The
coordinator ticks at 1 s.

## Status (2026-10-10)

All three phases are merged into `dev` (PR #615, 2026-09-21) and shown as
beta in the GUI. The first two-mower field test (2026-10) found the stop rule
alone unreliable: the peer feed only ran with coordination ON, at 2 s, from
the antenna fix without heading, and the no-LiDAR variant had no consumer for
the costmap at all — so the robot that was not told to stop had nothing to
make it avoid the other. The 3c redesign above (always-on 5 Hz footprint
feed, consumers enabled in both variants, bounded yield) is the response;
its own field check is still owed.

| Piece | Where | Verified by |
|-------|-------|-------------|
| `robot_name` identity, `robot_id` | template / seed / schema / onboarding / `fleet_identity.go` | `TestSchemaDefaultsMatchTemplate`, `check_config_drift.py`, `fleet_test.go` |
| Fleet view (registry, mirror, proxy, `/fleet` page) | `gui/pkg/providers/fleet*.go`, `gui/pkg/api/fleet.go`, `gui/web/src/pages/FleetPage.tsx` | two in-process robots in `gui/pkg/api/fleet_test.go` (handshake, live WebSocket mirror, proxy, removal); vitest `utils/fleet.test.ts` |
| BT area assignment + yield | `mowgli_behavior` (`~/set_fleet_assignment`, `~/coverage_session`, `GetNextUnmowedArea`, `FollowStrip`) | 8 new gtests in `test_get_next_unmowed_area.cpp` (497/497 behavior tests green on Lyrical) |
| Peer obstacles | `fleet_peer_obstacles.py`, Nav2 overlays | `test_fleet_peer_obstacles.py`, `test_nav2_params.py` (fleet source placement) |
| Coordinator (exclusions, completed memory, peer poses, yield rule, map push) | `fleet_coordinator*.go`, `fleet_map.go` | `fleet_coordinator_test.go` (pure rules + `tick()` against the ROS mock) |

Field checks still owed before calling it done:

1. Two mowers, one map, coordination ON: each picks a different area (rotation),
   neither re-enters an area the other finished (completed memory), and the
   run ends with both docked and MOWING_COMPLETE.
2. Force a same-area pick (`~/start_in_area` on both): the greater-id robot
   yields mid-pass, saves its cursor and moves on; the other keeps mowing.
3. Coordination OFF, both mowing near each other: each robot's local costmap
   shows the other's footprint block (Foxglove: `/fleet/peer_obstacles` and
   the costmap), FTC skirts or stops for it, a transit routes around it, and
   the no-LiDAR collision monitor halts before contact.
4. Coordination ON, drive the two within 3 m on transit: the lower-priority
   one holds (`COMMAND_STOP`) and resumes after the 5 m / 3 s hysteresis; park
   the priority robot in front of it and check the 45 s max-hold resume + the
   20 s cooldown.
5. The Fleet page's "Peers in costmap" tag is green with the right count on
   both robots; `fleet_peer_obstacles.py` logs its first `/fleet/peers`
   message (foxglove `clientPublish`, JSON `geometry_msgs/PoseArray`).

Known limits: partitioning is per AREA (a single-area lawn is not split);
priority is the lexical order of the generated `robot_id`, not configurable;
the yield rule needs a GPS fix on both robots; a peer that goes offline keeps
its last completed areas in the memory until the 12 h TTL or "Start fresh".

Fleet node parameters (`fleet_peer_obstacles.py`: `peer_margin_m` 0.15,
`fill_step_m` 0.05, `publish_rate_hz` 5, `peer_timeout_s` 3, `point_height_m`
0.30) are node defaults only, not template keys — change them in the launch
file if a site needs to; the chassis extents are injected from the robot
config. Coordinator knobs live in the GUI DB (`fleet.coordination`) and on
the Fleet page.
