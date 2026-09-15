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

## Phase 3 — Coordinated mowing

Enabled per fleet with the DB flag `fleet.coordination.enabled` (default off).
Everything below is inert while it is off.

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

- The coordinator publishes peer poses into the local ROS graph through
  foxglove `clientPublish` as `geometry_msgs/PoseArray` on `/fleet/peers`
  (map frame, 2 Hz, empty when alone).
- `fleet_peer_obstacles_node` (Python, `mowgli_bringup/scripts`) turns each
  peer pose into a ring of points of radius `fleet_peer_radius_m` (0.6 m) at
  z = 0.3 m and publishes `sensor_msgs/PointCloud2` `/fleet/peer_obstacles` at
  5 Hz continuously (an empty cloud when there are no peers, so a persisting
  costmap source never goes stale).
- Nav2: the LiDAR overlay adds a `fleet_peers` observation source to the LOCAL
  `obstacle_layer` (marking only, no raytrace clearing). The no-LiDAR overlay
  adds a dedicated `fleet_layer` (an `ObstacleLayer` under a different name;
  CI forbids `obstacle_layer` + `static_layer` together) to both costmaps.
  The global costmap in the LiDAR variant is left alone because FTC treats
  anything lethal there as "not an obstacle".
- **Yield rule** (the primary safety mechanism; no BT change): when two
  members are within `fleet_yield_distance_m` (3.0 m) and both AUTONOMOUS, the
  greater-`robot_id` member is sent `COMMAND_STOP`; once the distance exceeds
  5.0 m for 3 s it is sent `COMMAND_START`, which resumes at its saved cursor.
  A yielded robot reports IDLE, so the other never sees it as a competitor.

## Status (2026-09-15)

All three phases are implemented on this branch and unit-tested; **none has
been field-tested with two real mowers yet**.

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
3. Drive the two within 3 m on transit: the lower-priority one holds
   (`COMMAND_STOP`), the local costmap shows the peer ring, and it resumes
   after the 5 m / 3 s hysteresis.
4. `/fleet/peers` actually reaches `fleet_peer_obstacles.py` through
   foxglove `clientPublish` (JSON encoding of `geometry_msgs/PoseArray`).

Known limits: partitioning is per AREA (a single-area lawn is not split);
priority is the lexical order of the generated `robot_id`, not configurable;
the yield rule needs a GPS fix on both robots; a peer that goes offline keeps
its last completed areas in the memory until the 12 h TTL or "Start fresh".

Fleet node parameters (`fleet_peer_obstacles.py`: `peer_radius_m` 0.6,
`ring_points` 24, `publish_rate_hz` 5, `peer_timeout_s` 5, `point_height_m`
0.30) are node defaults only, not template keys — change them in the launch
file if a site needs to. Coordinator knobs live in the GUI DB
(`fleet.coordination`) and on the Fleet page.
