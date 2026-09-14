# Codemap: mowgli_interfaces

> The rosidl package that owns every MowgliNext `.msg` / `.srv` / `.action` definition plus five
> header-only helpers shared by several packages (WGS84↔ENU projection, `mowgli_robot.yaml`
> scalar splicing, GNSS status helpers, GPS motion-yaw fit, the coverage transit-gap constant).
> Ten ROS packages depend on it, and three out-of-tree generators (firmware `sync_ros_lib.py`,
> GUI `generate_go_msgs.sh` / `generate_ts_types.sh`) read its `msg/` and `srv/` directories.
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `ros2/CLAUDE.md`.

## Where to look
| Task | Start here |
|------|------------|
| Add / rename a message field | `ros2/src/mowgli_interfaces/msg/<Name>.msg`, then the three generators (see *Change coupling*) |
| Register a new `.msg`/`.srv`/`.action` so it builds | `ros2/src/mowgli_interfaces/CMakeLists.txt:17-58` (`msg_files` / `srv_files` / `action_files`) |
| Find who publishes / subscribes a type | *Runtime surface → Topics* below (file:line per producer/consumer) |
| High-level command codes (`COMMAND_START` = 1 … `COMMAND_STOP` = 8) | `ros2/src/mowgli_interfaces/srv/HighLevelControl.srv`; server `ros2/src/mowgli_behavior/src/behavior_tree_node.cpp:548`; semantics in `docs/claude/high-level-api.md` |
| BT state enum (`HIGH_LEVEL_STATE_*`) mirrored to firmware `HL_MODE_*` | `ros2/src/mowgli_interfaces/msg/HighLevelStatus.msg:1-5` ↔ `firmware/stm32/ros_usbnode/include/mowgli_protocol.h:418-422` |
| GNSS fix / RTK / capability bits | `ros2/src/mowgli_interfaces/msg/GnssStatus.msg` (constants), `include/mowgli_interfaces/gnss_status_utils.hpp` (`IsRtkFixed`, `BehaviorTreeRtkFixed`, `NormalizedQuality`) |
| Who publishes `/gps/status` (`GnssStatus`) | `sensors/gps/universal_gnss_topic_bridge.py:166-167` (`output_status_topic`), fed by `ros2/src/external/universal-gnss/gnss_ros2/src/receiver_node.cpp:739` |
| WGS84 → map-frame maths (Invariant 4) | `include/mowgli_interfaces/wgs84_projection.hpp` (`ToEnu` / `FromEnu` / `ReprojectEnu`) |
| Persist `dock_pose_x/y/yaw` into `mowgli_robot.yaml` without losing comments | `include/mowgli_interfaces/robot_yaml_scalar.hpp` (`UpdateDockPose`, `PersistScalar`, `SpliceScalar`) |
| Straight-line heading fit from GPS samples (dock-yaw calibration) | `include/mowgli_interfaces/motion_yaw_fit.hpp` (`FitMotionYaw`) |
| Blade-off transit vs blade-on join threshold (FollowStrip execution side) | `include/mowgli_interfaces/coverage_geometry.hpp:26` (`kSegmentTransitGapM = 0.6`) |
| Coverage plan payload (`segments`, `drivable_subpaths`, `full_path`) | `ros2/src/mowgli_interfaces/action/PlanCoverage.action`; server `ros2/src/mowgli_coverage/src/coverage_server.cpp:110`; client `ros2/src/mowgli_behavior/src/coverage_nodes.cpp:1914` |
| One-click dock calibration contract | `action/CalibrateDock.action` (server `ros2/src/mowgli_localization/src/calibrate_imu_yaw_node.cpp:316`) + its GUI façade `msg/DockCalibrationStatus.msg` (publisher `:335`, Trigger `~/dock_calibration/start` `:338`) |
| Dig detector event → pending keepout | `msg/DigEvent.msg` (pub `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp:731`, sub `ros2/src/mowgli_map/src/map_server_node.cpp:398`); accept/discard via `srv/PromoteObstacle.srv` / `srv/ClearObstacle.srv` |
| Obstacle identity carried with a `MapArea` | `msg/MapObstacleInfo.msg` (index-aligned with `MapArea.obstacles`, `SOURCE_USER/TRACKER/DIG`, `pending`, `id`) |
| Dock pose set semantics (`PRESERVE` / `REQUEST` / `MOTION` yaw source) | `srv/SetDockingPoint.srv`; server `ros2/src/mowgli_map/src/map_server_node.cpp:330` |
| Firmware version handshake fields | `msg/Status.msg:36-45` (`firmware_version`, `firmware_protocol_version`, `firmware_compatible`); filled at `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp:1297-1299` |
| GUI Go / TS type regeneration + drift gate | `gui/generate_go_msgs.sh`, `gui/generate_ts_types.sh`, `.github/workflows/msg-codegen-drift.yml` |
| Firmware header regeneration | `firmware/scripts/sync_ros_lib.py` (`--check` in CI) |
| COBS protocol version lockstep (NOT a rosidl concern, but adjacent) | `.github/workflows/protocol-version-drift.yml`, `firmware/scripts/protocol_version_guard.py` |
| Which interfaces are dead (defined but nobody serves/publishes) | *Pitfalls* → "Orphan definitions" |

## Files
| File | Lines | Purpose |
|------|-------|---------|
| **`ros2/src/mowgli_interfaces/`** | | |
| `CMakeLists.txt` | 91 | `rosidl_generate_interfaces` over the three lists (15 msg, 14 srv, 3 action) + INTERFACE target `mowgli_interfaces_headers` exporting `include/` |
| `package.xml` | 30 | Deps: `builtin_interfaces`, `std_msgs`, `geometry_msgs`, `nav_msgs`; member of `rosidl_interface_packages` |
| **`include/mowgli_interfaces/`** | | |
| `coverage_geometry.hpp` | 28 | `kSegmentTransitGapM` — FollowStrip's transit-vs-drive-through threshold for gaps between `drivable_subpaths` (the planner includes the header but no longer splits on it) |
| `gnss_status_utils.hpp` | 153 | `HasCapability/HasValue`, `IsRtkFixed/Float`, `AbsolutePoseFlags`, `BehaviorTreeFixType`, `NormalizedQuality`, `HardwareQualityPercent`, `BehaviorTreeRtkFixed` |
| `motion_yaw_fit.hpp` | 102 | `FitMotionYaw(samples) → (yaw, sigma)` total-least-squares line fit, ±π resolved chronologically |
| `robot_yaml_scalar.hpp` | 151 | `SpliceScalar`, `FormatScalar` (6 dp), `ReadEditWrite` (tmp+rename), `UpdateDockPose`, `PersistScalar` |
| `wgs84_projection.hpp` | 70 | `EARTH_RADIUS_M`, `METERS_PER_DEG`, `ToEnu`, `FromEnu`, `ReprojectEnu` (equirectangular) |
| **`msg/`** | | |
| `AbsolutePose.msg` | 24 | Legacy GPS pose (`SOURCE_*`, `FLAG_GPS_RTK*`, `position_accuracy`, `pose`, `motion_vector`, headings) |
| `CoveragePath.msg` | 2 | `is_outline` + `nav_msgs/Path` — **NOT in `CMakeLists.txt` `msg_files`**, never built by rosidl |
| `DigEvent.msg` | 28 | `header`, `position`, `wheel_distance`, `map_distance`, `position_sigma` (Invariant 16) |
| `DockCalibrationStatus.msg` | 28 | `PHASE_*` (0-7 incl. `IDLE`/`DONE`), `progress`, `cog_std_deg`, `displacement_m`, `charging`, `running`, `success`, `retry_reason`, `message` |
| `Emergency.msg` | 6 | `active_emergency`, `latched_emergency`, `lift_warning`, `lift_duration_sec`, `reason` |
| `ESCStatus.msg` | 12 | `ESC_STATUS_*` codes, `current`, `tacho`, `rpm`, temps — no ROS producer |
| `GnssStatus.msg` | 120 | `FIX_TYPE_*`, `RTK_MODE_*`, `BASELINE_STATUS_*`, `CORRECTION_STREAM_STATUS_*`, 25 `CAP_*` bits; `capability_flags` vs `value_flags`; dual-antenna baseline; MSM summary |
| `HighLevelStatus.msg` | 22 | `HIGH_LEVEL_STATE_*` (0-4), `state_name`, `sub_state_name`, area/path/swath counters, `coverage_percent`, `gps_quality_percent`, `battery_percent`, `is_charging`, `emergency` |
| `ImuRaw.msg` | 10 | `dt`, ax..gz, mx..mz — no ROS producer |
| `MapArea.msg` | 10 | `name`, `area` polygon, `obstacles[]`, `is_navigation_area`, `obstacle_info[]` |
| `MapObstacleInfo.msg` | 29 | `name`, `source` (`SOURCE_USER/TRACKER/DIG`), `pending`, session `id` |
| `ObstacleArray.msg` | 2 | `header` + `TrackedObstacle[]` |
| `Power.msg` | 6 | `v_charge`, `v_battery`, `charge_current`, `charger_enabled`, `charger_status` |
| `Status.msg` | 45 | `MOWER_STATUS_*`, `RESET_CAUSE_*`, board flags, blade ESC telemetry + `blade_status_stamp`, firmware handshake triple |
| `TrackedObstacle.msg` | 9 | `id`, `polygon`, `centroid`, `radius`, `first_seen`, `observation_count`, `status` (`TRANSIENT`/`PERSISTENT`) |
| `WheelTick.msg` | 18 | `WHEEL_VALID_*`, float `wheel_tick_factor`, per-wheel direction + ticks |
| **`srv/`** | | |
| `AddMowingArea.srv` | 4 | `MapArea area`, `is_navigation_area` → `success` |
| `AreaRecording.srv` | 12 | `COMMAND_START/FINISH/CANCEL`, `area_name`, `is_exclusion_zone` → polygon — no ROS server |
| `CalibrateImuYaw.srv` | 45 | `duration_sec`, `mag_only` → imu yaw/pitch/roll + dock pose block (`dock_valid`, `dock_pose_*`, `dock_yaw_sigma_deg`) |
| `ClearMap.srv` | 2 | empty → `success` — no ROS server (map_server uses `std_srvs/Trigger`) |
| `ClearObstacle.srv` | 4 | `obstacle_id` → `success`, `message` (reused for `~/discard_obstacle`) |
| `EmergencyStop.srv` | 3 | `emergency` (uint8) → `success` |
| `GetMowingArea.srv` | 4 | `index` → `MapArea area`, `success` |
| `GetRecoveryPoint.srv` | 6 | empty → `recovery_pose`, `distance_outside` |
| `HighLevelControl.srv` | 18 | `COMMAND_*` (1-8, 254, 255) → `success` |
| `MowerControl.srv` | 4 | `mow_enabled`, `mow_direction` → `success` |
| `PromoteObstacle.srv` | 59 | `area_index`, `obstacle_id` \| `polygon` \| `pending_id`, `name` → `success`, `message` |
| `SetDockingPoint.srv` | 37 | `docking_pose`, `use_gps_position`, `yaw_source` (`PRESERVE/REQUEST/MOTION`), `yaw_rad` → `success` |
| `StartInArea.srv` | 13 | `area` (uint8 index) → `success` |
| `TriggerReplan.srv` | 4 | `reason` → `success`, `message` — no ROS server |
| **`action/`** | | |
| `CalibrateDock.action` | 70 | Goal `include_imu_yaw`, `include_mag`; Result `RETRY_*` (0-7), dock pose, `cog_std_deg`, IMU block; Feedback `PHASE_*` (0-5) |
| `CoverageTask.action` | 11 | `area_index` → `coverage_percent` — no server or client |
| `PlanCoverage.action` | 53 | Goal `outer_boundary`, `obstacles[]`, `mow_angle_deg`; Result `SEGMENT_RING/SWATH`, `segments[]`, `segment_types[]`, `full_path`, `drivable_subpaths[]`, counts; Feedback `phase` |

No launch, config, or test files live in this package. Tests that pin its contracts live in the consumer packages (see *Build, test, run*).

## Runtime surface

### Nodes
None — this package builds no executables. Every type below is served/published by another package; node names come from the bringup launch files (`hardware_bridge` in `ros2/src/mowgli_bringup/launch/mowgli.launch.py:192`; `behavior_tree_node`, `map_server_node`, `navsat_to_absolute_pose`, `calibrate_imu_yaw_node`, `diagnostics_node`, `mqtt_bridge_node`, `obstacle_tracker`, `led_ring_node` in `full_system.launch.py`; `coverage_server` in `nav2_navigation_launch.py:241`; `costmap_scan_filter`, `gps_dock_detection` in `navigation.launch.py`; `fake_hardware_bridge` in `sim_full_system.launch.py:300`).

### Topics
| Topic | Type | Publisher (file:line) | Subscribers (file:line) | QoS |
|-------|------|-----------------------|-------------------------|-----|
| `/hardware_bridge/status` | `Status` | `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp:704` (`~/status`); sim `ros2/src/mowgli_simulation/src/fake_hardware_bridge_node.cpp:93` | `ros2/src/fusion_graph/src/fusion_graph_node_setup_comms.cpp:151`, `ros2/src/mowgli_localization/src/calibrate_imu_yaw_node.cpp:219`, `ros2/src/mowgli_localization/src/costmap_scan_filter_node.cpp:141` (param `status_topic`), `ros2/src/mowgli_map/src/map_server_node.cpp:223`, `ros2/src/mowgli_monitoring/src/diagnostics_node.cpp:153`, `ros2/src/mowgli_monitoring/src/mqtt_bridge_node.cpp:435`, GUI `gui/pkg/providers/ros.go:29` | reliable 10 (map_server subscribes depth 1) |
| `/hardware_bridge/emergency` | `Emergency` | `hardware_bridge_node.cpp:706`; sim `:98` | `calibrate_imu_yaw_node.cpp:227`, `diagnostics_node.cpp:161`, `mqtt_bridge_node.cpp:451`, GUI `ros.go:51` | reliable 10 |
| `/hardware_bridge/power` | `Power` | `hardware_bridge_node.cpp:707`; sim `:96` | `diagnostics_node.cpp:169`, `mqtt_bridge_node.cpp:443`, `ros2/src/mowgli_leds/src/led_ring_node.cpp:152`, GUI `ros.go:50` | reliable 10 |
| `/hardware_bridge/dig_event` | `DigEvent` | `hardware_bridge_node.cpp:731` (`~/dig_event`) | `map_server_node.cpp:398` (only if `dig_obstacle_enabled_`) | **transient_local** both ends |
| `/wheel_ticks` | `WheelTick` | `ros2/src/mowgli_hardware/src/odometry_publisher.cpp:40` (`~/wheel_ticks`, remapped `mowgli.launch.py:261`) | GUI `ros.go:45`; `ros2/src/mowgli_localization/src/wheel_odometry_node.cpp:102` (node not launched — `full_system.launch.py:438-446`) | reliable 10 |
| `/behavior_tree_node/high_level_status` | `HighLevelStatus` | `ros2/src/mowgli_behavior/src/status_nodes.cpp:58` (`~/high_level_status`) | `hardware_bridge_node.cpp:771`, `fusion_graph_node_setup_comms.cpp:166`, `calibrate_imu_yaw_node.cpp:235`, `led_ring_node.cpp:128`, GUI `ros.go:30`, `ros2/src/e2e_test.py:56` | depth 10 |
| `/gps/status` | `GnssStatus` | `sensors/gps/universal_gnss_topic_bridge.py:167` (converts `universal_gnss_ros2/GnssStatus`) | `hardware_bridge_node.cpp:753`, `ros2/src/mowgli_localization/src/navsat_to_absolute_pose_node.cpp:127`, `ros2/src/mowgli_behavior/src/behavior_tree_node.cpp:427`, `led_ring_node.cpp:137`, GUI `ros.go:35` | reliable 10 |
| `/gps/absolute_pose` | `AbsolutePose` | `navsat_to_absolute_pose_node.cpp:110` | `behavior_tree_node.cpp:345`, `calibrate_imu_yaw_node.cpp:420`, `ros2/src/mowgli_localization/src/gps_dock_detection_node.cpp:212`, `ros2/src/mowgli_localization/src/localization_monitor_node.cpp:101` | reliable 10 |
| `/obstacle_tracker/obstacles` | `ObstacleArray` | `ros2/src/mowgli_map/src/obstacle_tracker_node.cpp:87` | `map_server_node.cpp:380`, GUI `ros.go:57` (CDR path pinned by `gui/pkg/foxglove/obstacle_wire_test.go`) | depth 1 |
| `/calibrate_imu_yaw_node/dock_calibration/status` | `DockCalibrationStatus` | `calibrate_imu_yaw_node.cpp:335` | GUI `ros.go:33` | volatile — subscribe before triggering |

`MapArea` / `MapObstacleInfo` travel only inside `AddMowingArea` / `GetMowingArea` payloads (built in `ros2/src/mowgli_map/src/area_manager.cpp`, consumed by `ros2/src/mowgli_behavior/src/coverage_nodes.cpp:1824` `buildGoal`). `TrackedObstacle` only nests in `ObstacleArray`.

### Services & actions
| Name | Type | Server (file:line) | Clients (file:line) |
|------|------|--------------------|---------------------|
| `/behavior_tree_node/high_level_control` | `srv/HighLevelControl` | `behavior_tree_node.cpp:548` | `hardware_bridge_node.cpp:855`, `calibrate_imu_yaw_node.cpp:244`, `mqtt_bridge_node.cpp:486`, GUI `gui/pkg/api/mowglinext.go:564`, `ros2/src/e2e_test.py` |
| `/behavior_tree_node/start_in_area` | `srv/StartInArea` | `behavior_tree_node.cpp:598` | GUI `mowglinext.go:594` |
| `/hardware_bridge/mower_control` | `srv/MowerControl` | `hardware_bridge_node.cpp:809`; sim `fake_hardware_bridge_node.cpp:53` | `coverage_nodes.cpp:1122`, `ros2/src/mowgli_behavior/src/utility_nodes.cpp:63`, GUI `mowglinext.go:584` |
| `/hardware_bridge/emergency_stop` | `srv/EmergencyStop` | `hardware_bridge_node.cpp:817`; sim `:64` | `utility_nodes.cpp:243`, GUI `mowglinext.go:574`, `e2e_test.py:55` |
| `/map_server_node/add_area` | `srv/AddMowingArea` | `map_server_node.cpp:314` | `ros2/src/mowgli_behavior/src/recording_nodes.cpp:441`, GUI `mowglinext.go:114,189` |
| `/map_server_node/get_mowing_area` | `srv/GetMowingArea` | `map_server_node.cpp:322` | `ros2/src/mowgli_behavior/src/condition_nodes.cpp:508`, `coverage_nodes.cpp:1468,1903`, `obstacle_tracker_node.cpp:193`, GUI `ros.go:417` |
| `/map_server_node/set_docking_point` | `srv/SetDockingPoint` | `map_server_node.cpp:330` | `calibrate_imu_yaw_node.cpp:310`, GUI `mowglinext.go:238` |
| `/map_server_node/get_recovery_point` | `srv/GetRecoveryPoint` | `map_server_node.cpp:354` (`ros2/src/mowgli_map/src/progress_tracker.cpp:155`) | none in-repo |
| `/map_server_node/promote_obstacle` | `srv/PromoteObstacle` | `map_server_node.cpp:407` | GUI `mowglinext.go:631` |
| `/map_server_node/discard_obstacle` | `srv/ClearObstacle` | `map_server_node.cpp:415` | GUI `mowglinext.go:654` |
| `/obstacle_tracker/clear_obstacle` | `srv/ClearObstacle` | `obstacle_tracker_node.cpp:149` | none in-repo |
| `/calibrate_imu_yaw_node/calibrate` | `srv/CalibrateImuYaw` | `calibrate_imu_yaw_node.cpp:252` | GUI `gui/pkg/api/calibration.go:132,200` |
| `/calibrate_imu_yaw_node/calibrate_dock` | `action/CalibrateDock` | `calibrate_imu_yaw_node.cpp:314-316` | none — GUI drives `~/dock_calibration/start` (`std_srvs/Trigger`, `:338`; `calibration.go:84`) and watches the status topic |
| `/plan_coverage` | `action/PlanCoverage` | `ros2/src/mowgli_coverage/src/coverage_server.cpp:110` | `coverage_nodes.cpp:1914` (`PlanCoverageArea`) |

### Parameters
None declared here. Header helpers are pure functions; the constants that act like parameters are `coverage_geometry.hpp:26` (`kSegmentTransitGapM`), `wgs84_projection.hpp:29-35`, `gnss_status_utils.hpp:111-123` (quality thresholds 0.05 / 0.5 / 2.0 m) and `robot_yaml_scalar.hpp:77-82` (6-decimal format).

### TF frames
None. `wgs84_projection.hpp` defines the map-frame convention (X=east, Y=north — CLAUDE.md Invariant 4) but publishes nothing.

## Build, test, run
```bash
# Interfaces alone (from ros2/)
colcon build --packages-select mowgli_interfaces
# Everything that pins its contracts
colcon build --packages-up-to mowgli_behavior mowgli_map mowgli_localization mowgli_hardware
colcon test  --packages-select mowgli_behavior mowgli_map mowgli_localization mowgli_hardware && colcon test-result --verbose
# Out-of-tree generators (repo root); CI runs the same three
python3 firmware/scripts/sync_ros_lib.py --check
( cd gui && ./generate_go_msgs.sh && ./generate_ts_types.sh ) && git diff --exit-code -- gui/pkg/msgs gui/web/src/types/ros.generated.ts
```
Tests (all gtest, in consumer packages; run by `.github/workflows/ros2-ci.yml` "Build & Test" via `colcon test`):
| Test | Pins |
|------|------|
| `ros2/src/mowgli_localization/test/test_robot_yaml_scalar.cpp` | `UpdateDockPose` updates all three keys, preserves comments, ignores look-alike keys (`dock_pose_yaw_sigma_rad`), no leftover `.tmp`, returns false on missing file (`mowgli_localization/CMakeLists.txt:342`) |
| `ros2/src/mowgli_behavior/test/test_gnss_status_authority.cpp` | RTK-Fixed / Float `GnssStatus` samples yield the same answer from BT (`BehaviorTreeFixType`) and hardware (`HardwareQualityPercent`) helpers (`mowgli_behavior/CMakeLists.txt:323`) |
| `ros2/src/mowgli_behavior/test/test_coverage_transit_gap.cpp` | FollowStrip's threshold `==` `kSegmentTransitGapM` and the value is pinned (`mowgli_behavior/CMakeLists.txt:341`) |
| `ros2/src/mowgli_behavior/test/test_detour_resume.cpp` | `DetourResumeCfg{}.min_skip_dist_m` > `kSegmentTransitGapM`, so a detour crossing is provably blade-off (`mowgli_behavior/CMakeLists.txt:438`) |
| `ros2/src/mowgli_localization/test/test_navsat_projection_utils.cpp` | Typed `GnssStatus` RTK state survives covariance fallback; plain NavSatFix never masquerades as RTK (`mowgli_localization/CMakeLists.txt:301`) |
| `ros2/src/mowgli_map/test/test_map_server.cpp` (`DatumMigrationTest`, lines 790-915; DigEvent / PromoteObstacle / MapObstacleInfo cases) | `ReprojectEnu` datum migration of areas + obstacles + dock; pending-dig accept/discard flow |
| `ros2/src/mowgli_behavior/test/test_coverage_resume_location.cpp`, `test_get_next_unmowed_area.cpp` | `resolveResumeLocation` cursor mapping over the `PlanCoverage` result's sub-paths; `StartInArea` targeting |
| `ros2/src/mowgli_bringup/test/test_navsat_status_universal.launch.py` | launch_test: `AbsolutePose` emitted from the universal-GNSS status path (`mowgli_bringup/CMakeLists.txt:50`) |
| `gui/pkg/foxglove/obstacle_wire_test.go` | Real CDR frame of `ObstacleArray` decodes `status` at the right offset |
| `gui/pkg/msgs/mowgli/mower_control_bind_test.go` | `MowerControlReq` binds snake_case JSON only |
| `ros2/src/e2e_test.py`, `ros2/src/e2e_test_no_lidar.py` | Sim end-to-end via `HighLevelControl` + `HighLevelStatus` (manual; not wired into a workflow) |

CI gates outside colcon: `.github/workflows/msg-codegen-drift.yml` (firmware `--check`, Go + TS regen diff) and `.github/workflows/sensors-gps.yml` (builds `mowgli_interfaces` into the GPS sidecar image and imports `GnssStatus` in the smoke test, lines 37-64).

## Change coupling — "if you change X, also update Y"
- **Any `msg/*.msg` edit** → run all three generators and commit their output: `firmware/scripts/sync_ros_lib.py` → `firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/*.h`; `gui/generate_go_msgs.sh` → `gui/pkg/msgs/mowgli/types_generated.go`; `gui/generate_ts_types.sh` → `gui/web/src/types/ros.generated.ts`. `msg-codegen-drift.yml` fails otherwise. Run the Go/TS scripts with `LC_ALL=C` (sort order differs on macOS).
- **Any `srv/*.srv` edit** → `gui/generate_go_msgs.sh` also emits `gui/pkg/msgs/mowgli/services_generated.go` (`generate_go_msgs.sh:554`). TS and firmware generators ignore `srv/`.
- **`action/*.action`** has NO generator consumer. The GUI never calls an action (foxglove bridge has no action support — `msg/DockCalibrationStatus.msg:1-8`); expose new action progress through a msg façade + `std_srvs/Trigger` the way `CalibrateDock` does.
- **New `.msg`/`.srv`/`.action` file** → add it to the matching list in `CMakeLists.txt:17-58`, or rosidl silently skips it while the generators still emit Go/TS/firmware types for it (this is `CoveragePath.msg`'s current state).
- **`HighLevelStatus.msg` `HIGH_LEVEL_STATE_*`** ↔ `firmware/stm32/ros_usbnode/include/mowgli_protocol.h:418-422` `HL_MODE_*` (hand-mirrored; `hardware_bridge_node.cpp:771` forwards the state over `PKT_ID_HL_STATE`).
- **`Status.msg` handshake fields** ↔ `MOWGLI_PROTOCOL_VERSION` (`mowgli_protocol.h:59`) and `kMowgliProtocolVersion` (`ros2/src/mowgli_hardware/include/mowgli_hardware/ll_datatypes.hpp:51`); `protocol-version-drift.yml` enforces lockstep + fingerprint via `firmware/scripts/protocol_version_guard.py` / `protocol_baseline.json`.
- **`GnssStatus.msg` enums / `CAP_*` bits** ↔ the mapping tables in `sensors/gps/universal_gnss_topic_bridge.py:19-24` (and `sensors/gps/mowgli_gnss_bridge/`) that translate `universal_gnss_ros2/GnssStatus`; `sensors-gps.yml` rebuilds on any change here.
- **`wgs84_projection.hpp`** ↔ `ros2/src/mowgli_localization/src/navsat_to_absolute_pose_node.cpp` (`wgs84_to_enu`) and `gui/web/src/utils/map.tsx` (`transpose` / `itranspose`) — header comment lines 11-15 requires the three stay identical.
- **`coverage_geometry.hpp` `kSegmentTransitGapM`** ↔ `ros2/src/mowgli_behavior/include/mowgli_behavior/coverage_nodes.hpp:159-160` (`FollowStrip::kSegmentTransitGap`); `test_coverage_transit_gap.cpp` fails if that side stops using it. `mowgli_coverage`'s `buildContinuousSubPaths` includes the header but no longer splits on this value — its split is now connector-drivability (`coverage_planning.cpp:1589-1605`).
- **`robot_yaml_scalar.hpp`** is the ONLY dock-pose writer path, and it now has exactly TWO callers: `calibrate_imu_yaw_node.cpp:43/723` (dock pre-phase) and `area_manager.cpp:37` — `UpdateDockPose` at `:916` (map_server's `on_set_docking_point`) and `:1673` (datum migration). `mowgli_behavior/src/calibration_nodes.cpp` no longer persists dock yaw (`:40-49` — the `PersistScalar` EMA writeback was removed so there is exactly one file writer), so `PersistScalar` currently has no in-repo caller. Changing key names here changes what lands in the installed `mowgli_robot.yaml` (Invariant 15).
- **`MapArea.obstacle_info`** must stay index-aligned with `obstacles` (`MapObstacleInfo.msg:3-6`); GUI `gui/pkg/api/mowglinext.go` and `area_manager.cpp` both rely on the "empty = all user keepouts" fallback.
- **Adding a package that includes a header from here** → `<depend>mowgli_interfaces</depend>` in its `package.xml` + `ament_target_dependencies(... mowgli_interfaces)`; currently: `mowgli_behavior`, `mowgli_bringup`, `mowgli_coverage`, `mowgli_localization`, `fusion_graph`, `mowgli_hardware`, `mowgli_leds`, `mowgli_monitoring`, `mowgli_map`, `mowgli_simulation`.

## Pitfalls
- **Orphan definitions** (built, generated into Go/TS/firmware, but no ROS server/publisher anywhere in `ros2/src`): `msg/ESCStatus.msg`, `msg/ImuRaw.msg`, `srv/AreaRecording.srv`, `srv/ClearMap.srv` (map_server's `~/clear_map` is `std_srvs/Trigger`, `map_server_node.cpp:306-307`), `srv/TriggerReplan.srv` (replan is the `~/replan_needed` Bool topic, `map_server_node.cpp:363`), `action/CoverageTask.action`. Do not build new features on them without first wiring a server.
- **`msg/CoveragePath.msg` is not registered** in `CMakeLists.txt` `msg_files` (lines 17-33) so `mowgli_interfaces/msg/CoveragePath` does not exist at runtime, yet `gui/pkg/msgs/mowgli/types_generated.go:26`, `gui/web/src/types/ros.generated.ts:157` and `firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/CoveragePath.h` carry it — the generators glob the directory, not the CMake list.
- **Firmware never compiles the generated `mower_msgs/*.h`**: `firmware/stm32/ros_usbnode/platformio.ini:15-17` excludes `ros/ros_lib/**` from `build_src_filter` and nothing under `firmware/` includes `mower_msgs/`. The wire is COBS (`mowgli_protocol.h`), not rosserial. `sync_ros_lib.py --check` still gates CI, so keep regenerating, but do not expect a `.msg` change to reach the STM32 — change `mowgli_protocol.h` + `ll_datatypes.hpp` and bump `MOWGLI_PROTOCOL_VERSION` instead.
- **`ros2/src/mowgli_hardware/firmware/mowgli_protocol.h` is a stale copy** (`MOWGLI_PROTOCOL_VERSION 3u` at line 60 vs `6u` in `firmware/stm32/ros_usbnode/include/mowgli_protocol.h:59`) and is outside the `protocol_version_guard.py` fingerprint (which reads only the firmware header + `ll_datatypes.hpp`). The host truth is `ll_datatypes.hpp`.
- **`ClearObstacle.srv` is served twice** with different meanings: `obstacle_tracker/clear_obstacle` drops a tracker observation, `map_server_node/discard_obstacle` rejects a pending dig/tracker proposal (`PromoteObstacle.srv:10-15`). The GUI calls the latter (`mowglinext.go:654`).
- **`PromoteObstacle` lookup precedence** is `pending_id` > `polygon` > `obstacle_id` (`PromoteObstacle.srv:25-38`); re-sending a pending obstacle's polygon hits the centroid dedup guard and is a silent no-op — use `pending_id`.
- **`SetDockingPoint.yaw_source` defaults to `PRESERVE` (0)** and callers MUST set it explicitly (`SetDockingPoint.srv:14-30`); reading yaw back on the dock is circular (Invariant 6, `docking-10cm-offset` memory).
- **`HighLevelControl.srv` has a duplicate code**: `COMMAND_S1=3` and `COMMAND_RECORD_AREA=3` share the value (lines 3, 5); `COMMAND_S2=4` is "mow next area" per `docs/claude/high-level-api.md:11`.
- **`DigEvent` is transient_local on both ends** (`hardware_bridge_node.cpp:731-733`, `map_server_node.cpp:398-401`); a volatile subscriber will miss digs that latched before it started. map_server only subscribes when `dig_obstacle_enabled_` (`map_server_node.cpp:396`) — see Invariant 16.
- **`DockCalibrationStatus` phase codes must mirror `CalibrateDock.action` feedback `PHASE_*`** (0-5) and add `PHASE_IDLE=6` / `PHASE_DONE=7`; `retry_reason` mirrors the action's `RETRY_*` (msg header lines 1-8). Edit both files together.
- **`GnssStatus` boolean fields are three-state**: check `CAP_*` in `capability_flags` AND `value_flags` before trusting a `bool` (`GnssStatus.msg:7-14`); use `gnss_status_utils.hpp` rather than reading `fix_type` alone — `IsRtkFixed` accepts either `rtk_mode` or `fix_type` (lines 37-45).
- **`robot_yaml_scalar.hpp` has two deliberately different failure semantics**: `UpdateDockPose` ignores missing keys (lines 115-121), `PersistScalar` fails on a missing key (lines 134-141). `SpliceScalar` only matches indented keys (line 45) so top-level keys are never touched.
- **`wgs84_projection.hpp` is equirectangular** (~1 cm within 10 km of the datum, line 11); do not swap in a different projection in one consumer only — `test_map_server.cpp` `DatumMigrationTest` and the localizer would disagree (Invariant 4).
- **`AbsolutePose.msg` reuses flag values**: `FLAG_GPS_DEAD_RECKONING=8` and `FLAG_SENSOR_FUSION_DEAD_RECKONING=8`, `FLAG_GPS_RTK=1` and `FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE=1` (lines 5-10) — interpret against `source`.
- **`WheelTick` consumer `wheel_odometry_node` is not launched** (`full_system.launch.py:438-446` says `/wheel_ticks` has no publisher, but `odometry_publisher.cpp:40` + `mowgli.launch.py:261` do publish it — the comment is stale; the GUI's per-wheel panel reads it).
- **`gnss_status_utils::BehaviorTreeFixType` returns 4 for Fixed / 3 for Float / 2 for GPS-fix** (lines 82-100) — a different scale from `GnssStatus::FIX_TYPE_*` (3 = RTK_FIXED). Do not compare the two directly.

## Generated & vendored — do not hand-edit
- `gui/pkg/msgs/mowgli/types_generated.go`, `gui/pkg/msgs/mowgli/services_generated.go` — from `gui/generate_go_msgs.sh` (hand-maintained companions: `types.go`, `services.go`).
- `gui/web/src/types/ros.generated.ts` — from `gui/generate_ts_types.sh` (`gui/web/src/types/ros.ts` is a hand-written `export *` re-export).
- `firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/*.h` — from `firmware/scripts/sync_ros_lib.py` (MD5-stamped; excluded from the firmware build).
- `ros2/src/external/universal-gnss/` — git submodule that defines the upstream `universal_gnss_ros2/GnssStatus` this package's `GnssStatus.msg` is bridged from.
- `build/`, `install/` under `ros2/` — rosidl output (`mowgli_interfaces/msg/*.hpp`, Python bindings).
