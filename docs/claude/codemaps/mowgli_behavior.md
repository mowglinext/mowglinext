# Codemap: mowgli_behavior

> BehaviorTree.CPP v4 mission executor. `behavior_tree_node` ticks `trees/main_tree.xml` at `tick_rate` Hz, owns the
> `HighLevelControl`/`HighLevelStatus` surface the GUI talks to, and runs every safety guard (emergency, sensor, boundary,
> localization), the mow / home / record / manual / stop / idle branches, docking + undocking, and the disk-persisted
> coverage-resume state (`coverage_resume.txt`, incl. `current_command`). It also ships `trees/navigate_to_pose.xml`,
> the Nav2 `bt_navigator` tree used for every `/navigate_to_pose` goal.
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `ros2/CLAUDE.md`.

## Where to look
| Task | Start here |
|------|------------|
| Add / rename a BT node | `ros2/src/mowgli_behavior/src/register_nodes.cpp` (`registerAllNodes`) → class in the matching `include/mowgli_behavior/*_nodes.hpp` → `trees/main_tree.xml` |
| Change tree structure / a guard | `ros2/src/mowgli_behavior/trees/main_tree.xml` (Root `ReactiveSequence`: EmergencyGuard → SensorSafetyGuard → BoundaryGuard → LocalizationGuard → GPSModeSelector → Nav2ResumeGuard → MainLogic) |
| Add a subscriber / service / param to the node | `ros2/src/mowgli_behavior/src/behavior_tree_node.cpp` (`setupSubscribers` :140, `setupServiceServer` :543, `setupBehaviorTree` :807) |
| Shared state read by nodes (`ctx->…`) | `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp` (`BTContext`; thread-safety contract :77-103) |
| High-level command handling (`COMMAND_*`, `~/start_in_area`) | `behavior_tree_node.cpp` :547-608 (`COMMAND_S2`→`COMMAND_START` normalisation :561) |
| `HighLevelStatus` publish + 1 Hz republish with live fields | `src/status_nodes.cpp` (`PublishHighLevelStatus`), `src/status_snapshot.cpp` (`withLiveStatusFields`), `behavior_tree_node.cpp` :694-721 |
| Coverage loop (area iteration, plan, follow, resume) | `src/coverage_nodes.cpp`: `GetNextUnmowedArea` :1461+, `PlanCoverageArea` :1888+, `FollowStrip` :147-1240, `TransitToStrip` :1247+ |
| Resume cursor / restart persistence | `src/coverage_persistence.cpp` (`saveCoverageResumeState` / `loadCoverageResumeState` / `clearCoverageResumeState`, file header `mowgli_coverage_resume v2`) + `behavior_tree_node.cpp` :91-118 (auto-continue on boot) |
| Blade on/off during coverage & transit gap guard | `src/coverage_nodes.cpp` `FollowStrip::sendCurrentSwath` :600-690 (`kSegmentTransitGap` from `mowgli_interfaces/coverage_geometry.hpp:26`), `setBladeEnabled` :1118 |
| Obstacle detour inside a sub-path | `include/mowgli_behavior/detour_resume.hpp` (`decideDetour`, `footprintClear`) + `FollowStrip::tryStartDetour` `src/coverage_nodes.cpp` :1133+ |
| LocalizationGuard signal (`WAITING_FOR_RTK`) | `include/mowgli_behavior/localization_health.hpp` (`LocalizationHealthMonitor`, `PersistentLatch`) + `behavior_tree_node.cpp` :285-295, :330-340, :427-474, `updateLocalizationHealthLocked` :658 |
| BoundaryGuard soft recovery | `src/navigation_nodes.cpp` `NavigateInsideBoundary` :410-760 (get_recovery_point → keepout off → clear → nav → BackUp fallback → keepout on) |
| Start-pose-blocked (#487) recovery + escape motion | `include/mowgli_behavior/transit_failure.hpp` (`classifyTransitFailure`), `include/mowgli_behavior/start_blocked_escape.hpp` (`EscapeDecide`, ceilings), `src/escape_nodes.cpp` (`EscapeStartBlocked`), `src/condition_nodes.cpp` `IsCoverageStartBlocked` :885 |
| Docking / undocking flow | `src/docking_nodes.cpp` (`DockRobot` → `/dock_robot`), undock = `BackUp` in `src/navigation_nodes.cpp` :800+ (`/backup`), `main_tree.xml` `UndockSequence` :487-538 |
| Heading calibration on undock / off-dock start | `src/calibration_nodes.cpp` (`RecordUndockStart`, `CalibrateHeadingFromUndock` line-fit → `/fusion_graph_node/set_pose`, `SeedYawFromMotion` drives via `/cmd_vel_teleop`) |
| Dock-contact alignment log / dock_yaw drift check | `include/mowgli_behavior/dock_alignment.hpp` (`ComputeDockContactDelta`, `EvaluateDockYawDrift`) |
| Battery % derivation, low-battery false trips | `src/battery_filter.cpp` (`BatteryVoltageFilter`, `batteryPercentFromVoltage`), `behavior_tree_node.cpp` :204-229 |
| Rain handling (mode / debounce / dock-and-wait) | `src/condition_nodes.cpp` `IsNewRain`, `IsRainModeAtLeast`; `main_tree.xml` `RainGuard` :576-634 |
| Obstacle-stuck / sensor-fault guards | `src/condition_nodes.cpp` `IsObstacleStuck`, `WasRecentlyInCollisionStop`, `IsScanStale`, `IsCollisionStopSustained`; `behavior_tree_node.cpp` :479-538 (collision_monitor + scan liveness latches) |
| Idle Nav2 suspend (`idle_nav2_suspend`) | `src/navigation_nodes.cpp` `SetNav2Lifecycle` :170-255 (`/lifecycle_manager_navigation/manage_nodes`) |
| Transit/mowing speed → live controllers | `src/navigation_nodes.cpp` `SetNavMode` :915-970 (`FollowPath.desired_linear_vel`, `FollowCoveragePath.speed_fast` on `/controller_server`) |
| Area recording | `src/recording_nodes.cpp` (`RecordArea`: Douglas-Peucker, `~/recording_trajectory` preview, `/map_server_node/add_area`) |
| Nav2 navigate_to_pose tree (replan-only-if-invalid) | `ros2/src/mowgli_behavior/trees/navigate_to_pose.xml` (wired by `ros2/src/mowgli_bringup/launch/navigation.launch.py` :626-629) |
| Launch wiring / which params get forwarded | `ros2/src/mowgli_bringup/launch/full_system.launch.py` :216-354 (`behavior_tree_node` Node), `sim_full_system.launch.py` :190-199 |

## Files
| File | Lines | Purpose |
|------|-------|---------|
| **`ros2/src/mowgli_behavior/`** | | |
| `CMakeLists.txt` | 612 | One executable `behavior_tree_node`; 19 `ament_add_gtest` targets (each links only the `.cpp` it needs; two get `MOWGLI_MAIN_TREE_PATH` for structural XML checks) |
| `package.xml` | 41 | Deps: ament_index_cpp, rclcpp(_action), behaviortree_cpp, tf2*, nav2_msgs, nav_msgs, geometry_msgs, sensor_msgs, std_msgs, std_srvs, action_msgs, rcl_interfaces, mowgli_interfaces |
| `config/behavior_tree.yaml` | 11 | `behavior_params` for both launch files: `tree_file`, `tick_rate`, `battery_low_pct`, `battery_critical_pct` (the two `_pct` keys are NOT declared param names — ignored) |
| `config/behavior_tree_small_garden.yaml` | 12 | Same + `dock_pose` string; not referenced by any launch file |
| `trees/main_tree.xml` | ~1.1k | The mission tree (`MowgliMain`); every `state_name` the GUI sees is emitted here |
| `trees/navigate_to_pose.xml` | 79 | Nav2 `bt_navigator` tree: ControllerSelector/GoalCheckerSelector/PlannerSelector + replan only if path invalid, RoundRobin recovery |
| **`include/mowgli_behavior/`** | | |
| `action_nodes.hpp` | 33 | Umbrella header; declares `registerAllNodes()` |
| `bt_context.hpp` | 646 | `BTContext` shared via blackboard key `"context"`; `clearSingleAreaMode()` |
| `condition_nodes.hpp` | 830 | 25 `BT::ConditionNode` classes + ports |
| `coverage_nodes.hpp` | 600 | `FollowStrip`, `TransitToStrip`, `DetourAroundObstacle`, `GetNextUnmowedArea`, `PlanCoverageArea`; pure helpers `resolveResumeLocation`, `refreshSwathProgress`, `coveragePercentFromCursor`, `forwardSkipIndex`; `kMowAngleAutoDeg=-1` |
| `navigation_nodes.hpp` | 389 | `StopMoving`, `ClearCostmap`, `SetNav2Lifecycle`, `NavigateToPose`, `BackUp`, `SetNavMode`, `NavigateInsideBoundary` (phase enum) |
| `docking_nodes.hpp` | 147 | `DockRobot`, `UndockRobot`, `RecordResumeUndockFailure` |
| `escape_nodes.hpp` | 147 | `EscapeStartBlocked` (`kBladeStateMaxAgeSec=2`, `kMaxTickDtSec=0.5`, `kSignalHoldoffSec=1`) |
| `recording_nodes.hpp` | 203 | `RecordArea` (`kMinSampleSpacingM=0.05`, `kDefaultRecordRateHz=10`, preview 2 Hz) |
| `status_nodes.hpp` | 161 | `PublishHighLevelStatus` (IDLE debounce 3 ticks), `WasRainingAtStart`, `ClearCommand`, `EndSession`, `IncrementSkippedSwaths` |
| `utility_nodes.hpp` | 192 | `SetMowerEnabled`, `WaitForDuration`, `WaitForGpsFix`, `SaveObstacles`, `ResetEmergency` |
| `calibration_nodes.hpp` | 152 | `RecordUndockStart`, `CalibrateHeadingFromUndock`, `SeedYawFromMotion` |
| `localization_health.hpp` | 328 | Header-only `LocalizationHealthMonitor` (GNSS accuracy / fix-lost / stale latches + σ_xy divergence backstop) |
| `start_blocked_escape.hpp` | 329 | Header-only escape policy; compiled ceilings `kEscapeMaxSpeed=0.15`, `kEscapeMaxDistance=0.60`, `kEscapeMaxTimeout=15`, `SanitizeEscapeCfg` |
| `transit_failure.hpp` | 178 | `TransitFailure` enum + `classifyTransitFailure(nav2 error_code)`; `isStartPoseBlocked` |
| `detour_resume.hpp` | 233 | Header-only costmap footprint test + resume-pose search (`DetourCostmap`, `decideDetour`) |
| `dock_alignment.hpp` | 129 | Header-only along/cross-track dock delta + `EvaluateDockYawDrift` (`kDockStagingRunwayM=1.5`, σ floor 0.035 rad) |
| `coverage_persistence.hpp` | 59 | save/load/clear resume-state API |
| `battery_filter.hpp` | 108 | `BatteryVoltageFilter` (τ=2 s, min valid 10 V), `batteryPercentFromVoltage` |
| `status_snapshot.hpp` | 55 | `withLiveStatusFields(base, ctx)` |
| **`src/`** | | |
| `behavior_tree_node.cpp` | ~1.2k | `BehaviorTreeNode` (rclcpp::Node `mowgli_behavior_node`, launched as `behavior_tree_node`): subs, services, params, blackboard seeding, tick timer, Nav2 readiness poll, `main()` (MultiThreadedExecutor + `_bt_helper_node`) |
| `register_nodes.cpp` | 104 | `registerAllNodes` — the authoritative node registry (55 nodes) |
| `condition_nodes.cpp` | 917 | Condition ticks (pure reads of `BTContext`, except `PreFlightCheck`/`Nav2Active` which call services) |
| `coverage_nodes.cpp` | ~2.2k | Coverage nodes; resume cursor + fingerprint logic; START_OCCUPIED classification; detour |
| `navigation_nodes.cpp` | 976 | Motion/costmap/lifecycle nodes; `SetNavMode` speed push |
| `docking_nodes.cpp` | 272 | `DockRobot` (feedback-state logging, contact delta), `UndockRobot`, `RecordResumeUndockFailure` |
| `escape_nodes.cpp` | 230 | Bounded open-loop escape on `/cmd_vel_nav` |
| `calibration_nodes.cpp` | 433 | Undock line-fit yaw → `/fusion_graph_node/set_pose`; forward-drive yaw seed |
| `recording_nodes.cpp` | 515 | Area recording, DP simplification, save via `/map_server_node/add_area` |
| `status_nodes.cpp` | 240 | Status publish, `EndSession` (session-scoped clears :159-215), `ClearCommand` |
| `status_snapshot.cpp` | 58 | Tree-owned vs live field split for `HighLevelStatus` |
| `utility_nodes.cpp` | 267 | Blade service, waits, `SaveObstacles`, `ResetEmergency` |
| `coverage_persistence.cpp` | 219 | Text file `coverage_resume.txt` (atomic tmp+rename): `current_command`, `single_area_target`, `current_area`, `completed_areas`, per-`area` rows (pose_count, fingerprint, resume, completed swaths) |
| `battery_filter.cpp` | 85 | Rate-independent low-pass on `v_battery` |
| **`test/`** (gtest; all registered in `CMakeLists.txt`) | | |
| `test_recording_nodes.cpp` | 561 | 21 tests: DP simplification, spacing gate, area math, save/cancel paths of `RecordArea` |
| `test_obstacle_recovery.cpp` | 305 | 13 tests: `IsObstacleStuck` timing/cap/cooldown against latched collision state |
| `test_docking_boundary_exempt.cpp` | 232 | 8 tests: `IsDocking` + BoundaryGuard blade-off dock-transit exemption |
| `test_set_nav2_lifecycle.cpp` | 243 | 7 tests: `SetNav2Lifecycle` gating + fake `manage_nodes` transition |
| `test_get_next_unmowed_area.cpp` | 459 | 11 tests: nav-only areas skipped, targeted (`~/start_in_area`) runs stay clipped, `EndSession` boundary |
| `test_start_occupied_retry.cpp` | 348 | 13 tests: `classifyTransitFailure`, consume-once `IsCoverageStartBlocked`, structural check of `StartPoseBlockedRetry` in `main_tree.xml` |
| `test_coverage_persistence.cpp` | 248 | 10 tests: round-trip, header/version, malformed rows, `current_command` restore |
| `test_gnss_status_authority.cpp` | 56 | 2 tests: `mowgli_interfaces::gnss_status_utils` fix-type mapping the BT relies on |
| `test_coverage_transit_gap.cpp` | 50 | 2 tests: `FollowStrip::kSegmentTransitGap == coverage_geometry::kSegmentTransitGapM` |
| `test_coverage_resume_location.cpp` | 192 | 11 tests: `resolveResumeLocation` shared by `FollowStrip` and `PlanCoverageArea` |
| `test_swath_progress.cpp` | 153 | 7 tests: `refreshSwathProgress` / `coveragePercentFromCursor` climb during a pass |
| `test_detour_resume.cpp` | 365 | 15 tests: `footprintClear`, `decideDetour` |
| `test_start_blocked_escape.cpp` | 382 | 20 tests: escape direction, stand-downs, bounds, ceilings (safety-critical) |
| `test_localization_health.cpp` | 419 | 16 tests: pivot σ inflation must NOT pause; plain-GPS fallback must; stale feed |
| `test_battery_critical_resume.cpp` | 225 | 3 tests: critical-battery tail auto-continues, only dead charger ends session |
| `test_guard_fallthrough.cpp` | 327 | 5 tests: guard handlers return FAILURE + structural `<AlwaysFailure/>` check on every blocking guard in `main_tree.xml` |
| `test_high_level_status_snapshot.cpp` | 156 | 6 tests: republished status carries live battery/progress, tree-owned state untouched |
| `test_battery_filter.cpp` | 243 | 13 tests: sag immunity, rate independence, invalid reading never → 0 % |
| `test_dock_alignment.cpp` | 191 | 11 tests: along/cross decomposition, yaw-drift band |

## Runtime surface

### Nodes
| Node | Executable | Launched by | Type |
|------|-----------|-------------|------|
| `behavior_tree_node` (class default name `mowgli_behavior_node`; `test_nodes_startup.launch.py` keeps that name) | `behavior_tree_node` | `ros2/src/mowgli_bringup/launch/full_system.launch.py` :216, `sim_full_system.launch.py` :190 | plain `rclcpp::Node`, `MultiThreadedExecutor` also spins helper node `_bt_helper_node` (`behavior_tree_node.cpp` :1143-1146) |

Blackboard: `"context"` = `std::shared_ptr<BTContext>`; keys seeded at startup (`behavior_tree_node.cpp` :837-968): `dock_pose`, `undock_pose`, `undock_speed`, `undock_distance`, `idle_nav2_suspend`, `rain_delay_sec`, `rain_mode`, `rain_debounce_sec`, `battery_low_pct`, `battery_critical_pct`, `battery_full_pct`, `battery_critical_voltage`, `battery_critical_recovery_pct`, `mow_angle_deg`, `area_simplification_tolerance`, `area_record_rate_hz`, `bt_tick_rate`. XML consumes `{undock_speed}` `{undock_distance}` `{battery_*_pct}` `{battery_critical_voltage}` `{area_*}` `{current_area_index}`; C++ reads `rain_mode`, `rain_debounce_sec`, `mow_angle_deg`, `idle_nav2_suspend`, `bt_tick_rate`.

### Topics
| Topic | Type | Dir | QoS | Other end / where |
|-------|------|-----|-----|-------------------|
| `/hardware_bridge/status` | `mowgli_interfaces/msg/Status` | sub | 10 | `hardware_bridge_node` `~/status`; `ctx->latest_status` + `last_status_time` (:144) |
| `/hardware_bridge/emergency` | `mowgli_interfaces/msg/Emergency` | sub | 10 | `IsEmergency` treats >2 s silence as emergency (`condition_nodes.cpp` :41-46) |
| `/hardware_bridge/power` | `mowgli_interfaces/msg/Power` | sub | 10 | battery filter → `battery_percent`; `charger_enabled` = `IsCharging` (:204) |
| `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | sub | 10 | twist_mux merged output; last-motion sign for #487 escape (:171) |
| `/map_server_node/replan_needed`, `/boundary_violation`, `/lethal_boundary_violation` | `std_msgs/msg/Bool` | sub | 1 / 10 / 10 | `map_server_node` (:232-269) |
| `/odometry/filtered_map` | `nav_msgs/msg/Odometry` | sub | 5 | `fusion_graph_node`; σ_xy backstop only (:330) |
| `/gps/absolute_pose` | `mowgli_interfaces/msg/AbsolutePose` | sub | 10 | `navsat_to_absolute_pose_node`; `gps_x/y`, undock sample buffer, legacy fix fallback (:345) |
| `/gps/status` | `mowgli_interfaces/msg/GnssStatus` | sub | 10 | GPS bridge container (`sensors/gps/start_gps.sh` :547); authoritative `gps_fix_type` / `gps_is_fixed` (2 s debounce) + LocalizationGuard feed (:427) |
| `/collision_monitor_state` | `nav2_msgs/msg/CollisionMonitorState` | sub | 10 | Nav2 collision_monitor; STOP entry/exit stamps, stale-gap 3 s (:479) |
| `/scan_collision` | `sensor_msgs/msg/LaserScan` | sub | SensorDataQoS | liveness stamp only, for `IsScanStale` (:531) |
| `/global_costmap/costmap` | `nav_msgs/msg/OccupancyGrid` | sub | transient_local reliable(1) | `FollowStrip` detour clearance (`coverage_nodes.cpp` :397) |
| `~/high_level_status` → `/behavior_tree_node/high_level_status` | `mowgli_interfaces/msg/HighLevelStatus` | pub | 10 | `PublishHighLevelStatus` on transitions + 1 Hz republish with live fields (:694-721); GUI `gui/pkg/providers/ros.go` :30 |
| `~/coverage_resume_available` | `std_msgs/msg/Bool` | pub | transient_local(1) | GUI "Resume vs Start fresh" (`ros.go` :63) (:642) |
| `~/recording_trajectory` | `nav_msgs/msg/Path` | pub | transient_local(1) | `RecordArea` preview (`recording_nodes.cpp` :81); GUI `ros.go` :59 |
| `/coverage/full_plan` | `nav_msgs/msg/Path` | pub | transient_local(1) | `PlanCoverageArea` (`coverage_nodes.cpp` :2097) |
| `/controller_server/FollowCoveragePath/global_plan` | `nav_msgs/msg/Path` | pub | transient_local(1) | `FollowStrip` (`coverage_nodes.cpp` :390) |
| `/fusion_graph_node/set_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | pub | transient_local(1) reliable | `fusion_graph_node` `~/set_pose`; `CalibrateHeadingFromUndock` (:202), `SeedYawFromMotion` (:315) |
| `/cmd_vel_teleop` | `geometry_msgs/msg/TwistStamped` | pub | 10 | `SeedYawFromMotion` forward drive (`calibration_nodes.cpp` :305) |
| `/cmd_vel_emergency` | `geometry_msgs/msg/TwistStamped` | pub | 10 | `StopMoving` zero stream (`navigation_nodes.cpp` :101) |
| `/cmd_vel_nav` | `geometry_msgs/msg/TwistStamped` | pub | 10 | `EscapeStartBlocked` (`escape_nodes.cpp` :160) — lowest twist_mux lane, through collision_monitor |

### Services & actions
Served (`behavior_tree_node.cpp` :547-638): `~/high_level_control` (`mowgli_interfaces/srv/HighLevelControl`), `~/start_in_area` (`mowgli_interfaces/srv/StartInArea`), `~/clear_coverage_resume` (`std_srvs/srv/Trigger`, applied at the top of `tickTree` :1004). GUI callers: `gui/pkg/api/mowglinext.go` :564/:594/:691, `gui/pkg/providers/scheduler.go` :166, `homekit.go` :44, `mqtt.go` :118.

Clients (node → file:line):
| Target | Type | Used by |
|--------|------|---------|
| `/hardware_bridge/mower_control` | `mowgli_interfaces/srv/MowerControl` | `SetMowerEnabled` (`utility_nodes.cpp` :63), `FollowStrip::setBladeEnabled` (`coverage_nodes.cpp` :1122) |
| `/hardware_bridge/emergency_stop` | `mowgli_interfaces/srv/EmergencyStop` | `ResetEmergency` (`utility_nodes.cpp` :243) |
| `/map_server_node/get_mowing_area` | `mowgli_interfaces/srv/GetMowingArea` | `PreFlightCheck` (`condition_nodes.cpp` :508), `GetNextUnmowedArea` (:1468), `PlanCoverageArea` (:1903) |
| `/map_server_node/add_area` | `mowgli_interfaces/srv/AddMowingArea` | `RecordArea` (`recording_nodes.cpp` :441) |
| `/map_server_node/get_recovery_point` | `mowgli_interfaces/srv/GetRecoveryPoint` | `NavigateInsideBoundary` (`navigation_nodes.cpp` :415) |
| `/global_costmap/clear_entirely_global_costmap`, `/local_costmap/clear_entirely_local_costmap` | `nav2_msgs/srv/ClearEntireCostmap` | `ClearCostmap` (:143-148), `NavigateInsideBoundary` (:420) |
| `/global_costmap/global_costmap` `set_parameters` (`keepout_filter.enabled`) | rcl_interfaces | `NavigateInsideBoundary` (:405, :490, :735) |
| `/controller_server` `set_parameters` | rcl_interfaces | `SetNavMode` (:922-965): `FollowPath.desired_linear_vel`, `FollowCoveragePath.speed_fast` |
| `/lifecycle_manager_navigation/is_active` | `std_srvs/srv/Trigger` | `Nav2Active` (`condition_nodes.cpp` :605) |
| `/lifecycle_manager_navigation/manage_nodes` | `nav2_msgs/srv/ManageLifecycleNodes` | `SetNav2Lifecycle` (:226) |
| `/obstacle_tracker/save_obstacles` | `std_srvs/srv/Trigger` | `SaveObstacles` (`utility_nodes.cpp` :216) — no server exists in the repo; node skips with SUCCESS |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | readiness poll (:758), `NavigateToPose` (:266), `NavigateInsideBoundary` (:430), `FollowStrip` transit (:383), `TransitToStrip` (:1268), `DetourAroundObstacle` (:1398) |
| `/follow_path` | `nav2_msgs/action/FollowPath` | `FollowStrip` (:379) with `controller_id="FollowCoveragePath"`, `goal_checker_id="coverage_goal_checker"` (:581-582) |
| `/backup` | `nav2_msgs/action/BackUp` | `BackUp` (:815), `NavigateInsideBoundary` fallback (:434) |
| `/dock_robot` / `/undock_robot` | `nav2_msgs/action/DockRobot` / `UndockRobot` | `DockRobot` (`docking_nodes.cpp` :65), `UndockRobot` (:185) + readiness poll (:759) |
| `/plan_coverage` | `mowgli_interfaces/action/PlanCoverage` | `PlanCoverageArea` (`coverage_nodes.cpp` :1914); server `mowgli_coverage` `coverage_server.cpp` :110 |

### Registered BT nodes (source of truth: `src/register_nodes.cpp`)
| Group / file | Nodes |
|--------------|-------|
| Conditions — `condition_nodes.{hpp,cpp}` | `IsEmergency`, `IsCharging`, `IsBatteryLow`(threshold, voltage_threshold), `IsRainDetected`, `NeedsDocking`(threshold %), `IsBatteryAbove`, `IsCommand`(command), `IsGPSFixed`, `IsCoverageComplete`, `ReplanNeeded`†, `IsBoundaryViolation`, `IsLocalizationDegraded`, `IsLethalBoundaryViolation`, `IsDocking`, `IsNewRain`, `IsRainModeAtLeast`(mode), `IsResumeUndockAllowed`(max_attempts), `IsChargingProgressing`, `PreFlightCheck`(min_battery, min_gps_fix_type, tf_timeout_sec), `Nav2Active`(timeout_sec), `IsObstacleStuck`(min_duration_sec, max_count, cooldown_sec), `WasRecentlyInCollisionStop`(max_age_sec), `IsScanStale`(max_age_sec), `IsCollisionStopSustained`(min_duration_sec, max_state_age_sec), `IsCoverageStartBlocked` |
| Utility — `utility_nodes.{hpp,cpp}` | `SetMowerEnabled`(enabled), `WaitForDuration`(duration_sec), `WaitForGpsFix`(timeout_sec, min_fix_type), `SaveObstacles`, `ResetEmergency` |
| Navigation — `navigation_nodes.{hpp,cpp}` | `StopMoving`(duration_sec), `ClearCostmap`, `SetNav2Lifecycle`(command PAUSE/RESUME), `NavigateToPose`†(goal "x;y;yaw"), `BackUp`(backup_dist, backup_speed), `SetNavMode`(mode precise/degraded), `NavigateInsideBoundary` |
| Escape — `escape_nodes.{hpp,cpp}` | `EscapeStartBlocked` |
| Status — `status_nodes.{hpp,cpp}` | `PublishHighLevelStatus`(state, state_name), `WasRainingAtStart`, `ClearCommand`, `EndSession`, `IncrementSkippedSwaths`† |
| Calibration — `calibration_nodes.{hpp,cpp}` | `RecordUndockStart`, `CalibrateHeadingFromUndock`(min_displacement_m), `SeedYawFromMotion`(distance_m, speed_ms, timeout_sec, min_displacement_m) |
| Docking — `docking_nodes.{hpp,cpp}` | `DockRobot`(dock_id, dock_type), `UndockRobot`†(dock_type), `RecordResumeUndockFailure` |
| Coverage — `coverage_nodes.{hpp,cpp}` | `GetNextUnmowedArea`(max_areas → out `area_index`), `FollowStrip`(max_detours_per_segment, detour_footprint_radius_m), `TransitToStrip`, `DetourAroundObstacle`†(forward_m, lateral_m), `PlanCoverageArea`(area_index) |
| Recording — `recording_nodes.{hpp,cpp}` | `RecordArea`(simplification_tolerance, min_vertices, min_area, record_rate_hz, is_exclusion_zone) |

† registered but not referenced by `trees/main_tree.xml`.

`state_name` values emitted by `main_tree.xml` (35): AREA_UNREACHABLE, BOUNDARY_EMERGENCY_STOP, BOUNDARY_RECOVERY, CALIBRATING_HEADING, CHARGER_FAILED, CHARGING, COVERAGE_FAILED_DOCKING, CRITICAL_BATTERY_CHARGING, CRITICAL_BATTERY_DOCKING, CRITICAL_BATTERY_NAV_FAILED, DYNAMIC_OBSTACLE_CLEARED, EMERGENCY, IDLE, IDLE_DOCKED, LOW_BATTERY_DOCKING, MANUAL_MOWING, MOWING, MOWING_COMPLETE, NAV_TO_DOCK_FAILED, OBSTACLE_BACKOFF, PLANNING, PREFLIGHT_CHECK, RAIN_DETECTED_DOCKING, RAIN_TIMEOUT, RAIN_WAITING, RECORDING, RECORDING_COMPLETE, RESUMING_AFTER_RAIN, RESUMING_UNDOCKING, RETURNING_HOME, START_POSE_BLOCKED, TRANSIT, UNDOCK_FAILED, UNDOCKING, WAITING_FOR_RTK.

### Parameters
All declared in `behavior_tree_node.cpp` with `declare_parameter`, read ONCE at startup (no set-parameter callback; restart the node to apply). Defaults per CLAUDE.md Invariant 15 live in `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` and are forwarded by `full_system.launch.py` :216-354 — the C++ defaults below are fallbacks only.
| Param | C++ default (line) | Template line | Consumer |
|-------|--------------------|---------------|----------|
| `tick_rate` | 10.0 (:894) | :93 | tick timer :985; `bt_tick_rate` blackboard |
| `tree_file` | "" → `share/mowgli_behavior/trees/main_tree.xml` (:813) | — | `setupBehaviorTree` |
| `coverage_resume_path` | `/ros2_ws/maps/coverage_resume.txt` (:92) | — | `coverage_persistence.cpp`; "" disables |
| `undock_speed` / `undock_distance` | 0.15 / 1.0 (:853-855) | :433 / :432 | `{undock_speed}` `{undock_distance}` in XML BackUps (Invariant 10) |
| `transit_speed` / `mowing_speed` | 0.2 / 0.2 (:873-874) | :310 / :309 | `SetNavMode` → controller params |
| `idle_nav2_suspend` | false (:864) | :170 | `SetNav2Lifecycle` |
| `rain_mode` / `rain_debounce_sec` / `rain_delay_minutes` | 2 / 0.0 / 30 (:877-891) | :508-510 | `IsNewRain`, `IsRainModeAtLeast` (`rain_delay_sec` blackboard key has no consumer) |
| `battery_full_voltage` / `battery_empty_voltage` | 28.0 / 24.0 (:906-908) | :288-289 | `batteryPercentFromVoltage` |
| `battery_low_percent` / `battery_critical_percent` / `battery_full_percent` / `battery_critical_voltage` / `battery_critical_recovery_percent` | 20 / 10 / 95 / 0 / 30 (:913-925) | :290-294 | XML `{battery_low_pct}` `{battery_critical_pct}` `{battery_full_pct}` `{battery_critical_voltage}`; `battery_critical_recovery_pct` is seeded but unused by XML (CriticalBatteryDock resumes at `{battery_full_pct}`, `main_tree.xml` :394) |
| `loc_gnss_acc_pause_m` / `loc_gnss_acc_resume_m` / `loc_gnss_stale_s` / `loc_sigma_pause_persist_s` / `loc_sigma_resume_persist_s` / `loc_sigma_pause_m` / `loc_sigma_resume_m` / `loc_sigma_backstop_persist_s` | 0.30 / 0.15 / 5 / 3 / 2 / 5 / 2 / 10 (:286-294) | :148-163 | `LocalizationHealthMonitor` → `IsLocalizationDegraded` |
| `start_blocked_escape_enabled` / `_speed` / `_distance` / `_timeout_s` / `_min_signal_speed` / `_signal_max_age_s` | true / 0.10 / 0.40 / 6 / 0.03 / 90 (:309-316), clamped by `SanitizeEscapeCfg` | :546-558 | `EscapeStartBlocked`, `/cmd_vel` tracker |
| `mow_angle_deg` | -1 = AUTO (:947) | :338 | `PlanCoverageArea::buildGoal` → `PlanCoverage.mow_angle_deg` |
| `area_simplification_tolerance` / `area_record_rate_hz` | 0.05 / 10 (:958-961) | :115 / :129 | `RecordArea` ports via XML |
| `bt_debug_logging` | false (:973) | :164 | `BT::StdCoutLogger` |
| `dock_pose` / `undock_pose` / `chassis_width` / `chassis_length` | (:841-842, :899-900) | — | declared; no consumer in this package |

### TF frames
Publishes none. `ctx->tf_buffer` (`behavior_tree_node.cpp` :82) is used to look up `map → base_footprint` (`PreFlightCheck` `condition_nodes.cpp` :492, coverage/detour pose queries). Never `base_link` (Invariant 2).

## Build, test, run
- Build only this package (inside the dev container): `cd ros2 && make build-pkg PKG=mowgli_behavior` (`ros2/Makefile` :64 → `scripts/build.sh`), or `colcon build --packages-select mowgli_behavior`.
- Unit tests: `cd ros2 && PACKAGES="mowgli_behavior" ./scripts/test.sh`, or `colcon test --packages-select mowgli_behavior && colcon test-result --verbose`. All 19 gtest targets are pure/in-process (some spin a fake service) — no hardware, no Gazebo. `test_guard_fallthrough` and `test_start_occupied_retry` read the real `trees/main_tree.xml` via `MOWGLI_MAIN_TREE_PATH`.
- Integration: `ros2/src/mowgli_bringup/test/test_nodes_startup.launch.py` (`add_launch_test`, `mowgli_bringup/CMakeLists.txt` :46) launches `behavior_tree_node` as `mowgli_behavior_node` and probes `/mowgli_behavior_node/high_level_status` + `/mowgli_behavior_node/high_level_control`.
- E2E (sim): `cd ros2 && make e2e-test` → `ros2/src/e2e_test.py` calls `/behavior_tree_node/high_level_control` (:215) and watches `/behavior_tree_node/high_level_status` (:173). `docker/docker-compose.simulation.yaml` :77-78 bind-mounts `trees/` and `config/` so XML edits need no rebuild.
- CI: `.github/workflows/ros2-ci.yml` "Build workspace" / "Run tests" steps (:335-350) run the whole workspace `colcon build` + `colcon test --return-code-on-test-failure`. Format: `cd ros2 && make format` (clang-format 18 pinned in CI; uncrustify disabled in `CMakeLists.txt` :112).
- Run live: `ros2 service call /behavior_tree_node/high_level_control mowgli_interfaces/srv/HighLevelControl "{command: 1}"`; watch `ros2 topic echo /behavior_tree_node/high_level_status`; set `bt_debug_logging: true` in the installed `mowgli_robot.yaml` for per-transition BT logs.

## Change coupling — "if you change X, also update Y"
- New/renamed BT node → `src/register_nodes.cpp` AND `trees/main_tree.xml`; add a gtest target in `CMakeLists.txt` (link only the `.cpp` files the test needs — see existing targets).
- New blocking guard in `main_tree.xml` → must end with `<AlwaysFailure/>`; `test/test_guard_fallthrough.cpp` structurally asserts this. Touching `StartPoseBlockedRetry` → `test/test_start_occupied_retry.cpp`.
- New `state_name` in `main_tree.xml` → GUI maps `gui/web/src/components/dashboard/constants.ts` (`MOWER_STATES`), `gui/web/src/components/utils.tsx`, `gui/web/src/components/BTStateGraph.tsx` (+ `BTStateGraph.test.tsx`), and `wiki/Behavior-Trees.md`.
- New `HighLevelControl` command → `ros2/src/mowgli_interfaces/srv/HighLevelControl.srv`, the handler in `behavior_tree_node.cpp` :547, an `IsCommand` branch in `main_tree.xml`, the guard whitelists (`SensorSafetyGuard` :86-89, `BoundaryGuard` :155-182, `LocalizationGuard` :262-270), GUI bindings (see `docs/claude/commands.md` codegen), and `docs/claude/high-level-api.md`.
- New `HighLevelStatus` field → `status_snapshot.cpp` (`withLiveStatusFields`) or it will be frozen by the 1 Hz republish; `test/test_high_level_status_snapshot.cpp`; firmware `HL_MODE_*` mirror if `state` values change (see `docs/claude/high-level-api.md`).
- New node parameter → declare in `behavior_tree_node.cpp`, add the default to the TEMPLATE `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` (never the sparse installed file, Invariant 15), forward it in `full_system.launch.py` :216-354 (otherwise the node silently runs its C++ default), and — if operator-facing — `gui/web/src/components/settings/paramCatalog.ts`.
- `kSegmentTransitGap` is single-sourced from `ros2/src/mowgli_interfaces/include/mowgli_interfaces/coverage_geometry.hpp` :26 (`kSegmentTransitGapM = 0.6`) — `mowgli_coverage` splits sub-paths on the same constant; `test/test_coverage_transit_gap.cpp` pins it.
- `coverage_resume.txt` layout → bump `kHeader` in `coverage_persistence.cpp` :34 (unknown header = start fresh) and `test/test_coverage_persistence.cpp`.
- `FollowStrip` goal ids `FollowCoveragePath` / `coverage_goal_checker` (`coverage_nodes.cpp` :581-582) must match `controller_server` plugin names in `ros2/src/mowgli_bringup/config/nav2_params_base.yaml`; `SetNavMode` param names (`FollowPath.desired_linear_vel`, `FollowCoveragePath.speed_fast`) must match the RPP/FTC plugin params there.
- `navigate_to_pose.xml` `GoalCheckerSelector` default `stopped_goal_checker` and the selector topics must match `controller_server` `goal_checker_plugins` in `nav2_params_base.yaml`; the path is injected by `navigation.launch.py` :626.
- Topic names consumed here are hard-coded absolute: `map_server_node` `~/boundary_violation` etc. (`map_server_node.cpp` :363-367), `hardware_bridge` `~/status|emergency|power|mower_control|emergency_stop` (`hardware_bridge_node.cpp` :704-818), `fusion_graph_node` `~/set_pose` (`fusion_graph_node_setup_comms.cpp` :196). Renaming a node name on either side breaks the guard silently (see the `/map_server/…` regression note at `behavior_tree_node.cpp` :242-246).
- `EscapeStartBlocked` ceilings (`start_blocked_escape.hpp` :99-105) and `dig_*` in `hardware_bridge` are independent safety envelopes — CLAUDE.md Invariant 16 / What NOT to Do.

## Pitfalls
- The Root is a `ReactiveSequence`: every guard handler MUST terminate with `<AlwaysFailure/>` or a `SUCCESS` wait lets MainLogic run one tick per cycle (livelock #459/#445; `main_tree.xml` :282-299). `test_guard_fallthrough` enforces it.
- `IsEmergency` returns SUCCESS when `/hardware_bridge/emergency` is >2 s stale (`condition_nodes.cpp` :41-46) — a dead bridge looks like an emergency, by design.
- `SensorSafetyGuard`/`IsCollisionStopSustained` must stay freshness-gated (`max_state_age_sec`): collision_monitor only publishes while `cmd_vel_nav` flows, so a halted tree leaves `collision_action_type` as a stale STOP latch (`bt_context.hpp` :453-460).
- LocalizationGuard keys on `/gps/status` quality, NOT fused covariance; `loc_sigma_pause_m` must stay above fusion_graph's pivot inflation (~2.35 m) or mowing livelocks at 0 % (`behavior_tree_node.cpp` :271-295; `localization_health.hpp`).
- Coverage-tracking maps on `BTContext` are deliberately NOT under `context_mutex`; they rely on the default MutuallyExclusive callback group serialising tick/service/timer callbacks (`bt_context.hpp` :77-103). Do not add locking back, and do not move callbacks to a Reentrant group without re-deriving this. `~/clear_coverage_resume` is deferred to `tickTree` for the same reason (:612-638).
- `FollowStrip::sendCurrentSwath` forces the blade OFF before any transit >`kSegmentTransitGap` (`coverage_nodes.cpp` :614-640) — structural blade safety; never add a blade-on path around it.
- `EscapeStartBlocked` only moves behind `IsCoverageStartBlocked`'s arming token (≤30 s old, `bt_context.hpp` :212-228), with blade verified off from a fresh `/hardware_bridge/status`; the `/cmd_vel` tracker ignores samples during `last_motion_suppress_until` so the escape never becomes "the last motion".
- Persisted `current_command` auto-continues mowing on boot ONLY when it was `COMMAND_START` AND a resumable snapshot exists (a resume cursor OR a non-empty completed-area set); anything else is forced to IDLE (`behavior_tree_node.cpp` :93-118). `EndSession` deletes the file; a low-battery dock keeps it.
- `COMMAND_S2` (4) is normalised to `COMMAND_START` in the service handler (:561); there is no separate "next area" branch. A plain `COMMAND_START` clears single-area mode (:579-582); `~/start_in_area` sets `current_command` itself.
- `config/behavior_tree.yaml` `battery_low_pct` / `battery_critical_pct` are not declared parameter names; the real ones are `battery_low_percent` / `battery_critical_percent` forwarded from `mowgli_robot.yaml`. `sim_full_system.launch.py` :190-200 forwards ONLY `behavior_tree.yaml` + `use_sim_time`, so the sim BT runs the C++ fallbacks for every `mowgli_robot.yaml`-sourced knob.
- `BTContext::dock_x/dock_y/dock_yaw` have no writer in this package; `DockRobot::log_contact_delta` (`docking_nodes.cpp` :28) logs against (0,0,0) and `CalibrateHeadingFromUndock::warn_if_dock_yaw_stale` early-returns on `dock_yaw == 0.0` (`calibration_nodes.cpp` :248).
- `SaveObstacles` targets `/obstacle_tracker/save_obstacles`, which nothing in the repo serves — it always logs "service unavailable, skipping" and returns SUCCESS (`utility_nodes.cpp` :219-224).
- Undock is `BackUp` via `/backup`, not `UndockRobot` (Invariant 10); `undock_distance` must exceed `CalibrateHeadingFromUndock` `min_displacement_m` (`main_tree.xml` :534 passes 0.5; the C++ port default is 0.20) or the yaw refinement is skipped — and if the charger is STILL on at that point the node returns FAILURE and the whole undock sequence retries (`calibration_nodes.cpp` :124-152).
- `PreFlightCheck` requires only `min_gps_fix_type=2` on the dock; RTK-Fixed is enforced after undock by `WaitForGpsFix(min_fix_type=4)` (`main_tree.xml` :489-528). Do not raise the dock gate (chicken-and-egg under the canopy).
- `GetNextUnmowedArea` skips `is_navigation_area` areas (`coverage_nodes.cpp` :1693-1698) and retires an area after `kMaxAreaAttempts=5` no-progress dispatches (+`kMaxStartBlockedAttempts=3` exempted START_OCCUPIED passes) — `bt_context.hpp` :141-202.
- `BoundaryGuard`/`LocalizationGuard` whitelist commands 7/3/5/6/2 and the blade-off dock transit (`IsCommand 1` + `IsDocking`); dropping 5/6 from the whitelist silently loses every finished recording (`main_tree.xml` :146-154).
- `GUI` state maps: `WAITING_FOR_RTK` is emitted but present in none of `constants.ts` / `utils.tsx` / `BTStateGraph.tsx`; `SKIP_STRIP` is listed there but never emitted.

## Generated & vendored — do not hand-edit
- Nothing generated inside `ros2/src/mowgli_behavior/`. Message/service/action types come from `ros2/src/mowgli_interfaces` (`HighLevelStatus.msg`, `HighLevelControl.srv`, `StartInArea.srv`, `PlanCoverage.action`, …); after editing those, regenerate GUI bindings per `docs/claude/commands.md`.
- BehaviorTree.CPP v4 and Nav2 (`nav2_msgs`, `bt_navigator` stock nodes used by `navigate_to_pose.xml`) are system packages, not vendored here.
