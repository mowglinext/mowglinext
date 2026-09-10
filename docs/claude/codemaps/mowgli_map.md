# Codemap: mowgli_map

> `mowgli_map` is the map-side authority of the ROS2 stack: `map_server_node` owns the recorded
> area polygons (+ per-area obstacle keepouts), persists them to `areas.dat` with a WGS84 datum
> stamp, rasterises them into the Nav2 keepout mask, keeps the "actually mowed" `mow_progress`
> overlay, hosts the dock pose gates, and turns operator/tracker/dig-detector input into
> permanent or pending keepouts. `obstacle_tracker_node` clusters the global costmap into
> tracked obstacles the operator can promote. Index generated 2026-09-03 at f21729e9;
> regenerate when files are added/removed. Loaded on demand from `ros2/CLAUDE.md`.

## Where to look
| Task | Start here |
|------|------------|
| Add / rename a `map_server_node` publisher, subscriber, service or parameter | `ros2/src/mowgli_map/src/map_server_node.cpp` constructor `MapServerNode::MapServerNode` (L51-493); members in `include/mowgli_map/map_server_node.hpp` (L515-889) |
| Change how `areas.dat` is written / read (keys, obstacle identity lines) | `ros2/src/mowgli_map/src/area_manager.cpp` `save_areas_to_file` (L1381) / `load_areas_from_file` (L1460); mirror in `install/scripts/migrate_openmower.py` `write_areas_dat` (L286) |
| Datum stamp / datum-change migration (issue #216) | `area_manager.cpp` `migrate_areas_datum` (L1581), constants `kDatumUnsetEpsilonDeg`/`kDatumMatchEpsilonDeg` (L55-60); math in `ros2/src/mowgli_interfaces/include/mowgli_interfaces/wgs84_projection.hpp` `ReprojectEnu` (L57) |
| `~/add_area` behaviour (LAWN/NO_GO stamping, auto-save) | `area_manager.cpp` `on_add_area` (L459) |
| `~/get_mowing_area` payload (static obstacles + tracker polygons + `obstacle_info`) | `area_manager.cpp` `on_get_mowing_area` (L565); msg `ros2/src/mowgli_interfaces/msg/MapArea.msg`, `MapObstacleInfo.msg` |
| Dock pose set gates (charging / RTK σ / yaw convergence / GPS averaging / yaw_source) | `area_manager.cpp` `on_set_docking_point` (L640-956); write-back via `mowgli_interfaces/robot_yaml_scalar.hpp` `UpdateDockPose` (L122) |
| Dock body / corridor polygons (lethal body, carved corridor) | `map_server_node.cpp` `rebuild_dock_polygons` (L495); classification in `area_manager.cpp` `apply_area_classifications` (L1758-1797); mask carve `src/costmap_filters.cpp` L334-364 |
| Keepout mask rasterisation, outside-slack band, obstacle margin, Invariant-14 index mapping | `ros2/src/mowgli_map/src/costmap_filters.cpp` `publish_keepout_mask` (L46-385); `kOutsideSlackMaskCost` (L44) |
| `lethal_outside_areas` vs legacy `keepout_nav_margin` policy | `costmap_filters.cpp` L97-98; defaults `config/map_server.yaml` L76-112 |
| Promote a tracker observation / free-form polygon to a permanent keepout | `area_manager.cpp` `on_promote_obstacle` (L1014); `src/progress_tracker.cpp` `apply_promoted_obstacle` (L272) |
| Wheel-slip dig → PENDING keepout; accept / discard proposals | `area_manager.cpp` `on_dig_event` (L1120), `accept_pending_obstacle` (L1207), `discard_pending_obstacle` (L1237), `on_discard_obstacle` (L1281) |
| Obstacle de-duplication rule (centroid ε = 0.10 m) | `include/mowgli_map/internal_helpers.hpp` `kObstacleDedupEpsilonM` (L41), `has_duplicate_obstacle` (L78) |
| Dig keepout size floor / default | `internal_helpers.hpp` `kDefaultDigKeepoutSizeM` (L48), `kMinDigKeepoutSizeM` (L52) |
| Mow-progress stamping (blade-verified swept disc) | `map_server_node.cpp` `on_odom` (L624-717), `stamp_mow_progress` (L881); gate `include/mowgli_map/mow_progress.hpp` `GetMowProgressInhibitReason` (L32) |
| `~/mow_progress` publish throttle / cache | `map_server_node.cpp` `on_publish_timer` (L842), `rebuild_mow_progress_cache` (L947) |
| Boundary violation (soft debounce vs lethal) | `progress_tracker.cpp` `check_boundary_violation` (L63); pure classifier `include/mowgli_map/boundary_classifier.hpp` `ClassifyBoundary` (L45) |
| `~/get_recovery_point` pose computation | `progress_tracker.cpp` `on_get_recovery_point` (L155) |
| Map grid geometry / resize to fit areas | `area_manager.cpp` `init_map` (L178), `resize_map_to_areas` (L202, 5 m margin L227) |
| Binary map save/load (`map_file_path`, off by default) | `area_manager.cpp` `on_save_map` (L263), `on_load_map` (L330) |
| CellType enum / layer names | `include/mowgli_map/map_types.hpp` `CellType` (L27-41), `layers::` (L69-72) |
| Obstacle tracker clustering, association, promotion | `ros2/src/mowgli_map/src/obstacle_tracker_node.cpp` `process_costmap` (L258), `associate_clusters` (L1126, `association_dist` 0.5 m L1129), `promote_persistent` (L1313) |
| Obstacle tracker keepout re-detection suppression | `obstacle_tracker_node.cpp` `on_keepout_mask` (L471), `centroid_in_keepout_lethal` (L478) |
| Obstacle tracker boundary fetch / inset | `obstacle_tracker_node.cpp` `fetch_boundary` (L738-836) |
| `obstacles.yaml` persistence | `obstacle_tracker_node.cpp` `save_to_file` (L1364), `load_from_file` (L1403) |
| Tune tracker thresholds | `ros2/src/mowgli_map/config/obstacle_tracker.yaml` |
| Which launch injects which map_server param | `ros2/src/mowgli_bringup/launch/full_system.launch.py` L366-436; sim `sim_full_system.launch.py` L204-235 |

## Files
| File | Lines | Purpose |
|------|-------|---------|
| **`ros2/src/mowgli_map/`** | | |
| `CMakeLists.txt` | 264 | Two static libs (`mowgli_map_lib`, `mowgli_obstacle_tracker_lib`), two executables, four gtest targets |
| `package.xml` | 41 | ament_cmake deps (grid_map_*, nav2_msgs, map_msgs, tf2, Boost) |
| **`include/mowgli_map/`** | | |
| `map_server_node.hpp` | 893 | `MapServerNode` class: `AreaEntry`/`ObstacleEntry`, all members, test-only accessors (L96-239) |
| `map_types.hpp` | 84 | `CellType` enum, `cell_type_name`, `layers::OCCUPANCY/CLASSIFICATION`, defaults |
| `mow_progress.hpp` | 86 | Header-only `MowProgressInhibitReason`, `GetMowProgressInhibitReason`, `SweepStepCount` |
| `boundary_classifier.hpp` | 71 | Header-only `ClassifyBoundary` (soft debounce + undebounced lethal) |
| `internal_helpers.hpp` | 153 | Package-private: dedup ε, dig size constants, `polygon_centroid`, `closest_edge_point`, `point_to_polygon_distance` |
| `obstacle_tracker_node.hpp` | 277 | `ObstacleTrackerNode` class + `TrackedObstacle` struct; friend `ObstacleTrackerAlgorithmTest` |
| **`src/`** | | |
| `map_server_node.cpp` | 992 | Constructor (params, pubs, subs, services), dock polygons, `/map`/status/odom/costmap/tracker callbacks, publish timer, mow-progress stamping |
| `area_manager.cpp` | ~1.8k | Area params, map init/resize, save/load map + areas, `add_area`, `get_mowing_area`, `set_docking_point`, promote/dig/discard, datum migration, `apply_area_classifications` |
| `costmap_filters.cpp` | 387 | `publish_keepout_mask` + one-shot `CostmapFilterInfo` |
| `progress_tracker.cpp` | 324 | `point_in_polygon`, `check_boundary_violation`, `on_get_recovery_point`, `apply_promoted_obstacle` |
| `main.cpp` | 32 | `map_server_node` executable |
| `obstacle_tracker_node.cpp` | ~1.55k | Costmap flood-fill clustering, DBSCAN (map path), hulls, association, merge, promotion, YAML persistence |
| `obstacle_tracker_main.cpp` | 26 | `obstacle_tracker_node` executable |
| **`config/`** | | |
| `map_server.yaml` | 176 | `map_server_node` defaults loaded by both launchers (`map_params`); heavily commented tuning history |
| `obstacle_tracker.yaml` | 18 | Garden-tuned tracker thresholds (`persistence_threshold: 10.0`, `persistence_file: /ros2_ws/maps/obstacles.yaml`) |
| `map_server_small_garden.yaml` | 17 | Alt config: 9×7 m single area (not referenced by any launch file) |
| `map_server_obstacle_test.yaml` | 18 | Alt config: 6×6 m area with two obstacles, `areas_file_path: ""` (not referenced by any launch file) |
| **`test/`** | | |
| `test_map_server.cpp` | ~1.2k | Layers/geometry, area types, promote idempotence, dig proposals, keepout mask policy, obstacle margin, datum migration, areas.dat identity round-trip |
| `test_mow_progress.cpp` | 107 | Inhibit gate, sweep step count, full sweep stamping, reset semantics, cache invalidation |
| `test_boundary_classifier.cpp` | 140 | Pure `ClassifyBoundary` cases (no rclcpp) |
| `test_obstacle_tracker.cpp` | 433 | Hull/DBSCAN/merge/point-in-polygon algorithms + keepout suppression (one shared node per suite) |

## Runtime surface

### Nodes
| Node name | Executable | Launched by | Kind |
|-----------|------------|-------------|------|
| `map_server_node` | `map_server_node` | `full_system.launch.py` L366 (unconditional), `sim_full_system.launch.py` L204; `mowgli_bringup/test/test_nodes_startup.launch.py` L71 as name `map_server` | plain `rclcpp::Node`, wall timer at `publish_rate` |
| `obstacle_tracker` | `obstacle_tracker_node` | `full_system.launch.py` L636 gated on `use_obstacle_tracker` (default `true`) AND `use_lidar`; sim L282 gated on `use_lidar` | plain `rclcpp::Node`, publish timer `publish_rate` + 5 s boundary-fetch timer |

### Topics
`~/` resolves to `/map_server_node/` and `obstacle_tracker/` to `/obstacle_tracker/`.

| Topic | Type | Dir | QoS | Other end |
|-------|------|-----|-----|-----------|
| `~/mow_progress` | `nav_msgs/OccupancyGrid` | pub | depth 1, transient_local | GUI `gui/pkg/providers/ros.go` L53, `gui/web/src/hooks/useMowProgress.ts` |
| `/keepout_mask` | `nav_msgs/OccupancyGrid` | pub | depth 1, transient_local | Nav2 `keepout_filter` (both overlays); `obstacle_tracker` (`keepout_topic`) |
| `/costmap_filter_info` | `nav2_msgs/CostmapFilterInfo` | pub (once) | depth 1, transient_local | `nav2_params_base.yaml` L782-785 `keepout_filter.filter_info_topic` |
| `~/replan_needed` | `std_msgs/Bool` | pub | depth 1 | `mowgli_behavior/src/behavior_tree_node.cpp` L233 |
| `~/boundary_violation` | `std_msgs/Bool` | pub (every odom tick) | depth 1 | `behavior_tree_node.cpp` L248; `ros2/src/e2e_test.py` L209 |
| `~/lethal_boundary_violation` | `std_msgs/Bool` | pub (every odom tick) | depth 1 | `behavior_tree_node.cpp` L262 |
| `~/docking_pose` | `geometry_msgs/PoseStamped` | pub | depth 1, transient_local | GUI `gui/pkg/providers/ros.go` L360 |
| `/map` | `nav_msgs/OccupancyGrid` | sub → occupancy layer | depth 1 | **no publisher in `ros2/src`** (dead input; see Pitfalls) |
| `/hardware_bridge/status` | `mowgli_interfaces/Status` | sub | depth 1 | `mow_enabled`, `mower_esc_status`, `mower_motor_rpm`, `blade_status_stamp`, `is_charging` |
| `odom_topic` (default `/odometry/filtered_map`) | `nav_msgs/Odometry` | sub (tick only; pose taken from TF) | depth 1 | `fusion_graph_node` |
| `costmap_topic` (default `/global_costmap/costmap`) | `nav_msgs/OccupancyGrid` | sub → cached | depth 1, reliable | Nav2 global costmap; consumer `is_costmap_blocked` has **no callers** |
| `/gps/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | sub | SensorDataQoS | `navsat_to_absolute_pose_node`; only feeds `set_docking_point` gates/averaging |
| `/obstacle_tracker/obstacles` | `mowgli_interfaces/ObstacleArray` | sub (snapshot for id→polygon) | depth 1 | `obstacle_tracker` |
| `/hardware_bridge/dig_event` | `mowgli_interfaces/DigEvent` | sub (only if `dig_obstacle_enabled`) | depth 10, transient_local (matches `hardware_bridge_node.cpp` L730-732) | `hardware_bridge_node` |
| `obstacle_tracker/obstacles` | `mowgli_interfaces/ObstacleArray` | pub (1 Hz, all tracked, `status` TRANSIENT/PERSISTENT) | depth 1 | `map_server_node`, GUI `ros.go` L57, `TrackedObstaclesPanel.tsx` |
| `obstacle_tracker/markers` | `visualization_msgs/MarkerArray` | pub | depth 1 | Foxglove |
| `/global_costmap/costmap` + `/global_costmap/costmap_updates` | `OccupancyGrid` / `map_msgs/OccupancyGridUpdate` | sub (tracker cluster source, cost ≥ 50) | depth 1 | Nav2 global costmap (delta mode) |
| `map_topic` (default `/map`) | `nav_msgs/OccupancyGrid` | sub (tracker DBSCAN path) | KeepLast(1), transient_local | dead — nothing publishes `/map` |

### Services & actions
No actions. All `map_server_node` services are declared in `map_server_node.cpp` L290-421.

| Service | Type | Callers |
|---------|------|---------|
| `~/add_area` | `mowgli_interfaces/srv/AddMowingArea` | `mowgli_behavior/src/recording_nodes.cpp` L441; GUI `gui/pkg/api/mowglinext.go` L114, L189 |
| `~/get_mowing_area` | `mowgli_interfaces/srv/GetMowingArea` (index → `MapArea` + `obstacle_info`) | `coverage_nodes.cpp` L1469/L1904, `condition_nodes.cpp` L509 (readiness probe), `obstacle_tracker_node.cpp` L193, GUI `ros.go` L417 |
| `~/set_docking_point` | `mowgli_interfaces/srv/SetDockingPoint` (`use_gps_position`, `yaw_source` PRESERVE/REQUEST/MOTION, `yaw_rad`) | `mowgli_localization/src/calibrate_imu_yaw_node.cpp` L310; GUI `mowglinext.go` L238 |
| `~/promote_obstacle` | `mowgli_interfaces/srv/PromoteObstacle` (`pending_id` path, else `polygon`, else `obstacle_id` lookup) | GUI `mowglinext.go` L628 |
| `~/discard_obstacle` | `mowgli_interfaces/srv/ClearObstacle` (`obstacle_id` = pending id) | GUI `mowglinext.go` L651 |
| `~/get_recovery_point` | `mowgli_interfaces/srv/GetRecoveryPoint` | `mowgli_behavior/src/navigation_nodes.cpp` L415 (`NavigateInsideBoundary`) |
| `~/save_areas`, `~/load_areas`, `~/clear_map` | `std_srvs/srv/Trigger` | GUI `mowglinext.go` L138 (`DELETE /map` → clear_map), L176/L189/L193 (OpenMower import: clear_map → add_area×N → save_areas → set_docking_point); `~/load_areas` has no caller found |
| `~/save_map`, `~/load_map` | `std_srvs/srv/Trigger` | no caller found; no-op unless `map_file_path` set. `test_nodes_startup.launch.py` L204 still asserts `/map_server/save_map` is *advertised*, so removing it reddens CI |
| `obstacle_tracker/clear_obstacle` | `mowgli_interfaces/srv/ClearObstacle` | none found |
| `obstacle_tracker/clear_all`, `obstacle_tracker/save`, `obstacle_tracker/load` | `std_srvs/srv/Trigger` | BT `utility_nodes.cpp` L216 calls `/obstacle_tracker/save_obstacles` — **name mismatch**, never resolves |
| client → `/map_server_node/get_mowing_area` | | `obstacle_tracker_node.cpp` L193, index 0 only, retried every 5 s |

### Parameters
Defaults: `ros2/src/mowgli_map/config/map_server.yaml` (`map_params`, `full_system.launch.py` L173) overlaid by launch-injected values from `mowgli_robot.yaml` (`full_system.launch.py` L366-436: `dock_pose_x/y/yaw`, `dock_body_length_m/width_m`, `chassis_width`, `max_obstacle_avoidance_distance`, `obstacle_margin`, `lethal_outside_areas`, `enforce_boundary_margin_m`, `tool_width`, `datum_lat/lon`). All declared in `map_server_node.cpp` unless noted; read once at construction except the "dynamic" rows.

| Parameter | Default | Declared at | Notes |
|-----------|---------|-------------|-------|
| `resolution`, `map_size_x`, `map_size_y`, `map_frame` | 0.05, 20, 20, `map` | L55-58 | size overridden by `resize_map_to_areas` (+5 m margin) |
| `tool_width` | 0.18 | L59 | mow-progress disc radius = `tool_width/2` (see CLAUDE.md Invariant 6) |
| `dig_obstacle_enabled`, `dig_obstacle_size` | true, 0.60 | L70-71 | subscription only created when enabled; size floored at 0.05 |
| `areas_file_path` | `""` (yaml: `/ros2_ws/maps/areas.dat`) | L78 | empty = no persistence at all |
| `datum_lat`, `datum_lon` | 0/0 | L83-84 | 0/0 disables stamp + migration |
| `robot_yaml_path` | `/ros2_ws/config/mowgli_robot.yaml` | L85 | dock-pose splice target; tests redirect |
| `publish_rate`, `mow_progress_publish_period_s` | 1.0, 2.0 | L86-87 | |
| `mow_progress_tool_frame`, `mow_progress_min_blade_rpm`, `mow_progress_blade_telemetry_max_age_s` | `blade_link`, 1000, 1.0 | L88-92 | all three gate stamping |
| `keepout_nav_margin` | 0.45 | L93 | only honoured when `lethal_outside_areas=false` |
| `lethal_outside_areas`, `enforce_boundary_margin_m` | true, 0.40 | L101-102 | outside slack band = mask 50, not 0 |
| `lethal_boundary_margin_m`, `soft_boundary_margin_m`, `boundary_debounce_samples` | 0.5, 0.30, 3 | L109-134 | yaml has no override; code defaults rule |
| `boundary_recovery_offset_m`, `boundary_inner_margin_m` | 0.8, 0.0 | L135-136 | |
| `obstacle_margin` | 0.15 clamped [0,1] | L153 | lethal band around drawn obstacles; template `mowgli_robot.yaml` L665 = 0.2 |
| `dock_body_length_m/width_m`, `dock_approach_corridor_length_m/half_width_m` | 0.80/0.55, 1.5/0.40 | L159-170 | |
| `auto_promote_persistent_obstacles` | false | L180-181 | true = tracker PERSISTENT auto-stamped once per id |
| `odom_topic`, `costmap_topic`, `costmap_obstacle_threshold`, `costmap_max_age_s` | see Topics, 99, 2.0 | L231-249 | |
| `dock_set_gps_accuracy_max_m`, `dock_set_gps_max_age_s`, `dock_set_status_max_age_s` | 0.04, 2.0, 3.0 | L262-267 | **dynamic** (`get_parameter` in `area_manager.cpp` L655-679) |
| `yaw_convergence_threshold_rad/window_s/min_samples` | 0.00873, 5.0, 20 | L72-76 | **dynamic** (`map_server_node.cpp` L658, `area_manager.cpp` L729-732) |
| `dock_pose_x/y/yaw` | 0/0/0 | L429-431 | all-zero = "no dock"; read BEFORE areas load so migration can move it |
| `area_names/area_polygons/area_is_navigation/area_obstacles` | empty | `area_manager.cpp` L104-111 | file load (if any) replaces them |
| `strip_boundary_margin_m`, `mow_angle_deg`, `chassis_width`, `bypass_safety_margin_m`, `max_obstacle_avoidance_distance` | — | L137-149 | **declared, stored, never read** (leftovers of the removed strip planner) |
| tracker: `cluster_tolerance`, `min_cluster_points`, `persistence_threshold`, `transient_timeout`, `min/max_obstacle_radius`, `inflation_radius`, `persistence_file`, `map_frame`, `publish_rate`, `map_topic`, `occupied_threshold`, `map_obstacle_min_dist_from_boundary`, `boundary_margin`, `keepout_topic`, `keepout_lethal_threshold` | `obstacle_tracker_node.cpp` L50-66 | `obstacle_tracker.yaml` overrides 12 of them (not `map_topic`, `occupied_threshold`, `keepout_*`) | promotion needs age ≥ `persistence_threshold` AND ≥ 50 % of `age*publish_rate` observations AND ≥ 3 |

### TF frames
- `map_server_node` looks up `map_frame → base_footprint` on every odom tick (`map_server_node.cpp` L634) for boundary checks / yaw window, and `map_frame → mow_progress_tool_frame` (`blade_link`, L695) for stamping. Publishes no TF.
- `obstacle_tracker` creates a TF buffer (L82-83) but performs no lookup; both cluster sources are already in `map_frame`.

## Build, test, run
```bash
cd ros2 && make build-pkg PKG=mowgli_map          # ./scripts/build.sh, PACKAGES=mowgli_map
cd ros2 && make test                              # ./scripts/test.sh (whole workspace)
# direct colcon (inside the devcontainer / ros:kilted image):
colcon build --packages-select mowgli_map --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test  --packages-select mowgli_map --return-code-on-test-failure && colcon test-result --verbose
# single binary after build:
./build/mowgli_map/test_map_server --gtest_filter='DigProposalTest.*'
```
CI: `.github/workflows/ros2-ci.yml` job `build-and-test` runs `colcon build` + `colcon test --return-code-on-test-failure` over the whole workspace (L336-350); results uploaded from `ros2/build/**/test_results`. Integration: `mowgli_bringup/test/test_nodes_startup.launch.py` (launch_testing, `mowgli_bringup/CMakeLists.txt` L46) starts `map_server_node` with default params.

| Test target | File | Pins |
|-------------|------|------|
| `test_map_server` | `test/test_map_server.cpp` | `MapServerTest`: 2 layers + geometry + defaults + `clear_map`; `AreaTypeTest`: navigation vs mowing areas, save/load round-trip, `PromoteObstacleIsIdempotent`, dig inside/outside/nav-only/repeat, `mowing_area_containing`, keepout mask lethal-outside / nav-allowed / empty, drawn obstacle edge-tight; `ObstacleMarginTest` (`obstacle_margin` 0.3 band); `DatumMigrationTest` (stamp on save, same-datum no-op, re-projection of areas+obstacles+dock, unstamped legacy adopt, 0/0 never migrates — uses `robot_yaml_path` temp file); `DigProposalTest` (dig never reaches areas.dat, pending still lethal, accept persists with `_source: 2`, unknown id fails, discard writes nothing, accepted cannot be discarded, name round-trip, legacy file loads) |
| `test_mow_progress` | `test/test_mow_progress.cpp` | inhibit-reason truth table, `SweepStepCount`, straight-sweep stamping, no sweep across reset, cache invalidation |
| `test_boundary_classifier` | `test/test_boundary_classifier.cpp` | soft needs N consecutive samples, lethal immediate, reset on inside, exact-margin is inside, counter saturates |
| `test_obstacle_tracker` | `test/test_obstacle_tracker.cpp` | `boundary_hull` L/circle/small, `convex_hull`, `inflate_hull`, `merge_overlapping`, DBSCAN, point-in-polygon, `ClusterOnKeepoutCellIsDropped`, `NoKeepoutMaskFailsOpen` |

All node tests call handlers directly through the `*_for_test` accessors (`map_server_node.hpp` L96-239) — no executor spin, no DDS peers.

## Change coupling — "if you change X, also update Y"
- `areas.dat` key format (`area_<i>_…`, `_obstacle_<j>[_name|_source]`, `datum_lat/lon`) → `install/scripts/migrate_openmower.py` `write_areas_dat` (L286) writes the same format by hand; loader must stay tolerant of files without identity lines (`area_manager.cpp` L1534-1539).
- `MapArea.msg`, `MapObstacleInfo.msg`, `DigEvent.msg`, `ObstacleArray.msg`/`TrackedObstacle.msg`, `PromoteObstacle.srv`, `SetDockingPoint.srv`, `GetRecoveryPoint.srv` → regenerate `gui/pkg/msgs/mowgli/types_generated.go` (`cd gui && ./generate_go_msgs.sh`, `LC_ALL=C`) and `gui/web/src/types/ros.generated.ts` (`./generate_ts_types.sh`; `ros.ts` is only a one-line re-export barrel); drift gate `.github/workflows/msg-codegen-drift.yml`. `docs/claude/commands.md` has the full workflow.
- `MapObstacleInfo::SOURCE_*` values are mirrored in `ObstacleEntry::source` (`map_server_node.hpp` L247-261) and serialised as ints in `areas.dat` `_source:` lines — renumbering breaks existing files.
- `obstacle_margin` is injected into BOTH `map_server_node` (`full_system.launch.py` L405-406) and `coverage_server` (`navigation.launch.py`) — keep transit keepout band and F2C hole buffer in lockstep; GUI field `gui/web/src/components/settings/ObstaclesSection.tsx` L91-96.
- `tool_width` → CLAUDE.md Invariant 6 (`robot_config_util.DEFAULT_TOOL_WIDTH_M`); never hardcode a second default.
- `dock_pose_x/y/yaw` → every writer of `mowgli_robot.yaml` goes through `robot_yaml_scalar::UpdateDockPose` (Invariant 6): this node (`area_manager.cpp` L916 `on_set_docking_point`, L1673 datum migration) and `calibrate_imu_yaw_node.cpp` L723 — those are the only two files that call it. `mowgli_behavior/src/calibration_nodes.cpp` no longer persists the dock pose (`calibration_nodes.cpp` L42-48: it only publishes a `/set_pose` seed), so CLAUDE.md Invariant 6's "third writer" is stale. `hardware_bridge` reads them as parameters at startup (`hardware_bridge_node.cpp` L346-348).
- `datum_lat/lon` → must be the same `robot_params` read as `navsat_to_absolute_pose` (Invariant 4); sim injects its own pair (`sim_full_system.launch.py` L229-230).
- `/keepout_mask` semantics (0 free / 50 slack / 100 lethal) → `nav2_params_base.yaml` `keepout_filter` (L782-785, `base 0 / multiplier 1` set in `costmap_filters.cpp` L380-381) and plugin ORDER `keepout_filter` before `inflation_layer` in `nav2_params_lidar.yaml` L18 / `nav2_params_no_lidar.yaml` L33; `obstacle_tracker` treats ≥ `keepout_lethal_threshold` (100) as promoted.
- `~/boundary_violation` / `~/lethal_boundary_violation` / `~/replan_needed` → `BTContext` flags in `mowgli_behavior/include/mowgli_behavior/bt_context.hpp` L358-367 and `behavior_tree_node.cpp` L232-268.
- `mow_progress_tool_frame` (`blade_link`) → the link is defined in `ros2/src/mowgli_bringup/urdf/mowgli.urdf.xacro` (pinned by `mowgli_bringup/test/test_urdf_xacro.py`); renaming it there without updating the param inhibits stamping (warn-throttled, `map_server_node.cpp` L698-708).
- `Status.msg` blade fields (`mow_enabled`, `mower_esc_status`, `mower_motor_rpm`, `blade_status_stamp`, `is_charging`) → `on_mower_status` (`map_server_node.cpp` L614-622) and the `set_docking_point` charging gate.
- New `map_server_node` parameter with an operator-facing default → template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` + launch injection block (`full_system.launch.py` L366-437), NOT the installed sparse file (Invariant 15). Params passed by launch but not `declare_parameter`'d (e.g. `chassis_safety_inset`, L395-398) are silently ignored.

## Pitfalls
- **Dig keepouts are PENDING, not persisted** (`area_manager.cpp` L1186-1190; `save_areas_to_file` skips `pending` L1430-1433). They live in the mask + classification for this session only; `~/promote_obstacle{pending_id}` persists, `~/discard_obstacle` drops, restart forgets. CLAUDE.md Invariant 16's "persisted to `areas.dat`" wording is stale.
- Invariant 14 index mapping lives at `costmap_filters.cpp` L192-194 and L311-313/L327-329/L358-360 — four copies; change all or the mask rotates 90°. `build_keepout_mask_for_test` exists to pin it.
- Masks are (re)published only when `masks_dirty_` (`map_server_node.cpp` L854-858); `/costmap_filter_info` is sent ONCE per lifetime (`costmap_filters.cpp` L373-384) and the flag is reset by `~/clear_map` / `~/load_areas` only. Republishing every tick made Nav2 reload the filter and drop plans.
- `/map` is subscribed by both nodes but nothing in `ros2/src` publishes it (only `/no_lidar_static_map` from `mowgli_bringup/scripts/empty_static_map_pub.py`). The occupancy layer stays 0 and `obstacle_tracker::on_map` never runs; the tracker's live source is `/global_costmap/costmap` + `_updates`.
- `speed_mask_pub_`, `speed_filter_info_pub_`, `grid_map_pub_` are declared (`map_server_node.hpp` L819-827) but never created; `publish_speed_mask` is declared (L473) with **no definition** — calling it is a link error. No speed mask or `~/grid_map` topic exists.
- `keepout_nav_margin` is dead while `lethal_outside_areas=true` (default); the outside band is `enforce_boundary_margin_m` at cost 50 (`costmap_filters.cpp` L44, L97-98). Setting `lethal_outside_areas=false` only swaps the band WIDTH to `keepout_nav_margin`; the band cost stays 50 in both branches, never free (the "free band" wording in `map_server.yaml` L76/L93 and `costmap_filters.cpp` L96 is stale).
- Dock corridor carve-out forces cells to 0 AFTER obstacle/no-go passes (`costmap_filters.cpp` L340-364). The dock body is `OBSTACLE_PERMANENT` in the classification layer (`area_manager.cpp` L1761-1773) but the mask overlay only lifts `NO_GO_ZONE` (L318-332) — the body is not lethal in `/keepout_mask` by itself.
- `resize_map_to_areas` reallocates and wipes CLASSIFICATION (`area_manager.cpp` L243-253); always follow with `apply_area_classifications` (as `on_add_area` does, L534-539).
- `apply_promoted_obstacle` and `apply_area_classifications` take `map_mutex_` themselves (`progress_tracker.cpp` L278-283) — calling either with the lock held deadlocks. `on_obstacles` collects under lock then applies outside (`map_server_node.cpp` L779-790).
- Centroid dedup ε 0.10 m (`internal_helpers.hpp` L41) makes promotion / reload idempotent but also collapses two genuinely distinct keepouts closer than 10 cm.
- `set_docking_point` rejects unless charging + status < 3 s, `/gps/pose_cov` σ ≤ 4 cm and < 2 s old, ≥ 20 yaw samples with circular std ≤ 0.5°, and (GPS mode) ≥ 10 GPS samples (`area_manager.cpp` L652-863). `yaw_source` defaults to `PRESERVE` (0): a caller that omits it never changes the yaw.
- Dock pose is NOT in `areas.dat` (`area_manager.cpp` L1452-1455); stale `dock_x/dock_qw` keys in old files are ignored (L1560-1562). Migration re-projects it and splices `mowgli_robot.yaml` (L1663-1680).
- Datum migration runs inside `load_areas_from_file` BEFORE `resize_map_to_areas` (L1564-1575) and re-saves the file; a 0/0 launch datum disables it; `kDatumMatchEpsilonDeg` 1e-8° absorbs formatting round-trips.
- `mow_progress` is never written to disk (no save path touches `mow_progress_map_`; `on_save_map` writes occupancy+classification only, L305-315) and is reset on every `init_map`/resize. Resume state lives in the BT swath-completion model (`bt_context.hpp` L264), not here.
- Stamping needs `mow_enabled` AND blade telemetry ≤ 1 s old AND `mower_esc_status != 0` AND RPM ≥ 1000 AND a `map→blade_link` TF (`map_server_node.cpp` L672-709); any gap breaks the sweep (`have_last_mow_tool_position_ = false`) so a gap is never bridged.
- `boundary_violation` is debounced (3 samples) but `lethal_boundary_violation` is not (`boundary_classifier.hpp` L66-67); both topics publish every odom tick even when idle on the dock.
- `obstacle_tracker` filters clusters against area **index 0 only**, with a naive vertex-toward-centroid inset (`obstacle_tracker_node.cpp` L738-800); multi-area or concave sites are mis-filtered.
- Tracker promotion requires ~1 observation/s; delta-mode costmap updates are what keep the cadence (`obstacle_tracker_node.cpp` L104-115). Removing the `_updates` subscription silently stops promotion.
- BT `SaveObstacles` (`utility_nodes.cpp` L216) targets `/obstacle_tracker/save_obstacles`; the node serves `/obstacle_tracker/save` (`obstacle_tracker_node.cpp` L166). The call never completes.
- `test_obstacle_tracker` deliberately builds ONE node per suite (`test_obstacle_tracker.cpp` L33-50) — per-test node teardown deadlocked in CI. Keep new tests stateless or reset in `SetUp`.
- Do not add `area_names: []` etc. to `map_server.yaml` — ROS2 cannot type an empty YAML list and lifecycle bring-up throws (`map_server.yaml` L171-176); the sim injects its polygon via launch override. See CLAUDE.md "What NOT to Do".
- `map_server_small_garden.yaml` / `map_server_obstacle_test.yaml` are not loaded by any launch file; edit `map_server.yaml` for real defaults.

## Generated & vendored — do not hand-edit
- Nothing generated inside `ros2/src/mowgli_map`. Downstream generated artefacts of its interfaces: `gui/pkg/msgs/mowgli/types_generated.go`, `gui/web/src/types/ros.generated.ts` (regen scripts in `gui/`).
- `grid_map_*`, `nav2_msgs`, `map_msgs`, Boost come from the ROS Kilted image; `/opt/fields2cover-300` is not used here.
