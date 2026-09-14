# Codemap: fusion_graph

> `ros2/src/fusion_graph` is the GTSAM iSAM2 Pose2 factor-graph localizer — the sole owner of BOTH `map→odom` and `odom→base_footprint` (CLAUDE.md Invariants 1–2). It fuses `/wheel_odom` + `/imu/data` (between-factors), `/gps/fix` (`GnssLeverArmFactor`), `/imu/cog_heading` / `/imu/mag_yaw` (yaw unaries) and, with LiDAR, a persistent RTK-built map anchor used only after complete GNSS outages; it persists the graph and tiled map under `/ros2_ws/maps/fusion_graph*`. Scan-to-scan ICP and loop closure were removed. Index refreshed 2026-09-09; regenerate when files are added/removed. Loaded on demand from `ros2/CLAUDE.md`.

## Where to look
| Task | Start here |
|------|------------|
| Add / rename a ROS parameter | Graph-side (`GraphParams`) → `ros2/src/fusion_graph/src/fusion_graph_node.cpp:30-91`; node-side → `ros2/src/fusion_graph/src/fusion_graph_node_setup_params.cpp`; yaml default → `ros2/src/fusion_graph/config/fusion_graph.yaml`; struct default → `ros2/src/fusion_graph/include/fusion_graph/graph_params.hpp` |
| Add a publisher / subscription / service / timer | `ros2/src/fusion_graph/src/fusion_graph_node_setup_comms.cpp` (`SetupCommunications`) |
| Change what a node's factors look like (wheel/gyro/GNSS/yaw/LiDAR-anchor) | `ros2/src/fusion_graph/src/graph_manager_node.cpp` `CreateNodeLocked` (:76); factor classes in `ros2/src/fusion_graph/include/fusion_graph/factors.hpp` + `src/factors.cpp` |
| `GnssLeverArmFactor` residual / Jacobian | `ros2/src/fusion_graph/src/factors.cpp` (`evaluateError`), pinned by `test/test_factors.cpp` `JacobianMatchesNumeric` |
| GPS wrong-fix gate / unknown-covariance reject / max-σ reject | `ros2/src/fusion_graph/src/fusion_graph_node_callbacks_a.cpp` `OnGnss` (:108; rejects logged at :159, :200, :232); pure rule `include/fusion_graph/rtk_wrongfix_gate.hpp` |
| GNSS timestamp → node association (delayed fixes) | `OnGnss` (`FindNodeAtOrBefore`, warns at :393/:403/:413) + `ros2/src/fusion_graph/src/graph_manager.cpp:257`; `test/test_gnss_timestamp.cpp` |
| Dock hold while charging, dock-prior-vs-GPS yield (#512) | `OnGnss` block ending in `ForceAnchor` (ERROR at :346); pure rule `include/fusion_graph/dock_gps_consistency.hpp` |
| Dock-arrival seed / gauge reset / boot fallback | `ros2/src/fusion_graph/src/fusion_graph_node_misc.cpp` `SeedFromDockPose` (:25); `OnHardwareStatus` `callbacks_b.cpp:390`; boot fallback `fusion_graph_node_timer.cpp:31-58` |
| Slip veto (graph side, windowed, #516) | `graph_manager_node.cpp:222-231` + `include/fusion_graph/slip_window.hpp`; tests `test_slip_window.cpp`, `test_slip_pivot.cpp` |
| Slip veto (dead-reckoning side, #488) | `fusion_graph_node_callbacks_a.cpp` `OnImu` (:59) + `include/fusion_graph/dr_slip_veto.hpp` (`kDrSlipWheelMinDefaultRadPerS` 0.44) |
| Wheel σ random-walk model (#491), pivot release, adaptive inflation | `graph_manager_node.cpp:258-325`; `test_wheel_sigma_scaling.cpp`, `test_adaptive_noise.cpp` |
| Stationary yaw snap, hand-push gate, gyro-bias EMA | `graph_manager_node.cpp:125-190`; `graph_manager.cpp` `AddGyroDelta` (:109); `test_stationary_yaw.cpp`, `test_gyro_bias.cpp` |
| COG yaw gating, σ floor, 180° flip recovery | `fusion_graph_node_callbacks_b.cpp` `OnCogHeading` (:44); pure rules `include/fusion_graph/yaw_gates.hpp`, `include/fusion_graph/cog_flip_recovery.hpp` |
| Graph persistence (`.graph/.meta`), datum cross-garden guard, autoload, empty-graph refusal | `src/graph_manager_persistence.cpp` `Save` / `Load`; `test_persistence.cpp` |
| Save / clear services, auto-checkpoints | `fusion_graph_node_setup_comms.cpp:205-270`; `DispatchAsyncSave` `callbacks_b.cpp:363`; `OnHighLevelStatus` :276; `OnPeriodicSaveTimer` `misc.cpp:126` |
| map→odom anchor, slew limiter (unwired), odom re-base, TF thread | `src/fusion_graph_node_publish.cpp` (`PublishOutputs` :153, `SlewPublishedAnchor` :278, `TfBroadcastLoop` :320); re-base `fusion_graph_node_timer.cpp:483-500`; pure `include/fusion_graph/anchor_slew.hpp` |
| Published covariance frame (body→map) | `fusion_graph_node_publish.cpp:183` + `include/fusion_graph/covariance_frame.hpp`; `test_covariance_frame.cpp` |
| Sliding-window cap / iSAM2 rebase | `src/graph_manager_rebase.cpp` `RebaseISAM2`; maintenance timer `setup_comms.cpp`; `test_graph_window.cpp` |
| LiDAR map anchor (scan-to-MAP localisation for complete GNSS outages): grid, gate, particle filter, per-estimate trust, shadow mode | `src/fusion_graph_node_lidar_anchor.cpp` (RTK-only map learning, any-usable-GNSS outage age, warm-up, verdicts, `/fusion_graph/lidar_anchor_candidate`); pure pieces `include/fusion_graph/lidar_occupancy_mapper.hpp` (log-odds grid, `ScoreScan`, `ImportCells`), `lidar_map_anchor_gate.hpp` (RTK-age mapping hysteresis), `lidar_compute_gate.hpp` (outage warm-up / compute cadence), `lidar_anchor_validator.hpp` (score / spread / dead-reckoning gates); params `setup_params.cpp` (`use_lidar_map_anchor`, `lidar_anchor_*`, `lidar_map_*`); graph consumer `graph_manager_node.cpp` (`PoseTranslationPrior`, XY only) |
| Replaying a bag through the node on a laptop (reproduce a field failure) | `ros2/src/fusion_graph/tools/replay/README.md` |
| LiDAR anchor map persistence / reset | persisted as tiles under `<graph_save_prefix>.lidartiles`; `src/lidar_submap_store.cpp` + `include/fusion_graph/lidar_submap_store.hpp`; legacy `.lidarmap` import remains only for migration; `~/clear_lidar_map` wipes the tiles, local window and filter. Continuous filter odometry: `lidar_anchor_odom.hpp` |
| Diagnostics keys (`/fusion_graph/diagnostics`) / marker viz | `fusion_graph_node_setup_comms.cpp:328-512`; counters in `include/fusion_graph/graph_manager.hpp` `GraphStats` (:62) |
| `ros2/src/fusion_graph/launch/fusion_graph.launch.py` | | Reads datum, antenna lever arm and dock pose; declares map-anchor, magnetometer, cadence and primary-mode arguments |
| Perf regression | `test/test_perf.cpp` (Release only, `TIMEOUT 300` in `CMakeLists.txt`) |

## Files
| File | Lines | Purpose |
|------|-------|---------|
| **`ros2/src/fusion_graph/` (package root)** | | |
| `ros2/src/fusion_graph/CMakeLists.txt` | | `fusion_graph_core` library + `fusion_graph_node`; package unit-test targets |
| `ros2/src/fusion_graph/package.xml` | 46 | ament_cmake deps (rclcpp, nav/geometry/sensor/std/diagnostic/visualization msgs, std_srvs, tf2*, mowgli_interfaces, eigen); GTSAM is NOT a rosdep |
| `ros2/src/fusion_graph/config/fusion_graph.yaml` | | Deployed defaults for graph, GNSS, docking, persistence and LiDAR map anchor |
| `ros2/src/fusion_graph/launch/fusion_graph.launch.py` | | Reads datum, antenna lever arm and dock pose; declares map-anchor, magnetometer, cadence and primary-mode arguments |
| **`ros2/src/fusion_graph/include/fusion_graph/`** | | |
| `.../fusion_graph_node.hpp` | 744 | `FusionGraphNode` class: every member + the design rationale comments (DR, anchor, slew, gates, LiDAR map anchor) |
| `.../fusion_graph_node_util.hpp` | 60 | `YawFromQuat`, `QuatFromYaw`, `ForwardStampPose`, `kEarthRadius` |
| `.../graph_manager.hpp` | 582 | `GraphManager` API, `TickOutput`, `GraphStats`, accumulator/queue structs |
| `.../graph_params.hpp` | 373 | `GraphParams` aggregate + `kTunedNodePeriodS` (0.04) / `kMinWheelSigmaM` |
| `.../factors.hpp` | 160 | `GnssLeverArmFactor`, `GyroPreintFactor`, `YawUnaryFactor`, `WrapAngle` |
| `.../slip_window.hpp` | 125 | Pure windowed slip veto: `SlipWindowNodes`, `SlipDetectedOverWindow`, `SlipWindow` |
| `.../dr_slip_veto.hpp` | 79 | Pure DR slip veto + wheel yaw-rate quantum; shipped thresholds 0.44 / 0.15 rad/s |
| `.../rtk_wrongfix_gate.hpp` | 57 | Pure `GpsJumpImplausible` + `ResetRtkWrongFixAccumulators` (reset on EVERY fix) |
| `.../dock_gps_consistency.hpp` | 84 | Pure dock-prior-vs-RTK-Fixed yield rule (`DockPriorShouldYield`) |
| `.../yaw_gates.hpp` | 68 | Pure `CogShouldApply`, `CogEffectiveSigma`, `ScanYawSigma` |
| `.../cog_flip_recovery.hpp` | 86 | Pure 180° flip decision `CogFlipRecoveryFeed` |
| `.../anchor_slew.hpp` | 113 | Pure map→odom anchor slew limiter `AnchorSlewStep` |
| `.../covariance_frame.hpp` | 89 | Pure `BodyToMapCovariance`, `MaxPositionSigma` |
| `.../pose_extrapolator.hpp` | 112 | Header-only `PoseExtrapolator` (gyro-forward yaw for `/odometry/filtered_map_fast`) |
| **`ros2/src/fusion_graph/src/`** | | |
| `.../fusion_graph_node.cpp` | 209 | Ctor: declares `GraphParams` + node-side params, builds `GraphManager`; `main()` |
| `.../fusion_graph_node_setup_params.cpp` | | Node-side LiDAR-map-anchor, COG, persistence, dock and launch parameters |
| `.../fusion_graph_node_setup_comms.cpp` | 538 | Publishers, subscriptions, `~/save_graph`, `~/clear_graph`, tick/maintenance/diag timers, TF thread start |
| `.../fusion_graph_node_callbacks_a.cpp` | 501 | `OnWheelOdom`, `OnImu` (DR integration + DR slip veto), `OnGnss` (all GPS gates, dock hold, stamp association, autoload override) |
| `.../fusion_graph_node_callbacks_b.cpp` | | Dock command, COG, magnetometer, scan, high-level status, set-pose, save and hardware-status callbacks |
| `.../fusion_graph_node_timer.cpp` | | Main tick: boot dock fallback, LiDAR map-anchor step, graph tick, odom rebase and publish |
| `.../fusion_graph_node_publish.cpp` | | Initial pose, local odometry, map output, map→odom anchor and TF thread |
| `.../fusion_graph_node_misc.cpp` | 140 | `SeedFromDockPose` (rigid gauge reset > 5 cm), `OnPeriodicSaveTimer` |
| `.../graph_manager.cpp` | | Accumulators, queues, initialization, statistics and pose snapshots |
| `.../graph_manager_node.cpp` | 574 | `Tick` / `CreateNodeLocked`: all per-node gates + factor construction + iSAM2 update + throttled marginal |
| `.../graph_manager_rebase.cpp` | | Windowed iSAM2 rebase, robust update/reset, rigid whole-graph transforms |
| `.../graph_manager_persistence.cpp` | | Graph reset and `.graph`/`.meta` persistence with datum guard |
| `.../factors.cpp` | 162 | Factor `evaluateError` + analytic Jacobians + `clone` |
| **`ros2/src/fusion_graph/test/`** | | |
| `.../test_factors.cpp` | | Graph factor residual and Jacobian tests |
| `.../test_gnss_timestamp.cpp` | 142 | Delayed GNSS constrains the node at `header.stamp`, bounded by the window |
| `.../test_rtk_wrongfix_gate.cpp` | 167 | Wrong-fix budget math; reject-forever regression if reset only on accept |
| `.../test_dock_gps_consistency.cpp` | 127 | #512 field numbers; Float / no-fix never yields |
| `.../test_dr_slip_veto.cpp` | 106 | Threshold clears 2 encoder-tick quantum (#488) |
| `.../test_slip_window.cpp` | 159 | Windowed veto ignores jitter, catches sustained slip, cold window never vetoes |
| `.../test_slip_pivot.cpp` | 296 | End-to-end GraphManager slip scenarios incl. cadence scaling |
| `.../test_wheel_sigma_scaling.cpp` | 289 | σ random-walk: cadence-invariant, √d growth, creep floor, pivot release |
| `.../test_adaptive_noise.cpp` | 240 | σ_x inflation on wheel-vs-gyro residual EMA |
| `.../test_stationary_yaw.cpp` | 157 | Gyro bias does not drift map yaw when wheels are still |
| `.../test_gyro_bias.cpp` | 198 | EMA bias converges when stationary, frozen when moving |
| `.../test_graph_window.cpp` | | `max_graph_nodes` cap and estimate preservation across rebase |
| `.../test_rigid_transform.cpp` | 60 | `RigidTransformAll` shifts the live trajectory by exactly the correction |
| `.../test_persistence.cpp` | 175 | Empty-graph Save/Load refused; non-empty round trip; datum cross-garden guard |
| `.../test_map_odom_slew.cpp` | 147 | Anchor slew: rate limit, snap thresholds, shortest-path yaw |
| `.../test_yaw_gates.cpp` | | COG apply and covariance gates |
| `.../test_cog_flip_recovery.cpp` | 164 | N-consecutive consistent flips, rate limit, wrap |
| `.../test_covariance_frame.cpp` | 121 | Body→map rotation, `MaxPositionSigma` frame-invariance |
| `.../test_pose_extrapolator.cpp` | 105 | Yaw extrapolation, 200 ms cap, re-baseline |
| `.../test_perf.cpp` | | Graph tick throughput and representative mowing session |

## Runtime surface

### Nodes
| Node | Executable | Launched by | Type |
|------|------------|-------------|------|
| `ros2/src/fusion_graph/launch/fusion_graph.launch.py` | | Reads datum, antenna lever arm and dock pose; declares map-anchor, magnetometer, cadence and primary-mode arguments |

`primary_mode=false` (observer) exists in the launch file (remaps `/odometry/filtered_map`→`/fusion_graph/odometry`, no TF) but `navigation.launch.py` never selects it.

### Topics
| Topic | Type | Dir | QoS | Other end |
|-------|------|-----|-----|-----------|
| `/wheel_odom` | `nav_msgs/Odometry` | sub | depth 50 | `hardware_bridge` `~/wheel_odom` remapped in `ros2/src/mowgli_bringup/launch/mowgli.launch.py:260` |
| `/imu/data` | `sensor_msgs/Imu` | sub | SensorData | `hardware_bridge_node` (~91 Hz); only `angular_velocity.z` used |
| `/gps/fix` | `sensor_msgs/NavSatFix` | sub | SensorData | GPS container (`sensors/gps/start_gps.sh:525`); status ≥ `STATUS_FIX` required, `STATUS_GBAS_FIX` = RTK-Fixed |
| `/imu/cog_heading` | `sensor_msgs/Imu` | sub | SensorData | `ros2/src/mowgli_localization/src/cog_to_imu_node.cpp:252` |
| `/imu/mag_yaw` | `sensor_msgs/Imu` | sub (if `use_magnetometer`) | SensorData | `mowgli_localization/src/mag_yaw_publisher_node.cpp:61` |
| `scan_topic` (default `/scan_deskewed`) | `sensor_msgs/LaserScan` | sub (if `use_lidar_map_anchor`) | SensorData | `mowgli_localization/src/scan_deskew_node.cpp` |
| `/hardware_bridge/status` | `mowgli_interfaces/Status` | sub | depth 10 | `hardware_bridge_node.cpp:704`; `is_charging` drives dock seed + auto-save |
| `/cmd_vel_docking` | `geometry_msgs/TwistStamped` | sub | SensorData | docking_server remap `nav2_navigation_launch.py:229`, also `calibrate_imu_yaw_node.cpp:250` |
| `/behavior_tree_node/high_level_status` | `mowgli_interfaces/HighLevelStatus` | sub (if `auto_save_enabled`) | depth 10 | `behavior_tree_node` |
| `~/set_pose` (`/fusion_graph_node/set_pose`) | `geometry_msgs/PoseWithCovarianceStamped` | sub | reliable, transient_local, depth 1 | `mowgli_behavior/src/calibration_nodes.cpp:203,316` |
| `/odometry/filtered_map` | `nav_msgs/Odometry` (map→base_footprint) | pub | depth 10 | `map_server_node.cpp:231`, `behavior_tree_node.cpp:331`, `hardware_bridge_node.cpp:799` (dig detector), GUI `useFusionOdom.ts` |
| `/odometry/filtered` | `nav_msgs/Odometry` (odom→base_footprint, DR) | pub | depth 10 | Nav2 `bt_navigator` (`nav2_params_base.yaml:24`) |
| `/imu/fg_yaw` | `sensor_msgs/Imu` (yaw only) | pub | SensorData | no in-repo consumer today |
| `/fusion_graph/diagnostics` | `diagnostic_msgs/DiagnosticArray` 1 Hz | pub | depth 10 | GUI `gui/pkg/providers/ros.go:55`, `useFusionGraphDiagnostics.ts`, session monitor |
| `/fusion_graph/lidar_map` | `nav_msgs/OccupancyGrid` (800×800 @ 0.10 m, 0 free / 100 occupied / −1 unknown) | pub (only with `use_lidar_map_anchor`), every `lidar_map_rebuild_period_s` | QoS(1) transient_local | GUI map page (`lidarMap` stream), Foxglove |
| `/fusion_graph/lidar_anchor_candidate` | `geometry_msgs/PoseWithCovarianceStamped` | pub, one per particle-filter estimate (anchoring or shadow) | depth 10 | tooling: `covariance[35]` = verdict (0 accepted / 1 score / 2 spread / 3 dead reckoning), `[14]` = 1 when it became a factor |
| `lidar_map_import_topic` (default "", off) | `nav_msgs/OccupancyGrid` | sub, FIRST message only | transient_local | replay harness / persisted-map seed |
| `/fusion_graph/markers` | `visualization_msgs/MarkerArray` 1 Hz | pub | depth 1 transient_local | Foxglove (ids 0 nodes / 1 trajectory, ≤1500 nodes) |
| `/odometry/filtered_map_fast` | `nav_msgs/Odometry` | pub (if `fast_pose_publish_rate_hz` > 0) | SensorData | none by default |

Diagnostics keys (`setup_comms.cpp`): graph/dock state, scan and LiDAR map/filter counters, map-anchor verdict/rejection counters, GNSS wrong-fix rejects, stationary/slip telemetry, gyro bias, adaptive wheel noise, and `cov_xx` / `cov_yy` / `cov_yawyaw`.

### Services & actions
| Service | Type | Where | Caller |
|---------|------|-------|--------|
| `~/save_graph` (`/fusion_graph_node/save_graph`) | `std_srvs/Trigger` | `setup_comms.cpp:205` — async `DispatchAsyncSave("manual-service")` | GUI `gui/pkg/api/mowglinext.go:668` (`fusion_graph_save`) |
| `~/clear_graph` (`/fusion_graph_node/clear_graph`) | `std_srvs/Trigger` | `setup_comms.cpp` — exact IDLE + stationary gate; `Reset()`, persisted `.graph/.meta` deletion and DR zeroing; immediate dock-pose re-seed when charging, otherwise fresh-GPS re-init with retained heading | GUI `mowglinext.go:670` (`fusion_graph_clear`) |

No actions. Auto-save triggers: RECORDING exit (`callbacks_b.cpp:276`), `is_charging` rising edge / boot-docked (`callbacks_b.cpp:404`), `periodic_save_period_s` (300 s) while `HIGH_LEVEL_STATE_AUTONOMOUS` (`misc.cpp:126`).

### Parameters
All read ONCE at construction (`declare_parameter`, no dynamic reconfigure). Precedence: `GraphParams` struct default < `fusion_graph_node.cpp` / `setup_params.cpp` declare default < `config/fusion_graph.yaml` < launch dict in `fusion_graph.launch.py:140-160` (datum, lever arm, dock pose, `use_*`, `tf_publish_lead_s`, `node_period_s`); primary/observer `Node` actions at `:166-184`.

| Parameter | Declared | Yaml default | Notes |
|-----------|----------|--------------|-------|
| `node_period_s` | `fusion_graph_node.cpp:31` (0.1) | `fusion_graph.yaml:23` 0.02 | Overridden by `navigation.launch.py` from `mowgli_robot.yaml` `fusion_graph_node_period_s` (fallback 0.04); per-tick gates scale by `node_period_s/kTunedNodePeriodS` |
| `wheel_sigma_x_per_sqrt_m` / `_y_` / `wheel_creep_speed_mps` | `:33-35` | `:52-57` 0.05 / 0.005 / 0.04 | σ = k·√(step + creep·dt) (#491) |
| `gyro_sigma_theta`, `stationary_sigma_theta` | `:37`, `:61` | `:64` 0.005, `:97` 0.01 | stationary snap σ relaxed to 0.01 for iSAM2 conditioning |
| `slip_residual_thresh_rad` / `slip_gyro_max_rad` / `slip_wheel_min_rad` / `slip_window_s` | `:69-73` | `:138-160` 0.01 / 0.005 / 0.005 / 0.5 | windowed veto (#516); 0 = per node |
| `max_graph_nodes`, `isam2_rebase_every_nodes` | `fusion_graph_node.cpp`, `setup_params.cpp` | `6000`, code default `2000` | Bound live graph size and schedule asynchronous rebases |
| `rtk_wrongfix_max_jump_m` | `fusion_graph_node.cpp:94` | `:284` 0.05 | slack on top of wheel travel + lever sweep |
| `gps_sigma_floor`, `gps_sigma_speed_coeff`, `gps_max_sigma_reject_m` | `:38`, `:98`, `:104` | `:312` 0.003, `:294` 0.0, (0.0) | |
| `dock_reanchor_sigma_xy_m`, `dock_prior_max_gps_disagreement_m`, `dock_prior_max_gps_sigma_m` | `:109-124` | `:399-414` 0.03 / 0.50 / 0.05 | #512 |
| `dr_slip_gyro_max_rad_per_s`, `dr_slip_wheel_min_rad_per_s` | `:129-132` | not in yaml (0.15 / 0.44 from `dr_slip_veto.hpp:49-50`) | #488 |
| `use_magnetometer`, `primary_mode` | `setup_params.cpp` | via launch | Optional magnetometer; TF ownership mode |
| `use_lidar_map_anchor`, `lidar_anchor_shadow_mode` | `setup_params.cpp` | via launch, template defaults `true` / `false`, LiDAR-gated | Persistent scan-to-map fallback; fresh usable GNSS keeps the filter asleep; shadow runs bounded calibration under Fixed without factors |
| `lidar_map_*` (resolution, 10 m tile size, local radius, insertion/rebuild cadence), anchor timing/particle/filter knobs | `setup_params.cpp`, `fusion_graph.yaml` | Persistent tiled map; only the local window is resident. RTK Fixed controls learning; any usable GNSS controls outage timing |
| `lidar_anchor_min_hit_ratio` 0.5, `_min_hit_count` 30, `_max_sigma_m` 0.5, `_dr_budget_m` 0.3, `_dr_drift_frac` 0.02, `_reseed_after_s` 5.0 | `config/fusion_graph.yaml` → `LidarAnchorValidatorParams` | yaml only | Per-estimate trust; sized on the 2026-09-06 outage (lost filter scored 0.13, DR drifted < 0.4 m in 2.7 min) |
| `cog_require_rtk`, `cog_min_speed_mps`, `cog_min_sigma_rad`, `cog_flip_*` | `setup_params.cpp:108-121` | not in yaml | |
| `tf_publish_lead_s`, `tf_broadcast_rate_hz`, `anchor_*`, `odom_rebase_dist_m` | `fusion_graph_node.cpp:161-182` | `:358` 0.05, `:372` 20, (defaults), `:38` 6.0 | launch passes `fusion_graph_tf_lead_s` (hardware 0.05, sim 0.1) |
| `graph_save_prefix`, `autoload_graph`, `auto_save_enabled`, `periodic_save_period_s`, `rtk_autoload_override_threshold_m` | `setup_params.cpp:172-212`, `setup_comms.cpp:27` | not in yaml (`/ros2_ws/maps/fusion_graph`, true, true, 300, 0.3) | |
| `datum_lat/lon`, `lever_arm_x/y`, `dock_pose_x/y/yaw`, `dock_pose_yaw_sigma_rad` | `:140-141`, `:41-42`, `setup_params.cpp:35-38` | placeholders `:322-327` (`dock_pose_*` absent) | real values injected by launch from `mowgli_robot.yaml` (`gps_x`/`gps_y` template `mowgli_bringup/config/mowgli_robot.yaml:217-218`) |

### TF frames
`map_frame`=`map`, `odom_frame`=`odom`, `base_frame`=`base_footprint` (`fusion_graph_node.cpp:158-160`). `map→odom` = the constant per-node anchor `t_map_odom_anchor_` (`publish.cpp:266-275` inline, or `TfBroadcastLoop` at `tf_broadcast_rate_hz`) — `SlewPublishedAnchor` (:278) has NO call site, so `anchor_slew_*` / `t_map_odom_pub_` are dead config today; `odom→base_footprint` = wheel-vx + gyro dead reckoning (`publish.cpp:70-93`). Both carry a stamp advanced by `tf_publish_lead_s`, but only the odom→base pose is propagated by `ForwardStampPose` (the anchor is time-invariant). Only broadcast when `primary_mode`. `ros2/src/mowgli_bringup/test/test_tf_ownership.py:53-54` pins these two files as the only fusion_graph TF writers (the pinned set also holds `wheel_odometry_node`, whose `publish_tf` defaults false).

## Build, test, run
- Build (devcontainer, `/ros2_ws`): `cd ros2 && make build-pkg PKG=fusion_graph` (`ros2/Makefile:64` → `scripts/build.sh`); raw: `colcon build --packages-select fusion_graph --cmake-args -DCMAKE_BUILD_TYPE=Release`.
- Unit tests: `PACKAGES="fusion_graph" ./scripts/test.sh` (`ros2/scripts/test.sh`) or `colcon test --packages-select fusion_graph && colcon test-result --verbose`. Single binary: `./build/fusion_graph/test_factors`. All 22 targets are gtest, no ROS graph needed (pure headers + `GraphManager`). `test_perf` is Release-only meaningful, 300 s timeout.
- GTSAM 4.3a1 is built from source (`ros2/Dockerfile:2-23` stage `gtsam-builder`, `.devcontainer/Dockerfile:26-30`, CI `.github/workflows/ros2-ci.yml:186-226` cached under `/opt/gtsam`). `find_package(GTSAM CONFIG REQUIRED)` needs `CMAKE_PREFIX_PATH=/opt/gtsam`.
- CI: `.github/workflows/ros2-ci.yml` job `build-and-test` (:128) runs whole-workspace `colcon build` (:338) + `colcon test` (:347); `format-check` (:404, clang-format-18 on changed lines); `static-analysis` (:448). No launch_testing / e2e test targets fusion_graph specifically; `ros2/src/e2e_test.py:187` and `e2e_test_no_lidar.py:111` consume `/odometry/filtered_map`.
- Run standalone: `ros2 launch fusion_graph fusion_graph.launch.py use_lidar_map_anchor:=true`; manual save: `ros2 service call /fusion_graph_node/save_graph std_srvs/srv/Trigger`.

## Change coupling — "if you change X, also update Y"
- Rename/add a parameter → `config/fusion_graph.yaml` AND the declare site (`fusion_graph_node.cpp` for `GraphParams`, `setup_params.cpp` otherwise); if it is a per-tick radian/metre threshold, apply `tick_scale` in `graph_manager_node.cpp:98` or it silently changes meaning with cadence.
- `use_lidar_map_anchor` / `lidar_anchor_shadow_mode` / `use_magnetometer` / `fusion_graph_node_period_s` → template + `navigation.launch.py` + `fusion_graph.launch.py`; LiDAR options must remain ANDed with `use_lidar`.
- `gps_x`/`gps_y`/`datum_lat`/`datum_lon`/`dock_pose_*` → `fusion_graph.launch.py:43-84` `_read_robot_config` deep-merge (local copy of `robot_config_util.deep_merge`; keep in sync, no package dep allowed). `cog_to_imu` and `navsat_to_absolute_pose` read the same keys via `navigation.launch.py` — lever arm must agree (Invariant 4/6).
- Diagnostics key added/removed in `setup_comms.cpp:349-412` → GUI `gui/web/src/hooks/useFusionGraphDiagnostics.ts` + `DiagnosticsPage.tsx:827`, i18n `gui/web/src/i18n/locales/{en,fr}.json`, and the session-monitor JSONL (`docs/claude/session-monitoring.md`).
- Service name change → `gui/pkg/api/mowglinext.go:668-670`.
- Graph persistence format → `graph_manager_persistence.cpp` + `test_persistence.cpp`; LiDAR tile persistence → `lidar_submap_store.cpp` + `test_lidar_submap_store.cpp`.
- `~/set_pose` QoS (`setup_comms.cpp:193-199`) must match `mowgli_behavior/src/calibration_nodes.cpp:203,316` (reliable + transient_local).
- `/odometry/filtered` twist fields (`publish.cpp:105-113`) feed Nav2 `odom_topic` (`nav2_params_base.yaml:24`); controller_server uses `/wheel_odom` (`:77`, `:907`) — see CLAUDE.md "What NOT to Do".
- `Status.is_charging` / `HighLevelStatus.state` constants (`mowgli_interfaces`) drive dock seed and auto-save — regenerate bindings per `docs/claude/commands.md` if the msgs change.

## Pitfalls

- **`beluga_ros::Amcl`'s first update applies the WHOLE odom pose, not a delta** — its motion model reads a 2-pose odometry window with no first-update special case, so the first estimate after construction lands metres away (3.2 m in the 2026-09-07 replay). `RebuildLidarAnchorMap` does one discarded warm-up update right after construction; do not remove it. Re-seeds are unaffected (window already full).
- **Never feed a LiDAR-anchor estimate to the graph unvalidated.** The 2026-09-06 outage put the fused pose 10 m off with the filter's own covariance saying 5 cm. Every candidate goes through `ValidateLidarAnchor` (scan consistency AT THE ESTIMATE, particle spread, plausibility vs dead reckoning); dead reckoning is the better witness beyond its drift budget, and the cloud is re-seeded from it after `lidar_anchor_reseed_after_s` of rejects.
- The anchor map only grows while MAPPING (fresh RTK-Fixed): it is frozen during ANCHORING by design, so a robot that drives out of the mapped area during an outage is on dead reckoning until it comes back — the score gate makes that explicit (`lidar_anchor_rej_score` climbs, no factors).
- Three layers of defaults disagree: `GraphParams` struct (`graph_params.hpp`, e.g. `max_graph_nodes` 3000, `stationary_sigma_theta` 1e-3) vs declare defaults vs `fusion_graph.yaml` (6000, 0.01) vs launch (`node_period_s` 0.1 → yaml 0.02 → deployed 0.04). Unit tests construct `GraphParams` directly, so they run on struct defaults, not production values.
- The LiDAR map anchor factor is `PoseTranslationPrior<Pose2>` (XY only, `graph_manager_node.cpp` `lidar_map_xy` consumer). No LiDAR-derived yaw ever enters the graph as an absolute factor: a LiDAR yaw prior once flipped map→odom ~180° (2026-07-22, the since-removed scan-to-keyframe anchor). Do not add one back.
- `primary_mode=false` comments in `setup_params.cpp:134-137` and the arg description in `fusion_graph.launch.py:102-105` describe an `ekf_map_node` fallback that no longer exists; `navigation.launch.py:1088` always passes `"true"`.
- RTK wrong-fix gate accumulators reset on EVERY observation attempt (`callbacks_a.cpp` `OnGnss`, `rtk_wrongfix_gate.hpp`); this keeps the motion budget bounded per interval.
- A NavSatFix with `COVARIANCE_TYPE_UNKNOWN` or σ ≤ 0 is rejected outright (`callbacks_a.cpp:196-210`) — the old σ=-1 sentinel was clamped UP to 3 mm and fused garbage at RTK precision.
- While `is_charging` the node suppresses live GPS factors and re-asserts the dock prior once per node, and `seed_xy_` is NOT updated from the docked fix — the dock bootstrap is `SeedFromDockPose` (`misc.cpp:25`), not `TrySeedInitialPose`, which needs a GPS `seed_xy_` + COG/mag `seed_yaw_` (`publish.cpp:27-33`). A yielded node (#512) still marks `last_dock_reanchor_node_` (`callbacks_a.cpp:343-364`).
- `Save()` refuses an empty graph (`graph_manager_persistence.cpp:159`) — a Reset followed by auto-save used to persist `next_index=0` and crash the next boot (`test_persistence.cpp`).
- `Load()` rejects a map whose `.meta` datum differs from the configured one (cross-garden guard, `graph_manager_persistence.cpp:300-309`); (0,0) datum skips the check (`test_persistence.cpp` `LoadRefusesGraphSavedUnderAnotherDatum`).
- `clear_graph` rejects unless the high-level state is exactly `IDLE`/`IDLE_DOCKED`, wheel speeds are within 0.02 m/s and 0.05 rad/s, and no save/rebase is active. It deletes the persisted graph, zeroes `dr_*`, and invalidates the anchor; while charging it immediately seeds the calibrated dock pose, otherwise it retains the last heading and waits for a fresh GPS position.
- Per-tick gates (`stationary_thresh_*`, `pivot_gate_dtheta_rad`, `slip_*`) are tuned at 25 Hz and multiplied by `tick_scale` (`graph_manager_node.cpp:98`); raising the per-frame slip thresholds instead of using `slip_window_s` blinds the veto (`fusion_graph.yaml:141-160`).
- `dr_slip_wheel_min_rad_per_s` must exceed 2× the wheel yaw-rate quantum (`dr_slip_veto.hpp:10-32`, #488) — a lower value zeroes DR translation on straight drives and breaks Nav2 BackUp distances (Invariant 10).
- Heavy work off the executor: `RebaseISAM2` (maintenance timer, detached thread, `setup_comms.cpp:292-325`) and `Save` (`DispatchAsyncSave`) — never call them inline; TF has its own thread for the same reason (`fusion_graph_node.hpp:401-424`).
- `tf_publish_lead_s` is a genuine forward prediction (`ForwardStampPose`); sim needs 0.1, hardware 0.05 (`navigation.launch.py:214-217`). `fusion_graph.launch.py` arg default is 0.0 — only relevant when launched standalone.
- `/fusion_graph/markers` is transient_local and decimated to 1500 nodes; `GetAllPoses` is O(N) — keep it on the 1 Hz diag timer.
- Scan subscription defaults to `/scan_deskewed`, not `/scan` (`setup_comms.cpp:141`); with `use_lidar=false` both LiDAR flags are ANDed off upstream so the node never subscribes to a dead topic.
- Invariant 16: the graph-side slip veto is rotational only; do not extend it to catch straight-line digs — that is `hardware_bridge`'s dig detector's job.

## Generated & vendored — do not hand-edit
- GTSAM 4.3a1 lives outside the repo (`/opt/gtsam`, built by `ros2/Dockerfile` stage 0 / CI cache) — do not vendor it into `ros2/src`.
- `ros2/build/`, `ros2/install/`, `ros2/log/` are colcon outputs; `/ros2_ws/maps/fusion_graph.{graph,scans,meta}` are runtime artefacts written by `GraphManager::Save`.
