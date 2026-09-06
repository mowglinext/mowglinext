# Codemap: fusion_graph

> `ros2/src/fusion_graph` is the GTSAM iSAM2 Pose2 factor-graph localizer — the sole owner of BOTH `map→odom` and `odom→base_footprint` (CLAUDE.md Invariants 1–2). It fuses `/wheel_odom` + `/imu/data` (between-factors), `/gps/fix` (`GnssLeverArmFactor`), `/imu/cog_heading` / `/imu/mag_yaw` (yaw unaries) and, with LiDAR, scan-matching between-factors, loop closures and an RTK-anchored keyframe map; it persists the graph to `/ros2_ws/maps/fusion_graph.*`. Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed. Loaded on demand from `ros2/CLAUDE.md`.

## Where to look
| Task | Start here |
|------|------------|
| Add / rename a ROS parameter | Graph-side (`GraphParams`) → `ros2/src/fusion_graph/src/fusion_graph_node.cpp:30-91`; node-side → `ros2/src/fusion_graph/src/fusion_graph_node_setup_params.cpp`; yaml default → `ros2/src/fusion_graph/config/fusion_graph.yaml`; struct default → `ros2/src/fusion_graph/include/fusion_graph/graph_params.hpp` |
| Add a publisher / subscription / service / timer | `ros2/src/fusion_graph/src/fusion_graph_node_setup_comms.cpp` (`SetupCommunications`) |
| Change what a node's factors look like (wheel/gyro/GNSS/yaw/scan/keyframe) | `ros2/src/fusion_graph/src/graph_manager_node.cpp` `CreateNodeLocked` (:76); factor classes in `ros2/src/fusion_graph/include/fusion_graph/factors.hpp` + `src/factors.cpp` |
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
| Scan-matching between-factor + ICP guard rails + yield-to-RTK | `fusion_graph_node_timer.cpp:62-170`; ICP in `src/scan_matcher.cpp` `Match` (:65) |
| Loop closure search / rate gate (#513) / DCS robust factor | `fusion_graph_node_timer.cpp:334-428`; candidates `src/graph_manager_rebase.cpp:33`; `AddLoopClosure` :398 (`kDcsPhi` :433); `include/fusion_graph/loop_closure_gate.hpp` |
| Keyframe map capture / apply (RTK-Float carry) | apply `fusion_graph_node_timer.cpp:176-268`, capture :310-330; store `src/graph_manager_keyframe.cpp`; factor `graph_manager_node.cpp:461-490` |
| Cold-boot relocalization against a loaded graph | `fusion_graph_node_callbacks_b.cpp` `OnScan` (:222-259) and the RTK autoload override in `OnGnss` (WARN at :479) |
| Persistence (`.graph/.scans/.keyframes/.meta`), autoload, empty-graph refusal | `src/graph_manager_persistence.cpp` `Save` (:136) / `Load` (:217); `test_persistence.cpp`, `test_keyframe_map.cpp` `KeyframePersistence` |
| Save / clear services, auto-checkpoints | `fusion_graph_node_setup_comms.cpp:205-270`; `DispatchAsyncSave` `callbacks_b.cpp:363`; `OnHighLevelStatus` :276; `OnPeriodicSaveTimer` `misc.cpp:126` |
| map→odom anchor, slew limiter (unwired), odom re-base, TF thread | `src/fusion_graph_node_publish.cpp` (`PublishOutputs` :153, `SlewPublishedAnchor` :278, `TfBroadcastLoop` :320); re-base `fusion_graph_node_timer.cpp:483-500`; pure `include/fusion_graph/anchor_slew.hpp` |
| Published covariance frame (body→map) | `fusion_graph_node_publish.cpp:183` + `include/fusion_graph/covariance_frame.hpp`; `test_covariance_frame.cpp` |
| Sliding-window cap / iSAM2 rebase / scan pruning | `src/graph_manager_rebase.cpp` `RebaseISAM2` (:112), `PruneOldScans` (:96); maintenance timer `setup_comms.cpp:292-325`; `test_graph_window.cpp` |
| LiDAR map anchor (scan-to-MAP localisation for RTK-Float): grid, gate, particle filter, per-estimate trust, shadow mode | `src/fusion_graph_node_lidar_anchor.cpp` (step, seed, warm-up, verdicts, `/fusion_graph/lidar_anchor_candidate`); pure pieces `include/fusion_graph/lidar_occupancy_mapper.hpp` (log-odds grid, `ScoreScan`, `ImportCells`), `lidar_map_anchor_gate.hpp` (RTK-age hysteresis), `lidar_anchor_validator.hpp` (score / spread / dead-reckoning gates); params `setup_params.cpp` (`use_lidar_map_anchor`, `lidar_anchor_*`, `lidar_map_*`); graph consumer `graph_manager_node.cpp` (`PoseTranslationPrior`, XY only) |
| Replaying a bag through the node on a laptop (reproduce a field failure) | `ros2/src/fusion_graph/tools/replay/README.md` |
| Diagnostics keys (`/fusion_graph/diagnostics`) / marker viz | `fusion_graph_node_setup_comms.cpp:328-512`; counters in `include/fusion_graph/graph_manager.hpp` `GraphStats` (:62) |
| Launch wiring (datum, lever arm, LiDAR gating, cadence) | `ros2/src/fusion_graph/launch/fusion_graph.launch.py`; caller `ros2/src/mowgli_bringup/launch/navigation.launch.py:102-154`, :237-262, :1074-1091 |
| Perf regression | `test/test_perf.cpp` (Release only, `TIMEOUT 300` in `CMakeLists.txt`) |

## Files
| File | Lines | Purpose |
|------|-------|---------|
| **`ros2/src/fusion_graph/` (package root)** | | |
| `ros2/src/fusion_graph/CMakeLists.txt` | 282 | `fusion_graph_core` lib (factors, graph_manager*, scan_matcher) + `fusion_graph_node` exe; 22 `ament_add_gtest` targets; lints disabled in favour of clang-format |
| `ros2/src/fusion_graph/package.xml` | 46 | ament_cmake deps (rclcpp, nav/geometry/sensor/std/diagnostic/visualization msgs, std_srvs, tf2*, mowgli_interfaces, eigen); GTSAM is NOT a rosdep |
| `ros2/src/fusion_graph/config/fusion_graph.yaml` | 480 | Deployed node defaults with field-tuning rationale (overrides `GraphParams` struct + declare defaults; COG / anchor / scan_yield / DR-slip / persistence params are declare-only) |
| `ros2/src/fusion_graph/launch/fusion_graph.launch.py` | 195 | Deep-merges template + installed `mowgli_robot.yaml` for datum/gps_x/gps_y/dock_pose_*; declares `use_magnetometer`, `use_scan_matching`, `use_loop_closure`, `primary_mode`, `tf_publish_lead_s`, `node_period_s`; primary vs observer `Node` |
| **`ros2/src/fusion_graph/include/fusion_graph/`** | | |
| `.../fusion_graph_node.hpp` | 744 | `FusionGraphNode` class: every member + the design rationale comments (DR, anchor, slew, gates, keyframes) |
| `.../fusion_graph_node_util.hpp` | 60 | `YawFromQuat`, `QuatFromYaw`, `ForwardStampPose`, `kEarthRadius` |
| `.../graph_manager.hpp` | 582 | `GraphManager` API, `TickOutput`, `GraphStats`, `Keyframe`, accumulator/queue structs |
| `.../graph_params.hpp` | 373 | `GraphParams` aggregate + `kTunedNodePeriodS` (0.04) / `kMinWheelSigmaM` |
| `.../factors.hpp` | 160 | `GnssLeverArmFactor`, `GyroPreintFactor`, `YawUnaryFactor`, `WrapAngle` |
| `.../scan_matcher.hpp` | 89 | `ScanMatcherParams`, `ScanMatcherResult`, `ScanMatcher` (2D point-to-point ICP) |
| `.../slip_window.hpp` | 125 | Pure windowed slip veto: `SlipWindowNodes`, `SlipDetectedOverWindow`, `SlipWindow` |
| `.../dr_slip_veto.hpp` | 79 | Pure DR slip veto + wheel yaw-rate quantum; shipped thresholds 0.44 / 0.15 rad/s |
| `.../rtk_wrongfix_gate.hpp` | 57 | Pure `GpsJumpImplausible` + `ResetRtkWrongFixAccumulators` (reset on EVERY fix) |
| `.../loop_closure_gate.hpp` | 92 | Pure LC rate/travel gate + GPS σ floor (reset on ACCEPT only) |
| `.../dock_gps_consistency.hpp` | 84 | Pure dock-prior-vs-RTK-Fixed yield rule (`DockPriorShouldYield`) |
| `.../yaw_gates.hpp` | 68 | Pure `CogShouldApply`, `CogEffectiveSigma`, `ScanYawSigma`, `KeyframeYawWithinGate` |
| `.../cog_flip_recovery.hpp` | 86 | Pure 180° flip decision `CogFlipRecoveryFeed` |
| `.../anchor_slew.hpp` | 113 | Pure map→odom anchor slew limiter `AnchorSlewStep` |
| `.../covariance_frame.hpp` | 89 | Pure `BodyToMapCovariance`, `MaxPositionSigma` |
| `.../pose_extrapolator.hpp` | 112 | Header-only `PoseExtrapolator` (gyro-forward yaw for `/odometry/filtered_map_fast`) |
| **`ros2/src/fusion_graph/src/`** | | |
| `.../fusion_graph_node.cpp` | 209 | Ctor: declares `GraphParams` + node-side params, builds `GraphManager`; `main()` |
| `.../fusion_graph_node_setup_params.cpp` | 215 | `DeclareParameters`: dock seed, ICP/keyframe/COG/LC/persistence params, autoload |
| `.../fusion_graph_node_setup_comms.cpp` | 538 | Publishers, subscriptions, `~/save_graph`, `~/clear_graph`, tick/maintenance/diag timers, TF thread start |
| `.../fusion_graph_node_callbacks_a.cpp` | 501 | `OnWheelOdom`, `OnImu` (DR integration + DR slip veto), `OnGnss` (all GPS gates, dock hold, stamp association, autoload override) |
| `.../fusion_graph_node_callbacks_b.cpp` | 440 | `OnDockingCmd`, `OnCogHeading`, `OnMagYaw`, `OnScan` (+ cold-boot relocalize), `OnHighLevelStatus`, `OnSetPose`, `DispatchAsyncSave`, `OnHardwareStatus` |
| `.../fusion_graph_node_timer.cpp` | 517 | `OnTimer`: boot dock fallback, scan-between ICP, keyframe apply/capture, LC search, node tick, odom re-base, publish; `LatLonToMap` |
| `.../fusion_graph_node_publish.cpp` | 397 | `TrySeedInitialPose`, `PublishLocalOdom` (odom→base), `PublishIcpOdom`, `PublishOutputs` (map→odom + `/imu/fg_yaw`), `SlewPublishedAnchor`, `TfBroadcastLoop` |
| `.../fusion_graph_node_misc.cpp` | 140 | `SeedFromDockPose` (rigid gauge reset > 5 cm), `OnPeriodicSaveTimer` |
| `.../graph_manager.cpp` | 468 | Accumulators, queues, `Initialize`, `Stats`, `ForceAnchor`, scan storage, viz snapshots |
| `.../graph_manager_node.cpp` | 574 | `Tick` / `CreateNodeLocked`: all per-node gates + factor construction + iSAM2 update + throttled marginal |
| `.../graph_manager_rebase.cpp` | 452 | LC candidates, `PruneOldScans`, `RebaseISAM2` (window cap), `ApplyIsamUpdateLocked` (ill-posed → reset), `RigidTransformAll`, `AddLoopClosure` (DCS) |
| `.../graph_manager_persistence.cpp` | 440 | `Reset`, `Save`, `Load` (4 files, datum cross-garden guard, tolerant `.keyframes`) |
| `.../graph_manager_keyframe.cpp` | 180 | Keyframe store: add (decimation + cap), lookup, near-XY search, clear, binary (de)serialize |
| `.../scan_matcher.cpp` | 172 | ICP: brute-force NN, Kabsch `RigidAlign2D`, `Match` with per-call `min_inliers` override |
| `.../factors.cpp` | 162 | Factor `evaluateError` + analytic Jacobians + `clone` |
| **`ros2/src/fusion_graph/test/`** | | |
| `.../test_factors.cpp` | 444 | Factor residuals/Jacobians, ScanMatcher recovery, scan-between + keyframe composition conventions |
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
| `.../test_graph_window.cpp` | 194 | `max_graph_nodes` cap, estimate preserved across rebase, LC candidates inside window |
| `.../test_loop_closure_gate.cpp` | 199 | LC rate/travel gate polarity + σ floor |
| `.../test_loop_closure_robust.cpp` | 152 | DCS-wrapped outlier LC does not shift trajectory |
| `.../test_keyframe_map.cpp` | 314 | Keyframe store, xy-only pull, yaw left to gyro, persistence round-trip + datum guard, rigid transform |
| `.../test_persistence.cpp` | 127 | Empty-graph Save/Load refused; non-empty round trip |
| `.../test_map_odom_slew.cpp` | 147 | Anchor slew: rate limit, snap thresholds, shortest-path yaw |
| `.../test_yaw_gates.cpp` | 114 | COG apply/σ gates, scan yaw floor, keyframe mirror guard |
| `.../test_cog_flip_recovery.cpp` | 164 | N-consecutive consistent flips, rate limit, wrap |
| `.../test_covariance_frame.cpp` | 121 | Body→map rotation, `MaxPositionSigma` frame-invariance |
| `.../test_pose_extrapolator.cpp` | 105 | Yaw extrapolation, 200 ms cap, re-baseline |
| `.../test_perf.cpp` | 255 | Tick throughput, mowing session w/ scans+LC, single ICP — loose ceilings |

## Runtime surface

### Nodes
| Node | Executable | Launched by | Type |
|------|------------|-------------|------|
| `fusion_graph_node` | `fusion_graph/fusion_graph_node` | `ros2/src/fusion_graph/launch/fusion_graph.launch.py` (included by `navigation.launch.py:1074` with `primary_mode:=true`) | plain `rclcpp::Node`; single executor + dedicated TF thread + detached save/rebase workers |

`primary_mode=false` (observer) exists in the launch file (remaps `/odometry/filtered_map`→`/fusion_graph/odometry`, no TF) but `navigation.launch.py` never selects it.

### Topics
| Topic | Type | Dir | QoS | Other end |
|-------|------|-----|-----|-----------|
| `/wheel_odom` | `nav_msgs/Odometry` | sub | depth 50 | `hardware_bridge` `~/wheel_odom` remapped in `ros2/src/mowgli_bringup/launch/mowgli.launch.py:260` |
| `/imu/data` | `sensor_msgs/Imu` | sub | SensorData | `hardware_bridge_node` (~91 Hz); only `angular_velocity.z` used |
| `/gps/fix` | `sensor_msgs/NavSatFix` | sub | SensorData | GPS container (`sensors/gps/start_gps.sh:525`); status ≥ `STATUS_FIX` required, `STATUS_GBAS_FIX` = RTK-Fixed |
| `/imu/cog_heading` | `sensor_msgs/Imu` | sub | SensorData | `ros2/src/mowgli_localization/src/cog_to_imu_node.cpp:252` |
| `/imu/mag_yaw` | `sensor_msgs/Imu` | sub (if `use_magnetometer`) | SensorData | `mowgli_localization/src/mag_yaw_publisher_node.cpp:61` |
| `scan_topic` (default `/scan_deskewed`) | `sensor_msgs/LaserScan` | sub (if `use_scan_matching` or `use_loop_closure`) | SensorData | `mowgli_localization/src/scan_deskew_node.cpp` |
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
| `/fusion_graph/markers` | `visualization_msgs/MarkerArray` 1 Hz | pub | depth 1 transient_local | Foxglove (ids 0 nodes / 1 trajectory / 2 LC edges, ≤1500 nodes) |
| `/fusion_graph/icp_odometry` | `nav_msgs/Odometry` | pub (scan matching only) | depth 10 | GUI `ros.go:56`, `useIcpOdom.ts` |
| `/odometry/filtered_map_fast` | `nav_msgs/Odometry` | pub (if `fast_pose_publish_rate_hz` > 0) | SensorData | none by default |

Diagnostics keys (`setup_comms.cpp:349-412`): `total_nodes scans_attached loop_closures lc_rate_gated dock_gps_disagreement_m dock_prior_yielded scans_received scan_matches_ok scan_matches_fail keyframes_total kf_matches_ok kf_matches_fail gps_rejects_wrongfix icp_rejects_rmse icp_rejects_inliers icp_rejects_sanity icp_rejects_divergence stationary_hand_push slip_veto live_nodes gyro_bias_z_rad_per_s gyro_bias_updates residual_ema_rad wheel_sigma_x_eff cov_xx cov_yy cov_yawyaw`.

### Services & actions
| Service | Type | Where | Caller |
|---------|------|-------|--------|
| `~/save_graph` (`/fusion_graph_node/save_graph`) | `std_srvs/Trigger` | `setup_comms.cpp:205` — async `DispatchAsyncSave("manual-service")` | GUI `gui/pkg/api/mowglinext.go:668` (`fusion_graph_save`) |
| `~/clear_graph` (`/fusion_graph_node/clear_graph`) | `std_srvs/Trigger` | `setup_comms.cpp:229` — `Reset()` + `ClearKeyframes()` + seeds + DR zeroed | GUI `mowglinext.go:670` (`fusion_graph_clear`) |

No actions. Auto-save triggers: RECORDING exit (`callbacks_b.cpp:276`), `is_charging` rising edge / boot-docked (`callbacks_b.cpp:404`), `periodic_save_period_s` (300 s) while `HIGH_LEVEL_STATE_AUTONOMOUS` (`misc.cpp:126`).

### Parameters
All read ONCE at construction (`declare_parameter`, no dynamic reconfigure). Precedence: `GraphParams` struct default < `fusion_graph_node.cpp` / `setup_params.cpp` declare default < `config/fusion_graph.yaml` < launch dict in `fusion_graph.launch.py:140-160` (datum, lever arm, dock pose, `use_*`, `tf_publish_lead_s`, `node_period_s`); primary/observer `Node` actions at `:166-184`.

| Parameter | Declared | Yaml default | Notes |
|-----------|----------|--------------|-------|
| `node_period_s` | `fusion_graph_node.cpp:31` (0.1) | `fusion_graph.yaml:23` 0.02 | Overridden by `navigation.launch.py` from `mowgli_robot.yaml` `fusion_graph_node_period_s` (fallback 0.04); per-tick gates scale by `node_period_s/kTunedNodePeriodS` |
| `wheel_sigma_x_per_sqrt_m` / `_y_` / `wheel_creep_speed_mps` | `:33-35` | `:52-57` 0.05 / 0.005 / 0.04 | σ = k·√(step + creep·dt) (#491) |
| `gyro_sigma_theta`, `stationary_sigma_theta` | `:37`, `:61` | `:64` 0.005, `:97` 0.01 | stationary snap σ relaxed to 0.01 for iSAM2 conditioning |
| `slip_residual_thresh_rad` / `slip_gyro_max_rad` / `slip_wheel_min_rad` / `slip_window_s` | `:69-73` | `:138-160` 0.01 / 0.005 / 0.005 / 0.5 | windowed veto (#516); 0 = per node |
| `max_graph_nodes`, `isam2_rebase_every_nodes`, `scan_retention_nodes` | `:54`, `setup_params.cpp:177`, `:175` | `:183` 6000, (2000), (18000) | window MUST outlast `lc_min_age_s/node_period_s` |
| `lc_min_age_s`, `lc_skip_when_rtk_fixed`, `lc_min_travel_m`, `lc_min_interval_s`, `lc_gps_sigma_ratio` | `setup_params.cpp:147-170` | `:204-234` 30 / true / 1.0 / 2.0 / 1.0 | #513 gate |
| `rtk_wrongfix_max_jump_m` | `fusion_graph_node.cpp:94` | `:284` 0.05 | slack on top of wheel travel + lever sweep |
| `gps_sigma_floor`, `gps_sigma_speed_coeff`, `gps_max_sigma_reject_m` | `:38`, `:98`, `:104` | `:312` 0.003, `:294` 0.0, (0.0) | |
| `dock_reanchor_sigma_xy_m`, `dock_prior_max_gps_disagreement_m`, `dock_prior_max_gps_sigma_m` | `:109-124` | `:399-414` 0.03 / 0.50 / 0.05 | #512 |
| `dr_slip_gyro_max_rad_per_s`, `dr_slip_wheel_min_rad_per_s` | `:129-132` | not in yaml (0.15 / 0.44 from `dr_slip_veto.hpp:49-50`) | #488 |
| `use_scan_matching`, `use_loop_closure`, `use_magnetometer`, `primary_mode` | `setup_params.cpp:41,141,132,138` | via launch | LiDAR flags ANDed with `use_lidar` in `navigation.launch.py:237` |
| `use_lidar_map_anchor`, `lidar_anchor_shadow_mode` | `setup_params.cpp` | via launch, from `mowgli_robot.yaml` (template default `false` / `false`), LiDAR-gated | Shadow = run, score and publish the filter under RTK-Fixed too, never apply |
| `lidar_map_*` (resolution 0.10, half_extent 40, insert 0.5 s, rebuild 5 s), `lidar_anchor_engage_age_s` 1.0 / `_disengage_dwell_s` 1.0, particles 300–1500, `update_min_d/a` 0.05, seed σ 0.10 m / 0.10 rad, `z_hit` 0.7, `sigma_hit` 0.15, `alpha_slow/fast` 0.0 | `config/fusion_graph.yaml` | yaml only | Engage age must stay ≥ 1 s: a 5 Hz receiver's Fixed age idles at 0.08–0.28 s, 0.3 s flapped on timer phase |
| `lidar_anchor_min_hit_ratio` 0.5, `_min_hit_count` 30, `_max_sigma_m` 0.5, `_dr_budget_m` 0.3, `_dr_drift_frac` 0.02, `_reseed_after_s` 5.0 | `config/fusion_graph.yaml` → `LidarAnchorValidatorParams` | yaml only | Per-estimate trust; sized on the 2026-09-06 outage (lost filter scored 0.13, DR drifted < 0.4 m in 2.7 min) |
| `use_keyframe_map`, `kf_capture_sigma_max_m`, `kf_min_inliers`, `kf_match_max_divergence_xy_m`, `kf_apply_yaw_sigma_floor_rad` | `setup_params.cpp:80-103`, `fusion_graph_node.cpp:155` | `:429-480` true / 0.04 / 16 / 0.10 / 0.30 | declare defaults differ (false / 0.01 / 0.30) — yaml wins |
| `scan_topic`, `icp_max_*`, `scan_yield_*`, `scan_yaw_sigma_floor_rad` | `setup_comms.cpp:141`, `setup_params.cpp:61-73,124` | `:302-306` | |
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
- Run standalone: `ros2 launch fusion_graph fusion_graph.launch.py use_scan_matching:=true use_loop_closure:=true`; manual save: `ros2 service call /fusion_graph_node/save_graph std_srvs/srv/Trigger`.

## Change coupling — "if you change X, also update Y"
- Rename/add a parameter → `config/fusion_graph.yaml` AND the declare site (`fusion_graph_node.cpp` for `GraphParams`, `setup_params.cpp` otherwise); if it is a per-tick radian/metre threshold, apply `tick_scale` in `graph_manager_node.cpp:98` or it silently changes meaning with cadence.
- `use_scan_matching` / `use_loop_closure` / `use_magnetometer` / `fusion_graph_node_period_s` → read from `mowgli_robot.yaml` by `navigation.launch.py:130-140`; template defaults for the three `use_*` flags at `ros2/src/mowgli_bringup/config/mowgli_robot.yaml:238-264` — `fusion_graph_node_period_s` has NO template default (its 0.04 fallback lives only at `navigation.launch.py:139`, so the GUI cannot reset it); GUI toggles `LocalizationSection.tsx:35-41`, live-param catalog `gui/web/src/components/settings/paramCatalog.ts:63-65` (keyed on the ROS name `node_period_s`); pinned by `mowgli_bringup/test/test_robot_config_util.py:520`.
- `gps_x`/`gps_y`/`datum_lat`/`datum_lon`/`dock_pose_*` → `fusion_graph.launch.py:43-84` `_read_robot_config` deep-merge (local copy of `robot_config_util.deep_merge`; keep in sync, no package dep allowed). `cog_to_imu` and `navsat_to_absolute_pose` read the same keys via `navigation.launch.py` — lever arm must agree (Invariant 4/6).
- Diagnostics key added/removed in `setup_comms.cpp:349-412` → GUI `gui/web/src/hooks/useFusionGraphDiagnostics.ts` + `DiagnosticsPage.tsx:827`, i18n `gui/web/src/i18n/locales/{en,fr}.json`, and the session-monitor JSONL (`docs/claude/session-monitoring.md`).
- Service name change → `gui/pkg/api/mowglinext.go:668-670`.
- Persistence format (`Save`/`Load`, keyframe binary) → bump tolerance in `graph_manager_persistence.cpp:253-262`; `navigation.launch.py:150` tests for `/ros2_ws/maps/fusion_graph.graph` existence to enable loop closure on first boot; `test_persistence.cpp`, `test_keyframe_map.cpp KeyframePersistence`.
- `~/set_pose` QoS (`setup_comms.cpp:193-199`) must match `mowgli_behavior/src/calibration_nodes.cpp:203,316` (reliable + transient_local).
- `/odometry/filtered` twist fields (`publish.cpp:105-113`) feed Nav2 `odom_topic` (`nav2_params_base.yaml:24`); controller_server uses `/wheel_odom` (`:77`, `:907`) — see CLAUDE.md "What NOT to Do".
- `Status.is_charging` / `HighLevelStatus.state` constants (`mowgli_interfaces`) drive dock seed and auto-save — regenerate bindings per `docs/claude/commands.md` if the msgs change.
- Scan-between / keyframe composition direction is locked by `test_factors.cpp` `ScanBetweenConvention` + `ScanToKeyframeComposition`; `AddLoopClosure` expects `(prev, curr)` (`graph_manager_rebase.cpp:398`).

## Pitfalls

- **`beluga_ros::Amcl`'s first update applies the WHOLE odom pose, not a delta** — its motion model reads a 2-pose odometry window with no first-update special case, so the first estimate after construction lands metres away (3.2 m in the 2026-09-07 replay). `RebuildLidarAnchorMap` does one discarded warm-up update right after construction; do not remove it. Re-seeds are unaffected (window already full).
- **Never feed a LiDAR-anchor estimate to the graph unvalidated.** The 2026-09-06 outage put the fused pose 10 m off with the filter's own covariance saying 5 cm. Every candidate goes through `ValidateLidarAnchor` (scan consistency AT THE ESTIMATE, particle spread, plausibility vs dead reckoning); dead reckoning is the better witness beyond its drift budget, and the cloud is re-seeded from it after `lidar_anchor_reseed_after_s` of rejects.
- The anchor map only grows while MAPPING (fresh RTK-Fixed): it is frozen during ANCHORING by design, so a robot that drives out of the mapped area during an outage is on dead reckoning until it comes back — the score gate makes that explicit (`lidar_anchor_rej_score` climbs, no factors).
- Three layers of defaults disagree: `GraphParams` struct (`graph_params.hpp`, e.g. `max_graph_nodes` 3000, `stationary_sigma_theta` 1e-3) vs declare defaults vs `fusion_graph.yaml` (6000, 0.01) vs launch (`node_period_s` 0.1 → yaml 0.02 → deployed 0.04). Unit tests construct `GraphParams` directly, so they run on struct defaults, not production values.
- `lc_min_age_s` must stay ≪ `max_graph_nodes × node_period_s` or `FindLoopClosureCandidates` (`graph_manager_rebase.cpp:53-63`) never returns a candidate (`fusion_graph.yaml:171-204`). Loop closure is also force-off until a persisted `.graph` exists (`navigation.launch.py:150-154`).
- The keyframe factor is `PoseTranslationPrior<Pose2>` (xy only, `graph_manager_node.cpp:461-490`, reverted 2026-07-22) — the comments in `fusion_graph.yaml:420-423`, `graph_manager.hpp:130-137` and `fusion_graph_node.hpp:676-686` still describe a `PriorFactor<Pose2>` with yaw. Yaw enters only through the mirror-guard rejection (`timer.cpp:251`), never as a factor.
- `primary_mode=false` comments in `setup_params.cpp:134-137` and the arg description in `fusion_graph.launch.py:102-105` describe an `ekf_map_node` fallback that no longer exists; `navigation.launch.py:1088` always passes `"true"`.
- RTK wrong-fix gate accumulators reset on EVERY fix (`callbacks_a.cpp` `OnGnss`, `rtk_wrongfix_gate.hpp:45-51`); loop-closure gate accumulators reset on ACCEPT only (`loop_closure_gate.hpp:32-46`). Do not "harmonise" them — CLAUDE.md "GnssMobileGate" incident.
- A NavSatFix with `COVARIANCE_TYPE_UNKNOWN` or σ ≤ 0 is rejected outright (`callbacks_a.cpp:196-210`) — the old σ=-1 sentinel was clamped UP to 3 mm and fused garbage at RTK precision.
- While `is_charging` the node suppresses live GPS factors and re-asserts the dock prior once per node, and `seed_xy_` is NOT updated from the docked fix — the dock bootstrap is `SeedFromDockPose` (`misc.cpp:25`), not `TrySeedInitialPose`, which needs a GPS `seed_xy_` + COG/mag `seed_yaw_` (`publish.cpp:27-33`). A yielded node (#512) still marks `last_dock_reanchor_node_` (`callbacks_a.cpp:343-364`).
- `Save()` refuses an empty graph (`graph_manager_persistence.cpp:159`) — a Reset followed by auto-save used to persist `next_index=0` and crash the next boot (`test_persistence.cpp`).
- `Load()` rejects a map whose `.meta` datum differs from the configured one (cross-garden guard, `graph_manager_persistence.cpp:300-309`); (0,0) datum skips the check. `.keyframes` missing/corrupt degrades to an empty keyframe map, not a failed load (`:253-262`).
- `clear_graph` zeroes `dr_*` and invalidates the anchor (`setup_comms.cpp:246-266`); an odom→base discontinuity is expected — only call it parked.
- Per-tick gates (`stationary_thresh_*`, `pivot_gate_dtheta_rad`, `slip_*`) are tuned at 25 Hz and multiplied by `tick_scale` (`graph_manager_node.cpp:98`); raising the per-frame slip thresholds instead of using `slip_window_s` blinds the veto (`fusion_graph.yaml:141-160`).
- `dr_slip_wheel_min_rad_per_s` must exceed 2× the wheel yaw-rate quantum (`dr_slip_veto.hpp:10-32`, #488) — a lower value zeroes DR translation on straight drives and breaks Nav2 BackUp distances (Invariant 10).
- Heavy work off the executor: `RebaseISAM2` (maintenance timer, detached thread, `setup_comms.cpp:292-325`) and `Save` (`DispatchAsyncSave`) — never call them inline; TF has its own thread for the same reason (`fusion_graph_node.hpp:401-424`).
- `tf_publish_lead_s` is a genuine forward prediction (`ForwardStampPose`); sim needs 0.1, hardware 0.05 (`navigation.launch.py:214-217`). `fusion_graph.launch.py` arg default is 0.0 — only relevant when launched standalone.
- `/fusion_graph/markers` is transient_local and decimated to 1500 nodes; `GetAllPoses` is O(N) — keep it on the 1 Hz diag timer.
- Scan subscription defaults to `/scan_deskewed`, not `/scan` (`setup_comms.cpp:141`); with `use_lidar=false` both LiDAR flags are ANDed off upstream so the node never subscribes to a dead topic.
- Invariant 16: the graph-side slip veto is rotational only; do not extend it to catch straight-line digs — that is `hardware_bridge`'s dig detector's job.

## Generated & vendored — do not hand-edit
- GTSAM 4.3a1 lives outside the repo (`/opt/gtsam`, built by `ros2/Dockerfile` stage 0 / CI cache) — do not vendor it into `ros2/src`.
- `ros2/build/`, `ros2/install/`, `ros2/log/` are colcon outputs; `/ros2_ws/maps/fusion_graph.{graph,scans,keyframes,meta}` are runtime artefacts written by `GraphManager::Save`.
