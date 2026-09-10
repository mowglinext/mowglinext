# Codemap: mowgli_localization

> Sensor-preparation helpers that feed the `fusion_graph` localizer (see CLAUDE.md Invariants 1–2): WGS84→ENU GPS projection (`/gps/absolute_pose`, `/gps/pose_cov`), GPS course-over-ground yaw (`/imu/cog_heading`), magnetometer yaw (`/imu/mag_yaw`), the LiDAR scan pipeline (`/scan` → `/scan_deskewed` → `/scan_costmap` + `/scan_collision`), GPS-anchored dock detection for opennav_docking, the IMU/mag/dock calibration node, and a GPS-quality mode monitor. It publishes **no TF** in production and owns none of the map/odom estimate. The robot_localization EKFs are gone — only stale comments remain here (see Pitfalls).
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `ros2/CLAUDE.md`.

## Where to look
| Task | Start here |
|------|------------|
| Change how `/gps/fix` becomes `/gps/pose_cov` (lever arm, covariance inflation, RTK gate) | `ros2/src/mowgli_localization/src/navsat_to_absolute_pose_node.cpp` `on_navsat_fix()` (L192–428); gate at L345–353 |
| Map RTK state from `/gps/status` vs NavSatFix fallback into `AbsolutePose.flags` | `ros2/src/mowgli_localization/include/mowgli_localization/navsat_projection_utils.hpp` (`ResolveAbsolutePoseFlags`, `HasAuthoritativeRtkPoseState`) |
| Datum handling / `set_datum` service | `navsat_to_absolute_pose_node.cpp` `on_set_datum()` L152–183, `wgs84_to_enu()` L440 |
| COG yaw wrong during reverse / turns, lever-arm de-bias, σ_yaw model | `ros2/src/mowgli_localization/include/mowgli_localization/cog_yaw_math.hpp` (`compute_cog_body_yaw`, `cog_sweep_dominates`, `cog_latch_rotation_increment`) then `src/cog_to_imu_node.cpp` `on_fix()` L317 |
| Stationary yaw latch republish / staleness after pivots | `src/cog_to_imu_node.cpp` `republish_latched_when_stationary()` L573; params L120–161 |
| Online magnetometer hard-iron refit (writes `mag_calibration.yaml`) | `src/cog_to_imu_node.cpp` `maybe_refit_mag()` L707, `write_mag_cal()` L876 |
| Tilt-compensated mag yaw, declination, calibration reload | `ros2/src/mowgli_localization/src/mag_yaw_publisher_node.cpp` `on_mag()`, `reload_cal_if_needed()` L123 |
| Localization mode string/enum, debounce | `ros2/src/mowgli_localization/src/localization_monitor_node.cpp` `evaluate_mode()` L192, `on_publish_timer()`; enum in `include/.../localization_monitor_node.hpp` L62–68 |
| LiDAR scan smeared while rotating (deskew, IMU buffer, "no scans" watchdog) | `ros2/src/mowgli_localization/src/scan_deskew_node.cpp` `on_scan()` L342, `on_scan_watchdog()` L311 |
| Dock/chassis self-return blanking, gravity ground filter, `/scan_collision` split | `ros2/src/mowgli_localization/src/costmap_scan_filter_node.cpp` `filter_scan()` L217, `apply_ground_filter()` L250, `is_blank_active()` L437 |
| Docking goes to the wrong spot / `detected_dock_pose` geometry, Δ (map→odom yaw) gate | `ros2/src/mowgli_localization/src/gps_dock_detection_node.cpp` header doc L17–105, `apply_delta_sample()` L344, `on_timer()` L399; gate in `include/.../delta_gate.hpp` |
| One-click dock calibration (CalibrateDock action / GUI start service / status topic) | `ros2/src/mowgli_localization/src/calibrate_imu_yaw_node.cpp` `run_dock_calibration_core()` L1197, `persist_dock_via_map_server()` L1148, `dock_start_cb()` L1107 |
| COG-coherence gate for the dock reverse leg | `ros2/src/mowgli_localization/include/mowgli_localization/dock_cog_gate.hpp` `evaluate_dock_cog_gate()` |
| Legacy blocking IMU-yaw / pitch-roll / mag figure-8 calibration service | `calibrate_imu_yaw_node.cpp` `calibrate_cb()` L1679, legacy dock pre-phase `run_dock_yaw_drive()` L568 (writes yaml directly at L723) |
| Where dock_pose_x/y/yaw get written to `mowgli_robot.yaml` | `ros2/src/mowgli_interfaces/include/mowgli_interfaces/robot_yaml_scalar.hpp` (`UpdateDockPose`); callers: `mowgli_map/src/area_manager.cpp:916,1673`, `calibrate_imu_yaw_node.cpp:723` |
| Wheel-tick → odometry kinematics (node is NOT launched in production) | `ros2/src/mowgli_localization/src/wheel_odometry_node.cpp` `on_wheel_tick()` |
| Which launch file starts a node and with which params | `ros2/src/mowgli_bringup/launch/full_system.launch.py` L454–520; `navigation.launch.py` L1114–1289; `sim_full_system.launch.py` L348–360 |
| Template defaults for the params these nodes read | `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` L176–183 (mag), L204–219 (lidar/imu/gps mounts), L272–273 (datum), L467 (`use_gps_dock_detection`), L472–502 (`dock_calib_*`) |
| Add a unit test | `ros2/src/mowgli_localization/CMakeLists.txt` L254–349 (`ament_add_gtest` blocks) |

## Files
| File | Lines | Purpose |
|------|-------|---------|
| **`ros2/src/mowgli_localization/`** | | |
| `CMakeLists.txt` | 368 | 9 executables, install rules, 7 gtest registrations |
| `package.xml` | 50 | Deps (rclcpp, rclcpp_action, tf2*, mowgli_interfaces, yaml-cpp); description text is stale (see Pitfalls) |
| `config/wheel_odometry.yaml` | 18 | `wheel_odometry` params (0.325 m / 399 ticks/m / `publish_tf: false`); loaded by no launch file, pinned by `test_tf_ownership.py` |
| **`include/mowgli_localization/`** | | |
| `cog_yaw_math.hpp` | 166 | Pure COG→body-heading algebra, sweep-dominance gate, latch rotation increment |
| `delta_gate.hpp` | 163 | Pure `DeltaGate`: jump-reject + EMA + stale-latch re-seed for map→odom yaw Δ (issue #390) |
| `dock_cog_gate.hpp` | 184 | Pure circular mean/std + `evaluate_dock_cog_gate()` (dock yaw = reversed RTK travel bearing) |
| `localization_monitor_node.hpp` | 159 | `LocalizationMonitorNode`, `LocalizationMode` enum (0..3) |
| `navsat_projection_utils.hpp` | 53 | `/gps/status`-first RTK flag/eligibility resolution with NavSatFix fallback |
| `navsat_to_absolute_pose_node.hpp` | 151 | `NavSatToAbsolutePoseNode` members + covariance-guard rationale |
| `wheel_odometry_node.hpp` | 121 | `WheelOdometryNode` |
| **`src/`** | | |
| `calibrate_imu_yaw_node.cpp` | ~2.3k | IMU-yaw/pitch-roll/mag calibration service + one-click `CalibrateDock` action + GUI façade (start service + status topic) |
| `cog_to_imu_node.cpp` | ~1.0k | GPS course-over-ground → `/imu/cog_heading`; stationary latch; online mag hard-iron fit |
| `costmap_scan_filter_node.cpp` | 497 | Chassis/dock radial blank + gravity ground filter → `/scan_costmap`; blanked-only → `/scan_collision` |
| `gps_dock_detection_node.cpp` | 480 | RTK-anchored dock pose in `odom` for opennav_docking external detection |
| `localization_monitor_node.cpp` | 258 | GPS-freshness/RTK mode → `/mowgli/localization/mode(_id)` with dwell debounce |
| `mag_yaw_publisher_node.cpp` | 327 | Hard/soft-iron cal + tilt compensation → `/imu/mag_yaw`; idles until cal file appears |
| `navsat_to_absolute_pose_node.cpp` | 461 | `/gps/fix` + `/gps/status` → `/gps/absolute_pose` + `/gps/pose_cov`; `~/set_datum` |
| `scan_deskew_node.cpp` | 532 | Per-ray rotational (opt-in linear) deskew via IMU history buffer; silent-LiDAR watchdog |
| `wheel_odometry_node.cpp` | 283 | Differential-drive tick integration → `/wheel_odom` (dead in production) |
| **`test/`** | | |
| `test_cog_yaw_math.cpp` | 262 | Forward/reverse/turning heading recovery, sweep gate, latch staleness |
| `test_costmap_scan_filter.cpp` | 397 | Radial blank + ground filter (mount yaw π, run-length guard); **re-implements** the node's static helpers |
| `test_delta_gate.cpp` | 186 | Seed/accept/reject/re-seed/flapping behaviour of `DeltaGate` |
| `test_dock_cog_gate.cpp` | 185 | Circular stats + all four reject reasons + no-π-offset invariant |
| `test_navsat_projection_utils.cpp` | 62 | Typed RTK state wins over NavSatFix status; standalone fix never masquerades as RTK |
| `test_robot_yaml_scalar.cpp` | 161 | `UpdateDockPose` splice: values only, comments preserved, lookalike key untouched, atomic write |
| `test_wheel_odometry.cpp` | 241 | Kinematics (straight, pivot, arcs, quarter circle, reverse) — mirrors `on_wheel_tick()` |

## Runtime surface

### Nodes
| Executable | Node name | Launched by | Notes |
|------------|-----------|-------------|-------|
| `navsat_to_absolute_pose_node` | `navsat_to_absolute_pose` | `full_system.launch.py` L454 (datum only); `sim_full_system.launch.py` L348 (hardcoded sim datum) | plain node |
| `localization_monitor_node` | `localization_monitor_node` (launch) / `localization_monitor` (code) | `full_system.launch.py` L471 — only `use_sim_time` passed | plain |
| `calibrate_imu_yaw_node` | `calibrate_imu_yaw_node` | `full_system.launch.py` L484 — `undock_*`, `dock_calib_*` from `robot_params` | plain, `MultiThreadedExecutor`, reentrant cb group |
| `cog_to_imu` | `cog_to_imu` | `navigation.launch.py` L1114 — datum, `lever_arm_x/y`=`gps_x/gps_y` (L1127–1128), `enable_mag_cal`, `mag_calibration_path`, `stationary_seed_rate_hz` (from the `cog_stationary_seed_rate_hz` arg, default 2.0), `stationary_yaw_drift_rate: 0.001` | plain |
| `mag_yaw_publisher` | `mag_yaw_publisher` | `navigation.launch.py` L1157 — **only if** `use_magnetometer` AND `/ros2_ws/maps/mag_calibration.yaml` exists at launch time (L1156) | plain |
| `scan_deskew_node` | `scan_deskew` | `navigation.launch.py` L1187 — `IfCondition(use_lidar)` | plain |
| `costmap_scan_filter_node` | `costmap_scan_filter` | `navigation.launch.py` L1203 — `IfCondition(use_lidar)`; `chassis_blank_range 0.55`, `min_obstacle_z_m 0.15`, `lidar_height_m`=`lidar_z`, `lidar_mount_yaw`=`lidar_yaw−imu_yaw` | plain |
| `gps_dock_detection_node` | `gps_dock_detection` | `navigation.launch.py` L1259 — `IfCondition(use_gps_dock_detection)`; remap `detected_dock_pose`→`/detected_dock_pose` | plain |
| `wheel_odometry_node` | `wheel_odometry` | **not launched** (`full_system.launch.py` L439–446 comment); only `mowgli_bringup/test/test_nodes_startup.launch.py` | plain |

### Topics
| Topic | Type | Dir (node) | QoS | Other end |
|-------|------|------------|-----|-----------|
| `/gps/fix` | `sensor_msgs/NavSatFix` | sub (navsat L121; cog_to_imu L197) | reliable(10) / best-effort(10) | Universal GNSS `receiver_node`; sim relay `sim_navsat_rtk_fix.py` |
| `/gps/status` | `mowgli_interfaces/GnssStatus` | sub (navsat L128) | reliable(10) | Universal GNSS (authoritative RTK state) |
| `/gps/absolute_pose` | `mowgli_interfaces/AbsolutePose` | pub (navsat L110) | reliable(10) | subs: localization_monitor, gps_dock_detection, calibrate_imu_yaw, `behavior_tree_node.cpp:346`, GUI |
| `/gps/pose_cov` | `geometry_msgs/PoseWithCovarianceStamped` | pub (navsat L114), frame `map`, base_footprint position | reliable(10) | `fusion_graph` GnssLeverArmFactor; `map_server_node.cpp:269` (set_docking_point averaging) |
| `/wheel_odom` | `nav_msgs/Odometry` | sub (localization_monitor L93, cog_to_imu L204, calibrate_imu_yaw L429, scan_deskew if `linear_comp_enabled`); pub (wheel_odometry L92, dead) | | producer: `hardware_bridge` (`mowgli_hardware/src/odometry_publisher.cpp:35`, remap `mowgli.launch.py:260`) |
| `/wheel_ticks` | `mowgli_interfaces/WheelTick` | sub (wheel_odometry L103) | reliable(10) | `odometry_publisher.cpp:40` (`~/wheel_ticks`, remap `mowgli.launch.py:261`) |
| `/imu/data` | `sensor_msgs/Imu` | sub (mag_yaw L108, cog_to_imu L215, scan_deskew L68, costmap_scan_filter L119, calibrate_imu_yaw L405) | best-effort | `hardware_bridge` |
| `/imu/mag_raw` | `sensor_msgs/MagneticField` | sub (mag_yaw L115, cog_to_imu L257 when `enable_mag_cal`, calibrate_imu_yaw L413) | best-effort | `hardware_bridge` |
| `/imu/cog_heading` | `sensor_msgs/Imu` (yaw only, `frame_id base_footprint`, cov[8]=σ²_yaw) | pub (cog_to_imu L252) | best-effort(10) | `fusion_graph_node_setup_comms.cpp:120`; gps_dock_detection L224; calibrate_imu_yaw L300 |
| `/imu/mag_yaw` | `sensor_msgs/Imu` | pub (mag_yaw L61) | best-effort(10) | `fusion_graph_node_setup_comms.cpp:128` (only `use_magnetometer`); GUI `useMagYaw.ts` |
| `/mowgli/localization/mode` / `/mowgli/localization/mode_id` | `std_msgs/String` / `std_msgs/Int32` | pub (localization_monitor L85–87) | `QoS(1).transient_local()`, `publish_rate` Hz | **no subscriber in `ros2/src` or `gui/`** |
| `/scan` → `/scan_deskewed` | `sensor_msgs/LaserScan` | scan_deskew sub/pub | `SensorDataQoS` | LiDAR driver → costmap_scan_filter |
| `/scan_deskewed` → `/scan_costmap` | `LaserScan` | costmap_scan_filter sub/pub (L110–112) | `SensorDataQoS` | obstacle layers `nav2_params_lidar.yaml:27,95` |
| `/scan_collision` | `LaserScan` (blanked, NOT ground-filtered) | pub (costmap_scan_filter L116, L404) | `SensorDataQoS` | collision_monitor `nav2_params_lidar.yaml:273` |
| `/hardware_bridge/status` | `mowgli_interfaces/Status` | sub (costmap_scan_filter L118 `is_charging`; calibrate_imu_yaw L220) | reliable(10) | `hardware_bridge` |
| `/hardware_bridge/emergency`, `/behavior_tree_node/high_level_status` | `Emergency`, `HighLevelStatus` | sub (calibrate_imu_yaw L228, L236) | reliable(10) | abort / state gating for calibration drives |
| `/cmd_vel_docking` | `geometry_msgs/TwistStamped` | pub (calibrate_imu_yaw L250) | reliable(10) | twist_mux priority 15 (Invariant 13) — collision_monitor stays in loop |
| `detected_dock_pose` → `/detected_dock_pose` | `geometry_msgs/PoseStamped` in `odom` | pub (gps_dock_detection L210) | `QoS(1)`, `publish_rate_hz` | opennav_docking `SimpleChargingDock` (`use_external_detection_pose`) |
| `~/dock_calibration/status` (`/calibrate_imu_yaw_node/dock_calibration/status`) | `mowgli_interfaces/DockCalibrationStatus` | pub (L335) | reliable(10) | GUI `gui/web/src/hooks/useDockCalibration.ts` |

### Services & actions
| Name | Type | Role | Caller |
|------|------|------|--------|
| `/navsat_to_absolute_pose/set_datum` | `std_srvs/Trigger` | server (L144): sets in-memory datum from `last_fix_`; requires `NavSatFix.status >= STATUS_GBAS_FIX` (L165); returns `"lat,lon"` | GUI `gui/pkg/api/mowglinext.go:601` |
| `/calibrate_imu_yaw_node/calibrate` | `mowgli_interfaces/srv/CalibrateImuYaw` | server (L252): blocking IMU-yaw drive cycles; `mag_only=true` → figure-8 mag pass; dock pre-phase if charging | GUI `gui/pkg/api/calibration.go:129,197` |
| `/calibrate_imu_yaw_node/calibrate_dock` | `mowgli_interfaces/action/CalibrateDock` | action server (L314): WAIT_RTK → REVERSING → CHECK_COG → REDOCKING → VERIFY_CHARGE → PERSIST | (no GUI client; GUI uses the façade below) |
| `/calibrate_imu_yaw_node/dock_calibration/start` | `std_srvs/Trigger` | server (L337): non-blocking start of the same core, progress on the status topic | GUI `calibration.go:84` |
| `/map_server_node/set_docking_point` | `mowgli_interfaces/srv/SetDockingPoint` | **client** (L310): `use_gps_position=true`, `yaw_source=MOTION` | one-click persist path (L1148) |
| `/behavior_tree_node/high_level_control` | `mowgli_interfaces/srv/HighLevelControl` | client (L244): HOME / RECORD_AREA / STOP during calibration | — |

### Parameters
All are read once in the constructor (no dynamic reconfigure). Template defaults live in `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` (Invariant 15) only for the values the launch files forward; every other param uses the C++ default.

| Node | Key params (default) | Source of the launched value |
|------|---------------------|------------------------------|
| navsat | `datum_lat/lon` (0.0) L92–93; `lever_arm_yaw_sigma` (0.0524) L97; `pos_accuracy_inflation_threshold_m` (0.025) L101; `pos_accuracy_inflation_factor` (10) L102; `pos_accuracy_reject_threshold_m` (0.5) L104 | datum from `robot_params` (`full_system.launch.py` L460–463); rest C++ defaults |
| cog_to_imu | `min_abs_wheel_ms` (0.05) L90; `min_omega_for_anchor_rps` (0.50) L102; `cog_sweep_dominance_ratio` (1.0) L109; `cog_max_baseline_rotation_rad` (0.20) L114; `latch_republish_max_omega_rps` (0.05) L120; `latch_max_rotation_rad` (0.26) L136; `max_pos_accuracy_m` (0.05) L143; `min_baseline_displacement_m` (0.10) L156; `stationary_seed_rate_hz` (2.0) L159; `stationary_yaw_drift_rate` (0.005) L160; `lever_arm_x/y` (0.30/0.0) L177–178; `enable_mag_cal` (true in C++, **false** in template L177) L187 | `navigation.launch.py` L1114–1148 |
| mag_yaw_publisher | `calibration_path`, `declination_deg` (1.5), `min_horizontal_uT` (5.0), `yaw_variance` (2.7e-3) L51–55 | template L179–181 → `navigation.launch.py` L578–580, forwarded L1168–1170. `calibration_path` does **not** come from the template: L1155 rebinds `mag_cal_path` to the hardcoded `/ros2_ws/maps/mag_calibration.yaml` |
| localization_monitor | `gps_timeout` (2.0), `pose_timeout` (0.5, **declared but unused**), `publish_rate` (10), `mode_debounce_sec` (1.0) L73–76 | C++ defaults only |
| scan_deskew | `input_topic`, `output_topic`, `imu_topic`, `reference` ("end"), `imu_max_age_s` (0.5), `imu_buffer_horizon_s` (0.5), `linear_comp_enabled` (false), `wheel_topic`, `scan_watchdog_period_s` (20) L64–155 | `navigation.launch.py` L1193–1198 |
| costmap_scan_filter | `dock_blank_range` (0.70), `chassis_blank_range` (0.0), `post_undock_blank_sec` (5), `enable_ground_filter` (true), `min_obstacle_z_m` (0.08), `max_obstacle_z_m` (1.5), `lidar_height_m` (0.30), `lidar_mount_yaw` (0.0), `min_ground_run` (8), `imu_max_age_s` (0.5), `accel_g_tolerance_ms2` (3.0) L89–119 | `navigation.launch.py` L1209–1247 (`lidar_z`, `lidar_yaw`, `imu_yaw` from template L206–213) |
| gps_dock_detection | `dock_pose_x/y/yaw`, `fixed_frame` (odom), `base_frame` (base_footprint), `map_frame` (map), `delta_from_map_odom` (true), `publish_rate_hz` (10), `require_rtk_fixed` (true), `cog_yaw_max_jump_deg` (25), `cog_yaw_ema_alpha` (0.15), `reseed_after_s` (5) L152–204 | `navigation.launch.py` L1266–1280 (dock pose from the merged robot config `rt_rp`, L543–545 — calibration output, so normally the installed yaml) |
| calibrate_imu_yaw | `undock_distance/speed`, `dock_calib_reverse_distance_m` (2.0), `dock_calib_reverse_speed_ms` (0.15), `dock_calib_redock_overshoot_m` (0.30), `dock_calib_rtk_wait_timeout_s` (10), `dock_calib_redock_charge_timeout_s` (30), `dock_calib_cog_min_samples` (8), `dock_calib_cog_std_max_rad` (0.70), `dock_calib_cog_bearing_match_max_rad` (0.35), `dock_calib_min_baseline_displacement_m` (0.5) L264–294 | template L470–502 via `full_system.launch.py` L492–513 |
| wheel_odometry | `wheel_distance` (0.35), `ticks_per_meter` (300), `publish_tf` (false) L85–87 | never launched; yaml shipped but unused |

### TF frames
- navsat: looks up `base_footprint→gps_link` once (lever arm, L260) and `map→base_footprint` per fix (yaw for lever-arm rotation, L309). Falls back to raw antenna position until both exist.
- mag_yaw_publisher: `base_footprint→imu_link` per sample (L207).
- gps_dock_detection: `map→odom` (Δ source when `delta_from_map_odom`, L385) and `odom→base_footprint` (L425). Publishes NO TF.
- wheel_odometry: broadcasts `odom→base_footprint` only if `publish_tf` (L252) — must stay false (Invariant 2; guarded by `mowgli_bringup/test/test_tf_ownership.py`).

## Build, test, run
```bash
# devcontainer
cd ros2 && make build-pkg PKG=mowgli_localization        # scripts/build.sh with PACKAGES=...
cd ros2 && colcon build --packages-select mowgli_localization
cd ros2 && colcon test --packages-select mowgli_localization && colcon test-result --verbose
# single gtest binary after build
ros2/build/mowgli_localization/test_cog_yaw_math
```
Unit tests (all `ament_add_gtest`, `CMakeLists.txt` L254–349; no launch_testing in this package):
- `test_wheel_odometry` — differential-drive integration math (helper mirrors `on_wheel_tick()`).
- `test_cog_yaw_math` — reverse-motion yaw fix (2026-05-27 incident), sweep gate, latch staleness.
- `test_costmap_scan_filter` — links `sensor_msgs` only; the two static helpers are **copied** into the test (L19–23 comment) — keep in sync with `costmap_scan_filter_node.cpp` L217/L250 by hand.
- `test_navsat_projection_utils` — typed `/gps/status` beats NavSatFix status.
- `test_dock_cog_gate` — dock yaw = reversed travel bearing, COG mean is only a cross-check.
- `test_delta_gate` — issue #390 stale-latch re-seed semantics.
- `test_robot_yaml_scalar` — `mowgli_interfaces::robot_yaml_scalar::UpdateDockPose` splice contract (header lives in `mowgli_interfaces`, test lives here).

Cross-package tests that exercise these executables (`ros2/src/mowgli_bringup/CMakeLists.txt` L46–85):
- `test/test_nodes_startup.launch.py` — launches `wheel_odometry_node` + `localization_monitor_node`, expects `/wheel_odom` advertised.
- `test/test_navsat_status_universal.launch.py` — navsat advertises `/gps/absolute_pose` + `/gps/pose_cov`, never publishes `/gps/status`, never subscribes `/diagnostics`.
- `test/test_tf_ownership.py` — `wheel_odometry.yaml` ships `publish_tf: false` and no launch file overrides it.

CI: `.github/workflows/ros2-ci.yml` job `build-and-test` (L128) — `colcon build` L338 + `colcon test` L347 over the whole workspace (ROS_DISTRO kilted).

## Change coupling — "if you change X, also update Y"
- **Datum**: `navsat` and `map_server_node` must receive the same `datum_lat/lon` (`full_system.launch.py` L431–434 comment; Invariant 4). `cog_to_imu` gets its own copy (`navigation.launch.py` L1121–1122) and `navsat_to_absolute_pose_node.cpp` L440 re-implements the equirectangular math instead of using `mowgli_interfaces/wgs84_projection.hpp` — change all three together.
- **Lever arm**: `gps_x/gps_y` in the template (L217–218) feed `cog_to_imu.lever_arm_x/y` (launch L1127–1128) AND the URDF `gps_link` that navsat reads via TF AND `fusion_graph`; changing the antenna mount touches all three.
- **`/gps/status` contract** (`GnssStatus.msg`, `mowgli_interfaces/gnss_status_utils.hpp`): `navsat_projection_utils.hpp` maps it to `AbsolutePose.flags`; `localization_monitor` and `gps_dock_detection` read those flags. `.msg` edits → GUI/firmware codegen per `docs/claude/commands.md`.
- **`AbsolutePose.msg` flags** are consumed by `behavior_tree_node.cpp`, `gps_dock_detection`, `localization_monitor`, the GUI — keep `FLAG_GPS_RTK*` bit values stable.
- **Scan pipeline topics**: `scan_deskew.output_topic` → `costmap_scan_filter.input_topic` (launch L1196/L1211) → `/scan_costmap` in `nav2_params_lidar.yaml:27,95` and `/scan_collision` at `:273`. Renaming any hop needs all four.
- **`lidar_mount_yaw`** is derived at launch as `lidar_yaw − imu_yaw` (`navigation.launch.py` L303); changing either template key changes the ground filter's front/back sign.
- **Dock pose persistence**: `robot_yaml_scalar::UpdateDockPose` is called from `mowgli_map/src/area_manager.cpp:916,1673` and `calibrate_imu_yaw_node.cpp:723`; `test_robot_yaml_scalar.cpp` pins the splice format. Key names `dock_pose_x/y/yaw` are also read by `navigation.launch.py` L543–545 and the GUI (`gui/pkg/api/calibration_status.go:109–134`).
- **`CalibrateDock.action` / `DockCalibrationStatus.msg` / `CalibrateImuYaw.srv`** phase + `RETRY_*` codes are mirrored in `gui/web/src/hooks/useDockCalibration.ts` and `gui/pkg/api/calibration.go` — regenerate Go/TS types after editing.
- **`detected_dock_pose` frames**: `gps_dock_detection.fixed_frame/base_frame` must equal `docking_server.fixed_frame/base_frame` (launch L1273–1274); `use_gps_dock_detection` also flips `simple_charging_dock.use_external_detection_pose` in the same launch (L713).
- **`use_lidar` / `use_magnetometer` / `use_gps_dock_detection`** default from the merged robot config (`navigation.launch.py` L102–141; `use_lidar` comes from the install-only `lidar_enabled` key via `resolve_lidar_enabled`); GUI `paramCatalog.ts` exposes `use_gps_dock_detection`, `declination_deg`, `mag_yaw_variance`, `enable_mag_cal`, `min_horizontal_uT`, `datum_lat`, `lidar_height_m`.
- **`mag_calibration.yaml`** schema (`mag_calibration:` map with `offset_*_uT`, `scale_*`, `magnitude_mean_uT`) is written by `cog_to_imu_node.cpp` L878–893 and `calibrate_imu_yaw_node.cpp` L858–890, read by `mag_yaw_publisher_node.cpp` `load_calibration()` and `gui/pkg/api/calibration_status.go:29,64`.
- **`test_costmap_scan_filter.cpp`** duplicates `filter_scan`/`apply_ground_filter`; any change to the node's helpers must be mirrored in the test copy.

## Pitfalls
- **Topic names**: the monitor publishes `/mowgli/localization/mode(_id)` (`localization_monitor_node.cpp` L85–87); the header comment (`localization_monitor_node.hpp` L37–38) and `ros2/README.md` say `/localization/mode`. Nothing subscribes to either name today.
- **`wheel_odometry_node` is dead code in production** — `hardware_bridge` publishes `/wheel_odom` and `/wheel_ticks` itself (`mowgli_hardware/src/odometry_publisher.cpp:35,40`). The `full_system.launch.py` L442 comment claiming `/wheel_ticks` has no publisher is wrong; the node is still built and started only by `test_nodes_startup.launch.py`. Never flip `publish_tf` (Invariant 2).
- **EKF remnants**: `package.xml` L7–13 ("TFs are published by the stock robot_localization EKF nodes") and L33–41 (Python `.py` scripts, `rclpy`/`numpy` exec deps), `config/wheel_odometry.yaml` L10–12, `wheel_odometry_node.cpp` L40/L191–225, `wheel_odometry_node.hpp` L23/L32, `navsat_to_absolute_pose_node.hpp` L91–122, `full_system.launch.py` L448–450, and `mowgli_robot.yaml` L182 ("consumed by dock_yaw_to_set_pose") all still describe removed components. The code paths are correct; only the prose is stale.
- **`set_datum` gates on `NavSatFix.status >= STATUS_GBAS_FIX`** (L165), not on `/gps/status` like the rest of the node — a backend that leaves NavSatFix status at `STATUS_FIX` while `/gps/status` says RTK-Fixed will refuse the service. It also only updates the in-memory datum; persisting to `mowgli_robot.yaml` is the GUI's job.
- **`/gps/pose_cov` is base_footprint position, `/gps/absolute_pose` too** (both lever-arm-corrected once TF is up, L335–337); until `map→base_footprint` exists both carry the raw antenna position.
- **`/gps/pose_cov` is dropped, not inflated, above `pos_accuracy_reject_threshold_m`** (L353) and skipped entirely for non-RTK fixes (L345) — a robot on standalone GPS gets no position factor at all.
- **`enable_mag_cal` defaults `true` in C++ but `false` in the template** (`cog_to_imu_node.cpp` L187 vs template L177); running the node outside the launch file silently starts the online mag refit and writes `/ros2_ws/maps/mag_calibration.yaml`.
- **`mag_yaw_publisher` launch condition is evaluated once** (`os.path.isfile` at launch parse, `navigation.launch.py` L1156); a calibration produced after boot needs a stack restart even though the node itself would hot-reload (`reload_cal_if_needed()` L123).
- **`/imu/cog_heading` is body heading, not raw course-over-ground** (`dock_cog_gate.hpp` header) — never add π for reverse; `compute_cog_body_yaw()` already handles sign.
- **`cog_to_imu.stationary_seed_rate_hz` is no longer zeroed in sim.** The `navigation.launch.py` L1104–1112 comment and the `cog_stationary_seed_rate_hz` arg description (L199) still claim sim overrides it to 0.0 (issue #200), but `sim_full_system.launch.py` deliberately leaves the 2.0 default (L166–169 comment) now that the node self-gates the latch on `|wheel_omega|` (`latch_republish_max_omega_rps`).
- **Three dock-pose file writers were collapsed to `robot_yaml_scalar::UpdateDockPose`, but two callers remain**: `map_server` (`area_manager.cpp:916,1673`, the canonical `/set_docking_point` path) and the legacy `~/calibrate` dock pre-phase (`calibrate_imu_yaw_node.cpp:723`). `mowgli_behavior/calibration_nodes.cpp` no longer writes (its L42–49). The one-click action routes through map_server (`persist_dock_via_map_server()` L1148) and only after a verified re-dock.
- **No `do_mag_calibration` parameter exists** — the mag figure-8 runs only when `CalibrateImuYaw.Request.mag_only == true` (L1835); the GUI already calls it that way (`calibration.go:193`).
- **`detected_dock_pose` must resolve to `/detected_dock_pose`** (launch L1283–1288); the template comment at `mowgli_robot.yaml` L462 naming `/docking_server/detected_dock_pose` is the old, broken remap (error 904).
- **`gps_dock_detection` never goes silent once it has seen one RTK-Fixed sample** (header L91–104): on Float it republishes the last good odom-frame detection; expect stale-but-consistent targets, not a timeout.
- **`scan_deskew` passes scans through unchanged when IMU is stale** (`on_scan()` L355–366) and **`costmap_scan_filter` disables the ground filter when IMU is stale** — a dead IMU degrades to phantom obstacles, not to blindness. `/scan_collision` is deliberately NOT ground-filtered (L395–404): do not point collision_monitor at `/scan_costmap`.
- **`costmap_scan_filter` blanks the dock ring until it hears `/hardware_bridge/status`** (`is_blank_active()` L437–443) — near obstacles inside `dock_blank_range` are invisible to the costmap before the first status message.
- `localization_monitor.pose_timeout` is declared but never used (`evaluate_mode()` L192–218 only checks GPS); the monitor does not read `fusion_graph` health.
- `calibrate_imu_yaw_node` drives on `/cmd_vel_docking`, never commands the blade (Safety section), and aborts on `active_emergency || latched_emergency`.

## Generated & vendored — do not hand-edit
- Nothing generated or vendored inside `ros2/src/mowgli_localization/`. Message/service/action types it uses are generated from `ros2/src/mowgli_interfaces/{msg,srv,action}` at build time; GUI/firmware mirrors are regenerated with the scripts in `docs/claude/commands.md`.
