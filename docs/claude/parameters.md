# Configuration & Parameter Index

> Every knob on this robot, where its default lives, which node consumes it, and whether the GUI can edit it.
> Index generated 2026-09-03 at f21729e9; regenerate when config files or launch injections change.
> Read this with CLAUDE.md **Invariant 15** (sparse installed config over an in-package template) open — it is the rule this whole file describes.

**The one-sentence model:** defaults live in the in-package template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml`; the *installed* `/ros2_ws/config/mowgli_robot.yaml` is SPARSE and holds only install choices + calibration outputs + genuine overrides; `robot_config_util.load_robot_params()` deep-merges installed OVER template at launch; each launch file then **injects** individual keys into node parameters. **A key that no launch file injects is inert** — the node silently runs its compiled `declare_parameter` default no matter what the yaml says. Those are marked `INERT` below.

## Config files at a glance

| File | Role | Read by | Edited by |
|------|------|---------|-----------|
| `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` (739 L) | **TEMPLATE of every robot-param default** (Invariant 15) | `robot_config_util.load_robot_config` L91 (all launch files) | maintainer (commit) — never the robot |
| `install/config/mowgli/mowgli_robot.yaml` (79 L) | **SPARSE seed** of the installed config: install choices + calibration placeholders | copied to `docker/config/mowgli/` by `install/lib/config.sh` `write_config` L1327 | maintainer (commit); it is only a seed |
| `/ros2_ws/config/mowgli_robot.yaml` (runtime, gitignored) | the live sparse installed config, merged over the template | `robot_config_util.py:31` `DEFAULT_RUNTIME_PATH`; GUI via DB key `system.mower.yamlConfigFile` (`gui/pkg/providers/db.go:52`) | **installer** (`config.sh` L1400–1441), **GUI** (`settings.go` `PostSettingsYAML` L1344), **nodes** (dock pose line-splice, `mowgli_interfaces/robot_yaml_scalar.hpp`) |
| `gui/asserts/mower_config.schema.json` | GUI form definition **and** the GUI's authoritative default source (12 sections, 108 fields) | `settings.go` `getSchema` L1032, `GetSettingsYAMLDefaults` L1318 | maintainer; pinned to the template by `gui/pkg/api/schema_template_parity_test.go` |
| `ros2/src/mowgli_bringup/config/nav2_params_base.yaml` (1193 L) | shared Nav2 params for BOTH LiDAR and no-LiDAR variants | `navigation.launch.py:659` (deep-merge) | maintainer |
| `ros2/src/mowgli_bringup/config/nav2_params_lidar.yaml` (284 L) | LiDAR-only overlay (scan obstacle layers, scan collision_monitor) | `navigation.launch.py:270`, merged L663 | maintainer |
| `ros2/src/mowgli_bringup/config/nav2_params_no_lidar.yaml` (80 L) | GPS-only overlay (static layers, pass-through monitor) | `navigation.launch.py:271`, merged L663 | maintainer |
| `ros2/src/mowgli_bringup/config/hardware_bridge.yaml` (146 L) | serial port/baud/rates, IMU cal count, **dig detector** `dig_*` + repeat-dig escalation `dig_escalate_*` (Invariant 16) | `mowgli.launch.py:185` | maintainer |
| `ros2/src/mowgli_bringup/config/twist_mux.yaml` (53 L) | 5 cmd_vel lanes + priorities; deliberately **no `locks:`** | `mowgli.launch.py:271` | maintainer |
| `ros2/src/mowgli_bringup/config/foxglove_bridge.yaml` (11 L) | Foxglove params + GNSS-internal topic whitelist — **not referenced by any launch file** | nothing | maintainer |
| `ros2/src/fusion_graph/config/fusion_graph.yaml` (480 L) | 76 of the localizer's 133 declared params | `fusion_graph/launch/fusion_graph.launch.py:137` | maintainer |
| `ros2/src/mowgli_map/config/map_server.yaml` (176 L) | grid resolution/size, keepout margins, mow-progress gating, dig keepout | `full_system.launch.py:173` | maintainer (operator keys are injected over it) |
| `ros2/src/mowgli_map/config/obstacle_tracker.yaml` (18 L) | LiDAR cluster→obstacle promotion thresholds | `full_system.launch.py` obstacle_tracker node | maintainer |
| `ros2/src/mowgli_behavior/config/behavior_tree.yaml` (11 L) | `tree_file`, `tick_rate`, legacy `battery_*_pct` **aliases that no longer match the node's param names** | `full_system.launch.py:172` | maintainer |
| `ros2/src/mowgli_localization/config/wheel_odometry.yaml` (18 L) | `wheel_distance`, `ticks_per_meter`, `publish_tf: false` (Invariant 2 guard) | **no launch file** — `wheel_odometry_node` is not launched; the file is only read by `test_tf_ownership.py:81` | maintainer |
| `ros2/src/mowgli_monitoring/config/{diagnostics,mqtt_bridge}.yaml` | freshness/battery/temp thresholds; MQTT broker + prefix | `full_system.launch.py` | maintainer |
| `docker/.env` (gitignored; template `docker/.env.example`) | which **containers** run, image tags, host devices, GNSS sidecar wiring | compose fragments in `install/compose/*.yml` | installer `install/lib/env.sh` `setup_env` L210–374; operator by hand |
| `docker/docker-compose.yaml` (generated at install; gitignored, `.gitignore:33`) | the merged stack | `docker compose` | `install/lib/compose.sh` `build_compose_stack` (L11 `FINAL_COMPOSE_FILE`) |

Operator-facing rule of thumb: **the GUI only ever writes `mowgli_robot.yaml`.** Nav2/fusion_graph/map_server yamls are maintainer-only; the operator reaches a subset of them indirectly through the launch-time injections listed below.

## mowgli_robot.yaml keys

All 158 template keys. `L###` = line in `ros2/src/mowgli_bringup/config/mowgli_robot.yaml`. **GUI** = present in `mower_config.schema.json` (section name), or `no` — a `no` key still shows up in Settings → *Advanced* once it exists in the installed file (`gui/web/src/hooks/useSettingsManager.ts:679` `advancedKeys`). **Life** = `launch` (injected at launch, restart to apply), `dynamic` (also honoured live via `ros2 param set`), `INERT` (never injected — node compiled default wins), `sidecar` (consumed outside ROS2).

### Chassis, wheels, encoder — feed the URDF and the Nav2 footprint

| Key (L) | Default | Consumer · where read | GUI | Life |
|---|---|---|---|---|
| `mower_model` (L18) | `YardForce500` | no ROS consumer; `ros2/scripts/compute_nav2_params.py:293` picks the motor-spec row; installer hardware presets | Hardware | sidecar |
| `chassis_length` (L24) | 0.60 | xacro `mowgli.launch.py:94`; Nav2 footprint `navigation.launch.py:304` | Hardware | launch |
| `chassis_width` (L25) | 0.40 | xacro `mowgli.launch.py:95`; footprint `navigation.launch.py:305`; `map_server.chassis_width` `full_system.launch.py:389`; `coverage_server.robot_width` `navigation.launch.py:930` | Hardware | launch |
| `chassis_height` (L26) | 0.19 | xacro `mowgli.launch.py:96` | Hardware | launch |
| `chassis_mass_kg` (L27) | 8.76 | xacro `mowgli.launch.py:97` (base_link inertial; pinned by `test_urdf_xacro.py`) | Hardware | launch |
| `wheel_radius` | 0.1 | xacro `mowgli.launch.py:99` → `base_z_offset` (base_link height above ground, so every sensor z) + wheel visuals. Nothing else; odometry uses `ticks_per_meter`. Was 0.04475 (xacro default 0.093) until 2026-09-05 | Hardware | launch |
| `wheel_width` (L31) | 0.04 | xacro `mowgli.launch.py:100` | Hardware | launch |
| `wheel_track` (L32) | 0.325 | xacro `mowgli.launch.py:101`; `hardware_bridge.wheel_track` L209 (→ firmware diff-drive IK); turn-geometry check `navigation.launch.py:587`. Fallback single-sourced as `DEFAULT_WHEEL_TRACK_M` (`robot_config_util.py:64`) and MUST equal firmware `board.h` `WHEEL_BASE` | Hardware | launch |
| `wheel_x_offset` (L35) | 0.0 | xacro `mowgli.launch.py:102` | Hardware | launch |
| `chassis_center_x` (L36) | 0.18 | xacro `mowgli.launch.py:98`; footprint `navigation.launch.py:306` | Hardware | launch |
| `ticks_per_meter` (L43) | 399.0 | `hardware_bridge.ticks_per_meter` `mowgli.launch.py:210` (host odom + re-sent to STM32) | Hardware | launch |
| `caster_radius` (L190) | 0.03 | xacro `mowgli.launch.py:103` | Hardware | launch |
| `caster_track` (L191) | 0.36 | xacro `mowgli.launch.py:104` | Hardware | launch |
| `blade_radius` (L194) | 0.09 | xacro `mowgli.launch.py:105` (`blade_link`) | Hardware | launch |
| `tool_width` (L195) | 0.18 | `map_server.tool_width` `full_system.launch.py:426` (mow-progress stamp radius); `coverage_server.operation_width = tool_width − swath_overlap` `navigation.launch.py:924`. Fallback single-sourced as `DEFAULT_TOOL_WIDTH_M` (`robot_config_util.py:48`) — **Invariant 6** | Hardware | launch |

### Runtime limits pushed to the STM32 — all currently INERT

`hardware_bridge_node.cpp:379,386–392` declares each of these, but **no launch file injects them**, so the yaml value never reaches the node. The compiled defaults are identical to the template today, so behaviour is correct — an operator *override* would be silently ignored.

| Key (L) | Default | Declared at | GUI | Life |
|---|---|---|---|---|
| `max_mps` (L49) | 0.5 | `hardware_bridge_node.cpp:379` (`PACKET_ID_LL_SET_KINEMATICS`); also an input to the offline `compute_nav2_params.py:325` | no | INERT |
| `max_charge_voltage` (L57) | 29.4 | `hardware_bridge_node.cpp:386` (`LL_SET_SAFETY_LIMITS`) | no | INERT |
| `max_charge_current` (L58) | 1.2 | `hardware_bridge_node.cpp:387` | no | INERT |
| `one_wheel_lift_emergency_ms` (L59) | 2000 | `hardware_bridge_node.cpp:388` | no | INERT |
| `both_wheels_lift_emergency_ms` (L60) | 1000 | `hardware_bridge_node.cpp:389` | no | INERT |
| `tilt_emergency_ms` (L61) | 500 | `hardware_bridge_node.cpp:390` | no | INERT |
| `stop_button_emergency_ms` (L62) | 100 | `hardware_bridge_node.cpp:391` | no | INERT |
| `play_button_clear_emergency_ms` (L63) | 2000 | `hardware_bridge_node.cpp:392` | no | INERT |

### Drive loops (both close in FIRMWARE — CLAUDE.md preamble, Option C)

| Key (L) | Default | Consumer · where read | GUI | Life |
|---|---|---|---|---|
| `wheel_pid_kp` (L72) | 0.2 | `hardware_bridge` `mowgli.launch.py:214` → STM32 `SET_DRIVE_PID` | Hardware | launch |
| `wheel_pid_ki` (L73) | 0.092 | `mowgli.launch.py:215` | Hardware | launch |
| `wheel_pid_kd` (L74) | 0.01 | `mowgli.launch.py:216` | Hardware | launch |
| `wheel_pid_integral_limit` (L75) | 15.0 | `mowgli.launch.py:217` | Hardware | launch |
| `wheel_pid_pwm_per_mps` (L76) | 282.135 | `mowgli.launch.py:219` (open-loop feedforward) | Hardware | launch |
| `yaw_kp` (L84) | 0.12 | `mowgli.launch.py:247` → firmware gyro yaw-rate loop | no | launch |
| `yaw_ki` (L85) | 0.40 | `mowgli.launch.py:248` | no | launch |
| `yaw_trim_limit_mps` (L86) | 0.15 | `mowgli.launch.py:249` | no | launch |
| `yaw_loop_enabled` (L87) | `true` | `mowgli.launch.py:250` (false = open-diff passthrough) | no | launch |
| `yaw_gyro_sign` (L91) | 1 | `mowgli.launch.py:253` | no | launch |

### Behavior tree (all injected into `behavior_tree_node` by `full_system.launch.py`)

| Key (L) | Default | Consumer · where read | GUI | Life |
|---|---|---|---|---|
| `tick_rate` (L93) | 10.0 | `full_system.launch.py:226` | no | launch |
| `bt_debug_logging` (L164) | `false` | `full_system.launch.py:227` | no | launch |
| `idle_nav2_suspend` (L170) | `false` | `full_system.launch.py:238` (pause Nav2 lifecycle on the dock) | no | launch |
| `area_simplification_tolerance` (L115) | 0.05 | `full_system.launch.py:262` (Douglas–Peucker on `RecordArea`) | no | launch |
| `area_record_rate_hz` (L129) | 10.0 | `full_system.launch.py:265` | no | launch |
| `mowing_enabled` (L308) | `true` | **hardware_bridge only** `mowgli.launch.py:238` (dry-run blade inhibit; guarded by `test_launch_injection.py`) | Mowing | launch |
| `mowing_speed` (L309) | 0.20 | BT `full_system.launch.py:245` (→ `SetNavMode`); `FollowCoveragePath.speed_fast` `navigation.launch.py:755`; also raises FTC's `max_cmd_vel_speed` clamp L764 | Mowing | dynamic (BT sets it per nav mode, `navigation_nodes.cpp:962`) |
| `transit_speed` (L310) | 0.20 | BT `full_system.launch.py:244`; `FollowPath.desired_linear_vel` `navigation.launch.py:745` | Mowing | dynamic (`navigation_nodes.cpp:961`) |
| `undock_distance` (L432) | 1.5 | BT BackUp `full_system.launch.py:232`; dock-calib `:492` | Docking | launch |
| `undock_speed` (L433) | 0.16 | BT BackUp `full_system.launch.py:231`; dock-calib `:493` | Docking | launch |
| `mow_angle_deg` (L338) | -1.0 (auto) | BT `full_system.launch.py:250` → `PlanCoverage` goal | Mowing | launch |

### LocalizationGuard (BT pause/resume on GNSS quality)

Injected `full_system.launch.py:272–293`. The guard keys on `/gps/status` solution quality, **not** the fused covariance (`mowgli_behavior/localization_health.hpp`).

| Key (L) | Default | GUI | Life |
|---|---|---|---|
| `loc_gnss_acc_pause_m` (L148) | 0.30 | no | launch |
| `loc_gnss_acc_resume_m` (L149) | 0.15 | no | launch |
| `loc_gnss_stale_s` (L151) | 5.0 | no | launch |
| `loc_sigma_pause_persist_s` (L154) | 3.0 | no | launch |
| `loc_sigma_resume_persist_s` (L155) | 2.0 | no | launch |
| `loc_sigma_pause_m` (L161) | 5.0 — divergence backstop; must stay above fusion_graph's ~2.35 m pivot/slip ceiling or the 2026-08-20 livelock returns; 0 disables | no | launch |
| `loc_sigma_resume_m` (L162) | 2.0 | no | launch |
| `loc_sigma_backstop_persist_s` (L163) | 10.0 | no | launch |

### IMU / magnetometer calibration

| Key (L) | Default | Consumer · where read | GUI | Life |
|---|---|---|---|---|
| `imu_cal_samples` (L172) | 200 | `hardware_bridge` `mowgli.launch.py:222`. **This injection comes AFTER `hardware_bridge.yaml` (which sets 1000 at L21) in the `parameters=[…]` list, so 200 wins** | no | launch |
| `imu_cal_persist_path` (L173) | `/ros2_ws/maps/imu_calibration.txt` | `mowgli.launch.py:223` | no | launch |
| `imu_cal_auto_rest_sec` (L174) | 15.0 | `mowgli.launch.py:225` | no | launch |
| `imu_cal_periodic_recal_sec` (L175) | 600.0 | `mowgli.launch.py:227` | no | launch |
| `enable_mag_cal` (L177) | `false` | `mag_yaw_publisher` `navigation.launch.py:576` | no | launch |
| `mag_calibration_path` (L178) | `/ros2_ws/maps/mag_calibration.yaml` | `navigation.launch.py:577` | no | launch |
| `declination_deg` (L179) | 1.5 | `navigation.launch.py:578` | no | launch |
| `min_horizontal_uT` (L180) | 5.0 | `navigation.launch.py:579` | no | launch |
| `mag_yaw_variance` (L181) | 0.0027 | `navigation.launch.py:580` | no | launch |
| `dock_pose_yaw_sigma_rad` (L183) | 0.035 | `fusion_graph_node` via `navigation.launch.py:574` → `fusion_graph.launch.py:154` | no | launch |

### Sensor extrinsics (base_link → sensor static TFs)

All feed the xacro in `mowgli.launch.py:108–120`; `lidar_z`/`lidar_yaw`/`imu_yaw` are additionally read by `navigation.launch.py:302–303`. `gps_x`/`gps_y` become the fusion_graph antenna lever arm via `fusion_graph.launch.py` reading the yaml **itself** (L132–133 → `lever_arm_x/y` L146–147 — `navigation.launch.py` does NOT forward them); `navigation.launch.py:550–551` reads the same two keys for `cog_to_imu.lever_arm_x/y` (L1127–1128).

| Key (L) | Default | GUI | Life |
|---|---|---|---|
| `lidar_x` (L204) / `lidar_y` (L205) / `lidar_z` (L206) / `lidar_yaw` (L207) | 0.0 / 0.024 / 0.30 / 3.1408 | Sensor Mounting | launch |
| `imu_x` / `imu_y` / `imu_z` / `imu_yaw` | 0.18 / -0.195 / 0.095 / 0.0 | Sensor Mounting | launch |
| `gps_x` (L217) / `gps_y` (L218) / `gps_z` (L219) | 0.3 / 0.0 / 0.20 | Sensor Mounting | launch |
| `imu_roll`, `imu_pitch` — **not in the template** | 0.0 (hardcoded `mowgli.launch.py:115–116`, allow-listed in `schema_template_parity_test.go:41`) | Sensor Mounting | launch |

### Localizer toggles → launch args → `fusion_graph_node`

| Key (L) | Default | Consumer · where read | GUI | Life |
|---|---|---|---|---|
| `use_magnetometer` (L238) | `false` | launch-arg default `navigation.launch.py:133` → `fusion_graph.launch.py:148` | no | launch |
| `use_scan_matching` (L251) | `true` | `navigation.launch.py:135`; **ANDed with `use_lidar`** by `lidar_gated()` L237 | no | launch |
| `use_loop_closure` (L264) | `true` | `navigation.launch.py:137`; ANDed with `use_lidar` **and** force-off on first boot when `/ros2_ws/maps/fusion_graph.graph` is absent (L150–154) | no | launch |
| `use_lidar_map_anchor` | `false` | `navigation.launch.py` → `fusion_graph.launch.py` → node; ANDed with `use_lidar`. Scan-to-map particle-filter anchor for RTK-Float (grid on `/fusion_graph/lidar_map`, XY-only factors, per-estimate validation — `codemaps/fusion_graph.md`) | no | launch |
| `lidar_anchor_shadow_mode` | `false` | same path; with the anchor on, ALSO run/score/publish the filter under RTK-Fixed, never apply — the field measurement of anchor vs RTK (`/fusion_graph/lidar_anchor_candidate`, diagnostics `lidar_anchor_*`) | no | launch |
| `use_gps_dock_detection` (L467) | `true` | `navigation.launch.py:141` → launches `gps_dock_detection_node` + `simple_charging_dock.use_external_detection_pose` | no | launch |

### GNSS / NTRIP — consumed by the **GPS sidecar and the GUI**, not by ROS2

`sensors/gps/start_gps.sh` parses the installed yaml directly (resolver order: YAML → env → built-in default); `gui/pkg/api/gnss*.go` reads the same keys. No ROS2 node declares any of them.

| Key (L) | Default | GUI | Life |
|---|---|---|---|
| `gnss_receiver_family` (L269) | `auto` | GPS / Positioning | sidecar |
| `gnss_serial_device` (L270) | `/dev/ttyAMA4` | GPS / Positioning | sidecar |
| `gnss_serial_baud` (L271) | 921600 | GPS / Positioning | sidecar |
| `ntrip_enabled` (L278) | `false` | GPS / Positioning | sidecar |
| `ntrip_host` (L279) / `ntrip_port` (L280) / `ntrip_user` (L281) / `ntrip_password` (L282) / `ntrip_mountpoint` (L283) | `""` / 2101 / `""` / `""` / `""` | no (GUI has a dedicated NTRIP page) | sidecar |
| `gps_wait_after_undock_sec` (L274) | 10.0 | GPS / Positioning | **INERT** — no ROS consumer (only `check_config_drift.py:97` and the OpenMower import map) |
| `gps_timeout_sec` (L275) | 5.0 | GPS / Positioning | **INERT** — same |

### Datum + dock pose (map anchor; Invariants 4 and 6)

| Key (L) | Default | Consumer · where read | GUI | Life |
|---|---|---|---|---|
| `datum_lat` (L272) / `datum_lon` (L273) | 0.0 / 0.0 (= unset) | `navsat_to_absolute_pose` + `map_server` `full_system.launch.py:363–364,433–434`; `cog_to_imu` `navigation.launch.py:548–549` → `:1121–1122`; `fusion_graph` reads the yaml itself, `fusion_graph.launch.py:130–131` → `:144–145` | GPS / Positioning | launch (a change re-projects `areas.dat` + dock pose at load, issue #216) |
| `dock_pose_x` (L421) / `dock_pose_y` (L422) / `dock_pose_yaw` (L431) | 0.0 | `hardware_bridge` `mowgli.launch.py:200–202`; `map_server` `full_system.launch.py:380–382`; `docking_server.home_dock.pose` (with `dock_approach_overshoot` applied) `navigation.launch.py:682–686`; `fusion_graph` `fusion_graph.launch.py:151–153` | no | launch |

**Writers of `dock_pose_*` back into the installed yaml** (line-splice, comments preserved — `mowgli_interfaces/robot_yaml_scalar.hpp` `UpdateDockPose` L122): `calibrate_imu_yaw_node.cpp:723` (dock pre-phase) and `map_server`'s `area_manager.cpp:916` (`/set_docking_point`, the one-click dock calibration) + `:1673` (datum migration). `calibration_nodes.cpp` **no longer writes** (its `persist_dock_pose_*` helpers were removed — `calibration_nodes.cpp:48`), so CLAUDE.md Invariant 6's "third writer" is stale: there are TWO source files, three call sites.

### Battery thresholds (BT, `full_system.launch.py:342–353`)

| Key (L) | Default | GUI | Life |
|---|---|---|---|
| `battery_full_voltage` (L288) | 28.0 | Battery | launch |
| `battery_empty_voltage` (L289) | 24.0 | Battery | launch |
| `battery_critical_voltage` (L290) | 23.0 | Battery | launch |
| `battery_full_percent` (L291) | 95.0 | Battery | launch |
| `battery_low_percent` (L292) | 20.0 | Battery | launch |
| `battery_critical_percent` (L293) | 10.0 | Battery | launch |
| `battery_critical_recovery_percent` (L294) | 30.0 (hysteresis out of critical; node clamps it above `battery_critical_percent`) | Battery | launch |

> `mowgli_behavior/config/behavior_tree.yaml` still carries `battery_low_pct` / `battery_critical_pct` — **aliases that do not match the node's parameter names**; the injections above are what actually reach the node.

### Coverage geometry (→ `coverage_server`, `navigation.launch.py:920–948`)

| Key (L) | Default | Becomes | GUI | Life |
|---|---|---|---|---|
| `headland_width` | 0.18 | `coverage_server.default_headland_width` L931 — used ONLY for the AUTO ring count `ceil(headland_width / operation_width)`, i.e. only when `num_headland_passes == 0`; inert at the shipped `2` | Mowing | dynamic (read per plan) |
| `num_headland_passes` (L374) | 2 | `coverage_server.num_headland_passes` L932 — three-way sentinel: `<0` none, `0` auto, `>0` exact (injected **unclamped**, pinned by `test_launch_injection.py`) | Mowing | dynamic |
| `mow_direction` (L379) | 0 | `coverage_server.ring_direction` L934 | no | dynamic |
| `chassis_safety_inset` (L396) | 0.2 | `coverage_server.chassis_safety_inset` L935; mirrored to `map_server` `full_system.launch.py:396` | Mowing | dynamic |
| `swath_overlap` (L405) | 0.02 | subtracted: `operation_width = max(0.05, tool_width − swath_overlap)` L924 | no | dynamic |
| `min_turning_radius` (L416) | 0.15 | clamped to [0.10, 0.50] → `coverage_server.min_turning_radius` L944; also drives the `check_turn_geometry` warning (issue #499: 0.15 is **below** the 0.1625 m half-track) | Mowing | dynamic |
| `connector_turn_radius` — **not in the template**; launch fallback 0.18 (`navigation.launch.py:403`), node default `coverage_server.cpp:94` | 0.18 | clamped ≥ `min_turning_radius`, ≤ 0.50 → `coverage_server.connector_turn_radius` L948 | no | dynamic |
| `turn_speed_ratio` (L333) | 0.8 | `FollowCoveragePath.speed_slow = clamp(mowing_speed × ratio, min_speed_mps, mowing_speed)` via `derive_turn_speed` (`robot_config_util.py:268`), injected L781 | no | launch |

### Docking (→ `docking_server` / `simple_charging_dock`, `navigation.launch.py:665–739`)

| Key (L) | Default | Becomes | GUI | Life |
|---|---|---|---|---|
| `dock_approach_distance` (L440) | 1.5 | staging distance | Docking | launch |
| `dock_approach_overshoot` (L449) | 0.05 | forward shift of `home_dock.pose` L682–686 | Docking | launch |
| `dock_max_retries` (L450) | 3 | `docking_server.max_retries` L669 | Docking | launch |
| `dock_use_charger_detection` (L451) | `true` | `simple_charging_dock.use_battery_status` L697 | Docking | launch |
| `dock_charging_threshold` (L455) | 0.3 | `simple_charging_dock.charging_threshold` L693 | Docking | launch |

### One-click dock calibration (→ `calibrate_imu_yaw_node`, `full_system.launch.py:495–514`)

| Key (L) | Default | GUI | Life |
|---|---|---|---|
| `dock_calib_reverse_distance_m` (L476) | 2.0 | no | launch |
| `dock_calib_reverse_speed_ms` (L477) | 0.15 | no | launch |
| `dock_calib_redock_overshoot_m` (L480) | 0.30 | no | launch |
| `dock_calib_rtk_wait_timeout_s` (L483) | 10.0 | no | launch |
| `dock_calib_redock_charge_timeout_s` (L484) | 30.0 | no | launch |
| `dock_calib_cog_min_samples` (L490) | 8 | no | launch |
| `dock_calib_cog_std_max_rad` (L496) | 0.70 | no | launch |
| `dock_calib_cog_bearing_match_max_rad` (L501) | 0.35 | no | launch |
| `dock_calib_min_baseline_displacement_m` (L502) | 0.5 | no | launch |

### Rain

| Key (L) | Default | Consumer · where read | GUI | Life |
|---|---|---|---|---|
| `rain_mode` (L508) | 2 (0=ignore, 1=dock, 2=dock_until_dry, 3=pause_auto) | `behavior_tree_node.cpp:884` `declare_parameter` — **not injected by any launch file** | Rain | INERT |
| `rain_delay_minutes` (L509) | 30.0 | `behavior_tree_node.cpp:877` (node default 30.0, same) | Rain | INERT |
| `rain_debounce_sec` (L510) | 10.0 | `behavior_tree_node.cpp:891` — **node default is 0.0**, so the running robot has no debounce | Rain | INERT (and divergent) |

### Start-pose escape (SAFETY-CRITICAL bounded nudge, `full_system.launch.py:313–341`)

| Key (L) | Default | GUI | Life |
|---|---|---|---|
| `start_blocked_escape_enabled` (L546) | `true` | no | launch |
| `start_blocked_escape_speed` (L547) | 0.10 | no | launch |
| `start_blocked_escape_distance` (L548) | 0.40 (hard ceiling 0.60 in code) | no | launch |
| `start_blocked_escape_timeout_s` (L549) | 6.0 | no | launch |
| `start_blocked_escape_min_signal_speed` (L551) | 0.03 | no | launch |
| `start_blocked_escape_signal_max_age_s` (L558) | 90.0 | no | launch |

### Obstacles (GUI → Settings → Obstacles; injected `navigation.launch.py:811–868`, clamps shown)

| Key (L) | Default | Becomes · clamp | GUI | Life |
|---|---|---|---|---|
| `max_obstacle_avoidance_distance` (L569) | 1.0 | `FTC.max_lateral_deviation` = clamp(0.5, 10.0) L811; `map_server.bypass_max_length` `full_system.launch.py:400` | Obstacles | launch |
| `obstacle_inflation_radius` (L598) | 0.58 | **local** costmap `inflation_layer.inflation_radius` = clamp(0.58, 1.50) L859 (global stays 0.20) | Obstacles | launch |
| `obstacle_detection_range_m` (L611) | 2.0 | `FTC.obstacle_lookahead` = max(4, clamp(0.2, 5.0)/0.05 poses) L823 | Obstacles | launch |
| `obstacle_clearance_margin` (L627) | 0.2 | `FTC.obstacle_clearance_margin` = clamp(0.0, 0.50) L831 | Obstacles | launch |
| `obstacle_wait_timeout_s` (L632) | 2.5 | `FTC.obstacle_wait_timeout_s` = clamp(0.5, 60.0) L838 | Obstacles | launch |
| `obstacle_reverse_enabled` (L651) | `true` | `FTC.obstacle_reverse_enabled` L844 | no | dynamic (FTC param callback, `ftc_controller.cpp:260`) |
| `obstacle_reverse_max_dist_m` (L654) | 0.30 | clamp(0.0, 1.0) L845 | no | dynamic |
| `obstacle_reverse_speed_mps` (L657) | 0.15 | clamp(0.0, 0.30) L847 | no | dynamic |
| `obstacle_margin` (L665) | 0.2 | `coverage_server.obstacle_margin` = clamp(0.0, 1.0) L940 **and** `map_server.obstacle_margin` `full_system.launch.py:405` (planner + keepout stay consistent) | Obstacles | launch |
| `obstacle_slowdown_ratio` (L669) | 0.7 | `collision_monitor.PolygonSlow.slowdown_ratio` = clamp(0.05, 1.0) L866 — **only written when the merged doc has `PolygonSlow`**, i.e. the LiDAR variant | Obstacles | launch |

### Nav2 goal tolerances / progress (`navigation.launch.py:870–912`)

| Key (L) | Default | Becomes | GUI | Life |
|---|---|---|---|---|
| `xy_goal_tolerance` (L685) | 0.1 | `stopped_goal_checker.xy_goal_tolerance` L878 | Navigation | launch |
| `yaw_goal_tolerance` (L686) | 0.1 | `stopped_goal_checker.yaw_goal_tolerance` L879 | Navigation | launch |
| `coverage_xy_tolerance` (L687) | 0.50 | `coverage_goal_checker.xy_goal_tolerance` L905, **floored at FTC's `max_goal_distance_error` (0.50)** with a printed WARN — a tighter gate never completes an area and re-mows it | Navigation | launch |
| `progress_timeout_sec` (L703) | 30.0 | `progress_checker.movement_time_allowance` L910 | Navigation | launch |

### Status LED ring (`mowgli_leds`, `full_system.launch.py:668–707`)

`led_enabled` (L721, default `false`) is pre-read at `full_system.launch.py:100` to become the `led_enabled` **launch arg**, which gates whether the node is spawned at all.

| Key (L) | Default | GUI | Life |
|---|---|---|---|
| `led_enabled` (L721) | `false` | Status LEDs | launch (gates the node) |
| `led_count` (L722) | 16 | Status LEDs | launch |
| `led_spi_device` (L723) | `/dev/spidev4.1` | Status LEDs | launch |
| `led_brightness` (L724) | 0.6 | Status LEDs | launch |
| `led_refresh_hz` (L728) | 20.0 | Status LEDs | launch |
| `led_keepalive_s` (L731) | 2.0 | Status LEDs | launch |
| `led_status_timeout_s` (L732) | 5.0 | Status LEDs | launch |
| `led_device_retry_s` (L733) | 30.0 | Status LEDs | launch |
| `led_low_battery_percent` (L734) | 20.0 | Status LEDs | launch |
| `led_charge_full_percent` (L735) | 99.0 | Status LEDs | launch |
| `led_idle_scale` (L736) | 0.10 | Status LEDs | launch |
| `led_spi_speed_hz` (L737) | 2400000 (3 SPI bits per WS2812 bit — do not retune) | Status LEDs | launch |

### Read by a launch file but absent from the template

These fall back to a literal hardcoded in the launch file. Each is allow-listed in `gui/pkg/api/schema_template_parity_test.go:26–66` or has no schema entry at all.

| Key | Launch fallback | Consumer |
|---|---|---|
| `lidar_enabled` | `DEFAULT_LIDAR_ENABLED = False` (`robot_config_util.py:172`) | install-decided; absence is meaningful → loud startup warning (`warn_lidar_key_absent` L224) |
| `connector_turn_radius` | 0.18 (`navigation.launch.py:403`) | `coverage_server` |
| `fusion_graph_node_period_s` | 0.04 (`navigation.launch.py:139`) | `fusion_graph_node.node_period_s` (overrides `fusion_graph.yaml:23`'s 0.02) |
| `dock_body_length_m` / `dock_body_width_m` | 0.80 / 0.55 (`full_system.launch.py:383–384`) | `map_server` dock polygon |
| `lethal_outside_areas` | `true` (`full_system.launch.py:416`) | `map_server` (also a static default in `map_server.yaml:96`) |
| `enforce_boundary_margin_m` | 0.40 (`full_system.launch.py:418`) | `map_server` (static default `map_server.yaml:112`) |
| `lift_recovery_mode` / `lift_blade_resume_delay_sec` | `false` / 1.0 (`mowgli.launch.py:230–232`) | `hardware_bridge` — GUI section *Safety* |
| `imu_roll` / `imu_pitch` | 0.0 (`mowgli.launch.py:115–116`) | xacro |

## Keys that live only in the sparse installed file

`install/config/mowgli/mowgli_robot.yaml` is the committed **seed** of the runtime file. Two buckets, and the reason each stays out of the template:

**Bucket A — install-time choices.** The installer or onboarding wizard picks them; there is no sensible versioned default a maintainer could bump for everyone.

- `mower_model`, `lidar_enabled` (L28–29). `lidar_enabled` is the special case: it is **deliberately absent from the template**, so its *presence* is what proves an explicit operator choice was made. Absent ⇒ `DEFAULT_LIDAR_ENABLED = False` plus a multi-line startup warning naming the file and key (`robot_config_util.py:144–214`). The value must equal the GUI schema default (`false`), or the backend's sparse-prune would delete every "turn LiDAR on" write and the toggle would be inert in the ON direction forever.
- GNSS transport: `gnss_receiver_family`, `gnss_serial_device`, `gnss_serial_baud` (L32–34) and receiver-profile auto-apply `gnss_config_apply_enabled`, `gnss_config_profile`, `gnss_signal_profile` (L43–45, consumed by `sensors/gps/start_gps.sh:304,320`).
- Datum `datum_lat` / `datum_lon` (L49–50) — 0/0 means "not set" and disables datum migration entirely.
- NTRIP `ntrip_enabled`, `ntrip_host`, `ntrip_port`, `ntrip_user`, `ntrip_password`, `ntrip_mountpoint` (L54–59) — **credentials; never commit real values.**

**Bucket B — per-robot calibration outputs**, written back by nodes/GUI, meaningless as a shared default: `dock_pose_x/y/yaw` (L64–66), `ticks_per_meter` (L70), `imu_yaw` (L74), `enable_mag_cal` + `declination_deg` (L78–79).

The installer additionally patches keys that appear in **neither** the template nor the seed: `gnss_transport`, `gnss_frame_id`, `gnss_ntrip_gga_enabled`, `gnss_ntrip_gga_interval_s`, plus `use_scan_matching`/`use_loop_closure` slaved to the LiDAR choice (`install/lib/config.sh:1400–1441`).

**Why sparse matters.** Deleting a key from the installed file is exactly the GUI's "reset to default" — the deep-merge then falls through to the template. `PostSettingsYAML` also prunes any saved key whose value equals its schema default (`settings.go` `sparsifyFlat` L388), so the file stays sparse on its own. Padding it with defaults breaks both that and the propagate-a-new-default-to-every-robot property. `ros2/scripts/check_config_drift.py` is the CI guard: it fails on a structural field present in both files with different values, on an installed key with no template default, and on any installed key whose value merely *equals* the template default (issue #381).

## Launch arguments

`full_system.launch.py` is the container entry point (`install/compose/docker-compose.base.yml:42–44` runs it, passing only `enable_foxglove`). CLI/compose values always beat the config-derived defaults, because `DeclareLaunchArgument` applies its default only when nothing is passed.

### `full_system.launch.py`

| Arg | Default | Effect |
|---|---|---|
| `use_sim_time` | `false` | Gazebo/Webots clock |
| `serial_port` | `/dev/mowgli` | hardware_bridge serial device (overrides `hardware_bridge.yaml`) |
| `enable_mqtt` | `false` | launch the MQTT bridge |
| `enable_foxglove` | `true` | launch `foxglove_bridge` |
| `foxglove_port` | `8765` | Foxglove WebSocket port |
| `use_lidar` (L135) | `mowgli_robot.yaml:lidar_enabled`, else `false` + warning | gates LiDAR nodes, the Nav2 overlay choice, and the fusion_graph scan factors. **`LIDAR_ENABLED` in `.env` is NOT consulted** (removed 2026-08-31) |
| `use_obstacle_tracker` (L141) | `true` | persistent `/scan` cluster → `mow_progress` obstacle promotion; also gated on `use_lidar` |
| `led_enabled` (L147) | `mowgli_robot.yaml:led_enabled` (`false`) | spawn the WS2812 ring node |

There is **no `use_fusion_graph` arg** — it was removed with the dual EKF (Invariant 1). Do not write one from the GUI.

### `navigation.launch.py`

| Arg | Default | Effect |
|---|---|---|
| `use_sim_time` | `false` | — |
| `use_lidar` (L166) | as above | picks `nav2_params_lidar.yaml` vs `nav2_params_no_lidar.yaml` (L956–962) and force-ANDs the two scan flags |
| `use_magnetometer` (L172) | yaml `use_magnetometer` (`false`) | mag yaw unary factor |
| `use_scan_matching` (L178) | yaml (`true`) **AND** `use_lidar` | scan between-factors |
| `use_loop_closure` (L184) | yaml (`true`) AND `use_lidar` AND a persisted `/ros2_ws/maps/fusion_graph.graph` exists (L150–154) | loop-closure search |
| `use_gps_dock_detection` (L190) | yaml (`true`) | `gps_dock_detection_node` + external detection pose |
| `cog_stationary_seed_rate_hz` (L196) | `2.0` | `cog_to_imu` stationary anchor. The code comment at `navigation.launch.py:1111–1112` claims sim overrides it to `0.0` — **stale**: no launch file in the repo passes this arg |
| `fusion_graph_tf_lead_s` (L214) | `0.05` | forward-stamp on BOTH `map→odom` and `odom→base_footprint`; sim passes `0.1` (`sim_full_system.launch.py:182`) |
| `fusion_graph_node_period_s` (L219) | yaml `fusion_graph_node_period_s`, else `0.04` (25 Hz) | factor-graph cadence; sim passes `0.02` (`sim_full_system.launch.py:183`) |

`mowgli.launch.py` takes `use_sim_time` (L62) and `serial_port` (L68). `fusion_graph.launch.py` takes `use_sim_time`, `use_magnetometer`, `use_scan_matching`, `use_loop_closure` (all default `false`), `primary_mode` (`true`), `tf_publish_lead_s` (`0.0`), `node_period_s` (`0.04`) — L87–118.

### The remaining launch files in `mowgli_bringup/launch/`

| File | Args | Notes |
|---|---|---|
| `sim_full_system.launch.py` | `world` (`mowgli_garden.wbt`), `use_rviz` (`false`), `headless` (`true`, **deprecated Gazebo-era, ignored**), `use_lidar` (`true` — sim overrides the yaml), `mode` (`realtime`; `fast` starves the 20 Hz controller loop) | Webots entry point; injects the sim test polygon as `map_server` `area_*` overrides (L220–223) and `fusion_graph_tf_lead_s:=0.1` / `fusion_graph_node_period_s:=0.02` (L182–183) |
| `foxglove_bridge.launch.py` | `port` (`8765`), `send_buffer_limit` (`1000000` — 1 MB, was 10 MB) | inlines its own params; `config/foxglove_bridge.yaml` is dead |
| `nav2_navigation_launch.py` | vendored Nav2: `namespace`, `use_sim_time`, `params_file`, `autostart`, `use_composition`, `container_name`, `use_respawn`, `log_level` | included by `navigation.launch.py:1013–1024`, which passes only `use_sim_time`, `params_file` (the merged temp file) and `use_composition:=False`; the rest run on their declared defaults |

## Nav2 params

### How base + overlay merge

1. `navigation.launch.py:659–663` loads `nav2_params_base.yaml` and ONE overlay, and `deep_merge`s them with the **same** helper the robot config uses (`robot_config_util.deep_merge` L67 — one canonical implementation; `compute_nav2_params.py` and `test_nav2_params.py` import it too).
2. `_inject_dock_pose_and_speeds()` (L646–955) then writes the `mowgli_robot.yaml`-derived values into the merged dict (lists and nested keys that `RewrittenYaml` cannot express) and dumps it to a temp file.
3. Both variants are pre-built (L956–957); a `PythonExpression` on `use_lidar` picks one at runtime (L958–962); `RewrittenYaml` (L976) applies the remaining scalar substitutions.

**Edit shared params in the base. Edit genuine LiDAR/no-LiDAR differences in the overlay.** `test_base_has_no_costmap_layers_or_polygons` (`test_nav2_params.py:605`) enforces that split, and `test_overlays_select_disjoint_costmap_layers` (L624) pins the variants in lockstep.

### What is genuinely variant

| Difference | LiDAR overlay | No-LiDAR overlay |
|---|---|---|
| Global costmap `plugins` | `[obstacle_layer, keepout_filter, inflation_layer]` (L18) | `[static_layer, keepout_filter, inflation_layer]` (L33) fed by `/no_lidar_static_map` |
| Local costmap `plugins` | `[obstacle_layer, inflation_layer]` (L75) | `[static_layer, inflation_layer]` (L46) |
| `FollowPath.use_collision_detection` | `true` (L13) | `false` (L11) |
| `FollowCoveragePath` obstacle flags | from base (`check_obstacles: true`, `enable_obstacle_deviation: true`) | `check_obstacles: false`, `enable_obstacle_deviation: false` (L20–21) |
| `collision_monitor.polygons` | `[FootprintApproach, PolygonStopNarrow, PolygonSlow]` (L146) — all `enabled: true`; the static `PolygonStop` is kept **disabled** at L238–245 for the record | `[PolygonStop, FootprintApproach]` both `enabled: false`, `scan` source `enabled: false` (L56–80) — pass-through |
| `planner_server.costmap_update_timeout` | unset in base and overlay → Nav2 default 1.0 | `5.0` (L28) |

### Where the blocks live (`nav2_params_base.yaml`)

| Block | Lines | Notes |
|---|---|---|
| `bt_navigator` | 19–52 | `odom_topic: /odometry/filtered` L24 |
| `controller_server` (root) | 64–83 | **`odom_topic: /wheel_odom` L77** — never leave this unset, Nav2 would default to `"odom"` which nothing publishes; `controller_frequency: 10.0` L83 |
| `progress_checker` | 115–129 | `PoseProgressChecker`, `required_movement_radius: 0.15` L121, `movement_time_allowance: 30.0` L129 (injected from `progress_timeout_sec`) |
| `stopped_goal_checker` | 131–149 | transit goal gate |
| `coverage_goal_checker` | 170–189 | `mowgli_nav2_plugins/PathProgressGoalChecker` L171, `plan_topic: /controller_server/FollowCoveragePath/global_plan` L189 — **never `StoppedGoalChecker`** |
| `FollowPath` (transit) | 226–320 | RotationShim L227 wrapping RPP L228; `desired_linear_vel: 0.30` L276 (overwritten by `transit_speed`) |
| `FollowCoveragePath` (coverage) | 338–546 | `mowgli_nav2_plugins/FTCController` L339; `speed_fast` L346 / `speed_slow` L357 / `min_speed_mps` L360 (all launch-overwritten); `max_cmd_vel_ang: 0.8` L404; `max_goal_distance_error: 0.50` L410; `forward_only: true` L420; `check_obstacles` L425, `obstacle_lookahead` L431, `obstacle_body_half_width` L477, `ignore_obstacles_outside_zone` L508, `enable_obstacle_deviation` L509, `max_lateral_deviation` L515, reverse-escape trio L544–546 |
| `planner_server` | 551–590 | Smac |
| `smoother_server` / `behavior_server` / `waypoint_follower` | 591 / 604 / 655 | BackUp lives in `behavior_server` (undock, Invariant 10) |
| `global_costmap` | 674–789 | 70×70 m rolling L741–742, `resolution: 0.08` L715, `inflation_radius: 0.20` L780, **`keepout_filter` enabled** L782–785 |
| `local_costmap` | 790–871 | `global_frame: odom`, 10 Hz; its `inflation_layer.inflation_radius` is the one `obstacle_inflation_radius` overwrites |
| `map_server` / `map_saver` | 872 / 877 | — |
| `docking_server` | 888–1090 | `controller.odom_topic: /wheel_odom` L907, `controller_frequency: 20.0` L912 |
| `collision_monitor` (shared I/O only) | 1095–1143 | `source_timeout: 1.5` L1116, `cmd_vel_in_topic` L1101 / `cmd_vel_out_topic` L1104; polygons live in the overlays |
| `coverage_server` | 1150–1182 | static values are node defaults for isolated runs — **every geometry knob is injected at launch** |
| `lifecycle_manager_navigation` | 1188–1193 | `bond_timeout: 10.0` |

`test_nav2_params.py` (934 L, 43 tests) statically pins the merged result: plugin identities, the coverage-tolerance-vs-FTC-park-distance floor, the FTC clamp/speed injections, obstacle-knob injections, `odom_topic` not being the Nav2 default, both collision_monitor variants, and the turn-geometry helpers.

## fusion_graph params

`fusion_graph.yaml` carries **76** keys; the node declares **133** (`fusion_graph_node_setup_params.cpp` and siblings) — the other 57 run on their in-code defaults and have no yaml line. Groups (line = `ros2/src/fusion_graph/config/fusion_graph.yaml`):

| Concern | Lines | Representative keys |
|---|---|---|
| Graph cadence | 8–24 | `node_period_s` 0.02 (**overridden to 0.04 by `navigation.launch.py:1090`**) |
| Odom re-base | 25–39 | `odom_rebase_dist_m` 6.0 |
| Wheel between-factor noise | 40–59 | `wheel_sigma_x_per_sqrt_m` 0.05, `wheel_sigma_y_per_sqrt_m` 0.005 (non-holonomic σ_y ≪ σ_x), `wheel_creep_speed_mps`, `wheel_sigma_theta` |
| Gyro between-factor | 60–98 | `gyro_sigma_theta` 0.005 |
| Pivot downweight | 99–117 | `pivot_gate_dtheta_rad` 0.012, `pivot_wheel_sigma_x` 0.5 |
| Stationary gate | 118–126 (its `stationary_thresh_xy_m` / `stationary_thresh_theta` / `stationary_sigma_theta` sit at L79–97, inside the gyro block) | `stationary_gyro_thresh_rad_per_s` |
| **Slip veto** (rotational only — see Invariant 16) | 127–161 | `slip_residual_thresh_rad`, `slip_gyro_max_rad`, `slip_wheel_min_rad`, `slip_window_s` 0.5 (issue #516; `0` = old per-node gate) |
| Graph size | 162–184 | `max_graph_nodes` 6000 |
| Loop closure | 185–235 | `lc_min_age_s` 30, `lc_skip_when_rtk_fixed` true, `lc_min_travel_m` 1.0, `lc_min_interval_s` 2.0, `lc_gps_sigma_ratio` 1.0 (issue #513) |
| Gyro bias | 236–257 | `gyro_bias_estimation_enabled`, `gyro_bias_ema_tau_s` 30, `use_imu_preint` false |
| Adaptive process noise | 258–269 | `adaptive_noise_enabled_gain` 10.0 |
| **RTK wrong-fix gate** | 270–295 | `rtk_wrongfix_max_jump_m` 0.05 — bounded per-interval comparison; do NOT replace with an unbounded accumulator (CLAUDE.md *What NOT to Do*) |
| ICP guard rails | 296–307 | `icp_max_rmse_m`, `icp_max_delta_xy_m`, `icp_max_divergence_*` |
| GPS noise floor / prior | 308–318 | `gps_sigma_floor` 0.003, `prior_sigma_xy` 0.05 |
| Lever arm + datum | 319–328 | `lever_arm_x/y`, `datum_lat/lon` — **all four injected from `mowgli_robot.yaml` by `fusion_graph.launch.py:144–147`** |
| Frames | 329–333 | `map_frame` / `odom_frame` / `base_frame` (Invariant 2) |
| TF publish | 334–373 | `fast_pose_publish_rate_hz`, `tf_publish_lead_s` 0.05 (launch-overridden), `tf_broadcast_rate_hz` 20 |
| Docking | 374–415 | `docking_active_timeout_s`, `gate_cog_during_docking`, `dock_reanchor_sigma_xy_m` 0.03, `dock_prior_max_gps_disagreement_m` 0.50, `dock_prior_max_gps_sigma_m` 0.05 (issue #512) |

Declared **without** a yaml line (code defaults only, tune via `ros2 param set` or add a line): `anchor_*` (5), `auto_save_enabled`, `autoload_graph`, `cog_*` (10), `cov_update_every_n`, `dr_slip_*`, `gps_max_sigma_reject_m`, `graph_save_prefix`, `icp_max_iter`/`icp_max_corresp_dist`/`icp_sigma_*`/`icp_source_subsample`, `isam2_*`, `lc_max_candidates`/`lc_max_dist_m`/`lc_max_rmse`/`lc_min_delta_*`/`lc_sigma_*`, `periodic_save_period_s`, `rtk_autoload_override_threshold_m`, `scan_min_inliers`, `scan_retention_nodes`, `scan_topic`, `scan_yaw_sigma_floor_rad`, `scan_yield_*`, `stationary_motion_thresh_*`, `stationary_node_period_s`. Eight more come only from launch arguments: `primary_mode`, `use_scan_matching`, `use_loop_closure`, `use_magnetometer`, `dock_pose_x`/`_y`/`_yaw`, `dock_pose_yaw_sigma_rad`.

## Environment variables

`.env` decides **which containers exist and how the host is wired**. It does **not** configure the ROS2 stack: no file under `ros2/src` reads any environment variable (verified by grep for `getenv`/`os.environ`). Written by `install/lib/env.sh` `setup_env` (defaults L216–301, writes L325–367); every key must also be on `install/lib/state.sh` `is_allowed_installer_key` L11–33 or presets silently drop it. Full per-key table lives in [`codemaps/deploy.md`](codemaps/deploy.md) § *`docker/.env` → compose fragment → container*.

| `.env` key(s) | Selects | Container | Reaches the process as |
|---|---|---|---|
| `MOWGLI_ROS2_IMAGE`, `GPS_IMAGE`, `LIDAR_IMAGE`, `MAVROS_IMAGE`, `GUI_IMAGE`, `IMAGE_TAG` | image refs (`env.sh` `recompute_image_defaults`) | all | `image:` |
| `ROS_DOMAIN_ID` | `x-ros2-env` anchor (`docker-compose.base.yml:7`) | every ROS service | container env (with `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI`) |
| `ENABLE_FOXGLOVE` | `docker-compose.base.yml:18,44` | `mowgli-ros2` | `full_system.launch.py enable_foxglove:=…` — **the only launch arg the compose stack passes** |
| `ENABLE_MQTT`, `ENABLE_WATCHTOWER` | `docker/stack.sh` / `compose.sh` fragment selection | `mowgli-mqtt`, `watchtower` | container presence |
| `LIDAR_ENABLED`, `LIDAR_TYPE`, `LIDAR_MODEL`, `LIDAR_CONNECTION`, `LIDAR_PORT`, `LIDAR_UART_DEVICE`, `LIDAR_BAUD` | `compose.sh` L102–117 → one `docker-compose.lidar-*.yml` | `mowgli-lidar` | driver args. **`LIDAR_ENABLED` controls the container only** — the ROS stack's LiDAR mode is `mowgli_robot.yaml:lidar_enabled` (`docker-compose.base.yml:13–17`) |
| `GNSS_STACK`, `GNSS_BACKEND`, `GNSS_RECEIVER_FAMILY`, `GNSS_TRANSPORT`, `GNSS_SERIAL_DEVICE`, `GNSS_SERIAL_BAUD`, `GNSS_FRAME_ID` | `docker-compose.gps.yml:37–49` (deliberately no compose defaults) | `mowgli-gps` | `start_gps.sh` resolvers (YAML first, env second, built-in last) → `receiver_node --ros-args -p …` |
| `GNSS_NTRIP_{ENABLED,HOST,PORT,MOUNTPOINT,USERNAME,PASSWORD,GGA_ENABLED,GGA_INTERVAL_S}` | `docker-compose.gps.yml:50–57` | `mowgli-gps` | `ntrip_node -p caster_host/…` |
| `HARDWARE_BACKEND`, `MAVROS_*` | `compose.sh` L127 → `docker-compose.mavros.yml` | `mowgli-mavros`, `mowgli-ntrip` | MAVLink wiring; `mavros` forces `GNSS_BACKEND=disabled` (`env.sh` L303–305) |
| `TFLUNA_{FRONT,EDGE}_*` | `docker-compose.tfluna-*.yml` | range sidecars | driver args |
| `COMPOSE_PROJECT_NAME` | named-volume prefix (`install_mowgli_maps`) | all | **keep stable** — renaming orphans persisted maps |
| `MOWER_IP`, `DISABLE_BLUETOOTH`, `GPS_PROTOCOL` | host/ser2net helpers | — | `GPS_PROTOCOL` and `GNSS_BACKEND`/`HARDWARE_BACKEND` are passed into `mowgli-ros2` but **nothing there reads them** |

## Recipe: adding a new parameter

Follow all six steps, in order. Skipping step 2 is the classic failure: the key shows up in the GUI, the operator edits it, and the robot keeps running the compiled default forever (see every `INERT` row above).

1. **Template first (Invariant 15).** Add `my_param: <default>` with a comment explaining the value to `ros2/src/mowgli_bringup/config/mowgli_robot.yaml`. **Do NOT add it to `install/config/mowgli/mowgli_robot.yaml`** — that file is sparse and holds only install choices + calibration outputs. If the value is a genuine install-time choice, do the reverse (installed file only, plus an entry in the parity-test allowlist).
2. **Inject it in the launch file that owns the node.** `mowgli.launch.py` L194–268 for `hardware_bridge`; `full_system.launch.py` L216–354 for the BT and L371–435 for `map_server`; `navigation.launch.py` `_inject_dock_pose_and_speeds` L646–955 for anything under Nav2 / `coverage_server` / `collision_monitor`; `fusion_graph.launch.py` L143–158 for the localizer. Keep the inline `robot_params.get(key, <fallback>)` fallback equal to the template value. **Injected dicts are appended after the static params file, so they win** — that ordering is how `tool_width` overrides `map_server.yaml`, and it is also why `imu_cal_samples` overrides `hardware_bridge.yaml`.
3. **Declare it in the node.** `declare_parameter<T>("my_param", <same default>)` in the constructor (`.claude/rules/ros2.md` — no undeclared parameter access). If it should be live-tunable, handle it in the node's `add_on_set_parameters_callback` (`ftc_controller.cpp:260`) or read it per-operation (`coverage_server.cpp:445`).
4. **GUI schema.** Add the field to the right section of `gui/asserts/mower_config.schema.json` with `title`, `description`, and a `default` **identical to the template**. That default is what the GUI's reset-to-default writes and what the backend prunes against.
5. **Parity test.** `gui/pkg/api/schema_template_parity_test.go` `TestSchemaDefaultsMatchTemplate` fails on any schema default that differs from the template or has no template line. If the parameter legitimately has no template line, add it to `schemaDefaultsWithNoTemplateEntry` (L26) **with the reason**.
6. **Guards and docs.** Add an AST guard to `ros2/src/mowgli_bringup/test/test_launch_injection.py` if the injection has semantics a refactor could break (clamping, sentinel ranges, which node receives it); add a static pin to `test/test_nav2_params.py` if it lands in the merged Nav2 doc; extend `ros2/scripts/check_config_drift.py`'s field lists if it is structural or a calibration output. Then update this file, the relevant codemap, and — only if it changes an architecture rule — CLAUDE.md.

**Removing** a parameter: delete it from the template *and* the schema, add it to `settings.go` `retiredParamKeys` so existing installed yamls get scrubbed on the next save, and add it to `useSettingsManager.ts` `HIDDEN_FROM_ADVANCED` (L654) so it stops rendering in the meantime.
