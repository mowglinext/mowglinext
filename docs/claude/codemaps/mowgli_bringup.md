# Codemap: mowgli_bringup

> Launch files, the `mowgli_robot.yaml` TEMPLATE of defaults, the Nav2 base+overlay params, twist_mux, hardware_bridge config, and the URDF/xacro for the Mowgli mower. It owns no C++ node: it composes every other package into the running stack, deep-merges the sparse installed robot config over the template (`robot_config_util.py`, CLAUDE.md Invariant 15), and INJECTS operator knobs from that merged config into Nav2/coverage/BT/hardware node parameters at launch. Anything not injected here silently runs at a node's compiled default.
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `ros2/CLAUDE.md`.

## Where to look

| Task | Start here |
|------|------------|
| Understand how the SPARSE installed config becomes a full param set | `ros2/src/mowgli_bringup/launch/robot_config_util.py` `deep_merge` (L67), `load_robot_params` (L109); tests `test/test_robot_config_util.py` L71–159 |
| Add a new operator knob (`mowgli_robot.yaml` → node param) | 1) template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml`; 2) inject in the launch file that owns the node (`navigation.launch.py` `_inject_dock_pose_and_speeds` L646 for Nav2/coverage, `full_system.launch.py` L221–353 BT / L371–435 map_server, `mowgli.launch.py` L194–254 hardware_bridge); 3) GUI schema `gui/asserts/mower_config.schema.json`; 4) guard in `test/test_launch_injection.py` |
| LiDAR on/off: what `use_lidar` gates | `robot_config_util.py` `resolve_lidar_enabled` (L186, absent key → `DEFAULT_LIDAR_ENABLED=False` L172); `navigation.launch.py` `lidar_gated()` L237, overlay pick L956–962, scan nodes L1187–1248; `full_system.launch.py` L135, obstacle_tracker gate L636–647 |
| Change a Nav2 param shared by both variants | `ros2/src/mowgli_bringup/config/nav2_params_base.yaml` (never the overlays); pin with `test/test_nav2_params.py` |
| Change a LiDAR-only / GPS-only Nav2 difference | `config/nav2_params_lidar.yaml` (costmap obstacle layers, collision_monitor polygons) / `config/nav2_params_no_lidar.yaml` (static layers, pass-through monitor); `test_nav2_params.py` `test_base_has_no_costmap_layers_or_polygons` L605 |
| Coverage speeds / turn radius / swath spacing injection | `navigation.launch.py` L741–803 (FollowPath/FTC speeds, `derive_turn_speed`, `check_turn_geometry`), L922–948 (`coverage_server.*`); helpers `robot_config_util.py` L268, L309 |
| Docking tuning (staging distance, overshoot, retries, GPS dock detection) | `navigation.launch.py` L665–739 (`ds`/`home_dock`/`scd` writes), gps_dock_detection node L1259–1287; static base `nav2_params_base.yaml` L888–1090; template L421–467 |
| Obstacle-avoidance knobs (GUI Settings → Obstacles) | template L569–669 → `navigation.launch.py` L811–868 (clamps) → `nav2_params_base.yaml` `FollowCoveragePath` L421–546, local `inflation_layer` L831–866 |
| Robot geometry / sensor mount → URDF + Nav2 footprint | template L24–36, L204–219 → `mowgli.launch.py` xacro args L93–164 → `urdf/mowgli.urdf.xacro` L28–61; Nav2 footprint `navigation.launch.py` L285–317 (+5 cm margin) |
| cmd_vel lane priorities / timeouts | `config/twist_mux.yaml` (nav 10 < docking 15 < teleop 20 < tuning 30 < emergency 100; no `locks:`) |
| collision_monitor polygons | `nav2_params_lidar.yaml` L137–284 (`FootprintApproach`, `PolygonStopNarrow`, `PolygonSlow`; static `PolygonStop` disabled L238–245); no-LiDAR pass-through `nav2_params_no_lidar.yaml` L56–80; shared I/O topics `nav2_params_base.yaml` L1095–1118 |
| Add / remove a Nav2 lifecycle server | `launch/nav2_navigation_launch.py` `lifecycle_nodes` L49–59 + its `Node(...)` block L131–256 (vendored nav2_bringup, route_server removed) |
| Why Nav2 waits at startup / bond settings | `scripts/wait_for_tf.py` (gate on `map→odom`, 120 s) via `navigation.launch.py` L987–1034; `bond_timeout` 10 s / `bond_heartbeat_period` 0.5 s L1005–1012 |
| No-LiDAR global costmap "never current" | `scripts/empty_static_map_pub.py` (`/no_lidar_static_map`, transient_local) launched `navigation.launch.py` L1040–1046; consumed by `nav2_params_no_lidar.yaml` L35–40, L48–53 |
| BT operator params (speeds, undock, LocalizationGuard, battery, start-pose escape) | `full_system.launch.py` L216–354 (every `robot_params.get(...)` there is an injection) |
| Hardware bridge wiring (serial, PID push, dig detector, remaps) | `mowgli.launch.py` L184–268; static `config/hardware_bridge.yaml` (dig detector L44–103) |
| GUI/Foxglove bridge + manual-mow relay | `full_system.launch.py` L557–583 (foxglove :8765, GNSS-internal topic whitelist L66–68); `scripts/cmd_vel_ws_relay.py` (ws :8766 → `/cmd_vel_teleop`) |
| fusion_graph launch args from this package | `navigation.launch.py` L1074–1092 (`use_magnetometer`, lidar-gated `use_scan_matching`/`use_loop_closure`, `primary_mode`, `tf_publish_lead_s`, `node_period_s`); first-boot loop-closure gate L150–154 |
| Simulation stack | `launch/sim_full_system.launch.py` (Webots include L141–153, sim TF/cadence overrides L182–183, injected 9×6 m test polygon L219–231, sim helper nodes L297–505) |
| LED ring launch gate + params | `full_system.launch.py` L97–100, L147–151, L668–707; template L721–739 |
| TF sole-ownership guard (Inv 2) | `test/test_tf_ownership.py` `EXPECTED_TF_BROADCASTER_FILES` L52–57 |

## Files

| File | Lines | Purpose |
|------|-------|---------|
| **`ros2/src/mowgli_bringup/`** | | |
| `CMakeLists.txt` | 91 | Installs `launch/ config/ urdf/`, the 3 scripts; registers 2 launch_tests + 6 pytest tests |
| `package.xml` | 70 | exec_depends (Nav2 servers, `fusion_graph`, `twist_mux`, all `mowgli_*` incl. `mowgli_leds`, `foxglove_bridge`) |
| **`launch/`** | | |
| `launch/robot_config_util.py` | 371 | Shared loader: `deep_merge`, `load_robot_config/params`, `DEFAULT_TOOL_WIDTH_M`/`DEFAULT_WHEEL_TRACK_M`/`DEFAULT_LIDAR_ENABLED`, `resolve_lidar_enabled`, `warn_lidar_key_absent`, `derive_turn_speed`, `check_turn_geometry` |
| `launch/full_system.launch.py` | 742 | Tier-2 entry: includes `mowgli` + `navigation`, launches BT, map_server, navsat, monitors, calibrate_imu_yaw, diagnostics, mqtt, foxglove, LED ring, cmd_vel relay, obstacle_tracker |
| `launch/navigation.launch.py` | ~1.3k | Localizer helpers + fusion_graph include + Nav2 (base⊕overlay deep-merge, robot-yaml injection, RewrittenYaml, TF-gated bringup) |
| `launch/mowgli.launch.py` | 298 | Tier-1 hardware: robot_state_publisher (xacro args from robot config), hardware_bridge, twist_mux |
| `launch/nav2_navigation_launch.py` | 358 | Vendored nav2_bringup `navigation_launch.py` (Jazzy) minus route_server, plus `docking_server` + `coverage_server` |
| `launch/sim_full_system.launch.py` | 535 | Webots full stack (sim_time=true), sim helpers, own twist_mux/navsat/map_server |
| `launch/foxglove_bridge.launch.py` | 93 | Standalone foxglove_bridge (1 MB send buffer) — NOT included by `full_system` |
| **`config/`** | | |
| `config/mowgli_robot.yaml` | 739 | TEMPLATE of every robot-param default (Invariant 15); installed sparse twin is `install/config/mowgli/mowgli_robot.yaml` |
| `config/nav2_params_base.yaml` | ~1.2k | Shared Nav2 base: bt_navigator, controller_server (FollowPath RPP / FollowCoveragePath FTC / goal+progress checkers), planner, smoother, behavior_server, costmaps, docking_server, collision_monitor I/O, coverage_server |
| `config/nav2_params_lidar.yaml` | 284 | Overlay: `/scan_costmap` obstacle layers, `/scan_collision` collision_monitor polygons, `FollowPath.use_collision_detection: true` |
| `config/nav2_params_no_lidar.yaml` | 80 | Overlay: `/no_lidar_static_map` static layers, FTC obstacle flags off, monitor pass-through, planner `costmap_update_timeout: 5.0` |
| `config/twist_mux.yaml` | 53 | 5 lanes, `use_stamped: true`, deliberately no `locks:` |
| `config/hardware_bridge.yaml` | 103 | Serial/baud/rates, `imu_cal_samples: 1000`, dig-detector `dig_*` (Invariant 16) |
| `config/foxglove_bridge.yaml` | 11 | Foxglove params + GNSS-internal whitelist — **not referenced by any launch file** |
| **`scripts/`** (installed to `lib/mowgli_bringup/`) | | |
| `scripts/wait_for_tf.py` | 70 | `--parent map --child odom --timeout 120`; exit 0 when TF resolvable, 1 on timeout |
| `scripts/empty_static_map_pub.py` | 61 | Latched empty `OccupancyGrid` on `/no_lidar_static_map` (params `size_m` 200, `resolution` 0.5, `frame_id` map) |
| `scripts/cmd_vel_ws_relay.py` | 121 | WebSocket 127.0.0.1:8766 JSON → `/cmd_vel_teleop` (`TwistStamped`, clamps ±2.0 m/s / ±5.0 rad/s) |
| **`urdf/`** | | |
| `urdf/mowgli.urdf.xacro` | 408 | Frames + visuals only (no sim plugins): base_footprint→base_link→wheels/casters/blade/imu/gps/lidar; all dims are xacro args |
| **`test/`** | | |
| `test/test_nav2_params.py` | 934 | Static pins on merged base⊕overlay (plugins, tolerances, injections, polygons, odom_topic, turn geometry) |
| `test/test_robot_config_util.py` | 592 | Deep-merge semantics, tool_width/wheel_track single-source, turn-speed/geometry helpers, LiDAR resolution + GUI schema default, scan-factor gating |
| `test/test_launch_injection.py` | 262 | AST guards: `num_headland_passes` unclamped, `mowing_enabled` → hardware_bridge only, `dock_max_retries`/`dock_use_charger_detection` → docking_server |
| `test/test_tf_ownership.py` | 100 | Only fusion_graph + wheel_odometry construct `TransformBroadcaster`; `wheel_odometry.yaml publish_tf: false`; no launch file sets `publish_tf` |
| `test/test_gnss_launch_config.py` | 61 | `full_system` declares no `use_universal_gnss`, includes no `universal_gnss.launch.py`, passes no legacy `gnss_backend`/`gps_protocol` params |
| `test/test_urdf_xacro.py` | 49 | `chassis_mass_kg` reaches base_link inertial; `blade_joint` is fixed base_link→blade_link |
| `test/test_nodes_startup.launch.py` | 257 | launch_test: 5 nodes stay alive 5 s, `/wheel_odom` `/diagnostics` `/mowgli_behavior_node/high_level_status` advertised, map_server/BT services present, exit 0 |
| `test/test_navsat_status_universal.launch.py` | 138 | launch_test: `navsat_to_absolute_pose_node` publishes `/gps/absolute_pose` + `/gps/pose_cov`, never `/gps/status` |

## Runtime surface

### Nodes (what each launch file starts)

| Node name | package/executable | Launched by | Condition / notes |
|-----------|--------------------|-------------|-------------------|
| `robot_state_publisher` | robot_state_publisher | `mowgli.launch.py` L173 | `robot_description` = xacro(`urdf/mowgli.urdf.xacro`, args from merged robot params) |
| `hardware_bridge` | mowgli_hardware/`hardware_bridge_node` | `mowgli.launch.py` L189 | params `config/hardware_bridge.yaml` + injected keys (see Parameters); remaps L257–267 |
| `twist_mux` | twist_mux | `mowgli.launch.py` L273; sim L372 | `config/twist_mux.yaml`, `cmd_vel_out`→`/cmd_vel` |
| `static_gps_link_to_gps_alias` | tf2_ros/static_transform_publisher | `navigation.launch.py` L1055 | identity `gps_link`→`gps` |
| `fusion_graph_node` | fusion_graph (include `fusion_graph.launch.py`) | `navigation.launch.py` L1074 | unconditional (Invariant 1); args listed under Parameters |
| `cog_to_imu` | mowgli_localization/`cog_to_imu` | `navigation.launch.py` L1114 | datum + `lever_arm_x/y` (= `gps_x/gps_y`), `stationary_seed_rate_hz`, `stationary_yaw_drift_rate: 0.001` |
| `mag_yaw_publisher` | mowgli_localization/`mag_yaw_publisher` | `navigation.launch.py` L1157 | only if `use_magnetometer` AND `/ros2_ws/maps/mag_calibration.yaml` exists |
| `scan_deskew` | mowgli_localization/`scan_deskew_node` | `navigation.launch.py` L1187 | `use_lidar`; `/scan`→`/scan_deskewed` |
| `costmap_scan_filter` | mowgli_localization/`costmap_scan_filter_node` | `navigation.launch.py` L1203 | `use_lidar`; `/scan_deskewed`→`/scan_costmap` (+`/scan_collision`, node default); `lidar_height_m`=`lidar_z`, `lidar_mount_yaw`=`lidar_yaw−imu_yaw` |
| `gps_dock_detection` | mowgli_localization/`gps_dock_detection_node` | `navigation.launch.py` L1259 | `use_gps_dock_detection`; publishes `/detected_dock_pose` in `odom` |
| `wait_for_map_odom_tf` | ExecuteProcess `wait_for_tf.py` | `navigation.launch.py` L992 | Nav2 group starts `OnProcessExit` (L1029) — also on exit code 1 |
| `controller_server`, `smoother_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, `collision_monitor`, `docking_server` (opennav_docking), `coverage_server` (mowgli_coverage/`mowgli_coverage`), `lifecycle_manager_navigation` | Nav2 | `nav2_navigation_launch.py` L131–256 via `navigation.launch.py` L1013 | all lifecycle-managed, `use_composition: False`, `respawn` off; controller/behavior `cmd_vel`→`cmd_vel_nav`, docking `cmd_vel`→`cmd_vel_docking` |
| `empty_static_map_pub` | mowgli_bringup/`empty_static_map_pub.py` | `navigation.launch.py` L1040 | `UnlessCondition(use_lidar)` |
| `behavior_tree_node` | mowgli_behavior | `full_system.launch.py` L216; sim L190 | `behavior_tree.yaml` + ~35 injected keys (sim injects none) |
| `map_server_node` | mowgli_map | `full_system.launch.py` L366; sim L204 | `map_server.yaml` + dock/geometry/datum injection; sim adds polygon `main_mow` |
| `obstacle_tracker` | mowgli_map/`obstacle_tracker_node` | `full_system.launch.py` L636; sim L282 | `use_obstacle_tracker AND use_lidar` (sim: `use_lidar`) |
| `navsat_to_absolute_pose` | mowgli_localization/`navsat_to_absolute_pose_node` | `full_system.launch.py` L454; sim L348 | `datum_lat/lon` (sim hardcodes 48.137154/11.576124) |
| `localization_monitor_node`, `calibrate_imu_yaw_node` | mowgli_localization | `full_system.launch.py` L471, L485 | calibrate node gets `undock_*` + 9 `dock_calib_*` keys |
| `diagnostics_node` | mowgli_monitoring | `full_system.launch.py` L520; sim L238 | `lidar_enabled` = `use_lidar` (bool) |
| `mqtt_bridge_node` | mowgli_monitoring | `full_system.launch.py` L539 | `enable_mqtt` |
| `foxglove_bridge` | foxglove_bridge | `full_system.launch.py` L557; sim L256 | `enable_foxglove`; port `foxglove_port`; capabilities incl. `parameters`/`parametersSubscribe` |
| `led_ring_node` | mowgli_leds | `full_system.launch.py` L668 | `led_enabled`; 12 `led_*` keys from template |
| `cmd_vel_ws_relay` | mowgli_bringup/`cmd_vel_ws_relay.py` | `full_system.launch.py` L598 | always |
| `fake_hardware_bridge`, `sim_navsat_rtk_fix`, `sim_wheel_slip`, `sim_imu_noise`, `sim_actuation` | mowgli_simulation | `sim_full_system.launch.py` L297–505 | sim only; Webots via `mowgli_simulation/launch/webots_minimal.launch.py` |

### Launch arguments → what they control

| Launch file | Arg | Default | Controls |
|-------------|-----|---------|----------|
| `full_system` | `use_sim_time` | `false` | forwarded to every node |
| | `serial_port` | `/dev/mowgli` | → `mowgli.launch.py` → `hardware_bridge.serial_port` |
| | `enable_mqtt` / `enable_foxglove` / `foxglove_port` | `false` / `true` / `8765` | mqtt_bridge_node; foxglove_bridge + its port |
| | `use_lidar` | `mowgli_robot.yaml.lidar_enabled` (absent → `false` + loud warn) | obstacle_tracker gate, `diagnostics_node.lidar_enabled`, forwarded to `navigation.launch.py` |
| | `use_obstacle_tracker` | `true` | obstacle_tracker (ANDed with `use_lidar`) |
| | `led_enabled` | `mowgli_robot.yaml.led_enabled` | led_ring_node |
| `navigation` | `use_lidar` | as above (re-resolved, warning deduped) | overlay pick (`nav2_params_lidar` vs `_no_lidar`), scan_deskew/costmap_scan_filter, empty_static_map_pub, AND-gate on the two scan flags |
| | `use_magnetometer` | `mowgli_robot.yaml.use_magnetometer` | mag_yaw_publisher gate + fusion_graph arg |
| | `use_scan_matching` / `use_loop_closure` | template `true`/`true`; loop-closure forced `false` when `/ros2_ws/maps/fusion_graph.graph` absent | fusion_graph args, each `lidar_gated()` |
| | `use_gps_dock_detection` | `mowgli_robot.yaml.use_gps_dock_detection` (`true`) | gps_dock_detection node + `simple_charging_dock.use_external_detection_pose` and zeroed `external_detection_*` (L712–729) |
| | `cog_stationary_seed_rate_hz` | `2.0` | `cog_to_imu.stationary_seed_rate_hz` |
| | `fusion_graph_tf_lead_s` | `0.05` (sim passes `0.1`) | fusion_graph `tf_publish_lead_s` (both TF legs) |
| | `fusion_graph_node_period_s` | `mowgli_robot.yaml.fusion_graph_node_period_s` else `0.04` (sim `0.02`) | fusion_graph `node_period_s` |
| `mowgli` | `use_sim_time`, `serial_port` | `false`, `/dev/mowgli` | RSP/bridge/mux |
| `nav2_navigation_launch` | `namespace`, `use_sim_time`, `params_file`, `autostart`, `use_composition`, `container_name`, `use_respawn`, `log_level` | `''`, `false`, `nav2_params_base.yaml` (vestigial), `true`, `False`, `nav2_container`, `False`, `info` | `navigation.launch.py` always passes `params_file` (merged temp yaml) + `use_composition: False` |
| `sim_full_system` | `world` / `use_rviz` / `headless` / `use_lidar` / `mode` | `mowgli_garden.wbt` / `false` / `true` (**ignored**) / `true` / `realtime` | Webots world; `headless` is a Gazebo-era shim; `use_lidar` NOT read from yaml here |
| `foxglove_bridge` | `port` / `send_buffer_limit` | `8765` / `1000000` | standalone bridge only |

### Topics wired by this package

| Topic | Type | Direction / who |
|-------|------|-----------------|
| `cmd_vel_nav` → `cmd_vel_monitored` | TwistStamped | controller_server + behavior_server remap → collision_monitor (`cmd_vel_in/out_topic`, base L1101–1104) → twist_mux lane `navigation` (prio 10, timeout 0.6 s) |
| `/cmd_vel_docking` | TwistStamped | docking_server remap (`nav2_navigation_launch.py` L229) → lane prio 15; bypasses collision_monitor |
| `/cmd_vel_teleop` | TwistStamped | `cmd_vel_ws_relay.py` (GUI manual mow) → lane prio 20 |
| `/cmd_vel_tuning`, `/cmd_vel_emergency` | TwistStamped | lanes prio 30 / 100 (timeouts 0.5 / 0.2 s) |
| `/cmd_vel` | TwistStamped | twist_mux out → `hardware_bridge` `~/cmd_vel` (sim: Webots diff drive) |
| `/imu/data`, `/imu/mag_raw`, `/wheel_odom`, `/wheel_ticks`, `/hardware_bridge/{emergency,power,status}`, `/gnss/heading`, `/battery_state` | sensor/status | hardware_bridge remaps (`mowgli.launch.py` L257–267; `/battery_state` absolute, = docking `battery_topic`) |
| `/scan` → `/scan_deskewed` → `/scan_costmap` / `/scan_collision` | LaserScan | scan_deskew → costmap_scan_filter; costmaps read `/scan_costmap` (lidar overlay L27, L95), collision_monitor reads `/scan_collision` (L273) |
| `/no_lidar_static_map` | OccupancyGrid (transient_local) | empty_static_map_pub → no-LiDAR static_layers |
| `/detected_dock_pose` | PoseStamped | gps_dock_detection (remap L1286) → SimpleChargingDock |
| `/controller_server/FollowCoveragePath/global_plan` | Path | `coverage_goal_checker.plan_topic` (base L189) |
| `/local_costmap/published_footprint` | Polygon | `FootprintApproach.footprint_topic` (lidar L165) |
| `/wheel_odom` / `/odometry/filtered` | Odometry | `controller_server.odom_topic` + docking `controller.odom_topic` (base L77, L907) / `bt_navigator.odom_topic` (L24) |
| `/costmap_filter_info` | CostmapFilterInfo | `keepout_filter.filter_info_topic` (base L785); published by map_server |
| ws `:8765` / ws `127.0.0.1:8766` | — | foxglove_bridge / cmd_vel_ws_relay |

### Services & actions
None defined here. Launched servers expose Nav2's action set plus `plan_coverage` (`coverage_server`) and docking (`docking_server`, dock id `home_dock`, base L1024–1030). `wait_for_tf.py` is a process gate, not a service.

### Parameters — where defaults live and what is injected

All injection is READ ONCE at launch (container restart required); exceptions: `coverage_server.chassis_safety_inset` / `min_swath_length` are read live per plan (base L1176, L1180), and foxglove's `parameters` capability lets Foxglove set live params.

| Template key (`config/mowgli_robot.yaml`) | Injected as | By |
|------|------|------|
| `chassis_length/width/height/mass_kg`, `chassis_center_x`, `wheel_radius/width/track`, `wheel_x_offset`, `caster_*`, `blade_radius`, `lidar_x/y/z/yaw`, `imu_x/y/z/yaw` (no `imu_roll`/`imu_pitch` template key — launch defaults them to 0.0), `gps_x/y/z` (L24–36, L190–219) | xacro args | `mowgli.launch.py` L93–164 |
| `chassis_length/width/center_x` | Nav2 `footprint` (+0.05 m margin) | `navigation.launch.py` L304–317, L973–974 |
| `dock_pose_x/y/yaw`, `imu_yaw`, `wheel_track`, `ticks_per_meter`, `wheel_pid_kp/ki/kd/integral_limit/pwm_per_mps`, `imu_cal_*`, `lift_recovery_mode`, `lift_blade_resume_delay_sec`, `mowing_enabled`, `yaw_kp/ki`, `yaw_trim_limit_mps`, `yaw_loop_enabled`, `yaw_gyro_sign` | hardware_bridge params | `mowgli.launch.py` L200–253 |
| `tick_rate`, `bt_debug_logging`, `undock_speed/distance`, `idle_nav2_suspend`, `transit_speed`, `mowing_speed`, `mow_angle_deg`, `area_simplification_tolerance`, `area_record_rate_hz`, `loc_gnss_*`, `loc_sigma_*`, `start_blocked_escape_*`, `battery_*` | behavior_tree_node params | `full_system.launch.py` L226–352 |
| `dock_pose_*`, `dock_body_length_m/width_m`, `chassis_width`, `max_obstacle_avoidance_distance`, `obstacle_margin` (clamp 0–1), `lethal_outside_areas`, `enforce_boundary_margin_m`, `tool_width`, `datum_lat/lon` | map_server_node params | `full_system.launch.py` L380–434 |
| `datum_lat/lon` | navsat_to_absolute_pose + cog_to_imu | `full_system.launch.py` L460–463; `navigation.launch.py` L1121–1122 |
| `undock_*`, `dock_calib_*` (9 keys) | calibrate_imu_yaw_node | `full_system.launch.py` L492–513 |
| `led_*` (12 keys) | led_ring_node | `full_system.launch.py` L679–704 |
| `transit_speed` | `FollowPath.desired_linear_vel` | `navigation.launch.py` L745 |
| `mowing_speed`, `turn_speed_ratio` | `FollowCoveragePath.speed_fast`, `.speed_slow` (= `derive_turn_speed`), raises `.max_cmd_vel_speed` if needed | L755–781 |
| `blade_load_slowdown_enabled`, `blade_load_rpm_full`, `blade_load_rpm_min`, `blade_load_min_speed_ratio` [0.1,1] (= `derive_blade_load_params`; empty ramp → disabled + WARN) | `FollowCoveragePath.blade_load_*` | after `speed_slow` |
| `max_obstacle_avoidance_distance` [0.5,10], `obstacle_detection_range_m` [0.2,5]→poses/0.05 (≥4), `obstacle_clearance_margin` [0,0.5], `obstacle_wait_timeout_s` [0.5,60], `obstacle_reverse_enabled`, `obstacle_reverse_max_dist_m` [0,1], `obstacle_reverse_speed_mps` [0,0.3] | `FollowCoveragePath.max_lateral_deviation`, `.obstacle_lookahead`, `.obstacle_clearance_margin`, `.obstacle_wait_timeout_s`, `.obstacle_reverse_*` | L811–848 |
| `obstacle_inflation_radius` [0.58,1.5] | `local_costmap.inflation_layer.inflation_radius` ONLY (global 0.20 pinned) | L855–860 |
| `obstacle_slowdown_ratio` [0.05,1] | `collision_monitor.PolygonSlow.slowdown_ratio` (LiDAR merge only) | L864–868 |
| `xy_goal_tolerance`, `yaw_goal_tolerance` | `stopped_goal_checker.*` | L877–879 |
| `coverage_xy_tolerance` (floored at FTC `max_goal_distance_error` 0.50) | `coverage_goal_checker.xy_goal_tolerance` | L880–905 |
| `progress_timeout_sec` | `progress_checker.movement_time_allowance` | L909–910 |
| `tool_width − swath_overlap` (≥0.05), `chassis_width`, `headland_width`, `num_headland_passes` (UNCLAMPED sentinel), `mow_direction`, `chassis_safety_inset` (default 0.0 if absent), `obstacle_margin`, `min_turning_radius` [0.10,0.50], `connector_turn_radius` (≥ floor, ≤0.50) | `coverage_server.operation_width`, `.robot_width`, `.default_headland_width`, `.num_headland_passes`, `.ring_direction`, `.chassis_safety_inset`, `.obstacle_margin`, `.min_turning_radius`, `.connector_turn_radius` | L922–948 |
| `dock_pose_*` + `dock_approach_overshoot`, `dock_max_retries`, `dock_charging_threshold`, `dock_use_charger_detection`, `dock_approach_distance` (→ negative) | `docking_server.home_dock.pose` (list), `.max_retries`, `simple_charging_dock.charging_threshold`, `.use_battery_status`, `.staging_x_offset` | L669–739 |
| `gps_x/y` | `cog_to_imu.lever_arm_x/y` | L1127–1128 |
| `enable_mag_cal`, `mag_calibration_path`, `declination_deg`, `min_horizontal_uT`, `mag_yaw_variance` | cog_to_imu (`mag_calibration_path`) / mag_yaw_publisher (`yaw_variance`; its `calibration_path` is the hardcoded L1155 path, NOT the template key) | L1129–1130, L1167–1170 |
| `lidar_z`, `lidar_yaw`, `imu_yaw` | `costmap_scan_filter.lidar_height_m`, `.lidar_mount_yaw` | L302–303, L1234–1235 |

Static (non-template) defaults that matter: `controller_server.odom_topic: /wheel_odom` (base L77); `controller_frequency: 10` (L83); FTC block L338–546 (`max_goal_distance_error: 0.50` L410, `forward_only: true` L420, `use_footprint_clearance: false` L450, `require_clear_exit: true` L468, `ignore_obstacles_outside_zone: true` L508); RPP lookahead L293–296; `behavior_server.max_rotational_vel: 1.0` / `rotational_acc_lim: 2.0` (L643, L650); global costmap 70×70 m @0.08 (L715, L741–742), `keepout_filter` enabled (L782–785); local 12×12 m @0.05 (L812, L821–822); `docking_server.controller.*` L901–1023; `collision_monitor.source_timeout: 1.5` (L1116); hardware_bridge `dig_*` (`hardware_bridge.yaml` L44–103).

### TF frames
URDF (`urdf/mowgli.urdf.xacro`): `base_footprint` →(z=`wheel_radius`, L142–146) `base_link` → `left/right_wheel_link` (continuous, L199–206), `front_left/right_caster_link` (L250–256), `blade_link` (fixed, at `chassis_center_x`, L293–298), `imu_link` (rpy = `imu_roll/pitch/yaw`, L327–332), `gps_link` (L361–365), `lidar_link` (yaw = `lidar_yaw`, L394–399). `gps_link→gps` identity alias from `navigation.launch.py` L1055. `map→odom` and `odom→base_footprint` come from `fusion_graph_node` (Invariant 2; pinned by `test_tf_ownership.py`). Nav2: global frame `map`, local/behavior/docking frame `odom`, robot frame `base_footprint` everywhere.

## Build, test, run

```bash
# devcontainer (/ros2_ws)
cd ros2 && make build-pkg PKG=mowgli_bringup            # scripts/build.sh --packages-select
cd ros2 && PACKAGES="mowgli_bringup" ./scripts/test.sh    # colcon test --packages-select + test-result
cd ros2 && make test                                     # whole workspace
# ROS-free (PyYAML only) — runnable on a laptop:
python3 -m pytest ros2/src/mowgli_bringup/test/test_nav2_params.py ros2/src/mowgli_bringup/test/test_robot_config_util.py \
  ros2/src/mowgli_bringup/test/test_launch_injection.py ros2/src/mowgli_bringup/test/test_tf_ownership.py
python3 -m pytest ros2/src/mowgli_bringup/test/test_urdf_xacro.py   # needs the `xacro` binary
# run
ros2 launch mowgli_bringup full_system.launch.py use_lidar:=false serial_port:=/dev/ttyACM0
ros2 launch mowgli_bringup sim_full_system.launch.py mode:=fast
```

Test registration: `CMakeLists.txt` L38–86 — `add_launch_test` for `test_nodes_startup.launch.py` (60 s) and `test_navsat_status_universal.launch.py` (45 s); `ament_add_pytest_test` for `test_nav2_params`, `test_gnss_launch_config` (imports `launch`, needs ROS), `test_robot_config_util`, `test_urdf_xacro`, `test_tf_ownership`, `test_launch_injection`. Lint: `ament_lint_auto` with copyright/cpplint/uncrustify disabled.

CI (`.github/workflows/ros2-ci.yml`): job `config-drift` (L96–113) runs `ros2/scripts/check_config_drift.py` (template vs `install/config/mowgli/mowgli_robot.yaml`: structural-field drift, padded defaults) + `ros2/scripts/test_check_config_drift.py`; job `Build & Test (ROS2 kilted)` (L129+) runs `colcon build` then `colcon test --return-code-on-test-failure` over the whole workspace (L338–350).

Deployment entry points: `install/compose/docker-compose.base.yml` L42–44 `ros2 launch mowgli_bringup full_system.launch.py enable_foxglove:=${ENABLE_FOXGLOVE:-true}` (no `use_lidar:=` — the yaml decides; `/ros2_ws/maps` volume, config mounted RW); `docker/docker-compose.simulation.yaml` L37–41 `sim_full_system.launch.py headless:=true use_rviz:=false`.

## Change coupling — "if you change X, also update Y"

- **New/renamed template key** → the launch injection line (see Parameters table) → GUI schema `gui/asserts/mower_config.schema.json` (defaults must equal the template; the backend prunes values equal to schema defaults — `test_robot_config_util.py` L493 pins `lidar_enabled`) → `gui/web/src/components/settings/paramCatalog.ts` → `ros2/scripts/check_config_drift.py` field classes (STRUCTURAL / USER_OVERRIDE / INSTALL_SEED) → `test_launch_injection.py` if the line is load-bearing.
- **`deep_merge` semantics** (`robot_config_util.py` L67) → `ros2/src/fusion_graph/launch/fusion_graph.launch.py` carries a LOCAL copy (its L27–32 say so) → also imported by `ros2/scripts/compute_nav2_params.py` and `test_nav2_params.py`.
- **`DEFAULT_TOOL_WIDTH_M` / `DEFAULT_WHEEL_TRACK_M`** ↔ template `tool_width` (L195) / `wheel_track` (L32) ↔ firmware `WHEEL_BASE` (`firmware/stm32/ros_usbnode/include/board.h`) ↔ hardware_bridge `wheel_track` default; pinned by `test_robot_config_util.py` L162 and `test_nav2_params.py` L851.
- **FTC/goal-checker param added in `mowgli_nav2_plugins`** → `nav2_params_base.yaml` `FollowCoveragePath` (L338) / `coverage_goal_checker` (L170) → clamps in `navigation.launch.py` L811–848 → `test_nav2_params.py` (both variants must stay identical except `check_obstacles`/`enable_obstacle_deviation`, L448).
- **Nav2 lifecycle server list** `nav2_navigation_launch.py` L49–59 ↔ its `Node` blocks ↔ `nav2_params_base.yaml` section ↔ BT `idle_nav2_suspend` pause/resume set in `mowgli_behavior`.
- **twist_mux lanes** (`config/twist_mux.yaml`) ↔ publishers: collision_monitor `cmd_vel_out_topic` (base L1104), docking remap (`nav2_navigation_launch.py` L229), `cmd_vel_ws_relay.py` L44, GUI/BT `/cmd_vel_tuning`, `/cmd_vel_emergency`. Adding a `locks:` block requires a publisher in hardware_bridge first (CLAUDE.md What NOT to Do).
- **hardware_bridge remap names** (`mowgli.launch.py` L257–267) ↔ `~/` publisher names in `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp` ↔ subscribers in BT/diagnostics/GUI (`/hardware_bridge/status`, `/gnss/heading`).
- **Scan pipeline topics** (`/scan_deskewed`, `/scan_costmap`, `/scan_collision`) ↔ `navigation.launch.py` L1195–1212 ↔ `nav2_params_lidar.yaml` L27, L95, L273.
- **Sim datum** 48.137154/11.576124 appears twice (`sim_full_system.launch.py` L229–230, L356–357) and must match the Webots world.
- **`use_lidar` launch-arg name** is used by CI/dev scripts and compose; the yaml key is `lidar_enabled` (`robot_config_util.py` L139). `docker/.env LIDAR_ENABLED` only composes the LiDAR container.
- **URDF xacro arg added** → `mowgli.launch.py` Command list L133–164 → template default → `test_urdf_xacro.py`.

## Pitfalls

- `navigation.launch.py` evaluates `_inject_dock_pose_and_speeds` for BOTH overlays at description time (L956–957) and writes temp files under `mowgli_nav2_*.yaml`; the `use_lidar` substitution only picks which temp file feeds `RewrittenYaml` (L958–981). Values that need the YAML parser (lists, deep keys) go in the injector; `RewrittenYaml` handles only scalars (`use_sim_time`, `footprint`, BT XML paths, L968–974).
- Do NOT rebind `coverage_xy_tolerance` inside the nested injector — it becomes function-local and the launch crashes with `UnboundLocalError` (L891–896, guarded by `test_nav2_params.py` L180).
- Template keys with NO template default: `lidar_enabled` (deliberate, L146–172 of `robot_config_util.py`), `connector_turn_radius` (fallback 0.18 only in `navigation.launch.py` L403) and `fusion_graph_node_period_s` (fallback 0.04, L139) — the GUI cannot "reset" the last two, and `check_config_drift.py` flags installed structural keys with no template default.
- Injected but NOT declared by the target node: `map_server_node.chassis_safety_inset` (`full_system.launch.py` L395–398; no `chassis_safety_inset` in `ros2/src/mowgli_map`), `hardware_bridge.imu_yaw` (`mowgli.launch.py` L203; node comment L622 says URDF-only). Both are inert.
- Declared by a node but NOT injected from the template: hardware_bridge `max_mps`, `max_charge_voltage/current`, `*_emergency_ms` (template L49–63; node declares them at `hardware_bridge_node.cpp` L379–392; absent from `mowgli.launch.py` L194–254); behavior_tree_node `rain_mode`, `rain_delay_minutes`, `rain_debounce_sec` (template L508–510; node defaults `behavior_tree_node.cpp` L877–891 — `rain_debounce_sec` default 0.0 vs template 10.0). Editing these template keys changes nothing on the robot until an injection is added.
- Orphan template keys with no consumer anywhere in `ros2/src`: `gps_wait_after_undock_sec`, `gps_timeout_sec`. (`path_spacing` and `ticks_per_revolution` were the other two and were DELETED from the template on 2026-09-05; `path_spacing` survives in the GUI schema only, allow-listed in `schema_template_parity_test.go`.) `dock_pose_yaw_sigma_rad` is read by `navigation.launch.py` L574 but injected nowhere there — `fusion_graph.launch.py` reads it itself (its L154).
- `config/foxglove_bridge.yaml` is dead: both `full_system.launch.py` L563–582 and `foxglove_bridge.launch.py` inline their params. The whitelist regex lives in `full_system.launch.py` L66–68.
- `hardware_bridge.yaml`, `twist_mux.yaml` are loaded from the PACKAGE share dir (`mowgli.launch.py` L185–187, L271), never from `/ros2_ws/config/`; the copies in `install/config/mowgli/` differ and are not read. Only `mowgli_robot.yaml` is read from `/ros2_ws/config` (`robot_config_util.py` L31).
- `test_nav2_params.py` L579–600 forbids `min_turning_radius` in the STATIC `coverage_server` block, yet `navigation.launch.py` L944 injects it at launch — keep it out of the yaml, inject only.
- `num_headland_passes` is a three-way sentinel (<0 none, 0 auto, >0 forced); any `max/min/abs` on its path breaks NONE (`test_launch_injection.py` L134).
- `wait_for_tf.py` exit code 1 (timeout, 120 s) still triggers the Nav2 group (`OnProcessExit`, L1029) — Nav2 then starts without `map→odom`.
- `sim_full_system.launch.py` does NOT go through `mowgli.launch.py`: it launches its own twist_mux (L372) and skips hardware_bridge/RSP; `use_lidar` there is a plain `true` default (L95), not yaml-resolved; `headless` is ignored (L89–93); sim `map_server_node` gets a hardcoded 9×6 m polygon (L219–224) — never copy that into `map_server.yaml` (CLAUDE.md What NOT to Do).
- `nav2_navigation_launch.py` `params_file` default (L95) is the bare base yaml — costmap-incomplete; always launch through `navigation.launch.py`.
- `package.xml` still says "Nav2 + robot_localization dual EKF" and lists `nav2_mppi_controller` as "coverage controller" — neither is true (Invariants 1, 8); the exec_depend list is otherwise current.
- `full_system.launch.py` fallbacks (`.get(key, X)`) can lag the template (e.g. `dock_calib_cog_std_max_rad` 0.0524 at L509 vs template 0.70 at L496; `undock_distance` 2.0 at L492 vs 1.5). They only fire if the template itself loses the key — the template is the live default.
- Invariant 16's dig detector thresholds are in `config/hardware_bridge.yaml` L44–103, not the template — the GUI cannot reset them.

## Generated & vendored — do not hand-edit

- `launch/nav2_navigation_launch.py` — vendored from `nav2_bringup/launch/navigation_launch.py` (Apache-2.0, Intel), edited only to drop `route_server` and add `docking_server`/`coverage_server`.
- `/tmp/mowgli_nav2_*.yaml` — temp files written by `navigation.launch.py` `_inject_dock_pose_and_speeds` at every launch; never commit or edit.
- `/ros2_ws/config/mowgli_robot.yaml` (runtime) is written back by `calibrate_imu_yaw_node`, `map_server_node` (`/set_docking_point`), and the GUI (`gui/pkg/api/drive_tuning.go`) via line-splice — the committed seed is `install/config/mowgli/mowgli_robot.yaml`, not this package.
