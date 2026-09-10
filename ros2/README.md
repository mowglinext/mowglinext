# Mowgli ROS2

A complete ROS2 Kilted robot mower stack built from scratch. Autonomous coverage mowing with RTK-GPS, a GTSAM iSAM2 factor-graph localizer (`fusion_graph`), multi-area continuous-subpath coverage, and a BehaviorTree.CPP v4 mission executor. Targets ARM boards (Rockchip) deployed in Docker containers.

Originally inspired by the [OpenMower](https://github.com/ClemensElflein/open_mower_ros) project but rewritten from the ground up for ROS2 Kilted with Nav2, a REP-105-compliant GPS+IMU+wheels localizer, and multi-area continuous-subpath coverage. The localizer is `fusion_graph_node` (GTSAM iSAM2) — a factor-graph estimator that is the sole, default, unconditional localizer and owns **both** `map→odom` AND `odom→base_footprint`. It fuses RTK-GPS, wheel odometry, IMU gyro, GPS course-over-ground, magnetometer yaw, and optional LiDAR scan-matching + loop-closure factors in one Pose2 graph. It replaced the earlier robot_localization dual-EKF (`ekf_map_node` + `ekf_odom_node`), `navsat_transform_node`, slam_toolbox, and Kinematic-ICP, all of which were removed.

[![CI](https://github.com/mowglinext/mowglinext/actions/workflows/ros2-ci.yml/badge.svg)](https://github.com/mowglinext/mowglinext/actions/workflows/ros2-ci.yml)
[![Docker](https://github.com/mowglinext/mowglinext/actions/workflows/ros2-docker.yml/badge.svg)](https://github.com/mowglinext/mowglinext/actions/workflows/ros2-docker.yml)

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Features](#features)
3. [Packages](#packages)
4. [TF Tree](#tf-tree)
5. [Key Topics and Services](#key-topics-and-services)
6. [Configuration](#configuration)
7. [Building](#building)
8. [Docker Deployment](#docker-deployment)
9. [Launch Files](#launch-files)
10. [Behavior Tree](#behavior-tree)
11. [Simulation](#simulation)
12. [GUI Integration](#gui-integration)
13. [Hardware Protocol](#hardware-protocol)
14. [Contributing](#contributing)
15. [License](#license)

---

## Architecture Overview

```
 +---------------------------------------------------------------------+
 |                         MowgliNext GUI                              |
 |          Go backend (foxglove client) + React frontend              |
 +----------------------------+----------------------------------------+
                              | Foxglove WS (:8765) + teleop relay (:8766)
 +----------------------------v----------------------------------------+
 |                     behavior_tree_node                              |
 |  BehaviorTree.CPP v4   main_tree.xml                                |
 |  Guards: Emergency -> Sensors -> Boundary -> Localization ->        |
 |          GPS mode -> Nav2 resume -> MainLogic                       |
 |  Actions: Undock -> PlanCoverage -> Transit -> Mow -> Dock (+resume)|
 +------+--------------------------------------+----------------------+
        |                                      |
 +------v------+                        +------v---------------------+
 |  map_server |                        |         Nav2 Kilted         |
 |  GridMap    |                        |  FollowPath (RPP+Rotation)  |
 |  keepout    |                        |  FollowCoveragePath (FTC)   |
 |  mask       |                        |  SmacPlanner2D              |
 |  mow_progres|                        |  collision_monitor          |
 |  dock pose  |                        |  docking_server             |
 |  area CRUD  |                        |  coverage_server (F2C v3)   |
 +------+------+                        +-----------------------------+
        |
 +------v--------------------------------------------------------------+
 |                        Localization                                  |
 |                                                                      |
 |  fusion_graph_node  (GTSAM iSAM2 factor graph, sole localizer)      |
 |    Pose2 graph: wheel between-factor (non-holonomic),               |
 |    gyro between-factor, GnssLeverArmFactor (GPS + lever arm),       |
 |    COG / mag yaw unaries, optional LiDAR scan-match + loop-closure  |
 |    -> publishes BOTH map->odom AND odom->base_footprint            |
 |    -> /odometry/filtered_map (map)  +  /odometry/filtered (odom)    |
 |                                                                      |
 |  navsat_to_absolute_pose          cog_to_imu / mag_yaw_publisher     |
 |  NavSatFix -> /gps/pose_cov       COG + magnetometer yaw unaries     |
 |  (fed to GnssLeverArmFactor)      (/wheel_odom from hardware_bridge) |
 +----------------------------------------------------------------------+
        |
 +------v--------------------------------------------------------------+
 |                     hardware_bridge_node                             |
 |  COBS + CRC-16 binary protocol over USB serial to STM32             |
 |  Publishes: /imu/data  /wheel_odom  /hardware_bridge/status         |
 |             /hardware_bridge/power  /hardware_bridge/emergency       |
 |  Subscribes: /cmd_vel  (via twist_mux)                              |
 +----------------------------------------------------------------------+
        |
 +------v--------------------------------------------------------------+
 |          STM32 Firmware  (YardForce Classic 500 / OpenMower HW)     |
 |  IMU  |  Wheel encoders  |  Blade ESC  |  E-stop  |  Rain sensor   |
 +----------------------------------------------------------------------+
```

---

## Features

- **Full autonomous mowing** — plan, mow, dock, charge, resume. No manual intervention required.
- **Multi-area continuous-subpath coverage** — areas are mowed sequentially. `map_server_node` owns area polygons; BT outer loop `GetNextUnmowedArea` iterates areas. Per area, `PlanCoverageArea` calls `map_server_node/get_mowing_area` and forwards it to `mowgli_coverage`'s `plan_coverage` action (F2C v3), returning continuous hole-free `drivable_subpaths` that join headland rings + swaths with forward turn-around arcs. `FollowStrip` drives each sub-path as ONE continuous `FollowCoveragePath` goal (FTC tracks end-to-end). Multi-hole fields are split so each obstacle gap becomes a blade-off Nav2 transit. Resume via pose cursor into the concatenated sub-path. Progress tracked in the `mow_progress` grid and survives restarts, published on `/map_server_node/mow_progress` (OccupancyGrid); the concatenated plan is published on `/coverage/full_plan` for the GUI.
- **fusion_graph localizer (sole, default, unconditional — GTSAM iSAM2)** — A single Pose2 factor graph owns **both** `map→odom` AND `odom→base_footprint`. One node per `node_period_s`; factors are a wheel between-factor (non-holonomic σ_y << σ_x, fed by `/wheel_odom`), a gyro between-factor on yaw, a custom `GnssLeverArmFactor` (analytic Jacobian — the antenna lever-arm rotates with the node's yaw, fed by `/gps/pose_cov`), and unary yaw factors for GPS course-over-ground (`/imu/cog_heading`) and tilt-compensated magnetometer (`/imu/mag_yaw`, when calibrated). Optional LiDAR scan-matching between-factors and loop-closure factors — gated by `use_scan_matching` / `use_loop_closure` — keep the map-frame estimate stable across multi-minute RTK-Float windows. The local-frame dead-reckoning output (`odom→base_footprint` + `/odometry/filtered`) is integrated from the same wheel + gyro stream, replacing the removed `ekf_odom_node`. There is **no SLAM** back-end and **no `use_fusion_graph` toggle** — the graph is always the localizer; the map frame is GPS-anchored. Surface: `/odometry/filtered_map`, `/fusion_graph/diagnostics`, `/fusion_graph/markers`, `/imu/fg_yaw`, services `~/save_graph` + `~/clear_graph`.
- **RTK GPS localization** — UBX protocol. RTK Fixed gives σ ~3 mm. `fusion_graph_node` honors the NavSatFix covariance (via `/gps/pose_cov`) directly and converges to fix precision, with a bounded motion-consistent wrong-fix gate rejecting RTK glitches.
- **BehaviorTree.CPP v4 mission executor** — reactive guards for emergency, boundary, rain, and battery. Automatic rain-stop-dock-wait-resume cycle. Battery-aware dock-charge-undock-resume cycle.
- **Persistent obstacle tracking** — `obstacle_tracker_node` clusters the global costmap and promotes stable clusters to PERSISTENT after age and observation thresholds, publishing them on `/obstacle_tracker/obstacles`. A tracked obstacle only becomes a hard keepout once it is promoted via `map_server_node/promote_obstacle` (the GUI's Tracked Obstacles panel); `map_server_node` then rasterises it into `/keepout_mask` and persists it to `areas.dat`. A wheel-slip dig event raises the same kind of keepout as a *pending* proposal — lethal immediately, persisted only when accepted.
- **Nav2 Kilted** — SmacPlanner2D global planner, RegulatedPurePursuit for transit, FTCController for coverage strips, RotationShimController, `docking_server` (opennav_docking), `collision_monitor`.
- **FTCController Nav2 plugin** — Follow-the-Carrot controller with 3-axis PID for coverage strip following. Provides <10mm lateral accuracy on swaths.
- **Mow progress tracking** — `map_server_node` marks cells as mowed with time-based decay. Visualised as an OccupancyGrid on `/map_server_node/mow_progress`.
- **Keepout mask** — `map_server_node` publishes the Nav2 costmap filter mask (`/keepout_mask` + `/costmap_filter_info`) for mowing boundaries, promoted obstacles and the dock corridor. (The separate speed mask was removed — nothing consumed it.)
- **Cyclone DDS middleware** — `rmw_cyclonedds_cpp` selected in the runtime Docker image for reliable service discovery on ARM without shared memory issues.
- **Docker multi-stage build** — 10 stages from `ros:kilted-ros-base`: four source-builders (GTSAM, Fields2Cover 2.x, Fields2Cover 3.x, ublox msgs) feeding `base → deps → build-interfaces → build → runtime → simulation`. ARM-tested on Rockchip.
- **Foxglove Studio bridge** — WebSocket on port 8765. Pre-built layout at `foxglove/mowgli_sim.json`.
- **MowgliNext GUI integration** — the Go backend talks to ROS2 **only** through `foxglove_bridge` (`ws://<robot-ip>:8765`), plus the teleop relay on 8766.
- **Diagnostics** — `diagnostics_node` publishes nine `diagnostic_msgs/DiagnosticStatus` entries at 1 Hz on `/diagnostics`: Hardware Bridge, Emergency System, Battery, IMU, LiDAR, GPS, Odometry, EKF Map (freshness/attitude of `/odometry/filtered_map`) and Motors. Optional MQTT bridge.

---

## Packages

| Package | Executables | Description |
|---------|-------------|-------------|
| `mowgli_interfaces` | — | All ROS2 msg/srv/action definitions: 15 messages, 14 services, 3 actions |
| `mowgli_hardware` | `hardware_bridge_node` | COBS+CRC-16 serial bridge to STM32. Publishes sensor data + wheel odometry, subscribes to `cmd_vel`, hosts the wheel-slip dig detector |
| `mowgli_bringup` | `empty_static_map_pub.py` `cmd_vel_ws_relay.py` | Launch files, Nav2 config, URDF/xacro, `twist_mux` config, the `mowgli_robot.yaml` template |
| `mowgli_localization` | `wheel_odometry_node` `navsat_to_absolute_pose_node` `localization_monitor_node` `cog_to_imu` `mag_yaw_publisher` `calibrate_imu_yaw_node` `scan_deskew_node` `costmap_scan_filter_node` `gps_dock_detection_node` | GPS absolute pose + `/gps/pose_cov` conversion, localization mode monitor, COG-to-IMU absolute-yaw publisher, magnetometer yaw publisher, scan deskew/filtering, IMU + dock yaw calibration, GPS-based dock detection. The map+odom estimate itself comes from `fusion_graph_node` (`fusion_graph` package); `wheel_odometry_node` is built but not launched — `/wheel_odom` comes from `hardware_bridge_node` |
| `fusion_graph` | `fusion_graph_node` | GTSAM iSAM2 factor-graph localizer — sole map+odom estimator, publishes both `map→odom` and `odom→base_footprint`, LiDAR scan-matching + loop-closure factors |
| `mowgli_behavior` | `behavior_tree_node` | BehaviorTree.CPP v4 executor. Loads `main_tree.xml`. All BT action and condition nodes |
| `mowgli_coverage` | `mowgli_coverage` (node name `coverage_server`) | Fields2Cover v3 coverage planner. Serves `mowgli_interfaces/action/PlanCoverage` on `plan_coverage`: headland rings + serpentine swaths joined into continuous `drivable_subpaths` |
| `mowgli_map` | `map_server_node` `obstacle_tracker_node` | GridMap (`occupancy` + `classification` layers), area CRUD services, keepout filter mask, mow-progress tracking, obstacle promotion and dig proposals. Costmap-clustering obstacle tracker |
| `mowgli_nav2_plugins` | — | Nav2 plugin library loaded by `controller_server`: `FTCController` + `PathProgressGoalChecker` |
| `mowgli_monitoring` | `diagnostics_node` `mqtt_bridge_node` | Diagnostics aggregator publishing nine statuses at 1 Hz. Optional MQTT bridge |
| `mowgli_leds` | `led_ring_node` | Optional WS2812 status ring over SPI (off by default, `led_enabled`). Read-only — outside the motion and blade-safety paths |
| `mowgli_simulation` | `fake_hardware_bridge_node` `sim_actuation_node` (+ `sim_imu_noise.py`, `sim_navsat_rtk_fix.py`, `sim_wheel_slip.py`) | Webots world/PROTO/URDF, the `kinematic_drive.py` driver plugin, and sim-only sensor-degradation helpers (RTK quality is cycled by `sim_navsat_rtk_fix.py`) |

---

## TF Tree

```
map                   (map->odom published by fusion_graph_node —
 +-- odom              absorbs GPS corrections)
      +-- base_footprint (odom->base_footprint also published by fusion_graph_node —
      |                   continuous dead-reckoning, never jumps)
           +-- base_link              (fixed, rear wheel axle)
                +-- left_wheel_link        (continuous joint)
                +-- right_wheel_link       (continuous joint)
                +-- front_left_caster_link (continuous joint)
                +-- front_right_caster_link(continuous joint)
                +-- blade_link             (fixed, under chassis)
                +-- imu_link               (fixed — offset from mowgli_robot.yaml)
                +-- gps_link               (fixed — offset from mowgli_robot.yaml)
                +-- lidar_link             (fixed — offset from mowgli_robot.yaml)
```

Frame conventions follow REP-105: `map` (global, GPS-anchored via fixed datum), `odom` (continuous local, drift-only), `base_footprint` (robot frame for Nav2), `base_link` (rear wheel axis, OpenMower convention).

`base_link` is placed at the centre of the rear drive wheel axis at wheel axle height. The chassis geometric centre sits `chassis_center_x` (default 0.18 m) forward of `base_link`. `base_footprint` is directly below `base_link` on the ground plane. The Nav2 footprint polygon is computed at launch from `chassis_length`, `chassis_width`, and `chassis_center_x` read from `mowgli_robot.yaml`.

`fusion_graph_node` publishes **both** transforms: `odom→base_footprint` (continuous dead-reckoning from wheels + gyro) and `map→odom` (absorbing GPS corrections once a fix arrives). It is the sole, unconditional localizer — there is no SLAM back-end and no alternate map-frame backend to switch to.

---

## Key Topics and Services

The tables below are the operator-facing subset. The exhaustive, generated index of every
topic, service, action and TF frame — with the file and line that creates it and everything
that consumes it — is [`docs/claude/ros-interfaces.md`](../docs/claude/ros-interfaces.md).

### Published Topics

| Topic | Type | Source | Rate |
|-------|------|--------|------|
| `/hardware_bridge/status` | `mowgli_interfaces/msg/Status` | `hardware_bridge_node` | ~10 Hz |
| `/hardware_bridge/power` | `mowgli_interfaces/msg/Power` | `hardware_bridge_node` | ~1 Hz |
| `/hardware_bridge/emergency` | `mowgli_interfaces/msg/Emergency` | `hardware_bridge_node` | ~1 Hz |
| `/imu/data` | `sensor_msgs/msg/Imu` | `hardware_bridge_node` (remapped) | ~50 Hz |
| `/wheel_odom` | `nav_msgs/msg/Odometry` | `hardware_bridge_node` (`~/wheel_odom`, remapped) | ~20 Hz |
| `/gps/absolute_pose` | `mowgli_interfaces/msg/AbsolutePose` | `navsat_to_absolute_pose_node` | ~5 Hz |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | `fusion_graph_node` (local, odom frame — dead reckoning) | node rate |
| `/odometry/filtered_map` | `nav_msgs/msg/Odometry` | `fusion_graph_node` (global, map frame) | node rate |
| `/fusion_graph/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | `fusion_graph_node` | ~1 Hz |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR driver or Webots bridge | ~10 Hz |
| `/behavior_tree_node/high_level_status` | `mowgli_interfaces/msg/HighLevelStatus` | `behavior_tree_node` | on BT tick |
| `/coverage/full_plan` | `nav_msgs/msg/Path` | `behavior_tree_node` (`PlanCoverageArea`) | on plan, transient-local |
| `/map_server_node/mow_progress` | `nav_msgs/msg/OccupancyGrid` | `map_server_node` | configurable, transient-local |
| `/keepout_mask` | `nav_msgs/msg/OccupancyGrid` | `map_server_node` | on change, transient-local |
| `/map_server_node/docking_pose` | `geometry_msgs/msg/PoseStamped` | `map_server_node` | transient-local |
| `/map_server_node/boundary_violation` | `std_msgs/msg/Bool` | `map_server_node` | on change |
| `/map_server_node/lethal_boundary_violation` | `std_msgs/msg/Bool` | `map_server_node` | on change |
| `/obstacle_tracker/obstacles` | `mowgli_interfaces/msg/ObstacleArray` | `obstacle_tracker_node` | ~1 Hz |
| `/hardware_bridge/dig_event` | `mowgli_interfaces/msg/DigEvent` | `hardware_bridge_node` | on event, transient-local |
| `/mowgli/localization/mode` | `std_msgs/msg/String` | `localization_monitor_node` | 10 Hz, latched |
| `/mowgli/localization/mode_id` | `std_msgs/msg/Int32` | `localization_monitor_node` | 10 Hz, latched |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | `diagnostics_node` | ~1 Hz |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | `hardware_bridge_node` | ~1 Hz |

### Services

| Service | Type | Server |
|---------|------|--------|
| `/hardware_bridge/mower_control` | `mowgli_interfaces/srv/MowerControl` | `hardware_bridge_node` |
| `/hardware_bridge/emergency_stop` | `mowgli_interfaces/srv/EmergencyStop` | `hardware_bridge_node` |
| `/hardware_bridge/reboot_board` | `std_srvs/srv/Trigger` | `hardware_bridge_node` |
| `/behavior_tree_node/high_level_control` | `mowgli_interfaces/srv/HighLevelControl` | `behavior_tree_node` |
| `/behavior_tree_node/start_in_area` | `mowgli_interfaces/srv/StartInArea` | `behavior_tree_node` |
| `/behavior_tree_node/clear_coverage_resume` | `std_srvs/srv/Trigger` | `behavior_tree_node` |
| `/map_server_node/add_area` | `mowgli_interfaces/srv/AddMowingArea` | `map_server_node` |
| `/map_server_node/get_mowing_area` | `mowgli_interfaces/srv/GetMowingArea` | `map_server_node` |
| `/map_server_node/set_docking_point` | `mowgli_interfaces/srv/SetDockingPoint` | `map_server_node` |
| `/map_server_node/promote_obstacle` | `mowgli_interfaces/srv/PromoteObstacle` | `map_server_node` |
| `/map_server_node/discard_obstacle` | `mowgli_interfaces/srv/ClearObstacle` | `map_server_node` |
| `/map_server_node/get_recovery_point` | `mowgli_interfaces/srv/GetRecoveryPoint` | `map_server_node` |
| `/map_server_node/save_map` · `load_map` · `clear_map` | `std_srvs/srv/Trigger` | `map_server_node` |
| `/map_server_node/save_areas` · `load_areas` | `std_srvs/srv/Trigger` | `map_server_node` |
| `/obstacle_tracker/save` · `load` · `clear_all` | `std_srvs/srv/Trigger` | `obstacle_tracker_node` |
| `/fusion_graph_node/save_graph` · `clear_graph` | `std_srvs/srv/Trigger` | `fusion_graph_node` |

### Actions

| Action | Type | Server |
|--------|------|--------|
| `/plan_coverage` | `mowgli_interfaces/action/PlanCoverage` | `coverage_server` (`mowgli_coverage`) |
| `/dock_robot` | `opennav_docking_msgs/action/DockRobot` | `docking_server` (opennav_docking) |
| `/backup` | `nav2_msgs/action/BackUp` | Nav2 `behavior_server` (used for undock; `UndockRobot` is not used — Invariant 10) |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 `bt_navigator` |
| `/follow_path` | `nav2_msgs/action/FollowPath` | Nav2 `controller_server` (coverage uses `controller_id="FollowCoveragePath"`) |
| `/calibrate_imu_yaw_node/calibrate_dock` | `mowgli_interfaces/action/CalibrateDock` | `calibrate_imu_yaw_node` |

### Sending High-Level Commands

```bash
# Start mowing
ros2 service call /behavior_tree_node/high_level_control \
  mowgli_interfaces/srv/HighLevelControl "{command: 1}"

# Return to dock
ros2 service call /behavior_tree_node/high_level_control \
  mowgli_interfaces/srv/HighLevelControl "{command: 2}"

# Start area recording (COMMAND_RECORD_AREA)
ros2 service call /behavior_tree_node/high_level_control \
  mowgli_interfaces/srv/HighLevelControl "{command: 3}"
```

| Command | Constant | Effect |
|---------|----------|--------|
| `1` | `COMMAND_START` | Begin mowing sequence |
| `2` | `COMMAND_HOME` | Navigate to dock and charge |
| `3` | `COMMAND_RECORD_AREA` (`COMMAND_S1`) | Start area boundary recording |
| `4` | `COMMAND_S2` | "Mow next area" — normalised to `COMMAND_START` by `behavior_tree_node` |
| `5` | `COMMAND_RECORD_FINISH` | Finish recording, save polygon |
| `6` | `COMMAND_RECORD_CANCEL` | Cancel recording, discard trajectory |
| `7` | `COMMAND_MANUAL_MOW` | Enter manual mowing mode (teleop + blade) |
| `8` | `COMMAND_STOP` | Stop in place / hold — does **not** drive to the dock |
| `254` | `COMMAND_RESET_EMERGENCY` | Reset latched emergency |
| `255` | `COMMAND_DELETE_MAPS` | Delete all maps |

### AbsolutePose GPS Flags

| Flag | Value | Meaning |
|------|-------|---------|
| `FLAG_GPS_RTK` | `1` | GPS fix present (does not imply centimetre accuracy) |
| `FLAG_GPS_RTK_FIXED` | `2` | RTK fixed — centimetre accuracy |
| `FLAG_GPS_RTK_FLOAT` | `4` | RTK float — decimetre accuracy |
| `FLAG_GPS_DEAD_RECKONING` | `8` | Dead reckoning fallback |

### Localization Modes

`localization_monitor_node` publishes the current mode on `/mowgli/localization/mode` (string) and `/mowgli/localization/mode_id` (int32), both latched (transient-local) at `publish_rate` (default 10 Hz). The mode is a pure function of GPS freshness and the `AbsolutePose` RTK flags — it does **not** inspect the `fusion_graph` estimate. First matching rule wins:

| mode_id | mode string | Condition | Typical accuracy |
|---------|-------------|-----------|-----------------|
| `3` | `RTK_FIXED` | GPS fresh AND `FLAG_GPS_RTK_FIXED` | σ ~3 mm (bounded by GPS σ + graph innovation) |
| `2` | `RTK_FLOAT` | GPS fresh AND RTK active (float or DGPS) | ~5–20 cm; bounded further when `use_scan_matching` is on with a LiDAR |
| `1` | `GPS_ONLY` | GPS fresh but unaugmented | ~0.5–2 m |
| `0` | `DEAD_RECKONING` | GPS stale — relying on wheels + IMU | drifts ~1 %/m; with `use_scan_matching` and a LiDAR, scan-matching factors keep the map-frame estimate bounded |

The BT's own degraded-localization guard is separate: `LocalizationGuard` keys on GNSS health, not on this topic.

---

## Configuration

### mowgli_robot.yaml — Central Robot Configuration

`src/mowgli_bringup/config/mowgli_robot.yaml` is the **template of defaults** for all physical, operational, and safety parameters — the versioned source of truth for every value a robot has not explicitly overridden.

The *installed* file bind-mounted at `/ros2_ws/config/mowgli_robot.yaml` (seeded from `install/config/mowgli/mowgli_robot.yaml`) is deliberately **sparse**: it holds only install-time choices (datum, NTRIP, `lidar_enabled`, GNSS hardware, `mower_model`), calibration outputs (dock pose, `ticks_per_meter`, PID gains, IMU/mag yaw) and genuine overrides. At launch, `robot_config_util.load_robot_params()` **deep-merges the installed file over the template**, so an absent key falls through to its template default and nodes always receive a complete parameter set. Changes take effect on container restart without rebuilding the image; to restore a default, **delete** the key from the installed file rather than copying the template value into it.

All dimensions are in **metres**, angles in **radians**, speeds in **m/s**. The tables below list template defaults; the full generated index of every key, its default and its consumers is [`docs/claude/parameters.md`](../docs/claude/parameters.md).

#### Hardware / Physical

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mower_model` | `"YardForce500"` | Robot model identifier |
| `chassis_length` | `0.60` | Chassis length |
| `chassis_width` | `0.40` | Chassis width |
| `chassis_height` | `0.19` | Chassis height |
| `chassis_mass_kg` | `8.76` | Total robot mass |
| `wheel_radius` | `0.04475` | Drive wheel radius |
| `wheel_track` | `0.325` | Drive wheel centre-to-centre track width. MUST match the firmware `WHEEL_BASE` |
| `wheel_x_offset` | `0.0` | Drive wheel x offset from `base_link` (0 = at wheel axis) |
| `chassis_center_x` | `0.18` | Chassis geometric centre forward of wheel axis |
| `ticks_per_meter` | `399.0` | Encoder scale — read by `hardware_bridge_node` and pushed down to the STM32 at connect (calibration output) |
| `max_mps` | `0.5` | Runtime wheel-speed cap pushed to the STM32; firmware clamps it to its compiled ceiling |
| `caster_radius` | `0.03` | Front caster radius |
| `blade_radius` | `0.09` | Cutting blade disc radius |
| `tool_width` | `0.18` | Effective cut width (2 x blade_radius). Single source for `map_server.tool_width` and `coverage_server.operation_width` |

#### Sensor Positions (relative to base_link)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `lidar_x` | `0.0` | LiDAR forward offset |
| `lidar_y` | `0.024` | LiDAR lateral offset |
| `lidar_z` | `0.30` | LiDAR height offset |
| `lidar_yaw` | `3.1408` | LiDAR heading rotation (~π — mounted back-to-front) |
| `imu_x` | `0.18` | IMU forward offset |
| `imu_y` | `0.0` | IMU lateral offset |
| `imu_z` | `0.095` | IMU height offset |
| `gps_x` | `0.3` | GPS antenna forward offset |
| `gps_y` | `0.0` | GPS antenna lateral offset (feeds the `GnssLeverArmFactor`) |
| `gps_z` | `0.20` | GPS antenna height offset |

All sensor positions drive both the URDF (TF frames) and the Nav2 footprint polygon. The LiDAR defaults above are this project's reference chassis, not OpenMower values — measure your own mount.

#### GPS / Positioning

| Parameter | Default | Description |
|-----------|---------|-------------|
| `datum_lat` | `0.0` | Map origin latitude — **set per site** |
| `datum_lon` | `0.0` | Map origin longitude — **set per site** |
| `gnss_receiver_family` | `"auto"` | Universal GNSS receiver family |
| `gnss_serial_device` | `"/dev/ttyAMA4"` | Universal GNSS serial device |
| `gnss_serial_baud` | `921600` | Universal GNSS serial baud |
| `gps_wait_after_undock_sec` | `10.0` | Wait for RTK fix after undocking |
| `ntrip_enabled` | `false` | Enable NTRIP RTK correction stream |
| `ntrip_host` | `""` | NTRIP caster hostname |
| `ntrip_port` | `2101` | NTRIP caster port |
| `ntrip_mountpoint` | `""` | NTRIP mountpoint |

#### Battery

| Parameter | Default | Description |
|-----------|---------|-------------|
| `battery_full_voltage` | `28.0` | Fully charged threshold (V) |
| `battery_empty_voltage` | `24.0` | Start docking (V) |
| `battery_critical_voltage` | `23.0` | Immediate dock (V) |
| `battery_full_percent` | `95.0` | Resume mowing above this (%) |
| `battery_low_percent` | `20.0` | Start docking (%) |
| `battery_critical_percent` | `10.0` | Emergency dock — BT guard (%) |
| `battery_critical_recovery_percent` | `30.0` | Leave critical-battery state after recharge — hysteresis upper bound, must be > `battery_critical_percent` (%) |

#### Mowing

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mowing_speed` | `0.20` | Speed during coverage paths (m/s) |
| `transit_speed` | `0.20` | Speed during point-to-point navigation (m/s) → RPP `desired_linear_vel` |
| `swath_overlap` | `0.02` | F2C swath spacing = `tool_width − swath_overlap`, so adjacent swaths overlap |
| `num_headland_passes` | `2` | Concentric perimeter passes (`0` = auto) |
| `mow_angle_deg` | `-1.0` | Swath angle in degrees; negative = auto (swath-count-minimising) |
| `min_turning_radius` | `0.15` | Hard floor on every forward turn-around / corner fillet; injected into `coverage_server.min_turning_radius` |
| `connector_turn_radius` | `0.18` | Nominal radius of the swath-to-swath turn-around arc (floored at `min_turning_radius`) |
| `path_spacing` | `0.18` | **Informational only** — swath spacing is driven by `tool_width − swath_overlap`; no node reads this key |
| `headland_width` | `0.18` | **Currently inert** — the effective headland comes from `num_headland_passes` and the chassis inset |

#### Docking

| Parameter | Default | Description |
|-----------|---------|-------------|
| `dock_pose_x` | `0.0` | Dock position in map frame — **set per site** |
| `dock_pose_y` | `0.0` | Dock position in map frame — **set per site** |
| `dock_pose_yaw` | `0.0` | Dock heading in map frame, ENU rad — **set per site** via the GUI "Set Docking Point" or the undock/dock yaw calibration |
| `undock_distance` | `1.5` | Distance to reverse when undocking (must be > 0.5 m for `CalibrateHeadingFromUndock`) |
| `undock_speed` | `0.16` | Reverse speed during undocking (m/s) |
| `dock_approach_distance` | `1.5` | Straight-in runway behind the dock → `simple_charging_dock.staging_x_offset` |
| `dock_approach_overshoot` | `0.05` | Forward overshoot past the calibrated dock pose so the contacts seat |
| `dock_max_retries` | `3` | Maximum docking attempts (injected at launch into nav2 `docking_server.max_retries`) |
| `dock_charging_threshold` | `0.3` | Charging current (A) at which `SimpleChargingDock` calls the cradle reached |

#### Rain

| Parameter | Default | Description |
|-----------|---------|-------------|
| `rain_mode` | `2` | 0=ignore, 1=dock, 2=dock_until_dry, 3=pause_auto |
| `rain_delay_minutes` | `30.0` | Wait after rain stops before resuming |
| `rain_debounce_sec` | `10.0` | Rain must persist this long to trigger |

### Coverage Parameters

Coverage planning is handled by `coverage_server` (package `mowgli_coverage`, Fields2Cover v3) through the `plan_coverage` action; `map_server_node` only owns the area polygons and the mow-progress grid. Mowing parameters live in `src/mowgli_bringup/config/mowgli_robot.yaml` (see Mowing section above) and are **injected into the coverage server at launch** by `navigation.launch.py` — notably `operation_width = tool_width − swath_overlap` (the swath spacing), `min_turning_radius`, `connector_turn_radius` and `num_headland_passes`. `map_server.tool_width` (the mow-progress stamp radius) is injected separately by `full_system.launch.py` from the same `tool_width`.

### Key Config File Reference

| File | Controls |
|------|----------|
| `src/mowgli_bringup/config/mowgli_robot.yaml` | Template defaults for all physical, operational, and safety parameters (merged under the sparse installed config) |
| `src/mowgli_bringup/config/nav2_params_base.yaml` | Nav2 controllers, planner, costmaps, collision monitor (shared base, deep-merged with nav2_params_lidar.yaml or nav2_params_no_lidar.yaml) |
| `src/fusion_graph/config/fusion_graph.yaml` | `fusion_graph` localizer tuning (factor noise models, node cadence, scan-matching / loop-closure) |
| `src/mowgli_bringup/config/hardware_bridge.yaml` | Serial bridge (port/baud/rate, IMU cal sample count) |
| `src/mowgli_bringup/config/twist_mux.yaml` | `cmd_vel` multiplexer priorities and timeouts |
| `src/mowgli_map/config/map_server.yaml` | Map resolution/size, areas + mow-progress persistence paths |
| `src/mowgli_map/config/obstacle_tracker.yaml` | Costmap-cluster obstacle detection thresholds |
| `src/mowgli_behavior/config/behavior_tree.yaml` | BT node parameters |
| `src/mowgli_behavior/trees/main_tree.xml` | Full BT structure: guards, sequences, recovery |

---

## Building

The devcontainer post-create hook prepares the ROS2 workspace and links package
roots, but it does not build the full workspace by default. That keeps optional
full-stack coverage dependencies from blocking container startup.

### Prerequisites

- ROS2 Kilted on Ubuntu 24.04
- `colcon`, `rosdep`, `xacro` (`python3-colcon-common-extensions`, `python3-rosdep`)

### Build the Workspace

```bash
source /opt/ros/kilted/setup.bash
cd /path/to/mowgli-ros2

rosdep update --rosdistro kilted
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src --rosdistro kilted -y

colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers $(nproc) \
  --event-handlers console_cohesion+

source install/setup.bash
```

`universal_gnss_ros2` is vendored via the
`ros2/src/external/universal-gnss` git submodule, not installed via apt. The
main `mowgli-ros2` runtime no longer launches Universal GNSS directly; the
`mowgli-gps` sidecar owns that runtime path. The vendored package remains in
this workspace during the migration so ROS2 CI and local development can stay
in sync with the sidecar code until the final cleanup PR removes it.

The sidecar package under `tools/motor/` is linked or copied into the colcon
workspace as `mowgli_tools` and built into the `mowgli-ros2` runtime image.
This is intentional: the MowgliNext GUI Drive Motor calibration assistants call
`ros2 run mowgli_tools tune_drive_pid` inside the running ROS2 container, so
the runtime workspace must ship that package and its Python entrypoint.

If you are actively developing that upstream repo separately, set
`UNIVERSAL_GNSS_PATH=/path/to/universal-gnss` and the workspace sync helpers
will prefer that checkout over the vendored submodule.

### Running Tests

```bash
source /opt/ros/kilted/setup.bash && source install/setup.bash
colcon test --return-code-on-test-failure
colcon test-result --verbose
```

### Makefile Shortcuts

```bash
make build-dev       # focused dev set: interfaces, localization, GNSS, bringup
make build-full      # full linked workspace
make build           # alias for build-full
make build-pkg PKG=x # one package (--packages-up-to)
make build-debug     # colcon build (Debug)
make test            # colcon test + test-result
make clean           # remove build/ install/ log/
make sim             # sim-stop, then headless Webots (Foxglove ws://localhost:8765)
make sim-stop        # kill Webots/ROS2 processes, wipe DDS shm + Webots IPC
make e2e-test        # sim-stop + build + sim + 90 s + src/e2e_test.py
make e2e-test-no-lidar  # GPS-only variant (src/e2e_test_no_lidar.py)
make format          # clang-format all C++ files in-place
make format-check    # verify formatting without modifying files
make lint            # cppcheck + cpplint
```

`make format` / `format-check` / `lint` glob all of `src/`, **including** the vendored
`opennav_coverage` submodule. Use `./scripts/format.sh [--check]` to match CI and leave the
submodule alone. `make sim` and `make e2e-test` export `DISPLAY=:99` but do **not** start
Xvfb — start it yourself (compose does) or Webots fails to open a display.

The upstream `opennav_coverage` submodule is linked as
`opennav_coverage_msgs` by default for action definitions. Its server, BT,
demo, navigator, and row-coverage packages are optional full-stack packages and
require Fields2Cover to be installed. To include them in local package linking
and builds, run:

```bash
INCLUDE_OPENNAV_COVERAGE_STACK=1 ./scripts/sync_workspace_packages.sh
INCLUDE_OPENNAV_COVERAGE_STACK=1 ./scripts/build.sh
```

---

## Docker Deployment

### Image Stages

The Docker build context is the **repository root**, not `ros2/` (the Dockerfile COPYs `ros2/…` and `tools/motor/…`).

| Stage | From | Contents |
|-------|------|----------|
| `gtsam-builder` · `fields2cover-builder` · `fields2cover-v3-builder` · `ublox-msgs-builder` | `ros:kilted-ros-base` | Source builds of GTSAM 4.3a1, Fields2Cover 2.x and 3.x, and the forked `ublox_ubx_msgs`, copied into later stages |
| `base` | `ros:kilted-ros-base` | All apt runtime deps: Nav2, foxglove-bridge, twist_mux, BehaviorTree.CPP, grid_map, opennav_docking, Cyclone DDS + the source-built GTSAM / Fields2Cover |
| `deps` | `base` | Build tools, rosdep resolution over the copied `package.xml`/`CMakeLists.txt` set |
| `build-interfaces` | `deps` | `mowgli_interfaces` compiled only (cached layer, rarely rebuilt) |
| `build` | `build-interfaces` | All remaining packages compiled; `colcon test … \|\| true`, so image builds never fail on unit tests (the gate is CI) |
| `runtime` | `base` | Compiled install tree + launch/config overlays. Sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| `simulation` | `runtime` | Webots + TigerVNC + noVNC for GUI access (Linux amd64 only) |

### Docker Compose Services

`docker/docker-compose.simulation.yaml` (all three built from the `simulation` target):

| Service | Container | Ports | Use case |
|---------|-----------|-------|----------|
| `simulation` | `mowgli_simulation` | `8765` | Headless Webots + full nav stack (host network) |
| `dev-sim` | `mowgli_dev_sim` | `8765` | Same, with config/launch/trees bind-mounted for live editing |
| `simulation-gui` | `mowgli_sim_gui` | `8765`, `6080` | Webots GUI via noVNC |

Real-hardware deployment does not live here — the installer's stack owns it, as the `mowgli` service in `install/compose/docker-compose.base.yml` plus the modular sidecar overlays (`docker-compose.gps.yml`, `docker-compose.lidar-*.yml`, `docker-compose.gui.yml`, …).

The runtime image also includes the `mowgli_tools` package from
`tools/motor/`. After sourcing:

```bash
source /opt/ros/kilted/setup.bash
source /ros2_ws/install/setup.bash
ros2 pkg list | grep mowgli_tools
ros2 run mowgli_tools tune_drive_pid --help
```

the `tune_drive_pid` entrypoint should be available inside `mowgli-ros2`. The
GUI Drive Motor assistant depends on that command path. Its movement path now
uses a dedicated tuning lane: the tuner publishes `TwistStamped` commands on
`/cmd_vel_tuning`, `twist_mux` forwards them to `/cmd_vel`, and
`hardware_bridge` passes them to the STM32 once the BT is in `RECORDING`.

### Running Hardware

Real-robot bring-up is driven by the installer stack (`install/mowglinext.sh` writes an
`.env` + the compose overlay set). To run the ROS2 service on its own:

```bash
# Requires the /dev/mowgli USB serial symlink to the STM32 board
docker compose -f install/compose/docker-compose.base.yml up mowgli
```

### Running Webots Simulation

From a clean checkout, initialise submodules before building the GUI image:

```bash
git clone --recurse-submodules https://github.com/mowglinext/mowglinext.git
cd mowglinext
git submodule update --init --recursive
docker compose -f docker/docker-compose.simulation.yaml build simulation-gui
docker compose -f docker/docker-compose.simulation.yaml up -d simulation-gui
```

Open the Webots display through noVNC at
`http://localhost:6080/vnc.html`, and connect Foxglove Studio to
`ws://localhost:8765`.

On Windows Docker Desktop the GUI can use CPU software rendering. This is
functional but may be slower than native rendering. GPU acceleration has not
been verified by this project documentation. The GUI workflow has been
verified on Windows Docker Desktop; native Linux, macOS, ARM, and Raspberry Pi
GUI simulation have not been verified here. The normal runtime image remains
architecture-neutral, while the Webots simulation image requires the official
Linux amd64 Webots package.

To stop the GUI service and inspect it:

```bash
docker compose -f docker/docker-compose.simulation.yaml logs --tail=300 simulation-gui
docker compose -f docker/docker-compose.simulation.yaml down
```

For local ROS 2 development on a Linux ROS workspace, the headless Make target
is available from `ros2/`:

```bash
cd ros2
make sim
```

### Development Workflow

```bash
# Start the dev simulation (config/launch/trees bind-mounted from the host)
docker compose -f docker/docker-compose.simulation.yaml up -d dev-sim

# Open an interactive shell inside the running container:
docker exec -it mowgli_dev_sim bash

# Inside the devcontainer / on a Linux ROS workspace, rebuild from ros2/:
make build                              # full workspace
make build-pkg PKG=mowgli_behavior      # one package (--packages-up-to)

# Restart the simulation to pick up rebuilt code:
docker compose -f docker/docker-compose.simulation.yaml restart dev-sim
```

Config, launch and BT-tree files under `src/` are bind-mounted into `dev-sim`, so they are live-edited without rebuilding — just restart the container.

### Deploying to the Robot

```bash
# rsync compiled install tree to robot, then restart systemd service
make deploy ROBOT_HOST=mowgli.local ROBOT_USER=pi

# Pull mowing area / dock / mow-progress persistence from the robot
make backup-maps ROBOT_HOST=mowgli.local ROBOT_USER=pi
```

A `systemd/mowgli.service` unit file is provided for running the stack as a system service on the robot. Install it to `/etc/systemd/system/` and enable it with `systemctl enable mowgli`.

---

## Launch Files

### Two-Tier Structure

**Tier 1 — `mowgli.launch.py`** (hardware layer):

- `robot_state_publisher` — processes URDF xacro with all dimensions from `mowgli_robot.yaml`, publishes `/robot_description` and static TF frames
- `hardware_bridge_node` — COBS serial bridge to STM32
- `twist_mux` — priority multiplexer: navigation `/cmd_vel_monitored` (10) < docking `/cmd_vel_docking` (15) < teleop `/cmd_vel_teleop` (20) < drive tuning `/cmd_vel_tuning` (30) < emergency `/cmd_vel_emergency` (100). There is deliberately **no `locks:` block** — the e-stop is the firmware latch reached through `/hardware_bridge/emergency_stop`, not a mux lock

**Tier 2 — `full_system.launch.py`** (includes Tier 1 + full navigation stack):

- `navigation.launch.py` — `fusion_graph_node` (sole map+odom localizer, unconditional), the `cog_to_imu` + `mag_yaw_publisher` yaw helpers, `scan_deskew_node` + `costmap_scan_filter_node`, `gps_dock_detection_node`, and Nav2 bringup (including `coverage_server` and `docking_server`)
- `behavior_tree_node` — BT mission executor
- `map_server_node` + `obstacle_tracker_node` — area management and obstacle tracking
- `navsat_to_absolute_pose_node`, `localization_monitor_node`, `calibrate_imu_yaw_node` — GPS pose, localization mode, IMU/dock yaw calibration (`/wheel_odom` comes from `hardware_bridge_node`, not a separate odometry node)
- `diagnostics_node` — robot health monitoring
- `mqtt_bridge_node` — optional, `enable_mqtt:=true`
- `foxglove_bridge` — enabled by default on port 8765; the GUI's only ROS2 link
- `cmd_vel_ws_relay.py` — teleop WebSocket relay on port 8766 (`TwistStamped` JSON → `/cmd_vel_teleop`)
- `led_ring_node` — optional WS2812 status ring, `led_enabled`

### Launch Arguments

**`full_system.launch.py`:**

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use the simulation clock |
| `serial_port` | `/dev/mowgli` | Hardware bridge serial device |
| `use_lidar` | from `mowgli_robot.yaml` `lidar_enabled` | Launch LiDAR-dependent nodes (fusion_graph scan-matching + loop-closure, obstacle layer, collision-monitor scan). `false` runs a GPS-only configuration. The `LIDAR_ENABLED` env var is **not** consulted |
| `use_obstacle_tracker` | `true` | Launch `obstacle_tracker_node` (also requires `use_lidar`) |
| `led_enabled` | from `mowgli_robot.yaml` | Launch the WS2812 status ring (`mowgli_leds`) |
| `enable_mqtt` | `false` | Launch MQTT bridge node |
| `enable_foxglove` | `true` | Launch Foxglove Bridge |
| `foxglove_port` | `8765` | Foxglove Bridge WebSocket port |

**`navigation.launch.py`** (main arguments — see the file for the full fusion-graph tuning set):

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use the simulation clock |
| `use_lidar` | from `mowgli_robot.yaml` | Launch LiDAR-aware Nav2 config (base params + `nav2_params_lidar.yaml` overlay). `false` uses the `nav2_params_no_lidar.yaml` overlay and forces scan-matching / loop-closure off |
| `use_scan_matching` | from `mowgli_robot.yaml` | Add LiDAR scan-matching between-factors to the `fusion_graph` graph (ANDed with `use_lidar`) |
| `use_loop_closure` | from `mowgli_robot.yaml` | Add loop-closure factors to the `fusion_graph` graph (also gated on a persisted graph file existing) |
| `use_magnetometer` | from `mowgli_robot.yaml` | Enable magnetometer yaw fusion |
| `use_gps_dock_detection` | from `mowgli_robot.yaml` (`true`) | Approach the dock off RTK-Fixed `/gps/absolute_pose` instead of the map→odom TF |
| `fusion_graph_node_period_s` | from `mowgli_robot.yaml` | Factor-graph node cadence (seconds) |

### Simulation Launch Files

| File | Description |
|------|-------------|
| `mowgli_bringup/sim_full_system.launch.py` | Full stack against Webots. Does **not** go through `mowgli.launch.py` — it launches its own `twist_mux`, `fake_hardware_bridge_node` and sim-only sensor helpers. The `world` argument selects the `.wbt` file (default `mowgli_garden.wbt`); the `headless` argument is accepted for compatibility but **ignored** — Webots runs in `fast` mode and headless operation comes from the caller supplying an Xvfb `DISPLAY` |
| `mowgli_simulation/webots_minimal.launch.py` | Webots driver + `robot_state_publisher` + `ros2_control` spawners only; included by the above |

---

## Behavior Tree

The mission executor loads `src/mowgli_behavior/trees/main_tree.xml` using BehaviorTree.CPP v4. The root is a `ReactiveSequence`, which means all guards are re-evaluated on every tick before the child can proceed.

### Guard Priority (outer to inner, highest to lowest)

```
ReactiveSequence (Root)
  |
  +-- EmergencyGuard
  |     Polls /hardware_bridge/emergency every tick.
  |     On emergency: disable blade, stop, publish EMERGENCY.
  |     Auto-sends ResetEmergency when charging on the dock (Invariant 9).
  |
  +-- SensorSafetyGuard
  |     Stale /scan (IsScanStale) or a sustained collision stop ->
  |     disable blade and hold. Charging / docking / recording / manual exempt.
  |
  +-- BoundaryGuard
  |     Polls /map_server_node/boundary_violation and
  |     /map_server_node/lethal_boundary_violation every tick.
  |     Soft violation: disable blade, stop, navigate back inside;
  |     lethal violation: escalate to emergency stop. Docking transits exempt.
  |
  +-- LocalizationGuard
  |     Keyed on GNSS health (accuracy / RTK mode, with fused sigma as a
  |     secondary signal) — NOT on fusion_graph pivot covariance.
  |     Degraded -> disable blade and hold until it recovers. Docking exempt.
  |
  +-- GPSModeSelector
  |     RTK fixed  -> SetNavMode("precise")   full speed
  |     Otherwise  -> SetNavMode("degraded")  half speed + wider inflation
  |
  +-- Nav2ResumeGuard
  |     Re-activates the Nav2 lifecycle IdleSequence PAUSEd on the dock
  |     (only when idle_nav2_suspend is enabled; a no-op otherwise).
  |
  +-- MainLogic (Fallback — tried in order)
        |
        +-- CriticalBatteryDock     battery below battery_critical_percent
        |                           (or battery_critical_voltage) -> dock now
        +-- MowingSequence          COMMAND_START = 1
        +-- HomeSequence            COMMAND_HOME = 2
        +-- RecordingSequence       COMMAND_RECORD_AREA = 3
        +-- ManualMowingSequence    COMMAND_MANUAL_MOW = 7
        +-- StopHoldSequence        COMMAND_STOP = 8
        +-- IdleSequence            no command -> wait
```

### Mowing Sequence (COMMAND_START)

```
MowingSequence
  PublishHighLevelStatus("PREFLIGHT_CHECK")
  Nav2ReadyOrProceed            poll Nav2Active up to 60x1s, then proceed anyway
  PublishHighLevelStatus("UNDOCKING")
  UndockOrSkip
    NotDockedBranch (not charging — robot already in field):
      SeedYawFromMotion         drive 1 m to seed the map-frame heading
      ClearCostmap
    UndockSequence (charging):
      PreFlightCheck            battery / GPS fix / TF, retried up to 120x1s
      RecordUndockStart         snapshot GPS position
      BackUp                    Nav2 BackUp action for undocking
      WaitForGpsFix(20s, RTK-Fixed)
      CalibrateHeadingFromUndock compute map heading from GPS displacement
    UndockFailed:               stop, publish UNDOCK_FAILED, ClearCommand
  WasRainingAtStart             record rain state at session start
  WaitForDuration(3s)           wait for obstacle detection
  PublishHighLevelStatus("MOWING")

  MowingCommandGuard (ReactiveSequence — aborts if command changes)
    IsCommand(1)

    StripCoverageWithRecovery (Fallback)
      |
      StripGuards (ReactiveSequence)
        |
        +-- RainGuard
        |     IsNewRain? (rain started DURING mowing, not before)
        |     -> disable blade, stop, dock, wait up to 12h for rain to clear,
        |        wait rain_delay_minutes, undock, resume
        |
        +-- BatteryGuard
        |     NeedsDocking(battery_low_percent)?
        |     -> disable blade, stop, dock, wait until battery_full_percent
        |        (IsChargingProgressing detects a stalled charger after 30 min;
        |         CHARGER_FAILED aborts), undock, resume
        |
        +-- AreaLoop (Repeat x100)
              GetNextUnmowedArea            find next area with remaining coverage
              PlanCoverageArea              plan_coverage -> drivable_subpaths
              TransitToStrip                blade off, Nav2 transit to the start pose
              FollowStripRetry (x5)
                FollowStrip                 ONE FollowCoveragePath goal per sub-path;
                                            FTC tracks it end-to-end
                or recovery, chosen by cause:
                  EscapeStartBlocked        start pose occupied -> escape + retry
                  StuckBackoff              IsObstacleStuck -> back up 0.40 m
                  DynamicObstacleSkip       recent collision stop -> pause + retry
              AreaUnreachable               give up on this area, continue the loop
              (GetNextUnmowedArea returns FAILURE when all areas complete)

      CoverageEnded (Fallback)
        CoverageCompleteDock (IsCoverageComplete):
          SetMowerEnabled(false), MOWING_COMPLETE, SaveObstacles, ClearCostmap,
          DockRobot, IDLE_DOCKED, EndSession, ClearCommand
        FailedCoverageDock:
          SetMowerEnabled(false), COVERAGE_FAILED_DOCKING, ClearCostmap,
          DockRobot, IDLE_DOCKED, EndSession, ClearCommand
```

There is no SLAM map to save any more — the old `SaveSlamMap` step was removed with the SLAM
back-end; `fusion_graph_node` persists its own factor graph through `~/save_graph`.

### BT Condition Nodes

| Node | Returns SUCCESS when |
|------|---------------------|
| `IsEmergency` | Active emergency from hardware bridge |
| `IsBoundaryViolation` | Robot is outside all allowed mowing areas |
| `IsGPSFixed` | `FLAG_GPS_RTK_FIXED` set in AbsolutePose |
| `IsCharging` | Charger relay enabled (robot on dock) |
| `IsBatteryLow(threshold)` | `battery_percent` < threshold (default 22%) |
| `NeedsDocking(threshold)` | `battery_percent` <= threshold (default 20%) |
| `IsBatteryAbove(threshold)` | `battery_percent` >= threshold (default 95%) |
| `IsRainDetected` | Rain sensor reports rain |
| `IsNewRain` | Rain detected AND was not raining at mow session start |
| `IsChargingProgressing` | Battery increased >= 1% in the last 30 minutes |
| `IsCommand(command)` | `context.current_command` == input value |
| `IsLocalizationDegraded` | GNSS health latch (accuracy / RTK mode, fused σ as secondary) says localization is unfit for blade-on mowing |
| `IsLethalBoundaryViolation` | Robot is outside all areas by more than the lethal margin |
| `IsScanStale` | No `/scan` newer than `max_age_sec` |
| `IsCollisionStopSustained` | collision_monitor has been holding the robot for `min_duration_sec` |
| `WasRecentlyInCollisionStop` | A collision stop occurred within `max_age_sec` |
| `IsObstacleStuck` | Repeated no-progress against an obstacle |
| `IsCoverageStartBlocked` | The planned sub-path start pose is occupied |
| `IsCoverageComplete` | The last `GetNextUnmowedArea` run ended because every area is genuinely mowed (not a service error / timeout) |
| `IsDocking` | A docking/undocking transit is in flight |
| `IsRainModeAtLeast(mode)` | `rain_mode` >= input value |
| `IsResumeUndockAllowed(max_attempts)` | Resume-from-dock undock budget not yet exhausted |
| `Nav2Active` | Nav2 lifecycle is active |
| `PreFlightCheck` | Battery, GPS fix type and TF are all healthy |

### BT Action Nodes

| Node | What it does |
|------|-------------|
| `SetMowerEnabled(enabled)` | Calls `/hardware_bridge/mower_control` service |
| `StopMoving` | Publishes a zero `TwistStamped` on `/cmd_vel_emergency` (twist_mux priority 100) |
| `BackUp(backup_dist, backup_speed)` | Nav2 `/backup` action — also the undock manoeuvre (Invariant 10: `UndockRobot` is not used) |
| `NavigateToPose(goal)` | Nav2 `/navigate_to_pose` action; goal as `"x;y;yaw"` string |
| `GetNextUnmowedArea(max_areas)` | Finds next area with remaining coverage; returns FAILURE when all areas complete |
| `PlanCoverageArea(area_index)` | Calls `map_server_node/get_mowing_area`, forwards to `mowgli_coverage`'s `plan_coverage` action (F2C v3); returns hole-free `drivable_subpaths` + bookkeeping segments |
| `TransitToStrip` | Nav2 `navigate_to_pose` to the resume location of the next sub-path (blade off) |
| `FollowStrip` | Drives each sub-path as ONE continuous `FollowCoveragePath` goal; FTC tracks end-to-end with lateral obstacle deviation. Forces the blade OFF before any inter-sub-path transit |
| `DetourAroundObstacle` | After a lookahead-collision abort, a short side-step `NavigateToPose` so the robot physically clears the obstruction before replanning |
| `EscapeStartBlocked` | Frees the robot when the sub-path start pose is occupied, then hands back to the normal retry/clear-costmap branch |
| `DockRobot(dock_id, dock_type)` | `opennav_docking` `/dock_robot` action |
| `SaveObstacles` | Calls the `/obstacle_tracker/save` service |
| `ClearCostmap` | Clears global and local costmaps via Nav2 services |
| `SetNavMode(mode)` | Adjusts Nav2 speed limits based on GPS quality (`precise` / `degraded`) |
| `SetNav2Lifecycle(command)` | PAUSEs / RESUMEs the Nav2 lifecycle while parked on the dock. Gated behind the `idle_nav2_suspend` blackboard flag — a no-op unless enabled |
| `PublishHighLevelStatus(state, state_name)` | Publishes `mowgli_interfaces/msg/HighLevelStatus` |
| `WaitForDuration(duration_sec)` | Non-blocking wait; returns RUNNING until duration elapses |
| `WaitForGpsFix(timeout_sec, min_fix_type)` | Blocks until the GPS reaches the requested fix type |
| `RecordUndockStart` | Snapshots GPS position before undocking |
| `CalibrateHeadingFromUndock` | Computes map heading from GPS displacement vector during undock |
| `SeedYawFromMotion` | Drives forward a short distance, derives yaw from the GPS track and publishes a `set_pose` seed. Used in the not-docked branch, where there is no BackUp for `CalibrateHeadingFromUndock` to work from |
| `ResetEmergency` | Sends the firmware emergency reset (Invariant 9) |
| `RecordArea` | Area boundary recording (`COMMAND_RECORD_AREA` / `_FINISH` / `_CANCEL`) |
| `EndSession` | Resets per-session BT context flags (yaw seed, undock bookkeeping, skipped-swath counter). Only at confirmed session boundaries |
| `ClearCommand` | Resets `context.current_command` to 0 |

The full registered set is in `src/mowgli_behavior/src/register_nodes.cpp`; the behaviour of
each node is mapped in [`docs/claude/codemaps/mowgli_behavior.md`](../docs/claude/codemaps/mowgli_behavior.md).

---

## Simulation

The default Webots world is
`src/mowgli_simulation/worlds_webots/mowgli_garden.wbt`. Select another
available world with the `world` launch argument.

Before editing any simulation asset (`worlds_webots/`, `protos/`,
`urdf_webots/`, `kinematic_drive.py`), read
[`docs/WEBOTS_SIM.md`](../docs/WEBOTS_SIM.md) — the ODE quirks documented there
are load-bearing, and breaking one presents as a Nav2 bug rather than a sim bug.

| World | Size | Use |
|-------|------|-----|
| `mowgli_garden.wbt` | Default garden | End-to-end coverage testing |

The `MowgliMower.proto` robot runs an `<extern>` controller: `webots_ros2_driver`
loads the in-process `mowgli_simulation.kinematic_drive` plugin declared in
`urdf_webots/mowgli_webots.urdf`, which provides differential drive, LiDAR, IMU, GPS
and the simulated sensor interfaces to ROS 2. `sim_imu_noise.py`, `sim_wheel_slip.py`
and `sim_navsat_rtk_fix.py` then inject IMU noise, wheel slip and RTK-quality cycling
on top. Rebuild the Docker image after changing simulation assets that are copied
into the image.

### Foxglove Studio

Connect to `ws://localhost:8765`. Import the layout from `foxglove/mowgli_sim.json`.

Useful topics to visualize:

- `/scan` — LiDAR
- `/coverage/full_plan` — the planned coverage path for the current area
- `/local_costmap/costmap` and `/global_costmap/costmap` — obstacle and keepout maps
- `/behavior_tree_node/high_level_status` — current BT state
- `/map_server_node/mow_progress` — mowing coverage grid
- `/fusion_graph/markers` and `/fusion_graph/diagnostics` — localizer graph and health

### Monitoring Logs

```bash
# Stream logs, filter noisy BT idle chatter
docker logs mowgli_dev_sim -f 2>&1 \
  | grep -v "PublishHighLevelStatus\|Inverter\|Guard\|NeedsDocking\|IsRainDetected\|IsEmergency"

# Check robot position
docker exec mowgli_dev_sim bash -c \
  'source /opt/ros/kilted/setup.bash && ros2 topic echo /wheel_odom --once' \
  | grep -A3 position:

# Watch coverage path progress
docker exec mowgli_dev_sim bash -c \
  'source /opt/ros/kilted/setup.bash && ros2 topic echo /follow_path/_action/feedback --once' \
  | grep distance
```

---

## GUI Integration

The MowgliNext GUI (Go backend + React/Vite frontend, in [`gui/`](../gui/)) reaches ROS2 **only** through `foxglove_bridge` at `ws://<robot-ip>:8765`, plus the teleop relay on 8766. There is no rosbridge in this stack any more.

GUI settings use snake_case YAML keys: `datum_lat`, `datum_lon`, `tool_width`, `battery_full_voltage`, `battery_empty_voltage`, `battery_capacity_mah`. The backend writes them into the **sparse** installed `mowgli_robot.yaml` and prunes any key whose saved value equals the template default, so "reset to default" stays possible (see Invariant 15).

`/battery_state` (`sensor_msgs/msg/BatteryState`) is published by `hardware_bridge_node` for `opennav_docking` charging detection.

### GUI Pages and Features

| Page | Path | Features |
|------|------|----------|
| **Map** | `/#/map` | Live map view, area drawing/editing, coverage progress, obstacle overlays |
| **Diagnostics** | `/#/diagnostics` | Container status, CPU/memory, localization mode, sensor health, ROS diagnostics topics, Fusion Graph save/clear panel, cross-validation checks |
| **Statistics** | `/#/statistics` | Mowing session history, aggregate coverage stats, duration, battery usage, areas completed |
| **Settings** | `/#/settings` | Robot configuration (dimensions, sensor positions, GPS origin, battery thresholds) with per-key "overridden" markers and reset-to-default. Saves trigger a Docker container restart with a warning banner |
| **Schedule** | `/#/schedule` | Automatic mowing schedule |
| **Logs** · **Parameters** · **Onboarding** · **MowgliNext** | `/#/logs`, `/#/parameters`, `/#/onboarding`, `/#/mowglinext` | Container logs, live ROS parameter browsing, first-boot wizard, robot control panel |

See [`gui/CLAUDE.md`](../gui/CLAUDE.md) for the backend/frontend split and the settings write-path.

### BT Visualization and Mowing Sessions

- **Robot state:** driven by `/behavior_tree_node/high_level_status` (`mowgli_interfaces/HighLevelStatus`). The GUI also subscribes to `/behavior_tree_log`, which is **Nav2's** `bt_navigator` log — `mowgli_behavior` does not publish it
- **Automatic session recording:** Go backend monitors `HighLevelStatus` state transitions (AUTONOMOUS ↔ IDLE_DOCKED) and records mowing sessions with timestamps, coverage percentage, battery consumed
- **Map management:** API endpoints manage the persisted mowing-area database (`areas.dat`), dock pose, and `mow_progress` grid — there is no SLAM pose graph anymore

---

## Hardware Protocol

`hardware_bridge_node` communicates with the STM32 over USB serial using:

- **COBS** (Consistent Overhead Byte Stuffing) framing — `0x00` is the frame delimiter
- **CRC-16 CCITT-FALSE** checksum covering all payload bytes
- **Little-endian** packed structs (`#pragma pack(push,1)`)

Packet IDs (from `src/mowgli_hardware/include/mowgli_hardware/ll_datatypes.hpp`):

| ID | Direction | Description |
|----|-----------|-------------|
| `0x01` | STM32 -> Pi | System status (charging, rain, emergency, UI board) |
| `0x02` | STM32 -> Pi | IMU data (accelerometer, gyroscope) |
| `0x03` | STM32 -> Pi | UI button events |
| `0x04` | STM32 -> Pi | Wheel odometry (encoder ticks, delta) |
| `0x05` | STM32 -> Pi | Blade ESC status (temperature, current, RPM) |
| `0x06` | STM32 -> Pi | Boot reset cause |
| `0x11` / `0x12` | bidirectional | High-level config request / response (protocol compatibility key) |
| `0x42` | Pi -> STM32 | Heartbeat (4 Hz) |
| `0x43` | Pi -> STM32 | High-level state |
| `0x50` | Pi -> STM32 | Velocity command (linear x, angular z) |
| `0x51` | Pi -> STM32 | Blade motor control (enable / disable) |
| `0x52` | Pi -> STM32 | Reboot the board (`NVIC_SystemReset`) |
| `0x54` | Pi -> STM32 | Drive (per-wheel) PID gains |
| `0x55` | Pi -> STM32 | Firmware yaw-rate loop gains |
| `0x56` | Pi -> STM32 | Runtime kinematics: max wheel speed cap + wheel base |
| `0x57` | Pi -> STM32 | Runtime safety limits: charge ceiling + e-stop timeouts |

Both the wheel-velocity loop **and** the yaw-rate loop run in STM32 firmware; ROS2 sends
`cmd_vel` through unshaped (the former host-side angular-rate PI was removed in 2026-07).

`hardware_bridge_node` parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/mowgli` | Serial device path |
| `baud_rate` | `115200` | Serial baud rate |
| `heartbeat_rate` | `4.0` | Heartbeat frequency (Hz) |
| `publish_rate` | `100.0` | Sensor data publish rate (Hz) |

Reference C implementation for STM32 porting is in `src/mowgli_hardware/firmware/`.

---

## Contributing

Contributions are welcome. Before opening a pull request:

1. Run `./scripts/format.sh` to apply `clang-format` (style file: `.clang-format`, **clang-format 18**). `make format` also globs the vendored `opennav_coverage` submodule — `scripts/format.sh` matches CI and skips it.
2. Run `make lint` to check with `cppcheck` and `cpplint`.
3. Run `make test` and confirm all tests pass.

### Orientation for contributors (and AI assistants)

Start with [`CLAUDE.md`](CLAUDE.md) in this directory — it points at the generated,
kept-in-sync reference set:

- [`docs/claude/codemaps/`](../docs/claude/codemaps/) — one map per package: files, runtime surface, tests, pitfalls
- [`docs/claude/ros-interfaces.md`](../docs/claude/ros-interfaces.md) — every topic/service/action/TF frame and who creates it
- [`docs/claude/parameters.md`](../docs/claude/parameters.md) — every config key, its default and its consumers
- [`docs/claude/testing-ci.md`](../docs/claude/testing-ci.md) — every test and the CI job that gates it
- [`docs/claude/doc-index.md`](../docs/claude/doc-index.md) — which document is authoritative and which is historical
- [`.claude/rules/ros2.md`](../.claude/rules/ros2.md) — node/QoS/launch/testing conventions for new code

### Conventions

- **C++ standard:** C++17, `ament_cmake` build system (`mowgli_simulation` is C++20)
- **Naming:** `snake_case` for files and ROS parameters, `CamelCase` for C++ classes and node names
- **Units:** SI throughout — metres, radians, seconds
- **Frames:** `map` (global), `odom` (local), `base_footprint` (Nav2 robot frame), `base_link` (robot body, at the rear wheel axis)
- **Topics:** `~/topic` for node-private topics, `/topic` for shared topics; remap in launch files, never in C++

### Pre-commit

A `.pre-commit-config.yaml` is provided:

```bash
pip install pre-commit
pre-commit install
```

### CI Pipeline

`.github/workflows/ros2-ci.yml` runs on pushes to `main`, `dev` and `feat/**`, `fix/**`,
`refactor/**`, `chore/**`, `perf/**`, and on every pull request to `main` or `dev`:

- **`Build & Test (ROS2 kilted)`** on `ubuntu-24.04` — the required status check on `dev`
- `clang-format` compliance on **changed lines only** (`git-clang-format-18`)
- `cppcheck` static analysis on changed files — **report-only**, does not fail the build

---

## License

MowgliNext is **dual licensed** — GPLv3 for open source, personal, educational, non-profit and community use, with a separate commercial license for other cases. The repository-root [`LICENSE`](../LICENSE) is the authoritative statement.

Within `ros2/`, every package declares `GPL-3.0` in its `package.xml` and carries the SPDX identifier in source file headers, with one exception: `mowgli_coverage` declares `BSD-3-Clause`.

The firmware reference implementation in `src/mowgli_hardware/firmware/` is derived from the OpenMower STM32 firmware and is covered by the same GPL-3.0 license.
