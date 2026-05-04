# Mowgli ROS2

A complete ROS2 Kilted robot mower stack built from scratch. Autonomous coverage mowing with RTK-GPS, robot_localization dual-EKF sensor fusion (with an opt-in GTSAM iSAM2 factor-graph localizer for LiDAR scan-matching and loop-closure), cell-based strip coverage planning, and a BehaviorTree.CPP v4 mission executor. Targets ARM boards (Rockchip) deployed in Docker containers.

Originally inspired by the [OpenMower](https://github.com/ClemensElflein/open_mower_ros) project but rewritten from the ground up for ROS2 Kilted with Nav2, a REP-105-compliant dual-EKF GPS+IMU+wheels localizer, and cell-based strip coverage. The current localizer is the standard `ekf_odom_node` + `navsat_transform_node` + `ekf_map_node` trio under `two_d_mode`, with `fusion_graph_node` (GTSAM iSAM2) as an opt-in 1-for-1 replacement for `ekf_map_node` that adds LiDAR scan-matching between-factors and loop-closure factors.

[![CI](https://github.com/cedbossneo/mowgli-ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/cedbossneo/mowgli-ros2/actions/workflows/ci.yml)
[![Docker](https://github.com/cedbossneo/mowgli-ros2/actions/workflows/docker.yml/badge.svg)](https://github.com/cedbossneo/mowgli-ros2/actions/workflows/docker.yml)

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
 |                         openmower-gui                               |
 |           Go backend (rosbridge :9090) + React frontend             |
 +----------------------------+----------------------------------------+
                              | rosbridge WS / Foxglove WS (:8765)
 +----------------------------v----------------------------------------+
 |                     behavior_tree_node                              |
 |  BehaviorTree.CPP v4   main_tree.xml                                |
 |  Guards: Emergency -> Boundary -> GPS mode -> MainLogic             |
 |  Actions: Undock -> GetStrip -> Transit -> Mow -> Dock (+ resume)   |
 +------+------------------+-------------------+-----------------------+
        |                  |                   |
 +------v------+  +--------v--------+   +------v---------------------+
 |  map_server |                        |          Nav2 Kilted         |
 |  GridMap    |                        |  FollowPath (RPP+Rotation)  |
 |  keepout/   |                        |  FollowCoveragePath (FTC)   |
 |  speed masks|                        |  SmacPlanner2D              |
 |  mow_progres|                        |  collision_monitor          |
 |  strip plan |                        |  docking_server             |
 |  area CRUD  |                        +-----------------------------+
 +------+------+
        |
 +------v--------------------------------------------------------------+
 |                        Localization                                  |
 |                                                                      |
 |  robot_localization dual EKF      fusion_graph (opt-in)              |
 |  ekf_odom_node  (50 Hz)           GTSAM iSAM2 factor graph (10 Hz)   |
 |    wheels + gyro -> odom          replaces ekf_map_node 1-for-1      |
 |  navsat_transform_node (30 Hz)    + LiDAR scan-matching factors      |
 |    /gps/fix -> /odometry/gps      + loop-closure factors             |
 |  ekf_map_node (30 Hz)                                                 |
 |    + GPS + COG yaw -> map->odom                                      |
 |                                                                      |
 |  navsat_to_absolute_pose          wheel_odometry_node                |
 |  NavSatFix -> AbsolutePose        Wheel encoder integration          |
 |  (+ /gps/pose_cov for EKF)        cog_to_imu, mag_yaw_publisher      |
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
- **Multi-area strip coverage** — areas are mowed sequentially. For each area, strips are planned on demand by `map_server_node` and fetched one at a time by the BT via `GetNextUnmowedArea` (outer loop) and `GetNextStrip` (inner loop). Nav2 handles transit between strips (`TransitToStrip`), FTCController follows each strip (`FollowStrip`). No pre-planned full path. Progress tracked in `mow_progress` grid layer and survives restarts. Coverage status published on `/map_server_node/coverage_cells` (OccupancyGrid).
- **robot_localization dual EKF (sole localizer)** — Two cooperating EKFs under `two_d_mode` (roll/pitch/Z clamped). `ekf_odom_node` fuses wheel twist + IMU gyro_z → `odom→base_footprint` at 50 Hz (continuous dead reckoning, never jumps). `navsat_transform_node` converts `/gps/fix` + local pose + IMU → `/odometry/gps` in map frame using a fixed lat/lon datum from `mowgli_robot.yaml`. `ekf_map_node` adds `/gps/pose_cov` (the PoseWithCovarianceStamped twin of `/gps/absolute_pose`) and `/imu/cog_heading` (GPS-course-over-ground absolute yaw) and publishes `map→odom` at 30 Hz. Non-holonomic motion is enforced by a tight `vy≈0` covariance in `/wheel_odom`. There is **no SLAM**; the map frame is GPS-anchored.
- **fusion_graph factor-graph localizer** (opt-in, GTSAM iSAM2) — replaces `ekf_map_node` 1-for-1 when `use_fusion_graph:=true`. Same inputs as the EKF (wheel between-factor, gyro between-factor, custom `GnssLeverArmFactor` with analytic Jacobian, COG/mag yaw unary factors), plus optional LiDAR scan-matching between-factors and loop-closure factors that keep the map-frame estimate stable across multi-minute RTK-Float windows. Surface: `/fusion_graph/diagnostics`, `/fusion_graph/markers`, services `~/save_graph` + `~/clear_graph`.
- **RTK GPS localization** — UBX protocol. RTK Fixed gives σ ~3 mm. `ekf_map_node` honors the NavSatFix covariance directly and converges to fix precision without per-fix-type σ floors or outlier gating.
- **BehaviorTree.CPP v4 mission executor** — reactive guards for emergency, boundary, rain, and battery. Automatic rain-stop-dock-wait-resume cycle. Battery-aware dock-charge-undock-resume cycle.
- **Persistent obstacle tracking** — `obstacle_tracker_node` promotes LiDAR detections to persistent after age and observation thresholds. Obstacles are reflected in Nav2 costmaps for dynamic avoidance during strip transit and mowing.
- **Nav2 Kilted** — SmacPlanner2D global planner, RegulatedPurePursuit for transit, FTCController for coverage strips, RotationShimController, `docking_server` (opennav_docking), `collision_monitor`.
- **FTCController Nav2 plugin** — Follow-the-Carrot controller with 3-axis PID for coverage strip following. Provides <10mm lateral accuracy on swaths.
- **Mow progress tracking** — `map_server_node` GridMap layer marks cells as mowed with time-based decay. Visualised as OccupancyGrid.
- **Keepout and speed zone masks** — `map_server_node` publishes Nav2 costmap filter masks for mowing boundaries and perimeter speed limits.
- **Cyclone DDS middleware** — `rmw_cyclonedds_cpp` selected in the runtime Docker image for reliable service discovery on ARM without shared memory issues.
- **Docker multi-stage build** — 6 stages from `ros:kilted-ros-base`. ARM-tested on Rockchip. Dev workflow with bind-mounted source for fast iteration.
- **Foxglove Studio bridge** — WebSocket on port 8765. Pre-built layout at `foxglove/mowgli_sim.json`.
- **openmower-gui integration** — rosbridge WebSocket on port 9090.
- **Diagnostics** — `diagnostics_node` monitors 8+ subsystems: hardware bridge, GPS localization modes, the active map-frame localizer (rate and covariance of `/odometry/filtered_map`, plus `/fusion_graph/diagnostics` when `use_fusion_graph:=true`), obstacle tracker, wheel odometry, published as `diagnostic_msgs/DiagnosticArray` at 1 Hz. Optional MQTT bridge.

---

## Packages

| Package | Executables | Description |
|---------|-------------|-------------|
| `mowgli_interfaces` | — | All ROS2 msg/srv/action definitions: 12 messages, 9 services, 2 actions |
| `mowgli_hardware` | `hardware_bridge_node` | COBS+CRC-16 serial bridge to STM32. Publishes sensor data, subscribes to `cmd_vel` |
| `mowgli_bringup` | — | Launch files, Nav2 / robot_localization config, URDF/xacro, `twist_mux` config |
| `mowgli_localization` | `wheel_odometry_node` `navsat_to_absolute_pose_node` `localization_monitor_node` | Wheel odometry, GPS absolute pose + `/gps/pose_cov` conversion, localization mode monitor, COG-to-IMU absolute-yaw publisher, magnetometer yaw publisher, dock-yaw auto-capture. robot_localization's `ekf_odom_node`/`ekf_map_node`/`navsat_transform_node` are launched directly from the stock package |
| `mowgli_behavior` | `behavior_tree_node` | BehaviorTree.CPP v4 executor. Loads `main_tree.xml`. All BT action and condition nodes |
| `mowgli_map` | `map_server_node` `obstacle_tracker_node` | GridMap with 4 layers, area CRUD services, keepout/speed filter masks, mow progress, strip coverage planner (`~/get_next_strip`, `~/get_coverage_status`). Persistent LiDAR obstacle detection |
| `mowgli_nav2_plugins` | — | `FTCController` Nav2 controller plugin library loaded by `controller_server` |
| `mowgli_monitoring` | `diagnostics_node` `mqtt_bridge_node` | Diagnostics aggregator monitoring 8 subsystems at 1 Hz. Optional MQTT bridge |
| `mowgli_simulation` | `gps_degradation_sim_node` `navsat_to_pose_node` | Gazebo Harmonic worlds, SDF mower model, GPS degradation simulator |

---

## TF Tree

```
map                   (published by ekf_map_node at 30 Hz —
 +-- odom              absorbs GPS corrections)
      +-- base_footprint (published by ekf_odom_node at 50 Hz — odom->base_footprint)
           +-- base_link              (fixed, rear wheel axle)
                +-- left_wheel_link        (continuous joint)
                +-- right_wheel_link       (continuous joint)
                +-- front_left_caster_link (continuous joint)
                +-- front_right_caster_link(continuous joint)
                +-- blade_link             (continuous joint, under chassis)
                +-- imu_link               (fixed — offset from mowgli_robot.yaml)
                +-- gps_link               (fixed — offset from mowgli_robot.yaml)
                +-- lidar_link             (fixed — offset from mowgli_robot.yaml)
```

Frame conventions follow REP-105: `map` (global, GPS-anchored via fixed datum), `odom` (continuous local, drift-only), `base_footprint` (robot frame for Nav2/robot_localization), `base_link` (rear wheel axis, OpenMower convention).

`base_link` is placed at the centre of the rear drive wheel axis at wheel axle height. The chassis geometric centre sits `chassis_center_x` (default 0.18 m) forward of `base_link`. `base_footprint` is directly below `base_link` on the ground plane. The Nav2 footprint polygon is computed at launch from `chassis_length`, `chassis_width`, and `chassis_center_x` read from `mowgli_robot.yaml`.

`ekf_odom_node` publishes `odom→base_footprint` at 50 Hz from wheels + gyro; `ekf_map_node` (default) or `fusion_graph_node` (opt-in) publishes `map→odom` once GPS arrives — there is no SLAM back-end. The two map-frame backends are mutually exclusive in `navigation.launch.py`.

---

## Key Topics and Services

### Published Topics

| Topic | Type | Source | Rate |
|-------|------|--------|------|
| `/hardware_bridge/status` | `mowgli_interfaces/msg/Status` | `hardware_bridge_node` | ~10 Hz |
| `/hardware_bridge/power` | `mowgli_interfaces/msg/Power` | `hardware_bridge_node` | ~1 Hz |
| `/hardware_bridge/emergency` | `mowgli_interfaces/msg/Emergency` | `hardware_bridge_node` | ~1 Hz |
| `/imu/data` | `sensor_msgs/msg/Imu` | `hardware_bridge_node` (remapped) | ~50 Hz |
| `/wheel_odom` | `nav_msgs/msg/Odometry` | `wheel_odometry_node` | ~50 Hz |
| `/gps/absolute_pose` | `mowgli_interfaces/msg/AbsolutePose` | `navsat_to_absolute_pose_node` | ~5 Hz |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | `ekf_odom_node` (local, odom frame) | 50 Hz |
| `/odometry/filtered_map` | `nav_msgs/msg/Odometry` | `ekf_map_node` (global, map frame) | 30 Hz |
| `/odometry/gps` | `nav_msgs/msg/Odometry` | `navsat_transform_node` | on GPS arrival |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR driver or Gazebo bridge | ~10 Hz |
| `/behavior_tree_node/high_level_status` | `mowgli_interfaces/msg/HighLevelStatus` | `behavior_tree_node` | on BT tick |
| `/map_server_node/coverage_cells` | `nav_msgs/msg/OccupancyGrid` | `map_server_node` | on change |
| `/map_server_node/grid_map` | `grid_map_msgs/msg/GridMap` | `map_server_node` | configurable |
| `/map_server_node/mow_progress` | `nav_msgs/msg/OccupancyGrid` | `map_server_node` | configurable |
| `/map_server_node/docking_pose` | `geometry_msgs/msg/PoseStamped` | `map_server_node` | transient-local |
| `/map_server_node/boundary_violation` | `std_msgs/msg/Bool` | `map_server_node` | on change |
| `/obstacle_tracker/obstacles` | `mowgli_interfaces/msg/ObstacleArray` | `obstacle_tracker_node` | on change |
| `/localization/mode` | `std_msgs/msg/String` | `localization_monitor_node` | 10 Hz |
| `/localization/mode_id` | `std_msgs/msg/Int32` | `localization_monitor_node` | 10 Hz |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | `diagnostics_node` | ~1 Hz |
| `/battery_state` | `sensor_msgs/msg/BatteryState` | `hardware_bridge_node` | ~1 Hz |

### Services

| Service | Type | Server |
|---------|------|--------|
| `/hardware_bridge/mower_control` | `mowgli_interfaces/srv/MowerControl` | `hardware_bridge_node` |
| `/hardware_bridge/emergency_stop` | `mowgli_interfaces/srv/EmergencyStop` | `hardware_bridge_node` |
| `/behavior_tree_node/high_level_control` | `mowgli_interfaces/srv/HighLevelControl` | `behavior_tree_node` |
| `/map_server_node/add_mowing_area` | `mowgli_interfaces/srv/AddMowingArea` | `map_server_node` |
| `/map_server_node/get_mowing_area` | `mowgli_interfaces/srv/GetMowingArea` | `map_server_node` |
| `/map_server_node/set_docking_point` | `mowgli_interfaces/srv/SetDockingPoint` | `map_server_node` |
| `/map_server_node/save_map` | `std_srvs/srv/Trigger` | `map_server_node` |
| `/map_server_node/clear_map` | `std_srvs/srv/Trigger` | `map_server_node` |
| `/map_server_node/save_areas` | `std_srvs/srv/Trigger` | `map_server_node` |
| `/map_server_node/load_areas` | `std_srvs/srv/Trigger` | `map_server_node` |
| `/map_server_node/get_next_strip` | `mowgli_interfaces/srv/GetNextStrip` | `map_server_node` |
| `/map_server_node/get_coverage_status` | `mowgli_interfaces/srv/GetCoverageStatus` | `map_server_node` |
| `/obstacle_tracker/save_obstacles` | `std_srvs/srv/Trigger` | `obstacle_tracker_node` |

### Actions

| Action | Type | Server |
|--------|------|--------|
| `/dock_robot` | `opennav_docking_msgs/action/DockRobot` | `docking_server` |
| `/undock_robot` | `opennav_docking_msgs/action/UndockRobot` | `docking_server` |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 `bt_navigator` |
| `/follow_path` | `nav2_msgs/action/FollowPath` | Nav2 `controller_server` |
| `/backup` | `nav2_msgs/action/BackUp` | Nav2 behaviors |

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
| `3` | `COMMAND_RECORD_AREA` | Start area boundary recording |
| `5` | `COMMAND_RECORD_FINISH` | Finish recording, save polygon |
| `6` | `COMMAND_RECORD_CANCEL` | Cancel recording, discard trajectory |
| `7` | `COMMAND_MANUAL_MOW` | Enter manual mowing mode (teleop + blade) |
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

`localization_monitor_node` publishes the current mode on `/localization/mode` (string) and `/localization/mode_id` (int32). The modes reflect GPS quality + whether the active map-frame localizer is healthy:

| mode_id | mode string | Condition | Typical accuracy |
|---------|-------------|-----------|-----------------|
| `4` | `RTK_FIXED` | RTK Fixed, fusion healthy | σ ~3 mm (bounded by GPS σ + EKF innovation) |
| `3` | `RTK_FLOAT` | RTK Float, fusion healthy | ~5–20 cm; bounded further when `use_fusion_graph + use_scan_matching` are on |
| `2` | `DGPS` | DGPS / SBAS, fusion healthy | ~0.5–1 m |
| `1` | `DEAD_RECKONING` | GPS degraded, relying on wheels + IMU | drifts ~1 %/m; with `use_fusion_graph + use_scan_matching` and a LiDAR, scan-matching factors keep the map-frame estimate bounded |
| `0` | `NO_FIX` | All sources degraded | Return to dock |

---

## Configuration

### mowgli_robot.yaml — Central Robot Configuration

`src/mowgli_bringup/config/mowgli_robot.yaml` is the single source of truth for all physical, operational, and safety parameters. The launch files read it at startup. In Docker deployment it is bind-mounted from the host at `/ros2_ws/config/mowgli_robot.yaml` so changes take effect on container restart without rebuilding the image.

All dimensions are in **metres**, angles in **radians**, speeds in **m/s**.

#### Hardware / Physical

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mower_model` | `"YardForce500"` | Robot model identifier |
| `chassis_length` | `0.54` | Chassis length |
| `chassis_width` | `0.40` | Chassis width |
| `chassis_height` | `0.19` | Chassis height |
| `chassis_mass_kg` | `8.76` | Total robot mass |
| `wheel_radius` | `0.04475` | Drive wheel radius |
| `wheel_track` | `0.325` | Drive wheel centre-to-centre track width |
| `wheel_x_offset` | `0.0` | Drive wheel x offset from `base_link` (0 = at wheel axis) |
| `chassis_center_x` | `0.18` | Chassis geometric centre forward of wheel axis |
| `ticks_per_meter` | `300` | Encoder ticks per full wheel revolution (informational only — not used by runtime) |
| `caster_radius` | `0.03` | Front caster radius |
| `blade_radius` | `0.09` | Cutting blade disc radius |
| `tool_width` | `0.18` | Effective cut width (2 x blade_radius) |

#### Sensor Positions (relative to base_link)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `lidar_x` | `0.38` | LiDAR forward offset |
| `lidar_z` | `0.22` | LiDAR height offset |
| `lidar_yaw` | `0.0` | LiDAR heading rotation |
| `imu_x` | `0.18` | IMU forward offset |
| `imu_z` | `0.095` | IMU height offset |
| `gps_x` | `0.3` | GPS antenna forward offset |
| `gps_z` | `0.20` | GPS antenna height offset |

All sensor positions drive both the URDF (TF frames) and the Nav2 footprint polygon.

#### GPS / Positioning

| Parameter | Default | Description |
|-----------|---------|-------------|
| `datum_lat` | `0.0` | Map origin latitude — **set per site** |
| `datum_lon` | `0.0` | Map origin longitude — **set per site** |
| `gps_protocol` | `"UBX"` | GPS receiver protocol |
| `gps_wait_after_undock_sec` | `10.0` | Wait for RTK fix after undocking |
| `ntrip_enabled` | `false` | Enable NTRIP RTK correction stream |
| `ntrip_host` | `""` | NTRIP caster hostname |
| `ntrip_port` | `2101` | NTRIP caster port |
| `ntrip_mountpoint` | `""` | NTRIP mountpoint |

#### Battery

| Parameter | Default | Description |
|-----------|---------|-------------|
| `battery_full_voltage` | `28.5` | Fully charged threshold (V) |
| `battery_empty_voltage` | `24.0` | Start docking (V) |
| `battery_critical_voltage` | `23.0` | Immediate dock (V) |
| `battery_full_percent` | `95.0` | Resume mowing above this (%) |
| `battery_low_percent` | `20.0` | Start docking (%) |
| `battery_critical_percent` | `10.0` | Emergency dock — BT guard (%) |

#### Mowing

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mowing_speed` | `0.20` | Speed during coverage paths (m/s) |
| `transit_speed` | `0.25` | Speed during point-to-point navigation (m/s) |
| `path_spacing` | `0.18` | Distance between parallel swath centrelines |
| `headland_width` | `0.18` | Width of one headland pass (1 x tool_width) |
| `mow_angle_offset_deg` | `-1.0` | Swath angle; -1 = auto-detect optimal |
| `min_turning_radius` | `0.30` | Minimum turning radius for Dubins curves |

#### Docking

| Parameter | Default | Description |
|-----------|---------|-------------|
| `dock_pose_x` | `0.0` | Dock position in map frame — **set per site** |
| `dock_pose_y` | `0.0` | Dock position in map frame — **set per site** |
| `dock_pose_yaw` | `4.17` | Dock heading in map frame (rad) — **set per site** |
| `undock_distance` | `1.5` | Distance to reverse when undocking |
| `undock_speed` | `0.15` | Reverse speed during undocking (m/s) |
| `dock_max_retries` | `3` | Maximum docking attempts |

#### Rain

| Parameter | Default | Description |
|-----------|---------|-------------|
| `rain_mode` | `2` | 0=ignore, 1=dock, 2=dock_until_dry, 3=pause_auto |
| `rain_delay_minutes` | `30.0` | Wait after rain stops before resuming |
| `rain_debounce_sec` | `10.0` | Rain must persist this long to trigger |

### Coverage Parameters

Coverage strip planning is handled by `map_server_node`. Mowing parameters are configured in `src/mowgli_bringup/config/mowgli_robot.yaml` (see Mowing section above). Key parameters: `tool_width`, `path_spacing`, `mowing_speed`, `transit_speed`.

### Key Config File Reference

| File | Controls |
|------|----------|
| `src/mowgli_bringup/config/mowgli_robot.yaml` | All physical, operational, and safety parameters |
| `src/mowgli_bringup/config/nav2_params.yaml` | Nav2 controllers, planner, costmaps, collision monitor |
| `src/mowgli_bringup/config/robot_localization.yaml` | robot_localization dual-EKF tuning (process noise, fused inputs, datum) |
| `src/mowgli_bringup/config/hardware_bridge.yaml` | Serial bridge (port/baud/rate, IMU cal sample count) |
| `src/mowgli_bringup/config/twist_mux.yaml` | `cmd_vel` multiplexer priorities and timeouts |
| `src/mowgli_map/config/obstacle_tracker.yaml` | LiDAR obstacle detection thresholds |
| `src/mowgli_behavior/config/behavior_tree.yaml` | BT node parameters |
| `src/mowgli_behavior/trees/main_tree.xml` | Full BT structure: guards, sequences, recovery |

---

## Building

### Prerequisites

- ROS2 Kilted on Ubuntu 24.04
- `colcon`, `rosdep`, `xacro` (`python3-colcon-common-extensions`, `python3-rosdep`)

### Build the Workspace

```bash
source /opt/ros/kilted/setup.bash
cd /path/to/mowgli-ros2

rosdep update --rosdistro kilted
rosdep install --from-paths src --ignore-src --rosdistro kilted -y

colcon build \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers $(nproc) \
  --event-handlers console_cohesion+

source install/setup.bash
```

### Running Tests

```bash
source /opt/ros/kilted/setup.bash && source install/setup.bash
colcon test --return-code-on-test-failure
colcon test-result --verbose
```

### Makefile Shortcuts

```bash
make build           # colcon build (Release)
make build-debug     # colcon build (Debug)
make test            # colcon test + test-result
make clean           # remove build/ install/ log/
make format          # clang-format all C++ files in-place
make format-check    # verify formatting without modifying files
make lint            # cppcheck + cpplint
```

---

## Docker Deployment

### Image Stages

| Stage | From | Contents |
|-------|------|----------|
| `base` | `ros:kilted-ros-base` | All apt runtime deps: Nav2, robot_localization, rosbridge-suite, foxglove-bridge, Cyclone DDS, GTSAM (built from source for `fusion_graph`) |
| `deps` | `base` | Build tools, rosdep resolution |
| `build-interfaces` | `deps` | `mowgli_interfaces` compiled only (cached layer, rarely rebuilt) |
| `build` | `build-interfaces` | All remaining packages compiled, unit tests run |
| `runtime` | `base` | Compiled install tree + rosbridge CBOR patch applied. Sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` |
| `simulation` | `runtime` | Gazebo Harmonic + TigerVNC + noVNC for GUI access |

### Docker Compose Services

| Service | Image | Ports | Use case |
|---------|-------|-------|----------|
| `mowgli` | `runtime` | — | Real hardware — requires `/dev/mowgli` serial device |
| `simulation` | `simulation` | `8765` | Headless Gazebo + full nav stack |
| `simulation-gui` | `simulation` | `8765`, `6080` | Gazebo GUI via noVNC |
| `nav` | `runtime` | — | Navigation stack only (no hardware bridge) |
| `dev-sim` | `dev` | `8765`, `6080` | Development with bind-mounted source tree |
| `dev-sim-small` | `dev` | `8765`, `6080` | Small 10m x 8m garden for fast iteration |
| `dev` | `dev` | — | Interactive shell with bind-mounted source tree |

### Running Hardware

```bash
# Requires /dev/mowgli USB serial symlink to STM32 board
docker compose up mowgli
```

### Running Simulation

```bash
# Headless — connect Foxglove Studio to ws://localhost:8765
make run-sim

# With Gazebo GUI — open http://localhost:6080/vnc.html
make run-sim-gui
```

### Development Workflow

```bash
# Start dev simulation (source bind-mounted from host)
make dev-sim

# In another terminal: edit source files on the host, then rebuild:
make dev-build                             # rebuild all packages
make dev-build-pkg PKG=mowgli_behavior    # rebuild one package

# Restart simulation to pick up rebuilt code:
make dev-restart

# Open an interactive shell inside the running container:
make dev-shell
```

Config and YAML files under `src/` are live-edited without rebuilding — just restart the container.

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
- `twist_mux` — priority multiplexer: navigation (priority 10) < teleop (20) < emergency velocity (100) < `emergency_stop` lock (255)

**Tier 2 — `full_system.launch.py`** (includes Tier 1 + full navigation stack):

- `navigation.launch.py` — robot_localization dual EKF (`ekf_odom_node` + `navsat_transform_node` + `ekf_map_node`), optional `fusion_graph_node` (gated on `use_fusion_graph`), Nav2 bringup
- `behavior_tree_node` — BT mission executor
- `map_server_node` + `obstacle_tracker_node` — area management and obstacle tracking
- `wheel_odometry_node`, `navsat_to_absolute_pose_node`, `localization_monitor_node` — localization pipeline
- `diagnostics_node` — robot health monitoring
- `mqtt_bridge_node` — optional, `enable_mqtt:=true`
- `foxglove_bridge` — enabled by default on port 8765
- `rosbridge_websocket` + `rosapi_node` — enabled by default on port 9090

### Launch Arguments

**`full_system.launch.py`:**

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use Gazebo simulation clock |
| `serial_port` | `/dev/mowgli` | Hardware bridge serial device |
| `use_lidar` | `true` | Launch LiDAR-dependent nodes (fusion_graph scan-matching when `use_fusion_graph` is on, obstacle layer, collision-monitor scan). `false` runs a GPS-only configuration |
| `enable_mqtt` | `false` | Launch MQTT bridge node |
| `enable_foxglove` | `true` | Launch Foxglove Bridge |
| `enable_rosbridge` | `true` | Launch rosbridge_server |
| `foxglove_port` | `8765` | Foxglove Bridge WebSocket port |
| `rosbridge_port` | `9090` | rosbridge WebSocket port |

**`navigation.launch.py`:**

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use Gazebo simulation clock |
| `use_ekf` | `true` | Launch robot_localization dual EKF (always true on real hardware; disable only for isolated Nav2 tests) |
| `use_lidar` | `true` | Launch LiDAR-aware Nav2 config (`nav2_params.yaml`) and enable scan-matching when `use_fusion_graph` is on. `false` uses `nav2_params_no_lidar.yaml` |

### Simulation Launch Files

| File | Description |
|------|-------------|
| `sim_full_system.launch.py` | Full stack with Gazebo Harmonic (headless by default) |
| `sim_small_garden.launch.py` | 10m x 8m garden for boundary and path testing |
| `simulation.launch.py` | Gazebo and `ros_gz_bridge` only |

---

## Behavior Tree

The mission executor loads `src/mowgli_behavior/trees/main_tree.xml` using BehaviorTree.CPP v4. The root is a `ReactiveSequence`, which means all guards are re-evaluated on every tick before the child can proceed.

### Guard Priority (outer to inner, highest to lowest)

```
ReactiveSequence (Root)
  |
  +-- EmergencyGuard
  |     Polls /hardware_bridge/emergency every tick.
  |     On emergency: disable blade, stop, publish EMERGENCY, wait 1s.
  |
  +-- BoundaryGuard
  |     Polls /map_server_node/boundary_violation every tick.
  |     On violation: disable blade, stop, back up (5 attempts, 1m each),
  |     publish BOUNDARY_EMERGENCY_STOP if all attempts fail.
  |
  +-- GPSModeSelector
  |     RTK fixed  -> SetNavMode("precise")   full speed
  |     Otherwise  -> SetNavMode("degraded")  half speed + wider inflation
  |
  +-- MainLogic (Fallback — tried in order)
        |
        +-- CriticalBatteryDock  battery_percent < 10%  -> dock immediately
        +-- MowingSequence       COMMAND_START = 1
        +-- HomeSequence         COMMAND_HOME = 2
        +-- RecordingSequence    COMMAND_S1 = 3
        +-- IdleSequence         no command -> wait 0.5s
```

### Mowing Sequence (COMMAND_START)

```
MowingSequence
  PublishHighLevelStatus("UNDOCKING")
  UndockOrSkip
    IsCharging? -> skip (robot already in field)
    UndockSequence:
      Wait up to 30s for GPS fix (timeout -> proceed anyway)
      SaveSlamMap("/ros2_ws/maps/garden_map")
      RecordUndockStart         snapshot GPS position
      UndockRobot               opennav_docking undock action
      CalibrateHeadingFromUndock compute map heading from GPS displacement
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
        |     NeedsDocking(threshold=20%)?
        |     -> disable blade, stop, dock, wait until battery > 95%
        |        (IsChargingProgressing detects stalled charger after 30min),
        |        undock, resume
        |
        +-- AreaLoop (Repeat x100)
              GetNextUnmowedArea(max_areas=20)  find next area with remaining strips
              StripLoop (Repeat x500)
                MowOneStrip:
                  GetNextStrip(area_index=N)  fetch next strip from current area
                  TransitToStrip              Nav2 navigate to strip start
                  FollowStrip                 FTCController follows the strip path
                or StripRecovery:
                  disable blade, stop, back up 0.3m, ClearCostmap, wait 2s
              (GetNextStrip returns FAILURE when area coverage complete → next area)

      FailedCoverageDock:
        SaveSlamMap + DockRobot + ClearCommand

  On completion (GetNextUnmowedArea returns FAILURE = all areas mowed):
    SetMowerEnabled(false)
    SaveObstacles
    SaveSlamMap
    DockRobot
    PublishHighLevelStatus("IDLE_DOCKED")
    ClearCommand
```

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

### BT Action Nodes

| Node | What it does |
|------|-------------|
| `SetMowerEnabled(enabled)` | Calls `/hardware_bridge/mower_control` service |
| `StopMoving` | Publishes zero Twist to `/cmd_vel` |
| `BackUp(backup_dist, backup_speed)` | Nav2 `/backup` action |
| `NavigateToPose(goal)` | Nav2 `/navigate_to_pose` action; goal as `"x;y;yaw"` string |
| `GetNextUnmowedArea(max_areas)` | Finds next area with remaining strips; returns FAILURE when all areas complete. Updates `area_index` for `GetNextStrip` |
| `GetNextStrip(area_index)` | Calls `~/get_next_strip` on `map_server_node` for given area; returns FAILURE when area coverage complete |
| `TransitToStrip` | Nav2 `NavigateToPose` to the start of the current strip |
| `FollowStrip` | Follows the current strip path via Nav2 `FollowPath` with FTCController |
| `DockRobot(dock_id, dock_type)` | `opennav_docking` `/dock_robot` action |
| `UndockRobot(dock_type)` | `opennav_docking` `/undock_robot` action |
| `SaveSlamMap(map_path)` | Deprecated no-op stub (kept so older BT XML files load). Returns SUCCESS immediately — SLAM backend was removed once RTK proved sufficient (and fusion_graph now persists its own factor graph via `~/save_graph`) |
| `SaveObstacles` | Calls `/obstacle_tracker/save_obstacles` service |
| `ClearCostmap` | Clears global and local costmaps via Nav2 services |
| `SetNavMode(mode)` | Adjusts Nav2 speed limits based on GPS quality (`precise` / `degraded`) |
| `PublishHighLevelStatus(state, state_name)` | Publishes `mowgli_interfaces/msg/HighLevelStatus` |
| `WaitForDuration(duration_sec)` | Non-blocking wait; returns RUNNING until duration elapses |
| `RecordUndockStart` | Snapshots GPS position before undocking |
| `CalibrateHeadingFromUndock` | Computes map heading from GPS displacement vector during undock |
| `ClearCommand` | Resets `context.current_command` to 0 |

---

## Simulation

Gazebo Harmonic worlds in `src/mowgli_simulation/worlds/`:

| World | Size | Use |
|-------|------|-----|
| `garden.sdf` | Full garden | End-to-end coverage testing |
| `small_garden.sdf` | 10m x 8m | Boundary and path testing (fast iteration) |
| `empty_garden.sdf` | Flat plane | Controller and localization testing |

The SDF model in `src/mowgli_simulation/models/mowgli_mower/model.sdf` provides a differential drive plugin, 360-degree LiDAR (range_min=0.35m to suppress chassis self-reflections), IMU, and GPS. Sensor data is bridged to ROS2 via `ros_gz_bridge`.

Changes to `model.sdf` require a Docker image rebuild. Bind-mounting the models directory breaks Gazebo sensor initialization.

### Foxglove Studio

Connect to `ws://localhost:8765`. Import the layout from `foxglove/mowgli_sim.json`.

Useful topics to visualize:

- `/scan` — LiDAR
- `/map_server_node/coverage_cells` — strip coverage progress grid
- `/local_costmap/costmap` and `/global_costmap/costmap` — obstacle and keepout maps
- `/behavior_tree_node/high_level_status` — current BT state
- `/map_server_node/mow_progress` — mowing coverage grid

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

The `openmower-gui` (Go backend + React/Vite frontend) connects via rosbridge at `ws://<robot-ip>:9090`.

The GUI uses **`compression: "none"`** — the rosbridge CBOR encoder in Kilted crashes on `float64[36]` covariance arrays (Odometry, Imu). The Dockerfile applies `scripts/patch_rosbridge.py` at build time to patch the server-side encoder. Do not enable CBOR compression without verifying this fix is present.

GUI settings use snake_case YAML keys: `datum_lat`, `datum_lon`, `tool_width`, `battery_full_voltage`, `battery_empty_voltage`, `battery_capacity_mah`.

`/battery_state` (`sensor_msgs/msg/BatteryState`) is published by `hardware_bridge_node` for `opennav_docking` charging detection.

### GUI Pages and Features

| Page | Path | Features |
|------|------|----------|
| **Map** | `/#/map` | Live map view, area drawing/editing, coverage progress, obstacle overlays |
| **Diagnostics** | `/#/diagnostics` | Container status, CPU/memory, localization mode (RTK/SLAM/dead reckoning), BT state from `/behavior_tree_log`, sensor health, ROS diagnostics topics, SLAM tools (save/delete map), cross-validation checks |
| **Statistics** | `/#/statistics` | Mowing session history, aggregate coverage stats, duration, battery usage, areas completed |
| **Settings** | `/#/settings` | Robot configuration (dimensions, sensor positions, GPS origin, battery thresholds). Saves trigger Docker container restart with warning banner. Onboarding mode restarts ROS2 container. |

### BT Visualization and Mowing Sessions

- **BT visualization:** Active behavior tree nodes displayed from `/behavior_tree_log` topic (JSON publication from `behavior_tree_node`)
- **Automatic session recording:** Go backend monitors `HighLevelStatus` state transitions (AUTONOMOUS ↔ IDLE_DOCKED) and records mowing sessions with timestamps, coverage percentage, battery consumed
- **Map management:** API endpoints manage the persisted mowing-area database, dock pose, and `mow_progress` grid — there is no SLAM pose graph anymore

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
| `0x42` | Pi -> STM32 | Heartbeat (4 Hz) |
| `0x43` | Pi -> STM32 | High-level state |
| `0x50` | Pi -> STM32 | Velocity command (linear x, angular z) |
| `0x51` | Pi -> STM32 | Blade motor control (enable / disable) |

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

1. Run `make format` to apply `clang-format` (style file: `.clang-format`).
2. Run `make lint` to check with `cppcheck` and `cpplint`.
3. Run `make test` and confirm all tests pass.

### Conventions

- **C++ standard:** C++17, `ament_cmake` build system
- **Naming:** `snake_case` for files and ROS parameters, `CamelCase` for C++ classes and node names
- **Units:** SI throughout — metres, radians, seconds
- **Frames:** `map` (global), `odom` (local), `base_link` (robot body)
- **Topics:** `~/topic` for node-private topics, `/topic` for shared topics

### Pre-commit

A `.pre-commit-config.yaml` is provided:

```bash
pip install pre-commit
pre-commit install
```

### CI Pipeline

GitHub Actions runs on every push and pull request to `main`:

- Build and unit test on `ubuntu-24.04` / ROS2 Kilted
- `clang-format` compliance check (clang-format-17)
- `cppcheck` static analysis

---

## License

All packages in this repository declare **GPL-3.0-or-later** as their license in `package.xml` and carry the SPDX identifier `GPL-3.0` in source file headers.

The firmware reference implementation in `src/mowgli_hardware/firmware/` is derived from the OpenMower STM32 firmware and is covered by the same GPL-3.0 license.

No `LICENSE` file exists in the repository root. The SPDX identifiers in source files and `package.xml` declarations are the authoritative license statements.
