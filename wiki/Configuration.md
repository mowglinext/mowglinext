# Configuration Reference

Complete guide to all configuration files and parameters in the Mowgli ROS2 system.

This documentation is for ROS2 Kilted with Gazebo Harmonic.

[CLAUDE.md](https://github.com/cedbossneo/mowglinext/blob/main/CLAUDE.md) is the authoritative short-form reference. If any section here contradicts it, CLAUDE.md wins.

## Overview

Configuration is centralized in `src/mowgli_bringup/config/`:

```
config/
├── hardware_bridge.yaml          # Serial port, baud, publish rate, IMU cal sample count
├── robot_localization.yaml       # Dual EKF: ekf_odom (wheels + gyro → odom),
│                                  #   navsat_transform, ekf_map (+ GPS + COG yaw → map)
├── nav2_params.yaml               # Navigation stack with LiDAR (obstacle layer, FTCController, docking)
├── nav2_params_no_lidar.yaml      # Navigation stack for GPS-only operation
├── twist_mux.yaml                 # Velocity multiplexer priorities
├── foxglove_bridge.yaml           # Foxglove Studio visualization bridge
└── mowgli_robot.yaml              # Centralized robot config (bind-mounted from
                                   #   install/config/mowgli at /ros2_ws/config/)
```

There is **no** `slam_toolbox.yaml` or `kiss_icp.yaml`. robot_localization (the default dual-EKF localizer) is tuned through `robot_localization.yaml`. LiDAR scan-matching and loop-closure are handled by `fusion_graph` (opt-in — see [§5](#5-fusion_graph)).

Whether the LiDAR-aware stack (`nav2_params.yaml`) or the GPS-only stack (`nav2_params_no_lidar.yaml`) is loaded is decided by the **`lidar_enabled`** key in `mowgli_robot.yaml`, which is **authoritative**. The `LIDAR_ENABLED` environment variable (set by the installer `.env`) is only a **fallback** — it applies solely when `lidar_enabled` is absent from the yaml, so a stale installer `.env` can never override the user's GUI-managed config. A CLI/compose `use_lidar:=` override still wins for one-off runs.

The opt-in **fusion_graph** localizer (GTSAM iSAM2 — see [§7](#7-fusion_graph)) does **not** have a separate config file: its knobs are declared as ros2 parameters on `fusion_graph_node` and the high-level switches (`use_fusion_graph`, `use_scan_matching`, `use_loop_closure`, `use_magnetometer`) live in `mowgli_robot.yaml`. The Settings page exposes them under the *Localization* section.

All YAML files use the ROS2 `ros__parameters` namespace convention. Parameters can be overridden via command-line:

```bash
ros2 launch mowgli_bringup mowgli.launch.py \
    serial_port:=/dev/ttyACM0 \
    use_sim_time:=true
```

---

## 1. foxglove_bridge.yaml

**File:** `src/mowgli_bringup/config/foxglove_bridge.yaml`

**Purpose:** Configure the Foxglove Studio visualization bridge for remote monitoring and debugging.

**Full Example:**

```yaml
foxglove_bridge:
  ros__parameters:
    # Server port for Foxglove Studio client connections
    port: 8765

    # Enable/disable the WebSocket server
    enabled: true

    # Maximum number of concurrent WebSocket connections
    max_connections: 10

    # Message queue size for buffering (prevents dropping data)
    queue_size: 100

    # Send messages to all connected clients
    send_buffer_limit: 10000000      # bytes (~10 MB per client)

    # Subscribed topics (publish to all connected clients)
    subscribed_topics:
      - /scan                         # LiDAR scan
      - /odometry/filtered_map        # Robot pose estimate from ekf_map_node
                                      #   (or fusion_graph_node when use_fusion_graph:=true)
      - /costmap/costmap              # Global costmap
      - /local_costmap/costmap        # Local costmap
      - /path                         # Global plan
      - /cmd_vel                      # Velocity commands
      - /status                        # Hardware status
      - /power                         # Battery voltage
      - /gps/absolute_pose             # GPS absolute pose
      - /imu/data                     # IMU data

    # Camera feed (if available from camera node)
    # camera_topic: /usb_cam/image_raw

    # Custom transformation frame (if needed)
    # tf_frame: map
```

### Key Parameters

#### `port`

- **Type:** integer
- **Default:** `8765`
- **Description:** WebSocket server port for Foxglove Studio connections
- **Note:** Ensure this port is not blocked by firewall on deployment machine

#### `enabled`

- **Type:** boolean
- **Default:** `true`
- **Description:** Enable/disable the Foxglove bridge
- **Use case:** Disable on production for reduced overhead; enable for debugging

#### `max_connections`

- **Type:** integer
- **Default:** `10`
- **Description:** Maximum concurrent WebSocket connections
- **Tuning:** Increase if multiple users need simultaneous access (higher memory overhead)

#### `subscribed_topics`

- **Type:** list of strings
- **Description:** ROS2 topics to stream to Foxglove clients
- **Performance:** More topics = higher network bandwidth and CPU load
- **Typical setup:** Core navigation and hardware status topics (shown above)

---

## 2. hardware_bridge.yaml

**File:** `src/mowgli_bringup/config/hardware_bridge.yaml`

**Purpose:** Configure serial communication with the STM32 firmware.

**Full Example:**

```yaml
hardware_bridge:
  ros__parameters:
    # Serial port device path
    serial_port: "/dev/mowgli"

    # Baud rate (must match firmware configuration)
    baud_rate: 115200

    # Heartbeat transmission rate (Hz)
    # Keeps watchdog on firmware alive; also transmits emergency state
    heartbeat_rate: 4.0

    # Sensor publishing rate (Hz)
    # Controls frequency of /status, /power, /imu/data_raw messages
    publish_rate: 100.0

    # High-level state update rate (Hz)
    # Sends current_mode and gps_quality to firmware (informational)
    high_level_rate: 2.0
```

### Parameter Details

#### `serial_port`

- **Type:** string
- **Default:** `"/dev/mowgli"`
- **Description:** Device path for USB serial connection to STM32
- **Common values:**
  - `/dev/mowgli` – FTDI/CH340 USB-serial adapter
  - `/dev/ttyACM0` – Native STM32 CDC (preferred, more reliable)
  - `/dev/ttyACM1` – Alternative CDC address if multiple USB devices
  - `COM3` (Windows) – Serial port number
- **Note:** Use `ls /dev/tty*` to identify the correct device
- **Tip:** On Linux, persistent device names can be configured via udev rules

#### `baud_rate`

- **Type:** integer
- **Default:** `115200`
- **Description:** Serial port baud rate
- **Must match firmware setting exactly**
- **Performance vs. Latency:**
  - 115200 – Standard, good for 100 Hz sensors (current default)
  - 230400 – Higher throughput (rarely needed for Mowgli)
  - 57600 – Legacy (ROS1 default, not recommended)
- **Note:** USB virtual serial ports are not affected by baud rate in hardware; setting is mainly for consistency

#### `heartbeat_rate`

- **Type:** double (Hz)
- **Default:** `4.0`
- **Range:** 1.0–10.0 Hz typical
- **Description:** Rate at which hardware_bridge sends heartbeat packets to firmware
- **Purpose:**
  - Keeps firmware watchdog alive (typically 500 ms timeout)
  - Transmits emergency stop control bits
  - Transmits emergency release signal (one-shot per service call)
- **Formula:** `heartbeat_period = 1.0 / heartbeat_rate`
  - 4.0 Hz → 250 ms period
  - 2.0 Hz → 500 ms period (risky if firmware watchdog is 500 ms)
- **Tuning:** Increase if firmware reports watchdog timeout; decrease for tighter safety response

#### `publish_rate`

- **Type:** double (Hz)
- **Default:** `100.0`
- **Range:** 10.0–200.0 Hz typical
- **Description:** Rate at which hardware_bridge publishes sensor data to ROS2 topics
- **Affects:** `/hardware_bridge/status`, `/hardware_bridge/power`, `/hardware_bridge/imu/data_raw`
- **Note:** These are the node-local topic names (`~/status`, `~/power`, `~/imu/data_raw`). In `mowgli.launch.py`, they are remapped to `/status`, `/power`, and `/imu/data` respectively.
- **Upstream drivers:** Should match firmware's sensor acquisition rate (typically 100 Hz)
- **Tuning:**
  - Increase for more responsive sensor feedback (higher CPU load on Pi)
  - Decrease to reduce USB serial traffic (may miss fast transients)
  - Typical sweet spot: 50–100 Hz

#### `high_level_rate`

- **Type:** double (Hz)
- **Default:** `2.0`
- **Range:** 1.0–10.0 Hz typical
- **Description:** Rate at which hardware_bridge sends high-level state to firmware
- **Payload:** Current high-level mode (idle/mowing/docking/recording) and GPS RTK quality
- **Purpose:** Firmware uses this for telemetry, sound notifications, LED feedback
- **Tuning:** Lower rate OK (2 Hz sufficient for informational updates)

#### `angular_rate_loop_enabled` / `angular_rate_kp` / `angular_rate_ki`

- **Type:** bool / double / double
- **Defaults:** `true` / `0.4` / `2.0` (node defaults; **sourced from
  `mowgli_robot.yaml`** and injected at launch by `mowgli.launch.py`, so they
  are operator-tunable from the GUI config without editing `hardware_bridge.yaml`)
- **Description:** Host-side gyro angular-rate PI on `wz`. When enabled,
  `hardware_bridge` closes a yaw-rate loop on the IMU gyro before forwarding the
  twist to firmware. When disabled (`angular_rate_loop_enabled: false`), `wz`
  passes through untouched and the firmware per-wheel velocity PID owns all wheel
  control.
- **Note:** The host loop can ring in yaw (rotation overshoot). On this robot it
  is disabled (`angular_rate_loop_enabled: false`) — the firmware velocity loop
  handles wheel tracking. The default keeps the prior (enabled) behaviour so sim
  and other configs are unchanged.

### Typical Configurations

**High-Performance (Low Latency):**
```yaml
hardware_bridge:
  ros__parameters:
    serial_port: "/dev/ttyACM0"
    baud_rate: 115200
    heartbeat_rate: 10.0      # Fast watchdog feed
    publish_rate: 100.0       # High sensor rate
    high_level_rate: 5.0
```

**Reliable (Conservative):**
```yaml
hardware_bridge:
  ros__parameters:
    serial_port: "/dev/mowgli"
    baud_rate: 115200
    heartbeat_rate: 4.0       # Standard watchdog
    publish_rate: 50.0        # Reduced sensor rate
    high_level_rate: 1.0      # Minimal overhead
```

---

## 2. robot_localization.yaml

**File:** `src/mowgli_bringup/config/robot_localization.yaml`

**Purpose:** Configure robot_localization's dual EKF for sensor fusion (wheel odometry, IMU, GPS).

### Architecture

The system uses **robot_localization** with three cooperating nodes under `two_d_mode` (roll/pitch/Z clamped to zero):

- `ekf_odom_node` (50 Hz) — fuses `/wheel_odom` + `/imu/data` gyro_z, publishes `odom → base_footprint` (continuous dead reckoning, never jumps)
- `navsat_transform_node` (30 Hz) — converts `/gps/fix` + local pose + IMU → `/odometry/gps` in the map frame using a fixed lat/lon datum from `mowgli_robot.yaml`
- `ekf_map_node` (30 Hz) — same wheel + gyro inputs plus `/gps/pose_cov` (the PoseWithCovarianceStamped twin of `/gps/absolute_pose`) and `/imu/cog_heading` (GPS-COG absolute yaw), publishes `map → odom`

**Outputs:**
- `/odometry/filtered` — local EKF pose in odom frame
- `/odometry/filtered_map` — global EKF pose in map frame
- `/tf: odom → base_footprint` (from `ekf_odom_node`)
- `/tf: map → odom` (from `ekf_map_node`)

Non-holonomic motion is enforced by a tight `vy≈0` covariance published on `/wheel_odom`; no pseudo-measurement node is needed.

### Tuning

Full annotated config lives in `src/mowgli_bringup/config/robot_localization.yaml`. Key knobs:

- `process_noise_covariance` (15×15 diagonal under `two_d_mode`): raise x/y/yaw entries if the filter lags the wheels, lower them if GPS updates cause visible snapping.
- `odom0_config` (wheel_odom) fuses `vx` + `vy` + `vyaw` — do not enable absolute pose fusion on the wheel channel.
- `imu0_config` fuses roll+pitch (IMU orientation) + gyro_z. Yaw is sourced from `/imu/cog_heading` (imu1 on `ekf_map_node`) since the IMU has no magnetometer.
- `pose0_config` (`/gps/pose_cov` on `ekf_map_node`) fuses `(x, y)` position only; the EKF honors `NavSatFix.covariance` → RTK-Fixed σ ~3 mm flows through directly.

### Monitoring EKF Health

The diagnostics system monitors `/odometry/filtered_map`:
- **Rate:** Expect ~30 Hz (warn below 10 Hz)
- **Position variance:** Converge to GPS fix precision within a few fixes under RTK-Fixed
- **Orientation:** Yaw covariance should drop once the robot moves forward and `/imu/cog_heading` starts publishing

Access diagnostics at `http://<mower-ip>:4006/#/diagnostics` → Localization section.

---

## 3. nav2_params.yaml

**File:** `src/mowgli_bringup/config/nav2_params.yaml`

**Purpose:** Configure Nav2 navigation stack (planning, control, costmaps).

### Navigation Stack Overview

```
nav2_bringup (lifecycle manager)
  ├── planner_server (global planner)
  │   └── SmacPlanner2D: plans path from start to goal
  │
  ├── controller_server (local planner + motion controller)
  │   └── RegulatedPurePursuit: follows path and generates motor commands
  │
  ├── bt_navigator (behavior tree)
  │   └── Default Nav2 BT: compute path → follow path → success
  │
  ├── nav2_costmap_2d (obstacle map)
  │   ├── Static layer: /map from SLAM
  │   ├── Obstacle layer: /scan from LiDAR
  │   └── Inflation layer: costmap + inflation radius
  │
  └── nav2_map_server (loads static map, if available)
```

### Key Sections

#### bt_navigator Configuration

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_footprint       # REP-105: ground contact point
    odom_topic: /odometry/filtered         # Use local EKF (continuous, odom frame)

    # Behavior tree execution
    bt_loop_duration: 10                   # ms per tick (100 Hz)
    default_server_timeout: 20             # seconds to wait for action servers

    # Custom BT with GoalCheckerSelector for dual-mode navigation
    default_nav_to_pose_bt_xml: "src/mowgli_bringup/config/navigate_to_pose.xml"
    default_nav_through_poses_bt_xml: ""

    # Kilted auto-loads plugins; no manual registration needed
```

#### controller_server Configuration

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false

    # Update rate of velocity commands to motors
    controller_frequency: 10.0             # Hz

    # Minimum velocity thresholds (to avoid numerical issues)
    min_x_velocity_threshold: 0.001        # m/s
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001    # rad/s

    # Failure tolerance (stop after this duration of failed path tracking)
    failure_tolerance: 0.3                 # seconds

    # Plugin selection: dual controller setup
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["stopped_goal_checker", "coverage_goal_checker"]
    controller_plugins: ["FollowPath", "FollowCoveragePath"]

    # Enable stamped velocity commands (Kilted requirement)
    enable_stamped_cmd_vel: false

    # Progress checker: has robot moved at least 0.5 m in 10 seconds?
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5        # m
      movement_time_allowance: 10.0        # seconds

    # Transit goal checker: robot stopped within tolerance of goal pose
    stopped_goal_checker:
      plugin: "nav2_controller::StoppedGoalChecker"
      trans_stopped_velocity: 0.10         # m/s
      xy_goal_tolerance: 0.30              # m (overridden at launch from mowgli_robot.yaml.xy_goal_tolerance)
      yaw_goal_tolerance: 0.5              # rad (overridden at launch)

    # Coverage goal checker: PathProgressGoalChecker (mowgli_nav2_plugins).
    # Subscribes to FTC's republished <plugin_name>/global_plan and only
    # fires when the robot has monotonically tracked >= progress_threshold
    # of the path's poses AND is within xy/yaw tolerance of the goal pose.
    # Falls back to FAILURE when no global_plan arrives within
    # fallback_timeout_s (default 5 s) so the BT can replan.
    # Why not StoppedGoalChecker / SimpleGoalChecker: both fire on
    # proximity / velocity stoppage, which matches FTC's PRE_ROTATE pivots
    # mid-traversal — the action completes at <2 % coverage. Path-progress
    # gating is the only definition of "done" that survives a coverage
    # path whose start and end are both inside the headland ring.
    coverage_goal_checker:
      plugin: "mowgli_nav2_plugins/PathProgressGoalChecker"
      progress_threshold: 0.95
      xy_goal_tolerance: 0.10              # m (overridden at launch from mowgli_robot.yaml.coverage_xy_tolerance, capped at 0.15)
      yaw_goal_tolerance: 0.30
      plan_topic: "/controller_server/FollowCoveragePath/global_plan"
      fallback_timeout_s: 5.0

    # ─────────────────────────────────────────────────────────────
    # FollowPath (transit) AND FollowCoveragePath (mowing strips):
    # both use the same FTCController plugin. Single source of truth
    # for tuning. RPP/RotationShim/MPPI are no longer in the active
    # stack — FTC handles pivots and obstacle deviation natively.
    # ─────────────────────────────────────────────────────────────
    FollowPath: &FTC
      plugin: "mowgli_nav2_plugins/FTCController"
      speed_fast: 0.20                     # m/s (overridden at launch from transit_speed / mowing_speed)
      speed_slow: 0.12
      speed_angular: 45.0                  # deg/s
      acceleration: 0.2
      kp_lon: 1.0
      kp_lat: 2.0
      kd_lat: 1.0
      kp_ang: 1.5
      max_cmd_vel_speed: 0.30
      max_cmd_vel_ang: 0.8
      max_goal_distance_error: 0.10
      max_goal_angle_error: 15.0           # deg
      goal_timeout: 10.0
      max_follow_distance: 2.0
      forward_only: false
      check_obstacles: true
      obstacle_lookahead: 30               # ~1.5 m at F2C 0.05 m sampling
      obstacle_footprint: true
      enable_obstacle_deviation: true
      max_lateral_deviation: 1.5
      deviation_step: 0.05
      deviation_blend_rate: 0.5

    FollowCoveragePath: *FTC
```

#### planner_server Configuration

```yaml
planner_server:
  ros__parameters:
    use_sim_time: false

    # Expected planner plugins
    planner_plugins: ["GridBased"]

    # ─────────────────────────────────────────────────────────────
    # SmacPlanner2D: A* search on 2D grid
    # ─────────────────────────────────────────────────────────────
    GridBased:
      plugin: "nav2_smac_planner::SmacPlanner2D"

      # Planning parameters
      tolerance: 0.125                     # m, tolerance to goal
      downsample_costmap: false            # Use full resolution
      downsampling_factor: 1

      # Search algorithm tuning
      angle_quantization_bins: 72          # 5° angle resolution (360/72)
      maximum_iterations: 1000             # Max A* iterations
      max_planning_time: 5.0               # Max planning time (seconds)

      # Cost function
      w_cost: 100.0                        # Weight for path length vs. safety
      w_heuristic: 100.0                   # Weight for heuristic (A* guidance)

      # Lethal cost threshold (treat as obstacle if > this)
      lethal_cost: 252.0

      # Motion primitives (allowed moves)
      # For a differential drive, typical 16-direction grid
      use_grid_path_stitching: true
```

#### costmap_2d Configuration

```yaml
# Global costmap (used by planner)
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0                # Hz (slower for global map)
      publish_frequency: 1.0               # Hz
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false

      # Plugins (layers)
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      # Static layer: SLAM-generated map
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true
        subscribe_to_updates: true

      # Obstacle layer: LiDAR detects new obstacles
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.5         # m (tallest obstacle to consider)
          min_obstacle_height: 0.0         # m
          clearing: true                   # LiDAR clears old obstacles
          marking: true                    # LiDAR marks new obstacles
          expected_update_rate: 0.0        # No timeout (use latest)

      # Inflation layer: create safety buffer around obstacles
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.55             # m (half robot width + clearance)
        cost_scaling_factor: 10.0

      # Resolution and bounds
      resolution: 0.05                     # m/pixel (fine resolution for precise paths)
      width: 200                           # Grid width (pixels)
      height: 200                          # Grid height (pixels)
      origin_x: -5.0                       # m (relative to global origin)
      origin_y: -5.0                       # m

# Local costmap (used by controller, smaller window around robot)
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0                # Hz (faster for reactive obstacle avoidance)
      publish_frequency: 5.0               # Hz
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false

      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: true
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.5
          min_obstacle_height: 0.0
          clearing: true
          marking: true
          expected_update_rate: 0.0

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.55
        cost_scaling_factor: 10.0

      resolution: 0.05
      width: 50                            # Smaller window: ~2.5 m × 2.5 m
      height: 50
      origin_x: -1.25
      origin_y: -1.25
```

#### velocity_smoother Configuration

The velocity smoother applies acceleration limits to reduce jerky motion:

```yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0              # Hz
    scale_velocities: false
    feedback: "odometry"
    max_velocity: [0.5, 0.0, 0.8]          # [linear_x, linear_y, angular_z]
    max_accel: [0.4, 0.0, 1.0]             # Acceleration limits (m/s², rad/s²)
    max_decel: [0.4, 0.0, 1.0]             # Deceleration limits
    odom_topic: "/odometry/filtered"
    cmd_vel_in_topic: "/cmd_vel"
    cmd_vel_out_topic: "/cmd_vel_smoothed"
```

---

## 4. coverage_server (mowgli_coverage / Fields2Cover v2.0)

**Config:** `coverage_server` block inside `src/mowgli_bringup/config/nav2_params.yaml`.
**Action:** `compute_coverage_path` (type `opennav_coverage_msgs/action/ComputeCoveragePath`).
**Backend:** Fields2Cover **v2.0.0** at `/opt/fields2cover-200`. Legacy v1.2.1 at `/opt/fields2cover-121` is kept on the global `ld` path while the migration finishes; `mowgli_coverage`'s `CMakeLists.txt` sets explicit `INSTALL_RPATH`s on both `libmowgli_coverage_core.so` and the executable so the loader picks v2.

```yaml
coverage_server:
  ros__parameters:
    use_sim_time: false
    default_headland_width: 0.20         # m – headland inset width
    robot_width: 0.20                    # m – physical chassis width (collision)
    operation_width: 0.18                # m – F2C swath spacing (Robot::setCovWidth)
    # OVERRIDDEN at launch from mowgli_robot.yaml.tool_width — single source
    # of truth for blade cut width. The previous static 0.20 left thin un-mowed
    # strips between adjacent F2C swaths because map_server's stamp radius
    # (tool_width / 2) was narrower than F2C's spacing.
    min_turning_radius: 0.05             # m – differential drive pivots in place
    linear_curv_change: 200.0
    coordinates_in_cartesian_frame: true
    # Mode-string params declared for compatibility with the legacy
    # opennav_coverage YAML schema. The v2 server uses a fixed pipeline
    # (BruteForce → BoustrophedonOrder → DubinsCurvesCC + connectors)
    # and logs a line on startup if the operator overrode any of these.
    default_swath_angle_type: "BRUTE_FORCE"
    default_swath_type: "LENGTH"
    default_route_type: "BOUSTROPHEDON"
    default_path_type: "DUBIN"
    default_path_continuity_type: "DISCONTINUOUS"
```

**Pipeline:** `f2c::hg::ConstHL.generateHeadlands` (inset) + `generateHeadlandSwaths(cov_width, n_passes, dir_out2in=true)` (concentric perimeter passes, densified to 10 cm steps in our wrapper) → `f2c::decomp::TrapezoidalDecomp.decompose(inner)` (split around interior holes) → per sub-cell `f2c::sg::BruteForce.generateBestSwaths` → `f2c::rp::BoustrophedonOrder.genSortedSwaths` → `f2c::pp::PathPlanning.planPath(robot, ordered, DubinsCurvesCC)` + `pp.planPathForConnection(...)` Dubins arcs between sub-cells. The result is converted to `nav_msgs/Path` and returned via the action result.

The BT side (`PlanCoverageArea`) feeds it the area outer ring + `mow_progress` holes via `/map_server_node/get_remaining_area_polygon`. The legacy `opennav_coverage` upstream is a git submodule for the `_msgs` action definitions only — every server subpackage is `COLCON_IGNORE`'d.

---

## 5. fusion_graph

**Opt-in factor-graph localizer** that replaces `ekf_map_node` 1-for-1 when `use_fusion_graph:=true`. There is **no** dedicated YAML config — knobs are declared as ROS2 parameters on `fusion_graph_node` and the high-level switches (`use_fusion_graph`, `use_scan_matching`, `use_loop_closure`, `use_magnetometer`) live in `mowgli_robot.yaml`. The Settings page exposes them under the *Localization* section.

See [`wiki/Architecture.md`](Architecture.md#optional-factor-graph-localizer-fusion_graph) for the full design — Pose2 graph at 10 Hz, custom `GnssLeverArmFactor`, optional LiDAR scan-matching between-factors and loop-closure factors.

---

## 6. twist_mux.yaml

**File:** `src/mowgli_bringup/config/twist_mux.yaml`

**Purpose:** Priority-based multiplexing of velocity commands from multiple sources.

**Full Configuration:**

```yaml
twist_mux:
  ros__parameters:
    # Input topics (velocity sources)
    # Topics evaluated in ascending order of priority
    # Higher priority sources suppress lower priority when active
    topics:
      # Lowest priority: autonomous navigation
      navigation:
        topic: /cmd_vel_nav
        timeout: 0.5                 # seconds (stale if no message for 0.5 s)
        priority: 10                 # Lowest priority

      # Medium priority: manual teleoperation
      teleop:
        topic: /cmd_vel_teleop
        timeout: 0.5
        priority: 20                 # Override navigation

      # Highest velocity priority: emergency commands
      emergency:
        topic: /cmd_vel_emergency
        timeout: 0.2                 # Tighter timeout for safety-critical
        priority: 100                # Highest velocity priority

    # No `locks:` block.
    #
    # Emergency stop is enforced by the STM32 firmware (CLAUDE.md
    # invariant 9 — firmware is the sole safety authority). An earlier
    # configuration declared an /emergency_stop std_msgs/Bool lock with
    # priority 255, but no node in the codebase ever published to that
    # topic — the lock was dead and gave a false sense of software-side
    # redundancy. Real e-stops travel via the
    # /hardware_bridge/emergency_stop service which sets the firmware
    # latch; the firmware then refuses to forward cmd_vel to the motors.
    #
    # If a software-side mux lock is ever wanted in future, wire a
    # publisher in hardware_bridge_node first, then re-add the locks
    # block. The key is intentionally OMITTED (not `locks: {}`) because
    # ROS2 cannot infer the type of an empty YAML mapping at lifecycle
    # bring-up.
```

### Priority Resolution

**Example Scenario:**

```
Time 0: Navigation publishes cmd_vel_nav (0.1 m/s)
  → Output: 0.1 m/s (only source active)

Time 1: Teleop publishes cmd_vel_teleop (0.2 m/s)
  → Output: 0.2 m/s (teleop priority 20 > navigation priority 10)

Time 2: Emergency publishes cmd_vel_emergency (0.3 m/s)
  → Output: 0.3 m/s (emergency priority 100 > all others)

Time 3: All sources timeout or become stale
  → Output: 0 m/s (no active source, robot stops)

Time 4: Emergency latch asserted at the firmware via
        /hardware_bridge/emergency_stop service
  → Firmware refuses to forward cmd_vel to motors regardless of what
    twist_mux outputs. (twist_mux itself has no software lock — see
    "No `locks:` block" note above.)
```

### Service Interface

```bash
# Assert the firmware emergency latch (the only e-stop path).
# The latch is held inside the STM32 firmware; ROS2 publishing on
# /cmd_vel cannot move the motors while it's asserted.
ros2 service call /hardware_bridge/emergency_stop \
  mowgli_interfaces/srv/EmergencyStop "{emergency: true}"

# Release the latch. The firmware only actually clears the latch if
# the physical trigger (lift / tilt / e-stop button) is no longer
# asserted — firmware is the sole safety authority.
ros2 service call /hardware_bridge/emergency_stop \
  mowgli_interfaces/srv/EmergencyStop "{emergency: false}"
```

### Typical Deployments

**Autonomous Mowing (Default):**
```yaml
# High-priority emergency source, low-priority autonomous
# Allows emergency override at any time
```

**Teleoperation (Manual Control):**
```yaml
topics:
  teleop:
    priority: 10
  emergency:
    priority: 100
# Remove navigation source entirely
```

---

## 7. fusion_graph (factor-graph localizer) {#7-fusion_graph}

**Files:** none — `fusion_graph_node` declares all knobs as ros2 parameters at startup. The high-level toggles live in `mowgli_robot.yaml`; runtime overrides are passed on the `navigation.launch.py` command line.

**Purpose:** opt-in replacement for `ekf_map_node` built on **GTSAM iSAM2**. Same wire contract — `/odometry/filtered_map` + `map → odom` TF — but the map-frame estimate is the result of a Pose2 factor graph that can carry LiDAR scan-matching and loop-closure factors through extended RTK-Float windows. See [Architecture → Optional: Factor-Graph Localizer](Architecture#optional-factor-graph-localizer-fusion_graph) for the steady-state design.

### Activation

```bash
# Per-launch override (one-shot)
ros2 launch mowgli_bringup navigation.launch.py use_fusion_graph:=true

# Persistent: set in mowgli_robot.yaml (also exposed in the Settings → Localization section)
mowgli:
  ros__parameters:
    use_fusion_graph: true
    use_scan_matching: true
    use_loop_closure: true
    use_magnetometer: false
```

The legacy EKF (`ekf_map_node`) and `fusion_graph_node` are **mutually exclusive** in `navigation.launch.py`: only one publishes `map → odom`. `ekf_odom_node` (local EKF, `odom → base_footprint`) runs in both modes.

### Key parameters

| Parameter | Default | Notes |
|---|---|---|
| `node_period_s` | 0.1 | Graph node creation cadence (10 Hz). |
| `stationary_node_period_s` | 5.0 | Throttled node period when motion is below the stationary threshold — bounds graph growth on the dock. |
| `wheel_sigma_x / sigma_y / sigma_theta` | 0.05 / 0.005 / 0.01 | Body-frame between-factor noise. `sigma_y` ≪ `sigma_x` enforces non-holonomic motion. |
| `gyro_sigma_theta` | 0.005 | Yaw between-factor noise from `/imu/data`. |
| `gps_sigma_floor` | 0.003 | Lower bound for the GPS XY noise (3 mm) — prevents over-trusting RTK-Fixed reports with under-estimated covariance. |
| `cov_update_every_n` | 10 | Skip-rate for the marginal covariance recompute (the diagonals on `/odometry/filtered_map`). |
| `isam2_relinearize_skip` | 5 | iSAM2 relinearization throttle. |
| `isam2_rebase_every_nodes` | 2000 | Periodic iSAM2 rebase to bound per-tick update cost. |
| `scan_retention_nodes` | 18000 | Drop body-frame scans older than this many nodes (~30 minutes at 10 Hz). |
| `lc_max_dist_m` / `lc_min_age_s` / `lc_max_candidates` / `lc_max_rmse` | 5.0 / 600.0 / 3 / 0.20 | Loop-closure search/accept gates. |
| `icp_max_iter` / `icp_max_corresp_dist` / `icp_source_subsample` | 10 / 0.5 / 1 | Per-tick scan matcher; ten iterations converge within 1 mm of the 15-iteration solution on outdoor LiDAR shapes. |
| `autoload_graph` | true | Resume from `<graph_save_prefix>.{graph,scans,meta}` on startup. |
| `auto_save_enabled` | true | Auto-checkpoint on RECORDING→IDLE, dock arrival, and every `periodic_save_period_s` during AUTONOMOUS state. |
| `graph_save_prefix` | `/ros2_ws/maps/fusion_graph` | Base path for the three persistence files. |
| `primary_mode` | true | Broadcast `map → odom` TF. Set false to run as an observer alongside `ekf_map_node`. |

### Topics, services

- **`/fusion_graph/diagnostics`** (`diagnostic_msgs/DiagnosticArray`, 1 Hz) — exposes `total_nodes`, `scans_attached`, `loop_closures`, `scans_received`, `scan_matches_ok`, `scan_matches_fail`, `cov_xx`, `cov_yy`, `cov_yawyaw`. Surfaced in the GUI's *Diagnostics → Fusion Graph (iSAM2)* panel.
- **`/fusion_graph/markers`** (`visualization_msgs/MarkerArray`, 1 Hz, transient_local) — node positions, trajectory, loop-closure edges. Visible in Foxglove with no extra setup.
- **`/imu/fg_yaw`** (`sensor_msgs/Imu`, 10 Hz) — yaw-only output that can be fused by `ekf_map_node` as a tight yaw source in observer mode (replacing `/imu/mag_yaw` when the magnetometer is unreliable).
- **`~/save_graph`** (`std_srvs/Trigger`) — persists the graph immediately. Wired to the *Save graph* button in the GUI.
- **`~/clear_graph`** (`std_srvs/Trigger`) — wipes the graph. The next valid pose seed (GPS, set_pose, or scan-match relocalization) re-initializes. Wired to the *Clear graph* button in the GUI.

### Persistence

Graph state lives on disk under `<graph_save_prefix>.*`:

- `.graph` — gtsam factor graph + optimized values (XML).
- `.scans` — binary blob: per-node body-frame LiDAR points.
- `.meta` — text: next index, last node time, datum lat/lon.

Idempotent overwrite. Saving from the GUI button is identical to the auto-checkpoint that fires on dock arrival; the operator typically only invokes Save explicitly before manually shutting down ROS2.

### Tuning notes

- **Drift after a long RTK-Float window**: lower `gps_sigma_floor` only if you trust RTK-Fixed bursts more than the wheel/scan factors — most installations should leave it at 3 mm.
- **CPU budget**: scan-matching costs ~5 ms/tick at 10 Hz on a Pi 4. If you see the maintenance timer overrunning, raise `isam2_relinearize_skip` to 10 or set `icp_source_subsample` to 2 before disabling `use_scan_matching` outright.
- **Graph too large after weeks**: tune `isam2_rebase_every_nodes` down to 1500 — the rebase preserves the optimized values but drops accumulated between-factors.
- **LiDAR is unreliable in winter (snow on rotor, low visibility)**: leave `use_scan_matching:=true`, just disable `use_loop_closure` to avoid a stale match getting promoted to a loop-closure factor.

---

## Parameter Tuning Workflow

### Step 1: Identify Performance Issue

| Issue | Likely Culprit | Action |
|-------|----------------|--------|
| Robot drifts without sensor updates | EKF process noise too low | Increase `process_noise_covariance` in localization.yaml |
| Robot ignores GPS corrections | EKF process noise too high | Decrease `process_noise_covariance` |
| Path tracking oscillates | Lookahead or velocity scaling too aggressive | Adjust `lookahead_dist`, `desired_linear_vel` in RegulatedPurePursuit |
| Robot can't follow curves | Lookahead distance too short | Increase `lookahead_dist` in nav2_params.yaml |
| Planner is very slow | Grid resolution too fine or search space too large | Increase resolution (0.05 → 0.10) or reduce grid size |
| SLAM diverges in loop closure | Loop closure parameters too aggressive | Decrease `loop_match_minimum_chain_size`, increase `loop_search_maximum_distance` |

### Step 2: Modify Parameters

Edit the relevant YAML file:
```bash
nano src/mowgli_bringup/config/localization.yaml
```

### Step 3: Test and Monitor

```bash
# Launch with new parameters
ros2 launch mowgli_bringup mowgli.launch.py

# Monitor topics in RViz
rviz2 -d src/mowgli_bringup/config/mowgli.rviz

# Check diagnostics
ros2 topic echo /localization/status
ros2 topic echo /odometry/filtered_map
```

### Step 4: Iterate

Rerun with adjusted parameters, observe results, adjust again.

---

## Reference: Default Parameters Quick Lookup

| Parameter | Default | Unit | Range |
|-----------|---------|------|-------|
| `serial_port` | `/dev/mowgli` | – | any `/dev/tty*` |
| `baud_rate` | 115200 | baud | 9600–230400 |
| `ekf_odom_node` frequency | 50.0 | Hz | 30–100 |
| `SLAM` frequency | 20.0 | Hz | 5–50 |
| `controller_frequency` | 10.0 | Hz | 5–50 |
| `desired_linear_vel` | 0.3 | m/s | 0.1–1.0 |
| `lookahead_dist` | 0.6 | m | 0.2–1.5 |
| `linear_p_gain` | 2.0 | – | 0.5–5.0 |
| `slam_resolution` | 0.05 | m/pixel | 0.01–0.20 |
| `loop_search_max_distance` | 3.0 | m | 1.0–5.0 |

---

**For detailed system architecture, see [ARCHITECTURE.md](ARCHITECTURE.md).**

**For firmware integration, see [FIRMWARE_MIGRATION.md](FIRMWARE_MIGRATION.md).**
