# Configuration Reference

Complete guide to all configuration files and parameters in the Mowgli ROS2 system.

This documentation is for ROS2 Kilted. The simulator is Webots — see [Simulation](Simulation).

[CLAUDE.md](https://github.com/mowglinext/mowglinext/blob/main/CLAUDE.md) is the authoritative short-form reference. If any section here contradicts it, CLAUDE.md wins.

## Overview

Configuration is centralized in `src/mowgli_bringup/config/`:

```
config/
├── hardware_bridge.yaml          # Serial port, baud, publish rate, IMU cal sample
│                                 #   count, wheel-slip dig detection
├── nav2_params_base.yaml         # Navigation stack — everything COMMON to both variants
├── nav2_params_lidar.yaml        # LiDAR overlay (obstacle layers, scan collision_monitor)
├── nav2_params_no_lidar.yaml     # GPS-only overlay (static layers, pass-through monitor)
├── twist_mux.yaml                # Velocity multiplexer priorities
├── foxglove_bridge.yaml          # Foxglove Studio visualization bridge
└── mowgli_robot.yaml             # In-package TEMPLATE: the default value of
                                  #   every robot parameter (see "Sparse robot
                                  #   config model" below)
```

The **installed** robot config lives outside the package at `/ros2_ws/config/mowgli_robot.yaml` (bind-mounted from `install/config/mowgli/`). See the next section — it is **sparse**, and the in-package `config/mowgli_robot.yaml` above is its **template of defaults**.

There is **no** `robot_localization.yaml`, `slam_toolbox.yaml` or `kiss_icp.yaml`. The dual EKF (`ekf_map_node` + `ekf_odom_node`), slam_toolbox and Kinematic-ICP were all removed; `fusion_graph_node` is the **sole, unconditional** localizer and owns both `map → odom` and `odom → base_footprint`. It has no config file of its own — see [§5](#5-fusion_graph).

Which overlay is deep-merged onto `nav2_params_base.yaml` — the LiDAR-aware `nav2_params_lidar.yaml` or the GPS-only `nav2_params_no_lidar.yaml` — is decided by the **`lidar_enabled`** key in `mowgli_robot.yaml`, and by **nothing else**. The `LIDAR_ENABLED` environment variable is **not read by the ROS2 stack at all** — it used to be a "fallback when the yaml is silent", and that fallback let a stale installer `.env` silently override the operator's GUI toggle (config absent + `LIDAR_ENABLED=false` ran the whole stack GPS-only while the GUI said LiDAR on). A CLI/compose `use_lidar:=` override still wins for one-off runs, because typing it is a deliberate act.

If `lidar_enabled` is **absent** from the installed config, the stack resolves **`use_lidar=false`** and prints a prominent startup warning naming the file, the key and the resolved mode. Absence means no LiDAR was ever recorded (`mowglinext.sh` always writes the key), and a wrongly-off stack is coherent GPS-only operation whereas a wrongly-on one is a half-state: obstacle layer with no observation source, scan-based collision monitor with a dead source, and `fusion_graph` subscribing to a topic nothing publishes.

`LIDAR_ENABLED` in `.env` still does one real job: it decides whether the **`mowgli-lidar` container** is composed in. So the two can now disagree in the opposite direction (config on, container never started). `scan_deskew_node` — which only runs when `use_lidar` is true — warns loudly if no scan arrives within `scan_watchdog_period_s` (20 s default) and names that cause.

`use_scan_matching` and `use_loop_closure` are **ANDed with `use_lidar`** before they reach `fusion_graph_node`, so the "scan-matching enabled with no scanner" state is unreachable.

The **fusion_graph** localizer (GTSAM iSAM2 — see [§7](#7-fusion_graph)) does **not** have a separate config file: its knobs are declared as ros2 parameters on `fusion_graph_node` and the high-level switches (`use_scan_matching`, `use_loop_closure`, `use_magnetometer`, `fusion_graph_node_period_s`) live in `mowgli_robot.yaml`. There is **no** `use_fusion_graph` switch — the node is launched unconditionally. The Settings page exposes the switches under the *Localization* section.

### Sparse robot config model

`mowgli_robot.yaml` exists in **two** places, and they play different roles:

| File | Role |
|------|------|
| `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` | **Template** — ships with the software, versioned, holds the **default value of every robot parameter**. |
| `/ros2_ws/config/mowgli_robot.yaml` (bind-mounted from `install/config/mowgli/`) | **Installed config** — **sparse**, holds only per-robot values (see below). |

The installed file is **deliberately sparse**. It contains only:

- **Install-time choices** — `datum_lat`/`datum_lon`, NTRIP credentials, `lidar_enabled`, GNSS hardware (`gnss_receiver_family`, `gnss_serial_device`, `gnss_serial_baud`) and its auto-applied receiver profile (`gnss_config_apply_enabled`, `gnss_config_profile`, `gnss_signal_profile`), `mower_model`.
- **Calibration outputs** — `dock_pose_x/y/yaw`, `ticks_per_meter`, the `wheel_pid_*` gains, `imu_yaw`, and the magnetometer settings (`enable_mag_cal`, `declination_deg`).

**Every other parameter's default lives in the in-package template.** At launch, `ros2/src/mowgli_bringup/launch/robot_config_util.py` (`load_robot_params`) **deep-merges the sparse installed config *over* the template**, so any key absent from the installed file falls through to its template default. Nodes always receive a complete parameter set — only the on-disk installed file is allowed to be sparse.

Consequences:

- **Template edits propagate.** A maintainer bumping a default in the in-package template changes that value on **every robot** whose installed config does not explicitly override it. Reserve the sparse installed file for genuine per-robot overrides.
- **Reset to default from the GUI.** The Settings page shows a small **dot** next to any value that differs from its default and a **reset (undo) button** to revert it. Reset works by **deleting the key** from the installed file so it falls back to the template — the GUI backend also **prunes any key whose saved value equals its default on write**, keeping the installed config sparse automatically.
- **Older full configs still work.** A pre-split *full* installed `mowgli_robot.yaml` merges to itself (every key overrides its identical template default — a no-op merge), so upgrading needs no migration.

> **To change a value:** set it in the installed config (it overrides the template). **To restore a default:** delete its line from the installed config (do NOT copy the template value back in) — or use the GUI reset button.

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

    # Bind address (0.0.0.0 = reachable from the LAN)
    address: "0.0.0.0"

    # Per-client send buffer
    send_buffer_limit: 10000000      # bytes (~10 MB per client)

    # 0 = let foxglove_bridge size its thread pool
    num_threads: 0

    # Keep sidecar-internal Universal GNSS topics off the public GUI surface.
    # Everything NOT matched by these negative look-aheads is advertised —
    # there is no explicit subscribe list; the bridge offers every topic.
    topic_whitelist:
      - "^(?!/(?:_gps_internal|gps_internal|universal_gnss)(?:/.*)?$).*"
    client_topic_whitelist:
      - "^(?!/(?:_gps_internal|gps_internal|universal_gnss)(?:/.*)?$).*"
```

### Key Parameters

#### `port`

- **Type:** integer
- **Default:** `8765`
- **Description:** WebSocket server port for Foxglove Studio connections
- **Note:** Ensure this port is not blocked by firewall on deployment machine

#### `address`

- **Type:** string
- **Default:** `"0.0.0.0"`
- **Description:** Interface the WebSocket server binds to; `0.0.0.0` accepts connections from any host on the LAN

#### `send_buffer_limit`

- **Type:** integer (bytes)
- **Default:** `10000000` (~10 MB)
- **Description:** Per-client outgoing buffer; messages are dropped for a client that falls this far behind

#### `topic_whitelist` / `client_topic_whitelist`

- **Type:** list of regexes
- **Description:** Which topics the bridge advertises (`topic_whitelist`) and which a client may subscribe to (`client_topic_whitelist`). The shipped patterns are **negative look-aheads**: everything is exposed *except* the GPS sidecar's internal `/_gps_internal`, `/gps_internal` and `/universal_gnss` trees.
- **Performance:** The bridge only pays for topics a client actually subscribes to, so a broad whitelist costs nothing until someone opens the panel.

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

    # Serial-link RX watchdog: reopen the port if no STM32 bytes arrive for
    # this long (auto-reconnect after a firmware flash / board reboot / USB
    # re-enumeration, which otherwise leaves the bridge writing to a dead fd)
    serial_rx_timeout_s: 2.0

    # Sensor read tick (Hz). The actual /imu/data rate is capped by the
    # firmware packet rate (~47 Hz); a 100 Hz tick just makes sure every
    # available packet is consumed without serial backlog.
    publish_rate: 100.0

    # IMU boot-time / re-dock calibration sample count (~20 s at ~48 Hz)
    imu_cal_samples: 1000

    # Wheel-slip dig detection — see "Dig detection" below
    dig_detect_enabled: true
    dig_window_s: 1.2
    dig_min_cmd_speed: 0.05
    dig_min_wheel_dist: 0.15
    dig_progress_fraction: 0.35
    dig_max_pos_sigma: 0.10
    dig_max_yaw_rate: 0.20
    dig_reverse_speed: 0.12
    dig_reverse_dist: 0.30
    dig_reverse_timeout_s: 4.0
    dig_monitor_rate: 10.0
```

> `high_level_rate` is a declared node parameter (default `2.0` Hz) but is **not** listed in the shipped yaml — add a line only if you need to change it.

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
- **Description:** Rate at which hardware_bridge reads the serial link and publishes sensor data to ROS2 topics
- **Affects:** `~/status`, `~/power`, `~/imu/data_raw`, `~/wheel_odom`
- **Note:** In `mowgli.launch.py` these node-local names are remapped to `/hardware_bridge/status`, `/hardware_bridge/power`, `/imu/data` and `/wheel_odom` respectively.
- **Upstream drivers:** The firmware packet rate (~47 Hz) is the real ceiling; the 100 Hz tick only guarantees packets are drained promptly
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

#### Yaw-rate loop — now firmware-side (Option C, 2026-07-17)

- **History:** `hardware_bridge` used to run a host-side closed-loop gyro
  angular-rate PI (`angular_rate_loop_enabled` / `angular_rate_kp` /
  `angular_rate_ki` / `angular_rate_kff`, `angular_rate_controller.hpp`) that
  closed a yaw-rate loop on the IMU gyro before forwarding `wz` to firmware.
  It went through an "Option B" refit (task #24, ki→0/kff refit) to fix a
  left-right weave caused by a 3-loop integrating cascade (FTC heading →
  host PI → per-wheel firmware PIs) fighting itself over the ~50-90 ms USB
  round trip.
- **REMOVED (Option C, task #33/#34):** the whole host-side loop —
  `angular_rate_controller.hpp`, the four `angular_rate_*` params, and their
  GUI catalog entries — is gone. `hardware_bridge` now sends the commanded
  `wz` straight through with no shaping. The yaw-rate loop moved INTO
  FIRMWARE (task #33), which closes it on the same gyro without the host's
  USB round-trip latency — collapsing the 3-loop cascade to 2 without the
  Option B feed-forward-refit tradeoffs. Tunable via the firmware's
  `yaw_kp`/`yaw_ki` (sent over `SET_DRIVE_PID`); GUI catalog entries for
  those will land once the firmware param interface is finalized.

#### Dig detection (`dig_*`)

Wheel-slip **dig** detection lives in `hardware_bridge_node` (`mowgli_hardware/dig_detector.hpp`) because `~/cmd_vel` is twist_mux's *merged* output — one check therefore covers every motion lane (coverage, transit, docking, teleop).

It is the only wheel-**independent** stuck check on the robot: it compares the encoders' claimed travel against the GNSS-anchored fused pose (`/odometry/filtered_map`) over `dig_window_s`, and on a mismatch hard-stops on the wire and drives a bounded reverse that `on_cmd_vel` cannot override. The bridge then publishes `~/dig_event`, and `map_server` promotes the spot to a permanent keepout so coverage routes around it next pass.

| Parameter | Default | Notes |
|---|---|---|
| `dig_detect_enabled` | `true` | Master switch. |
| `dig_window_s` | 1.2 | Sustained evidence before latching. Plus one monitor tick this is the detection floor (~1.3 s); it is the false-fire guard, deliberately not shortened. |
| `dig_min_cmd_speed` | 0.05 | Below this we aren't commanding travel [m/s]. |
| `dig_min_wheel_dist` | 0.15 | The **worst wheel** must claim this much ground travel first [m]. Measured on `max(abs(dL), abs(dR))`, not the chassis centre — digs are asymmetric (issue #499). Without it a *blocked* wheel (the firmware's case) looks identical. |
| `dig_progress_fraction` | 0.35 | Latch if map travel < 35 % of wheel travel. Map travel is **net displacement** across the window, never a sum of per-tick steps. |
| `dig_max_pos_sigma` | 0.10 | Stand down above this fused-pose 1σ — fed from the **receiver's** horizontal accuracy under a confirmed RTK-Fixed solution, not the factor graph's marginal. Under Float the map pose cannot tell slip from GNSS noise. |
| `dig_max_yaw_rate` | 0.20 | Stand down above this **gyro** yaw rate: through a turn the encoders measure arc while the map measures chord. Gyro, never wheel-derived yaw. |
| `dig_reverse_speed` / `dig_reverse_dist` / `dig_reverse_timeout_s` | 0.12 / 0.30 / 4.0 | Bounded escape reverse [m/s, m, s]. |
| `dig_monitor_rate` | 10.0 | Monitor tick [Hz]. |
| `dig_gnss_timeout_s` / `dig_gyro_timeout_s` / `dig_pose_timeout_s` | 2.0 / 0.5 / 1.0 | Stale-input stand-down windows [s]. |

The firmware `ANTIDIG_*` gate is unchanged and remains the un-bypassable backstop for the complementary case (**blocked** wheels), which this detector deliberately ignores via `dig_min_wheel_dist`.

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

## 3. nav2_params_base.yaml (+ LiDAR / no-LiDAR overlay)

**Files:** `src/mowgli_bringup/config/nav2_params_base.yaml` deep-merged with **exactly one** of `nav2_params_lidar.yaml` / `nav2_params_no_lidar.yaml` (chosen by the `use_lidar` arg in `navigation.launch.py`).

**Purpose:** Configure Nav2 navigation stack (planning, control, costmaps).

The base file holds everything **common** to the two variants; each overlay carries **only** the genuine differences (costmap obstacle vs static layers, scan-based vs pass-through `collision_monitor`, `FollowPath.use_collision_detection`, FTC's obstacle flags). **Edit shared params in the base, variant params in the overlay** — `test_nav2_params.py` validates the merged result and pins the two variants in lockstep.

### Navigation Stack Overview

```
nav2 (lifecycle manager)
  ├── planner_server (global planner)
  │   └── SmacPlanner2D: plans path from start to goal
  │
  ├── controller_server (local planner + motion controller)
  │   ├── FollowPath          → RotationShim + RegulatedPurePursuit (transit)
  │   └── FollowCoveragePath  → mowgli_nav2_plugins/FTCController (mowing)
  │
  ├── bt_navigator (behavior tree)
  │   └── Custom navigate_to_pose.xml with GoalCheckerSelector
  │
  ├── nav2_costmap_2d (obstacle map)
  │   ├── Obstacle layer: /scan_costmap from LiDAR (LiDAR variant)
  │   ├── Static layer: /no_lidar_static_map (GPS-only variant)
  │   ├── Keepout filter: area boundary mask from map_server (global only)
  │   └── Inflation layer: costmap + inflation radius
  │
  ├── behavior_server (spin / backup / wait — undock uses BackUp)
  ├── docking_server (opennav_docking, cmd_vel on /cmd_vel_docking)
  ├── collision_monitor (/cmd_vel_nav → /cmd_vel_monitored)
  └── coverage_server (mowgli_coverage — see §4)
```

### Key Sections

#### bt_navigator Configuration

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_footprint       # REP-105: ground contact point
    odom_topic: /odometry/filtered         # continuous odom-frame pose

    # Behavior tree execution
    bt_loop_duration: 10                   # ms per tick (100 Hz)
    # 1000 ms (was 60): IsPathValid calls planner_server's is_path_valid at
    # 1 Hz on planner_server's own callback group, which easily exceeds 60 ms
    # on ARM — the timeout aborted NavigateToPose and broke HOME/docking.
    default_server_timeout: 1000           # milliseconds
    # 10 s: on ARM the action servers are not registered in the DDS graph yet
    # when bt_navigator activates, and the 1 s default aborted bring-up.
    wait_for_service_timeout: 10000        # milliseconds

    # Custom BT with GoalCheckerSelector for dual-mode navigation.
    # Paths are INJECTED at launch via RewrittenYaml — keep these empty here.
    default_nav_to_pose_bt_xml: ""
    default_nav_through_poses_bt_xml: ""

    enable_stamped_cmd_vel: true           # Kilted: all Nav2 nodes use TwistStamped
    # Kilted auto-loads plugins; no manual registration needed
```

#### controller_server Configuration

```yaml
controller_server:
  ros__parameters:
    use_sim_time: false
    enable_stamped_cmd_vel: true           # Kilted requirement

    # Velocity feedback for the controllers. MUST be set: Nav2 defaults to
    # "odom", which NOTHING publishes on this robot, so RPP/FTC would get zero
    # velocity feedback (RPP's velocity-scaled lookahead collapses → transit
    # weaving). /odometry/filtered is not usable yet (twist.angular.z stays 0).
    odom_topic: /wheel_odom

    # Update rate of velocity commands to motors. 20 Hz was chronically
    # missed by the ARM host (10-25 Hz jitter), which hurt FTC tracking more
    # than the lower nominal rate does.
    controller_frequency: 10.0             # Hz

    # Plan→costmap transform tolerance (separate from each plugin's own).
    controller.transform_tolerance: 0.5    # s
    costmap_update_timeout: 5.0            # s — ARM costmap stalls

    # Velocity thresholds DISABLED: hardware_bridge has a closed-loop deadband
    # compensator, and the old 0.001/0.08 floors zeroed ramp-up commands before
    # they ever reached it, leaving the wheels parked in deadband.
    min_x_velocity_threshold: 0.0
    min_y_velocity_threshold: 0.0
    min_theta_velocity_threshold: 0.0

    failure_tolerance: 1.0                 # seconds

    # Plugin selection: dual controller setup
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["stopped_goal_checker", "coverage_goal_checker"]
    controller_plugins: ["FollowPath", "FollowCoveragePath"]

    # Progress checker: PoseProgressChecker counts ROTATION as progress —
    # headland corners pivot in place, and SimpleProgressChecker fired
    # "Failed to make progress" mid-pivot even with a 90 s allowance.
    progress_checker:
      plugin: "nav2_controller::PoseProgressChecker"
      required_movement_radius: 0.15       # m
      required_movement_angle: 0.5         # rad (~28°), gates corner pivots
      # OVERWRITTEN at launch from mowgli_robot.yaml.progress_timeout_sec —
      # change the timeout THERE, not here.
      movement_time_allowance: 30.0        # seconds

    # Transit goal checker: robot stopped within tolerance of goal pose
    stopped_goal_checker:
      plugin: "nav2_controller::StoppedGoalChecker"
      trans_stopped_velocity: 0.10         # m/s
      xy_goal_tolerance: 0.30              # m (overridden at launch from mowgli_robot.yaml.xy_goal_tolerance)
      # 0.10 rad (≈5.7°), was 0.5: the loose value let the docking server's
      # nav-to-staging phase report SUCCESS 20-25° off the dock axis, so the
      # robot entered the cradle corridor "en biais".
      yaw_goal_tolerance: 0.10             # rad (overridden at launch)

    # Coverage goal checker: PathProgressGoalChecker (mowgli_nav2_plugins).
    # Subscribes to FTC's republished <plugin_name>/global_plan and only
    # fires when the robot has monotonically tracked >= progress_threshold
    # of the path's poses AND is within xy/yaw tolerance of the goal pose.
    # Why not StoppedGoalChecker / SimpleGoalChecker: both fire on
    # proximity / velocity stoppage, which matches FTC's PRE_ROTATE pivots
    # mid-traversal — the action completes at <2 % coverage. Path-progress
    # gating is the only definition of "done" that survives a coverage
    # path whose start and end are both inside the headland ring.
    # (`fallback_timeout_s`, default 5 s, is declared but not set here: if the
    # plan topic never arrives the checker warns and falls back to plain
    # SimpleGoalChecker semantics rather than blocking until goal_timeout.)
    coverage_goal_checker:
      plugin: "mowgli_nav2_plugins/PathProgressGoalChecker"
      progress_threshold: 0.95
      # 0.50 matches FTC's max_goal_distance_error: FTC stops driving that far
      # short of the last pose, so a tighter gate is never met — the goal was
      # never accepted, the progress checker fired, the action aborted and the
      # whole area was re-mowed (2026-06-25).
      xy_goal_tolerance: 0.50              # m (overridden at launch from mowgli_robot.yaml.coverage_xy_tolerance, floored at FTC's 0.50)
      yaw_goal_tolerance: 3.14             # ≈π → final heading is meaningless for coverage
      plan_topic: "/controller_server/FollowCoveragePath/global_plan"

    # ─────────────────────────────────────────────────────────────
    # FollowPath = TRANSIT (dock approach, home, undock recovery).
    # RotationShimController wrapping RegulatedPurePursuit: the shim
    # does ONE in-place pivot to the path-start heading, then RPP
    # pursues a velocity-scaled carrot. Both accept 2-pose direct
    # lines, which FTC rejects — that is why transit is not FTC.
    # ─────────────────────────────────────────────────────────────
    FollowPath:
      plugin: "nav2_rotation_shim_controller::RotationShimController"
      primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
      # --- RotationShim ---
      angular_dist_threshold: 0.785        # rad (45°) — below this, hand straight to RPP
      forward_sampling_distance: 0.5
      rotate_to_heading_angular_vel: 0.75  # decisive — clears the firmware pivot deadband
      max_angular_accel: 2.5
      rotate_to_heading_once: true         # never re-engage mid-path (frozen plan → stale target)
      rotate_to_goal_heading: true         # terminal pivot; the docking server needs it
      closed_loop: false                   # sub-deadband feedback reads ~0 and stalls the pivot
      # --- RegulatedPurePursuit (primary) ---
      desired_linear_vel: 0.30             # m/s transit cap
      min_approach_linear_velocity: 0.16
      use_velocity_scaled_lookahead_dist: true
      lookahead_time: 2.0
      min_lookahead_dist: 0.45             # long enough to damp the pursuit S-weave
      max_lookahead_dist: 0.90
      use_regulated_linear_velocity_scaling: true
      regulated_linear_scaling_min_radius: 0.9
      regulated_linear_scaling_min_speed: 0.20
      use_rotate_to_heading: false         # mid-path re-rotate aims at a stale target
      allow_reversing: false
      # use_collision_detection: true in the LiDAR overlay, false without

    # ─────────────────────────────────────────────────────────────
    # FollowCoveragePath = MOWING. FTCController (Follow-the-Carrot,
    # decoupled lon/lat/ang PID). RESTORED 2026-06-19, reverting the
    # MPPI experiment: as a SAMPLING controller MPPI cut corners and
    # omega-looped at swath U-turns, and sharpening its corners made
    # it weave on the straights.
    # ─────────────────────────────────────────────────────────────
    FollowCoveragePath:
      plugin: "mowgli_nav2_plugins/FTCController"
      # BOTH speeds are overridden at launch: speed_fast = mowing_speed,
      # speed_slow = clamp(mowing_speed × turn_speed_ratio, min_speed_mps,
      # mowing_speed). Editing the literals does NOT change the running robot.
      speed_fast: 0.20                     # m/s
      speed_slow: 0.16                     # m/s at every bend / turn-around arc
      speed_angular: 45.0                  # deg/s
      acceleration: 0.2
      min_speed_mps: 0.15
      # Anti-wheelspin: ease the carrot to a crawl instead of flooring it
      # when actual speed stays below stall_speed_ratio × command.
      stall_speed_ratio: 0.35
      stall_grace_s: 0.6
      stall_crawl_speed: 0.08
      kp_lon: 1.0
      kp_lat: 0.8
      kd_lat: 0.5                          # 1.5 pumped a ~0.5 Hz cross-track limit cycle
      kp_ang: 1.5                          # PRE_ROTATE pivot gain — MUST stay high (deadband)
      kp_ang_following: 1.0                # straight-swath heading gain — kills the weave
      derivative_filter_tau: 0.2           # one-pole LPF on the PID derivative
      max_cmd_vel_speed: 0.30
      max_cmd_vel_ang: 0.8
      max_goal_distance_error: 0.50
      max_goal_angle_error: 30.0           # deg
      goal_timeout: 10.0
      max_follow_distance: 2.0
      forward_only: true                   # skirt obstacles forward, never reverse a segment
      check_obstacles: true                # false in the no-LiDAR overlay
      obstacle_lookahead: 30               # poses; overridden at launch from obstacle_detection_range_m
      obstacle_footprint: true
      use_footprint_clearance: false       # full-footprint sweep proved too conservative in the field
      obstacle_footprint_front_length_m: 0.30
      require_clear_exit: true             # cul-de-sac guard: never skirt into a pocket
      obstacle_body_half_width: 0.12
      obstacle_clearance_margin: 0.05      # overridden at launch from obstacle_clearance_margin
      ignore_obstacles_outside_zone: true  # keepout-masked cells are not obstacles (issue #517)
      enable_obstacle_deviation: true      # false in the no-LiDAR overlay
      max_lateral_deviation: 1.5           # overridden at launch from max_obstacle_avoidance_distance
      deviation_step: 0.05
      deviation_blend_rate: 0.5
      obstacle_wait_timeout_s: 2.5
      obstacle_clear_hold_s: 1.5
      # Bounded straight reverse-escape when wedged (SAFETY-CRITICAL: the rear
      # footprint is checked against true-lethal cells before every tick).
      obstacle_reverse_enabled: true
      obstacle_reverse_max_dist_m: 0.30
      obstacle_reverse_speed_mps: 0.10
```

> Many of the values above are **injected at launch** from `mowgli_robot.yaml` (speeds, goal tolerances, obstacle distances, progress timeout). Editing the yaml literal has no effect on the running robot — change the robot config instead.

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
      tolerance: 0.50                      # m, tolerance to goal
      downsample_costmap: false
      downsampling_factor: 2
      allow_unknown: false

      # Search budget. Raised from 100000 / 3.5 s for the enlarged 70×70 m
      # global costmap — a long cross-garden transit must not false-fail
      # on the iteration/time cap.
      max_iterations: 1000000
      max_on_approach_iterations: 1000
      max_planning_time: 8.0               # seconds

      # 3.0 (was 2.0, issue #393): weight of the costmap cell cost in A*'s
      # traversal term. Higher = the transit planner prefers routes with more
      # BERTH from inflated cells. This is the SAFE berth lever — it only
      # re-weights traversable cells, unlike raising global inflation_radius
      # (which made Smac report "Start occupied" and stall transit for 9 min).
      cost_travel_multiplier: 3.0
      use_final_approach_orientation: false

      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        w_data: 0.2
        tolerance: 1.0e-10
```

#### costmap_2d Configuration

Both costmaps use **`base_footprint`** as `robot_base_frame` (REP-105) — never `base_link`. The **layer list and the obstacle source live in the variant overlay**, not the base.

```yaml
# Global costmap (used by planner) — base file
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0                # Hz (1 Hz after the idle-CPU triage)
      publish_frequency: 1.0               # Hz
      global_frame: map
      robot_base_frame: base_footprint
      transform_tolerance: 0.5             # s — caps per-scan rotation smear at ≤22°
      always_send_full_costmap: false      # delta updates
      footprint: "[[0.50, 0.20], [0.50, -0.20], [-0.10, -0.20], [-0.10, 0.20]]"
      resolution: 0.08                     # m/cell (local stays 0.05)
      track_unknown_space: false           # unknown = free, so transit can cross unmapped ground
      rolling_window: true
      # 70×70 m (was 20×20): a 20 m window rejected area-to-area goals >10 m
      # away with "Goal Coordinates was outside bounds" and the mission
      # reported AREA_UNREACHABLE. Sizing rule: ≥ 2× the longest per-axis
      # area-to-area transit. Safe because keepout_filter makes everything
      # outside the authorised zones LETHAL.
      width: 70
      height: 70

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 10.0
        # 0.20 m (was 0.40, briefly 0.30). Larger radii put the footprint
        # corners inside the inscribed band near the boundary → Smac
        # "Start occupied" → multi-minute TRANSIT stalls.
        inflation_radius: 0.20

      keepout_filter:
        plugin: "nav2_costmap_2d::KeepoutFilter"
        enabled: true
        filter_info_topic: "/costmap_filter_info"

  # LiDAR overlay adds:    plugins: ["obstacle_layer", "keepout_filter", "inflation_layer"]
  #                        obstacle_layer source /scan_costmap (IMU-tilt-filtered),
  #                        min_obstacle_height 0.12, max 1.5, obstacle_max_range 3.0,
  #                        raytrace_max_range 8.0, observation_persistence 0.0
  # no-LiDAR overlay adds: plugins: ["static_layer", "keepout_filter", "inflation_layer"]
  #                        static_layer map_topic /no_lidar_static_map

# Local costmap (used by controller, smaller window around robot) — base file
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0               # Hz (> controller_frequency)
      publish_frequency: 2.0               # Hz (visualisation only)
      global_frame: odom
      robot_base_frame: base_footprint
      transform_tolerance: 0.5
      always_send_full_costmap: true
      footprint: "[[0.50, 0.20], [0.50, -0.20], [-0.10, -0.20], [-0.10, 0.20]]"
      resolution: 0.05
      rolling_window: true
      # 12×12 m: a 6 m window truncated the obstacle signal mid-strip.
      # NO keepout_filter here — marking the boundary lethal locally made the
      # controller report "collision ahead" on swaths near the edge.
      width: 12
      height: 12

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.5           # gentle, WIDE gradient → smooth deviation
        # 0.58 m == the clamp floor (chassis circumscribed radius ~0.572 m).
        # The earlier 1.0 m halo smeared a side obstacle's cost across the whole
        # front of a 0.5-1.0 m gap, so transit read "collision ahead" and Nav2
        # recoveries could not escape a physically passable pocket.
        # OVERWRITTEN at launch from mowgli_robot.yaml.obstacle_inflation_radius
        # (clamped [0.58, 1.50]).
        inflation_radius: 0.58

  # LiDAR overlay adds:    plugins: ["obstacle_layer", "inflation_layer"] on /scan_costmap
  # no-LiDAR overlay adds: plugins: ["static_layer", "inflation_layer"] on /no_lidar_static_map
```

#### collision_monitor Configuration

`collision_monitor` sits between the controllers and twist_mux: it reads `cmd_vel_nav` and republishes `cmd_vel_monitored`. Shared settings live in the base; the polygons and observation sources are variant-specific.

```yaml
collision_monitor:
  ros__parameters:
    base_frame_id: base_footprint
    odom_frame_id: odom
    cmd_vel_in_topic: cmd_vel_nav
    cmd_vel_out_topic: cmd_vel_monitored
    # 1.5 s (was 5.0): at 0.20 m/s a 5 s stale window let a dead LiDAR chain
    # mow BLIND for a full metre before the stop fired.
    source_timeout: 1.5
    base_shift_correction: true
    stop_pub_timeout: 2.0

  # LiDAR overlay:    polygons ["FootprintApproach", "PolygonStopNarrow", "PolygonSlow"]
  #                   — velocity-projected approach stops + a soft slowdown on
  #                   /scan_collision. A static stop box was tried first and
  #                   froze cmd_vel mid-skirt on obstacles already steered around.
  # no-LiDAR overlay: pass-through — every polygon and the scan source disabled.
```

The STM32 firmware remains the **sole** emergency-stop authority; `collision_monitor` is only the soft pre-contact layer.

> **`velocity_smoother` was removed on 2026-04-26.** Acceleration and velocity caps now live in the controllers themselves (`acceleration`, `max_cmd_vel_speed`, `max_cmd_vel_ang` for FTC; `desired_linear_vel`, `max_angular_accel` for RPP), and `collision_monitor` publishes straight to twist_mux on `cmd_vel_monitored`.

---

## 4. coverage_server (mowgli_coverage / Fields2Cover v3)

**Config:** `coverage_server` block inside `src/mowgli_bringup/config/nav2_params_base.yaml`.
**Action:** `plan_coverage` (type `mowgli_interfaces/action/PlanCoverage`).
**Backend:** Fields2Cover **3.0.0** at `/opt/fields2cover-300`. `mowgli_coverage`'s `CMakeLists.txt` pins explicit `INSTALL_RPATH`s on both `libmowgli_coverage_core.so` and the executable so the loader cannot pick up the retained v2 tree at `/opt/fields2cover-200`.

```yaml
coverage_server:
  ros__parameters:
    use_sim_time: false
    # PHYSICAL chassis width — injected at launch from mowgli_robot.yaml.chassis_width.
    # Semantic only; the geometry is driven by operation_width + the insets.
    robot_width: 0.40                    # m
    # Swath SPACING (F2C cov_width). INJECTED at launch as
    # tool_width − swath_overlap, so adjacent swaths slightly OVERLAP.
    operation_width: 0.16                # m
    # Headland band width — the planner mows ceil(headland_width / operation_width)
    # concentric perimeter rings (min 1) unless num_headland_passes forces a count.
    default_headland_width: 0.20         # m
    # THREE-WAY sentinel: <0 = NO perimeter rings, 0 = auto, >0 = exactly N rings.
    # Deliberately NOT clamped. Read once at on_configure (GUI change needs a restart).
    num_headland_passes: 0
    # Polygon pull-back applied before all planning so the chassis cannot cross
    # the boundary under tracking error. Injected at launch; read LIVE per plan.
    chassis_safety_inset: 0.0            # m
    min_swath_length: 0.15               # m — drop sliver swaths; read LIVE per plan
    # Also declared (defaults only, injected at launch, read LIVE per plan):
    #   ring_direction        0 = planner default, 1 = CW, 2 = CCW  (#335)
    #   min_turning_radius    0.15 m — hard floor on every arc in the continuous path
    #   connector_turn_radius 0.18 m — nominal turn-around radius; floored at
    #                         min_turning_radius. Larger values balloon the U-turn
    #                         into a teardrop that overshoots into the headland.
```

There are **no** mode-string params (`default_swath_type`, `default_route_type`, `default_path_type`, …) and **no** decomposition step — those belonged to the legacy `opennav_coverage` schema.

**Pipeline:** `f2c::hg::ConstHL.generateHeadlands` (inset) + `generateHeadlandSwaths(op_width, n_rings, dir_out2in=true)` (concentric perimeter rings, outermost first) → per mainland cell `f2c::sg::BruteForce.generateBestSwaths` (each disjoint clip becomes its own swath, so concave boundaries and interior holes need no decomposition) → `f2c::rp::BoustrophedonOrder.genSortedSwaths` (serpentine) → `buildContinuousSubPaths` joins rings + swaths with **custom forward turn-around arcs** of nominal radius `connector_turn_radius`. F2C's own turn planners (Dubins / CC-Dubins / Reeds-Shepp) are **not** used — every variant was field-tested and failed.

The result carries both an ordered list of discrete `segments` + `segment_types` (rings then swaths — kept for the GUI and resume bookkeeping) and one or more **hole-free `drivable_subpaths`**, which is what execution actually drives: `FollowStrip` dispatches each sub-path as ONE continuous `FollowCoveragePath` goal and bridges consecutive sub-paths with a blade-off `NavigateToPose` transit around the obstacle. `full_path` is their concatenation, for visualisation only.

The BT side (`PlanCoverageArea`) feeds it the area outer ring + obstacle holes fetched from `/map_server_node/get_mowing_area`. The upstream `opennav_coverage` submodule is kept for its `_msgs` action definitions only — every server subpackage is `COLCON_IGNORE`'d.

---

## 5. fusion_graph

`fusion_graph_node` (GTSAM iSAM2) is the **sole and default** localizer: `navigation.launch.py` launches it unconditionally and it publishes **both** `map → odom` **and** `odom → base_footprint`. There is **no** dedicated YAML config — knobs are declared as ROS2 parameters on the node, and the high-level switches (`use_scan_matching`, `use_loop_closure`, `use_magnetometer`, `fusion_graph_node_period_s`) live in `mowgli_robot.yaml`, exposed on the Settings page under *Localization*. `use_scan_matching` / `use_loop_closure` are ANDed with `use_lidar` before they reach the node.

Full parameter table in [§7](#7-fusion_graph); the design is in [Architecture → Factor-Graph Localizer](Architecture#optional-factor-graph-localizer-fusion_graph).

**Monitoring localization health.** The diagnostics system watches `/odometry/filtered_map` and `/fusion_graph/diagnostics`:

- **Rate:** expect the configured node cadence (25 Hz at the default `fusion_graph_node_period_s: 0.04`); warn well below that.
- **Position covariance:** `cov_xx` / `cov_yy` should collapse toward the GPS fix precision within a few fixes under RTK-Fixed.
- **Yaw:** `cov_yawyaw` should drop once the robot moves forward and `/imu/cog_heading` starts publishing.

Access diagnostics at `http://<mower-ip>:4006/#/diagnostics` → Localization / Fusion Graph sections.

---

## 6. twist_mux.yaml

**File:** `src/mowgli_bringup/config/twist_mux.yaml`

**Purpose:** Priority-based multiplexing of velocity commands from multiple sources.

**Full Configuration:**

```yaml
twist_mux:
  ros__parameters:
    # Kilted Kaiju: all Nav2 nodes use TwistStamped
    use_stamped: true

    # Input topics (velocity sources)
    # Topics evaluated in ascending order of priority
    # Higher priority sources suppress lower priority when active
    topics:
      # Lowest priority: autonomous navigation, POST collision_monitor
      navigation:
        topic: /cmd_vel_monitored
        # 0.6 s (was 2.0): if collision_monitor dies, twist_mux kept
        # republishing the last nav command for up to 2 s, refreshing the
        # firmware's 200 ms cmd_vel watchdog and letting the robot coast
        # ~0.4 m after the safety filter was gone.
        timeout: 0.6
        priority: 10                 # Lowest priority

      # Docking / undocking manoeuvres (opennav_docking)
      docking:
        topic: /cmd_vel_docking
        timeout: 0.5
        priority: 15                 # Between navigation and teleop

      # Manual teleoperation
      teleop:
        topic: /cmd_vel_teleop
        timeout: 0.5
        priority: 20                 # Override navigation

      # Dedicated drive-tuning lane above the shared teleop/ws relay
      tuning:
        topic: /cmd_vel_tuning
        timeout: 0.5
        priority: 30

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
Time 0: Navigation publishes cmd_vel_monitored (0.1 m/s, via collision_monitor)
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

**Purpose:** the **sole and default** localizer, built on **GTSAM iSAM2**. It publishes `/odometry/filtered_map` and owns **both** `map → odom` **and** `odom → base_footprint` — the removed `ekf_map_node` / `ekf_odom_node` pair used to own one each. The map-frame estimate is the result of a Pose2 factor graph that carries LiDAR scan-matching and loop-closure factors through extended RTK-Float windows. See [Architecture → Factor-Graph Localizer](Architecture#optional-factor-graph-localizer-fusion_graph) for the steady-state design.

### Switches

`navigation.launch.py` launches `fusion_graph_node` **unconditionally** — there is no `use_fusion_graph` argument. What is configurable is the feature set:

```yaml
# mowgli_robot.yaml (also exposed in the Settings → Localization section)
mowgli:
  ros__parameters:
    use_scan_matching: true       # ANDed with lidar_enabled before it reaches the node
    use_loop_closure: true        # ANDed with lidar_enabled AND a persisted graph on disk
    use_magnetometer: false       # off on stock chassis (motor field bias)
    fusion_graph_node_period_s: 0.04   # 25 Hz on hardware; 0.1 (10 Hz) is the Pi-friendly value
```

```bash
# Per-launch override (one-shot)
ros2 launch mowgli_bringup navigation.launch.py use_scan_matching:=false
```

### Key parameters

| Parameter | Default | Notes |
|---|---|---|
| `node_period_s` | 0.04 | Graph node creation cadence. The node's own default is 0.1 (10 Hz), but `navigation.launch.py` injects `mowgli_robot.yaml.fusion_graph_node_period_s` (hardware fallback 0.04 = 25 Hz). |
| `stationary_node_period_s` | 5.0 | Throttled node period when motion is below the stationary threshold — bounds graph growth on the dock. |
| `wheel_sigma_x_per_sqrt_m / wheel_sigma_y_per_sqrt_m` | 0.05 / 0.005 | Body-frame between-factor **translational** noise, in m/√m. The sigma applied to a node is `k · √(step_m + wheel_creep_speed_mps · dt)` — variance grows with the distance the step covered, so the accumulated uncertainty tracks distance travelled and is invariant to `node_period_s` (issue #491). `sigma_y` ≪ `sigma_x` still enforces non-holonomic motion: both scale by the same `√d`. At 1 m of travel per node these reproduce the old fixed per-node 0.05 / 0.005 m. |
| `wheel_creep_speed_mps` | 0.04 | Floor on the noise distance above, as a creep *speed*: motion the encoders may have missed (towed, lifted, both wheels skating). Expressed as a distance so the floor stays cadence-invariant. |
| `wheel_sigma_theta` | 0.01 | Yaw between-factor noise, still a **per-node** sigma — used only when no gyro sample arrived for the tick. |
| `gyro_sigma_theta` | 0.005 | Yaw between-factor noise from `/imu/data`. |
| `gps_sigma_floor` | 0.003 | Lower bound for the GPS XY noise (3 mm) — prevents over-trusting RTK-Fixed reports with under-estimated covariance. |
| `cov_update_every_n` | 10 | Skip-rate for the marginal covariance recompute (the diagonals on `/odometry/filtered_map`). |
| `isam2_relinearize_skip` | 5 | iSAM2 relinearization throttle. |
| `isam2_rebase_every_nodes` | 2000 | Periodic iSAM2 rebase to bound per-tick update cost. |
| `scan_retention_nodes` | 18000 | Drop body-frame scans older than this many nodes (~30 minutes at 10 Hz). |
| `lc_max_dist_m` / `lc_min_age_s` / `lc_max_candidates` / `lc_max_rmse` | 5.0 / 30.0 / 3 / 0.20 | Loop-closure search/accept gates. |
| `lc_skip_when_rtk_fixed` / `lc_min_travel_m` / `lc_min_interval_s` / `lc_gps_sigma_ratio` | true / 1.0 / 2.0 / 1.0 | Rate + travel gate on loop closures (issue #513). `lc_skip_when_rtk_fixed` is the bound that stops the stationary-dwell factor leak that OOM-killed the node — leave it on unless a site mows under permanent RTK-Float. |
| `icp_max_iter` / `icp_max_corresp_dist` / `icp_source_subsample` | 10 / 0.5 / 40 | Per-tick scan matcher; ten iterations converge within 1 mm of the 15-iteration solution on outdoor LiDAR shapes, and 40 source points keep the rmse within a few mm of the 60-point result at half the nearest-neighbour cost. |
| `scan_min_inliers` / `kf_min_inliers` | 30 / 16 | Inlier floors for the scan-to-scan between-factor and for keyframe (cross-viewpoint) matching. The keyframe floor is looser because the overlap is partial. |
| `autoload_graph` | true | Resume from `<graph_save_prefix>.{graph,scans,meta}` on startup. |
| `auto_save_enabled` | true | Auto-checkpoint on RECORDING→IDLE, dock arrival, and every `periodic_save_period_s` during AUTONOMOUS state. |
| `graph_save_prefix` | `/ros2_ws/maps/fusion_graph` | Base path for the three persistence files. |
| `primary_mode` | true | Broadcast the TFs. `navigation.launch.py` always passes `true`; the `false` (observer) path exists for the standalone test harness — it dates from when a second localizer could own `map → odom`, which is no longer the case. |

### Topics, services

- **`/fusion_graph/diagnostics`** (`diagnostic_msgs/DiagnosticArray`, 1 Hz) — exposes `total_nodes`, `scans_attached`, `loop_closures`, `scans_received`, `scan_matches_ok`, `scan_matches_fail`, `cov_xx`, `cov_yy`, `cov_yawyaw`. Surfaced in the GUI's *Diagnostics → Fusion Graph (iSAM2)* panel.
- **`/fusion_graph/markers`** (`visualization_msgs/MarkerArray`, 1 Hz, transient_local) — node positions, trajectory, loop-closure edges. Visible in Foxglove with no extra setup.
- **`/imu/fg_yaw`** (`sensor_msgs/Imu`) — yaw-only output of the graph, published for downstream consumers and for debugging against `/imu/mag_yaw` / `/imu/cog_heading`.
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
- **CPU budget**: scan-matching costs ~5 ms/tick at 10 Hz on a Pi 4. If you see the maintenance timer overrunning, raise `fusion_graph_node_period_s` to 0.1 (10 Hz) or `isam2_relinearize_skip` to 10, or lower `icp_source_subsample`, before disabling `use_scan_matching` outright.
- **Graph too large after weeks**: tune `isam2_rebase_every_nodes` down to 1500 — the rebase preserves the optimized values but drops accumulated between-factors.
- **LiDAR is unreliable in winter (snow on rotor, low visibility)**: leave `use_scan_matching:=true`, just disable `use_loop_closure` to avoid a stale match getting promoted to a loop-closure factor.

---

## Parameter Tuning Workflow

### Step 1: Identify Performance Issue

| Issue | Likely Culprit | Action |
|-------|----------------|--------|
| Pose drifts through a long RTK-Float window | Graph leaning on the wheel factors | Enable `use_scan_matching` / `use_loop_closure` (needs LiDAR); check `cov_xx`/`cov_yy` on `/fusion_graph/diagnostics` |
| Pose snaps when a fix arrives | GPS trusted too much relative to the wheels | Raise `gps_sigma_floor`, or lower `wheel_sigma_x_per_sqrt_m` |
| Slow, metre-scale S-weave on **transit** | Pure-pursuit limit cycle: lookahead too short | Raise `lookahead_time` / `min_lookahead_dist` on `FollowPath` |
| Fast 2–4 Hz buzz on transit | Firmware yaw-rate loop lagging the wheel command | Tune `yaw_kp`/`yaw_ki` in **firmware**, not here |
| Weave on a straight **swath** | FTC lateral derivative pumping a limit cycle | Lower `kd_lat`, then `kp_ang_following` (see the 2026-06-25 field notes in `nav2_params_base.yaml`) |
| Robot passes obstacles too closely | Skirt margin too small | Raise `obstacle_clearance_margin` (via `mowgli_robot.yaml`); costmap `inflation_radius` cannot do it |
| Planner is very slow | Global grid too fine or window too large | Coarsen `global_costmap.resolution` (0.08 → 0.10) rather than shrinking the 70 m window |
| Thin un-mowed strips between swaths | Swath spacing wider than the stamp radius | Check `tool_width` and `swath_overlap` in `mowgli_robot.yaml` — never hardcode `operation_width` |

### Step 2: Modify Parameters

Most operator-facing knobs live in the **installed** robot config, not in the Nav2 yaml — see [Sparse robot config model](#sparse-robot-config-model):

```bash
nano /ros2_ws/config/mowgli_robot.yaml     # per-robot overrides (sparse)
```

Nav2/controller internals that are *not* injected at launch live in the base file:

```bash
nano src/mowgli_bringup/config/nav2_params_base.yaml
```

### Step 3: Test and Monitor

```bash
# Launch with new parameters
ros2 launch mowgli_bringup full_system.launch.py

# Watch live in Foxglove (foxglove_bridge on ws://<mower-ip>:8765)

# Check diagnostics
ros2 topic echo /fusion_graph/diagnostics
ros2 topic echo /odometry/filtered_map
```

### Step 4: Iterate

Rerun with adjusted parameters, observe results, adjust again.

---

## Reference: Default Parameters Quick Lookup

| Parameter | Default | Unit | Where |
|-----------|---------|------|-------|
| `serial_port` | `/dev/mowgli` | – | `hardware_bridge.yaml` |
| `baud_rate` | 115200 | baud | `hardware_bridge.yaml` |
| `publish_rate` | 100.0 | Hz | `hardware_bridge.yaml` (firmware caps the real rate at ~47 Hz) |
| `fusion_graph_node_period_s` | 0.04 | s | `mowgli_robot.yaml` (25 Hz; 0.1 is the Pi-friendly value) |
| `controller_frequency` | 10.0 | Hz | `nav2_params_base.yaml` |
| `desired_linear_vel` (transit, RPP) | 0.30 | m/s | `nav2_params_base.yaml` → `FollowPath` |
| `min_lookahead_dist` / `max_lookahead_dist` | 0.45 / 0.90 | m | `nav2_params_base.yaml` → `FollowPath` |
| `speed_fast` (mowing, FTC) | 0.20 | m/s | injected from `mowgli_robot.yaml.mowing_speed` |
| `tool_width` | 0.18 | m | `mowgli_robot.yaml` (drives `operation_width = tool_width − swath_overlap`) |
| `swath_overlap` | 0.02 | m | `mowgli_robot.yaml` |
| `global_costmap.resolution` / window | 0.08 / 70×70 | m, m | `nav2_params_base.yaml` |
| `local_costmap.inflation_radius` | 0.58 | m | injected from `mowgli_robot.yaml.obstacle_inflation_radius` |

---

**For detailed system architecture, see [Architecture](Architecture).**

**For firmware integration, see [Firmware](Firmware).**

**Working on the code?** The Claude-facing indexes are the fastest way in: per-area codemaps in [`docs/claude/codemaps/`](https://github.com/mowglinext/mowglinext/tree/main/docs/claude/codemaps), plus [`docs/claude/parameters.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/parameters.md) (every config key, its default and its consumers), [`docs/claude/ros-interfaces.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/ros-interfaces.md) and [`docs/claude/doc-index.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/doc-index.md) (which document is authoritative vs historical).
