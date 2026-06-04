# Mowgli ROS2 Architecture

Comprehensive technical documentation of the Mowgli ROS2 system design, including package organization, data flow, communication protocols, and integration points.

Built on **ROS2 Kilted** with **Gazebo Harmonic** simulation, this architecture spans 12 focused packages providing complete autonomous lawn mower functionality. [CLAUDE.md](https://github.com/cedbossneo/mowglinext/blob/main/CLAUDE.md) is the authoritative short-form reference and the place to check first if any page on the wiki looks out of date.

## Localization at a glance

The map-frame localizer has **two interchangeable backends**, picked at launch time via the `use_fusion_graph` arg on `navigation.launch.py` (defaults to `false`). The local odometry path and the area / coverage layers are unaffected.

| Layer | Owner | Notes |
|---|---|---|
| `map → odom` (default, `use_fusion_graph:=false`) | `ekf_map_node` @ 30 Hz | robot_localization global EKF; fuses wheels + gyro + `/gps/pose_cov` + GPS-COG absolute yaw + (optional) `/imu/mag_yaw` under `two_d_mode`. Datum from `mowgli_robot.yaml`. No SLAM. |
| `map → odom` (opt-in, `use_fusion_graph:=true`) | `fusion_graph_node` @ 10 Hz | GTSAM iSAM2 factor-graph localizer. Same inputs as the EKF, plus a custom `GnssLeverArmFactor` (analytic Jacobian) and optional LiDAR scan-matching / loop-closure factors. Publishes the same `/odometry/filtered_map` + `map → odom` TF; adds `/fusion_graph/diagnostics` and `~/save_graph` / `~/clear_graph` services. |
| `odom → base_footprint` | `ekf_odom_node` @ 50 Hz | robot_localization local EKF; fuses wheels + gyro_z. Continuous, never jumps. **Same in both backends.** |
| LiDAR (opt-in) | `fusion_graph_node` | When `use_fusion_graph:=true` and `use_scan_matching` / `use_loop_closure` are on, `/scan` enters the same factor graph as scan-matching between-factors and loop-closure factors. No parallel TF tree, no separate twist channel. |
| Map / areas | `map_server_node` | Polygon-based area DB + `mow_progress` GridMap layer, persisted to disk. No SLAM back-end. |

> The EKF and the factor-graph are **mutually exclusive** — only one publishes `map → odom` at a time. See [Optional: Factor-Graph Localizer](#optional-factor-graph-localizer-fusion_graph) below for when to enable each.

## System Overview

Mowgli ROS2 is organized as a **12-package ecosystem** with clear separation of concerns and layered dependencies:

```
┌──────────────────────────────────────────────────────────────────────────┐
│                        Application / Remote Control                      │
│                    (GUI, teleoperation, mission planning)                │
└──────────────────────────────────────────────────────────────────────────┘
                                     │
┌──────────────────────────────────────────────────────────────────────────┐
│                   High-Level Control & Decision Layer                    │
│  ┌──────────────────┐  ┌──────────────────┐  ┌──────────────────────┐   │
│  │  mowgli_behavior │  │ mowgli_nav2_     │  │  mowgli_localization │   │
│  │  (Behavior Tree) │  │  plugins         │  │  (EKF + monitoring)  │   │
│  │                  │  │  (FTC + RPP      │  │                      │   │
│  │  10 Hz reactive  │  │   Controllers)   │  │  Multiple nodes:     │   │
│  │  control         │  │                  │  │  - Wheel odometry    │   │
│  │                  │  │  Nav2 local plan │  │  - GPS converter     │   │
│  │                  │  │  10 Hz           │  │  - Health monitor    │   │
│  │                  │  │                  │  │  - Diagnostics       │   │
│  └──────────────────┘  └──────────────────┘  └──────────────────────┘   │
│                                                                           │
│  ┌──────────────────────────────────┐                                    │
│  │  mowgli_monitoring               │                                    │
│  │  (Diagnostics aggregator,        │                                    │
│  │   MQTT bridge)                   │                                    │
│  └──────────────────────────────────┘                                    │
└──────────────────────────────────────────────────────────────────────────┘
                                     │
┌──────────────────────────────────────────────────────────────────────────┐
│                         Config & Launch Layer                            │
│                      (mowgli_bringup: URDF, params)                      │
│                      (mowgli_map: map server, storage)                   │
└──────────────────────────────────────────────────────────────────────────┘
                                     │
┌──────────────────────────────────────────────────────────────────────────┐
│                    Hardware Abstraction & Protocol                       │
│            (mowgli_hardware: COBS serial bridge to STM32)                │
│                                                                           │
│  Publishers:                         Subscribers:                        │
│    - ~/status (Status msg)            - ~/cmd_vel (Twist)                │
│    - ~/emergency (Emergency msg)                                         │
│    - ~/power (Power msg)            Services:                            │
│    - ~/imu/data_raw (Imu msg)         - ~/mower_control                  │
│                                        - ~/emergency_stop                │
└──────────────────────────────────────────────────────────────────────────┘
                                     │
                      [USB Serial: COBS-framed packets]
                                     │
┌──────────────────────────────────────────────────────────────────────────┐
│                        STM32 Firmware (Mowgli Board)                     │
│  Motor control, sensor acquisition, real-time loop, watchdog            │
└──────────────────────────────────────────────────────────────────────────┘
```

## Package Overview

| Package | Purpose | Dependencies |
|---------|---------|--------------|
| **mowgli_interfaces** | Message, service, and action type definitions | ROS2 core |
| **mowgli_hardware** | Serial bridge to STM32 firmware (COBS + CRC-16 protocol) | mowgli_interfaces |
| **mowgli_localization** | Helper nodes around robot_localization's dual EKF (wheel odometry, NavSatFix→pose conversion, COG-to-IMU absolute yaw, magnetometer yaw, dock-yaw auto-capture, localization monitor) | mowgli_interfaces, robot_localization |
| **mowgli_nav2_plugins** | Nav2 controller plugins (FTC, RPP + RotationShimController, goal checkers) | nav2_core, mowgli_interfaces |
| **mowgli_map** | Map server with cell-based strip coverage planner (`~/get_next_strip` service), area storage/persistence, mow progress tracking (`mow_progress` grid layer), and obstacle_tracker_node | mowgli_interfaces, nav_msgs, nav2_map_server |
| **mowgli_behavior** | Reactive behavior tree control (BehaviorTree.CPP v4) | mowgli_interfaces, nav2_msgs |
| **mowgli_monitoring** | Diagnostics aggregation and MQTT bridge for external monitoring | diagnostic_msgs |
| **mowgli_simulation** | Gazebo Harmonic worlds, robot models, and ros_gz_bridge configuration | mowgli_bringup, ros_gz_sim, ros_gz_bridge |
| **mowgli_map** | Map server, storage, persistence for offline maps, and obstacle_tracker_node (persistent LiDAR obstacle detection) | nav_msgs, nav2_map_server, mowgli_interfaces |
| **mowgli_description** | URDF/xacro robot model and meshes | xacro, robot_state_publisher |
| **mowgli_bringup** | Configuration, launch orchestration, and integration layer | All packages above |

## Package Dependency Graph

```
mowgli_interfaces (base layer)
    │
    ├──→ mowgli_hardware
    │       └──→ mowgli_bringup
    │
    ├──→ mowgli_localization
    │       └──→ mowgli_bringup
    │
    ├──→ mowgli_nav2_plugins
    │       └──→ mowgli_bringup
    │
    ├──→ mowgli_behavior
    │       └──→ mowgli_bringup
    │
    ├──→ mowgli_monitoring
    │       └──→ mowgli_bringup
    │
    └──→ mowgli_map
            └──→ mowgli_bringup

mowgli_description (robot model)
    └──→ mowgli_bringup

mowgli_simulation (standalone testing)
    ├──→ mowgli_bringup
    ├──→ ros_gz_sim, ros_gz_bridge
    └──→ Gazebo Harmonic

mowgli_bringup (integration layer)
    ├──→ launch files
    ├──→ URDF/xacro
    └──→ configuration files

Application layer
    └──→ mowgli_bringup (and sub-packages)
```

## Detailed Package Architecture

### 1. mowgli_interfaces

**Purpose:** Define all ROS2 message, service, and action types.

**Location:** `src/mowgli_interfaces/`

**Key Definitions:**

#### Messages

- **Status.msg** – Mower operational state
  ```
  builtin_interfaces/Time stamp
  uint8 mower_status              # MOWER_STATUS_OK, MOWER_STATUS_INITIALIZING
  bool raspberry_pi_power         # Pi on/off switch state
  bool is_charging                # Battery charging active
  bool rain_detected              # Rain sensor
  bool sound_module_available     # Sound module present
  bool sound_module_busy          # Sound playing
  bool ui_board_available         # UI board detected
  bool mow_enabled                # Cutting blade enabled
  bool esc_power                  # Motor power enabled
  ```

- **Emergency.msg** – Safety stop status
  ```
  builtin_interfaces/Time stamp
  bool latched_emergency          # Emergency is latched (requires explicit release)
  bool active_emergency           # Any emergency condition active
  string reason                   # Human-readable description
  ```

- **Power.msg** – Battery and charging information
  ```
  builtin_interfaces/Time stamp
  float32 v_charge                # Charging port voltage
  float32 v_battery               # Battery voltage
  float32 charge_current          # Charging current (mA)
  bool charger_enabled            # Charger plugged and active
  string charger_status           # "charging", "idle", "error"
  ```

- **WheelTick.msg** – Encoder pulse counts with validity bitmasks
  ```
  builtin_interfaces/Time stamp
  float32 wheel_tick_factor       # Ticks-to-distance conversion factor
  uint8 valid_wheels              # Bitmask: WHEEL_VALID_FL=1, FR=2, RL=4, RR=8
  bool wheel_direction_fl         # Front-left direction
  bool wheel_direction_fr         # Front-right direction
  bool wheel_direction_rl         # Rear-left direction
  bool wheel_direction_rr         # Rear-right direction
  uint32 wheel_ticks_fl           # Front-left tick count
  uint32 wheel_ticks_fr           # Front-right tick count
  uint32 wheel_ticks_rl           # Rear-left tick count
  uint32 wheel_ticks_rr           # Rear-right tick count
  ```

- **AbsolutePose.msg** – Robot position with GPS quality flags (FLAG_GPS_RTK=1, FLAG_GPS_RTK_FIXED=2, FLAG_GPS_RTK_FLOAT=4, FLAG_GPS_DEAD_RECKONING=8)
- **HighLevelStatus.msg** – Behavior tree state with coverage progress
  ```
  uint8 HIGH_LEVEL_STATE_NULL=0
  uint8 HIGH_LEVEL_STATE_IDLE=1
  uint8 HIGH_LEVEL_STATE_AUTONOMOUS=2
  uint8 HIGH_LEVEL_STATE_RECORDING=3
  uint8 HIGH_LEVEL_STATE_MANUAL_MOWING=4

  uint8 state
  string state_name
  string sub_state_name
  int16 current_area / current_path / current_path_index
  int16 total_swaths / completed_swaths / skipped_swaths
  float32 gps_quality_percent / battery_percent
  bool is_charging / emergency
  ```
- **ESCStatus.msg** – Motor ESC telemetry
- **ImuRaw.msg** – Raw IMU data from STM32 firmware
- **MapArea.msg** – Mowing area polygon definition for map_server_node
- **CoveragePath.msg** – Coverage path with metadata
- **ObstacleArray.msg** – Collection of tracked obstacles from obstacle_tracker_node
- **TrackedObstacle.msg** – Individual persistent obstacle with position, age, and observation count

#### Services

- **MowerControl.srv** – Blade and drive control
  ```
  Request:
    bool mow_enabled              # Enable/disable blade motor
    uint8 mow_direction           # CW, CCW, or off
  Response:
    bool success
  ```

- **EmergencyStop.srv** – Safety control
  ```
  Request:
    bool emergency                # true=assert, false=release
  Response:
    bool success
  ```

- **HighLevelControl.srv** – Behavior tree mode commands from GUI
  ```
  Constants:
    COMMAND_START=1             # Begin autonomous mowing
    COMMAND_HOME=2              # Return to dock
    COMMAND_RECORD_AREA=3       # Start area recording (drive boundary)
    COMMAND_S2=4                # Mow next area
    COMMAND_RECORD_FINISH=5     # Finish recording, save polygon
    COMMAND_RECORD_CANCEL=6     # Cancel recording, discard trajectory
    COMMAND_MANUAL_MOW=7        # Enter manual mowing mode (teleop + blade)
    COMMAND_RESET_EMERGENCY=254 # Reset latched emergency
    COMMAND_DELETE_MAPS=255     # Delete all maps

  Request:
    uint8 command
  Response:
    bool success
  ```

- **AddMowingArea.srv** – Save a mowing area polygon to map_server_node
  ```
  Request:
    MapArea area                  # Polygon defining the area
    bool is_navigation_area       # true=navigation, false=mowing
  Response:
    bool success
  ```

- **AreaRecording.srv** – Area recording lifecycle control
  ```
  Constants:
    COMMAND_START=1 / COMMAND_FINISH=2 / COMMAND_CANCEL=3

  Request:
    uint8 command
    string area_name
    bool is_exclusion_zone
  Response:
    bool success
    string message
    geometry_msgs/Polygon polygon
  ```

#### Actions

- **NavigateToPose.action** – Nav2 navigation goal (standard nav2_msgs)

**Design Notes:**
- All timestamps use `builtin_interfaces/Time` (ROS2 idiom, replacing `rosgraph_msgs/Time` from ROS1)
- Floating-point values are `float32` (hardware native) except for precise GPS data (`float64`)
- Bitmasks used for compactness in Status and Emergency (reduces firmware packet size)

---

### 2. mowgli_hardware

**Purpose:** Serial bridge between STM32 firmware and ROS2 via COBS protocol.

**Location:** `src/mowgli_hardware/`

**Architecture:**

```
SerialPort (open/read/write raw bytes)
    ↓
PacketHandler (COBS framing, CRC16 validation)
    ↓
HardwareBridgeNode (ROS2 topics/services interface)
    ↓
ROS2 ecosystem
```

#### Key Components

**SerialPort (serial_port.cpp/.hpp)**
- Low-level serial port abstraction
- Non-blocking read/write
- Auto-reconnect on firmware flash / board reboot / USB re-enumeration: a
  dead-link RX watchdog (`serial_rx_timeout_s`, default 2 s) closes a port that
  is nominally open but no longer delivering bytes (or returns a hard read
  error), so the next read tick reopens it and re-resolves `/dev/mowgli` to the
  live `ttyACM`. The STM32 streams status/odom/IMU continuously, so a
  multi-second RX gap unambiguously means the endpoint went away. Self-heals
  without a manual container restart.
- Configurable baud rate (115200 default)

**PacketHandler (packet_handler.cpp/.hpp)**
- COBS (Consistent Overhead Byte Stuffing) encoding/decoding
- CRC-16 CCITT checksum calculation and verification
- Packet type dispatch via enum `PacketId`
- Thread-safe callback for complete packets

**HardwareBridgeNode (hardware_bridge_node.cpp)**
- ROS2 node instantiation (singleton pattern)
- Parameter declaration (serial_port, baud_rate, heartbeat_rate, etc.)
- Publishers, subscribers, services
- Timer-based read loop (100 Hz default)
- Heartbeat transmission (4 Hz default)
- High-level state updates (2 Hz default, GPS quality + mode)
- Publishes `WheelTick` on `~/wheel_ticks` (→ `/wheel_ticks`), once per firmware
  packet (~47 Hz), for the GUI "Per-Wheel Encoders" panel. The 2-wheel
  diff-drive maps left→RL, right→RR (`valid_wheels = RL|RR`); Front L/R remain
  invalid so the panel correctly shows "no encoder reading" for them.

#### Wire Protocol: COBS + CRC-16

**Packet Structure:**

```
[COBS_FLAG] [COBS_ENCODED_PAYLOAD] [COBS_FLAG]
   0x00                                 0x00

PAYLOAD structure (binary, native endianness):
  [packet_type: uint8] [payload_data] [crc16: uint16_le]
```

**Example: LlCmdVel (motor command)**
```c
struct LlCmdVel {
  uint8_t type;           // PACKET_ID_LL_CMD_VEL (0x02)
  float linear_x;         // m/s linear velocity
  float angular_z;        // rad/s angular velocity
  uint16_t crc16;         // Calculated by hardware_bridge
};
// 1 + 4 + 4 + 2 = 11 bytes unencoded
// After COBS: 13 bytes (overhead for 0x00 bytes)
```

**COBS Encoding:**
- Byte stuffing scheme: encodes data so no 0x00 bytes appear in the payload
- Overhead: worst-case +1 byte per 254 data bytes
- Delimiter: 0x00 frame flag marks packet boundaries (both start and end)
- Enables robust framing even without external length fields

**CRC-16 (CCITT polynomial 0x1021):**
- Calculated over [packet_type][payload_data] only (not the CRC field itself)
- Polynomial: 0xA001 (reversed CCITT)
- Detects single and double bit errors, all error patterns < 16 bits

**Packet Types (from ll_datatypes.hpp):**

| Type ID | Name | Direction | Purpose |
|---------|------|-----------|---------|
| 0x00 | LL_HEARTBEAT | Pi → STM32 | Keep-alive, emergency control, release |
| 0x01 | LL_HIGH_LEVEL_STATE | Pi → STM32 | Mode, GPS quality, localization health |
| 0x02 | LL_CMD_VEL | Pi → STM32 | Motor velocity commands |
| 0x10 | LL_STATUS | STM32 → Pi | Mower state, charging, rain, sensors |
| 0x11 | LL_IMU | STM32 → Pi | Accelerometer + gyroscope data |
| 0x12 | LL_UI_EVENT | STM32 → Pi | Button press, duration |

#### Data Flow Diagrams

**Incoming (STM32 → Pi → ROS2):**
```
LlStatus (firmware)
    ↓ [COBS + CRC]
SerialPort::read()
    ↓
PacketHandler::feed() → on_packet_received()
    ↓
handle_status() → Status msg + Emergency msg + Power msg → pub_status_, pub_emergency_, pub_power_
    ↓
ROS2 network: /status, /emergency, /power
```

**Outgoing (ROS2 → Pi → STM32):**
```
ROS2: /cmd_vel (Twist msg)
    ↓
on_cmd_vel() callback
    ↓
Create LlCmdVel packet
    ↓
send_raw_packet() → PacketHandler::encode_packet() → [COBS + CRC]
    ↓
SerialPort::write()
    ↓
STM32 firmware
```

**Heartbeat (periodic, Pi → STM32):**
```
Timer callback (4 Hz)
    ↓
send_heartbeat()
    ↓
Create LlHeartbeat with emergency_active, emergency_release_pending flags
    ↓
send_raw_packet() → [COBS + CRC] → STM32
    ↓
STM32 watchdog reset
```

#### Configuration

**File:** `src/mowgli_bringup/config/hardware_bridge.yaml`

```yaml
hardware_bridge:
  ros__parameters:
    serial_port: "/dev/mowgli"     # Device path (USB serial)
    baud_rate: 115200               # Must match firmware
    heartbeat_rate: 4.0             # Hz – watchdog feed
    publish_rate: 100.0             # Hz – sensor polling
    high_level_rate: 2.0            # Hz – mode/GPS updates
```

**Topics Published (rates):**
- `~/status` (Status msg) – 100 Hz max (firmware sensor rate)
- `~/emergency` (Emergency msg) – 100 Hz max (with Status)
- `~/power` (Power msg) – 100 Hz max (with Status)
- `~/imu/data_raw` (sensor_msgs/Imu) – 100 Hz max (firmware IMU rate)

**Topics Subscribed:**
- `~/cmd_vel` (geometry_msgs/Twist) – On-demand callback (no rate limit)

**Services:**
- `~/mower_control` – Synchronous, blocks until acknowledged
- `~/emergency_stop` – Synchronous, blocks until acknowledged

---

### 3. mowgli_localization

**Purpose:** Multi-source localization pipeline (odometry, GPS fusion, health monitoring).

**Location:** `src/mowgli_localization/`

**Architecture:**

```
Inputs:
  - /status (WheelTick in Status msg)
  - /imu/data (sensor_msgs/Imu) → smoothed IMU
  - /gps/rtk_fix (sensor_msgs/NavSatFix, RTK status)

↓

Source nodes:

1) wheel_odometry_node
   - Integrates RL/RR encoder ticks
   - Publishes /wheel_odom (Odometry)
   - 50 Hz

2) navsat_to_absolute_pose_node
   - RTK fix → local ENU pose (for GUI and BT reference only)
   - Publishes /gps/absolute_pose (PoseWithCovarianceStamped)
   - Variable rate (10-20 Hz depending on RTK health)

3) localization_monitor_node
   - Monitors /odometry/filtered_map covariance + GPS status
   - Detects degradation (5 modes)
   - Publishes /localization/status (DiagnosticStatus)

↓

robot_localization dual EKF (launched by mowgli_bringup):

ekf_odom_node (50 Hz, local EKF, two_d_mode):
   - Fuses: /wheel_odom + /imu/data (roll/pitch + gyro_z)
   - Output: /odometry/filtered (odom frame)
   - Publishes TF: odom → base_footprint  (continuous, never jumps)

navsat_transform_node (30 Hz):
   - Fuses /gps/fix + /odometry/filtered + /imu/data
   - Uses a fixed lat/lon datum from mowgli_robot.yaml
   - Publishes /odometry/gps in the map frame

ekf_map_node (30 Hz, global EKF, two_d_mode) — DEFAULT:
   - Fuses: same wheel + IMU inputs
             + /gps/pose_cov  (PoseWithCovarianceStamped from
                               navsat_to_absolute_pose_node)
             + /imu/cog_heading  (GPS-COG absolute yaw from cog_to_imu.py)
             + /imu/mag_yaw     (optional, when use_magnetometer=true)
   - Output: /odometry/filtered_map (map frame)
   - Publishes TF: map → odom

— OR — (use_fusion_graph:=true, opt-in)

fusion_graph_node (10 Hz, GTSAM iSAM2 factor-graph):
   - 1-for-1 replacement for ekf_map_node. Same inputs, same outputs.
   - Pose2 graph with wheel between-factor (non-holo σ_y << σ_x), gyro
     between-factor on yaw, custom GnssLeverArmFactor (analytic Jacobian),
     and unary yaw factors for COG / mag.
   - Optional LiDAR scan-matching between-factors and loop-closure factors
     (gated by use_scan_matching / use_loop_closure).
   - Output: /odometry/filtered_map (map frame)
   - Publishes TF: map → odom
   - Adds: /fusion_graph/diagnostics, /fusion_graph/markers
   - Services: ~/save_graph (Trigger), ~/clear_graph (Trigger)

No SLAM: the /map OccupancyGrid is generated by map_server_node from
user-defined area polygons, not from scan matching.

↓

Final Output:
  /tf tree: map → odom → base_footprint
  /odometry/filtered      (local EKF, odom frame)
  /odometry/filtered_map  (global EKF, map frame)
  /map (area polygons from map_server_node, not SLAM)
```

#### 3a. wheel_odometry_node

**Inputs:**
- Hardware bridge's Status messages (contains WheelTick data)

**Outputs:**
- `/wheel_odom` (nav_msgs/Odometry, 50 Hz)

**Algorithm: Differential Drive Kinematics**

```
Input:
  RL/RR tick deltas since last update
  Odometry estimate: (x, y, theta)

Process (midpoint integration):
  d_left  = ticks_rl_delta / TICKS_PER_METER
  d_right = ticks_rr_delta / TICKS_PER_METER

  d_center = (d_left + d_right) / 2       # Forward motion
  d_theta  = (d_right - d_left) / TRACK   # Rotation (TRACK = wheel separation)

  # Midpoint integration: use orientation at mid-turn
  theta_mid = theta + d_theta / 2
  x += d_center * cos(theta_mid)
  y += d_center * sin(theta_mid)
  theta += d_theta

Output:
  Odometry message with pose (x, y, theta) and twist (vx, vy, vtheta)
  Covariance:
    - Pose covariance: large (odometry-only estimates drift)
    - Twist covariance: moderate (reflects encoder noise)
```

**Covariance Strategy:**
- Only `vx` and `vyaw` components are configured in the odom EKF (see localization.yaml)
- Pose covariance intentionally large to prevent odometry from dominating the filter
- EKF will apply heavier corrections from IMU and GPS

**Parameters (wheel_odometry.yaml):**
```yaml
wheel_odometry:
  ros__parameters:
    wheel_separation_distance: 0.35    # Left-to-right wheel centre distance (m)
    ticks_per_meter: 300              # Encoder resolution
    timeout_period_ms: 5000            # Warn if no WheelTick for 5s
```

#### 3b. navsat_to_absolute_pose_node

**Inputs:**
- `/gps/rtk_fix` (sensor_msgs/NavSatFix with fix type indicator)

**Outputs:**
- `/gps/absolute_pose` (geometry_msgs/PoseWithCovarianceStamped)

**Purpose:** Convert NavSatFix (latitude/longitude) to a local ENU pose for GUI visualization, Behavior Tree reference, and a PoseWithCovarianceStamped twin (`/gps/pose_cov`) that `ekf_map_node` fuses. `navsat_transform_node` handles NavSatFix directly for the EKF pipeline; this node publishes the parallel Mowgli-specific `AbsolutePose` topic.

**Algorithm: GNSS to Local ENU**

```
1. GPS origin (datum) set from mowgli_robot.yaml (datum_lat, datum_long)
2. All fixes converted to ENU relative to datum origin:

   Δlat = lat - datum_lat
   Δlon = lon - datum_long
   Δalt = alt - datum_alt

   e = EARTH_RADIUS_M * Δlon * cos(datum_lat)
   n = EARTH_RADIUS_M * Δlat
   u = Δalt

   Output: [e, n, u] in local ENU frame

3. Covariance scaling based on RTK fix type:
   RTK Fixed:     covariance *= 1.0   (best, ~0.01-0.05 m)
   RTK Float:     covariance *= 10.0  (good, ~0.1-0.5 m)
   DGPS/SPS:      covariance *= 100.0 (poor, ~1-5 m)
   No fix:        skip publishing
```

**Parameters (navsat_to_absolute_pose.yaml):**
```yaml
navsat_to_absolute_pose:
  ros__parameters:
    map_frame_id: "map"
    earth_radius_m: 6371008.8
    # Datum origin read from mowgli_robot.yaml
```

#### 3c. localization_monitor_node

**Inputs:**
- `/odometry/filtered_map` (from `ekf_map_node`)
- `/status` (for wheel tick freshness)
- `/gps/rtk_fix` (for fix status)

**Outputs:**
- `/localization/status` (diagnostic_msgs/DiagnosticStatus)
- `/mowgli/localization/mode` (std_msgs/String, latched, for debug/logging)
- `/mowgli/localization/mode_id` (std_msgs/Int32, latched, numeric mode)

**Mode debounce (`mode_debounce_sec`, default 1.0 s):** the published mode is
hysteretic — a changed mode must persist continuously for the debounce window
before it is committed and published. This filters the per-epoch RTK
Fixed↔Float flicker (an F9P's `carrSoln` can toggle every epoch under motion
while position σ stays sub-cm), so the mode — and anything gating on it — does
not flap. Set `mode_debounce_sec: 0` to commit immediately.

**Degradation Modes (5 levels):**

| Level | Name | Condition | Response |
|-------|------|-----------|----------|
| 0 | OK | EKF healthy, all sensors fresh | Continue normally |
| 1 | ODOMETRY_STALE | No wheel ticks for 2s | Warn in logs, reduce planner timeout |
| 2 | GPS_TIMEOUT | No fix for 10s (RTK Float OK) | Use wheel odom only, increase drift tolerance |
| 3 | GPS_DEGRADED | RTK Float (not Fixed) | Use GPS but with higher variance |
| 4 | FILTER_DIVERGENCE | EKF position covariance > threshold | Emergency stop recommended |

**Parameters (localization_monitor.yaml):**
```yaml
localization_monitor:
  ros__parameters:
    odom_stale_timeout_sec: 2.0
    gps_timeout_sec: 10.0
    max_acceptable_fusion_variance: 0.25   # m² for position
    mode_debounce_sec: 1.0                 # hysteresis on published mode (0 = off)
```

#### 3d. robot_localization Diagnostics Integration

**Monitoring:**
The diagnostics_node monitors EKF health by subscribing to `/odometry/filtered_map` and checking:
- **Rate:** Verify ~30 Hz update rate (report WARN if < 10 Hz)
- **Position variance:** Monitor x, y components of pose covariance (Z is clamped to zero under `two_d_mode`)
- **Orientation (yaw):** Check yaw angle convergence and variance
- **GPS consistency:** Compare `/odometry/filtered_map` to `/gps/absolute_pose` and warn if the gap widens beyond RTK accuracy expectations

**Diagnostics Output:**
Aggregated into `/diagnostics` as sub-status with levels:
- **OK:** Rate nominal, variances converged, no drift
- **WARN:** Rate degraded or variance increasing
- **ERROR:** Rate critical or variance diverging (filter failure)

---

### Optional: Factor-Graph Localizer (fusion_graph) {#optional-factor-graph-localizer-fusion_graph}

**Package:** `mowgli_localization` co-hosts `ros2/src/fusion_graph/` (separate ament_cmake package).

**Purpose:** opt-in replacement for `ekf_map_node` built on **GTSAM iSAM2**. Same wire contract — `/odometry/filtered_map` + `map → odom` TF — but the map-frame estimate is the result of an incremental Pose2 factor graph, which lets LiDAR scan-matching and loop-closure factors carry the position through multi-minute RTK-Float windows where the EKF would otherwise drift on dead-reckoning alone.

**Activation:** set `use_fusion_graph:=true` on `navigation.launch.py` (or in `mowgli_robot.yaml`). The two backends are **mutually exclusive**: only one of `ekf_map_node` or `fusion_graph_node` publishes `map → odom` at any time. `ekf_odom_node` (local EKF, `odom → base_footprint`) is unchanged in either mode.

**Graph structure (per node, 10 Hz):**

| Factor | Source | Notes |
|---|---|---|
| Wheel `BetweenFactor` | `/wheel_odom` | Body-frame Pose2; non-holonomic σ_y ≪ σ_x. |
| Gyro `BetweenFactor` (yaw only) | `/imu/data` | Integrated `wz` over the inter-node window. |
| Custom `GnssLeverArmFactor` | `/gps/fix` | Analytic Jacobian — antenna lever-arm rotates with the node's yaw, so GPS XY couples to heading correctly. Robust Huber when fix < `STATUS_GBAS_FIX`. |
| Yaw unary | `/imu/cog_heading` | GPS course-over-ground absolute yaw, gated on forward motion. |
| Yaw unary (optional) | `/imu/mag_yaw` | Tilt-compensated mag yaw, only when `use_magnetometer:=true`. |
| Scan `BetweenFactor` (optional) | `/scan` | ICP between consecutive nodes; gated on `use_scan_matching:=true`. |
| Loop-closure `BetweenFactor` (optional) | `/scan` + scan storage | Candidate search around new nodes; gated on `use_loop_closure:=true`. |

**Public surface (in addition to the EKF surface):**

- **Topics**:
  - `/fusion_graph/diagnostics` (`diagnostic_msgs/DiagnosticArray`, 1 Hz) — `total_nodes`, `scans_attached`, `loop_closures`, `scans_received`, `scan_matches_ok`, `scan_matches_fail`, `cov_xx`, `cov_yy`, `cov_yawyaw`. Surfaced in the GUI's *Diagnostics → Fusion Graph (iSAM2)* panel.
  - `/fusion_graph/markers` (`visualization_msgs/MarkerArray`, 1 Hz, transient_local) — node positions, trajectory, loop-closure edges (decimated to ≤1500 nodes).
  - `/imu/fg_yaw` (`sensor_msgs/Imu`, 10 Hz) — yaw-only output that can feed `ekf_map_node` as a tight yaw source when running side-by-side in observer mode.
- **Services** (both `std_srvs/Trigger`):
  - `~/save_graph` — persists graph (XML), per-node scans (binary), and metadata (datum + indices) to `/ros2_ws/maps/fusion_graph.{graph,scans,meta}`. Auto-fires on RECORDING→IDLE, on dock arrival, and every `periodic_save_period_s` (default 5 min) during AUTONOMOUS state.
  - `~/clear_graph` — wipes iSAM2 + scans + queues. The next valid pose seed (GPS, set_pose, or scan-match relocalization) re-initializes.
- **Parameter knobs** worth knowing: `node_period_s` (10 Hz default), `wheel_sigma_*`, `gyro_sigma_theta`, `gps_sigma_floor`, `cov_update_every_n`, `isam2_relinearize_skip`, `isam2_rebase_every_nodes`, `scan_retention_nodes`, plus the LC / ICP block (`lc_max_dist_m`, `lc_min_age_s`, `icp_max_iter`, …). All declared at startup; no dynamic_reconfigure.

**State persistence:**
- On disk: `<graph_save_prefix>.graph` (gtsam serialised Values), `.scans` (per-node Eigen `Vector2d` blobs), `.meta` (next index, last node time, datum lat/lon).
- On startup: if `autoload_graph:=true` and the files exist, the node resumes from the last-saved state; subsequent ticks add nodes after the highest loaded index. This is what makes "park the robot, restart ROS2, resume mowing" work without losing the graph.

**Why it exists:** the EKF treats GPS as an unbiased XY observation and times out cleanly during dropouts, but it has no mechanism to use LiDAR for an absolute pose lock. The factor graph adds those constraints (scan between-factors and loop-closure factors) without giving up the GPS / wheel / IMU fusion the EKF already does well — at the cost of a richer node and a non-trivial CPU budget (~5 ms/tick at 10 Hz on a Pi 4 with scan matching enabled).

**Purpose:** Configuration, URDF, and launch orchestration for the entire stack.

**Location:** `src/mowgli_bringup/`

#### URDF: mowgli.urdf.xacro

**Robot Description:**

```
base_footprint (on ground, fixed to base_link)
    │
    ├── base_link (chassis centre at wheel height, 0.10 m above ground)
    │   │
    │   ├── left_wheel_link
    │   │   └── left_wheel_joint (revolute, axis Y)
    │   │
    │   ├── right_wheel_link
    │   │   └── right_wheel_joint (revolute, axis Y)
    │   │
    │   ├── blade_link
    │   │   └── blade_joint (revolute, axis Z)
    │   │
    │   ├── imu_link (fixed to chassis)
    │   │   └── imu_joint (fixed)
    │   │
    │   ├── gps_link (fixed to chassis top)
    │   │   └── gps_joint (fixed)
    │   │
    │   └── laser_link (LiDAR mount, typically on top)
    │       └── laser_joint (fixed)
    │
    └── (caster wheels as collision-only links, no TF)
```

**Key Dimensions:**

- **Chassis:** 0.55 m long × 0.40 m wide × 0.25 m tall
- **Wheels:** 0.10 m radius, 0.05 m width, 0.35 m track (centre-to-centre)
- **Ground clearance:** 0.10 m (wheel radius, base_link height)
- **Blade:** 0.15 m radius disc, 0.02 m height (under base_link)
- **Mass distribution:**
  - Chassis: 8.0 kg
  - Each wheel: 0.5 kg
  - Blade: 0.3 kg

**Transform Tree (TF):**

```
Map frame (GPS-anchored via navsat_transform datum)
    │
    ├── [ekf_map_node publishes map→odom @ 30 Hz]
    │
Odometry frame (continuous, drift-only)
    │
    ├── [ekf_odom_node publishes odom→base_footprint @ 50 Hz]
    │
Base footprint (on ground, robot frame for Nav2/robot_localization)
    │
    ├── [robot_state_publisher outputs static TFs]
    │
Base link (rear wheel axle, OpenMower convention)
    │
Sensor frames:
    ├── imu_link (IMU data frame)
    ├── laser_link (LiDAR data frame)
    ├── gps_link (GPS antenna location)
    └── [wheel links for visualization]
```

#### Launch Files

**mowgli.launch.py** – Real Hardware

Starts:
1. `robot_state_publisher` – Processes URDF/xacro, publishes robot_description and static TFs
2. `hardware_bridge_node` – Serial bridge to STM32
3. `twist_mux` – Priority-based cmd_vel multiplexer
4. `ekf_odom_node` + `navsat_transform_node` + `ekf_map_node` (robot_localization dual EKF) – Wheel odometry + IMU + GPS fusion
5. `wheel_odometry_node`, `imu_filter_madgwick`, `navsat_to_absolute_pose_node`, `localization_monitor_node` – Sensor preparation
6. (Optional) SLAM, Nav2, behavior tree nodes

**simulation.launch.py** – Gazebo Ignition

Starts:
1. `robot_state_publisher` – Same as real hardware
2. Gazebo Ignition (empty world or custom SDF)
3. `ros_gz_sim create` – Spawns Mowgli model at specified pose
4. `ros_gz_bridge` – Bridges sensor topics and actuator commands
5. `joint_state_publisher` – Publishes wheel joint states for visualization

**navigation.launch.py** – robot_localization + Nav2 stack (included by main)

Starts (in order):
1. `ekf_odom_node`, `navsat_transform_node`, `ekf_map_node` (robot_localization).
   `ekf_odom_node` publishes `odom→base_footprint`; `ekf_map_node` publishes
   `map→odom`. Gated on `use_ekf`.
2. `navsat_to_absolute_pose_node`, `cog_to_imu.py`, `mag_yaw_publisher.py`,
   `dock_yaw_to_set_pose` (mowgli_localization helpers).
3. `nav2_bringup` — all Nav2 servers (controller, planner, behaviors,
   costmaps, lifecycle manager). Uses `nav2_params.yaml` when
   `use_lidar=true`, `nav2_params_no_lidar.yaml` otherwise.

#### Configuration Files

**hardware_bridge.yaml** – Serial communication, IMU cal sample count.
**robot_localization.yaml** – Dual-EKF tuning: per-EKF sensor configs (wheel odom, IMU, GPS pose, COG-yaw IMU), process noise covariance, datum settings for `navsat_transform_node`.
**nav2_params.yaml** – Navigation stack (costmaps, planner, controller) with LiDAR.
**nav2_params_no_lidar.yaml** – Navigation stack for GPS-only operation (no obstacle layer, collision_monitor pass-through).
**twist_mux.yaml** – Velocity command multiplexing (nav < teleop < emergency).

---

### 5. mowgli_nav2_plugins

**Purpose:** Custom Nav2 controller plugins for navigation and coverage path following.

**Location:** `src/mowgli_nav2_plugins/`

**Plugin Registration:** `ftc_controller_plugin.xml`

```xml
<library path="libftc_controller_plugin">
  <class type="mowgli_nav2_plugins::FTCController"
         base_class_type="nav2_core::Controller">
    <description>Follow-The-Carrot controller for both transit and coverage navigation</description>
  </class>
</library>
```

**Controller Profiles (Active Configuration):**

Two controllers are active for different navigation modes:
1. **FollowPath** – Transit navigation and docking (RPP + RotationShimController wrapper)
2. **FollowCoveragePath** – Coverage path following (FTCController with 3-axis PID, <10mm lateral accuracy)

#### FTCController: 4-State FSM & Path-Indexed Algorithm (Active for Coverage)

**State Machine:**

```
      Initial State
           │
           ▼
  ┌───────────────────┐
  │   PRE_ROTATE      │
  │ (align with path) │
  └───────────────────┘
          │
          ▼
  ┌───────────────────┐
  │   FOLLOWING       │ ← advance along path via path index
  │ (path tracking)   │   (not lookahead-based, but index-based)
  └───────────────────┘
          │
          ▼
  ┌────────────────────────────────────┐
  │   WAITING_FOR_GOAL_APPROACH        │
  │ (robot near goal, slow approach)   │
  └────────────────────────────────────┘
          │
          ▼
  ┌───────────────────┐
  │   POST_ROTATE     │
  │ (align to goal    │
  │  orientation)     │
  └───────────────────┘
          │
          ▼
  ┌───────────────────┐
  │   FINISHED        │
  │ (goal reached)    │
  └───────────────────┘

Oscillation Recovery (any state):
  If velocity < threshold for > 5 sec → hold, retry
```

**Algorithm: Path-Indexed PID Control**

Inputs:
- Global path (array of PoseStamped with positions and orientations)
- Robot pose (from TF: odom → base_link)
- Costmap (for obstacle checking)

Process:

1. **Path Index Advancement:**
   - Maintain `current_index_` along the path (not lookahead distance)
   - Advance index as robot progresses along path
   - Control point is the pose at current_index_ (or interpolated between indices)

2. **Error Calculation (in base_link frame):**
   ```
   lateral_error = cross-track distance to path
   longitudinal_error = error along path heading
   angular_error = angle to target orientation
   ```

3. **Three Independent PID Channels:**
   ```
   Lateral PID:        u_lat  = Kp_lat * lat_error   + Ki_lat * ∫lat_error   + Kd_lat * d(lat_error)/dt
   Longitudinal PID:   u_lon  = Kp_lon * lon_error   + Ki_lon * ∫lon_error   + Kd_lon * d(lon_error)/dt
   Angular PID:        u_ang  = Kp_ang * ang_error   + Ki_ang * ∫ang_error   + Kd_ang * d(ang_error)/dt
   ```

4. **Velocity Command Generation:**
   - Lateral error modulates steering (angular output)
   - Longitudinal error and state-dependent speeds control forward motion
   - Speed profile: fast (0.5 m/s) → slow (0.2 m/s) near goal

5. **Collision Avoidance:**
   - Monitor costmap within robot footprint
   - If obstacle within lookahead, trigger oscillation recovery
   - If persistent, return FAILURE to planner

**State Transitions:**

- **PRE_ROTATE → FOLLOWING:** Robot roughly aligned with path start
- **FOLLOWING → WAITING_FOR_GOAL_APPROACH:** Current_index approaches path end (robot < max_follow_distance from goal)
- **WAITING_FOR_GOAL_APPROACH → POST_ROTATE:** Robot within xy_goal_tolerance, ready to orient to final pose
- **POST_ROTATE → FINISHED:** Robot within yaw_goal_tolerance (orientation correct)
- Any state: Oscillation detected → recover, retry

**Oscillation Detection & Recovery:**

The `FailureDetector` class tracks velocity history:
- If |velocity| < `oscillation_v_eps` and |angular_velocity| < `oscillation_omega_eps` for > `oscillation_recovery_min_duration`
- Robot is considered stuck
- Controller holds position for 2 seconds, then retries the path

**Parameters (mowgli_nav2_plugins section, nav2_params.yaml):**

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"

FollowCoveragePath:
  plugin: "mowgli_nav2_plugins::FTCController"

  # Speed profiles (state-dependent)
  speed_fast: 0.5                           # m/s in FOLLOWING state
  speed_slow: 0.2                           # m/s in WAITING_FOR_GOAL_APPROACH
  speed_fast_threshold: 1.5                 # distance to goal (m) before slowing
  speed_angular: 20.0                       # angular speed target (rad/s, virtual)
  acceleration: 1.0                         # m/s²

  # PID longitudinal
  kp_lon: 1.0
  ki_lon: 0.0
  ki_lon_max: 10.0
  kd_lon: 0.0

  # PID lateral
  kp_lat: 1.0
  ki_lat: 0.0
  ki_lat_max: 10.0
  kd_lat: 0.0

  # PID angular
  kp_ang: 1.0
  ki_ang: 0.0
  ki_ang_max: 10.0
  kd_ang: 0.0

  # Robot limits
  max_cmd_vel_speed: 0.5                    # m/s (clamping saturation)
  max_cmd_vel_ang: 1.0                      # rad/s
  max_goal_distance_error: 1.0              # m (triggers failure if exceeded)
  max_goal_angle_error: 10.0                # degrees
  goal_timeout: 5.0                         # seconds before goal declared unreachable
  max_follow_distance: 1.0                  # m (distance at which path end is "reached")

  # Options
  forward_only: true                        # no backward driving
  debug_pid: false
  debug_obstacle: false

  # Recovery
  oscillation_recovery: true
  oscillation_v_eps: 5.0                    # cm/s threshold for velocity stagnation
  oscillation_omega_eps: 5.0                # deg/s threshold for rotation stagnation
  oscillation_recovery_min_duration: 5.0    # seconds

  # Obstacle checking
  check_obstacles: true
  obstacle_lookahead: 5                     # number of path points to check ahead
  obstacle_footprint: true                  # use full robot footprint
```

#### FailureDetector (oscillation_detector.hpp)

Ring-buffer-based failure detection:

```cpp
class FailureDetector {
public:
  void setBufferLength(int length);
  void update(double linear_vel, double angular_vel);
  bool isOscillating() const;               // Returns true if stuck

private:
  std::vector<double> velocity_history_;
  int buffer_index_{0};
};
```

Fills a history buffer with (v, omega) samples at each `computeVelocityCommands()` call. Returns true if all buffered values fall below threshold (stagnation detected).

---

### 6. mowgli_behavior

**Purpose:** High-level reactive control using BehaviorTree.CPP v4 with multi-mode state machine.

**Location:** `src/mowgli_behavior/`

**Architecture:**

```
BehaviorTreeNode (main ROS2 node, 10 Hz)
    │
    ├── BTContext (shared state across all nodes)
    │   ├── node reference (for publishing, services, actions)
    │   ├── latest_status (from hardware bridge)
    │   ├── latest_emergency (latched emergency flag)
    │   ├── latest_power (battery voltage)
    │   ├── command_queue (high-level commands from GUI)
    │   └── [other sensory state]
    │
    └── BehaviorTree instance (XML: main_tree.xml)
        │
        └── ReactiveSequence: Root
            │
            ├── Fallback: EmergencyGuard
            │   ├── Inverter(IsEmergency) → continue if safe
            │   └── Sequence: EmergencyHandler
            │       ├── SetMowerEnabled(false)
            │       ├── StopMoving()
            │       ├── PublishHighLevelStatus(EMERGENCY)
            │       └── Fallback: AutoResetOrWait
            │           ├── Sequence: DockAutoReset
            │           │   ├── IsCharging() → on dock?
            │           │   ├── ResetEmergency() → release firmware latch
            │           │   └── WaitForDuration(1.0s)
            │           └── WaitForDuration(1.0s)  (not on dock — retry)
            │
            ├── Fallback: BoundaryGuard
            │   ├── Inverter(IsBoundaryViolation) → continue if inside
            │   └── Sequence: BoundaryHandler
            │       ├── SetMowerEnabled(false), StopMoving()
            │       ├── RetryUntilSuccessful(5): BackUp + check boundary
            │       └── Fallback: BoundaryEmergencyStop
            │
            ├── Fallback: GPSModeSelector
            │   ├── IsGPSFixed → SetNavMode(precise)
            │   └── SetNavMode(degraded)
            │
            └── Fallback: MainLogic
                ├── Sequence: CriticalBatteryDock (battery < 10%)
                │   ├── IsBatteryLow(10.0)
                │   ├── SetMowerEnabled(false), StopMoving()
                │   ├── SaveObstacles(), SaveSlamMap()
                │   └── DockRobot() → IDLE_DOCKED, ClearCommand
                │
                ├── Sequence: MowingSequence (COMMAND_START = 1)
                │   ├── IsCommand(1)
                │   ├── Undock (with GPS wait, heading calibration via undock TF delta)
                │   ├── Multi-area coverage loop:
                │   │   └── Repeat(num_cycles=100): AreaLoop
                │   │       ├── GetNextUnmowedArea() — picks next area with un-mowed cells
                │   │       ├── Fallback MowOrSkipArea:
                │   │       │   ├── Sequence PlanThenFollow:
                │   │       │   │   ├── PlanCoverageArea(area_index, headland_width_m=0.20)
                │   │       │   │   │   — calls map_server/get_remaining_area_polygon
                │   │       │   │   │     (outer + mow_progress holes), then
                │   │       │   │   │     mowgli_coverage compute_coverage_path action
                │   │       │   │   │     (F2C v2: ConstHL → TrapezoidalDecomp →
                │   │       │   │   │      BoustrophedonOrder → PathPlanning + Dubins
                │   │       │   │   │      connectors). ONE plan per area per session.
                │   │       │   │   └── RetryUntilSuccessful(num_attempts=5):
                │   │       │   │       Fallback MowOrRecover:
                │   │       │   │         ├── FollowStrip — sends path to
                │   │       │   │         │   FollowCoveragePath (FTCController)
                │   │       │   │         ├── StuckBackoff — IsObstacleStuck →
                │   │       │   │         │   BackUp(0.40m) + ClearCostmap → re-tick
                │   │       │   │         └── DynamicObstacleSkip —
                │   │       │   │             WasRecentlyInCollisionStop →
                │   │       │   │             ClearCostmap + Wait → re-tick
                │   │       │   └── Sequence AreaUnreachable (PlanCoverageArea FAILURE
                │   │       │       OR 5 retries exhausted) — advance AreaLoop
                │   └── All areas complete → disable blade, save, dock → IDLE_DOCKED
                │
                ├── Sequence: HomeSequence (COMMAND_HOME = 2)
                │   ├── IsCommand(2)
                │   ├── SetMowerEnabled(false), StopMoving()
                │   ├── SaveObstacles(), SaveSlamMap()
                │   └── DockRobot() → IDLE_DOCKED, ClearCommand
                │
                ├── Sequence: RecordingSequence (COMMAND_RECORD_AREA = 3)
                │   ├── IsCommand(3)
                │   ├── PublishHighLevelStatus(RECORDING)
                │   ├── RecordArea (records trajectory at 2 Hz, Douglas-Peucker
                │   │   simplification, saves polygon via /map_server_node/add_area)
                │   │   Listens for COMMAND_RECORD_FINISH=5 or COMMAND_RECORD_CANCEL=6
                │   ├── PublishHighLevelStatus(RECORDING_COMPLETE)
                │   └── ClearCommand
                │
                ├── Sequence: ManualMowingSequence (COMMAND_MANUAL_MOW = 7)
                │   ├── IsCommand(7)
                │   ├── PublishHighLevelStatus(MANUAL_MOWING)
                │   └── WaitForDuration(0.5s)  (teleop via /cmd_vel_teleop)
                │
                └── Sequence: IdleSequence (default)
                    ├── SetMowerEnabled(false), StopMoving()
                    ├── PublishHighLevelStatus(IDLE)
                    └── WaitForDuration(0.5s)

Update frequency: 10 Hz tick() cycle
Execution pattern: ReactiveSequence re-evaluates all children each tick
```

**Tree Structure (from main_tree.xml):**

The tree implements a priority-based fallback selector with reactive guards:
1. **Emergency Guard (highest priority):** If emergency active → disable, stop, halt. Auto-resets when robot is placed on dock (charging detected) by calling `ResetEmergency` — firmware decides whether to actually clear the latch based on physical trigger state.
2. **Boundary Guard:** If outside mowing area → stop, back up (up to 5 attempts), emergency stop if still outside.
3. **GPS Mode Selector:** Switch between precise (RTK) and degraded navigation modes.
4. **Critical Battery Dock:** If battery < 10% → dock immediately (uninterruptible).
5. **Mowing (COMMAND_START=1):** Multi-area coverage: iterates through all unmowed areas via GetNextUnmowedArea, then cell-based strip coverage per area with GPS wait, heading calibration from undock, rain/battery reactive guards, strip-by-strip execution via GetNextStrip/TransitToStrip/FollowStrip. Coverage progress tracked per area.
6. **Home (COMMAND_HOME=2):** Return to dock on user request.
7. **Area Recording (COMMAND_RECORD_AREA=3):** Drive the boundary, record trajectory at 2 Hz, finish (cmd 5) saves Douglas-Peucker simplified polygon via map_server_node, cancel (cmd 6) discards.
8. **Manual Mowing (COMMAND_MANUAL_MOW=7):** Teleop via `/cmd_vel_teleop` (twist_mux priority). Blade managed by GUI (fire-and-forget to firmware). Collision_monitor, GPS, SLAM all remain active.
9. **Idle (default):** Standby, periodic status updates.

Each sequence transitions through defined high-level states (NULL=0, IDLE=1, AUTONOMOUS=2, RECORDING=3, MANUAL_MOWING=4) published via HighLevelStatus.msg for GUI and firmware synchronization.

#### Condition Nodes (condition_nodes.cpp)

```cpp
class IsEmergency : public BT::ConditionNode
// Returns SUCCESS if active_emergency bit set

class IsCharging : public BT::ConditionNode
// Returns SUCCESS if dock charging state is active

class IsBatteryLow : public BT::ConditionNode
// Checks battery below threshold

class NeedsDocking : public BT::ConditionNode
// Checks battery_voltage < threshold parameter (default 20.0 V)

class IsBatteryAbove : public BT::ConditionNode
// Checks battery_percent > threshold (used for charge-to-95% logic)

class IsCommand : public BT::ConditionNode
// Port In: command (uint8)
// Returns SUCCESS if command matches current high-level command from GUI

class IsGPSFixed : public BT::ConditionNode
// Returns SUCCESS if GPS has RTK fix

class IsBoundaryViolation : public BT::ConditionNode
// Returns SUCCESS if robot is outside mowing area boundary

class IsRainDetected : public BT::ConditionNode
// Returns SUCCESS if rain sensor detects rain

class IsNewRain : public BT::ConditionNode
// Returns SUCCESS only on new rain onset (not if it was raining at start)

class IsResumeUndockAllowed : public BT::ConditionNode
// Tracks resume-undock attempts (max_attempts port), prevents infinite loops

class IsChargingProgressing : public BT::ConditionNode
// Returns SUCCESS if charger is active and battery level increasing

class ReplanNeeded : public BT::ConditionNode
// Returns SUCCESS if coverage replanning is required
```

#### Action Nodes (action_nodes.cpp, utility_nodes.cpp, recording_nodes.cpp)

```cpp
class NavigateToPose : public BT::AsyncActionNode
// Contacts Nav2 /navigate_to_pose action server
// Port In: goal="x;y;yaw" (string format)
// Returns RUNNING (in progress), SUCCESS (reached), FAILURE (abort/timeout)

class SetMowerEnabled : public BT::ActionNode
// Calls /mower_control service
// Port In: enabled (bool)
// Fire-and-forget; always returns SUCCESS (or gracefully continues in simulation)

class StopMoving : public BT::ActionNode
// Publishes zero Twist to /cmd_vel
// Returns SUCCESS

class PublishHighLevelStatus : public BT::ActionNode
// Publishes HighLevelStatus.msg (state enum + state_name string + coverage progress)
// Port In: state (uint8), state_name (string)
// Returns SUCCESS

class WaitForDuration : public BT::ActionNode
// Sleep for specified duration
// Port In: duration_sec (double)
// Returns SUCCESS after duration elapsed

class ClearCommand : public BT::ActionNode
// Clears the pending high-level command (e.g., COMMAND_START)
// Returns SUCCESS

class ClearCostmap : public BT::ActionNode
// Clears Nav2 local costmap
// Returns SUCCESS

class SaveSlamMap : public BT::ActionNode
// Persists SLAM map to disk
// Port In: map_path (string)
// Returns SUCCESS

class SaveObstacles : public BT::ActionNode
// Persists tracked obstacles to disk
// Returns SUCCESS

class SetNavMode : public BT::ActionNode
// Switches between "precise" (RTK) and "degraded" navigation modes
// Port In: mode (string)
// Returns SUCCESS

class BackUp : public BT::ActionNode
// Drives robot backward
// Port In: backup_dist (double), backup_speed (double)
// Returns SUCCESS/FAILURE

class ResetEmergency : public BT::ActionNode
// Calls /hardware_bridge/emergency_stop with emergency=false to release firmware latch
// Firmware is safety authority — only clears if physical trigger is no longer asserted
// Returns SUCCESS/FAILURE

class RecordUndockStart : public BT::ActionNode
// Records robot position at start of undock for heading calibration
// Returns SUCCESS

class CalibrateHeadingFromUndock : public BT::ActionNode
// Reads EKF TF to compute heading after undock, clears costmaps
// Returns SUCCESS

class WasRainingAtStart : public BT::ActionNode
// Records rain state at mowing start (to distinguish new rain from ongoing)
// Returns SUCCESS

class RecordResumeUndockFailure : public BT::ActionNode
// Increments resume-undock failure counter
// Returns SUCCESS

class DockRobot : public BT::AsyncActionNode
// Uses opennav_docking to dock the robot
// Port In: dock_id, dock_type
// Returns RUNNING/SUCCESS/FAILURE

class UndockRobot : public BT::AsyncActionNode
// Uses opennav_docking to undock the robot
// Port In: dock_type
// Returns RUNNING/SUCCESS/FAILURE

// Multi-area coverage node:
class GetNextUnmowedArea : public BT::AsyncActionNode
// Fetches next unmowed area from map_server_node ~/get_next_unmowed_area
// Returns SUCCESS with area polygon and coverage status, FAILURE when all areas complete

// Cell-based coverage nodes:
class GetNextStrip : public BT::AsyncActionNode
// Fetches next unvisited strip from map_server_node ~/get_next_strip
// Port In: area_index
// Returns SUCCESS with strip path, FAILURE when current area coverage complete

class TransitToStrip : public BT::AsyncActionNode
// Navigates to start of current strip using Nav2 (RPP controller)
// Returns RUNNING/SUCCESS/FAILURE

class FollowStrip : public BT::AsyncActionNode
// Follows current strip using Nav2 (FTCController for <10mm accuracy)
// Returns RUNNING/SUCCESS/FAILURE

// Area recording node:
class RecordArea : public BT::StatefulActionNode
// Records robot trajectory at configurable rate while user drives boundary
// Douglas-Peucker simplification, saves polygon via /map_server_node/add_area
// Publishes live preview on ~/recording_trajectory
// Port In: simplification_tolerance, min_vertices, min_area, record_rate_hz, is_exclusion_zone
// Returns RUNNING (recording), SUCCESS (finish cmd 5), FAILURE (cancel cmd 6 or error)
```

#### Tree Control (BehaviorTreeNode)

**Subscriptions:**
- `/status` – Mower state, rain sensor, blade status
- `/emergency` – Latched emergency flag
- `/power` – Battery voltage (v_battery)
- `/high_level_control` (service) – Receive mode commands from GUI (START, HOME, S1, S2, RECORD_AREA, RECORD_FINISH, RECORD_CANCEL, MANUAL_MOW)

**Services Called:**
- `/mower_control` – Enable/disable blade
- `/emergency_stop` – Release latched emergency
- `/navigate_to_pose` (Nav2) – Send navigation goals
- `~/get_next_strip` (mowgli_map/map_server_node) – Fetch next coverage strip on demand

**Publishing:**
- `/high_level_status` (std_msgs/UInt8) – Current state (IDLE, UNDOCKING, MOWING, etc.)

**Execution Model:**
- 10 Hz tick() cycle (100 ms)
- ReactiveSequence: re-evaluates all children on each tick
- Emergency guard is always first: any emergency → abort all activity
- Fallback selectors: try sequences in priority order (docking > mowing > home > idle)
- Action nodes (NavigateToPose, FollowCoveragePath) are async: return RUNNING while in progress

#### Node Registration (register_nodes.cpp)

BehaviorTree factory registration:

```cpp
void registerAllNodes(BT::BehaviorTreeFactory& factory) {
  // Condition nodes
  factory.registerNodeType<IsEmergency>("IsEmergency");
  factory.registerNodeType<IsCharging>("IsCharging");
  factory.registerNodeType<IsBatteryLow>("IsBatteryLow");
  factory.registerNodeType<IsRainDetected>("IsRainDetected");
  factory.registerNodeType<NeedsDocking>("NeedsDocking");
  factory.registerNodeType<IsBatteryAbove>("IsBatteryAbove");
  factory.registerNodeType<IsCommand>("IsCommand");
  factory.registerNodeType<IsGPSFixed>("IsGPSFixed");
  factory.registerNodeType<ReplanNeeded>("ReplanNeeded");
  factory.registerNodeType<IsBoundaryViolation>("IsBoundaryViolation");
  factory.registerNodeType<IsNewRain>("IsNewRain");
  factory.registerNodeType<IsResumeUndockAllowed>("IsResumeUndockAllowed");
  factory.registerNodeType<IsChargingProgressing>("IsChargingProgressing");

  // Action nodes
  factory.registerNodeType<SetMowerEnabled>("SetMowerEnabled");
  factory.registerNodeType<StopMoving>("StopMoving");
  factory.registerNodeType<ClearCostmap>("ClearCostmap");
  factory.registerNodeType<PublishHighLevelStatus>("PublishHighLevelStatus");
  factory.registerNodeType<WaitForDuration>("WaitForDuration");
  factory.registerNodeType<NavigateToPose>("NavigateToPose");
  factory.registerNodeType<SaveSlamMap>("SaveSlamMap");
  factory.registerNodeType<BackUp>("BackUp");
  factory.registerNodeType<ClearCommand>("ClearCommand");
  factory.registerNodeType<SaveObstacles>("SaveObstacles");
  factory.registerNodeType<SetNavMode>("SetNavMode");
  factory.registerNodeType<WasRainingAtStart>("WasRainingAtStart");
  factory.registerNodeType<RecordUndockStart>("RecordUndockStart");
  factory.registerNodeType<CalibrateHeadingFromUndock>("CalibrateHeadingFromUndock");
  factory.registerNodeType<DockRobot>("DockRobot");
  factory.registerNodeType<UndockRobot>("UndockRobot");
  factory.registerNodeType<RecordResumeUndockFailure>("RecordResumeUndockFailure");
  factory.registerNodeType<ResetEmergency>("ResetEmergency");

  // Multi-area coverage node
  factory.registerNodeType<GetNextUnmowedArea>("GetNextUnmowedArea");

  // Cell-based coverage nodes
  factory.registerNodeType<GetNextStrip>("GetNextStrip");
  factory.registerNodeType<FollowStrip>("FollowStrip");
  factory.registerNodeType<TransitToStrip>("TransitToStrip");

  // Area recording node
  factory.registerNodeType<RecordArea>("RecordArea");
}
```

---

### 7. Coverage Planning (mowgli_map + mowgli_coverage)

**Purpose:** Per-area F2C v2.0 coverage planning with `mow_progress` resume semantics. Two packages cooperate.

**Locations:** `src/mowgli_map/` (area storage, mow_progress grid, remaining-polygon service), `src/mowgli_coverage/` (Fields2Cover v2.0 coverage server).

**Coverage Loop (driven by BT nodes):**

1. `GetNextUnmowedArea` — outer loop. Picks the next area whose `mow_progress` layer still has un-mowed cells; writes its index to the blackboard. Returns FAILURE when all areas are complete.
2. `PlanCoverageArea` (ONE per area per session) — calls `/map_server_node/get_remaining_area_polygon` to fetch `area outer ring + mow_progress holes` (Boost.Geometry difference of the area polygon and the already-mowed cells, CCW outer / CW holes), then sends a `compute_coverage_path` action goal to `mowgli_coverage`. Stores the resulting `nav_msgs/Path` on the BT blackboard.
3. `FollowStrip` — sends the stored path to `FollowCoveragePath` (FTCController). On abort (collision wedge, FTC `WAITING_FOR_GOAL_APPROACH` timeout, etc.), `RetryUntilSuccessful(num_attempts=5)` re-ticks `FollowStrip` on the SAME path; FTC's `setPlan` resyncs to the closest pose, effectively "continue from where you left off" without re-planning. `IsObstacleStuck` and `WasRecentlyInCollisionStop` fallback branches insert `BackUp` + `ClearCostmap` between retries.

Progress is tracked in the `mow_progress` grid layer (survives restarts, stamp radius `tool_width / 2`). Coverage status is available via `~/get_coverage_status` service and `/map_server_node/coverage_cells` OccupancyGrid topic.

The legacy on-demand strip planner (`~/get_next_strip` service + `GetNextStrip` / `TransitToStrip` BT nodes) still exists in `map_server_node` as a fallback and to drive the `mow_progress` cell stamping logic, but the live coverage path is the F2C output.

**mowgli_coverage server (Fields2Cover v2.0):**

- Action: `compute_coverage_path` (type `opennav_coverage_msgs/action/ComputeCoveragePath`), same interface as the legacy `opennav_coverage` server — the BT client is unchanged.
- Backend pinned to F2C v2.0.0 at `/opt/fields2cover-200`. Legacy v1.2.1 install at `/opt/fields2cover-121` is still on the global `ld` path, so the package's `CMakeLists.txt` sets explicit `INSTALL_RPATH`s on both `libmowgli_coverage_core.so` and the executable so the loader picks v2.
- F2C pipeline per `computeCoveragePath()` call:
  1. **Robot setup** — `f2c::types::Robot(robot_width, op_width)`, `setMinTurningRadius`, `setMaxDiffCurv`. `op_width` is injected at launch from `mowgli_robot.yaml.tool_width`.
  2. **Cell construction** — `goal.polygons[0]` is the outer ring; subsequent polygons are interior holes (obstacles + already-mowed islands). F2C linear rings are closed if the caller didn't.
  3. **Headland inset** — `f2c::hg::ConstHL.generateHeadlands(cells, headland_width)`. Returns the inner planning field.
  4. **Headland traversal** — `f2c::hg::ConstHL.generateHeadlandSwaths(cov_width, n_passes, dir_out2in=true)` emits N concentric perimeter rings; we densify each ring to 10 cm steps (sparse vertex lists leave FTC's carrot jumping between corners → PolygonStop). Prepended to the path so the perimeter strip gets mowed first.
  5. **Trapezoidal decomposition** — `f2c::decomp::TrapezoidalDecomp.decompose(inner)` splits the inner field around interior holes into sub-cells. Without this, the routing step serialises ALL swaths north-to-south and `PathPlanning` connects non-adjacent endpoints with straight lines (`min_turning_radius=0.05 m`, no real arc) — visible as long diagonal connectors cutting across the field around obstacles. Skipped when `cell.size() == 1` (no holes).
  6. **Per sub-cell** — `f2c::sg::BruteForce.generateBestSwaths(NSwath, cov_width, sub_cell)` → `f2c::rp::BoustrophedonOrder.genSortedSwaths()` → `f2c::pp::PathPlanning.planPath(robot, ordered, DubinsCurvesCC)`.
  7. **Inter-cell connectors** — `pp.planPathForConnection(robot, prev_end_pt, prev_end_ang, {}, first_state.point, first_state.angle, DubinsCurvesCC)` inserts a continuous-curvature Dubins arc between the previous sub-cell's end pose and the next sub-cell's start pose (and between headland end / first sub-cell start). Without this, FTC's `WAITING_FOR_GOAL_APPROACH` state times out trying to drive between unconnected endpoints.
  8. **Conversion** — F2C path states → `nav_msgs/Path` (`PoseStamped` per state, yaw via half-angle quaternion).

**Key Parameters:**

`mowgli_robot.yaml`:

```yaml
tool_width: 0.18            # m – single source: map_server stamp + F2C operation_width
coverage_xy_tolerance: 0.05 # m – must stay < tool_width (capped at 0.15 in launch)
mowing_speed: 0.5           # m/s – injected into FollowCoveragePath.speed_fast
```

`nav2_params.yaml`:

```yaml
coverage_server:
  ros__parameters:
    default_headland_width: 0.20
    robot_width: 0.20
    operation_width: 0.18         # OVERRIDDEN at launch from tool_width
    min_turning_radius: 0.05
    linear_curv_change: 200.0
```

The mode-string params (`default_swath_angle_type: BRUTE_FORCE`, `default_route_type: BOUSTROPHEDON`, `default_path_type: DUBIN`, etc.) are declared for compatibility with the legacy `opennav_coverage` YAML schema but the v2 server uses a fixed pipeline — log line on startup tells the operator their override is being ignored.

#### Multi-Area Coverage

**Concept:** Instead of a single coverage path, users can define multiple mowing areas (polygons) that are mowed sequentially in a single autonomous session.

**Workflow:**
1. User records multiple areas using `COMMAND_RECORD_AREA` (3) — each area is a polygon boundary saved to map_server
2. User initiates mowing with `COMMAND_START` (1)
3. Behavior tree calls `GetNextUnmowedArea()` to fetch first area
4. Coverage planner generates strip path for that area
5. Robot mows current area strip-by-strip via GetNextStrip/TransitToStrip/FollowStrip
6. Once area is complete, BT calls `GetNextUnmowedArea()` again to fetch next area
7. Process repeats until all areas are mowed

**Progress Tracking:**
- Each area maintains its own coverage grid (`mow_progress` layer) persisted across sessions
- High-level status includes `current_area` and `areas_remaining` fields
- GUI shows progress per area and overall session progress
- If session interrupted, robot resumes from last completed area on restart

**Map Server Integration:**
- `map_server_node` maintains list of mowing areas and their coverage state
- Service: `/map_server_node/get_next_unmowed_area` — returns next area and its current coverage grid
- Service: `/map_server_node/mark_area_complete` — marks area as finished
- Enables robust recovery from power loss or manual pause

---

### 8. mowgli_monitoring

**Purpose:** System health diagnostics aggregation and external MQTT bridge.

**Location:** `src/mowgli_monitoring/`

**Architecture:**

```
Monitoring System
    │
    ├── DiagnosticsNode (1 Hz publish rate)
    │   │
    │   ├── Subscriptions (sensor QoS):
    │   │   ├── /status (Status) – mower state, sensors
    │   │   ├── /emergency (Emergency) – emergency status
    │   │   ├── /power (Power) – battery voltage, charging
    │   │   ├── /imu/data_raw (sensor_msgs/Imu)
    │   │   ├── /scan (sensor_msgs/LaserScan)
    │   │   ├── /wheel_odom (nav_msgs/Odometry)
    │   │   └── /gps/fix (sensor_msgs/NavSatFix)
    │   │
    │   └── Diagnostic Checks (aggregated to DiagnosticArray):
    │       ├── check_hardware_bridge() – last status age, mower state
    │       ├── check_emergency() – latched/active emergency status
    │       ├── check_battery() – voltage → SOC %, charger status
    │       ├── check_imu() – data freshness
    │       ├── check_lidar() – scan freshness, obstacles
    │       ├── check_gps() – fix type, lat/lon, age
    │       ├── check_odometry() – wheel odom freshness
    │       └── check_motors() – ESC/motor temperature
    │
    └── MqttBridgeNode (optional, for cloud telemetry)
        └── Republishes selected diagnostics to MQTT broker
            └── Topic pattern: /mowgli/diagnostics/{subsystem}
```

**Diagnostic Levels:**
- **OK** – All systems nominal
- **WARN** – Degraded but operational (e.g., GPS float, high temp)
- **ERROR** – Critical failure (e.g., no GPS fix, emergency active)
- **STALE** – Data stream timeout (sensor not reporting)

**Health Classification Functions:**

```cpp
uint8_t classify_freshness(age_sec, never, warn_sec, error_sec)
  // Returns OK, WARN, or ERROR based on age threshold

uint8_t classify_battery(percentage, warn_pct, error_pct)
  // Returns OK, WARN, or ERROR based on SOC threshold

uint8_t classify_temperature(temp_c, warn_c, error_c)
  // Returns OK, WARN, or ERROR based on temperature threshold
```

**Parameters (monitoring.yaml):**

```yaml
diagnostics_node:
  ros__parameters:
    publish_rate: 1.0                    # Hz – how often to aggregate
    freshness_warn_sec: 5.0              # sensor data age before warn
    freshness_error_sec: 10.0            # sensor data age before error
    battery_warn_pct: 20.0               # SOC % before warn
    battery_error_pct: 10.0              # SOC % before error
    motor_temp_warn_c: 60.0
    motor_temp_error_c: 80.0
```

**Output:**

Publishes `diagnostic_msgs/DiagnosticArray` to `/diagnostics` topic:
- Used by system monitors, RViz diagnostics viewer, and external dashboards
- Also ingested by BehaviorTree condition nodes (e.g., IsLocalizationHealthy, IsBatteryLow)

#### Behavior Tree Visualization

**BT State Logging:**
The behavior_tree_node publishes active node information via `/behavior_tree_log` topic:
- **Message type:** `BehaviorTreeLog` (custom msg in mowgli_interfaces)
- **Contents:** Active node name, node type, tick timestamp, execution status
- **Frequency:** Every 10 Hz tick, only when BT status changes
- **Use:** GUI displays active BT node in real-time diagnostics page

**GUI Integration:**
- Diagnostics page shows current BT node path and state
- Helps identify where robot is stuck or failing (e.g., "FollowCoveragePath" stuck on obstacle)
- Updates in real-time as BT executes

---

### 9. mowgli_simulation

**Purpose:** Gazebo Harmonic simulation environment with ros_gz_bridge integration.

**Location:** `src/mowgli_simulation/`

**Architecture:**

```
Simulation Stack (ros_gz_sim + ros_gz_bridge)
    │
    ├── Gazebo Harmonic Server
    │   └── World SDF (garden.sdf or empty_garden.sdf)
    │       └── Robot model (model.sdf)
    │           ├── Physics (differential drive plugin)
    │           ├── LiDAR sensor (Gazebo plugin publishes Gazebo topic)
    │           ├── IMU sensor (Gazebo plugin)
    │           └── Caster wheels (collision only, no TF)
    │
    ├── ros_gz_bridge (YAML-configured)
    │   └── Bridges Gazebo topics ↔ ROS2 topics
    │       ├── gz/model/mowgli_mower/cmd_vel → /cmd_vel (Twist)
    │       ├── gz/model/mowgli_mower/scan ← /scan (LaserScan)
    │       ├── gz/model/mowgli_mower/imu ← /imu/data_raw (Imu)
    │       └── gz/model/mowgli_mower/odometry ← /odom (Odometry, if available)
    │
    ├── Static TF Bridges (identity transforms)
    │   ├── lidar_link → mowgli_mower/laser_link/lidar_sensor
    │   └── imu_link → mowgli_mower/base_link/imu_sensor
    │
    └── ROS2 Stack
        └── (identical to real hardware)
            ├── robot_state_publisher (URDF)
            ├── Nav2 stack
            ├── SLAM (if enabled)
            └── Behavior tree
```

**Launch File: simulation.launch.py**

```python
Sequence:
  1. Declare arguments (world, use_rviz, headless, spawn_x/y/z/yaw)
  2. Launch Gazebo Ignition with SDF world file
  3. Spawn mowgli_mower model at (spawn_x, spawn_y, spawn_z, spawn_yaw)
  4. Start robot_state_publisher (URDF from mowgli_bringup)
  5. Start ros_gz_bridge with gazebo_bridge.yaml
  6. Create static TF bridges (Gazebo sensor frames → URDF frames)
  7. Optionally start RViz2 (mowgli_sim.rviz config)
```

**Gazebo Worlds:**

| World | File | Purpose |
|-------|------|---------|
| garden | worlds/garden.sdf | Realistic lawn with obstacles, trees, slope |
| empty_garden | worlds/empty_garden.sdf | Flat rectangular field (testing, no obstacles) |

**Robot Model (model.sdf):**

- **Differential drive plugin:** Subscribes to `cmd_vel`, drives wheels
- **LiDAR plugin:** 16-beam, 25 m range, publishes point cloud + LaserScan
- **IMU plugin:** 6-DOF gyro + accel, publishes Imu messages
- **Gazebo physics:** ODE or Bullet engine with friction/gravity
- **Collision meshes:** Wheel contact points, chassis boundary
- **Visual models:** 3D mesh for rendering

**Bridge Configuration (gazebo_bridge.yaml):**

```yaml
# Example bridge configuration
bridges:
  - topic_name: "/cmd_vel"
    ros_type_name: "geometry_msgs/msg/Twist"
    gz_type_name: "gz.msgs.Twist"
    direction: ROS_TO_GZ

  - topic_name: "/scan"
    ros_type_name: "sensor_msgs/msg/LaserScan"
    gz_type_name: "gz.msgs.LaserScan"
    direction: GZ_TO_ROS

  - topic_name: "/imu/data_raw"
    ros_type_name: "sensor_msgs/msg/Imu"
    gz_type_name: "gz.msgs.IMU"
    direction: GZ_TO_ROS
```

**Usage:**

```bash
# Full simulation with RViz
ros2 launch mowgli_simulation simulation.launch.py

# Headless (CI/testing, no GUI)
ros2 launch mowgli_simulation simulation.launch.py headless:=true use_rviz:=false

# Custom world
ros2 launch mowgli_simulation simulation.launch.py world:=/path/to/custom.sdf
```

The simulated robot is fully compatible with the real robot's ROS2 stack, allowing testing of Nav2, behavior trees, and coverage planning without hardware.

---

### 10. mowgli_map

**Purpose:** Map storage, persistence, and serving for offline navigation.

**Location:** `src/mowgli_map/`

**Features:**
- Loads pre-recorded SLAM maps from disk
- Serves /map topic (occupancy grid) to Nav2
- Persists maps generated during online SLAM runs
- Supports multi-map environments (e.g., different properties/zones)

---

## Custom Navigate-to-Pose Behavior Tree

Nav2's internal behavior tree is extended with a **GoalCheckerSelector** node to support the dual goal-checker architecture:

**File:** `src/mowgli_bringup/config/navigate_to_pose.xml`

```xml
<BehaviorTree ID="NavigateToPose">
  <Fallback name="Root">
    <!-- Try path-following with stopped_goal_checker (transit mode) -->
    <Sequence name="TransitSequence">
      <GoalCheckerSelector goal_checker="stopped_goal_checker"/>
      <FollowPath path="global_path"/>
    </Sequence>

    <!-- Fallback to coverage goal-checker (coverage mode) -->
    <Sequence name="CoverageSequence">
      <GoalCheckerSelector goal_checker="coverage_goal_checker"/>
      <FollowCoveragePath path="coverage_path"/>
    </Sequence>
  </Fallback>
</BehaviorTree>
```

The **GoalCheckerSelector** invokes the appropriate goal checker based on the current navigation mode, allowing different success criteria for transit (full orientation alignment) vs. coverage (path completion index).

---

## Foxglove Bridge Integration

Instead of rosbridge_suite, the system uses **Foxglove Bridge** for remote web UI and telemetry:

**Port:** 8765 (WebSocket)

**Benefits:**
- Modern TypeScript client library
- Native ROS2 support (Foxglove Studio)
- Lower latency than rosbridge
- Reduced CPU overhead

**Launch:** Included in main bringup
```yaml
foxglove_bridge:
  port: 8765
  num_threads: 2
```

---

## Complete Data Flow Diagram

### Scenario: Autonomous Coverage Mowing Run

```
1. User sends START command via GUI (or mobile app via Foxglove Bridge)
   └─→ /high_level_control message: COMMAND_START (1)

2. BehaviorTree (10 Hz):
   └─→ MowingSequence triggered:
       ├─ SetMowerEnabled(true) → blade motor on
       ├─ PublishHighLevelStatus(UNDOCKING)
       └─ Cell-based strip coverage loop:
            └─→ map_server_node (mowgli_map) plans strips on demand
                ├─ GetNextUnmowedArea (outer loop, iterates all areas)
                ├─ GetNextStrip → TransitToStrip → FollowStrip (inner loop)
                └─ Progress tracked in mow_progress grid layer

3. Navigation to coverage start:
   NavigateToPose(first_waypoint):
     ├─ Nav2 planner: global path from odometry to start
     ├─ RPP controller (RegulatedPurePursuit + RotationShimController)
     ├─ Costmap: /scan + odom → local obstacles
     └─ cmd_vel → hardware_bridge → STM32 → wheels

4. Coverage path following (CoverageWithRecovery loop):
   FollowCoveragePath:
     ├─ FTCController (Follow-the-Carrot with 3-axis PID, <10mm lateral accuracy)
     ├─ State machine: PRE_ROTATE → FOLLOWING → WAITING_FOR_GOAL_APPROACH → POST_ROTATE
     ├─ In-place rotation at swath turns (differential drive)
     ├─ Oscillation detection (FailureDetector)
     ├─ Obstacle avoidance (costmap checking)
     └─ Updates: state → WAITING_FOR_GOAL_APPROACH → POST_ROTATE → FINISHED
        Returns: RUNNING (in progress), SUCCESS (path complete), FAILURE (stuck)

5. Feedback loop (real-time):
   STM32 (100 Hz):
     ├─ LL_STATUS packet (encoder ticks, IMU, sensors, rain detection)
     └─→ hardware_bridge

   wheel_odometry_node (50 Hz):
     ├─ Integrates left/right encoder ticks
     └─→ /wheel_odom (odometry only, high drift)

   imu_filter_madgwick (50 Hz):
     ├─ Fuses IMU gyro + accel
     └─→ /imu/data (filtered orientation)

   robot_localization dual EKF:
     ├─ ekf_odom_node (50 Hz): wheel + gyro + optional /encoder2/odom
     │     → /odometry/filtered, /tf: odom → base_footprint
     ├─ navsat_transform_node (30 Hz): /gps/fix + local pose → /odometry/gps
     └─ ekf_map_node (30 Hz): wheel + gyro + /gps/pose_cov + /imu/cog_heading
           → /odometry/filtered_map, /tf: map → odom

   The /map OccupancyGrid comes from map_server_node's user-defined area
   polygons, not from SLAM.

   FTCController (10 Hz, coverage) / RPP Controller (10 Hz, transit):
     ├─ Reads robot pose from /tf: odom → base_link
     ├─ FTC: path-indexed PID on longitudinal, lateral, angular axes
     ├─ RPP: regulated pure pursuit with curvature-based speed scaling (transit only)
     └─→ /cmd_vel (geometry_msgs/Twist)

6. Command routing (twist_mux, priority-based):
   /cmd_vel sources:
     ├─ /cmd_vel_emergency (highest priority)
     ├─ /cmd_vel_teleop (manual override)
     └─ /cmd_vel_nav (navigation, from Nav2)
   Route to:
     └─→ /hardware_bridge/cmd_vel

7. Hardware bridge → STM32:
   /cmd_vel (Twist) → LlCmdVel packet (COBS + CRC16) → USB serial

8. STM32 motor control:
   LlCmdVel:
     ├─ linear.x → left/right ESC PWM (duty cycle)
     ├─ angular.z → differential for steering
     └─ Watchdog: expects heartbeat every 250 ms (4 Hz)
        If no heartbeat: safe stop (motor PWM cut)

9. Safety monitoring (BehaviorTree, 10 Hz):
   Condition checks:
     ├─ IsEmergency (latched_emergency bit)
     ├─ NeedsDocking (battery < 20 V)
     ├─ IsLocalizationHealthy (EKF variance < threshold)
     └─ IsCommand (COMMAND_START still active)
   On failure:
     ├─ SetMowerEnabled(false)
     ├─ StopMoving() → /cmd_vel = 0
     └─ PublishHighLevelStatus(RECOVERING or DOCKING)

10. Completion:
    FollowCoveragePath returns SUCCESS:
      ├─ Robot completed coverage path
      ├─ All path indices traversed
      └─ Final orientation aligned

    BehaviorTree continues:
      ├─ SetMowerEnabled(false) → blade off
      ├─ PublishHighLevelStatus(MOWING_COMPLETE)
      ├─ NavigateToPose(dock_pose) → return to dock
      └─ PublishHighLevelStatus(IDLE_DOCKED)

11. Telemetry (Foxglove Bridge, 8765/ws):
    → Web UI receives:
       ├─ /odometry/filtered_map (fused pose + covariance)
       ├─ /map (area polygons as OccupancyGrid from map_server_node)
       ├─ /map_server_node/coverage_cells (mow progress grid)
       ├─ /coverage_path and /coverage_outline (visualization)
       ├─ /scan (LiDAR LaserScan)
       ├─ /fusion_graph/diagnostics (when use_fusion_graph:=true)
       ├─ /diagnostics (system health)
       └─ /tf tree (all frame transformations)
```

---

## TF Tree Reference

**Standard ROS2 conventions (REP-103 + REP-105):**

```
map (GPS-anchored via navsat_transform fixed datum)
  │ [published by ekf_map_node @ 30 Hz]
  │
  odom (continuous, drift-only)
  │ [published by ekf_odom_node @ 50 Hz]
  │
  base_footprint (on ground, robot frame for Nav2/robot_localization)
  │ [base_footprint → base_link static, from URDF]
  │
  ├── base_link (robot body frame, wheel axle height per OpenMower convention)
  │   └── [published by robot_state_publisher from URDF]
  │
  ├── imu_link (fixed to chassis, IMU measurement frame)
  │   └── [hardware_bridge publishes IMU data in this frame;
  │        robot_localization consumes orientation and angular velocity]
  │
  ├── lidar_link (fixed to chassis, LiDAR origin)
  │   └── [LD19 publishes /scan in this frame; collision_monitor + obstacle_layer consume here]
  │
  ├── gps_link (fixed to antenna, GPS measurement point;
  │   navsat_to_absolute_pose_node rotates the antenna→base offset via TF
  │   so /gps/pose_cov lands at the robot origin)
  │
  ├── left_wheel_link (rotating joint, visual only)
  │
  └── right_wheel_link (rotating joint, visual only)
```

**Frame Hierarchy:**

| Frame | Publisher | Rate | Purpose |
|-------|-----------|------|---------|
| map | ekf_map_node | 30 Hz | Global frame, anchored to navsat_transform datum |
| odom | ekf_odom_node | 50 Hz | Continuous local frame (dead-reckoning) |
| base_footprint | ekf_odom_node | 50 Hz | On ground, robot frame for Nav2/controllers (REP-105) |
| base_link | robot_state_publisher | Static | Rear wheel axle (OpenMower convention) |
| imu_link | robot_state_publisher | Static | IMU sensor frame |
| lidar_link | robot_state_publisher | Static | LiDAR origin (collision_monitor + obstacle_layer consume here) |
| gps_link | robot_state_publisher | Static | GPS antenna (navsat_to_absolute_pose rotates the offset to base_footprint) |
| [Gazebo sensor frames] | (simulation only) | Static | Gazebo model sensor origins |

**Frame Conventions:**

- `map` – Global frame, z-up, x-east, y-north (REP-103). Anchored to the navsat_transform datum from `mowgli_robot.yaml`. Published by `ekf_map_node`.
- `odom` – Continuous local frame, drift-only. Published by `ekf_odom_node`.
- `base_footprint` – Ground contact point, robot frame for Nav2. Published by `ekf_odom_node`.
- `base_link` – Robot body frame at rear wheel axle height (OpenMower convention). Static offset from base_footprint.

**Simulation (Gazebo Harmonic):**

Static TF bridges connect Gazebo sensor frame names to URDF frames:
- `mowgli_mower/lidar_link/lidar_sensor` ↔ `lidar_link` (identity)
- `mowgli_mower/base_link/imu_sensor` ↔ `imu_link` (identity)

This lets costmap, collision_monitor, fusion_graph (when enabled), and every other node use standard ROS2 frame names regardless of simulation vs. real hardware.

---

## Topic Map

**Publishers (Sources):**

| Topic | Type | Publisher | Rate | Purpose |
|-------|------|-----------|------|---------|
| `/map` | nav_msgs/OccupancyGrid | map_server_node | on area change | Mowing-area polygons rasterised to an OccupancyGrid (NOT SLAM output) |
| `/map_server_node/coverage_cells` | nav_msgs/OccupancyGrid | map_server_node | 1 Hz | Mow progress grid (cells marked mowed; persists across restarts) |
| `/scan` | sensor_msgs/LaserScan | Gazebo bridge / ldlidar driver | 10 Hz | LiDAR range data (LD19 on real hardware) |
| `/imu/data` | sensor_msgs/Imu | hardware_bridge_node | ~48 Hz | Gyro + accel, bias-corrected on dock (1000-sample calibration) |
| `/status` | mowgli_interfaces/Status | hardware_bridge_node | ~4 Hz | Mower state (blade on/off, rain, charging) |
| `/hardware_bridge/emergency` | mowgli_interfaces/Emergency | hardware_bridge_node | ~4 Hz | Emergency stop status (latched, active) |
| `/hardware_bridge/power` | mowgli_interfaces/Power | hardware_bridge_node | ~4 Hz | Battery voltage, charging current (consumed by SimpleChargingDock) |
| `/wheel_odom` | nav_msgs/Odometry | hardware_bridge_node | ~10 Hz | Integrated wheel velocities (RELIABLE; tight vy covariance enforces non-holonomic constraint) |
| `/gps/fix` | sensor_msgs/NavSatFix | ublox_nav_sat_fix_hp_node | 5 Hz | RTK Fixed when available (σ ~3 mm) |
| `/gps/absolute_pose` | geometry_msgs/PoseWithCovarianceStamped | navsat_to_absolute_pose_node | 5 Hz | RTK position converted to local ENU (diagnostics / BT) |
| `/odometry/filtered` | nav_msgs/Odometry | ekf_odom_node | 50 Hz | Local EKF (odom frame, continuous) |
| `/odometry/filtered_map` | nav_msgs/Odometry | ekf_map_node (default) or fusion_graph_node (opt-in) | 30 Hz / 10 Hz | Global pose (map frame, GPS-corrected; LiDAR-aware when fusion_graph is enabled) |
| `/odometry/gps` | nav_msgs/Odometry | navsat_transform_node | on GPS arrival | NavSatFix converted to map-frame pose |
| `/fusion_graph/diagnostics` | diagnostic_msgs/DiagnosticArray | fusion_graph_node | 1 Hz | Factor-graph health (when use_fusion_graph:=true) |
| `/localization/status` | diagnostic_msgs/DiagnosticStatus | localization_monitor_node | 2 Hz | GPS quality + EKF health |
| `/coverage_server/_action/feedback` and `/coverage_server/_action/result` | opennav_coverage_msgs/ComputeCoveragePath | mowgli_coverage (coverage_server) | Per BT PlanCoverageArea call | F2C v2.0 coverage path (`result.nav_path`) — fed to FollowCoveragePath by FollowStrip BT node |
| `/path` | nav_msgs/Path | planner_server (Nav2) | 1 Hz | Global path from Nav2 planner |
| `/cmd_vel` | geometry_msgs/Twist | twist_mux | 10–50 Hz | Final velocity command (to hardware/Gazebo) |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | diagnostics_node (mowgli_monitoring) | 1 Hz | System health aggregation |
| `/behavior_tree_log` | mowgli_interfaces/BehaviorTreeLog | behavior_tree_node | 10 Hz | Active BT node, node type, execution status (for GUI diagnostics visualization) |
| `/high_level_status` | mowgli_interfaces/HighLevelStatus | behavior_tree_node | 10 Hz | Current high-level mode (IDLE=1, AUTONOMOUS=2, RECORDING=3, MANUAL_MOWING=4) with coverage progress per area |

**Subscribers (Sinks):**

| Topic | Subscriber | Purpose |
|-------|-----------|---------|
| `/cmd_vel` | hardware_bridge_node / Gazebo | Motor/wheel commands |
| `/scan` | fusion_graph_node (when use_fusion_graph + use_scan_matching), collision_monitor, obstacle_layer, diagnostics_node | Scan-matching factors, obstacle detection, monitoring |
| `/imu/data` | ekf_odom_node, ekf_map_node, diagnostics_node | Sensor fusion + monitoring |
| `/odometry/filtered_map` | nav2 (bt_navigator), BT, diagnostics_node, GUI | Localized pose for control, behavior tree, diagnostics |
| `/map` | nav2 planner, behavior_tree (area polygons), diagnostics_node | Global navigation reference (area-polygon grid) |
| `/gps/fix` | navsat_transform_node, navsat_to_absolute_pose_node, diagnostics_node | Sensor fusion, convert fix to local frame, monitor GPS |
| (removed) | — | The legacy `/encoder2/odom` K-ICP twist topic is gone; ekf_odom_node uses only `/wheel_odom`. |
| `/status` | behavior_tree_node, localization_monitor_node, diagnostics_node | Health checks, sensor freshness |
| `/emergency` | behavior_tree_node, diagnostics_node | Emergency monitoring |
| `/power` | behavior_tree_node, diagnostics_node | Battery level monitoring |
| `/wheel_odom` | diagnostics_node | Monitor odometry freshness |
| `/controller_server/FollowCoveragePath/global_plan` | PathProgressGoalChecker | Coverage completion gating (>= 95% path-pose tracking + xy/yaw to goal pose) |

**Services (Request-Response):**

| Service | Type | Server | Client | Purpose |
|---------|------|--------|--------|---------|
| `/mower_control` | MowerControl | hardware_bridge_node | behavior_tree_node | Enable/disable blade motor |
| `/emergency_stop` | EmergencyStop | hardware_bridge_node | behavior_tree_node, ResetEmergency BT node | Assert/release latched emergency |
| `/high_level_control` | HighLevelControl | behavior_tree_node | GUI | Mode commands (START=1, HOME=2, RECORD_AREA=3, S2=4, RECORD_FINISH=5, RECORD_CANCEL=6, MANUAL_MOW=7) |
| `/map_server_node/add_area` | AddMowingArea | map_server_node | RecordArea BT node | Save recorded mowing area polygon |
| `/map_server_node/get_next_unmowed_area` | GetNextUnmowedArea | map_server_node | behavior_tree_node (GetNextUnmowedArea BT node) | Fetch next unmowed area polygon and coverage grid |
| `/map_server_node/get_remaining_area_polygon` | GetRemainingAreaPolygon | map_server_node | behavior_tree_node (PlanCoverageArea BT node) | Returns area outer ring + `mow_progress` holes (Boost.Geometry difference, CCW outer / CW holes) — fed to mowgli_coverage's compute_coverage_path action |
| `/map_server_node/mark_area_complete` | MarkAreaComplete | map_server_node | behavior_tree_node | Mark area as mowing complete, persist coverage state |
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | nav2_behavior_tree_navigator | behavior_tree_node | Send goal to Nav2 |

**Actions (Async Request-Response):**

| Action | Type | Server | Client | Purpose |
|--------|------|--------|--------|---------|
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | nav2_behavior_tree_navigator | behavior_tree_node (NavigateToPose BT node) | Non-blocking navigation goal |
| `~/get_next_strip` | mowgli_interfaces/GetNextStrip | map_server_node (mowgli_map) | behavior_tree_node (GetNextStrip BT node) | On-demand strip planning for cell-based coverage |

---

## Summary: Architectural Principles

1. **14-Package Modular Design:** Separation of concerns across hardware, localization, navigation, planning, monitoring, and behavior layers.
   - **Core:** mowgli_interfaces (message + service definitions, including `GetRemainingAreaPolygon.srv`)
   - **Hardware:** mowgli_hardware (STM32 bridge via COBS)
   - **Perception:** mowgli_localization (EKF fusion helpers — `cog_to_imu_node`, `mag_yaw_publisher`, `navsat_to_absolute_pose_node`, `costmap_scan_filter_node`, `scan_deskew_node`)
   - **Localization (opt-in):** fusion_graph (GTSAM iSAM2 factor graph; replaces `ekf_map_node` when `use_fusion_graph:=true`)
   - **Control:** mowgli_nav2_plugins (FTCController for both FollowPath/FollowCoveragePath, `PathProgressGoalChecker` for coverage completion)
   - **Planning (areas + cells):** mowgli_map (area storage, `mow_progress` grid, `~/get_remaining_area_polygon` for F2C, legacy strip planner via `~/get_next_strip`)
   - **Planning (coverage path):** mowgli_coverage (Fields2Cover v2.0 server, action `compute_coverage_path`)
   - **Behavior:** mowgli_behavior (BehaviorTree.CPP v4, 10 Hz reactive control; F2C-driven coverage via `PlanCoverageArea` + `FollowStrip`)
   - **Monitoring:** mowgli_monitoring (diagnostics, MQTT bridge)
   - **Simulation:** mowgli_simulation (Webots, perfect-IMU mode synthesised from `/cmd_vel`)
   - **Infrastructure:** mowgli_bringup (launch, config), mowgli_description (URDF/xacro)
   - **Third-party:** `opennav_coverage` git submodule for the `_msgs` action definitions only — every other subpackage is `COLCON_IGNORE`'d.

2. **ROS2 Kilted + Gazebo Harmonic:** Modern robotics stack with first-class simulation support and lifecycle management.

3. **Decoupled Communication:** ROS2 pub/sub (topics), services, and actions isolate packages. Easy to substitute, test, or extend components independently.

4. **Robust Serial Protocol (COBS + CRC-16):** Enables reliable bidirectional communication between Raspberry Pi and STM32 firmware over noisy USB at 115200 baud.

5. **Dual-EKF Localization (robot_localization — default localizer):**
   - `ekf_odom_node` (50 Hz, `two_d_mode`) fuses wheel odometry + IMU
     gyro_z → `odom → base_footprint`.
   - `navsat_transform_node` (30 Hz) converts `/gps/fix` + local pose to
     `/odometry/gps` in the map frame, using a fixed datum from
     `mowgli_robot.yaml`.
   - `ekf_map_node` (30 Hz) fuses wheel + IMU + `/gps/pose_cov` +
     `/imu/cog_heading` → `map → odom`. Replaceable 1-for-1 by
     `fusion_graph_node` (GTSAM iSAM2) when `use_fusion_graph:=true`.
   - No SLAM back-end. The map frame is GPS-anchored; saved area polygons
     and dock pose survive restarts.
   - Non-holonomic motion enforced by tight `vy` covariance on `/wheel_odom`.
     Absolute yaw comes from `cog_to_imu.py` (GPS course-over-ground) and
     `mag_yaw_publisher.py` (tilt-compensated magnetometer) since the IMU
     has no magnetometer.
   - Graceful degradation: operates without GPS in GNSS-denied areas
     (wheel + IMU; with `use_fusion_graph:=true` and LiDAR, scan-matching
     between-factors keep the map-frame estimate stable across multi-minute
     RTK-Float windows). `ekf_odom_node` keeps `odom→base_footprint`
     flowing even when the map-frame backend stalls.

6. **Coverage Path Following:** FTCController (Follow-the-Carrot, 3-axis PID + native obstacle deviation) is the active controller for BOTH `FollowPath` (transit) and `FollowCoveragePath` (coverage strips). Single plugin, single tuning. RPP is no longer in the active stack. Coverage completion is gated by `mowgli_nav2_plugins/PathProgressGoalChecker` (>= 95 % path-pose tracking + xy/yaw to goal pose) — `StoppedGoalChecker` and `SimpleGoalChecker` both fired on velocity stoppage during PRE_ROTATE pivots, completing the action at <2 % coverage.

7. **F2C v2 Coverage Planning:** `mowgli_coverage` server (in-tree, action `compute_coverage_path`) wraps Fields2Cover **2.0.0**: `ConstHL.generateHeadlands` insets + `generateHeadlandSwaths` emits concentric perimeter passes (densified 10 cm), `TrapezoidalDecomp` splits inner field around interior holes, then per sub-cell `BruteForce` swath generation → `BoustrophedonOrder` → `PathPlanning(DubinsCurvesCC)` with `planPathForConnection` Dubins arcs between sub-cells. `mowgli_map`'s `~/get_remaining_area_polygon` feeds it the area polygon minus already-mowed cells (Boost.Geometry difference). The legacy `opennav_coverage` upstream is kept as a git submodule for the `_msgs` package only — the server subpackages are `COLCON_IGNORE`'d (pinned to F2C 1.2.1, missing the v2 features above).

8. **Reactive Behavior Trees (BehaviorTree.CPP v4):** 10 Hz non-preemptive tree execution with priority-based fallback selection:
   - Emergency guard: highest priority, interrupts all activity
   - Multi-mode state machine: IDLE → UNDOCKING → MOWING (with recovery) → DOCKING
   - Multi-area coverage: iterates through multiple mowing areas sequentially, each with independent coverage progress tracking
   - Composable async action nodes (NavigateToPose, GetNextUnmowedArea, PlanCoveragePath, FollowCoveragePath)

9. **Priority-Based Command Routing:** twist_mux mediates three command sources (emergency > teleoperation > navigation) before forwarding to hardware bridge.

10. **Comprehensive Health Monitoring:** Diagnostics aggregator tracks 8 subsystems (hardware bridge, emergency, battery, IMU, LiDAR, GPS, odometry, motors) and EKF health (rate and position covariance of `/odometry/filtered_map`) with multi-level status (OK, WARN, ERROR, STALE) for autonomous decision-making. Real-time BT visualization shows active node path and execution state in GUI diagnostics page.

11. **Unified Simulation-to-Hardware Workflow:**
    - Gazebo Harmonic with ros_gz_bridge enables identical ROS2 stack on both sim and real hardware
    - Static TF bridges map Gazebo sensor frames to URDF frames
    - Behavior tree, Nav2, SLAM, and diagnostics unchanged between environments

12. **Foxglove Bridge for Remote UI:** Modern WebSocket-based telemetry (port 8765) replaces legacy rosbridge, reducing latency and CPU overhead.

---

## Development & Testing

**Key Resources:**

| Resource | Location | Purpose |
|----------|----------|---------|
| URDF/Xacro | src/mowgli_bringup/urdf/mowgli.urdf.xacro | Robot kinematics, sensor frames, collision geometry |
| robot_localization Config | src/mowgli_bringup/config/robot_localization.yaml | Dual-EKF per-sensor configs, process noise, navsat_transform datum |
| Nav2 Config | src/mowgli_bringup/config/nav2_params.yaml | Planner, controller, costmap tuning |
| Behavior Tree | src/mowgli_behavior/trees/main_tree.xml | High-level state machine and sequencing |
| Coverage Config | src/mowgli_bringup/config/mowgli_robot.yaml | Coverage parameters (path_spacing, mow_angle_offset_deg, tool_width) |
| Gazebo Worlds | src/mowgli_simulation/worlds/ | garden.sdf (realistic), empty_garden.sdf (testing) |

**Testing Workflow:**

```bash
# Simulation (Gazebo Harmonic, full stack)
ros2 launch mowgli_simulation simulation.launch.py

# Real hardware (Raspberry Pi + STM32)
ros2 launch mowgli_bringup mowgli.launch.py

# SLAM + mapping
ros2 launch mowgli_bringup navigation.launch.py

# Coverage planning (standalone test)
ros2 service call /plan_coverage mowgli_interfaces/srv/PlanCoverage '{outer_boundary: {...}}'

# Diagnostics monitoring
ros2 topic echo /diagnostics
```

---

## Next Steps

- See [CONFIGURATION.md](CONFIGURATION.md) for parameter tuning (PID gains, speed profiles, costmap).
- See [FIRMWARE_MIGRATION.md](FIRMWARE_MIGRATION.md) for STM32 packet protocol and firmware integration.
- See [SIMULATION.md](SIMULATION.md) for detailed Gazebo world setup and physics tuning.
- See [DEVELOPMENT.md](DEVELOPMENT.md) for build instructions and dev container setup.
