# Mowgli ROS2 Architecture

Comprehensive technical documentation of the Mowgli ROS2 system design, including package organization, data flow, communication protocols, and integration points.

Built on **ROS2 Kilted** with **Webots** simulation, this architecture spans 12 focused packages providing complete autonomous lawn mower functionality. [CLAUDE.md](https://github.com/mowglinext/mowglinext/blob/main/CLAUDE.md) is the authoritative short-form reference and the place to check first if any page on the wiki looks out of date.

Machine-generated indexes of the same code live in the repo and are refreshed with it — useful when you need file/line precision rather than prose: [`docs/claude/codemaps/`](https://github.com/mowglinext/mowglinext/tree/main/docs/claude/codemaps) (one per package), [`docs/claude/ros-interfaces.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/ros-interfaces.md) (every topic/service/action/TF and who publishes it), [`docs/claude/parameters.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/parameters.md), [`docs/claude/testing-ci.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/testing-ci.md), and [`docs/claude/doc-index.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/doc-index.md) (which document is authoritative vs historical). Each top-level directory also carries a short `CLAUDE.md` pointer file.

## Localization at a glance

The map-frame and local-odometry legs are both owned by a **single localizer**, `fusion_graph_node` (GTSAM iSAM2 Pose2 factor graph). It is launched **unconditionally** by `navigation.launch.py` — there is **no `use_fusion_graph` arg**, and the previous robot_localization dual EKF (`ekf_map_node` + `ekf_odom_node`) was removed. The area / coverage layers are independent of the localizer.

| Layer | Owner | Notes |
|---|---|---|
| `map → odom` AND `odom → base_footprint` | `fusion_graph_node` (sole, default, unconditional) | GTSAM iSAM2 Pose2 factor graph; publishes **both** TF legs from a dedicated TF broadcast thread. Fuses a wheel between-factor (non-holonomic σ_y ≪ σ_x, from `/wheel_odom`), a gyro between-factor on yaw, a custom `GnssLeverArmFactor` (analytic Jacobian, antenna lever-arm rotates with node yaw, fed by the raw `/gps/fix` NavSatFix — *not* `/gps/pose_cov`, which is the legacy twin the dock-calibration gate uses), and unary yaw factors for COG (`/imu/cog_heading`) and tilt-compensated mag (`/imu/mag_yaw`, when calibrated). Datum + lever-arm from `mowgli_robot.yaml`. Config in `ros2/src/fusion_graph/config/fusion_graph.yaml`. Surface: `/odometry/filtered_map`, `/odometry/filtered`, `/fusion_graph/diagnostics`, `/fusion_graph/markers`, `~/save_graph` / `~/clear_graph` services. No SLAM. |
| LiDAR (optional) | `fusion_graph_node` | When `use_scan_matching` / `use_loop_closure` are on, the scan (`/scan_deskewed` by default, `scan_topic` to override) enters the same factor graph as scan-matching between-factors and loop-closure factors, carrying the map-frame estimate through multi-minute RTK-Float windows. No parallel TF tree, no separate twist channel. |
| Map / areas | `map_server_node` | Polygon-based area DB + `mow_progress` GridMap layer, persisted to disk. No SLAM back-end. |

> `fusion_graph_node` is the **only** node that publishes `map → odom` or `odom → base_footprint`. The robot_localization dual EKF, slam_toolbox, Kinematic-ICP, and FusionCore were all removed. See [Factor-Graph Localizer](#optional-factor-graph-localizer-fusion_graph) below for details.

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
│  │  (Behavior Tree) │  │  plugins         │  │  + fusion_graph      │   │
│  │                  │  │  (FTCController  │  │                      │   │
│  │  10 Hz reactive  │  │   + PathProgress │  │  Multiple nodes:     │   │
│  │  control         │  │   GoalChecker)   │  │  - GPS converter     │   │
│  │                  │  │                  │  │  - COG / mag yaw     │   │
│  │                  │  │  Nav2 local plan │  │  - Health monitor    │   │
│  │                  │  │  10 Hz           │  │  - Factor graph      │   │
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
| **mowgli_localization** | Helper nodes feeding the `fusion_graph` localizer: NavSatFix→pose conversion, COG-to-IMU absolute yaw, magnetometer yaw, IMU-yaw + one-click dock calibration, localization monitor, LiDAR scan deskew, costmap scan filter, GPS dock detection. (`wheel_odometry_node` is still built but not launched — `hardware_bridge` owns `/wheel_odom`.) | mowgli_interfaces |
| **fusion_graph** | Sole map+odom localizer: GTSAM iSAM2 Pose2 factor graph publishing both `map→odom` and `odom→base_footprint` | mowgli_interfaces, GTSAM |
| **mowgli_nav2_plugins** | Exactly two Nav2 plugins: `FTCController` (Follow-the-Carrot coverage controller) and `PathProgressGoalChecker`. RPP, RotationShim and MPPI are **upstream** Nav2 packages, not built here. | nav2_core, nav2_costmap_2d, nav2_util, pluginlib |
| **mowgli_coverage** | Fields2Cover v3 coverage planner: `coverage_server` lifecycle node serving the `plan_coverage` action (headland rings + serpentine swaths + continuous, hole-free `drivable_subpaths`) | mowgli_interfaces, nav2_util, Fields2Cover 3.0.0 (`/opt/fields2cover-300`) |
| **mowgli_map** | `map_server_node`: area polygons + per-area keepouts persisted to `areas.dat` with a WGS84 datum stamp, keepout-mask rasterisation, the `mow_progress` overlay, dock-pose gates; plus `obstacle_tracker_node` (persistent LiDAR obstacle detection) | mowgli_interfaces, nav_msgs, nav2_msgs, grid_map_ros |
| **mowgli_behavior** | Reactive behavior tree control (BehaviorTree.CPP v4) | mowgli_interfaces, nav2_msgs |
| **mowgli_monitoring** | Diagnostics aggregation and MQTT bridge for external monitoring | diagnostic_msgs |
| **mowgli_leds** | Optional WS2812 status ring driven over SPI (`led_ring_node`). Off by default (`led_enabled: false`); not in the blade-safety path. | mowgli_interfaces, rclcpp |
| **mowgli_simulation** | Webots worlds (`worlds_webots/mowgli_garden.wbt`), the `MowgliMower` PROTO, the Webots URDF and the kinematic-drive plugin | mowgli_bringup, webots_ros2_driver |
| **mowgli_bringup** | URDF/xacro robot model, configuration, launch orchestration, and integration layer | All packages above |

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
    ├──→ mowgli_behavior
    │       └──→ mowgli_bringup
    │
    ├──→ mowgli_coverage
    │       └──→ mowgli_bringup
    │
    ├──→ mowgli_monitoring
    │       └──→ mowgli_bringup
    │
    ├──→ mowgli_leds
    │       └──→ mowgli_bringup
    │
    ├──→ mowgli_map
    │       └──→ mowgli_bringup
    │
    └──→ fusion_graph
            └──→ mowgli_bringup

mowgli_nav2_plugins (nav2_core only — no mowgli_interfaces dep)
    └──→ mowgli_bringup

mowgli_simulation (standalone testing)
    ├──→ mowgli_bringup
    └──→ webots_ros2_driver / Webots

mowgli_bringup (integration layer)
    ├──→ launch files
    ├──→ URDF/xacro (mowgli.urdf.xacro)
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
  bool esc_power                  # Legacy field — blade/ESC related, NOT traction power
  uint8 reset_cause               # STM32 boot reset cause (+ reset_cause_name)
  string firmware_version         # "major.minor.patch" from the CONFIG_RSP handshake
  uint8 firmware_protocol_version # Wire protocol version reported by the firmware
  bool firmware_compatible        # False until a matching reply arrives; PreFlightCheck
                                  #   blocks mowing while false (operator must reflash)
  ```
  (Also carries the blade-controller telemetry block: `mower_esc_status`,
  `mower_esc_temperature`, `mower_esc_current`, `mower_motor_temperature`,
  `mower_motor_rpm`, `blade_status_stamp`.)

- **Emergency.msg** – Safety stop status
  ```
  builtin_interfaces/Time stamp
  bool active_emergency           # Any emergency condition active
  bool latched_emergency          # Emergency is latched (requires explicit release)
  bool lift_warning               # Lift detected, blade disabled, not yet emergency
  float32 lift_duration_sec       # Seconds of continuous lift detection
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
  uint8 wheel_direction_fl        # Front-left direction
  uint32 wheel_ticks_fl           # Front-left tick count
  uint8 wheel_direction_fr        # Front-right direction
  uint32 wheel_ticks_fr           # Front-right tick count
  uint8 wheel_direction_rl        # Rear-left direction
  uint32 wheel_ticks_rl           # Rear-left tick count
  uint8 wheel_direction_rr        # Rear-right direction
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
  float32 coverage_percent        # smooth 0..100 for the current area (pose-cursor
                                  #   based; this is the primary GUI percentage)
  float32 gps_quality_percent / battery_percent
  bool is_charging / emergency
  ```
- **ESCStatus.msg** – Motor ESC telemetry
- **ImuRaw.msg** – Raw IMU data from STM32 firmware
- **MapArea.msg** – Mowing area polygon definition for map_server_node
- **CoveragePath.msg** – Coverage path with metadata
- **ObstacleArray.msg** – Collection of tracked obstacles from obstacle_tracker_node
- **TrackedObstacle.msg** – Individual persistent obstacle with position, age, and observation count
- **MapObstacleInfo.msg** – Per-area obstacle identity/state returned by `~/get_mowing_area`
- **DigEvent.msg** – Wheel-slip dig report from `hardware_bridge_node`'s dig detector (`~/dig_event`), promoted to a keepout by `map_server_node`
- **GnssStatus.msg** – Authoritative receiver status from the GNSS sidecar (`/gps/status`)
- **DockCalibrationStatus.msg** – Progress/result feed for the one-click dock calibration

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
    COMMAND_STOP=8              # Stop-in-place hold: mower off, halt, stay put
                               #   (NOT dock — that is COMMAND_HOME). Resumable.
                               #   GUI Pause / Stop use this.
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

- **PlanCoverage.action** – Coverage-plan request served by `mowgli_coverage`'s `coverage_server`. The result carries the ordered headland/swath segments, the continuous hole-free `drivable_subpaths`, and their concatenation `full_path`.
- **CoverageTask.action** – Per-area coverage task (goal `area_index`, feedback `progress_percent`). Defined but currently unused — no server implements it.
- **CalibrateDock.action** – One-click dock calibration (RTK-Fixed gate on `/gps/pose_cov`, then dock pose + yaw capture).

Nav2's own `NavigateToPose` / `FollowPath` actions come from `nav2_msgs` and are **not** redefined here.

**Design Notes:**
- All timestamps use `builtin_interfaces/Time` (ROS2 idiom, replacing `rosgraph_msgs/Time` from ROS1)
- Floating-point values are `float32` (hardware native) except for precise GPS data (`float64`)
- The ROS messages expose plain `bool` fields; the bitmasks (`STATUS_BIT_*`, `EMERGENCY_BIT_*` in `ll_datatypes.hpp`) live on the **wire**, where `hardware_bridge` unpacks them — that is what keeps the firmware packet small

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
  uint8_t type;           // PACKET_ID_LL_CMD_VEL (0x50)
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

**CRC-16/CCITT-FALSE:**
- Polynomial 0x1021, init 0xFFFF, input and output **not** reflected (`crc16.hpp`) — must stay byte-identical to the STM32 implementation
- Calculated over [packet_type][payload_data] only (not the CRC field itself), appended little-endian
- Detects single and double bit errors, all error patterns < 16 bits

**Packet Types (from ll_datatypes.hpp):**

| Type ID | Name | Direction | Purpose |
|---------|------|-----------|---------|
| 0x01 | LL_STATUS | STM32 → Pi | Mower state, charging, rain, sensors, firmware version |
| 0x02 | LL_IMU | STM32 → Pi | Accelerometer + gyroscope + magnetometer data |
| 0x03 | LL_UI_EVENT | STM32 → Pi | Button press, duration |
| 0x04 | LL_ODOMETRY | STM32 → Pi | Wheel ticks / wheel odometry |
| 0x05 | LL_BLADE_STATUS | STM32 → Pi | Blade motor telemetry |
| 0x06 | LL_RESET_CAUSE | STM32 → Pi | Boot reset cause |
| 0x11 / 0x12 | LL_HIGH_LEVEL_CONFIG_REQ / _RSP | Bidirectional | Firmware version + config handshake |
| 0x42 | LL_HEARTBEAT | Pi → STM32 | Keep-alive, emergency control, release |
| 0x43 | LL_HIGH_LEVEL_STATE | Pi → STM32 | Mode, GPS quality, localization health |
| 0x50 | LL_CMD_VEL | Pi → STM32 | Motor velocity commands |
| 0x51 | LL_CMD_BLADE | Pi → STM32 | Blade motor control |
| 0x52 | LL_REBOOT | Pi → STM32 | Reboot the board (guarded by a magic byte) |
| 0x54 / 0x55 | LL_SET_DRIVE_PID / LL_SET_YAW_PID | Pi → STM32 | Runtime tuning of the firmware wheel-velocity and yaw-rate loops |
| 0x56 / 0x57 | LL_SET_KINEMATICS / LL_SET_SAFETY_LIMITS | Pi → STM32 | Runtime max-speed + wheel base; charge ceiling + e-stop timeouts |

The wire protocol is versioned: `kMowgliProtocolVersion` in `ll_datatypes.hpp` must match what the firmware reports through the config handshake, or `PreFlightCheck` blocks mowing.

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
ROS2 network (after the mowgli.launch.py remaps):
  /hardware_bridge/status, /hardware_bridge/emergency, /hardware_bridge/power
  (~/imu/data_raw → /imu/data, ~/dock_heading → /gnss/heading)
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

**Topics Published:**
- `~/status` (Status msg) – one per firmware status packet (~47 Hz)
- `~/emergency` (Emergency msg) – with Status
- `~/power` (Power msg) – with Status
- `~/imu/data_raw` (sensor_msgs/Imu) and `~/imu/mag_raw` (sensor_msgs/MagneticField) – firmware IMU rate (~47 Hz)
- `~/wheel_odom` (nav_msgs/Odometry) – ticks aggregated into ~20 Hz odometry. **This is the live `/wheel_odom` publisher**; `mowgli_localization`'s standalone `wheel_odometry_node` is not launched.
- `~/wheel_ticks` (WheelTick msg) – once per firmware packet, for the GUI encoder panel
- `/battery_state` (sensor_msgs/BatteryState) – absolute name, for `SimpleChargingDock`
- `~/dock_heading` (sensor_msgs/Imu) and `~/dig_event` (DigEvent msg, transient_local)

**Topics Subscribed:**
- `~/cmd_vel` (geometry_msgs/Twist) – the **merged** twist_mux output, so the dig detector on it covers every motion lane

**Services:**
- `~/mower_control`, `~/emergency_stop`, `~/reboot_board` (Trigger), `~/set_firmware_debug` (SetBool)

---

### 3. mowgli_localization

**Purpose:** Multi-source localization pipeline (odometry, GPS fusion, health monitoring).

**Location:** `src/mowgli_localization/`

**Architecture:**

```
Inputs:
  - /wheel_odom  (nav_msgs/Odometry, from hardware_bridge's OdometryPublisher)
  - /imu/data    (sensor_msgs/Imu — hardware_bridge's ~/imu/data_raw, remapped;
                  there is no madgwick filter node in the stack)
  - /gps/fix     (sensor_msgs/NavSatFix, RTK status)
  - /gps/status  (mowgli_interfaces/GnssStatus — authoritative fix state)

↓

Helper nodes (this package):

1) navsat_to_absolute_pose_node
   - /gps/fix → local ENU, lever-arm corrected
   - Publishes /gps/absolute_pose (mowgli_interfaces/AbsolutePose) for the GUI
     and BT, plus the /gps/pose_cov twin used by map_server's dock-pose gate
   - Variable rate (follows the receiver, typically 10-20 Hz)

2) cog_to_imu, mag_yaw_publisher
   - Absolute-yaw observations: /imu/cog_heading (GPS course-over-ground,
     with a stationary anchor) and /imu/mag_yaw (tilt-compensated, optional)

3) localization_monitor_node
   - Watches /wheel_odom and /gps/absolute_pose freshness + RTK flags
   - Publishes /mowgli/localization/mode and /mowgli/localization/mode_id
     (4 modes, debounced)

4) scan_deskew_node, costmap_scan_filter_node, gps_dock_detection_node,
   calibrate_imu_yaw_node
   - LiDAR deskew / costmap scan filtering / GPS-based dock detection /
     IMU-yaw + dock calibration

(wheel_odometry_node is still built but NOT launched — /wheel_ticks has no
 publisher and hardware_bridge already owns /wheel_odom.)

↓

fusion_graph_node (GTSAM iSAM2 Pose2 factor graph) — SOLE/DEFAULT,
launched unconditionally by navigation.launch.py:
   - Fuses: wheel between-factor (non-holo σ_y << σ_x, from /wheel_odom),
             gyro between-factor on yaw (from /imu/data),
             custom GnssLeverArmFactor (analytic Jacobian, lever-arm rotates
                               with node yaw, fed DIRECTLY by /gps/fix —
                               not by /gps/pose_cov),
             yaw unary  /imu/cog_heading  (GPS-COG absolute yaw),
             yaw unary  /imu/mag_yaw      (optional, when calibrated)
   - Optional LiDAR scan-matching between-factors and loop-closure factors
     (gated by use_scan_matching / use_loop_closure).
   - Output: /odometry/filtered_map (map frame) and /odometry/filtered
             (odom frame, dead reckoning — twist.angular.z left at 0)
   - Publishes BOTH TFs from a dedicated TF broadcast thread:
       map → odom               (absorbs GPS corrections)
       odom → base_footprint    (continuous dead-reckoning, never jumps)
   - Adds: /fusion_graph/diagnostics, /fusion_graph/markers
   - Services: ~/save_graph (Trigger), ~/clear_graph (Trigger)
   - Config: ros2/src/fusion_graph/config/fusion_graph.yaml

(The robot_localization dual EKF — ekf_map_node + ekf_odom_node —
 navsat_transform_node, robot_localization.yaml, slam_toolbox,
 Kinematic-ICP, and FusionCore were all removed. fusion_graph_node now
 owns both TF legs.)

No SLAM: the /map OccupancyGrid is generated by map_server_node from
user-defined area polygons, not from scan matching.

↓

Final Output:
  /tf tree: map → odom → base_footprint  (both legs from fusion_graph_node)
  /odometry/filtered_map  (fused pose, map frame)
  /map (area polygons from map_server_node, not SLAM)
```

#### 3a. wheel_odometry_node

> **Not launched.** `hardware_bridge_node`'s `OdometryPublisher` integrates the same
> ticks internally and publishes `/wheel_odom` directly (~20 Hz), so this standalone
> node was dropped from `full_system.launch.py` — it subscribes to `/wheel_ticks`,
> which no longer has a publisher on that path. The source is kept in the package and
> the kinematics below still describe what the bridge does.

**Inputs:**
- `/wheel_ticks` (mowgli_interfaces/WheelTick)

**Outputs:**
- `/wheel_odom` (nav_msgs/Odometry)

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
- Tight `vy` covariance enforces the non-holonomic constraint when the wheel between-factor is built
- Pose covariance intentionally large to prevent odometry from dominating the factor graph
- The factor graph applies heavier corrections from GPS and the gyro/COG/mag yaw factors

**Parameters (`mowgli_localization/config/wheel_odometry.yaml`):**
```yaml
wheel_odometry:
  ros__parameters:
    wheel_distance: 0.325     # Left-to-right drive wheel centre distance (m)
    ticks_per_meter: 399.0    # Encoder resolution (calibrated per robot)
    publish_tf: false         # fusion_graph_node owns odom → base_footprint
```
On the live robot the equivalent values reach `hardware_bridge_node` (and the
STM32, via `LL_SET_KINEMATICS`) from `mowgli_robot.yaml` — `ticks_per_meter` and
`wheel_track` — which is the file calibration writes back to.

#### 3b. navsat_to_absolute_pose_node

**Inputs:**
- `/gps/fix` (sensor_msgs/NavSatFix) and `/gps/status` (mowgli_interfaces/GnssStatus, authoritative fix state)

**Outputs:**
- `/gps/absolute_pose` (mowgli_interfaces/AbsolutePose)
- `/gps/pose_cov` (geometry_msgs/PoseWithCovarianceStamped)

**Purpose:** Convert NavSatFix (latitude/longitude) to a local ENU pose for GUI visualization and Behavior Tree reference, and publish the `PoseWithCovarianceStamped` twin `/gps/pose_cov`, which `map_server_node` uses as the RTK gate for `~/set_docking_point` and the one-click dock calibration. **`fusion_graph_node` does not consume either of these** — its `GnssLeverArmFactor` reads the raw `/gps/fix` and applies its own lever-arm correction.

**Algorithm: GNSS to Local ENU**

```
1. GPS origin (datum) set from mowgli_robot.yaml (datum_lat, datum_lon)
2. Equirectangular projection about the datum
   (mowgli_interfaces/wgs84_projection.hpp, METERS_PER_DEG = 6378137.0 · π/180
    — the SAME math map_server uses, so areas.dat and the localizer agree):

   e = (lon - datum_lon) * cos(datum_lat) * METERS_PER_DEG
   n = (lat - datum_lat) * METERS_PER_DEG
   u = altitude

3. Covariance for /gps/pose_cov comes from the receiver's own reported
   accuracy (σ² on the diagonal), not from a per-fix-type multiplier:
   σ > pos_accuracy_reject_threshold_m (0.50 m)      → drop the sample
   σ > pos_accuracy_inflation_threshold_m (0.025 m)  → var ×= factor² (10²)
   Lever-arm yaw uncertainty (lever_arm_yaw_sigma, default 0.0524 rad ≈ 3°)
   is propagated through the antenna-offset Jacobian as a rank-1 inflation.
   STATUS_NO_FIX                                     → skip publishing
```

**Parameters (injected at launch by `full_system.launch.py`; no dedicated yaml):**
```yaml
navsat_to_absolute_pose:
  ros__parameters:
    datum_lat: 0.0                          # from mowgli_robot.yaml
    datum_lon: 0.0                          # from mowgli_robot.yaml
    lever_arm_yaw_sigma: 0.0524             # rad, ≈3°
    pos_accuracy_inflation_threshold_m: 0.025
    pos_accuracy_inflation_factor: 10.0
    pos_accuracy_reject_threshold_m: 0.500
```

#### 3c. localization_monitor_node

**Inputs:**
- `/wheel_odom` (nav_msgs/Odometry — wheel-odometry liveness)
- `/gps/absolute_pose` (mowgli_interfaces/AbsolutePose — freshness + RTK flags)

**Outputs:**
- `/mowgli/localization/mode` (std_msgs/String, latched, for debug/logging)
- `/mowgli/localization/mode_id` (std_msgs/Int32, latched, numeric mode)

Both are published at `publish_rate` (default 10 Hz) with `transient_local`
durability, so a late subscriber always gets the current mode. This node is a
**source-health classifier**, not a filter monitor — it does not read the
localizer's covariance and publishes no `DiagnosticStatus`
(`mowgli_monitoring`'s `diagnostics_node` owns `/diagnostics`).

**Mode debounce (`mode_debounce_sec`, default 1.0 s):** the published mode is
hysteretic — a changed mode must persist continuously for the debounce window
before it is committed and published. This filters the per-epoch RTK
Fixed↔Float flicker (an F9P's `carrSoln` can toggle every epoch under motion
while position σ stays sub-cm), so the mode — and anything gating on it — does
not flap. Set `mode_debounce_sec: 0` to commit immediately.

**Localization modes (`mode_id`, best to worst):**

| ID | Name | Condition |
|----|------|-----------|
| 3 | RTK_FIXED | `/gps/absolute_pose` fresh with `FLAG_GPS_RTK_FIXED` — ~2 cm absolute |
| 2 | RTK_FLOAT | Fresh with RTK / RTK-Float flags — ~5–20 cm |
| 1 | GPS_ONLY | Fresh fix, no RTK augmentation |
| 0 | DEAD_RECKONING | GPS or wheel odometry stale / all sources degraded — return to dock |

**Parameters (defaults; no dedicated yaml, declared in the node):**
```yaml
localization_monitor:
  ros__parameters:
    gps_timeout: 2.0          # s — declare GPS stale after this gap
    pose_timeout: 0.5         # s — declare wheel odom stale
    publish_rate: 10.0        # Hz
    mode_debounce_sec: 1.0    # hysteresis on published mode (0 = off)
```

#### 3d. Localizer Diagnostics Integration

**Monitoring:**
`mowgli_monitoring`'s `diagnostics_node` subscribes to `/odometry/filtered_map` and checks:
- **Freshness:** message age against `freshness_warn_sec` (5 s) / `freshness_error_sec` (10 s). The measured `rate_hz` is *reported* as a key/value but is not itself thresholded.
- **Flat constraint:** |roll| and |pitch| < 5° — a ground robot that reports otherwise has a bad orientation source.
- **Z drift:** |z| < 2 m.

It does **not** read the pose covariance, and it does **not** cross-check against
`/gps/absolute_pose` — for covariance use `/fusion_graph/diagnostics`
(`cov_xx` / `cov_yy` / `cov_yawyaw`), and for fix quality use
`/mowgli/localization/mode`.

**Diagnostics Output:**
Aggregated into `/diagnostics` as a sub-status (still named `EKF Map`, a legacy
label from the removed dual EKF) with levels:
- **OK:** Fresh, flat, no z-drift — message reports the observed rate
- **WARN:** Roll/pitch drift, z-drift, or age past `freshness_warn_sec`
- **ERROR:** Nothing received, or age past `freshness_error_sec`

---

### Factor-Graph Localizer (fusion_graph) {#optional-factor-graph-localizer-fusion_graph}

**Package:** `ros2/src/fusion_graph/` (separate ament_cmake package).

**Purpose:** the **sole, default, unconditional** map+odom localizer, built on **GTSAM iSAM2**. It publishes **both** `map → odom` AND `odom → base_footprint` (from a dedicated TF broadcast thread), plus `/odometry/filtered_map`. The map-frame estimate is the result of an incremental Pose2 factor graph, which lets LiDAR scan-matching and loop-closure factors carry the position through multi-minute RTK-Float windows where dead-reckoning alone would drift.

**Activation:** none — `fusion_graph_node` is launched **unconditionally** by `navigation.launch.py`. There is **no `use_fusion_graph` arg**. The robot_localization dual EKF (`ekf_map_node` + `ekf_odom_node`), `robot_localization.yaml`, slam_toolbox, Kinematic-ICP, and FusionCore were all removed; `fusion_graph_node` is the only node that publishes either TF leg. Config lives in `ros2/src/fusion_graph/config/fusion_graph.yaml`.

**Graph structure (per node, `node_period_s` cadence):**

| Factor | Source | Notes |
|---|---|---|
| Wheel `BetweenFactor` | `/wheel_odom` | Body-frame Pose2; non-holonomic σ_y ≪ σ_x. |
| Gyro `BetweenFactor` (yaw only) | `/imu/data` | Integrated `wz` over the inter-node window. |
| Custom `GnssLeverArmFactor` | `/gps/fix` | Analytic Jacobian — antenna lever-arm rotates with the node's yaw, so GPS XY couples to heading correctly. Robust Huber when fix < `STATUS_GBAS_FIX`. |
| Yaw unary | `/imu/cog_heading` | GPS course-over-ground absolute yaw, gated on forward motion. |
| Yaw unary (optional) | `/imu/mag_yaw` | Tilt-compensated mag yaw, only when `use_magnetometer:=true`. |
| Scan `BetweenFactor` (optional) | `/scan_deskewed` (`scan_topic`) | ICP between consecutive nodes; gated on `use_scan_matching:=true`. Both LiDAR toggles are ANDed with `use_lidar` at launch. |
| Loop-closure `BetweenFactor` (optional) | `/scan_deskewed` + scan storage | Candidate search around new nodes; gated on `use_loop_closure:=true` **and** a persisted graph file existing on disk (the first session cannot loop-close against itself). |

**Public surface:**

- **Topics**:
  - `/fusion_graph/diagnostics` (`diagnostic_msgs/DiagnosticArray`, 1 Hz) — `total_nodes`, `scans_attached`, `loop_closures`, `scans_received`, `scan_matches_ok`, `scan_matches_fail`, `cov_xx`, `cov_yy`, `cov_yawyaw`, plus keyframe (`keyframes_total`, `kf_matches_ok|fail`), gate (`gps_rejects_wrongfix`, `slip_veto`, `stationary_hand_push`) and ICP-reject counters. Surfaced in the GUI's *Diagnostics → Fusion Graph (iSAM2)* panel.
  - `/fusion_graph/markers` (`visualization_msgs/MarkerArray`, 1 Hz, transient_local) — node positions, trajectory, loop-closure edges (decimated to ≤1500 nodes).
  - `/imu/fg_yaw` (`sensor_msgs/Imu`, 10 Hz) — yaw-only output exposed for downstream consumers / diagnostics.
- **Services** (both `std_srvs/Trigger`):
  - `~/save_graph` — persists graph (XML), per-node scans (binary), and metadata (datum + indices) to `/ros2_ws/maps/fusion_graph.{graph,scans,meta}`. Auto-fires on RECORDING→IDLE, on dock arrival, and every `periodic_save_period_s` (default 5 min) during AUTONOMOUS state.
  - `~/clear_graph` — wipes iSAM2 + scans + queues. The next valid pose seed (GPS, set_pose, or scan-match relocalization) re-initializes.
- **Parameter knobs** worth knowing: `node_period_s` (the yaml carries 0.02 = 50 Hz, but `navigation.launch.py` overrides it from `mowgli_robot.yaml` — 0.04 = 25 Hz is the deployed hardware cadence), `wheel_sigma_*`, `gyro_sigma_theta`, `gps_sigma_floor`, `cov_update_every_n`, `isam2_relinearize_skip`, `isam2_rebase_every_nodes`, `scan_retention_nodes`, plus the LC / ICP block (`lc_max_dist_m`, `lc_min_age_s`, `icp_max_iter`, …). All declared at startup; no dynamic_reconfigure.

**State persistence:**
- On disk: `<graph_save_prefix>.graph` (gtsam serialised Values), `.scans` (per-node Eigen `Vector2d` blobs), `.meta` (next index, last node time, datum lat/lon).
- On startup: if `autoload_graph:=true` and the files exist, the node resumes from the last-saved state; subsequent ticks add nodes after the highest loaded index. This is what makes "park the robot, restart ROS2, resume mowing" work without losing the graph.

**Why it exists:** a plain EKF treats GPS as an unbiased XY observation and times out cleanly during dropouts, but has no mechanism to use LiDAR for an absolute pose lock — which is why the previous robot_localization dual EKF was replaced. The factor graph adds scan between-factors and loop-closure factors on top of the same GPS / wheel / IMU fusion, at the cost of a richer node and a non-trivial CPU budget (~5 ms/tick on a Pi 4 with scan matching enabled).

---

### 4. mowgli_bringup

**Purpose:** Configuration, URDF, and launch orchestration for the entire stack.

**Location:** `src/mowgli_bringup/`

#### URDF: mowgli.urdf.xacro

**Robot Description:**

```
base_footprint (on ground, fixed to base_link)
    │
    ├── base_link (centre of the REAR drive-wheel axis, one wheel radius up)
    │   │
    │   ├── left_wheel_link
    │   │   └── left_wheel_joint (continuous, axis Y)
    │   │
    │   ├── right_wheel_link
    │   │   └── right_wheel_joint (continuous, axis Y)
    │   │
    │   ├── front_left_caster_link / front_right_caster_link
    │   │   └── front_{left,right}_caster_joint (continuous, axis Y)
    │   │
    │   ├── blade_link
    │   │   └── blade_joint (FIXED — the blade disc is not articulated)
    │   │
    │   ├── imu_link (fixed to chassis)
    │   │   └── imu_joint (fixed)
    │   │
    │   ├── gps_link (fixed to chassis top)
    │   │   └── gps_joint (fixed)
    │   │
    │   └── lidar_link (LiDAR mount, typically on top)
    │       └── lidar_joint (fixed)
```

**Key Dimensions.** Every shape number is a xacro *argument*; `mowgli.launch.py`
reads the real value from `mowgli_robot.yaml` (template merged under the sparse
installed file) and passes it on the `xacro` command line, so the yaml — not the
`.xacro` default — is what the running robot uses. Current template values
(measured on a YardForce 500):

- **Chassis:** 0.60 m long × 0.40 m wide × 0.19 m tall, geometric centre 0.18 m ahead of `base_link` (rear bumper −0.12 m, front bumper +0.48 m). Nav2's footprint is derived from these.
- **Drive wheels (rear):** 0.04475 m radius, 0.04 m width, 0.325 m track (centre-to-centre). `wheel_track` is also pushed to the STM32 via `LL_SET_KINEMATICS` and must match the firmware's assumption.
- **Casters:** front pair, 0.03 m radius, 0.36 m track
- **Ground clearance:** `base_link` sits exactly one wheel radius above ground
- **Blade:** 0.09 m radius disc, 0.01 m height (under base_link); `tool_width` = 2 × `blade_radius` = 0.18 m
- **Mass distribution:** chassis 8.76 kg · each drive wheel 0.5 kg · each caster 0.2 kg · blade 0.3 kg
- **Sensor placement** (`lidar_x/y/z/yaw`, `imu_*`, `gps_x/y/z`) also comes from `mowgli_robot.yaml` — the GPS values are the antenna lever arm the localizer corrects for.

**Transform Tree (TF):**

```
Map frame (GPS-anchored via fixed datum)
    │
    ├── [fusion_graph_node publishes map→odom]
    │
Odometry frame (continuous, drift-only)
    │
    ├── [fusion_graph_node publishes odom→base_footprint]
    │
Base footprint (on ground, robot frame for Nav2)
    │
    ├── [robot_state_publisher outputs static TFs]
    │
Base link (rear wheel axle, OpenMower convention)
    │
Sensor frames:
    ├── imu_link (IMU data frame)
    ├── lidar_link (LiDAR data frame)
    ├── gps_link (GPS antenna location)
    └── [wheel + caster links for visualization]
```

#### Launch Files

**full_system.launch.py** – the real-robot entry point

Includes `mowgli.launch.py` and `navigation.launch.py`, then adds the
behavior tree node, `map_server_node`, `navsat_to_absolute_pose_node`,
`localization_monitor_node`, `diagnostics_node`, the optional MQTT bridge,
`foxglove_bridge` and the optional `led_ring_node`.

**mowgli.launch.py** – base hardware layer only

Starts exactly three things:
1. `robot_state_publisher` – Runs `xacro` on `mowgli.urdf.xacro` with the shape/sensor values from `mowgli_robot.yaml`, publishes `/robot_description` and the static TFs
2. `hardware_bridge_node` – Serial bridge to STM32 (also the `/wheel_odom` publisher and the wheel-slip dig detector)
3. `twist_mux` – Priority-based cmd_vel multiplexer, output remapped to `/cmd_vel`

**webots_minimal.launch.py** (`mowgli_simulation`) – Webots

Starts the Webots supervisor on `worlds_webots/mowgli_garden.wbt` with the
`MowgliMower` PROTO, `webots_ros2_driver` (loading the `kinematic_drive`
plugin from `urdf_webots/mowgli_webots.urdf`), `robot_state_publisher` and
the diff-drive / joint-state controllers. For the whole stack in simulation
use `mowgli_bringup sim_full_system.launch.py` instead.

**navigation.launch.py** – fusion_graph localizer + Nav2 stack (included by `full_system`)

Starts (in order):
1. `fusion_graph_node` (GTSAM iSAM2 factor graph) — launched
   **unconditionally** (there is no `use_fusion_graph` arg). Publishes
   **both** `map→odom` AND `odom→base_footprint`. Reads datum + lever-arm
   from `mowgli_robot.yaml`; config in `fusion_graph.yaml`. The
   `dock_yaw_to_set_pose` node that used to sit here was **inlined into
   `fusion_graph_node`** and no longer exists.
2. `cog_to_imu`, `mag_yaw_publisher`, `scan_deskew_node`,
   `costmap_scan_filter_node`, `gps_dock_detection_node` (mowgli_localization
   helpers feeding the graph). `navsat_to_absolute_pose_node` is launched one
   level up, by `full_system.launch.py`.
3. `nav2_navigation_launch.py` — all Nav2 servers plus `coverage_server`
   (controller, smoother, planner, behaviors, collision_monitor, bt_navigator,
   waypoint_follower, docking_server, coverage_server, lifecycle manager),
   held back until `map→odom` exists. Deep-merges `nav2_params_base.yaml` with
   `nav2_params_lidar.yaml` when `use_lidar=true`, `nav2_params_no_lidar.yaml`
   otherwise, and injects the operator knobs from `mowgli_robot.yaml`
   (`mowing_speed`, `operation_width = tool_width − swath_overlap`,
   `progress_timeout_sec`, obstacle-avoidance distances, …).

#### Configuration Files

**hardware_bridge.yaml** – Serial communication, IMU cal sample count.
**fusion_graph.yaml** (`ros2/src/fusion_graph/config/`) – Factor-graph tuning: node cadence, wheel/gyro/GPS sigmas, lever-arm, COG/mag gates, iSAM2 knobs, LiDAR scan-matching / loop-closure block.
**nav2_params_base.yaml** – Shared Nav2 base (costmaps, planner, controller) deep-merged with one overlay.
**nav2_params_lidar.yaml** / **nav2_params_no_lidar.yaml** – Thin overlays for the LiDAR vs GPS-only variants (obstacle vs static layers, scan-based vs pass-through collision_monitor).
**twist_mux.yaml** – Velocity command multiplexing (nav < teleop < emergency).
**mowgli_robot.yaml** – Per-robot config. **Sparse model:** the *installed* file (`/ros2_ws/config/mowgli_robot.yaml`, from `install/config/mowgli/`) holds only install-time choices (datum, NTRIP, `lidar_enabled`, GNSS hardware, `mower_model`) + calibration outputs (dock pose, `ticks_per_meter`, `wheel_pid_*`, `imu_yaw`, mag). Every other default lives in the in-package template `mowgli_bringup/config/mowgli_robot.yaml`; at launch `mowgli_bringup/launch/robot_config_util.py` (`load_robot_params`) **deep-merges the installed sparse file over the template**, so absent keys fall through to template defaults. This is what lets the GUI "reset to default" by simply deleting a key, and lets a maintainer change a template default for all robots that never overrode it. Older *full* installed configs still merge to themselves (no-op).

---

### 5. mowgli_nav2_plugins

**Purpose:** The two Nav2 plugins Mowgli builds itself — the coverage controller and the coverage goal checker. Everything else in the controller stack (RPP, RotationShim, the progress checkers) is upstream Nav2.

**Location:** `src/mowgli_nav2_plugins/`

**Plugin Registration:** `ftc_controller_plugin.xml` and `goal_checker_plugin.xml`, both exported from the single `mowgli_nav2_plugins` shared library.

```xml
<library path="mowgli_nav2_plugins">
  <class
    name="mowgli_nav2_plugins/FTCController"
    type="mowgli_nav2_plugins::FTCController"
    base_class_type="nav2_core::Controller">
    <description>
      Follow-the-Carrot (FTC) controller with decoupled PID for outdoor mowing robots.
      Ported from ftc_local_planner (ROS1 / move_base_flex) to Nav2.
      State machine: PRE_ROTATE -> FOLLOWING -> WAITING_FOR_GOAL_APPROACH -> POST_ROTATE -> FINISHED.
    </description>
  </class>
</library>
```

The second file registers `mowgli_nav2_plugins/PathProgressGoalChecker`
(`nav2_core::GoalChecker`) — the coverage goal checker that fires only once the
controller has monotonically tracked ≥ `progress_threshold` (0.95) of the
republished plan *and* is within xy/yaw tolerance. `StoppedGoalChecker` must not
be used for coverage: it fires on velocity stoppage, which matches FTC's
in-place `PRE_ROTATE` pivots and completes the action at < 2 % coverage.

**Controller Profiles (Active Configuration):**

Two controller slots are configured in `nav2_params_base.yaml`:
1. **FollowPath** – Transit navigation and docking: `nav2_rotation_shim_controller::RotationShimController` wrapping `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` (RPP). One start pivot, then pure pursuit; `rotate_to_heading_once: true`, `use_rotate_to_heading: false`, `rotate_to_goal_heading: true`.
2. **FollowCoveragePath** – Coverage path following: `mowgli_nav2_plugins/FTCController`, **standalone** (no RotationShim in front of it — FTC does its own `PRE_ROTATE`), fed the WHOLE continuous sub-path as one goal rather than segment-by-segment.

> **History:** the coverage slot was switched FTC → MPPI (`nav2_mppi_controller`)
> in an experiment and **reverted to FTCController on 2026-06-19**. MPPI is a
> sampling controller: it cut corners and made omega-loops at swath U-turns, and
> every attempt to sharpen its corners made it weave on the straights. Do not
> switch coverage back to MPPI without re-reading the FTC block and tuning notes
> in `nav2_params_base.yaml`.

#### FTCController: 5-State FSM & Path-Indexed Algorithm

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
- Global path (array of PoseStamped with positions and orientations) — for coverage this is the WHOLE continuous sub-path, fed as one goal
- Robot pose from `costmap_ros_->getRobotPose()`, i.e. `base_footprint` in the local costmap's global frame (`odom`)
- Local costmap (for obstacle checking and the lateral-deviation clearance search)

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
   - Speed profile is **curvature**-driven, not distance-to-goal driven: `speed_fast` on straights, `speed_slow` wherever the path bends (every swath-end turn-around arc and headland corner fillet), selected by `speed_fast_threshold` / `speed_fast_threshold_angle`. Both are **overridden at launch** from `mowgli_robot.yaml`: `speed_fast = mowing_speed`, `speed_slow = clamp(mowing_speed × turn_speed_ratio, min_speed_mps, mowing_speed)` — editing the literals in the yaml does not change the running robot.
   - Traction control: if the actual `/wheel_odom` speed stays below `stall_speed_ratio ×` the command for `stall_grace_s`, the carrot eases to `stall_crawl_speed` instead of flooring it (flooring is what digs holes in soft turf).

5. **Obstacle handling** (`enable_obstacle_deviation`, LiDAR variant only):
   - Reads the local `obstacle_layer`, and skirts obstacles by deviating the path **laterally** in real time, up to `max_lateral_deviation`
   - `require_clear_exit` refuses a sideways skirt when the obstacle's far edge is not visible inside the lookahead window (a wall/pocket) — that is how the robot used to box itself in
   - When wedged: hold for `obstacle_wait_timeout_s`, optionally reverse straight back a bounded distance (`obstacle_reverse_*`, rear footprint checked against true-lethal cells every tick), then abort the strip so the coverage layer can route a blade-off transit around it
   - In the no-LiDAR overlay this is off; `collision_monitor` and the firmware are the guard

**State Transitions:**

- **PRE_ROTATE → FOLLOWING:** Robot roughly aligned with path start
- **FOLLOWING → WAITING_FOR_GOAL_APPROACH:** Current_index approaches path end (robot < max_follow_distance from goal)
- **WAITING_FOR_GOAL_APPROACH → POST_ROTATE:** Robot within xy_goal_tolerance, ready to orient to final pose
- **POST_ROTATE → FINISHED:** Robot within yaw_goal_tolerance (orientation correct)
- Any state: Oscillation detected → recover, retry

**Oscillation Detection & Recovery:**

The `FailureDetector` class keeps a rolling window of *normalised* (v, ω) pairs.
Once the buffer is at least half full it flags oscillation when the mean
normalised linear velocity is below `oscillation_v_eps`, the mean normalised
angular velocity is below `oscillation_omega_eps`, **and** the angular velocity
has crossed zero more than once — the zero-crossing test is what distinguishes
"stuck oscillating" from "legitimately slow". Recovery holds position, then
retries the path.

**Parameters (`FollowCoveragePath` block, `nav2_params_base.yaml` — live values):**

```yaml
FollowPath:                                 # transit / dock approach
  plugin: "nav2_rotation_shim_controller::RotationShimController"
  primary_controller: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"

FollowCoveragePath:                         # coverage
  plugin: "mowgli_nav2_plugins/FTCController"

  # Speed profiles — BOTH overridden at launch from mowgli_robot.yaml
  speed_fast: 0.20                          # m/s on straights (= mowing_speed)
  speed_slow: 0.16                          # m/s wherever the path bends
  speed_fast_threshold: 0.5                 # straight-ahead distance (m) for "fast"
  speed_fast_threshold_angle: 10.0          # deg of upcoming bend for "fast"
  speed_angular: 45.0
  acceleration: 0.2
  min_speed_mps: 0.15

  # Anti-wheelspin (traction control)
  stall_speed_ratio: 0.35
  stall_grace_s: 0.6
  stall_crawl_speed: 0.08

  # Decoupled PID gains (field-tuned 2026-06-19 / 2026-06-25)
  kp_lon: 1.0
  ki_lon: 0.0
  kd_lon: 0.0
  kp_lat: 0.8
  ki_lat: 0.0
  kd_lat: 0.5                               # 1.5 pumped a ~0.5 Hz cross-track weave
  kp_ang: 1.5                               # PRE_ROTATE pivot — must clear the deadband
  kp_ang_following: 1.0                     # straight-swath heading gain
  derivative_filter_tau: 0.2                # one-pole LPF on the D term (fc ≈ 0.8 Hz)

  # Robot limits
  max_cmd_vel_speed: 0.30                   # m/s (clamping saturation)
  max_cmd_vel_ang: 0.8                      # rad/s
  max_goal_distance_error: 0.50             # m (triggers failure if exceeded)
  max_goal_angle_error: 30.0                # degrees
  goal_timeout: 10.0                        # seconds before goal declared unreachable
  max_follow_distance: 2.0                  # m (distance at which path end is "reached")

  # Options
  forward_only: true                        # never drive a path segment in reverse;
                                            #   obstacles are skirted forward
  # Recovery (defaults declared in ftc_controller.cpp)
  oscillation_recovery: true
  oscillation_v_eps: 0.05                   # NORMALISED mean |v| (fraction of v_max)
  oscillation_omega_eps: 0.05               # NORMALISED mean |omega|
  oscillation_recovery_min_duration: 5.0    # seconds

  # Obstacle checking / lateral deviation (LiDAR variant)
  check_obstacles: true
  obstacle_lookahead: 30                    # path poses ahead; overridden at launch from
                                            #   obstacle_detection_range_m
  obstacle_footprint: true
  use_footprint_clearance: false            # field: full-footprint sweep too conservative
  require_clear_exit: true                  # cul-de-sac guard — do not skirt a wall
  ignore_obstacles_outside_zone: true
  enable_obstacle_deviation: true
  max_lateral_deviation: 1.5                # overridden from max_obstacle_avoidance_distance
  obstacle_wait_timeout_s: 2.5
  obstacle_clear_hold_s: 1.5
  obstacle_reverse_enabled: true
  obstacle_reverse_max_dist_m: 0.30
  obstacle_reverse_speed_mps: 0.10
```

`test_nav2_params.py` validates the merged base+overlay result and pins the two
variants in lockstep, so a change here that breaks the LiDAR / no-LiDAR pairing
fails in CI.

#### FailureDetector (oscillation_detector.hpp)

Rolling-window failure detection:

```cpp
class FailureDetector {
public:
  void setBufferLength(int length);
  void update(double v, double omega,
              double v_max, double v_backwards_max, double omega_max,
              double v_eps, double omega_eps);
  bool isOscillating() const noexcept;      // last computed decision

private:
  std::deque</* normalised (v, omega) samples */> buffer_;
};
```

`update()` normalises each sample by the velocity limits, pushes it into the
window and recomputes the decision immediately (so `isOscillating()` is a cheap
accessor). Once the window is at least half full it flags oscillation when the
mean normalised |v| and |ω| are both below their eps **and** ω has crossed zero
more than once.

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
    └── BehaviorTree instance (XML: trees/main_tree.xml, ID "MowgliMain")
        │
        └── ReactiveSequence: Root   (every guard handler ends in AlwaysFailure)
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
            ├── ReactiveFallback: SensorSafetyGuard
            │   ├── exempt while charging/docking, or under cmd 7/3/5/6
            │   ├── Inverter(IsScanStale OR IsCollisionStopSustained)
            │   └── SensorFaultHandler: blade off, StopMoving(), wait
            │
            ├── ReactiveFallback: BoundaryGuard  (two-tier)
            │   ├── LethalBoundaryHandler (IsLethalBoundaryViolation)
            │   │   → blade off, stop, BOUNDARY_EMERGENCY_STOP, wait
            │   └── SoftBoundaryHandler (IsBoundaryViolation)
            │       → NavigateInsideBoundary (×2), else escalate to lethal
            │
            ├── ReactiveFallback: LocalizationGuard
            │   ├── Inverter(IsLocalizationDegraded) — keyed on GNSS health
            │   └── hold in place, WAITING_FOR_RTK, until GNSS recovers
            │
            ├── Fallback: GPSModeSelector
            │   ├── IsGPSFixed → SetNavMode(precise)
            │   └── SetNavMode(degraded)
            │
            ├── Fallback: Nav2ResumeGuard
            │   └── SetNav2Lifecycle(RESUME) unless idle-on-dock
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
                │   │       │   │   │   — calls map_server/get_mowing_area
                │   │       │   │   │     (outer ring + obstacle holes), then
                │   │       │   │   │     mowgli_coverage plan_coverage action
                │   │       │   │   │     (F2C v3: ConstHL rings → BruteForce
                │   │       │   │   │      swaths → BoustrophedonOrder →
                │   │       │   │   │      Mowgli forward turn-around arcs
                │   │       │   │   │      connectors). ONE plan per area per session.
                │   │       │   │   └── RetryUntilSuccessful(num_attempts=5):
                │   │       │   │       Fallback MowOrRecover:
                │   │       │   │         ├── FollowStrip — drives each
                │   │       │   │         │   drivable_subpath as ONE
                │   │       │   │         │   FollowCoveragePath goal (FTCController)
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
5. **Mowing (COMMAND_START=1):** Multi-area coverage: iterates through all unmowed areas via GetNextUnmowedArea, then plans each area once with PlanCoverageArea and drives its continuous sub-paths with TransitToStrip + FollowStrip, with GPS wait, heading calibration from undock, rain/battery reactive guards, one plan per area then sub-path execution via TransitToStrip/FollowStrip. Coverage progress tracked per area.
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

// Coverage nodes (the plan comes from PlanCoverageArea, above):
class TransitToStrip : public BT::AsyncActionNode
// Navigates to start of current strip using Nav2 (RPP controller)
// Returns RUNNING/SUCCESS/FAILURE

class FollowStrip : public BT::AsyncActionNode
// Drives each continuous drivable_subpath as one Nav2 FollowCoveragePath goal (FTCController)
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
- `~/get_mowing_area` (mowgli_map/map_server_node) – Fetch an area's outer ring + obstacle holes for planning

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
  factory.registerNodeType<FollowStrip>("FollowStrip");
  factory.registerNodeType<TransitToStrip>("TransitToStrip");

  // Area recording node
  factory.registerNodeType<RecordArea>("RecordArea");
}
```

---

### 7. Coverage Planning (mowgli_map + mowgli_coverage)

**Purpose:** Per-area Fields2Cover v3 coverage planning with `mow_progress` resume semantics. Two packages cooperate.

**Locations:** `src/mowgli_map/` (area storage, mow_progress grid, `get_mowing_area` service), `src/mowgli_coverage/` (Fields2Cover v2.0 coverage server).

**Coverage Loop (driven by BT nodes):**

1. `GetNextUnmowedArea` — outer loop. Picks the next area whose `mow_progress` layer still has un-mowed cells; writes its index to the blackboard. Returns FAILURE when all areas are complete.
2. `PlanCoverageArea` (ONE per area per session) — calls `map_server_node`'s `get_mowing_area` service to fetch `area outer ring + obstacle holes` (Boost.Geometry difference, CCW outer / CW holes), then sends a `plan_coverage` action goal (`mowgli_interfaces/action/PlanCoverage`) to `mowgli_coverage`. Stores the resulting discrete segments AND the continuous, hole-free `drivable_subpaths` on the BT blackboard.
3. `FollowStrip` — drives each `drivable_subpath` as ONE `FollowCoveragePath` goal, tracked end-to-end by `FTCController` (no per-segment dispatch), bridging consecutive sub-paths with a blade-off `NavigateToPose` transit around the obstacle. On abort, `RetryUntilSuccessful(num_attempts=5)` re-ticks `FollowStrip`, which **trims the plan at a pose cursor** to resume where it stopped — the plan is deterministic, so the cursor is stable across re-plans. (FTC's own `setPlan` starts at index 0; the legacy nearest-pose snap is behind `snap_to_nearest_on_set_plan`, default `false`, because it skipped most of a closed headland ring.) `IsObstacleStuck` and `WasRecentlyInCollisionStop` fallback branches insert `BackUp` + `ClearCostmap` between retries.

Progress is tracked in the `mow_progress` grid layer (survives restarts, stamp radius `tool_width / 2`). Coverage status is available via `~/get_coverage_status` service and `/map_server_node/coverage_cells` OccupancyGrid topic.

The legacy on-demand strip planner (`~/get_next_strip` service and its `GetNextStrip` BT node) has been **removed**. `map_server_node` still owns the `mow_progress` grid and its cell stamping, but the only coverage path is the F2C output.

**mowgli_coverage server (Fields2Cover 3.0.0):**

- Action: `plan_coverage`, type `mowgli_interfaces/action/PlanCoverage` (an in-tree definition — the upstream `opennav_coverage_msgs/ComputeCoveragePath` interface is no longer used).
- Backend pinned to **Fields2Cover 3.0.0** at `/opt/fields2cover-300` (`find_package(Fields2Cover 3.0.0 CONFIG REQUIRED PATHS /opt/fields2cover-300)`). A v2.0.0 tree is retained at `/opt/fields2cover-200` for the devcontainer but is not `ldconfig`'d, so nothing links it; the version pin alone rejects it.
- F2C pipeline per `planBoustrophedon()` call — deliberately minimal, and what it does **not** do matters as much as what it does:
  1. **Robot setup** — `f2c::types::Robot`, `setCovWidth(operation_width)`, `setMinTurningRadius`. `operation_width` is injected at launch as `tool_width − swath_overlap`, so adjacent swaths deliberately overlap.
  2. **Cell construction** — `goal.polygons[0]` is the outer ring; subsequent polygons are interior holes (obstacles). Degenerate, millimetre-scale rings are sanitised first — they used to yield silent partial coverage.
  3. **Chassis-safety pre-inset** — `ConstHL::generateHeadlands(chassis_safety_inset)`.
  4. **Headland rings** — `ConstHL::generateHeadlandSwaths(op_width, n_rings, out2in)` emits N concentric perimeter passes, outermost first, each split into corner-to-corner arcs. `num_headland_passes` is a three-way sentinel: negative plans **zero** rings, `0` means auto (`ceil(headland_width / op_width)`, floored at 1), positive forces the count.
  5. **Mainland inset** — `ConstHL::generateHeadlands(n_rings * op_width)`.
  6. **Swaths** — `BruteForce::generateBestSwaths` (fixed `mow_angle_deg` or auto) then `BoustrophedonOrder` for the serpentine order. Each disjoint clip of a sweep line is its own swath, so concave fields and holes need **no** decomposition.
  7. **Continuous sub-paths** — `buildContinuousSubPaths` joins the rings and swaths with Mowgli's **own** forward-only turn-around arcs (`buildConnector`, nominal radius `connector_turn_radius`, floored at `min_turning_radius`, cusp-free and bounded to the safety inset). Every turn is validated to stay inside the recorded boundary; a field with holes is split so each obstacle gap becomes a blade-off Nav2 transit, and the sub-paths are ordered by a greedy nearest-neighbour pass to minimise that transit.
  8. **Result** — the ordered discrete segments (typed `ring` or `swath`, kept for the GUI and resume bookkeeping), the hole-free `drivable_subpaths`, and their concatenation as `full_path`.

  **Not in the pipeline, on purpose:** no `TrapezoidalDecomp`, no `f2c::pp::PathPlanning`, no F2C turn planner (Dubins, CC-Dubins or Reeds-Shepp), no OR-Tools routing, and no boundary clipping. Every F2C turn variant was field-tested and failed — forward-only variants omega-loop at this swath spacing, and Reeds-Shepp cusps are untrackable because the controller does not reverse mid-swath. Reintroducing any of them is explicitly forbidden by the root `CLAUDE.md`.

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
5. Robot mows the current area by driving each planned sub-path via TransitToStrip → FollowStrip
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

**Purpose:** Webots R2025a simulation environment, driven through `webots_ros2_driver`. It exists so the Nav2 + behaviour-tree + FTC stack can be exercised end-to-end (coverage strips, obstacle deviation, dock approach) without hardware — **not** to be a faithful physics model of the drivetrain. Gazebo Ignition was the original backend and was replaced wholesale.

**Location:** `src/mowgli_simulation/`

**Architecture:**

```
Simulation Stack (webots_ros2_driver + ros2_control)
    │
    ├── Webots R2025a server  (WebotsLauncher, Xvfb headless)
    │   └── worlds_webots/mowgli_garden.wbt
    │       └── protos/MowgliMower.proto      (the robot: 2 RotationalMotor
    │           │                              + 2 PositionSensor wheels)
    │           ├── Lidar        → /scan            (sensor_msgs/LaserScan)
    │           ├── GPS          → /gps/fix_raw     (sensor_msgs/NavSatFix)
    │           └── InertialUnit → /imu/data_sim    (sensor_msgs/Imu)
    │
    ├── WebotsController (urdf_webots/mowgli_webots.urdf)
    │   ├── declares the device→topic bindings above
    │   ├── ros2_control: diff_drive_controller + joint_state_broadcaster
    │   │   (config_webots/ros2_control.yaml)
    │   └── plugin: mowgli_simulation.kinematic_drive  ← chassis motion
    │
    ├── Sim-only shims (make the perfect sim look like the real robot)
    │   ├── sim_actuation_node        (C++)  wheel command → firmware model
    │   ├── fake_hardware_bridge_node (C++)  stands in for the STM32 bridge
    │   ├── sim_navsat_rtk_fix.py     /gps/fix_raw    → /gps/fix
    │   ├── sim_wheel_slip.py         /wheel_odom_raw → /wheel_odom
    │   └── sim_imu_noise.py          /imu/data_sim   → noisy IMU stream
    │
    └── ROS2 stack (identical to real hardware)
        ├── robot_state_publisher (URDF), twist_mux
        ├── Nav2 + fusion_graph localizer
        ├── map_server, obstacle_tracker, diagnostics, foxglove_bridge
        └── behavior_tree_node
```

**How the chassis actually moves — read this before touching the sim.**
`mowgli_simulation/kinematic_drive.py` is a Webots **Supervisor** plugin that does *not* rely on wheel-floor friction. Each timestep it integrates the diff-drive kinematics from the latest `/cmd_vel` (`TwistStamped`, from twist_mux) and **teleports** the robot node by writing its `translation` and `rotation` fields. The reason is recorded in the plugin header and in `protos/MowgliMower.proto`: after exhausting mass, damping, friction, centre-of-mass and motor-torque tuning, ODE still pitched the chassis ~13° forward at static equilibrium and delivered only ~10 % of a commanded 0.10 m/s to the body. The `diff_drive_controller` still drives the wheel motors from the same command, so encoders and wheel odometry stay consistent with the teleported pose. The plugin also publishes `/sim/ground_truth_pose` (`geometry_msgs/PoseStamped`) — the sim's authoritative chassis location, immune to sensor noise and localizer drift; `fake_hardware_bridge_node` reads it for charging detection, mirroring the real robot's physical-contact charging signal.

**Sim-only nodes:**

| Node | Language | What it does |
|------|----------|-------------|
| `kinematic_drive.py` | Python (Webots plugin) | Kinematic teleport of the chassis from `/cmd_vel`; publishes `/sim/ground_truth_pose` |
| `sim_actuation_node` | C++ | Inserts the missing actuator physics between the nav command and the wheels, reproducing the STM32 wheel model (`include/mowgli_simulation/firmware_wheel_model.hpp`, unit-tested by `test/test_firmware_wheel_model.cpp`). The host-side yaw-rate loop it once carried was removed when that loop moved into firmware |
| `fake_hardware_bridge_node` | C++ | Stands in for `hardware_bridge_node`: battery/charging/emergency/status surface without an STM32 |
| `sim_navsat_rtk_fix.py` | Python | Programmable GPS-quality controller. Rewrites `/gps/fix_raw` onto the production `/gps/fix` with the status, covariance and position noise of a chosen RTK regime (RTK-Fixed by default) |
| `sim_wheel_slip.py` | Python | Relays `/wheel_odom_raw` → `/wheel_odom`, injecting periodic slip events so the localizer sees encoder-vs-ground-truth divergence |
| `sim_imu_noise.py` | Python | Adds noise and a bias random walk to the perfect IMU stream so the stack runs against MEMS-representative data |

**Launch entry points:**

| Command | What it starts |
|---------|----------------|
| `cd ros2 && make sim` | The full stack, headless, no RViz (runs `sim-stop` first) |
| `ros2 launch mowgli_bringup sim_full_system.launch.py` | The full stack. Note it does **not** go through `mowgli.launch.py`: it launches its own twist_mux and skips `hardware_bridge`/`robot_state_publisher` wiring, so BT knobs sourced from `mowgli_robot.yaml` fall back to C++ defaults — sim behaviour is not evidence about the robot |
| `ros2 launch mowgli_simulation webots_minimal.launch.py` | Webots + the world + ros2_control only, no Nav2 stack. `world:=` selects a file inside `worlds_webots/` |
| `cd ros2 && make e2e-test` | Self-contained: `sim-stop`, build, launch the sim, wait, run `src/e2e_test.py`, stop the sim |

**Container:** the `simulation` Docker stage (`ros2/Dockerfile`) extends `runtime` with Xvfb, TigerVNC + noVNC for GUI access, `ros-kilted-webots-ros2`, and the Webots release `.deb`. Cyberbotics publishes that `.deb` for **linux/amd64 only**, and the stage asserts `TARGETARCH = amd64`, so the simulation image cannot be built on ARM (Apple Silicon or a Raspberry Pi needs emulation or an x86 host). The `runtime` image stays architecture-neutral.

**After any crash, run `make sim-stop` before relaunching** — a stale Webots IPC socket or leftover Cyclone DDS shared memory hangs the next launch in a retry loop.

For the ODE quirks and load-bearing workarounds behind the world, the PROTO and `kinematic_drive.py`, read [`docs/WEBOTS_SIM.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/WEBOTS_SIM.md) before editing any of them.

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
                ├─ PlanCoverageArea → TransitToStrip → FollowStrip (inner loop)
                └─ Progress tracked in mow_progress grid layer

3. Navigation to coverage start:
   NavigateToPose(first_waypoint):
     ├─ Nav2 planner: global path from odometry to start
     ├─ RPP controller (RegulatedPurePursuit + RotationShimController)
     ├─ Costmap: /scan + odom → local obstacles
     └─ cmd_vel → hardware_bridge → STM32 → wheels

4. Coverage path following (CoverageWithRecovery loop):
   FollowCoveragePath (FTCController, one whole drivable_subpath at a time):
     ├─ ONE in-place PRE_ROTATE pivot to the path-start heading, then it
     │  tracks the continuous sub-path end-to-end, following the planner's
     │  forward turn-around arcs through every swath U-turn
     ├─ Obstacle avoidance by deviating the path LATERALLY in real time
     │  (enable_obstacle_deviation, reading the LiDAR obstacle_layer; off in
     │  the no-LiDAR overlay. collision_monitor PolygonSlow is the soft
     │  slowdown fallback)
     └─ Returns: RUNNING (in progress), SUCCESS (path complete), FAILURE (stuck)

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

   fusion_graph_node (GTSAM iSAM2 factor graph, sole localizer):
     ├─ Fuses wheel between-factor + gyro between-factor + GnssLeverArmFactor
     │  (/gps/fix, projected in-node) + COG/mag yaw unaries (+ optional LiDAR factors)
     ├─ → /odometry/filtered_map (map frame)
     └─ Publishes BOTH /tf legs from a dedicated TF broadcast thread:
           odom → base_footprint  (continuous dead-reckoning)
           map → odom             (absorbs GPS corrections)

   The /map OccupancyGrid comes from map_server_node's user-defined area
   polygons, not from SLAM.

   FTCController (coverage, FollowCoveragePath) / RPP (transit, FollowPath):
     ├─ Read robot pose from /tf and velocity from controller_server.odom_topic
     ├─ FTC: deterministic Follow-the-Carrot, decoupled lon/lat/ang PID,
     │  tracking one whole continuous sub-path
     ├─ RPP: regulated pure pursuit with curvature-based speed scaling,
     │  wrapped in RotationShim (transit)
     └─→ /cmd_vel_nav → twist_mux → /cmd_vel

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
     ├─ IsLocalizationHealthy (fusion-graph cov_xx/cov_yy < threshold)
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
       ├─ /fusion_graph/diagnostics (factor-graph health)
       ├─ /diagnostics (system health)
       └─ /tf tree (all frame transformations)
```

---

## TF Tree Reference

**Standard ROS2 conventions (REP-103 + REP-105):**

```
map (GPS-anchored via fixed datum from mowgli_robot.yaml)
  │ [published by fusion_graph_node]
  │
  odom (continuous, drift-only)
  │ [published by fusion_graph_node]
  │
  base_footprint (on ground, robot frame for Nav2)
  │ [base_footprint → base_link static, from URDF]
  │
  ├── base_link (robot body frame, wheel axle height per OpenMower convention)
  │   └── [published by robot_state_publisher from URDF]
  │
  ├── imu_link (fixed to chassis, IMU measurement frame)
  │   └── [hardware_bridge publishes IMU data in this frame;
  │        fusion_graph consumes gyro yaw rate + tilt for the gyro/mag factors]
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
| map | fusion_graph_node | TF thread (20 Hz) | Global frame, anchored to the fixed datum from `mowgli_robot.yaml` |
| odom | fusion_graph_node | TF thread (20 Hz) | Continuous local frame (dead-reckoning) |
| base_footprint | fusion_graph_node | TF thread (20 Hz) | On ground, robot frame for Nav2/controllers (REP-105) |
| base_link | robot_state_publisher | Static | Rear wheel axle (OpenMower convention) |
| imu_link | robot_state_publisher | Static | IMU sensor frame |
| lidar_link | robot_state_publisher | Static | LiDAR origin (collision_monitor + obstacle_layer consume here) |
| gps_link | robot_state_publisher | Static | GPS antenna (navsat_to_absolute_pose rotates the offset to base_footprint) |
| `gps_link` | `base_link` | Static | GNSS antenna origin (`gps_x/y/z` lever arm in `mowgli_robot.yaml`) |

**Frame Conventions:**

- `map` – Global frame, z-up, x-east, y-north (REP-103). Anchored to the fixed datum from `mowgli_robot.yaml`. Published by `fusion_graph_node`.
- `odom` – Continuous local frame, drift-only. Published by `fusion_graph_node`.
- `base_footprint` – Ground contact point, robot frame for Nav2. Published by `fusion_graph_node`.
- `base_link` – Robot body frame at rear wheel axle height (OpenMower convention). Static offset from base_footprint.

**Simulation (Webots):**

There are **no** sensor-frame bridges. The sim URDF (`src/mowgli_simulation/urdf_webots/mowgli_webots.urdf`) declares the production frame names directly through fixed joints — `base_footprint_to_base_link`, `base_link_to_lidar`, `base_link_to_imu`, `base_link_to_gps` — so `lidar_link`, `imu_link` and `gps_link` mean the same thing in sim and on the robot. Costmap, collision_monitor, fusion_graph and every other node therefore need no simulation-specific frame handling.

The one sim-only addition is `/sim/ground_truth_pose`, published by the `kinematic_drive` Supervisor plugin. It is a topic, not a TF frame, and nothing in the production stack reads it (only `fake_hardware_bridge_node`, for charging detection).

---

## Topic Map

**Publishers (Sources):**

| Topic | Type | Publisher | Rate | Purpose |
|-------|------|-----------|------|---------|
| `/map` | nav_msgs/OccupancyGrid | map_server_node | on area change | Mowing-area polygons rasterised to an OccupancyGrid (NOT SLAM output) |
| `/map_server_node/coverage_cells` | nav_msgs/OccupancyGrid | map_server_node | 1 Hz | Mow progress grid (cells marked mowed; persists across restarts) |
| `/scan` | sensor_msgs/LaserScan | ldlidar driver (real) / the Webots `Lidar` device via `WebotsController` (sim) | 10 Hz | LiDAR range data (LD19 on real hardware) |
| `/imu/data` | sensor_msgs/Imu | hardware_bridge_node | ~48 Hz | Gyro + accel, bias-corrected on dock (1000-sample calibration) |
| `/status` | mowgli_interfaces/Status | hardware_bridge_node | ~4 Hz | Mower state (blade on/off, rain, charging) |
| `/hardware_bridge/emergency` | mowgli_interfaces/Emergency | hardware_bridge_node | ~4 Hz | Emergency stop status (latched, active) |
| `/hardware_bridge/power` | mowgli_interfaces/Power | hardware_bridge_node | ~4 Hz | Battery voltage, charging current (consumed by SimpleChargingDock) |
| `/wheel_odom` | nav_msgs/Odometry | hardware_bridge_node | ~10 Hz | Integrated wheel velocities (RELIABLE; tight vy covariance enforces non-holonomic constraint) |
| `/gps/fix` | sensor_msgs/NavSatFix | ublox_nav_sat_fix_hp_node | 5 Hz | RTK Fixed when available (σ ~3 mm) |
| `/gps/absolute_pose` | geometry_msgs/PoseWithCovarianceStamped | navsat_to_absolute_pose_node | 5 Hz | RTK position converted to local ENU (diagnostics / BT) |
| `/odometry/filtered_map` | nav_msgs/Odometry | fusion_graph_node | factor-graph cadence | Fused pose (map frame, GPS-corrected; LiDAR-aware when scan matching is enabled) |
| `/fusion_graph/diagnostics` | diagnostic_msgs/DiagnosticArray | fusion_graph_node | 1 Hz | Factor-graph health (total_nodes, scan-match success, loop closures, cov_xx/yy) |
| `/fusion_graph/markers` | visualization_msgs/MarkerArray | fusion_graph_node | 1 Hz | Node positions, trajectory, loop-closure edges |
| `/localization/status` | diagnostic_msgs/DiagnosticStatus | localization_monitor_node | 2 Hz | GPS quality + localizer health |
| `/plan_coverage/_action/feedback` and `/plan_coverage/_action/result` | mowgli_interfaces/action/PlanCoverage | mowgli_coverage (coverage_server) | Per BT PlanCoverageArea call | F2C v3 plan: typed segments + hole-free `drivable_subpaths` + `full_path` — each sub-path is driven by the FollowStrip BT node as one FollowCoveragePath goal |
| `/path` | nav_msgs/Path | planner_server (Nav2) | 1 Hz | Global path from Nav2 planner |
| `/cmd_vel` | geometry_msgs/TwistStamped | twist_mux | 10–50 Hz | Final velocity command — to `hardware_bridge_node` on the robot, to the `kinematic_drive` Supervisor plugin in the sim |
| `/diagnostics` | diagnostic_msgs/DiagnosticArray | diagnostics_node (mowgli_monitoring) | 1 Hz | System health aggregation |
| `/behavior_tree_log` | mowgli_interfaces/BehaviorTreeLog | behavior_tree_node | 10 Hz | Active BT node, node type, execution status (for GUI diagnostics visualization) |
| `/high_level_status` | mowgli_interfaces/HighLevelStatus | behavior_tree_node | 10 Hz | Current high-level mode (IDLE=1, AUTONOMOUS=2, RECORDING=3, MANUAL_MOWING=4) with coverage progress per area |

**Subscribers (Sinks):**

| Topic | Subscriber | Purpose |
|-------|-----------|---------|
| `/cmd_vel` | hardware_bridge_node (real); `kinematic_drive.py` Supervisor (Webots sim) | Motor/wheel commands |
| `/scan` | fusion_graph_node (when use_scan_matching/use_loop_closure), collision_monitor, obstacle_layer, diagnostics_node | Scan-matching factors, obstacle detection, monitoring |
| `/imu/data` | fusion_graph_node, diagnostics_node | Sensor fusion (gyro between-factor) + monitoring |
| `/wheel_odom` | fusion_graph_node, controller_server (odom_topic), diagnostics_node | Wheel between-factor + FTC/RPP velocity feedback + monitoring |
| `/gps/fix` | fusion_graph_node | GnssLeverArmFactor (projects lat/lon in-node; the graph's GNSS input) |
| `/gps/pose_cov` | map_server_node | RTK-quality gate for `~/set_docking_point`; also GUI/BT |
| `/odometry/filtered_map` | nav2 (bt_navigator), BT, diagnostics_node, GUI | Localized pose for control, behavior tree, diagnostics |
| `/map` | nav2 planner, behavior_tree (area polygons), diagnostics_node | Global navigation reference (area-polygon grid) |
| `/gps/fix` | navsat_to_absolute_pose_node, diagnostics_node | Convert fix to local frame, monitor GPS |
| `/status` | behavior_tree_node, localization_monitor_node, diagnostics_node | Health checks, sensor freshness |
| `/emergency` | behavior_tree_node, diagnostics_node | Emergency monitoring |
| `/power` | behavior_tree_node, diagnostics_node | Battery level monitoring |
| `/controller_server/FollowCoveragePath/global_plan` | PathProgressGoalChecker | Coverage completion gating (>= 95% path-pose tracking + xy/yaw to goal pose) |

**Services (Request-Response):**

| Service | Type | Server | Client | Purpose |
|---------|------|--------|--------|---------|
| `/mower_control` | MowerControl | hardware_bridge_node | behavior_tree_node | Enable/disable blade motor |
| `/emergency_stop` | EmergencyStop | hardware_bridge_node | behavior_tree_node, ResetEmergency BT node | Assert/release latched emergency |
| `/high_level_control` | HighLevelControl | behavior_tree_node | GUI | Mode commands (START=1, HOME=2, RECORD_AREA=3, S2=4, RECORD_FINISH=5, RECORD_CANCEL=6, MANUAL_MOW=7) |
| `/map_server_node/add_area` | AddMowingArea | map_server_node | RecordArea BT node | Save recorded mowing area polygon |
| `/map_server_node/get_next_unmowed_area` | GetNextUnmowedArea | map_server_node | behavior_tree_node (GetNextUnmowedArea BT node) | Fetch next unmowed area polygon and coverage grid |
| `/map_server_node/get_mowing_area` | GetMowingArea | map_server_node | behavior_tree_node (PlanCoverageArea BT node) | Returns the area outer ring + obstacle holes (Boost.Geometry difference, CCW outer / CW holes) — fed to mowgli_coverage's `plan_coverage` action |
| `/map_server_node/mark_area_complete` | MarkAreaComplete | map_server_node | behavior_tree_node | Mark area as mowing complete, persist coverage state |
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | nav2_behavior_tree_navigator | behavior_tree_node | Send goal to Nav2 |

**Actions (Async Request-Response):**

| Action | Type | Server | Client | Purpose |
|--------|------|--------|--------|---------|
| `/navigate_to_pose` | nav2_msgs/NavigateToPose | nav2_behavior_tree_navigator | behavior_tree_node (NavigateToPose BT node) | Non-blocking navigation goal |

---

## Summary: Architectural Principles

1. **12-Package Modular Design:** Separation of concerns across hardware, localization, navigation, planning, monitoring, and behavior layers.
   - **Core:** mowgli_interfaces (message + service + action definitions, including `GetMowingArea.srv` and `PlanCoverage.action`)
   - **Hardware:** mowgli_hardware (STM32 bridge via COBS)
   - **Perception:** mowgli_localization (localization helpers feeding the factor graph — `cog_to_imu_node`, `mag_yaw_publisher`, `navsat_to_absolute_pose_node`, `costmap_scan_filter_node`, `scan_deskew_node`)
   - **Localization (sole, default):** fusion_graph (GTSAM iSAM2 factor graph; publishes both `map→odom` and `odom→base_footprint`, launched unconditionally)
   - **Control:** mowgli_nav2_plugins (`FTCController` for FollowCoveragePath, `PathProgressGoalChecker` for coverage completion; FollowPath/transit uses upstream RotationShim + RPP, which are not in this package)
   - **Planning (areas + cells):** mowgli_map (area storage in `areas.dat`, `mow_progress` grid, `~/get_mowing_area` for F2C, keepout mask + obstacle promotion)
   - **Planning (coverage path):** mowgli_coverage (Fields2Cover 3.0.0 server, action `plan_coverage`)
   - **Behavior:** mowgli_behavior (BehaviorTree.CPP v4, 10 Hz reactive control; F2C-driven coverage via `PlanCoverageArea` + `FollowStrip`)
   - **Monitoring:** mowgli_monitoring (diagnostics, MQTT bridge)
   - **Simulation:** mowgli_simulation (Webots, perfect-IMU mode synthesised from `/cmd_vel`)
   - **Status:** mowgli_leds (optional WS2812 status ring over SPI, off by default)
   - **Infrastructure:** mowgli_bringup (launch files, the config template, Nav2 params, and the URDF/xacro — there is no separate `mowgli_description` package)
   - **Third-party:** `opennav_coverage` git submodule for the `_msgs` action definitions only — that is the sole subpackage `ros2/scripts/sync_workspace_packages.sh` symlinks into the workspace, so colcon never sees the upstream servers (there are no `COLCON_IGNORE` files).

2. **ROS2 Kilted + Webots R2025a:** Modern robotics stack with lifecycle management, and a simulator that runs the identical ROS2 stack (see [§9](#9-mowgli_simulation)).

3. **Decoupled Communication:** ROS2 pub/sub (topics), services, and actions isolate packages. Easy to substitute, test, or extend components independently.

4. **Robust Serial Protocol (COBS + CRC-16):** Enables reliable bidirectional communication between Raspberry Pi and STM32 firmware over noisy USB at 115200 baud.

5. **Factor-Graph Localization (fusion_graph — sole, default, unconditional):**
   - `fusion_graph_node` (GTSAM iSAM2 Pose2 graph) publishes **both**
     `odom → base_footprint` (continuous dead-reckoning, never jumps) AND
     `map → odom` (absorbs GPS corrections), from a dedicated TF broadcast
     thread. There is no `use_fusion_graph` arg — it is the only localizer.
   - Fuses a wheel between-factor (non-holonomic σ_y ≪ σ_x, from
     `/wheel_odom`), a gyro between-factor on yaw, a custom
     `GnssLeverArmFactor` (analytic Jacobian, lever-arm rotates with node
     yaw, fed by `/gps/fix` and projected in-node), and unary yaw factors for COG
     (`/imu/cog_heading`) and tilt-compensated mag (`/imu/mag_yaw`, when
     calibrated). Datum + lever-arm from `mowgli_robot.yaml`.
   - The robot_localization dual EKF (`ekf_map_node` + `ekf_odom_node`),
     `navsat_transform_node`, `robot_localization.yaml`, slam_toolbox,
     Kinematic-ICP, and FusionCore were all removed.
   - No SLAM back-end. The map frame is GPS-anchored; saved area polygons,
     dock pose, and the persisted graph survive restarts.
   - Graceful degradation: operates without GPS in GNSS-denied areas
     (wheel + gyro dead-reckoning; with LiDAR `use_scan_matching` /
     `use_loop_closure` on, scan-matching between-factors keep the map-frame
     estimate stable across multi-minute RTK-Float windows).

6. **Coverage Path Following:** `FollowCoveragePath` = `mowgli_nav2_plugins/FTCController`
   standalone (no RotationShim), fed ONE whole continuous `drivable_subpath`:
   it does a single in-place `PRE_ROTATE` pivot to the path-start heading,
   then tracks the sub-path end-to-end through the planner's forward
   turn-around arcs, deviating the path laterally in real time to skirt
   obstacles. `FollowPath` (transit) = RotationShim wrapping RPP
   (`nav2_regulated_pure_pursuit_controller`). FTC was restored on 2026-06-19,
   reverting an MPPI experiment; `test_nav2_params.py` now forbids FTC in the
   transit slot and MPPI in the coverage slot. Coverage completion is gated by
   `mowgli_nav2_plugins/PathProgressGoalChecker` (>= 95 % path-pose tracking +
   xy/yaw to goal pose) — `StoppedGoalChecker`/`SimpleGoalChecker` fired on
   velocity stoppage during the in-place pivot, completing the action at <2 %
   coverage.

7. **F2C v3 Coverage Planning:** the in-tree `mowgli_coverage` server (action `plan_coverage`, `mowgli_interfaces/action/PlanCoverage`) wraps Fields2Cover **3.0.0** in a deliberately minimal pipeline (`planBoustrophedon`): `ConstHL::generateHeadlands` pre-inset, `ConstHL::generateHeadlandSwaths` for the concentric mowed rings, then `BruteForce::generateBestSwaths` + `BoustrophedonOrder` for the serpentine swaths. There is **no** `TrapezoidalDecomp` and **no** F2C turn planner — every Dubins/Reeds-Shepp variant was field-tested and failed, so rings and swaths are joined by Mowgli's own forward-only turn-around arcs (`buildConnector`, `buildContinuousSubPaths`), which are cusp-free and bounded to the chassis-safety inset. The result carries the discrete segments (for the GUI and resume bookkeeping) plus one or more continuous, hole-free `drivable_subpaths`. `mowgli_map`'s `~/get_mowing_area` feeds it the area outer ring plus obstacle holes. The upstream `opennav_coverage` submodule is kept for its `_msgs` package only — that is the sole subpackage `ros2/scripts/sync_workspace_packages.sh` symlinks into the workspace, so colcon never sees the upstream servers (pinned to F2C 1.2.1, missing the features above).

8. **Reactive Behavior Trees (BehaviorTree.CPP v4):** 10 Hz non-preemptive tree execution with priority-based fallback selection:
   - Emergency guard: highest priority, interrupts all activity
   - Multi-mode state machine: IDLE → UNDOCKING → MOWING (with recovery) → DOCKING
   - Multi-area coverage: iterates through multiple mowing areas sequentially, each with independent coverage progress tracking
   - Composable async action nodes (`NavigateToPose`, `GetNextUnmowedArea`, `PlanCoverageArea`, `TransitToStrip`, `FollowStrip`)

9. **Priority-Based Command Routing:** twist_mux mediates three command sources (emergency > teleoperation > navigation) before forwarding to hardware bridge.

10. **Comprehensive Health Monitoring:** `diagnostics_node` publishes **nine** statuses — hardware bridge, emergency, battery, IMU, LiDAR, GPS, odometry, motors, and the fused-pose check (`check_fusion`: rate and position covariance of `/odometry/filtered_map`; `/fusion_graph/diagnostics` carries the graph's own detail). Levels emitted are **OK, WARN and ERROR**; a `STALE` level exists in the enum but no check ever assigns it — a never-received stream maps to ERROR or WARN depending on the check. Real-time BT visualization shows active node path and execution state in GUI diagnostics page.

11. **Unified Simulation-to-Hardware Workflow:**
    - Webots R2025a via `webots_ros2_driver` runs the identical ROS2 stack in sim and on real hardware
    - The sim URDF declares the production frame names directly, so no frame translation is needed
    - Sim-only shims (`sim_navsat_rtk_fix`, `sim_wheel_slip`, `sim_imu_noise`, `sim_actuation_node`, `fake_hardware_bridge_node`) degrade the perfect sim signals to hardware-representative ones
    - Behavior tree, Nav2, localization and diagnostics unchanged between environments (there is no SLAM in either)

12. **Foxglove Bridge for Remote UI:** Modern WebSocket-based telemetry (port 8765) replaces legacy rosbridge, reducing latency and CPU overhead.

---

## Development & Testing

**Key Resources:**

| Resource | Location | Purpose |
|----------|----------|---------|
| URDF/Xacro | src/mowgli_bringup/urdf/mowgli.urdf.xacro | Robot kinematics, sensor frames, collision geometry |
| fusion_graph Config | src/fusion_graph/config/fusion_graph.yaml | Factor-graph tuning: node cadence, wheel/gyro/GPS sigmas, lever-arm, COG/mag gates, iSAM2 + LiDAR knobs |
| Nav2 Config | src/mowgli_bringup/config/nav2_params_base.yaml (+ lidar/no_lidar overlays) | Planner, controller, costmap tuning |
| Behavior Tree | src/mowgli_behavior/trees/main_tree.xml | High-level state machine and sequencing |
| Coverage Config | src/mowgli_bringup/config/mowgli_robot.yaml | Coverage parameters (tool_width, mow_angle_deg, swath_overlap, num_headland_passes) |
| Webots World | src/mowgli_simulation/worlds_webots/mowgli_garden.wbt | Garden world; `MowgliMower` PROTO in `protos/`, sim URDF in `urdf_webots/` |

**Testing Workflow:**

```bash
# Simulation (Webots, full stack) — or `cd ros2 && make sim`
ros2 launch mowgli_bringup sim_full_system.launch.py headless:=true use_rviz:=false

# Webots world only, no Nav2 stack
ros2 launch mowgli_simulation webots_minimal.launch.py

# Real hardware (Raspberry Pi + STM32)
ros2 launch mowgli_bringup full_system.launch.py

# Navigation + localization only (Nav2 + fusion_graph)
ros2 launch mowgli_bringup navigation.launch.py

# Coverage planning (standalone test) — plan_coverage is an ACTION, not a service
ros2 action send_goal /plan_coverage mowgli_interfaces/action/PlanCoverage '{area_id: 0}'

# Diagnostics monitoring
ros2 topic echo /diagnostics
ros2 topic echo /fusion_graph/diagnostics
```

---

## Next Steps

- [Configuration](Configuration) — every parameter explained (PID gains, speed profiles, costmap, coverage).
- [Firmware](Firmware) — STM32 packet protocol and firmware integration.
- [Simulation](Simulation) — running the Webots sim and the E2E test. For the sim's ODE quirks and load-bearing workarounds, see [`docs/WEBOTS_SIM.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/WEBOTS_SIM.md).
- [Getting Started](Getting-Started) — build instructions and devcontainer setup.
- Machine-readable indexes for coding agents: [`docs/claude/doc-index.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/doc-index.md) points at the per-area codemaps, the full topic/service index, the parameter index and the test/CI index.
