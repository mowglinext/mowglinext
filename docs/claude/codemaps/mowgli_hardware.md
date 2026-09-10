# Codemap: mowgli_hardware

> `ros2/src/mowgli_hardware` is the single ROS2 ↔ STM32 boundary: `hardware_bridge_node` owns the
> USB-serial link (COBS framing + CRC-16, `/dev/mowgli`), decodes firmware packets into
> status/emergency/power/IMU/wheel-odometry topics, forwards the merged `cmd_vel` and blade/e-stop
> requests to the board, pushes runtime tuning (drive PID, yaw loop, kinematics, safety limits) on
> every (re)connect, runs the at-rest IMU bias calibration, and hosts the wheel-slip dig detector
> (CLAUDE.md Invariant 16). The firmware stays the sole blade/e-stop authority (CLAUDE.md Safety).
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `ros2/CLAUDE.md`.

## Where to look

| Task | Start here |
|------|------------|
| Add / rename a ROS parameter | `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp` `declare_parameters()` (:330–689); live-tunable set in the `add_on_set_parameters_callback` (:434) |
| Add a published topic / change QoS | `hardware_bridge_node.cpp` `create_publishers()` (:702); wheel topics live in `src/odometry_publisher.cpp` ctor (:32–41) |
| Add a subscription | `hardware_bridge_node.cpp` `create_subscribers()` (:740) |
| Add a hosted service / a client | `create_services()` (:807) / `create_service_clients()` (:846) |
| Add or change a wire packet (struct, ID, size) | `include/mowgli_hardware/ll_datatypes.hpp` (`PacketId` enum :58–78, packed structs + `static_assert` sizes) → `on_packet_received()` switch (:1108) → `test/test_protocol.cpp` |
| Bump the protocol version | `ll_datatypes.hpp` `kMowgliProtocolVersion` (:51) **and** `firmware/stm32/ros_usbnode/include/mowgli_protocol.h` `MOWGLI_PROTOCOL_VERSION` (:59) — see Change coupling |
| Firmware version handshake / `firmware_compatible` | `handle_config_rsp()` (:2441), `rearm_firmware_handshake()` (:2516), `service_firmware_handshake()` (:2530), `send_config_request()` (:2558) |
| Serial reconnect / dead-link watchdog | `read_serial_tick()` (:990), `send_raw_packet()` (:1065), param `serial_rx_timeout_s` (:344); POSIX layer `src/serial_port.cpp` (`open` :69 O_NONBLOCK, `write_all` :170) |
| COBS / CRC framing bug | `src/packet_handler.cpp` (`feed` :34, `dispatch_frame` :75, `append_crc` :169), `src/cobs.cpp`, `src/crc16.cpp` |
| Status / emergency / power / battery publishing | `handle_status()` (:1210) — one `LlStatus` packet fans out to `~/status`, `~/emergency`, `~/power`, `/battery_state` |
| Lift-recovery blade logic | `handle_status()` emergency block (`lift_recovery_mode`, `lift_blade_resume_delay_sec`, :613–614) |
| IMU publish, covariance, calibration apply | `handle_imu()` (:1752); dock/off-dock cal triggers in `handle_status()` (:1210) and `handle_imu()`; `start_imu_calibration()` (:1734), `apply_imu_calibration()` (:1637), `discard_imu_calibration()` (:1615) |
| IMU calibration persistence file | `persist_imu_calibration()` (:1445), `load_persisted_imu_calibration()` (:1468); format `# mowgli_imu_calibration_v1`; path param `imu_cal_persist_path` (:631) |
| Dead-IMU (hung WT901 bus) detection | `include/mowgli_hardware/imu_liveness.hpp` (pure); `track_imu_liveness()` (:1709); gates in `handle_imu()` + `load_persisted_imu_calibration()` |
| Wheel odometry (`/wheel_odom`, `/wheel_ticks`) | `src/odometry_publisher.cpp` `handle_packet()` (:54): 16-bit unwrap (:80), spike limit (:96), 50 ms aggregation (:156), dock force-zero (:228), covariances (:241–251) |
| Timestamp smoothing (firmware `dt_millis` → host stamp) | `include/mowgli_hardware/clock_fit.hpp` `HostFirmwareClockFit`, `src/clock_fit.cpp` (`Ingest`, reset gap :16, window :29) |
| Dig detector (wheel-slip) | `include/mowgli_hardware/dig_detector.hpp` (`DigDecide`, `DigTrustSigma`, `DigEscapeStep`); node glue `dig_monitor_tick()` (:2786), `on_dig_detected()` (:2896), `publish_dig_event()` (:2928), `on_filtered_map_odom()` (:2744) |
| `cmd_vel` path to the wire | `on_cmd_vel()` (:2622) → `send_cmd_vel_packet()` (:2944); `min_linear_vel` clamp; NULL→AUTONOMOUS fallback |
| Blade enable / dry-run inhibit | `on_mower_control()` (:2958) + `include/mowgli_hardware/blade_gate.hpp` (`blade_enable_allowed`) → `send_blade_command()` (:2192) |
| Emergency stop service / heartbeat bits | `on_emergency_stop()` (:2987), `send_heartbeat()` (:2154) (`emergency_requested` / `emergency_release_requested`) |
| Runtime tuning pushed to firmware | `send_drive_pid()` (:2214), `send_yaw_pid()` (:2260), `send_kinematics()` (:2300), `send_safety_limits()` (:2326); resend burst in the heartbeat timer (`create_timers()` :890, `pid_resend_count_`) |
| Panel buttons (START/HOME) | `handle_ui_event()` (:2062) → client `/behavior_tree_node/high_level_control` |
| Board reboot / firmware debug flag | `on_reboot_board()` (:2356), `on_set_firmware_debug()` (:2374), `LlReboot` magic `kLlRebootMagic` (`ll_datatypes.hpp` :83) |
| Reset-cause / WWDG breadcrumb decoding | `handle_reset_cause()` (:1146), `reset_cause_name()` / `watchdog_stage_name()` tables (:149–299), constants in `ll_datatypes.hpp` (:89–182) |
| Launch wiring, remaps, param pass-through | `ros2/src/mowgli_bringup/launch/mowgli.launch.py` (:184–268), `ros2/src/mowgli_bringup/config/hardware_bridge.yaml`, template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` |
| Firmware-side C reference of the protocol | `ros2/src/mowgli_hardware/firmware/README.md` — **stale mirror, see Pitfalls**; authoritative is `firmware/stm32/ros_usbnode/` |

## Files

| File | Lines | Purpose |
|------|-------|---------|
| **`ros2/src/mowgli_hardware/`** | | |
| `ros2/src/mowgli_hardware/CMakeLists.txt` | 203 | Static lib `mowgli_hardware_core` (cobs, crc16, serial_port, packet_handler, clock_fit) + exe `hardware_bridge_node`; 8 gtests |
| `ros2/src/mowgli_hardware/package.xml` | 27 | Deps: rclcpp, std_msgs, std_srvs, sensor_msgs, geometry_msgs, nav_msgs, mowgli_interfaces |
| **`src/`** | | |
| `ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp` | ~3.3k | `HardwareBridgeNode` (plain `rclcpp::Node`, name `hardware_bridge`): params, pub/sub/services, serial loop, packet handlers, IMU cal, dig detector glue, `main()` |
| `ros2/src/mowgli_hardware/src/odometry_publisher.cpp` | 256 | `OdometryPublisher`: `LlOdometry` → `~/wheel_ticks` (per packet) + `~/wheel_odom` (50 ms aggregate); `tyre_travelled()` worst-wheel odometer for the dig detector |
| `ros2/src/mowgli_hardware/src/packet_handler.cpp` | 184 | `PacketHandler`: 0x00-delimited COBS deframer, CRC verify/append, rx counters, `kMaxPacketBytes` 512 |
| `ros2/src/mowgli_hardware/src/serial_port.cpp` | 267 | `SerialPort` termios RAII wrapper (8N1, O_NONBLOCK, `write_all` EAGAIN retry, baud → `B*`) |
| `ros2/src/mowgli_hardware/src/clock_fit.cpp` | 105 | `HostFirmwareClockFit::Ingest/Reset/Refit` — sliding-window linear fit firmware-ms → host-ns |
| `ros2/src/mowgli_hardware/src/cobs.cpp` | 111 | `cobs_encode` / `cobs_decode` |
| `ros2/src/mowgli_hardware/src/crc16.cpp` | 47 | `crc16_ccitt` (CCITT-FALSE, poly 0x1021, init 0xFFFF) |
| **`include/mowgli_hardware/`** | | |
| `ros2/src/mowgli_hardware/include/mowgli_hardware/ll_datatypes.hpp` | 536 | Wire format: `kMowgliProtocolVersion`, `PacketId`, `STATUS_BIT_*`, `EMERGENCY_BIT_*`, `RESET_CAUSE_*`, `WATCHDOG_STAGE_*`, `CONFIG_FLAG_*`, packed `Ll*` structs + size asserts |
| `ros2/src/mowgli_hardware/include/mowgli_hardware/dig_detector.hpp` | 346 | Pure dig logic: `DigDetectorCfg/State`, `DigDecide`, `DigTrustSigma`, `DigEscapeCfg/State/Step/Done` (+ the field-measured rationale) |
| `ros2/src/mowgli_hardware/include/mowgli_hardware/imu_liveness.hpp` | 127 | Pure: `IsImuSampleDead`, `UpdateImuLiveness`, `IsCalibrationPlausible`, `IsDeadSensorCovariance`; `kMinPlausibleAccelMps2`=3, `kImuDeadSampleThreshold`=45 |
| `ros2/src/mowgli_hardware/include/mowgli_hardware/blade_gate.hpp` | 49 | Pure: `blade_enable_allowed(requested, mowing_enabled)` — suppresses ENABLE only |
| `ros2/src/mowgli_hardware/include/mowgli_hardware/odometry_publisher.hpp` | 126 | `OdometryPublisher` API (`handle_packet`, `reset`, `wheels_stationary`, `tyre_travelled`) |
| `ros2/src/mowgli_hardware/include/mowgli_hardware/clock_fit.hpp` | 121 | `HostFirmwareClockFit` (window 100 samples, reset gap 5000 ms defaults) |
| `ros2/src/mowgli_hardware/include/mowgli_hardware/packet_handler.hpp` | 172 | `PacketHandler` API + counters (`rx_ok`, `rx_crc_errors`, `rx_overflow`, `rx_cobs_errors`) |
| `ros2/src/mowgli_hardware/include/mowgli_hardware/serial_port.hpp` | 133 | `SerialPort` API |
| `ros2/src/mowgli_hardware/include/mowgli_hardware/cobs.hpp` | 83 | COBS API + `cobs_max_encoded_size` |
| `ros2/src/mowgli_hardware/include/mowgli_hardware/crc16.hpp` | 43 | CRC API |
| **`test/`** (all `ament_add_gtest`) | | |
| `ros2/src/mowgli_hardware/test/test_protocol.cpp` | 499 | Pins the `Ll*` struct sizes (all but `LlCmdBlade` / `LlBladeStatus`, which only the header's `static_assert`s cover), field offsets (odometry, SetDrivePid), `PacketId` values, bitmask positions, COBS+CRC round-trips |
| `ros2/src/mowgli_hardware/test/test_dig_detector.cpp` | 498 | `DigDecide` gates (sigma, turn, cmd, window), field-recorded one-wheel slip, worst-wheel vs centre, net-displacement vs summed steps, `DigTrustSigma`, escape budget |
| `ros2/src/mowgli_hardware/test/test_packet_handler.cpp` | 317 | CRC append/verify, round-trips, chunked feeds, corrupt/oversize/empty frames, counters |
| `ros2/src/mowgli_hardware/test/test_cobs.cpp` | 232 | COBS edge cases (zero runs, 254/255-byte runs, known vectors, decode rejects) |
| `ros2/src/mowgli_hardware/test/test_imu_liveness.cpp` | 213 | Dead-sample threshold, debounce/revive, calibration plausibility, dead-covariance file gate, no input mutation |
| `ros2/src/mowgli_hardware/test/test_clock_fit.cpp` | 180 | First-packet passthrough, jitter averaging, window cap, reset on gap, slope drift |
| `ros2/src/mowgli_hardware/test/test_serial_port.cpp` | 67 | `write_all` over a pty (links `util`), closed-port failure |
| `ros2/src/mowgli_hardware/test/test_blade_gate.cpp` | 43 | ENABLE suppressed only when `mowing_enabled=false`; DISABLE always passes; constexpr |
| **`firmware/`** (C reference copy for the STM32 side — not built by colcon) | | |
| `ros2/src/mowgli_hardware/firmware/mowgli_protocol.h` | 535 | `PKT_ID_*`, `HL_MODE_*`, `pkt_*_t` structs — **at `MOWGLI_PROTOCOL_VERSION 3`, stale vs v6** |
| `ros2/src/mowgli_hardware/firmware/mowgli_comms.c` / `.h` | 312 / 249 | `mowgli_comms_init/process_rx/register_handler/send_*`, 512 B rx buf, 8 handlers |
| `ros2/src/mowgli_hardware/firmware/cobs.c` / `.h` | 135 / 95 | C COBS |
| `ros2/src/mowgli_hardware/firmware/crc16.c` / `.h` | 59 / 57 | C CRC-16 (table-free) |
| `ros2/src/mowgli_hardware/firmware/README.md` | 181 | Integration steps for the C files |

## Runtime surface

### Nodes

| Node | Executable | Launched by | Kind |
|------|------------|-------------|------|
| `hardware_bridge` (`HardwareBridgeNode`) | `hardware_bridge_node` | `ros2/src/mowgli_bringup/launch/mowgli.launch.py` (:189–268; Tier 1, included by `full_system.launch.py`) | plain `rclcpp::Node`, single-threaded spin |
| sim stand-in | `fake_hardware_bridge_node` (`ros2/src/mowgli_simulation`) | `sim_full_system.launch.py` (:297) | mirrors the topic/service names below |

Timers (`create_timers()` :890): serial read at `publish_rate` (100 Hz), heartbeat at `heartbeat_rate` (4 Hz, also drives the tuning resend burst + version handshake), high-level state at `high_level_rate` (2 Hz), dig monitor at `dig_monitor_rate` (10 Hz, created only when `dig_detect_enabled`). The `~/dock_heading` 1 Hz timer is created in `create_publishers()` (:733) instead.

### Topics

Relative names are remapped in `mowgli.launch.py` (:257–267). QoS is `rclcpp::QoS(10)` (reliable) unless noted.

| Topic (node-relative → global) | Type | Dir | Notes / other end |
|-------------------------------|------|-----|--------------------|
| `~/status` → `/hardware_bridge/status` | `mowgli_interfaces/msg/Status` | pub | one per `LlStatus`; carries `firmware_version`, `firmware_protocol_version`, `firmware_compatible` (PreFlightCheck reads it, `mowgli_behavior/src/condition_nodes.cpp` :551). Subs: BT, fusion_graph, map_server, diagnostics, mqtt_bridge, calibrate_imu_yaw, costmap_scan_filter, GUI |
| `~/emergency` → `/hardware_bridge/emergency` | `mowgli_interfaces/msg/Emergency` | pub | same packet; `lift_warning` only in `lift_recovery_mode`. Subs: BT, diagnostics, mqtt_bridge, calibrate_imu_yaw, GUI |
| `~/power` → `/hardware_bridge/power` | `mowgli_interfaces/msg/Power` | pub | same packet. Subs: BT, led_ring_node, diagnostics, mqtt_bridge, GUI |
| `/battery_state` (absolute) | `sensor_msgs/msg/BatteryState` | pub | `current = abs(charging_current)` when charging else 0 (Invariant 12); frame `base_link`; consumed by Nav2 docking (`nav2_params_base.yaml` :1088) |
| `~/imu/data_raw` → `/imu/data` | `sensor_msgs/msg/Imu` | pub | **RELIABLE on purpose** (:708 comment); frame `imu_link`; stamp from `imu_clock_fit_`; calibrated offsets applied; orientation = identity w/ roll/pitch var 0.001, yaw var 99; Z accel uncalibrated |
| `~/imu/mag_raw` → `/imu/mag_raw` | `sensor_msgs/msg/MagneticField` | pub | only when a subscriber exists; µT→T; cov[0] = −1. Subs: `mag_yaw_publisher_node`, `calibrate_imu_yaw_node` |
| `~/wheel_odom` → `/wheel_odom` | `nav_msgs/msg/Odometry` | pub | `odom`→`base_link`, twist only, ~20 Hz (50 ms aggregate); vy var 1e-4 (non-holonomic), zeroed while charging (Invariant 11). Subs: fusion_graph, Nav2 `controller_server.odom_topic`, cog_to_imu, localization_monitor, scan_deskew, diagnostics, mqtt_bridge, GUI |
| `~/wheel_ticks` → `/wheel_ticks` | `mowgli_interfaces/msg/WheelTick` | pub | per firmware packet; RL/RR only. Sub: GUI (`gui/pkg/providers/ros.go` :45) |
| `~/dock_heading` → `/gnss/heading` | `sensor_msgs/msg/Imu` | pub | 1 Hz while charging, `dock_pose_yaw` as ENU quaternion, frame `base_footprint`, σ=π for `kChargingAnchorWindowSec` (5 s) after dock. **No subscriber in the tree** (see Pitfalls) |
| `~/dig_event` (**not remapped** → `/hardware_bridge/dig_event`) | `mowgli_interfaces/msg/DigEvent` | pub | `QoS(10).transient_local()`; frame `map`; `position_sigma` = GNSS accuracy. Sub: `map_server_node.cpp` :399 (also transient_local) |
| `~/cmd_vel` → `/cmd_vel` | `geometry_msgs/msg/TwistStamped` | sub | `SystemDefaultsQoS`; publisher is twist_mux `cmd_vel_out` (`mowgli.launch.py` :284; lanes in `config/twist_mux.yaml`, no `locks:`) |
| `/gps/status` | `mowgli_interfaces/msg/GnssStatus` | sub | → `gps_quality_` for the firmware LED, and the dig trust gate (`HorizontalAccuracyMeters`, `BehaviorTreeRtkFixed` from `mowgli_interfaces/gnss_status_utils.hpp`) |
| `/behavior_tree_node/high_level_status` | `mowgli_interfaces/msg/HighLevelStatus` | sub | → `current_mode_` mirrored to firmware via `LlHighLevelState` |
| `/odometry/filtered_map` | `nav_msgs/msg/Odometry` | sub | `SensorDataQoS` (best-effort; the in-code comment claims it matches fusion_graph, which actually publishes plain reliable `QoS(10)` — compatible either way); dig detector reference pose |

### Services & actions

| Name (resolves under `/hardware_bridge/`) | Type | Role | Callers |
|--------------------------------------------|------|------|---------|
| `~/mower_control` | `mowgli_interfaces/srv/MowerControl` (`mow_enabled`, `mow_direction`) | host | BT `utility_nodes.cpp` :64, `coverage_nodes.cpp` :1123; GUI `gui/pkg/api/mowglinext.go` :584; MQTT `gui/pkg/providers/mqtt.go` :120 |
| `~/emergency_stop` | `mowgli_interfaces/srv/EmergencyStop` (`emergency`) | host | BT `utility_nodes.cpp` :244; GUI `mowglinext.go` :574; MQTT `mqtt.go` :119; `ros2/src/e2e_test.py` |
| `~/reboot_board` | `std_srvs/srv/Trigger` | host | GUI `mowglinext.go` :707 (sends `LlReboot` twice) |
| `~/set_firmware_debug` | `std_srvs/srv/SetBool` | host | GUI `gui/pkg/api/diagnostics.go` :155 (`CONFIG_FLAG_FIRMWARE_DEBUG` via `LlConfigReq`) |
| `/behavior_tree_node/high_level_control` | `mowgli_interfaces/srv/HighLevelControl` | **client** | panel button 4 → `COMMAND_START`, 5 → `COMMAND_HOME` (`handle_ui_event` :2062) |

No actions. Dynamic parameters: see below.

### Wire protocol (STM32 ↔ bridge)

Frame = `0x00 | COBS(payload + CRC16-LE) | 0x00`; CRC-16/CCITT-FALSE over `type…last field`. `kMowgliProtocolVersion = 6` (`ll_datatypes.hpp` :51) must equal the firmware's `MOWGLI_PROTOCOL_VERSION`; mismatch or no `LlConfigRsp` within `fw_handshake_timeout_s_` (5 s, :3205) → `firmware_compatible=false`.

| ID | Struct (size) | Dir | Handler / sender |
|----|---------------|-----|------------------|
| 0x01 | `LlStatus` (38) | FW→host | `handle_status` |
| 0x02 | `LlImu` (41) | FW→host | `handle_imu` |
| 0x03 | `LlUiEvent` (5) | FW→host | `handle_ui_event` |
| 0x04 | `LlOdometry` (17) | FW→host | `handle_odometry` → `OdometryPublisher` |
| 0x05 | `LlBladeStatus` (16) | FW→host | `handle_blade_status` |
| 0x06 | `LlResetCause` (5) | FW→host | `handle_reset_cause` |
| 0x11 / 0x12 | `LlConfigReq` (4) / `LlConfigRsp` (8) | host→FW / FW→host | `send_config_request` / `handle_config_rsp` |
| 0x42 | `LlHeartbeat` (5) | host→FW | `send_heartbeat` (e-stop request/release bits) |
| 0x43 | `LlHighLevelState` (5) | host→FW | `send_high_level_state` (`HL_MODE_*` :75–79, `gps_quality`) |
| 0x50 | `LlCmdVel` (11) | host→FW | `send_cmd_vel_packet` |
| 0x51 | `LlCmdBlade` (5) | host→FW | `send_blade_command` |
| 0x52 | `LlReboot` (4, magic 0xB0) | host→FW | `send_reboot_command` |
| 0x54 | `LlSetDrivePid` (27) | host→FW | `send_drive_pid` |
| 0x55 | `LlSetYawPid` (21, v6 adds `gyro_bias_radps`) | host→FW | `send_yaw_pid` |
| 0x56 | `LlSetKinematics` (11) | host→FW | `send_kinematics` |
| 0x57 | `LlSetSafetyLimits` (21) | host→FW | `send_safety_limits` |

### Parameters

Layering at launch (`mowgli.launch.py` :194–254, later entries override earlier): code defaults (`declare_parameters` :330–689) ← `ros2/src/mowgli_bringup/config/hardware_bridge.yaml` (serial, rates, `imu_cal_samples`, all `dig_*`) ← dict entries from `robot_params` (= template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` deep-merged with the sparse installed file, Invariant 15). **Live-tunable** via the set-parameters callback (:434) — every one except the host-side `min_linear_vel` is re-pushed to firmware on change: `min_linear_vel`, `ticks_per_meter`, `wheel_pid_kp/ki/kd/integral_limit/pwm_per_mps`, `yaw_kp`, `yaw_ki`, `yaw_trim_limit_mps`. Everything else is read once at startup.

| Param | Declared (line) | Default source at runtime | Notes |
|-------|-----------------|---------------------------|-------|
| `serial_port`, `baud_rate` | :332–333 | `hardware_bridge.yaml`; `serial_port` also a launch arg | `/dev/mowgli`, 115200 |
| `publish_rate`, `heartbeat_rate`, `high_level_rate`, `serial_rx_timeout_s` | :334–345 | `hardware_bridge.yaml` (except `high_level_rate`, which is not in any yaml — code default) | 100 / 4 / 2 Hz / 2 s; `publish_rate` is the serial **read** tick |
| `dock_pose_x/y/yaw` | :346–348 | launch from `robot_params` (Invariant 6) | `dock_pose_yaw` is map-frame ENU |
| `wheel_track`, `ticks_per_meter` | :357, :370 | template `mowgli_robot.yaml` :32, :43 (399.0) | `ticks_per_meter` range 50–5000, pushed in `LlSetDrivePid`; `wheel_track` pushed as `wheel_base` in `LlSetKinematics` |
| `max_mps` | :379 | code default 0.5 (template :49 is **not forwarded** by `mowgli.launch.py`) | range 0.01–0.5; `LlSetKinematics`; firmware can only lower its compiled cap |
| `max_charge_voltage`, `max_charge_current`, `one_wheel_lift_emergency_ms`, `both_wheels_lift_emergency_ms`, `tilt_emergency_ms`, `stop_button_emergency_ms`, `play_button_clear_emergency_ms` | :386–392 | code defaults (template :57–63 is **not forwarded** by `mowgli.launch.py`) | `LlSetSafetyLimits`; firmware clamps so the wire can only tighten |
| `wheel_pid_kp/ki/kd/integral_limit/pwm_per_mps` | :401–408 | template :72–76 via launch | bounds `kMin/MaxRuntimeWheel*` / `*PwmPerMps` (:82–91) mirror firmware clamps |
| `yaw_kp`, `yaw_ki`, `yaw_trim_limit_mps`, `yaw_loop_enabled`, `yaw_gyro_sign` | :422–429 | template :84–91 via launch | `LlSetYawPid`; `gyro_bias_radps` comes from the IMU cal, not a param |
| `min_linear_vel` | :433 | code (0.05) | sub-deadband |vx| → 0 in `on_cmd_vel` |
| `lift_recovery_mode`, `lift_blade_resume_delay_sec` | :613–614 | launch dict (`mowgli.launch.py` :230–232) — **no template entry**, so the launch fallbacks false / 1.0 apply unless the installed file sets them (GUI schema exposes both) | blade-off-on-lift instead of emergency |
| `mowing_enabled` | :621 | template :308 via launch | dry-run inhibit (`blade_gate.hpp`), restart to change |
| `imu_cal_samples` | :623 | **launch dict from template :172 (200)** overrides `hardware_bridge.yaml` :21 (1000) | see Pitfalls |
| `imu_cal_persist_path`, `imu_cal_auto_rest_sec`, `imu_cal_periodic_recal_sec` | :631–655 | template :173–175 via launch | periodic recal effective 600 s (template), not the code's 60 |
| `dig_detect_enabled`, `dig_window_s`, `dig_min_cmd_speed`, `dig_min_wheel_dist`, `dig_progress_fraction`, `dig_max_pos_sigma`, `dig_gnss_timeout_s`, `dig_max_yaw_rate`, `dig_gyro_timeout_s`, `dig_reverse_speed`, `dig_reverse_dist`, `dig_reverse_timeout_s`, `dig_monitor_rate`, `dig_pose_timeout_s` | :671–689 | `hardware_bridge.yaml` :44–103 | not in the template / GUI; constants `dig_rearm_delay_s_` 2 s (:3038) and `dig_cmd_timeout_s_` 0.5 s (:3054) are not params |

### TF frames

Publishes **no TF**. Stamps `imu_link` (`/imu/data`, `/imu/mag_raw`), `odom`→`base_link` (`/wheel_odom`, twist only), `base_link` (`/battery_state`), `base_footprint` (`/gnss/heading`), `map` (`/hardware_bridge/dig_event`). See CLAUDE.md Invariant 2 for who owns TF.

## Build, test, run

```bash
# inside the /ros2_ws container / devcontainer (scripts assume WORKSPACE=/ros2_ws)
cd ros2 && make build-pkg PKG=mowgli_hardware          # scripts/build.sh
cd ros2 && PACKAGES=mowgli_hardware make test           # scripts/test.sh
# raw colcon equivalent
colcon build --packages-select mowgli_hardware --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test  --packages-select mowgli_hardware && colcon test-result --verbose
# run on hardware (needs /dev/mowgli)
ros2 launch mowgli_bringup mowgli.launch.py serial_port:=/dev/mowgli
```

Unit tests (all gtest, registered in `CMakeLists.txt` :114–186): `test_cobs`, `test_packet_handler`, `test_serial_port` (links `util` for `openpty`), `test_protocol`, `test_clock_fit` (link `mowgli_hardware_core`); `test_dig_detector`, `test_imu_liveness`, `test_blade_gate` (header-only, include path only). What each pins: see the Files table. No launch_testing / integration test in this package; hardware integration is exercised by `ros2/src/e2e_test.py` (calls `/hardware_bridge/emergency_stop`, watches `/wheel_odom`) against the sim's `fake_hardware_bridge_node`.

CI: `.github/workflows/ros2-ci.yml` (`colcon build` + `colcon test --return-code-on-test-failure` over the whole workspace, :338–350); `.github/workflows/protocol-version-drift.yml` runs `firmware/scripts/protocol_version_guard.py --check`, which fingerprints the wire tokens of `firmware/stm32/ros_usbnode/include/mowgli_protocol.h` (guard :31) and reads only the version constant out of `ll_datatypes.hpp` (:35), failing if the wire changed without a version bump or the two versions differ.

## Change coupling — "if you change X, also update Y"

- **Any `Ll*` struct / `PacketId` change** → bump `kMowgliProtocolVersion` (`ll_datatypes.hpp` :51) AND `MOWGLI_PROTOCOL_VERSION` (`firmware/stm32/ros_usbnode/include/mowgli_protocol.h` :59) in lockstep, update the `static_assert` sizes/offsets, `test/test_protocol.cpp`, `firmware/scripts/protocol_baseline.json` (via `protocol_version_guard.py`), and the C reference in `ros2/src/mowgli_hardware/firmware/` (currently already behind).
- **`HL_MODE_*`** are triplicated: `HighLevelStatus.msg` ↔ `hardware_bridge_node.cpp` :75–79 (the copy the bridge actually compiles) ↔ firmware `mowgli_protocol.h`. `docs/claude/commands.md` step 4 names only the two `mowgli_protocol.h` files.
- **New node parameter** → `declare_parameters()`; default in the template `mowgli_robot.yaml` (site/robot-level, Invariant 15) or `hardware_bridge.yaml` (node-local); pass-through dict in `mowgli.launch.py` (:194–254) if it comes from `robot_params`; GUI exposure needs `gui/asserts/mower_config.schema.json`, `gui/web/src/components/settings/paramCatalog.ts`, `gui/web/src/i18n/locales/{en,fr}.json` (today the catalog carries `imu_cal_*` and `yaw_*`; the schema carries `wheel_pid_*`, `mowing_enabled`, `ticks_per_meter`, `wheel_track`, `lift_*`).
- **Firmware clamp ranges** (`kMin/MaxRuntime*`, :80–101) mirror the firmware's `pid_constrain()`; change both or `ros2 param set` will accept values the board silently re-clamps.
- **`Status/Emergency/Power/DigEvent/WheelTick.msg`, `MowerControl/EmergencyStop.srv`** live in `mowgli_interfaces` → regenerate GUI bindings (`gui/pkg/msgs/mowgli/types_generated.go`, `gui/web/src/types/ros.generated.ts`; `docs/claude/commands.md`, `.github/workflows/msg-codegen-drift.yml`).
- **Topic/service names** → `mowgli.launch.py` remaps (:257–267), the consumer list above, `mowgli_simulation/src/fake_hardware_bridge_node.cpp` (hard-codes the global names), `map_server_node.cpp` :399 (`/hardware_bridge/dig_event`, transient_local on both ends).
- **`/wheel_odom` semantics** (rate, twist covariances, `odom`→`base_link`) are consumed by fusion_graph's wheel between-factor and by `controller_server.odom_topic` (`nav2_params_base.yaml`; CLAUDE.md "What NOT to Do").
- **Dig detector** ↔ `map_server` (`dig_obstacle_*` handling of `DigEvent`) ↔ firmware `ANTIDIG_*` (complementary, Invariant 16) ↔ `OdometryPublisher::tyre_travelled()` (worst-wheel measure — do not change to centre distance).
- **IMU calibration file** `# mowgli_imu_calibration_v1` (persist :1445 / load :1468): changing the layout needs a header bump, else old files parse as garbage.
- **`ticks_per_meter` / `wheel_track`** are also URDF inputs and firmware `TICKS_PER_M` fallbacks (template :32–43).

## Pitfalls

- **Param precedence:** `hardware_bridge.yaml` is first in the `parameters=[...]` list (`mowgli.launch.py` :195) and the `robot_params` dicts come after, so `imu_cal_samples` is effectively the template's 200 (~2.2 s at ~90 Hz), not the yaml's 1000/20 s, and `imu_cal_periodic_recal_sec` is 600 s, not the code default 60 (:655). Change the template, not the yaml, for those.
- `publish_rate` only sets the serial read timer; every topic is published per received packet (`handle_status` :1210, `handle_imu` :1752) or per 50 ms odom window.
- `~/imu/data_raw` is RELIABLE by design (:708 comment) and `/odometry/filtered_map` is subscribed `SensorDataQoS`; mixing them up starves fusion_graph or the dig detector.
- `~/dig_event` is **not** in the launch remaps; it resolves to `/hardware_bridge/dig_event` via the node name. Renaming the node breaks `map_server`.
- `on_cmd_vel` (:2622) promotes firmware mode NULL→AUTONOMOUS on ANY non-zero merged `cmd_vel` (documented authority leak, gated only by `fw_latched_emergency_`); while `dig_escaping_` it drops incoming commands entirely — the escape is driven from `dig_monitor_tick` (:2786) on purpose (an aborted Nav2 goal stops publishing).
- Dig trust gate = GNSS receiver accuracy under RTK-Fixed (`DigTrustSigma`); `last_map_sigma_` (major axis of the graph covariance, `on_filtered_map_odom` :2744) is log-only. Do not re-point the gate at it (Invariant 16, CLAUDE.md "What NOT to Do").
- IMU cal aborts if the wheels move or the charger drops mid-window (`handle_imu`); a window whose |accel| is not gravity or whose gyro variance is exactly 0 on all three axes is discarded and never persisted (:1615); a persisted file with all-zero covariances or |gyro offset| > 0.2 rad/s is rejected at load (:1468). `apply_imu_calibration` (:1637) also re-sends `LlSetYawPid` with the new `gyro_bias_radps`.
- Member initializers are NOT the defaults: `double yaw_kp_{0.30}` vs declared 0.12 (:422); read `declare_parameters()`.
- The file-header comment of `hardware_bridge_node.cpp` (:17–47) is stale (says `Twist`, 2 services, 5 params); `~/dock_heading` comments reference `dock_yaw_to_set_pose` (inlined into fusion_graph, `navigation.launch.py` :1094) — `/gnss/heading` has no subscriber, and `/tmp/dock_start_pose.txt` (:1919) has no reader since SLAM was removed.
- `mowgli.launch.py` :203 passes `imu_yaw`, which the node never declares (:622 comment) — it is silently ignored.
- `blade_gate.hpp` suppresses ENABLE only. Never add a path that swallows a DISABLE, and never present `mowing_enabled` as a safety interlock (CLAUDE.md Safety, Invariant 9).
- Serial: `write_all` retries EAGAIN briefly (`serial_port.cpp` :170–205); a short write or read error closes the port and the next read tick reopens it (`read_serial_tick` :990); reconnect re-arms `pid_resend_count_ = 5` and the version handshake, and resets firmware debug to OFF.
- Odometry: int32 ticks wrap at 16 bits (`unwrap_16bit` :80); |delta| > `kTickSpikeLimit` 100 is dropped (:96); `OdometryPublisher::reset()` does not zero `tyre_travelled_m_` or the WheelTick magnitudes (monotonic by design).
- `ros2/src/mowgli_hardware/firmware/` is a hand-copied C mirror at protocol v3 (missing 0x05/0x51/0x52/0x55–0x57); nothing compiles it and `protocol-version-drift.yml` does not check it. Use `firmware/stm32/ros_usbnode/` as truth.
- `max_mps`, `max_charge_*` and the `*_emergency_ms` params are declared (:379–392) but not forwarded by `mowgli.launch.py`, so template values (:49, :57–63) never reach the node unless added to the launch pass-through; only the code defaults (identical today) are pushed to the firmware.

## Generated & vendored — do not hand-edit

- Nothing in this package is generated. `ros2/src/mowgli_hardware/firmware/*.{c,h}` is a hand-maintained reference copy of the STM32 side (authoritative source: `firmware/stm32/ros_usbnode/include|src`); `ll_datatypes.hpp` is a port of OpenMower's `ll_datatypes.h`.
- Message/service types come from `ros2/src/mowgli_interfaces`; their GUI bindings (`gui/pkg/msgs/mowgli/types_generated.go`, `gui/web/src/types/ros.generated.ts`) are generated — see `docs/claude/commands.md`.
