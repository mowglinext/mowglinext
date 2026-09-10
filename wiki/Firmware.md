# Firmware: STM32 COBS Protocol

The Mowgli firmware lives in this repository at [`firmware/stm32/ros_usbnode/`](https://github.com/mowglinext/mowglinext/tree/main/firmware/stm32/ros_usbnode) (PlatformIO + STM32Cube HAL). It talks to `hardware_bridge_node` over USB CDC using a binary COBS (Consistent Overhead Byte Stuffing) wire protocol with a CRC-16 per packet, and it owns the per-wheel velocity PI loop, the gyro yaw-rate loop, the anti-dig cutout and every blade / emergency / charger safety decision.

> **History:** the firmware was migrated off the ROS1 rosserial bridge onto this COBS
> protocol; the rosserial path is gone. The comparison table below is kept because it
> explains *why* the wire looks the way it does.

**Working on the firmware?** The maintained, code-generated references are
[`firmware/CLAUDE.md`](https://github.com/mowglinext/mowglinext/blob/main/firmware/CLAUDE.md)
(conventions, build/test, gotchas) and
[`docs/claude/codemaps/firmware.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/codemaps/firmware.md)
(file map, full packet table, main-loop order, change coupling). The single source of
truth for the wire format is
[`include/mowgli_protocol.h`](https://github.com/mowglinext/mowglinext/blob/main/firmware/stm32/ros_usbnode/include/mowgli_protocol.h).

## Overview

**rosserial → COBS, and why:**

| Aspect | ROS1 (rosserial) | ROS2 (COBS) |
|--------|-----------------|-----------|
| Protocol | rosserial-over-serial | Binary COBS framing |
| Serialization | ROS message serialization | Custom C structs + CRC-16 |
| Baud rate | 57600 | 115200 (configurable) |
| Handshake | rosserial negotiation | `CONFIG_REQ`/`CONFIG_RSP` version exchange + heartbeat |
| Transport | USB serial (CDC) | USB serial (CDC) |
| Error detection | None | CRC-16/CCITT-FALSE |
| Real-time loop | Arbitrary | Cooperative timers: 100 Hz IMU, 50 Hz motor loop, 4 Hz status |

**Benefits:**

- **Minimal overhead:** COBS adds <1% overhead vs. ROS serialization's variable overhead
- **Deterministic latency:** Binary format, no variable-length serialization
- **Robust:** CRC-16 catches transmission errors
- **Simple:** No ROS message format dependencies on firmware side
- **Portable:** Works with any serial port, no ROS middleware required

## Drive Motor Control — Per-Wheel Velocity PI

The host sends a single `CMD_VEL` packet (`vx`, `wz`); the **firmware** owns the
diff-drive split and the per-wheel velocity loop. A host-side velocity loop over
the USB round-trip was tried and abandoned — it was fragile across the USB
dead-time — so the loop lives in firmware where local encoder feedback is fast.

**Source:** `firmware/stm32/ros_usbnode/src/ros/ros_custom/cpp_main.cpp`
(`on_cmd_vel`, `motors_handler`, `init_ROS`) and the vendored PX4 PID core in
`firmware/stm32/ros_usbnode/include/pid.hpp`.

- **Inverse kinematics:** `on_cmd_vel` converts (`vx`, `wz`) to per-wheel target
  speeds (`vx ± wz·wheel_base/2`), clamps them to the runtime cap
  `DRIVEMOTOR_GetMaxMps()`, and hands them to `motors_handler`. The command is
  dropped outright while the high-level mode is `IDLE`.
- **Loop:** `motors_handler` runs at `MOTORS_NBT_TIME_MS` = 20 ms (50 Hz). Per
  wheel it derives the actual speed from the signed cumulative encoder count
  (`left/right_ticks_signed`) over the 20 ms period, scaled by the runtime
  `DRIVEMOTOR_GetTicksPerMeter()`.
- **Controller:** a vendored PX4 `PID` (BSD-3, header-only `pid.hpp`,
  derivative-on-measurement, NaN/inf guards) runs as a **PI** loop: P =
  `WHEEL_PI_KP_PWM_PER_MPS` (30), I = `WHEEL_PI_KI_PWM_PER_MPS_S` (5000),
  **D = 0**, integral clamp `WHEEL_PI_INT_MAX_PWM` ±100, output clamp ±255 PWM.
- **Feedforward:** the PI trim is added on top of an open-loop feedforward
  `target_mps × g_pwm_per_mps`. The integrator bridges the brushed-DC
  static-friction deadband (~PWM 40) on sub-deadband commands, where open-loop
  alone leaves the motor buzzing without moving.
- **Anti-windup (direction-aware conditional integration):** the integrator is
  frozen only in the direction that would worsen an already-railed ±255 output;
  it is always allowed to unwind out of saturation (keyed on the error sign, not
  just the saturation bit).
- **Resets / safety:** the integrator is reset on target sign-flip, stop-to-go,
  and hard-stop (emergency or cmd_vel watchdog); a stopped wheel is forced to PWM
  0 to kill residual hum. Set `USE_WHEEL_PI 0` to fall back to open-loop
  forwarding for bring-up.

Field-validated on the robot: linear speed tracked ~0.85–1.03 of commanded.

**Nothing is persisted on the board.** The compile-time values in `board.h`
(`PWM_PER_MPS`, `TICKS_PER_M`, `MAX_MPS`, `WHEEL_BASE` — 337 / 339 / 0.5 / 0.325 on the
Yardforce 500, 275 / 277 / 0.5 / 0.325 on the 500B) are only the power-on fallback. The
live values come from `mowgli_robot.yaml` and are re-sent by `hardware_bridge_node` on
every connect via `PKT_ID_SET_DRIVE_PID` / `PKT_ID_SET_KINEMATICS`, so a tuning change
never needs a reflash. Runtime setters can only *tighten*: `max_mps` is clamped to at
most the compiled `MAX_MPS`.

## Yaw-Rate Control — Gyro Loop in Firmware (Option C)

Since **2026-07-17** the firmware also closes the **yaw-rate** loop, not just the
per-wheel speed loops. ROS2 sends `cmd_vel` straight through with no host-side shaping;
the host-side angular-rate PI that used to live in `hardware_bridge_node` was removed.

**Source:** `cpp_main.cpp` (yaw constants and rationale in the block above the yaw
globals; loop body inside `motors_handler`; runtime retune in `on_set_yaw_pid`).

- The per-wheel PIs regulate each wheel's *speed* independently, so chassis yaw was only
  their emergent difference — on soft or uneven turf the actual yaw lagged or overshot
  the commanded `wz` (the weave). This loop regulates (`commanded wz − measured gyro wz`)
  at the same 50 Hz motor cadence.
- The output is injected as a **symmetric differential velocity trim** (+right / −left)
  onto the per-wheel setpoints, so it rotates the robot without changing mean forward
  speed, and the per-wheel deadband-bridging integrators keep working.
- Defaults: `YAW_PI_KP_DEFAULT` 0.30, `YAW_PI_KI_DEFAULT` 0.40, trim clamped to
  `YAW_TRIM_LIMIT_MPS_DEFAULT` ±0.15 m/s, gyro feedback low-passed
  (`YAW_GYRO_LP_ALPHA` 0.30) and the trim slew-limited
  (`YAW_TRIM_SLEW_MPS_PER_CYCLE` 0.03) to stop the loop self-exciting a 2–4 Hz
  limit cycle against the wheel deadband.
- Gains, `enabled`, `gyro_sign` and the host-measured `gyro_bias_radps` are all
  runtime-tunable over `PKT_ID_SET_YAW_PID` (0x55) — no reflash for an A/B.
- **Fail-safe by construction:** on gyro read failure, hard-stop or `enabled=0` the trim
  is 0 and the integrator is reset, degrading to the previous open-diff behaviour. The
  hard clamp means even an inverted `gyro_sign` can only produce a bounded veer, never
  an unbounded spin.

## Anti-Dig Cutout (Firmware)

`ANTIDIG_*` in `cpp_main.cpp` (`antidig_step()`) is always active, in every mode, and
cannot be bypassed by any ROS controller — `motors_handler` drives the wheels for mowing,
transit and docking alike. Per wheel: if it is commanded to move
(`|target| > ANTIDIG_MIN_TARGET_MPS` 0.02 m/s) and the loop is really pushing
(`|PWM| > ANTIDIG_MIN_ABS_PWM` 60, i.e. the integrator has wound past the deadband) but
the encoder logs less than `ANTIDIG_PROGRESS_FRACTION` (30 %) of the travel the commanded
speed implies over `ANTIDIG_WINDOW_MS` (1500 ms), that wheel is latched off (PWM forced
to 0) until the command clears. It only ever reduces output.

This covers **blocked** wheels. **Slipping** wheels — spinning freely while the chassis
does not move — are invisible to any encoder-based check and are caught by the host-side
dig detector in `hardware_bridge_node` (see
[Architecture](Architecture)). The two cover different failures; neither substitutes for
the other.

## Files Overview

### Firmware Directory Structure

```
firmware/
├── scripts/                        # protocol_version_guard.py, board_defaults_parity.py,
│                                   # package_release.py, sync_ros_lib.py
└── stm32/
    ├── ros_usbnode/                # ← the live firmware (PlatformIO)
    │   ├── platformio.ini          # envs: Yardforce500, Yardforce500B (+ ST-Link v3 variants)
    │   ├── include/
    │   │   ├── mowgli_protocol.h   # single source of truth for the wire format
    │   │   ├── mowgli_comms.h      # RX framing / CRC / handler dispatch API
    │   │   ├── cobs.h / crc16.h    # COBS + CRC-16/CCITT-FALSE
    │   │   ├── pid.hpp             # vendored PX4 PID core (BSD-3)
    │   │   ├── board.h             # board variant, pinout, compile-time defaults
    │   │   ├── board.h.template    # Go-template twin rendered by the GUI flasher
    │   │   └── board_defaults.h    # single source of the safety defaults
    │   └── src/
    │       ├── main.c              # boot, init order, main loop, IWDG + WWDG
    │       ├── ros/ros_custom/
    │       │   └── cpp_main.cpp    # packet handlers, wheel PI, yaw loop, anti-dig,
    │       │                       # telemetry broadcasts, init_ROS()
    │       ├── mowgli_comms.c      # frame scan, CRC verify, handler table, send
    │       ├── cobs.c / crc16.c
    │       ├── drivemotor.c        # PAC5210 UART, encoders, runtime caps
    │       ├── blademotor.c / charger.c / adc.c / emergency.c / panel.c
    │       ├── imu/                # LSM6 / WT901 / MPU6050 / ICM45686 / LIS3MDL
    │       └── usbd_cdc_if.c       # USB CDC RX/TX queues and back-pressure
    ├── custom_panel_fw/            # STM32F0 replacement panel firmware
    ├── test_code/                  # pre-ROS UART-proxy bring-up firmware
    ├── mainboard_firmware/         # stock mainboard flash backup/restore (OpenOCD)
    └── panel_firmware/             # stock panel flash backup/restore (OpenOCD)
```

The host end of the same wire is `ros2/src/mowgli_hardware/` — `cobs.cpp`, `crc16.cpp`,
`packet_handler.cpp`, and `ll_datatypes.hpp`, which is the hand-kept C++ mirror of
`mowgli_protocol.h`.

## How the Link Is Implemented

The rosserial→COBS migration is complete; both ends of the wire are in this repository and
there is nothing left to port. This is how the pieces fit together.

**Frame format.** Every packet is a packed struct whose first byte is its `PKT_ID_*` and
whose last two bytes are a CRC-16 over everything before them. The struct is then
COBS-encoded and framed with `0x00` delimiters:

```
[0x00] [COBS(struct bytes + CRC-16, little-endian)] [0x00]
```

**Firmware side** (`src/mowgli_comms.c`). `mowgli_comms_process_rx()` accumulates USB CDC
bytes, splits on the `0x00` delimiters, rejects oversized frames before decoding (the
decode buffer is `MAX_RAW_PKT_SIZE` = 64 bytes), COBS-decodes, verifies the CRC, then
dispatches on the type byte
through a handler table. `mowgli_comms_send()` does the reverse: fill the CRC, COBS-encode,
frame, enqueue on the CDC TX queue.

**Registering a handler.** Each Host→Firmware packet gets a `static void on_*()` callback
registered in `init_ROS()` with `mowgli_comms_register_handler()`. There are 10
registrations against a `MOWGLI_COMMS_MAX_HANDLERS` cap of 16 — registrations past the cap
are silently dropped and show up as "firmware incompatible" forever, so check the cap when
adding one.

> Handlers run in **USB-RX interrupt context**. Do not `debug_printf` on a success path
> (it starved the main loop and tripped the windowed watchdog); take an `__disable_irq()`
> snapshot the way `motors_handler()` does.

**Loop cadences** (non-blocking timers in the cooperative main loop; `broadcast_handler()`
emits at most one packet group per pass so a delayed loop cannot stretch the watchdog
window):

| Stream | Period constant | Rate |
|--------|-----------------|------|
| IMU | `IMU_NBT_TIME_MS` 10 | 100 Hz |
| Motor loop (wheel PI + yaw + anti-dig) | `MOTORS_NBT_TIME_MS` 20 | 50 Hz |
| Odometry | per drive-controller frame | ~50 Hz |
| Status (+ reset cause) | `STATUS_NBT_TIME_MS` 250 | 4 Hz |
| Blade status | `BLADE_NBT_TIME_MS` 250 | 4 Hz |
| Panel / UI | `PANEL_NBT_TIME_MS` 100 | 10 Hz |

**Handshake and watchdogs.**

- On every (re)connect the bridge sends `PKT_ID_CONFIG_REQ`; the firmware replies with
  `PKT_ID_CONFIG_RSP` carrying `MOWGLI_PROTOCOL_VERSION` and its semantic firmware
  version. The bridge compares that against its own `kMowgliProtocolVersion` — a mismatch
  (or no reply at all, which is what pre-handshake firmware does) sets
  `firmware_compatible = false` and **blocks mowing** until the board is reflashed.
- The bridge then re-pushes `SET_DRIVE_PID` / `SET_YAW_PID` / `SET_KINEMATICS` /
  `SET_SAFETY_LIMITS`, because the board persists nothing.
- Heartbeat: the host sends `PKT_ID_HEARTBEAT` at 4 Hz (`heartbeat_rate` in
  `hardware_bridge.yaml`). Absent for more than `HEARTBEAT_TIMEOUT_MS` (2000 ms) the
  firmware asserts an emergency stop. A latch raised *only* by this watchdog, with no
  physical sensor asserted, auto-clears when heartbeats resume; a physical trigger still
  needs an explicit release.
- `cmd_vel` watchdog: no twist for 200 ms → hard stop; for 25 s → blade off as well.
- `main_eOpenmowerStatus` boots to `IDLE`. Until the host sends an `HL_STATE` other than
  `HL_MODE_IDLE`, `cmd_vel` is dropped and the blade target is forced to 0.

## Packet Structure Reference

Authoritative source:
[`firmware/stm32/ros_usbnode/include/mowgli_protocol.h`](https://github.com/mowglinext/mowglinext/blob/main/firmware/stm32/ros_usbnode/include/mowgli_protocol.h).
Every struct is `#pragma pack(push,1)` and pinned by a `_Static_assert` on its size (and,
for the runtime-tuning packets, on every field offset). The C++ mirror on the host is
`ros2/src/mowgli_hardware/include/mowgli_hardware/ll_datatypes.hpp`; the two must be
changed together.

Current wire version: `MOWGLI_PROTOCOL_VERSION` **6**.

### Firmware → Host

| ID | Struct | Bytes | Contents |
|----|--------|-------|----------|
| `0x01` | `pkt_status_t` | 38 | `status_bitmask`, `uss_ranges_m[5]`, `emergency_bitmask`, `v_charge`, `v_system`, `charging_current`, `batt_percentage` (always 0 today) |
| `0x02` | `pkt_imu_t` | 41 | `dt_millis`, `acceleration_mss[3]` (m/s²), `gyro_rads[3]` (rad/s), `mag_uT[3]` (µT) |
| `0x03` | `pkt_ui_event_t` | 5 | `button_id` (1=S1, 2=S2, 3=LOCK, 4=START, 5=HOME), `press_duration` (0=short) |
| `0x04` | `pkt_odometry_t` | 17 | `dt_millis`, signed cumulative `left_ticks`/`right_ticks`, firmware-computed `left/right_velocity_mm_s` |
| `0x05` | `pkt_blade_status_t` | 16 | `is_active`, `rpm`, `power_watts`, `temperature`, `error_count` |
| `0x06` | `pkt_reset_cause_t` | 5 | `reset_cause` (`RESET_CAUSE_*`), `last_stage_before_reset` (WWDG breadcrumb) |
| `0x12` | `pkt_config_rsp_t` | 8 | `protocol_version`, `active_flags`, `fw_version_{major,minor,patch}` |

### Host → Firmware

| ID | Struct | Bytes | Contents |
|----|--------|-------|----------|
| `0x11` | `pkt_config_req_t` | 4 | `flags` (`CONFIG_FLAG_FIRMWARE_DEBUG`); sent on every (re)connect |
| `0x42` | `pkt_heartbeat_t` | 5 | `emergency_requested`, `emergency_release_requested` |
| `0x43` | `pkt_hl_state_t` | 5 | `current_mode` (`HL_MODE_*`), `gps_quality` (0-100) |
| `0x50` | `pkt_cmd_vel_t` | 11 | `linear_x` (m/s), `angular_z` (rad/s) |
| `0x51` | `pkt_cmd_blade_t` | 5 | `blade_on`, `blade_dir` — fire-and-forget; firmware decides |
| `0x52` | `pkt_reboot_t` | 4 | `magic` must equal `PKT_REBOOT_MAGIC` (0xB0) |
| `0x54` | `pkt_set_drive_pid_t` | 27 | `ticks_per_meter`, `kp`, `ki`, `kd`, `integral_limit`, `pwm_per_mps` |
| `0x55` | `pkt_set_yaw_pid_t` | 21 | `yaw_kp`, `yaw_ki`, `trim_limit_mps`, `enabled`, `gyro_sign`, `gyro_bias_radps` |
| `0x56` | `pkt_set_kinematics_t` | 11 | `max_mps` (clamped ≤ compiled `MAX_MPS`), `wheel_base` |
| `0x57` | `pkt_set_safety_limits_t` | 21 | charge V/I ceiling (lower-only) + four emergency trip timeouts (shorten-only) and `play_clear_ms` (lengthen-only) |

Every struct starts with a `uint8_t type` holding its packet ID and ends with a
`uint16_t crc`; the byte counts above include both.

### Bitmasks and mode constants

```c
/* pkt_status_t::status_bitmask                 (bit 3 is reserved) */
#define STATUS_BIT_INITIALIZED  (1u << 0u)
#define STATUS_BIT_RASPI_POWER  (1u << 1u)
#define STATUS_BIT_CHARGING     (1u << 2u)
#define STATUS_BIT_RAIN         (1u << 4u)
#define STATUS_BIT_SOUND_AVAIL  (1u << 5u)
#define STATUS_BIT_SOUND_BUSY   (1u << 6u)
#define STATUS_BIT_UI_AVAIL     (1u << 7u)

/* pkt_status_t::emergency_bitmask */
#define EMERGENCY_BIT_LATCH     (1u << 0u)
#define EMERGENCY_BIT_STOP      (1u << 1u)
#define EMERGENCY_BIT_LIFT      (1u << 2u)

/* pkt_hl_state_t::current_mode — MUST match
 * mowgli_interfaces/msg/HighLevelStatus.msg HIGH_LEVEL_STATE_* */
#define HL_MODE_NULL           0u  /* emergency or transitional */
#define HL_MODE_IDLE           1u  /* idle, docked, charging     */
#define HL_MODE_AUTONOMOUS     2u  /* autonomous mowing          */
#define HL_MODE_RECORDING      3u  /* area boundary recording    */
#define HL_MODE_MANUAL_MOWING  4u  /* manual teleop with blade   */
```

## Build & Flash

Most users never build the firmware: the GUI's **Flash Board** step defaults to the
**prebuilt** source. It fetches
`https://github.com/mowglinext/mowglinext/releases/latest/download/manifest.json`, picks
the binary matching the selected board + panel, sha256-verifies the download, flashes it
over an ST-Link with `openocd … program … verify reset exit`, and then re-reads the
firmware handshake to confirm the reported protocol/firmware version matches the manifest
entry. Because every tuning value is pushed over the wire at runtime, **changing a PID
gain, the wheel base or a safety limit never requires a reflash** — only a wire-format
change does.

The expert path renders `board.h.template` from the GUI form and builds from source:

```bash
cd firmware/stm32/ros_usbnode

pio run                                      # default env: Yardforce500 (STM32F103VC)
pio run -e Yardforce500B                     # STM32F401VC
pio run -e Yardforce500 -t upload            # ST-Link v2 (upload_protocol = stlink)
pio run -e Yardforce500_STLINK_V3 -t upload  # OpenOCD custom upload command
pio run -t swo_viewer                        # SWO/ITM trace target (500B debug output)
pio device monitor -b 115200 -p /dev/ttyAMA0 # UART debug text (Yardforce500 only)
```

> **Remove the blades before any bench work.** A flash reboots the board with the motors
> powered, and the bench harness has no tilt sensing.

## Testing & Validation

### Protocol Guards

There are no firmware-side unit tests (`test/` is PlatformIO's placeholder). What actually
pins the wire behaviour, all runnable from the repo root without a toolchain:

```bash
# Wire fingerprint vs the committed baseline; fails on un-versioned drift or on a
# firmware/host protocol-version mismatch. CI: protocol-version-drift.yml
python3 firmware/scripts/protocol_version_guard.py --check

# board.h / board.h.template must consume board_defaults.h and never redefine a
# managed safety macro. CI: firmware-ci.yml `defaults-parity`
python3 firmware/scripts/board_defaults_parity.py
```

The host-side mirror is covered by `ros2/src/mowgli_hardware/test/test_protocol.cpp`
(`ProtocolSizes`, `ProtocolIds`, `OdometryPacket.*`, `SetDrivePidPacket.*`,
`StatusBitmask`, `EmergencyBitmask`), run with `cd ros2 && make test`. The PI loop, the
yaw loop and the anti-dig cutout are validated by a supervised manual field procedure only
— it is written out in the comment above `antidig_step()` in `cpp_main.cpp`.

### Hardware Test with ROS2 Stack

```bash
# 1. Bring up the stack
ros2 launch mowgli_bringup mowgli.launch.py serial_port:=/dev/mowgli

# 2. Monitor incoming packets (~4 Hz; firmware_compatible must be true)
ros2 topic echo /hardware_bridge/status

# 3. Send a velocity command — the bridge subscribes to TwistStamped
ros2 topic pub /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.1}, angular: {z: 0.0}}}"

# 4. Verify motor response (should move forward slowly)

# 5. Debug serial with raw bytes
timeout 5 cat /dev/mowgli | xxd
```

`/cmd_vel` is normally twist_mux's output, so stop the navigation stack (or publish on a
mux input instead) before driving it by hand.

### Common Issues

**Issue 1: Packets not received**

- Check the serial port (`/dev/mowgli`) and that the bridge holds it open — the firmware
  deliberately stops transmitting telemetry when no host has the CDC port open
- Verify the board enumerates (VID `0x0483`, PID `0x5740`, product string "Mowgli")

**Issue 2: CRC mismatch**

- Verify the algorithm: CRC-16/CCITT-FALSE, polynomial `0x1021`, init `0xFFFF`
- CRC is computed over the packed struct *excluding* its own 2-byte `crc` field, stored
  little-endian, and only then COBS-encoded
- Check the struct sizes against the `_Static_assert`s in `mowgli_protocol.h`

**Issue 3: COBS framing errors**

- Verify start/end delimiters (`0x00`)
- Check that no `0x00` bytes appear in the encoded payload
- Frames larger than the RX size guard are dropped before decoding

**Issue 4: Motors not responding to cmd_vel**

- The firmware ignores `cmd_vel` while the high-level mode is `HL_MODE_IDLE` — the host
  must send an `HL_STATE` packet with a non-idle mode first
- Check emergency stop status (a latch requires an explicit release, unless it was raised
  by the heartbeat watchdog alone)
- Verify the heartbeat is arriving; a gap over 2 s makes the firmware assert an emergency
- A twist gap over 200 ms hard-stops the wheels; the anti-dig cutout may also have latched
  a wheel off

**Issue 5: "Firmware incompatible — reflash required"**

- The firmware's `MOWGLI_PROTOCOL_VERSION` does not match the image's
  `kMowgliProtocolVersion`, or the board never answered the config request at all. Mowing
  stays blocked until the board is reflashed with a matching build.

## Changing the Wire Protocol — Checklist

Adding or changing a packet touches both ends of the wire and the
`protocol-version-drift` CI gate:

- [ ] Edit the struct / `PKT_ID_*` in `firmware/.../include/mowgli_protocol.h`, keeping
      the `_Static_assert` on its size (and field offsets, for tuning packets)
- [ ] Bump `MOWGLI_PROTOCOL_VERSION`
- [ ] Mirror the struct **and** `kMowgliProtocolVersion` into
      `ros2/src/mowgli_hardware/include/mowgli_hardware/ll_datatypes.hpp`
- [ ] For a new Host→Firmware packet, add a `static void on_*()` handler and register it
      in `init_ROS()` — check it fits under `MOWGLI_COMMS_MAX_HANDLERS`
- [ ] Update the sizes/offsets in `ros2/src/mowgli_hardware/test/test_protocol.cpp`
- [ ] Refresh the baseline: `python3 firmware/scripts/protocol_version_guard.py`
- [ ] If `HL_MODE_*` changed, update `HighLevelStatus.msg` and the bridge's local copy
- [ ] Remember every existing board now reports "incompatible" until it is reflashed

## References

- **COBS Algorithm:** https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
- **CRC-16 CCITT:** https://en.wikipedia.org/wiki/Cyclic_redundancy_check
- **STM32 USB CDC:** STM32CubeMX HAL documentation
- **ROS2 Message Types:** https://index.ros.org/doc/ros2/

---

**Next:** see [Architecture](Architecture) for where the board sits in the whole stack, and [Configuration](Configuration) for the `mowgli_robot.yaml` values the bridge pushes down to it.
