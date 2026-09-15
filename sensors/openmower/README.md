# OpenMower hardware bridge (`HARDWARE_BACKEND=openmower`)

Runs MowgliNext on **stock OpenMower v1 electronics** — the LowLevel (Raspberry
Pi Pico) mainboard firmware and the xESC motor controllers — **without
reflashing anything**. The `mowgli-openmower` sidecar container speaks the
OpenMower wire protocols on the Pi's UARTs and publishes the exact
`/hardware_bridge` contract the rest of the stack expects, so `mowgli-ros2`
simply skips its own `hardware_bridge_node` (`mowgli.launch.py` gates it on
`HARDWARE_BACKEND=mowgli`). Everything above the bridge — fusion_graph, Nav2,
the behaviour tree, the GUI — is unchanged.

```
 ┌─────────────── mowgli-openmower (this image) ───────────────┐
 │ openmower_bridge_node  (ROS node name: hardware_bridge)      │
 │   LowLevel link  ──COBS/CRC16──▶ /dev/ttyAMA0  Pico board     │
 │   left  xESC link ──────────────▶ /dev/ttyAMA5                │
 │   right xESC link ──────────────▶ /dev/ttyAMA3                │
 │   mow   xESC link ──────────────▶ /dev/ttyAMA4                │
 └──────────────────────────────────────────────────────────────┘
        ▲ /cmd_vel  │ /hardware_bridge/{status,emergency,power}
        │           │ /battery_state /imu/data /wheel_odom /wheel_ticks
        │           ▼ services mower_control, emergency_stop
   twist_mux      BT · fusion_graph · Nav2 · GUI   (mowgli-ros2, unchanged)
```

GNSS is **not** part of this backend: the OpenMower GPS is a plain u-blox
receiver and stays with the Universal GNSS sidecar (`GNSS_STACK=universal`).

## Selecting it

```bash
curl -sSL https://mowgli.garden/install.sh | bash -s -- --backend=openmower --lidar=none
```

or pick **[3] OpenMower electronics** in the installer's backend menu. The
installer then asks for the ESC type (`OM_MOWER_ESC_TYPE` in OpenMower terms):

| Choice | `OPENMOWER_XESC_TYPE` | Hardware |
|--------|-----------------------|----------|
| xESC mini | `xesc_mini` | STM32 ESC, VESC protocol — stock YardForce builds |
| xESC 2040 | `xesc_2040` | RP2040 ESC, COBS protocol |

The Rev4 motor adapter (`xesc_yfr4`) is not supported yet.

Device paths default to OpenMower's own for kernels ≥ 6.1.28 and can be
overridden in `docker/.env`:

| Key | Default | Meaning |
|-----|---------|---------|
| `OPENMOWER_LL_PORT` | `/dev/ttyAMA0` | LowLevel board UART, 115200 |
| `OPENMOWER_XESC_LEFT_PORT` | `/dev/ttyAMA5` | left drive ESC |
| `OPENMOWER_XESC_RIGHT_PORT` | `/dev/ttyAMA3` | right drive ESC |
| `OPENMOWER_XESC_MOW_PORT` | `/dev/ttyAMA4` | mow ESC |
| `OPENMOWER_IMAGE` | `ghcr.io/<owner>/<repo>/openmower:<tag>` | this image |

The installer enables `uart1…uart5` overlays and disables Bluetooth, which is
what OpenMowerOS does too. On an older kernel (< 6.1.28) OpenMower used
`ttyAMA4/2/3` for left/right/mow — set the keys above accordingly.

## Per-robot values (sparse `mowgli_robot.yaml`)

The launch file reads the operator's sparse config directly (it is mounted
read-only at `/config`) for the keys below; anything absent keeps the package
default from `mowgli_openmower_bridge/config/openmower_bridge.yaml`.

| Key | Note |
|-----|------|
| `ticks_per_meter` | xESC hall ticks per metre — **1600** on a YardForce 500 (OpenMower `OM_WHEEL_TICKS_PER_M`). The installer seeds it when the file still holds the Mowgli seed 399.0. |
| `wheel_track` | 0.325 m (OpenMower `OM_WHEEL_DISTANCE_M`) |
| `max_mps`, `mowing_enabled` | as on the Mowgli backend |
| `max_charge_voltage`, `max_charge_current`, `battery_full_voltage`, `battery_empty_voltage` | forwarded to the LowLevel board's config packet (charge / battery protection) |
| `one_wheel_lift_emergency_ms`, `both_wheels_lift_emergency_ms` | LowLevel tilt / lift periods |
| `openmower_wheel_loop_enabled`, `openmower_wheel_duty_per_mps`, `openmower_wheel_kp`, `openmower_wheel_ki` | host-side wheel velocity loop (below) |
| `openmower_blade_duty` | mow ESC duty when the blade is on (default 1.0) |
| `openmower_emergency_input_config` | OpenMower `OM_EMERGENCY_INPUT_CONFIG`, e.g. `!L,!L,S,S`; empty keeps the board's compiled hall configuration |

`install/scripts/migrate_openmower.py` imports the OpenMower map and datum;
`OM_WHEEL_TICKS_PER_M` / `OM_WHEEL_DISTANCE_M` still have to be checked by hand.

## What the bridge does

| Surface | Source |
|---------|--------|
| `/hardware_bridge/status` | LowLevel `ll_status` bits (initialized, charging, rain, UI board) + mow ESC telemetry (rpm, current, temperatures). `firmware_compatible` is **true only when the LowLevel stream is live and both drive ESCs are connected** — PreFlightCheck refuses to mow otherwise. `firmware_version` names the ESC type and firmware versions. |
| `/hardware_bridge/emergency` | LowLevel emergency bitmask (STOP button, lift/tilt, latch) plus the host request; reasons match the STM32 bridge |
| `/hardware_bridge/power`, `/battery_state` | `v_charge`, `v_system`, `charging_current`, SoC; current is `abs(charge)` while charging, else 0 (docking convention) |
| `/imu/data`, `/imu/mag_raw` | LowLevel `ll_imu`, frame `imu_link`, at-rest gyro/accel bias removed after a docked or 15 s stationary calibration |
| `/wheel_odom`, `/wheel_ticks` | xESC tachometers (signed, right side mirrored), 50 ms windows, zeroed while charging |
| `/cmd_vel` | twist_mux output → per-wheel targets → **host-side PI + feed-forward** on the tachometer → xESC duty at 50 Hz. Same slew limits as `hardware_bridge.yaml`. |
| `mower_control` | mow ESC duty ±`blade_duty` (direction as OpenMower: 1 = +, 0 = −) |
| `emergency_stop` | asks the LowLevel board to latch / release through the heartbeat, exactly as `mower_comms_v1` |
| `reboot_board`, `set_firmware_debug` | exist so GUI calls fail with a reason; the Pico has neither feature |
| `/hardware_bridge/dig_escalated` | always `false` — the wheel-slip dig detector is not part of this backend |
| Panel buttons | CoverUI HOME (2) / PLAY (3) → `high_level_control`; very long LOCK (6) → reset emergency |

### Wheel velocity loop

OpenMower's ESCs only accept a duty cycle, and OpenMower's own comms node
maps m/s to duty 1:1 open loop. MowgliNext's controllers expect commanded
velocities to be tracked (the Mowgli STM32 does it in firmware), so the bridge
closes a PI loop per wheel on the tachometer, with `wheel_duty_per_mps` as
feed-forward. `openmower_wheel_loop_enabled: false` reverts to the OpenMower
mapping. Tune `openmower_wheel_kp` / `openmower_wheel_ki` on the lawn; the
defaults (0.5 / 2.0, integral clamp 0.3 duty) are deliberately soft.

## Safety model — read before mowing

On the Mowgli STM32 backend the firmware is the sole blade / e-stop authority
and refuses actuation on its own. **OpenMower's controllers have no such
policy: whoever sends duty spins the motor.** The LowLevel board owns the
physical inputs (stop buttons, lift/tilt halls) and a latch, but stopping the
motors is up to the host — the same trust model OpenMower's own stack runs
with. This bridge therefore:

- commands **zero duty to every ESC** whenever the LowLevel board reports an
  emergency, the host requested one, the LowLevel stream is stale, or
  `/cmd_vel` is older than `cmd_vel_timeout_s` (1 s); the blade additionally
  needs a `HighLevelStatus` younger than `high_level_status_timeout_s` (5 s),
  so a dead behaviour tree cannot leave it spinning on its last reported mode;
- spins the blade only in `AUTONOMOUS` / `MANUAL_MOWING` with `mowing_enabled`
  and no emergency (`blade_policy.hpp`), and holds the wheels in `IDLE` like
  the STM32 firmware does (that is what lets `DigObstructionGuard` stop the
  robot);
- relies on the xESC firmware watchdog (`FAULT_WATCHDOG`) to stop a motor if
  the bridge itself dies — control packets stop, the ESC coasts.

Any change to `blade_policy.hpp`, the emergency tracker or the actuation tick
is **safety-critical**; treat it as such in review.

## Not covered (yet)

- Wheel-slip dig detection and the repeat-dig escalation (CLAUDE.md
  Invariant 16) — needs the firmware anti-dig backstop the Pico lacks.
- `xesc_yfr4` (Rev4 motor adapter) and OpenMower **v2** mainboards
  (xbot_framework over Ethernet).
- Lift-recovery mode (`lift_recovery_mode`) — a lift is a full emergency here.
- IMU bias persistence across container restarts (re-learned after 15 s at
  rest or on the next dock).
- Field validation: this backend has been exercised against the protocol
  definitions and in simulation only. Bench it with the wheels off the ground
  first (`mowing_enabled: false`).

## Build · test

```bash
# image — build context is the repo root
docker build -t mowgli-openmower -f sensors/openmower/Dockerfile .

# unit tests (47 gtest cases) — same recipe as .github/workflows/sensors-openmower.yml
mkdir -p /ws/src && cp -r ros2/src/mowgli_interfaces ros2/src/mowgli_hardware \
  sensors/openmower/mowgli_openmower_bridge /ws/src/
colcon build --packages-skip mowgli_openmower_bridge --packages-up-to mowgli_openmower_bridge \
  --cmake-args -DBUILD_TESTING=OFF          # mowgli_hardware's tests need the firmware tree
colcon build --packages-select mowgli_openmower_bridge --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select mowgli_openmower_bridge && colcon test-result --verbose
```

Protocol references: `open_mower_ros/src/mower_comms_v1` (LowLevel packets,
`ll_high_level_config`), `xesc_ros/xesc_2040_driver` and
`xesc_ros/vesc_driver` (xESC framing and the GET_VALUES layout).
