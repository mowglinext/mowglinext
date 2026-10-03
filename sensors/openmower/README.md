# OpenMower v1 hardware bridge (`HARDWARE_BACKEND=openmower`)

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

## v1 only — v2 is a different machine

OpenMower has two hardware generations and `open_mower_ros` ships a separate
comms node for each, selected by `HARDWARE_PLATFORM` in
`launch/include/_comms.launch`:

| | v1 (`HARDWARE_PLATFORM=1`) | v2 (`HARDWARE_PLATFORM=2`) |
|---|---|---|
| Pi-side node | `mower_comms_v1` | `mower_comms_v2` |
| Mainboard | LowLevel, RP2040 Pico | `fw-openmower-v2` on hw-openmower-yardforce / -sabo |
| Transport | UART, COBS frames + CRC-16 | `xbot_framework` service interfaces (Ethernet/UDP) |
| Motors | xESC mini / xESC 2040 on their own UARTs | driven through the mainboard's DiffDrive service |
| This bridge | **supported** | **not supported** |

**Only v1 is supported for now.** v2 is not a variant of the same protocol: it replaces the serial link with a service-oriented framework
(`BmsServiceInterface`, `DiffDriveServiceInterface`, `EmergencyServiceInterface`,
`ImuServiceInterface`, …) over the network, so none of `vesc_protocol.hpp`,
`xesc_2040_protocol.hpp` or the LowLevel packet handling applies. What it does
reuse is the ROS-facing half of this node — the publishers, the blade and
emergency policy, the wheel loop and the Status/Emergency/Power projection —
which is why those pieces are already split out of the serial code.

A v2 robot that selects `--backend=openmower` gets a bridge
that opens UARTs nothing answers on: `firmware_compatible` stays false,
PreFlightCheck refuses to mow, and the wheels never move. That is a visible,
safe failure rather than a silent half-working stack.

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
| `openmower_docked_charge_voltage` | charge-contact voltage above which the robot is "on the dock" (default 10 V, OpenMower `mower_logic`'s own rule) |
| `openmower_ll_v_charge_cutoff`, `openmower_ll_i_charge_cutoff`, `openmower_ll_v_battery_cutoff`, `openmower_ll_v_battery_empty`, `openmower_ll_v_battery_full` | the LowLevel board's charge / battery protection. **Absent = the board keeps its own** (30 V, 1.5 A, 29 V, 22.0 V, 28.4 V on v1-fw) |
| `openmower_ll_lift_period_ms`, `openmower_ll_tilt_period_ms` | the board's lift (≥ 2 wheels) / tilt (1 wheel) e-stop delays. **Absent = the board keeps its own** (100 ms / 2500 ms) |
| `openmower_wheel_loop_enabled`, `openmower_wheel_duty_per_mps`, `openmower_wheel_kp`, `openmower_wheel_ki` | host-side wheel velocity loop (below) |
| `openmower_blade_duty` | mow ESC duty when the blade is on (default 1.0) |
| `openmower_emergency_input_config` | OpenMower `OM_EMERGENCY_INPUT_CONFIG`, e.g. `!L,!L,S,S`; empty keeps the board's compiled hall configuration |

**The STM32 keys are deliberately NOT forwarded to the board.**
`max_charge_voltage`, `battery_*_voltage` and `*_lift_emergency_ms` describe
the Mowgli STM32, which clamps every value it receives toward the safer side.
The Pico does not clamp, and it saves what it receives to flash. An earlier
version of this bridge forwarded them, and the template's
`both_wheels_lift_emergency_ms` (1000) replaced the board's 100 ms lift period:
a lifted robot would have taken ten times longer to e-stop, and the setting
would have survived a return to OpenMower. Every LowLevel field is now sent as
"undefined" unless an `openmower_ll_*` key sets it, and
`lowlevel_config.hpp`'s `BuildHighLevelConfig` test pins that.

`install/scripts/migrate_openmower.py` imports the OpenMower map and datum;
`OM_WHEEL_TICKS_PER_M` / `OM_WHEEL_DISTANCE_M` still have to be checked by hand.

## What the bridge does

| Surface | Source |
|---------|--------|
| `/hardware_bridge/status` | LowLevel `ll_status` bits (initialized, rain, UI board) + mow ESC telemetry (rpm, current, temperatures). `is_charging` means **on the dock** and comes from the charge-contact voltage (`power_semantics.hpp`), not from status bit 2 (see *Firmware facts* below). `firmware_compatible` is **true only when the LowLevel stream is live and both drive ESCs are connected** — PreFlightCheck refuses to mow otherwise. `firmware_version` names the ESC type and firmware versions. |
| `/hardware_bridge/emergency` | LowLevel emergency bitmask (STOP button, lift/tilt, latch) plus the host request; reasons match the STM32 bridge |
| `/hardware_bridge/power`, `/battery_state` | `v_charge`, `v_system`, `charging_current`, SoC; current is `abs(charge)` while docked, else 0 (docking convention). `charger_enabled` is the board's charge relay; a docked robot whose relay opened (full battery) reads `docked, not charging` / `POWER_SUPPLY_STATUS_NOT_CHARGING` and stays docked. |
| `/imu/data`, `/imu/mag_raw` | LowLevel `ll_imu`, frame `imu_link`. The at-rest gyro/accel bias is removed by a calibration that is armed on every dock visit and starts once the wheels have been still for 1 s; an attempt aborted by the robot creeping onto the contacts is retried. Off the dock it runs after 15 s at rest if none has completed yet. |
| `/wheel_odom`, `/wheel_ticks` | xESC tachometers (signed, right side mirrored), 50 ms windows, zeroed while docked. A tick jump no wheel could make (> 2 m/s) is a controller reset, so odometry re-primes instead of publishing it (`IsPlausibleTickDelta`). |
| `/cmd_vel` | twist_mux output → per-wheel targets → **host-side PI + feed-forward** on the tachometer → xESC duty at 50 Hz. Same slew limits as `hardware_bridge.yaml`. |
| `mower_control` | mow ESC duty ±`blade_duty` (direction as OpenMower: 1 = +, 0 = −) |
| `emergency_stop` | asks the LowLevel board to latch through the heartbeat (re-asserted every heartbeat while active). A **release** is sent on the next 3 heartbeats only, and dropped as soon as the board reports a stop/lift still asserted. This deliberately differs from `mower_comms_v1`, which keeps asking: the v1 firmware clears its latch on any release bit, so a pending release made the latch drop by itself the moment a held stop button was let go. |
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
  emergency, the host requested one, no LowLevel status arrived for
  `ll_status_timeout_s` (0.5 s — while it is missing a stop button or a lift
  cannot be seen), or `/cmd_vel` is older than `cmd_vel_timeout_s` (1 s); the blade additionally
  needs a `HighLevelStatus` younger than `high_level_status_timeout_s` (5 s),
  so a dead behaviour tree cannot leave it spinning on its last reported mode;
- spins the blade only in `AUTONOMOUS` / `MANUAL_MOWING` with `mowing_enabled`
  and no emergency (`blade_policy.hpp`), and holds the wheels in `IDLE` like
  the STM32 firmware does (that is what lets `DigObstructionGuard` stop the
  robot);
- relies on the controllers' own watchdogs if the bridge itself dies: the
  xESC 2040 stops after 500 ms without a control packet (`FAULT_WATCHDOG`),
  the xESC mini coasts after 1000 ms (VESC `APPCONF_TIMEOUT_MSEC`), and the
  LowLevel board latches its emergency after 500 ms without a heartbeat. The
  simulation kills the bridge with SIGKILL mid-mow (no destructor, no last
  zero) and measures all three;
- never changes the board's own safety configuration unless the operator sets
  an `openmower_ll_*` key (see above).

Any change to `blade_policy.hpp`, the emergency tracker or the actuation tick
is **safety-critical**; treat it as such in review.

## Not covered (yet)

- Wheel-slip dig detection and the repeat-dig escalation (CLAUDE.md
  Invariant 16) — needs the firmware anti-dig backstop the Pico lacks.
- `xesc_yfr4` (Rev4 motor adapter). OpenMower **v2** mainboards have their own
  section above — they need a separate sidecar, not a flag here.
- Lift-recovery mode (`lift_recovery_mode`) — a lift is a full emergency here.
- IMU bias persistence across container restarts (re-learned after 15 s at
  rest or on the next dock).
- IMU mounting: the emulated IMU is aligned with `base_link`; a real board's
  mounting is calibrated like on Mowgli (`imu_yaw`).
- Field validation: this backend has been exercised against the firmware
  sources and in simulation only. Bench it with the wheels off the ground
  first (`mowing_enabled: false`).

## Firmware facts this bridge relies on

Read from the hardware's own sources, and reproduced by the emulator in `sim/`:

| Fact | Source |
|------|--------|
| The board boots latched; > 500 ms without a heartbeat re-latches | `OpenMower` `Firmware/LowLevel/src/main.cpp` (branch `v1-fw`), `HEARTBEAT_MILLIS`, `updateEmergency` |
| A release bit clears the latch unconditionally; a still-asserted trigger re-latches on the next loop | same, heartbeat handler + `updateEmergency` |
| Emergency bits: latch 0, stop 1, lift 2 (v1-fw). v0.13.x put stops on 1–2 and lifts on 3–4: still an emergency here (any bit zeroes the motors), with a less precise reason | `datatypes.h` on `v1-fw` vs tag `v0.13.2` |
| Status bit 2 is the charge **relay** (`charging_allowed`), not "docked": 1 off the dock on v0.13.x, 0 on a full-battery dock on both | `updateChargingEnabled` on `v1-fw` and `v0.13.2` |
| `applyConfig` starts from the board's defaults and takes over only defined fields, then saves to flash | `applyConfig`, `saveConfigToFlash` (`v1-fw`) |
| xESC 2040: status every 20 ms, `FAULT_UNINITIALIZED` until settings, 500 ms watchdog, `direction = hall_diff < 0` | `ClemensElflein/xESC2040` `firmware/src/{main.cpp,config.h}` |
| xESC mini: coasts after 1000 ms without a command | `ClemensElflein/xesc_firmware` `applications/appconf_default.h` |
| Right ESC is wired mirrored: forward is a negative duty | `open_mower_ros` `mower_comms_v1.cpp` (`setDutyCycle(-speed_r)`) |

## Simulation

`sim/run_simulation.py` runs the real bridge, through its production launch
file, against `sim/openmower_v1_emulator.py`: four pseudo-terminals emulating
the LowLevel board and three xESC controllers with the firmware rules above,
plus a differential-drive plant with deliberately imperfect motors (0.85 m/s
per unit duty, a deadband, a 120 ms lag) so an open-loop bridge visibly misses
its target. It drives the bridge over ROS like the stack does and checks what
the emulated hardware received, timed on the emulator's clock: boot handshake
and the config the board keeps, closed-loop speed and yaw rate, the
acceleration limit, cmd_vel / behaviour-tree / LowLevel / controller timeouts,
stop button, lift and tilt, software e-stop, the release-while-held case, blade
direction and gates, docking with a full battery and IMU calibration, the
v0.13 charging bit, panel buttons, a controller brown-out mid-drive, and a
SIGKILL of the bridge mid-mow. Both controller types are run.

`sim/run_stack_integration.py` adds the real `mowgli_bringup/launch/mowgli.launch.py`:
exactly one `/hardware_bridge` with `HARDWARE_BACKEND=openmower`, a command on
the twist_mux teleop lane reaching the emulated wheels, the STM32 bridge still
starting for `mowgli`, and an unknown backend aborting the launch.

Writing the emulator from the firmware sources found five bridge bugs that the
unit tests could not see; all are fixed and now covered by both layers:

1. **Charging bit read as "docked"** — would zero `/wheel_odom` while mowing on v0.13.x firmware.
2. **STM32 template values written to the board's flash** — a 10× slower lift e-stop.
3. **Release request kept armed while a stop button was held** — the latch dropped by itself on release.
4. **2 s blind window after a LowLevel link loss** — now 0.5 s.
5. **A controller reboot published as a 2.2e7 m/s odometry spike**, and a single creeping tick at dock contact skipping that visit's IMU calibration.


## Build · test

```bash
# image — build context is the repo root
docker build -t mowgli-openmower -f sensors/openmower/Dockerfile .

# unit tests — same recipe as .github/workflows/sensors-openmower.yml
mkdir -p /ws/src && cp -r ros2/src/mowgli_interfaces ros2/src/mowgli_hardware \
  sensors/openmower/mowgli_openmower_bridge /ws/src/
colcon build --packages-skip mowgli_openmower_bridge --packages-up-to mowgli_openmower_bridge \
  --cmake-args -DBUILD_TESTING=OFF          # mowgli_hardware's tests need the firmware tree
colcon build --packages-select mowgli_openmower_bridge --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select mowgli_openmower_bridge && colcon test-result --verbose

# simulation (needs the workspace above + mowgli_bringup for the stack check)
python3 sensors/openmower/sim/run_simulation.py            # both controller types
python3 sensors/openmower/sim/run_stack_integration.py
```

Protocol references: `open_mower_ros/src/mower_comms_v1` (LowLevel packets,
`ll_high_level_config`), `xesc_ros/xesc_2040_driver` and
`xesc_ros/vesc_driver` (xESC framing and the GET_VALUES layout).
