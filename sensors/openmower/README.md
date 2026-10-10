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

Device paths default to OpenMower's own for kernels ≥ 6.1.28. The installer
writes its choice to `docker/.env`; after that they are changed in the GUI
(**Settings → Hardware** (backend card at the top)), never by editing a file:

| GUI field (`mowgli_robot.yaml` key) | Installer key (`docker/.env`) | Default | Meaning |
|-------------------------------------|-------------------------------|---------|---------|
| Motor controller (`openmower_xesc_type`) | `OPENMOWER_XESC_TYPE` | `xesc_mini` | `xesc_mini` / `xesc_2040` |
| LowLevel board port (`openmower_ll_port`) | `OPENMOWER_LL_PORT` | `/dev/ttyAMA0` | LowLevel board UART, 115200 |
| Left wheel controller port (`openmower_xesc_left_port`) | `OPENMOWER_XESC_LEFT_PORT` | `/dev/ttyAMA5` | left drive ESC |
| Right wheel controller port (`openmower_xesc_right_port`) | `OPENMOWER_XESC_RIGHT_PORT` | `/dev/ttyAMA3` | right drive ESC |
| Blade controller port (`openmower_xesc_mow_port`) | `OPENMOWER_XESC_MOW_PORT` | `/dev/ttyAMA4` | mow ESC |
| Emergency inputs (`openmower_emergency_input_config`) | — | empty = the board's own | OpenMower `OM_EMERGENCY_INPUT_CONFIG`, e.g. `!L,!L,S,S` |

A value saved in the GUI wins over `docker/.env`, which wins over the default.
Until a port is saved in the GUI, the GUI shows the installer's value. The
bridge reads them at start-up: **Restart ROS2** after saving (it restarts the
`mowgli-openmower` container too). `OPENMOWER_IMAGE` stays an installer key.

The installer prepares the Pi the way OpenMowerOS does
(`stage-openmower/10-pi-serial`, `15-pi-config`), then asks for a reboot:

- `enable_uart=1`, `dtoverlay=disable-bt` and one `dtoverlay=uartN` per port
  in use — `uart3/4/5` for the xESCs, `uart2` for the GPS;
- the kernel console is removed from `cmdline.txt` (backup
  `cmdline.txt.mowgli.bak`) and the `serial-getty` logins on those UARTs are
  masked — Ubuntu ships `console=serial0`, which otherwise prints the boot log
  into the LowLevel board on `ttyAMA0`;
- the GNSS defaults to `/dev/ttyAMA2` (the board's GPS UART; `ttyAMA4`, the
  usual GNSS port, is the mow xESC here), and a GNSS or LiDAR port an xESC or
  the LowLevel board already uses is refused. Every Pi UART is wired on this
  board, so a LiDAR must be a USB one.

On an older kernel (< 6.1.28) OpenMower used `ttyAMA4/2/3` for
left/right/mow and `ttyAMA1` for the GPS — set the ports accordingly.

### Updates

The host updater manages this backend like the Mowgli board: the installer
installs it (`bash install/mowglinext.sh --only=updater` adds it to an existing
install), and each release ships the `openmower` image with the rest of the
stack. The release's mainboard firmware protocol is the Mowgli STM32 one and is
not checked here (the bridge reports 0); an update still waits for the mower
to be idle, stopped and blade-off, with the bridge reaching the LowLevel board
and both drive xESCs (`firmware_compatible`). See `docs/UPDATES.md`.

## Settings: shared with every backend, OpenMower defaults

A setting MowgliNext already has keeps its MowgliNext key and its usual place
in the GUI. The bridge maps it onto the OpenMower hardware:

| Setting (GUI section) | OpenMower hardware | OpenMower default |
|-----------------------|--------------------|-------------------|
| `ticks_per_meter` (Hardware) | xESC hall ticks per metre | **1600** (`OM_WHEEL_TICKS_PER_M`) |
| `both_wheels_lift_emergency_ms` (Safety) | LowLevel `lift_period` (≥ 2 wheels lifted) | **100 ms** |
| `one_wheel_lift_emergency_ms` (Safety) | LowLevel `tilt_period` (1 wheel lifted) | **2500 ms** |
| `max_charge_voltage` (Battery) | LowLevel `v_battery_cutoff` | **29.0 V** |
| `max_charge_current` (Battery) | LowLevel `i_charge_cutoff` | 1.2 A (template) |
| `battery_full_voltage`, `battery_empty_voltage` (Battery) | LowLevel battery gauge (UI board LEDs) | template / model |
| `wheel_track`, `max_mps`, `mowing_enabled` | host side | template |

**Defaults follow the backend.** The bold values differ from the Mowgli STM32
template, so they live in
[`ros2/src/mowgli_bringup/config/backends/openmower.yaml`](../../ros2/src/mowgli_bringup/config/backends/openmower.yaml),
layered between the template and the robot's sparse file **everywhere**:
`robot_config_util.load_robot_config` (mowgli-ros2), this bridge's launch file,
and the GUI's "default" / reset-to-default / sparse prune
(`gui/asserts/backend_defaults.json`, generated). On an OpenMower robot,
"reset to default" therefore gives OpenMower's value. Nothing is seeded into the
robot's file, so switching backend later needs no file rewrite. A mower-model
preset never writes these keys either: the preset describes the machine, the
backend its electronics.

**Safety envelope.** The Pico saves what it receives to flash and, unlike the
Mowgli STM32, does not clamp. `lowlevel_config.hpp` therefore applies the STM32
firmware's own envelope (`fw_param_catalog.h`) before sending: lift 10–3000 ms,
tilt 10–5000 ms, battery cutoff 25.2–29.4 V, charge current 0.1–1.2 A. A value
outside it — including the 0 ms the Pico would take as "disabled" — is not
sent; the board keeps its own value and the bridge logs a warning. The board's
charger-input cutoff (30 V) and its options have no MowgliNext equivalent and
are never sent.

**OpenMower-only settings** (no MowgliNext equivalent): the wiring above, and
the host-side wheel loop in **Settings → Drive Motor** (`openmower_wheel_*`,
below). The STM32-only settings (wheel PID in PWM counts, firmware yaw loop,
stop-button / tilt / inclination trips, firmware parameters and flashing) are
hidden on an OpenMower robot.

`install/scripts/migrate_openmower.py` imports the OpenMower map and datum;
`OM_WHEEL_TICKS_PER_M` / `OM_WHEEL_DISTANCE_M` still have to be checked by hand.

## What the bridge does

| Surface | Source |
|---------|--------|
| `/hardware_bridge/status` | LowLevel `ll_status` bits (initialized, rain, UI board) + mow ESC telemetry (rpm, current, temperatures). `mower_motor_rpm` is a speed (≥ 0) on both ESC types, as on the STM32: the xESC mini's signed eRPM is divided by `motor_pole_pairs` and made positive, and the xESC 2040, which reports no speed, gets it from its hall counter (6 steps per electrical revolution, `blade_speed.hpp`). The direction is `blade_requested_direction`. map_server only records mowing progress above 1000 rpm. `is_charging` means **on the dock** and comes from the charge-contact voltage (`power_semantics.hpp`), not from status bit 2 (see *Firmware facts* below). `firmware_compatible` is **true only when the LowLevel stream is live and both drive ESCs are connected** — PreFlightCheck refuses to mow otherwise. `firmware_version` names the ESC type and firmware versions. |
| `/hardware_bridge/emergency` | LowLevel emergency bitmask (STOP button, lift/tilt, latch) plus the host request; reasons match the STM32 bridge |
| `/hardware_bridge/power`, `/battery_state` | `v_charge`, `v_system`, `charging_current`, SoC; current is `abs(charge)` while docked, else 0 (docking convention); `Power.charge_current` is also 0 off the dock (the sensor reads ~0.05 A of nothing). `charger_enabled` means ON THE DOCK, as on the STM32 — the behaviour tree reports it as `HighLevelStatus.is_charging`; it is never the charge relay, which v0.13 firmware keeps closed while mowing. The relay shows in `charger_status`: a docked robot whose relay opened (full battery) reads `docked, not charging` / `POWER_SUPPLY_STATUS_NOT_CHARGING` and stays docked. |
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
- sends the board only values inside the STM32 firmware's safety envelope
  (see above), and none for settings MowgliNext does not have.

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
v0.13 charging bit, panel buttons, a controller brown-out mid-drive, the bridge
process frozen for 350 ms mid-drive (SIGSTOP), and a SIGKILL of the bridge
mid-mow. Both controller types are run. The robot's file holds only what a
fresh install writes, so the board's configuration and `ticks_per_meter` come
from the template + OpenMower defaults, exactly as on a robot; a last run
saves the wiring and a lift delay "from the GUI" while `docker/.env` names the
wrong ports and controller type, and checks the GUI values win and a 0 ms
period is refused.

`sim/run_stack_integration.py` adds the real `mowgli_bringup/launch/mowgli.launch.py`:
exactly one `/hardware_bridge` with `HARDWARE_BACKEND=openmower`, a command on
the twist_mux teleop lane reaching the emulated wheels, the STM32 bridge still
starting for `mowgli`, and an unknown backend aborting the launch.

Writing the emulator from the firmware sources, and running it on a loaded CI
runner, found seven bridge bugs that the unit tests could not see; all are fixed
and now covered by both layers:

1. **Charging bit read as "docked"** — would zero `/wheel_odom` while mowing on v0.13.x firmware.
2. **STM32 template values written to the board's flash** — a 10× slower lift e-stop.
3. **Release request kept armed while a stop button was held** — the latch dropped by itself on release.
4. **2 s blind window after a LowLevel link loss** — now 0.5 s.
5. **A controller reboot published as a 2.2e7 m/s odometry spike**, and a single creeping tick at dock contact skipping that visit's IMU calibration.
6. **Phantom speed after a stall** — wheel speed and `/wheel_odom` divided the ticks by the control period, clamped to 100 ms: a bridge starved for 300 ms reported 0.8 m/s while driving at 0.3. Ticks are now divided by the time between the controllers' own status samples (`WheelTickSampler`); the xESC mini's replies carry the time of their request (`RequestStampQueue`).
7. **Robot driven 10–25 % too slow on a busy machine** — the wheel loop averaged tick/time *ratios* per sample, which is biased upward whenever ticks and intervals do not line up exactly; it measured 0.30 m/s while the robot did 0.23. Speed is now total ticks over total time in an 80 ms window.


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
python3 -m pytest sensors/openmower/mowgli_openmower_bridge/test/test_launch_config.py  # launch layering

# simulation (needs the workspace above + mowgli_bringup for the stack check)
python3 sensors/openmower/sim/run_simulation.py            # both controller types
python3 sensors/openmower/sim/run_stack_integration.py
```

Protocol references: `open_mower_ros/src/mower_comms_v1` (LowLevel packets,
`ll_high_level_config`), `xesc_ros/xesc_2040_driver` and
`xesc_ros/vesc_driver` (xESC framing and the GET_VALUES layout).
