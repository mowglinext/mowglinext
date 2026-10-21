# Airseekers Tron hardware bridge (`HARDWARE_BACKEND=airseekers`)

Runs MowgliNext on an **Airseekers Tron** without reflashing anything. The
`mowgli-airseekers` sidecar container speaks to the Tron's vendor
chassis/cutter MCU, reads its WIT JY61P IMU, its top-panel keys and its Unicore
UM960 GNSS receiver, and publishes the exact `/hardware_bridge` contract plus
`/gps/fix` and `/gps/status`. `mowgli-ros2` skips its own
`hardware_bridge_node` (`mowgli.launch.py` gates it on `HARDWARE_BACKEND=mowgli`)
and, with the backend default `gnss_stack: disabled`, its GNSS bridge.
Everything above — fusion_graph, Nav2, the behaviour tree, the GUI — is
unchanged.

```
 ┌──────────────────── mowgli-airseekers (this image) ────────────────────┐
 │ airseekers_bridge_node  (ROS node name: hardware_bridge)                │
 │   MCU link  ──A5…5A frames──▶ /dev/serial_mower  cutter + chassis boards │
 │   IMU       ◀──WIT 0x55────── /dev/serial_imu    JY61P (also forwarded   │
 │                                                  to the MCU, 45 Hz)      │
 │   keys      ◀──evdev───────── /dev/keyboard      top panel               │
 │ um960_node  (um960_gps_node)                                            │
 │   GNSS      ◀──NMEA/Unicore── /dev/serial_rtk    UM960 behind the rover  │
 │             ──RTCM (NTRIP)──▶                    board (LoRa or NTRIP)   │
 └─────────────────────────────────────────────────────────────────────────┘
      ▲ /cmd_vel   │ /hardware_bridge/{status,emergency,power,dig_escalated,
      │            │   cmd_vel_applied}  /battery_state /imu/data
      │            │ /wheel_odom /wheel_ticks /gps/fix /gps/status
      │            ▼ services mower_control, emergency_stop
   twist_mux     BT · fusion_graph · Nav2 · GUI   (mowgli-ros2, unchanged)
```

**Language.** Unlike the C++ OpenMower bridge this sidecar is Python (rclpy),
like the GNSS tooling under `sensors/gps`: it is a restructured port of the
drivers that already ran on the Tron (their tests are its oracle, see
*Tests*). Its safety-relevant pure logic mirrors the OpenMower headers one to
one (`blade_policy.hpp`, `cmd_vel_slew.hpp`, `imu_bias_estimator.hpp`); a C++
port onto the shared bridge core (MowgliNext plan U2,
`mowgli_bridge_core`) can follow once that core exists.

## Bring-up

The Tron's host (RK3588, vendor Linux) must not be modified, so the installer's
host steps (UART overlays, `/boot`, systemd) do not apply. Run the compose
stack by hand from a MowgliNext checkout on the robot:

1. Stop the vendor stack's serial users first (`systemctl stop mower-base.service`
   and the vendor GNSS driver): **two writers on one UART garble both**.
2. `docker/.env`:

   ```bash
   HARDWARE_BACKEND=airseekers
   GNSS_STACK=disabled          # the sidecar owns the UM960; no gps container
   LIDAR_ENABLED=false
   MOWGLI_ROS2_IMAGE=ghcr.io/mowglinext/mowglinext/mowgli-ros2:main
   GUI_IMAGE=ghcr.io/mowglinext/mowglinext/mowglinext-gui:main
   AIRSEEKERS_IMAGE=ghcr.io/mowglinext/mowglinext/airseekers:main
   # optional, only if the udev symlinks differ (the GUI value wins):
   # AIRSEEKERS_MCU_PORT=/dev/ttyS9
   # AIRSEEKERS_IMU_PORT=/dev/serial_imu
   ```

3. `docker/config/mowgli/mowgli_robot.yaml` (sparse): the datum, the dock pose
   once calibrated, `lidar_enabled: false`, NTRIP if you use network RTK, and
   the chassis geometry (below). Do **not** copy defaults into it.
4. Start it:

   ```bash
   docker compose -f install/compose/docker-compose.base.yml \
                  -f install/compose/docker-compose.gui.yml \
                  -f install/compose/docker-compose.airseekers.yml up -d
   ```

   On a host the installer can manage, `--backend=airseekers` (or menu entry
   **[4]**) selects the same fragments and writes `GNSS_STACK=disabled`.
5. Check: `docker logs mowgli-airseekers` shows `opened MCU port`, `opened IMU
   port`, `MCU firmware: airseekers cutter … / chassis …`; the GUI's
   Diagnostics shows the firmware as compatible.
   `ros2 topic echo /hardware_bridge/airseekers/state` prints a JSON summary
   (link, IMU, dock, emergency, tilt band, frame counters).
6. The bridge **boots latched** (emergency). Reset it from the GUI (or the
   short power key) once the robot is level and the stop button is released.
7. First motion: wheels off the ground, blade off (`mowing_enabled: false`),
   then on the lawn with the e-stop in reach. See *Open measurements*.

**Chassis geometry** is per machine and set in the GUI (Settings → Hardware,
model "Custom"); MowgliNext has no Tron model preset, and the backend overlay
only carries the drive values the bridge itself needs. Starting values from the
Tron URDF (to be measured on your robot): `wheel_radius` 0.10,
`chassis_length` 0.70, `chassis_width` 0.50, `chassis_center_x` 0.12,
`tool_width` 0.20, `blade_radius` 0.10, `gps_x` 0.0, `gps_z` 0.30.

## Settings

A setting MowgliNext already has keeps its key; the bridge maps it onto the
Tron. Values that differ from the MowgliNext template live in
[`ros2/src/mowgli_bringup/config/backends/airseekers.yaml`](../../ros2/src/mowgli_bringup/config/backends/airseekers.yaml)
(layered template ← backend ← robot file, here and in mowgli-ros2 and the GUI):

| Setting | Tron default | Used for |
|---------|--------------|----------|
| `wheel_track` | 0.48 m | splitting body velocity into the derived per-wheel ticks |
| `max_mps` | 0.3 m/s | the MCU's own clamp, mirrored before the wire |
| `gnss_stack` | `disabled` | mowgli-ros2 does not start the Universal GNSS bridge; this sidecar publishes `/gps/*` |
| `gnss_serial_device` / `gnss_serial_baud` | `/dev/serial_rtk` / 115200 | the UM960 port (behind the rover board) |
| `battery_full/empty/critical_voltage` | 24.6 / 20.0 / 19.5 V | the BT's voltage-derived battery % (**estimates**, see below) |
| `tilt_emergency_ms` | 300 ms | debounce of the host tilt trip |
| `mowing_enabled`, `ntrip_*`, `gnss_frame_id`, `imu_cal_*`, `ticks_per_meter` | template | as on every backend (`ntrip_enabled` off = LoRa base, on = NTRIP) |

Airseekers-only settings (`airseekers_*` template keys, **Settings → Hardware**
and **Settings → Drive Motor** on this backend): `airseekers_mcu_port`,
`airseekers_imu_port`, `airseekers_keys_device`, `airseekers_cutter_height_mm`
(30–90 mm, also live: `ros2 param set /hardware_bridge cutter_height_mm 50`),
`airseekers_imu_roll/pitch_offset_deg`, `airseekers_max_angular_radps` (0.3,
the MCU clamp), and the straight-line compensation the vendor MCU lacks
(`airseekers_angular_trim_radps`, `_angular_deadband_radps`, `_heading_hold`,
`_heading_hold_kp/kd/max_radps`, all neutral by default). The bridge reads them
at start-up: **Restart ROS2** after saving (it restarts the sidecar too).

Deck height is a backend extension: the contract's `MowerControl` has no height
field, so it is the parameter above, published latched on
`/hardware_bridge/airseekers/cutter_height_mm` (`std_msgs/Int16`).

## What the bridge does

| Surface | Source |
|---------|--------|
| `/hardware_bridge/status` | MCU link (SensorInfo ~100 Hz) + cutter motor telemetry. `is_charging` = **on the dock** (the battery's `dock_ok`). `firmware_compatible` only while the MCU stream is live, its versions are known and the IMU is live (without the IMU the MCU reports a lift). `rain_detected` from the MCU flag. `mower_motor_rpm` = \|cutter rpm\|. |
| `/hardware_bridge/emergency` | MCU `stop` / `lift`, the host tilt trip, a host request; latched on the host, see *Safety model* |
| `/hardware_bridge/power`, `/battery_state` | MCU BatteryInfo (0.1 V, 0.1 A). `charger_enabled` = docked; `BatteryState.current` = \|charge current\| while docked, else 0 (Invariant 12). `v_charge` is 0: the Tron has no separate charge-contact voltage |
| `/imu/data` | WIT JY61P (100 Hz), frame `imu_link`, at-rest bias removed (dock-armed, as on OpenMower). No `/imu/mag_raw` (6-axis IMU) |
| `/wheel_odom` | the MCU's own measured body speed (SpeedData ~100 Hz) at 50 Hz; yaw rate from the IMU gyro (the vendor never used the MCU's angular value, which is unvalidated); zeroed while docked (Invariant 11) |
| `/wheel_ticks` | **derived**: the MCU does not put encoder counts on the wire, so per-wheel counts are integrated from the measured speed (`wheel_track`, `ticks_per_meter`) |
| `/cmd_vel` → MCU | clamp (0.3 m/s, 0.3 rad/s) → straight-line shaping → `cmd_vel_slew.hpp` semantics → SpeedData at 20 Hz while moving; see *Safety model* for the stop policy |
| `mower_control` | CutterControl (enable, 100 %, deck height); blade gated by `blade_policy.hpp` semantics; always forward (the vendor never reversed the blade) |
| `emergency_stop` | 1: host latch + immediate zero (only if moving) + blade off. 0: released only while no trigger is asserted, then the MCU's own lift/stop latch is cleared (all-zero SensorInfoControl) |
| `/hardware_bridge/dig_escalated` | always `false` (latched): the dig detector is not ported yet |
| `reboot_board`, `set_firmware_debug`, `clear_dig_escalation` | answer `success=false` with a reason |
| `/hardware_bridge/cmd_vel_applied` | the exact command on the wire, on change |
| Panel keys | go-dock → `HOME`, start/pause → `START`/`STOP`, short power → reset emergency (as the vendor's "clear e-stop") |
| `/gps/fix`, `/gps/status`, `/gps/corrections` | `um960_gps_node`: NMEA + Unicore ASCII/binary BESTNAV, accuracy from the receiver's sigmas, fix type from its solution; corrections from the LoRa base (rover board, nothing to do) or the host NTRIP client (`ntrip_enabled`). A heading is never published (single antenna) |

Charging: on dock arrival (and every 30 s while docked) the bridge sends
ChargeControl(on); the MCU's BMS decides the rest.

## Safety model — read before mowing

The Tron's MCU only enforces a few things itself: it ignores SpeedData while
its own `lift` or `stop` flag is set, and it zeroes the drive setpoint some
time after host traffic stops. **It keeps executing the last SpeedData for as
long as host traffic (the heartbeat) continues**, and the vendor host software
did all the rest. So, as on OpenMower, the host is in the actuation path, and
this bridge:

- holds the wheels on any emergency, when the MCU stream is older than
  `mcu_status_timeout_s` (0.5 s — it cannot see the stop button then), when
  `/cmd_vel` is older than `cmd_vel_timeout_s` (0.5 s), and in `IDLE`;
- ends **every** motion with three explicit zero frames 100 ms apart, then
  goes silent. It never streams zeros at rest: any SpeedData, zeros included,
  takes the chassis out of its position hold, and on the dock that lets the
  wheels creep off the contacts (observed on the robot);
- on the dock sends nothing until a fresh `HighLevelStatus` says the BT left
  `IDLE` (an undock), and refuses reverse motion there (the Tron reverses onto
  its charger);
- runs the blade only in `AUTONOMOUS` / `MANUAL_MOWING` with a
  `HighLevelStatus` younger than 5 s, `mowing_enabled`, no emergency and a live
  link; a blade-off command is repeated three times (nothing acknowledges it);
- latches every emergency (stop button, lift, host tilt trip at 30° roll /
  32° pitch — below the MCU's own lift window of ~38°/37–40° — and the service);
  only `emergency_stop(0)` clears it, and only while no trigger is asserted,
  so letting go of the stop button never frees the robot by itself;
- forwards the IMU attitude to the MCU at ~45 Hz: without it the MCU reports a
  permanent lift, and the MCU's own lift detection relies on it.

If the bridge itself dies, only the MCU's host-traffic watchdog stops the
robot. The simulation SIGKILLs the bridge mid-mow; that check passes against
the emulator's **assumed** 2 s window, which has not been measured on hardware
(below). Any change to `safety.py`, the control tick or the emergency handling
is **safety-critical**.

## Open measurements (do these before relying on it)

1. **Heartbeat failsafe timing.** How long the MCU keeps the last SpeedData and
   the blade after host traffic stops. The firmware counter is "about 200 ticks"
   with an unknown tick period; the emulator assumes 2 s. Measure wheels-up:
   drive at 0.1 m/s, kill the container, time the stop; repeat with the blade.
   Also whether the MCU stops sooner because the forwarded IMU stream stops.
2. **Battery voltage thresholds** (`battery_*_voltage`): from two readings only.
3. **Measured angular speed** from the MCU (sign, units) — the bridge uses the
   gyro until this is checked; `odom_angular_source: mcu` switches.
4. **Drive-board current units** (MotorInfo) — only the cutter's are used.
5. Chassis geometry and IMU/GNSS lever arms (template keys, see *Bring-up*).
6. Dock direction: the Tron docks reversing on an ArUco marker (MowgliNext
   plan P10: `dock_backwards` plus a marker detector). Until that lands,
   autonomous **docking** is not supported on this backend. **Undocking** is:
   the backend sets `undock_forward: true`, so the BT's undock legs drive
   forward off the charger (`/drive_on_heading`, `undock_distance` at
   `undock_speed`) and the heading calibration reads the heading along the
   motion. The bridge still refuses reverse motion while docked.

## Not covered (yet)

- Wheel-slip dig detection (Invariant 16) — `dig_escalated` stays `false`
  until the detector is ported onto the shared bridge core.
- Bumper as an emergency source is available (`bumper_is_emergency`) but off:
  MowgliNext has no bumper policy yet (plan U8).
- Pivot assist / both-wheels turn shaping from the port (vendor floor of
  ~0.06 m/s per wheel while turning): a planner concern, not the bridge's.
- LoRa base pairing from the GUI (`set_lora` exists in the UM960 node but is
  disabled by default).
- Cameras, fill light, rain ADC (`/dev/rain`): the MCU rain flag is used.
- Field validation: exercised against the emulator and the port's tests only.

## Origin of the protocol and licence note

The MCU frame format, struct layouts and behaviour were recovered by the
project owner from the stock vendor system on their own Tron (serial captures,
~170k frames with zero checksum errors, and reverse engineering of the vendor
host binaries and MCU firmware). `protocol.py` is a clean re-implementation of
that behaviour; it contains no vendor code, no vendor protobuf bindings and no
vendor schema (the serial link does not use protobuf). The owner has decided
to publish it; the earlier hand-off note excluded "protocol material recovered
from vendor builds" pending that decision. Vendor firmware, binaries and
captures are not in this tree.

`um960_gps_driver` and the ported pure modules (`tilt.py`, `wit.py`,
`keys.py`, `drive_shaper.py`, `sim/kinematics.py`) come from the Airseekers
ROS 2 port by the same author (originally MIT / Apache-2.0); the UM960 package
keeps its MIT notice, the rest is GPL-3.0 like MowgliNext.

## Tests

| What | Where | Runs |
|------|-------|------|
| Framing / payload codec vs golden frames from the port's driver; optional randomised parity with that driver (`AIRSEEKERS_ORACLE_DRIVER=<path to mower_mcu_driver>`) | `mowgli_airseekers_bridge/test/test_protocol.py` | plain Python |
| Safety model: blade / wheel / dock gates, emergency latch, SpeedData stop policy, slew, derived ticks | `test/test_safety.py` | plain Python |
| Power / Status projection, IMU bias, tilt bands, straight-line shaping | `test/test_contract.py`, `test/test_shaping.py` | plain Python |
| Launch parameter layering (template ← backend ← env ← robot file), GNSS ownership | `test/test_launch_config.py` | needs `launch_ros` |
| Emulated MCU rules (watchdog, lift latch, IMU-stream lift, clear frame) | `sim/test_emulator.py` | plain Python |
| UM960 parsers, NTRIP client, node, `/gps/status` projection | `um960_gps_driver/test` | plain Python (shim) or ROS |
| Closed loop: boot, round trip cmd_vel → SpeedData → plant → `/wheel_odom`, stop policy, IDLE, stop button, tilt + lift, blade gates, dock hold, link loss, panel key, **SIGKILL watchdog** | `sim/run_simulation.py` | sourced ROS workspace |

```bash
# image — build context is the repo root
docker build -t mowgli-airseekers -f sensors/airseekers/Dockerfile .

# unit tests (no ROS needed for most)
python3 -m pytest sensors/airseekers/mowgli_airseekers_bridge/test sensors/airseekers/sim/test_emulator.py
(cd sensors/airseekers/um960_gps_driver && python3 -m pytest test)

# simulation inside the image, isolated from any network
docker run --rm --network none -e ROS_DOMAIN_ID=79 -e ROS_LOCALHOST_ONLY=1 \
  -v "$PWD:/repo:ro" mowgli-airseekers \
  python3 /repo/sensors/airseekers/sim/run_simulation.py
```

CI: `.github/workflows/sensors-airseekers.yml` (image build + smoke test via
`_sensor-docker.yml`, the unit tests, and the simulation).
