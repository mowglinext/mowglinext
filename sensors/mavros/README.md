# External MowgliMAVROS sidecar integration

This directory is MowgliNext's integration contract for the externally built
MowgliMAVROS sidecar. It intentionally contains no Dockerfile and no copy of
the MowgliMAVROS source: the external image remains the runtime owner of
MAVROS, its hardware bridge, battery observation, and ESC wheel odometry.

## GNSS ownership: canonical adapter architecture

Universal GNSS remains responsible for GNSS decoding, receiver status,
RTK/correction metadata and NTRIP while MAVROS supplies the flight-controller
transport.

When `GNSS_SOURCE=mavros`, the MowgliMAVROS `mowgli_gnss` plugin consumes the
selected Universal GNSS MAVROS receiver (`GNSS_MAVROS_SOURCE=gps1|gps2`) and
owns the canonical MowgliNext outputs:

- `/gps/fix`
- `/gps/status`

The MowgliMAVROS hardware bridge consumes `/gps/status` for readiness only; it
does not project raw MAVLink GPS messages into the canonical topics.

When `GNSS_SOURCE=direct`, MowgliNext keeps the direct Universal GNSS bridge
and the MowgliMAVROS canonical GNSS adapter stays inactive. This preserves
single ownership of `/gps/fix` and `/gps/status`.

`MAVROS_GPS1_CANONICAL` is retained only as a deprecated deployment
consistency guard. `GNSS_SOURCE` and `GNSS_MAVROS_SOURCE` are authoritative.
For MAVROS GPS1 the compatibility flag is `true`; for GPS2 and direct GNSS it
is `false`.

RTCM/NTRIP remains owned by Universal GNSS. In MAVROS-source mode the
MowgliNext GNSS sidecar runs the NTRIP path without opening a direct receiver
serial port and publishes corrections to `/rtcm`, which the Universal GNSS
MAVROS plugin forwards to the flight controller.

## Image and compose boundary

`image.env` is the authoritative MowgliNext default for the external image:

```text
ghcr.io/pepeuch/mowglimavros/mowgli-mavros-sidecar:latest
```

During active MowgliMAVROS development, MowgliNext follows the
multi-architecture `latest` image published from MowgliMAVROS `main`. The
workflow currently builds both linux/amd64 and linux/arm64 and publishes
`latest` from the primary Lyrical build.

This floating reference is intentional while the MAVROS/GNSS integration is
still evolving, so MowgliNext can consume new sidecar fixes without a manual
repin after every build. Before production/release acceptance, replace
`latest` with an immutable digest and validate that exact image on the target
hardware.

`install/lib/config.sh` sources that file, then writes `MAVROS_IMAGE` to the
generated `docker/.env`. `install/compose/docker-compose.mavros.yml` remains
the installer-owned compose fragment because the installer merges it into the
generated stack. It supplies host networking, `/dev:/dev`, the read-only
derived `docker/config/mavros` mount at `/ros2_ws/config`, the Cyclone DDS
mount, and `MAVROS_PORT`, `MAVROS_BAUD`, autopilot, GCS, and
target-system/component inputs. MowgliNext derives that YAML from the
operator-facing config and overrides only `ntrip_enabled: false`.

Use `MAVROS_BY_ID=/dev/serial/by-id/<stable-device>` when a stable path is
available. The installer makes that path the `MAVROS_PORT`; `/dev/mavros` is
only the compatibility fallback. No machine-specific USB identity belongs in
this repository.

## Backend ownership and fail-closed behavior

`HARDWARE_BACKEND=mavros` selects the `mowgli-mavros` sidecar alongside
Universal GNSS. GNSS ownership is selected independently with `GNSS_SOURCE`:
`mavros` uses the canonical MowgliMAVROS GNSS adapter, while `direct` keeps
the direct Universal GNSS bridge. MAVROS does not own NTRIP. In the
main MowgliNext launch, only the legacy `mowgli_hardware/hardware_bridge_node` is excluded;
`robot_state_publisher` and `twist_mux` remain running. The external sidecar
is expected to own `mavros_hardware_bridge_node`.

The sidecar's expected integration surfaces include `/mavros/state`,
`/mavros/global_position/global`, the backend's public GNSS contract, and its
hardware bridge surfaces. A running container is not readiness evidence. The
external image must keep command and blade paths disabled by default;
MowgliNext passes no command- or blade-enable override. Hardware acceptance is
`HARDWARE_PENDING` and this integration does not alter ArduPilot, motion, or
blade behavior.

## Wheel-odometry configuration contract

The sidecar contract is defined by
[`Pepeuch/mowglimavros@b92925b`](https://github.com/Pepeuch/mowglimavros/tree/b92925b2bc969585350b824ac066004eae903e3b/ros2/src/mavros_esc_wheel_odometry):

- MowgliNext keeps one canonical `ticks_per_meter` value in
  `mowgli_robot.yaml` for every hardware backend.
- With `HARDWARE_BACKEND=mowgli`, its live destination remains
  `hardware_bridge.ticks_per_meter`; the existing STM32 PID/feed-forward
  settings and tuning tool are unchanged.
- With `HARDWARE_BACKEND=mavros`, the live destinations are
  `mavros/esc_wheel_odometry.ticks_per_meter` and
  `mavros/esc_wheel_odometry.track_width_m` (from canonical `wheel_track`).
  No Mowgli `wheel_pid_*` value is routed or exposed as a drive control.

At boot, the installer resolves `ticks_per_meter` and `wheel_track` from the
complete ROS template plus the operator's sparse `mowgli_robot.yaml`, then
writes the sidecar-only `esc_wheel_odometry.yaml`. The sidecar launch loads it
after its packaged defaults. This keeps the sidecar from parsing the Mowgli
robot configuration and preserves template fallbacks for sparse installations.

The GUI persists these canonical values and applies them through the MAVROS
parameter service when it is available. Its MAVROS Drive page exposes only
wheel-odometry calibration, an explicit traction opt-in and live source/tick diagnostics; it never
offers STM32 PID, feed-forward, or flash actions.

`/wheel_ticks` is projected from validated signed motor-revolution observations
only. It uses the established magnitude-plus-direction `WheelTick` convention;
the fixed-point transport scale is paired with `wheel_tick_factor`, so no wheel
radius, gear ratio, or physical calibration is inferred. Target validation of
the source, geometry and calibrated metric odometry remains `HARDWARE_PENDING`.

The RTK calibration card records independent `/wheel_ticks` and `/gps/fix`
observations through the existing GUI provider. The operator drives a straight
2–10 m pass using the existing manual controls. The result is the mean of
left/right motor revolutions divided by RTK distance, independent of the old
calibration or `/wheel_odom`. It rejects lost RTK Fixed, stale/unpaired data,
direction changes, source/epoch/segment changes, curves and inconsistent wheels.
Apply changes and persists only `ticks_per_meter`, with live confirmation.

`mavros_manual_control_enabled` defaults to `false`. Its explicit GUI confirmation
enables `/hardware_bridge.manual_control_enabled`; it never arms the FCU. Nonzero
commands still require a connected, already armed FCU, a released physical Safety
Switch and no active/latched emergency. `blade_control_enabled` remains `false`.
`mavros_wheel_lift_safety_enabled` defaults to `true` and maps to
`/hardware_bridge.wheel_lift_safety_enabled`. Disabling it requires GUI confirmation;
raw left/right lift telemetry remains visible and physical Safety always applies.
Both settings persist in dedicated `hardware_bridge.yaml` and reload at boot.

## Safety wiring and target acceptance

The verified software contract reads Safety from `MOTOR_OUTPUTS` in MAVROS
`/mavros/sys_status`: absent from `sensors_present` means Unknown; present with
the enabled bit clear means Engaged; present with it set means Released.
Wheel-lift observation uses the existing `/uas1/mavlink_source` AP_Button stream.
The GUI shows one lifted wheel orange and two red, including with protection off.

Keep the installation's verified physical Safety Switch wiring. This integration
does not establish a connector pinout or electrical polarity for another FCU.
Hall wheel-lift wiring remains `HARDWARE_PENDING`: the exact GPIOs, supply and
signal voltage, pull-ups, polarity and isolation must be confirmed on the actual
board before connection. No pinout is inferred here.

Exact baseline capture, read-only commands and acceptance procedure are in
[`VALIDATION.md`](VALIDATION.md). New sidecar graph/build acceptance remains
`ENVIRONMENT_PENDING` where MAVROS message/MAVLink dependencies are absent;
the already established motor feedback evidence is not re-audited.

The floating published image remains the deployment baseline. Its exact target
image digest and robot acceptance are separate follow-up evidence; this source
contract does not claim that a robot has received this change.

## Validation

Run the source-level contract check without Docker or external image builds:

```bash
python3 sensors/mavros/test_integration.py
```

It validates the image pin, installer/compose mapping, by-id support, launch
ownership boundaries, and that no local command/blade enable override or
machine-specific Pixhawk identity has been introduced.
