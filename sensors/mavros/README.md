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

The published image above is the current deployment baseline. MowgliMAVROS has
not yet been repinned and republished against Universal GNSS dev
`ef31c4c95a8e831d2c926a5557298806505d52a7`; this directory does not claim
otherwise. Updating that external repository and replacing this digest is a
separate follow-up.

## Validation

Run the source-level contract check without Docker or external image builds:

```bash
python3 sensors/mavros/test_integration.py
```

It validates the image pin, installer/compose mapping, by-id support, launch
ownership boundaries, and that no local command/blade enable override or
machine-specific Pixhawk identity has been introduced.
