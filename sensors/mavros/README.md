# External MowgliMAVROS sidecar integration

This directory is MowgliNext's integration contract for the externally built
MowgliMAVROS sidecar. It intentionally contains no Dockerfile and no copy of
the MowgliMAVROS source: the external image remains the runtime owner of
MAVROS, its hardware bridge, battery observation, and ESC wheel odometry.

## GNSS ownership: nominal architecture and temporary cutover

Nominal architecture: Universal GNSS owns the receiver, GNSS processing and
NTRIP; MAVROS supplies the flight-controller transport. This is the target
once Universal GNSS's `backend=mavros` and RTCM sink are complete.

Temporary current state: `GNSS_STACK=disabled` with
`MAVROS_GPS1_CANONICAL=true`, so MAVROS GPS1 is the sole canonical GPS
publisher. Keep this cutover until the Universal GNSS MAVROS backend and RTCM
sink are implemented and validated; do not run both GPS owners concurrently.

## Image and compose boundary

`image.env` is the authoritative MowgliNext default for the external image:

```text
ghcr.io/pepeuch/mowglimavros/mowgli-mavros-sidecar:lyrical@sha256:96e23dca25c1a854af191e1a3c94ecad977c41af345733d56214cbac08195c94
```

This is the Lyrical multi-architecture image built from MowgliMAVROS Git
`v0.5b` (commit `649e797da1b948b5a4941833cb59bfa8287794e8`). The Docker
workflow publishes `lyrical` on `main` but does not create a Docker `v0.5b`
tag for Git tag pushes. The verified index digest above contains linux/amd64
and linux/arm64; its arm64 manifest is
`sha256:8183c9102166b96608241bde4a178a834f0efc1c1e03707b9580b517c5871819`.
Use the digest-pinned `lyrical` reference; do not substitute `:v0.5b`.

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

Nominally, `HARDWARE_BACKEND=mavros` selects the `mowgli-mavros` sidecar
alongside Universal GNSS. During the temporary cutover above, GNSS is disabled
instead. MAVROS does not own NTRIP in the nominal architecture. In the
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
