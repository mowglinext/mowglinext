# External MowgliMAVROS sidecar integration

This directory is MowgliNext's integration contract for the externally built
MowgliMAVROS sidecar. It intentionally contains no Dockerfile and no copy of
the MowgliMAVROS source: the external image remains the runtime owner of
MAVROS, the Universal GNSS MAVROS plugin, NTRIP, the MAVROS hardware bridge,
battery observation, and ESC wheel odometry.

## Image and compose boundary

`image.env` is the authoritative MowgliNext default for the external image:

```text
ghcr.io/pepeuch/mowglimavros/mowgli-mavros-sidecar:kilted@sha256:04e4eb17b0f5ce38f882f68346b1694774fa87e1945b38b57c94f90da34dd560
```

`install/lib/config.sh` sources that file, then writes `MAVROS_IMAGE` to the
generated `docker/.env`. `install/compose/docker-compose.mavros.yml` remains
the installer-owned compose fragment because the installer merges it into the
generated stack. It supplies host networking, `/dev:/dev`, the read-only
`/ros2_ws/config` and Cyclone DDS mounts, and `MAVROS_PORT`, `MAVROS_BAUD`,
autopilot, GCS, and target-system/component inputs.

Use `MAVROS_BY_ID=/dev/serial/by-id/<stable-device>` when a stable path is
available. The installer makes that path the `MAVROS_PORT`; `/dev/mavros` is
only the compatibility fallback. No machine-specific USB identity belongs in
this repository.

## Backend ownership and fail-closed behavior

`HARDWARE_BACKEND=mavros` selects exactly one external `mowgli-mavros`
sidecar and disables the direct GPS sidecar. In the main MowgliNext launch,
only the legacy `mowgli_hardware/hardware_bridge_node` is excluded;
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
