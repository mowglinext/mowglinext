# GNSS Integration Audit

This document records the current GNSS integration state of MowgliNext while Universal GNSS is being promoted to the official stack.

## Current State

### Installer and Environment

- `install/lib/gps.sh` still exposes three operator choices: `gps`, `ublox`, `unicore`.
- `GNSS_BACKEND=ublox` is no longer a distinct runtime. It is a compatibility alias that resolves to the shared GPS container with a USB by-id selection.
- `GNSS_STATUS_SOURCE` now controls who owns the typed `/gps/status` topic:
  - `mowgli_local` keeps the existing `navsat_to_absolute_pose_node` adapter active.
  - `universal` disables the Mowgli-local `/gps/status` publisher so Universal GNSS can become the sole typed status source.
- `docker/.env` remains the source of truth for runtime GNSS wiring:
  - `GNSS_BACKEND`
  - `GNSS_STATUS_SOURCE`
  - `GPS_CONNECTION`
  - `GPS_PROTOCOL`
  - `GPS_PORT`
  - `GPS_BY_ID`
  - `GPS_BAUD`
  - Unicore-specific tuning variables

### Devcontainer and Workspace

- `.devcontainer/devcontainer.json` now binds the reference repository host path into `/workspaces/universal-gnss`.
- `/workspaces/universal-gnss` is now visible inside the devcontainer and contains the mounted reference repository.
- `.devcontainer/post-create.sh` now reuses `ros2/scripts/sync_workspace_packages.sh` so the same package-linking logic is used for post-create, ad-hoc builds, tests, and dev simulation boots.
- Workspace linking now exposes only `gnss_ros2` as `universal_gnss_ros2` from the external repository at this milestone. The low-level Universal GNSS libraries remain sibling CMake subdirectories of that ROS package instead of separate colcon packages here.
- The devcontainer now requests `--privileged` so a rebuilt local container can see attached GNSS USB/UART hardware for validation.

### Compose Generation

- `install/lib/compose.sh` already converges `gps` and `ublox` onto `install/compose/docker-compose.gps.yml`.
- `unicore` still uses its own compose fragment and image.
- The dormant standalone NMEA compose fragment has been removed in this cleanup step.

### Runtime Stacks

- `sensors/gps/start_gps.sh`
  - Runs the shared GPS service for UBX and NMEA.
  - Starts `ublox_dgnss_node` for UBX.
  - Starts `nmea_navsat_driver` for NMEA.
  - Starts `ntrip_client_node` when enabled.
  - Uses `gps_health_aggregator.py` and `rtcm_serial_bridge.py` for Mowgli-local diagnostics and NMEA RTCM injection.
- `sensors/unicore/start_gps.sh`
  - Still carries UM98x-specific receiver configuration and runtime tuning.
  - Remains required until Universal GNSS replaces the vendor-specific path.

### ROS2 and GUI

- `mowgli_localization/navsat_to_absolute_pose_node`
  - Always publishes `/gps/absolute_pose` and `/gps/pose_cov`.
  - Publishes `/gps/status` only when `publish_gnss_status=true`.
  - Builds `/gps/status` from `NavSatFix` plus parsed `/diagnostics` only in that local-adapter mode.
- `gnss_runtime_state_builder.cpp`
  - Is the current Mowgli-local GNSS parser/adapter layer.
  - Contains backend-specific diagnostic parsing for u-blox and Unicore.
- GUI code reads only the typed `/gps/status` topic, which is already the right integration seam.

## Low-Risk Cleanup Applied

The following clearly obsolete paths were removed:

- `install/compose/docker-compose.nmea.yaml`
- `sensors/nmea/`
- `sensors/gps/serial_ublox_driver.py`
- `ros2/src/mowgli_bringup/launch/ublox_gnss.launch.py`
- `ros2/src/mowgli_bringup/config/ublox_gnss.yaml`
- `install/tests/test_ublox_runtime.sh`
- `NMEA_IMAGE` is no longer written into `docker/.env`

These removals reduce dead GNSS surface area without changing the active runtime graph.

## Target Architecture

```text
Receivers
  -> Universal GNSS
  -> ROS2
  -> MowgliNext components
  -> GUI
```

Avoid reintroducing:

```text
Receiver
  -> vendor driver
  -> Mowgli parser A
  -> adapter B
  -> GUI logic C
```

## Remaining Blockers

### Source Integration

- The reference `universal-gnss` repository is available at `/workspaces/universal-gnss`.
- `ros2/scripts/sync_workspace_packages.sh` links `universal_gnss_ros2` into `/ros2_ws/src`, making it visible to normal workspace builds without vendoring the external repository into MowgliNext.
- The current Mowgli-local parser in `gnss_runtime_state_builder.cpp` still remains in place for the next milestone, but the real Universal GNSS ROS package can now be built from the same workspace.

### Workspace Build

- `universal_gnss_ros2` is now intended to build from the MowgliNext workspace using the standard `ros2/scripts/build.sh`, `ros2/scripts/test.sh`, `ros2/Makefile`, and `.devcontainer/post-create.sh` entrypoints.
- `mowgli_interfaces` and `mowgli_localization` still build successfully after the GNSS handoff changes.
- A broader `colcon build --packages-up-to mowgli_bringup` is currently blocked by an unrelated `opennav_coverage` vs `Fields2Cover` API mismatch already present in this workspace.

### Hardware Validation

Hardware validation is currently blocked in this devcontainer:

- `/dev/ttyACM0` not present
- `/dev/ttyUSB0` not present
- `/dev/serial/by-id` not present
- `/dev/bus/usb` not present
- `docker` CLI not available in the container

That means serial, USB, live NTRIP, RTCM injection, and runtime compose validation could not be executed from this environment.

## Next Small Steps

1. Wire `universal_gnss_ros2` receiver and NTRIP launch/config into `mowgli_bringup`.
2. Switch typed `/gps/status` ownership to Universal GNSS-backed status while keeping the legacy parser in place until replacement is validated.
3. Replace `mowgli_localization` diagnostic parsing with Universal GNSS-native status/messages where available.
4. Collapse installer choices so `ublox` is treated as a legacy preset only, not a first-class backend.
5. Replace the remaining vendor-specific runtime split (`sensors/gps` vs `sensors/unicore`) with one GNSS service once Universal GNSS owns both receivers.
