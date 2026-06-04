# GNSS Integration Audit

This document records the current GNSS integration state of MowgliNext while Universal GNSS is being promoted to the official stack.

## Current State

### Installer and Environment

- `install/lib/gps.sh` now prefers Universal GNSS and exposes Universal-first operator choices, while keeping a legacy fallback path available for migration.
- `GNSS_BACKEND=ublox` is no longer a distinct runtime. It is a compatibility alias that resolves to the shared GPS container with a USB by-id selection.
- `GNSS_STATUS_SOURCE=universal` and `GNSS_STACK=universal` are now the preferred defaults for direct GNSS deployments.
- `GNSS_STATUS_SOURCE` still controls who owns the typed `/gps/status` topic:
  - `mowgli_local` keeps the existing `navsat_to_absolute_pose_node` adapter active for the legacy fallback stack.
  - `universal` disables the Mowgli-local `/gps/status` publisher so Universal GNSS becomes the sole typed status source.
- `docker/.env` remains the source of truth for runtime GNSS wiring and now writes both the preferred Universal contract and the legacy compatibility keys:
  - `GNSS_STACK`
  - `GNSS_RECEIVER_FAMILY`
  - `GNSS_TRANSPORT`
  - `GNSS_SERIAL_DEVICE`
  - `GNSS_SERIAL_BAUD`
  - `GNSS_NTRIP_ENABLED`
  - `GNSS_NTRIP_HOST`
  - `GNSS_NTRIP_PORT`
  - `GNSS_NTRIP_MOUNTPOINT`
  - `GNSS_NTRIP_USERNAME`
  - `GNSS_NTRIP_PASSWORD`
  - plus legacy compatibility keys:
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
- The devcontainer now also bind-mounts the host `/dev` tree at `/host-dev`, and `.devcontainer/post-start.sh` re-links `/dev/serial/by-id` inside the container when the host provides those stable udev symlinks.
- `.devcontainer/post-create.sh` now reuses `ros2/scripts/sync_workspace_packages.sh` so the same package-linking logic is used for post-create, ad-hoc builds, tests, and dev simulation boots.
- Workspace linking now exposes only `gnss_ros2` as `universal_gnss_ros2` from the external repository at this milestone. The low-level Universal GNSS libraries remain sibling CMake subdirectories of that ROS package instead of separate colcon packages here.
- The devcontainer now requests `--privileged` so a rebuilt local container can see attached GNSS USB/UART hardware for validation.

### Compose Generation

- `install/lib/compose.sh` now treats Universal GNSS as the default stack for Mowgli hardware.
- In `GNSS_STACK=universal`, no direct GNSS compose fragment is added; the Universal receiver and NTRIP nodes run inside `mowgli-ros2` through `mowgli_bringup`.
- `GNSS_STACK=legacy` still selects the old direct GNSS fragments:
  - `install/compose/docker-compose.gps.yml` for `gps` and `ublox`
  - `install/compose/docker-compose.unicore.yaml` for `unicore`
- The dormant standalone NMEA compose fragment was removed during the Milestone 8 targeted cleanup.

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
- In preferred Universal mode, these vendor-specific runtimes are intentionally not started by compose anymore, but the code stays in-tree as a validated migration fallback.

## Legacy GNSS Removal Plan

| Reference | Classification | Universal-mode isolation evidence | Removal readiness |
|-----------|----------------|-----------------------------------|-------------------|
| `sensors/gps/` | Legacy fallback only | `GNSS_STACK=universal` does not add `docker-compose.gps.yml`; `test_compose_validity.sh` rejects `sensors/gps`, `gps_health_aggregator.py`, `rtcm_serial_bridge.py`, `nmea_navsat_driver`, and `ublox_dgnss_node` in generated universal compose. | Pending until `GNSS_STACK=legacy` is removed. |
| `sensors/unicore/` | Legacy fallback only | `GNSS_STACK=universal` does not add `docker-compose.unicore.yaml`; compose tests reject `sensors/unicore` and `gnss_unicore` in generated universal compose. | Pending until the legacy UM98x fallback and live validation scripts are retired. |
| `sensors/nmea/` | Removed | No universal compose or launch path references the standalone NMEA sidecar; generic NMEA now routes through Universal GNSS or the legacy shared GPS fallback. | Removed in Milestone 8. |
| `install/compose/docker-compose.gps.yml` | Legacy fallback only | Added only when `GNSS_STACK=legacy` and backend resolves to `gps`; installer matrix tests reject it for universal presets. | Pending until legacy fallback compose is removed. |
| `install/compose/docker-compose.unicore.yaml` | Legacy fallback only | Added only when `GNSS_STACK=legacy` and backend resolves to `unicore`; installer matrix tests reject it for universal presets. | Pending until legacy fallback compose is removed. |
| `install/compose/docker-compose.nmea.yaml` | Removed | No supported backend maps to this fragment; NMEA is not a separate universal sidecar. | Removed in Milestone 8. |
| `GNSS_BACKEND` | Still required | Installer, env parsing, checks, compatibility presets, and legacy compose mapping still consume it. | Keep until preset migration and `GNSS_STACK=legacy` removal are complete. |
| `GPS_*` env keys | Still required | Installer tests cover USB by-id selection, UART fallback, baud probing, udev rules, and legacy compose compatibility. Universal launch also uses `GPS_BY_ID`/`GPS_PORT` as fallback inputs when the new `GNSS_SERIAL_*` keys are absent. | Keep until older `.env` files and udev helpers no longer need compatibility. |
| `gnss_runtime_state_builder.cpp` | Legacy fallback only | `navsat_to_absolute_pose_node` uses it only when `publish_gnss_status=true`; universal launch/tests set this false and assert no local `/gps/status` publisher or `/diagnostics` parser subscription. | Pending until legacy Mowgli-local `/gps/status` reconstruction is removed. |
| `rtcm_serial_bridge.py` | Legacy fallback only | Started only by `sensors/gps/start_gps.sh` for NMEA RTCM injection; universal compose tests reject the script name in generated universal compose. | Keep while `sensors/gps` remains a legacy fallback. |
| `gps_health_aggregator.py` | Legacy fallback only | Started only by `sensors/gps/start_gps.sh`; universal compose tests reject the script name in generated universal compose. | Keep while `sensors/gps` remains a legacy fallback. |
| `nmea_navsat_driver` | Legacy fallback only | Invoked only by `sensors/gps/start_gps.sh` when `GPS_PROTOCOL=NMEA`; universal compose tests reject it in generated universal compose. | Keep while legacy generic NMEA fallback remains. |
| `ublox_dgnss_node` | Legacy fallback only | Invoked only by `sensors/gps/start_gps.sh` when `GPS_PROTOCOL=UBX`; universal compose tests reject it in generated universal compose. | Keep while legacy u-blox fallback remains. |
| Unicore-specific startup scripts | Legacy fallback only | `sensors/unicore/start_gps.sh` and `configure_receiver.sh` are reached only via the legacy Unicore compose fragment or direct validation scripts. | Keep until Universal GNSS fully replaces Unicore fallback operations and live validation helpers. |

No additional source files are safe to delete at this milestone because every remaining legacy runtime path is still reachable through `GNSS_STACK=legacy`, installer recovery checks, or focused tests.

### ROS2 and GUI

- `mowgli_localization/navsat_to_absolute_pose_node`
  - Always publishes `/gps/absolute_pose` and `/gps/pose_cov`.
  - Publishes `/gps/status` only when `publish_gnss_status=true`.
  - Builds `/gps/status` from `NavSatFix` plus parsed `/diagnostics` only in that local-adapter mode.
  - Does not subscribe to `/diagnostics` at all when `publish_gnss_status=false`.
- `gnss_runtime_state_builder.cpp`
  - Is the current Mowgli-local GNSS parser/adapter layer.
  - Contains backend-specific diagnostic parsing for u-blox and Unicore.
- `mowgli_bringup`
  - Keeps the local parser path enabled only for legacy `GNSS_STATUS_SOURCE` values.
  - Leaves `/gps/status` entirely to Universal GNSS when `GNSS_STATUS_SOURCE=universal`.
- GUI/backend code still reads only the typed `/gps/status` topic.
  - In legacy mode that payload is already `mowgli_interfaces/msg/GnssStatus`.
  - In universal mode the backend now performs a temporary mechanical JSON adapter from `universal_gnss_ros2/msg/GnssStatus` into the existing Mowgli GUI schema without parsing vendor strings or diagnostics text.

## Low-Risk Cleanup Applied

The following clearly obsolete paths were removed by the targeted legacy GNSS cleanup:

- `install/compose/docker-compose.nmea.yaml`
- `sensors/nmea/`
- `sensors/gps/serial_ublox_driver.py`
- `ros2/src/mowgli_bringup/launch/ublox_gnss.launch.py`
- `ros2/src/mowgli_bringup/config/ublox_gnss.yaml`
- `install/tests/test_ublox_runtime.sh`
- `NMEA_IMAGE` is no longer written into `docker/.env`

Verification notes for those removals:

- `GNSS_STACK=universal` runs Universal GNSS inside `mowgli-ros2` and does not reference any of the removed files.
- `GNSS_STACK=legacy` still routes through `install/compose/docker-compose.gps.yml` or `install/compose/docker-compose.unicore.yaml`; it does not require the standalone NMEA sidecar or the old dedicated u-blox bringup.
- The shared GPS legacy fallback uses `sensors/gps/start_gps.sh`, `ublox_dgnss_node`, `nmea_navsat_driver`, `gps_health_aggregator.py`, and `rtcm_serial_bridge.py`; those retained files remain intentionally in-tree.

These removals reduce dead GNSS surface area without changing either the preferred Universal runtime graph or the retained legacy fallback graph.

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
- `mowgli_bringup/launch/universal_gnss.launch.py` now wraps `universal_gnss_ros2` receiver/NTRIP nodes with defaults sourced from the existing GNSS env contract plus `mowgli_robot.yaml`.
- `full_system.launch.py` now has a `use_universal_gnss` launch arg and includes the wrapper when enabled. The default flips on only when `GNSS_STATUS_SOURCE=universal` and GNSS is not disabled, keeping the current legacy compose path stable until runtime validation switches over.
- `ros2/scripts/build.sh` now defaults `PACKAGES` builds to `--packages-up-to`, so launch packages like `mowgli_bringup` pull in their required workspace deps during milestone validation. Exact legacy behavior remains available with `PACKAGES_MODE=select`.
- The milestone validation command `PACKAGES="mowgli_interfaces mowgli_localization universal_gnss_ros2 mowgli_bringup" ros2/scripts/build.sh` now succeeds in this devcontainer after aligning `Fields2Cover` discovery between `mowgli_coverage` and the dev image.
- Regression coverage now verifies:
  - legacy mode still advertises and publishes the Mowgli-local `/gps/status`
  - universal mode keeps `/gps/absolute_pose` and `/gps/pose_cov` alive while suppressing the local `/gps/status` publisher and `/diagnostics` parser subscription
  - bringup launch helpers keep the Universal GNSS `/gps/status` remap and local-parser switch aligned

### Hardware Validation

Corrected real hardware validation was re-run on June 4, 2026 after the extra
ArduPilot USB device was removed from the setup.

Sanitized commands used:

- `ros2 launch mowgli_bringup universal_gnss.launch.py receiver_family:=ublox serial_device:=/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00 serial_baud:=921600 ntrip_enabled:=true caster_host:=<host> caster_port:=2101 mountpoint:=<mountpoint> username:=<username> password:=<redacted> status_topic:=/gps/status fix_topic:=/gps/fix diagnostics_topic:=/diagnostics rtcm_topic:=/rtcm`
- `ros2 launch mowgli_bringup universal_gnss.launch.py receiver_family:=unicore serial_device:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 serial_baud:=921600 ntrip_enabled:=true caster_host:=<host> caster_port:=2101 mountpoint:=<mountpoint> username:=<username> password:=<redacted> status_topic:=/gps/status fix_topic:=/gps/fix diagnostics_topic:=/diagnostics rtcm_topic:=/rtcm`
- `bash install/tests/test_compose_validity.sh`

Observed device reality in the validation container:

- The preferred stable identifiers are `/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00` for the F9P and `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` for the UM982.
- Milestone 6 adds a host `/dev` bind plus a post-start helper so rebuilt devcontainers can re-expose `/dev/serial/by-id` directly when the host provides it.
- The live F9P was correctly exposed as `/dev/ttyACM0` and identified via sysfs
  as `u-blox GNSS receiver`.
- The live UM982 was correctly exposed in sysfs as `ttyUSB0` (`1a86:7523`,
  `USB Serial`) and validated as `/dev/ttyUSB0`.
- Stale `/dev/ttyACM1` and `/dev/ttyACM2` nodes could still exist in `/dev`
  without matching live sysfs entries, so direct `/dev/tty*` assumptions remain
  less reliable than by-id paths.

F9P result:

- Receiver family: `ublox`
- Device path: `/dev/ttyACM0`
- Configured baud: `921600`
- Universal GNSS topics were live:
  - `/gps/fix`
  - `/gps/status`
  - `/diagnostics`
  - `/rtcm`
- `/gps/status` had exactly one publisher and it was `universal_gnss_receiver`.
- Live NTRIP succeeded against the local caster without writing credentials into
  the repository or validation notes.
- Receiver-side RTCM activity was confirmed through diagnostics:
  - `forwarded_frame_count=557`
  - `receiver_rtcm_messages_seen=557`
  - `receiver_rtcm_messages_used=557`
  - `receiver_rtcm_messages_not_used=0`
  - `receiver_rtcm_crc_failed=0`
  - `receiver_last_message_type=1127`
- NTRIP-side RTCM forwarding was also active:
  - `published_frame_count=557`
  - `last_message_type=1127`
- `/gps/fix` and `/gps/status` both reported a live position fix.
- RTK mode stayed at `rtk_mode=2` (`FLOAT`) throughout the sampled 20-second
  status window.
- RTK fixed was not observed during this validation session.

UM982 result:

- Receiver family: `unicore`
- Device path: `/dev/ttyUSB0`
- Configured baud: `921600`
- Universal GNSS topics were live:
  - `/gps/fix`
  - `/gps/status`
  - `/diagnostics`
  - `/rtcm`
- `/gps/status` had exactly one publisher and it was `universal_gnss_receiver`.
- Live NTRIP succeeded against the same local caster.
- Correction delivery was visible through the typed Universal GNSS path:
  - `correction_available=true`
  - `forwarded_frame_count=312`
  - `published_frame_count=312`
  - `last_message_type=1127`
  - `correction_age_s` sampled between roughly `0.4` and `1.3`
- `/gps/status` mostly reported `fix_type=2`, `rtk_mode=1` (plain fix / no RTK)
  but did intermittently promote to `fix_type=3`, `rtk_mode=2` (`RTK FLOAT`)
  during the sampled 20-second window.
- Post-session wire sampling confirmed live Unicore correction-state records:
  - `BESTNAVA` and `PVTSLNA` switched to `PSRDIFF`
  - `RTKSTATUSA` was present on the raw stream
- `RTCMSTATUSA` was not observed in the sampled raw output windows.
- Current Universal GNSS behavior matches the upstream runtime mapping:
  - `RTKSTATUSA` meaning is surfaced through typed `/gps/status`
  - `RTCMSTATUSA` is parsed semantically but is not projected into a dedicated
    ROS status field yet
  - the current equivalent correction visibility therefore comes from `/rtcm`,
    `correction_age_s`, `correction_available`, and `RTCM forwarding active`
    diagnostics instead of a literal `RTCMSTATUSA` field

Compose/runtime validation from the same milestone:

- Installer-side universal compose validation passed and the generated
  universal stack omitted `mowgli-gps`, `gnss_unicore`, and standalone GNSS
  sidecars.
- A direct `docker compose up` check could not be run in this devcontainer because the Docker CLI is not available here.

Requested regression suite executed after live validation:

- `bash install/tests/test_env_output.sh`
- `bash install/tests/test_gps_matrix.sh`
- `bash install/tests/test_compose_validity.sh`
- `PACKAGES="mowgli_interfaces mowgli_localization universal_gnss_ros2 mowgli_bringup" ros2/scripts/build.sh`
- `PACKAGES="mowgli_localization universal_gnss_ros2 mowgli_bringup" ros2/scripts/test.sh`
- `cd gui && go test ./pkg/providers -run 'TestAdaptGnssStatus' -count=1`

Results:

- Installer tests passed.
- Focused ROS2 build passed.
- Focused ROS2 tests passed with `160 tests, 0 errors, 0 failures, 23 skipped`.
- GUI provider GNSS adapter test passed.

## Next Small Steps

1. Confirm the rebuilt devcontainer/runtime sees the host `/dev/serial/by-id` entries on target boards and keep raw tty/sysfs checks as a diagnostic fallback only.
2. Decide whether the current typed correction visibility is sufficient for the Unicore migration or whether a dedicated ROS projection for `RTCMSTATUSA`-equivalent detail is still needed before deleting the old Unicore path.
3. Decide whether to keep the temporary GUI JSON adapter or promote `universal_gnss_ros2/msg/GnssStatus` into a first-class shared interface after more field validation.
4. Decide whether to collapse the remaining installer aliases (`GNSS_BACKEND`, `GPS_*`) after enough migration runway has passed for older deployments.
5. Replace the remaining vendor-specific runtime split (`sensors/gps` vs `sensors/unicore`) with one GNSS service once Universal GNSS owns both receivers.
6. Remove `gnss_runtime_state_builder.cpp` and the old diagnostics-driven status path only after the replacement is validated on both u-blox and Unicore hardware and after a real compose-up check is completed outside this Docker-less devcontainer.
