# Deployment

MowgliNext is deployed through an installer-generated compose stack written to `docker/docker-compose.yaml`.

## Compose Generation

The installer selects fragments from `install/compose/` based on hardware choices:

- `docker-compose.base.yml`, `docker-compose.gui.yml`, `docker-compose.mqtt.yml` and `docker-compose.watchtower.yml` — always included
- `docker-compose.gps.yml` — the Universal GNSS sidecar, included whenever `GNSS_STACK=universal` (the default on the Mowgli/STM32 backend). It is the only GNSS fragment the installer can select; `GNSS_STACK=disabled` selects none.
- optional LiDAR / MAVROS / TF-Luna fragments

`LIDAR_ENABLED` in `docker/.env` only decides whether the `mowgli-lidar` *container* is composed in — it is deliberately not passed into `mowgli-ros2`. The ROS-side LiDAR mode (scan matching, loop closure, the LiDAR Nav2 overlay) comes from `lidar_enabled` in `docker/config/mowgli/mowgli_robot.yaml` and nothing else.

## Runtime Services

| Container | Purpose |
|-----------|---------|
| `mowgli-ros2` | Main ROS2 stack, localization, Nav2, behavior tree, API |
| `mowgli-gps` | Universal GNSS sidecar (receiver node + topic bridge + optional NTRIP client) when `GNSS_STACK=universal` |
| `mowgli-lidar` | LiDAR runtime when enabled |
| `mowgli-gui` | Web UI |
| `mowgli-mqtt` | MQTT broker |
| `mowgli-watchtower` | Image updates |
| `mowgli-mavros` + `mowgli-ntrip` | Pixhawk backend only (`HARDWARE_BACKEND=mavros`), which forces `GNSS_STACK=disabled` and replaces `mowgli-gps` |

## GNSS Deployment Shape

```text
GNSS_STACK=universal              (default when HARDWARE_BACKEND=mowgli)
  -> install/compose/docker-compose.gps.yml
  -> mowgli-gps
  -> sensors/gps/start_gps.sh
  -> universal_gnss_ros2 receiver_node
     + mowgli_gnss_bridge universal_gnss_topic_bridge
     + universal_gnss_ros2 ntrip_node   (only when NTRIP is enabled)

GNSS_STACK=disabled               (forced when HARDWARE_BACKEND=mavros)
  -> no direct-GNSS fragment
  -> GNSS arrives from the Pixhawk through mowgli-mavros + mowgli-ntrip
```

`GNSS_STACK` accepts only `universal` or `disabled`; the installer errors on anything else, and the older `legacy` / `fallback` spellings are normalized to `universal` on load.

Preferred env contract:

- `GNSS_STATUS_SOURCE=universal`
- `GNSS_STACK=universal`
- `GNSS_RECEIVER_FAMILY=auto|ublox|unicore|nmea`
- `GNSS_TRANSPORT=serial`
- `GNSS_SERIAL_DEVICE=/dev/serial/by-id/...`
- `GNSS_SERIAL_BAUD=921600`
- `GNSS_FRAME_ID=gps_link`
- `GNSS_NTRIP_ENABLED=true|false`
- `GNSS_NTRIP_HOST`, `GNSS_NTRIP_PORT`, `GNSS_NTRIP_MOUNTPOINT`
- `GNSS_NTRIP_USERNAME`, `GNSS_NTRIP_PASSWORD`
- `GNSS_NTRIP_GGA_ENABLED`, `GNSS_NTRIP_GGA_INTERVAL_S`

These are fallback-only. `docker-compose.gps.yml` passes them through with no defaults, and `start_gps.sh` resolves each setting as YAML first, then env, then its own built-in default — so an explicit value in `docker/config/mowgli/mowgli_robot.yaml` (which is what the GUI edits) always wins over `docker/.env`.

`GNSS_BACKEND` is still written to `docker/.env`, but it can now only hold `universal` or — on the MAVROS backend — `disabled`; the older values (`gps`, `ublox`, `unicore`, `nmea`, `legacy`) are normalized to `universal` on load, and nothing under `ros2/src` reads it. The legacy `GPS_*`, `UBLOX_*`, `UNICORE_*` keys and `NMEA_IMAGE` are no longer compatibility shims: the installer actively deletes them from `docker/.env` on every run.

Universal GNSS owns `/gps/status`, `/diagnostics`, and `/rtcm` while Mowgli keeps `/gps/absolute_pose` and `/gps/pose_cov` for downstream localization consumers.

In that universal mode:

- `navsat_to_absolute_pose_node` no longer subscribes to `/diagnostics` for GNSS status reconstruction
- the GUI backend mechanically normalizes Universal GNSS status onto the existing frontend JSON contract
- the old Mowgli-local status publisher stays disabled on the supported direct-GNSS path

Recommended validation/default baud for advanced profiles is `921600`.

Latest live validation on June 4, 2026:

- Universal GNSS was validated successfully on a live u-blox F9P and a live Unicore UM982 at `921600`.
- The preferred stable receiver paths are `/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00` for the F9P and `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` for the UM982.
- The corrected raw tty mappings in that session were `/dev/ttyACM0` for the F9P and `/dev/ttyUSB0` for the UM982.
- The generated universal compose stayed free of the `gnss_unicore` service and the legacy `UNICORE_*` / `GPS_*` env keys. (At that point Universal GNSS was still being moved out of the standalone sidecars; today the universal path deliberately *does* compose `mowgli-gps`, because that container is the Universal GNSS sidecar — `install/tests/test_compose_validity.sh` requires it and still rejects `gnss_unicore`.)
- The F9P stayed in RTK float during the sampled NTRIP window.
- The UM982 accepted corrections, exposed correction age through typed status, and intermittently promoted into RTK float during the sampled window.
- The current Unicore limitation is not device access anymore; it is that `RTCMSTATUSA` still has no dedicated ROS projection beyond the generic correction diagnostics.
- The devcontainer now mounts the host `/dev` tree at `/host-dev` and re-exposes `/dev/serial/by-id` inside the container when the host provides it.

## Legacy GNSS Removal — Complete

The legacy direct-GNSS path is gone; there is no fallback stack left to select. Universal GNSS is the only direct-GNSS runtime, and it lives in the `mowgli-gps` sidecar.

Removed, and no longer present anywhere in the tree:

| Removed | Note |
|---------|------|
| `GNSS_STACK=legacy` / `fallback` | Normalized to `universal`; the installer accepts only `universal` or `disabled` and errors otherwise. |
| `install/compose/docker-compose.unicore.yaml` and the `gnss_unicore` service | The UM98x receiver is handled by Universal GNSS via `GNSS_RECEIVER_FAMILY=unicore`. |
| `install/compose/docker-compose.nmea.yaml` and `NMEA_IMAGE` | NMEA is a receiver family, not a separate sidecar image. |
| `sensors/unicore/` | Only `sensors/gps/` remains. |
| `GPS_*`, `UBLOX_*`, `UNICORE_*` `.env` keys | Deleted from `docker/.env` on every installer run. |
| `ublox_gnss.launch.py` / `ublox_gnss.yaml` | Superseded by Universal GNSS. |
| `mowgli_bringup/universal_gnss.launch.py` | Universal GNSS runs in the `mowgli-gps` sidecar, not inside `mowgli-ros2`; `test_gnss_launch_config.py` asserts `full_system.launch.py` includes no such file. |

`install/tests/test_compose_validity.sh` pins the outcome: a generated universal compose must contain `mowgli-ros2`, `mowgli-gps`, `mowgli-gui`, `mowgli-lidar`, `mowgli-mqtt` and `mowgli-watchtower`, and must not contain `gnss_unicore`, `UNICORE_IMAGE` or the legacy `GPS_*` service env keys.

## Troubleshooting

- No `/gps/fix`: confirm the selected `/dev/serial/by-id/...` device exists inside the runtime and the receiver baud matches the installer-generated `.env`.
- Wrong receiver path: inspect `/dev/serial/by-id` first and wire `GNSS_SERIAL_DEVICE` to that stable symlink. If by-id is unavailable, use `/sys/class/tty/*/../manufacturer` and `/sys/class/tty/*/../product` only as a diagnostic fallback before touching raw `ttyACM*` or `ttyUSB*`.
- Stale `/dev/tty*` entries in a container can survive old hardware layouts. When `/dev` and `/sys/class/tty` disagree, trust the live sysfs mapping rather than the stale node list.
- No RTK corrections: confirm NTRIP settings in `docker/.env` and `docker/config/mowgli/mowgli_robot.yaml`, then check `/rtcm` and `/diagnostics`.
- No `mowgli-gps` container in compose: expected only when `GNSS_STACK=disabled`, i.e. `HARDWARE_BACKEND=mavros`, where the Pixhawk supplies GNSS through `mowgli-mavros` + `mowgli-ntrip`. On the Mowgli/STM32 backend `GNSS_STACK=universal` must compose `mowgli-gps` — if it does not, regenerate with the installer.
- Wrong compose shape: regenerate with the installer and inspect `docker/docker-compose.yaml` plus `docker/.env`.
