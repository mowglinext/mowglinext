# Deployment

MowgliNext is deployed through an installer-generated compose stack written to `docker/docker-compose.yaml`.

## Compose Generation

The installer selects fragments from `install/compose/` based on hardware choices:

- `docker-compose.base.yml`
- `docker-compose.gui.yml`
- in `GNSS_STACK=legacy`, one GNSS fragment:
  - `docker-compose.gps.yml` for the shared GPS service
  - `docker-compose.unicore.yaml` for the UM98x path
- optional LiDAR / MAVROS / TF-Luna fragments

## Runtime Services

| Container | Purpose |
|-----------|---------|
| `mowgli-ros2` | Main ROS2 stack, localization, Nav2, behavior tree, API |
| `mowgli-gps` | Legacy fallback GNSS runtime only when `GNSS_STACK=legacy` |
| `mowgli-lidar` | LiDAR runtime when enabled |
| `mowgli-gui` | Web UI |
| `mowgli-mqtt` | MQTT broker |
| `mowgli-watchtower` | Image updates |

## GNSS Deployment Shape

```text
GNSS_STACK=universal
  -> install/compose/docker-compose.base.yml
  -> mowgli-ros2
  -> mowgli_bringup/universal_gnss.launch.py
  -> universal_gnss_ros2 receiver_node + ntrip_node

GNSS_STACK=legacy
  -> install/compose/docker-compose.gps.yml or docker-compose.unicore.yaml
  -> sensors/gps/start_gps.sh or sensors/unicore/start_gps.sh
```

Preferred env contract:

- `GNSS_STATUS_SOURCE=universal`
- `GNSS_STACK=universal`
- `GNSS_RECEIVER_FAMILY=auto|ublox|unicore|nmea`
- `GNSS_TRANSPORT=serial`
- `GNSS_SERIAL_DEVICE=/dev/...`
- `GNSS_SERIAL_BAUD=921600`
- `GNSS_NTRIP_ENABLED=true|false`
- `GNSS_NTRIP_HOST`, `GNSS_NTRIP_PORT`, `GNSS_NTRIP_MOUNTPOINT`
- `GNSS_NTRIP_USERNAME`, `GNSS_NTRIP_PASSWORD`

Legacy compatibility keys (`GNSS_BACKEND`, `GPS_*`) are still written so older installer logic, udev helpers, and fallback compose fragments keep working during migration.

`GNSS_BACKEND=ublox` is still accepted as a compatibility preset, but it resolves onto the preferred Universal GNSS stack unless `GNSS_STACK=legacy` is selected explicitly.

`GNSS_STATUS_SOURCE=universal` hands `/gps/status`, `/diagnostics`, and `/rtcm` over to Universal GNSS while Mowgli keeps `/gps/absolute_pose` and `/gps/pose_cov` for downstream localization consumers.

In that universal mode:

- `navsat_to_absolute_pose_node` no longer subscribes to `/diagnostics` for GNSS status reconstruction
- the GUI backend mechanically normalizes Universal GNSS status onto the existing frontend JSON contract
- the old Mowgli-local status publisher remains available only for legacy `GNSS_STATUS_SOURCE` values

Recommended validation/default baud for advanced profiles is `921600`.

Latest live validation on June 4, 2026:

- Universal GNSS was validated successfully on a live u-blox F9P and a live Unicore UM982 at `921600`.
- The corrected receiver paths in that session were `/dev/ttyACM0` for the F9P and `/dev/ttyUSB0` for the UM982.
- The generated universal compose stayed free of `mowgli-gps`, `gnss_unicore`, and standalone GNSS sidecars.
- The F9P stayed in RTK float during the sampled NTRIP window.
- The UM982 accepted corrections, exposed correction age through typed status, and intermittently promoted into RTK float during the sampled window.
- The current Unicore limitation is not device access anymore; it is that `RTCMSTATUSA` still has no dedicated ROS projection beyond the generic correction diagnostics.

## Troubleshooting

- No `/gps/fix`: confirm the selected serial device exists inside the runtime and the receiver baud matches the installer-generated `.env`.
- Wrong receiver path: inspect `/dev/serial/by-id` first. If that is unavailable, confirm the actual USB device identity through `/sys/class/tty/*/../manufacturer` and `/sys/class/tty/*/../product` before assuming `ttyACM0` or `ttyUSB0`.
- Stale `/dev/tty*` entries in a container can survive old hardware layouts. When `/dev` and `/sys/class/tty` disagree, trust the live sysfs mapping rather than the stale node list.
- No RTK corrections: confirm NTRIP settings in `docker/.env` and `docker/config/mowgli/mowgli_robot.yaml`, then check `/rtcm` and `/diagnostics`.
- No direct GNSS container in compose: this is expected in `GNSS_STACK=universal`; Universal GNSS now runs inside `mowgli-ros2`.
- Wrong compose shape: regenerate with the installer and inspect `docker/docker-compose.yaml` plus `docker/.env`.
