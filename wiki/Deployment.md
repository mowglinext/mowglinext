# Deployment

MowgliNext is deployed through an installer-generated compose stack written to `docker/docker-compose.yaml`.

## Compose Generation

The installer selects fragments from `install/compose/` based on hardware choices:

- `docker-compose.base.yml`
- `docker-compose.gui.yml`
- one GNSS fragment:
  - `docker-compose.gps.yml` for the shared GPS service
  - `docker-compose.unicore.yaml` for the UM98x path
- optional LiDAR / MAVROS / TF-Luna fragments

## Runtime Services

| Container | Purpose |
|-----------|---------|
| `mowgli-ros2` | Main ROS2 stack, localization, Nav2, behavior tree, API |
| `mowgli-gps` | GNSS runtime selected by the installer |
| `mowgli-lidar` | LiDAR runtime when enabled |
| `mowgli-gui` | Web UI |
| `mowgli-mqtt` | MQTT broker |
| `mowgli-watchtower` | Image updates |

## GNSS Deployment Shape

```text
GNSS_BACKEND=gps
  -> install/compose/docker-compose.gps.yml
  -> sensors/gps/start_gps.sh

GNSS_BACKEND=unicore
  -> install/compose/docker-compose.unicore.yaml
  -> sensors/unicore/start_gps.sh
```

`GNSS_BACKEND=ublox` is still accepted as a compatibility preset, but it resolves onto the shared GPS service instead of a separate container.

`GNSS_STATUS_SOURCE=universal` hands the typed `/gps/status` topic over to Universal GNSS while Mowgli keeps `/gps/absolute_pose` and `/gps/pose_cov` for downstream localization consumers.

## Troubleshooting

- No `/gps/fix`: confirm the selected device path exists inside the runtime and the receiver baud matches the installer-generated `.env`.
- No RTK corrections: confirm NTRIP settings in `docker/config/mowgli/mowgli_robot.yaml`, then check `/ntrip_client/rtcm` and `/diagnostics`.
- Wrong compose shape: regenerate with the installer and inspect `docker/docker-compose.yaml` plus `docker/.env`.
