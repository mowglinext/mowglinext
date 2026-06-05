# Mowgli ROS2 Runtime Configuration

This directory is bind-mounted read-only into the `mowgli` container at
`/ros2_ws/config/`.

The `mowgli_ros2` runtime image reads parameter files from that path at launch.
Place any customised YAML parameter files here to override the built-in
defaults without rebuilding the image.

## Shipped default

| File | Purpose |
|------|---------|
| `mowgli_robot.yaml` | **Site-specific overrides** — Universal GNSS serial settings, datum, NTRIP, dock pose, battery, mowing speed. Edit this first. |

## Additional overrideable files

| File | Purpose |
|------|---------|
| `hardware_bridge.yaml` | Serial port, baud rate, heartbeat/publish rates |
| `foxglove_bridge.yaml` | Foxglove bridge settings |
| `twist_mux.yaml` | Twist mux priorities |

## How parameter override works

The launch files in `mowgli_bringup` check for the presence of files under
`/ros2_ws/config/` and, when found, pass them to each node via the
`params_file` argument.  If a file is absent the built-in default is used.

## Quick start

1. Edit `mowgli_robot.yaml` — set your Universal GNSS serial device, datum, NTRIP, and dock pose
2. Restart: `docker compose restart mowgli`
3. For advanced tuning, copy more files from the image:

```bash
# See all built-in config files
docker exec mowgli-ros2 ls /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/

# Copy one out for local editing
docker exec mowgli-ros2 cat /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/nav2_params.yaml > config/mowgli/nav2_params.yaml
```

## Note on GPS device

Prefer wiring Universal GNSS to the stable `/dev/serial/by-id/...` path for
your receiver. Use a raw `ttyUSB*` or `ttyACM*` node only as a temporary
diagnostic fallback when by-id is unavailable.

If you need a manual `devices` override, point it at the by-id path:

```yaml
services:
  mowgli:
    devices:
      - /dev/mowgli:/dev/mowgli
      - /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00:/dev/gps
```
