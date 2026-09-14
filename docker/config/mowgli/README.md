# Mowgli ROS2 Runtime Configuration

This directory is bind-mounted **read-write** into the `mowgli` container at
`/ros2_ws/config/` — the container writes calibration results (dock pose,
datum, IMU yaw, drive PID) back into `mowgli_robot.yaml` from inside. The
`ntrip` and `gps` sidecars mount the same directory read-only, and the GUI
container mounts it at `/mowgli_config`.

`mowgli_robot.yaml` is the **only** file the ROS2 stack reads from this path.
Every other parameter file (Nav2, hardware bridge, twist_mux, behavior tree,
map server, MQTT bridge, …) is loaded from inside the image and cannot be
overridden by dropping a copy in here.

## Git-ignored — your edits are safe

`mowgli_robot.yaml` (and `../cyclonedds.xml`, `../mqtt/mosquitto.conf`,
`../om/mower_config.sh`) are **git-ignored**. The installer seeds them
from `install/config/` on first run and patches a small whitelist of
keys (datum, NTRIP, dock pose, LiDAR flags) on subsequent runs. Any
other edits you make — through the GUI Settings page or by hand —
persist across `git pull` and `install/mowglinext.sh` upgrades.

Caveat: `../cyclonedds.xml` is listed in `.gitignore` but is also **tracked**
in git, so `.gitignore` does not apply to it and a `git pull` can overwrite
your edits. Change `install/config/cyclonedds.xml` too.

If you ever want to reset to the installer's seed:

```bash
cp install/config/mowgli/mowgli_robot.yaml docker/config/mowgli/mowgli_robot.yaml
```

## The one file that is read

| File | Purpose |
|------|---------|
| `mowgli_robot.yaml` | **Per-robot config** — install-time choices (GPS datum, NTRIP, `lidar_enabled`, GNSS receiver/serial, `mower_model`) plus calibration outputs (dock pose, `ticks_per_meter`, wheel PID, `imu_yaw`, magnetometer). Edit this — or better, use the GUI Settings page. |

## What you cannot override here

Dropping a `nav2_params.yaml`, `hardware_bridge.yaml`, `twist_mux.yaml`,
`behavior_tree.yaml` or `mqtt_bridge.yaml` into this directory has **no
effect** — the launch files load those from the package share directory inside
the image, not from `/ros2_ws/config/`. To change them you have to rebuild the
image (or bind-mount over the share dir, as the simulation compose file does).

## How parameter override works

`mowgli_robot.yaml` here is deliberately **sparse**: it holds only install/site
choices and calibration outputs. Every other parameter's *default* lives in the
in-image template `mowgli_bringup/config/mowgli_robot.yaml`. At launch,
`robot_config_util.load_robot_params()` **deep-merges this file over that
template**, so a key you omit falls through to its template default and the
nodes always receive a complete parameter set.

Consequences: to change a value, add it here; to restore a default, **delete
its line** — do not copy the template value in. That deletion is exactly what
the GUI Settings page's "reset" button does, and it is why a template default
bumped in a new release reaches every robot that never overrode it.

## Quick start

1. Edit `mowgli_robot.yaml` — set your GPS datum, NTRIP, and dock pose
2. Restart: `docker compose restart mowgli`
3. To see which defaults you are inheriting from the in-image template:

```bash
# The template every key falls back to
docker exec mowgli-ros2 cat /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/mowgli_robot.yaml

# The other built-in config files (Nav2 base + lidar/no-lidar overlays, …).
# These are read from the image — copying one here does nothing.
docker exec mowgli-ros2 ls /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/
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
