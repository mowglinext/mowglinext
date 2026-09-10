# Mowgli ROS2 Runtime Configuration

This directory is the **versioned seed**, not the runtime config. The installer
copies `mowgli_robot.yaml` from here into `docker/config/mowgli/` (git-ignored,
so operator edits survive `git pull`), and *that* directory is bind-mounted
**read-write** into the `mowgli` container at `/ros2_ws/config/`.

Read-write is deliberate: `calibrate_imu_yaw_node` and `map_server`'s
`/set_docking_point` service splice `dock_pose_x/y/yaw`, `datum_lat/lon` and
`imu_yaw` back into the live file. So on a running robot, edit
`docker/config/mowgli/mowgli_robot.yaml`; editing the seed here only changes
what a *fresh* install starts from.

## The one file the container reads

| File | Purpose |
|------|---------|
| `mowgli_robot.yaml` | The robot's **sparse** config — install-time choices (Universal GNSS receiver + serial device, datum, NTRIP, `mower_model`, `lidar_enabled`) plus per-robot calibration outputs (dock pose, `ticks_per_meter`, `imu_yaw`, magnetometer). Edit this one. |

`hardware_bridge.yaml`, `twist_mux.yaml` and `foxglove_bridge.yaml` also sit in
this directory, but they are **inert reference copies**: the installer never
copies them into `docker/config/mowgli/`, `mowgli.launch.py` loads
`hardware_bridge.yaml` and `twist_mux.yaml` from the package share, and
`foxglove_bridge.launch.py` takes no params file at all. Dropping any other
YAML into the mount has no effect either.

## How parameter override works

`mowgli_robot.yaml` is **sparse**. Every parameter's default lives in the
in-package template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml`; at
launch, `robot_config_util.load_robot_params()` deep-merges the installed file
over that template, so nodes always receive a complete parameter set.

- To change a value — set it in the installed file (or use the GUI Settings page).
- To restore a default — **delete** the line; do not copy the template value in.
  An absent key falls through to the template default, which is exactly how the
  GUI's "reset to default" works.
- Keep the file sparse. A key whose value merely equals the template default is
  pruned by the GUI on write, and CI's config-drift check rejects it in the seed.
- Exception: the calibration keys (`dock_pose_*`, `imu_yaw`, `datum_*`) stay in
  the file as placeholders — the nodes write back by splicing an *existing*
  line, so a missing key is a silent no-op.

## Quick start

1. Edit `docker/config/mowgli/mowgli_robot.yaml` — set your Universal GNSS
   serial device, datum and NTRIP credentials. Dock pose and IMU yaw come from
   calibration, not by hand (see [`docs/FIRST_BOOT.md`](../../../docs/FIRST_BOOT.md)).
2. Restart: `docker compose restart mowgli`
3. To see what you are overriding, read the defaults shipped inside the image
   (read-only — changing them needs an image rebuild):

```bash
# All built-in config files
docker exec mowgli-ros2 ls /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/

# The template your sparse file is merged over
docker exec mowgli-ros2 cat /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/mowgli_robot.yaml
```

Nav2 tuning is not exposed through this mount. The knobs an operator normally
wants (speeds, dock pose, `tool_width`, `min_turning_radius`,
`connector_turn_radius`) are set in `mowgli_robot.yaml` and injected into the
Nav2 parameters at launch; everything else lives in the image's
`nav2_params_base.yaml` plus its `nav2_params_lidar.yaml` /
`nav2_params_no_lidar.yaml` overlay.

## Note on GPS device

The Universal GNSS sidecar takes the device path from `gnss_serial_device` in
`mowgli_robot.yaml` (falling back to the `GNSS_SERIAL_DEVICE` env value, then
`/dev/ttyAMA4`). A UART receiver keeps its fixed node; for a USB receiver,
prefer the stable `/dev/serial/by-id/...` path and use a raw `ttyUSB*` or
`ttyACM*` node only as a temporary diagnostic fallback when by-id is
unavailable.

```yaml
# docker/config/mowgli/mowgli_robot.yaml
mowgli:
    ros__parameters:
        gnss_serial_device: /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00
        gnss_serial_baud: 921600
```

No compose `devices:` mapping is involved: every container that touches
hardware runs privileged with `/dev:/dev` bind-mounted, so the path above is
the only thing to get right.
