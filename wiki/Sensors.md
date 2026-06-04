# Sensors

Sensor integration lives under `sensors/` and is consumed through the installer-generated compose stack.

## GNSS

MowgliNext currently exposes one GNSS contract to the rest of the system:

- `/gps/fix` — `sensor_msgs/msg/NavSatFix`
- `/gps/azimuth` — heading when the receiver provides it
- `/gps/status` — typed GNSS status consumed by the GUI and backend
- `/diagnostics` — receiver + transport diagnostics

The current backends are:

| Backend | Directory | Current role |
|---------|-----------|--------------|
| Shared GPS | `sensors/gps/` | Legacy fallback runtime for u-blox UBX and generic NMEA when `GNSS_STACK=legacy` |
| Unicore | `sensors/unicore/` | Legacy fallback runtime for UM98x when `GNSS_STACK=legacy` |

### GNSS Flow

```text
Receiver
  -> Universal GNSS receiver_node / ntrip_node
  -> /gps/fix + /gps/status + /diagnostics + /rtcm
  -> mowgli_localization/navsat_to_absolute_pose_node
  -> /gps/absolute_pose + /gps/pose_cov
  -> localization / GUI / diagnostics

Legacy-only status path
  -> NavSatFix + /diagnostics
  -> gnss_runtime_state_builder.cpp
  -> /gps/status
```

### Notes

- Universal GNSS is the preferred stack and is launched from `mowgli-ros2` through `mowgli_bringup/universal_gnss.launch.py`.
- The installer now writes a preferred Universal GNSS env contract: `GNSS_STACK`, `GNSS_RECEIVER_FAMILY`, `GNSS_TRANSPORT`, `GNSS_SERIAL_DEVICE`, `GNSS_SERIAL_BAUD`, and `GNSS_NTRIP_*`.
- `921600` is the recommended validation baud for advanced u-blox and Unicore profiles.
- The old standalone NMEA container path has been removed. NMEA now routes through Universal GNSS by default and through `sensors/gps/start_gps.sh` only in `GNSS_STACK=legacy`.
- The old standalone `ublox_gnss.launch.py` bringup has been removed.
- `GNSS_STATUS_SOURCE=universal` disables the Mowgli-local `/gps/status` publisher and skips the local `/diagnostics` GNSS parser subscription.
- In universal mode, Universal GNSS owns `/gps/fix`, `/gps/status`, `/diagnostics`, and `/rtcm`.
- The GUI/backend still consumes `/gps/status` through the existing Mowgli schema, but that shape is now produced by a thin backend adapter in universal mode instead of new frontend vendor parsing.
- `sensors/gps`, `sensors/unicore`, and `gnss_runtime_state_builder.cpp` still remain for the legacy fallback path until field validation is complete.
- Do not commit real `GNSS_NTRIP_PASSWORD` values or copy them into docs/logs.

## LiDAR

LiDAR support remains installer-selected through dedicated compose fragments.

| Sensor | Topic | Notes |
|--------|-------|-------|
| LDLiDAR / RPLIDAR / STL27L | `/scan` | Selected by `LIDAR_TYPE` and corresponding compose fragment |
