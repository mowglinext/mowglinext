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
- For USB receivers, `GNSS_SERIAL_DEVICE` should normally be a stable `/dev/serial/by-id/...` path rather than a raw `ttyACM*` or `ttyUSB*` node.
- `921600` is the recommended validation baud for advanced u-blox and Unicore profiles.
- Corrected field validation on June 4, 2026 confirmed the Universal GNSS path on both a live u-blox F9P and a live Unicore UM982 with `/gps/fix`, `/gps/status`, `/diagnostics`, and `/rtcm` active in universal mode.
- The preferred stable receiver paths are `/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00` for the F9P and `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` for the UM982.
- The corrected raw tty mappings from that validation were `/dev/ttyACM0` for the F9P and `/dev/ttyUSB0` for the UM982.
- The F9P stayed in RTK float through the sampled NTRIP window and did not reach RTK fixed.
- The UM982 mostly stayed in a plain fix state but did intermittently promote into RTK float while corrections were active.
- Raw UM982 output after the NTRIP session showed `PSRDIFF` position solutions and `RTKSTATUSA` records on the wire.
- `RTCMSTATUSA` was not observed in the sampled raw output windows, and Universal GNSS still treats it as semantic-only rather than a dedicated ROS status field.
- The old standalone NMEA container path has been removed. NMEA now routes through Universal GNSS by default and through `sensors/gps/start_gps.sh` only in `GNSS_STACK=legacy`.
- The old standalone `ublox_gnss.launch.py` bringup has been removed.
- `GNSS_STATUS_SOURCE=universal` disables the Mowgli-local `/gps/status` publisher and skips the local `/diagnostics` GNSS parser subscription.
- In universal mode, Universal GNSS owns `/gps/fix`, `/gps/status`, `/diagnostics`, and `/rtcm`.
- The GUI/backend still consumes `/gps/status` through the existing Mowgli schema, but that shape is now produced by a thin backend adapter in universal mode instead of new frontend vendor parsing.
- `sensors/gps`, `sensors/unicore`, and `gnss_runtime_state_builder.cpp` still remain for the legacy fallback path until field validation is complete.
- Do not commit real `GNSS_NTRIP_PASSWORD` values or copy them into docs/logs.
- The devcontainer now mirrors the host `/dev` tree at `/host-dev` and re-links `/dev/serial/by-id` when the host provides it.
- Prefer `/dev/serial/by-id` when it is available. If it is missing in the runtime, confirm the live device through `/sys/class/tty/*/../manufacturer` and `/sys/class/tty/*/../product` only as a diagnostic fallback before wiring `GNSS_SERIAL_DEVICE`.
- Be cautious with raw `/dev/tty*` enumeration in containers: stale device nodes can persist even when the live sysfs mapping has changed.

## LiDAR

LiDAR support remains installer-selected through dedicated compose fragments.

| Sensor | Topic | Notes |
|--------|-------|-------|
| LDLiDAR / RPLIDAR / STL27L | `/scan` | Selected by `LIDAR_TYPE` and corresponding compose fragment |
