# Sensors

Dockerized ROS2 drivers for each supported sensor. Each subdirectory contains a Dockerfile and configuration for one sensor model.

## Supported Sensors

| Sensor | Type | Directory | ROS2 Topic | Protocol |
|--------|------|-----------|------------|----------|
| u-blox ZED-F9P | RTK GPS | [`gps/`](gps/) | `/ublox_gps_node/fix` (NavSatFix) | USB-CDC |
| Unicore UM98x | RTK GPS | [`unicore/`](unicore/) | `/gps/fix` (NavSatFix) | UART/USB |
| Generic NMEA-0183 | GPS (any vendor) | [`nmea/`](nmea/) | `/gps/fix` (NavSatFix), `/gps/vel` (TwistStamped) | UART/USB |
| LDRobot LD19 | 2D LiDAR | [`lidar/`](lidar/) | `/scan` (LaserScan) | UART 230400 |

### GPS backend selection

The installer offers three GNSS backends (`GNSS_BACKEND` env): `gps` (legacy container for generic UBX or NMEA receivers), `ublox` (F9P UBX HP + bundled NTRIP), and `unicore` (UM98x). For generic receivers that emit standard NMEA-0183 sentences (GGA, RMC, GSA, VTG) — Emlid Reach, BN-220, LC29H, NEO-M8N in NMEA mode, etc. — use `GNSS_BACKEND=gps` with `GPS_PROTOCOL=NMEA`. Caveats:

- Position covariance is derived from HDOP only, no UBX `position_accuracy`. The map-frame EKF / `fusion_graph` will see a more pessimistic σ than RTK-Fixed implies.
- RTK still works if the receiver outputs RTK-quality GGA fix-quality codes (4=Fixed, 5=Float), but the project's BT/GUI cannot distinguish Fixed from Float beyond GGA quality.
- NTRIP/RTCM relay is **not bundled**. Configure NTRIP on the receiver itself (Emlid and most modern modules support it natively). Relaying RTCM from a ROS-side ntrip client back to the same `/dev/gps` port requires a custom sidecar — not provided here.

## Adding a New Sensor

To add support for a different GPS or LiDAR model:

1. Create a new directory (e.g., `sensors/lidar-rplidar/`)
2. Add a `Dockerfile` that builds the ROS2 driver and publishes the expected topic
3. Add a `ros2_entrypoint.sh` for environment setup
4. Update `docker/docker-compose.yaml` to point the service's `build.context` at your new directory
5. Ensure the driver publishes on the standard topic (`/scan` for LiDAR, `/ublox_gps_node/fix` or `/gps/fix` for GPS)

## Building

Images are built automatically by the CI workflow (`.github/workflows/docker.yml`) for `linux/amd64` and `linux/arm64`.

To build locally:

```bash
docker build -t mowgli-gps sensors/gps/
docker build -t mowgli-lidar --target runtime sensors/lidar/
```
