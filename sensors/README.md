# Sensors

Dockerized ROS2 drivers for each supported sensor. Each subdirectory contains a Dockerfile and configuration for one sensor model.

## Supported Sensors

| Sensor | Type | Directory | ROS2 Topic | Protocol |
|--------|------|-----------|------------|----------|
| Universal GNSS sidecar | GNSS | [`gps/`](gps/) | `/gps/fix` (NavSatFix) + `/gps/status` (GnssStatus) | UART/USB |
| LDRobot LD19 | 2D LiDAR | [`lidar/`](lidar/) | `/scan` (LaserScan) | UART 230400 |

### GNSS receiver selection

Direct GNSS installs now use the Universal GNSS sidecar only. Receiver choice is expressed through `GNSS_RECEIVER_FAMILY=auto|ublox|unicore|nmea`, while the public runtime contract stays backend-agnostic:

- Common runtime topics stay backend-agnostic: `/gps/fix` remains `sensor_msgs/NavSatFix`, `/gps/azimuth` remains `compass_msgs/Azimuth` when available, `/gps/status` carries typed GNSS/RTK state, and `/diagnostics` stays human/debug-only.

- Position covariance is derived from HDOP only, so the map-frame EKF / `fusion_graph` can remain more conservative than RTK-Fixed implies.
- Generic NMEA receivers are supported through the Universal GNSS parser family selection instead of a separate runtime path.
- NTRIP/RTCM forwarding is handled in the Universal GNSS sidecar path.

## Adding a New Sensor

To add support for a different GPS or LiDAR model:

1. Create a new directory (e.g., `sensors/lidar-rplidar/`)
2. Add a `Dockerfile` that builds the ROS2 driver and publishes the expected topic
3. Add a `ros2_entrypoint.sh` for environment setup
4. Update `docker/docker-compose.yaml` to point the service's `build.context` at your new directory
5. Ensure the driver publishes on the standard topic contract (`/scan` for LiDAR; `/gps/fix` plus optional `/gps/azimuth` for GNSS, with `/gps/status` produced through the shared adapter layer)

## Building

Images are built automatically by the CI workflow (`.github/workflows/docker.yml`) for `linux/amd64` and `linux/arm64`.

To build locally:

```bash
docker build -t mowgli-gps -f sensors/gps/Dockerfile .
docker build -t mowgli-lidar --target runtime sensors/lidar/
```

The `gps` image now expects the monorepo root as its Docker build context so it
can bundle the vendored Universal GNSS prep overlay from `ros2/src/`.
