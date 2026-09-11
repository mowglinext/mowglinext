# Sensors

Dockerized ROS2 drivers for each supported sensor. Each subdirectory contains a Dockerfile and configuration for one sensor model.

## Supported Sensors

| Sensor | Type | Directory | ROS2 Topic | Protocol |
|--------|------|-----------|------------|----------|
| Universal GNSS sidecar | GNSS | [`gps/`](gps/) | `/gps/fix` (NavSatFix) + `/gps/status` (GnssStatus) + `/rtcm` (rtcm_msgs/Message) | UART/USB |
| External MowgliMAVROS sidecar integration | Hardware/GNSS backend | [`mavros/`](mavros/) | MAVROS + external backend contract | Pixhawk MAVLink |
| LDRobot LD19 | 2D LiDAR | [`lidar-ldlidar/`](lidar-ldlidar/) | `/scan` (LaserScan) | UART 230400 (hardcoded in `ldlidar.yaml`) |
| LDRobot STL27L | 2D LiDAR | [`lidar-stl27l/`](lidar-stl27l/) | `/scan` (LaserScan) | UART, `LIDAR_BAUD` (921600) |
| Slamtec RPLiDAR A1/A2/A3/S1/S2/S3/C1 | 2D LiDAR | [`lidar-rplidar/`](lidar-rplidar/) | `/scan` (LaserScan) | UART, `LIDAR_BAUD` |

Exactly one LiDAR container is composed, selected by `LIDAR_TYPE=ldlidar|rplidar|stl27l` in `docker/.env` (`install/lib/compose.sh`). `LIDAR_ENABLED` there only decides whether that container runs — the ROS-side LiDAR mode is `lidar_enabled` in `mowgli_robot.yaml`.

### External MAVROS backend

`sensors/mavros/` is a source-free MowgliNext integration contract for the
independently released MowgliMAVROS image. It does not build, vendor, or
publish that image. See its [`README`](mavros/README.md) for the immutable
image pin, device/config mounts, ownership split, and validation command.

### GNSS receiver selection

Direct GNSS installs use the externally published Universal GNSS sidecar only
(`GNSS_STACK` accepts `universal|disabled`; "no GNSS" means not composing the
container). `UNIVERSAL_GNSS_IMAGE` is required and must be pinned separately
from MowgliNext `IMAGE_TAG`. `GNSS_DEVICE` maps one stable host path (prefer
`/dev/serial/by-id/...`) to `/dev/gnss-receiver`; the sidecar is the sole
owner of that device, parsing and NTRIP. `HARDWARE_BACKEND=mavros` does not
disable this stack. Receiver choice remains **YAML → env → default**:
`gnss_receiver_family` in `mowgli_robot.yaml`, then `GNSS_RECEIVER_FAMILY` in
`docker/.env`, then `auto`. MowgliNext generates the image-specific parameter
file and runs the existing `mowgli_gnss_bridge` in `mowgli-ros2` solely to
adapt UG types onto the public contract:

- Common runtime topics stay backend-agnostic: `/gps/fix` remains `sensor_msgs/NavSatFix`, `/gps/status` carries typed GNSS/RTK state (`mowgli_interfaces/GnssStatus`), `/rtcm` mirrors the RTCM stream as `rtcm_msgs/Message`, and `/diagnostics` stays human/debug-only.
- Position covariance is taken from the receiver's own reported horizontal/vertical accuracy and published as `COVARIANCE_TYPE_APPROXIMATED`; if either term is missing it is left `COVARIANCE_TYPE_UNKNOWN` rather than invented. Downstream, `navsat_to_absolute_pose_node` turns it into the `/gps/pose_cov` 1-sigma fed to `fusion_graph`; an unknown covariance type becomes a deliberately large 10 m sigma, which exceeds the node's 0.5 m reject threshold, so no `/gps/pose_cov` is published at all rather than a fabricated one.
- Generic NMEA receivers are supported through the Universal GNSS parser family selection instead of a separate runtime path.
- NTRIP/RTCM forwarding is handled only in the Universal GNSS sidecar when `ntrip_enabled=true`. With `false`, it runs receiver-only; `/rtcm` has no producer. UG internal topics remain `/universal_gnss_receiver/*`; the bridge publishes the public `/gps/status` and `/rtcm` types.

## Adding a New Sensor

To add support for a different GPS or LiDAR model:

1. Create a new directory (e.g., `sensors/lidar-<model>/`)
2. Add a `Dockerfile` that builds the ROS2 driver and publishes the expected topic
3. Add a `ros2_entrypoint.sh` for environment setup
4. Add a CI caller `.github/workflows/sensors-<name>.yml` that calls the reusable `_sensor-docker.yml` with your `image`/`context` (and `target: runtime` if your Dockerfile has one), so the image is published to GHCR
5. Add a compose fragment `install/compose/docker-compose.<name>.yml` referencing that published `image:`, and wire it into `install/lib/compose.sh` so the installer composes it. Do **not** hand-edit `docker/docker-compose.yaml` — it is generated from those fragments and gitignored (regenerate with `./docker/stack.sh regen`)
6. Ensure the driver publishes on the standard topic contract (`/scan` with `frame_id: lidar_link` for LiDAR; `/gps/fix` for GNSS, with `/gps/status` and `/rtcm` produced through the `mowgli_gnss_bridge` adapter layer)

## Building

Each image has its own CI caller (`.github/workflows/sensors-{gps,lidar-ldlidar,lidar-rplidar,lidar-stl27l}.yml`), all delegating to the reusable `.github/workflows/_sensor-docker.yml`, which builds `linux/amd64` and `linux/arm64` and pushes a merged multi-arch manifest to GHCR.

To build locally:

```bash
# Universal GNSS is built and published by its own repository. MowgliNext does
# not build or publish a GPS image; set UNIVERSAL_GNSS_IMAGE to its published
# pinned reference before composing the stack.

# LiDAR images — build context is the sensor directory
docker build -t mowgli-lidar-ldlidar --target runtime sensors/lidar-ldlidar/
docker build -t mowgli-lidar-stl27l  --target runtime sensors/lidar-stl27l/
docker build -t mowgli-lidar-rplidar --target runtime sensors/lidar-rplidar/
```

The legacy `sensors/gps/Dockerfile` is not part of the deployment path. The
bridge is built into `mowgli-ros2`; MowgliNext does not duplicate Universal
GNSS sources or runtime ownership.

## For contributors

The Claude-facing reference index lives under [`docs/claude/`](../docs/claude/): per-area codemaps
([`codemaps/deploy.md`](../docs/claude/codemaps/deploy.md) covers `install/` + `docker/` + `sensors/`),
[`ros-interfaces.md`](../docs/claude/ros-interfaces.md), [`parameters.md`](../docs/claude/parameters.md),
[`testing-ci.md`](../docs/claude/testing-ci.md) and [`doc-index.md`](../docs/claude/doc-index.md)
(which document is authoritative vs historical). [`sensors/CLAUDE.md`](CLAUDE.md) holds the working
notes and gotchas for this tree.
