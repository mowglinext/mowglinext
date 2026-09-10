# Mowgli Docker — v3 (ROS2 Kilted)

Docker Compose deployment for the **Mowgli** open-source robot mower.
v3 is a ground-up rewrite: the ROS1 Noetic stack has been replaced by a
single `mowgli_ros2` container running **ROS2 Kilted**, Nav2, the
`fusion_graph` GTSAM factor-graph localizer, and a full behavior-tree
coverage planner.

## What changed from v2

| v2 (ROS1 Noetic) | v3 (ROS2 Kilted) |
|---|---|
| `roscore` | Removed — DDS has no master |
| `rosserial` | Removed — hardware bridge is inside `mowgli_ros2` |
| `openmower` | Replaced by `mowgli_ros2` (Nav2 + BT + coverage) |
| Foxglove Bridge as separate image | Built into `mowgli_ros2`, enabled via launch arg |
| Rosbridge as separate container | Removed — the GUI and Foxglove Studio both speak to Foxglove Bridge on `:8765` |
| FastDDS | Cyclone DDS (all containers share `config/cyclonedds.xml`) |

---

## Hardware requirements

### Compute board

Any ARM64 SBC running Linux with Docker support. The project is actively
used on **Rockchip** boards (RK3566, RK3588). A Raspberry Pi 4 or 5
also works.

Minimum recommended: 4-core ARM64, 4 GB RAM, 16 GB storage.

### Mower models

Set `mower_model` in `config/mowgli/mowgli_robot.yaml` to one of:

- `YardForce500`
- `YardForce500B`
- `YardForceSA650`
- `YardForce900ECO`
- `LUV1000RI`
- `Sabo` (Sabo Mestercut)
- `CUSTOM`

### Sensors and serial devices

| Device | Preferred stable path | Compatibility symlink | USB IDs (for udev) |
|---|---|---|---|
| Mowgli STM32 board | `/dev/mowgli` | USB-CDC | `product=="Mowgli"` |
| u-blox ZED-F9P (simpleRTK2B) | `/dev/serial/by-id/...` | `/dev/gps` | VID `1546` PID `01a9` |
| u-blox RTK1010Board (ESP USB-CDC) | `/dev/serial/by-id/...` | `/dev/gps` | VID `303a` PID `4001` |
| LDRobot LD19 LiDAR | `/dev/ttyS1` (hardware UART) | `/dev/lidar` | — |

The LiDAR connects to a hardware UART on the compute board, not USB. The LD19
image opens `/dev/lidar` at 230400 baud — both are hardcoded in
`sensors/lidar-ldlidar/ldlidar.yaml`, so the `/dev/lidar` udev symlink must
exist. `LIDAR_PORT` / `LIDAR_BAUD` in `.env` only reach the RPLiDAR and STL27L
driver images.

For Universal GNSS, prefer the stable `/dev/serial/by-id/...` receiver path in
`.env` and let `/dev/gps` remain a compatibility symlink only.

---

## Quick start

### Option A — web composer + one-line install (recommended)

Visit [mowgli.garden](https://mowgli.garden/#getting-started) to configure
your hardware (GPS, LiDAR, rangefinders) and get a personalized install
command. Or run the installer directly:

```bash
curl -sSL https://mowgli.garden/install.sh | bash
```

The installer handles Docker, udev rules, sensor configuration, image pull,
and first startup. Run it again at any time to upgrade.

To run diagnostics only against an existing installation:

```bash
cd ~/mowglinext/install && ./mowglinext.sh --check
```

### Option B — manual install

#### 1. Install Docker

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER
# Log out and back in before proceeding
```

Docker Compose v2 (the `docker compose` plugin, not `docker-compose`) is
required. Verify with `docker compose version`.

#### 2. Clone this repository

```bash
git clone --depth 1 https://github.com/mowglinext/mowglinext.git
cd mowglinext/docker
```

#### 3. Install udev rules (stable device symlinks)

Create `/etc/udev/rules.d/50-mowgli.rules`:

```
# Mowgli STM32 board
SUBSYSTEM=="tty", ATTRS{product}=="Mowgli", SYMLINK+="mowgli", MODE="0666"

# GPS: simpleRTK2B (u-blox ZED-F9P)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps", MODE="0666"

# GPS: RTK1010Board (ESP32 USB-CDC)
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4001", SYMLINK+="gps", MODE="0666"

# LiDAR on a hardware UART — adjust KERNEL to your board's UART device
SUBSYSTEM=="tty", KERNEL=="ttyS1", SYMLINK+="lidar", MODE="0666"
```

Reload and verify:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/mowgli /dev/gps /dev/lidar
```

#### 4. Create `.env`

```bash
cp .env.example .env
```

Edit `.env` — see the [Configuration reference](#env--image-tags-and-ports)
section below for all variables.

#### 5. Seed and edit `config/mowgli/mowgli_robot.yaml`

`docker/config/mowgli/mowgli_robot.yaml` is git-ignored runtime state. The full
installer seeds it; for a manual deploy copy the sparse seed yourself:

```bash
mkdir -p config/mowgli
cp ../install/config/mowgli/mowgli_robot.yaml config/mowgli/mowgli_robot.yaml
```

At minimum, set your GPS datum (dock coordinates) and NTRIP parameters. See
[mowgli_robot.yaml reference](#mowgli_robotyaml-key-parameters).

#### 6. Generate the compose file, pull images and start

`docker/docker-compose.yaml` is **generated** by merging the fragments in
`install/compose/` against `.env`; it is not checked in. Drive the stack with
`stack.sh`, which regenerates it, rather than calling `docker compose` directly:

```bash
./stack.sh pull
./stack.sh up
```

#### 7. Open the GUI

```
http://<board-ip>:4006
```

---

## Configuration reference

### `.env` — image tags and ports

Copy `.env.example` to `.env` and edit. These are the keys `.env.example`
ships; the full installer writes many more (`GNSS_*`, `LIDAR_TYPE`,
`HARDWARE_BACKEND`, `MAVROS_*`, `IMAGE_TAG`, …) — their defaults live in
`install/lib/env.sh` `setup_env`.

| Variable | Default | Description |
|---|---|---|
| `COMPOSE_PROJECT_NAME` | `install` | Compose project name — prefixes the `mowgli_maps` named volume. Keep it stable; renaming it orphans the persisted map data |
| `ENABLE_MQTT` | `false` | Compose the `mowgli-mqtt` broker. Honoured by `stack.sh` only — the full installer always includes it |
| `ENABLE_WATCHTOWER` | `false` | Compose `mowgli-watchtower`. `stack.sh` only, same as above |
| `ROS_DOMAIN_ID` | `0` | DDS domain ID — must be the same across all containers |
| `MOWER_IP` | `10.0.0.161` | Informational only — printed by the login MOTD, read by nothing else |
| `LIDAR_PORT` | `/dev/ttyS1` | Device path passed to the RPLiDAR / STL27L drivers (the LD19 image ignores it and opens `/dev/lidar`) |
| `LIDAR_BAUD` | `230400` | Baud rate for the RPLiDAR / STL27L drivers (the LD19 image ignores it) |
| `MOWGLI_ROS2_IMAGE` | `ghcr.io/mowglinext/mowglinext/mowgli-ros2:main` | Full ROS2 stack |
| `GPS_IMAGE` | `ghcr.io/mowglinext/mowglinext/gps:main` | Universal GNSS sidecar + NTRIP client |
| `LIDAR_IMAGE` | `ghcr.io/mowglinext/mowglinext/lidar-ldlidar:main` | LD19 LiDAR driver |
| `MAVROS_IMAGE` | `ghcr.io/mowglinext/mowglinext/mavros:main` | MAVROS bridge |
| `GUI_IMAGE` | `ghcr.io/mowglinext/mowglinext/mowglinext-gui:main` | Web GUI |

For Universal GNSS, set `GNSS_SERIAL_DEVICE=/dev/serial/by-id/...` — note that
`gnss_serial_device` in `mowgli_robot.yaml` wins over the env value when both
are set. The legacy `GPS_PORT` / `GPS_BAUD` / `GPS_PROTOCOL` keys are obsolete;
the installer deletes them from `.env` on every run. Use raw `ttyACM*` or
`ttyUSB*` paths only as a temporary diagnostic fallback when
`/dev/serial/by-id` is unavailable.

### `mowgli_robot.yaml` key parameters

`config/mowgli/mowgli_robot.yaml` is bind-mounted **read-write** into the
`mowgli` container (calibration nodes splice `dock_pose_*`, `datum_*` and
`imu_yaw` back into it) and read-only into `gps`. The GUI writes this file
directly; restart the affected containers to apply manual edits.

The installed file is deliberately **sparse** — it holds only install-time
choices and calibration outputs. Every other parameter's default lives in the
in-package template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml`, which
the launch files deep-merge *underneath* it, so an absent key falls through to
its template default and deleting a key restores that default. Do not paste
defaults in; the tables below give the template value for reference.

**GPS and datum**

| Parameter | Example | Description |
|---|---|---|
| `datum_lat` | `48.879640599999995` | Map origin latitude — set to your dock's GPS coordinates |
| `datum_lon` | `2.1728332999999997` | Map origin longitude |
| `gnss_serial_device` | `/dev/ttyAMA4` | Receiver device path — a UART, or a `/dev/serial/by-id/...` path for USB receivers |
| `gnss_serial_baud` | `921600` | Serial baud rate for the receiver |
| `gnss_receiver_family` | `auto` | Receiver family for the Universal GNSS driver (`auto` probes) |
| `gps_timeout_sec` | `5.0` | Seconds to wait for a fix before raising a warning |
| `gps_x` | `0.3` | Antenna lever arm from `base_link` (rear wheel axis), metres (forward) |
| `gps_y` | `0.0` | Antenna lever arm, metres (left) |
| `gps_z` | `0.20` | Antenna height above base plane, metres |

**NTRIP RTK corrections**

| Parameter | Example | Description |
|---|---|---|
| `ntrip_enabled` | `true` | Enable NTRIP RTK correction stream |
| `ntrip_host` | `crtk.net` | NTRIP caster hostname (Centipede now lives at `crtk.net`) |
| `ntrip_port` | `2101` | NTRIP caster port |
| `ntrip_user` | `centipede` | Username (Centipede network is free, no registration needed) |
| `ntrip_password` | `centipede` | Password |
| `ntrip_mountpoint` | `NEAR` | Mountpoint — `NEAR` auto-routes to the closest base via NMEA GGA (use `NEAR4` on legacy receivers, or pick a specific base from https://centipede.fr) |

**Dock and undocking**

| Parameter | Example | Description |
|---|---|---|
| `dock_pose_x` | `0` | Dock position X in map frame (metres) |
| `dock_pose_y` | `0` | Dock position Y in map frame (metres) |
| `dock_pose_yaw` | `3.8921` | Dock approach heading (radians) |
| `dock_approach_distance` | `1.5` | Stop distance before the final dock approach (metres) — injected as nav2 `simple_charging_dock.staging_x_offset` |
| `undock_distance` | `1.5` | How far to reverse before turning (metres) |
| `undock_speed` | `0.16` | Reverse speed during undocking (m/s) |
| `dock_use_charger_detection` | `true` | Confirm the dock from the charging current (injected at launch into nav2 `simple_charging_dock.use_battery_status`). `false` = dock confirmed on pose proximity alone |

**Robot geometry**

| Parameter | Example | Description |
|---|---|---|
| `mower_model` | `YardForce500` | Hardware model — determines URDF and firmware expectations |
| `wheel_radius` | `0.04475` | Wheel radius in metres |
| `wheel_track` | `0.325` | Lateral distance between wheel centres (metres) |
| `ticks_per_revolution` | `84` | Encoder ticks per full wheel revolution. The scale the bridge actually uses is `ticks_per_meter` (default `399.0`), calibrated per robot |
| `chassis_center_x` | `0.18` | Longitudinal offset from axle to chassis centre (metres) |
| `blade_radius` | `0.09` | Cutting disc radius (metres) |
| `tool_width` | `0.18` | Effective cut width used for coverage path spacing (metres) |

**LiDAR mounting**

| Parameter | Example | Description |
|---|---|---|
| `lidar_enabled` | `true` | Enable LiDAR obstacle detection plus the `fusion_graph` scan-matching / loop-closure factors |
| `lidar_x` | `0.0` | LiDAR position, forward from `base_link` (metres) |
| `lidar_y` | `0.024` | LiDAR position, left from `base_link` (metres) |
| `lidar_z` | `0.30` | LiDAR height above base plane (metres) |
| `lidar_yaw` | `3.1408` | LiDAR rotation in radians (≈ π = 180° = scanner facing rear) |

**Mowing behaviour**

| Parameter | Example | Description |
|---|---|---|
| `mowing_speed` | `0.2` | Mowing speed in m/s |
| `transit_speed` | `0.2` | Transit-to-area speed in m/s |
| `path_spacing` | `0.18` | **Deprecated / informational** — no node reads it. Swath spacing is `tool_width − swath_overlap` |
| `swath_overlap` | `0.02` | How much adjacent coverage swaths overlap (metres) — the live swath-spacing knob |
| `headland_width` | `0.18` | **Inert** — the behavior tree derives the headland from `chassis_width` and overrides this in the coverage goal |
| `num_headland_passes` | `2` | Concentric perimeter rings before the inner field. `<0` = none, `0` = auto |
| `min_turning_radius` | `0.15` | Floor on every coverage turn-around / corner-fillet arc (metres) |

**Battery**

| Parameter | Example | Description |
|---|---|---|
| `battery_full_voltage` | `28.0` | Voltage threshold for "fully charged" |
| `battery_empty_voltage` | `24` | Voltage threshold for "return to dock" |
| `battery_critical_voltage` | `23` | Voltage threshold for emergency stop |

**Removed keys**

`slam_mode`, `map_save_path` and `map_save_on_dock` are dead. slam_toolbox was
removed in favour of the `fusion_graph` (GTSAM iSAM2) localizer, and nothing in
the stack reads them any more. They may still be present in an older installed
`mowgli_robot.yaml`; deleting them changes nothing.

### Advanced parameters

`/ros2_ws/config/` (the `config/mowgli/` bind mount) is **not** a general
override directory: `mowgli_robot.yaml` is the only file the launch files read
from it. Dropping a `nav2_params.yaml`, `localization.yaml`,
`hardware_bridge.yaml` or `mqtt_bridge.yaml` in there has no effect — every
other config is loaded from the package share directory baked into the image.

So the operator-facing surface is `mowgli_robot.yaml` (which the launch files
inject into the relevant nodes) plus the GUI Settings page. Everything else —
Nav2 costmaps/planner/controller, the collision monitor, behaviour-tree
internals, coverage-server tuning, the MQTT bridge — is changed by editing the
package `config/` files in `ros2/src/` and rebuilding the image. Note that Nav2
params are a shared base plus a thin LiDAR / no-LiDAR overlay
(`nav2_params_base.yaml` + `nav2_params_{lidar,no_lidar}.yaml`), deep-merged at
launch; there is no single `nav2_params.yaml`.

To read a built-in default:

```bash
# List the bringup configs shipped in the image
docker exec mowgli-ros2 ls /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/

# Print one
docker exec mowgli-ros2 cat \
  /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/nav2_params_base.yaml
```

---

## Container architecture

```
┌─────────────────────────────────────────────────────────┐
│  Docker host  (ARM64 SBC, network_mode: host)           │
│                                                         │
│  ┌────────────────────────────────────────────────┐     │
│  │  mowgli-ros2                                   │     │
│  │  ros2 launch mowgli_bringup full_system.launch │     │
│  │  ├─ hardware_bridge  ←→  /dev/mowgli (STM32)   │     │
│  │  ├─ Nav2 stack (planner, controllers, costmaps)│     │
│  │  ├─ behavior_tree_node (mission logic)         │     │
│  │  ├─ map_server + coverage_server (areas, F2C)  │     │
│  │  ├─ fusion_graph  ←  /gps/fix, /wheel_odom,    │     │
│  │  │                  /imu/data, /scan_deskewed  │     │
│  │  │    → map→odom AND odom→base_footprint       │     │
│  │  └─ foxglove_bridge   :8765                    │     │
│  └────────────────────────────────────────────────┘     │
│                                                         │
│  ┌──────────────────┐   ┌──────────────────────────┐   │
│  │  mowgli-gps      │   │  mowgli-lidar            │   │
│  │  receiver_node   │   │  ldlidar_node (LD19)     │   │
│  │  ntrip_node      │   │  publishes /scan          │   │
│  │  gnss_bridge     │   │  frame_id: lidar_link     │   │
│  │  pub: /gps/fix,  │   │  port: /dev/lidar         │   │
│  │   /gps/status    │   │  baud: 230400             │   │
│  └──────────────────┘   └──────────────────────────┘   │
│                                                         │
│  ┌──────────────────┐   ┌──────────────────────────┐   │
│  │  mowgli-gui      │   │  mowgli-mqtt             │   │
│  │  Go + React      │   │  eclipse-mosquitto        │   │
│  │  ws://…:8765     │   │  :1883 (MQTT)             │   │
│  │  port: 4006      │   │  :9001 (MQTT-WS)          │   │
│  └──────────────────┘   └──────────────────────────┘   │
│                                                         │
│  ┌──────────────────────────────────────────────────┐   │
│  │  mowgli-watchtower  (polls gui label every 4h)   │   │
│  └──────────────────────────────────────────────────┘   │
│                                                         │
│  Volume: mowgli_maps  →  /ros2_ws/maps (mowgli + gui)   │
└─────────────────────────────────────────────────────────┘
```

### Service summary

| Container | Image | Purpose | Exposed ports |
|---|---|---|---|
| `mowgli-ros2` | `ghcr.io/mowglinext/mowglinext/mowgli-ros2:main` | Full ROS2 stack: hardware bridge, Nav2, behavior tree, map/coverage servers, fusion_graph localizer, Foxglove Bridge | 8765 (Foxglove) |
| `mowgli-gps` | `ghcr.io/mowglinext/mowglinext/gps:main` | Universal GNSS sidecar (u-blox / Unicore / NMEA) + NTRIP RTK corrections; publishes `/gps/fix`, `/gps/status`, `/rtcm` | — |
| `mowgli-lidar` | `ghcr.io/mowglinext/mowglinext/lidar-ldlidar:main` | LDRobot LD19 driver, publishes `/scan` | — |
| `mowgli-gui` | `ghcr.io/mowglinext/mowglinext/mowglinext-gui:main` | Web UI — area mapping, mowing control, config editor. Talks to the stack over Foxglove Bridge | 4006 (host networking) |
| `mowgli-mqtt` | `eclipse-mosquitto:latest` | MQTT broker for Home Assistant and telemetry | 1883, 9001 |
| `mowgli-watchtower` | `ghcr.io/nicholas-fedor/watchtower:latest` | Auto-updates containers labelled `com.centurylinklabs.watchtower.enable: "true"` | — |

`mowgli-ros2` also carries the ROS2 package `mowgli_tools`, built from the
sidecar source at `tools/motor/`. This is required by the MowgliNext GUI Drive
Motor assistant, which launches:

```bash
source /opt/ros/kilted/setup.bash
source /ros2_ws/install/setup.bash
ros2 run mowgli_tools tune_drive_pid --help
```

inside the running `mowgli-ros2` container.

### DDS middleware

All ROS2 containers use **Cyclone DDS** (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).
The shared config at `config/cyclonedds.xml` is bind-mounted to
`/cyclonedds.xml` in every container. Because all containers run with
`network_mode: host`, every ROS2 participant shares the host loopback, so the
config pins DDS to `lo` and disables multicast — keeping discovery and traffic
off the WiFi/Ethernet NICs (letting DDS pick an external interface broke
GPS/RTK, see issue #418):

```xml
<General>
  <AllowMulticast>false</AllowMulticast>
  <Interfaces>
    <NetworkInterface name="lo" priority="default"/>
  </Interfaces>
</General>
<Discovery>
  <MaxAutoParticipantIndex>500</MaxAutoParticipantIndex>
  <Peers>
    <Peer Address="localhost"/>
  </Peers>
</Discovery>
```

This raises the default participant ceiling to handle the 35+ DDS
participants that the full stack starts simultaneously.

All containers run with `network_mode: host` and `ipc: host` so DDS
discovery works over the loopback interface without multicast routing.
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` restricts discovery to the local
machine, preventing DDS traffic from leaking to the LAN.

### Persistent state (`mowgli_maps` volume)

Persistent robot state lives in the Docker named volume `mowgli_maps`, mounted
at `/ros2_ws/maps` in `mowgli-ros2` (and in `mowgli-gui`). There is no SLAM map:
slam_toolbox was removed in favour of the `fusion_graph` localizer. What the
volume holds:

| File | Written by |
|---|---|
| `areas.dat` | `map_server_node` — mowing-area polygons, keepouts, dock point, datum stamp |
| `obstacles.yaml` | `obstacle_tracker_node` — promoted permanent obstacles |
| `fusion_graph.*` | `fusion_graph_node` — the factor graph; auto-saved on dock arrival and periodically while mowing, reloaded at startup |
| `imu_calibration.txt`, `mag_calibration.yaml` | IMU / magnetometer calibration |
| `coverage_resume.txt` | behavior tree — mid-area resume cursor |

The volume survives `docker compose down` and image updates. It is prefixed
with `COMPOSE_PROJECT_NAME` (`install_mowgli_maps` by default) — renaming that
`.env` key orphans all of the above.

---

## Deployment modes

### Standard — all-in-one on one board (the only supported mode)

The generated `docker-compose.yaml` runs everything on the board that has the
serial devices attached — Rockchip SBCs and Raspberry Pis where compute and
hardware interfaces sit on the same machine.

```bash
./stack.sh up
```

Split deployments were removed. Until 2026 the repo also carried a ser2net
mode (serial over TCP) and a remote split (Nav2 on a desktop, hardware on the
Pi); `docker-compose.ser2net.yaml`, `docker-compose.remote.pi.yaml` and
`docker-compose.remote.host.yaml` no longer exist, and Cyclone DDS is now
deliberately pinned to the loopback interface (see *DDS middleware* above), so
the ROS graph cannot span two machines. `docker/fastdds.xml` is kept as a reference profile
only — nothing mounts it, and FastRTPS is not supported.

### Foxglove Bridge as a separate container

By default, Foxglove Bridge runs inside `mowgli-ros2` on port 8765. To
run it as a restartable, standalone container instead:

```bash
# First, disable the built-in bridge to avoid a port conflict:
#   set ENABLE_FOXGLOVE=false in .env, then ./stack.sh regen
docker compose -f docker-compose.yaml -f docker-compose.foxglove.yaml up -d
```

Connect Foxglove Studio to `ws://<board-ip>:8765`.

---

## Updating images

Re-run the installer (handles pull, restart, and any new config keys):

```bash
cd ~/mowglinext/install && ./mowglinext.sh
```

Or, from a dev checkout, pull and re-create in one step:

```bash
./stack.sh update
```

The `gui` container is the only one labelled for Watchtower, which polls every
4 hours (`WATCHTOWER_POLL_INTERVAL=14400`) and restarts it automatically when a
new image is pushed. Watchtower is always composed by the full installer; from
a dev checkout `stack.sh` only includes it when `ENABLE_WATCHTOWER=true`.

To force an immediate Watchtower check:

```bash
docker exec mowgli-watchtower /watchtower --run-once
```

---

## Troubleshooting

### Diagnostics script

```bash
cd ~/mowglinext/install && ./mowglinext.sh --check
```

Checks: Docker, device nodes, the generated GPS YAML, container health,
firmware response, GPS fix quality, NTRIP connection, LiDAR `/scan` publisher,
rangefinders, and GUI + Foxglove accessibility. Prints a numbered list of
issues to fix.

### Containers do not start

```bash
docker compose logs mowgli --tail 50
docker compose logs gps --tail 30
docker compose logs lidar --tail 30
```

### Hardware devices not found

Verify that udev rules are in place and have been applied:

```bash
cat /etc/udev/rules.d/50-mowgli.rules
ls -l /dev/mowgli /dev/gps
```

If a device is missing after reconnecting USB, reload rules:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The LiDAR connects via UART, not USB. Confirm both the UART and the symlink
the LD19 container opens exist:

```bash
ls -l /dev/ttyS1 /dev/lidar   # UART, and the udev symlink pointing at it
```

### GPS has no fix or poor accuracy

Check the GPS container logs:

```bash
docker compose logs gps --tail 50
```

Look for `NTRIP enabled: true` and `Connected to http://...` to confirm
the NTRIP connection. If NTRIP fails, verify `ntrip_host`,
`ntrip_mountpoint`, and network connectivity from the board.

RTK convergence after connecting to NTRIP can take 1–5 minutes for
FLOAT and 5–15 minutes for FIXED, depending on sky view and caster
distance.

Check the live fix quality from inside the container:

```bash
docker exec mowgli-gps bash -c "
  source /opt/ros/kilted/setup.bash
  ros2 topic echo /gps/fix --once"
```

`status.status` values: `-1` = no fix, `0` = standard fix,
`1` = RTK float, `2` = RTK fixed.

### GPS datum not set

If `datum_lat` and `datum_lon` are `0.0`, the robot position will be
wrong. Set them to your dock's GPS coordinates:

```yaml
# config/mowgli/mowgli_robot.yaml
mowgli:
  ros__parameters:
    datum_lat: 48.879640599999995
    datum_lon: 2.1728332999999997
```

Then restart:

```bash
docker compose restart mowgli
```

### DDS discovery failures — nodes not seeing each other

Symptoms: `ros2 topic list` inside a container shows fewer topics than
expected; Nav2 does not receive LiDAR scans.

1. Confirm all containers share the same `ROS_DOMAIN_ID`:
   ```bash
   grep ROS_DOMAIN_ID .env
   docker exec mowgli-ros2 env | grep ROS_DOMAIN_ID
   docker exec mowgli-lidar env | grep ROS_DOMAIN_ID
   ```

2. Confirm `cyclonedds.xml` is mounted in every container:
   ```bash
   docker exec mowgli-ros2 cat /cyclonedds.xml
   docker exec mowgli-lidar cat /cyclonedds.xml
   ```

3. The `MaxAutoParticipantIndex` in `config/cyclonedds.xml` defaults to
   `500`. If you add more nodes and discovery still fails, increase it.

4. Confirm DDS is still pinned to `lo` — `NetworkInterface name="lo"` plus
   `AllowMulticast=false`. Letting Cyclone autodetermine the interface makes it
   route DDS over WiFi/Ethernet, where a link flap turns into
   `ddsi_udp_conn_write ... failed with retcode -1` floods and dying nodes
   (issue #418).

5. Run `ros2` CLI commands **inside** a container (`docker exec mowgli-ros2 …`).
   A host-native `ros2` binary will not see the graph unless it uses the same
   config: `export CYCLONEDDS_URI=file://$PWD/config/cyclonedds.xml`.

There is no multi-host deployment: every container runs on the one board, and
DDS is pinned to loopback on purpose, so nothing about DDS needs to be routable
between machines.

### Nav2 does not start or times out on ARM

On resource-constrained ARM boards, Nav2 nodes can take 30–60 seconds to
initialise because all nodes start in parallel. If the lifecycle manager
times out waiting for a node:

1. Check which node timed out:
   ```bash
   docker compose logs mowgli | grep -i "timed out\|lifecycle\|error"
   ```

2. Check whether `fusion_graph_node` is reloading a large saved graph — it
   loads `/ros2_ws/maps/fusion_graph.*` at startup if present:
   ```bash
   docker exec mowgli-ros2 ls -lh /ros2_ws/maps/
   ```
   Deleting the saved graph makes the node start from a fresh one (the map
   frame is re-anchored from GPS, so no mowing areas are lost).

3. A full restart often resolves transient timing failures:
   ```bash
   docker compose restart mowgli
   ```

### LiDAR scans not reaching the stack

Confirm the LiDAR is publishing:

```bash
docker exec mowgli-ros2 bash -c "
  source /opt/ros/kilted/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 topic info /scan"
```

`Publisher count` must be `1`. If it is `0`, check the `lidar` container logs
and confirm `/dev/lidar` resolves to the board's LiDAR UART (the LD19 image
opens that path at 230400 baud regardless of `.env`).

Also confirm the TF chain from `base_link` to `lidar_link` is complete:

```bash
docker exec mowgli-ros2 bash -c "
  source /opt/ros/kilted/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 run tf2_tools view_frames"
```

### Mowing areas or the factor graph are not persisted

Verify the `mowgli_maps` volume is mounted:

```bash
docker inspect mowgli-ros2 | grep -A3 mowgli_maps
```

`areas.dat` is written by `map_server_node` whenever areas change. The factor
graph auto-saves on dock arrival and periodically while mowing; to checkpoint
it on demand call the service directly (the GUI exposes it under
Diagnostics → Fusion Graph):

```bash
docker exec mowgli-ros2 bash -c "
  source /opt/ros/kilted/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 service call /fusion_graph_node/save_graph std_srvs/srv/Trigger"
```

### ROS nodes crashed inside `mowgli-ros2`

```bash
docker compose logs mowgli | grep "process has died"
```

Each crashed node is identified in the log. Common causes: missing device
(`/dev/mowgli` not found), a YAML syntax error in
`config/mowgli/mowgli_robot.yaml`, or out-of-memory on a board with less than
4 GB RAM.

### Mowgli firmware not responding

The hardware bridge publishes `/hardware_bridge/status`. If nothing
arrives:

```bash
docker exec mowgli-ros2 bash -c "
  source /opt/ros/kilted/setup.bash
  source /ros2_ws/install/setup.bash
  timeout 5 ros2 topic echo /hardware_bridge/status --once"
```

If the topic is empty, the STM32 board is not communicating. Check:

- `/dev/mowgli` exists on the host and is passed to the container
- The Mowgli firmware is flashed — source at
  <https://github.com/cedbossneo/Mowgli>
- `mower_status` field: value `255` means the board is connected but not
  initialised (try pressing the mower power button)

---

## Access points

| Service | URL |
|---|---|
| MowgliNext GUI | `http://<board-ip>:4006` |
| Foxglove Bridge | `ws://<board-ip>:8765` |
| MQTT broker | `<board-ip>:1883` |
| MQTT over WebSocket | `<board-ip>:9001` |

---

## Useful commands

```bash
# View all container logs live
docker compose logs -f

# Restart a single container after config change
docker compose restart mowgli

# Open a shell inside the ROS2 container
docker exec -it mowgli-ros2 bash

# List all active ROS2 nodes
docker exec mowgli-ros2 bash -c "
  source /opt/ros/kilted/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 node list"

# List all active topics
docker exec mowgli-ros2 bash -c "
  source /opt/ros/kilted/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 topic list"

# Stop the stack (areas.dat and the factor graph survive in mowgli_maps)
docker compose down

# Stop the stack and delete all volumes — this ERASES areas.dat, the factor
# graph and the IMU/mag calibration
docker compose down -v
```

---

## Contributing

1. Fork the repository on GitHub.
2. Create a branch from `dev` (`feat/`, `fix/`, `refactor/`, `chore/` or
   `perf/` — the CI push triggers match exactly these prefixes).
3. Make your changes. Test on real hardware where possible.
4. Open a pull request against `dev`. `main` is the release branch; both are
   protected and require a review.

Do not hand-edit `docker-compose.yaml`, `.env` or `config/mowgli/*` — they are
generated or runtime state. Change the source instead: `install/compose/*.yml`
(services), `install/lib/env.sh` (`.env` keys), `install/config/` (seeds), then
`./stack.sh regen`.

The sensor images have one workflow each — `.github/workflows/sensors-gps.yml`
and `sensors-lidar-{ldlidar,rplidar,stl27l}.yml` — all calling the reusable
`_sensor-docker.yml`, which builds `linux/amd64` and `linux/arm64` on native
runners and publishes multi-arch manifests to GHCR. `ros2-docker.yml` builds
`mowgli-ros2` and `gui-docker.yml` builds the GUI. Note that **no CI workflow
watches this directory** — the only checks on the generated compose are
`install/tests/test_compose_validity.sh` and `install/tests/test_env_output.sh`,
which you must run by hand.

Working on this directory with Claude? Start from
[`CLAUDE.md`](CLAUDE.md) here, then
[`../docs/claude/codemaps/deploy.md`](../docs/claude/codemaps/deploy.md)
(the `.env` → fragment → container map),
[`../docs/claude/parameters.md`](../docs/claude/parameters.md) and
[`../docs/claude/doc-index.md`](../docs/claude/doc-index.md).

---

## License

MowgliNext is dual-licensed: **GPLv3** for open-source, personal, educational,
non-profit and community use, and a separate commercial licence for any
commercial use. See [`../LICENSE`](../LICENSE). The upstream Mowgli firmware
project lives at <https://github.com/cedbossneo/Mowgli>.
