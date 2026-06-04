# Mowgli Docker — v3 (ROS2 Kilted)

Docker Compose deployment for the **Mowgli** open-source robot mower.
v3 is a ground-up rewrite: the ROS1 Noetic stack has been replaced by a
single `mowgli_ros2` container running **ROS2 Kilted**, Nav2, SLAM Toolbox,
and a full behavior-tree coverage planner.

## What changed from v2

| v2 (ROS1 Noetic) | v3 (ROS2 Kilted) |
|---|---|
| `roscore` | Removed — DDS has no master |
| `rosserial` | Removed — hardware bridge is inside `mowgli_ros2` |
| `openmower` | Replaced by `mowgli_ros2` (Nav2 + BT + coverage) |
| Foxglove Bridge as separate image | Built into `mowgli_ros2`, enabled via launch arg |
| Rosbridge as separate container | Built into `mowgli_ros2`, enabled via launch arg |
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
- `SA650`
- `900ECO`
- `LUV1000RI`
- `Sabo` (Sabo Mestercut)

### Sensors and serial devices

| Device | Preferred stable path | Compatibility symlink | USB IDs (for udev) |
|---|---|---|---|
| Mowgli STM32 board | `/dev/mowgli` | USB-CDC | `product=="Mowgli"` |
| u-blox ZED-F9P (simpleRTK2B) | `/dev/serial/by-id/...` | `/dev/gps` | VID `1546` PID `01a9` |
| u-blox RTK1010Board (ESP USB-CDC) | `/dev/serial/by-id/...` | `/dev/gps` | VID `303a` PID `4001` |
| LDRobot LD19 LiDAR | `/dev/ttyS1` (hardware UART) | UART | — |

The LiDAR connects to a hardware UART on the compute board, not USB. Set
`LIDAR_PORT` in `.env` if your board exposes it differently.

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
git clone --depth 1 https://github.com/cedbossneo/mowglinext.git
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
```

Reload and verify:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/mowgli /dev/gps
```

#### 4. Create `.env`

```bash
cp .env.example .env
```

Edit `.env` — see the [Configuration reference](#env--image-tags-and-ports)
section below for all variables.

#### 5. Edit `config/mowgli/mowgli_robot.yaml`

At minimum, set your GPS datum (dock coordinates), NTRIP parameters, and
dock pose. See [mowgli_robot.yaml reference](#mowgli_robotyaml-key-parameters).

#### 6. Pull images and start

```bash
docker compose pull
docker compose up -d
```

#### 7. Open the GUI

```
http://<board-ip>
```

---

## Configuration reference

### `.env` — image tags and ports

Copy `.env.example` to `.env` and edit. All keys and their defaults:

| Variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `0` | DDS domain ID — must be the same across all containers and any remote machines |
| `MOWER_IP` | `10.0.0.161` | IP of the mower Pi — only used in ser2net mode |
| `LIDAR_PORT` | `/dev/ttyS1` | Host device path for the LD19 UART |
| `LIDAR_BAUD` | `230400` | LD19 baud rate |
| `MOWGLI_ROS2_IMAGE` | `ghcr.io/cedbossneo/mowgli-ros2/mowgli-ros2:main` | Full ROS2 stack |
| `GPS_IMAGE` | `ghcr.io/cedbossneo/mowgli-docker/gps:v3` | u-blox + NTRIP driver |
| `LIDAR_IMAGE` | `ghcr.io/cedbossneo/mowgli-docker/lidar:v3` | LD19 LiDAR driver |
| `GUI_IMAGE` | `ghcr.io/cedbossneo/openmower-gui:v3` | OpenMower GUI |

For Universal GNSS, set `GNSS_SERIAL_DEVICE=/dev/serial/by-id/...` and keep
`GPS_PORT` aligned with the same by-id path when you need the legacy
compatibility keys. Use raw `ttyACM*` or `ttyUSB*` paths only as a temporary
diagnostic fallback when `/dev/serial/by-id` is unavailable.

### `mowgli_robot.yaml` key parameters

`config/mowgli/mowgli_robot.yaml` is bind-mounted read-only into the
`mowgli` and `gps` containers. The GUI writes this file directly; restart
the affected containers to apply manual edits.

**GPS and datum**

| Parameter | Example | Description |
|---|---|---|
| `datum_lat` | `48.879640599999995` | Map origin latitude — set to your dock's GPS coordinates |
| `datum_lon` | `2.1728332999999997` | Map origin longitude |
| `gps_port` | `/dev/gps` | GPS device path inside the container (mapped from host by udev) |
| `gps_baudrate` | `921600` | Serial baud rate for the F9P |
| `gps_protocol` | `UBX` | Protocol — leave as `UBX` for u-blox receivers |
| `gps_timeout_sec` | `5` | Seconds to wait for a fix before raising a warning |
| `gps_antenna_x` | `0.3` | Antenna offset from `base_link` centre, metres (forward) |
| `gps_antenna_y` | `0` | Antenna offset, metres (left) |
| `gps_antenna_z` | `0.2` | Antenna height above base plane, metres |

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
| `dock_approach_distance` | `1` | Stop distance before final dock approach (metres) |
| `undock_distance` | `1.5` | How far to reverse before turning (metres) |
| `undock_speed` | `0.15` | Reverse speed during undocking (m/s) |
| `dock_use_charger_detection` | `true` | Stop when charging current detected |

**Robot geometry**

| Parameter | Example | Description |
|---|---|---|
| `mower_model` | `YardForce500` | Hardware model — determines URDF and firmware expectations |
| `wheel_radius` | `0.04475` | Wheel radius in metres |
| `wheel_track` | `0.325` | Lateral distance between wheel centres (metres) |
| `ticks_per_revolution` | `1050` | Encoder ticks per full wheel revolution |
| `chassis_center_x` | `0.18` | Longitudinal offset from axle to chassis centre (metres) |
| `blade_radius` | `0.09` | Cutting disc radius (metres) |
| `tool_width` | `0.18` | Effective cut width used for coverage path spacing (metres) |

**LiDAR mounting**

| Parameter | Example | Description |
|---|---|---|
| `lidar_enabled` | `true` | Enable LiDAR-based obstacle detection and SLAM |
| `lidar_x` | `0` | LiDAR position, forward from `base_link` (metres) |
| `lidar_y` | `0.026` | LiDAR position, left from `base_link` (metres) |
| `lidar_z` | `0.22` | LiDAR height above base plane (metres) |
| `lidar_yaw` | `-3.1416` | LiDAR rotation in radians (`-3.1416` = 180° = scanner facing rear) |

**Mowing behaviour**

| Parameter | Example | Description |
|---|---|---|
| `mowing_speed` | `0.3` | Mowing speed in m/s |
| `transit_speed` | `0.4` | Transit-to-area speed in m/s |
| `path_spacing` | `0.13` | Distance between parallel mowing passes (metres) |
| `headland_width` | `0.18` | Width of the headland strip around the boundary |
| `outline_passes` | `1` | Number of perimeter passes before filling |
| `outline_offset` | `0.05` | Inset from the mapped boundary (metres) |
| `min_turning_radius` | `0.3` | Minimum turning radius for path planning (metres) |
| `mow_angle_increment_deg` | `0` | Rotate mowing angle by this many degrees each session |

**SLAM**

| Parameter | Example | Description |
|---|---|---|
| `slam_mode` | `lifelong` | SLAM Toolbox mode: `mapping` (build new map) or `lifelong` (update existing) |
| `map_save_path` | `/ros2_ws/maps/garden_map` | Path inside container where the map is saved on dock |
| `map_save_on_dock` | `true` | Automatically serialise the SLAM map when the mower docks |

**Battery**

| Parameter | Example | Description |
|---|---|---|
| `battery_full_voltage` | `28.5` | Voltage threshold for "fully charged" |
| `battery_empty_voltage` | `24` | Voltage threshold for "return to dock" |
| `battery_critical_voltage` | `23` | Voltage threshold for emergency stop |

### Advanced parameter overrides

The `mowgli` container also reads optional YAML files from
`config/mowgli/` at startup. Drop any of these files into that directory
to override the built-in defaults without rebuilding the image:

| File | What it tunes |
|---|---|
| `nav2_params.yaml` | Nav2 costmap, planner, controller, collision monitor |
| `localization.yaml` | Dual EKF — GPS/odometry fusion weights and process noise |
| `slam_toolbox.yaml` | SLAM loop closure, scan matching, map resolution |
| `hardware_bridge.yaml` | Serial port, baud rate, heartbeat and publish rates |
| `coverage_planner.yaml` | B-RV coverage planner (tool width, headland, Voronoi transit, sweep direction) |
| `behavior_tree.yaml` | Behavior tree tick rate, battery thresholds |
| `mqtt_bridge.yaml` | MQTT broker connection for Home Assistant |

To extract a built-in default for editing:

```bash
# List all built-in configs
docker exec mowgli-ros2 ls /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/

# Copy one out for local editing
docker exec mowgli-ros2 cat \
  /ros2_ws/install/mowgli_bringup/share/mowgli_bringup/config/nav2_params.yaml \
  > config/mowgli/nav2_params.yaml
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
│  │  ├─ Nav2 stack (planner, controller, BT)       │     │
│  │  ├─ SLAM Toolbox  ←  /scan (from mowgli-lidar) │     │
│  │  ├─ dual EKF  ←  /gps/fix + /wheel_odom        │     │
│  │  ├─ rosbridge_server  :9090                    │     │
│  │  └─ foxglove_bridge   :8765                    │     │
│  └────────────────────────────────────────────────┘     │
│                                                         │
│  ┌──────────────────┐   ┌──────────────────────────┐   │
│  │  mowgli-gps      │   │  mowgli-lidar            │   │
│  │  ublox_gps_node  │   │  ldlidar_stl_ros2_node   │   │
│  │  ntrip_client    │   │  publishes /scan          │   │
│  │  rtcm_serial_    │   │  frame_id: lidar_link     │   │
│  │    bridge        │   │  port: /dev/ttyS1         │   │
│  │  pub: /gps/fix   │   │  baud: 230400             │   │
│  └──────────────────┘   └──────────────────────────┘   │
│                                                         │
│  ┌──────────────────┐   ┌──────────────────────────┐   │
│  │  openmower-gui   │   │  mowgli-mqtt             │   │
│  │  Go + React      │   │  eclipse-mosquitto        │   │
│  │  ws://…:9090     │   │  :1883 (MQTT)             │   │
│  │  port: 80        │   │  :9001 (MQTT-WS)          │   │
│  └──────────────────┘   └──────────────────────────┘   │
│                                                         │
│  ┌──────────────────────────────────────────────────┐   │
│  │  mowgli-watchtower  (polls gui label every 4h)   │   │
│  └──────────────────────────────────────────────────┘   │
│                                                         │
│  Volume: mowgli_maps  →  /ros2_ws/maps (in mowgli)     │
└─────────────────────────────────────────────────────────┘
```

### Service summary

| Container | Image | Purpose | Exposed ports |
|---|---|---|---|
| `mowgli-ros2` | `ghcr.io/cedbossneo/mowgli-ros2/mowgli-ros2:main` | Full ROS2 stack: hardware bridge, Nav2, SLAM, rosbridge, Foxglove | 9090 (rosbridge), 8765 (Foxglove) |
| `mowgli-gps` | `ghcr.io/cedbossneo/mowgli-docker/gps:v3` | u-blox ZED-F9P driver + NTRIP RTK corrections | — |
| `mowgli-lidar` | `ghcr.io/cedbossneo/mowgli-docker/lidar:v3` | LDRobot LD19 driver, publishes `/scan` | — |
| `openmower-gui` | `ghcr.io/cedbossneo/openmower-gui:v3` | Web UI — area mapping, mowing control, config editor | 80 (host networking) |
| `mowgli-mqtt` | `eclipse-mosquitto:latest` | MQTT broker for Home Assistant and telemetry | 1883, 9001 |
| `mowgli-watchtower` | `ghcr.io/nicholas-fedor/watchtower:latest` | Auto-updates containers labelled `com.centurylinklabs.watchtower.enable: "true"` | — |

### DDS middleware

All ROS2 containers use **Cyclone DDS** (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).
The shared config at `config/cyclonedds.xml` is bind-mounted to
`/cyclonedds.xml` in every container and sets:

```xml
<Discovery>
  <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
</Discovery>
```

This raises the default participant ceiling to handle the 35+ DDS
participants that the full stack starts simultaneously.

All containers run with `network_mode: host` and `ipc: host` so DDS
discovery works over the loopback interface without multicast routing.
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` restricts discovery to the local
machine, preventing DDS traffic from leaking to the LAN.

### SLAM maps

Maps are persisted in the Docker named volume `mowgli_maps`, mounted at
`/ros2_ws/maps` inside the `mowgli-ros2` container. The map is saved
automatically when the mower docks (controlled by `map_save_on_dock` in
`mowgli_robot.yaml`). The volume survives `docker compose down` and image
updates.

---

## Deployment modes

### Standard — all-in-one on one board (default)

The default `docker-compose.yaml` runs everything on the board that has
the USB serial devices attached. Use this for Rockchip SBCs where the
compute and hardware interfaces are on the same machine.

```bash
docker compose up -d
```

### Ser2net — serial over TCP

Use when the compute board (running Docker) is physically separate from
the mower board, connected via Ethernet. The mower Pi must run `ser2net`
to expose serial ports as TCP sockets:

```yaml
# /etc/ser2net.yaml on the mower Pi
connection: &mowgli
  accepter: tcp,4001
  connector: serialdev,/dev/mowgli,115200n81
connection: &gps
  accepter: tcp,4002
  connector: serialdev,/dev/gps,460800n81
connection: &lidar
  accepter: tcp,4003
  connector: serialdev,/dev/lidar,230400n81
```

On the brain machine, set `MOWER_IP` in `.env`, then:

```bash
docker compose -f docker-compose.ser2net.yaml up -d
```

Three `socat` relay containers create virtual PTYs (`/dev/mowgli`,
`/dev/gps`, `/dev/lidar`) that the `mowgli` container reads as normal
serial ports.

### Remote split — Nav2 on a powerful host, hardware on a Pi

Run only the hardware bridge on the mower Pi, and the full Nav2 / GUI
stack on a desktop or server.

On the mower Pi (serial access required):

```bash
docker compose -f docker-compose.remote.pi.yaml up -d
```

On the remote host:

```bash
docker compose -f docker-compose.remote.host.yaml up -d
```

Both machines must have the same `ROS_DOMAIN_ID` in `.env`. DDS
multicast must be routable between them (same L2 segment). If the
machines are on different subnets, configure FastDDS unicast peer
discovery (see `fastdds.xml` in the repository root for a reference
profile).

### Foxglove Bridge as a separate container

By default, Foxglove Bridge runs inside `mowgli-ros2` on port 8765. To
run it as a restartable, standalone container instead:

```bash
# First, disable the built-in bridge to avoid port conflicts.
# Set enable_foxglove:=false in the mowgli command in docker-compose.yaml,
# then:
docker compose -f docker-compose.yaml -f docker-compose.foxglove.yaml up -d
```

Connect Foxglove Studio to `ws://<board-ip>:8765`.

---

## Updating images

Re-run the installer (handles pull, restart, and any new config keys):

```bash
cd ~/mowglinext/install && ./mowglinext.sh
```

Or manually:

```bash
docker compose pull
docker compose up -d
```

The `gui` container is also watched by Watchtower, which polls every
4 hours (`WATCHTOWER_POLL_INTERVAL=14400`) and restarts the container
automatically when a new image is pushed.

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

Checks: Docker, device nodes, container health, firmware response,
GPS fix quality, NTRIP connection, LiDAR `/scan` publisher, SLAM state,
and GUI accessibility. Prints a numbered list of issues to fix.

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

The LiDAR connects via UART, not USB. Confirm the port exists:

```bash
ls -l /dev/ttyS1     # or whatever LIDAR_PORT is set to in .env
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
   `120`. If you add more nodes and discovery still fails, increase it.

4. For the remote split deployment, confirm DDS multicast is routable
   between the Pi and the host, or switch to a FastDDS unicast peer
   configuration using `fastdds.xml` as a reference.

### Nav2 does not start or times out on ARM

On resource-constrained ARM boards, Nav2 nodes can take 30–60 seconds to
initialise because all nodes start in parallel. If the lifecycle manager
times out waiting for a node:

1. Check which node timed out:
   ```bash
   docker compose logs mowgli | grep -i "timed out\|lifecycle\|error"
   ```

2. The most common cause is SLAM Toolbox taking longer than Nav2's
   lifecycle timeout allows. If maps are not loading, confirm
   `slam_mode: lifelong` is set and a `.posegraph` file exists:
   ```bash
   docker exec mowgli-ros2 ls /ros2_ws/maps/
   ```
   If there is no saved map, change `slam_mode` to `mapping` for the
   first run, then switch back to `lifelong` after a map has been saved.

3. A full restart often resolves transient timing failures:
   ```bash
   docker compose restart mowgli
   ```

### SLAM not building a map

Confirm the LiDAR is publishing:

```bash
docker exec mowgli-ros2 bash -c "
  source /opt/ros/kilted/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 topic info /scan"
```

`Publisher count` must be `1`. If it is `0`, check the `lidar` container
logs and verify the UART port and baud rate in `.env`.

Also confirm the TF chain from `base_link` to `lidar_link` is complete:

```bash
docker exec mowgli-ros2 bash -c "
  source /opt/ros/kilted/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 run tf2_tools view_frames"
```

### SLAM map not saved after docking

Ensure `map_save_on_dock: true` and verify the `mowgli_maps` volume is
mounted:

```bash
docker inspect mowgli-ros2 | grep -A3 mowgli_maps
```

To save the map manually without docking:

```bash
docker exec mowgli-ros2 bash -c "
  source /opt/ros/kilted/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 service call /slam_toolbox/save_map \
    slam_toolbox/srv/SaveMap \
    '{name: {data: /ros2_ws/maps/garden_map}}'"
```

### ROS nodes crashed inside `mowgli-ros2`

```bash
docker compose logs mowgli | grep "process has died"
```

Each crashed node is identified in the log. Common causes: missing device
(`/dev/mowgli` not found), YAML syntax error in a config override, or
out-of-memory on a board with less than 4 GB RAM.

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
| OpenMower GUI | `http://<board-ip>` |
| Rosbridge WebSocket | `ws://<board-ip>:9090` |
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

# Stop the stack (maps are preserved in the mowgli_maps volume)
docker compose down

# Stop the stack and delete all volumes including the map
docker compose down -v
```

---

## Contributing

1. Fork the repository on GitHub.
2. Create a branch from `v3`.
3. Make your changes. Test on real hardware where possible.
4. Open a pull request against the `v3` branch.

The CI workflow (`.github/workflows/docker.yml`) builds the `gps` and
`lidar` images for both `linux/amd64` and `linux/arm64` on native runners
and publishes multi-arch manifests to GHCR on every push to `v3`.

---

## License

This repository does not currently include a `LICENSE` file. The upstream
Mowgli project is open source — refer to
<https://github.com/cedbossneo/Mowgli> for licensing information.
