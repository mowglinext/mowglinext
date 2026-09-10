# Getting Started

> **Looking for the short version?** See [`docs/FIRST_BOOT.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/FIRST_BOOT.md) in the main repo for the docked-only post-install checklist (GUI open → RTK Fixed → IMU calibration → dock pose → record area → first autonomous mow).

## Hardware

### Compute Board

Any 64-bit Linux board with Docker support (the installer supports `arm64` and `amd64`):
- **Recommended:** Rockchip RK3566, RK3588
- **Also works:** Raspberry Pi 4, Pi 5
- **Minimum:** 4-core ARM64, 4 GB RAM, 16 GB storage

### Mower Chassis

| Model | Status |
|-------|--------|
| YardForce Classic 500 | Primary target |
| YardForce 500B | Supported |
| YardForce SA650 | Supported |
| YardForce 900 ECO | Supported |
| YardForce LUV1000RI | Supported |
| Sabo MOWiT 500F | Supported |
| Custom Robot | Every chassis parameter configured by hand |

The model is picked in the GUI (onboarding wizard, or Settings → Hardware) and stored as `mower_model` in `mowgli_robot.yaml`; each preset seeds the chassis dimensions, wheel geometry, `tool_width`, encoder ticks and battery thresholds.

### Sensors

| Sensor | Model | Connection |
|--------|-------|------------|
| RTK GPS | u-blox ZED-F9P (simpleRTK2B), or a Unicore UM980/UM981/UM982-class receiver | USB-CDC or UART |
| LiDAR | LDRobot LD06 / LD14 / LD19 (also STL27L and RPLIDAR) | UART 230400 or USB |
| IMU | WitMotion WT901 (accel + gyro + magnetometer) | I2C (on STM32) |

The GPS and LiDAR are chosen in the [install composer](https://mowgli.garden/#getting-started) or interactively by the installer. The firmware auto-detects its IMU at boot — LSM6, WT901, MPU6050 or ICM-45686, plus a LIS3MDL magnetometer when the accel/gyro chip has none of its own.

### Firmware Board

Custom Mowgli STM32F103 PCB — handles motor control, IMU, blade safety, battery monitoring.

## Development with GitHub Codespaces / DevContainer

The fastest way to explore and develop MowgliNext — no local setup required:

### GitHub Codespaces (cloud)

1. Go to [github.com/mowglinext/mowglinext](https://github.com/mowglinext/mowglinext)
2. Click **Code → Codespaces → Create codespace on main**
3. Select a **8-core** machine type (16-core recommended for simulation)
4. Wait for the container to build (~10 min first time, cached after)

### VS Code DevContainer (local)

1. Install [Docker](https://docs.docker.com/get-docker/) and the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
2. Clone the repo: `git clone https://github.com/mowglinext/mowglinext.git`
3. Open in VS Code → **Reopen in Container** when prompted

### What's included

The devcontainer provides a complete ROS2 Kilted development environment:

- Full Nav2 navigation stack plus GTSAM 4.3a1, for the `fusion_graph` factor-graph localizer — the stack's sole localizer, with optional LiDAR scan-matching and loop-closure factors
- Nav2 controller plugins as the stack uses them: `mowgli_nav2_plugins/FTCController` follows the coverage path, RotationShim + Regulated Pure Pursuit handle transit
- Foxglove Bridge + rosbridge for visualization
- GUI stack (Go 1.24, Node 22, yarn) so `cd gui && go build` and `cd gui/web && yarn dev` work out of the box
- Python linting (ruff, pre-commit), C++ linting (cppcheck, clang-format), gdb, htop
- Claude Code CLI + GitHub CLI for AI-assisted development
- Auto-sourced ROS2 workspace

Two things the devcontainer deliberately does **not** carry: the Webots simulator (see [Run the simulation](#run-the-simulation) below), and Fields2Cover v3. The image builds Fields2Cover 2.0.0 into `/opt/fields2cover-200`, while `mowgli_coverage` pins Fields2Cover 3.0.0 at `/opt/fields2cover-300` — which only the full `ros2/Dockerfile` build provides. Build the focused development set in the devcontainer rather than the whole workspace.

**Forwarded ports:**

| Port | Service |
|------|---------|
| 4006 | Mowgli GUI |
| 8765 | Foxglove WebSocket |
| 6080 | noVNC (simulation GUI) |
| 9090 | rosbridge (legacy GUI path) |

### Build in the devcontainer

```bash
# Build the focused development set (mowgli_interfaces, mowgli_localization,
# universal_gnss_ros2, mowgli_bringup) into /ros2_ws
cd ros2 && make build-dev

# Source the workspace
source /ros2_ws/install/setup.bash
```

`make build-full` builds every linked package, but needs the Fields2Cover v3 tree noted above. `make help` lists the rest of the targets.

### Run the simulation

The simulation runs on Webots (`mowgli_bringup/sim_full_system.launch.py`), and Webots itself is installed by the `simulation` stage of `ros2/Dockerfile` — not by the devcontainer image. Run it from the simulation compose file instead:

```bash
docker compose -f docker/docker-compose.simulation.yaml build
docker compose -f docker/docker-compose.simulation.yaml up dev-sim
```

Foxglove Studio then connects to `ws://localhost:8765`; the noVNC view of the simulator is on `http://localhost:6080` (`simulation-gui` service). See [Simulation](Simulation) for the full workflow.

## Quick Start on Hardware (Automated)

The easiest way to deploy on real hardware is the install composer at [mowgli.garden](https://mowgli.garden/#getting-started). Pick your hardware (GPS, LiDAR, rangefinders) and copy the generated command. Or run the installer directly:

```bash
curl -sSL https://mowgli.garden/install.sh | bash
```

The installer handles:
- Docker installation (if needed)
- udev rules for serial devices (`/dev/mowgli`, `/dev/gps`, `/dev/lidar`)
- Sensor configuration (pre-filled if you used the web composer, interactive otherwise)
- Mower configuration (GPS datum, dock position, NTRIP credentials)
- Pulling and launching all containers
- Post-install diagnostics

Run diagnostics on an existing installation:

```bash
cd ~/mowglinext/install && ./mowglinext.sh --check
```

## Manual Install

If you prefer to set things up manually:

### 1. Clone

```bash
git clone https://github.com/mowglinext/mowglinext.git
cd mowglinext
```

### 2. Set Up udev Rules

Create stable device symlinks:

```bash
# /etc/udev/rules.d/50-mowgli.rules
# Mowgli STM32 board — matched by product string, because its 0483:5740 VID:PID
# is the generic STM32 VCP pair and would collide with other adapters.
SUBSYSTEM=="tty", ATTRS{product}=="Mowgli", SYMLINK+="mowgli", MODE="0666"
# GPS: simpleRTK2B (u-blox ZED-F9P)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps", MODE="0666"
```

Reload: `sudo udevadm control --reload-rules && sudo udevadm trigger`

### 3. Configure

`docker/config/mowgli/mowgli_robot.yaml` is git-ignored — the installer normally seeds it from the shipped template, so on a manual install copy it yourself first:

```bash
cd docker
cp .env.example .env
cp ../install/config/mowgli/mowgli_robot.yaml config/mowgli/
nano config/mowgli/mowgli_robot.yaml
```

That file is **sparse**: it holds only your install-time choices and calibration results. Every other parameter falls back to the in-package template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml`, so do not paste defaults into it — delete a line to go back to the default.

Key settings to change:
- `datum_lat` / `datum_lon` — your GPS reference point
- `dock_pose_x` / `dock_pose_y` / `dock_pose_yaw` — dock position. `dock_pose_yaw`
  is a **map-frame ENU heading in radians** (not a phone-compass bearing); it is
  normally captured automatically by "Set Docking Point" / undock calibration
  rather than hand-entered.
- `ntrip_host` / `ntrip_user` / `ntrip_password` — RTK correction source

### 4. Launch

`docker/docker-compose.yaml` does not exist in a fresh clone — it is assembled from the fragments in `install/compose/` according to your `.env`. Use the stack manager, which regenerates it and then brings the stack up:

```bash
./stack.sh up
```

`./stack.sh down`, `restart`, `pull`, `update`, `logs -f <service>` and `ps` cover the rest; anything else is passed straight through to `docker compose`.

### 5. Access

| Service | URL |
|---------|-----|
| GUI | `http://<mower-ip>:4006` |
| Foxglove | `ws://<mower-ip>:8765` |

Foxglove Bridge is the only ROS websocket the stack starts (`enable_foxglove:=true` on the `mowgli` container); `rosbridge_server` was replaced by it and is no longer launched on the robot.

## GUI Features

### Diagnostics Page

Access the diagnostics dashboard at `http://<mower-ip>:4006/#/diagnostics`.

**Displays:**
- **System:** state, status and uptime of each Docker service (`mowgli`, `gps`, `lidar`, `gui`, …), CPU temperature, and an optional live firmware debug log stream
- **Localisation:** the fused map-frame pose published by `fusion_graph_node` (`/odometry/filtered_map`) — x/y/z, roll/pitch/yaw — next to the live GNSS diagnostics card
- **Fusion Graph:** nodes in the graph, loop closures, scan-match success rate, pose sigma, keyframes and rejects, straight from `/fusion_graph/diagnostics`
- **Heading sources:** the yaw estimates the localizer fuses, side by side
- **BT State & Coverage:** current behavior-tree state and sub-state, battery / charging / emergency latch, Nav2 recovery activity, and mow progress (current area, sub-paths done and skipped, coverage %)
- **Configuration Cross-checks:** see below
- **Calibration status:** the dock, IMU-yaw and magnetometer calibration artefacts, each with a button that runs the calibration
- **Sensors:** raw IMU rates and accelerations, wheel odometry, per-wheel encoder RPM/ticks/direction
- **Rosbag Recording:** start/stop a recording and manage the resulting bags
- **ROS Diagnostics:** aggregated health status (OK, WARN, ERROR, STALE) for all subsystems

Mowing areas, mow progress and the docking point are edited on the Map page (`http://<mower-ip>:4006/#/map`), not here.

**Configuration Cross-Checks:**
Reads the runtime `mowgli_robot.yaml` and shows the configured dock pose (x, y, yaw) alongside the GPS datum, warning when:
- the config file is missing or cannot be parsed
- the GPS datum is still unset (`lat=0, lon=0`)
- the dock heading is still unset (`yaw=0`)

### Statistics Page

Access session statistics at `http://<mower-ip>:4006/#/statistics`.

**Automatic Session Recording:**
Each mowing session is automatically logged with:
- Start time, end time, duration
- Which area was mowed, and the coverage reached (%)
- Sub-paths completed and skipped
- Distance travelled
- Outcome (completed / aborted / error), how many recharge pauses interrupted it, and any errors

**Session History:**
- A table of past sessions — date, duration, area, coverage, status
- Distance mowed per week over the last 12 weeks
- Per-area coverage (cells mowed out of cells total)
- Clear the whole history from the same page

**Aggregate Statistics:**
- Total distance since install
- Total hours active, across all recorded sessions
- Completion rate (%) and number of runs
- Average coverage per run

## Next Steps

- [Simulation](Simulation) — test without hardware, run E2E tests
- [Configuration](Configuration) — tune parameters for your environment
- [Deployment](Deployment) — advanced Docker deployment options
- [Contributing](Contributing) — contribute to MowgliNext

Working on the code (or pointing an AI agent at it)? Start from [`CLAUDE.md`](https://github.com/mowglinext/mowglinext/blob/main/CLAUDE.md) at the repo root, then the indexes under [`docs/claude/`](https://github.com/mowglinext/mowglinext/tree/main/docs/claude): `codemaps/` (where the code for each area lives), `ros-interfaces.md`, `parameters.md`, `testing-ci.md`, and `doc-index.md` — which tells you which document is authoritative and which is a historical record.
