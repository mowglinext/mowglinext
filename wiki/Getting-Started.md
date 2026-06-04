# Getting Started

> **Looking for the short version?** See [`docs/FIRST_BOOT.md`](https://github.com/cedbossneo/mowglinext/blob/main/docs/FIRST_BOOT.md) in the main repo for the docked-only post-install checklist (GUI open → RTK Fixed → IMU calibration → dock pose → record area → first autonomous mow).

## Hardware

### Compute Board

Any ARM64 SBC running Linux with Docker support:
- **Recommended:** Rockchip RK3566, RK3588
- **Also works:** Raspberry Pi 4, Pi 5
- **Minimum:** 4-core ARM64, 4 GB RAM, 16 GB storage

### Mower Chassis

| Model | Status |
|-------|--------|
| YardForce Classic 500 | Primary target |
| YardForce Classic 500B | Supported |
| YardForce LUV1000Ri | Supported |

### Sensors

| Sensor | Model | Connection |
|--------|-------|------------|
| RTK GPS | u-blox ZED-F9P (simpleRTK2B) | USB-CDC |
| LiDAR | LDRobot LD19 | UART 230400 |
| IMU | WitMotion WT901 (6-axis gyro + accel) | I2C (on STM32) |

### Firmware Board

Custom Mowgli STM32F103 PCB — handles motor control, IMU, blade safety, battery monitoring.

## Development with GitHub Codespaces / DevContainer

The fastest way to explore and develop MowgliNext — no local setup required:

### GitHub Codespaces (cloud)

1. Go to [github.com/cedbossneo/mowglinext](https://github.com/cedbossneo/mowglinext)
2. Click **Code → Codespaces → Create codespace on main**
3. Select a **8-core** machine type (16-core recommended for simulation)
4. Wait for the container to build (~10 min first time, cached after)

### VS Code DevContainer (local)

1. Install [Docker](https://docs.docker.com/get-docker/) and the [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
2. Clone the repo: `git clone https://github.com/cedbossneo/mowglinext.git`
3. Open in VS Code → **Reopen in Container** when prompted

### What's included

The devcontainer provides a complete ROS2 Kilted development environment:

- Full Nav2 navigation stack, robot_localization (default dual-EKF localizer), and an opt-in GTSAM iSAM2 factor-graph localizer (`fusion_graph`) with LiDAR scan-matching and loop-closure factors
- Webots simulation (headless)
- `mowgli_coverage` (Fields2Cover 2.0.0 at `/opt/fields2cover-200`) per-area path planner + FTCController for both transit and coverage following
- Foxglove Bridge + rosbridge for visualization
- GUI stack (Go 1.24, Node 22, yarn) so `cd gui && go build` and `cd gui/web && yarn dev` work out of the box
- Python linting (ruff, pre-commit), C++ linting (cppcheck, clang-format), gdb, htop
- Claude Code CLI + GitHub CLI for AI-assisted development
- Auto-sourced ROS2 workspace

**Forwarded ports:**

| Port | Service |
|------|---------|
| 8765 | Foxglove WebSocket |
| 6080 | noVNC (Gazebo GUI) |
| 9090 | rosbridge |

### Build and run simulation in devcontainer

```bash
# Build all packages
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the workspace
source install/setup.bash

# Launch simulation
ros2 launch mowgli_bringup sim_full_system.launch.py headless:=true
```

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
git clone https://github.com/cedbossneo/mowglinext.git
cd mowglinext
```

### 2. Set Up udev Rules

Create stable device symlinks:

```bash
# /etc/udev/rules.d/50-mowgli.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="6018", SYMLINK+="mowgli", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", SYMLINK+="gps", MODE="0666"
```

Reload: `sudo udevadm control --reload-rules && sudo udevadm trigger`

### 3. Configure

```bash
cd docker
cp .env.example .env
nano config/mowgli/mowgli_robot.yaml
```

Key settings to change:
- `datum_lat` / `datum_long` — your GPS reference point
- `dock_pose_x` / `dock_pose_y` / `dock_pose_yaw` — dock position. `dock_pose_yaw`
  is a **map-frame ENU heading in radians** (not a phone-compass bearing); it is
  normally captured automatically by "Set Docking Point" / undock calibration
  rather than hand-entered.
- `ntrip_host` / `ntrip_user` / `ntrip_password` — RTK correction source

### 4. Launch

```bash
docker compose up -d
```

### 5. Access

| Service | URL |
|---------|-----|
| GUI | `http://<mower-ip>:4006` |
| Foxglove | `ws://<mower-ip>:8765` |
| Rosbridge | `ws://<mower-ip>:9090` |

## GUI Features

### Diagnostics Page

Access the diagnostics dashboard at `http://<mower-ip>:4006/#/diagnostics`.

**Displays:**
- **Containers:** Status of each Docker service (ros2, slam, nav2, etc.)
- **System metrics:** CPU usage, memory, disk
- **Localization:** robot_localization pose (`/odometry/filtered_map`), position covariance, orientation
- **Sensors:** GPS status, LiDAR freshness, IMU health
- **Behavior Tree:** Current active node, execution state, mode
- **Coverage:** Areas remaining, current strip progress, cells mowed
- **ROS Diagnostics:** Aggregated health status (OK, WARN, ERROR, STALE) for all subsystems
- **Map Management:** View/edit mowing areas, clear mow progress, manage dock pose calibration.

**Configuration Cross-Checks:**
Verifies consistency between:
- GPS datum origin and robot odometry
- SLAM resolution vs. coverage cell width
- Motor power limits and battery voltage
- Emergency stop state and hardware ready status

### Statistics Page

Access session statistics at `http://<mower-ip>:4006/#/statistics`.

**Automatic Session Recording:**
Each mowing session is automatically logged with:
- Start time, end time, duration
- Total distance traveled
- Area mowed (m²), mowing efficiency (%)
- Blade time active
- Obstacles detected and avoided
- Power consumed (Wh)

**Session History:**
- View past sessions with date, duration, area covered
- Compare efficiency across multiple runs
- Track seasonal trends in mowing patterns

**Aggregate Statistics:**
- Total mowed area across all sessions
- Average efficiency and speed
- Maintenance intervals (blade usage hours)
- Weather exposure (rain detection events)

## Next Steps

- [Simulation](Simulation) — test without hardware, run E2E tests
- [Configuration](Configuration) — tune parameters for your environment
- [Deployment](Deployment) — advanced Docker deployment options
- [Contributing](Contributing) — contribute to MowgliNext
