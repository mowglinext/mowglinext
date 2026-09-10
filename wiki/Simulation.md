# Simulation Guide

This guide explains how to run the Mowgli ROS2 system in Webots simulation using Docker containers for testing and development without physical hardware.

> **Maintainer note:** this guide is operator-facing. The simulator migrated
> from Gazebo Ignition to Webots R2025a in commit `364cad30`; any Gazebo-era
> leftover you find in the tree is a bug, not a supported path. For the Webots
> sim's ODE quirks and load-bearing workarounds — read this before editing
> worlds, PROTOs, or `kinematic_drive.py` — see
> [`docs/WEBOTS_SIM.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/WEBOTS_SIM.md).
> For the exact runtime surface (every topic/service/action and its publisher,
> every parameter, every test) see the machine-readable indexes under
> [`docs/claude/`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/doc-index.md).

## Overview

The simulation provides:

- **Virtual Mowgli robot** (the `MowgliMower` PROTO) in a Webots R2025a garden world
- **Simulated sensors:** LiDAR (2D laser scan), IMU (inertial unit + gyro + accelerometer), GPS, wheel odometry
- **Physics + actuation model:** Webots/ODE rigid-body world; the chassis is moved by a Supervisor teleport from the integrated `/cmd_vel` (`kinematic_drive.py`), and `sim_actuation_node` reproduces the STM32 deadband/motor model on the wheel command
- **ROS2 Kilted integration:** Full integration with the ROS2 navigation stack
- **Repeatable scenarios:** Consistent environment for testing mowing patterns and navigation
- **Docker-based workflow:** Containerized simulation eliminates environment conflicts

**What's NOT simulated:**
- The GNSS receiver itself — NTRIP, correction streams and constellation health
  have no counterpart in the sim. Webots publishes a plain fix on `/gps/fix_raw`
  and `sim_navsat_rtk_fix.py` rewrites the status and covariance to an RTK
  regime (RTK-Fixed by default, σ ≈ 3 mm) before republishing on `/gps/fix`.
  GUI validation for public `GnssStatus` baseline/correction/MSM scenarios lives
  in the existing frontend test/mock seam under `gui/web/src/test/mocks.tsx`
- Battery drain / charging (`fake_hardware_bridge_node` reports a constant full
  pack, and fakes "charging" from dock proximity)
- Grass cutting blade physics (motor control still available)
- Weathering / seasonal map changes

## Requirements

### Docker & Compose
- **Docker Engine** 20.10+
- **Docker Compose** 2.0+
- **Host ports available:** 6080 (noVNC), 8765 (Foxglove WebSocket)

### System
- **CPU:** Multi-core processor (8+ cores recommended)
- **RAM:** 4 GB minimum, 8 GB recommended
- **GPU:** Optional (faster rendering with GPU, CPU fallback available)
- **Disk:** ~5 GB for Docker images

### No Bare-Metal Installation Required
All dependencies (ROS2 Kilted, Webots R2025a, tools) are containerized. No installation on your host system needed.

> **amd64 only.** Cyberbotics ships the Webots `.deb` for Linux amd64 only, so
> the `simulation` Docker stage fails fast on any other architecture (ARM
> included — Apple Silicon and Raspberry Pi need emulation or an x86 host). The
> normal `runtime` image stays architecture-neutral.

## Quick Start

### 1. Build Docker Images

```bash
cd docker

# Build simulation image (used by all sim services)
docker compose -f docker-compose.simulation.yaml build simulation

# Or build all images at once
docker compose -f docker-compose.simulation.yaml build
```

### 2. Run Simulation with GUI

Production mode with the Webots GUI accessible via browser (the container is
named `mowgli_sim_gui`):

```bash
cd docker
docker compose -f docker-compose.simulation.yaml up simulation-gui
```

**Output** (from `ros2/scripts/start_vnc.sh`, the service's entrypoint):
```
=== Starting VNC server on :1 (1280x720) ===
=== Starting noVNC web proxy on port 6080 ===

========================================================
  Simulator GUI available at:
    http://localhost:6080/vnc.html

  Foxglove Studio connects to:
    ws://localhost:8765
========================================================
```

**Access:**
- **noVNC GUI:** Open http://localhost:6080/vnc.html in your browser (Webots 3D view)
- **Foxglove Studio:** Connect to ws://localhost:8765 in Foxglove or load layout from `ros2/foxglove/mowgli_sim.json`

### 3. Run Development Simulation

Development mode with source volume mounts for fast iteration:

```bash
cd docker
docker compose -f docker-compose.simulation.yaml up dev-sim
```

This container has config, launch, and behavior tree files bind-mounted, allowing you to:
- Edit config/launch files on your host — changes take effect on restart (no rebuild)
- Edit C++ source — rebuild inside the container
- Run E2E tests directly

**Bind-mounted paths (live-editable, no rebuild):**
- `mowgli_bringup/config/` and `launch/`
- `mowgli_behavior/trees/` and `config/`
- `mowgli_map/config/`
- `mowgli_localization/config/`
- `mowgli_simulation/launch/`

The Webots assets themselves (`mowgli_simulation/worlds_webots/`, `protos/`,
`urdf_webots/`, `config_webots/`) are **not** bind-mounted — editing a world,
the `MowgliMower` PROTO or the URDF needs an image rebuild.

### 4. Test in Development Container

In the dev container shell:

```bash
# Build a single package (fast rebuild)
docker compose -f docker-compose.simulation.yaml exec dev-sim bash -c \
  'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && colcon build --packages-select mowgli_behavior'

# Restart simulation services
docker compose -f docker-compose.simulation.yaml restart dev-sim

# Open a shell in the running container
docker compose -f docker-compose.simulation.yaml exec dev-sim bash

# View logs
docker compose -f docker-compose.simulation.yaml logs -f dev-sim
```

## Accessing Simulation

### GUI Access (noVNC)

Open **http://localhost:6080/vnc.html** in your browser.

The noVNC session shows the Webots window: the 3D viewport plus the toolbar
that runs, pauses and steps the simulation. Viewport navigation and keyboard
shortcuts are Webots' own — see the
[Webots user interface guide](https://cyberbotics.com/doc/guide/the-user-interface).

### Visualization (Foxglove Studio)

1. Open [Foxglove Studio](https://foxglove.dev/) (web or desktop app)
2. Click "Open Connection"
3. Select "Foxglove WebSocket" and enter: **ws://localhost:8765**
4. Import layout from `ros2/foxglove/mowgli_sim.json` (pre-configured with panels for LiDAR, odometry, transforms, status)

**Foxglove displays:**
- 3D view with robot pose and sensor data
- LiDAR laser scan (/scan)
- IMU accelerometer/gyroscope (/imu/data)
- Odometry and transform tree (/tf)
- Behavior tree state (if running)
- Command velocity monitoring

## End-to-End (E2E) Test

MowgliNext includes an automated E2E test that validates the full mowing cycle in simulation — from undocking through coverage mowing to docking, including obstacle avoidance with rerouting.

### Running the E2E test

```bash
cd docker

# Start the dev simulation
docker compose -f docker-compose.simulation.yaml up -d dev-sim

# Wait for Webots + Nav2 to initialize (~30-60 seconds)

# Run the E2E test
docker compose -f docker-compose.simulation.yaml exec dev-sim \
  bash -c "source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/e2e_test.py"
```

### What the E2E test validates

The test runs a complete autonomous mowing cycle and tracks structured phases:

| Phase | What's tested |
|-------|--------------|
| **UNDOCKING** | Robot undocks via Nav2 BackUp behavior (1.5 m / 0.15 m/s) — `opennav_docking` UndockRobot is unreliable with GPS drift near the dock |
| **PLANNING** | `mowgli_coverage` (Fields2Cover 3.0.0) generates the per-area path: `f2c::hg::ConstHL` headland rings + `f2c::sg::BruteForce` swaths ordered by `f2c::rp::BoustrophedonOrder`, joined by Mowgli's own forward-only turn-around connectors (`buildConnector` — largest in-bounds Dubins radius, shrinking toward `op_width/2`, straight fallback). F2C's own turn planners are deliberately not used. One plan per area per session. |
| **MOWING** | Robot follows coverage path with FTCController (decoupled lon/lat/ang PID). Coverage completion gated by `PathProgressGoalChecker` (`progress_threshold: 0.95`). |
| **OBSTACLE AVOIDANCE** | The Webots world carries a static 0.5 m box (`test_obstacle` at 3.0, 1.5) on the lawn; the test watches for stop-and-resume around it. FTC's native `enable_obstacle_deviation` skirts it (`max_lateral_deviation: 1.5` m); on harder failures the BT's `StuckBackoff` branch runs `BackUp(0.40 m, 0.15 m/s) + ClearCostmap`, then returns FAILURE so `FollowStripRetry` re-ticks `FollowStrip`, which resumes at the furthest pose it reached. FTC's `setPlan` always starts at index 0 — resume trimming is `FollowStrip`'s job, not the controller's. |
| **DOCKING** | Robot returns to dock via `opennav_docking` DockRobot after mowing or on failure |

### Metrics collected

The test collects and reports:

- **Obstacle proximity** — minimum LiDAR ranges, collision detection
- **Reroute events** — counts mid-swath obstacle rerouting attempts
- **BT state transitions** — full timeline of behavior tree state changes
- **Movement metrics** — total distance, average speed
- **Swath progress** — total / completed / skipped swaths, read from `HighLevelStatus`

> **Three sections of the report are currently dead** and always print empty:
> *Path tracking quality* subscribes to `/coverage_planner_node/coverage_path`
> (the live coverage plan is published on `/coverage/full_plan`), *SLAM map
> growth* subscribes to `/map` (nothing publishes it since slam_toolbox was
> removed — `fusion_graph` is the sole localizer and builds no occupancy map),
> and *GPS degradation events* subscribes to `/gps_degradation_sim/status`
> (no such node exists). Ignore those three lines until the harness is rewired.

### Interpreting results

The test produces a final report with PASS/FAIL verdict (illustrative — the
path-tracking block is one of the dead sections noted above):

```
========== E2E TEST RESULTS ==========
Phase Results:
  UNDOCKING:  PASS (12.3s)
  PLANNING:   PASS (2.1s)
  MOWING:     PASS (845.2s)
  DOCKING:    PASS (18.7s)

Path Tracking Quality: Excellent
  Mean: 0.12m | Median: 0.08m | P95: 0.28m

Obstacle Avoidance: PASS
  Robot stopped: YES | Rerouted: YES

OVERALL: PASS
=======================================
```

**Quality thresholds:**
- **Excellent:** median deviation < 30cm
- **Acceptable:** median deviation < 50cm
- **Needs tuning:** median deviation 50cm – 1m
- **Fail:** median deviation > 1m

### Test timeout

The mowing cycle has a 20-minute timeout (`timeout = 1200.0`); the manual-mowing, area-recording and emergency-auto-reset feature tests then run regardless. The script shuts down cleanly on SIGTERM. Its obstacle-cleanup step is a Gazebo-era leftover that shells out to `gz service` and is a silent no-op under Webots — harmless, because the world's obstacle is static and nothing is spawned.

## Manual Mowing Cycle Test

For interactive testing with visual monitoring:

```bash
cd docker

# Terminal 1: Start simulation with GUI
docker compose -f docker-compose.simulation.yaml up simulation-gui
# Wait for the "Simulator GUI available at:" banner, then for Nav2 to activate

# Terminal 2: Send high-level control command
docker exec mowgli_sim_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 service call /behavior_tree_node/high_level_control \
    mowgli_interfaces/srv/HighLevelControl \
    '{command: 1}'"
```

**Available commands:**
- `command: 1` → START (begin mowing cycle)
- `command: 2` → HOME (return to home position)
- `command: 3` → RECORD_AREA (start area boundary recording; alias `COMMAND_S1`)

**Monitor the behavior tree:**

```bash
# In another terminal
docker exec mowgli_sim_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 topic echo /behavior_tree_node/high_level_status"
```

## World Environments

### Garden World (Default)

**Location:** `ros2/src/mowgli_simulation/worlds_webots/mowgli_garden.wbt`

Features:
- 20m × 20m green `Floor` (the "lawn")
- One static 0.5 m box (`test_obstacle`, at 3.0, 1.5) for coverage-deviation testing
- `basicTimeStep 20` (50 Hz), ENU coordinate system, and a WGS84 `gpsReference`
  matching the datum in `mowgli_robot.yaml`
- The `MowgliMower` PROTO instance (`protos/MowgliMower.proto`) with the LiDAR,
  GPS, inertial unit, gyro and accelerometer in its `extensionSlot`

To verify it's loaded, check the noVNC view or the Foxglove 3D display.

### Custom Worlds

To create a custom world:

1. Copy `mowgli_garden.wbt` to a new `.wbt` file in
   `ros2/src/mowgli_simulation/worlds_webots/`
2. Pass it with the `world` launch argument, e.g.
   `ros2 launch mowgli_bringup sim_full_system.launch.py world:=my_yard.wbt`
3. Rebuild the Docker image — the Webots assets are baked in, not bind-mounted

Keep the `EXTERNPROTO "../protos/MowgliMower.proto"` import, the `MowgliMower`
instance (with `controller "<extern>"` and `supervisor TRUE`), `WorldInfo`'s
`basicTimeStep 20` and the `gpsCoordinateSystem "WGS84"` / `gpsReference` block
— the ROS side depends on all four.

> Before editing any world, PROTO or the URDF, read
> [`docs/WEBOTS_SIM.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/WEBOTS_SIM.md).
> Several non-obvious constructs there (wheel-cylinder rotations, the negative
> hinge axis, the Supervisor teleport) are load-bearing, and breaking one
> presents as a Nav2 bug rather than a sim bug.

## Development Workflow

### Edit and Rebuild in Development Mode

```bash
cd docker

# Terminal 1: Start dev container
docker compose -f docker-compose.simulation.yaml up dev-sim

# Terminal 2: Edit your code
# e.g., modify ros2/src/mowgli_behavior/src/behavior_tree_node.cpp

# Terminal 3: Rebuild package inside container
docker compose -f docker-compose.simulation.yaml exec dev-sim bash -c \
  'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && colcon build --packages-select mowgli_behavior'

# Restart simulation to pick up changes:
docker compose -f docker-compose.simulation.yaml restart dev-sim
```

### Interactive Development Shell

For direct container access:

```bash
docker compose -f docker-compose.simulation.yaml exec dev-sim bash

# Now you're inside the container
cd /ros2_ws
colcon build --packages-select mowgli_behavior
source install/setup.bash
ros2 launch mowgli_bringup sim_full_system.launch.py
```

Useful arguments: `world:=<file>.wbt`, `mode:=realtime|fast|pause`,
`use_lidar:=false` (GPS-only variant), `use_rviz:=true`. The `headless` argument
is a Gazebo-era back-compat shim: it is still accepted, but ignored. Whether you
see a Webots window depends on the container's X display — the `simulation` and
`dev-sim` services point it at Xvfb, `simulation-gui` at the VNC server.

### Logging and Debugging

```bash
cd docker

# View container logs
docker compose -f docker-compose.simulation.yaml logs -f dev-sim

# Check ROS2 nodes running inside container
docker compose -f docker-compose.simulation.yaml exec dev-sim bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 node list"

# Echo a specific topic
docker compose -f docker-compose.simulation.yaml exec dev-sim bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 topic echo /scan --no-arr"
```

## ROS2 Topics and Services

### Raw Topics (Webots driver → ROS2)

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/clock` | rosgraph_msgs/Clock | per sim step (`basicTimeStep 20` → 50 Hz) | Simulation time (`use_sim_time`) |
| `/scan` | sensor_msgs/LaserScan | 10 Hz | 2D LiDAR, 720 rays, 0.10–12 m |
| `/imu/data_sim` | sensor_msgs/Imu | 100 Hz | Noise-free IMU (inertial unit + gyro + accelerometer) |
| `/gps/fix_raw` | sensor_msgs/NavSatFix | 5 Hz | Webots GPS device, plain `STATUS_FIX` |
| `/wheel_odom_raw` | nav_msgs/Odometry | 50 Hz | `diffdrive_controller` wheel odometry |
| `/cmd_vel_wheels` | geometry_msgs/TwistStamped | 50 Hz | Wheel command after `sim_actuation_node`'s firmware deadband + PI motor model |
| `/cmd_vel` | geometry_msgs/TwistStamped | – | Nav command; also drives the Supervisor body teleport |

### ROS2 Processed Topics

Each raw sensor stream is passed through a sim-only post-processor before the
production stack sees it, so the topic the rest of the system consumes is not
the one Webots publishes:

| Topic | Type | Purpose |
|-------|------|---------|
| `/imu/data` | sensor_msgs/Imu | `/imu/data_sim` + MEMS white noise and bias walk (`sim_imu_noise.py`) |
| `/gps/fix` | sensor_msgs/NavSatFix | `/gps/fix_raw` with an RTK status + covariance regime (`sim_navsat_rtk_fix.py`) |
| `/wheel_odom` | nav_msgs/Odometry | `/wheel_odom_raw` + periodic slip events (`sim_wheel_slip.py`) |
| `/odometry/filtered` | nav_msgs/Odometry | `fusion_graph_node`, odom frame (dead reckoning) |
| `/odometry/filtered_map` | nav_msgs/Odometry | `fusion_graph_node`, map frame (GPS-anchored) |
| `/coverage/full_plan` | nav_msgs/Path | Current area's continuous coverage plan (latched) |
| `/tf` | tf2_msgs/TFMessage | Transform tree — `fusion_graph_node` publishes **both** `map → odom` and `odom → base_footprint` |

### High-Level Control Service

```bash
ros2 service call /behavior_tree_node/high_level_control \
  mowgli_interfaces/srv/HighLevelControl \
  '{command: 1}'

# 1 = START
# 2 = HOME
# 3 = RECORD_AREA (alias COMMAND_S1)
```

### Monitor Behavior Tree Status

```bash
docker exec mowgli_sim_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 topic echo /behavior_tree_node/high_level_status"
```

## Performance Tuning

### Optimize CPU Usage

Development mode runs lighter than production. Use `docker compose up dev-sim` for iterative work.

Production mode can be optimized:
```bash
# Reduce sensor frequencies in the Webots device wiring
#   urdf_webots/mowgli_webots.urdf: <updateRate> per device
#   Lower /scan from 10 Hz to 2 Hz, /imu/data_sim from 100 Hz to 50 Hz
# Reduces CPU load and network overhead (needs an image rebuild)
```

Webots runs in `realtime` mode by default (1x wall clock). `mode:=fast` lets the
simulator run ahead, but the Nav2 controller loop then gets CPU-starved and
FTC's `PRE_ROTATE` can miss its goal timeout — only use it when the timing
budget allows.

### GPU Acceleration

Docker simulation can use GPU if available:

```bash
# Modify docker-compose.yml to include:
# runtime: nvidia
# and rebuild
docker compose build simulation
```

Check the Webots rendering speed in noVNC. GPU provides 2-3x faster rendering.

## Container Management

### Stop Containers

```bash
# Stop production container
docker stop mowgli_sim_gui

# Stop development container
docker stop mowgli_dev_sim

# Stop and remove everything
docker compose down
```

### View Running Containers

```bash
docker ps | grep mowgli_simulation
```

### Access Container Filesystem

```bash
# Explore container
docker exec -it mowgli_sim_gui bash

# Copy files from container
docker cp mowgli_sim_gui:/ros2_ws/install/lib /tmp/extracted_lib

# View container logs
docker logs -f mowgli_sim_gui
```

## Troubleshooting

### Issue: noVNC Not Accessible (http://localhost:6080/vnc.html)

**Solution:**
- Check port isn't already in use: `lsof -i :6080`
- Verify container is running: `docker ps | grep mowgli_sim_gui`
- Check firewall settings (if on remote machine)
- Wait 30 seconds after starting container for VNC to initialize

### Issue: Foxglove WebSocket Connection Failed (ws://localhost:8765)

**Cause:** WebSocket bridge not running or port blocked.

**Check:**
```bash
docker exec mowgli_sim_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 node list | grep bridge"
```

**Solution:**
- Restart container: `docker compose restart simulation-gui`
- Check firewall: `lsof -i :8765`
- Verify ROS2 domain ID matches (should be 0)

### Issue: Robot Doesn't Appear in Webots

**Check:**
```bash
docker exec mowgli_sim_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 topic list | grep -E 'scan|odom|cmd_vel'"
```

**Solution:**
- Give Webots 20-40 s to finish loading the world before the `<extern>`
  controller slot opens. `webots_controller` has a hard-coded 30 s connect
  timeout, so the first attempt often dies; the launch file sets `respawn=True`
  and the second attempt normally succeeds
- Check simulation logs: `docker compose logs -f dev-sim` (if using dev mode)
- Rebuild and restart: `docker compose build dev-sim && docker compose restart dev-sim`

### Issue: Slow Rendering or CPU Maxed Out

**Solution:**
- Use `docker compose up dev-sim` instead of production image (lighter)
- Reduce sensor rates in configuration
- Close unnecessary applications on host
- Check GPU acceleration is enabled (if available)

### Issue: Cannot Connect to Development Container

**Check:**
```bash
docker ps -a | grep mowgli_dev_sim
docker logs mowgli_dev_sim
```

**Solution:**
- Rebuild: `docker compose build dev-sim`
- Restart: `docker compose up dev-sim`
- Check disk space: `docker system df`

### Issue: Source Edits Don't Take Effect in Dev Mode

**Solution:**
```bash
# Rebuild the specific package
docker compose exec dev-sim bash -c \
  'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && colcon build --packages-select mowgli_behavior'

# Or full rebuild
docker compose exec dev-sim bash -c \
  'source /opt/ros/kilted/setup.bash && colcon build'

# Restart simulation
docker compose restart dev-sim
```

## Integration Testing Examples

### Test 1: Full Mowing Cycle

```bash
# Terminal 1
docker compose up simulation-gui

# Terminal 2 (after 30-60 seconds for Webots + Nav2 to load)
docker exec mowgli_sim_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 service call /behavior_tree_node/high_level_control \
    mowgli_interfaces/srv/HighLevelControl '{command: 1}'"

# Terminal 3: Monitor behavior
docker exec mowgli_sim_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 topic echo /behavior_tree_node/high_level_status"
```

Open noVNC (http://localhost:6080/vnc.html) to watch robot execute mowing pattern.

### Test 2: LiDAR Sensor Verification

```bash
# Terminal 1
docker compose up simulation-gui

# Terminal 2: Record laser scan
docker exec mowgli_sim_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 topic echo /scan --no-arr | head -20"
```

Verify scan data shows valid ranges and angles.

### Test 3: Odometry and Localization

```bash
# Terminal 1
docker compose up simulation-gui

# Terminal 2: Start recording odometry data
docker exec mowgli_sim_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 bag record -o test_odom_0 \
    /odometry/filtered_map \
    /wheel_odom \
    /tf \
    --duration 30"

# Terminal 3: Send navigation goal
docker exec mowgli_sim_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose \
    'pose: {header: {frame_id: \"map\"}, pose: {position: {x: 5.0, y: 5.0}, orientation: {w: 1.0}}}'"
```

Analyze recorded data for drift and localization convergence.

## Next Steps

- **[Configuration](Configuration)** – Tune navigation and localization parameters
- **[Architecture](Architecture)** – Understand the full system design
- **[Webots sim internals](https://github.com/mowglinext/mowglinext/blob/main/docs/WEBOTS_SIM.md)** – ODE quirks and load-bearing workarounds before editing sim assets
- **Real Hardware** – Follow the [README](https://github.com/mowglinext/mowglinext/blob/main/README.md) to deploy on an actual Mowgli robot

---

**Happy simulating!**
