# Simulation Guide

This guide explains how to run the Mowgli ROS2 system in Gazebo Harmonic simulation using Docker containers for testing and development without physical hardware.

## Overview

The simulation provides:

- **Virtual Mowgli robot** in a realistic Gazebo Harmonic environment
- **Simulated sensors:** LiDAR (2D laser scan), IMU (accelerometer + gyroscope), wheel odometry
- **Physics simulation:** Motor torques, friction, collision detection
- **ROS2 Kilted integration:** Full integration with the ROS2 navigation stack
- **Repeatable scenarios:** Consistent environment for testing mowing patterns and navigation
- **Docker-based workflow:** Containerized simulation eliminates environment conflicts

**What's NOT simulated:**
- RTK-GPS (robot-side receiver behavior is not simulated; GUI validation for
  public `GnssStatus` baseline/correction/MSM scenarios lives in the existing
  frontend test/mock seam under `gui/web/src/test/mocks.tsx`)
- Battery drain / charging
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
All dependencies (ROS2 Kilted, Gazebo Harmonic, tools) are containerized. No installation on your host system needed.

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

Production mode with Gazebo GUI accessible via browser:

```bash
cd docker
docker compose -f docker-compose.simulation.yaml up simulation-gui
```

**Output:**
```
Starting mowgli_simulation_gui container...
noVNC available at: http://localhost:6080/vnc.html
Foxglove WebSocket: ws://localhost:8765
Simulation running...
```

**Access:**
- **noVNC GUI:** Open http://localhost:6080/vnc.html in your browser (Gazebo 3D view)
- **Foxglove Studio:** Connect to ws://localhost:8765 in Foxglove or load layout from `foxglove/mowgli_sim.json`

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
- `mowgli_simulation/launch/`, `worlds/`, `config/`

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

**Controls in Gazebo (inside noVNC):**
- **Mouse wheel:** Zoom in/out
- **Right-click drag:** Pan camera
- **Middle-click drag:** Rotate view
- **G:** Toggle grid
- **T:** Toggle collision shapes (debug mode)
- **V:** Cycle camera view
- **?:** Show all hotkeys

### Visualization (Foxglove Studio)

1. Open [Foxglove Studio](https://foxglove.dev/) (web or desktop app)
2. Click "Open Connection"
3. Select "Foxglove WebSocket" and enter: **ws://localhost:8765**
4. Import layout from `foxglove/mowgli_sim.json` (pre-configured with panels for LiDAR, odometry, transforms, status)

**Foxglove displays:**
- 3D view with robot pose and sensor data
- LiDAR point cloud (/scan)
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

# Wait for Gazebo + Nav2 to initialize (~30-60 seconds)

# Run the E2E test
docker compose -f docker-compose.simulation.yaml exec dev-sim \
  bash -c "source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/e2e_test.py"
```

### What the E2E test validates

The test runs a complete autonomous mowing cycle and tracks structured phases:

| Phase | What's tested |
|-------|--------------|
| **UNDOCKING** | Robot undocks via Nav2 BackUp behavior (1.5 m / 0.15 m/s) — `opennav_docking` UndockRobot is unreliable with GPS drift near the dock |
| **PLANNING** | `mowgli_coverage` (Fields2Cover v2.0) generates the per-area path (ConstHL headland + TrapezoidalDecomp + BoustrophedonOrder + Dubins connectors). One plan per area per session. |
| **MOWING** | Robot follows coverage path with FTCController (3-axis PID, <10 mm lateral). Coverage completion gated by `PathProgressGoalChecker` (>= 95 % path-pose tracking). |
| **OBSTACLE AVOIDANCE** | Spawns a 30 cm obstacle on the first swath. FTC's native `enable_obstacle_deviation` skirts the obstacle (lateral offset up to 1.5 m); on harder failures, the BT inserts `BackUp(0.40m) + ClearCostmap` and re-ticks `FollowStrip` on the SAME path (FTC `setPlan` resyncs to the closest pose). |
| **DOCKING** | Robot returns to dock via `opennav_docking` DockRobot after mowing or on failure |

### Metrics collected

The test collects and reports:

- **Path tracking quality** — deviation between planned and actual poses (target: median < 50cm)
- **SLAM map growth** — verifies the map is being built during mowing
- **GPS degradation events** — logs any GPS state transitions
- **Obstacle proximity** — minimum LiDAR ranges, collision detection
- **Reroute events** — counts mid-swath obstacle rerouting attempts
- **BT state transitions** — full timeline of behavior tree state changes
- **Movement metrics** — total distance, average speed

### Interpreting results

The test produces a final report with PASS/FAIL verdict:

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
- **Needs tuning:** median deviation > 50cm

### Test timeout

The E2E test has a 20-minute timeout and handles graceful cleanup (removes spawned obstacles, shuts down cleanly on SIGTERM).

## Manual Mowing Cycle Test

For interactive testing with visual monitoring:

```bash
cd docker

# Terminal 1: Start simulation with GUI
docker compose -f docker-compose.simulation.yaml up simulation-gui
# Wait for "Simulation running..." message

# Terminal 2: Send high-level control command
docker exec mowgli_simulation_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 service call /behavior_tree_node/high_level_control \
    mowgli_interfaces/srv/HighLevelControl \
    '{command: 1}'"
```

**Available commands:**
- `command: 1` → START (begin mowing cycle)
- `command: 2` → HOME (return to home position)
- `command: 3` → S1 (recording mode)

**Monitor the behavior tree:**

```bash
# In another terminal
docker exec mowgli_simulation_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 topic echo /behavior_tree_node/status"
```

## World Environments

### Garden World (Default)

The default world includes obstacles and realistic garden layout:

**Location:** `src/mowgli_simulation/worlds/garden.sdf`

Features:
- 10m × 10m grass area (green plane)
- Obstacles (walls, trees) for navigation testing
- Proper lighting and physics

To verify it's loaded, check noVNC view or Foxglove 3D display.

### Custom Worlds

To create a custom world:

1. Create a new `.sdf` file in `src/mowgli_simulation/worlds/`
2. Update the simulation launch configuration to point to your world
3. Rebuild the Docker image if you want it included in the production image

**Example custom world structure:**
```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="custom_yard">
    <physics name="default" type="dart">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
    </physics>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.2 0.5 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Add obstacles, models, lighting, etc. -->
  </world>
</sdf>
```

## Development Workflow

### Edit and Rebuild in Development Mode

```bash
cd docker

# Terminal 1: Start dev container
docker compose -f docker-compose.simulation.yaml up dev-sim

# Terminal 2: Edit your code
# e.g., modify src/mowgli_behavior/src/behavior_tree.cpp

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
ros2 launch mowgli_bringup sim_full_system.launch.py headless:=true
```

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

### Bridged Topics (Gazebo → ROS2)

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/clock` | rosgraph_msgs/Clock | 1000 Hz | Simulation time (use_sim_time) |
| `/scan` | sensor_msgs/LaserScan | ~10 Hz | 2D LiDAR scan |
| `/imu/data` | sensor_msgs/Imu | 100 Hz | IMU (accelerometer + gyroscope) |
| `/wheel_odom` | nav_msgs/Odometry | ~50 Hz | Wheel encoder odometry |
| `/cmd_vel` | geometry_msgs/Twist | – | Motor velocity commands (ROS → Gazebo) |

### ROS2 Processed Topics

| Topic | Type | Purpose |
|-------|------|---------|
| `/odometry/filtered` | nav_msgs/Odometry | Local EKF (wheels + gyro, odom frame) |
| `/odometry/filtered_map` | nav_msgs/Odometry | Global EKF (+ GPS + COG yaw, map frame) |
| `/map` | nav_msgs/OccupancyGrid | Occupancy grid from SLAM |
| `/tf` | tf2_msgs/TFMessage | Transform tree (map → odom → base_footprint) |

### High-Level Control Service

```bash
ros2 service call /behavior_tree_node/high_level_control \
  mowgli_interfaces/srv/HighLevelControl \
  '{command: 1}'

# 1 = START
# 2 = HOME
# 3 = S1 (recording)
```

### Monitor Behavior Tree Status

```bash
docker exec mowgli_simulation_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 topic echo /behavior_tree_node/status"
```

## Performance Tuning

### Optimize CPU Usage

Development mode runs lighter than production. Use `docker compose up dev-sim` for iterative work.

Production mode can be optimized:
```bash
# Reduce sensor frequencies in launch configuration
# Lower /scan rate from 10 Hz to 2 Hz
# Lower /imu rate from 100 Hz to 50 Hz
# Reduces CPU load and network overhead
```

### GPU Acceleration

Docker simulation can use GPU if available:

```bash
# Modify docker-compose.yml to include:
# runtime: nvidia
# and rebuild
docker compose build simulation
```

Check Gazebo rendering speed in noVNC. GPU provides 2-3x faster rendering.

## Container Management

### Stop Containers

```bash
# Stop production container
docker stop mowgli_simulation_gui

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
docker exec -it mowgli_simulation_gui bash

# Copy files from container
docker cp mowgli_simulation_gui:/ros2_ws/install/lib /tmp/extracted_lib

# View container logs
docker logs -f mowgli_simulation_gui
```

## Troubleshooting

### Issue: noVNC Not Accessible (http://localhost:6080/vnc.html)

**Solution:**
- Check port isn't already in use: `lsof -i :6080`
- Verify container is running: `docker ps | grep mowgli_simulation_gui`
- Check firewall settings (if on remote machine)
- Wait 30 seconds after starting container for VNC to initialize

### Issue: Foxglove WebSocket Connection Failed (ws://localhost:8765)

**Cause:** WebSocket bridge not running or port blocked.

**Check:**
```bash
docker exec mowgli_simulation_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 node list | grep bridge"
```

**Solution:**
- Restart container: `docker compose restart simulation-gui`
- Check firewall: `lsof -i :8765`
- Verify ROS2 domain ID matches (should be 0)

### Issue: Robot Doesn't Appear in Gazebo

**Check:**
```bash
docker exec mowgli_simulation_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 topic list | grep -E 'scan|odom|cmd_vel'"
```

**Solution:**
- Wait 10-15 seconds for Gazebo to initialize model spawn
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

# Terminal 2 (after 30 seconds for Gazebo to load)
docker exec mowgli_simulation_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 service call /behavior_tree_node/high_level_control \
    mowgli_interfaces/srv/HighLevelControl '{command: 1}'"

# Terminal 3: Monitor behavior
docker exec mowgli_simulation_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 topic echo /behavior_tree_node/status"
```

Open noVNC (http://localhost:6080/vnc.html) to watch robot execute mowing pattern.

### Test 2: LiDAR Sensor Verification

```bash
# Terminal 1
docker compose up simulation-gui

# Terminal 2: Record laser scan
docker exec mowgli_simulation_gui bash -c "\
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
docker exec mowgli_simulation_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 bag record -o test_odom_0 \
    /odometry/filtered_map \
    /wheel_odom \
    /tf \
    --duration 30"

# Terminal 3: Send navigation goal
docker exec mowgli_simulation_gui bash -c "\
  source /opt/ros/kilted/setup.bash && \
  source /ros2_ws/install/setup.bash && \
  ros2 action send_goal navigate_to_pose nav2_msgs/action/NavigateToPose \
    'pose: {header: {frame_id: \"map\"}, pose: {position: {x: 5.0, y: 5.0}, orientation: {w: 1.0}}}'"
```

Analyze recorded data for drift and localization convergence.

## Next Steps

- **[CONFIGURATION.md](CONFIGURATION.md)** – Tune navigation and localization parameters
- **[ARCHITECTURE.md](ARCHITECTURE.md)** – Understand the full system design
- **Real Hardware** – Follow [README.md](../README.md) to deploy on actual Mowgli robot

---

**Happy simulating!**
