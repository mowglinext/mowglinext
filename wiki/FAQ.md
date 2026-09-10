# FAQ

## General

### What hardware do I need?

At minimum: YardForce Classic 500, ARM64 SBC (Pi 4+), u-blox ZED-F9P GPS, and the Mowgli STM32 board. A LiDAR (LDRobot LD19, RPLidar or STL27L) is **optional but strongly recommended** — with `lidar_enabled: false` the stack runs GPS-only, and obstacle avoidance falls back to the firmware alone (`collision_monitor` is configured as a pass-through in that variant). See [Getting Started](Getting-Started#hardware).

### Is this compatible with OpenMower?

MowgliNext is a complete ROS2 rewrite inspired by OpenMower. It uses the same hardware but a completely different software stack (ROS2 Kilted vs ROS1 Noetic).

### Do I need an NTRIP service for RTK?

Yes, for centimeter-accurate positioning you need an RTK correction source. Many countries have free government NTRIP services, or you can set up your own base station.

## Deployment

### Can I run this on a Raspberry Pi 4?

Yes, with 4GB+ RAM. The Pi 5 is recommended for better performance. Rockchip RK3588 boards offer the best experience.

### Why Cyclone DDS instead of FastRTPS?

FastRTPS had stale shared memory issues on ARM boards causing DDS discovery failures. Cyclone DDS is more reliable for containerized multi-process setups.

### How do I update?

Watchtower runs in label-enable mode and polls every 4 h, but only the **GUI** container carries the `com.centurylinklabs.watchtower.enable` label — the ROS2, GPS and LiDAR images are updated manually. On an installed robot the helper scripts do it:
```bash
mowgli-pull && mowgli-up -d
```
Or, from the deployment directory directly:
```bash
cd docker && docker compose pull && docker compose up -d
```

## Navigation

### The robot stops but doesn't avoid obstacles

Check the whole scan chain, not just the driver: the LiDAR publishes `/scan`, and `costmap_scan_filter_node` republishes it as `/scan_collision` (what `collision_monitor` polls) and `/scan_costmap` (what the costmap obstacle layers consume). If either derived topic is silent, nothing avoids anything.

During coverage the path comes from `mowgli_coverage` (Fields2Cover), not from a costmap planner — but the costmap obstacle layers are *not* disabled: `FTCController` reads the local `obstacle_layer` and skirts obstacles by deviating the path laterally in real time (`enable_obstacle_deviation`), with `collision_monitor` as the hard stop underneath. With `lidar_enabled: false` both of those are off by design and only the firmware stops the robot.

### GPS position drifts after undocking

This is expected — GPS needs a few seconds to converge after the robot moves away from the dock. The behavior tree handles it explicitly: right after the BackUp undock it runs `WaitForGpsFix` (20 s timeout, `min_fix_type=4`), so mowing only starts once RTK-Fixed is back. The `gps_wait_after_undock_sec` setting you may see in the GUI is a leftover with no consumer in the ROS2 stack.

### Where does the map come from?

There is **no SLAM**. The `/map` OccupancyGrid is built by `map_server_node` from polygons you record during area-recording mode (drive the boundary, hit *Finish*, the trajectory is Douglas-Peucker simplified and saved). Persistence is a single plain-text file, `/ros2_ws/maps/areas.dat`, inside the `mowgli_maps` Docker volume — make sure that volume is mounted (and that `COMPOSE_PROJECT_NAME` never changes, or the volume is orphaned).

### Which map-frame localizer does MowgliNext use?

There is only one, and you don't choose it: `fusion_graph_node`, a GTSAM iSAM2 factor graph. `navigation.launch.py` starts it unconditionally, and it owns **both** transforms — `map → odom` *and* `odom → base_footprint` — plus `/odometry/filtered_map`.

Until 2026-05 the stack ran a robot_localization dual EKF (`ekf_map_node` + `ekf_odom_node`) with the factor graph as an opt-in alternative behind a `use_fusion_graph` launch argument. The EKFs, that launch argument, and the earlier slam_toolbox / Kinematic-ICP / FusionCore experiments were all removed; nothing else may publish those two transforms.

What you *can* still toggle, in the GUI's *Settings → Localization* section (then *Restart ROS2*), are the optional factors the graph adds on top of wheels + IMU + GPS + GPS-COG yaw:

- **`use_scan_matching`** — LiDAR scan-matching between consecutive graph nodes.
- **`use_loop_closure`** — loop-closure search against earlier nodes (also needs a persisted graph on disk, so it does nothing on the very first session).
- **`use_magnetometer`** — magnetometer yaw factor, once the compass is calibrated.

Both LiDAR toggles are ANDed with `lidar_enabled`: with no LiDAR there is no `/scan`, so the factors cannot exist. They are worth enabling if your garden has multi-minute RTK-Float windows or GPS-denied corners. See [Architecture](Architecture#optional-factor-graph-localizer-fusion_graph) for the full picture.

### What do the *Save graph* / *Clear graph* buttons in Diagnostics do?

They call the `~/save_graph` / `~/clear_graph` services on `fusion_graph_node`.

- **Save graph** — persists `<graph_save_prefix>.{graph,scans,meta}` to disk (`/ros2_ws/maps/fusion_graph.*` by default). The node also auto-saves on dock arrival, whenever it leaves the RECORDING state, and every 5 min while AUTONOMOUS, so the button is mostly a "checkpoint before I shut down ROS2 manually" affordance.
- **Clear graph** — wipes iSAM2 + accumulated factors + per-node scans. The node stays alive; the next valid pose seed (GPS, set_pose, or scan-match relocalization) re-initializes. Use after relocating the robot to a new garden.

## Development

### How do I test without a real mower?

Use the Webots simulation (`mowgli_simulation`, Webots R2025a). The fastest way:

```bash
cd docker
docker compose -f docker-compose.simulation.yaml up dev-sim
```

There's also an automated E2E test that validates the full mowing cycle:

```bash
docker compose -f docker-compose.simulation.yaml exec dev-sim \
  bash -c "source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/e2e_test.py"
```

See [Simulation](Simulation) for full details.

### Can I develop in the cloud without local setup?

Yes! MowgliNext supports **GitHub Codespaces** with a pre-configured devcontainer. Click **Code → Codespaces** on the repo page to get a full ROS2 Kilted development environment with Nav2, GTSAM, the GUI toolchain and the linters — no local installation needed. 8-core machine recommended. Note that the devcontainer does **not** ship Webots, so it is a build/test environment rather than a simulation one; run the sim through `docker-compose.simulation.yaml` instead. See [Getting Started](Getting-Started#development-with-github-codespaces--devcontainer).

### How do I add support for a different LiDAR?

Create a new directory in `sensors/` with a Dockerfile for your LiDAR driver. It must publish `/scan` (LaserScan). See [Sensors](Sensors#adding-a-new-sensor).

### Can Claude help me with my contribution?

Yes! Mention `@claude` in any issue or PR comment and it will read the codebase, answer questions, and suggest code. The repo also includes Claude Code configuration (CLAUDE.md files) for local AI-assisted development.

Point it at the machine-readable indexes checked into the repo — they save a lot of blind grepping: [`docs/claude/codemaps/`](https://github.com/mowglinext/mowglinext/tree/main/docs/claude/codemaps) (one map per package), [`docs/claude/ros-interfaces.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/ros-interfaces.md) (every topic/service/action and who publishes it), [`docs/claude/parameters.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/parameters.md) (every config key and its consumers), [`docs/claude/testing-ci.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/testing-ci.md), and [`docs/claude/doc-index.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/doc-index.md) (which document is authoritative and which is history). Each top-level directory also has its own `CLAUDE.md`.

See [AI-Assisted Contributing](AI-Assisted-Contributing).
