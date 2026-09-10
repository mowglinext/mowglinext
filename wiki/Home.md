# MowgliNext Wiki

Welcome to the MowgliNext documentation wiki — the reference hub for the open-source ROS2 autonomous robot mower.

## Quick Links

| Resource | Description |
|----------|-------------|
| [Getting Started](Getting-Started) | First-time setup, DevContainer, and deployment |
| [User Guide](User-Guide) | Day-to-day operation of the mower |
| [Architecture](Architecture) | System design, packages, data flow |
| [Configuration](Configuration) | All YAML parameters explained |
| [Deployment](Deployment) | Docker Compose setup and troubleshooting |
| [Simulation](Simulation) | Webots simulation and E2E test |
| [Sensors](Sensors) | GPS and LiDAR driver setup |
| [Firmware](Firmware) | STM32 integration and COBS protocol |
| [Behavior Trees](Behavior-Trees) | BT nodes, tree structure, control flow |
| [GUI](GUI) | Web interface (React + Go) |
| [Contributing](Contributing) | How to contribute to MowgliNext |
| [AI-Assisted Contributing](AI-Assisted-Contributing) | Working on MowgliNext with a coding agent |
| [FAQ](FAQ) | Frequently asked questions |

## Project Links

- **Website:** https://mowgli.garden
- **GitHub:** https://github.com/mowglinext/mowglinext
- **Issues:** https://github.com/mowglinext/mowglinext/issues
- **Discussions:** https://github.com/mowglinext/mowglinext/discussions

## Working on the Code

[Contributing](Contributing) covers the workflow; [AI-Assisted Contributing](AI-Assisted-Contributing) covers changes made with a coding agent. The repository also carries a set of terse, code-derived indexes that are useful to read directly — and to hand to an assistant:

- [`CLAUDE.md`](https://github.com/mowglinext/mowglinext/blob/main/CLAUDE.md) at the root (safety, layout, architecture invariants), plus a per-area `CLAUDE.md` in `ros2/`, `gui/`, `firmware/`, `install/`, `docker/` and `sensors/`
- [`docs/claude/codemaps/`](https://github.com/mowglinext/mowglinext/tree/main/docs/claude/codemaps) — one map per package: files, runtime surface, known pitfalls
- [`docs/claude/ros-interfaces.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/ros-interfaces.md) — every topic, service, action and TF, and who publishes it
- [`docs/claude/parameters.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/parameters.md) — every config key, its default and its consumers
- [`docs/claude/testing-ci.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/testing-ci.md) — every test and the CI job that runs it
- [`docs/claude/doc-index.md`](https://github.com/mowglinext/mowglinext/blob/main/docs/claude/doc-index.md) — which document is authoritative and which is historical

## Monorepo Structure

```
mowglinext/
├── ros2/        ROS2 stack (Nav2, fusion_graph localizer (GTSAM iSAM2), BT, Fields2Cover coverage server)
├── docker/      Docker Compose deployment and config
├── sensors/     Dockerized sensor drivers (GPS, LiDAR)
├── gui/         React + Go web interface
├── firmware/    STM32 firmware (motor, IMU, blade)
├── install/     Interactive installer, hardware presets, modular Docker Compose configs
├── docs/        GitHub Pages landing page + install composer + first-boot checklist
└── wiki/        This wiki (auto-synced to the GitHub wiki)
```

## Key Design Decisions

1. **base_link at rear wheel axis** — OpenMower convention.
2. **One localizer: `fusion_graph_node` (GTSAM iSAM2 factor graph).** It is launched unconditionally and owns **both** `map → odom` and `odom → base_footprint`. It fuses raw `/gps/fix` (projected in-node, through an antenna lever-arm factor), wheel odometry, gyro yaw, GPS-COG yaw and optional magnetometer yaw, plus optional LiDAR scan-matching and loop-closure factors. Until 2026-05 this was a robot_localization dual EKF (`ekf_map_node` + `ekf_odom_node`) with the factor graph as an opt-in alternative; both EKFs and the `use_fusion_graph` switch have been removed. No SLAM — the `/map` is built from user-recorded area polygons.
3. **Cyclone DDS** — replaces FastRTPS (stale shm on ARM).
4. **Map frame = GPS frame** — X=east, Y=north, no rotation.
5. **Firmware is blade safety authority** — ROS2 is fire-and-forget.
6. **Collision monitor for avoidance** — costmap obstacles disabled in planner.
7. **Per-area F2C v3 coverage** — `mowgli_coverage` (Fields2Cover 3.0.0) plans one path per area per session from the area polygon and its obstacle holes (`get_mowing_area`). The plan comes back as continuous, hole-free `drivable_subpaths` — headland rings and serpentine swaths already joined by forward turn-around arcs — and `FollowStrip` drives each as one `FollowCoveragePath` goal with `FTCController`, bridging consecutive sub-paths with a blade-off Nav2 transit. Resume is `FollowStrip`'s job: it trims the path at its persisted pose cursor before dispatch (the plan is deterministic, so the cursor stays valid), while FTC `setPlan` always starts tracking at index 0. The `mow_progress` grid is progress bookkeeping for the GUI, not a planner input.
8. **Emergency auto-reset on dock** — firmware decides whether to clear latch.
9. **Area recording via BT** — drive boundary, Douglas-Peucker simplification, save polygon.
10. **LiDAR feeds the map-frame estimate via fusion_graph** — the `use_scan_matching` / `use_loop_closure` factors (both gated on `lidar_enabled`) keep the map-frame pose stable across multi-minute RTK-Float windows.
11. **Dedicated manual mowing mode** — teleop with collision_monitor, GPS, and the fusion_graph localizer all running.
