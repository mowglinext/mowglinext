# MowgliNext Wiki

Welcome to the MowgliNext documentation wiki — the reference hub for the open-source ROS2 autonomous robot mower.

## Quick Links

| Resource | Description |
|----------|-------------|
| [Getting Started](Getting-Started) | First-time setup, DevContainer, and deployment |
| [Architecture](Architecture) | System design, packages, data flow |
| [Configuration](Configuration) | All YAML parameters explained |
| [Deployment](Deployment) | Docker Compose setup and troubleshooting |
| [Simulation](Simulation) | Gazebo Harmonic testing and E2E test |
| [Sensors](Sensors) | GPS and LiDAR driver setup |
| [Firmware](Firmware) | STM32 integration and COBS protocol |
| [Behavior Trees](Behavior-Trees) | BT nodes, tree structure, control flow |
| [GUI](GUI) | Web interface (React + Go) |
| [Contributing](Contributing) | How to contribute to MowgliNext |
| [FAQ](FAQ) | Frequently asked questions |

## Project Links

- **Website:** https://mowgli.garden
- **GitHub:** https://github.com/mowglinext/mowglinext
- **Issues:** https://github.com/mowglinext/mowglinext/issues
- **Discussions:** https://github.com/mowglinext/mowglinext/discussions

## Monorepo Structure

```
mowglinext/
├── ros2/        ROS2 stack (Nav2, robot_localization dual EKF, opt-in fusion_graph (GTSAM iSAM2), BT, coverage planner)
├── docker/      Docker Compose deployment and config
├── sensors/     Dockerized sensor drivers (GPS, LiDAR)
├── gui/         React + Go web interface
├── firmware/    STM32 firmware (motor, IMU, blade)
├── install/     Interactive installer + gh-pages install composer
├── docs/        GitHub Pages landing page + first-boot checklist
└── wiki/        This wiki (auto-synced to the GitHub wiki)
```

## Key Design Decisions

1. **base_link at rear wheel axis** — OpenMower convention.
2. **Two interchangeable map-frame localizers, picked at launch.** `ekf_odom_node` always owns `odom → base_footprint` (wheels + gyro, continuous). For `map → odom`, the default is `ekf_map_node` (robot_localization global EKF, fuses `/gps/pose_cov` + GPS-COG yaw + optional mag yaw under `two_d_mode`). Setting `use_fusion_graph:=true` swaps it for `fusion_graph_node` — a GTSAM iSAM2 factor graph with the same inputs plus optional LiDAR scan-matching and loop-closure factors. No SLAM in either case — the `/map` is built from user-recorded area polygons.
3. **Cyclone DDS** — replaces FastRTPS (stale shm on ARM).
4. **Map frame = GPS frame** — X=east, Y=north, no rotation.
5. **Firmware is blade safety authority** — ROS2 is fire-and-forget.
6. **Collision monitor for avoidance** — costmap obstacles disabled in planner.
7. **Per-area F2C v2 coverage** — `mowgli_coverage` (Fields2Cover 2.0.0) plans one path per area per session from the area polygon minus already-mowed cells (`mow_progress` holes). FTCController follows it; on abort, the BT re-ticks `FollowStrip` on the same path and FTC `setPlan` resyncs to the closest pose. Legacy cell-based strip planner kept as fallback and as the source of `mow_progress` cell stamping.
8. **Emergency auto-reset on dock** — firmware decides whether to clear latch.
9. **Area recording via BT** — drive boundary, Douglas-Peucker simplification, save polygon.
10. **LiDAR feeds the map-frame estimate via fusion_graph** — opt-in via `use_fusion_graph:=true`; scan-matching between-factors and loop-closure factors keep the map-frame pose stable across multi-minute RTK-Float windows.
11. **Dedicated manual mowing mode** — teleop with collision_monitor, GPS, and the active map-frame localizer all running.
