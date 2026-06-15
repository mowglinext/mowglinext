# MowgliNext

Open-source autonomous robot mower monorepo. ROS2 Kilted, Nav2, and a GTSAM iSAM2 factor-graph localizer (`fusion_graph`, in `ros2/src/fusion_graph/`) that is the **sole and default** map+odom localizer — it owns **both** `map→odom` AND `odom→base_footprint` and adds optional LiDAR scan-matching + loop-closure factors. **The robot_localization dual-EKF (`ekf_map_node` + `ekf_odom_node`), `config/robot_localization.yaml`, slam_toolbox, and Kinematic-ICP were all removed**; `navigation.launch.py` launches `fusion_graph_node` unconditionally (there is no `use_fusion_graph` arg). BehaviorTree.CPP v4, cell-based strip coverage. (`docs/HANDOFF_FUSION_GRAPH.md` is historical — the migration record, **not** the steady-state reference; see [`wiki/Architecture.md`](https://github.com/cedbossneo/mowglinext/wiki/Architecture#optional-factor-graph-localizer-fusion_graph) for current behaviour.) **Drive base:** the wheel-velocity loop runs in STM32 firmware (vendored PX4 PID); ROS2 sends `cmd_vel` and the firmware closes the per-wheel loop.

**Website:** https://mowgli.garden | **Wiki:** https://github.com/cedbossneo/mowglinext/wiki | **First-boot checklist:** [`docs/FIRST_BOOT.md`](docs/FIRST_BOOT.md)

## Safety — READ FIRST

This robot has spinning blades. The STM32 firmware is the sole blade safety authority.

- NEVER bypass firmware blade safety checks from ROS2
- Blade commands from ROS2 are fire-and-forget — firmware decides whether to execute
- Emergency stop is handled by firmware, not software
- Flag ANY change that could affect physical behavior as safety-critical in PR reviews

## Monorepo Layout

| Directory | Language | Build | Description |
|-----------|----------|-------|-------------|
| `ros2/` | C++17, Python | `colcon build` | ROS2 stack: 14 packages (Nav2, robot_localization, fusion_graph, BT, cell-based strip planner, F2C v2 coverage server, hardware bridge) |
| `install/` | Shell | `./mowglinext.sh` | Interactive installer, hardware presets, modular Docker Compose configs |
| `gui/` | Go, TypeScript/React | `go build`, `yarn build` | Web interface for config, map editing, monitoring |
| `docker/` | YAML, Shell | `docker compose` | Manual deployment configs, DDS, service orchestration |
| `sensors/` | Dockerfile | `docker build` | Dockerized sensor drivers (GPS, LiDAR) |
| `firmware/` | C | `pio run` | STM32F103 firmware (motor, IMU, blade, battery) |
| `docs/` | HTML, CSS, JS | GitHub Pages | Landing page + install composer at mowgli.garden |

## Architecture Invariants (DO NOT VIOLATE)

1. **Localizer = `fusion_graph` (GTSAM iSAM2), sole and default.** `fusion_graph_node` is launched unconditionally by `navigation.launch.py` and publishes **both** `map→odom` AND `odom→base_footprint` (replacing the removed `ekf_map_node` + `ekf_odom_node`). There is **no `use_fusion_graph` launch arg** — do not write it from the GUI. It runs a Pose2 graph (`node_period_s` cadence) with a custom `GnssLeverArmFactor` (analytic Jacobian — antenna lever-arm rotates with the node's yaw), a wheel between-factor with non-holonomic covariance (σ_y << σ_x), a gyro between-factor on yaw, and COG/mag yaw unary factors. Inputs it still consumes (unchanged from the old EKF wiring): `/gps/pose_cov` (from `navsat_to_absolute_pose_node`, datum + lever-arm from `mowgli_robot.yaml`), `/imu/cog_heading` (GPS COG yaw, `cog_to_imu_node`), `/imu/mag_yaw` (when calibrated). RTK-Fixed σ ~3 mm flows through as-is. **LiDAR scan-matching between-factors and loop-closure factors are wired**, gated by `use_scan_matching` / `use_loop_closure` (the GUI's LiDAR toggle drives these), letting the map-frame estimate ride through multi-minute RTK-Float windows. Surface: `/fusion_graph/diagnostics` (1 Hz `DiagnosticArray`: `total_nodes / loop_closures / scan_matches_ok|fail / cov_xx|yy|yawyaw`), `/fusion_graph/markers`, `/imu/fg_yaw`, services `~/save_graph` + `~/clear_graph` (`std_srvs/Trigger`, in the GUI's Diagnostics → Fusion Graph panel).

   **slam_toolbox, Kinematic-ICP, and FusionCore are gone.** The previous RTK-fallback architecture (parallel TF tree, `slam_pose_anchor_node` EWMA, `wheel_odom_tf_node`, `slam_scan_frame_relay`, `slam_map_persist_node`, `rtk_pose_mux_node`, fusion as `pose1`), the K-ICP encoder adapter, and the FusionCore 22D UKF were all removed in favor of the factor-graph approach. `fusion_graph` is the only LiDAR-aware localizer; the `kinematic_icp` and `fusioncore` submodules are no longer part of the build.
2. **TF chain follows REP-105** — `map → odom → base_footprint → base_link → sensors`. **Both `map→odom` AND `odom→base_footprint` are published by `fusion_graph_node`** (the `ekf_map_node`/`ekf_odom_node` that used to own them are removed). Do NOT publish either TF from any other node. All Nav2 nodes use `base_footprint` as the robot frame. `base_link` is at the rear wheel axis (OpenMower convention, do not move).
3. **Cyclone DDS** — not FastRTPS (stale shm issues on ARM)
4. **Map frame = GPS frame** — X=east, Y=north, no rotation transform
5. **Costmap obstacles disabled in coverage mode** — collision_monitor handles real-time avoidance
6. **dock_pose + tool_width live in `mowgli_robot.yaml`** — single source of truth. `dock_pose_x/y/yaw`: `calibrate_imu_yaw_node`'s dock pre-phase and `/map_server_node/set_docking_point` (called by the GUI to manually pin the dock to the current robot pose) both write back to `mowgli_robot.yaml` via line-splice updates that preserve comments. `hardware_bridge`, `map_server`, and `dock_yaw_to_set_pose` read the values as ROS parameters. There is no `dock_calibration.yaml` anymore. `tool_width` (effective blade cut width): `navigation.launch.py` flows the same scalar into `map_server.tool_width` (mark_cells_mowed stamp radius + sliver detection) AND `coverage_server.operation_width` (F2C `Robot::setCovWidth`, controls swath spacing). The two used to be configured separately (`mower_width=0.18` + static `operation_width=0.20`), which left thin un-mowed strips between adjacent F2C swaths because map_server's stamp radius was narrower than F2C's spacing.
7. **Multi-area coverage = explicit segments from `plan_coverage` (F2C v3, no turn planning).** `map_server_node` owns area polygons and the `mow_progress` grid layer. BT outer loop `GetNextUnmowedArea` iterates areas; per area, `PlanCoverageArea` calls `map_server_node/get_mowing_area` (outer ring + obstacle holes) and forwards it to `mowgli_coverage`'s `plan_coverage` action (`mowgli_interfaces/action/PlanCoverage`, Fields2Cover v3 at `/opt/fields2cover-300`). The result is an EXPLICIT ordered list of drivable segments — concentric headland rings (outermost first) then straight serpentine swaths — plus a concatenated `full_path` for the GUI. There is NO turn geometry in the plan and NO heuristic re-segmentation: `FollowStrip` dispatches ONE segment per `FollowCoveragePath` goal (RotationShim pivots in place at the segment start, MPPI tracks it), records completion per segment index in `ctx->area_completed_swaths` (plan is deterministic so indices are stable across re-plans), and runs an automatic NavigateToPose transit when the next segment starts >0.6 m away (resume, skipped segment, concave cross-lobe hop). Too-small areas are rejected by the server (`success=false`), not by a BT-side size gate.
8. **RotationShim+MPPI for coverage; RotationShim+RPP for transit (FTC retired).** The `FollowCoveragePath` slot uses `nav2_rotation_shim_controller` wrapping `nav2_mppi_controller::MPPIController`: RotationShim does an in-place pivot to the segment-start heading (the ~180-degree flip between adjacent swaths), then MPPI tracks the SINGLE segment it was handed — a straight swath or a smooth headland ring (`closed_loop: false` is REQUIRED on this deadband chassis). The `FollowPath` (transit) slot uses RotationShim wrapping `nav2_regulated_pure_pursuit_controller` (RPP). **FTCController is gone** — retired from both slots, do not re-introduce it. **Nav2 params are now a shared base + thin overlays:** `nav2_params_base.yaml` (everything common) is deep-merged with `nav2_params_lidar.yaml` OR `nav2_params_no_lidar.yaml` (only the genuine differences — costmap obstacle vs static layers, scan-based vs pass-through collision_monitor, `FollowPath.use_collision_detection`), selected by the `use_lidar` arg in `navigation.launch.py`. Edit shared params in the **base**; variant params in the **overlay**. `test_nav2_params.py` validates the merged result and pins the variants in lockstep.
9. **Emergency auto-reset on dock** — When emergency is active and robot is on dock (charging detected), BT auto-sends `ResetEmergency` to firmware. Firmware is sole safety authority and only clears latch if physical trigger is no longer asserted.
10. **Undock via Nav2 BackUp behavior** — BackUp (1.5m, 0.15 m/s) via `behavior_server`, not `opennav_docking` UndockRobot (isDocked() unreliable with GPS drift near the dock). Costmaps are cleared after undock.
11. **Zero-odom only when charging AND idle** — `hardware_bridge_node` does not reset odometry during undock sequence.
12. **Battery current for dock detection** — Hardware bridge publishes `abs(charging_current)` when charging, `0.0` when not, for `SimpleChargingDock` compatibility.
13. **Docking server cmd_vel** — Remapped to `/cmd_vel_docking` through twist_mux (priority 15).
14. **Coverage grid_map → OccupancyGrid convention** — easy to get wrong. `grid_map::GridMap::getSize()(0)` = cells along X, `getSize()(1)` = cells along Y. grid_map iterates with `r=0 → X_max` (decreasing) and `c=0 → Y_max` (decreasing). `nav_msgs/OccupancyGrid` has `width = X_cells`, `height = Y_cells`, row-major with `data[y_row * width + x_col]`, where `col=0 ↔ origin.x` (X_min) and `row=0 ↔ origin.y` (Y_min). To convert a grid_map cell `(r, c)` to an OccupancyGrid index: `og_col = nx - 1 - r`, `og_row = ny - 1 - c`, `flat_idx = og_row * nx + og_col`. ALWAYS set `mask.info.width = nx` and `mask.info.height = ny` — swapping produces a 90°-rotated mask, which silently marks valid interior polygon cells as lethal and breaks Smac planning with "Start occupied" errors. See `mow_progress_to_occupancy_grid()` in `map_server_node.cpp` as the reference implementation.

## High-Level Commands and States

### HighLevelControl.srv Commands
| Value | Constant | Description |
|-------|----------|-------------|
| 1 | `COMMAND_START` | Begin autonomous mowing |
| 2 | `COMMAND_HOME` | Return to dock |
| 3 | `COMMAND_RECORD_AREA` | Start area boundary recording |
| 4 | `COMMAND_S2` | Mow next area |
| 5 | `COMMAND_RECORD_FINISH` | Finish recording, save polygon |
| 6 | `COMMAND_RECORD_CANCEL` | Cancel recording, discard trajectory |
| 7 | `COMMAND_MANUAL_MOW` | Enter manual mowing mode (teleop + blade) |
| 254 | `COMMAND_RESET_EMERGENCY` | Reset latched emergency |
| 255 | `COMMAND_DELETE_MAPS` | Delete all maps |

### HighLevelStatus.msg States
| Value | Constant | Description |
|-------|----------|-------------|
| 0 | `HIGH_LEVEL_STATE_NULL` | Emergency or transitional |
| 1 | `HIGH_LEVEL_STATE_IDLE` | Idle, docked, charging, returning home |
| 2 | `HIGH_LEVEL_STATE_AUTONOMOUS` | Autonomous mowing (undocking, transit, mowing, recovering) |
| 3 | `HIGH_LEVEL_STATE_RECORDING` | Area recording in progress |
| 4 | `HIGH_LEVEL_STATE_MANUAL_MOWING` | Manual mowing via teleop |

### Area Recording Flow
1. GUI sends `COMMAND_RECORD_AREA` (3) to start recording
2. BT enters `RecordArea` node — records position at 2 Hz, publishes live preview on `~/recording_trajectory`
3. User drives robot along boundary
4. GUI sends `COMMAND_RECORD_FINISH` (5) — trajectory is simplified (Douglas-Peucker) and saved via `/map_server_node/add_area`
5. Or GUI sends `COMMAND_RECORD_CANCEL` (6) — trajectory discarded

### Manual Mowing
- Dedicated BT state with `COMMAND_MANUAL_MOW` (7) — does not hijack recording mode
- Teleop via `/cmd_vel_teleop` (twist_mux priority)
- Blade managed by GUI (fire-and-forget to firmware)
- Collision_monitor, GPS, and the active map-frame localizer all remain active

## Code Style

| Component | Style | Tool |
|-----------|-------|------|
| C++ (ros2/) | 2-space indent, `snake_case` files/params, `CamelCase` classes | `clang-format` (config in `ros2/.clang-format`) |
| Go (gui/) | Standard Go | `gofmt` |
| TypeScript (gui/web/) | Prettier + ESLint | `yarn lint` |
| Python (launch files) | PEP 8 | — |
| YAML (config) | 2-space indent, `snake_case` keys | — |

## Commit Conventions

```
<type>: <description>

Types: feat, fix, refactor, docs, test, chore, perf, ci
```

No Co-Authored-By lines. Keep messages concise and focused on "why".

## ROS2 Specifics

- **Distro:** Kilted
- **DDS:** Cyclone DDS (all containers share `docker/config/cyclonedds.xml`)
- **Topics:** Mowgli-specific topics under `/mowgli/` namespace
- **Frames:** `map` (global, GPS-anchored via fixed datum), `odom` (continuous local, dead-reckoning only — never jumps), `base_footprint` (robot frame for Nav2), `base_link` (rear axle), `lidar_link`, `imu_link`
- **TF chain:** `map→odom` (`ekf_map_node`, 30 Hz — absorbs GPS corrections), `odom→base_footprint` (`ekf_odom_node`, 50 Hz — continuous dead-reckoning), `base_footprint→base_link` (static), `base_link→sensors` (static — `base_link→imu_link` rotation = `imu_yaw/pitch/roll` from `mowgli_robot.yaml`, auto-calibratable via GUI button)
- **Units:** SI throughout (metres, radians, seconds)
- **Sensor fusion:** robot_localization dual EKF under `two_d_mode`. `ekf_odom_node` (25 Hz, wheel + gyro_z, publishes `odom→base_footprint`), `ekf_map_node` (10 Hz, wheel + gyro + `/gps/pose_cov` (`pose0`) + `/slam/pose_cov` (`pose1`) + `/imu/cog_heading` + `/imu/mag_yaw`, publishes `map→odom`). Config in `ros2/src/mowgli_bringup/config/robot_localization.yaml`. Non-holonomic motion enforced by tight `vy` covariance in `/wheel_odom`. Absolute yaw for the map-frame EKF comes from `cog_to_imu_node.cpp` (GPS course-over-ground, gated on BOTH `wheel_omega` AND IMU `gyro_z` for `on_fix` and `republish_latched_when_stationary` — wheel-only gating let multi-degree COG yaw jumps slip through during PRE_ROTATE tight arcs, because the Gazebo diff-drive plugin under-reports `wheel.angular.z` while the IMU still sees real angular motion) and `mag_yaw_publisher.py` (tilt-compensated magnetometer yaw, when magnetometer is calibrated). The `hardware_bridge_node` runs a 20 s IMU bias calibration (`imu_cal_samples: 1000`) every time the robot docks, and logs the implied mounting pitch/roll so the operator can promote any >1° offset into `mowgli_robot.yaml` → `imu_pitch/imu_roll`.
- **Navigation:** `FollowCoveragePath` = RotationShim + MPPI (tracks the continuous CC-Dubins coverage path; PathAlignCritic holds the swath line; `closed_loop: false`). `FollowPath` (transit) = RotationShim + RPP. FTCController is retired from both slots. Earlier the docs said "MPPI jumps between swaths" — that was the per-swath, Euclidean-nearest-point setup; with the CONTINUOUS full-path goal + `max_robot_pose_search_dist: 1.5` (arc-length-bounded) MPPI tracks swaths cleanly.
- **Coverage:** Two-package pipeline. `map_server_node` (`mowgli_map`) owns area polygons + the `mow_progress` grid layer. `coverage_server` (`mowgli_coverage`, action `plan_coverage` = `mowgli_interfaces/action/PlanCoverage`) runs a deliberately minimal F2C v3 pipeline in `planBoustrophedon()` (`coverage_planning.cpp`): `ConstHL::generateHeadlands(chassis_safety_inset)` pre-inset; `ConstHL::generateHeadlandSwaths(op_width, n_rings, out2in)` concentric mowed rings (n = `num_headland_passes` or ceil(headland/op_width)); `ConstHL::generateHeadlands(n_rings*op_width)` mainland; `BruteForce` swaths (fixed or auto angle; each disjoint clip of a sweep line is its OWN swath, so concave fields and holes need NO decomposition); `BoustrophedonOrder` serpentine. NO `pp::PathPlanning`, NO turn planners, NO OR-Tools genRoute, NO TrapezoidalDecomp, NO boundary clipping (the plan is built from insets; an out-of-bounds pose is logged as a planner bug). Result = explicit per-segment `nav_msgs/Path[]` + types (ring|swath). One `PlanCoverageArea` per area per session; `FollowStrip` resumes by segment index.
- **Coverage goal-checker:** `coverage_goal_checker` is `mowgli_nav2_plugins/PathProgressGoalChecker` (NOT `StoppedGoalChecker` — that one fires on velocity stoppage and matches mid-traversal during FTC's PRE_ROTATE pivots, which would complete the action at <2% coverage). It subscribes to `<plugin_name>/global_plan` (= `/controller_server/FollowCoveragePath/global_plan`) and only fires when the robot has monotonically tracked >= `progress_threshold` (0.95) of path poses AND is within xy/yaw tolerance of the goal pose. Includes a `fallback_timeout_s` watchdog (5 s) for missing global_plan and an `n<=1` guard for degenerate paths.
- **Area Recording:** `RecordArea` BT node records trajectory at 2 Hz, Douglas-Peucker simplification, saves polygon via `/map_server_node/add_area`. Live preview on `~/recording_trajectory`.
- **Manual Mowing:** Dedicated BT state (COMMAND_MANUAL_MOW=7). Teleop via `/cmd_vel_teleop`, blade managed by GUI. Collision_monitor, GPS, robot_localization remain active.
- **Emergency Auto-Reset:** BT auto-resets emergency when robot placed on dock (charging detected). Firmware is safety authority.
- **GPS fusion:** `navsat_to_absolute_pose_node` reads `/gps/fix` (NavSatFix) and emits `/gps/pose_cov` (PoseWithCovarianceStamped in `map` frame) with covariance derived from `position_accuracy` and a lever-arm correction applied via the current EKF yaw. `ekf_map_node` fuses `/gps/pose_cov` as `pose0`. `/gps/absolute_pose` remains exposed for the GUI and BT. With RTK-Fixed (σ ~3 mm) and frequent updates, the EKF converges to fix precision without special-case outlier gating.
- **fusion_graph (opt-in replacement for `ekf_map_node`):** Set `use_fusion_graph:=true` on `navigation.launch.py` (or persistently in `mowgli_robot.yaml`, exposed in the GUI's *Settings → Localization* section) to replace `ekf_map_node` with `fusion_graph_node` (GTSAM iSAM2). Pose2 graph at 10 Hz; one node per `node_period_s`, with a wheel between-factor (non-holonomic σ_y << σ_x), a gyro between-factor on yaw, a custom `GnssLeverArmFactor` (analytic Jacobian — antenna lever-arm rotates with the node's yaw, so it couples to heading correctly), and unary yaw factors for COG and tilt-compensated mag. **LiDAR scan-matching between-factors and loop-closure factors are wired** (gated by `use_scan_matching` / `use_loop_closure`) — they let the map-frame estimate carry through multi-minute RTK-Float windows, which is why this design exists. Reads datum + lever-arm from `mowgli_robot.yaml` via `fusion_graph.launch.py`. Persists `<graph_save_prefix>.{graph,scans,meta}` (default `/ros2_ws/maps/fusion_graph.*`); auto-saves on RECORDING→IDLE, dock arrival, and every `periodic_save_period_s` (5 min) in AUTONOMOUS state. Surface: `/fusion_graph/diagnostics`, `/fusion_graph/markers`, `/imu/fg_yaw`, services `~/save_graph` + `~/clear_graph` (both `Trigger`).
- **IMU mounting calibration:** `base_link→imu_link` rotation (imu_roll, imu_pitch, imu_yaw in mowgli_robot.yaml) is critical — if wrong, gravity-removal leaks into pitch and yaw integration degrades. Use the GUI's "Auto-calibrate" button next to IMU Yaw — the robot drives itself ~0.6 m forward then back and solves `imu_yaw = atan2(-ay_chip, ax_chip)` from accel direction vs wheel-derived `a_body`.
- **Nav2 tuning:** Global costmap 30m x 30m rolling window; keepout_filter disabled in global costmap (blocks transit/docking); collision_monitor in coverage mode uses **PolygonSlow only** (`polygons: ["PolygonSlow"]` — PolygonStop kept firing whenever the chassis grazed a costmap obstacle, freezing cmd_vel; obstacle avoidance during coverage is MPPI's CostCritic over the LiDAR `obstacle_layer`, with collision_monitor's PolygonSlow as the soft-slowdown fallback); source_timeout 5.0s (ARM TF jitter); PoseProgressChecker `required_movement_radius: 0.15` + `required_movement_angle: 0.5 rad` (counts rotation as progress so headland pivots don't trip "no progress"), `movement_time_allowance: 30s` default (operator-tunable via `progress_timeout_sec`); failure_tolerance 1.0; speeds default `transit 0.5 / mowing 0.5 m/s` (per-site overrides in `mowgli_robot.yaml`).
- **Joystick:** Foxglove client passes `schemaName` in `clientAdvertise` for JSON-to-CDR conversion. GUI shows joystick during "RECORDING" state (not just "AREA_RECORDING").

See sections below for detailed package descriptions, topics, and architecture.

## Git Workflow

**NEVER commit directly to main.** Always use feature branches and PRs:
```bash
git checkout main && git pull
git checkout -b feat/my-feature    # or fix/, refactor/, test/, chore/, docs/
# ... make changes ...
git add <files> && git commit -m "feat: description"
gh pr create --title "feat: my feature" --body "..."
```

### Dev Branch Workflow

Docker builds trigger on both `main` and `dev` branches. Images are tagged `:main` and `:dev` respectively. Use `mowgli-dev` / `mowgli-main` commands to switch between environments. Iterate on `dev`, merge to `main` when stable.

## Quick Commands

All ROS2 commands assume you are inside the devcontainer.

```bash
# Build ROS2 workspace
cd ros2 && make build

# Build a single package
cd ros2 && make build-pkg PKG=mowgli_behavior

# Run headless simulation
cd ros2 && make sim

# Run E2E tests (simulation must be running in another terminal)
cd ros2 && make e2e-test

# Format C++ code
cd ros2 && make format

# Run unit tests
cd ros2 && make test

# Build firmware
cd firmware/stm32/ros_usbnode && pio run

# GUI development
cd gui && go build -o openmower-gui && cd web && yarn dev

# --- Code generation (run after changing .msg/.srv files) ---

# Regenerate firmware rosserial C++ headers from ROS2 .msg files
python3 firmware/scripts/sync_ros_lib.py          # writes to firmware/stm32/ros_usbnode/src/ros/ros_lib/mower_msgs/
python3 firmware/scripts/sync_ros_lib.py --check   # diff-only, no writes (CI)

# Regenerate Go message/service structs from ROS2 .msg/.srv files
cd gui && ./generate_go_msgs.sh                    # writes to gui/pkg/msgs/*/types_generated.go

# Regenerate TypeScript ROS types (snake_case fields matching rosbridge JSON)
cd gui && ./generate_ts_types.sh                   # writes to gui/web/src/types/ros.ts
```

### Code Generation Workflow

When you modify `ros2/src/mowgli_interfaces/msg/*.msg` or `srv/*.srv`:
1. **Firmware headers:** `python3 firmware/scripts/sync_ros_lib.py` — regenerates rosserial C++ headers
2. **Go types:** `cd gui && ./generate_go_msgs.sh` — regenerates Go structs with JSON tags for rosbridge
3. **TypeScript types:** `cd gui && ./generate_ts_types.sh` — regenerates `gui/web/src/types/ros.ts` with snake_case fields matching rosbridge JSON
4. **Protocol constants:** Update `HL_MODE_*` defines in `firmware/stm32/ros_usbnode/include/mowgli_protocol.h` AND `ros2/src/mowgli_hardware/firmware/mowgli_protocol.h` (these are manually maintained — keep in sync with `HighLevelStatus.msg`)

Do NOT hand-edit `*_generated.go`, `ros_lib/mower_msgs/*.h`, or `gui/web/src/types/ros.ts` — re-run the scripts instead.

## Mowing Session Monitoring

**Whenever a mowing test is run (COMMAND_START, undock, autonomous motion, or any tuning session that involves the robot moving), also run the session monitor in parallel.** Output is a JSONL timeline that can be diffed/plotted across sessions to see how tuning changes affect behavior.

```bash
# Detached background from the host (writes to /ros2_ws/logs/mow_sessions/
# inside the container, which is not mounted — better to bind-mount docker/logs/
# or redirect via --output-dir):
docker exec -d mowgli-ros2 bash -c '
  source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && \
  python3 /ros2_ws/scripts/mow_session_monitor.py \
    --session 2026-04-29-fusion-graph-tuning-v1 \
    --output-dir /ros2_ws/maps'

# Interactively from inside the container (Ctrl-C to stop + write summary):
docker exec -it mowgli-ros2 bash -c '
  source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && \
  python3 /ros2_ws/scripts/mow_session_monitor.py --session <name> \
    --output-dir /ros2_ws/maps'
```

The `--output-dir /ros2_ws/maps` redirects to the bind-mounted `install_mowgli_maps` Docker volume so logs persist outside the container. Or bind-mount `docker/logs/mow_sessions/` explicitly in compose for a host-visible path.

**What it records** (per-sample, 10 Hz default):
- Fused pose + twist from `/odometry/filtered_map` (x/y/z, yaw, vx/vy/wz) **+ position covariance (cov_xx, cov_yy, derived sigma_xy_m)**
- TF snapshots: `map→base_footprint` (composed through `map→odom→base_footprint`), `odom→base_footprint` (local EKF)
- Wheel twist + covariance + integrated distance and yaw
- IMU gyro + accel + integrated gyro yaw
- GPS NavSatFix (lat/lon/alt/status/covariance) + `/gps/absolute_pose` ENU
- Dock heading (`/gnss/heading` while charging)
- BT state (state_name, current_area, current_strip), hardware mode, emergency flags, battery
- `cmd_vel_nav` (Nav2 output) + `cmd_vel` (post-safety, what reaches motors)
- Nav2 `/plan` length, next pose, goal pose, distance-to-goal
- LiDAR scan health (valid point count, min range)
- Fusion graph stats (when `use_fusion_graph:=true`): pulled from `/fusion_graph/diagnostics` (node count, scan-match success rate, loop closures, marginal cov)
- **Cross-source consistency**: `fusion ↔ gps` distance, `wheel ↔ gyro` yaw drift, and (when `use_fusion_graph:=true`) `fusion ↔ scan-match` ICP success rate from `/fusion_graph/diagnostics`
- **RTK covariance-drop health**: on every RTK-Fixed GPS arrival, confirm `/odometry/filtered_map` cov drops to σ≤~3 cm within 300 ms — surfaced as `cross_checks.rtk_cov_check.{arrivals,ok,violations}` per sample and rolled into a `rtk_cov_check.verdict` ("healthy" / "intermittent" / "gate_rejecting" / "no_rtk") in the summary.

**Metadata header** (first line of the JSONL): session name, UTC timestamp, git branch + commit + dirty flag, docker image tags from `.env`, SHA-256 truncated hashes of `mowgli_robot.yaml`, `fusion_graph.yaml`, and the Nav2 params (`nav2_params_base.yaml` + `nav2_params_lidar.yaml` + `nav2_params_no_lidar.yaml`) — so sessions from different tunings are grouped/comparable.

**Summary record** (last line, written on Ctrl-C or clean shutdown): total duration, samples written, wheel-integrated distance, straight-line displacement, peak `fusion↔gps` error, peak `wheel↔gyro` yaw drift, RTK cov-check totals + verdict, final BT state.

**Log directory:** `docker/logs/mow_sessions/<session_name>.jsonl`. Commit notable sessions (golden runs, failure cases) so they survive in git history.

## Recommended Skills and Agents

When using Claude Code on this project:

### Skills to Use
- `/ros2-engineering` — ROS2 node patterns, QoS, launch files, Nav2 (use for any ros2/ work)
- `/cpp-coding-standards` — C++ Core Guidelines (use for C++ reviews)
- `/docker-patterns` — Dockerfile and compose patterns (use for docker/ and sensors/ work)
- `/tdd` — Test-driven development (use when adding new features)

### Agents to Invoke
- **code-reviewer** — after any code changes
- **cpp-reviewer** — after C++ changes in ros2/
- **security-reviewer** — before commits touching auth, configs, or firmware commands
- **build-error-resolver** — when colcon or Docker builds fail
- **tdd-guide** — when implementing new features
- **architect** — for design decisions spanning multiple packages

## What NOT to Do

- Do NOT add ROS1 patterns (rosserial, roscore, catkin) — this is ROS2 only
- Do NOT use FastRTPS — Cyclone DDS is required
- Do NOT mock the database/firmware in integration tests — use real interfaces
- Do NOT publish a `map→odom` TF from any node other than the active map-frame localizer. `map→odom` is owned by `ekf_map_node` (default) or `fusion_graph_node` (when `use_fusion_graph:=true`) — never both. The two are mutually exclusive in `navigation.launch.py`.
- Do NOT re-introduce slam_toolbox, Kinematic-ICP, or FusionCore. All three were tried and removed: slam_toolbox required a parallel TF tree + EWMA anchor + gating that was fragile across RTK-Float windows; K-ICP only gave a body-frame twist with no absolute-pose information; FusionCore's 22D UKF would not collapse σ_xy below ~30 cm even under RTK-Fixed. The factor-graph design (`fusion_graph`) is the replacement — LiDAR enters the same graph as scan-matching between/loop-closure factors (see `wiki/Architecture.md` § *Optional: Factor-Graph Localizer*).
- Do NOT send blade commands without firmware safety checks
- Do NOT hardcode GPS coordinates, dock poses, or NTRIP credentials
- Do NOT use RPP for coverage paths, and do NOT re-introduce FTCController — the coverage slot is RotationShim + MPPI fed ONE segment at a time (`closed_loop: false`)
- Do NOT re-introduce F2C turn planning (Dubins/CC-Dubins/Reeds-Shepp) or heading-jump path re-segmentation into coverage. Every variant was field-tested and failed: forward-only turns make Omega-loops at 0.16 m spacing, Reeds-Shepp cusps are untrackable (MPPI never reverses), and segmentSwaths' 0.6 rad heading-jump detector silently never fires on smooth densified arcs (one 3982-pose 'swath' — 2026-06-12). The coverage server returns EXPLICIT segments; turns are RotationShim in-place pivots.
- Do NOT use `base_link` as robot_base_frame in Nav2/robot_localization — use `base_footprint` (REP-105)
- Do NOT use opennav_docking UndockRobot — use Nav2 BackUp behavior (isDocked() unreliable with GPS drift)
- Do NOT refer to `mower_width` — the param is `tool_width`, lives in `mowgli_robot.yaml`, and is the single source for both `map_server.tool_width` and `coverage_server.operation_width` (injected at launch). Reintroducing a separate `mower_width` will re-open the swath-spacing-vs-stamp-radius gap that caused 54 % coverage in 2026-05-12.
- Do NOT use `StoppedGoalChecker` for the `coverage_goal_checker` slot — use `mowgli_nav2_plugins/PathProgressGoalChecker`. StoppedGoalChecker fires on velocity stoppage and matches FTC's PRE_ROTATE pivots mid-traversal, completing the coverage action at <2 % coverage.
- Do NOT use the upstream `opennav_coverage` server. The repo carries it as a git submodule for the `opennav_coverage_msgs` action definitions only — every other subpackage (`opennav_coverage`, `_bt`, `_demo`, `_navigator`, `opennav_row_coverage`) is `COLCON_IGNORE`'d. The upstream server is pinned to F2C 1.2.1 (no `TrapezoidalDecomp`, no `generateHeadlandSwaths`, no `planPathForConnection`). Use `mowgli_coverage` (F2C 2.0.0).
- Do NOT call `buildHeadlandRingPath()` from the BT side — the coverage server owns headland traversal (`generateHeadlandSwaths()` ring segments in `planBoustrophedon`).
- Do NOT reintroduce the hardcoded sim test polygon into `ros2/src/mowgli_map/config/map_server.yaml`. A non-empty default makes a fresh real-robot install try to mow a phantom polygon at the GPS datum the moment `COMMAND_START` fires. The sim injects its polygon via `sim_full_system.launch.py` parameter override.
- Do NOT add a `locks:` block to `twist_mux.yaml` without first wiring a publisher in `hardware_bridge_node`. The earlier `/emergency_stop` lock was listening on a topic no node ever published; the real e-stop path is the `/hardware_bridge/emergency_stop` service which sets the firmware latch.
