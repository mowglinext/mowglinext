# Codemap: ros2_workspace_tooling

> Workspace-level tooling around the ROS2 stack: `ros2/Makefile` + `ros2/scripts/` (link / build / test / sim / format / deploy helpers, field-calibration and diagnostic scripts, the config-drift CI gate, the mow-session recorder), the multi-stage production `ros2/Dockerfile` (plus the orphaned `Dockerfile.dev`), the bare-metal `systemd` unit, the Foxglove layout, the two sim E2E harnesses under `ros2/src/`, and the two git submodules (`opennav_coverage`, `universal-gnss`). It owns HOW the workspace is linked, built, imaged and exercised — not the ROS nodes themselves. Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed. Loaded on demand from `ros2/CLAUDE.md`.

## Where to look
| Task | Start here |
|------|------------|
| Build the workspace (devcontainer) | `ros2/Makefile` `build-full` / `build-pkg PKG=x` / `build-dev` → `ros2/scripts/build.sh` (env `BUILD_TYPE`, `PACKAGES`, `PACKAGES_MODE=up-to\|select`, L51-83) |
| Control which package roots colcon sees | `ros2/scripts/sync_workspace_packages.sh` — globs `ros2/src/mowgli_*/` (L129), links `fusion_graph` (L175-177), `tools/motor` as `mowgli_tools` (L179-180), `opennav_coverage_msgs`, `universal_gnss_ros2`; `--print-base-paths` feeds build/test |
| Run unit tests | `make test` → `ros2/scripts/test.sh` (`PACKAGES` env; requires `/ros2_ws/install/setup.bash`) |
| Run headless Webots sim / E2E | `ros2/Makefile` `sim` (L81), `e2e-test` (L91), `e2e-test-no-lidar` (L111); harnesses `ros2/src/e2e_test.py`, `ros2/src/e2e_test_no_lidar.py` |
| Sim will not start ("Failed to find a free participant", Webots IPC socket) | `ros2/scripts/sim-stop.sh` — SIGINT `ros2 launch`, kills Webots + node stragglers, wipes `/dev/shm/cyclone*`, `/tmp/webots/*` (L60-64) |
| Change the production image | `ros2/Dockerfile` stage table below; `.github/workflows/ros2-docker.yml` builds target `runtime` for amd64+arm64 |
| Bump GTSAM / Fields2Cover / ublox_dgnss pins | `ros2/Dockerfile` L13-129 (GTSAM 4.3a1, F2C v2.0.0 + v3 @ `884d895`) and L142-168 (`UBLOX_DGNSS_REF/SHA`) **and** the mirrored recipes in `.github/workflows/ros2-ci.yml` (GTSAM + F2C cache keys) |
| Add a new ROS package to the image | `ros2/Dockerfile` deps-stage `COPY … package.xml/CMakeLists.txt` list (L297-340); the sync script picks up `mowgli_*` automatically, non-`mowgli_*` names need an explicit link (see `fusion_graph`, L175-177) |
| Format C++ | `make format` / `ros2/scripts/format.sh [--check]` (clang-format 18, style `ros2/.clang-format`, skips `opennav_coverage`); CI = `git-clang-format-18` on changed lines vs `origin/main` |
| Lint C++ | `make lint` (cppcheck `--enable=all` + cpplint with `ros2/CPPLINT.cfg`); CI cppcheck is report-only on changed files |
| Config drift / padded defaults CI failure | `ros2/scripts/check_config_drift.py` (`STRUCTURAL` L72, `USER_OVERRIDE` L90, `CALIBRATION_OUTPUT` L132, `INSTALL_SEED` L145) + `ros2/scripts/test_check_config_drift.py` |
| Record a mowing / tuning session | `ros2/scripts/mow_session_monitor.py --session NAME [--output-dir DIR --rate HZ]` (copied to `/ros2_ws/scripts/` in the runtime image, Dockerfile L471); usage in `docs/claude/session-monitoring.md` |
| Read a session JSONL | `ros2/scripts/diagnostics/analyze_session.py`, `ros2/scripts/diagnostics/session_timeline.py` |
| Characterise drivetrain on the real robot | `ros2/scripts/diagnostics/rotate_at_rate.py WZ DUR`, `undock_and_rotate_90.py`, `forward_5m.py` (all enter BT `RECORDING` via `/behavior_tree_node/high_level_control`, drive `/cmd_vel_teleop`) |
| Wheel / yaw calibration numbers | `ros2/scripts/wheel_radius_cal.py` (wheel vs raw RTK ENU), `ros2/scripts/wheel_yaw_cal.py` (wheel vs gyro 360°), `ros2/scripts/forward_1m_yaw_test.py`, `ros2/scripts/gps_log.py` |
| Derive Nav2 params from chassis physics | `ros2/scripts/compute_nav2_params.py --report\|--yaml\|--compare [--profile calm\|responsive\|both]` + design note `ros2/scripts/compute_nav2_params.md` |
| Live robot health snapshot from the host | `ros2/scripts/robot_monitor.sh [--loop N]` (docker-exec into `mowgli-ros2`) |
| ASCII coverage snapshot | `ros2/scripts/coverage_ascii_render.py [--rows N --timeout S]` |
| GNSS receiver hardware probes | `ros2/scripts/f9p_set_nav_prio.py`, `serial_latency_probe.py` (u-blox F9P over `/dev/ttyACM0`), `unicore_signalgroup_benchmark.py --host … ` (UM982 over SSH) |
| Foxglove panels for the sim | `ros2/foxglove/mowgli_sim.json` (copied to `/ros2_ws/foxglove/` in the simulation stage, Dockerfile L552) |
| Bare-metal (non-Docker) robot deploy | `ros2/systemd/mowgli.service` + `make deploy` / `make backup-maps` (`ROBOT_HOST`, `ROBOT_USER`) |
| Submodule pins | `.gitmodules` (`universal-gnss` → `mowglinext` fork, branch `main`; `opennav_coverage` → upstream `main`) |
| Container startup env | `ros2/scripts/ros2_entrypoint.sh` (sources kilted + `/opt/ublox_msgs` + `/ros2_ws/install`) |

## Files
| File | Lines | Purpose |
|------|-------|---------|
| **ros2/ top level** | | |
| `ros2/README.md` | 912 | Stack reference for the ROS2 workspace (architecture, packages, TF tree, topic/service table, build + Docker + launch docs) |
| `ros2/Makefile` | 179 | Devcontainer entry: build/test/sim/e2e/docker/lint/format/deploy targets; `DEV_PACKAGES` default L9 |
| `ros2/Dockerfile` | 559 | Production multi-stage image (10 stages, see Runtime surface); build context = **repo root** |
| `ros2/Dockerfile.dev` | 173 | Orphaned dev image (header says Jazzy/Gazebo, installs `ros-kilted-slam-toolbox` L47 + `ros-kilted-ros-gz-sim` L71); context = `ros2/`; no compose/CI consumer |
| `ros2/CPPLINT.cfg` | 3 | cpplint filters (`-whitespace/newline,-runtime/string,-build/namespaces,-build/include_order`), linelength 100 |
| `.gitmodules` | 15 | Two submodule pins + the issue #395 fork rationale comment |
| `ros2/systemd/mowgli.service` | 54 | Bare-metal unit: `User=pi`, `WorkingDirectory=/opt/mowgli_ros2`, `ExecStart=/opt/mowgli_ros2/scripts/ros2_entrypoint.sh ros2 launch mowgli_bringup mowgli.launch.py`, `ROS_DOMAIN_ID=42`, `KillSignal=SIGINT` |
| `ros2/foxglove/mowgli_sim.json` | 505 | Foxglove Studio layout (3D + plots + CallService/Publish panels) |
| **ros2/src/ (harness scripts, not packages)** | | |
| `ros2/src/e2e_test.py` | 1648 | LiDAR sim E2E: undock→plan→mow→dock + manual-mow, area-recording, emergency-reset feature tests; 13 PASS/FAIL criteria |
| `ros2/src/e2e_test_no_lidar.py` | 505 | GPS-only sim E2E: waits for `map→base_link` TF, START, median path deviation < 10 cm gate; exit code 0/1 |
| `ros2/src/precision_monitor.py` | 309 | Publishes `/precision/*` Float64 metrics at 2 Hz for Foxglove plots (subscribes `/gps/pose_sim`, which nothing publishes) |
| **ros2/scripts/ — workspace helpers (bash)** | | |
| `ros2/scripts/build.sh` | 98 | `colcon build --symlink-install` over `sync_workspace_packages.sh --print-base-paths`; cwd fixed to `/ros2_ws` |
| `ros2/scripts/test.sh` | 80 | `colcon test --return-code-on-test-failure` + `colcon test-result --verbose` |
| `ros2/scripts/sync_workspace_packages.sh` | 201 | Symlinks package roots into `/ros2_ws/src`; `INCLUDE_OPENNAV_COVERAGE_STACK=1`, `UNIVERSAL_GNSS_PATH`, `MONOREPO_ROOT`, `WORKSPACE_ROOT` env |
| `ros2/scripts/ros2_entrypoint.sh` | 35 | Image ENTRYPOINT; hardcodes `/opt/ros/kilted/setup.bash` (L15) |
| `ros2/scripts/start_vnc.sh` | 64 | TigerVNC `:1` + noVNC `6080` + `sim_full_system.launch.py headless:=false`; compose `simulation-gui` command |
| `ros2/scripts/start_dev_sim.sh` | 79 | `Dockerfile.dev`-only entry: first-run `build.sh`, VNC, `LAUNCH_FILE`/`LAUNCH_ARGS` env; echoes non-existent `make dev-*` targets (L63-66) |
| `ros2/scripts/sim-stop.sh` | 66 | Graceful-then-SIGKILL stop of `ros2 launch`, Webots, listed node executables; DDS shm + Webots IPC cleanup |
| `ros2/scripts/format.sh` | 40 | clang-format wrapper, `REQUIRED_MAJOR=18` (L12), excludes `*/opennav_coverage/*` (L27) |
| `ros2/scripts/fix-copyright.sh` | 72 | Prepends the GPL header to staged `.cpp/.hpp/.h/.py` lacking one (pre-commit helper; not wired in `ros2/.pre-commit-config.yaml`) |
| `ros2/scripts/robot_monitor.sh` | 251 | Host-side health dump via `docker exec mowgli-ros2` (containers, `high_level_status`, `/gps/fix`, TF chain, coverage service, sensor topics, error log grep) |
| **ros2/scripts/ — CI gates & config tooling (python)** | | |
| `ros2/scripts/check_config_drift.py` | 271 | CI gate for CLAUDE.md Invariant 15: template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` (L65) vs sparse `install/config/mowgli/mowgli_robot.yaml` (L66); exit 1 on structural drift / no-default / padded default |
| `ros2/scripts/test_check_config_drift.py` | 174 | pytest: 15 tests incl. `test_committed_repo_configs_are_clean` (real yaml pair) |
| `ros2/scripts/compute_nav2_params.py` | 1125 | Physics-derived Nav2/FTC/costmap/collision-monitor/coverage params; imports `deep_merge` from `ros2/src/mowgli_bringup/launch/robot_config_util.py` (L95); read-only |
| `ros2/scripts/compute_nav2_params.md` | 172 | Design note + per-model motor spec table (`MODEL_SPECS`) behind the script |
| **ros2/scripts/ — session recording & analysis** | | |
| `ros2/scripts/mow_session_monitor.py` | 1150 | Node `mow_session_monitor`: 10 Hz JSONL timeline (metadata header, per-sample state, summary); RTK cov-drop check (`RTK_FIXED_GPS_COV_THRESHOLD`, `FUSION_COV_TARGET`, `RTK_COV_WINDOW_SEC` L134-141); default `--output-dir /home/ubuntu/mowglinext/docker/logs/mow_sessions` (L1114) |
| `ros2/scripts/diagnostics/README.md` | 73 | How to `docker cp` + run the diagnostic scripts, typical monitor+motion+analyze workflow |
| `ros2/scripts/diagnostics/analyze_session.py` | 67 | One-shot JSONL summary (pose/yaw ranges, σ_xy trend, wheel/gyro drift, RTK counters) |
| `ros2/scripts/diagnostics/session_timeline.py` | 19 | Every-100th-sample fusion pose / σ_xy / RTK-check snapshots (5 s at the README's `--rate 20`; 10 s at the monitor's 10 Hz default) |
| `ros2/scripts/diagnostics/forward_5m.py` | 172 | Node `fwd_5m`: 5 m straight drive, fusion vs wheel vs gyro yaw |
| `ros2/scripts/diagnostics/rotate_at_rate.py` | 135 | Node `diag_rotate_param`: `WZ DURATION` rotation, wheel+gyro integrated yaw vs expected |
| `ros2/scripts/diagnostics/undock_and_rotate_90.py` | 175 | Node `undock_and_rotate`: 1.5 m back-up @0.15 m/s then 90° @0.30 rad/s |
| **ros2/scripts/ — field calibration probes** | | |
| `ros2/scripts/wheel_radius_cal.py` | 215 | 1 m fwd + 1 m rev, wheel distance vs raw `/gps/fix` ENU (needs RTK-Fixed) |
| `ros2/scripts/wheel_yaw_cal.py` | 127 | Node `wheel_yaw_cal`: spin to 360° gyro, prints wheel/gyro ratio |
| `ros2/scripts/forward_1m_yaw_test.py` | 282 | 1 m @0.15 m/s logging fusion / gyro / wheel / GPS COG yaw per second |
| `ros2/scripts/gps_log.py` | 132 | Node `gps_log`: every `/gps/fix` during fwd/rev 1 m in ENU + stamps |
| `ros2/scripts/coverage_ascii_render.py` | 171 | Node `coverage_ascii_render`: one-shot ASCII grid of `/map_server_node/coverage_cells` + robot pose |
| **ros2/scripts/ — GNSS hardware (no ROS)** | | |
| `ros2/scripts/f9p_set_nav_prio.py` | 105 | UBX-CFG-VALSET `CFG_RATE_NAV_PRIO=7` to RAM+BBR+Flash via pyserial (stop `mowgli-gps` first) |
| `ros2/scripts/serial_latency_probe.py` | 139 | Reads UBX NAV-PVT from `/dev/ttyACM0` for 25 s, measures stamp − iTOW latency |
| `ros2/scripts/unicore_signalgroup_benchmark.py` | 1043 | SSH-driven UM982 `SIGNALGROUP` A/B benchmark, CSV/JSON summary, restore script |
| **ros2/scripts/ — stale duplicates** | | |
| `ros2/scripts/e2e_test.py` | 690 | Older copy of the E2E harness (subscribes `/mowgli/coverage/path`, `/pose`; spawns a Gazebo cylinder; 900 s timeout); not referenced by Makefile/compose |
| `ros2/scripts/precision_monitor.py` | 317 | Older copy of `ros2/src/precision_monitor.py` (`/precision/slam_map_known_pct`, `TwistStamped` cmd_vel) |
| **Adjacent config (ros2/ top level, not in this area)** | | |
| `ros2/.clang-format`, `ros2/.pre-commit-config.yaml`, `ros2/.dockerignore`, `ros2/.gitignore` | — | Style file (Google-based, Allman, 100 cols), pre-commit hooks (clang-format v18.1.8, ruff, cmakelint, detect-secrets), ignore lists |

## Runtime surface

### Make targets (`ros2/Makefile`, run from `ros2/` inside the devcontainer)
| Target | Does |
|--------|------|
| `build` / `build-full` | `./scripts/build.sh` (Release), whole linked workspace |
| `build-dev` | `PACKAGES="$(DEV_PACKAGES)"` = `mowgli_interfaces mowgli_localization universal_gnss_ros2 mowgli_bringup` |
| `build-pkg PKG=x` / `build-debug` | single package (`--packages-up-to` unless `PACKAGES_MODE=select`) / `BUILD_TYPE=Debug` |
| `test` / `clean` | `./scripts/test.sh` / `rm -rf build/ install/ log/` |
| `sim-stop` | `scripts/sim-stop.sh` |
| `sim` | `sim-stop` then `ros2 launch mowgli_bringup sim_full_system.launch.py headless:=true use_rviz:=false` with `DISPLAY=:99` |
| `e2e-test` | `sim-stop build` → launch sim in background → `sleep 90` → `python3 src/e2e_test.py` → kill + `sim-stop`, exits with the test's code |
| `e2e-test-no-lidar` | same with `use_lidar:=false simulate_gps_degradation:=false` and `src/e2e_test_no_lidar.py` |
| `docker` / `docker-sim` | `docker build --target runtime\|simulation -t mowgli-ros2[-sim]:latest .` (context `ros2/` — **wrong context**, see Pitfalls) |
| `lint` / `format` / `format-check` | cppcheck + cpplint / `clang-format -i` / `--dry-run --Werror` over `src/` (**includes** `opennav_coverage`, unlike `format.sh`) |
| `deploy` | `rsync install/ pi@mowgli.local:/opt/mowgli_ros2/install/` + `sudo systemctl restart mowgli` |
| `backup-maps` | `scp …:/opt/mowgli_ros2/maps/* maps_backup/<ts>/` |

### Docker image stages (`ros2/Dockerfile`, context = repo root; compose files use `context: ..`, CI uses `context: .`)
| Stage | Line | From | Contents |
|-------|------|------|----------|
| `gtsam-builder` | 13 | `ros:kilted-ros-base` | GTSAM 4.3a1 from source → `/opt/gtsam` |
| `fields2cover-builder` | 50 | ros base | F2C v2.0.0 → `/opt/fields2cover-200` (kept on disk as revert fallback, NOT ldconfig'd) |
| `fields2cover-v3-builder` | 91 | ros base | F2C v3 @ `884d895b…` + `<iomanip>` patch → `/opt/fields2cover-300` |
| `ublox-msgs-builder` | 142 | ros base | `ublox_ubx_msgs` + `ublox_ubx_interfaces` from `cedbossneo/ublox_dgnss` (`UBLOX_DGNSS_SHA=5e1d0cf…`) → `/opt/ublox_msgs` (schema resolution for foxglove_bridge only) |
| `base` | 171 | ros base | apt runtime deps (Nav2, twist_mux, BT.CPP, grid_map, foxglove-bridge, rtcm-msgs, opennav-docking, Cyclone DDS, ortools, python3-websockets), GTSAM + both F2C trees copied, ldconfig **only** `/opt/fields2cover-300/lib` (L275); L175-178 forces apt IPv4 (marked LOCAL-ONLY) |
| `deps` | 282 | base | colcon/rosdep; COPY of every package's `package.xml`+`CMakeLists.txt` (incl. `opennav_coverage_msgs`, `external/universal-gnss/gnss_ros2`, `tools/motor` → `src/mowgli_tools`); `rosdep install … \|\| true` |
| `build-interfaces` | 357 | deps | `mowgli_interfaces` only (cache layer) |
| `build` | 374 | build-interfaces | COPY `ros2/src/` + `tools/motor/`; `touch` COLCON_IGNORE in the 5 upstream `opennav_coverage` subpackages (L387-392); `colcon build -DBUILD_TESTING=OFF --parallel-workers 2` (L404-410); `colcon test -L gtest … \|\| true` (non-blocking, L416-419) |
| `runtime` | 425 | base | `install/` from build, `/opt/ublox_msgs`, launch+config dirs re-COPYed, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, entrypoint (L466), `mow_session_monitor.py` → `/ros2_ws/scripts/` (L471); `CMD ros2 launch mowgli_bringup mowgli.launch.py` (L475) |
| `simulation` | 487 | runtime | Webots R2025a (amd64 only, `TARGETARCH` guard), xvfb, TigerVNC/noVNC/openbox, `ros-kilted-webots-ros2`, `is_wsl()` patch, CRLF fix, `start_vnc.sh`, `foxglove/`; `EXPOSE 8765 6080`; `CMD sim_full_system.launch.py headless:=true use_rviz:=false` (L559) |

### Who runs what
- `install/compose/docker-compose.base.yml` service `mowgli` (container `mowgli-ros2`, image `${MOWGLI_ROS2_IMAGE}`) overrides CMD with `ros2 launch mowgli_bringup full_system.launch.py enable_foxglove:=${ENABLE_FOXGLOVE:-true}`; mounts `mowgli_maps:/ros2_ws/maps`, `./docker/config/mowgli:/ros2_ws/config`.
- `docker/docker-compose.simulation.yaml`: `simulation` + `dev-sim` (both `target: simulation`, `Xvfb :99` then headless launch; `dev-sim` bind-mounts config/launch/trees and `../ros2/src/e2e_test.py:/ros2_ws/src/e2e_test.py`), `simulation-gui` (`command: /ros2_ws/scripts/start_vnc.sh`, ports 6080/8765).
- `.devcontainer/post-create.sh` calls `ros2/scripts/build.sh` with `DEV_PACKAGES` when `MOWGLI_POST_CREATE_BUILD=1`.
- `ros2/systemd/mowgli.service`: `After=dev-ttyUSB0.device`, `ExecStart` via `/opt/mowgli_ros2/scripts/ros2_entrypoint.sh`, `ReadWritePaths=/opt/mowgli_ros2 /dev/mowgli /tmp`, `Restart=on-failure`.

### CI (`.github/workflows/`)
| Job (`ros2-ci.yml`) | Gate | What |
|------|------|------|
| `changes` | — | path filter as a job gate (`ros2/**`, `tools/motor/**`, `install/config/mowgli/**`, the workflow) so the required check always reports |
| `config-drift` | hard | `python3 ros2/scripts/check_config_drift.py` + `pytest ros2/scripts/test_check_config_drift.py` |
| `build-and-test` = **`Build & Test (ROS2 kilted)`** (required on `dev`) | hard | submodules recursive, `ln -s ../../tools/motor src/mowgli_tools`, cached GTSAM + F2C v3 source builds mirroring the Dockerfile, `touch` COLCON_IGNORE on the 5 opennav subpackages, `rosdep --skip-keys` them, `colcon build`/`colcon test`, `ros2 run mowgli_tools tune_drive_pid --help`, `full_system.launch.py --show-args` must NOT list `use_universal_gnss` |
| `format-check` | hard | `git-clang-format-18 --style=file:ros2/.clang-format --diff <merge-base origin/main> -- ros2/src` (changed lines only) |
| `static-analysis` | report-only | cppcheck `--enable=warning,performance,portability --inline-suppr` on changed `ros2/src` `.cpp/.hpp` (excl. `/test/`, `opennav_coverage`) |
| `ros2-docker.yml` | — | `docker/build-push-action` target `runtime`, amd64 + arm64 native runners, `ghcr.io/<repo>/mowgli-ros2`, SBOM off, smoke test (`ros2 pkg prefix mowgli_bringup/mowgli_tools`, `tune_drive_pid --help`, `--show-args` grep) |

### ROS surface of the scripts (all use absolute topics; `HighLevelControl` = `mowgli_interfaces/srv/HighLevelControl` on `/behavior_tree_node/high_level_control`)
| Script | Subscribes | Publishes / calls |
|--------|-----------|-------------------|
| `mow_session_monitor.py` | `/odometry/filtered_map`, `/wheel_odom`, `/imu/data`, `/gps/fix`, `/gnss/heading`, `/imu/cog_heading`, `/imu/fg_yaw`, `/fusion_graph/diagnostics`, `/behavior_tree_node/high_level_status`, `/hardware_bridge/status`, `/hardware_bridge/emergency`, `/gps/absolute_pose`, `/battery_state`, `/cmd_vel_nav`, `/cmd_vel`, `/plan`, `/scan` + TF `map→odom→base_footprint` | nothing (JSONL file only) |
| `diagnostics/{forward_5m,rotate_at_rate,undock_and_rotate_90}.py` | `high_level_status`, `/hardware_bridge/status`, `/hardware_bridge/emergency`, `/wheel_odom`, `/imu/data` (+ `/odometry/filtered_map` in forward_5m) | `/cmd_vel_teleop` (`TwistStamped`); `HighLevelControl` cmd 3 (`COMMAND_RECORD_AREA`) then 6 (`COMMAND_RECORD_CANCEL`) |
| `wheel_radius_cal.py`, `gps_log.py` | `/wheel_odom`, `/gps/fix` | `/cmd_vel_teleop` |
| `wheel_yaw_cal.py` | `/wheel_odom`, `/imu/data` | `/cmd_vel_teleop` |
| `forward_1m_yaw_test.py` | `/odometry/filtered_map`, `/wheel_odom`, `/imu/data`, `/gps/fix` | `/cmd_vel_teleop`; `HighLevelControl` |
| `coverage_ascii_render.py` | `/map_server_node/coverage_cells` (RELIABLE/VOLATILE depth 1), `/odometry/filtered_map` | — |
| `robot_monitor.sh` | echoes `high_level_status`, `/gps/fix`, `/odometry/filtered_map`, `/map_server_node/coverage_cells`, `/map`; tf2_echo; service `/map_server_node/get_coverage_status` (typed `mowgli_interfaces/srv/GetCoverageStatus` — **no such .srv exists and no node advertises it; the call is dead**) | — |
| `src/e2e_test.py` | `high_level_status`, `/coverage_planner_node/coverage_path`, `/wheel_odom`, `/odometry/filtered_map`, `/scan`, `/gps_degradation_sim/status`, `/map`, `/cmd_vel_smoothed`, `/cmd_vel` (`Twist`), `/map_server_node/boundary_violation` | `HighLevelControl` cmds 1/2/7/3/6; `/hardware_bridge/emergency_stop` (`mowgli_interfaces/srv/EmergencyStop`); `gz service /world/garden/{create/blocking,remove}` |
| `src/e2e_test_no_lidar.py` | `high_level_status`, `/mowgli/coverage/path`, `/wheel_odom`, `/odometry/filtered_map`, `/cmd_vel` | `HighLevelControl` cmd 1 |
| `src/precision_monitor.py` | `/wheel_odom`, `/gps/pose_sim`, `/scan`, `/map` (TRANSIENT_LOCAL), `/cmd_vel` | `/precision/gps_error_m`, `/precision/wheel_odom_speed`, `/precision/lidar_scan_count`, `/precision/lidar_min_range`, `/precision/map_known_pct`, `/precision/localization_quality` (`std_msgs/Float64`, 2 Hz) |

### Foxglove layout (`ros2/foxglove/mowgli_sim.json`)
- `3D!main` (`followTf: base_link`): `/scan`, `/map`, `/map_server_node/coverage_cells`, `/map_server_node/mow_progress`, `/local_costmap/costmap`, `/global_costmap/costmap`, `/robot_description`, `/plan`, `/local_plan`, `/keepout_mask`, `/speed_mask`, `/obstacle_tracker/markers`, `/FollowCoveragePath/global_plan`, `/FollowCoveragePath/global_point`, `/wheel_odom`, `/odometry/filtered_map`, `/odometry/filtered`, `/gps/pose_cov`.
- Plots: `/cmd_vel.linear.x|angular.z`, `/hardware_bridge/power.v_battery|v_charge`, `/gps/fix.position_covariance[0]`, `/wheel_odom.twist.twist.linear.x`, `/imu/cog_heading|/imu/mag_yaw|/odometry/filtered_map …orientation.z`, `/odometry/filtered_map.pose…`, `/odometry/filtered.twist…`.
- Status: `RawMessages!status` + `StateTransitions!bt` + `Indicator!state|emergency` + `Gauge!coverage` all on `/behavior_tree_node/high_level_status`; `RawMessages!diagnostics` on `/diagnostics`; `Log!rosout`; `CallService!start_mowing|return_home|stop`; `Publish!teleop`.

## Build, test, run
```bash
cd ros2                                   # inside the devcontainer (/ros2_ws must exist)
make build-full                           # or: make build-pkg PKG=mowgli_behavior ; PACKAGES_MODE=select make build-pkg PKG=x
make test                                 # or: PACKAGES="mowgli_behavior" ./scripts/test.sh
make format-check && make lint            # ./scripts/format.sh --check is the CI-equivalent (skips opennav_coverage)
make sim                                  # headless Webots; Foxglove ws://localhost:8765 (needs an X server on :99)
make e2e-test                             # self-contained: sim-stop + build + sim + 90 s wait + src/e2e_test.py + sim-stop
make e2e-test-no-lidar                    # GPS-only variant, src/e2e_test_no_lidar.py (exit 1 on FAIL)
python3 -m pytest -q ros2/scripts/test_check_config_drift.py && python3 ros2/scripts/check_config_drift.py   # from repo root, PyYAML only
python3 ros2/scripts/compute_nav2_params.py --compare --overlay lidar --robot-yaml ros2/src/mowgli_bringup/config/mowgli_robot.yaml
docker build -f ros2/Dockerfile --target runtime -t mowgli-ros2 .    # from repo ROOT (context must contain tools/motor)
docker compose -f docker/docker-compose.simulation.yaml up dev-sim   # then: exec dev-sim bash -c "source /ros2_ws/install/setup.bash && python3 /ros2_ws/src/e2e_test.py"
```
- Test files in this area: `ros2/scripts/test_check_config_drift.py` (pytest; pins the three drift checks + that the committed yaml pair is clean). The E2E harnesses are not colcon tests; nothing in CI runs them. Package unit tests live in each package (`colcon test`, run by `Build & Test (ROS2 kilted)`); the Docker `build` stage runs them non-blocking.
- Diagnostic scripts on the real robot: `docker cp ros2/scripts/diagnostics/X.py mowgli-ros2:/tmp/ && docker exec mowgli-ros2 bash -c 'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && python3 /tmp/X.py'` (`ros2/scripts/diagnostics/README.md`).

## Change coupling — "if you change X, also update Y"
- **GTSAM / F2C v3 pin or cmake flags** in `ros2/Dockerfile` ↔ the same recipe + cache keys (`gtsam-4.3a1-…`, `f2c-3.0.0-884d895-…`) in `.github/workflows/ros2-ci.yml`, and `.devcontainer/Dockerfile` (mirrors stage 0). F2C install prefix ↔ `ros2/src/mowgli_coverage/CMakeLists.txt:42` (`find_package(Fields2Cover 3.0.0 … PATHS /opt/fields2cover-300)`).
- **New `mowgli_*` package**: auto-linked by `sync_workspace_packages.sh`, but must be added to the `deps` stage COPY list in `ros2/Dockerfile` (L297-340) for the rosdep cache layer, and its launch/config dirs re-COPYed in `runtime` if it ships any (L450-460 pattern). Non-`mowgli_*` names need an explicit `link_workspace_package` line.
- **Upstream `opennav_coverage` subpackages**: the COLCON_IGNORE marker list is duplicated in `sync_workspace_packages.sh` L141-169, `ros2/Dockerfile` L387-392, and `ros2-ci.yml` (`touch` + `rosdep --skip-keys`) — change all three.
- **`ublox_dgnss` pin** (`UBLOX_DGNSS_REF/SHA`, Dockerfile L145-147): the comment says it must match `sensors/gps/Dockerfile`, but that file currently has no `UBLOX_DGNSS_*` args — verify the GPS sidecar's actual driver source before bumping.
- **`mowgli_robot.yaml` keys**: adding/removing a key → update `STRUCTURAL` / `USER_OVERRIDE` / `CALIBRATION_OUTPUT` / `INSTALL_SEED` in `check_config_drift.py`, `retiredParamKeys` in `gui/pkg/api/settings.go` (L367), and the physics inputs in `compute_nav2_params.py` (`gather_inputs`, L284) if Nav2 derives from it. Retired keys must be REMOVED from `USER_OVERRIDE` (so the orphan report surfaces them).
- **Nav2 param split** (`nav2_params_base.yaml` + overlays): `mow_session_monitor.py` hashes all three (`_CONFIG_LOCATIONS`, L1050-1081) and `compute_nav2_params.py --compare` deep-merges them with `robot_config_util.deep_merge` — keep file names in sync.
- **Topics renamed in the stack** → `mow_session_monitor.py` subscriptions (L306-359), `mowgli_sim.json` topic list, `robot_monitor.sh`, and both E2E harnesses (already carry dead topics, see Pitfalls).
- **`HighLevelControl` command ids / `HighLevelStatus.state_name` strings** → the constants `HL_CMD_RECORD_AREA=3`, `HL_CMD_RECORD_CANCEL=6`, `HL_STATE_RECORDING=3` in the three `diagnostics/*.py` motion scripts and the literal ids / state names in `src/e2e_test*.py` (`send_command(2, …)`, `"IDLE_DOCKED"`, `"MOWING_COMPLETE"`, …).
- **`ros2_entrypoint.sh`** is COPYed by `ros2/Dockerfile` (L466) and `ros2/Dockerfile.dev` (L156), and referenced by `systemd/mowgli.service` — keep it distro-agnostic or update all consumers. The sensor images do NOT share it: each ships its own copy (e.g. `sensors/gps/ros2_entrypoint.sh`).
- **Runtime image CMD/launch** (`mowgli.launch.py`, Dockerfile L475) vs compose (`full_system.launch.py`) vs systemd (`mowgli.launch.py`): three launch entry points, change deliberately.
- **`sim_full_system.launch.py` args** used by the Makefile (`headless`, `use_rviz`, `use_lidar`) and by `start_vnc.sh` / `start_dev_sim.sh` / compose commands.

## Pitfalls
- `make docker` / `make docker-sim` build with context `ros2/` (`docker build … .`, Makefile L133-143) but `ros2/Dockerfile` COPYs `ros2/src/…` and `tools/motor/…` relative to the **repo root** — they fail; build from the root (`-f ros2/Dockerfile .`) as `ros2-docker.yml` and the compose files do. Consequently `ros2/.dockerignore` is inert (no root `.dockerignore` exists) — `build/`, `install/`, `.git` are sent in the context.
- `ros2/Dockerfile.dev` uses context `ros2/` (`COPY scripts/…`, L156-161), advertises Jazzy/Gazebo and `make dev-*` targets that do not exist in `ros2/Makefile`, and pulls `ros-kilted-slam-toolbox` + `ros-kilted-ros-gz-*`; no compose service or workflow builds it. Do not treat it as the devcontainer (that is `.devcontainer/Dockerfile`).
- `ros2/Dockerfile` L178 (`Acquire::ForceIPv4`) is labelled "LOCAL-ONLY (do not commit)" (L175) yet is committed and reaches every CI/ghcr image.
- `make sim` / `make e2e-test` export `DISPLAY=:99` but do not start Xvfb (compose does: `Xvfb :99 … &`); run Xvfb yourself in the devcontainer or Webots fails to open a display.
- `make e2e-test` is **self-contained** (kills any running sim first, rebuilds, waits a fixed 90 s). `src/e2e_test.py` still drives Gazebo (`gz service /world/garden/…`, L561/577; "garden.sdf" L1447) although the sim is Webots (`sim_full_system.launch.py:20`, `worlds_webots/mowgli_garden.wbt`), and subscribes topics nothing publishes (`/coverage_planner_node/coverage_path` L179, `/gps_degradation_sim/status` L195, `/cmd_vel_smoothed` L203); `e2e_test_no_lidar.py` subscribes `/mowgli/coverage/path` (L102) and its docstring/comments (L12, L458) still say robot_localization dual EKF (see CLAUDE.md Invariant 1). Expect path-deviation and map criteria to report no data until these are re-pointed.
- `src/precision_monitor.py` subscribes `/gps/pose_sim` (L109) — no publisher in the tree; the `/precision/*` topics stay at their defaults. Neither `precision_monitor.py` copy is in any image.
- `ros2/scripts/e2e_test.py` and `ros2/scripts/precision_monitor.py` are stale duplicates of the `ros2/src/` versions (the scripts copy still references `/pose`, SLAM map growth, `/ros2_ws/src/scripts/e2e_test.py`); edit the `ros2/src/` files.
- `systemd/mowgli.service` sets `ROS_DISTRO=jazzy` (L36) and its comment (L19) says jazzy, while `ros2_entrypoint.sh` hardcodes `/opt/ros/kilted/setup.bash` (L15); `make deploy` rsyncs only `install/` to `/opt/mowgli_ros2/install/` but the unit's `ExecStart` needs `/opt/mowgli_ros2/scripts/ros2_entrypoint.sh` (never synced). The unit `Documentation=` URL points at `cedricziel/mowgli-ros2`. The supported deployment is the Docker installer (`install/mowglinext.sh`); treat the unit as unmaintained.
- `make backup-maps` help text says "Pull SLAM maps" (L48) and pulls `/opt/mowgli_ros2/maps/*` — the Docker deployment keeps maps in the `mowgli_maps` volume mounted at `/ros2_ws/maps`; there is no SLAM (CLAUDE.md "Do NOT re-introduce slam_toolbox"). Same for `sim-stop.sh` L63-64 (`garden_map.posegraph`), `robot_monitor.sh` L120/L209-212 ("SLAM" labels on `map→odom` and `/map`), and `mow_session_monitor.py` comments mentioning `ekf_map_node` (L118) — labels only, harmless.
- `Makefile` `lint`/`format`/`format-check` glob all of `src/` **including** `opennav_coverage` submodule sources (Makefile L145-166) — use `./scripts/format.sh --check` (excludes it, L27) to match CI; `make format` will rewrite submodule files.
- `format.sh` warns (does not fail) on clang-format ≠ 18; CI pins 18 and the pre-commit hook pins v18.1.8 — a brew clang-format 22 silently produces diffs CI rejects.
- `check_config_drift.py` paths are repo-relative to `Path(__file__).parents[2]` (L64-66): run from a checkout, not from the container's `/ros2_ws/config`.
- `compute_nav2_params.py --robot-yaml` defaults to `docker/config/mowgli/mowgli_robot.yaml` (L1054), which does not exist in the repo (`docker/config/mowgli/` holds `drive_tuning` + README) — always pass the template or `install/config/mowgli/mowgli_robot.yaml`. Its `--compare` still emits MPPI-era keys (`mppi()`, L431) while the coverage controller is FTC (CLAUDE.md Invariant 8).
- `mow_session_monitor.py` defaults (`--output-dir /home/ubuntu/mowglinext/docker/logs/mow_sessions`, git lookups and `install/.env` under `/home/ubuntu/mowglinext`, L985-1030) assume the robot host layout; inside the container pass `--output-dir /ros2_ws/maps` (bind-mounted volume) or the file lands in an unmounted path.
- `robot_monitor.sh` hardcodes container `mowgli-ros2` (L14) and expects `mowgli-gui/gps/lidar/mqtt` containers — the `install/compose` names; it is host-side, not for the devcontainer.
- `sync_workspace_packages.sh` refuses to overwrite a non-symlink entry in `/ros2_ws/src` (L101-104); CI's `ln -s ../../tools/motor src/mowgli_tools` and the script's `mowgli_tools` link are the same package under two mechanisms.
- `opennav_coverage` COLCON_IGNORE markers are **untracked** files created at sync/build/CI time (`touch`), never committed (unforked submodule) — a fresh checkout without running the sync script or Dockerfile will try to compile the F2C-1.2.1 server packages and fail. Even with `INCLUDE_OPENNAV_COVERAGE_STACK=1` they are source-inspection only.
- No in-tree package depends on `opennav_coverage_msgs` any more (`mowgli_coverage/package.xml` deps L17-28 → `mowgli_interfaces`; coverage action is `mowgli_interfaces/action/PlanCoverage`, CLAUDE.md Invariant 7). The submodule is still linked, COPYed and built (sync L135-138, Dockerfile L331-332) purely by inertia.
- `universal-gnss` is pinned to the `mowglinext` fork branch `main` (gitlink `ab32f673`) per `.gitmodules`; only `gnss_ros2/` is a colcon package (`universal_gnss_ros2`), the `gnss_*` siblings are plain CMake subdirs pulled in at build. `UNIVERSAL_GNSS_PATH` overrides the vendored copy; legacy `/workspaces/universal-gnss` is only the fallback tried after it (sync L35-37, L65-72). The runtime stack does not launch it (`full_system.launch.py --show-args` must not list `use_universal_gnss`, asserted in CI and the Docker smoke test); the `mowgli-gps` sidecar owns GNSS.
- Dockerfile `build` stage runs `colcon test … || true` (L412-419) — image builds never fail on unit tests; the gate is `ros2-ci.yml`.
- `ros2/CPPLINT.cfg` `set noparent` applies to `ros2/` only; cpplint is run by `make lint`, not by CI.

## Generated & vendored — do not hand-edit
- `ros2/src/opennav_coverage/` — upstream `open-navigation/opennav_coverage` submodule @ `d6e41a29` (`main`); only `opennav_coverage_msgs` is ever linked/built; the 5 server subpackages are COLCON_IGNORE'd by untracked markers (see CLAUDE.md "Do NOT use the upstream `opennav_coverage` server").
- `ros2/src/external/universal-gnss/` — `mowglinext/universal-gnss` fork submodule @ `ab32f673` (branch `main`, tracking `Pepeuch/universal-gnss` main); revert `.gitmodules` to `pepeuch/universal-gnss` directly once the fork is no longer needed. Top level: `gnss_core/ gnss_driver/ gnss_ntrip/ gnss_protocols/ gnss_ros2/ gnss_tools/ gnss_transport/ docs/ examples/ testdata/ MOWGLINEXT_TODO.md`.
- `/ros2_ws/src/*` symlinks, `build/ install/ log/`, `maps_backup/` — produced by `sync_workspace_packages.sh` / colcon / `make backup-maps`; gitignored.
- `docker/logs/mow_sessions/*.jsonl` — session recordings written by `mow_session_monitor.py`; gitignored (`.gitignore` L64-65, only `.gitkeep` is force-tracked). The ad-hoc `.py`/`.md` analysis files sitting next to them ARE tracked.
