# ROS2 workspace — working notes for Claude

`ros2/` holds every ROS2 node, launch file, param default and unit test on the robot, plus the workspace tooling (`Makefile`, `scripts/`, `Dockerfile`) that links, builds, images and exercises them. It does **not** own STM32 behaviour (`firmware/` — see [`../firmware/CLAUDE.md`](../firmware/CLAUDE.md)), the operator UI or the settings write-path (`gui/` — [`../gui/CLAUDE.md`](../gui/CLAUDE.md)), container/compose orchestration and the installed config seed (`install/`, `docker/`), or the sensor driver images (`sensors/`).

Read the root [`CLAUDE.md`](../CLAUDE.md) first — Safety, the 16 Architecture Invariants and "What NOT to Do" are binding here and are **not** repeated below.

## Read next

| File | Read it when… |
|------|---------------|
| [`docs/claude/codemaps/fusion_graph.md`](../docs/claude/codemaps/fusion_graph.md) | Touching the GTSAM localizer: factors, GPS gates, LiDAR map anchor, persistence, TF publication |
| [`docs/claude/codemaps/mowgli_behavior.md`](../docs/claude/codemaps/mowgli_behavior.md) | BT nodes, `main_tree.xml`, guards, coverage resume, docking/undocking, `HighLevelControl` handling |
| [`docs/claude/codemaps/mowgli_bringup.md`](../docs/claude/codemaps/mowgli_bringup.md) | Launch files, the `mowgli_robot.yaml` template, Nav2 base+overlay params, twist_mux, URDF |
| [`docs/claude/codemaps/mowgli_coverage.md`](../docs/claude/codemaps/mowgli_coverage.md) | The F2C v3 `plan_coverage` server: rings, swaths, connectors, sub-path splitting, verification |
| [`docs/claude/codemaps/mowgli_hardware.md`](../docs/claude/codemaps/mowgli_hardware.md) | `hardware_bridge_node`: serial/COBS link, odometry, IMU cal, blade gate, dig detector |
| [`docs/claude/codemaps/mowgli_interfaces.md`](../docs/claude/codemaps/mowgli_interfaces.md) | Adding/changing a `.msg`/`.srv`/`.action`, or the shared header-only helpers |
| [`docs/claude/codemaps/mowgli_leds.md`](../docs/claude/codemaps/mowgli_leds.md) | The optional WS2812 status ring (SPI, off by default, hardware-unverified) |
| [`docs/claude/codemaps/mowgli_localization.md`](../docs/claude/codemaps/mowgli_localization.md) | GPS→ENU projection, COG/mag yaw, the scan pipeline, dock detection, IMU/mag/dock calibration |
| [`docs/claude/codemaps/mowgli_map.md`](../docs/claude/codemaps/mowgli_map.md) | Area polygons, `areas.dat`, keepout/mow-progress masks, obstacle promotion, dock pose gates |
| [`docs/claude/codemaps/mowgli_monitoring.md`](../docs/claude/codemaps/mowgli_monitoring.md) | `/diagnostics` aggregation or the (stubbed) MQTT bridge |
| [`docs/claude/codemaps/mowgli_nav2_plugins.md`](../docs/claude/codemaps/mowgli_nav2_plugins.md) | FTCController or `PathProgressGoalChecker` — the coverage controller lane |
| [`docs/claude/codemaps/mowgli_simulation.md`](../docs/claude/codemaps/mowgli_simulation.md) | The Webots world, `MowgliMower` PROTO, `kinematic_drive.py`, sim-only helper nodes |
| [`docs/claude/codemaps/ros2_workspace_tooling.md`](../docs/claude/codemaps/ros2_workspace_tooling.md) | Makefile/scripts/Dockerfile/systemd, the E2E harnesses, field-calibration + diagnostic scripts |
| [`docs/claude/ros-interfaces.md`](../docs/claude/ros-interfaces.md) | You need the resolved name of any topic/service/action/TF frame, or who creates it |
| [`docs/claude/parameters.md`](../docs/claude/parameters.md) | Adding/renaming a knob — where its default lives, who injects it, whether the GUI can edit it |
| [`docs/claude/testing-ci.md`](../docs/claude/testing-ci.md) | You need the exact test command for a package, or which CI job gates it |
| [`docs/claude/doc-index.md`](../docs/claude/doc-index.md) | Deciding whether a doc you found is authoritative, historical or generated |
| [`docs/claude/ros2-specifics.md`](../docs/claude/ros2-specifics.md) | The detail behind Invariants 1/2/7/8 — localizer, Nav2, coverage, IMU/GPS fusion |
| [`docs/claude/high-level-api.md`](../docs/claude/high-level-api.md) | `HighLevelControl`/`HighLevelStatus`, BT states, area recording, manual mowing |
| [`docs/claude/commands.md`](../docs/claude/commands.md) | After editing `mowgli_interfaces/msg|srv` — the three codegen scripts and their outputs |
| [`docs/claude/session-monitoring.md`](../docs/claude/session-monitoring.md) | Running a mow/tuning test — record the JSONL timeline in parallel |
| [`docs/WEBOTS_SIM.md`](../docs/WEBOTS_SIM.md) | Before touching `worlds_webots/`, `protos/`, `urdf_webots/` or `kinematic_drive.py` (ODE quirks) |
| [`.claude/rules/ros2.md`](../.claude/rules/ros2.md) | Node/QoS/topic-naming/launch/test conventions for new code |
| [`README.md`](README.md) | Stack-level orientation: package list, TF tree, topic table, launch files, hardware protocol |
| [`scripts/diagnostics/README.md`](scripts/diagnostics/README.md) | Driving the real robot for a drivetrain/yaw measurement |
| [`scripts/compute_nav2_params.md`](scripts/compute_nav2_params.md) | Deriving Nav2/FTC/costmap numbers from chassis physics instead of guessing |
| [`src/mowgli_leds/README.md`](src/mowgli_leds/README.md) | Wiring or enabling the LED ring (U-Boot SPI overlay is manual) |
| [`wiki/Architecture.md`](../wiki/Architecture.md) | Steady-state architecture reference (the fusion_graph section supersedes `docs/HANDOFF_FUSION_GRAPH.md`) |
| [`wiki/Behavior-Trees.md`](../wiki/Behavior-Trees.md) | Explaining or documenting BT states to a human |
| [`wiki/Configuration.md`](../wiki/Configuration.md) | Operator-facing meaning of a `mowgli_robot.yaml` key |
| [`wiki/Simulation.md`](../wiki/Simulation.md) | Operator-facing sim setup |

## Build · test · run

All of these assume the devcontainer (`/ros2_ws` exists, sourced ROS Kilted), run from `ros2/`.

```bash
make build                     # = build-full: colcon build --symlink-install, Release
make build-pkg PKG=mowgli_behavior          # --packages-up-to; PACKAGES_MODE=select for just that one
make build-dev                 # mowgli_interfaces mowgli_localization universal_gnss_ros2 mowgli_bringup
make build-debug               # BUILD_TYPE=Debug ; make clean removes build/ install/ log/
make test                      # colcon test + colcon test-result --verbose (needs install/setup.bash)
PACKAGES="mowgli_hardware" ./scripts/test.sh      # single package
cd /ros2_ws && colcon test --packages-select fusion_graph --return-code-on-test-failure
./scripts/format.sh --check    # CI-equivalent format gate (excludes both submodules)
make format                    # clang-format -i — see gotchas, it still rewrites opennav_coverage
make lint                      # cppcheck --enable=all + cpplint (CI runs its own report-only cppcheck instead)
make sim                       # sim-stop, then headless Webots; Foxglove ws://localhost:8765
make sim-stop                  # after any crash: kills Webots/nodes, wipes DDS shm + Webots IPC
make e2e-test                  # self-contained: sim-stop + build + sim + 90 s + src/e2e_test.py
make e2e-test-no-lidar         # GPS-only variant (src/e2e_test_no_lidar.py)
```

From the **repo root** (not `ros2/`):

```bash
python3 ros2/scripts/check_config_drift.py && python3 -m pytest -q ros2/scripts/test_check_config_drift.py
python3 ros2/scripts/compute_nav2_params.py --compare --overlay lidar \
        --robot-yaml ros2/src/mowgli_bringup/config/mowgli_robot.yaml
docker build -f ros2/Dockerfile --target runtime -t mowgli-ros2 .    # context MUST be the repo root
```

## Conventions

- **C++ format:** clang-format **18** only, style `ros2/.clang-format` (Google base, Allman braces, 100 cols, 2-space indent). CI runs `git-clang-format-18` on changed lines vs `origin/main`; `scripts/format.sh` only *warns* on a version mismatch and the pre-commit hook pins `v18.1.8` — a brew clang-format 19+/22 produces diffs CI rejects.
- **Lint:** cpplint filters live in `ros2/CPPLINT.cfg` (linelength 100). `ros2/.pre-commit-config.yaml` adds ruff (+ruff-format) for Python, cmakelint (100 cols), detect-secrets, and the XML/YAML checks.
- **Launch files are `.launch.py` only**, params loaded from a package `share/config/*.yaml`; declare every parameter in the node constructor. See `.claude/rules/ros2.md` for QoS profiles and topic-naming (`~/relative` in C++, remap in launch).
- **Defaults go in the TEMPLATE** `src/mowgli_bringup/config/mowgli_robot.yaml`, never the sparse installed file (Invariant 15) — and a new key must also be *injected* by the launch file or it is inert (see gotchas).
- **Codegen:** after editing `src/mowgli_interfaces/msg|srv`, run the three generators in [`docs/claude/commands.md`](../docs/claude/commands.md) (firmware `sync_ros_lib.py`, GUI `generate_go_msgs.sh`, `generate_ts_types.sh`) — CI gates the drift. Never hand-edit `*_generated.go`, `ros_lib/mower_msgs/*.h`, `ros.generated.ts`. Wire-protocol constants (`mowgli_protocol.h`, `ll_datatypes.hpp`) are mirrored **by hand** and versioned by `MOWGLI_PROTOCOL_VERSION`.
- **Do not hand-edit** `src/opennav_coverage/` or `src/external/universal-gnss/` (git submodules), `/tmp/mowgli_nav2_*.yaml` (written at every launch), or `build/ install/ log/`.
- **GPL headers:** `scripts/fix-copyright.sh` prepends one to staged sources that lack it (not wired into pre-commit).
- **Tests:** `ament_cmake_gtest` per package, `ament_add_pytest_test`/`launch_testing` in `mowgli_bringup`; new BT nodes need a tick test (`.claude/rules/ros2.md`).

## Component-specific gotchas

- `/ros2_ws/src` is a set of **symlinks** created by `scripts/sync_workspace_packages.sh`; it auto-globs `src/mowgli_*/` only. A new non-`mowgli_*` package needs an explicit `link_workspace_package` line there **and** a `package.xml`/`CMakeLists.txt` entry in the `deps` stage of `ros2/Dockerfile`. `mowgli_tools` is `tools/motor/` — outside `ros2/`.
- `make docker` / `make docker-sim` build with context `ros2/` while the Dockerfile COPYs from the repo root — they fail. Build from the root as CI and compose do.
- `make lint` / `make format` / `make format-check` glob all of `src/`, **including** the `opennav_coverage` submodule; use `./scripts/format.sh [--check]` to match CI and avoid rewriting vendored files. `format.sh` itself prunes BOTH submodules since 2026-09-04 (`*/opennav_coverage/*` and `*/external/*`) — before that fix, every pre-push rewrote 155 vendored universal-gnss files and blocked re-pinning it.
- `make sim` and `make e2e-test` export `DISPLAY=:99` but do **not** start Xvfb (compose does) — start it yourself or Webots fails to open a display. After any crash, `scripts/sim-stop.sh` (a stale Webots IPC socket hangs the next launch in "retrying").
- **A parameter no launch file injects is inert** — the node silently runs its compiled `declare_parameter` default no matter what the YAML says ([`docs/claude/parameters.md`](../docs/claude/parameters.md); the mowgli_bringup codemap lists today's inert and orphan keys). Param **order** matters too: the injected dicts are appended AFTER the static params file, so the launch-injected value wins for shared keys — in `mowgli.launch.py` the injected `imu_cal_samples: 200` beats `hardware_bridge.yaml`'s 1000.
- Expect 3–4 disagreeing defaults per knob (struct initialiser vs `declare_parameter` vs package YAML vs launch fallback): `GraphParams::max_graph_nodes` is 3000 in the struct but 6000 in `fusion_graph.yaml`; `obstacle_reverse_enabled` is `false` in FTC's struct **and** its `declare_bool`, `true` in `nav2_params_base.yaml`. Read `declare_parameters()`, not the member initialiser — and note unit tests construct the structs directly, so they run on struct defaults, not production values.
- Topics resolved from a node **name** are not remapped (`~/dig_event` → `/hardware_bridge/dig_event`, `~/boundary_violation`, `~/set_pose`): renaming a node silently breaks its subscribers with no build error.
- A large number of in-code comments still describe removed components (robot_localization EKFs, slam_toolbox, MPPI, Gazebo, `dock_yaw_to_set_pose`, per-segment coverage dispatch). Each codemap's Pitfalls section enumerates the stale ones — trust the code plus the root invariants, and fix prose only when you are already in the file.
- `sim_full_system.launch.py` does **not** go through `mowgli.launch.py`: it launches its own twist_mux, skips hardware_bridge/RSP, and forwards only `behavior_tree.yaml` — so the sim BT runs C++ fallbacks for every `mowgli_robot.yaml`-sourced knob. Sim behaviour is not evidence about the robot.
- Lifecycle nodes' `on_configure` can run twice (cleanup → configure): guard declarations with `has_parameter` or Nav2 bring-up bricks (`coverage_server.cpp`).
- ROS2 cannot type an **empty YAML list** in a params file — lifecycle bring-up throws. Omit the key instead (`src/mowgli_map/config/map_server.yaml`).
- The GUI/firmware generators **glob** `mowgli_interfaces/msg/`, not the `CMakeLists.txt` `msg_files` list: an unregistered `.msg` still produces Go/TS/firmware bindings while not existing at runtime (`CoveragePath.msg` today).
- The E2E harnesses are gated by nothing and carry dead topics plus Gazebo `gz service` calls under a Webots sim — some criteria report "no data" or SKIP by design. `scripts/e2e_test.py` and `scripts/precision_monitor.py` are stale duplicates; edit the `src/` copies.
- The Docker `build` stage runs `colcon test … || true` — image builds never fail on unit tests. The gate is `ros2-ci.yml` → **`Build & Test (ROS2 kilted)`**.

## Safety

Everything in this workspace can move a robot with spinning blades. Re-read the root [Safety section](../CLAUDE.md#safety--read-first) before changing motion or blade paths, and flag such changes as safety-critical in the PR.

- `mowgli_hardware` is the **only** writer to the STM32; blade commands stay fire-and-forget and firmware remains the sole blade/e-stop authority (root Safety, Invariant 9). `blade_gate.hpp` suppresses ENABLE only — never add a path that swallows a DISABLE.
- Motion-reducing behaviours belong in `hardware_bridge` (dig hard-stop + bounded reverse, Invariant 16); overridable motion belongs in the BT on `/cmd_vel_nav` behind collision_monitor (`EscapeStartBlocked`). Do not swap those layers.
- `FollowStrip` forces the blade OFF before any transit over `kSegmentTransitGap` (`mowgli_behavior/src/coverage_nodes.cpp`) — structural, never route around it.
- `mowgli_leds` and `mowgli_monitoring` are deliberately outside the motion path; `mqtt_bridge_node`'s command relay reaches the same `HighLevelControl` service the GUI uses, so an open broker is remote control.
