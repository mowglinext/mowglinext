# Testing & CI Index

> Index generated 2026-09-03 at f21729e9.
>
> Every test suite in the monorepo, the exact command that runs it, and the workflow (if any) that gates it. Loaded on demand from [`../../CLAUDE.md`](../../CLAUDE.md). Per-package detail lives in [`codemaps/`](codemaps/); this file is the cross-cutting map.

**The short version:** ROS2 C++/Python tests and the GUI web suite are gated in CI. **Go tests, Playwright e2e, the installer bash suite, the `docs/` static checks and the simulation E2E are gated by NOTHING** — run them by hand.

---

## Test map

| Area | Test files | Framework | Exact local command | CI workflow |
|---|---|---|---|---|
| `fusion_graph` (27 suites) | `ros2/src/fusion_graph/test/test_*.cpp` | GoogleTest (`ament_add_gtest`, `CMakeLists.txt:145–252`) | `cd /ros2_ws && colcon test --packages-select fusion_graph --return-code-on-test-failure` | `ros2-ci.yml` → `build-and-test` |
| `mowgli_behavior` (19 suites) | `ros2/src/mowgli_behavior/test/test_*.cpp` | GoogleTest (`CMakeLists.txt:116–601`) | `colcon test --packages-select mowgli_behavior --return-code-on-test-failure` | `ros2-ci.yml` → `build-and-test` |
| `mowgli_hardware` (9) | `ros2/src/mowgli_hardware/test/test_*.cpp` — incl. `test_dig_detector.cpp` + `test_dig_escalation.cpp` (Invariant 16), `test_cobs.cpp`, `test_protocol.cpp`, `test_blade_gate.cpp` | GoogleTest (`CMakeLists.txt:114–190`) | `colcon test --packages-select mowgli_hardware --return-code-on-test-failure` | `ros2-ci.yml` → `build-and-test` |
| `mowgli_localization` (7) | `ros2/src/mowgli_localization/test/test_*.cpp` | GoogleTest (`CMakeLists.txt:271–342`) | `colcon test --packages-select mowgli_localization --return-code-on-test-failure` | `ros2-ci.yml` → `build-and-test` |
| `universal_gnss_ros2` (6) — vendored submodule | `ros2/src/external/universal-gnss/gnss_ros2/tests/test_*.cpp` | GoogleTest (`CMakeLists.txt:266–386`; `test_ntrip_node` is Linux-only) | `colcon test --packages-select universal_gnss_ros2 --return-code-on-test-failure` | `ros2-ci.yml` → `build-and-test` |
| `mowgli_nav2_plugins` (5) | `ros2/src/mowgli_nav2_plugins/test/test_ftc_*.cpp`, `test_obstacle_deviation.cpp`, `test_oscillation_detector.cpp` | GoogleTest (`CMakeLists.txt:115–160`) | `colcon test --packages-select mowgli_nav2_plugins --return-code-on-test-failure` | `ros2-ci.yml` → `build-and-test` |
| `mowgli_map` (4) | `ros2/src/mowgli_map/test/test_*.cpp` | GoogleTest (`CMakeLists.txt:172–232`) | `colcon test --packages-select mowgli_map --return-code-on-test-failure` | `ros2-ci.yml` → `build-and-test` |
| `mowgli_leds` (2) | `ros2/src/mowgli_leds/test/test_*.cpp` | GoogleTest (`CMakeLists.txt:96–103`) | `colcon test --packages-select mowgli_leds --return-code-on-test-failure` | `ros2-ci.yml` → `build-and-test` |
| `mowgli_coverage` (1) | `ros2/src/mowgli_coverage/test/test_coverage_planning.cpp` | GoogleTest (`CMakeLists.txt:129`) | `colcon test --packages-select mowgli_coverage --return-code-on-test-failure` | `ros2-ci.yml` → `build-and-test` |
| `mowgli_monitoring` (1) | `ros2/src/mowgli_monitoring/test/test_diagnostics.cpp` | GoogleTest (`CMakeLists.txt:167`) | `colcon test --packages-select mowgli_monitoring --return-code-on-test-failure` | `ros2-ci.yml` → `build-and-test` |
| `mowgli_simulation` (1) | `ros2/src/mowgli_simulation/test/test_firmware_wheel_model.cpp` | GoogleTest (`CMakeLists.txt:121`) | `colcon test --packages-select mowgli_simulation --return-code-on-test-failure` | `ros2-ci.yml` → `build-and-test` |
| `mowgli_bringup` static config guards (6) | `ros2/src/mowgli_bringup/test/test_nav2_params.py`, `test_gnss_launch_config.py`, `test_robot_config_util.py`, `test_urdf_xacro.py`, `test_tf_ownership.py`, `test_launch_injection.py` | pytest (`ament_add_pytest_test`, `CMakeLists.txt:59–86`) | `colcon test --packages-select mowgli_bringup --return-code-on-test-failure` | `ros2-ci.yml` → `build-and-test` |
| `mowgli_bringup` launch integration (2) | `ros2/src/mowgli_bringup/test/test_nodes_startup.launch.py` (TIMEOUT 60), `test_navsat_status_universal.launch.py` (TIMEOUT 45) | `launch_testing` (`add_launch_test`, `CMakeLists.txt:46–52`) | `colcon test --packages-select mowgli_bringup` | `ros2-ci.yml` → `build-and-test` |
| `mowgli_tools` (drive-PID tuner sidecar) | `tools/motor/test/test_drive_pid_math.py` (registered), `tools/motor/test/test_drive_pid_tuner.py` (**not registered**) | pytest (`tools/motor/CMakeLists.txt:24–27`) | `colcon test --packages-select mowgli_tools` (math only) · `python3 -m pytest tools/motor/test` (both) | `ros2-ci.yml` → `build-and-test` (CI symlinks `tools/motor` into the workspace as `src/mowgli_tools`, `ros2-ci.yml:151–159`) |
| Config-drift guard | `ros2/scripts/test_check_config_drift.py` over `ros2/scripts/check_config_drift.py` | pytest (run directly, not via colcon) | `python3 -m pytest -q ros2/scripts/test_check_config_drift.py` | `ros2-ci.yml` → `config-drift` |
| Firmware ↔ host msg/wire guards | `firmware/scripts/sync_ros_lib.py`, `protocol_version_guard.py`, `board_defaults_parity.py` (self-checking scripts, no test files) | plain Python `--check` gates | `python3 firmware/scripts/sync_ros_lib.py --check` · `python3 firmware/scripts/protocol_version_guard.py --check` · `python3 firmware/scripts/board_defaults_parity.py` | `msg-codegen-drift.yml`, `protocol-version-drift.yml`, `firmware-ci.yml` → `defaults-parity` |
| GUI web unit tests (45 files) | `gui/web/src/**/*.test.ts`, `*.test.tsx` | vitest + jsdom + Testing Library (`gui/web/vitest.config.ts`) | `cd gui/web && yarn test` | `gui-ci.yml` → `unit-tests` |
| GUI backend (36 files) | `gui/pkg/api/*_test.go`, `gui/pkg/providers/*_test.go`, `gui/pkg/foxglove/*_test.go`, `gui/pkg/msgs/mowgli/mower_control_bind_test.go`, `gui/pkg/types/mocks_test.go` | Go `testing` | `cd gui && go test ./...` | **NONE** — no workflow contains `go test` |
| GUI browser E2E (5 specs) | `gui/web/tests/e2e/*.spec.ts` (+ `mock/` backend, fully mocked REST + WebSocket) | Playwright (`gui/web/playwright.config.ts`) | `cd gui/web && yarn test:e2e` | **NONE** |
| Installer suite (20 scripts) | `install/test_mowglinext.sh` + `install/tests/test_*.sh` (harness in `install/tests/lib/`) | Hand-rolled bash framework (`install/tests/lib/framework.sh`) | `bash install/test_mowglinext.sh` · `for t in install/tests/test_*.sh; do bash "$t" \|\| echo "FAILED: $t"; done` | **NONE** |
| Landing page / bootstrap static checks | `docs/test_install.sh` (over `docs/install.sh`), `docs/test_web_composer.sh` (over `docs/index.html`) | Static grep assertions | `bash docs/test_install.sh` · `bash docs/test_web_composer.sh` | **NONE** — and both exit 1 at this SHA (2 failures each) |
| Simulation E2E — LiDAR | `ros2/src/e2e_test.py` | rclpy script, self-scored | `cd ros2 && make e2e-test` | **NONE** |
| Simulation E2E — GPS-only | `ros2/src/e2e_test_no_lidar.py` | rclpy script, self-scored | `cd ros2 && make e2e-test-no-lidar` | **NONE** |
| STM32 firmware | none — `firmware/stm32/ros_usbnode/test/` holds only PlatformIO's placeholder `README` | (no `pio test` env defined in `platformio.ini`) | — | `firmware-ci.yml` build-checks only (`pio run -e Yardforce500` / `-e Yardforce500B`) |

`ros2/scripts/e2e_test.py` is an **older, separate** harness from `ros2/src/e2e_test.py` (different file, different metrics); the Makefile targets drive the `src/` pair only.

Two more piles of test files exist but **never execute**: the 62 CTest suites in the universal-gnss submodule's non-ROS libs (`gnss_core/`, `gnss_protocols/`, `gnss_driver/`, `gnss_transport/`, `gnss_ntrip/`, `gnss_tools/` — `gnss_ros2/CMakeLists.txt:31–66` forces `BUILD_TESTING OFF` around their `add_subdirectory` calls, so only `gnss_ros2`'s own 6 survive colcon), and the upstream `ros2/src/opennav_coverage/*/test/*.cpp` suites, whose packages are `COLCON_IGNORE`'d (only `opennav_coverage_msgs` is built).

---

## CI workflows

16 workflow files in `.github/workflows/`. Most PR-facing gates target `main` and/or `dev` — `firmware-ci.yml` has no base-branch filter at all, and `gui-docker.yml` also lists the `feat/**`-style prefixes.

| Workflow | Trigger | Jobs | Builds / tests / lints | Required check | Notes |
|---|---|---|---|---|---|
| `ros2-ci.yml` | push: `main`, `dev`, `feat/**`, `fix/**`, `refactor/**`, `chore/**`, `perf/**` on `ros2/**`, `tools/motor/**`, `install/config/mowgli/**`, self · PR: `[main, dev]` **with no `paths:` filter** | `changes`, `config-drift`, `build-and-test`, `format-check`, `static-analysis` | Full `colcon build` (Release) + `colcon test` + `colcon test-result --verbose`; GTSAM 4.3a1 and Fields2Cover v3 `@884d895` built from source and cached; `mowgli_robot.yaml` drift gate; clang-format-18 changed-lines gate; cppcheck | **`Build & Test (ROS2 kilted)`** (protected on `dev`) | The PR trigger has no `paths:` on purpose — a workflow skipped by a trigger filter reports *nothing* and leaves the required check pending forever. Path filtering moved into the `changes` job gate (L43–84); an `if:`-skipped job *does* satisfy branch protection. Do **not** add a `strategy.matrix` to `build-and-test`: it rewrites the published check name (L115–127). Checks out `submodules: recursive` (`opennav_coverage_msgs` must exist for `rosdep --ignore-src`), and `COLCON_IGNORE`s the five upstream `opennav_coverage` server subpackages (L302–307) plus `--skip-keys` for them (L321). |
| `gui-ci.yml` | push (same branch set) + PR `[main, dev]`, paths `gui/web/**`, self; `workflow_dispatch` | `unit-tests` | `yarn install --frozen-lockfile` → `npx tsc --noEmit` → `yarn lint` → `yarn test` (vitest) | `Unit Tests (vitest + tsc)` | Node **22**, pinned to match `gui/Dockerfile`'s web build stage. Deliberately separate from `gui-docker.yml` so test feedback does not wait on an emulated arm64 image build. Nothing here runs `go test`. |
| `msg-codegen-drift.yml` | push (same branch set) · **PR: `branches: [main]` only**; paths `ros2/src/mowgli_interfaces/**`, `gui/generate_*.sh`, `gui/pkg/msgs/**`, `gui/web/src/types/ros.generated.ts`, `firmware/scripts/sync_ros_lib.py`, firmware `ros_lib/mower_msgs/**`, self | `codegen-drift` | `sync_ros_lib.py --check`, then re-runs `gui/generate_go_msgs.sh` + `gui/generate_ts_types.sh` and `git diff --exit-code` on `gui/pkg/msgs` and `gui/web/src/types/ros.generated.ts` | `Codegen Drift (Go / TS / firmware msg types)` | Pure Python + bash, no toolchain. A `.msg` change without regenerating leaves the GUI reading the wrong JSON keys and the firmware (de)serializing a shifted byte layout. |
| `protocol-version-drift.yml` | push (same branch set) · **PR: `branches: [main]` only**; paths `mowgli_protocol.h`, `ll_datatypes.hpp`, the guard script + baseline, self | `protocol-version-drift` | `firmware/scripts/protocol_version_guard.py --check` — fingerprints the wire-defining region and fails if a `pkt_*_t`/`PKT_ID_*` changed without a `MOWGLI_PROTOCOL_VERSION` bump, or if firmware/host versions fell out of lockstep | `Protocol Version Drift (COBS wire vs MOWGLI_PROTOCOL_VERSION)` | — |
| `firmware-ci.yml` | push `main`, `dev`, `feat/**`, `fix/**`, `refactor/**`, `chore/**`, `perf/**` + tags `v*.*.*` · PR (any base), paths `firmware/**`, self | `defaults-parity`, `build` (matrix `Yardforce500`, `Yardforce500B`), `release` | `board_defaults_parity.py`; `pio run -e <env>` per board; on a `v*.*.*` tag, `package_release.py` attaches `dist/*.bin`, `dist/*.elf`, `dist/manifest.json` to the GitHub Release | `board_defaults single-source guard`, `Publish prebuilt firmware` | The release job needs `fetch-depth: 0` so `git rev-list`-based build IDs resolve. No firmware unit tests run. |
| `ros2-docker.yml` | push (same branch set) + tags `v*.*.*`, paths `ros2/**`, `tools/motor/**`, self; `workflow_dispatch` (`no-cache`) | `build` (amd64 + arm64 native runners), `merge`, `notify-failure` | Builds `ghcr.io/<repo>/mowgli-ros2` `runtime` target, then a **smoke test inside the image**: `ros2 pkg prefix mowgli_bringup` / `mowgli_tools`, `tune_drive_pid --help` contains "Controlled drive PID tuning", and `full_system.launch.py --show-args` does **not** expose `use_universal_gnss` | — | `sbom: false` — syft's SPDX doc exceeds buildkit's hardcoded 40 MiB attestation cap; `provenance` stays on. `notify-failure` opens a `ci-failure` issue only on `main`. |
| `gui-docker.yml` | push + PR (same branch set incl. `feat/**` etc.), paths `gui/**`, self; `workflow_dispatch` (`no-cache`) | `build` (amd64 + arm64), `merge` | Builds/pushes `ghcr.io/<repo>/mowglinext-gui`; on PRs `outputs: type=cacheonly` (no push) | — | The image build runs `yarn build` (`tsc && vite build`), so it typechecks — it does not run the tests. |
| `_sensor-docker.yml` | `workflow_call` only | `build`, `merge` | Reusable multi-arch sensor image builder; optional `smoke-test` script run inside the built image | — | Sensor images keep `sbom: true`. One thin caller per sensor. |
| `sensors-gps.yml` | push (same branch set), paths `sensors/gps/**`, `sensors/README.md`, `ros2/src/mowgli_interfaces/**`, `ros2/src/external/universal-gnss/**`, self, `_sensor-docker.yml` | via `_sensor-docker.yml` | Builds the `gps` image; the only sensor caller with a real smoke test (L37–66): asserts `mowgli_interfaces`, `universal_gnss_ros2` (`receiver_node`, `ntrip_node`) and `mowgli_gnss_bridge` are importable, and that `GnssStatus.CAP_RTK_MODE`, `RtcmFrame.data`, `rtcm_msgs/Message.message` exist | — | — |
| `sensors-lidar-ldlidar.yml` / `-rplidar.yml` / `-stl27l.yml` | push (same branch set), that sensor's dir + self + `_sensor-docker.yml` | via `_sensor-docker.yml` | Build only, no smoke test | — | `ldlidar` and `stl27l` build `target: runtime`; `rplidar` has no target. |
| `pages.yml` | push `main`, paths `docs/**`; `workflow_dispatch` | `build`, `deploy` | Publishes `docs/` to GitHub Pages (mowgli.garden) | — | `concurrency: pages`, `cancel-in-progress: false`. Does **not** run `docs/test_install.sh` or `docs/test_web_composer.sh`. |
| `wiki-sync.yml` | push `main`, paths `wiki/**`; `workflow_dispatch` | `sync` | Copies `wiki/*.md` into the `<repo>.wiki` repo and commits | — | Needs `contents: write`. |
| `auto-label.yml` | `pull_request_target` (opened, synchronize) | `label` | `actions/labeler@v6` with `.github/labeler.yml` | — | — |
| `welcome.yml` | `pull_request_target` (opened), `issues` (opened) | `welcome` | First-interaction greeting | — | — |

### Version pins that must move together

- **clang-format 18** — `ros2-ci.yml:419` installs `clang-format-18`; `ros2/scripts/format.sh:12` sets `REQUIRED_MAJOR=18` but only **warns** on a mismatch. A locally-installed clang-format 19+/22 reformats files CI never asked about.
- **GTSAM 4.3a1** — `ros2-ci.yml:191` cache key `gtsam-4.3a1-…` ↔ `ros2/Dockerfile` stage 0 ↔ `.devcontainer/Dockerfile`. The Ubuntu apt 4.2 package ships a broken `GTSAMConfig.cmake` and the legacy custom-factor API.
- **Fields2Cover v3 @ `884d895b59192882476e986ba44ea9143a06a6a9`** → `/opt/fields2cover-300`; `ros2-ci.yml:245` cache key `f2c-3.0.0-884d895-…` ↔ `ros2/Dockerfile` ↔ `mowgli_coverage/CMakeLists.txt`'s `find_package(Fields2Cover 3.0.0 … PATHS /opt/fields2cover-300)`.
- **Node 22** — `gui-ci.yml:45–50` ↔ `gui/Dockerfile` web build stage.

### Gaps worth knowing

- **`static-analysis` is `continue-on-error: true`** (`ros2-ci.yml:472`) — cppcheck findings never fail a PR; they only land in the log and the `cppcheck-report` artifact.
- **`msg-codegen-drift.yml` and `protocol-version-drift.yml` have `pull_request: branches: [main]` only.** A PR into `dev` gets them only through the `push` trigger on the source branch — a branch name outside `feat|fix|refactor|chore|perf/**` (e.g. `codex/…`) matches neither, so both gates can be silently absent. `ros2-ci.yml` solved exactly this with its `changes` job.
- **No workflow runs `go test`, Playwright, the installer suite, the `docs/` static checks, or the simulation E2E.**

---

## Running ROS2 tests locally

Only what the repo actually ships — the devcontainer (`.devcontainer/`), `ros2/Makefile`, and `ros2/scripts/`.

**Workspace layout:** the devcontainer bind-mounts the repo at `/ros2_ws/src/mowglinext` and `ros2/scripts/sync_workspace_packages.sh` symlinks the intended package roots into `/ros2_ws/src` (so nested `package.xml` files from the monorepo and submodules do not leak into colcon discovery). `build.sh`/`test.sh` hardcode `WORKSPACE=/ros2_ws`, so **they only work inside the devcontainer**.

```bash
# --- Build (devcontainer) ---
cd /Users/cedric/orca/workspaces/mowglinext/doc-update/ros2
make build-full          # BUILD_TYPE=Release ./scripts/build.sh  (whole workspace)
make build-dev           # focused set: mowgli_interfaces mowgli_localization universal_gnss_ros2 mowgli_bringup
make build-pkg PKG=mowgli_behavior
make build-debug         # BUILD_TYPE=Debug
make clean               # rm -rf build/ install/ log/

# scripts/build.sh honours PACKAGES + PACKAGES_MODE (up-to | select)
PACKAGES="mowgli_behavior mowgli_nav2_plugins" ./scripts/build.sh
PACKAGES="mowgli_map" PACKAGES_MODE=select ./scripts/build.sh
```

```bash
# --- Test ---
# `make test` -> ./scripts/test.sh is BROKEN at this SHA: test.sh:52 reads
# ${PACKAGES} under `set -u` (test.sh:18) and never defaults it the way
# build.sh:27 does, so it dies with "PACKAGES: unbound variable".
# Two working forms:
PACKAGES="" ./scripts/test.sh                      # whole workspace
PACKAGES="mowgli_behavior" ./scripts/test.sh       # one package

# Or drive colcon directly (this is what CI does):
source /opt/ros/kilted/setup.bash && source install/setup.bash
colcon test --return-code-on-test-failure --event-handlers console_cohesion+
colcon test-result --verbose
colcon test --packages-select fusion_graph --return-code-on-test-failure
```

```bash
# --- CI gates you can run with NO ROS2 toolchain ---
python3 -m pip install pyyaml pytest
python3 ros2/scripts/check_config_drift.py                    # ros2-ci config-drift
python3 -m pytest -q ros2/scripts/test_check_config_drift.py  # ros2-ci config-drift
python3 firmware/scripts/sync_ros_lib.py --check              # msg-codegen-drift
python3 firmware/scripts/protocol_version_guard.py --check    # protocol-version-drift
python3 firmware/scripts/board_defaults_parity.py             # firmware-ci defaults-parity
python3 -m pytest tools/motor/test                            # both drive-PID files
```

Reference: [`commands.md`](commands.md) for the code-generation workflow after a `.msg`/`.srv` change, and `ros2/README.md` § *Building* / § *Running Tests* for the from-scratch `rosdep` + `colcon build` recipe.

---

## Simulation & E2E

Two rclpy harnesses, both driven from `ros2/Makefile`, both **ungated by CI** — they need a running Webots sim, which no runner provides.

```bash
cd ros2
make sim                 # sim-stop, then sim_full_system.launch.py headless:=true use_rviz:=false
make sim-stop            # scripts/sim-stop.sh — kills ROS2/Webots and clears DDS shm; MUST run before a new sim
make e2e-test            # self-contained: sim-stop -> build -> launch sim -> sleep 90 -> python3 src/e2e_test.py -> sim-stop
make e2e-test-no-lidar   # same, with use_lidar:=false simulate_gps_degradation:=false -> src/e2e_test_no_lidar.py
```

Both `e2e-test` targets depend on `sim-stop build`, launch the sim themselves, and wait a fixed **90 s** for Nav2 to activate before running the script — you do **not** need a sim already running.

**`ros2/src/e2e_test.py`** (LiDAR path) sends `COMMAND_START` and spins for up to **1200 s**, scoring 13 criteria: undock→plan→mow→dock cycle, path tracking (median < 50 cm), SLAM map growth, no collisions, stayed within boundary, obstacle avoidance (against three obstacles pre-placed in the world), mowing efficiency ≥ 0.85, area coverage ≥ 80 %, idle ratio < 20 %, path overlap < 30 %, manual mowing mode, area recording mode, and emergency auto-reset on dock. It prints `OVERALL: PASS / NEEDS ATTENTION`.

**`ros2/src/e2e_test_no_lidar.py`** (GPS-only path) scores two criteria — undock→plan→mow→dock cycle and path tracking (median < 10 cm) — and `sys.exit(0 if overall else 1)`.

**Simulator requirements:** Webots via `webots_ros2_driver` (`mowgli_simulation/package.xml`), 8 CPU cores minimum / 16 recommended (`.devcontainer/devcontainer.json` `hostRequirements` + the comment above it), and an X display for headless — `ros2/Makefile` exports `DISPLAY=:99` but does **not** start Xvfb; or the `docker/docker-compose.simulation.yaml` `simulation` / `simulation-gui` / `dev-sim` services. Before touching anything under `mowgli_simulation/worlds_webots/`, `protos/`, `urdf_webots/`, or `kinematic_drive.py`, read [`../WEBOTS_SIM.md`](../WEBOTS_SIM.md) — it documents five load-bearing ODE workarounds, each of which presents as a *Nav2* bug when broken.

**GUI browser E2E** is a separate, much cheaper suite: `cd gui/web && yarn test:e2e` runs Playwright against the vite dev server with a fully mocked Go backend (REST via `page.route`, the multiplex WebSocket via `page.routeWebSocket`, scenarios in `tests/e2e/mock/scenarios.ts`). No robot, no backend, no CI job. See `gui/web/tests/e2e/README.md`.

---

## Session monitoring

Any test that makes the **real robot move** (a `COMMAND_START`, an undock, a tuning run) should be recorded in parallel with `ros2/scripts/mow_session_monitor.py`, which writes a 10 Hz JSONL timeline — fused pose + covariance, TF snapshots, wheel/IMU/GPS raw, BT state, `cmd_vel_nav` vs `cmd_vel`, Nav2 plan geometry, `/fusion_graph/diagnostics`, and cross-source consistency checks (fusion↔GPS distance, wheel↔gyro yaw drift, RTK covariance-drop health) — plus a metadata header stamping the git commit, image tags and config hashes so runs from different tunings are comparable. Full invocation, output-directory caveats and the field list are in [`session-monitoring.md`](session-monitoring.md); post-hoc analysis helpers live in `ros2/scripts/diagnostics/` (`analyze_session.py`, `session_timeline.py`).

---

## Lint & format gates

| Tool | Command | Config | Enforced where |
|---|---|---|---|
| clang-format (18) | `cd ros2 && make format` (in-place) · `make format-check` (whole tree, `--dry-run --Werror`) · `./scripts/format.sh [--check]` | `ros2/.clang-format` | `ros2-ci.yml` → `format-check`, but **only on changed lines**: `git-clang-format-18 --binary clang-format-18 --style=file:ros2/.clang-format --diff $(git merge-base origin/main HEAD) -- ros2/src`. The legacy tree is not whole-tree clean, so `make format-check` is redder than CI. |
| clang-format (pre-push) | `git config core.hooksPath .githooks` (opt-in; nothing installs it) | `.githooks/pre-push` | Local only. Runs `format.sh --check` (**whole tree**) and, on failure, formats and **auto-creates a `chore: auto-format C++ files` commit**. With any clang-format major ≠ 18 this rewrites files CI never flagged. |
| cppcheck | `cd ros2 && make lint` (`--enable=all`) | — | `ros2-ci.yml` → `static-analysis` uses narrower flags (`--enable=warning,performance,portability --inline-suppr`, suppressing `missingInclude`, `unmatchedSuppression`, `useInitializationList`, `normalCheckLevelMaxBranches`), scoped to changed `.cpp`/`.hpp` outside `*/test/*` and `*/opennav_coverage/*`, and is **`continue-on-error`**. |
| cpplint | `cd ros2 && make lint` | `ros2/CPPLINT.cfg` (`linelength=100`, filters `-whitespace/newline,-runtime/string,-build/namespaces,-build/include_order`) | Local `make lint` only. Packages also `set(ament_cmake_cpplint_FOUND TRUE)` in `BUILD_TESTING`, i.e. the ament cpplint hook is **disabled** in `colcon test`. |
| ament lint (copyright/uncrustify) | — | `set(ament_cmake_copyright_FOUND TRUE)` / `set(ament_cmake_uncrustify_FOUND TRUE)` in nearly every package's `BUILD_TESTING` block (uncrustify in all 11 that have one; copyright in 10 — `mowgli_behavior` omits it) | Deliberately disabled — clang-format is the formatter of record. |
| ESLint 9 (flat config) | `cd gui/web && yarn lint` (= `eslint src --report-unused-disable-directives --max-warnings 900`) · `yarn lint:fix` | `gui/web/eslint.config.js` | `gui-ci.yml` → `unit-tests`. **The hard gate is "0 errors"** — eslint exits non-zero on any error regardless of `--max-warnings`. The 900 ceiling is a second-order debt ratchet over a measured 799 warnings (2026-08-20); lowering it is the point, raising it needs a written reason. `src/types/ros.generated.ts` is ignored (it is codegen-drift-gated instead). `src/test/eslint-config.test.ts` is a vitest guard that the blocking rules are still blocking. |
| TypeScript | `cd gui/web && npx tsc --noEmit` | `gui/web/tsconfig.json` | `gui-ci.yml` → `unit-tests` (and implicitly by `yarn build` = `tsc && vite build` inside `gui-docker.yml`). |
| vitest | `cd gui/web && yarn test` (= `vitest run`) · `yarn test:watch` | `gui/web/vitest.config.ts` (jsdom, `testTimeout: 20000`, `tests/e2e/**` excluded) | `gui-ci.yml` → `unit-tests`. |
| Prettier | — | — | **Not installed and not configured anywhere.** No `prettier` dependency, no `.prettierrc`, no `format` script. Several docs still say to run it. |
| gofmt / go vet | `cd gui && gofmt -l .` · `go vet ./...` | — | **Nothing enforces them.** No workflow contains `gofmt`, `go vet`, `go test` or `setup-go`. |

---

## Before opening a PR

**PRs land on `dev`** (`gh pr create --base dev`); `main` is the release branch. `dev` carries branch protection requiring one approving review, and the required status check on it is literally named **`Build & Test (ROS2 kilted)`**.

Grounded in the workflows, in the order things fail:

1. **Branch name matters.** Use `feat/`, `fix/`, `refactor/`, `chore/` or `perf/`. `msg-codegen-drift.yml` and `protocol-version-drift.yml` only run on PRs into `main`; on a `dev` PR they fire solely from the `push` trigger, whose branch list is exactly that set. A `codex/…`-style name gets neither gate.
2. **Touched `ros2/src/mowgli_interfaces/**`?** Regenerate all three consumers or `msg-codegen-drift.yml` fails on `git diff --exit-code`: `python3 firmware/scripts/sync_ros_lib.py`, `cd gui && ./generate_go_msgs.sh`, `cd gui && ./generate_ts_types.sh`. Regenerate with `LC_ALL=C` — macOS sort order fabricates ~20 lines of phantom drift.
3. **Touched the COBS wire** (`mowgli_protocol.h`, `ll_datatypes.hpp`)? Bump `MOWGLI_PROTOCOL_VERSION` **and** its host mirror `kMowgliProtocolVersion`, then `python3 firmware/scripts/protocol_version_guard.py --check`. Otherwise `protocol-version-drift.yml` fails.
4. **Touched `mowgli_robot.yaml`?** `python3 ros2/scripts/check_config_drift.py`. `ros2-ci.yml` → `config-drift` fails on a structural field defined in both files with divergent values, a key added to the installed file with no template default, **or** a sparse-file key padded with a value equal to its template default (Invariant 15).
5. **Touched C++?** Format the changed lines with clang-format **18**: `git-clang-format-18 --binary clang-format-18 --style=file:ros2/.clang-format --diff $(git merge-base origin/main HEAD) -- ros2/src`. `format-check` fails otherwise. Do not `--no-verify` past the pre-push hook with a non-18 clang-format installed — it amends a formatting commit CI will then disagree with.
6. **Touched `ros2/**` or `tools/motor/**`?** The whole workspace must build and `colcon test` must pass — that is the required check. Run it locally first (`PACKAGES="" ./scripts/test.sh`, or `colcon test --return-code-on-test-failure`).
7. **Touched `gui/web/**`?** `cd gui/web && npx tsc --noEmit && yarn lint && yarn test` — all three run in `gui-ci.yml` and any one of them fails the job. Zero eslint **errors** is the gate.
8. **Touched Go under `gui/pkg/**`?** `cd gui && go test ./...` — **no workflow will catch a Go regression for you.** Same for `gofmt`.
9. **Touched `install/**`?** `bash install/test_mowglinext.sh` and `for t in install/tests/test_*.sh; do bash "$t"; done` — ungated. (`ros2-ci.yml` watches only `install/config/mowgli/**`, and only for the config-drift job.)
10. **Touched `docs/install.sh` or `docs/index.html`?** `bash docs/test_install.sh` / `bash docs/test_web_composer.sh` — ungated, and both already fail at this SHA (2 assertions each), so compare against the current baseline rather than expecting green.
11. **Touched `firmware/**`?** `pio run -e Yardforce500 && pio run -e Yardforce500B` from `firmware/stm32/ros_usbnode`, plus `python3 firmware/scripts/board_defaults_parity.py`. There are no firmware unit tests.
12. **Changed physical behaviour?** Flag it as safety-critical in the PR description (root `CLAUDE.md` § Safety), tick the right boxes in `.github/PULL_REQUEST_TEMPLATE.md`, and — if it is a motion change — run a sim E2E (`make e2e-test`) or a monitored field session before merging.
