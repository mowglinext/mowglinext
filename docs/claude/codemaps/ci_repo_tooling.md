# Codemap: ci_repo_tooling

> Everything that gates, builds, labels and publishes the repo from outside the runtime stack: the 16 GitHub Actions workflows (`.github/workflows/`), the drift gates (config / msg-codegen / protocol-version), the GHCR image pipelines, the wiki + GitHub Pages publishers, the repo metadata (CODEOWNERS, labeler, issue/PR templates), the `.githooks/pre-push` formatter, the Codespaces/devcontainer image, the agent config under `.claude/` + `AGENTS.md`, the `tools/motor` sidecar ROS2 package (`mowgli_tools` drive-PID tuner), and the mowgli.garden landing page + one-line install composer under `docs/`.
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `CLAUDE.md`.

## Where to look

| Task | Start here |
|------|------------|
| A PR is stuck on a pending required check | `.github/workflows/ros2-ci.yml` L11–23 + L115–127 — `Build & Test (ROS2 kilted)` is the required check on `dev`; the `pull_request` trigger has NO `paths:` filter and the job has NO `strategy.matrix` on purpose (both broke the published check name) |
| Skip ROS2 CI on a non-ROS2 PR without breaking branch protection | job `changes` (`ros2-ci.yml` L43–84) — path filtering moved from the trigger into an `if:` job gate; outputs `ros2` |
| Add / change a ROS2 CI step | `ros2-ci.yml` job `build-and-test` L128–401 (GTSAM L186–231, F2C L240–287, rosdep L289–322, colcon build L335–341, colcon test L343–350) |
| clang-format version pin / "changed lines only" formatting gate | `ros2-ci.yml` job `format-check` L404–445 — installs `clang-format-18`, runs `git-clang-format-18 --style=file:ros2/.clang-format --diff $(git merge-base origin/main HEAD) -- ros2/src` |
| cppcheck findings in CI | `ros2-ci.yml` job `static-analysis` L448–515 — `continue-on-error: true`, changed files only, flags at L497–505 |
| `mowgli_robot.yaml` drift / padded-default failure | `ros2-ci.yml` job `config-drift` L96–113 → `ros2/scripts/check_config_drift.py` (CLAUDE.md Invariant 15) |
| Go/TS/firmware message types out of sync after a `.msg` change | `.github/workflows/msg-codegen-drift.yml` L49–69 — `firmware/scripts/sync_ros_lib.py --check`, then regenerates and `git diff --exit-code` on `gui/pkg/msgs` and `gui/web/src/types/ros.generated.ts` |
| COBS wire changed but `MOWGLI_PROTOCOL_VERSION` not bumped | `.github/workflows/protocol-version-drift.yml` L42–43 → `firmware/scripts/protocol_version_guard.py --check` |
| GUI tests / lint / typecheck gate | `.github/workflows/gui-ci.yml` L58–76 (`npx tsc --noEmit`, `yarn lint`, `yarn test`); node 22 pinned L45–50 |
| Firmware build matrix + prebuilt release binaries | `.github/workflows/firmware-ci.yml` jobs `defaults-parity` L25–34, `build` L36–62 (`Yardforce500`, `Yardforce500B`), `release` L71–123 (tag `v*.*.*`) |
| Publish a new sensor container image | add a thin caller like `.github/workflows/sensors-lidar-rplidar.yml` calling `_sensor-docker.yml` (inputs: `image`, `context`, `dockerfile`, `target`, `smoke-test`, `no-cache`) |
| Multi-arch ROS2 / GUI image + manifest merge | `.github/workflows/ros2-docker.yml` (jobs `build`/`merge`/`notify-failure`), `.github/workflows/gui-docker.yml` (jobs `build`/`merge`) |
| Landing page / wiki did not update after a merge | `.github/workflows/pages.yml` (push to `main`, paths `docs/**`) and `.github/workflows/wiki-sync.yml` (push to `main`, paths `wiki/**`, copies `wiki/*.md` into the `.wiki` repo) |
| PR auto-labels / first-time-contributor greeting | `.github/workflows/auto-label.yml` + `.github/labeler.yml`; `.github/workflows/welcome.yml` (both `pull_request_target`) |
| C++ gets auto-committed on push | `.githooks/pre-push` L20–28 — runs `ros2/scripts/format.sh --check` (WHOLE tree, `--Werror`) and, on failure, formats and creates a `chore: auto-format C++ files` commit. Opt-in: `git config core.hooksPath .githooks` |
| Codespaces / devcontainer image contents | `.devcontainer/Dockerfile` (stage `gtsam-builder` L35–61, apt layer L71–176, F2C v2 L178–188, Go 1.24 L202–212, node 22 L217–220) |
| Devcontainer workspace linking / first build | `.devcontainer/post-create.sh` (L36–38 `sync_workspace_packages.sh`, L90 `DEV_PACKAGES`, L91 `MOWGLI_POST_CREATE_BUILD=0` default) |
| USB `/dev/serial/by-id` missing in the devcontainer | `.devcontainer/post-start.sh` — symlinks `/host-dev/serial/by-id` → `/dev/serial/by-id` |
| Tune drive-wheel PID on a real robot | `tools/motor/README.md`, then `tools/motor/mowgli_tools/drive_pid_tuner.py` `_build_parser` L174, `DrivePidTuner.run` L433 |
| Change PID recommendation math / oscillation classification | `tools/motor/mowgli_tools/drive_pid_math.py` (`classify_oscillation` L584, `compute_trial_metrics` L715, `recommend_drive_pid_params` L1279, `recommend_pid_only_params` L992, `resolve_robot_tuning_tier` L146) |
| Change the one-line installer flags | `docs/install.sh` (flag parse L102–167, help L145–161, forwarding L223–324) + composer `docs/index.html` `updateCommand` L487–507 |
| Agent permissions / enabled plugins for this repo | `.claude/settings.json` (`permissions.allowedTools` L3–25, `enabledPlugins` L27–44) |

## Files

| File | Lines | Purpose |
|------|-------|---------|
| **`.github/workflows/`** | | |
| `ros2-ci.yml` | 516 | The ROS2 gate: jobs `changes`, `config-drift`, `build-and-test` (**required check** `Build & Test (ROS2 kilted)`), `format-check`, `static-analysis` |
| `ros2-docker.yml` | 234 | GHCR `mowgli-ros2` runtime image, native amd64 + arm64 runners, digest merge, failure-issue bot |
| `_sensor-docker.yml` | 219 | Reusable multi-arch sensor-image builder (`workflow_call`); one thin caller per sensor |
| `gui-docker.yml` | 166 | GHCR `mowglinext-gui` image; PR builds are `type=cacheonly` (no push) |
| `firmware-ci.yml` | 123 | `board_defaults_parity.py` guard, PlatformIO build matrix, tagged prebuilt-binary release |
| `gui-ci.yml` | 76 | vitest + `tsc --noEmit` + eslint on `gui/web/**` (node 22) |
| `msg-codegen-drift.yml` | 70 | Firmware `ros_lib` + Go + TS message-type drift gate |
| `sensors-gps.yml` | 67 | Builds `gps` image from `sensors/gps/Dockerfile`; long in-image smoke test (L37–66) |
| `wiki-sync.yml` | 50 | Copies `wiki/*.md` into the GitHub wiki repo and commits as `github-actions[bot]` |
| `pages.yml` | 43 | Uploads `docs/` and deploys to GitHub Pages (env `github-pages`) |
| `protocol-version-drift.yml` | 43 | COBS wire fingerprint vs `MOWGLI_PROTOCOL_VERSION` |
| `sensors-lidar-ldlidar.yml`, `sensors-lidar-stl27l.yml` | 32, 32 | Callers with `target: runtime` |
| `sensors-lidar-rplidar.yml` | 31 | Caller with no `target` |
| `welcome.yml` | 32 | `actions/first-interaction@v1` greeting on first issue/PR |
| `auto-label.yml` | 17 | `actions/labeler@v6` using `.github/labeler.yml` |
| **`.github/` metadata** | | |
| `scripts/install-ros-apt-source.sh` | 89 | Resolves the `ros2-apt-source` `.deb` from the GitHub release API (`--print-url` mode) instead of synthesizing the filename |
| `ISSUE_TEMPLATE/bug_report.yml` | 67 | Component dropdown, repro, hardware, logs; label `bug` |
| `ISSUE_TEMPLATE/feature_request.yml` | 56 | Component, motivation, proposal; label `enhancement` |
| `ISSUE_TEMPLATE/config.yml` | 8 | `blank_issues_enabled: true` + links to mowgli.garden and Discussions |
| `PULL_REQUEST_TEMPLATE.md` | 38 | Summary / Changes / Component / Testing / Checklist |
| `labeler.yml` | 28 | Path→label: `ros2`, `gui`, `docker`, `sensors`, `firmware`, `docs`, `ci` |
| `CODEOWNERS` | 10 | `*` and all six components → `@cedbossneo` |
| `FUNDING.yml` | 1 | `github: cedbossneo` |
| **`.githooks/`** | | |
| `pre-push` | 29 | Opt-in formatter hook; warns and exits 0 if `clang-format` or `ros2/scripts/format.sh` is missing (L9-17) |
| **`.devcontainer/`** | | |
| `Dockerfile` | 266 | Codespaces/devcontainer image: GTSAM 4.3a1 from source, Nav2/ROS apt set, F2C **v2.0.0** → `/opt/fields2cover-200`, ros-gz, Go 1.24, node 22, ruff/pre-commit, gh CLI, Cyclone DDS config |
| `devcontainer.json` | 126 | Mounts repo at `/ros2_ws/src/mowglinext`, `--privileged`, `/dev`→`/host-dev` (ro), ports 4006/8765/6080/9090, features `claude-code` + `docker-outside-of-docker` |
| `post-create.sh` | 129 | Cleans stale `src/{install,build,log}`, inits submodules, runs `ros2/scripts/sync_workspace_packages.sh`, rosdep, optional focused build |
| `post-start.sh` | 35 | Re-exposes the host's `serial/by-id` symlinks inside the container |
| **`tools/motor/`** (colcon package `mowgli_tools`) | | |
| `mowgli_tools/drive_pid_tuner.py` | 2207 | `DrivePidTuner(Node)` — the live tuning session: profiles, backup/rollback, undock, BT `RECORDING` entry, speed trials, report |
| `mowgli_tools/drive_pid_math.py` | 1292 | Pure math/decision layer (no ROS): metrics, oscillation/stall classification, mass tiers, PID + feed-forward recommendations |
| `test/test_drive_pid_math.py` | 1061 | 35 pytest cases over the pure layer |
| `README.md` | 83 | Operator usage, flags, preset rationale (`1964 / 3` ticks/m) |
| `mowgli_tools/robot_hardware_config.py` | 51 | Reads `chassis_mass_kg` / `wheel_radius` / `ticks_per_revolution` out of a `mowgli_robot.yaml`-shaped payload |
| `test/test_drive_pid_tuner.py` | 31 | 2 cases over `extract_robot_hardware_config` — **not registered in `CMakeLists.txt`** |
| `CMakeLists.txt` | 30 | `ament_python_install_package`, installs `scripts/tune_drive_pid`, registers only `test_drive_pid_math` |
| `package.xml` | 28 | `mowgli_tools`, ament_cmake, exec_deps rclpy / geometry_msgs / nav_msgs / sensor_msgs / mowgli_interfaces / python3-yaml |
| `scripts/tune_drive_pid` | 7 | Console entry point → `mowgli_tools.drive_pid_tuner:main` |
| `setup.cfg` | 3 | flake8 `max-line-length = 130` |
| `mowgli_tools/__init__.py` | 1 | package marker |
| **`.claude/` + root** | | |
| `.claude/settings.json` | 45 | `permissions.allowedTools` (colcon/ros2/docker/pio/clang-format/cppcheck/go/yarn/npm) + 16 `enabledPlugins` |
| `.claude/rules/ros2.md` | 58 | ROS2 node/QoS/topic/launch/testing/safety/ARM conventions (referenced from the root `CLAUDE.md`) |
| `.claude/memory/MEMORY.md` | 3 | Index of the three memory notes below |
| `.claude/memory/project_e2e_status.md` | 24 | Historical (2026-04-04) E2E sim status |
| `.claude/memory/feedback_sim_cleanup.md` | 15 | "Always `make sim-stop` before launching sim" |
| `.claude/memory/user_cedric.md` | 7 | Project-owner profile |
| `AGENTS.md` | 31 | Agent rules: clang-format-clean `ros2/` changes; never build as root |
| `.editorconfig` | 24 | 2-space default; py 4; Go/Makefile tabs; md keeps trailing whitespace |
| `.gitattributes` | 5 | `*.sh` and `*.py` forced to `eol=lf` (Windows checkouts) |
| **`docs/` (GitHub Pages)** | | |
| `docs/index.html` | 648 | mowgli.garden landing page + install composer (`state` L455, `updateCommand` L487, copy button L525) |
| `docs/install.sh` | 335 | Served at `https://mowgli.garden/install.sh`; clones/updates the repo then `exec`s `install/mowglinext.sh` with forwarded flags |
| `docs/test_web_composer.sh` | 90 | Static grep assertions over `docs/index.html` |
| `docs/test_install.sh` | 80 | Static grep assertions over `docs/install.sh` `--help` + source text |

Also deployed by `pages.yml` but not part of the composer logic: `docs/CNAME` (`mowgli.garden`), `docs/style.css`, `docs/logo.svg`, `docs/icon.svg`, `docs/screenshots/`, `docs/gui-walkthrough/screenshots/`, and the Markdown notes under `docs/`.

## CI surface

### Workflow triggers

| Workflow | push branches | push paths | pull_request | Other |
|----------|---------------|-----------|--------------|-------|
| `ros2-ci.yml` | main, dev, feat/**, fix/**, refactor/**, chore/**, perf/** | `ros2/**`, `tools/motor/**`, `install/config/mowgli/**`, self | **`[main, dev]`, no paths** (deliberate, L11–23) | `concurrency: ros2-ci-${{ github.ref }}` |
| `ros2-docker.yml` | same set + tags `v*.*.*` | `ros2/**`, `tools/motor/**`, self | — | `workflow_dispatch` input `no-cache` |
| `gui-ci.yml` | same set | `gui/web/**`, self | `[main, dev]`, same paths | `workflow_dispatch` |
| `gui-docker.yml` | same set | `gui/**`, self | same branch set + paths | `workflow_dispatch` input `no-cache` |
| `firmware-ci.yml` | same set + tags `v*.*.*` | `firmware/**`, self | any base, `firmware/**` | — |
| `msg-codegen-drift.yml` | same set | `mowgli_interfaces/**`, `gui/generate_*`, `gui/pkg/msgs/**`, `gui/web/src/types/ros.generated.ts`, `firmware/scripts/sync_ros_lib.py`, firmware `mower_msgs/**` | **`[main]` only** | — |
| `protocol-version-drift.yml` | same set | `mowgli_protocol.h`, `ll_datatypes.hpp`, `protocol_version_guard.py`, `protocol_baseline.json` | **`[main]` only** | — |
| `sensors-*.yml` | same set | that sensor's dir + self + `_sensor-docker.yml` (gps also `ros2/src/mowgli_interfaces/**`, `ros2/src/external/universal-gnss/**`) | — | `workflow_dispatch` |
| `pages.yml` | main | `docs/**` | — | `workflow_dispatch`; `concurrency: pages` |
| `wiki-sync.yml` | main | `wiki/**` | — | `workflow_dispatch`; needs `contents: write` |
| `auto-label.yml` / `welcome.yml` | — | — | `pull_request_target` (opened/synchronize) / (opened) + `issues` | — |

### Required / named checks
- `Build & Test (ROS2 kilted)` — required on `dev`. Do NOT rename the job, add a `strategy.matrix`, or move path filtering back onto the `pull_request` trigger (`ros2-ci.yml` L11–23, L115–127).
- Other job display names: `Detect ROS2 changes`, `Config Drift (mowgli_robot.yaml)`, `Formatting (clang-format)`, `Static Analysis (cppcheck)`, `Codegen Drift (Go / TS / firmware msg types)`, `Protocol Version Drift (COBS wire vs MOWGLI_PROTOCOL_VERSION)`, `board_defaults single-source guard`, `Publish prebuilt firmware`, `Unit Tests (vitest + tsc)`, `Notify on failure`.

### Published artifacts
| Target | Produced by | Tags |
|--------|-------------|------|
| `ghcr.io/<repo>/mowgli-ros2` (target `runtime`, `ros2/Dockerfile`) | `ros2-docker.yml` | `type=ref,event=branch`, semver, `sha-<short>` |
| `ghcr.io/<repo>/mowglinext-gui` (context `./gui`) | `gui-docker.yml` | same |
| `ghcr.io/<repo>/{gps,lidar-ldlidar,lidar-rplidar,lidar-stl27l}` | `_sensor-docker.yml` callers | same |
| GitHub Release assets `dist/*.bin`, `dist/*.elf`, `dist/manifest.json` | `firmware-ci.yml` `release` | tag `v*.*.*` |
| GitHub Pages site (mowgli.garden) | `pages.yml` | — |
| GitHub wiki | `wiki-sync.yml` | — |
| Actions artifacts `test-results-kilted`, `cppcheck-report`, `digests-*` | `ros2-ci.yml`, image workflows | 14 / 7 / 1-day retention |

Image names are lowercased into `GITHUB_ENV` in every image job (fork owners with mixed-case names). ROS2 images set `sbom: false` (syft SPDX exceeded buildkit's 40 MiB attestation cap); sensor images keep `sbom: true`. Build caches: `type=registry,ref=<image>:buildcache-{amd64,arm64}`.

### `mowgli_tools` runtime surface (`tools/motor`)
- Executable: `ros2 run mowgli_tools tune_drive_pid -- <flags>`; node name `tune_drive_pid`.
- Publishes `TwistStamped` on `/cmd_vel_tuning` by default (`_resolve_cmd_topic` L572; twist_mux lane priority 30), overridable with `--cmd-topic`.
- Subscribes: `/hardware_bridge/status`, `/hardware_bridge/emergency`, `/behavior_tree_node/high_level_status`, `/wheel_odom`, `/wheel_ticks`, `/gps/absolute_pose`, `/gps/status` (all RELIABLE), `/odometry/filtered_map` (BEST_EFFORT) — constructor L297–310.
- Service client `/behavior_tree_node/high_level_control` (`mowgli_interfaces/srv/HighLevelControl`); sends `HL_CMD_RECORD_AREA = 3` to enter, `HL_CMD_RECORD_CANCEL = 6` to leave (L97–101, `_enter_recording_if_needed` L1145).
- `AsyncParameterClient` against `--hardware-node` (default `hardware_bridge`) get/set of `PARAMETER_NAMES` = `ticks_per_meter`, `wheel_pid_kp`, `wheel_pid_ki`, `wheel_pid_kd`, `wheel_pid_integral_limit`, `wheel_pid_pwm_per_mps` (L88–95).
- Backup file: `~/.ros/mowgli_tools/drive_pid_last_backup.yaml` (`_default_backup_path` L166); `--rollback` restores it. `--apply` keeps values LIVE only — it never writes `mowgli_robot.yaml`.
- Presets `PROFILE_PRESETS` L75–86: `yardforce_8w_1964` (655 ticks/m, kp 20, ki 1000, ilim 40, pwm/mps 450) and `yardforce_12w_1600` / alias `yardforce_1600_12w` (533, 18, 700, 30, 550).
- Reads chassis mass for the tuning tier from the first existing of `--hardware-config`, `/ros2_ws/config/mowgli_robot.yaml`, `…/mowgli_bringup/config/mowgli_robot.yaml`, then parent-walk (`_robot_config_candidates` L723).

### Install composer surface (`docs/`)
- Composer state (`docs/index.html` L455): `backend` (`mowgli`|`mavros`), `gnssReceiverFamily` (`auto`|`ublox`|`unicore`|`nmea`), `gnssConnection` (`usb`|`uart`), `lidar`, `channel` (`main`|`dev`). Emits `--backend=`, `--gnss-receiver-family=`, `--gnss-connection=`, `--lidar=`, and `--branch=` only when channel ≠ `main`. `mavros` disables the GNSS group (`setGroupDisabled('gnss-group', mavrosSelected)` L474).
- `docs/install.sh` accepts `--backend`, `--gnss`, `--gnss-receiver-family`, `--gnss-connection`, `--gps` (deprecated, normalized L108–140), `--lidar`, `--tfluna` (deprecated), `--branch`, `--image-tag`, `--help`. Clones `https://github.com/mowglinext/mowglinext.git` into `${MOWGLI_HOME:-$HOME/mowglinext}` and `exec`s `install/mowglinext.sh` with `</dev/tty` reattached (L330–334).

## Build, test, run

```bash
# CI gates you can run locally (no ROS2 toolchain needed)
python3 ros2/scripts/check_config_drift.py                     # config-drift job
python3 -m pytest -q ros2/scripts/test_check_config_drift.py
python3 firmware/scripts/sync_ros_lib.py --check               # msg-codegen-drift, step 1
(cd gui && ./generate_go_msgs.sh && ./generate_ts_types.sh) && git diff --exit-code -- gui/pkg/msgs gui/web/src/types/ros.generated.ts
python3 firmware/scripts/protocol_version_guard.py --check     # protocol-version-drift
python3 firmware/scripts/board_defaults_parity.py              # firmware-ci defaults-parity
bash .github/scripts/install-ros-apt-source.sh --print-url ros2-apt-source noble

# Formatting exactly as CI does it (clang-format 18 only)
git-clang-format-18 --binary clang-format-18 --style=file:ros2/.clang-format \
  --diff "$(git merge-base origin/main HEAD)" -- ros2/src

# mowgli_tools
colcon test --packages-select mowgli_tools                     # only test_drive_pid_math is registered
python3 -m pytest tools/motor/test                             # runs BOTH test files (needs mowgli_tools importable)
ros2 run mowgli_tools tune_drive_pid --help

# docs/ static checks (NOT wired into any workflow)
bash docs/test_install.sh
bash docs/test_web_composer.sh

# GUI gate
(cd gui/web && yarn install --frozen-lockfile && npx tsc --noEmit && yarn lint && yarn test)
```

Where the tests run in CI: `tools/motor` is symlinked to `ros2/src/mowgli_tools` (`ros2-ci.yml` L151–159) and built/tested by the whole-workspace `colcon build` + `colcon test` in `build-and-test`; the same job then asserts `ros2 pkg prefix mowgli_tools` and that `tune_drive_pid --help` prints `Controlled drive PID tuning` (L352–358), and that `full_system.launch.py --show-args` does NOT expose `use_universal_gnss` (L360–391). `ros2-docker.yml` L108–120 repeats both assertions inside the built image. `docs/test_install.sh` / `docs/test_web_composer.sh` have no CI caller anywhere in `.github/`.

## Change coupling — "if you change X, also update Y"

- **Rename a job or add a matrix in `ros2-ci.yml`** → the `dev` branch-protection required-check name must change with it (`ros2-ci.yml` L115–127 spells out both past failure modes).
- **GTSAM / Fields2Cover pins**: `ros2/Dockerfile` (GTSAM 4.3a1; F2C v2 → `/opt/fields2cover-200`, v3 @ `884d895` → `/opt/fields2cover-300`) ↔ `ros2-ci.yml` L191 `gtsam-4.3a1-…` and L245 `f2c-3.0.0-884d895-…` cache keys ↔ `.devcontainer/Dockerfile` L45 / L178 ↔ `ros2/src/mowgli_coverage/CMakeLists.txt:42` `find_package(Fields2Cover 3.0.0 CONFIG REQUIRED PATHS /opt/fields2cover-300)`.
- **New `opennav_coverage` subpackage** → add it to the `COLCON_IGNORE` loop (`ros2-ci.yml` L302–307) AND the `rosdep --skip-keys` list (L321) AND the equivalent Dockerfile step (CLAUDE.md "Do NOT use the upstream `opennav_coverage` server").
- **`.msg`/`.srv` change in `mowgli_interfaces`** → regenerate all three consumers or `msg-codegen-drift.yml` fails; regenerate with `LC_ALL=C` so macOS sort order does not fake drift.
- **`pkt_*_t` / `PKT_ID_*` change** → bump `MOWGLI_PROTOCOL_VERSION` and the host mirror `kMowgliProtocolVersion`, then refresh `firmware/scripts/protocol_baseline.json` (`protocol-version-drift.yml`).
- **New top-level directory** → add a `.github/labeler.yml` rule and a `CODEOWNERS` entry; add a `Component` option to both issue templates and `PULL_REQUEST_TEMPLATE.md` if it is user-visible.
- **New installer flag** → `docs/index.html` composer state + `updateCommand` → `docs/install.sh` parse + forward → `install/mowglinext.sh` argument contract → assertions in `docs/test_install.sh` / `docs/test_web_composer.sh`.
- **New sensor image** → `sensors/<name>/` + a caller workflow modelled on `sensors-lidar-rplidar.yml` + the compose profile in `install/compose/` that pulls the tag.
- **New apt/tool dependency for local dev** → `.devcontainer/Dockerfile` AND the equivalent CI step in `ros2-ci.yml` (the two are hand-mirrored, not shared).
- **`ros2/scripts/format.sh` behaviour** → `.githooks/pre-push` (whole-tree `--check`) and `ros2-ci.yml` `format-check` (changed lines) diverge; changing one does not change the other.
- **New `hardware_bridge` PID parameter** → `tools/motor/mowgli_tools/drive_pid_tuner.py` `PARAMETER_NAMES` L88 + `DrivePidParams` (`drive_pid_math.py` L92) + the backup YAML schema (`_write_backup` L1088).

## Pitfalls

- `.devcontainer/Dockerfile` installs Fields2Cover **v2.0.0** into `/opt/fields2cover-200` (L178–188) and the apt `ros-kilted-fields2cover`, but `mowgli_coverage` requires `Fields2Cover 3.0.0`, searched under `PATHS /opt/fields2cover-300` (`ros2/src/mowgli_coverage/CMakeLists.txt:42-48` — `NO_DEFAULT_PATH` is deliberately NOT set; the 3.0.0 version pin alone rejects the retained v2 tree). `make build-full` inside a stock devcontainer fails at `mowgli_coverage`; post-create sidesteps it only by not building at all (`MOWGLI_POST_CREATE_BUILD` defaults to `0`, L91) — its `DEV_PACKAGES` build is `--packages-up-to`, so setting it to `1` still pulls `mowgli_coverage` in through `mowgli_bringup`'s `exec_depend`.
- The devcontainer ships Gazebo (`ros-kilted-ros-gz-*`, L126–129) and no Webots. The sim is Webots (`ros2/src/mowgli_simulation`, `webots_ros2_driver`), installed only in `ros2/Dockerfile` L509–516 — `make sim` does not work in a plain Codespace.
- `.githooks/pre-push` runs `format.sh --check`, which is a **whole-tree** `clang-format --dry-run --Werror`. With any clang-format major ≠ 18 installed locally it reformats files CI never asked about and auto-creates a commit (`format.sh` L12, L21–24 only *warn* on a version mismatch). The hook is opt-in (`git config core.hooksPath .githooks`) and nothing in the repo installs it.
- `msg-codegen-drift.yml` and `protocol-version-drift.yml` have `pull_request: branches: [main]` only (L15–16 / L12–13). A PR into `dev` gets those gates only through the `push` trigger on the source branch — a `codex/…`-style branch name matches neither, so both gates can be silently absent on a `dev` PR. `ros2-ci.yml` solved exactly this problem with the `changes` job.
- `docs/test_install.sh` and `docs/test_web_composer.sh` are both RED at this SHA and no workflow runs them: `test_install.sh` L57–58 asserts help strings without the `first-boot default:` wording that `install.sh` L151–152 actually prints; `test_web_composer.sh` L75–76 asserts a `tfluna-group` that no longer exists in `docs/index.html`.
- `install.sh` still parses `--tfluna=` (L142, L306–317) and forwards it to the installer, but the composer never emits it and the UI group is gone.
- `tools/motor/CMakeLists.txt` L24–27 registers only `test_drive_pid_math`; `test/test_drive_pid_tuner.py` never runs under `colcon test`.
- `static-analysis` is `continue-on-error: true` (`ros2-ci.yml` L472) — cppcheck findings never fail a PR; only the log/artifact records them.
- `auto-label.yml` and `welcome.yml` use `pull_request_target`, which runs with repo write scope on fork PRs. Neither checks out PR head code — keep it that way.
- `changes` reads `github.base_ref` through `env:` rather than inlining it (`ros2-ci.yml` L56–60) because a fork branch name is attacker-controlled; preserve that pattern in any new script step.
- `ros2-ci.yml` checks out with `submodules: recursive` (L149) because `opennav_coverage_msgs` must be present for `rosdep --ignore-src`; a new workflow that builds the workspace needs the same.
- `wiki-sync.yml` copies with `cp main/wiki/*.md wiki/` — it never deletes, so a page removed from `wiki/` stays live on the GitHub wiki.
- `pages.yml` deploys the entire `docs/` tree, including the Markdown design notes and `docs/claude/` — anything committed there becomes public at mowgli.garden.
- The `.claude/memory/*.md` notes are historical snapshots: `project_e2e_status.md` is dated 2026-04-04 and still describes SLAM-era behaviour ("SLAM map grows", L14), an RPP-followed coverage path (L12) and F2C Dubins turn waypoints (L21); `feedback_sim_cleanup.md` L9, L15 talk about Gazebo processes and `pkill gz`, but the sim is Webots. Treat both as history; the root `CLAUDE.md` invariants win.

## Generated & vendored — do not hand-edit

- Nothing in this area is code-generated. The drift gates in `msg-codegen-drift.yml` police generated files that live in *other* areas (`gui/pkg/msgs/**`, `gui/web/src/types/ros.generated.ts`, `firmware/.../ros_lib/mower_msgs/**`) — regenerate with the `gui/generate_*.sh` scripts and `firmware/scripts/sync_ros_lib.py`, never by hand.
- `.github/scripts/install-ros-apt-source.sh` downloads a third-party `.deb` from the `ros-infrastructure/ros-apt-source` releases; the asset is resolved at run time, not pinned in-tree.
- `/tmp/digests/*`, `cppcheck-report.xml`, `dist/` (firmware release packaging) are CI scratch — never committed.
