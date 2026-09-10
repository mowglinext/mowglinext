# install/ — working notes for Claude

The interactive Bash installer (`mowglinext.sh` + `lib/*.sh`), the compose fragments it merges into ONE generated `docker/docker-compose.yaml`, the versioned config **seeds** copied into `docker/config/`, the `locale/` message catalogues, the OpenMower migration script and a hand-rolled bash test suite. It owns the **host** side of the contract: UARTs, udev symlinks, rc.local/systemd, `/usr/local/bin/mowgli-*`, image tags and `docker/.env`.
It owns **no ROS node, no ROS parameter default, and no runtime file**: parameter defaults belong in the in-package template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` (root CLAUDE.md **Invariant 15**), everything under `docker/` that the installer writes is gitignored *output*, and the GNSS/LiDAR sidecar images live in `sensors/`.

## Read next

Paths in this table are repo-root relative. Everywhere else below, bare `lib/…`, `compose/…`, `config/…`, `locale/…`, `tests/…` paths are relative to `install/`.

| File | Read it when… |
|------|---------------|
| `docs/claude/codemaps/deploy.md` | **First, always.** File/line index of `install/` + `docker/` + `sensors/`, the `.env`→fragment→container→ROS-param table, bind mounts, host artefacts, change-coupling, pitfalls. Everything below is only the delta. |
| `docs/claude/parameters.md` | Touching `install/config/mowgli/mowgli_robot.yaml` or any key the installer patches — where each default lives, who consumes it, and which keys are `INERT`. |
| `docs/claude/testing-ci.md` | Before pushing: what CI gates (the installer suite is gated by **nothing**; only `install/config/mowgli/**` triggers the config-drift job). |
| `docs/claude/ros-interfaces.md` | Wiring a container to a topic/service/TF frame — every endpoint resolved with `node · file:line`. |
| `docs/claude/doc-index.md` | Deciding whether a doc you found is current, historical or stale, and which doc wins a conflict. |
| `docs/claude/codemaps/mowgli_bringup.md` | Tracing where a seeded `mowgli_robot.yaml` key is actually consumed (`robot_config_util.load_robot_params`). |
| `docs/claude/commands.md` | Running the ROS2-side build/tests after changing a seed the ROS2 stack reads. |
| `install/config/mowgli/README.md` | Changing the `/ros2_ws/config` mount contract — the operator-facing list of which YAMLs the container reads (seed only; the installer never copies it into `docker/config/`). |
| `docs/FIRST_BOOT.md` | Changing the post-install experience: `--check` output, MOTD, onboarding order, calibration write-back. |
| `wiki/Getting-Started.md` | Changing prerequisites / supported boards / install entry points (stale localizer paragraph at L55). |
| `wiki/Deployment.md` | Operator view of compose generation — **stale GNSS sections** (`GNSS_STACK=legacy`, `docker-compose.unicore.yaml` no longer exist). Trust the codemap. |
| `wiki/Sensors.md` · `sensors/README.md` | The GNSS (`/gps/fix`, `/gps/status`, `/rtcm`) and LiDAR (`/scan`) contract the sidecar containers must keep. |
| `wiki/Configuration.md` · `docker/README.md` | Operator-facing YAML / manual-compose reference; both are partly stale (see `doc-index.md`). |

## Build · test · run

```bash
# Test suite — pure bash, no docker and no network (mocks shim sudo/apt/git/docker/udevadm/systemctl)
bash install/test_mowglinext.sh
for t in install/tests/test_*.sh; do bash "$t" || echo "FAILED: $t"; done

# Syntax gate — test_mowglinext.sh only checks mowglinext.sh + 10 libs; cover the rest by hand
for f in install/mowglinext.sh install/lib/*.sh install/locale/*.sh; do bash -n "$f" || echo "SYNTAX: $f"; done

# Interactive install / upgrade / diagnose (on the robot)
bash install/mowglinext.sh
bash install/mowglinext.sh --check          # diagnostics only
bash install/mowglinext.sh --branch=dev --image-tag=dev --backend=mowgli \
     --gnss=auto --gnss-connection=uart --gnss-baud=auto --lidar=ldlidar-uart --tfluna=none
# also: --lang= --gnss-device= --gnss-receiver-family= --lidar-uart= --tfluna-{front,edge}-uart=
#       --gps= / --gps-uart= / --channel=  (deprecated aliases, still parsed)

# Aim the installer or the test harness at a sandbox instead of ~/mowglinext
MOWGLI_HOME=/tmp/sandbox bash install/mowglinext.sh --check

# Regenerate only docker/docker-compose.yaml from a checkout (reuses install/lib/compose.sh)
./docker/stack.sh regen        # also: up down restart pull update logs ps config

# Seed-vs-template drift gate (the one CI job that watches install/config/mowgli/**)
python3 ros2/scripts/check_config_drift.py
python3 -m pytest -q ros2/scripts/test_check_config_drift.py

# OpenMower → MowgliNext migration
python3 install/scripts/migrate_openmower.py --source ~/mowgli-docker \
        --target ~/mowglinext/docker/config --dry-run
```

## Conventions

- Bash with `set -euo pipefail` in `mowglinext.sh`. `lib/*.sh` are **sourced, never executed**: define functions and globals only, no top-level side effects beyond path computation.
- No shellcheck/shfmt in CI; `# shellcheck source=` / `disable=` directives are already used — keep them accurate.
- Every user-visible string goes through a `MSG_*` variable defined in **both** `locale/en.sh` and `locale/fr.sh` (`lib/i18n.sh` `load_locale`). Adding a prompt means adding two entries.
- Use the `lib/common.sh` primitives (`info/warn/error/step/prompt/confirm`) — they feed the progress bar, the install log and the final issue summary; raw `echo`/`read` breaks `progress_run*` capture and the `/dev/tty` prompt contract the test harness bypasses.
- `install/config/**` are **seed templates** (versioned). Their runtime copies under `docker/config/**`, plus `docker/.env` and `docker/docker-compose.yaml`, are gitignored generated output — regenerate, never hand-edit.
- Keep `install/config/mowgli/mowgli_robot.yaml` SPARSE (Invariant 15): install-time choices + calibration placeholders only. `check_config_drift.py` fails on a key whose value merely equals the template default.
- `install/.preset`, `install/.preset.consumed` and `install/lib/config.local.sh` are gitignored operator/fork files (`.gitignore:49–53`); fork overrides start from `lib/config.local.sh.example` (it only sets `REPO_URL`, which drives the GHCR prefix).
- New test = `tests/test_*.sh` that calls `setup_sandbox` + `install_all_mocks` (`tests/lib/framework.sh`, `mocks.sh`) and ends with `test_summary`; drive the installer through `tests/lib/harness.sh`, never the real host or the network.

## Component-specific gotchas

- A new `docker/.env` key must be added to `lib/state.sh` `is_allowed_installer_key` (L11–33) or presets and `.env` reload **silently drop it**; also extend the `REQUIRED_KEYS` list in `tests/test_env_output.sh` L41.
- `write_config` (`lib/config.sh` L1327–1443) NEVER overwrites an existing installed `mowgli_robot.yaml` — it line-splices ~25 keys. Changing the seed alone does nothing on an already-installed robot.
- `write_config` L1425–1427 forces `use_scan_matching` + `use_loop_closure` to follow `lidar_enabled` on **every** re-run — an operator who turned one off in the GUI gets it back on after the next installer pass.
- `LIDAR_ENABLED` in `.env` decides only whether the *container* is composed; the ROS-side LiDAR mode is `mowgli_robot.yaml:lidar_enabled` (comment in `compose/docker-compose.base.yml` L13–17).
- Empty `GNSS_*` values in `compose/docker-compose.gps.yml` L45–57 mean "not set" on purpose — `sensors/gps/start_gps.sh` resolves YAML → env → default, so a compose default silently masks the operator's YAML.
- `config/mowgli/{hardware_bridge,twist_mux,foxglove_bridge}.yaml` are **dead seeds** — the installer never copies them into `docker/config/mowgli/`, and launch loads `hardware_bridge.yaml` / `twist_mux.yaml` from the package share. Only `mowgli_robot.yaml` is read from `/ros2_ws/config`.
- `config/cyclonedds.xml` is only a seed — the stack mounts the *tracked* `docker/config/cyclonedds.xml`, so the seed-if-absent guard never fires on a repo clone. Edit both.
- `compose/docker-compose.foxglove.yml` is dead and would break the stack (`enable_coverage:=`, an arg no launch file declares); `compose/docker-compose.mavros.yml` L39–40 launches a `mowgli_ntrip_client` package that does not exist in `ros2/src`.
- TF-Luna and VESC prompts exist but are hard-gated off (`lib/config.sh` `range_services_available` L354, `vesc_service_available` L373 returns 1 unconditionally); `tests/test_optional_features.sh` pins that they never leak into the generated compose.
- `write_compose_merged` runs `docker compose config --no-interpolate`; the pure-Bash fallback (`lib/compose.sh` L169–206) is a naive section concatenator. Re-run `tests/test_compose_validity.sh` after touching any fragment.
- `config.sh` recomputes `REPO_DIR` from `MOWGLI_HOME` at source time, so `docker/stack.sh` L76–83 re-asserts the paths afterwards — any new lib that caches a path at source time needs the same treatment.
- `migrate_runtime_paths` backs up `.env` + `docker-compose.yaml` to `.old.<ts>` on **every** run (`lib/deploy.sh` L279–280) and they accumulate; policy still open in `TODO-runtime-backups.md`.
- `lib/udev.sh` L58–63 falls back to a bare `KERNEL=="<kernel name>"` rule when `udevadm` cannot resolve USB attributes — unstable across re-enumeration, which is exactly the bug the VID/PID form fixes.
- `COMPOSE_PROJECT_NAME` must stay stable (default `install`): renaming it orphans the `install_mowgli_maps` volume holding `areas.dat` and the saved fusion graph.
- `parse_args` (`lib/config.sh` L842–1078) only `warn`s on an unknown argument, so a typo'd flag silently no-ops. A new flag also needs the `docs/install.sh` bootstrap allowlist and the web composer under `docs/`.

## Safety

This component writes host artefacts that decide what the robot physically talks to: udev symlinks (`/dev/mowgli` → the STM32, `/dev/gps`, `/dev/lidar`), UART overlays, `rc.local`/`rc-local.service`, and the compose `devices:` mappings. Root CLAUDE.md **Safety** applies unchanged — the STM32 firmware is the sole blade and emergency-stop authority. Never add a software e-stop path from here: `config/mowgli/twist_mux.yaml` L33–45 records why there is deliberately no `locks:` block (root "What NOT to Do"), and that copy is not even loaded. Treat any change to udev rules, device paths, `HARDWARE_BACKEND` or UART enablement as **safety-critical** in review: a mis-pointed `/dev/mowgli` means ROS2 is driving the wrong board.
