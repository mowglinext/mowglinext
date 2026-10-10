# install/ — working notes for Claude

The interactive Bash installer (`mowglinext.sh` + `lib/*.sh`), the compose fragments it merges into ONE generated `docker/docker-compose.yaml`, the versioned config **seeds** copied into `docker/config/`, the `locale/` message catalogues, the OpenMower migration script and a hand-rolled bash test suite. It owns the **host** side of the contract: UARTs, udev symlinks, rc.local/systemd, `/usr/local/bin/mowgli-*`, image tags and `docker/.env`.
It owns **no ROS node, no ROS parameter default, and no runtime file**: parameter defaults belong in the in-package template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` (root CLAUDE.md **Invariant 15**), everything under `docker/` that the installer writes is gitignored *output*, and the GNSS/LiDAR sidecar images live in `sensors/`.

## Read next

Paths in this table are repo-root relative. Everywhere else below, bare `lib/…`, `compose/…`, `config/…`, `locale/…`, `tests/…` paths are relative to `install/`.

| File | Read it when… |
|------|---------------|
| `docs/claude/codemaps/deploy.md` | **First, always.** File/line index of `install/` + `docker/` + `sensors/`, the `.env`→fragment→container→ROS-param table, bind mounts, host artefacts, change-coupling, pitfalls. Everything below is only the delta. |
| `docs/claude/parameters.md` | Touching `install/config/mowgli/mowgli_robot.yaml` or any key the installer patches — where each default lives, who consumes it, and which keys are `INERT`. |
| `docs/claude/testing-ci.md` | Before pushing: what CI gates. `install/config/mowgli/**` triggers the config-drift job; `.github/workflows/updater.yml`'s `test-build` job (any push/PR touching `gui/**`, `install/**`, `ros2/**`, `docker/**`) additionally runs `install/tests/test_updater.sh`, `test_deployment_publication.sh`, `test_compose_validity.sh` and `test_idempotency.sh` specifically — the rest of `install/tests/test_*.sh` is **not** CI-gated, run it locally before pushing. |
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

# Syntax gate — test_mowglinext.sh only checks mowglinext.sh + 7 libs; cover the rest by hand
for f in install/mowglinext.sh install/lib/*.sh install/locale/*.sh; do bash -n "$f" || echo "SYNTAX: $f"; done

# Modes (on the robot). `install` is the default; interactive when a tty is
# attached, `--non-interactive` (or no tty) makes every unset choice its default.
bash install/mowglinext.sh                  # install, interactive
bash install/mowglinext.sh install --non-interactive --branch=dev --image-tag=dev \
     --backend=mowgli --gnss-connection=uart --gnss-device=/dev/ttyAMA4 --lidar=ldlidar-uart
bash install/mowglinext.sh update           # MANUAL update: sync checkout, regen .env+compose, pull, up.
                                            # No host updater, no readiness/firmware gate (docs/UPDATES.md)
bash install/mowglinext.sh update --branch=dev
bash install/mowglinext.sh repair           # re-apply udev/UART/sysctl/.env/compose/helpers from saved choices, never prompts
bash install/mowglinext.sh check            # diagnostics only (alias --check)
bash install/mowglinext.sh uninstall        # remove everything (stack+images, updater, host files, checkout);
                                            # KEEPS the maps volume + docker/config/mowgli/mowgli_robot.yaml in place;
                                            # confirms on a tty, needs an explicit --yes without one (lib/uninstall.sh)
# also: --lang= --gnss-baud= --gnss-receiver-family= --lidar-uart= --no-updater
#       --gps= / --gps-uart= / --channel= / --tfluna* (deprecated, parsed and ignored)
# The installer asks ONLY host wiring. Datum, NTRIP, the GNSS receiver profile
# (family/baud/config apply) and LiDAR mounting are GUI-owned (onboarding wizard).

# --only=<step> (issue #632): run exactly one step of run_install() — for
# adding one thing (e.g. the host updater) to an already-working install.
# Skips the branch/language prompts, still loads the existing docker/.env.
# See list_only_steps()/run_only_step() in mowglinext.sh for the step names.
bash install/mowglinext.sh --only=updater
bash install/mowglinext.sh --only=bogus     # rejected, lists valid step names

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

- **A compose fragment must work on a robot that never ran this installer version.** Updater-managed robots receive new fragments through the release bundle with their OLD `.env` and no installer run, so every `${VAR}` needs a `:-default` and a fragment may not depend on a file only the installer generates. It may not ADD writable storage either: the host updater refuses such a release (`validateStackMounts`: "adds writable storage …; explicit data/layout migration required", no backup contract) — keep runtime scratch in a bounded `tmpfs`. The one exception is the host device tree (`/dev:/dev`, `isDeviceMount`); `gui/pkg/updater/sidecar_mounts_test.go` runs every release sidecar through both mount gates. The GNSS sidecar (`compose/docker-compose.gps.yml`) is the reference: it mounts `mowgli_robot.yaml` — the ONE GNSS configuration, edited in the GUI — and its embedded launcher derives the receiver's ROS parameters in a tmpfs at container start. There is deliberately no `docker/config/universal_gnss/parameters.yaml` and no `GNSS_DEVICE` / `UNIVERSAL_GNSS_*_DIR` variable any more (pinned by `tests/test_gnss_external_sidecar_contract.sh` and `mowgli_bringup/test/test_gnss_sidecar_launcher.py`, which also keeps the launcher's defaults equal to the template).

- **"Local changes" means modified TRACKED files** (`repo_local_changes`, `lib/deploy.sh` and its twin in `docs/install.sh`): untracked files and submodule state never count. Anything the installer, the GUI or the host updater writes into the checkout must be in `.gitignore`, and the installer must never `submodule update --init` — a robot builds nothing from `ros2/src`, and an initialised submodule makes the operator's next `git pull` die in the on-demand submodule fetch as soon as its URL or pin moves (`tests/test_repo_self_update.sh`).
- The plain installer records `docker/stack-definition.sha256` for every Compose file it generates (`record_compose_baseline`); that is what lets the host updater adopt an older-but-untouched file later. A new writer of `docker-compose.yaml` must record it too.
- A new `docker/.env` key must be added to `lib/state.sh` `is_allowed_installer_key` (L11–33) or presets and `.env` reload **silently drop it**; also extend the `REQUIRED_KEYS` list in `tests/test_env_output.sh` L41.
- A new `HARDWARE_BACKEND` value must land in FOUR places at once: `lib/config.sh` (`SUPPORTED_HARDWARE_BACKENDS` + `is_supported_hardware_backend`, which live there and NOT in `lib/backend_choice.sh` because `docker/stack.sh` never sources that file) plus `parse_args --backend=`, the menu in `lib/backend_choice.sh`, `docs/install.sh` + the composer in `docs/index.html`, and `SUPPORTED_HARDWARE_BACKENDS` in `ros2/src/mowgli_bringup/launch/mowgli.launch.py` (pinned by `test_hardware_backend_launch.py`). Only `mowgli` launches the in-tree bridge; `mavros`/`openmower` own it in a sidecar.
- `write_config` (`lib/config.sh`) writes `mowgli_robot.yaml` ONLY when it does not exist yet (seed + the GNSS link / `lidar_enabled` just chosen). An existing file is never touched — not by `update`, `repair`, nor a reinstall: the GUI owns every key (Invariant 15) and the containers write calibration into it as root (Invariant 6), so a reinstall that changes the wiring tells the operator to flip it in the GUI. Changing the seed alone does nothing on an already-installed robot; retired keys are scrubbed by the GUI settings backend on save, not here.
- `LIDAR_ENABLED` in `.env` decides only whether the *container* is composed; the ROS-side LiDAR mode is `mowgli_robot.yaml:lidar_enabled` (comment in `compose/docker-compose.base.yml` L13–17).
- **The GNSS sidecar is an external image, not a local build** (2026-09-19, commit `bfd44f1a`, "#625"): `compose/docker-compose.gps.yml`'s `gps` service no longer builds `sensors/gps/Dockerfile`/`start_gps.sh` with `GNSS_*` env passthrough — it runs `${UNIVERSAL_GNSS_IMAGE}` and reads `mowgli_robot.yaml` directly via an inline Python `command:` block (`GPS_IMAGE` was removed from `.env`). `mowgli_gnss_bridge` is now compiled straight into the `mowgli-ros2` image instead of a `sensors/gps` sidecar image.
- `config/mowgli/{hardware_bridge,twist_mux,foxglove_bridge}.yaml` are **dead seeds** — the installer never copies them into `docker/config/mowgli/`, and launch loads `hardware_bridge.yaml` / `twist_mux.yaml` from the package share. Only `mowgli_robot.yaml` is read from `/ros2_ws/config`.
- `config/cyclonedds.xml` is only a seed — the stack mounts the *tracked* `docker/config/cyclonedds.xml`, so the seed-if-absent guard never fires on a repo clone. Edit both.
- The standalone `mowgli-ntrip` service was removed from `compose/docker-compose.mavros.yml` entirely (2026-09-19, commit `bfd44f1a`, same PR as the GNSS sidecar change above) — it used to launch a `mowgli_ntrip_client` package that never existed in `ros2/src`; there is now no separate NTRIP container on this backend.
- **Watchtower is gone** (2026-09-29): the host updater handles managed releases and `mowglinext.sh update` everything else; `install_host_updater` still removes a leftover `mowgli-watchtower` container. **The mosquitto broker is opt-in** (`--mqtt=on`, `.env` `ENABLE_MQTT`, default off): it is a local (non-release) service, so `installer-stack` reads `MOWGLI_ENABLE_MQTT` to add or drop it when it regenerates the file, and `--check` only expects it when enabled.
- TF-Luna and VESC are RETIRED (2026-09-29, with the optional tools, the datum/NTRIP prompts and the host-side GNSS baud probing/upgrade): their fragments, libs and `.env` keys are gone; `setup_env` scrubs the keys from an older `.env` and `tests/test_optional_features.sh` pins that they never come back. `lib/tools.sh` now only installs the `mowgli-*` helpers (always, no prompt).
- `prompt`/`confirm` (`lib/common.sh`) return their default under `NON_INTERACTIVE=true`, which `parse_args` also sets when `/dev/tty` is unreadable — every prompt MUST therefore carry a sensible default as `$2`; a new prompt without one silently answers "" in a composer run. `confirm` answers yes; keep destructive confirmations out of non-interactive paths (or gate them on an explicit flag, as `MOWGLI_ADOPT_LEGACY_COMPOSE` does).
- `update`/`repair` require an existing `docker/.env` (`require_installed_runtime`) and never invent hardware choices; `repair` forces `NON_INTERACTIVE`. `update` leaves an installed host updater running on purpose (operator decision 2026-09-29): it uses the same writers as `install`, so the baseline is re-recorded and the updater can adopt the result; the GUI reports "drifted" until the next managed release.
- `write_compose_merged` runs `docker compose config --no-interpolate`; the pure-Bash fallback (`lib/compose.sh` L169–206) is a naive section concatenator. Re-run `tests/test_compose_validity.sh` after touching any fragment.
- `config.sh` recomputes `REPO_DIR` from `MOWGLI_HOME` at source time, so `docker/stack.sh` L76–83 re-asserts the paths afterwards — any new lib that caches a path at source time needs the same treatment.
- `migrate_runtime_paths` still backs up `.env` + `docker-compose.yaml` to `.old.<ts>` unconditionally up front on every run (`lib/deploy.sh`), but `prune_backup_if_unchanged` removes that backup afterward if the regenerated file turns out byte-identical — a re-run that changes nothing no longer accumulates one. Remaining backup-policy ideas (rotation, a dedicated backup dir, `--no-backup`) are still open in `TODO-runtime-backups.md`.
- `lib/udev.sh` L58–63 falls back to a bare `KERNEL=="<kernel name>"` rule when `udevadm` cannot resolve USB attributes — unstable across re-enumeration, which is exactly the bug the VID/PID form fixes.
- `COMPOSE_PROJECT_NAME` must stay stable (default `install`): renaming it orphans the `install_mowgli_maps` volume holding `areas.dat` and the saved fusion graph.
- `parse_args` (`lib/config.sh`) only `warn`s on an unknown argument, so a typo'd flag silently no-ops. A new flag also needs the `docs/install.sh` bootstrap allowlist and the web composer under `docs/` (which always appends `--non-interactive`; `docs/test_web_composer.sh` pins it).

## Safety

This component writes host artefacts that decide what the robot physically talks to: udev symlinks (`/dev/mowgli` → the STM32, `/dev/gps`, `/dev/lidar`), UART overlays, `rc.local`/`rc-local.service`, and the compose `devices:` mappings. Root CLAUDE.md **Safety** applies unchanged — the STM32 firmware is the sole blade and emergency-stop authority. Never add a software e-stop path from here: `config/mowgli/twist_mux.yaml` L33–45 records why there is deliberately no `locks:` block (root "What NOT to Do"), and that copy is not even loaded. Treat any change to udev rules, device paths, `HARDWARE_BACKEND` or UART enablement as **safety-critical** in review: a mis-pointed `/dev/mowgli` means ROS2 is driving the wrong board.
