# Codemap: deploy (`install/` + `docker/` + `sensors/`)

> Everything that turns this repo into a running robot: the interactive Bash installer (`install/mowglinext.sh` + `install/lib/*.sh`), the compose fragments it selects and merges into ONE generated `docker/docker-compose.yaml`, the config seeds it copies into `docker/config/`, the dockerised sensor drivers (`sensors/`), and the standalone stack manager `docker/stack.sh`. It owns the **host** side of the contract: UARTs, udev symlinks, rc.local/systemd, image tags, and `docker/.env`. It owns NO ROS node except `sensors/gps/mowgli_gnss_bridge`.
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `install/CLAUDE.md`.

## Where to look

Managed installer and updater share `install/compose/stack.json` via the Go `installer-stack` command. Releases ship a checksummed Compose bundle; installed releases survive older-checkout installer reruns. See `docs/UPDATES.md` for selection, ownership and migration constraints.

Coordinated updates: `install/deployment.json` owns the publication build list and per-image compatibility contracts and verified service projection. Installed Compose fragments declare `garden.mowgli.update.*` labels; absent optional services stay absent. See `docs/UPDATES.md` for dependency, health and supported persistence contracts.


| Task | Start here |
|------|------------|
| Trace an install end-to-end (12 steps) / a manual `update` (5) / `repair` (9) | `install/mowglinext.sh` `run_install()` / `run_update()` / `run_repair()`, dispatched from `main()` on `INSTALL_MODE` (positional `install|update|repair|check`, parsed in `config.sh` `parse_args`; `--non-interactive` makes `prompt`/`confirm` return their defaults) |
| Run a single install step in isolation (issue #632) | `install/mowglinext.sh` `run_only_step()`/`list_only_steps()`, dispatched from `main()`'s `ONLY_STEP` branch (`--only=<step>`, parsed in `config.sh` `parse_args`) — skips `select_repo_branch`/`select_language` on purpose; see `install/tests/test_only_step.sh` |
| Add / change a `docker/.env` key | `install/lib/env.sh` `setup_env` L210–374 (defaults L216–301, writes L325–367) + whitelist `install/lib/state.sh` `is_allowed_installer_key` L11–33 + guard `install/tests/test_env_output.sh` L38 |
| "Which env var starts which container?" | `install/lib/compose.sh` `build_compose_stack` L52–138 — see the env→service table below |
| Change how the merged compose is produced | `install/lib/compose.sh` `write_compose_merged` L208–244 (`docker compose config --no-interpolate`), pure-Bash fallback L169–206 |
| Add a CLI flag / web-composer preset key | `install/lib/config.sh` `parse_args` L842–1078; preset loader `install/lib/state.sh` `load_preset_file` L133; the web bootstrap `docs/install.sh` forwards choices as CLI flags (it does not write `.preset`) |
| Change what the installer writes into `mowgli_robot.yaml` | `install/lib/config.sh` `write_config` — seeds the file ONLY when absent (template + GNSS link + `lidar_enabled`); an existing file is never written (GUI-owned, root-owned by the containers) |
| Change the SPARSE seed shipped to new robots | `install/config/mowgli/mowgli_robot.yaml` (CLAUDE.md Invariant 15 — defaults belong in `ros2/src/mowgli_bringup/config/mowgli_robot.yaml`) |
| GNSS backend/stack resolution logic | `install/lib/config.sh` L462–840 (`normalize_*`, `effective_gnss_backend` L609, `effective_gnss_stack` L625, `compose_gnss_service_name` L814) |
| LiDAR selection (type/connection/baud) | `install/lib/lidar.sh` `configure_lidar` L3–116 |
| udev symlinks (`/dev/mowgli`, `/dev/gps`, `/dev/lidar`, `/dev/tfluna_*`) | `install/lib/udev.sh` `build_dynamic_udev_rules` L91–152, `install_udev_rules` L154 (writes `/etc/udev/rules.d/50-mowgli.rules`) |
| UART overlays / Pi-5 boot tweaks / rc.local | `install/lib/uart.sh` `enable_all_platform_uarts` L124, `configure_raspberry_pi_5_hardware` L94; `install/lib/rc_local.sh` (installs `rc-local.service`) |
| `--check` diagnostics | `install/lib/checks.sh` (`expected_runtime_services` L28, `check_devices` L73, `check_generated_gps_yaml_alignment` L175, `check_gps` L318) |
| Image tag / channel (`:main`, `:dev`, custom) | `install/lib/config.sh` `_ghcr_prefix` L35, `recompute_image_defaults` L41, `select_image_channel` L202 |
| Run/regenerate the stack from a dev checkout | `docker/stack.sh` (`regen`/`up`/`update`, optional-fragment filter L88–103) |
| GNSS sidecar runtime resolution (YAML → env → default) | `sensors/gps/start_gps.sh` resolvers L66–376, process launch L515–598 |
| Public GNSS topic contract (`/gps/status`, `/rtcm`) | `sensors/gps/mowgli_gnss_bridge/src/universal_gnss_topic_bridge.cpp` L271–305 |
| LiDAR container behaviour | `sensors/lidar-ldlidar/ldlidar_scan.launch.py` + `sensors/lidar-ldlidar/ldlidar.yaml`; RPLiDAR/STL27L are `command:` overrides in their compose fragments |
| Run the installer test suite | `install/test_mowglinext.sh` + `install/tests/test_*.sh` (see Build, test, run) |
| Migrate an OpenMower install | `install/scripts/migrate_openmower.py` |

## Files

| File | Lines | Purpose |
|------|-------|---------|
| **`install/`** | | |
| `install/mowglinext.sh` | 202 | Entry point: sources all libs, `load_preset`/`load_install_state`, 15-step `main()`, then health checks |
| `install/test_mowglinext.sh` | 132 | Core installer test: preset parsing, `configure_gps`, udev rules, end-to-end harness run, `bash -n` syntax gate |
| **`install/lib/`** | | |
| `lib/config.sh` | 1538 | Globals (`REPO_URL`, `REPO_DIR`, `IMAGE_TAG`, `UDEV_RULES_FILE`), image defaults, branch/tag selection, GNSS normalisation, `parse_args`, `interactive_config`, `write_config`, `auto_detect_position` |
| `lib/env.sh` | 374 | `upsert_env_key`/`remove_env_key`, NTRIP-from-YAML defaults, `sync_gnss_env_contract_values`, `setup_env` (writes `docker/.env`) |
| `lib/compose.sh` | 262 | `ensure_default_configs` (seeds `docker/config/`), `build_compose_stack` (fragment selection), `write_compose_merged`, `run_compose_stack` |
| `lib/checks.sh` | 610 | `--check` health checks: devices, containers, firmware, GPS, LiDAR, rangefinders, GUI |
| `lib/deploy.sh` | 445 | Git sync (`report_repository_sync_status`, `sync_repo_branch_to_selected_branch`), self-update (`update_repo_checkout_if_behind`: fetch without submodules → `resolve_repo_local_changes` stash/keep/abort → ff-only → re-exec), submodules (never initialised, only followed), `setup_directory`, `migrate_runtime_paths`, `fix_path_type_conflict` |
| `lib/state.sh` | 173 | Strict KV parser for `install/.preset` and `docker/.env` + the allowed-key whitelist; preset consume/backup |
| `lib/udev.sh` | 232 | Static + dynamic udev rule generation and install |
| `lib/backend_choice.sh` | ~340 | Mowgli-STM32 vs Pixhawk/MAVROS vs OpenMower selection MAVROS device detect, GCS URL, OpenMower ESC type |
| `lib/tools.sh` | ~120 | `/usr/local/bin/mowgli-{up,down,restart,logs,ps,pull,check,gps-logs,lidar-logs,shell,status}` (always installed; the optional ctop/mc/ranger/debug tools were retired 2026-09-29) |
| `lib/progress.sh` | 212 | Step progress bar/spinner, install log capture (`init_install_logs`) |
| `lib/gps.sh` | ~110 | `pick_serial_by_id`, `preset_key_loaded`, `configure_gps` — transport + port ONLY; receiver family/baud are first-boot defaults, the sidecar (`gnss_config_apply`) and GUI own the receiver configuration |
| `lib/platform.sh` | 163 | CPU arch / board family detection, `assert_supported_platform` |
| `lib/uart.sh` | ~215 | Boot-config line upsert; `required_uart_overlays()` derives which of `dtoverlay=uart1..5` the configured GNSS/LiDAR/TF-Luna ports actually need (issue #631 — used to enable all five unconditionally) plus the OpenMower LowLevel/xESC ports (`openmower_uart_assignments`) — runs after GPS/LiDAR/rangefinder config, not before; `free_uarts_from_serial_console` strips `console=serial0` from `cmdline.txt` and masks `serial-getty` on every UART in use; `uart_port_is_free`/`pick_free_uart_port` refuse a GNSS/LiDAR port an OpenMower controller is wired to (`default_gnss_uart_device` in `config.sh`: `ttyAMA2` on OpenMower, else `ttyAMA4`); Bluetooth disable, Pi-5 USB/fan params |
| `lib/common.sh` | 135 | `info/warn/fail/error/step/prompt/confirm`, `detect_uart_ports`, `pick_uart_port`, `require_root*` |
| `lib/motd.sh` | 127 | Writes `/etc/profile.d/mowgli-motd.sh` (reads `~/mowglinext/docker/.env`) |
| `lib/lidar.sh` | 120 | LiDAR type/connection/baud selection + `LIDAR_UART_RULE` |
| `lib/docker.sh` | 97 | `docker_cmd`/`docker_compose_cmd` (sg-docker wrapper), `install_docker` |
| `lib/system.sh` | 77 | apt update/upgrade, release pinning |
| `lib/banner.sh` | 64 | Banner + final `print_summary` (issue list) |
| `lib/rc_local.sh` | 50 | `/etc/rc.local` + `rc-local.service` (Pi UART platforms only) |
| `lib/i18n.sh` | 42 | `load_locale` / `select_language` |
| `lib/config.local.sh.example` | 14 | Template for the gitignored `config.local.sh` fork override of `REPO_URL` (drives the GHCR prefix) |
| **`install/locale/`** | | |
| `locale/en.sh`, `locale/fr.sh` | 73 / 73 | `MSG_*` strings sourced by `load_locale` |
| **`install/compose/`** (fragments; see table below) | | |
| `compose/docker-compose.base.yml` | 62 | `mowgli` service (mowgli-ros2): `full_system.launch.py`, shared `x-ros2-env`, `mowgli_maps` volume, config mounted **RW** |
| `compose/docker-compose.gui.yml` | 36 | `gui` (mowgli-gui), `pid: host`, docker socket, `FOXGLOVE_URL=ws://localhost:8765` |
| `compose/docker-compose.gps.yml` | 213 | `gps` (mowgli-gps) — runs the EXTERNAL `${UNIVERSAL_GNSS_IMAGE}` (not built from `sensors/gps/Dockerfile` any more, commit `bfd44f1a`, 2026-09-19, "#625"); an inline Python `command:` block reads `/config/mowgli_robot.yaml` directly — there is no `GNSS_*` env passthrough left at all |
| `compose/docker-compose.mqtt.yml` | 19 | `mosquitto` (mowgli-mqtt), ports 1883/9001 |
| `compose/docker-compose.lidar-ldlidar.yml` | 37 | `lidar` (mowgli-lidar), no command override — image CMD |
| `compose/docker-compose.lidar-rplidar.yml` | 38 | `lidar` with `rplidar_a2m8_launch.py` + `LIDAR_PORT`/`LIDAR_BAUD` |
| `compose/docker-compose.lidar-stl27l.yml` | 38 | `lidar` with `ldlidar_stl_ros2_node` + `LIDAR_MODEL`/`LIDAR_PORT`/`LIDAR_BAUD` |
| `compose/docker-compose.mavros.yml` | 30 | `mavros` (mowgli-mavros) only — the standalone `ntrip` (mowgli-ntrip) service was deleted entirely (commit `bfd44f1a`, 2026-09-19, same PR as the GNSS sidecar rewrite) |
| `compose/docker-compose.openmower.yml` | 52 | `openmower` (mowgli-openmower) — OpenMower LowLevel + xESC bridge sidecar, mounts `docker/config/mowgli` read-only at `/config` |
| **`install/config/`** (seeds copied to `docker/config/`) | | |
| `config/mowgli/mowgli_robot.yaml` | 79 | The SPARSE installed robot config seed (Invariant 15) |
| `config/cyclonedds.xml` | ~45 | Loopback-only DDS: `AllowMulticast=false`, iface `lo`, `MaxAutoParticipantIndex=500`, unicast peer `localhost`; no `Peer PruneDelay` attribute because deployed Cyclone DDS versions may reject it during ROS initialization |
| `config/mqtt/mosquitto.conf` | 18 | Anonymous listeners 1883 + 9001 (websockets) |
| `config/mowgli/hardware_bridge.yaml` | 6 | **Dead copy** — launch reads the package share copy |
| `config/mowgli/twist_mux.yaml` | 45 | **Dead copy** — launch reads the package share copy |
| `config/mowgli/foxglove_bridge.yaml` | 9 | **Dead copy** — no launch file reads it |
| `config/mowgli/README.md` | 58 | Operator note on the config mount (see stale claims) |
| **`install/scripts/`** | | |
| `scripts/migrate_openmower.py` | 463 | One-shot OpenMower → MowgliNext config + `areas.dat` migration (`--dry-run`, `--force`) |
| **`install/tests/`** | | |
| `tests/lib/framework.sh` | 226 | `assert_*`, `section`, `setup_sandbox`, `sandbox_repo`, `test_summary` |
| `tests/lib/harness.sh` | 308 | `harness_init`/`harness_set_preset`/`harness_run` — replays the installer data-flow non-interactively |
| `tests/lib/mocks.sh` | 185 | PATH shims for sudo/apt/dnf/git/docker/udevadm/systemctl/raspi-config + call log |
| `tests/test_compose_validity.sh` | 179 | Merged compose validates; required services present; no unresolved `${}` |
| `tests/test_env_output.sh` | 230 | `.env` key union, preset propagation, ghcr refs, no secret leaks, permissions |
| `tests/test_deploy_repo_flow.sh` | 326 | `setup_directory` non-destructive; branch vs image tag decoupled |
| `tests/test_state_parsing.sh` | 218 | Strict preset/.env parser, no shell execution, preset consumption |
| `tests/test_idempotency.sh` | 180 | Re-runs preserve YAML GNSS config and create `.old.<ts>` backups |
| `tests/test_negative.sh` | 168 | Bad flags, invalid `GNSS_BACKEND`, missing tooling |
| `tests/test_lidar_matrix.sh` | 150 | `LIDAR_TYPE` × connection → fragment + baud + `LIDAR_IMAGE` |
| `tests/test_robot_yaml.sh` | 115 | Installed yaml shape, datum placeholders, no legacy `mower_config.sh` |
| `tests/test_bootstrap_repo_update.sh` | 126 | `docs/install.sh` stays conservative on existing checkouts; untracked files do not block it |
| `tests/test_repo_self_update.sh` | 232 | Installer self-update against real git: moved submodule, tracked local edits (stash/keep/abort), own commits, foreign-owned `.git`, runtime files ignored |
| `tests/test_compose_legacy_adoption.sh` | 130 | Plain installer records `stack-definition.sha256`; baseline-less Compose needs consent (`installer-stack` exit 3) |
| `tests/test_hardware_presets.sh` | ~150 | mowgli vs mavros vs openmower backend matrix |
| `tests/test_check_mode.sh` | 101 | `--check` targets the right services/commands |
| `tests/test_optional_features.sh` | ~60 | Retired TF-Luna / VESC fragments are gone and stale `.env` keys are scrubbed |
| `tests/test_udev_install.sh` | 87 | `install_udev_rules` under `set -u` |
| `tests/test_gps_matrix.sh` | 82 | UART and USB-by-id GNSS paths |
| `tests/test_config_dir_squat.sh` | 77 | `ensure_default_configs` heals a directory squatting a file mount |
| `tests/test_smoke.sh` | 58 | Default preset runs to completion, no network calls |
| **`docker/`** | | |
| `docker/stack.sh` | 178 | Dev-checkout stack manager reusing `install/lib/compose.sh` (`regen up down restart pull update logs ps config`) |
| `docker/README.md` | 792 | Operator deployment manual (largely stale — see stale claims) |
| `docker/.env.example` | 28 | Template for `docker/.env` (`COMPOSE_PROJECT_NAME`, `ENABLE_MQTT`, `ENABLE_WATCHTOWER`, image refs) |
| `docker/config/cyclonedds.xml` | ~49 | TRACKED runtime copy actually mounted by the stack; must stay in sync with the `install/config/` seed |
| `install/lib/sysctl.sh` | ~45 | `install_dds_sysctl`: writes `/etc/sysctl.d/90-mowgli-dds.conf` (rmem/wmem 8 MiB) and applies it — the DDS boot burst overflowed the 208 KiB default (`UdpRcvbufErrors`); run in the udev step |
| `docker/config/mowgli/README.md` | 77 | Runtime-config note (stale) |
| `docker/config/mowgli/drive_tuning/drive_pid_last_backup.yaml` | 9 | GUI drive-PID backup sample (dir otherwise gitignored) |
| `docker/docker-compose.simulation.yaml` | 125 | `simulation` / `dev-sim` / `simulation-gui` built from `ros2/Dockerfile` target `simulation` |
| `docker/docker-compose.foxglove.yaml` | 41 | Optional standalone `mowgli-foxglove` override (manual `-f` only) |
| `docker/fastdds.xml` | 36 | Reference only — Cyclone DDS is mandatory, nothing mounts this |
| `docker/diagrams/yardforce_wiring.drawio` | — | YardForce chassis wiring diagram (draw.io source, no build step) |
| `docker/logs/*.py`, `docker/logs/mow_sessions/*.py` | 37–130 each | Ad-hoc field-analysis tools (`motion_test.py`, `yaw_compare.py`, `xtrack.py`, `analyze_swath_turns.py`, `square_test.py`, …) |
| `docker/logs/mow_sessions/*.md` | 70 / 135 | Archived 2026-06-11 fusion-graph field reviews |
| **`sensors/`** | | |
| `sensors/gps/Dockerfile`, `start_gps.sh`, `ros2_entrypoint.sh` | — | **Historical GNSS sidecar implementation; no longer built or deployed.** Runtime GNSS now uses the external `${UNIVERSAL_GNSS_IMAGE}` selected by the installer compose fragments. |
| `sensors/mavros/{README.md,image.env,test_integration.py}` | — | Source-free MowgliNext integration contract for the independently built MowgliMAVROS sidecar: image pin, ownership/device/config documentation, and static validation. It does **not** build or vendor MowgliMAVROS. |
| `sensors/gps/universal_gnss_topic_bridge.py` | 406 | Retained historical Python bridge implementation; not the deployed GNSS sidecar entrypoint. |
| `sensors/gps/mowgli_gnss_bridge/src/universal_gnss_topic_bridge.cpp` | 489 | Default C++ bridge: universal→public enum/capability projection + diagnostics merge |
| `sensors/gps/mowgli_gnss_bridge/include/mowgli_gnss_bridge/universal_gnss_topic_bridge.hpp` | 78 | Node class, pub/sub members, QoS contract |
| `sensors/gps/mowgli_gnss_bridge/src/main.cpp` | 16 | `rclcpp::spin` entry |
| `sensors/gps/mowgli_gnss_bridge/test/test_universal_gnss_topic_bridge.cpp` | 157 | gtest on the projection logic |
| `sensors/gps/mowgli_gnss_bridge/CMakeLists.txt` / `package.xml` | 106 / 34 | Lib + exe split so gtest can link the node |
| `sensors/lidar-ldlidar/Dockerfile` | 122 | Myzhar `ldrobot-lidar-ros2` pinned `LDLIDAR_REF=v0.3.0` / `LDLIDAR_SHA=4ee53a8b…`; patches FATAL_ERROR + `count_subscribers` bug; installs `libldlidar.so` |
| `sensors/lidar-ldlidar/ldlidar_scan.launch.py` | 61 | Composable container + `nav2_lifecycle_manager` autostart; remaps `~/scan` → `/scan` |
| `sensors/lidar-ldlidar/ldlidar.yaml` | 22 | `comm.serial_port: /dev/lidar`, `comm.baudrate: 230400`, `lidar.bins: 455`, `lidar.frame_id: lidar_link` |
| `sensors/lidar-ldlidar/start_lidar.sh`, `ros2_entrypoint.sh` | 11 / 8 | CMD + entrypoint (sources `/opt/ldlidar`) |
| `sensors/lidar-rplidar/Dockerfile`, `ros2_entrypoint.sh` | 58 / 8 | Slamtec `rplidar_ros` (branch `ros2`), CMD `rplidar_a2m8_launch.py` |
| `sensors/lidar-stl27l/Dockerfile`, `ros2_entrypoint.sh` | 62 / 8 | `ldrobotSensorTeam/ldlidar_stl_ros2`, pthread include fix |
| `sensors/README.md` | 44 | Sensor overview (stale paths — see stale claims) |

## Runtime surface

### `docker/.env` → compose fragment → container → launch arg → ROS param

`install/lib/env.sh` `setup_env` writes every key; `install/lib/compose.sh` `build_compose_stack` reads them to pick fragments.

| `.env` key | Selects (file:line) | Container | Reaches the process as | Ends up as |
|---|---|---|---|---|
| `HARDWARE_BACKEND` (`mowgli`\|`mavros`\|`openmower`) | `compose.sh` selects the robot hardware fragment | `mowgli-ros2` plus the selected hardware sidecar where applicable | container env | robot propulsion/IMU/battery hardware backend; **does not select the GNSS source** |
| `GNSS_SOURCE` (`direct`\|`mavros`) | `compose.sh` selects `docker-compose.gps.yml` or `docker-compose.gps-mavros.yml` independently from `HARDWARE_BACKEND` | `mowgli-gps`, `mowgli-ros2`, `mowgli-mavros` | container env | `direct`: Universal GNSS owns the receiver; `mavros`: Universal GNSS consumes the selected MAVROS GNSS source and provides NTRIP/RTCM |
| `GNSS_MAVROS_SOURCE` (`gps1`\|`gps2`) | MAVROS GNSS source selection | `mowgli-gps`, `mowgli-ros2`, `mowgli-mavros` | container env | selects `/mavros/universal_gnss/<gps1|gps2>/...` when `GNSS_SOURCE=mavros` |
| `GNSS_STACK` (`universal`\|`disabled`), `GNSS_BACKEND` | installer/runtime compatibility contract | GNSS-capable services | container env | keeps Universal GNSS enablement independent from `HARDWARE_BACKEND`; direct receiver configuration itself comes from `mowgli_robot.yaml` |
| `HARDWARE_BACKEND` = `openmower` | `compose.sh` → `docker-compose.openmower.yml` | `mowgli-openmower` (+ GNSS sidecar according to `GNSS_SOURCE`) | container env; `mowgli.launch.py` skips `hardware_bridge_node` for anything but `mowgli` | GNSS remains independently selected; `write_config` seeds `ticks_per_meter: 1600.0` once |
| `OPENMOWER_LL_PORT`, `OPENMOWER_XESC_TYPE`, `OPENMOWER_XESC_{LEFT,RIGHT,MOW}_PORT`, `OPENMOWER_ENABLED` | `env.sh` `setup_env` | `mowgli-openmower` | container env → `openmower_bridge.launch.py` | defaults `/dev/ttyAMA0`, `xesc_mini`, `/dev/ttyAMA5`/`3`/`4` |
| `LIDAR_ENABLED` + `LIDAR_TYPE` | `compose.sh` L102–117 | `mowgli-lidar` | — | **container presence only.** Deliberately NOT passed to `mowgli-ros2` (`docker-compose.base.yml` L13–17); the ROS-side LiDAR mode is `mowgli_robot.yaml:lidar_enabled` |
| `LIDAR_LDLIDAR_IMAGE`/`LIDAR_RPLIDAR_IMAGE`/`LIDAR_STL27L_IMAGE` | `config.sh` `recompute_image_defaults` (per `LIDAR_TYPE`) | `mowgli-lidar` | image ref | — |
| `LIDAR_PORT` / `LIDAR_BAUD` / `LIDAR_MODEL` | rplidar L26–27, stl27l L26–30 | `mowgli-lidar` | `serial_port:=`/`serial_baudrate:=` (rplidar) or `-p port_name/port_baudrate/product_name` (stl27l) | driver params. **ldlidar ignores them** — hardcoded in `sensors/lidar-ldlidar/ldlidar.yaml` L6–7 |
| `ENABLE_FOXGLOVE` | `docker-compose.base.yml` L44 | `mowgli-ros2` | `full_system.launch.py enable_foxglove:=` | `foxglove_bridge` node on :8765 |
| `MOWGLI_ROS2_IMAGE`/`UNIVERSAL_GNSS_IMAGE`/`GUI_IMAGE`/`MAVROS_IMAGE`/`OPENMOWER_IMAGE`/`LIDAR_{LDLIDAR,RPLIDAR,STL27L}_IMAGE` | `config.sh` `recompute_image_defaults` | all | image refs | `ghcr.io/<owner>/<repo>/<name>:${IMAGE_TAG}`, except `UNIVERSAL_GNSS_IMAGE` which is pinned by digest on a THIRD-PARTY registry, independent of `IMAGE_TAG` (`GPS_IMAGE` was removed 2026-09-19, `bfd44f1a`) |
| `IMAGE_TAG` (`main`\|`dev`\|sanitised branch) | `config.sh` `select_image_channel` L202, `sanitize_image_tag` L61 | all | — | which tag the host updater/`pull` fetches |
| `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`, `ROS_AUTOMATIC_DISCOVERY_RANGE` | `x-ros2-env` anchor in every ROS fragment (base/gps/lidar-*/mavros/openmower; gui and mqtt carry none) | ROS services | container env | Cyclone DDS via `/cyclonedds.xml` |

**TF-Luna and VESC are RETIRED (2026-09-29, alongside Watchtower)**: their compose fragments, `install/lib` helpers (`effective_tfluna_*`, `effective_vesc_enabled`, `range_services_available`, `vesc_service_available`) and `.env` keys (`TFLUNA_*`, `ENABLE_VESC`, `VESC_IMAGE`, `VESC_CAN_INTERFACE`, `RANGE_IMAGE`) are gone; `setup_env` actively scrubs them from an older `.env` (`install/lib/env.sh:361` `remove_env_key ... "ENABLE_VESC"`) and `install/lib/state.sh:13` lists them only so the scrub recognises old keys, not because anything still writes them.
| `ENABLE_MQTT` | `compose.sh` `build_compose_stack` and `docker/stack.sh` `filter_optional_fragments` | `mowgli-mqtt` | — | opt-in broker (`--mqtt=on`); `installer-stack` honours `MOWGLI_ENABLE_MQTT` on regeneration |
| `MOWER_IP` | — | — | — | printed by the MOTD only (`lib/motd.sh` L76) |
| `DISABLE_BLUETOOTH` | — | — | — | written but never read (Bluetooth is disabled unconditionally, `uart.sh` L146) |
| `GNSS_BACKEND`, `GPS_PROTOCOL`, `HARDWARE_BACKEND` in `docker-compose.base.yml` L19–24 | — | `mowgli-ros2` | container env | **inert** — no file under `ros2/src` reads them |

### Compose services (container names, from `install/lib/checks.sh` L3–19)

`mowgli`→`mowgli-ros2`, `gps`→`mowgli-gps`, `lidar`→`mowgli-lidar`, `gui`→`mowgli-gui`, `mosquitto`→`mowgli-mqtt`, `mavros`→`mowgli-mavros`, `openmower`→`mowgli-openmower`. Always composed: base + gui; mqtt only with `ENABLE_MQTT=true`; Watchtower, the standalone `ntrip` service, TF-Luna and VESC were all removed 2026-09-29 (Watchtower)/2026-09-19 (`ntrip`, `bfd44f1a`)/2026-09-29 (TF-Luna, VESC) (`compose.sh` `build_compose_stack`). Named volume `mowgli_maps` → `/ros2_ws/maps` (base L47, L61) and also mounted into `gui` (gui L33).

### Bind mounts (host → container)

| Host path | Container path | Mode | Service |
|---|---|---|---|
| `./docker/config/mowgli` | `/ros2_ws/config` | **rw** | `mowgli` (base L52 — calibration write-back) |
| `./docker/config/mowgli` | `/config` | ro | `gps`; the external Universal GNSS runtime reads `/config/mowgli_robot.yaml` |
| `./docker/config/mowgli` | `/mowgli_config` | rw | `gui` (gui L29) |
| `./docker/config/cyclonedds.xml` | `/cyclonedds.xml` | ro | every ROS service |
| `./docker/config/om` / `./docker/config/db` / `./docker` | `/config` / `/db` / `/runtime_config` | ro / rw / rw | `gui` |
| `/dev`, `/var/run/docker.sock` | same | — | privileged hardware services / `gui` as required by their compose fragments |

### Sensor container topic contract

| Container | Publishes | Notes |
|---|---|---|
| `mowgli-gps` (`GNSS_SOURCE=direct`) | Universal GNSS receiver/status/fix/RTCM topics | External Universal GNSS receiver runtime; `mowgli-ros2` runs the Mowgli public-topic adapter for the direct path. |
| `mowgli-gps` (`GNSS_SOURCE=mavros`) | `/rtcm` (`universal_gnss_msgs/RtcmFrame`) | NTRIP-only Universal GNSS runtime; subscribes to `/mavros/universal_gnss/<gps1|gps2>/status` for GGA/status and publishes RTCM for the MAVROS Universal GNSS plugin. |
| `mowgli-mavros` | `/mavros/universal_gnss/<gps1|gps2>/status`, `/mavros/universal_gnss/<gps1|gps2>/fix`; subscribes `/rtcm` in MAVROS GNSS mode | Universal GNSS MAVROS plugin exposes the FCU GNSS through the canonical UG messages and forwards RTCM to the FCU as MAVLink `GPS_RTCM_DATA`. |
| `mowgli-lidar` | `/scan` (`sensor_msgs/LaserScan`), `frame_id: lidar_link` | ldlidar remaps `~/scan`→`/scan` and lifecycle-autostarts |

`mowgli_gnss_bridge` params (`universal_gnss_topic_bridge.cpp` L271–284): `backend` (`universal`), `receiver_family` (`auto`), `frame_id` (`gps_link`), `input_status_topic`, `output_status_topic`, `input_diagnostics_topic`, `input_rtcm_topic`, `output_rtcm_topic`. QoS: reliable depth 10 (status/diagnostics), depth 50 (RTCM).

### Host artefacts the installer creates

| Artefact | Written by |
|---|---|
| `/etc/udev/rules.d/50-mowgli.rules` → `/dev/mowgli`, `/dev/gps`, `/dev/lidar`, `/dev/tfluna_front`, `/dev/tfluna_edge`, `/dev/mavros` | `lib/udev.sh` L154 |
| `/etc/rc.local` + `/etc/systemd/system/rc-local.service` (enabled) | `lib/rc_local.sh` L3 |
| boot config: `enable_uart=1`, `dtoverlay=uart1..5`, Pi-5 `usb_max_current_enable`/fan curve | `lib/uart.sh` L124, L94 |
| `/etc/profile.d/mowgli-motd.sh` | `lib/motd.sh` L3 |
| `/usr/local/bin/mowgli-*` helpers | `lib/tools.sh` L134 |
| `docker/.env`, `docker/docker-compose.yaml`, `docker/config/**` | `lib/env.sh`, `lib/compose.sh` |

## Build, test, run

```bash
# Installer test suite (bash only, no docker needed — mocks provide sudo/apt/git/docker)
bash install/test_mowglinext.sh
for t in install/tests/test_*.sh; do bash "$t"; done

# Interactive install / upgrade / diagnose
bash install/mowglinext.sh
bash install/mowglinext.sh --check
bash install/mowglinext.sh --branch=dev --image-tag=dev --gnss=auto --gnss-connection=uart --lidar=ldlidar-uart --tfluna=none

# Stack from a dev checkout (needs docker/.env; copy docker/.env.example first)
./docker/stack.sh regen | up | pull | update | logs -f mowgli | ps | config

# Simulation
docker compose -f docker/docker-compose.simulation.yaml up dev-sim

# Sensor images still owned by this repository
docker build -t mowgli-lidar --target runtime sensors/lidar-ldlidar/

# External sidecar contract
python3 sensors/mavros/test_integration.py
```

CI: LiDAR sensor images and the OpenMower image build through their dedicated sensor workflows and the reusable sensor Docker workflow. Universal GNSS is consumed as the external `${UNIVERSAL_GNSS_IMAGE}` and is not built from `sensors/gps/Dockerfile`. `sensors-mavros.yml` runs the source-level external-sidecar contract check and never builds or publishes MowgliMAVROS. `ros2-docker.yml` builds `mowgli-ros2`, `gui-docker.yml` builds `mowglinext-gui`. `ros2-ci.yml` watches `install/config/mowgli/**` (L9, L76) for the config-drift job. **No workflow runs `install/test_mowglinext.sh` or `install/tests/*` — run them by hand before touching the installer.**

## Change coupling — "if you change X, also update Y"

- **New `.env` key** → `lib/env.sh` `setup_env` default + `upsert_env_key` → `lib/state.sh` `is_allowed_installer_key` L11–33 (otherwise presets and `.env` reload silently drop it) → `install/tests/test_env_output.sh` L38 required-key list → the compose fragment that consumes it → `docker/.env.example` if operators set it by hand.
- **New compose fragment** → `lib/compose.sh` `build_compose_stack` → `lib/checks.sh` `container_name_for_service` L3 + `expected_runtime_services` L28 → `compose_restart_services_for_backend` (`config.sh` L317) → `install/tests/test_compose_validity.sh`.
- **New installer CLI flag** → `config.sh` `parse_args` → `docs/install.sh` (the bootstrap forwards only a fixed allowlist) → the web composer under `docs/` → `install/tests/test_negative.sh`.
- **New key patched into the installed yaml** → `config.sh` `write_config` L1400–1440 → `install/config/mowgli/mowgli_robot.yaml` seed → `ros2/scripts/check_config_drift.py` field classes → GUI schema `gui/asserts/mower_config.schema.json`. Keep the seed SPARSE (CLAUDE.md Invariant 15).
- **`cyclonedds.xml`** exists twice: `install/config/cyclonedds.xml` (seed) and `docker/config/cyclonedds.xml` (tracked, actually mounted). Edit BOTH — the seed-if-absent guard never fires on a repo clone (`docker/config/cyclonedds.xml` L2–5).
- **GNSS param name change** → `sensors/gps/start_gps.sh` resolver → `install/lib/env.sh` `load_gnss_ntrip_runtime_defaults` L90 + `install/lib/checks.sh` `check_generated_gps_yaml_alignment` L175 → `install/config/mowgli/mowgli_robot.yaml` → the GUI's GNSS settings page.
- **`mowgli_gnss_bridge` behaviour** → keep `sensors/gps/universal_gnss_topic_bridge.py` behaviour-identical (it is the `GNSS_BRIDGE_IMPL=python` fallback) → `sensors/gps/mowgli_gnss_bridge/test/` → `sensors-gps.yml` smoke test asserts both executables exist.
- **universal-gnss submodule bump** → `.gitmodules` pins the **mowglinext fork** on branch `main` (both issue #395 fixes are upstream in that fork now); `sensors/gps/Dockerfile` L39–46 copies seven of its packages → re-pin the gitlink, not just the branch.
- **New udev symlink** → `lib/udev.sh` `build_dynamic_udev_rules` → `lib/checks.sh` `check_devices` L73 → the compose fragment's device path default → `install/tests/test_udev_install.sh`.
- **Image name change** → `config.sh` `recompute_image_defaults` L45–51 must match the workflow `IMAGE_NAME`/`inputs.image` exactly, and `docker/.env.example` L24–28.

## Pitfalls

- `install/compose/docker-compose.foxglove.yml` is dead and would break the stack if wired in: it passes `enable_coverage:=true`, an argument no launch file in `ros2/src` declares.
- The standalone `mowgli-ntrip` service (which used to launch `mowgli_ntrip_client mowgli_ntrip_client.launch.py` — a package that never existed in `ros2/src`) was deleted from `docker-compose.mavros.yml` entirely (commit `bfd44f1a`, 2026-09-19) — there is no NTRIP container on this backend any more, working or not.
- The gps image **must** be built with the monorepo root as context (`sensors/gps/Dockerfile` L5–8); `docker build sensors/gps/` fails because it copies from `ros2/src/`.
- `install/config/mowgli/{hardware_bridge,twist_mux,foxglove_bridge}.yaml` are never read: `mowgli.launch.py` L185–187, L271 load them from the package share dir. Only `mowgli_robot.yaml` is read from `/ros2_ws/config` (`robot_config_util.py` L31).
- `LIDAR_ENABLED` in `.env` only decides whether the **container** is composed. Flipping it does NOT change the ROS stack's LiDAR mode — that is `mowgli_robot.yaml:lidar_enabled` (`docker-compose.base.yml` L13–17; guarded by `ros2/src/mowgli_bringup/test/test_robot_config_util.py` L436–453).
- `write_compose_merged` uses `config --no-interpolate` so `${MOWGLI_ROS2_IMAGE}` stays a literal in the generated file; the Bash fallback (`compose.sh` L169–206) is a naive section-concatenator — never hand-edit `docker/docker-compose.yaml`, regenerate it.
- `write_config` NEVER overwrites an existing `docker/config/mowgli/mowgli_robot.yaml`; it line-splices ~25 keys (`config.sh` L1400–1440). Changing the seed alone does nothing on an upgraded robot.
- `write_config` removes retired ICP, loop-closure and dense-map keys from upgraded sparse configs; these entries are migration cleanup and are not runtime settings.
- `docker-compose.gps.yml` no longer has any `GNSS_*` env values at all (removed `bfd44f1a`, 2026-09-19) — the `gps` service's inline Python launcher reads `/config/mowgli_robot.yaml` directly. `sensors/gps/start_gps.sh`'s YAML→env→default resolution is ORPHANED code, not what runs in the deployed container.
- The NTRIP `centipede/centipede` fallback only applies when the caster is `crtk.net` — in BOTH `env.sh` L178–181 and `start_gps.sh` L177–215. Keep them in sync.
- TF-Luna and VESC prompts exist but are hard-gated off (`config.sh` L354–378); their compose fragments still carry `ghcr.io/...` placeholders. `install/tests/test_optional_features.sh` pins this.
- `docker/stack.sh` re-asserts `REPO_DIR`/`DOCKER_DIR` AFTER sourcing `config.sh` (L76–83) because `config.sh` recomputes them from `MOWGLI_HOME` at source time. Any new lib that caches a path at source time needs the same treatment.
- `docker/stack.sh` requires `COMPOSE_PROJECT_NAME` to stay stable (default `install`) — renaming it orphans the `install_mowgli_maps` volume holding `areas.dat` and the fusion graph.
- `migrate_runtime_paths` still backs up `docker/.env` and `docker/docker-compose.yaml` to `.old.<timestamp>` unconditionally up front on every run (the regenerated content isn't known until later steps run) — but `prune_backup_if_unchanged` (`deploy.sh`, called from `setup_env` and `run_startup_step_live`) removes the backup retroactively when the regenerated file turns out byte-identical, so a re-run that changes nothing no longer accumulates one. See `TODO-runtime-backups.md` and `install/tests/test_backup_pruning.sh`.
- `install/lib/udev.sh` L62 falls back to a `KERNEL=="ttyACMn"` rule when `udevadm` cannot resolve USB attributes — unstable across re-enumeration (the exact bug the VID/PID form fixes).
- Emergency stop is firmware-owned; nothing in this area may add a software e-stop path (CLAUDE.md Safety, Invariant 9). `install/config/mowgli/twist_mux.yaml` L33–45 documents why there is no `locks:` block — and that copy is not even loaded.

## Generated & vendored — do not hand-edit

- `docker/docker-compose.yaml`, `docker/.env`, `docker/config/mowgli/mowgli_robot.yaml`, `docker/config/cyclonedds.xml`, `docker/config/mqtt/mosquitto.conf`, `docker/config/om/mower_config.sh`, `docker/config/mowgli/drive_tuning/*`, `docker/logs/mow_sessions/*.jsonl` — runtime files, gitignored (`.gitignore` L28–48, L64–65); regenerate via `install/mowglinext.sh` or `docker/stack.sh regen`.
- `install/.preset` / `install/.preset.consumed` — optional hardware preset dropped next to the installer; read and renamed to `.consumed` by `lib/state.sh` (`mark_preset_consumed` L160). Nothing in this repo writes it — `docs/install.sh` passes web-composer choices as CLI flags instead.
- `ros2/src/external/universal-gnss` — git submodule, pinned to the mowglinext fork; the gps image copies its packages, never patch them in place.
- LiDAR driver sources (`Myzhar/ldrobot-lidar-ros2`, `Slamtec/rplidar_ros`, `ldrobotSensorTeam/ldlidar_stl_ros2`) are cloned inside the Dockerfiles; the `sed` patches there are the only supported way to modify them.

External release components: `gui/cmd/publish-deployment/definition.go` reads built/external entries in `install/deployment.json`. The workflow builds only built entries; publisher resolves external Docker Hub/GHCR index digests and both platforms. Deployment schema 3 and journal schema 5 preserve external provenance; existing storage, health, installer-selection and core-image guards still apply. See `docs/UPDATES.md`, External images in standard deployments.
