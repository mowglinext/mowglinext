# Codemap: deploy (`install/` + `docker/` + `sensors/`)

> Everything that turns this repo into a running robot: the interactive Bash installer (`install/mowglinext.sh` + `install/lib/*.sh`), the compose fragments it selects and merges into ONE generated `docker/docker-compose.yaml`, the config seeds it copies into `docker/config/`, the dockerised sensor drivers (`sensors/`), and the standalone stack manager `docker/stack.sh`. It owns the **host** side of the contract: UARTs, udev symlinks, rc.local/systemd, image tags, and `docker/.env`. It owns NO ROS node except `sensors/gps/mowgli_gnss_bridge`.
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `install/CLAUDE.md`.

## Where to look

| Task | Start here |
|------|------------|
| Trace an install end-to-end (15 steps) | `install/mowglinext.sh` `main()` L88–199 (each `progress_run*` names the lib function) |
| Add / change a `docker/.env` key | `install/lib/env.sh` `setup_env` L210–374 (defaults L216–301, writes L325–367) + whitelist `install/lib/state.sh` `is_allowed_installer_key` L11–33 + guard `install/tests/test_env_output.sh` L38 |
| "Which env var starts which container?" | `install/lib/compose.sh` `build_compose_stack` L52–138 — see the env→service table below |
| Change how the merged compose is produced | `install/lib/compose.sh` `write_compose_merged` L208–244 (`docker compose config --no-interpolate`), pure-Bash fallback L169–206 |
| Add a CLI flag / web-composer preset key | `install/lib/config.sh` `parse_args` L842–1078; preset loader `install/lib/state.sh` `load_preset_file` L133; the web bootstrap `docs/install.sh` forwards choices as CLI flags (it does not write `.preset`) |
| Change what the installer writes into `mowgli_robot.yaml` | `install/lib/config.sh` `write_config` L1327–1443 (line-splice via `_yaml_patch_key` L1286) — it patches ONLY the ~25 keys listed there |
| Change the SPARSE seed shipped to new robots | `install/config/mowgli/mowgli_robot.yaml` (CLAUDE.md Invariant 15 — defaults belong in `ros2/src/mowgli_bringup/config/mowgli_robot.yaml`) |
| GNSS backend/stack resolution logic | `install/lib/config.sh` L462–840 (`normalize_*`, `effective_gnss_backend` L609, `effective_gnss_stack` L625, `compose_gnss_service_name` L814) |
| GNSS interactive probing / baud upgrade | `install/lib/gps.sh` `configure_gps` L66; `install/lib/serial_probe.sh`; `install/lib/ublox_config.sh`, `install/lib/unicore_config.sh` |
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
| `lib/deploy.sh` | 297 | Git sync (`report_repository_sync_status`, `sync_repo_branch_to_selected_branch`, submodules), `setup_directory`, `migrate_runtime_paths`, `fix_path_type_conflict` |
| `lib/state.sh` | 173 | Strict KV parser for `install/.preset` and `docker/.env` + the allowed-key whitelist; preset consume/backup |
| `lib/udev.sh` | 232 | Static + dynamic udev rule generation and install |
| `lib/backend_choice.sh` | 271 | Mowgli-STM32 vs Pixhawk/MAVROS selection, MAVROS device detect, GCS URL |
| `lib/tools.sh` | 205 | Optional host tools + `/usr/local/bin/mowgli-{up,down,restart,logs,ps,pull,check,gps-logs,lidar-logs,shell,status}` |
| `lib/progress.sh` | 212 | Step progress bar/spinner, install log capture (`init_install_logs`) |
| `lib/gps.sh` | 176 | `pick_serial_by_id`, `preset_key_loaded`, `configure_gps` |
| `lib/platform.sh` | 163 | CPU arch / board family detection, `assert_supported_platform` |
| `lib/uart.sh` | 150 | Boot-config line upsert, `dtoverlay=uart1..5`, Bluetooth disable, Pi-5 USB/fan params |
| `lib/common.sh` | 135 | `info/warn/fail/error/step/prompt/confirm`, `detect_uart_ports`, `pick_uart_port`, `require_root*` |
| `lib/motd.sh` | 127 | Writes `/etc/profile.d/mowgli-motd.sh` (reads `~/mowglinext/docker/.env`) |
| `lib/range.sh` | 118 | TF-Luna front/edge rangefinder prompts |
| `lib/lidar.sh` | 120 | LiDAR type/connection/baud selection + `LIDAR_UART_RULE` |
| `lib/ublox_config.sh` | 114 | u-blox detect + 921600 baud upgrade (via `lib/ublox_config_helper.py`) |
| `lib/unicore_config.sh` | 107 | Unicore UM98x `CONFIG COM1` baud upgrade |
| `lib/serial_probe.sh` | 100 | NMEA baud probing, `prompt_or_probe_baud` |
| `lib/docker.sh` | 97 | `docker_cmd`/`docker_compose_cmd` (sg-docker wrapper), `install_docker` |
| `lib/system.sh` | 77 | apt update/upgrade, release pinning |
| `lib/banner.sh` | 64 | Banner + final `print_summary` (issue list) |
| `lib/rc_local.sh` | 50 | `/etc/rc.local` + `rc-local.service` (Pi UART platforms only) |
| `lib/i18n.sh` | 42 | `load_locale` / `select_language` |
| `lib/ublox_config_helper.py` | 192 | UBX CFG frame builder used by `ublox_config.sh` |
| `lib/config.local.sh.example` | 14 | Template for the gitignored `config.local.sh` fork override of `REPO_URL` (drives the GHCR prefix) |
| **`install/locale/`** | | |
| `locale/en.sh`, `locale/fr.sh` | 73 / 73 | `MSG_*` strings sourced by `load_locale` |
| **`install/compose/`** (fragments; see table below) | | |
| `compose/docker-compose.base.yml` | 62 | `mowgli` service (mowgli-ros2): `full_system.launch.py`, shared `x-ros2-env`, `mowgli_maps` volume, config mounted **RW** |
| `compose/docker-compose.gui.yml` | 36 | `gui` (mowgli-gui), `pid: host`, docker socket, `FOXGLOVE_URL=ws://localhost:8765` |
| `compose/docker-compose.gps.yml` | 68 | `gps` (mowgli-gps) — canonical Universal GNSS sidecar; passes `GNSS_*` with **no** hard defaults |
| `compose/docker-compose.mqtt.yml` | 19 | `mosquitto` (mowgli-mqtt), ports 1883/9001 |
| `compose/docker-compose.watchtower.yml` | 16 | `watchtower` (mowgli-watchtower), label-gated, 4 h poll |
| `compose/docker-compose.lidar-ldlidar.yml` | 37 | `lidar` (mowgli-lidar), no command override — image CMD |
| `compose/docker-compose.lidar-rplidar.yml` | 38 | `lidar` with `rplidar_a2m8_launch.py` + `LIDAR_PORT`/`LIDAR_BAUD` |
| `compose/docker-compose.lidar-stl27l.yml` | 38 | `lidar` with `ldlidar_stl_ros2_node` + `LIDAR_MODEL`/`LIDAR_PORT`/`LIDAR_BAUD` |
| `compose/docker-compose.mavros.yml` | 49 | `mavros` (mowgli-mavros) + `ntrip` (mowgli-ntrip) |
| `compose/docker-compose.vesc.yml` | 20 | `vesc` — gated off (`vesc_service_available` always returns 1) |
| `compose/docker-compose.tfluna-front.yml` / `-edge.yml` | 8 / 8 | TF-Luna — gated off (image placeholder `ghcr.io/...`) |
| `compose/docker-compose.foxglove.yml` | 5 | **Unused** — no code selects it; overrides `mowgli.command` with a non-existent `enable_coverage:=` arg |
| **`install/config/`** (seeds copied to `docker/config/`) | | |
| `config/mowgli/mowgli_robot.yaml` | 79 | The SPARSE installed robot config seed (Invariant 15) |
| `config/cyclonedds.xml` | 27 | Loopback-only DDS: `AllowMulticast=false`, iface `lo`, `MaxAutoParticipantIndex=500`, unicast peer `localhost` |
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
| `tests/test_ublox_config.sh` / `test_unicore_config.sh` | 161 / 89 | Baud-upgrade command sequences and no-op guards |
| `tests/test_lidar_matrix.sh` | 150 | `LIDAR_TYPE` × connection → fragment + baud + `LIDAR_IMAGE` |
| `tests/test_robot_yaml.sh` | 115 | Installed yaml shape, datum placeholders, no legacy `mower_config.sh` |
| `tests/test_bootstrap_repo_update.sh` | 111 | `docs/install.sh` stays conservative on existing checkouts |
| `tests/test_hardware_presets.sh` | 103 | mowgli vs mavros backend matrix |
| `tests/test_check_mode.sh` | 101 | `--check` targets the right services/commands |
| `tests/test_optional_features.sh` | 97 | TF-Luna / VESC never leak into the generated compose |
| `tests/test_udev_install.sh` | 87 | `install_udev_rules` under `set -u` |
| `tests/test_gps_matrix.sh` | 82 | UART and USB-by-id GNSS paths |
| `tests/test_config_dir_squat.sh` | 77 | `ensure_default_configs` heals a directory squatting a file mount |
| `tests/test_serial_probe.sh` | 68 | NMEA baud detection / failure / manual fallback |
| `tests/test_smoke.sh` | 58 | Default preset runs to completion, no network calls |
| **`docker/`** | | |
| `docker/stack.sh` | 178 | Dev-checkout stack manager reusing `install/lib/compose.sh` (`regen up down restart pull update logs ps config`) |
| `docker/README.md` | 792 | Operator deployment manual (largely stale — see stale claims) |
| `docker/.env.example` | 28 | Template for `docker/.env` (`COMPOSE_PROJECT_NAME`, `ENABLE_MQTT`, `ENABLE_WATCHTOWER`, image refs) |
| `docker/config/cyclonedds.xml` | 31 | TRACKED runtime copy actually mounted by the stack; must stay in sync with the `install/config/` seed |
| `docker/config/mowgli/README.md` | 77 | Runtime-config note (stale) |
| `docker/config/mowgli/drive_tuning/drive_pid_last_backup.yaml` | 9 | GUI drive-PID backup sample (dir otherwise gitignored) |
| `docker/docker-compose.simulation.yaml` | 125 | `simulation` / `dev-sim` / `simulation-gui` built from `ros2/Dockerfile` target `simulation` |
| `docker/docker-compose.foxglove.yaml` | 41 | Optional standalone `mowgli-foxglove` override (manual `-f` only) |
| `docker/fastdds.xml` | 36 | Reference only — Cyclone DDS is mandatory, nothing mounts this |
| `docker/diagrams/yardforce_wiring.drawio` | — | YardForce chassis wiring diagram (draw.io source, no build step) |
| `docker/logs/*.py`, `docker/logs/mow_sessions/*.py` | 37–130 each | Ad-hoc field-analysis tools (`motion_test.py`, `yaw_compare.py`, `xtrack.py`, `analyze_swath_turns.py`, `square_test.py`, …) |
| `docker/logs/mow_sessions/*.md` | 70 / 135 | Archived 2026-06-11 fusion-graph field reviews |
| **`sensors/`** | | |
| `sensors/gps/Dockerfile` | 91 | Universal GNSS sidecar image — **build context = repo root**; builds `mowgli_interfaces`, `universal_gnss_ros2`, `mowgli_gnss_bridge` + the `gnss_tools` CLI into `/opt/gnss_sidecar` |
| `sensors/gps/start_gps.sh` | 599 | Image CMD: resolves config (YAML → env → default), applies the receiver profile, then runs `receiver_node` + topic bridge + optional `ntrip_node` |
| `sensors/gps/universal_gnss_topic_bridge.py` | 406 | Retained Python bridge (`GNSS_BRIDGE_IMPL=python`) |
| `sensors/gps/ros2_entrypoint.sh` | 12 | Sources `/opt/ros/kilted` + `/opt/gnss_sidecar` |
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
| `HARDWARE_BACKEND` = `mavros` | `compose.sh` L127 → `docker-compose.mavros.yml` | `mowgli-mavros` + `mowgli-ntrip` | container env | forces `GNSS_BACKEND=disabled`, `GNSS_STACK=disabled` (`env.sh` L303–305) |
| `GNSS_STACK` (`universal`\|`disabled`), `GNSS_BACKEND` | `compose.sh` L72–95 (`compose_gnss_service_name` → service `gps`) | `mowgli-gps` | `GNSS_STACK` env; `start_gps.sh` L389 rejects `disabled` | which sidecar (if any) runs |
| `GNSS_RECEIVER_FAMILY` / `GNSS_TRANSPORT` / `GNSS_SERIAL_DEVICE` / `GNSS_SERIAL_BAUD` / `GNSS_FRAME_ID` | `docker-compose.gps.yml` L45–49 (no defaults) | `mowgli-gps` | `start_gps.sh` `resolve_*` L66–123, L269 | `receiver_node --ros-args -p receiver_family/transport/serial_device/serial_baud/frame_id` (L515–527) |
| `GNSS_NTRIP_*` (`ENABLED/HOST/PORT/MOUNTPOINT/USERNAME/PASSWORD/GGA_ENABLED/GGA_INTERVAL_S`) | same | `mowgli-gps` | `resolve_ntrip_*` L125–267 | `ntrip_node -p caster_host/caster_port/mountpoint/username/password/gga_enabled/gga_interval_s` (L553–566) |
| `LIDAR_ENABLED` + `LIDAR_TYPE` | `compose.sh` L102–117 | `mowgli-lidar` | — | **container presence only.** Deliberately NOT passed to `mowgli-ros2` (`docker-compose.base.yml` L13–17); the ROS-side LiDAR mode is `mowgli_robot.yaml:lidar_enabled` |
| `LIDAR_IMAGE` | `env.sh` L284–290 (per `LIDAR_TYPE`) | `mowgli-lidar` | image ref | — |
| `LIDAR_PORT` / `LIDAR_BAUD` / `LIDAR_MODEL` | rplidar L26–27, stl27l L26–30 | `mowgli-lidar` | `serial_port:=`/`serial_baudrate:=` (rplidar) or `-p port_name/port_baudrate/product_name` (stl27l) | driver params. **ldlidar ignores them** — hardcoded in `sensors/lidar-ldlidar/ldlidar.yaml` L6–7 |
| `ENABLE_FOXGLOVE` | `docker-compose.base.yml` L44 | `mowgli-ros2` | `full_system.launch.py enable_foxglove:=` | `foxglove_bridge` node on :8765 |
| `MOWGLI_ROS2_IMAGE`/`GPS_IMAGE`/`GUI_IMAGE`/`MAVROS_IMAGE`/`LIDAR_IMAGE` | `config.sh` `recompute_image_defaults` L41–52 | all | image refs | `ghcr.io/<owner>/<repo>/<name>:${IMAGE_TAG}` |
| `IMAGE_TAG` (`main`\|`dev`\|sanitised branch) | `config.sh` `select_image_channel` L202, `sanitize_image_tag` L61 | all | — | which tag Watchtower/`pull` fetches |
| `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`, `ROS_AUTOMATIC_DISCOVERY_RANGE` | `x-ros2-env` anchor in every ROS fragment (base/gps/lidar-*/mavros/vesc; gui, mqtt, watchtower and tfluna carry none) | ROS services | container env | Cyclone DDS via `/cyclonedds.xml` |
| `TFLUNA_FRONT_ENABLED` / `TFLUNA_EDGE_ENABLED` | `compose.sh` L119–125 via `effective_tfluna_*` | `mowgli-tfluna-front/-edge` | — | **gated off**: `range_services_available` (`config.sh` L354) returns 1 while the fragments carry the `ghcr.io/...` placeholder |
| `ENABLE_VESC` | `compose.sh` L130 via `effective_vesc_enabled` | `mowgli-vesc` | `VESC_CAN_INTERFACE` | **gated off**: `vesc_service_available` L373 always returns 1 |
| `ENABLE_MQTT` / `ENABLE_WATCHTOWER` | `docker/stack.sh` `filter_optional_fragments` L88–103 only | `mowgli-mqtt` / `mowgli-watchtower` | — | the full installer always includes both (`compose.sh` L60, L97) |
| `MOWER_IP` | — | — | — | printed by the MOTD only (`lib/motd.sh` L76) |
| `DISABLE_BLUETOOTH` | — | — | — | written but never read (Bluetooth is disabled unconditionally, `uart.sh` L146) |
| `GNSS_BACKEND`, `GPS_PROTOCOL`, `HARDWARE_BACKEND` in `docker-compose.base.yml` L19–24 | — | `mowgli-ros2` | container env | **inert** — no file under `ros2/src` reads them |

### Compose services (container names, from `install/lib/checks.sh` L3–19)

`mowgli`→`mowgli-ros2`, `gps`→`mowgli-gps`, `lidar`→`mowgli-lidar`, `gui`→`mowgli-gui`, `mosquitto`→`mowgli-mqtt`, `mavros`→`mowgli-mavros`, `ntrip`→`mowgli-ntrip`, `vesc`→`mowgli-vesc`, `tfluna_front`/`tfluna_edge`→`mowgli-tfluna-front`/`-edge`. Always composed: base + gui + mqtt + watchtower (`compose.sh` L58–60, L97). Named volume `mowgli_maps` → `/ros2_ws/maps` (base L47, L61) and also mounted into `gui` (gui L33).

### Bind mounts (host → container)

| Host path | Container path | Mode | Service |
|---|---|---|---|
| `./docker/config/mowgli` | `/ros2_ws/config` | **rw** | `mowgli` (base L52 — calibration write-back) |
| `./docker/config/mowgli` | `/config` | ro | `gps` (gps L60; `start_gps.sh` reads `/config/mowgli_robot.yaml`) |
| `./docker/config/mowgli` | `/mowgli_config` | rw | `gui` (gui L29) |
| `./docker/config/mowgli` | `/ros2_ws/config` | ro | `ntrip` (mavros L42) |
| `./docker/config/cyclonedds.xml` | `/cyclonedds.xml` | ro | every ROS service |
| `./docker/config/om` / `./docker/config/db` / `./docker` | `/config` / `/db` / `/runtime_config` | ro / rw / rw | `gui` |
| `/dev`, `/var/run/docker.sock` | same | — | privileged services / `gui` + `watchtower` |

### Sensor container topic contract

| Container | Publishes | Notes |
|---|---|---|
| `mowgli-gps` | `/gps/fix` (`sensor_msgs/NavSatFix`), `/gps/status` (`mowgli_interfaces/GnssStatus`), `/rtcm` (`rtcm_msgs/Message`), `/diagnostics` | internal hops `/_gps_internal/universal/status` and `/_gps_internal/universal/rtcm` (`start_gps.sh` L417–418) are bridged to the public names |
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

# Sensor images (gps needs the REPO ROOT as context)
docker build -t mowgli-gps -f sensors/gps/Dockerfile .
docker build -t mowgli-lidar --target runtime sensors/lidar-ldlidar/
GNSS_DRY_RUN=true GNSS_CONFIG_PATH=install/config/mowgli/mowgli_robot.yaml bash sensors/gps/start_gps.sh   # prints the commands, launches nothing
```

CI: sensor images build via `.github/workflows/sensors-{gps,lidar-ldlidar,lidar-rplidar,lidar-stl27l}.yml`, each calling the reusable `_sensor-docker.yml` (multi-arch amd64+arm64, push-by-digest then manifest merge; `sensors-gps.yml` L37–66 carries the only smoke test). `ros2-docker.yml` builds `mowgli-ros2`, `gui-docker.yml` builds `mowglinext-gui`. `ros2-ci.yml` watches `install/config/mowgli/**` (L9, L76) for the config-drift job. **No workflow runs `install/test_mowglinext.sh` or `install/tests/*` — run them by hand before touching the installer.** No workflow builds a `mavros` image, though `MAVROS_IMAGE` defaults to one.

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
- `docker-compose.mavros.yml` L39–40 launches `mowgli_ntrip_client mowgli_ntrip_client.launch.py`, but **no `mowgli_ntrip_client` package exists in `ros2/src`** — the `mowgli-ntrip` container cannot start as written.
- The gps image **must** be built with the monorepo root as context (`sensors/gps/Dockerfile` L5–8); `docker build sensors/gps/` fails because it copies from `ros2/src/`.
- `install/config/mowgli/{hardware_bridge,twist_mux,foxglove_bridge}.yaml` are never read: `mowgli.launch.py` L185–187, L271 load them from the package share dir. Only `mowgli_robot.yaml` is read from `/ros2_ws/config` (`robot_config_util.py` L31).
- `LIDAR_ENABLED` in `.env` only decides whether the **container** is composed. Flipping it does NOT change the ROS stack's LiDAR mode — that is `mowgli_robot.yaml:lidar_enabled` (`docker-compose.base.yml` L13–17; guarded by `ros2/src/mowgli_bringup/test/test_robot_config_util.py` L436–453).
- `write_compose_merged` uses `config --no-interpolate` so `${MOWGLI_ROS2_IMAGE}` stays a literal in the generated file; the Bash fallback (`compose.sh` L169–206) is a naive section-concatenator — never hand-edit `docker/docker-compose.yaml`, regenerate it.
- `write_config` NEVER overwrites an existing `docker/config/mowgli/mowgli_robot.yaml`; it line-splices ~25 keys (`config.sh` L1400–1440). Changing the seed alone does nothing on an upgraded robot.
- `write_config` L1425–1427 forces `use_scan_matching` and `use_loop_closure` to match `lidar_enabled` on every re-run — an operator who turned one off in the GUI gets it turned back on by the next installer pass.
- Empty `GNSS_*` env values in `docker-compose.gps.yml` L45–57 are deliberate ("not set"): `start_gps.sh` resolves YAML first, env second, built-in default last. Adding a compose default silently masks the operator's YAML choice.
- The NTRIP `centipede/centipede` fallback only applies when the caster is `crtk.net` — in BOTH `env.sh` L178–181 and `start_gps.sh` L177–215. Keep them in sync.
- TF-Luna and VESC prompts exist but are hard-gated off (`config.sh` L354–378); their compose fragments still carry `ghcr.io/...` placeholders. `install/tests/test_optional_features.sh` pins this.
- `docker/stack.sh` re-asserts `REPO_DIR`/`DOCKER_DIR` AFTER sourcing `config.sh` (L76–83) because `config.sh` recomputes them from `MOWGLI_HOME` at source time. Any new lib that caches a path at source time needs the same treatment.
- `docker/stack.sh` requires `COMPOSE_PROJECT_NAME` to stay stable (default `install`) — renaming it orphans the `install_mowgli_maps` volume holding `areas.dat` and the fusion graph.
- `migrate_runtime_paths` backs up `docker/.env` and `docker/docker-compose.yaml` to `.old.<timestamp>` on **every** run (`deploy.sh` L279–280); these accumulate.
- `install/lib/udev.sh` L62 falls back to a `KERNEL=="ttyACMn"` rule when `udevadm` cannot resolve USB attributes — unstable across re-enumeration (the exact bug the VID/PID form fixes).
- Emergency stop is firmware-owned; nothing in this area may add a software e-stop path (CLAUDE.md Safety, Invariant 9). `install/config/mowgli/twist_mux.yaml` L33–45 documents why there is no `locks:` block — and that copy is not even loaded.

## Generated & vendored — do not hand-edit

- `docker/docker-compose.yaml`, `docker/.env`, `docker/config/mowgli/mowgli_robot.yaml`, `docker/config/cyclonedds.xml`, `docker/config/mqtt/mosquitto.conf`, `docker/config/om/mower_config.sh`, `docker/config/mowgli/drive_tuning/*`, `docker/logs/mow_sessions/*.jsonl` — runtime files, gitignored (`.gitignore` L28–48, L64–65); regenerate via `install/mowglinext.sh` or `docker/stack.sh regen`.
- `install/.preset` / `install/.preset.consumed` — optional hardware preset dropped next to the installer; read and renamed to `.consumed` by `lib/state.sh` (`mark_preset_consumed` L160). Nothing in this repo writes it — `docs/install.sh` passes web-composer choices as CLI flags instead.
- `ros2/src/external/universal-gnss` — git submodule, pinned to the mowglinext fork; the gps image copies its packages, never patch them in place.
- LiDAR driver sources (`Myzhar/ldrobot-lidar-ros2`, `Slamtec/rplidar_ros`, `ldrobotSensorTeam/ldlidar_stl_ros2`) are cloned inside the Dockerfiles; the `sed` patches there are the only supported way to modify them.
