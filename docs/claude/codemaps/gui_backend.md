# Codemap: gui_backend

> The Go half of `gui/`: a gin HTTP/WebSocket server (`:4006`) that fronts the React bundle, talks to ROS2 **only** through `foxglove_bridge` (`ws://localhost:8765`, CDR↔JSON in `pkg/foxglove`) plus a raw teleop relay (`ws://localhost:8766`), owns the settings backend for the sparse installed `mowgli_robot.yaml` (CLAUDE.md Invariant 15), a bitcask key-value DB, the mowing scheduler + IrriSense soil gate, session statistics, Docker-driven tools (GNSS configurator, drive tuning, rosbag), firmware flashing, and the optional MQTT/HomeKit bridges.
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `gui/CLAUDE.md`. Frontend (`gui/web/`) is a separate codemap.

## Where to look
| Task | Start here |
|------|------------|
| Add / change an HTTP route | `gui/pkg/api/api.go:37-61` (`NewAPI` registers every `*Routes` fn) → the per-feature file; add `// @Router` swag annotations |
| Expose a new ROS topic to the browser | `gui/pkg/providers/ros.go:28-70` (`topicMap`) **and** `gui/pkg/api/mowglinext.go:68-85` (`topicSubscribeInterval`) — both must list the key; optional adapter in `foxgloveAdapters` (`ros.go:200`), decimation in `upstreamDecimationMs` (`ros.go:213`) |
| Call a new ROS service from the GUI | `gui/pkg/api/mowglinext.go:541-724` (`ServiceRoute` switch) — pattern: `provider.CallService(ctx, "/node/srv", &req, &res, "pkg/srv/Type")` with a `WithTimeout` ctx |
| Settings save / sparse-prune / reset-to-default | `gui/pkg/api/settings.go:1344-1445` (`PostSettingsYAML`) → `sparsifyFlat` `:388`, `pruneNestedKeys` `:404`, `retiredParamKeys` `:367`, `valuesEqual` `:335` |
| Settings schema (defaults, groups, node mapping) | `gui/asserts/mower_config.schema.json` (12 groups, `properties.<group>.properties.<key>.default`); loaded by `getSchema` `settings.go:1032` (+ `applyMowgliOverlay` `:952`); `extractNodeMappings` `:212` |
| Schema ↔ ROS2 template parity | `gui/pkg/api/schema_template_parity_test.go` (`TestSchemaDefaultsMatchTemplate`, allowlist `schemaDefaultsWithNoTemplateEntry`) |
| GNSS receiver configure / plan / apply / restart | `gui/pkg/api/gnss.go` (`runApplyFlow` `:244`, `buildGNSSApplyCommand` `:518`, container `mowgli-gps` `:19`), env/yaml resolution in `gui/pkg/api/gnss_runtime_config.go`, `GNSS_*` env derivation `settings.go:685-848` |
| foxglove protocol / CDR bug | `gui/pkg/foxglove/client.go` (subscribe/publish/CallService/reconnect), `cdr.go` (`ParseSchema`, `DeserializeCDR`), `cdr_write.go` (`SerializeCDR`, XCDR1 header `:19-27`), `parameters.go` (get/setParameters ops) |
| Per-topic JSON reshaping for the frontend | `gui/pkg/providers/transform.go` (`adaptGPS` `:254`, `adaptPose` `:287`, `adaptGnssStatus` `:325`, `adaptLidar` `:222`) |
| Manual-mow joystick lag / cmd_vel path | `gui/pkg/providers/cmd_vel_relay.go` + `ros.go:547-552` (`Publish` prefers relay for `/cmd_vel_teleop`); server side `ros2/src/mowgli_bringup/scripts/cmd_vel_ws_relay.py` |
| Scheduler fires / does not fire | `gui/pkg/providers/scheduler.go` (`checkSchedules` `:118`, `safeToStart` `:227`, `shouldRun` `:245`); CRUD in `gui/pkg/api/schedules.go` |
| IrriSense soil gate | `gui/pkg/providers/irrisense.go` (poll/backoff/`SoilStatus`), rule `irrisense_wetness.go`, DB keys `irrisense_config.go:18-28`, HTTP `irrisense_client.go`, API `gui/pkg/api/irrisense.go` |
| Session statistics wrong | `gui/pkg/providers/session_tracker.go` (`OnHighLevelStatus` `:174`, `OnOdometry` `:138`, DB key `mowing.sessions` `:80`); read/delete in `gui/pkg/api/diagnostics.go:332-443` |
| Map polling / dock pose shown on map | `gui/pkg/providers/ros.go:351-468` (`initDockPoseSubscription`, `pollMap` every 5 s → virtual `"map"` topic) |
| Map save / replace / OpenMower import | `gui/pkg/api/mowglinext.go:156-229` (`mapWriteBudget`, `replaceMapInternal`), `gui/pkg/api/openmower_import.go` (`postImportOpenMower` `:187`, reprojection `:497-596`, `gui/pkg/api/utm.go`) |
| Container list / logs / restart | `gui/pkg/api/containers.go` (WS log stream `:128`, `StreamContainerLogLines` `:196`), `gui/pkg/providers/docker.go` |
| Firmware flash (prebuilt / custom / Vermut) | `gui/pkg/providers/firmware.go` (`FlashFirmware` `:76` routing, `flashPrebuilt` `:258`, `postFlashProtocolCheck` `:322`), manifest `firmware_manifest.go:21`, template `gui/asserts/board.h.template`, SSE route `gui/pkg/api/setup.go:27` |
| Drive PID / feed-forward tuning tool | `gui/pkg/api/drive_tuning.go` (`docker exec` into `mowgli-ros2` → `ros2 run mowgli_tools tune_drive_pid`, `:829-842`; reports under `/ros2_ws/config/drive_tuning`) |
| Rosbag record / download | `gui/pkg/api/rosbag.go` (state on disk `/ros2_ws/maps/rosbags/<name>/.rosbag.pid`, `ROSBAG_DIR` override `:106`) |
| DB key / env fallback / default | `gui/pkg/providers/db.go:21-57` (`EnvFallbacks`, `Defaults`), bitcask at `$DB_PATH` with corruption auto-recovery `:108` |
| Host reboot / shutdown | `gui/pkg/api/system.go:56-66` (`nsenter -t 1 … systemctl`; needs `pid: host` + `privileged` from `install/compose/docker-compose.gui.yml:17-18`) |
| Live ROS param read/write (no restart) | `gui/pkg/api/params.go` → `ros.go:555-570` → `foxglove/parameters.go` |
| MQTT / HomeKit bridge | `gui/pkg/providers/mqtt.go` (embedded mochi broker, `<prefix>/<key>` + `<prefix>/call<service>`), `gui/pkg/providers/homekit.go` (hap switch, `:8000`, pin `system.homekit.pincode`) |
| Static bundle / gzip sidecars / SPA fallback | `gui/pkg/api/web_static.go` (`registerWebUI`), gzip produced in `gui/Dockerfile:10-12` |
| Regenerate Go/TS msg types after a `.msg`/`.srv` change | `gui/generate_go_msgs.sh` → `gui/pkg/msgs/*/…_generated.go`; `gui/generate_ts_types.sh` → `gui/web/src/types/ros.generated.ts`; drift gate `.github/workflows/msg-codegen-drift.yml` |

## Files
| File | Lines | Purpose |
|------|-------|---------|
| **root** | | |
| `gui/main.go` | 40 | Wires providers (DB → Docker → Ros → Firmware → optional HomeKit/MQTT → IrriSense → Scheduler) then `api.NewAPI` |
| `gui/go.mod` | 107 | Module `github.com/mowglinext/mowglinext`, Go 1.24; gin, gorilla/websocket, bitcask, mochi-mqtt, brutella/hap, docker client, swaggo |
| `gui/Makefile` | 18 | `deps`, `build` (docker), `run-gui` (web), `run-backend` (`CGO_ENABLED=0 go run main.go`) |
| `gui/Dockerfile` | 54 | 4-stage image: go build → `yarn build` + gzip sidecars → ubuntu 22.04 deps with openocd + platformio → final assembly; `WORKDIR /app`, `WEB_DIR=/app/web`, `DB_PATH=/app/db` |
| `gui/Dockerfile.msg` | 8 | Tiny bash image whose CMD is `generate_go_msgs.sh` (README's `generate-msg` tag) |
| `gui/tygo.yaml` | 26 | tygo config from the OpenMower era (references `pkg/msgs/mower_msgs`, `xbot_msgs`, `dynamic_reconfigure` — none exist); unused by the scripts |
| `gui/generate_go_msgs.sh` | 558 | Bash generator: embeds std ROS msgs, parses `ros2/src/mowgli_interfaces/{msg,srv}` → `pkg/msgs/{geometry,nav,sensor,std,visualization,mowgli}` |
| `gui/generate_ts_types.sh` | 331 | Same parser → `gui/web/src/types/ros.generated.ts` (snake_case fields) |
| `gui/README.md` | 157 | Dev/deploy notes; MQTT + env sections are OpenMower-era and stale (see § Pitfalls) |
| `gui/.env`, `gui/.env.dist` | 10 / 5 | Local-dev env loaded by `godotenv.Load()` in `main.go` (`DB_PATH`, `WEB_DIR`, `DOCKER_HOST`, `MQTT_ENABLED`, …) |
| `gui/.devcontainer/{devcontainer.json,Dockerfile}` | 17 / 16 | VS Code / WebStorm devcontainer used by `make deps` + `make run-*` |
| **asserts/** | | |
| `gui/asserts/mower_config.schema.json` | 838 | Settings JSON Schema = GUI's default source (12 groups: `sensor_extrinsics` plus `{hardware,gps,docking,mowing,battery,safety,obstacle,mapping,rain,led,nav}_settings`). No `x-yaml-node` entries → every key nests under `mowgli.ros__parameters` |
| `gui/asserts/board.h.template` | 340 | Go `text/template` for the STM32 `board.h` (custom firmware build) |
| `gui/asserts/board.h` | 330 | Rendered example of the template (not consumed by code) |
| **pkg/api/** (gin handlers) | | |
| `gui/pkg/api/api.go` | 64 | `NewAPI`: CORS allow-all, static UI, `/api` group, route registration order, swagger, tile proxy gate, `r.Run(system.api.addr)` |
| `gui/pkg/api/mowglinext.go` | 724 | ROS-facing routes: service switch, map add/clear/replace/dock, WS subscribe/publish/multiplex (msgpack frames), `topicSubscribeInterval` |
| `gui/pkg/api/settings.go` | 1445 | YAML settings (flatten/nest/sparsify), schema loader + Mowgli overlay, `GNSS_*` env derivation, `writeRuntimeEnvFile`, legacy `mower_config.sh` routes, `writePreservingPerms` |
| `gui/pkg/api/gnss.go` | 957 | `/settings/gnss/*`: builds `gnss_config_plan/apply` CLI runs in a throwaway `mowgli-gps` image container, restart, baud persistence |
| `gui/pkg/api/gnss_runtime_config.go` | 362 | Resolves yaml vs `.env` fallback per GNSS field, enumerates `/dev/serial/by-id/*`, `/dev/ttyUSB*`, `/dev/ttyACM*` |
| `gui/pkg/api/drive_tuning.go` | 1394 | `/tools/drive/*`: async `docker exec` jobs, YAML report parsing/validation, rollback, `persistRobotYamlUpdates` |
| `gui/pkg/api/rosbag.go` | 508 | `/tools/rosbag/*`: `ros2 bag record -a` in `mowgli-ros2`, tar.gz download, traversal-safe names |
| `gui/pkg/api/openmower_import.go` | 1026 | `/import/openmower`: parse OpenMower `map.json` (+legacy bag form), UTM datum reprojection, preview or apply |
| `gui/pkg/api/diagnostics.go` | 451 | `/diagnostics/*`: snapshot (containers, CPU temp, dock/datum cross-checks), firmware debug toggle, sessions CRUD/stats, SLAM 410 stubs |
| `gui/pkg/api/calibration.go` | 213 | IMU-yaw / magnetometer / one-click dock calibration service calls (150 s budget) |
| `gui/pkg/api/calibration_status.go` | 240 | `/calibration/status`: dock pose from yaml, IMU/mag calibration files under `/ros2_ws/maps` |
| `gui/pkg/api/containers.go` | 234 | Docker list/start/stop/restart + WS log stream with stdcopy demux |
| `gui/pkg/api/schedules.go` | 224 | Schedule CRUD (`schedule:<id>` DB keys, validation) |
| `gui/pkg/api/irrisense.go` | 228 | IrriSense settings/status/gardens (token masked, write-only) |
| `gui/pkg/api/weather.go` | 158 | `/weather`: open-meteo at the yaml datum, 10 min cache |
| `gui/pkg/api/ntrip.go` | 139 | `/ntrip/sourcetable`: fetch + parse a caster sourcetable |
| `gui/pkg/api/utm.go` | 138 | WGS84↔UTM + grid convergence (import reprojection) |
| `gui/pkg/api/web_static.go` | 123 | Static bundle, `.gz` sidecar negotiation, no-cache HTML shell, SPA `NoRoute` |
| `gui/pkg/api/config.go` | 116 | Raw DB key get/set + `/config/envs` (tile URI) |
| `gui/pkg/api/system.go` | 101 | CPU temp, host reboot/poweroff via `nsenter` |
| `gui/pkg/api/utils.go` | 65 | `unmarshalROSMessage` (mapstructure, case/underscore-insensitive), `snakeToCamel` |
| `gui/pkg/api/params.go` | 64 | Live ROS2 parameter list/set |
| `gui/pkg/api/setup.go` | 61 | `/setup/flashBoard` SSE stream around `FlashFirmware` |
| `gui/pkg/api/tiles.go` | 47 | Reverse proxy `/tiles/*` → `system.map.tileServer` |
| `gui/pkg/api/types.go` | 31 | `OkResponse`, `ErrorResponse`, small DTOs |
| **pkg/providers/** | | |
| `gui/pkg/providers/ros.go` | 586 | `RosProvider` (IRosProvider): `topicMap`, lazy upstream subscribe, per-listener mailbox `RosSubscriber`, map polling, dock pose cache, param passthrough |
| `gui/pkg/providers/transform.go` | 509 | Per-topic adapters NavSatFix→AbsolutePose, Odometry→AbsolutePose, universal GnssStatus→mowgli shape, LaserScan decimation (≤360 beams) |
| `gui/pkg/providers/firmware.go` | 373 | Flash routing (prebuilt/custom/Vermut), openocd + platformio invocations, post-flash handshake check |
| `gui/pkg/providers/docker.go` | 339 | Docker SDK wrapper (list/logs/start/stop/restart/inspect/run/exec) |
| `gui/pkg/providers/session_tracker.go` | 333 | Mowing session state machine from `highLevelStatus` + odometer from `wheelOdom`; keeps last 500 |
| `gui/pkg/providers/irrisense.go` | 289 | Poll loop (10 min, backoff 1→30 min), `SoilStatus` verdict |
| `gui/pkg/providers/scheduler.go` | 267 | 1-min ticker → `COMMAND_START` (=1) via `/behavior_tree_node/high_level_control` |
| `gui/pkg/providers/irrisense_config.go` | 237 | `irrisense.*` DB keys, defaults, validation, masking |
| `gui/pkg/providers/db.go` | 203 | bitcask DB, env fallbacks, defaults, corruption backup+recovery |
| `gui/pkg/providers/irrisense_wetness.go` | 145 | Pure wetness rule (`EvaluateWetness`, `EvaluateZones`) |
| `gui/pkg/providers/mqtt.go` | 144 | Embedded MQTT broker (`system.mqtt.host`, default `:1883`) bridging topics + 4 service calls |
| `gui/pkg/providers/firmware_manifest.go` | 133 | Release `manifest.json` fetch, permutation lookup, sha256 download |
| `gui/pkg/providers/cmd_vel_relay.go` | 132 | Persistent WS client to `ws://localhost:8766` |
| `gui/pkg/providers/irrisense_client.go` | 116 | HTTP client for `/api/ha/gardens[/{id}]` (401/404/429 mapped) |
| `gui/pkg/providers/homekit.go` | 111 | HAP switch accessory ("MowgliNext"), on→START(1) / off→HOME(2) |
| `gui/pkg/providers/irrisense_model.go` | 53 | Wire structs mirroring IrriSense ReadOnlyGarden/Zone |
| **pkg/foxglove/** | | |
| `gui/pkg/foxglove/client.go` | 952 | foxglove ws-protocol client (`foxglove.sdk.v1`): channels, subscribe/unsubscribe, binary publish, CDR service calls, decimators, reconnect |
| `gui/pkg/foxglove/cdr.go` | 651 | `.msg` schema parser (multi-block, dependency-order-safe) + CDR deserializer |
| `gui/pkg/foxglove/cdr_write.go` | 263 | JSON→CDR serializer (XCDR1 LE, 8-byte max align, empty-body padding) |
| `gui/pkg/foxglove/protocol.go` | 156 | Opcodes + JSON envelope structs of the protocol |
| `gui/pkg/foxglove/parameters.go` | 124 | `getParameters` / `setParameters` request/response routing |
| **pkg/types/** | | |
| `gui/pkg/types/ros.go` | 51 | `IRosProvider`, `RosParameter` |
| `gui/pkg/types/soil.go` | 49 | `SoilStatus.BlocksScheduledMowing()`, `ISoilProvider` |
| `gui/pkg/types/docker.go` | 76 | `IDockerProvider`, run/exec specs |
| `gui/pkg/types/firmware.go` | 45 | `IFirmwareProvider`, `FirmwareConfig` (JSON body of flashBoard) |
| `gui/pkg/types/db.go` | 15 | `IDBProvider` |
| `gui/pkg/types/homekit.go` | 5 | `IHAProvider` |
| `gui/pkg/types/mocks.go` | 145 | `MockDBProvider`, `MockRosProvider` (records `ServiceCalls`, `Dispatch`) used by all api/provider tests |
| **pkg/msgs/** (hand-written only) | | |
| `gui/pkg/msgs/mowgli/types.go` | 24 | GUI-internal `Map` (virtual topic payload), `DockingSensor` placeholder |
| `gui/pkg/msgs/mowgli/services.go` | 15 | GUI-internal `ReplaceMapReq` (not a ROS srv) |
| **tests** | ~8.7k | 36 `_test.go` files — listed in § Build, test, run |

## Runtime surface
### HTTP / WebSocket routes (all under `/api` unless noted; `WS` = websocket upgrade, `SSE` = event stream)
| Method + path | Handler (file:line) | Notes |
|---|---|---|
| `POST /config/keys/get`, `POST /config/keys/set`, `GET /config/envs` | `gui/pkg/api/config.go:27,59,94` | raw DB keys; envs = `tileUri` (empty unless `system.map.enabled`) |
| `GET/POST /settings/status` | `gui/pkg/api/settings.go:106,123` | `onboarding.completed` |
| `GET/POST /settings` | `settings.go:1178,1081` | legacy `mower_config.sh` (`system.mower.configFile`) |
| `GET /settings/schema` | `settings.go:1248` | schema + Mowgli overlay, 1 h cache |
| `GET /settings/yaml`, `GET /settings/yaml/defaults`, `POST /settings/yaml` | `settings.go:1273,1319,1345` | flat map ↔ `mowgli_robot.yaml`; POST also writes `GNSS_*` into `system.mower.runtimeEnvFile`; `null` value = delete key |
| `GET /settings/gnss/runtime-config` | `gui/pkg/api/gnss.go:105` → `gnss_runtime_config.go:74` | yaml vs `.env` source per field + serial device list |
| `POST /settings/gnss/{plan,apply,factory-reset-apply,restart}` | `gnss.go:106-109` (`:112,149,180,211`) | apply needs `confirm`; factory-reset needs `confirm_factory_reset`; runs `/opt/gnss_sidecar/bin/gnss_config_{plan,apply}` in the `mowgli-gps` image |
| `GET /containers/` | `gui/pkg/api/containers.go:39` | |
| `POST /containers/:containerId/:command` | `containers.go:75` | `start|stop|restart` |
| `GET /containers/:containerId/logs` (WS) | `containers.go:128` | base64 text frames, `--tail 100 --timestamps` |
| `POST /mowglinext/call/:command` | `gui/pkg/api/mowglinext.go:542` | see command table below |
| `POST /mowglinext/map/area/add` | `mowglinext.go:99` | `/map_server_node/add_area` |
| `DELETE /mowglinext/map` | `mowglinext.go:134` | `/map_server_node/clear_map` |
| `PUT /mowglinext/map` | `mowglinext.go:211` | clear → add×N → `save_areas`, budget `mapWriteBudget` (60 s + 5 s/area, cap 6 min) |
| `POST /mowglinext/map/docking` | `mowglinext.go:253` | `/map_server_node/set_docking_point` |
| `GET /mowglinext/subscribe/:topic` (WS) | `mowglinext.go:278` | one topic, base64 JSON text frames |
| `GET /mowglinext/multiplex` (WS) | `mowglinext.go:361` | `{"op":"subscribe|unsubscribe","topic":key}` in, msgpack `{topic,data}` binary frames out |
| `GET /mowglinext/publish/:topic` (WS) | `mowglinext.go:317` | body `geometry_msgs/TwistStamped` JSON → `/cmd_vel_teleop` (path param ignored) |
| `POST /setup/flashBoard` (SSE) | `gui/pkg/api/setup.go:27` | body `types.FirmwareConfig` |
| `GET /system/info`, `POST /system/reboot`, `POST /system/shutdown` | `gui/pkg/api/system.go:21-23` | |
| `GET /diagnostics/snapshot`, `POST /diagnostics/firmware_debug` | `gui/pkg/api/diagnostics.go:115,116` | firmware_debug → `/hardware_bridge/set_firmware_debug` (`std_srvs/SetBool`) |
| `GET /diagnostics/slam/info`, `POST /diagnostics/slam/{save,delete}` | `diagnostics.go:122-124` | always `410 Gone` |
| `GET/POST/DELETE /diagnostics/sessions`, `GET /diagnostics/sessions/stats` | `diagnostics.go:127-130` | DB key `mowing.sessions` |
| `GET /tools/rosbag/status`, `POST …/start`, `POST …/stop`, `GET …/download/:name`, `DELETE …/:name` | `gui/pkg/api/rosbag.go:121-125` | container `mowgli-ros2` |
| `GET /weather` | `gui/pkg/api/weather.go:41` | |
| `GET/POST /params` | `gui/pkg/api/params.go:17-18` | foxglove `parameters` capability |
| `GET /ntrip/sourcetable?host&port&user&pass` | `gui/pkg/api/ntrip.go:42` | |
| `POST /calibration/imu-yaw`, `POST /calibration/magnetometer`, `POST /calibration/dock/start`, `GET /calibration/status` | `gui/pkg/api/calibration.go:55-57`, `calibration_status.go:91` | services on `/calibrate_imu_yaw_node/{calibrate,dock_calibration/start}` |
| `POST /tools/drive/ff-calibration/start`, `POST …/pid-tuning/start`, `POST …/tuning/rollback`, `GET …/tuning/status`, `GET …/tuning/report/latest` | `gui/pkg/api/drive_tuning.go:297-301` | container `mowgli-ros2` |
| `GET/POST /schedules`, `PUT/DELETE /schedules/:id` | `gui/pkg/api/schedules.go:35-38` | |
| `GET /irrisense/status`, `GET/PUT /irrisense/settings`, `GET /irrisense/gardens` | `gui/pkg/api/irrisense.go:64-67` | |
| `POST /import/openmower` | `gui/pkg/api/openmower_import.go:162` | `{map, om_datum_lat?, om_datum_lon?, apply}` — preview unless `apply` |
| `ANY /tiles/*proxyPath` (root, not `/api`) | `gui/pkg/api/tiles.go:46` | only when `system.map.enabled=true` |
| `GET /swagger/*any` (root) | `gui/pkg/api/api.go:62` | from `gui/docs` |
| `GET /`, `/assets/*`, SPA fallback (root) | `gui/pkg/api/web_static.go:19-25` | |

`/mowglinext/call/:command` → ROS service (`mowglinext.go:554-717`): `high_level_control`→`/behavior_tree_node/high_level_control`; `emergency`→`/hardware_bridge/emergency_stop`; `mow_enabled`→`/hardware_bridge/mower_control`; `start_in_area`→`/behavior_tree_node/start_in_area`; `set_datum`→`/navsat_to_absolute_pose/set_datum`; `promote_obstacle`→`/map_server_node/promote_obstacle`; `discard_obstacle`→`/map_server_node/discard_obstacle`; `fusion_graph_save|clear`→`/fusion_graph_node/{save_graph,clear_graph}`; `coverage_clear_resume`→`/behavior_tree_node/clear_coverage_resume`; `reboot_board`→`/hardware_bridge/reboot_board`. All 10 s timeout.

### foxglove_bridge consumption (`gui/pkg/providers/ros.go`)
| Logical key (browser) | ROS2 topic | Type | Adapter / decimation / throttle |
|---|---|---|---|
| `status` | `/hardware_bridge/status` | `mowgli_interfaces/msg/Status` | — |
| `highLevelStatus` | `/behavior_tree_node/high_level_status` | `mowgli_interfaces/msg/HighLevelStatus` | feeds SessionTracker, scheduler, HomeKit |
| `dockCalibrationStatus` | `/calibrate_imu_yaw_node/dock_calibration/status` | `mowgli_interfaces/msg/DockCalibrationStatus` | — |
| `gps` | `/gps/fix` | `sensor_msgs/msg/NavSatFix` | `adaptGPS`→AbsolutePose; 80 ms / 100 ms |
| `gnssStatus` | `/gps/status` | `mowgli_interfaces/msg/GnssStatus` | `adaptGnssStatus`; 80 / 100 ms |
| `pose`, `fusionRaw` | `/odometry/filtered_map` | `nav_msgs/msg/Odometry` | `adaptPose` (pose only); 80 / 100–200 ms |
| `imu` | `/imu/data` | `sensor_msgs/msg/Imu` | 80 / 100 ms |
| `ticks` | `/wheel_ticks` | `mowgli_interfaces/msg/WheelTick` | 80 / 100 ms |
| `wheelOdom` | `/wheel_odom` | `nav_msgs/msg/Odometry` | 80 / 100 ms; feeds session odometer |
| `lidar` | `/scan` | `sensor_msgs/msg/LaserScan` | `adaptLidar` (≤360 beams); 80 / 100 ms |
| `map` | *(virtual)* | `mowgli.Map` | `pollMap` every 5 s via `/map_server_node/get_mowing_area` + cached `/map_server_node/docking_pose` |
| `path` / `plan` | `/coverage/full_plan` / `/plan` | `nav_msgs/msg/Path` | unthrottled |
| `power`, `emergency` | `/hardware_bridge/power`, `/hardware_bridge/emergency` | `mowgli_interfaces/msg/{Power,Emergency}` | unthrottled |
| `mowProgress` | `/map_server_node/mow_progress` | `nav_msgs/msg/OccupancyGrid` | 500 ms |
| `diagnostics`, `fusionDiag` | `/diagnostics`, `/fusion_graph/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | unthrottled |
| `icpOdom` | `/fusion_graph/icp_odometry` | `nav_msgs/msg/Odometry` | 200 ms |
| `obstacles` | `/obstacle_tracker/obstacles` | `mowgli_interfaces/msg/ObstacleArray` | 200 ms |
| `btLog`, `robotDescription`, `recordingTrajectory`, `coverageResumeAvailable` | `/behavior_tree_log`, `/robot_description`, `/behavior_tree_node/recording_trajectory`, `/behavior_tree_node/coverage_resume_available` | `nav2_msgs/msg/BehaviorTreeLog`, `std_msgs/msg/String`, `nav_msgs/msg/Path`, `std_msgs/msg/Bool` | unthrottled |
| `cogHeading`, `magYaw` | `/imu/cog_heading`, `/imu/mag_yaw` | `sensor_msgs/msg/Imu` | 150 / 200 ms |

Publishes: `/cmd_vel_teleop` (`geometry_msgs/msg/TwistStamped`) — via relay `ws://localhost:8766` when connected, else foxglove `clientAdvertise` with `encoding: json`. Upstream subscriptions are lazy (first listener subscribes, last listener unsubscribes, `ros.go:263-320`); MQTT/HomeKit/scheduler listeners keep their keys permanently subscribed.

### Key-value DB (`bitcask` at `$DB_PATH`; `gui/pkg/providers/db.go`)
`system.api.addr` (`API_ADDR`, `:4006`), `system.api.webDirectory` (`WEB_DIR`, `/app/web`), `system.map.{enabled,tileServer,tileUri}` (`MAP_TILE_*`), `system.homekit.{enabled,pincode}` (`HOMEKIT_ENABLED`, `HOMEKIT_PINCODE`), `system.mqtt.{enabled,host,prefix}` (`MQTT_*`), `system.mower.configFile` (`MOWER_CONFIG_FILE`, `/config/mower_config.sh`), `system.mower.yamlConfigFile` (`MOWER_YAML_CONFIG_FILE`, default `/config/mowgli_robot.yaml`; compose sets `/mowgli_config/mowgli_robot.yaml`), `system.mower.runtimeEnvFile` (`MOWER_RUNTIME_ENV_FILE`, `/runtime_config/.env`), `system.ros.foxgloveUrl` (`FOXGLOVE_URL`, `ws://localhost:8765`), `system.ros.{masterUri,nodeName,nodeHost}` (declared, read by nothing). App keys: `onboarding.completed`, `schedule:<id>`, `mowing.sessions`, `gui.firmware.config`, `irrisense.{enabled,baseUrl,token,gardenId,zoneIds,wetDeficitMm,dryAfterWateringHours,maxStaleMinutes,gateScheduler}`.

### Other integrations
- **MQTT** (`mqtt.go`, when `system.mqtt.enabled=true`): broker on `system.mqtt.host`; publishes retained `<prefix>/<key>` for `highLevelStatus,status,pose,gps,imu,ticks,wheelOdom,map,path,plan,mowingPath` (prefix default `/gui`); subscribes `<prefix>/call/behavior_tree_node/high_level_control`, `…/hardware_bridge/emergency_stop`, `…/hardware_bridge/mower_control`, `…/behavior_tree_node/start_in_area` (JSON body = request struct).
- **HomeKit** (`homekit.go`, when `system.homekit.enabled=true`): HAP server `:8000`, switch ON while state is `MOWING|DOCKING|UNDOCKING`.
- **IrriSense**: `GET <baseUrl>/api/ha/gardens[/<id>]` with Bearer token, 10 min poll, fail-open (see CLAUDE.md § high-level-api).
- **External HTTP**: open-meteo (`weather.go:102`), NTRIP caster (`ntrip.go:69`), GitHub release `manifest.json` (`firmware_manifest.go:21`), Docker socket (`DOCKER_HOST`).
- **Files read**: `asserts/mower_config.schema.json` (**relative to CWD**, `settings.go:1043`), `/ros2_ws/maps/{imu_calibration.txt,mag_calibration.yaml}`, `/sys/class/thermal/thermal_zone0/temp`. **Files written**: `mowgli_robot.yaml` + runtime `.env` (settings, gnss, drive_tuning rollback), legacy `mower_config.sh`, `/ros2_ws/maps/rosbags/*`, `$TMPDIR/firmware_prebuilt.bin`.
- **Compose contract** (`install/compose/docker-compose.gui.yml`): `network_mode: host`, `pid: host`, `privileged`, env `FOXGLOVE_URL`, `MOWER_CONFIG_FILE`, `MOWER_YAML_CONFIG_FILE`, `MOWER_RUNTIME_ENV_FILE`, `DOCKER_HOST`, `DB_PATH`; volumes `./docker/config/db:/db`, `./docker/config/om:/config:ro`, `./docker/config/mowgli:/mowgli_config`, `./docker:/runtime_config`, `/var/run/docker.sock:/var/run/docker.sock`, `/dev:/dev`, `mowgli_maps:/ros2_ws/maps`.

## Build, test, run
```bash
cd gui && CGO_ENABLED=0 go build -o mowglinext .        # what gui/Dockerfile does (Makefile run-backend = go run main.go)
cd gui && go test ./...                                  # all Go unit tests (~30 s; no ROS/Docker needed)
cd gui && go test ./pkg/api -run 'TestSchemaDefaultsMatchTemplate|TestLed'   # schema↔template parity (reads ../ros2/src/mowgli_bringup/config/mowgli_robot.yaml)
cd gui && ./generate_go_msgs.sh && ./generate_ts_types.sh && git diff --exit-code pkg/msgs web/src/types/ros.generated.ts
docker build -t mowglinext-gui gui/                      # multi-arch image; CI: .github/workflows/gui-docker.yml
```
**CI:** `gui-docker.yml` (image build only), `gui-ci.yml` (web tsc/eslint/vitest only — `working-directory: gui/web`), `msg-codegen-drift.yml` (regenerates Go+TS msgs and fails on diff). **No workflow runs `go test`** — run it locally before pushing Go changes.

Tests (what each pins):
- `gui/pkg/api/settings_test.go` — legacy + YAML settings round-trips, sparse prune, reset-to-default, retired-key scrub, GNSS normalisation, env fallback precedence, legacy `GPS_*` env purge (uses `chdirToGuiRoot` so `asserts/` resolves).
- `gui/pkg/api/schema_template_parity_test.go` — every schema default equals the ROS2 template value or is on the allowlist. `settings_leds_test.go` — `led_enabled` default false + LED SPI clock assumption.
- `gui/pkg/api/gnss_test.go` — plan/apply/factory-reset command construction, confirm gates, device validation, restart-on-success, baud persistence stays sparse.
- `gui/pkg/api/mowglinext_test.go` — service switch → correct ROS service, map routes, multiplex/subscribe WS fan-out, `topicSubscribeInterval` covers every known key, `mapWriteBudget`.
- `gui/pkg/api/openmower_import_test.go`, `openmower_import_realmap_test.go` (+`testdata/openmower_legacy_real.json`) — parsing (bare/wrapped/legacy), reprojection, apply = clear→add→save→dock. `utm_test.go` — UTM reference values.
- `gui/pkg/api/rosbag_test.go` — status/start/stop/download/delete via mocked docker exec, traversal rejection. `drive_tuning_command_test.go`, `drive_tuning_report_test.go` — CLI args, report validation tiers, NaN sanitising.
- `gui/pkg/api/irrisense_test.go` — token masking/write-only, validation, error classes. `params_test.go` — fractional `ticks_per_meter` survives. `config_test.go`, `containers_logstream_test.go` (docker stream demux), `diagnostics_*_test.go`, `ntrip_test.go`, `weather_test.go`, `web_static_test.go` (gzip sidecars, no-cache shell), `utils_test.go`.
- `gui/pkg/providers/scheduler_test.go`, `scheduler_soil_test.go` — due-time matching, safety gate, 2-min double-fire guard, soil gate fail-open. `session_tracker_test.go` — 20 s noise floor, recharge pause, coverage/strips peak, odometer. `irrisense_test.go`, `irrisense_wetness_test.go` — poll/backoff/stale, wetness rule. `transform_test.go` — adapter shapes. `db_test.go` — env/default precedence, corruption recovery. `firmware_test.go` — routing + board.h template. `ros_subscriber_test.go` — coalescing mailbox.
- `gui/pkg/foxglove/cdr_test.go`, `realwire_test.go`, `obstacle_wire_test.go`, `client_test.go` — XCDR1/CDR2 alignment, real captured Imu/ObstacleArray frames, NaN sanitising. `gui/pkg/msgs/mowgli/mower_control_bind_test.go` — snake_case JSON binding. `gui/pkg/types/mocks_test.go`.

## Change coupling — "if you change X, also update Y"
- `.msg`/`.srv` in `ros2/src/mowgli_interfaces` → run `gui/generate_go_msgs.sh` **and** `gui/generate_ts_types.sh`, commit both (`msg-codegen-drift.yml` fails otherwise); also `firmware/scripts/sync_ros_lib.py` (see `docs/claude/commands.md`).
- New parameter default in `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` → add the same `default` to `gui/asserts/mower_config.schema.json` (or allowlist it in `schema_template_parity_test.go`), else `TestSchemaDefaultsMatchTemplate` fails and the GUI's "at default" dot lies (Invariant 15).
- New browser topic → `topicMap` (`ros.go`) + `topicSubscribeInterval` (`mowglinext.go`) + `TestTopicSubscribeInterval_CoversKnownSubscriberRouteTopics` + the frontend hook.
- Container names are hardcoded: `mowgli-gps` (`gnss.go:19`), `mowgli-ros2` (`rosbag.go:44`, `drive_tuning.go:24`); renaming them in `install/compose/*.yml` breaks those tools.
- Paths shared with the ROS2 container: `/ros2_ws/maps` (rosbags, calibration files; `mowgli_maps` volume mounted in both), `/ros2_ws/config/mowgli_robot.yaml` (drive tuning passes it to `tune_drive_pid`), `/ros2_ws/config/drive_tuning`.
- `GNSS_*` env keys written by `gnssRuntimeEnvFallbackFromFlat` (`settings.go:749`) are consumed by the sensors/GPS compose service — keep names in sync with `install/compose` and `docs/UNIVERSAL_GNSS_SIDECAR_MIGRATION.md`; `legacyGnssEnvKeys` (`settings.go:850`) are actively purged.
- `api.Schedule` (`schedules.go:13`) and `providers.schedule` (`scheduler.go:16`) are duplicated structs (import-cycle) — change both.
- `MowingSession` (`diagnostics.go:78`) mirrors `providers.MowingSessionRecord` (`session_tracker.go:15`) — same JSON tags on both.
- `metersPerDegreeLat` (`openmower_import.go:147`) must equal `METERS_PER_DEG` in `gui/web/src/utils/map.tsx`.
- `HighLevelControl` command numbers (`Command: 1/2`) in `scheduler.go`, `homekit.go` mirror `HighLevelControl.srv` constants — see `docs/claude/high-level-api.md`.
- Host power actions need `pid: host` + `privileged` in `install/compose/docker-compose.gui.yml` and `util-linux` in `gui/Dockerfile`.
- Swagger annotations (`// @Router`) feed `gui/docs/*` (swaggo) — regenerate when routes change (no script in repo invokes `swag`).

## Pitfalls
- `getSchema` opens `asserts/mower_config.schema.json` **relative to the process CWD** (`settings.go:1043`); run the binary from `gui/` (Dockerfile sets `WORKDIR /app`) or every settings route 500s. Tests call `chdirToGuiRoot`.
- Keys with **no schema default are never pruned** once written (`sparsifyFlat` only sees `defaults`; `settings.go:382-397`) — the reason `retiredParamKeys` and `setGnssStringIfNeeded` exist. `use_scan_matching` / `use_loop_closure` / `use_magnetometer` are not schema properties; the frontend writes them straight through `POST /settings/yaml` and they persist verbatim.
- The schema has **no `x-yaml-node`** entries, so `extractNodeMappings` maps every key to the `mowgli` node; a param under another `ros__parameters` block in the existing file is cloned by `nestToROS2YAML` — and then duplicated under `mowgli` if it is also in `flat`.
- `flattenROS2YAML` last-writer-wins on key collisions across nodes, in Go map order (`settings.go:253-275`).
- `writePreservingPerms` keeps the file's uid/gid/mode; a freshly created yaml is `0664`, so ROS-side line-splice writers (dock pose, calibration, drive rollback — Invariant 6) need the same gid (`settings.go:48-57`).
- `topicSubscribeInterval` lists `dockingSensor` and MQTT subscribes `mowingPath`, but neither key exists in `topicMap` — they silently never deliver (`mowglinext.go:77`, `mqtt.go:101`, `ros.go:28-70`).
- `CallService` fails with "service not advertised" until foxglove has sent `advertiseServices` (`client.go:420-425`); after a reconnect channel/service tables are wiped and re-advertised (`client.go:920-945`). `pollMap` starts 5 s after connect for this reason.
- `Client.Subscribe`'s `msgType` argument is ignored (bridge supplies the schema, `client.go:243-248`); `Advertise` is a no-op.
- A duplicate `ServiceRoute` body-bind returns HTTP 400 JSON, other failures 500 with `{error}`; `set_datum`/`promote_obstacle`/… return `{message}` on success, not `OkResponse` (`mowglinext.go:595-714`) — the frontend's `useMowerAction` depends on this.
- `/mowglinext/publish/:topic` ignores the path param and always publishes `/cmd_vel_teleop` (`mowglinext.go:336`).
- Multiplex frames are **msgpack binary**, dedicated `/subscribe` frames are **base64 text** (`mowglinext.go:375-407,491-527`); WS writes have a 5 s deadline and the whole tab is closed on a stuck write.
- WebSocket `CheckOrigin` requires `Origin` host == request `Host` exactly (`mowglinext.go:33-48`, `containers.go:109-124`) — a reverse proxy that rewrites Host breaks all WS streams; there is no auth layer anywhere.
- `main.go` panics if `system.homekit.enabled` / `system.mqtt.enabled` cannot be read; `NewDBProvider` panics on non-recoverable bitcask errors (`db.go:108-138`).
- `SchedulerProvider` starts with `lastHighLevelState=0` (NULL) → `safeToStart` is false until the first `highLevelStatus` arrives (`scheduler.go:227-243`); `sched.Time` is matched as local `"15:04"` of the container's TZ.
- `SessionTracker` drops sessions < 20 s unless emergency (`session_tracker.go:87`), keeps only the last 500 (`:319-321`), and treats `CHARGING` mid-session as a pause, not an end (`:231-238`).
- Firmware flashing is **safety-critical** (CLAUDE.md § Safety): `flashPrebuilt` sha256-verifies + openocd `verify` + post-flash protocol check (`firmware.go:250-372`); only the custom/expert path can set `DisableEmergency`.
- `POST /system/{reboot,shutdown}` execute `nsenter -t 1 … systemctl` on the **host** (`system.go:56`).
- GNSS apply is destructive to the receiver: `factory-reset-apply` refuses u-blox (`TestGNSSFactoryResetApply_RejectsUbloxWithoutRunningDestructiveWorkflow`) and both gates on explicit confirm flags.
- `main.go` swagger header says `@host localhost:4200`; the real default listen address is `:4006` (`db.go:41`).
- `gui/README.md` is OpenMower-era in two sections: its MQTT topic/command lists (`/gui/mower_logic/current_state`, `/gui/call/mower_service/*`, `dynamic_reconfigure`) do not match `mqtt.go:91-101,118-121`, and it documents `ROS_NODE_HOST` as the listening port — the port comes from `API_ADDR` / `system.api.addr`, and `ROS_NODE_HOST` is read by nothing.
- `gui/openmower-gui` is a tracked 40 MB compiled binary at the repo path `docs/claude/commands.md:32` builds to — do not rely on it and avoid re-committing it.

## Generated & vendored — do not hand-edit
- `gui/pkg/msgs/{geometry,nav,sensor,std,visualization,mowgli}/types_generated.go`, `gui/pkg/msgs/mowgli/services_generated.go` — from `gui/generate_go_msgs.sh` ("Code generated … DO NOT EDIT"); CI-gated by `msg-codegen-drift.yml`.
- `gui/web/src/types/ros.generated.ts` — from `gui/generate_ts_types.sh` (frontend area, same gate).
- `gui/docs/{docs.go,swagger.json,swagger.yaml}` — swaggo/swag output from `// @…` annotations; served at `/swagger/`.
- `gui/asserts/board.h` — rendered sample of `board.h.template`; the template is the source.
- `gui/openmower-gui` — build artifact (binary) checked into git; `gui/go.sum` — Go module checksums.
