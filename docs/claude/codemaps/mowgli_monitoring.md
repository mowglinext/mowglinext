# Codemap: mowgli_monitoring

> Health aggregation for the robot: `diagnostics_node` folds hardware-bridge, emergency, battery, IMU,
> LiDAR, GPS, wheel-odom, fused-pose and motor telemetry into one `/diagnostics` `DiagnosticArray`
> (1 Hz), and the optional `mqtt_bridge_node` mirrors a handful of ROS topics to/from an MQTT broker as
> JSON. Nothing here owns TF, blades, or motion; it is read-only except for the MQTT → `HighLevelControl`
> command path. "BT visualization" and Foxglove publishing are NOT in this package (see Pitfalls).
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `ros2/CLAUDE.md`.

## Where to look
| Task | Start here |
|------|------------|
| Add / change a health check (one `DiagnosticStatus`) | `ros2/src/mowgli_monitoring/src/diagnostics_node.cpp` — the `check_*()` block (:327–667); add the call in `publish_diagnostics()` (:303–321); declare it in `ros2/src/mowgli_monitoring/include/mowgli_monitoring/diagnostics_node.hpp:166–174`; extend `expected_names` in `ros2/src/mowgli_monitoring/test/test_diagnostics.cpp:227–236` |
| Change WARN/ERROR thresholds (freshness, battery %, motor °C) | `ros2/src/mowgli_monitoring/config/diagnostics.yaml` (defaults) ↔ `declare_parameters()` `diagnostics_node.cpp:127–146` (code defaults must match) |
| Subscribe to a new input topic | `create_subscriptions()` `diagnostics_node.cpp:148–218`; add the snapshot fields to `DiagnosticsState` `diagnostics_node.hpp:66–103` |
| Change the publish cadence | `create_timer()` `diagnostics_node.cpp:225–233`; `publish_rate` clamp to [0.1, 100] Hz at :139–145 |
| LiDAR "disabled" vs "no scan" gating | `check_lidar()` `diagnostics_node.cpp:451–479`; `lidar_enabled` is injected from the `use_lidar` launch arg in `ros2/src/mowgli_bringup/launch/full_system.launch.py:531` and `sim_full_system.launch.py:247` |
| Fused-pose ("EKF Map") status: rate window, flat/z checks | `on_fusion_odom()` `diagnostics_node.cpp:273–290` (5 s sliding-window rate); `check_fusion()` :545–618 (`flat_ok` <5° roll/pitch :579, `z_ok` <2 m :581) |
| Battery % formula (4S LiPo 12.0–16.8 V) | `check_battery()` `diagnostics_node.cpp:407–414`; duplicated in `serialise_power()` `mqtt_bridge_node.cpp:656–661` |
| Which GUI code depends on a status `name` | `gui/web/src/utils/gpsStatus.ts:257` (exact `"GPS"`), `gui/web/src/components/settings/LocalizationSection.tsx:67–76` (regex `/lidar|laser ?scan/i` at :71), `gui/web/src/pages/DiagnosticsPage.tsx:219–225` (alerts = `level >= 1`), merge-by-name in `gui/web/src/hooks/useDiagnostics.ts` |
| Add a field to an MQTT JSON payload | `serialise_status/power/emergency/position/diagnostics` `ros2/src/mowgli_monitoring/src/mqtt_bridge_node.cpp:616–737` — fixed `snprintf` buffers (512/256/256/128/512 B), strings must go through `json_escape()` :748 |
| Add an outbound MQTT topic | `create_subscriptions()` `mqtt_bridge_node.cpp:431–482`; `full_topic()` :743; update the topic list comment in `ros2/src/mowgli_monitoring/config/mqtt_bridge.yaml:14–17` |
| Inbound MQTT command → BT | `on_mqtt_command()` `mqtt_bridge_node.cpp:534–572` (payload = decimal uint8 matching `ros2/src/mowgli_interfaces/srv/HighLevelControl.srv` codes); client created at :484–488 |
| Broker host/auth/TLS, reconnect behaviour | `MosquittoMqttClient` ctor `mqtt_bridge_node.cpp:179–234` (clean-session :188, TLS :197–221, user/pw :223–229), `connect()` :247 (keepalive 60 s), `spin_once()` :329–346 (reconnect), `on_timer()` :578–592; defaults in `ros2/src/mowgli_monitoring/config/mqtt_bridge.yaml` |
| Build with / without libmosquitto | `ros2/src/mowgli_monitoring/CMakeLists.txt:35–55` (pkg-config then `find_library`), `MOWGLI_HAS_MOSQUITTO` define :83; `#ifdef` islands in `mqtt_bridge_node.hpp:161–199` and `mqtt_bridge_node.cpp:100–353`, `:404–420` |
| Unit-test a check function | `ros2/src/mowgli_monitoring/test/test_diagnostics.cpp` — fixture + `make_node()` :48–69; parameter-override example :310–313 |
| Which launch file starts what | `full_system.launch.py:520–534` (diagnostics, always) and `:539–549` (mqtt, `IfCondition(enable_mqtt)`); `sim_full_system.launch.py:238–250`; `ros2/src/mowgli_bringup/test/test_nodes_startup.launch.py:79–85` |
| Enable the MQTT bridge on a real deployment | launch arg `enable_mqtt` `full_system.launch.py:117–121`; the compose command passes only `enable_foxglove` (`install/compose/docker-compose.base.yml:42–44`); `ENABLE_MQTT` in `docker/.env` (template `docker/.env.example:10`) only gates the broker container (`docker/stack.sh:93–94`) |
| Lint / format settings for this package | `ros2/src/mowgli_monitoring/CMakeLists.txt:158–162` (copyright, cpplint, uncrustify skipped; clang-format via `ros2/.clang-format`) |

## Files
| File | Lines | Purpose |
|------|-------|---------|
| **`ros2/src/mowgli_monitoring/`** | | |
| `ros2/src/mowgli_monitoring/CMakeLists.txt` | 200 | Static lib `mowgli_monitoring_core` (both nodes) + 2 executables + gtest; optional libmosquitto detection → `MOWGLI_HAS_MOSQUITTO` |
| `ros2/src/mowgli_monitoring/package.xml` | 31 | ament_cmake; deps rclcpp, diagnostic_msgs, sensor_msgs, nav_msgs, geometry_msgs, mowgli_interfaces. **No** `libmosquitto-dev` rosdep key |
| `ros2/src/mowgli_monitoring/config/diagnostics.yaml` | 18 | `/**` defaults for `diagnostics_node` (rate + 6 thresholds; `lidar_enabled` is NOT here — launch injects it) |
| `ros2/src/mowgli_monitoring/config/mqtt_bridge.yaml` | 26 | `/**` defaults for `mqtt_bridge_node` (broker, creds, client id, prefix, rate, ssl) |
| `ros2/src/mowgli_monitoring/include/mowgli_monitoring/diagnostics_node.hpp` | 246 | `DiagnosticsState` snapshot struct, pure classifiers, `DiagnosticsNode` (check_* public for tests) |
| `ros2/src/mowgli_monitoring/include/mowgli_monitoring/mqtt_bridge_node.hpp` | 310 | `IMqttClient` interface, `StubMqttClient`, `MosquittoMqttClient` (ifdef), `MqttBridgeNode` (+ client-injection ctor) |
| `ros2/src/mowgli_monitoring/src/diagnostics_node.cpp` | 689 | Classifiers, subscriptions, 1 Hz aggregation, 9 `check_*()` functions |
| `ros2/src/mowgli_monitoring/src/diagnostics_main.cpp` | 31 | `rclcpp::spin(DiagnosticsNode)` |
| `ros2/src/mowgli_monitoring/src/mqtt_bridge_node.cpp` | 779 | Stub + mosquitto clients, ROS→MQTT JSON serialisers, MQTT→`HighLevelControl` command path |
| `ros2/src/mowgli_monitoring/src/mqtt_bridge_main.cpp` | 31 | `rclcpp::spin(MqttBridgeNode)` |
| `ros2/src/mowgli_monitoring/test/test_diagnostics.cpp` | 352 | gtest: classifier boundaries, status names/hardware_ids/levels, LiDAR gating |
| **Wiring outside the package (read-only context)** | | |
| `ros2/src/mowgli_bringup/launch/full_system.launch.py` | — | :117–121 `enable_mqtt` arg; :176–177 config paths; :520–549 both `Node(...)` blocks |
| `ros2/src/mowgli_bringup/launch/sim_full_system.launch.py` | — | :238–250 diagnostics in sim (`use_sim_time: True`, `lidar_enabled` from `use_lidar`) |
| `ros2/src/mowgli_bringup/test/test_nodes_startup.launch.py` | — | :79–85 launches `diagnostics_node` bare; :178 requires `/diagnostics` advertised within 10 s |
| `ros2/src/mowgli_bringup/launch/mowgli.launch.py` | — | :257–264 remaps that produce `/hardware_bridge/{status,emergency,power}` and `/imu/data` |
| `gui/pkg/providers/ros.go` | — | :54 GUI subscribes `/diagnostics` over foxglove_bridge (`"diagnostics"` channel) |

## Runtime surface

### Nodes
| Node name | Executable | Launched by | Kind |
|-----------|------------|-------------|------|
| `diagnostics_node` | `mowgli_monitoring/diagnostics_node` | `full_system.launch.py:520` (always), `sim_full_system.launch.py:238`, `test_nodes_startup.launch.py:79` | plain `rclcpp::Node`, wall timer |
| `mqtt_bridge_node` | `mowgli_monitoring/mqtt_bridge_node` | `full_system.launch.py:539` only when `enable_mqtt:=true` (default `false`) | plain `rclcpp::Node`, wall timer drives the MQTT loop |

### Topics
| Topic | Type | Dir | QoS (this side) | Other end |
|-------|------|-----|-----------------|-----------|
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | pub (diagnostics_node) `diagnostics_node.cpp:222` | reliable, depth 10 | `mqtt_bridge_node` (:468), GUI via foxglove (`gui/pkg/providers/ros.go:54`), `test_nodes_startup.launch.py:178` |
| `/hardware_bridge/status` | `mowgli_interfaces/msg/Status` | sub (both nodes) `diagnostics_node.cpp:153`, `mqtt_bridge_node.cpp:435` | reliable, depth 10 | `hardware_bridge` `~/status` remapped in `mowgli.launch.py:264` |
| `/hardware_bridge/emergency` | `mowgli_interfaces/msg/Emergency` | sub (both) :161 / :451 | reliable, depth 10 | `mowgli.launch.py:262` |
| `/hardware_bridge/power` | `mowgli_interfaces/msg/Power` | sub (both) :169 / :443 | reliable, depth 10 | `mowgli.launch.py:263` |
| `/imu/data` | `sensor_msgs/msg/Imu` | sub (diag) :177 | `SensorDataQoS` | `hardware_bridge` `~/imu/data_raw` remapped `mowgli.launch.py:258` |
| `/scan` | `sensor_msgs/msg/LaserScan` | sub (diag) :185 | `SensorDataQoS` | LiDAR driver container (`sensors/lidar-*`) |
| `/wheel_odom` | `nav_msgs/msg/Odometry` | sub (both) :193 / :459 | `SensorDataQoS` | `hardware_bridge` `~/wheel_odom` (`ros2/src/mowgli_hardware/src/odometry_publisher.cpp:35`) remapped `mowgli.launch.py:260` — `mowgli_localization/wheel_odometry_node` is deliberately NOT launched (`full_system.launch.py:438–446`) |
| `/odometry/filtered_map` | `nav_msgs/msg/Odometry` | sub (diag) :202 | `SensorDataQoS` | `fusion_graph_node` (`ros2/src/fusion_graph/src/fusion_graph_node_setup_comms.cpp:35`) |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | sub (diag) :211 | `SensorDataQoS` | GNSS sidecar (not in `ros2/src`); sim relay `sim_full_system.launch.py:321–322` |
| MQTT `<prefix>/status`, `/power`, `/emergency` | JSON (retain=true) | out `mqtt_bridge_node.cpp:504–517` | QoS 1 | any broker client |
| MQTT `<prefix>/position` | JSON `{x,y,theta}` from `/wheel_odom` (odom frame), rate-limited to `publish_rate` | out :594–609 | QoS 1, retain=false | — |
| MQTT `<prefix>/diagnostics` | JSON `[{name,level,message},…]` | out :525–528 | QoS 1, retain=false | — |
| MQTT `<prefix>/command` | decimal uint8 payload | in :477–481 → `on_mqtt_command` :534 | QoS 1 | any broker client |

Published `DiagnosticStatus.name` / `hardware_id` pairs (order = array order, `diagnostics_node.cpp:310–318`):
`Hardware Bridge`/`mowgli/hardware_bridge` (:331), `Emergency System`/`mowgli/emergency` (:357), `Battery`/`mowgli/battery` (:395),
`IMU`/`mowgli/imu` (:433), `LiDAR`/`mowgli/lidar` (:454), `GPS`/`mowgli/gps` (:484), `Odometry`/`mowgli/odometry` (:527),
`EKF Map`/`robot_localization/filtered_map` (:548–549), `Motors`/`mowgli/motors` (:623).

### Services & actions
| Name | Type | Role | Where |
|------|------|------|-------|
| `/behavior_tree_node/high_level_control` | `mowgli_interfaces/srv/HighLevelControl` | **client** (mqtt_bridge_node); fire-and-forget `async_send_request`, dropped if `!service_is_ready()` | `mqtt_bridge_node.cpp:484–488`, `:547–571`; server in `ros2/src/mowgli_behavior/src/behavior_tree_node.cpp:548` |

No services or actions are served by this package. No actions used.

### Parameters (all read once in the constructor; none dynamic)
| Node | Param | Default (code / yaml) | Code | YAML |
|------|-------|-----------------------|------|------|
| diagnostics | `publish_rate` | 1.0 Hz (clamped to [0.1,100] → 1.0) | `diagnostics_node.cpp:129`, :139 | `ros2/src/mowgli_monitoring/config/diagnostics.yaml:4` |
| diagnostics | `freshness_warn_sec` / `freshness_error_sec` | 5.0 / 10.0 s (shared by HW bridge, IMU, LiDAR, odom, fusion, GPS-with-fix) | :130–131 | :9–10 |
| diagnostics | `battery_warn_pct` / `battery_error_pct` | 20 / 10 % (`<=` comparisons) | :132–133 | :13–14 |
| diagnostics | `motor_temp_warn_c` / `motor_temp_error_c` | 60 / 80 °C (`>=`, worst of ESC + motor) | :134–135 | :17–18 |
| diagnostics | `lidar_enabled` | `false` — **not in yaml**; launch sets it from `use_lidar` | :136 | `full_system.launch.py:531` |
| mqtt | `mqtt_host` / `mqtt_port` | `"localhost"` / 1883 | `mqtt_bridge_node.cpp:386–387` | `ros2/src/mowgli_monitoring/config/mqtt_bridge.yaml:4–5` |
| mqtt | `mqtt_username` / `mqtt_password` | `""` / `""` (empty user → no auth) | :388–389 | :8–9 |
| mqtt | `mqtt_client_id` | `"mowgli_ros2"` | :390 | :12 |
| mqtt | `mqtt_topic_prefix` | `"mowgli"` | :391 | :18 |
| mqtt | `publish_rate` | 1.0 Hz (clamped to [0.01,100]); also the MQTT network-loop period | :392, :395 | :22 |
| mqtt | `use_ssl` | `false` (TLS against `/etc/ssl/certs`; setup failure → refuses to connect) | :393, :197–216 | :26 |

Both YAMLs use the `/**:` wildcard, so they apply to any node name they are passed to.

### TF frames
None. Neither node uses tf2 (no tf2 dependency in `ros2/src/mowgli_monitoring/CMakeLists.txt`/`ros2/src/mowgli_monitoring/package.xml`).

## Build, test, run
```bash
# inside the ros2 devcontainer / image (workspace at /ros2_ws)
cd ros2 && make build-pkg PKG=mowgli_monitoring          # scripts/build.sh → colcon build --packages-up-to
cd ros2 && PACKAGES=mowgli_monitoring make test           # scripts/test.sh → colcon test + test-result
# raw colcon equivalent
colcon build --packages-up-to mowgli_monitoring && colcon test --packages-select mowgli_monitoring && colcon test-result --verbose
# run the gtest binary directly
./build/mowgli_monitoring/test_diagnostics
# run nodes standalone
ros2 run mowgli_monitoring diagnostics_node --ros-args --params-file src/mowgli_monitoring/config/diagnostics.yaml -p lidar_enabled:=true
ros2 run mowgli_monitoring mqtt_bridge_node --ros-args --params-file src/mowgli_monitoring/config/mqtt_bridge.yaml
ros2 topic echo /diagnostics
# formatting (CI checks changed lines with clang-format 18)
cd ros2 && make format
```

Tests:
| Test | What it pins |
|------|--------------|
| `ros2/src/mowgli_monitoring/test/test_diagnostics.cpp` (`ament_add_gtest(test_diagnostics)`, `ros2/src/mowgli_monitoring/CMakeLists.txt:167–182`) | `classify_freshness` OK/WARN/ERROR incl. exact boundaries and `never=true`; `classify_battery` `<=` boundaries; `classify_temperature`; `level_name` incl. `UNKNOWN`; the 8 status names (`:227–236` — **omits `EKF Map`**) and non-empty `hardware_id`; levels ≤ STALE; `Hardware Bridge` = ERROR with no status; `lidar_enabled=false` → OK "LiDAR disabled", `true` → ERROR "No LiDAR scan received" |
| `ros2/src/mowgli_bringup/test/test_nodes_startup.launch.py` (`add_launch_test`, `ros2/src/mowgli_bringup/CMakeLists.txt:46`) | `diagnostics_node` survives a 5 s soak, advertises `/diagnostics` within 10 s, exits 0 |

No test covers `MqttBridgeNode` (serialisers, `json_escape`, command parsing, rate limiting) even though the client-injection ctor (`mqtt_bridge_node.hpp:223`) exists for it.

CI: `.github/workflows/ros2-ci.yml` — "Build workspace" (:335–341, whole-workspace `colcon build`) and "Run tests" (:343–350, `colcon test --return-code-on-test-failure`); "Formatting (clang-format)" job (:405–445) diffs changed lines with clang-format 18. `ament_lint_auto` runs the remaining `ament_lint_common` linters (cppcheck, lint_cmake, xmllint …) because only copyright/cpplint/uncrustify are marked found (`ros2/src/mowgli_monitoring/CMakeLists.txt:158–162`).

## Change coupling — "if you change X, also update Y"
- **Status `name` strings** are the GUI contract: `"GPS"` is matched exactly in `gui/web/src/utils/gpsStatus.ts:257` (feeds `useGnssStatus.ts`); `"LiDAR"` is matched by regex in `gui/web/src/components/settings/LocalizationSection.tsx:71`; the GUI merges entries by `name` into an accumulator that nothing prunes (`gui/web/src/hooks/useDiagnostics.ts:44–49`), so a rename shows up as a new row while the old one lingers at its last value. Also update `expected_names` in `test_diagnostics.cpp:227–236`.
- **Status `message` text** is user-visible in the GUI alert list (`DiagnosticsPage.tsx:219–225`, any `level >= 1`) and is echoed as the LiDAR "latest diagnostic" line (`LocalizationSection.tsx:140`; the badge label itself is level-derived, `:78–87`).
- **Thresholds**: a new default must be changed in BOTH `ros2/src/mowgli_monitoring/config/diagnostics.yaml` and `declare_parameters()` (`diagnostics_node.cpp:129–136`) — `test_nodes_startup.launch.py:79–85` launches the node with no params file, so code defaults are what CI sees.
- **`lidar_enabled`** is not a yaml key; it is derived from `use_lidar` in `full_system.launch.py:531` / `sim_full_system.launch.py:247`, which itself comes from `mowgli_robot.yaml.lidar_enabled` (`full_system.launch.py:92–93`, Invariant 15). A new launch file that starts `diagnostics_node` must wire it the same way or LiDAR reports ERROR on GPS-only robots.
- **Input topic names** are remap outputs of `mowgli.launch.py:257–264` (`/hardware_bridge/*`, `/imu/data`); changing a remap there silently starves the corresponding check (ERROR/WARN "No … received").
- **`mowgli_interfaces` fields** used here: `Status.{mower_status,is_charging,mow_enabled,mower_esc_status,mower_esc_temperature,mower_esc_current,mower_motor_temperature,mower_motor_rpm,raspberry_pi_power,esc_power,rain_detected,sound_module_available,sound_module_busy,ui_board_available}`, `Emergency.{active_emergency,latched_emergency,reason}`, `Power.{v_charge,v_battery,charge_current,charger_enabled,charger_status}`. Removing/renaming any of these breaks `diagnostics_node.cpp:347–349,385–387,419–425,635–664` and `serialise_*` in `mqtt_bridge_node.cpp:616–697`; `.msg` edits also require the GUI codegen step (`docs/claude/commands.md`).
- **`HighLevelControl` command codes** (`ros2/src/mowgli_interfaces/srv/HighLevelControl.srv`) are what MQTT `<prefix>/command` payloads mean; the comment at `mqtt_bridge_node.cpp:536–537` lists examples and must track the `.srv`.
- **Battery % formula** (16.8/12.0 V) is duplicated in `diagnostics_node.cpp:409–410` and `mqtt_bridge_node.cpp:657–658`; change both.
- **MQTT topic list** in `ros2/src/mowgli_monitoring/config/mqtt_bridge.yaml:14–17` and `mqtt_bridge_node.hpp:29–37` is documentation only — the truth is `full_topic()` call sites in `mqtt_bridge_node.cpp:477,506,511,516,527,603`.
- **`motor_temp_warn_c` / `motor_temp_error_c`** are referenced as the ONLY temperature surface by `install/scripts/migrate_openmower.py:117–122` and the template comment `ros2/src/mowgli_bringup/config/mowgli_robot.yaml:515–521`; do not move them into `mowgli_robot.yaml` without updating both.
- **Adding a dependency**: `ros2/src/mowgli_monitoring/CMakeLists.txt` (find_package + the four `ament_target_dependencies` lists :73–80, :101–108, :122–129, :175–182) AND `ros2/src/mowgli_monitoring/package.xml`.

## Pitfalls
- `mqtt_bridge_node` is a **stub in every shipped image**: `ros2/Dockerfile` never installs `libmosquitto-dev` (0 hits), `ros2/src/mowgli_monitoring/package.xml` has no rosdep key for it, and CI does not apt-install it, so `MOWGLI_HAS_MOSQUITTO` is unset and `StubMqttClient` only logs at DEBUG (`mqtt_bridge_node.cpp:415–419`). The compose stack also never passes `enable_mqtt:=true` (`install/compose/docker-compose.base.yml:42–44`); `ENABLE_MQTT=true` in `docker/.env` (template `docker/.env.example:10`) starts only the `eclipse-mosquitto` broker (`install/compose/docker-compose.mqtt.yml`, `docker/stack.sh:93–94`). The GUI has its own embedded MQTT server (`gui/pkg/providers/mqtt.go`, settings `system.mqtt.*` in `gui/web/src/hooks/useSettings.ts:86–105`) — that is a different code path.
- **`EKF Map` / `robot_localization/filtered_map`** (`diagnostics_node.cpp:548–549`) and the header comments `diagnostics_node.hpp:30, :90` predate Invariant 1; the data actually comes from `fusion_graph_node`. Do not read this name as evidence an EKF exists.
- `diagnostics_node.hpp:26` says the IMU input is `/imu/data_raw`; the code subscribes `/imu/data` (`diagnostics_node.cpp:178`).
- **STALE is never emitted.** `classify_freshness` returns OK/WARN/ERROR only (`:47–58`); "never received" is ERROR for HW bridge/IMU/LiDAR/odom/fusion and WARN for emergency/battery/GPS/motors. `level_name()` maps STALE for logging only. The GUI exports a 30 s staleness helper (`useDiagnostics.ts:22–28` `DIAGNOSTIC_STALE_MS` / `isDiagnosticStale`) but no component calls it yet, so a stopped entry keeps showing its last value.
- `check_gps` treats `status.status >= 0` as "has fix" (`:499`) — `STATUS_FIX` (0) counts, RTK Fixed/Float are not distinguished; RTK quality for the GUI comes from `universal_gnss/summary` entries, not from this status (`gpsStatus.ts:256–257`).
- `check_fusion` does NOT read covariance or compare against GPS; it only checks freshness, roll/pitch < 5°, |z| < 2 m, and reports `rate_hz` without thresholding it (`:545–618`).
- Freshness ages use `now()` at reception, not the message stamp (`:242, :257, :263, :269, :275, :295`); under `use_sim_time` the rate window in `on_fusion_odom` (`:281–289`) only advances with `/clock`.
- `publish_rate` is also the **MQTT network-loop period** (`on_timer()` `mqtt_bridge_node.cpp:578–581`): at the 1 Hz default an inbound command can wait up to 1 s, and `spin_once` uses `mosquitto_loop(…, 0, 1)` (non-blocking, :336).
- `<prefix>/position` is built from `/wheel_odom` (odom frame, `mqtt_bridge_node.cpp:459–466, :699–711`), not the map-frame fused pose; `theta = 2·atan2(qz,qw)` assumes a planar quaternion (:706).
- MQTT subscriptions are re-issued in `on_connect_cb` (`:122–139`) because the client is clean-session; a `subscribe()` call before the async connect completes would otherwise be lost. Keep that loop if you touch `Impl`.
- `on_mqtt_command` accepts any integer 0–255 and forwards it (`:539`); unknown codes are rejected by the BT server, not here. The service call goes straight to `HighLevelControl` — the same channel as the GUI buttons — so an open broker = remote control of the mower (blade safety remains firmware-side, root `CLAUDE.md` Safety section).
- `snprintf` payload buffers are fixed (`:618, :663, :685, :708, :726`); a long `Emergency.reason` or status message is truncated, not escaped into invalid JSON, but adding fields to `serialise_status` can overflow the 512 B budget silently.
- `test_diagnostics.cpp:207–244` enumerates only 8 categories; `check_fusion` ("EKF Map") is untested and can be renamed without a test failing.
- The two `/**:` YAMLs are loaded from the **package share** path (`full_system.launch.py:176–177`); there is no `/ros2_ws/config/diagnostics.yaml` or `mqtt_bridge.yaml` override lookup (the only runtime-config read in the launch files is `mowgli_robot.yaml`).
- "BT visualization" lives elsewhere: `/behavior_tree_log` is Nav2's `nav2_msgs/msg/BehaviorTreeLog` consumed by the GUI (`gui/pkg/providers/ros.go:43`, `gui/web/src/hooks/useBTLog.ts`); no node in `ros2/src` publishes it and `mowgli_interfaces` has no such msg. Foxglove publishing is `foxglove_bridge` in `full_system.launch.py:557–583` / `ros2/src/mowgli_bringup/config/foxglove_bridge.yaml`; the ad-hoc `ros2/src/precision_monitor.py` script publishes `/precision/*` Float64s for Foxglove plots. Localizer-specific health is `/fusion_graph/diagnostics` from `fusion_graph_node` (root `CLAUDE.md` Invariant 1), not this package.

## Generated & vendored — do not hand-edit
- Nothing generated or vendored inside `ros2/src/mowgli_monitoring/`. `libmosquitto` (when present) is a system library found by CMake, not vendored. Message/service headers come from `mowgli_interfaces` codegen at build time.
