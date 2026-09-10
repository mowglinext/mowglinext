# Codemap: mowgli_simulation

> Webots R2025a simulation of the mower: the garden world + `MowgliMower` PROTO, the `webots_ros2_driver`
> URDF that wires Webots devices to ROS topics, the in-process `KinematicDrive` plugin that teleports the
> chassis through a mirror of the firmware wheel loop, and the sim-only helper nodes (fake hardware bridge,
> firmware actuation model, IMU noise, RTK-quality relay, wheel-slip injector). The full stack is composed by
> `ros2/src/mowgli_bringup/launch/sim_full_system.launch.py` (outside this package, indexed here). ODE
> quirks and their history live in `docs/WEBOTS_SIM.md` — this map points there instead of repeating it.
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `ros2/CLAUDE.md`.

## Where to look
| Task | Start here |
|------|------------|
| Robot stalls / creeps / won't pivot on a small `cmd_vel` | `ros2/src/mowgli_simulation/mowgli_simulation/kinematic_drive.py` `__simulate_firmware_motor_model` (:292–409) and `ros2/src/mowgli_simulation/src/sim_actuation_node.cpp` `tick()` (:110–155); the deadband is intentional |
| Change a firmware gain the sim mirrors (PWM/m/s, deadband, PI) | THREE copies: `include/mowgli_simulation/firmware_wheel_model.hpp` `FirmwareWheelModelParams` (:60–72), `kinematic_drive.py` `FIRMWARE_*` (:147–155), `ros2/src/mowgli_bringup/launch/sim_full_system.launch.py` `sim_actuation` params (:491–502) |
| Add / move a sensor | `worlds_webots/mowgli_garden.wbt` `extensionSlot` (:82–125) + `urdf_webots/mowgli_webots.urdf` frame joints (:126–158) + `<webots>` device block (:161–204) |
| Change chassis geometry, mass, wheels | `protos/MowgliMower.proto` — chassis (:119–131), `LEFT_JOINT` (:143–199), `RIGHT_JOINT` (:202–238), casters (:246–275), mass/COM (:294–298) |
| Add a world or a static obstacle | `worlds_webots/mowgli_garden.wbt` (`test_obstacle` Solid :46–60); `world` launch arg in `launch/webots_minimal.launch.py:50` and `sim_full_system.launch.py:73` |
| Change the sim mow polygon | `sim_full_system.launch.py:219–224` (`area_names` / `area_polygons` override on `map_server_node`) — see CLAUDE.md "What NOT to Do" on `map_server.yaml` |
| Change the sim GPS datum | `mowgli_garden.wbt:19` (`gpsReference`) **and** `sim_full_system.launch.py:229–230` (map_server) **and** `:356–357` (navsat converter) |
| Simulate RTK-Float / no-fix windows | `scripts/sim_navsat_rtk_fix.py` `quality_pattern` (:116–118, grammar :76–101); launch value `sim_full_system.launch.py:330` (currently `""` = always FIXED) |
| IMU noise, or a perfect IMU | `scripts/sim_imu_noise.py` params (:67–123); launch values `sim_full_system.launch.py:445–461` |
| Wheel-slip injection / `/wheel_odom` covariance | `scripts/sim_wheel_slip.py` (:59–67 params, :131–138 covariance override); launch `sim_full_system.launch.py:406–412` |
| Fake charging / dock detection / firmware handshake | `src/fake_hardware_bridge_node.cpp` — dock params (:83–90), `near_dock` (:151–166), `Status` field parity (:168–200) |
| Webots "Gives up" at boot, spawners never start | `launch/webots_minimal.launch.py:114–125` (`respawn=True`), `:132` (90 s spawner timeout), `ros2/scripts/sim-stop.sh` (stale IPC socket) |
| Robot stutters under `mode:=fast` | `kinematic_drive.py:160–170` (`CMD_VEL_TIMEOUT_S` is wall-clock) and `:419–428` |
| Need the true chassis pose (not the localizer belief) | `/sim/ground_truth_pose` — `kinematic_drive.py:252–254`, `:462–470` |
| Wheel-odom chain (`/wheel_odom_raw` → `/wheel_odom`) | `webots_minimal.launch.py:111–112` (remaps) → `scripts/sim_wheel_slip.py` |
| diff_drive limits / controller_manager rate | `config_webots/ros2_control.yaml` |
| Sim-only fusion_graph overrides (TF lead, node period) | `sim_full_system.launch.py:182–183` |
| Run the sim / E2E locally | `ros2/Makefile:78–129` (`sim`, `sim-stop`, `e2e-test`, `e2e-test-no-lidar`) |
| Run the sim in Docker / with a GUI | `docker/docker-compose.simulation.yaml` (`simulation`, `dev-sim`, `simulation-gui`), `ros2/scripts/start_vnc.sh`, `ros2/Dockerfile:487–559` (stage `simulation`) |
| Unit-test the wheel model | `test/test_firmware_wheel_model.cpp` |
| Anything that "looks like a Nav2 bug" in sim | `docs/WEBOTS_SIM.md` §3–§8 (five load-bearing ODE workarounds + symptom table) |

## Files
| File | Lines | Purpose |
|------|-------|---------|
| **`ros2/src/mowgli_simulation/`** | | |
| `CMakeLists.txt` | 130 | Builds `fake_hardware_bridge_node`, `sim_actuation_node`; installs the 3 scripts, the `mowgli_simulation` Python package (:92), data dirs (:97–106); registers the gtest (:121) |
| `package.xml` | 58 | Deps: `webots_ros2_driver`, `controller_manager`, `diff_drive_controller`, `joint_state_broadcaster`, `mowgli_interfaces`, `mowgli_bringup` |
| `launch/webots_minimal.launch.py` | 165 | Webots + `WebotsController` + RSP + ros2_control spawners; the only launch file in the package |
| `mowgli_simulation/__init__.py` | 6 | Python package loaded in-process by `webots_ros2_driver` |
| `mowgli_simulation/kinematic_drive.py` | 565 | `KinematicDrive` plugin: `/cmd_vel` → firmware wheel model → teleport + `setVelocity`; publishes ground truth |
| `src/fake_hardware_bridge_node.cpp` | 280 | Stand-in for `hardware_bridge_node`: status/power/emergency/battery/dock-heading publishers + 2 services |
| `src/sim_actuation_node.cpp` | 183 | `/cmd_vel` → per-wheel firmware model → `/cmd_vel_wheels` (the wheels' input) |
| `include/mowgli_simulation/firmware_wheel_model.hpp` | 170 | Header-only, ROS-free per-wheel PI + stiction model (`step_firmware_wheel_model`) |
| `scripts/sim_imu_noise.py` | 269 | `/imu/data_sim` → `/imu/data` with white noise + bias walk (optional cmd_vel-synthesised IMU) |
| `scripts/sim_navsat_rtk_fix.py` | 274 | `/gps/fix_raw` → `/gps/fix`; rewrites `status`/covariance per `RTK_FIXED`/`RTK_FLOAT`/`NO_FIX` regime |
| `scripts/sim_wheel_slip.py` | 162 | `/wheel_odom_raw` → `/wheel_odom`; periodic vx over-report + production twist covariance |
| `config_webots/ros2_control.yaml` | 60 | `controller_manager` 50 Hz, `diffdrive_controller` (0.325 m / 0.093 m, `use_stamped_vel`, `enable_odom_tf: false`) |
| `urdf_webots/mowgli_webots.urdf` | 222 | Device→topic wiring + ros2_control joints + static sensor frames; no physics |
| `worlds_webots/mowgli_garden.wbt` | 126 | 20×20 m floor, one obstacle, `MowgliMower` instance with sensors in `extensionSlot` |
| `protos/MowgliMower.proto` | 300 | Physical robot body: chassis, 2 hinge wheels (`axis 0 -1 0`), 2 ball-joint casters |
| `rviz/mowgli_sim.rviz` | 302 | RViz layout; installed (:104) but **no launch file opens it** |
| `test/test_firmware_wheel_model.cpp` | 190 | gtest for the wheel model (8 cases) |
| **Related, outside the package** | | |
| `docs/WEBOTS_SIM.md` | 360 | The ODE-quirk reference (rules, anchors, symptoms, smoke test, known drift) |
| `ros2/src/mowgli_bringup/launch/sim_full_system.launch.py` | 535 | Composes Webots + `navigation.launch.py` + BT + map_server + all sim helpers |
| `ros2/src/e2e_test.py` | 1648 | E2E harness run by `make e2e-test` and mounted by compose `dev-sim` |
| `ros2/src/e2e_test_no_lidar.py` | 505 | GPS-only E2E harness (`make e2e-test-no-lidar`) |
| `ros2/scripts/e2e_test.py` | 690 | Older, shorter E2E copy; referenced by nothing (see Pitfalls) |
| `ros2/Makefile` | 179 | `sim`, `sim-stop`, `e2e-test`, `e2e-test-no-lidar`, `docker-sim` targets (:78–141) |
| `ros2/scripts/sim-stop.sh` | 66 | Kills ros2 launch + Webots + straggler nodes, wipes Cyclone shm and `/tmp/webots/*` |
| `ros2/scripts/start_vnc.sh` | 64 | TigerVNC + noVNC + `sim_full_system.launch.py` (compose `simulation-gui` entrypoint) |
| `ros2/scripts/start_dev_sim.sh` | 79 | Same as above with a first-run `build.sh`; `LAUNCH_FILE`/`LAUNCH_ARGS` env overrides |
| `docker/docker-compose.simulation.yaml` | 125 | Services `simulation` (Xvfb :99), `dev-sim` (bind mounts), `simulation-gui` (6080/8765) |
| `ros2/Dockerfile` (stage `simulation`, :478–559) | 559 | Adds Webots R2025a (amd64 only), `ros-kilted-webots-ros2`, Xvfb/VNC; patches `webots_ros2_driver.utils.is_wsl` |
| `ros2/foxglove/mowgli_sim.json` | 505 | Foxglove layout copied into the sim image (`Dockerfile:552`) |

## Runtime surface

### Nodes
| Node | Executable / plugin | Launched by | Kind |
|------|---------------------|-------------|------|
| `MowgliMower_kinematic_drive` (name derived at `kinematic_drive.py:234`) | `mowgli_simulation.kinematic_drive.KinematicDrive`, in-process in `webots_controller_MowgliMower` (`mowgli_webots.urdf:201`) | `webots_minimal.launch.py:88` `WebotsController(robot_name='MowgliMower')` | rclpy node inside the driver |
| Webots driver + `Ros2IMU` + `webots_ros2_control::Ros2Control` | `webots_ros2_driver` (`mowgli_webots.urdf:182–193`) | `webots_minimal.launch.py:88–126` (`respawn=True`) | plain |
| `controller_manager` / `diffdrive_controller` / `joint_state_broadcaster` | ros2_control inside the driver; spawners `webots_minimal.launch.py:134–145` | `WaitForControllerConnection` (:149) | ros2_control lifecycle |
| `robot_state_publisher` | `robot_state_publisher` with URDF text (`webots_minimal.launch.py:79–86`); driver-internal RSP disabled (:99) | `webots_minimal.launch.py` | plain |
| `fake_hardware_bridge` | `fake_hardware_bridge_node` (`fake_hardware_bridge_node.cpp:50`) | `sim_full_system.launch.py:297` | plain, 10 Hz wall timer (:137) |
| `sim_actuation` | `sim_actuation_node` (`sim_actuation_node.cpp:59`) | `sim_full_system.launch.py:483` | plain, `control_hz` 50 Hz wall timer (:93) |
| `sim_imu_noise` | `sim_imu_noise.py` (:65) | `sim_full_system.launch.py:424` | plain |
| `sim_navsat_rtk_fix` | `sim_navsat_rtk_fix.py` (:107) | `sim_full_system.launch.py:313` | plain |
| `sim_wheel_slip` | `sim_wheel_slip.py` (:51) | `sim_full_system.launch.py:396` | plain |
| Production nodes also started by `sim_full_system.launch.py` | `navigation.launch.py` include (:159, `use_sim_time: true`, `use_lidar`), `behavior_tree_node` (:190), `map_server_node` (:204), `diagnostics_node` (:238), `foxglove_bridge` port 8765 (:256), `obstacle_tracker` if `use_lidar` (:282), `navsat_to_absolute_pose` (:348), `twist_mux` with `cmd_vel_out`→`/cmd_vel` (:372–381) | | |

Every node gets `use_sim_time: True` (`sim_full_system.launch.py`); `webots_minimal.launch.py:44` defaults `use_sim_time` to `true`.

### Topics
| Topic | Type | Dir (from this area) | QoS | Other end |
|-------|------|----------------------|-----|-----------|
| `/cmd_vel` | `geometry_msgs/TwistStamped` | sub — `kinematic_drive.py:241` (depth 1), `sim_actuation_node.cpp:82` (SystemDefaults) | | pub: `twist_mux` (`sim_full_system.launch.py:381`) |
| `/cmd_vel_wheels` | `geometry_msgs/TwistStamped` | pub — `sim_actuation_node.cpp:80` | SystemDefaults | sub: `diffdrive_controller` via remap `webots_minimal.launch.py:111` |
| `/wheel_odom_raw` | `nav_msgs/Odometry` | pub — `diffdrive_controller` remap `webots_minimal.launch.py:112`; sub — `sim_wheel_slip.py:94` | reliable | internal to the sim chain |
| `/wheel_odom` | `nav_msgs/Odometry` | pub — `sim_wheel_slip.py:93` (covariance :131–138: vx 0.01, vy 1e-4, wz 9e-4) | reliable | `fusion_graph_node`, `controller_server.odom_topic` (CLAUDE.md "What NOT to Do") |
| `/imu/data_sim` | `sensor_msgs/Imu` | pub — `Ros2IMU` plugin 100 Hz (`mowgli_webots.urdf:182–191`); sub — `sim_imu_noise.py:143` | best-effort | raw Webots gyro/accel/inertial unit |
| `/imu/data` | `sensor_msgs/Imu` | pub — `sim_imu_noise.py:140` | best-effort | `fusion_graph_node`, `cog_to_imu` |
| `/gps/fix_raw` | `sensor_msgs/NavSatFix` | pub — Webots `GPS` device 5 Hz (`mowgli_webots.urdf:172–180`); sub — `sim_navsat_rtk_fix.py:158` | best-effort | raw, `status` always `STATUS_FIX` |
| `/gps/fix` | `sensor_msgs/NavSatFix` | pub — `sim_navsat_rtk_fix.py:155` | **reliable** (:148–153, required by the consumer) | `navsat_to_absolute_pose_node` (gates on `STATUS_GBAS_FIX`, `navsat_to_absolute_pose_node.cpp:165`) |
| `/scan` | `sensor_msgs/LaserScan` | pub — Webots `Lidar` 10 Hz (`mowgli_webots.urdf:162–170`; noise/resolution `mowgli_garden.wbt:122–123`) | driver default | `costmap_scan_filter_node` (→ `/scan_costmap` costmaps + `/scan_collision` collision_monitor), `scan_deskew_node` (→ `/scan_deskewed`, `fusion_graph` scan-matching) |
| `/joint_states` | `sensor_msgs/JointState` | pub — `joint_state_broadcaster` | | `robot_state_publisher` (wheel TFs) |
| `/robot_description` | `std_msgs/String` | pub — RSP only (`webots_minimal.launch.py:71–86`, driver copy disabled :99) | transient_local | `controller_manager` |
| `/sim/ground_truth_pose` | `geometry_msgs/PoseStamped` (`frame_id: map`) | pub — `kinematic_drive.py:252`, every physics tick; sub — `fake_hardware_bridge_node.cpp:120` (SensorDataQoS) | depth 10 | sim-only truth; nothing production reads it |
| `/hardware_bridge/status` | `mowgli_interfaces/Status` | pub — `fake_hardware_bridge_node.cpp:93` | QoS(10) | BT, map_server, diagnostics, `calibrate_imu_yaw_node`, `costmap_scan_filter_node`, `fusion_graph` (same names the real bridge is remapped to, `mowgli.launch.py:260–266`) |
| `/hardware_bridge/power` | `mowgli_interfaces/Power` | pub — :95 (28 V; 1.5 A + `charger_enabled` when `near_dock`) | QoS(10) | BT battery filter, diagnostics |
| `/hardware_bridge/emergency` | `mowgli_interfaces/Emergency` | pub — :97 (`active_emergency`/`latched_emergency` = service state) | QoS(10) | BT, LED ring, diagnostics |
| `/battery_state` | `sensor_msgs/BatteryState` | pub — :100 (`CHARGING` iff `near_dock`) | QoS(10) | `docking_server` `SimpleChargingDock` (`nav2_params_base.yaml:1087–1088`) |
| `/gnss/heading` | `sensor_msgs/Imu` (orientation = `dock_pose_yaw`) | pub — :106, only while `near_dock` (:207–219) | QoS(10) | same topic the real bridge's `~/dock_heading` is remapped to (`mowgli.launch.py:266`) |

### Services & actions
| Name | Type | Server | Notes |
|------|------|--------|-------|
| `/hardware_bridge/mower_control` | `mowgli_interfaces/srv/MowerControl` | `fake_hardware_bridge_node.cpp:53` | always `success=true`; latches `mow_enabled_` into `Status` |
| `/hardware_bridge/emergency_stop` | `mowgli_interfaces/srv/EmergencyStop` | `fake_hardware_bridge_node.cpp:64` | `emergency != 0` sets both emergency flags; `0` clears (no physical-trigger check, unlike firmware) |
| `/behavior_tree_node/high_level_control` | `mowgli_interfaces/srv/HighLevelControl` | production BT | what the E2E harness calls (`ros2/src/e2e_test.py:213`) |

No actions are defined in this package.

### Parameters
| Param | Default (file:line) | Launch override (`sim_full_system.launch.py`) | Read |
|-------|---------------------|-----------------------------------------------|------|
| `sim_actuation.deadband_enabled` | `true` (`sim_actuation_node.cpp:65`) | `:491` `True` | startup |
| `sim_actuation.wheel_separation` / `firmware_max_mps` / `firmware_pwm_per_mps` / `firmware_pwm_max` | 0.325 / 0.5 / 300 / 255 (`:66–69`) | `:492–495` | startup |
| `sim_actuation.firmware_deadband_pwm_static` / `_kinetic` | 40 / 30 (`:70–71`) | `:496–497` | startup |
| `sim_actuation.firmware_pi_kp_pwm_per_mps` / `_ki_pwm_per_mps_s` / `_int_max_pwm` / `_hold_thresh_mps` | 30 / 5000 / 100 / 0.02 (`:72–75`) | `:498–501` | startup |
| `sim_actuation.min_linear_vel` / `control_hz` | 0.05 / 50 (`:76–78`) | `:502` (min_linear_vel only) | startup |
| `fake_hardware_bridge.dock_x` / `dock_y` / `dock_pose_yaw` / `dock_proximity` | 0 / 0 / 0 / 0.3 m (`fake_hardware_bridge_node.cpp:83–86`) | none — dock is the world origin | startup |
| `sim_imu_noise.input_topic` / `output_topic` | `/imu/data_gz` / `/imu/data` (`sim_imu_noise.py:68,71`) | `:437` `/imu/data_sim`, `:438` | startup |
| `sim_imu_noise.gyro_white_std` / `gyro_bias_walk_std` / `gyro_bias_init_std` | 0.005 / 1e-4 / 1e-3 (`:76–83`) | `:445–447` (same) | startup |
| `sim_imu_noise.accel_white_std` / `accel_bias_walk_std` / `accel_bias_init_std` | 0.05 / 1e-3 / 0.05 (`:87–94`) | `:448–450` (same) | startup |
| `sim_imu_noise.synthesize_from_cmd_vel` / `cmd_vel_topic` / `noise_seed` | false / `/cmd_vel` / 42 (`:119,122,98`) | `:460` `False`, `:461`, `:451` | startup |
| `sim_navsat_rtk_fix.quality_pattern` | `""` = always `RTK_FIXED` (`sim_navsat_rtk_fix.py:117`; grammar `"30,RTK_FIXED;15,RTK_FLOAT;5,NO_FIX"`) | `:330` `""` | startup |
| `sim_navsat_rtk_fix.input_topic` / `output_topic` / `noise_seed` | `/gps/fix_raw` / `/gps/fix` / 42 (`:110,113,122`) | `:321–322`, `:331` | startup |
| `sim_wheel_slip.slip_period_s` / `slip_duration_s` / `slip_vx_bias` | 30 / 1.0 / 0.05 (`sim_wheel_slip.py:60,63,66`) | `:406–412` (same; `0.0` bias = slip-free baseline) | startup |
| `KinematicDrive` `cmdVelTopic` (URDF plugin property) | `/cmd_vel` (`mowgli_webots.urdf:202`; fallback `kinematic_drive.py:158`) | — | plugin init |
| `KinematicDrive` `FIRMWARE_*`, `CMD_VEL_TIMEOUT_S` | class constants `kinematic_drive.py:147–170` — **not ROS params** | — | import time |
| Webots `mode` | `realtime` (`webots_minimal.launch.py:45–49`, `sim_full_system.launch.py:109–113`) | pass `mode:=fast` for E2E only | launch |
| `fusion_graph_tf_lead_s` / `fusion_graph_node_period_s` (into `navigation.launch.py`) | hardware 0.05 s (`navigation.launch.py:216`) / from `mowgli_robot.yaml`, fallback 0.04 s (`:221`) | `:182–183` → 0.1 s / 0.02 s | launch |
| `map_server_node.area_names` / `area_polygons` / `area_is_navigation` / `area_obstacles` | absent in `map_server.yaml` (deliberate) | `:219–224` `main_mow` 9×6 m rectangle | lifecycle configure |
| `datum_lat` / `datum_lon` | — | `:229–230` (map_server) and `:356–357` (navsat) = `mowgli_garden.wbt:19` | startup |

### TF frames
- `map→odom` and `odom→base_footprint`: `fusion_graph_node` only (CLAUDE.md Invariant 2); `diffdrive_controller` has `enable_odom_tf: false` (`ros2_control.yaml:46`).
- `base_footprint→base_link`: fixed, z = 0.093 (`mowgli_webots.urdf:67–73`, base_link at the rear axle).
- `base_link→left_wheel` / `right_wheel`: continuous joints `left_wheel_motor` / `right_wheel_motor` at y = ±0.1625 (`:90–124`), animated from `/joint_states`.
- `base_link→lidar_link` (0, 0.024, 0.390, yaw 3.1408 — composes with `mowgli_garden.wbt:106` to net 0) `:128–141`; `base_link→imu_link` (0.18, 0, 0.095) `:144–149`; `base_link→gps_link` (0.30, 0, 0.19) `:152–158`.
- Webots side: sensors sit in the PROTO `extensionSlot`, lifted +0.283 m (`MowgliMower.proto:137–140`); wheel-bottom is z = 0 in the robot frame so the world instance is at `translation 0 0 0` (`mowgli_garden.wbt:72`).

## Build, test, run
```bash
# Build only this package (from ros2/)
colcon build --packages-select mowgli_simulation --cmake-args -DCMAKE_BUILD_TYPE=Release
# Unit tests (gtest, ROS-free header) — also runs under `make test` / scripts/test.sh
colcon test --packages-select mowgli_simulation && colcon test-result --verbose
PACKAGES="mowgli_simulation" ./scripts/test.sh

# Smoke test: Webots + wheels only, no Nav2 (docs/WEBOTS_SIM.md §9 has the expected readings)
ros2 launch mowgli_simulation webots_minimal.launch.py mode:=realtime

# Full stack, headless (needs an X server: Makefile exports DISPLAY=:99; compose starts Xvfb :99)
cd ros2 && make sim            # = make sim-stop + sim_full_system.launch.py headless:=true use_rviz:=false
cd ros2 && make e2e-test       # sim-stop → build → launch → sleep 90 → python3 src/e2e_test.py → sim-stop
cd ros2 && make e2e-test-no-lidar   # same with use_lidar:=false, runs src/e2e_test_no_lidar.py

# Docker
docker compose -f docker/docker-compose.simulation.yaml up -d simulation      # headless, Foxglove :8765
docker compose -f docker/docker-compose.simulation.yaml up -d simulation-gui  # noVNC http://localhost:6080/vnc.html
cd ros2 && make docker-sim     # builds Dockerfile stage `simulation` (amd64 only — Dockerfile:510)
```
Test files:
- `test/test_firmware_wheel_model.cpp` — pins `step_firmware_wheel_model`: straight-line and pivot convergence (:46, :57), sub-deadband stall then bounded buzz (:74), kinetic hysteresis keeps a rolling wheel moving (:102), saturation to `max_mps` (:124), stop settles to exactly 0 (:134), integrator clamp (:156), sign-flip integrator reset (:175). Registered `CMakeLists.txt:121`.
- `ros2/src/mowgli_bringup/test/test_gnss_launch_config.py:59` — pins that `sim_full_system.launch.py` carries no legacy `publish_gnss_status` / `gnss_backend` / `gps_protocol` params.
- E2E (`ros2/src/e2e_test.py`, `ros2/src/e2e_test_no_lidar.py`) drive the BT through `/behavior_tree_node/high_level_control` and watch `/behavior_tree_node/high_level_status`, `/wheel_odom`, `/odometry/filtered_map`, `/scan`, `/map`, `/map_server_node/boundary_violation`; the fake bridge serves their `/hardware_bridge/emergency_stop` calls (`e2e_test.py:217`).

CI: `.github/workflows/ros2-ci.yml` builds the whole workspace and runs `colcon test` (:347) — the gtest runs there; **no Webots/E2E in CI**. `.github/workflows/ros2-docker.yml` builds only `target: [runtime]` (:36) — the `simulation` image is never built or published by CI.

## Change coupling — "if you change X, also update Y"
- **Wheel model gains** are quadruplicated: `firmware_wheel_model.hpp:60–72` ⇄ `kinematic_drive.py:147–155` ⇄ `sim_full_system.launch.py:491–502` ⇄ firmware `firmware/stm32/ros_usbnode/src/ros/ros_custom/cpp_main.cpp:111–115` (`WHEEL_PI_KP_PWM_PER_MPS` 30, `KI` 5000, `INT_MAX` 100) and `include/board.h` (`MAX_MPS`, `PWM_PER_MPS`, `WHEEL_BASE`). Body pose (Python) and `/wheel_odom_raw` (C++) only agree while the two copies match.
- **Track / radius** `0.325` / `0.093`: `ros2_control.yaml:34–35`, `kinematic_drive.py:129–130`, `sim_actuation_node.cpp:66`, `MowgliMower.proto:147,205` (anchors ±0.1625), `mowgli_webots.urdf:72,97,121`.
- **Sensor placement** must be changed in two places: Webots `extensionSlot` (`mowgli_garden.wbt:82–125`) and the URDF fixed joint (`mowgli_webots.urdf:126–158`); frame names in `<frameName>` (`:168,178,187`) must match the URDF links.
- **Device names** `left_wheel_motor` / `right_wheel_motor` / `left_wheel_sensor` / `right_wheel_sensor` / `lidar` / `gps` / `inertial_unit` / `gyro` / `accelerometer`: PROTO (`:155,161,211,216`; world `:84–86,92,107`) ⇄ URDF `<device reference>` / plugin names (`:162,172,188–190`) ⇄ `ros2_control.yaml:31–32`.
- **Datum**: `mowgli_garden.wbt:19` ⇄ `sim_full_system.launch.py:229–230` ⇄ `:356–357` (Invariant 4). The 9×6 m polygon at `:221` is in that datum's map frame.
- **`basicTimeStep 20`** (`mowgli_garden.wbt:16`) ⇄ `controller_manager.update_rate: 50` (`ros2_control.yaml:21`) ⇄ `sim_actuation.control_hz` 50 ⇄ `kDt = 0.02` in the gtest (:24).
- **Topic renames** ripple: `/cmd_vel_wheels` and `/wheel_odom_raw` remaps (`webots_minimal.launch.py:111–112`) ⇄ `sim_actuation_node.cpp:80` ⇄ `sim_wheel_slip` `input_topic`; `/imu/data_sim` (`mowgli_webots.urdf:185`) ⇄ `sim_full_system.launch.py:437`; `/gps/fix_raw` (`:176`) ⇄ `:321`.
- **Status / Power / Emergency fields** added to `mowgli_interfaces` must be mirrored in `fake_hardware_bridge_node.cpp:168–246` or the BT sees defaults (e.g. `firmware_compatible` had to be forced `true`, `:189`).
- **New launch arg on `sim_full_system.launch.py`** → also `ros2/Makefile:89,98,117`, `docker/docker-compose.simulation.yaml` commands, `ros2/scripts/start_vnc.sh:62–64`, `start_dev_sim.sh:72–73`.
- **New sim helper node** → add its process pattern to `ros2/scripts/sim-stop.sh:49` or it survives `make sim-stop` and holds a DDS participant.
- **New sim script** → `CMakeLists.txt:78–84` (`install(PROGRAMS …)`); the Dockerfile CRLF fix (`:543`) only covers `lib/mowgli_simulation/*.py`.

## Pitfalls
- Read `docs/WEBOTS_SIM.md` §3–§7 before touching `worlds_webots/`, `protos/`, `urdf_webots/`, or `kinematic_drive.py`: wheel `Cylinder`+`boundingObject` need the `Pose { rotation 1 0 0 -1.5708 }` (`MowgliMower.proto:168,187,226`); `axis 0 -1 0` is deliberate (`:146,204`); the teleport + `setVelocity([…,0.0,…,wz])` pair and the single init-time `resetPhysics()` are load-bearing (`kinematic_drive.py:273–275,456–457,513–515`); never let the plugin write the motors (`:214–225,542–565`).
- `/cmd_vel` in sim is `TwistStamped` (`kinematic_drive.py:241`, `ros2_control.yaml:40`). `ros2/src/e2e_test.py:202–206` subscribes `/cmd_vel` and `/cmd_vel_smoothed` as `geometry_msgs/Twist` — those callbacks never fire under Webots.
- `ros2/src/e2e_test.py:561–583` spawns its obstacle with `gz service` (Gazebo); under Webots it fails and the obstacle test reports `SKIP`. `/gps_degradation_sim/status` (`:195`) has no publisher — no such node exists (grep the tree). `make e2e-test-no-lidar` passes `simulate_gps_degradation:=false` (`Makefile:118`), which no launch file declares.
- `headless` and `use_rviz` args on `sim_full_system.launch.py` are accepted and ignored (`:85–93`, `use_rviz` is never consumed); GUI vs headless is decided by `DISPLAY` (Makefile `:87`, compose Xvfb). `rviz/mowgli_sim.rviz` is installed but never launched. The `mode_arg` comment at `:148–150` says "Defaults to fast" — the default is `realtime` (`:111`).
- `sim_navsat_rtk_fix.py` **must** publish RELIABLE (`:144–153`); the docstring's "GBAS_FIX (4)" (`:13`) is wrong — the constant is `STATUS_GBAS_FIX = 2`, the code uses the symbol. `quality_pattern` is currently `""` in launch (`sim_full_system.launch.py:330`), so RTK-Float behaviour is untested unless you set it.
- `sim_imu_noise.py` defaults `input_topic` to the Gazebo-era `/imu/data_gz` (`:68`); standalone it produces nothing — only the launch override (`sim_full_system.launch.py:437`) makes it work (WEBOTS_SIM.md §10c).
- `fake_hardware_bridge` reports `is_charging = true` before the first ground-truth pose arrives (`:162–165`) and whenever the truth pose is within `dock_proximity` (0.3 m) of the origin — the dock is hard-wired at (0,0) (`:83–86`), not read from `mowgli_robot.yaml` (Invariant 6 does not apply in sim). Its `emergency_stop` clears without a physical-trigger check (`:64–80`) — do not use it to reason about firmware latch semantics (Invariant 9).
- The sim `PWM_PER_MPS = 300` matches `board.h:89` (LUV1000RI) only; YardForce 500 classic is 337 (`board.h:50`) and variant B is 275 (`:71`), and the firmware value is host-tunable at runtime (`cpp_main.cpp:171`). Deadband 40 comes from the `cpp_main.cpp:92` comment; kinetic 30 and `hold 0.02` are sim-side choices with no firmware `#define`.
- Invariant 16's dig detector reads `/odometry/filtered_map`; in sim the only wheel-independent truth is `/sim/ground_truth_pose`. `sim_wheel_slip` over-reports `/wheel_odom` vx by 0.05 m/s for 1 s every 30 s (`sim_full_system.launch.py:406–412`) — expect periodic encoder-vs-GNSS divergence by design.
- Stale in-code comments still name `ekf_odom_node` / `ekf_map_node` / robot_localization (`kinematic_drive.py:43,85,223,495,528,555`; `ros2_control.yaml:12,44–46`; `mowgli_webots.urdf:56`; `MowgliMower.proto:11`; `sim_full_system.launch.py:28–29,342`; `sim_wheel_slip.py:8,19`; `e2e_test_no_lidar.py:12`). The localizer is `fusion_graph_node` (Invariant 1); the mechanism described is otherwise right.
- `navigation.launch.py:1106–1112` says `cog_stationary_seed_rate_hz` is "overridden to 0.0 in sim_full_system.launch.py" — it is not (`sim_full_system.launch.py:166–169` deliberately keeps the default). Likewise `sim_full_system.launch.py:171–181`, `navigation.launch.py:203–212` and `fusion_graph.yaml:345–346` describe the hardware TF lead as 0.0 / 25 Hz; the live defaults are 0.05 s (`navigation.launch.py:216`) and the `mowgli_robot.yaml` period (fallback 0.04, `:221`).
- `docker/docker-compose.simulation.yaml:82–83` bind-mounts `../ros2/src/mowgli_simulation/worlds` and `/config`, which do not exist (the dirs are `worlds_webots/` and `config_webots/`); Docker creates empty host dirs and the mounts are inert. The compose comments (`:24`, `:94`) still say "Gazebo".
- After any crash run `ros2/scripts/sim-stop.sh` — a stale `/tmp/webots/*/ipc/MowgliMower/extern` socket makes the next launch hang in "retrying" (`sim-stop.sh:54–61`).
- `webots-controller` has a hard 30 s connect timeout vs a 20–40 s world load; keep `respawn=True` (`webots_minimal.launch.py:114–125`).
- The `simulation` Docker stage is amd64-only (`Dockerfile:510`) — there is no ARM sim image.

## Generated & vendored — do not hand-edit
- `protos/MowgliMower.proto:90` pulls `webots://projects/appearances/protos/TireRubber.proto` from the Webots install; `/opt/webots/resources/nodes/*.wrl` are Webots' own node definitions referenced in comments.
- `webots_ros2_driver` / `webots_ros2_control` come from `ros-kilted-webots-ros2` (`Dockerfile:509`); `Dockerfile:524–538` patches its `utils.is_wsl()` in place at image build — re-check that patch if the driver version changes.
- `ros2/foxglove/mowgli_sim.json` is an exported Foxglove layout (edit in Foxglove, re-export).
