# Codemap: mowgli_nav2_plugins

> Nav2 plugin library (`libmowgli_nav2_plugins.so`) for the COVERAGE lane of `controller_server`. It owns `mowgli_nav2_plugins/FTCController` (Follow-the-Carrot: 5-state FSM, decoupled lon/lat/ang PID, anti-wheelspin stall crawl, lateral obstacle deviation with zone guard/mask, cul-de-sac guard, bounded reverse-escape, oscillation override) in the `FollowCoveragePath` slot, and `mowgli_nav2_plugins/PathProgressGoalChecker` in the `coverage_goal_checker` slot. Transit (`FollowPath`) is upstream RotationShim+RPP and is NOT in this package (CLAUDE.md Invariant 8). No node of its own — everything runs inside `controller_server`.
> Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `ros2/CLAUDE.md`.

## Where to look

| Task | Start here |
|------|------------|
| Carrot speed target / accel ramp / carrot lead cap (1.0 m) | `ros2/src/mowgli_nav2_plugins/src/ftc_controller.cpp` `update_control_point()` (L1176) + `distanceLookahead()` (L1144) |
| PID mix, forward_only clamp, min_speed floor, stall cap, oscillation override | `ftc_controller.cpp` `calculate_velocity_commands()` (L1407) |
| FSM transitions / timeouts (`PRE_ROTATE → FOLLOWING → WAITING_FOR_GOAL_APPROACH → POST_ROTATE → FINISHED`) | `ftc_controller.cpp` `update_planner_state()` (L987); enum at `include/mowgli_nav2_plugins/ftc_controller.hpp:93` |
| Where a fresh plan starts tracking (idx 0 vs legacy nearest snap) | `ftc_controller.cpp` `setPlan()` (L616) + `include/mowgli_nav2_plugins/ftc_start_index.hpp` `ChooseStartIndex` |
| Anti-wheelspin stall (crawl at `stall_crawl_speed`, freeze carrot) | `include/mowgli_nav2_plugins/ftc_stall.hpp` `StallDecision`; consumed in `update_control_point()` and `calculate_velocity_commands()` (`is_stalled_`) |
| Blade-load slowdown (scale carrot speed by blade RPM sag, fail-open) | `include/mowgli_nav2_plugins/ftc_blade_load.hpp` `BladeLoadDecision`; `applyBladeLoad()` in `update_control_point()`, `is_blade_limited_` lowers the `min_speed_mps` floor in `calculate_velocity_commands()`; telemetry from `/hardware_bridge/status` |
| Obstacle skirt policy: side choice, grow, min floor, clear-hold, wait-or-abort | `ftc_controller.cpp` `updateLateralDeviation()` (L1821); pure helpers `src/obstacle_deviation.cpp` |
| Zone confine (offset must stay in-zone) / zone mask (#517, out-of-zone lethal is not an obstacle) | `BoundaryGuard` in `include/mowgli_nav2_plugins/obstacle_deviation.hpp`; built in `updateLateralDeviation()` from `boundary_costmap_` (sub `/global_costmap/costmap`, `ftc_controller.cpp:75`) |
| Footprint vs half-width line model, front clip, lateral expand | `obstacle_deviation.cpp` `footprintBlocked` / `clipFootprintFront` / `expandFootprintLateral`; toggled by `use_footprint_clearance` |
| Cul-de-sac guard (refuse to skirt a wall) | `obstacle_deviation.cpp` `hasClearExit` + `require_clear_exit` branch in `updateLateralDeviation()` |
| Bounded straight reverse-escape (SAFETY-CRITICAL, only place FTC reverses) | `include/mowgli_nav2_plugins/ftc_reverse_escape.hpp` + `ftc_controller.cpp` `reverseEscapeOrWait()` (L1730); emitted at L921 |
| Wait-before-abort window (`obstacle_wait_timeout_s`) | `ftc_controller.cpp` `waitOrThrowForObstacle()` (L1703) — the ControllerException throw is L1717 |
| Body-in-lethal check at the ACTUAL robot pose (SAFETY_REVIEW F-C1) | `ftc_controller.cpp` `currentBodyInLethal()` (L1663), gated at L907 |
| Legacy collision throw (deviation OFF, e.g. no-LiDAR) | `ftc_controller.cpp` `checkCollision()` (L1577) — frame caveat at L1621 |
| Oscillation detector (ring buffer, zero-crossings) | `src/oscillation_detector.cpp` `FailureDetector::detect`; wrapper `checkOscillation()` (L2296) |
| Goal-checker progress gate / short-path proximity fallback / empty-plan watchdog | `src/path_progress_goal_checker.cpp` `isGoalReached()` (L151); new-path fingerprint in `onPath()` (L125) |
| Which topic the goal checker tracks | `path_progress_goal_checker.cpp:63` (`plan_topic`); FTC publishes `<plugin>/global_plan` at `ftc_controller.cpp:66`, `:762` |
| Add / rename an FTC parameter | 4 places: `ftc_controller.hpp` `struct Config` (L345), `declareParameters()` (L140), `onParameterChange()` (L267, finite + range check — out-of-range is REJECTED, not clamped), `ros2/src/mowgli_bringup/config/nav2_params_base.yaml` `FollowCoveragePath:` (L338) |
| Expose a param to operators / GUI | template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml` → `ros2/src/mowgli_bringup/launch/navigation.launch.py` `_inject_dock_pose_and_speeds` (L646, `fcp[...]` L755–847) → `gui/web/src/components/settings/paramCatalog.ts` → `ros2/src/mowgli_bringup/test/test_nav2_params.py` |
| Tune shipped defaults | `nav2_params_base.yaml` L338–546 (`FollowCoveragePath`), L170–189 (`coverage_goal_checker`); no-LiDAR diff `ros2/src/mowgli_bringup/config/nav2_params_no_lidar.yaml` L19–21 |
| Live speed change from the BT (SetNavMode) | `ros2/src/mowgli_behavior/src/navigation_nodes.cpp:962` sets `FollowCoveragePath.speed_fast` → `onParameterChange` |
| Speed limit from collision_monitor | `ftc_controller.cpp` `setSpeedLimit()` (L771) — restores `base_max_cmd_vel_speed_` on negative |
| Register a new plugin class | `ros2/src/mowgli_nav2_plugins/ftc_controller_plugin.xml` / `goal_checker_plugin.xml`, `CMakeLists.txt` `pluginlib_export_plugin_description_file`, `package.xml` `<export>` |
| Add a unit test | `CMakeLists.txt` `if(BUILD_TESTING)` block — every test is ROS-free (pure headers or synthetic `Costmap2D`) |
| Diagnose "strip aborted" (ControllerException) | throw sites: `ftc_controller.cpp:835` (crashed latch), `:957` (deviation TF), `:965` (legacy collision), `:973` (crash latched during PID), `:1357` (carrot TF), `:1717` (wait timeout) |

## Files

| File | Lines | Purpose |
|------|-------|---------|
| **`ros2/src/mowgli_nav2_plugins/`** | | |
| `CMakeLists.txt` | 192 | One shared lib from 5 .cpp; exports both plugin XMLs to `nav2_core`; 5 gtests |
| `package.xml` | 40 | ament_cmake; deps nav2_core/nav2_costmap_2d/nav2_util/pluginlib/tf2*/Eigen; `<nav2_core plugin=...>` exports |
| `ftc_controller_plugin.xml` | 12 | pluginlib: `mowgli_nav2_plugins/FTCController` → `nav2_core::Controller`, `<library path="mowgli_nav2_plugins">` |
| `goal_checker_plugin.xml` | 16 | pluginlib: `mowgli_nav2_plugins/PathProgressGoalChecker` → `nav2_core::GoalChecker` |
| **`include/mowgli_nav2_plugins/`** | | |
| `ftc_controller.hpp` | 581 | `FTCController` class: FSM enum, carrot/PID/deviation/reverse/oscillation state, `struct Config` (all params + C++ defaults) |
| `ftc_stall.hpp` | 74 | Pure `StallDecision()` — stall_time debounce, crawl easing, `in_stall` flag |
| `ftc_blade_load.hpp` | 105 | Pure `BladeLoadScale()` / `BladeLoadDecision()` — linear RPM→speed ramp, inactive/stale/degenerate gates fail OPEN, `stall_crawl_speed` floor |
| `ftc_reverse_escape.hpp` | 83 | Pure `ReverseEscapeDecide()` / `ReverseEscapeAdvance()` — opt-in, budget cap, rear-clear gate |
| `ftc_start_index.hpp` | 80 | Pure `ChooseStartIndex()` — idx 0 by default; legacy nearest snap breaks ties to the earlier index |
| `obstacle_deviation.hpp` | 236 | `BoundaryGuard` (zone guard + zone mask) and `ObstacleDeviation` static helpers; thresholds `kLethalThreshold=253`, `kLethalOnlyThreshold=254` |
| `oscillation_detector.hpp` | 113 | `FailureDetector` — rolling (v, ω) window, mean + zero-crossing test |
| `path_progress_goal_checker.hpp` | 122 | `PathProgressGoalChecker` — progress-gated goal checker state + params |
| **`src/`** | | |
| `ftc_controller.cpp` | ~2.4k | Lifecycle, params + dynamic callback, setPlan, computeVelocityCommands, FSM, carrot, PID, collision, deviation, reverse-escape, oscillation |
| `ftc_controller_plugin.cpp` | 20 | `PLUGINLIB_EXPORT_CLASS(FTCController, nav2_core::Controller)` |
| `obstacle_deviation.cpp` | 486 | cell/body/footprint samplers, `isObstacleCell`, `hasClearExit`, `findFirstObstacleIndex`, `chooseDeviationSide`, `isPathClearWithDeviation`, `growDeviationUntilClear` |
| `oscillation_detector.cpp` | 153 | `FailureDetector` impl (normalise by v_max/ω_max, half-full buffer before deciding) |
| `path_progress_goal_checker.cpp` | 317 | `initialize` (params + plan sub), `onPath` fingerprint, `isGoalReached`, `getTolerances`; `PLUGINLIB_EXPORT_CLASS` at bottom |
| **`test/`** | | |
| `test_ftc_stall.cpp` | 160 | 10 cases on `StallDecision` (disable, grace, crawl, reset, cap-not-floor) |
| `test_ftc_blade_load.cpp` | 180 | 17 cases on `BladeLoadScale` / `BladeLoadDecision` (disabled/inactive/stale/degenerate fail open, ramp endpoints + midpoint, floor never raises speed, immediate recovery) |
| `test_ftc_reverse_escape.cpp` | 108 | 10 cases: opt-in default, rear-blocked never reverses, budget cap, advance arithmetic |
| `test_ftc_start_index.cpp` | 77 | 4 cases: fresh plan → 0, closed ring never resolves to its end, legacy snap, empty plan |
| `test_obstacle_deviation.cpp` | 823 | ~50 cases on a 400×400 @0.05 m synthetic costmap: detection, side choice (left bias), grow, boundary guard, zone mask (#517), footprint model, clip/expand, `hasClearExit`, lookahead clamp |
| `test_oscillation_detector.cpp` | 195 | 11 cases: capacity, half-full gate, alternating ω detected, steady motion not |

## Runtime surface

### Nodes
None. Both classes are pluginlib plugins loaded by Nav2's `controller_server` (launched in `ros2/src/mowgli_bringup/launch/nav2_navigation_launch.py`, params merged by `navigation.launch.py`). Plugin instance names: `FollowCoveragePath` (controller) and `coverage_goal_checker` (goal checker), declared at `nav2_params_base.yaml:112-113`. Lifecycle follows `controller_server` (`configure/activate/deactivate/cleanup` in `ftc_controller.cpp:41-136`).

### Topics
| Topic | Type | Dir | QoS | Notes / other end |
|-------|------|-----|-----|-------------------|
| `/controller_server/FollowCoveragePath/global_point` | `geometry_msgs/PoseStamped` | pub | depth 1, lifecycle | Carrot pose (map frame) every tick — viz only |
| `/controller_server/FollowCoveragePath/global_plan` | `nav_msgs/Path` | pub | `QoS(1).transient_local()` | Published ONCE per `setPlan` with the tail pose duplicated (`ftc_controller.cpp:762`). Sub: `PathProgressGoalChecker` (`KeepLast(1).reliable()`, `path_progress_goal_checker.cpp:67-70`). ALSO published by BT `FollowStrip` (`ros2/src/mowgli_behavior/src/coverage_nodes.cpp:391`, same QoS) |
| `/controller_server/FollowCoveragePath/costmap_marker` | `visualization_msgs/Marker` | pub | depth 10 | Only when `debug_obstacle` and deviation OFF (`debugObstacle`) |
| `/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | sub | `QoS(1).transient_local()` | Rebuilt into `boundary_costmap_` (`data >= 99 → 254`, else 0) for the zone guard/mask (`ftc_controller.cpp:75-96`) |
| `/hardware_bridge/status` | `mowgli_interfaces/Status` | sub | `QoS(1)` | `mower_esc_status` / `mower_motor_rpm` / `blade_status_stamp` → `blade_active_` / `blade_rpm_` / `blade_status_time_` under `blade_mutex_` for the blade-load slowdown (always subscribed; inert unless `blade_load_slowdown_enabled`) |
| (via controller_server) `odom_topic` = `/wheel_odom` | `nav_msgs/Odometry` | in | — | `velocity.linear.x` feeds stall detection + reverse-escape budget (`nav2_params_base.yaml:77`) |

### Services & actions
None owned. Reached through Nav2 `follow_path` with `controller_id="FollowCoveragePath"`, `goal_checker_id="coverage_goal_checker"` (`coverage_nodes.cpp:581-582`). Failure surface = `nav2_core::ControllerException` → action ABORTED → BT FollowStrip detour/skip logic. Parameters are settable live via `controller_server`'s parameter services (used by SetNavMode).

### Parameters
FTC: all `FollowCoveragePath.*` keys are declared in `declareParameters()`; all of them EXCEPT `snap_to_nearest_on_set_plan` (declare-time only — no `onParameterChange` branch, so it is read once at configure) are DYNAMIC through `onParameterChange()` (finite + range check per key, rejected not clamped; `speed_fast` ∈ [0, 2.0]). Goal checker keys are read ONCE in `initialize()`. Prefix omitted below.

| Param | base.yaml | C++ default | Launch override (`navigation.launch.py`) |
|-------|-----------|-------------|------------------------------------------|
| `speed_fast` | L346 = 0.20 | 0.5 | L755 = `mowing_speed`; also set live by SetNavMode |
| `speed_slow` | L357 = 0.16 | 0.2 | L781 = `derive_turn_speed(mowing_speed, turn_speed_ratio, min_speed_mps)` (`robot_config_util.py:268`) |
| `max_cmd_vel_speed` / `max_cmd_vel_ang` | L403 = 0.30 / L404 = 0.8 | 2.0 / 2.0 | L764 raised to `mowing_speed` if larger |
| `min_speed_mps` | L360 = 0.15 | 0.15 | — (read back at L778 as `derive_turn_speed`'s floor) |
| `stall_speed_ratio` / `stall_grace_s` / `stall_crawl_speed` | L366–368 = 0.35 / 0.6 / 0.08 | same | — (pinned by `test_ftc_stall_trio_present_in_both_variants`) |
| `blade_load_slowdown_enabled` / `blade_load_rpm_full` / `blade_load_rpm_min` / `blade_load_min_speed_ratio` / `blade_load_telemetry_max_age_s` | false / 2500 / 1800 / 0.4 / 1.0 | same | first four from template `blade_load_*` via `derive_blade_load_params` (`navigation.launch.py`, after `speed_slow`); pinned by `test_ftc_blade_load_keys_present_in_both_variants` + `test_navigation_launch_injects_ftc_blade_load` |
| `kp_lat` / `kd_lat` / `kp_ang` / `kp_ang_following` / `derivative_filter_tau` | L388 0.8 / L390 0.5 / L391 1.5 / L392 1.0 / L402 0.2 | 1.0 / 0 / 1.0 / =kp_ang / 0 | — |
| `max_goal_distance_error` | L410 = 0.50 | 1.0 | — but FLOORS `coverage_goal_checker.xy_goal_tolerance` (L890–905) |
| `max_goal_angle_error` / `goal_timeout` / `max_follow_distance` | L411 30.0 / L412 10.0 / L413 2.0 | 10.0 / 5.0 / 1.0 | — |
| `forward_only` | L420 = true | true | — |
| `snap_to_nearest_on_set_plan` / `min_lateral_deviation` | not in yaml | false / 0.30 | — |
| `check_obstacles` / `enable_obstacle_deviation` | L425 / L509 = true | true | `nav2_params_no_lidar.yaml:20-21` = false |
| `obstacle_lookahead` (poses) | L431 = 30 | 5 | L823 = `max(4, round(clamp(obstacle_detection_range_m, 0.2, 5.0) / 0.05))` |
| `use_footprint_clearance` / `obstacle_footprint_front_length_m` | L450 = false / L458 = 0.30 | false (declare) / 0.30 | — |
| `obstacle_body_half_width` / `obstacle_clearance_margin` | L477 = 0.12 / L490 = 0.05 | 0.20 / 0.0 | margin: L831 = clamp(`obstacle_clearance_margin`, 0, 0.5) |
| `require_clear_exit` / `confine_deviation_to_zone` / `ignore_obstacles_outside_zone` | L468 true / (absent) / L508 true | true / true / true | — |
| `max_lateral_deviation` / `deviation_step` / `deviation_blend_rate` | L515 1.5 / L516 0.05 / L517 0.5 | same | L811 = clamp(`max_obstacle_avoidance_distance`, 0.5, 10.0) |
| `obstacle_wait_timeout_s` / `obstacle_clear_hold_s` | L525 2.5 / L526 1.5 | 2.5 / 1.5 | L838 = clamp(`obstacle_wait_timeout_s`, 0.5, 60) |
| `obstacle_reverse_enabled` / `_max_dist_m` / `_speed_mps` | L544 true / L545 0.30 / L546 0.10 | **false** / 0.30 / 0.10 | L844–847 from `mowgli_robot.yaml` (template: true / 0.30 / 0.15), clamps dist [0,1], speed [0,0.3] |
| `oscillation_recovery` / `_v_eps` / `_omega_eps` / `_recovery_min_duration` | not in yaml | true / 0.05 / 0.05 / 5.0 | buffer len = round(duration × 10) samples |
| `coverage_goal_checker.progress_threshold` | L172 = 0.95 | 0.95 | — |
| `coverage_goal_checker.xy_goal_tolerance` / `yaw_goal_tolerance` | L187 0.50 / L188 3.14 | 0.20 / 0.30 | xy: L905 = max(`coverage_xy_tolerance`, FTC `max_goal_distance_error`) |
| `coverage_goal_checker.plan_topic` | L189 | `/controller_server/FollowCoveragePath/global_plan` | — |
| `coverage_goal_checker.short_path_poses` / `max_idx_advance_per_call` / `fallback_timeout_s` | not in yaml | 10 / 10 / 5.0 | — |

### TF frames
- Carrot → PID errors: `base_link ← map` (`ftc_controller.cpp:1348`, re-projected after deviation at `:947`); `cmd_vel.header.frame_id = "base_link"`. FTC ignores the `pose` argument Nav2 passes in.
- Deviation window: plan frame (map) → `costmap_ros_->getGlobalFrameID()` (odom) before sampling the local costmap.
- Zone guard affine: `boundary_frame_` (global costmap frame, map) ← odom.
- Both `map→odom` and `odom→base_footprint` come from `fusion_graph_node` (Invariant 2); FTC only consumes.

## Build, test, run

```bash
# inside the devcontainer, from ros2/
make build-pkg PKG=mowgli_nav2_plugins            # = scripts/build.sh with PACKAGES=...
colcon build --packages-select mowgli_nav2_plugins --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon test  --packages-select mowgli_nav2_plugins && colcon test-result --verbose
make test                                          # whole workspace (scripts/test.sh)
make format                                        # clang-format (CI pins clang-format-18)
# config-side guards for this package's wiring:
colcon test --packages-select mowgli_bringup       # runs test/test_nav2_params.py via ament_add_pytest_test (mowgli_bringup/CMakeLists.txt:60)
```

Unit tests (all ROS-free gtests, registered in `CMakeLists.txt`):
- `test/test_ftc_stall.cpp` — `StallDecision` disable/grace/crawl/reset; stall flag caps output instead of flooring.
- `test/test_ftc_blade_load.cpp` — `BladeLoadDecision` fails open on disabled/inactive/stale/degenerate; linear ramp endpoints; floor never raises the speed; no hysteresis.
- `test/test_ftc_reverse_escape.cpp` — default is opt-in OFF; rear-blocked never reverses; budget hard cap; negative dt ignored.
- `test/test_ftc_start_index.cpp` — fresh plan starts at 0; closed ring never resolves to its last index.
- `test/test_obstacle_deviation.cpp` — detection reach vs clearance reach are separate; left bias on ties; boundary guard rejects out-of-zone offsets; zone mask ignores out-of-zone lethal but keeps in-zone; footprint model thresholds 254 while line model 253; `hasClearExit` false when obstacle fills the window.
- `test/test_oscillation_detector.cpp` — half-full gate, alternating-ω detection, capacity trimming.

Config tests in `ros2/src/mowgli_bringup/test/test_nav2_params.py` that pin this package's wiring: `test_coverage_goal_checker_is_path_progress`, `test_coverage_goal_checker_progress_threshold_is_high`, `test_followcoveragepath_uses_ftc`, `test_coverage_is_ftc_transit_is_not`, `test_ftc_stall_trio_present_in_both_variants`, `test_navigation_launch_injects_ftc_{max_lateral_deviation,clearance_margin,wait_timeout}`, `test_navigation_launch_floors_coverage_tolerance_at_ftc_park`, `test_base_ftc_clamp_admits_speed_fast`, `test_no_lidar_followcoveragepath_uses_ftc_without_obstacle_checks`, `test_base_ftc_can_command_its_own_turn_speed`.

CI: `.github/workflows/ros2-ci.yml` job `build-and-test` (L128) runs `colcon build` + `colcon test` over the whole `ros2/` workspace; job `format-check` (L404) runs `git-clang-format-18` on changed lines.

## Change coupling — "if you change X, also update Y"

- **New/renamed FTC param** → `ftc_controller.hpp` `Config` + `declareParameters()` + `onParameterChange()` (else it is not dynamic and SetNavMode/GUI writes fail) + `nav2_params_base.yaml`. Operator-facing: template `mowgli_robot.yaml` + `navigation.launch.py` injection + `paramCatalog.ts` + `gui/web/src/hooks/useSettingsManager.ts:197-198` + a `test_nav2_params.py` pin.
- **Plugin slot name `FollowCoveragePath`** appears in: `path_progress_goal_checker.cpp:64` (default `plan_topic`), `nav2_params_base.yaml:113,189,338`, `coverage_nodes.cpp:391,581`, `navigation_nodes.cpp:962`, `navigation.launch.py:754`, `test_nav2_params.py`.
- **`max_goal_distance_error`** ↔ `coverage_goal_checker.xy_goal_tolerance` floor (`navigation.launch.py:890-905`), template `coverage_xy_tolerance` (`mowgli_robot.yaml:687`), `test_*_ftc_park` tests.
- **`max_cmd_vel_speed` clamp** ↔ `navigation.launch.py:762-764` (raise-to-`mowing_speed`); `speed_fast` callback range [0, 2.0].
- **`min_speed_mps` / `max_cmd_vel_ang`** are read by `robot_config_util.derive_turn_speed` / `check_turn_geometry` (`test_robot_config_util.py:251,296`).
- **`obstacle_lookahead` is a pose count** assuming F2C 0.05 m sampling (`kF2CSamplingM`, `navigation.launch.py:822`); change coverage sampling → change the conversion.
- **Line-model threshold 253** relies on the local costmap inscribed band → `local_costmap.inflation_layer.inflation_radius` floor 0.58 (`navigation.launch.py:859-860`). Footprint model (254) does not; clearance there is `obstacle_clearance_margin` only.
- **Goal-checker topic has TWO publishers** (FTC `setPlan` and BT `FollowStrip`); keep QoS identical (`coverage_nodes.cpp:387-391`).
- **`oscillation_recovery_min_duration` × 10 = buffer length** assumes `controller_frequency: 10.0` (`nav2_params_base.yaml:83`).
- **`<library path="mowgli_nav2_plugins">`** in both XMLs must equal the CMake target name.
- **`controller_server.odom_topic`** must stay a published topic (CLAUDE.md "Do NOT leave controller_server.odom_topic unset") — stall detection and the reverse budget read `velocity.linear.x`.

## Pitfalls

- `obstacle_reverse_enabled` has FOUR defaults: C++ `false` (`ftc_controller.hpp:551`), `nav2_params_base.yaml:544` `true`, template `mowgli_robot.yaml:651` `true`, launch fallback `False` (`navigation.launch.py:523`). On a real robot the launch injection from the installed/template yaml wins. Safety-critical — reversing with blades.
- `use_footprint_clearance`: `Config` initialiser says `true` (`ftc_controller.hpp:517`) but `declareParameters()` declares `false` and `nav2_params_base.yaml:450` ships `false` (field 2026-07-22: full-footprint model found no clear side). The struct initialiser is dead.
- `nav2_params_base.yaml:330-331` comment says FTC "follows U-turn arcs in reverse (forward_only=false)"; the live value is `forward_only: true` (L420). Reverse motion exists ONLY as the escape sub-state.
- `checkCollision()` samples map-frame plan poses against the odom-frame local costmap without a transform (`ftc_controller.cpp:1621-1626`). It only runs when `enable_obstacle_deviation=false`; do not turn `check_obstacles` on with deviation off.
- Never call `goal_checker->reset()` inside `computeVelocityCommands` (`ftc_controller.cpp:844-853`): `max_reached_index_` can only advance `max_idx_advance_per_call` (10) poses per call, so a per-tick reset pins progress < 95 % forever.
- `setPlan` duplicates the last pose and re-orients the second-to-last (`ftc_controller.cpp:744-748`); the FSM's `size() - 2` test depends on it. Plans with < 3 poses go straight to `FINISHED` — one reason FTC is not the transit controller.
- `setPlan` starts at idx 0 by default; re-adding a nearest-point snap skipped 46–99 % of closed headland rings on 2026-08-24 (`ftc_start_index.hpp` header). Resume trimming is `FollowStrip`'s job.
- Goal-checker "new path" fingerprint = pose-count change OR front pose moved > 2 m (`path_progress_goal_checker.cpp:123-125`). Two consecutive plans of identical length starting < 2 m apart are treated as the SAME path (progress carries over).
- Paths with ≤ `short_path_poses` (10) poses complete on xy+yaw proximity only (`path_progress_goal_checker.cpp:209`), bypassing the progress gate.
- `chooseDeviationSide` scans LEFT first at each radius (`obstacle_deviation.cpp:400-412`) — equal clearance always skirts left.
- `updateLateralDeviation` holds the costmap mutex (`ftc_controller.cpp:1832`) and `boundary_mutex_` for its whole body; do not call `costmap_ros_` methods that re-lock from inside.
- With `confine_deviation_to_zone=true` and no `/global_costmap/costmap` received yet, deviation is SKIPPED for the tick (fail-safe, throttled warn) — after a costmap restart FTC drives the nominal line until the latched grid arrives.
- `speed_fast` set outside [0, 2.0] via `set_parameters` is rejected (`result.successful=false`) — SetNavMode does not check the result.
- PID errors are in `base_link` (rear axle, Invariant 2), while Nav2's `robot_base_frame` is `base_footprint`; `max_goal_distance_error` is measured from base_link.
- Invariants to respect: CLAUDE.md 5 (costmap obstacles disabled in coverage — collision_monitor is the real-time guard), 8 (FTC only in the coverage slot; base + overlay YAML), "Do NOT use StoppedGoalChecker for coverage_goal_checker", "Do NOT use RPP or MPPI for coverage paths".

## Generated & vendored — do not hand-edit

- Nothing in-package is generated or vendored. Build artefacts land in `ros2/build/mowgli_nav2_plugins/` and `ros2/install/mowgli_nav2_plugins/` (git-ignored). The FTC algorithm is a port of ROS1 `ftc_local_planner` (see `package.xml` description), not a submodule.
