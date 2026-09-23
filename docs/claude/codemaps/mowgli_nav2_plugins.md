# Codemap: mowgli_nav2_plugins

> Nav2 plugin library (`libmowgli_nav2_plugins.so`) for the COVERAGE lane of `controller_server`. It owns `mowgli_nav2_plugins/FTCController` (Follow-the-Carrot: 5-state FSM plus in-place PIVOT at the coverage planner's explicit corners, decoupled lon/lat/ang PID, anti-wheelspin stall crawl, lateral obstacle deviation with zone guard/mask, debounced obstacle recovery, cul-de-sac guard, bounded reverse-escape, turn fallback for a turn the lattice cannot drive, oscillation override) in the `FollowCoveragePath` slot, and `mowgli_nav2_plugins/PathProgressGoalChecker` in the `coverage_goal_checker` slot. Transit (`FollowPath`) is upstream RotationShim+RPP and is NOT in this package (CLAUDE.md Invariant 8). No node of its own — everything runs inside `controller_server`.
> Index updated 2026-09-22 for the FTC turn fallback (ftc_turn_fallback.hpp, ftc_controller_turn_fallback.cpp); regenerate when files are added/removed.
> Loaded on demand from `ros2/CLAUDE.md`.

> Lyrical API: full plans enter `newPathReceived`; local processed plans do not replace FTC progress. The goal checker transforms the costmap-frame query into the full-plan frame and exposes both XY and XY+yaw checks. Regression tests: `test/test_lyrical_goal_checker.cpp`; end-approach rule (short sub-paths FTC parks short of): `test/test_path_progress_end_approach.cpp`.

## Where to look

| Task | Start here |
|------|------------|
| Carrot speed target / accel ramp / carrot lead cap (1.0 m) | `ros2/src/mowgli_nav2_plugins/src/ftc_controller.cpp` `update_control_point()` (L1176) + `distanceLookahead()` (L1144) |
| PID mix, forward_only clamp, min_speed floor, stall cap, oscillation override | `ftc_controller.cpp` `calculate_velocity_commands()` (L1407) |
| FSM transitions / timeouts (`PRE_ROTATE → FOLLOWING → WAITING_FOR_GOAL_APPROACH → POST_ROTATE → FINISHED`, plus `FOLLOWING ⇄ PIVOT` at planner corners) | `ftc_controller.cpp` `update_planner_state()`; enum at `include/mowgli_nav2_plugins/ftc_controller.hpp` (`PIVOT` appended last so the logged numbers of the other states did not change: PIVOT = 5) |
| **In-place pivots at planner corners** (pivot joins, field 2026-09-21) | corner contract `ros2/src/mowgli_interfaces/include/mowgli_interfaces/coverage_geometry.hpp` (two consecutive poses at the same position: incoming then outgoing heading); pure helpers `include/mowgli_nav2_plugins/ftc_pivot.hpp` (`FindPivotCorners`, `PivotLeg`, `ClipWindowToCorner`, `PivotArrived` 2 cm, `PivotAligned`, `PivotSweepYaws`); controller: corners found in `newPathReceived` (`pivot_corners_`, plan opening on a corner starts at its outgoing twin), carrot CAPPED at the next corner in `update_control_point()`, FOLLOWING → PIVOT in `update_planner_state()` once base_link is within 2 cm (never mid reverse-escape / obstacle hold), `enterPivot()` / `leavePivot()`, PIVOT branch of `computeVelocityCommands()` (no deviation planner, `pivotSweepGate()` → `pivotSweepBlocked()` = rotated footprint vs TRUE-lethal local cells, else PRE_ROTATE's angular PID + oscillation override); tests `test/test_ftc_pivot.cpp` |
| Where a fresh plan starts tracking (idx 0 vs legacy nearest snap) | `ftc_controller.cpp` `newPathReceived()` (L616) + `include/mowgli_nav2_plugins/ftc_start_index.hpp` `ChooseStartIndex` |
| Anti-wheelspin stall (crawl at `stall_crawl_speed`, freeze carrot) | `include/mowgli_nav2_plugins/ftc_stall.hpp` `StallDecision`; consumed in `update_control_point()` and `calculate_velocity_commands()` (`is_stalled_`) |
| Blade-load slowdown (scale carrot speed by blade RPM sag, fail-open) | `include/mowgli_nav2_plugins/ftc_blade_load.hpp` `BladeLoadDecision`; `applyBladeLoad()` in `update_control_point()`, `is_blade_limited_` lowers the `min_speed_mps` floor in `calculate_velocity_commands()`; telemetry from `/hardware_bridge/status` |
| Obstacle skirt policy: side choice, grow, min floor, clear-hold, wait-or-abort | `ftc_controller.cpp` `updateLateralDeviation()` (L1821); pure helpers `src/obstacle_deviation.cpp` |
| Offset lattice: which (pose, offset) nodes are blocked, and why (`O` local lethal under the body / `Z` zone guard, off the line only); the degrading solve (reaction slack → none → ignore the stations under the body, over a shrinking horizon) behind "horizon cut", "fallback level N" and "WEDGED (no collision-free offset profile…)" | `include/mowgli_nav2_plugins/ftc_lattice_solver.hpp` + `src/ftc_lattice_solver.cpp` (`ResampleLatticeWindow`, `LatticeSolver::PoseBlock/SpanBlock/Solve`); the committed-side logic, preview and return debounce stay in `FTCController::planOffsetLattice()`. Tests `test/test_ftc_lattice_solver.cpp` (incl. a WEDGED recorded on 2026-09-22) |
| Replay a field WEDGED offline on the recorded costmaps | `ros2/scripts/lattice_replay_dump.py <bag> <out> T0:T1:name` then `build/mowgli_nav2_plugins/lattice_replay [--offset M --side S --no-zone --keepout-zone -v] <out>/<name>/*` (`test/lattice_replay.cpp`, built with the tests, not installed). Reproduced the 2026-09-22 horizon cuts to the centimetre. `--fallback` (or `--fallback-all`) also runs the turn fallback from the recorded robot pose (`odom_base` in the sample) and prints verdict, reverse, rejoin, skipped arc, clearances and planning time; `--ascii` draws it |
| **Turn fallback** (a turn the lattice cannot drive: hedge past the recorded line at a U-turn / omega / kink, field 2026-09-22 8 of 8 WEDGED) | Decision (pure): `include/mowgli_nav2_plugins/ftc_turn_fallback.hpp` + `src/ftc_turn_fallback.cpp` — `PlanTurnFallback` (first zero-offset lattice-body blockage within `avoidance_horizon_m`; turn = heading sweep over [-0.30, +0.75] m around it ≥ `turn_fallback_min_turn_deg`; rear strip reverse limit; rejoin search least-skipped-arc first, then least reverse; rotations = real footprint vs true-lethal, > 160° need the FULL turn clear; connector = lattice body; body AXIS vs the zone off the plan; 0.05 m planning margin; caller's `followable` predicate), `FallbackWindow`, `RotationSweepClear` / `FullTurnClear` / `StraightSweepClear` / `ReverseSweepClear` (`RearStrip`), `MeasureTurnFallbackClearances`. Lattice-side predicate: `LatticeFeasibleFrom` (`ftc_lattice_solver.hpp`). Controller side: `src/ftc_controller_turn_fallback.cpp` — `tryTurnFallback` (called from `planOffsetLattice` where it would go WEDGED; FOLLOWING only, never over a reverse-escape / hold, re-armed only 1 m past the last rejoin), `planTurnFallback`, `turnFallbackReverseTick` (straight reverse, rear strip probed every tick, odom-measured, time-capped, then re-plans with no further reverse), `spliceTurnFallback` (runtime pivot corners + 0.05 m connector poses into `global_plan_`, `pivot_corners_` recomputed; PIVOT via `enterPivot`), `turnFallbackProgress` (completion at the rejoin → republishes the remainder on `progress_plan_pub_`; `turn_fallback_timeout_s` deadline throws), `failTurnFallback` (charges the reverse to the reverse-escape budget, hold → abort). Tests: `test/test_ftc_turn_fallback.cpp` (pure, incl. the recorded 08:08:25 case), `test/test_ftc_turn_fallback_controller.cpp` (real FTCController + configured Costmap2DROS + kinematic robot, closed loop) |
| Plan handed to the progress trackers after a rejoin | `progress_plan_pub_` = `~/<plugin>/global_plan` = `/controller_server/FollowCoveragePath/global_plan` (goal checker's `plan_topic`, FollowStrip's dispatch topic). NOT `global_plan_pub_`, which resolves to `/FollowCoveragePath/global_plan` (viz only — the goal checker never saw it) |
| Zone confine (offset must stay in-zone) / zone mask (#517, out-of-zone lethal is not an obstacle) | `BoundaryGuard` in `include/mowgli_nav2_plugins/obstacle_deviation.hpp`; built in `updateLateralDeviation()` from `boundary_costmap_` (sub `/global_costmap/costmap`, `ftc_controller.cpp:75`) |
| Footprint vs half-width line model, front clip, lateral expand | `obstacle_deviation.cpp` `footprintBlocked` / `clipFootprintFront` / `expandFootprintLateral`; toggled by `use_footprint_clearance` |
| Cul-de-sac guard (refuse to skirt a wall) | `obstacle_deviation.cpp` `hasClearExit` + `require_clear_exit` branch in `updateLateralDeviation()` |
| Bounded straight reverse-escape (SAFETY-CRITICAL, only place FTC reverses) | `include/mowgli_nav2_plugins/ftc_reverse_escape.hpp` + `ftc_controller.cpp` `reverseEscapeOrWait()` (L1730); emitted at L921 |
| Wait-before-abort window (`obstacle_wait_timeout_s`) | `ftc_controller.cpp` `waitOrThrowForObstacle()` (L1703) — the ControllerException throw is L1717 |
| Smooth recovery after obstacle hold (continuous-clear debounce, one episode deadline, PID/carrot reset, linear + angular ramps) | `include/mowgli_nav2_plugins/ftc_obstacle_wait.hpp`, `ftc_stall.hpp` `ClampForwardToMovementRamp()`, and `ftc_controller.cpp` `holdObstacleMotion()` |
| Body-in-lethal check at the ACTUAL robot pose (SAFETY_REVIEW F-C1) | `ftc_controller.cpp` `currentBodyInLethal()` (L1663), gated at L907 |
| Legacy collision throw (deviation OFF, e.g. no-LiDAR) | `ftc_controller.cpp` `checkCollision()` (L1577) — frame caveat at L1621 |
| Oscillation detector (ring buffer, zero-crossings) | `src/oscillation_detector.cpp` `FailureDetector::detect`; wrapper `checkOscillation()` (L2296) |
| Goal-checker progress gate (end approach: path still ahead ≤ xy tolerance, OR 95 % of poses with ≤ `kRatioRuleMaxRemainingM` 1.0 m still ahead) / short-path proximity fallback / empty-plan watchdog | `src/path_progress_goal_checker.cpp` `isGoalReached()` (L208, gate at L350); path length still ahead `remainingPathLength()` (L178); new-path fingerprint + arc table in `onPath()` (L118) |
| Which topic the goal checker tracks | `path_progress_goal_checker.cpp:88` (`plan_topic`); FTC publishes `<plugin>/global_plan` at `ftc_controller.cpp:66`, `:762` |
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
| `CMakeLists.txt` | ~215 | One shared lib from 5 .cpp; exports both plugin XMLs to `nav2_core`; ROS-free gtests (incl. `test_ftc_pivot`, linked to `mowgli_interfaces::mowgli_interfaces_headers` for the corner contract) |
| `package.xml` | 40 | ament_cmake; deps nav2_core/nav2_costmap_2d/nav2_util/pluginlib/tf2*/Eigen; `<nav2_core plugin=...>` exports |
| `ftc_controller_plugin.xml` | 12 | pluginlib: `mowgli_nav2_plugins/FTCController` → `nav2_core::Controller`, `<library path="mowgli_nav2_plugins">` |
| `goal_checker_plugin.xml` | 16 | pluginlib: `mowgli_nav2_plugins/PathProgressGoalChecker` → `nav2_core::GoalChecker` |
| **`include/mowgli_nav2_plugins/`** | | |
| `ftc_controller.hpp` | 594 | `FTCController` class: FSM enum, carrot/PID/deviation/reverse/oscillation state, `struct Config` (all params + C++ defaults) |
| `ftc_stall.hpp` | 95 | Pure `StallDecision()` plus acceleration-ramp clamp used after obstacle holds |
| `ftc_obstacle_wait.hpp` | 47 | Pure continuous-followable debounce and command slew clamp for obstacle-hold recovery |
| `ftc_blade_load.hpp` | 105 | Pure `BladeLoadScale()` / `BladeLoadDecision()` — linear RPM→speed ramp, inactive/stale/degenerate gates fail OPEN, `stall_crawl_speed` floor |
| `ftc_reverse_escape.hpp` | 83 | Pure `ReverseEscapeDecide()` / `ReverseEscapeAdvance()` — opt-in, budget cap, rear-clear gate |
| `ftc_start_index.hpp` | 80 | Pure `ChooseStartIndex()` — idx 0 by default; legacy nearest snap breaks ties to the earlier index |
| `ftc_offset_lattice.hpp` | — | Whole-profile avoidance planner (pure, costmap-free): (station x lateral offset) lattice + DP, hard critics (blocked node, slope) and soft critics (un-mowed area, smoothness, side stability, return to line). Selected by `use_offset_lattice`; wired in `FTCController::planOffsetLattice()`. Tests: `test_ftc_offset_lattice.cpp` |
| `ftc_lattice_solver.hpp` | — | Costmap side of the lattice: `ResampleLatticeWindow` (lead behind → horizon ahead, clipped to the pivot leg), `LatticeSolver` (memoised node test: body polygon vs RAW lethal local cells; zone guard = body AXIS vs global ≥ 99, off the line only; span over lead + reaction; degrading solve). Impl `src/ftc_lattice_solver.cpp`. Tests: `test_ftc_lattice_solver.cpp`; offline replay: `test/lattice_replay.cpp` |
| `ftc_turn_fallback.hpp` | ~330 | Turn fallback decision + sweep checks + clearance measure (pure, costmap-only); parameter bounds and execution constants (`kTurnFallback*`). Impl `src/ftc_turn_fallback.cpp`; controller side `src/ftc_controller_turn_fallback.cpp` |
| `ftc_carrot_lead.hpp` | — | Derived longitudinal carrot lead cap (1.5 x `speed_fast` / `kp_lon`) |
| `ftc_resync.hpp` | — | Carrot resync bounded to a PATH-LENGTH window (never jumps to a neighbouring ring); the controller further bounds it to the current pivot LEG (`PivotLeg`) so a resync can neither skip a pivot nor repeat one |
| `ftc_pivot.hpp` | ~190 | Pure pivot-corner helpers (see "In-place pivots" above); detection threshold = half the planner's 15° guarantee |
| `obstacle_deviation.hpp` | 236 | `BoundaryGuard` (zone guard + zone mask) and `ObstacleDeviation` static helpers; thresholds `kLethalThreshold=253`, `kLethalOnlyThreshold=254` |
| `oscillation_detector.hpp` | 113 | `FailureDetector` — rolling (v, ω) window, mean + zero-crossing test |
| `path_progress_goal_checker.hpp` | 155 | `PathProgressGoalChecker` — progress-gated goal checker state + params (`path_arc_m_` cumulative arc table) |
| **`src/`** | | |
| `ftc_controller.cpp` | ~2.4k | Lifecycle, params + dynamic callback, newPathReceived, computeVelocityCommands, FSM, carrot, PID, collision, deviation, reverse-escape, oscillation |
| `ftc_controller_plugin.cpp` | 20 | `PLUGINLIB_EXPORT_CLASS(FTCController, nav2_core::Controller)` |
| `obstacle_deviation.cpp` | 486 | cell/body/footprint samplers, `isObstacleCell`, `hasClearExit`, `findFirstObstacleIndex`, `chooseDeviationSide`, `isPathClearWithDeviation`, `growDeviationUntilClear` |
| `oscillation_detector.cpp` | 153 | `FailureDetector` impl (normalise by v_max/ω_max, half-full buffer before deciding) |
| `path_progress_goal_checker.cpp` | 422 | `initialize` (params + plan sub), `onPath` fingerprint + arc table, `remainingPathLength`, `isGoalReached`, `getTolerances`; `PLUGINLIB_EXPORT_CLASS` at bottom |
| **`test/`** | | |
| `test_ftc_stall.cpp` | 182 | 13 cases on stall behavior and acceleration-limited obstacle restart |
| `test_ftc_obstacle_wait.cpp` | 61 | 6 cases: continuous-clear hold, blocked-scan reset, alternating scans never resume, angular restart slew |
| `test_ftc_blade_load.cpp` | 180 | 17 cases on `BladeLoadScale` / `BladeLoadDecision` (disabled/inactive/stale/degenerate fail open, ramp endpoints + midpoint, floor never raises speed, immediate recovery) |
| `test_ftc_reverse_escape.cpp` | 108 | 10 cases: opt-in default, rear-blocked never reverses, budget cap, advance arithmetic |
| `test_ftc_start_index.cpp` | 77 | 4 cases: fresh plan → 0, closed ring never resolves to its end, legacy snap, empty plan |
| `test_ftc_lattice_solver.cpp` | ~380 | 10 cases: window lead/horizon and pivot-corner clipping; clear line kept; post skirted by the body width; a wall 2.2 m ahead only cuts the horizon, inside the 1 m minimum horizon it is WEDGED; zone never tested on the line, refuses leaving into it; obstacle vs zone cause; the recorded 2026-09-22 08:08:25 WEDGED (107° corner, body front on hedge cells 0.2 m past the recorded line) stays WEDGED, and is driven without those three cells |
| `lattice_replay.cpp` | ~470 | Not a test: offline replay tool (see "Where to look") |
| `test_ftc_turn_fallback.cpp` | ~560 | 14 cases: heading sweep / window; blocked U-turn beside a hedge → planned, every leg re-checked with the bare shapes, clearances ≥ the planning margin; post on a straight → `kNotATurn`; nothing on the line → `kNoBlockage`; return swath blocked / search bound → `kNoRejoin`; pivot needs a reverse, reverse bounded by `max_reverse_m` and the rear strip, boxed in → none; refused followability; near-180° pivot needs the full turn; new obstacle on a planned leg seen by the per-tick checks; zone band refuses off-plan motion; bad input; the recorded 2026-09-22 08:08:25 kink (local lethal cells, zone runs, plan slice) → pivot −64°, 0.98 m, pivot −45°, rejoin idx 455, 1.40 m skipped |
| `test_ftc_turn_fallback_controller.cpp` | ~590 | 5 closed-loop cases on the real `FTCController` (configured `Costmap2DROS`, painted grid, kinematic robot, controlled ROS clock): U-turn into a hedge improvised and finished, never in lethal, remainder republished; same with the fallback off → WEDGED/reverse/abort as before; wall across a straight → old path; obstacle appearing in the first pivot's sweep → hold + abort; no-LiDAR flags → unreachable |
| `test_ftc_pivot.cpp` | ~215 | 10 cases: the planner's corners found exactly; turn-around arcs (r 0.10–0.30), straights and twin-less sharp corners NEVER trigger a pivot; duplicate without a turn / sub-threshold kink / corner at the plan end ignored; leg bounds; obstacle window stops at the corner; plan opening on a corner; arrival/alignment; shortest-rotation sweep probes |
| `test_obstacle_deviation.cpp` | 823 | ~50 cases on a 400×400 @0.05 m synthetic costmap: detection, side choice (left bias), grow, boundary guard, zone mask (#517), footprint model, clip/expand, `hasClearExit`, lookahead clamp |
| `test_oscillation_detector.cpp` | 195 | 11 cases: capacity, half-full gate, alternating ω detected, steady motion not |
| `test_path_progress_end_approach.cpp` | 388 | 13 cases on the checker (needs a node): field sub-path parked 0.30/0.45/0.499 m short and off the line → reached; start / > tolerance short → not; U-turn and closed ring whose end is near their start → not reached at the start or while passing near the end, reached at the end; long path unchanged; 95 % rule kept; path shorter than the tolerance completes on proximity |

## Runtime surface

### Nodes
None. Both classes are pluginlib plugins loaded by Nav2's `controller_server` (launched in `ros2/src/mowgli_bringup/launch/nav2_navigation_launch.py`, params merged by `navigation.launch.py`). Plugin instance names: `FollowCoveragePath` (controller) and `coverage_goal_checker` (goal checker), declared at `nav2_params_base.yaml:112-113`. Lifecycle follows `controller_server` (`configure/activate/deactivate/cleanup` in `ftc_controller.cpp:41-136`).

### Topics
| Topic | Type | Dir | QoS | Notes / other end |
|-------|------|-----|-----|-------------------|
| `/FollowCoveragePath/global_point` | `geometry_msgs/PoseStamped` | pub | depth 1, lifecycle | Carrot pose (map frame) every tick — viz only. The relative name `<plugin>/global_point` resolves at the root namespace, NOT under `/controller_server` (the 2026-09-22 bag) |
| `/FollowCoveragePath/global_plan` | `nav_msgs/Path` | pub | `QoS(1).transient_local()` | `global_plan_pub_`: the plan FTC drives, ONCE per `newPathReceived` (tail pose duplicated) and once per turn-fallback splice (detour included) — viz only; nothing subscribes |
| `/controller_server/FollowCoveragePath/global_plan` | `nav_msgs/Path` | pub + BT pub | `QoS(1).transient_local()` | The progress trackers' plan. Published by BT `FollowStrip` at each dispatch (`coverage_nodes.cpp`) and by FTC's `progress_plan_pub_` (`~/<plugin>/global_plan`) ONLY when a turn fallback rejoins: the remainder from the rejoin pose, stamped now. Subs: `PathProgressGoalChecker` (`plan_topic`, resets progress on the new pose count) and FollowStrip (`ControllerRejoin` → cursor jump) |
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
| `obstacle_restart_angular_acceleration` | L363 = 1.0 rad/s² | 1.0 | — (active only from an obstacle hard hold through its moving probation) |
| `stall_speed_ratio` / `stall_grace_s` / `stall_crawl_speed` | L366–368 = 0.35 / 0.6 / 0.08 | same | — (pinned by `test_ftc_stall_trio_present_in_both_variants`) |
| `blade_load_slowdown_enabled` / `blade_load_rpm_full` / `blade_load_rpm_min` / `blade_load_min_speed_ratio` / `blade_load_telemetry_max_age_s` | false / 2500 / 1800 / 0.4 / 1.0 | same | first four from template `blade_load_*` via `derive_blade_load_params` (`navigation.launch.py`, after `speed_slow`); pinned by `test_ftc_blade_load_keys_present_in_both_variants` + `test_navigation_launch_injects_ftc_blade_load` |
| `kp_lat` / `kd_lat` / `kp_ang` / `kp_ang_following` / `derivative_filter_tau` | L388 0.8 / L390 0.5 / L391 1.5 / L392 1.0 / L402 0.2 | 1.0 / 0 / 1.0 / =kp_ang / 0 | — |
| `max_goal_distance_error` | L410 = 0.50 | 1.0 | — but FLOORS `coverage_goal_checker.xy_goal_tolerance` (L890–905) |
| `max_goal_angle_error` / `goal_timeout` / `max_follow_distance` | L411 30.0 / L412 10.0 / L413 2.0 | 10.0 / 5.0 / 1.0 | — |
| `pivot_angle_tolerance_deg` | not in yaml | 10.0 (dynamic, [1, 45]) | — ends a PIVOT; tighter than PRE_ROTATE's 30° because the straight after a corner is one swath spacing long. PIVOT reuses `goal_timeout` (abort) and `obstacle_wait_timeout_s` (sweep hold) |
| `forward_only` | L420 = true | true | — |
| `snap_to_nearest_on_set_plan` / `min_lateral_deviation` | not in yaml | false / 0.30 | — |
| `check_obstacles` / `enable_obstacle_deviation` | L425 / L509 = true | true | `nav2_params_no_lidar.yaml:20-21` = false |
| `obstacle_lookahead` (poses) | L431 = 30 | 5 | L823 = `max(4, round(clamp(obstacle_detection_range_m, 0.2, 5.0) / 0.05))` |
| `use_footprint_clearance` / `obstacle_footprint_front_length_m` | L450 = false / L458 = 0.30 | false (declare) / 0.30 | — |
| `obstacle_body_half_width` / `obstacle_clearance_margin` | L477 = 0.12 / L490 = 0.05 | 0.20 / 0.0 | margin: L831 = clamp(`obstacle_clearance_margin`, 0, 0.5) |
| `require_clear_exit` / `confine_deviation_to_zone` | L468 true / (absent) | true / true | — |
| `max_lateral_deviation` / `deviation_step` / `deviation_blend_rate` | L515 1.5 / L516 0.05 / L517 0.5 | same | L811 = clamp(`max_obstacle_avoidance_distance`, 0.5, 10.0) |
| `obstacle_wait_timeout_s` / `obstacle_clear_hold_s` | L529 2.5 / L530 1.5 | 2.5 / 1.5 | L838 = clamp(`obstacle_wait_timeout_s`, 0.5, 60) |
| `obstacle_reverse_enabled` / `_max_dist_m` / `_speed_mps` | L544 true / L545 0.30 / L546 0.10 | **false** / 0.30 / 0.10 | L844–847 from `mowgli_robot.yaml` (template: true / 0.30 / 0.15), clamps dist [0,1], speed [0,0.3] |
| `oscillation_recovery` / `_v_eps` / `_omega_eps` / `_recovery_min_duration` | not in yaml | true / 0.05 / 0.05 / 5.0 | buffer len = round(duration × 10) samples |
| `turn_fallback_enabled` / `_max_reverse_m` / `_max_rejoin_arc_m` / `_min_turn_deg` / `_timeout_s` | true / 0.40 / 3.0 / 45 / 45 | same (declare clamps; dynamic updates outside [0, 1.0] / [0.2, 10] / [10, 180] / [5, 300] are rejected) | — (static nav2 params, not operator-facing). The reverse is 0 whenever `obstacle_reverse_enabled` is false; lattice planner only; pinned by `test_ftc_turn_fallback_is_configured_and_bounded` |
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
- `test/test_ftc_obstacle_wait.cpp` — an obstacle wait resumes only after a continuous followable window; any blocked scan resets the evidence; obstacle restart steering respects its slew rate without overshoot.
- `test/test_ftc_blade_load.cpp` — `BladeLoadDecision` fails open on disabled/inactive/stale/degenerate; linear ramp endpoints; floor never raises the speed; no hysteresis.
- `test/test_ftc_reverse_escape.cpp` — default is opt-in OFF; rear-blocked never reverses; budget hard cap; negative dt ignored.
- `test/test_ftc_start_index.cpp` — fresh plan starts at 0; closed ring never resolves to its last index.
- `test/test_obstacle_deviation.cpp` — detection reach vs clearance reach are separate; left bias on ties; boundary guard rejects out-of-zone offsets; zone mask ignores out-of-zone lethal but keeps in-zone; footprint model thresholds 254 while line model 253; `hasClearExit` false when obstacle fills the window.
- `test/test_oscillation_detector.cpp` — half-full gate, alternating-ω detection, capacity trimming.

Config tests in `ros2/src/mowgli_bringup/test/test_nav2_params.py` that pin this package's wiring: `test_coverage_goal_checker_is_path_progress`, `test_coverage_goal_checker_progress_threshold_is_high`, `test_followcoveragepath_uses_ftc`, `test_coverage_is_ftc_transit_is_not`, `test_ftc_stall_trio_present_in_both_variants`, `test_navigation_launch_injects_ftc_{max_lateral_deviation,clearance_margin,wait_timeout}`, `test_navigation_launch_floors_coverage_tolerance_at_ftc_park`, `test_base_ftc_clamp_admits_speed_fast`, `test_no_lidar_followcoveragepath_uses_ftc_without_obstacle_checks`, `test_base_ftc_can_command_its_own_turn_speed`.

CI: `.github/workflows/ros2-ci.yml` job `build-and-test` (L128) runs `colcon build` + `colcon test` over the whole `ros2/` workspace; job `format-check` (L404) runs `git-clang-format-18` on changed lines.

## Change coupling — "if you change X, also update Y"

- **New/renamed FTC param** → `ftc_controller.hpp` `Config` + `declareParameters()` + `onParameterChange()` (else it is not dynamic and SetNavMode/GUI writes fail) + `nav2_params_base.yaml`. Operator-facing: template `mowgli_robot.yaml` + `navigation.launch.py` injection + `paramCatalog.ts` + `gui/web/src/hooks/useSettingsManager.ts:197-198` + a `test_nav2_params.py` pin.
- **Plugin slot name `FollowCoveragePath`** appears in: `path_progress_goal_checker.cpp:88` (default `plan_topic`), `nav2_params_base.yaml:113,189,338`, `coverage_nodes.cpp:391,581`, `navigation_nodes.cpp:962`, `navigation.launch.py:754`, `test_nav2_params.py`.
- **`max_goal_distance_error`** ↔ `coverage_goal_checker.xy_goal_tolerance` floor (`navigation.launch.py:890-905`), template `coverage_xy_tolerance` (`mowgli_robot.yaml:687`), `test_*_ftc_park` tests.
- **`max_cmd_vel_speed` clamp** ↔ `navigation.launch.py:762-764` (raise-to-`mowing_speed`); `speed_fast` callback range [0, 2.0].
- **`min_speed_mps` / `max_cmd_vel_ang`** are read by `robot_config_util.derive_turn_speed` / `check_turn_geometry` (`test_robot_config_util.py:251,296`).
- **`obstacle_lookahead` is a pose count** assuming F2C 0.05 m sampling (`kF2CSamplingM`, `navigation.launch.py:822`); change coverage sampling → change the conversion.
- **Line-model threshold 253** relies on the local costmap inscribed band → by default `local_costmap.inflation_layer.inflation_radius` is floored at the live chassis circumscribed radius (`navigation.launch.py:945-960`; the shipped 0.45 × 0.60 chassis is ≈0.597 m). An enabled `local_inflation_inscribed_radius` override becomes the floor instead; the operator inflation setting (default 0.58) can raise either floor, up to the 1.50 m cap. Footprint model (254) does not; clearance there is `obstacle_clearance_margin` only.
- **Goal-checker topic has TWO publishers** (BT `FollowStrip` at each dispatch, FTC `progress_plan_pub_` at a turn-fallback rejoin) and TWO subscribers (the goal checker, FollowStrip's `ControllerRejoin`); keep QoS identical (reliable + transient_local, depth 1). FTC's republish must stay stamped `now()` and start with an exact pose of the goal path — FollowStrip ignores anything stamped before its dispatch and matches the front pose exactly. FTC reaches the topic through `"~/" + plugin_name_`, i.e. the node name `controller_server` + slot `FollowCoveragePath` — the same derivation as the goal checker's default `plan_topic` and FollowStrip's hardcoded topic: rename one, rename all.
- **`oscillation_recovery_min_duration` × 10 = buffer length** assumes `controller_frequency: 10.0` (`nav2_params_base.yaml:83`).
- **`<library path="mowgli_nav2_plugins">`** in both XMLs must equal the CMake target name.
- **`controller_server.odom_topic`** must stay a published topic (CLAUDE.md "Do NOT leave controller_server.odom_topic unset") — stall detection and the reverse budget read `velocity.linear.x`.

## Pitfalls

- `obstacle_reverse_enabled` has FOUR defaults: C++ `false` (`ftc_controller.hpp:551`), `nav2_params_base.yaml:544` `true`, template `mowgli_robot.yaml:651` `true`, launch fallback `False` (`navigation.launch.py:523`). On a real robot the launch injection from the installed/template yaml wins. Safety-critical — reversing with blades.
- `use_footprint_clearance`: `Config` initialiser says `true` (`ftc_controller.hpp:517`) but `declareParameters()` declares `false` and `nav2_params_base.yaml:450` ships `false` (field 2026-07-22: full-footprint model found no clear side). The struct initialiser is dead.
- `nav2_params_base.yaml:330-331` comment says FTC "follows U-turn arcs in reverse (forward_only=false)"; the live value is `forward_only: true` (L420). Reverse motion exists ONLY as the escape sub-state.
- `checkCollision()` samples map-frame plan poses against the odom-frame local costmap without a transform (`ftc_controller.cpp:1621-1626`). It only runs when `enable_obstacle_deviation=false`; do not turn `check_obstacles` on with deviation off.
- Never call `goal_checker->reset()` inside `computeVelocityCommands` (`ftc_controller.cpp:844-853`): `max_reached_index_` can only advance `max_idx_advance_per_call` (10) poses per call, so a per-tick reset pins progress < 95 % forever.
- `newPathReceived` duplicates the last pose and re-orients the second-to-last (`ftc_controller.cpp:744-748`); the FSM's `size() - 2` test depends on it. Plans with < 3 poses go straight to `FINISHED` — one reason FTC is not the transit controller.
- `newPathReceived` starts at idx 0 by default; re-adding a nearest-point snap skipped 46–99 % of closed headland rings on 2026-08-24 (`ftc_start_index.hpp` header). Resume trimming is `FollowStrip`'s job.
- Goal-checker "new path" fingerprint = pose-count change OR front pose moved > 2 m (`path_progress_goal_checker.cpp:148-150`). Two consecutive plans of identical length starting < 2 m apart are treated as the SAME path (progress carries over).
- Paths with ≤ `short_path_poses` (10) poses complete on xy+yaw proximity only (`path_progress_goal_checker.cpp:267`), bypassing the progress gate.
- The progress gate passes on EITHER ≥ `progress_threshold` of the poses OR the path length still ahead of the furthest monotonically-reached point ≤ `xy_goal_tolerance` (`path_progress_goal_checker.cpp:350`); xy + yaw are checked after it either way. The second rule exists because FTC parks up to `max_goal_distance_error` short and then emits zero velocity: on a sub-path shorter than ~10 m that is more than 5 % of the poses, so the pose ratio alone never passed and `progress_checker` aborted the goal after `movement_time_allowance` (field 2026-09-21, 0.6 m sub-paths, 30 s each). It is measured ALONG the path from the bounded monotonic cursor, so it stays false at the start of a looped path whose end is near its start; do not replace it with a straight-line distance. `xy_goal_tolerance` must stay ≥ FTC `max_goal_distance_error` (launch floor) or FTC parks where neither rule can pass. A path whose whole length is ≤ `xy_goal_tolerance` completes on proximity at its start. The bounded search still lets `max_reached_index_` creep up to `max_idx_advance_per_call` poses per call toward a robot that is AHEAD along the path (catch-up by design) — that is unchanged.
- `chooseDeviationSide` scans LEFT first at each radius (`obstacle_deviation.cpp:400-412`) — equal clearance always skirts left.
- `updateLateralDeviation` holds the costmap mutex (`ftc_controller.cpp:1832`) and `boundary_mutex_` for its whole body; do not call `costmap_ros_` methods that re-lock from inside.
- With `confine_deviation_to_zone=true` and no `/global_costmap/costmap` received yet, deviation is SKIPPED for the tick (fail-safe, throttled warn) — after a costmap restart FTC drives the nominal line until the latched grid arrives.
- The global costmap is used for CONFINEMENT ONLY (it rejects lateral OFFSET candidates that leave the zone). It is **not** an obstacle source and it no longer SUBTRACTS from one: `ignore_obstacles_outside_zone` (the issue-#517 zone mask) was removed 2026-09-17 after FTC drove into the same mapped tree twice on 2026-09-16 — `keepout_filter` stamps drawn obstacles lethal globally, so the mask deleted exactly the obstacles the operator had mapped. Detection = plain local-costmap threshold (`ObstacleDeviation::isObstacleCell`).
- `speed_fast` set outside [0, 2.0] via `set_parameters` is rejected (`result.successful=false`) — SetNavMode does not check the result.
- PID errors are in `base_link` (rear axle, Invariant 2), while Nav2's `robot_base_frame` is `base_footprint`; `max_goal_distance_error` is measured from base_link.
- **Pivots** happen about base_link (the REAR axle): the front of the body sweeps a disc of the chassis circumscribed radius. The planner only emits a corner where that disc fits the recorded line + soft band and clears drawn obstacles; FTC adds the LiDAR check (`pivotSweepGate`, same hold-then-abort as every obstacle stop). Never let anything but an explicit corner twin trigger PIVOT (ordinary curvature must stay FOLLOWING), never apply `lateral_deviation_` in PIVOT, and never run the deviation planners there (reverse-escape must not engage mid-rotation).
- A lateral skirt must be back on the line AT a corner: the lattice plans the corner pose as a zero-offset station (hard constraint) and releases its return debounce once the remaining path is only what the return needs; an obstacle that leaves no room to return before the corner therefore WEDGES (reverse-escape / wait / abort to the BT) where a split plan used to end the sub-path at the offset. When the carrot sits on the corner the target offset is forced to 0.
- The obstacle windows (legacy `window`, lattice stations behind AND ahead) stop at the pivot corners (`ClipWindowToCorner`, `PivotLeg`): an obstacle on the next leg is examined once the robot has pivoted onto it — the rotation itself is covered by the sweep gate.
- **Turn fallback.** It only ever REPLACES the WEDGED path at its onset (FOLLOWING, lattice infeasible, no reverse-escape / hold active, re-armed 1 m past the previous rejoin) and only for a blockage in a turn; anything it cannot do safely returns false and the WEDGED → reverse-escape → hold → abort path runs unchanged. Its motions are executed by the EXISTING PIVOT / FOLLOWING states on runtime corners spliced into `global_plan_` — do not add motion code for it. Everything it plans must be checked with what executes it (pivot: real footprint vs 254, FOLLOWING: the lattice body) or the executing checks will stop it half-way. `global_plan_` indices change at a splice (prefix dropped): anything index-based (`reverse_engaged_index_`, `pivot_corners_`) is reset there. The rejoin is signalled to the progress trackers on `progress_plan_pub_`, never by `global_plan_pub_` (a different topic — see "Where to look"). It pivots where the robot stands when the lattice wedges (~1.2 m before the blockage), so the drivable end of the approach swath is skipped too; moving the first pivot forward along the plan is a possible follow-up.
- Invariants to respect: CLAUDE.md 5 (costmap obstacles disabled in coverage — collision_monitor is the real-time guard), 8 (FTC only in the coverage slot; base + overlay YAML), "Do NOT use StoppedGoalChecker for coverage_goal_checker", "Do NOT use RPP or MPPI for coverage paths".

## Generated & vendored — do not hand-edit

- Nothing in-package is generated or vendored. Build artefacts land in `ros2/build/mowgli_nav2_plugins/` and `ros2/install/mowgli_nav2_plugins/` (git-ignored). The FTC algorithm is a port of ROS1 `ftc_local_planner` (see `package.xml` description), not a submodule.
