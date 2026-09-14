# Behavior Trees

MowgliNext uses BehaviorTree.CPP v4 for reactive, composable robot control.

## Overview

- **Tick rate:** 10 Hz (`tick_rate` in `mowgli_robot.yaml`)
- **Root:** ReactiveSequence — every child is re-ticked from the top on each tick
- **Reactive guards:** Emergency, sensor safety, boundary violation, localization quality, GPS mode, Nav2 resume — checked every tick before main logic
- **Tree file:** `ros2/src/mowgli_behavior/trees/main_tree.xml`
- **Node registry:** `ros2/src/mowgli_behavior/src/register_nodes.cpp`

## High-Level States (HighLevelStatus.msg)

| Value | Constant | Description |
|-------|----------|-------------|
| 0 | `HIGH_LEVEL_STATE_NULL` | Emergency or transitional |
| 1 | `HIGH_LEVEL_STATE_IDLE` | Idle, docked, or returning home |
| 2 | `HIGH_LEVEL_STATE_AUTONOMOUS` | Autonomous mowing (undocking, transit, mowing, recovering) |
| 3 | `HIGH_LEVEL_STATE_RECORDING` | Area recording in progress |
| 4 | `HIGH_LEVEL_STATE_MANUAL_MOWING` | Manual mowing via teleop |

## Commands (HighLevelControl.srv)

| Value | Constant | Description |
|-------|----------|-------------|
| 1 | `COMMAND_START` | Begin autonomous mowing |
| 2 | `COMMAND_HOME` | Return to dock |
| 3 | `COMMAND_RECORD_AREA` (= `COMMAND_S1`) | Start area boundary recording |
| 4 | `COMMAND_S2` | "Mow next area" — normalised to `COMMAND_START` in the service handler (mowing always resumes from the next un-mowed area, so there is no separate branch) |
| 5 | `COMMAND_RECORD_FINISH` | Finish recording, simplify and save polygon |
| 6 | `COMMAND_RECORD_CANCEL` | Cancel recording, discard trajectory |
| 7 | `COMMAND_MANUAL_MOW` | Enter manual mowing mode |
| 8 | `COMMAND_STOP` | Stop in place / hold — motion halted, blade off, stays put (does **not** drive to the dock; that is `COMMAND_HOME`). Backs the GUI's "Stop Manual" and "Pause" buttons |
| 254 | `COMMAND_RESET_EMERGENCY` | Reset latched emergency |
| 255 | `COMMAND_DELETE_MAPS` | Delete all maps |

The service is `/behavior_tree_node/high_level_control`. Two sibling services complete the surface: `~/start_in_area` ("mow this area only") and `~/clear_coverage_resume` ("start fresh" — discard the persisted resume cursor).

## Tree Structure

```
Root (ReactiveSequence) — re-evaluates all children every tick
│
├── EmergencyGuard
│   ├── Inverter(IsEmergency) → continue if safe
│   └── EmergencyHandler
│       ├── SetMowerEnabled(false), StopMoving()
│       ├── PublishHighLevelStatus(EMERGENCY)
│       └── AutoResetOrWait
│           ├── IsCharging → ResetEmergency (auto-reset on dock)
│           └── WaitForDuration(1s) (retry if not on dock)
│
├── SensorSafetyGuard (exempt while charging/docking, and under cmd 7/3/5/6)
│   ├── Inverter(IsScanStale OR IsCollisionStopSustained) → continue if healthy
│   └── SensorFaultHandler: blade off, stop, wait
│
├── BoundaryGuard (exempt while charging, under cmd 7/3/5/6/2,
│   │              and during the blade-off dock transit of a mow run)
│   ├── Inverter(IsBoundaryViolation) → continue if inside
│   ├── LethalBoundaryHandler (IsLethalBoundaryViolation) → stop, latch
│   │   BOUNDARY_EMERGENCY_STOP, hold for operator
│   └── SoftBoundaryHandler → stop, NavigateInsideBoundary (2 attempts,
│       via map_server/get_recovery_point), else escalate to lethal
│
├── LocalizationGuard (same exemptions as BoundaryGuard)
│   ├── Inverter(IsLocalizationDegraded) → continue if position is trustworthy
│   └── Hold in place (WAITING_FOR_RTK): blade off, wheels stopped
│
├── GPSModeSelector
│   ├── IsGPSFixed → SetNavMode(precise)
│   └── SetNavMode(degraded)
│
├── Nav2ResumeGuard — RESUME the Nav2 lifecycle stack before any motion branch
│   (no-op unless idle_nav2_suspend is on; skipped while idle on the dock)
│
└── MainLogic (Fallback — priority order)
    ├── CriticalBatteryDock (IsBatteryLow at battery_critical_pct, default 10 %)
    │   → save, dock, hold on the charger to battery_full_pct (default 95 %),
    │     then undock and AUTO-CONTINUE the run (no ClearCommand). Only a dead
    │     charger (IsChargingProgressing) ends the session.
    │
    ├── MowingSequence (COMMAND_START = 1)
    │   ├── Nav2ReadyPoll, then UndockOrSkip:
    │   │     ├── not on dock → SeedYawFromMotion (1 m forward drive) + clear
    │   │     └── on dock → PreFlightCheck, RecordUndockStart, BackUp,
    │   │         WaitForGpsFix(min_fix_type=4), CalibrateHeadingFromUndock
    │   ├── Multi-area coverage loop (reactive rain/battery guards):
    │   │   AreaLoop (Repeat 100):
    │   │     ├── GetNextUnmowedArea → current_area_index
    │   │     ├── PlanCoverageArea(area_index)
    │   │     │   — calls map_server/get_mowing_area (outer ring + obstacle
    │   │     │     holes), then mowgli_coverage's plan_coverage action
    │   │     │     (F2C v3). ONE plan/area/session.
    │   │     ├── TransitToStrip — blade-off Nav2 transit to the path start
    │   │     ├── RetryUntilSuccessful(5):
    │   │     │   ├── FollowStrip — drives each drivable sub-path as ONE
    │   │     │   │   FollowCoveragePath goal (FTC). On abort, FollowStrip
    │   │     │   │   trims the path at its resume cursor before re-dispatch.
    │   │     │   ├── StartPoseBlockedRetry: IsCoverageStartBlocked →
    │   │     │   │   EscapeStartBlocked + ClearCostmap + Wait
    │   │     │   ├── StuckBackoff: IsObstacleStuck → BackUp(0.40m)+ClearCostmap
    │   │     │   └── DynamicObstacleSkip: WasRecentlyInCollisionStop →
    │   │     │       ClearCostmap + Wait
    │   │     └── AreaUnreachable (PlanFAILURE OR retries exhausted) → advance
    │   └── CoverageEnded → IsCoverageComplete ? MOWING_COMPLETE
    │       : COVERAGE_FAILED_DOCKING; either way blade off, save, dock
    │
    ├── HomeSequence (COMMAND_HOME = 2) → save, dock
    │
    ├── RecordingSequence (COMMAND_RECORD_AREA = 3; guard accepts 3/5/6)
    │   ├── RecordArea (samples at area_record_rate_hz, live preview at 2 Hz)
    │   ├── Finish (cmd 5) → Douglas-Peucker simplify → save polygon
    │   └── Cancel (cmd 6) → discard
    │
    ├── ManualMowingSequence (COMMAND_MANUAL_MOW = 7)
    │   ├── Teleop via /cmd_vel_teleop (twist_mux priority)
    │   ├── Blade enabled by the BT *after* the MANUAL_MOWING status is
    │   │   published (the firmware zeroes the blade while its mirrored HL
    │   │   mode is still IDLE/NULL)
    │   └── Collision_monitor, GPS and the localizer all remain active
    │
    ├── StopHoldSequence (COMMAND_STOP = 8) → blade off, stop, stay put
    │   (unlike IdleSequence it does NOT pause the Nav2 lifecycle)
    │
    └── IdleSequence (default) → disable blade, stop, PAUSE Nav2 if
        idle_nav2_suspend, wait
```

## Node Types

### Condition Nodes
- `IsEmergency` — checks emergency stop state (>2 s of `/hardware_bridge/emergency` silence counts as emergency, by design)
- `IsCharging` — dock charging state
- `IsBatteryLow` — battery **percent** below `threshold` (default 22 %), plus an optional redundant `voltage_threshold` trip on the filtered pack voltage (0 = disabled)
- `IsBatteryAbove` — checks battery percent above threshold (for charge-to-95%)
- `NeedsDocking` — battery **percent** at or below threshold (default 20 %)
- `IsRainDetected` — rain sensor active
- `IsNewRain` — new rain onset (not if raining at mow start), debounced by `rain_debounce_sec`; returns FAILURE outright when `rain_mode` is 0
- `IsRainModeAtLeast` — configured `rain_mode` is at least the requested level (0 = off, 1 = pause-in-place, 2 = dock-and-pause)
- `IsGPSFixed` — GPS RTK fix quality check
- `IsCommand` — matches current high-level command from GUI
- `IsBoundaryViolation` — robot outside mowing area boundary
- `IsLethalBoundaryViolation` — outside every area by more than the lethal margin (escalates BoundaryGuard to a latched stop)
- `IsLocalizationDegraded` — absolute position no longer trustworthy. Keys on `/gps/status` GNSS quality (accuracy / fix-lost / stale latches), **not** on the fused covariance, with a σ_xy backstop; `fusion_graph` deliberately inflates σ on pivots, so the covariance alone would livelock mowing at 0 %
- `IsDocking` — a dock transit is in flight (used to exempt the blade-off dock leg from the boundary/localization guards)
- `IsCoverageComplete` — the last `GetNextUnmowedArea` ended because every area is genuinely mowed (vs a transient failure)
- `IsCoverageStartBlocked` — the last coverage transit was refused with `START_OCCUPIED`. Also the ONLY place that arms `EscapeStartBlocked`'s escape token
- `IsObstacleStuck` — collision_monitor has held STOP for `min_duration_sec`, with an attempt cap and cooldown
- `WasRecentlyInCollisionStop` — a collision STOP ended within `max_age_sec`
- `IsCollisionStopSustained` — STOP asserted continuously for `min_duration_sec`, freshness-gated by `max_state_age_sec`
- `IsScanStale` — `/scan_collision` has not updated within `max_age_sec`
- `PreFlightCheck` — pre-undock gate: no emergency, `min_battery`, `min_gps_fix_type`, a resolvable `map → base_footprint` TF, at least one mowing area defined, and a compatible firmware protocol version. The dock gate is `min_gps_fix_type=2` on purpose — the canopy pins the receiver at RTK Float there, so RTK Fixed is only enforced *after* undock by `WaitForGpsFix(min_fix_type=4)`
- `Nav2Active` — `/lifecycle_manager_navigation/is_active` reports the stack up
- `IsResumeUndockAllowed` — tracks resume-undock attempts (max_attempts)
- `IsChargingProgressing` — charger active and battery increasing
- `ReplanNeeded` — coverage replanning required (registered, but not referenced by the current `main_tree.xml`)

### Action Nodes
- `NavigateToPose` — Nav2 navigate_to_pose async action
- `NavigateInsideBoundary` — BoundaryGuard's soft recovery: ask `map_server_node/get_recovery_point` for the nearest in-area pose, disable the keepout filter, clear the costmaps, navigate there, then re-enable the filter (BackUp fallback if the plan fails)
- `DockRobot` — opennav_docking `/dock_robot`. `UndockRobot` is registered but **not** used by the tree: undocking is a Nav2 `BackUp` behavior (opennav's `isDocked()` is unreliable with GPS drift near the dock)
- `SetMowerEnabled` — fire-and-forget blade commands via `/hardware_bridge/mower_control`
- `StopMoving` — streams zero `TwistStamped` on `/cmd_vel_emergency` for `duration_sec`
- `BackUp` — drives robot backward via Nav2 `/backup` (configurable distance/speed); also the undock primitive
- `EscapeStartBlocked` — bounded open-loop nudge opposite the last commanded motion, on `/cmd_vel_nav`, to get off a `START_OCCUPIED` start pose. Only fires behind `IsCoverageStartBlocked`'s arming token, blade verified off, and is hard-capped in speed/distance/time
- `ClearCostmap` — clears both the Nav2 global and local costmaps
- `SetNav2Lifecycle` — PAUSE/RESUME the Nav2 lifecycle stack (only acts on a real transition; a no-op unless `idle_nav2_suspend` is on)
- `PublishHighLevelStatus` — publishes state + state_name to HighLevelStatus topic
- `SaveObstacles` — calls `/obstacle_tracker/save_obstacles`. Nothing in the repo currently serves it, so in practice the node logs "service unavailable, skipping" and returns SUCCESS
- `SetNavMode` — switches between precise/degraded navigation (pushes `FollowPath.desired_linear_vel` and `FollowCoveragePath.speed_fast` to `controller_server`)
- `ClearCommand` — clears pending high-level command
- `EndSession` — resets per-session state (yaw seed flag, undock bookkeeping, skipped-swath counter) and deletes the persisted resume file. Only at confirmed session boundaries
- `IncrementSkippedSwaths` — bumps the skipped-swath counter (registered, not currently used by the tree)
- `WaitForDuration` — timed wait
- `WaitForGpsFix` — waits up to `timeout_sec` for `min_fix_type` (4 = RTK Fixed), then proceeds anyway rather than freezing
- `RecordUndockStart` / `CalibrateHeadingFromUndock` — heading calibration from the GPS displacement measured across the undock BackUp; the refined yaw is seeded to the localizer on `/fusion_graph_node/set_pose`
- `SeedYawFromMotion` — off-dock start: drives 1 m forward on `/cmd_vel_teleop` and derives yaw from the GPS track
- `WasRainingAtStart` — records rain state at mow start
- `RecordResumeUndockFailure` — tracks resume failures
- `ResetEmergency` — calls `/hardware_bridge/emergency_stop` with emergency=0 (firmware decides whether to clear)

### Coverage Nodes (F2C-driven, one plan per area per session)
- `GetNextUnmowedArea` — outer loop. Picks the next area whose `mow_progress` layer still has un-mowed cells; writes `current_area_index` to the blackboard. Skips navigation-only areas, and retires an area after a bounded number of no-progress passes. FAILURE when all areas done.
- `PlanCoverageArea` — takes a single `area_index` port. Calls `/map_server_node/get_mowing_area` (area outer ring + obstacle holes), then sends a `plan_coverage` action goal (`mowgli_interfaces/action/PlanCoverage`) to `mowgli_coverage`, which wraps Fields2Cover v3. The goal carries `outer_boundary`, `obstacles` and `mow_angle_deg` — all other coverage geometry (`operation_width`, headland width, insets) lives in the coverage server's own parameters, injected at launch from `mowgli_robot.yaml`. The result carries an explicit `segments` list (headland rings + serpentine swaths, for the GUI and resume bookkeeping) **and** the hole-free continuous `drivable_subpaths` that execution actually drives. ONE invocation per area per session.
- `FollowStrip` — drives each entry of `drivable_subpaths` as ONE `FollowCoveragePath` goal (FTCController tracks it end-to-end via a pose cursor, not per-segment dispatch), bridging the gap between consecutive sub-paths with a blade-off Nav2 transit. On abort, `RetryUntilSuccessful(5)` re-ticks the node, which **trims the path at its own resume cursor before re-dispatch** — FTC's `setPlan` always starts at index 0 (the old nearest-pose snap is behind `snap_to_nearest_on_set_plan`, off by default, because on closed headland rings start == end and it skipped 46–99 % of the ring). Recovery branches (`IsCoverageStartBlocked`, `IsObstacleStuck`, `WasRecentlyInCollisionStop`) insert an escape nudge / `BackUp` / `ClearCostmap` between retries.
- `TransitToStrip` — blade-off `NavigateToPose` transit to the coverage path start, so the Nav2 global planner routes *around* an obstacle instead of letting FTC abort against it. Soft: if the start is genuinely unreachable it falls through and `FollowStrip` tries anyway.
- `DetourAroundObstacle` — registered, but not referenced by the current `main_tree.xml`; in-sub-path detours are handled inside `FollowStrip`.

### Recording Nodes
- `RecordArea` — records robot trajectory while user drives boundary, Douglas-Peucker simplification, saves polygon via `/map_server_node/add_area`. Publishes live trajectory preview on `~/recording_trajectory`. Listens for finish (cmd 5) or cancel (cmd 6).
  - Ports: `simplification_tolerance` (0.05 m), `min_vertices` (3), `min_area` (1.0 m^2), `record_rate_hz` (10.0), `is_exclusion_zone` (false). The tree feeds the first and fourth from `mowgli_robot.yaml` (`area_simplification_tolerance` / `area_record_rate_hz`); `min_vertices` and `min_area` stay hardcoded as degenerate-polygon guards. Samples are additionally gated to a 0.05 m minimum spacing, and the preview republishes on its own 2 Hz clock.

## Adding a New BT Node

1. Define the node class in `ros2/src/mowgli_behavior/include/`
2. Implement in `ros2/src/mowgli_behavior/src/`
3. Register in `ros2/src/mowgli_behavior/src/register_nodes.cpp`
4. Use in `main_tree.xml`
5. Add a gtest target in `ros2/src/mowgli_behavior/CMakeLists.txt` (each target links only the `.cpp` files it needs — follow the existing ones)

Two structural rules the tests enforce, so know them before you edit the XML:

- Every **blocking guard** handler must terminate with `<AlwaysFailure/>`. The Root is a `ReactiveSequence`, so a handler that returns SUCCESS lets `MainLogic` run one tick per cycle — the livelock this rule exists to prevent. `test_guard_fallthrough.cpp` asserts it against the real tree.
- A new `state_name` also has to be added to the GUI's state maps (`gui/web/src/components/dashboard/constants.ts`, `utils.tsx`, `BTStateGraph.tsx`) and to this page.

For a file-by-file map of the package — every registered node, its ports, the topics/services/actions it touches, and the known pitfalls — see `docs/claude/codemaps/mowgli_behavior.md`.

## Configuration

Behavior tree parameters live in the template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml`, under the single `mowgli:` namespace that the launch files forward to every node:

```yaml
mowgli:
  ros__parameters:
    tick_rate: 10.0
    battery_low_percent: 20.0      # start docking
    battery_critical_percent: 10.0 # emergency dock
    battery_full_percent: 95.0     # resume mowing above this %
    rain_mode: 2                   # 0 = off, 1 = pause-in-place, 2 = dock-and-pause
    rain_delay_minutes: 30.0
    rain_debounce_sec: 10.0
```

The *installed* `mowgli_robot.yaml` is sparse — it holds only install-time choices, calibration outputs and genuine overrides, and is deep-merged **over** this template at launch. Set a value there to override it; delete the line to fall back to the template default.

The behavior tree reads its parameters once at startup — restart the node to apply a change.

See [Configuration](Configuration) for the full parameter reference.
