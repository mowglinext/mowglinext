# Spec — Obstacle wedge & erratic recovery (dedicated session)

_Written 2026-07-22 after a long field-debug session. Robot deployed image `755d58eb` (dev HEAD `876c1455`)._

## The single root cause (not three separate bugs)

The robot **boxes itself in** next to an obstacle (a "wedge"). Once wedged, **any**
`NavigateToPose` needed to get out — a coverage segment transit, the HOME transit, or
the dock-staging transit — fails with **"collision ahead"**, and Nav2's **recovery
subtree (Spin → BackUp → Wait)** runs. Those recoveries publish **no planned path** and
drive the robot in circles / erratically until it either luckily escapes or the operator
e-stops it.

Operator-visible symptoms, all the SAME root:
- Coverage frozen (~44%) in a resume→abort loop; the robot never self-recovers.
- On the way to dock: "after the approach phase it spins in circles / drives anywhere,
  **no transit trajectory on the map**" → that is the Nav2 **Spin/BackUp recovery**, not a
  planned transit.
- Docking itself is fine (works reliably); the failure is UPSTREAM — the wedge/recovery.

## Evidence gathered (live robot, image 755d58eb)

- Wedge: robot at map `(-6.3, 7.8)` beside a **real 90-cell lethal cluster** (global costmap;
  not phantom). FTC: `Failed to make progress` → **detour NOT firing** (`abort … not
  obstacle-related — no lethal cell ahead — not detouring`) → flip-flop
  `lateral deviation needed > max_lateral_deviation — holding` ↔ `obstacle cleared, resuming`
  at 10 Hz, then abort. Coverage stuck ~4 min.
- Nav2 recovery cascade (bt_navigator/behavior_server):
  `Begin navigating (-6.37,7.79)→(5.36,3.90)` → `Running backup → Collision Ahead - Exiting
  DriveOnHeading` → `Running spin → Collision Ahead / Exceeded time allowance` → `Running
  wait` → `goalCompleted error 104`. Retries; eventually `Goal succeeded`.
- Docking: `Successful navigation to staging pose` + `Successful initial dock detection`,
  then an **external emergency-stop** (operator/GUI/button) → `EmergencyGuard`
  (`main_tree.xml:32`) halts the tree → `DockRobot: canceling active goal` → throttled
  `Behavior tree root returned FAILURE`. **Not a docking/BT/config bug** (verified).
- **Not digging** (slip_veto/stationary_hand_push flat, residual ~0.001, wheel_sigma nominal),
  **no surge** (vx ≤ 0.20), **no weave**, fusion healthy mid-field (`cov_yawyaw` ~1.3°).
- `FootprintApproach.max_points is not initialized` warning = **benign** (deprecated nav2 alias
  of `min_points`, honored; the line comes from an external Foxglove param-panel watch). Do
  NOT add `max_points` — nav2 would then emit its own deprecation warning and it shadows
  `min_points`.

## The core tension (why this is design work, not a patch)

Two FTC skirt models, both wrong:
- **Footprint clearance** (`use_footprint_clearance=true`, added `b35905d2`): TOO CONSERVATIVE
  — found no clear side even at `max_lateral_deviation=3.0 m` for skirtable obstacles →
  flip-flop hold/resume → aborted whole segments. Reverted to default **OFF** (`beab3141`).
- **Half-width line** (current default, `obstacle_body_half_width=0.12`): TOO AGGRESSIVE —
  skirts into pockets the footprint model would have refused → the robot **boxes itself in**
  → cannot recover.

Neither prevents the wedge. The fix needs a middle ground PLUS a sane recovery.

## The fix — two parts

### Part A (highest leverage): stop the robot skirting into un-recoverable pockets
FTC obstacle deviation. Evaluate:
- A **less-conservative footprint** clearance: probe the FRONT of the footprint + a realistic
  body half-width, NOT the full 0.60 m length (the middle ground between the two models).
- A **committed-skirt / exit check**: before deviating, verify the deviated corridor has a
  clear EXIT within the lookahead — never skirt into a cul-de-sac.
- Reconsider `max_lateral_deviation` (deep skirts reach traps) — but only WITH Part B, else it
  re-opens the abort-whole-segment problem.
- Files: `ros2/src/mowgli_nav2_plugins/src/obstacle_deviation.cpp`
  (`footprintBlocked`/`chooseDeviationSide`/`decideDetour` sampling),
  `.../src/ftc_controller.cpp` (`updateLateralDeviation`),
  `ros2/src/mowgli_bringup/config/nav2_params_base.yaml` (`FollowCoveragePath`).

### Part B (safety net): widen the detour gate + a clean wedge recovery
- **Detour gate** (`a87ac67d`) currently fires only when a lethal cell is DEAD-AHEAD on the
  nominal path (`decideDetour.obstacle_confirmed`). It must ALSO fire on
  **"stalled beside an obstacle"** — FTC aborts on "make progress" AND lethal cells are near
  the robot's ACTUAL pose (not just dead-ahead). Files:
  `ros2/src/mowgli_behavior/include/mowgli_behavior/detour_resume.hpp` (`decideDetour`),
  `.../src/coverage_nodes.cpp` (`tryStartDetour`).
- **Boxed-in recovery**: when Nav2 recoveries all fail "collision ahead", a blind Spin/BackUp
  just twitches. Need a **bounded reverse-out** (back the way it came). Two levers:
  - The FTC `obstacle_reverse_enabled` maneuver already exists (opt-in, default OFF, UNTESTED,
    straight reverse, hard-capped 0.30 m, rear-footprint checked) — enabling + validating it may
    cover the coverage-wedge case. `ros2/src/mowgli_nav2_plugins/include/.../ftc_reverse_escape.hpp`.
  - **Tame the Nav2 recovery subtree** so transit failures clear costmaps + wait + retry rather
    than Spin/BackUp erratically. Find the mowgli custom `navigate_to_pose` BT XML (under
    `mowgli_behavior` / `mowgli_bringup` — the subtree with `Spin`/`BackUp`) and the
    `behavior_server` params (spin/backup enable, `time_allowance`).

## Deployed state on the robot NOW (`755d58eb`)
- `use_footprint_clearance = false` (aggressive half-width skirt — drives into pockets).
- Detour-and-continue = ON, gate too narrow (lethal-dead-ahead only).
- `obstacle_reverse_enabled = false` (opt-in reverse-escape, untested).
- Dig fixes (FTC stall-cap + firmware anti-dig ratio) = working. Surge fix, keyframe yaw
  xy-only = working. Dashboard 1 Hz republish (`876c1455`) = deploying.
- Live param drift to reset: `max_lateral_deviation` was set to 3.5 (default 3.0);
  `obstacle_clearance_margin` to 0.02 (default 0.05) — both revert on node restart.

## Also owed (field / hardware, not this code effort)
- **IMU at-rest calibration** (wheels immobile): the calibration keeps aborting "wheels moving"
  → `gyro_bias ~3°/s` (firmware yaw loop). Do this with the robot parked.
- **Dock localization**: RTK wrong-fix under the metal canopy → yaw error at the dock (`gps_dock_detection`
  157–169° map→odom jumps, `gps_quality=1%`). Robust fix is dual-antenna moving-base RTK
  (see the `map-odom-slew` memory, "Level 3").

## Suggested order for the dedicated session
1. Reproduce the wedge; capture the local+global costmap + FTC deviation decisions AT the wedge
   (scratchpad `fgcostmap.py`, SSH `docker logs mowgli-ros2`).
2. Do **Part A first** (prevent the wedge) — if the robot never boxes itself in, the recovery
   problem largely disappears.
3. Do **Part B** as the safety net for genuinely-unavoidable wedges.
4. Supervised field test after each change. Blade-safety is paramount (firmware is sole authority).
