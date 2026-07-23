# Safety Review — 2026-07-23 (architecture + FTC + fusion_graph)

_Triggered by field reports: the robot sometimes HITS physical obstacles while mowing._
_Three parallel deep reviews: (1) FTC + obstacle-avoidance chain, (2) fusion_graph localizer, (3) end-to-end sensor→actuator safety chain. All findings verified against code on `dev` (76a6862c)._

## The systemic story (why obstacles get hit)

Coverage planning is **obstacle-blind by construction** (invariant 5/7: F2C plans from map polygon +
recorded holes only). So the reactive layers are **load-bearing safety** — and every reactive layer
has a verified hole:

1. **FTC never checks its own body** in the deployed (deviation-enabled) path — it only samples path
   poses ahead. During PRE_ROTATE pivots, stall-crawl pushes, and lateral blends, nothing in FTC
   knows the chassis is touching something.
2. **collision_monitor — the sole real-time backstop — is geometrically blind to thin obstacles**
   (LD19 455 bins/rev = 0.791°/bin → hard-stop needs width ≥ 0.11·d at `min_points:8`: a 3 cm post
   is invisible beyond 0.27 m; slowdown at `min_points:12` never fires on it) and to anything below
   the single scan plane (~0.30 m).
3. **A dead LiDAR is treated as "all clear", not "fault"** — container crash → after 5 s the monitor
   is inert, costmap frozen, robot keeps mowing blade-on indefinitely. No liveness actuator exists.
4. **fusion_graph steps `map→odom` discontinuously and both step mitigations are dead in prod**
   (slew = zero call sites since revert 4249e33c; `odom_rebase_dist_m=0`): a 2° yaw correction at
   25 m from dock = **0.87 m map-pose teleport in one TF cycle**. The coverage path and obstacle
   holes live in the map frame → the blade lands off-lane over excluded obstacles.
5. **No layer self-checks**: fusion_graph has no divergence monitor (covariance is decorative);
   a collision_monitor STOP neither cuts the blade nor informs the BT (robot parks blade-on against
   the trigger); firmware blade heartbeat is 25 s vs 200 ms for wheels.

Field hits = (2)+(1) for direct undetected contact, (4) for "mowed over an excluded obstacle",
(3)+(5) for why nothing catches it.

---

## Consolidated findings (ranked)

### CRITICAL
| ID | Area | Defect | Ref |
|----|------|--------|-----|
| A-C1 | collision_monitor | Thin obstacles invisible to BOTH polygons (`min_points` 8/12 vs 0.79°/bin geometry) | `nav2_params_lidar.yaml:172,202` |
| A-C2 | scan pipeline | LiDAR death undetected → mows blind indefinitely (absent source = "clear"; no liveness watchdog; no `expected_update_rate`) | `nav2_params_base.yaml:1025` |
| F-C1 | FTC | Deviation-enabled path never collision-checks the robot's ACTUAL current footprint (`checkCollision` only runs when deviation disabled) | `ftc_controller.cpp:844-900,1534` |
| G-C1 | fusion_graph | `map→odom` steps; slew limiter is dead code (no call sites), `odom_rebase_dist_m=0` → lever-arm-amplified sub-metre map teleports | `fusion_graph_node_publish.cpp:368`, `fusion_graph_node.cpp:150` |
| A-C3 | architecture | Coverage planner obstacle-blind by design → reactive layers are load-bearing (context for all of the above) | invariant 5/7 |
| A-C4 | no-LiDAR variant | ZERO forward detection; "collision_monitor + firmware remain the safety net" comment is FALSE (monitor pass-through, firmware has no forward sensor) | `nav2_params_no_lidar.yaml:15-17,56-80` |

### HIGH
| ID | Area | Defect | Ref |
|----|------|--------|-----|
| A-H1 | BT / blade | collision_monitor STOP neither halts BT nor cuts blade — robot idles blade-on against the trigger | `main_tree.xml:32-60` |
| A-H2 | firmware | Blade heartbeat 25 s vs wheels 200 ms — BT crash leaves blade energized 25 s on a stopped mower | `cpp_main.cpp:848,853` |
| A-H3 | scan filter | Post-undock: collision_monitor blind within 0.70 m for 5 s (radial blank on /scan_collision) | `costmap_scan_filter_node.cpp:393-404` |
| F-H1 | FTC | collision_monitor is sole real-time guard (consequence of F-C1) with sub-plane + thin + pure-rotation gaps | `nav2_params_lidar.yaml:157-205` |
| F-H2 | FTC | `reverse_escape_active_` leaks across `updateLateralDeviation` early-returns → unbounded, un-rechecked reverse. INERT today (`obstacle_reverse_enabled=false`) — MUST fix before enabling | `ftc_controller.cpp:853,1787,1822,1846` |
| F-H3 | FTC | Stall-crawl keeps pushing at 0.08 m/s into an undetected blocker for up to 30 s (only the Nav2 progress checker ends it) | `ftc_controller.cpp:1404-1413` |
| G-H1 | fusion_graph | No divergence self-check; published covariance consumed by nothing; no safe-stop on bad pose | `fusion_graph_node_publish.cpp:177` |
| G-H2 | fusion_graph | Keyframe map (default ON) engages during RTK-Float and can pin a wrong absolute position (xy guard 0.30 m too loose) | `_timer.cpp:184-276` |
| G-H3 | fusion_graph | Wrong-fix gate has no cumulative bound — slow wrong-fix walk accepted indefinitely | `rtk_wrongfix_gate.hpp:34-43` |
| G-H4 | fusion_graph | Two unguarded `isam.update()` (rebase thread + dock RigidTransformAll) → GTSAM throw = `std::terminate` = dead localizer | `graph_manager_rebase.cpp:176,193,307` |

### MEDIUM (abbreviated — see agent reports)
- F-M1: line-model reach (0.12+inscribed 0.10) silently coupled to costmap internals, 2 cm margin, no assert.
- F-M2: lateral blend traverses UNCHECKED intermediate offsets (diagonal into the obstacle edge).
- F-M3: `hasClearExit` evaluated only at avoidance entry, never re-checked while avoiding.
- F-M4: reverse-escape rear-clear trusts a costmap that may not carry the rear obstacle (occlusion/sub-plane).
- G-M1: gyro-only yaw during long Float (no live bias correction while moving; `use_imu_preint:false`).
- G-M2: stationary-window corrections snap in as one step on motion resume.
- G-M3: datum self-seed from first fix if datum unset.
- A-M1: docking/undock cmd_vel bypasses collision_monitor (twist_mux prio15 lane).
- A-M2: ground filter can strip a low+thin obstacle on a slope.
- A-M3: `setBladeEnabled(false)` fire-and-forget — silently no-ops if service down.
- A-M4: twist_mux nav-lane timeout 2.0 s defeats the firmware 200 ms watchdog (≤0.4 m coasting on stale cmd after monitor death).

---

## Action plan (ranked by collision-risk-reduction ÷ effort)

### P0 — config-only, ship immediately
1. **A-C1**: `FootprintApproach.min_points: 8→3`; add a narrow near-field `PolygonStop` (`min_points:2`)
   covering chassis-front + blade sweep. (`nav2_params_lidar.yaml`)
2. **G-C1 (half)**: `odom_rebase_dist_m: 0 → ~6.0` — bounds the lever arm so yaw corrections can't
   amplify into sub-metre teleports. (config/template + verify launch pass-through)
3. **G-H2**: tighten `kf_match_max_divergence_xy_m: 0.30 → 0.10`.
4. **A-M4**: twist_mux nav-lane timeout `2.0 → 0.6`.

### P1 — small code changes, high value
5. **F-C1**: current-pose oriented-footprint lethal check every tick in the deviation-enabled path;
   refuse motion when the body is already in a lethal cell.
6. **A-C2**: scan-liveness watchdog → zero on `/cmd_vel_emergency` (prio 100) + blade-off when /scan
   age > ~0.5 s during a mow; `expected_update_rate` on obstacle sources.
7. **A-H1**: BT consumes `collision_monitor_state` → sustained STOP cuts blade + triggers detour/abort.
8. **F-H3**: in-FTC no-progress timer bounding the stall-crawl (seconds, not 30 s).
9. **G-H4**: wrap the two unguarded `isam.update()` in the existing try/catch→Reset pattern.
10. **F-H2**: clear `reverse_escape_active_` on every `updateLateralDeviation` early-return
    (prerequisite before `obstacle_reverse_enabled` is EVER turned on).
11. **A-M3**: retry + ERROR log in `setBladeEnabled` on unreachable service.

### P2 — firmware (coordinate; firmware is sole blade authority)
12. **A-H2**: blade cut on the same short watchdog as wheels (≤1 s), or whenever wheel hard_stop asserts.

### P3 — design work (dedicated sessions)
13. **G-C1 (full)**: re-wire `SlewPublishedAnchor` (or delete the dead code+params+test — currently lies
    to readers) — smooth residual map→odom corrections.
14. **G-H1/G-H3**: divergence monitor (graph vs wheel-DR/GPS, rolling window) → health flag consumed by
    BT → blade-off safe-stop on sustained divergence.
15. **A-H3/A-M1**: near-dock blind spots (sector blank instead of radial; route docking cmd_vel
    through collision_monitor).
16. **A-C4**: no-LiDAR variant — gate blade-on mowing to LiDAR configs or require a front bumper;
    fix the false safety comment.
17. **F-M2**: verify clearance along the blended diagonal (or slow forward speed while blending).

### Hardware note (unchanged from spec)
Sub-scan-plane obstacles (< ~0.30 m) are a **hard sensing limit** — no software fix; document it and
consider a bumper/cliff sensor long-term.
