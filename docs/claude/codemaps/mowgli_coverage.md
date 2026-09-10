# Codemap: mowgli_coverage

> `ros2/src/mowgli_coverage` is the Fields2Cover **v3** coverage planner: one lifecycle node
> (`coverage_server`) serving the `plan_coverage` action (`mowgli_interfaces/action/PlanCoverage`).
> It owns the whole plan pipeline — ring sanitization → headland rings → serpentine swaths
> (`planBoustrophedon`) → forward turn-around connectors, corner fillets and hole-free sub-path
> splitting (`buildContinuousSubPaths`) — and the in-bounds/hole verification of the result. It does
> NOT own area polygons (`mowgli_map`), does not drive (`mowgli_behavior` FollowStrip + FTC), and
> uses no TF. Index generated 2026-09-03 at f21729e9; regenerate when files are added/removed.
> Loaded on demand from `ros2/CLAUDE.md`.

## Where to look

| Task | Start here |
|------|------------|
| Add / rename a `coverage_server` parameter | `ros2/src/mowgli_coverage/src/coverage_server.cpp:53-105` (`on_configure`, idempotent `declare_double`/`declare_int` lambdas), then the coupling list below (yaml + launch injection + GUI) |
| Understand how a goal becomes a result | `ros2/src/mowgli_coverage/src/coverage_server.cpp:419-842` (`CoverageServer::planCoverage`) |
| Polygon ingestion: closing, dedup, obstacle margin | `coverage_server.cpp:201-231` (`buildCellFromGoal`) → `ros2/src/mowgli_coverage/src/coverage_planning.cpp:2010-2088` (`dedupClosedRing`), `:1872-1933` (`bufferRingOutward`), `:1954-2006` (`makeRingValid`, OGR `Buffer(0)`) |
| Degenerate recorded ring (mm slivers, 1.2 mm closure seam) planned only partially | `coverage_planning.cpp:1945-1946` (`kRingDedupTolM` 1 cm, `kRingSpikeTolM` 5 mm) + test `test_coverage_planning.cpp:2024` (`DegenerateRecordedRingSanitizedToFullCoverage`) |
| Headland ring count (`num_headland_passes` three-way sentinel) | `coverage_planning.cpp:810-820` (`n_rings`), contract in `include/mowgli_coverage/coverage_planning.hpp:135-149` |
| Where the outermost ring sits vs the recorded line (`chassis_safety_inset`) | `coverage_planning.cpp:853-877` (`field_offset = inset − op_width/2`, negative → `expandCellOutward` `:741-753`) |
| Headland ring generation, direction, corner fillets, mid-edge start | `coverage_planning.cpp:1003-1125` (`generateHeadlandSwaths` `:1013`, `kMinRingPerimeter` `:1003`, `ring_direction` flip `:1034-1050`, `sparsifyCollinear` + rotate-to-longest-edge + `roundSharpCorners` `:1072-1118`) |
| Mainland + straight swaths, swath angle (fixed / AUTO / large-field fallback) | `coverage_planning.cpp:1133` (mainland), `:1149-1225` (`BruteForce`, `bf.setStepAngle(kAutoAngleStepRad)` `:1157`, `generateBestSwaths` `:1198`, `BoustrophedonOrder` `:1203`, `min_swath_length` drop `:1215`); constants `kAutoAngleMaxAreaM2` `:57`, `kAutoAngleStepRad` `:67`, `longestEdgeAngle` `:99` |
| `mow_angle_deg` flow (GUI → BT → goal) | template `ros2/src/mowgli_bringup/config/mowgli_robot.yaml:334-338` → `ros2/src/mowgli_bringup/launch/full_system.launch.py:246-250` → `ros2/src/mowgli_behavior/src/behavior_tree_node.cpp:946-948` (blackboard) → `ros2/src/mowgli_behavior/src/coverage_nodes.cpp:1823-1845` (`PlanCoverageArea::buildGoal`) → `coverage_server.cpp:450-451` (deg→rad, `<0` = auto) |
| Turn-around connector (Dubins words, radius shrink loop, straight fallback) | `coverage_planning.cpp:543-580` (`buildConnector`, `r -= 0.02` loop `:555`), Dubins helpers `:257-500` (`dubinsLSL/RSR/LSR/RSL/RLR/LRL`, `sampleDubins`) |
| Corner fillets (rings and sub-paths) | `coverage_planning.cpp:585-680` (`roundSharpCorners`); ring call `:1108-1116` (`kRingCornerThreshold` 30°); sub-path call `:1679-1690` (`kCornerThreshold` 88°, `fillet_r = max(turn_radius/2, min_radius)`) |
| Sub-path splitting (holes / unsafe joins), ring drive order, swath chaining, NN reorder | `coverage_planning.cpp:1337-1806` (`buildContinuousSubPaths`): ring outer/hole partition `:1359-1400`, ring start alignment `:1420-1462` (`kAlignCos` 45°), swath nearest-endpoint chain `:1469-1530`, join/split decision `:1578-1650`, sub-path NN reorder `:1724-1800` |
| Which boundary the connectors are bounded by | `coverage_server.cpp:583-596` (`connector_clearance_boundary` → `safe_boundary` → raw), definitions `coverage_planning.hpp:76-125`, computed `coverage_planning.cpp:876-1000` |
| Out-of-bounds poses at convex ring corners (#388) | `coverage_planning.cpp:1275-1335` (`kClearanceClampMarginM` 2 cm, `clampInsideRing`), applied `:1701-1707` |
| Connector outcome stats / "fallback rate" WARN (#499) | `coverage_planning.hpp:188-243` (`ConnectorStats`), `coverage_server.cpp:161-192` (`kConnectorFallbackWarnPct` 25 %, `MOWGLI_CONNECTOR_STATS_FMT`), `:731-762` (log) |
| Plan diagnostics: dropped pieces, planned fraction, per-stage timing | `coverage_planning.hpp:44-61` (`PlanDiagnostics`), `coverage_planning.cpp:35-40` (`fmtDrop`), `:1228-1260`; logged `coverage_server.cpp:495-512`, timing `:713-723` |
| Verification of the final path (out-of-bounds, chassis footprint, in-hole) | `coverage_server.cpp:622-711` (`kBoundarySlackM` 0.05 `:642`, footprint check only when `inset >= robot_width/2` `:636`), ERROR logs `:771-803` |
| Discrete `segments` for the GUI (rings split at corners) | `coverage_server.cpp:248-294` (`douglasPeucker`), `:310-385` (`ringToArcs`, `kDpTol` 0.08), `:388-404` (`swathToPath`, `kSwathStep` 0.10) |
| "Field too small" failure path | `coverage_server.cpp:514-533` (`success=false`, message names the disabled-headland case) |
| Fields2Cover / OR-Tools linkage, RPATH, v3 pin | `ros2/src/mowgli_coverage/CMakeLists.txt:23-48` (`find_package(ortools)` `:32-33`, `Fields2Cover 3.0.0` at `/opt/fields2cover-300` `:42-48`), `:82`, `:97`, `:135` (`INSTALL_RPATH`); `ros2/Dockerfile:79-125` (stage `fields2cover-v3-builder`, SHA 884d895), `.github/workflows/ros2-ci.yml:233-287` |
| Turn-radius vs wheel-track sanity (WARN only) | `ros2/src/mowgli_bringup/launch/robot_config_util.py:309` (`check_turn_geometry`), called `ros2/src/mowgli_bringup/launch/navigation.launch.py:794-803` |
| Reproduce a field plan offline | Fixtures `ros2/src/mowgli_coverage/test/test_coverage_planning.cpp:127-160` (`recordedArea1Pts`, `makeRecordedArea1`), `:1522` (`RecordedArea1FullTraceAnalysis`), `:1671` (`RecordedArea1NoCuspInBounds`) |

## Files

| File | Lines | Purpose |
|------|-------|---------|
| `ros2/src/mowgli_coverage/CMakeLists.txt` | 153 | `mowgli_coverage_core` shared lib + `mowgli_coverage` exe; ortools/F2C 3.0.0 discovery; RPATHs; gtest registration |
| `ros2/src/mowgli_coverage/package.xml` | 37 | ament_cmake deps (rclcpp, rclcpp_action, rclcpp_lifecycle, rclcpp_components, nav2_util, nav_msgs, geometry_msgs, builtin_interfaces, mowgli_interfaces, ortools_vendor); test deps ament_lint_auto/common, ament_cmake_gtest |
| `ros2/src/mowgli_coverage/include/mowgli_coverage/coverage_server.hpp` | 68 | `CoverageServer : nav2_util::LifecycleNode`; lifecycle overrides; static param members |
| `ros2/src/mowgli_coverage/include/mowgli_coverage/coverage_planning.hpp` | 353 | Pure-geometry API: `PlanDiagnostics`, `BoustrophedonPlan`, `ConnectorStats`, `planBoustrophedon`, `buildContinuousPath`, `buildContinuousSubPaths`, `pointInRing`, `distanceToRing`, `dedupClosedRing`, `bufferRingOutward` |
| `ros2/src/mowgli_coverage/src/coverage_server.cpp` | 846 | Lifecycle + action server; goal→Cell; plan→`segments`/`full_path`/`drivable_subpaths`; verification + logging; component registration |
| `ros2/src/mowgli_coverage/src/coverage_planning.cpp` | ~2.1k | F2C pipeline (`planBoustrophedon`), Dubins connectors, fillets, sub-path builder, ring sanitization/buffering |
| `ros2/src/mowgli_coverage/src/main.cpp` | 15 | `rclcpp::spin` of `CoverageServer` |
| `ros2/src/mowgli_coverage/test/test_coverage_planning.cpp` | ~2.3k | 45 gtests against the REAL F2C v3 library (no ROS) — see Build, test, run |

No README, launch file, or config YAML lives in this package. Node defaults live in
`ros2/src/mowgli_bringup/config/nav2_params_base.yaml:1150-1180`; the launch entry is in
`ros2/src/mowgli_bringup/launch/nav2_navigation_launch.py:238-248`.

## Runtime surface

### Nodes

| Node | Executable | Launched by | Kind |
|------|------------|-------------|------|
| `coverage_server` | `mowgli_coverage` (package `mowgli_coverage`) | `ros2/src/mowgli_bringup/launch/nav2_navigation_launch.py:238-248` (params = merged Nav2 `configured_params`; `respawn`, `respawn_delay=2.0`); listed in `lifecycle_nodes` `:49-59` so `lifecycle_manager_navigation` configures/activates it | `nav2_util::LifecycleNode` (`coverage_server.cpp:25`); bond created on activate `:131`, destroyed on deactivate `:139`; also a composable component `RCLCPP_COMPONENTS_REGISTER_NODE` `:846` (not used by the launch) |

### Topics

None. The node publishes/subscribes nothing besides the action's internal topics. Every
`nav_msgs/Path` in the result carries `header.frame_id = "map"` (`coverage_server.cpp:437-439`).

### Services & actions

| Name | Type | Role | Notes |
|------|------|------|-------|
| `plan_coverage` (absolute `/plan_coverage`, no namespace) | `mowgli_interfaces/action/PlanCoverage` (`ros2/src/mowgli_interfaces/action/PlanCoverage.action`, registered `ros2/src/mowgli_interfaces/CMakeLists.txt:56`) | server (`nav2_util::SimpleActionServer`, `coverage_server.cpp:109-115`, 500 ms server timeout, `result_timeout` = param `action_server_result_timeout` 15 s) | Client: `PlanCoverageArea` in `ros2/src/mowgli_behavior/src/coverage_nodes.cpp:1914` (`/plan_coverage`; BT waits 3 s for the goal handshake `:2029` and 60 s for the result `:2054`) |

Goal: `outer_boundary` (`geometry_msgs/Polygon`, open ring OK, ≥3 points else `invalid_argument`
`:204-207`), `obstacles[]` (holes, <3-point holes silently skipped `:223-229`), `mow_angle_deg`
(`<0` = auto). Result: `success`, `message`, `segments[]` + `segment_types[]`
(`SEGMENT_RING=0` / `SEGMENT_SWATH=1`, GUI + bookkeeping only), `full_path` (concatenation of
sub-paths), `drivable_subpaths[]` (**what the BT drives**), `total_distance`, `ring_count`,
`swath_count`, `planning_time_s`. Feedback `phase` is declared but never published
(no `publish_feedback` call in `coverage_server.cpp`). Cancel → `terminate_all` `:430-435`;
`std::invalid_argument` → "Invalid coverage goal" `:828-834`; any other exception → "Internal
Fields2Cover error" `:835-841`, both `success=false` with `message = e.what()`.

### Parameters

Declared in `on_configure` (`coverage_server.cpp:53-105`). "Injected" = overwritten by
`navigation.launch.py` (`_inject_dock_pose_and_speeds`, `:922-948`) from the deep-merged
`mowgli_robot.yaml` (CLAUDE.md Invariant 15); static node defaults also sit in
`nav2_params_base.yaml:1150-1180`.

| Param | Node default | Injected from (`mowgli_robot.yaml` key → launch line) | Read |
|-------|--------------|-------------------------------------------------------|------|
| `robot_width` | 0.40 | `chassis_width` → `navigation.launch.py:930` (semantic only + footprint check `coverage_server.cpp:635-636`) | configure |
| `operation_width` | 0.18 (yaml 0.16) | `max(0.05, tool_width − swath_overlap)` → `:924`; template `tool_width` `:195`, `swath_overlap` `:405` | configure |
| `default_headland_width` | 0.20 | `headland_width` (template `:344`) → `:931`; only used when `num_headland_passes == 0` (AUTO) | configure |
| `num_headland_passes` | 0 | `num_headland_passes` (template `:374`, default 2) → `:932` **unclamped** (`<0` NONE / `0` AUTO / `>0` FORCED) | configure (restart to change) |
| `chassis_safety_inset` | 0.0 | `chassis_safety_inset` (template `:396`, 0.2) → `:935`; launch fallback 0.0 `:615-623`; server clamps ≥0 `:468` | LIVE per plan |
| `min_swath_length` | 0.15 | not injected (`nav2_params_base.yaml:1180`) | LIVE |
| `ring_direction` | 0 | `mow_direction` (template `:379`) → `:934` (0 F2C natural, 1 CW, 2 CCW) | LIVE |
| `min_turning_radius` | 0.15 | `min_turning_radius` (template `:416`) clamped [0.10, 0.50] `:790` → `:944` | LIVE |
| `connector_turn_radius` | 0.18 | `connector_turn_radius` — **no template key**; launch default `:403`, override read `:591-592`, clamped [floor, 0.50] `:791-792` → `:948` | LIVE |
| `obstacle_margin` | 0.0 | `obstacle_margin` (template `:665`, 0.2) clamped [0, 1] `:940`; server re-clamps `coverage_server.cpp:473-474` | LIVE |
| `action_server_result_timeout` | 15.0 | not injected | configure |
| `use_sim_time` | false | `nav2_params_base.yaml:1152` | configure |

Internal constants that act like parameters: `kSwathStep` 0.10 (`coverage_server.cpp:159`, discrete
segments), `kConnectorStep` 0.03 (`:602`, densification of `drivable_subpaths` swaths + connectors),
`kBoundarySlackM` 0.05 (`:642`), `kDensifyStep` 0.10 (`coverage_planning.cpp:30`, ring polylines),
`kAutoAngleMaxAreaM2` 400 (`:57`), `kAutoAngleStepRad` 5° (`:67`), `kOnEdgeTolM` 0.001 (`:94`),
`kMinRingPerimeter` 1.0 (`:1003`), `kClearanceClampMarginM` 0.02 (`:1275`), `kRingDedupTolM` 0.01
and `kRingSpikeTolM` 0.005 (`:1945-1946`).

### Planning pipeline (order of operations)

1. `buildCellFromGoal` (`coverage_server.cpp:201-231`): rings → `dedupClosedRing`, holes →
   `bufferRingOutward(hole, obstacle_margin)`.
2. `planBoustrophedon` (`coverage_planning.cpp:755-1260`): `n_rings` → `field_offset` →
   `safe_cells` (inset or outward expansion) → `safe_boundary` / `connector_clearance_boundary` /
   `safe_holes` → rings (`generateHeadlandSwaths`, out→in, min perimeter, winding, mid-edge start,
   fillets, re-densify) → mainland (`generateHeadlands(safe_cells, n_rings·op_width)` or
   `safe_cells` when `n_rings == 0`) → swaths per mainland cell (`BruteForce` fixed / best angle /
   longest-edge above 400 m², `BoustrophedonOrder`, `min_swath_length` drop) → diagnostics.
3. `ringToArcs` / `swathToPath` (`coverage_server.cpp:535-565`) → `result.segments`.
4. `buildContinuousSubPaths` (`coverage_planning.cpp:1337-1806`) bounded by
   `connector_clearance_boundary`: ring order partition → ring start alignment → swath
   nearest-endpoint chaining seeded from `BoustrophedonOrder`'s first swath → per join
   `buildConnector` (Dubins, shrink 0.02 m steps to `min_turning_radius`, straight fallback) →
   split only when the join is un-drivable blade-on (leaves boundary / crosses hole) →
   `roundSharpCorners` (>88°) → `clampInsideRing` → greedy NN sub-path reorder (adopted only if
   shorter; sub-path 0 fixed).
5. Verification + logs (`coverage_server.cpp:622-803`), result fill `:805-826`.

### TF frames

None used. Geometry is map-frame metres end to end (CLAUDE.md Invariant 4).

## Build, test, run

```bash
# Build (devcontainer / CI image; needs F2C 3.0.0 at /opt/fields2cover-300 + ros-kilted-ortools-vendor)
cd ros2 && make build-pkg PKG=mowgli_coverage        # = PACKAGES=mowgli_coverage ./scripts/build.sh (--packages-up-to)
cd ros2 && PACKAGES=mowgli_coverage PACKAGES_MODE=select ./scripts/build.sh   # this package only
# raw: colcon build --packages-up-to mowgli_coverage --cmake-args -DCMAKE_BUILD_TYPE=Release

# Unit tests (gtest, real F2C, no ROS graph)
cd ros2 && PACKAGES=mowgli_coverage ./scripts/test.sh
# raw: colcon test --packages-select mowgli_coverage --return-code-on-test-failure && colcon test-result --verbose
# single suite: ./build/mowgli_coverage/test_coverage_planning --gtest_filter='CoverageConnectorStats.*'

# Run the node alone and plan by hand
ros2 run mowgli_coverage mowgli_coverage --ros-args -p operation_width:=0.16 -p num_headland_passes:=2
ros2 lifecycle set /coverage_server configure && ros2 lifecycle set /coverage_server activate
ros2 action send_goal /plan_coverage mowgli_interfaces/action/PlanCoverage \
  "{outer_boundary: {points: [{x: 0.0, y: 0.0}, {x: 6.0, y: 0.0}, {x: 6.0, y: 6.0}, {x: 0.0, y: 6.0}]}, obstacles: [], mow_angle_deg: -1.0}"
# Field tuning between plans (LIVE params only): ros2 param set /coverage_server connector_turn_radius 0.25
```

Test registration: `CMakeLists.txt:117-137` (`ament_add_gtest(test_coverage_planning …)` `:129`, RPATH to
`/opt/fields2cover-300/lib` `:135`; `ament_lint_auto` with copyright/cpplint/uncrustify disabled `:118-122`).

`ros2/src/mowgli_coverage/test/test_coverage_planning.cpp` — 45 `TEST`s, by suite (line = first test):

| Suite | Pins |
|-------|------|
| `CoverageRepro` `:108` | Sim rectangle geometry (inset 0.20, r 0.18) builds plan + sub-paths without heap corruption |
| `CoveragePlanning` `:260-335`, `:585`, `:765-847`, `:930-1223`, `:1458-1486`, `:1974-2024` | Square/L-shape/concave coverage, serpentine order, fixed angle determinism, hole not crossed, `ring_direction` winding, too-small → empty plan, headland pass override, `num_headland_passes<0` yields no rings and same border as rings-on (#429), outermost swath at op_width/2 inside planning cell, clearance ring == `safe_boundary` with rings off, chassis-inset opt-in keeps blades inside, turn-arc footprint inside recorded boundary, ring rides ON line at inset 0, `planned_fraction` reported, large-field longest-edge fallback (>400 m²) + determinism, degenerate recorded ring sanitized to full coverage |
| `CoverageContinuousPath` `:376-496`, `:616`, `:1336`, `:1402`, `:1671` | Continuous path avoids hole, hole-free field = 1 sub-path, single central hole does not over-fragment, sub-path reorder stays hole-free + deterministic, notch field lobe-chained (no mid-field joins), outermost ring stays inside clearance ring, rings-off = 1 sub-path, recorded area 1 no cusp + in-bounds with deployed knobs |
| `BoundaryClipGeometry` `:1499` | `pointInRing` on a concave notch |
| `CoverageIntegration` `:1522` | Full trace analysis on recorded area 1 (runs split at 0.6 m `kSegmentTransitGap`, in-bounds) |
| `RingDedup` `:1803-1839` | Doubled leading vertex dropped, clean ring unchanged, deduped degenerate ring still plans |
| `ObstacleMargin` `:1858-1921` | `bufferRingOutward` grows by margin, zero margin passthrough, degenerate ring falls back to input, plan keeps margin off drawn obstacle |
| `CoverageConnectorStats` `:2181-2302` | Outcomes partition `attempted`; every join is an arc when spacing > 2R; swath-end arcs decided by headland apron not radius; raising the floor barely moves fallback rate; null stats pointer safe (#499) |

Related tests elsewhere: `ros2/src/mowgli_bringup/test/test_launch_injection.py:116,134`
(`num_headland_passes` injected and NOT clamped), `gui/web/src/components/settings/MowingSection.test.tsx`
(GUI three-way sentinel, #429).

CI: `.github/workflows/ros2-ci.yml` — F2C v3 built from source at SHA `884d895` and cached
(`:233-276`), ldconfig + `CMAKE_PREFIX_PATH` (`:278-287`), `COLCON_IGNORE` for upstream
`opennav_coverage` subpackages (`:301-306`), whole-workspace `colcon build` (`:337-342`) and
`colcon test` (`:344-350`). There is no per-package CI job.

## Change coupling — "if you change X, also update Y"

- **`PlanCoverage.action`** (`ros2/src/mowgli_interfaces/action/PlanCoverage.action`) → consumers
  `ros2/src/mowgli_behavior/src/coverage_nodes.cpp` (`buildGoal` `:1823`, result handling `:2017-2080`),
  `ros2/src/mowgli_behavior/include/mowgli_behavior/bt_context.hpp:557`, `coverage_server.cpp`;
  regenerate GUI bindings per `docs/claude/commands.md` (Go `generate_go_msgs.sh`, TS `generate_ts_types.sh`).
- **New/renamed `coverage_server` param** → `coverage_server.cpp:53-105` + `nav2_params_base.yaml:1150-1180`
  + `navigation.launch.py` read (`:382-453`, `:591-592`) and `cov_params[...]` write (`:922-948`)
  + template `mowgli_robot.yaml` key (Invariant 15) + GUI `gui/web/src/components/settings/MowingSection.tsx`,
  `paramCatalog.ts:41-46`, `useSettingsManager.ts:144` (key list) + a `test_launch_injection.py` guard.
- **`operation_width` = `tool_width − swath_overlap`** (`navigation.launch.py:924`) must stay coupled to
  `map_server.tool_width` (`full_system.launch.py:421-428`) — CLAUDE.md Invariant 6; both fallbacks import
  `robot_config_util.DEFAULT_TOOL_WIDTH_M` (`robot_config_util.py:48`).
- **`obstacle_margin`** is applied twice: here on holes (`bufferRingOutward`) and in `map_server`'s keepout
  mask (`full_system.launch.py:402-405`). Change the clamp band in both.
- **`chassis_safety_inset`** also feeds BT bypass arcs (`full_system.launch.py:390-398`, fallback
  `chassis_width/2`) while `navigation.launch.py:615-623` falls back to 0.0 — the template value (0.2)
  normally masks the disagreement.
- **`min_turning_radius` / `connector_turn_radius`** clamp bands (`navigation.launch.py:783-792`) feed
  `check_turn_geometry` (`robot_config_util.py:309`) and the dig detector's `dig_max_yaw_rate` rationale
  (`ros2/src/mowgli_bringup/config/hardware_bridge.yaml:88-97`).
- **Pose spacing of `drivable_subpaths`** (`kConnectorStep` 0.03 on swaths/connectors, `kDensifyStep`
  0.10 on rings) is assumed by FTC's `obstacle_lookahead` conversion in `navigation.launch.py:813-824`
  (`kF2CSamplingM = 0.05`) and by the BT resume cursor — change one, re-derive the other.
- **`kSegmentTransitGapM` 0.6** (`ros2/src/mowgli_interfaces/include/mowgli_interfaces/coverage_geometry.hpp:26`)
  is the BT's gap classifier for the gaps BETWEEN sub-paths (`coverage_nodes.cpp:618`); the planner no
  longer splits on it (`coverage_planning.cpp:1578-1600`). Test `:1522` mirrors it as `kTransitGap`.
- **F2C pin** (`CMakeLists.txt:42-48`, 3.0.0 at `/opt/fields2cover-300`) ↔ `ros2/Dockerfile:79-125`
  (stage + SHA), `:273-275` (copy + ldconfig), `ros2-ci.yml:233-276` (same SHA in cache key), RPATHs
  in `CMakeLists.txt:82,97,135`. Moving the SHA means all four.
- **`opennav_coverage` submodule**: only `opennav_coverage_msgs` builds; `COLCON_IGNORE` markers are
  created at build time (`ros2/Dockerfile:388-392`, `ros2-ci.yml:301-306`, `ros2/scripts/sync_workspace_packages.sh`),
  not committed. `mowgli_coverage` has no dependency on it (`package.xml`).
- **`mow_angle_deg`** is a BT/blackboard param, not a `coverage_server` param: template
  `mowgli_robot.yaml:338` → `full_system.launch.py:250` → `behavior_tree_node.cpp:947`.

## Pitfalls

- Coverage-slot rules are CLAUDE.md Invariants 7 and 8 plus the "What NOT to Do" bans (no F2C turn
  planners, no upstream `opennav_coverage`, no `buildHeadlandRingPath` on the BT side). The driven
  artefact is `drivable_subpaths`, never `segments`.
- `on_configure` can run twice (cleanup → configure); use the `has_parameter` guarded lambdas
  (`coverage_server.cpp:34-51`) — a bare `declare_parameter` throws and bricks Nav2 bringup.
- `num_headland_passes` is read once at configure (`:70`); `<0` must reach the node unclamped
  (`navigation.launch.py:418-422`, guarded by `test_launch_injection.py:134`). `headland_width` only
  matters when it is 0 (AUTO) — the template forces 2.
- `chassis_safety_inset` is "how far inside the recorded line the outermost DRIVEN pass sits":
  `field_offset = inset − op_width/2` (`coverage_planning.cpp:853`), negative → outward expansion so
  ring 0 rides ON the line at inset 0; floored at 0 when rings are off. The chassis-footprint check
  only runs when `inset ≥ robot_width/2` (`coverage_server.cpp:636`).
- Connectors/fillets are bounded by `connector_clearance_boundary` (outermost-ring centreline),
  NOT `safe_boundary` (`coverage_server.cpp:583-596`, rationale `coverage_planning.hpp:85-113`).
  With rings off it is `safe_boundary` exactly; on-edge swath ends pass via `kOnEdgeTolM` 1 mm
  (`coverage_planning.cpp:69-94`) — do NOT re-add the 0.03 m outward expansion (`:86-93`).
- Expect the `PlanCoverage connectors … fallback rate` WARN on every plan at shipped defaults
  (~97 % straight joins, 1 arc in 32): the headland apron (`num_headland_passes × operation_width`
  = 0.32 m) is narrower than an omega turn's forward extent, so no radius in the shrink range fits
  (`coverage_planning.hpp:200-218`, `coverage_server.cpp:161-176`). Raising `min_turning_radius`
  is not the lever (#499).
- `clampInsideRing` (#388, `coverage_planning.cpp:1275-1335`) silently nudges out-of-bounds poses
  2 cm inside the clearance ring; only residuals beyond `kBoundarySlackM` 0.05 reach the ERROR log
  (`coverage_server.cpp:673-677`).
- AUTO swath angle: exhaustive 5° sweep only below 400 m² (`kAutoAngleMaxAreaM2`), longest-edge
  above; both constants are fixed for resume determinism (`coverage_planning.cpp:42-67`). Budget:
  the server's `action_server_result_timeout` is 15 s (`coverage_server.cpp:102-105`) while the BT
  actually waits 60 s for the result (`coverage_nodes.cpp:2054`) — the "BT waits 12 s" the server
  comment claims is stale, so the ordering that comment asserts no longer holds.
- Swath chaining must be seeded from `BoustrophedonOrder`'s own first swath, not the last ring's
  end (`coverage_planning.cpp:1484-1495`) — seeding from the ring end relocates U-turns to ends
  with no headland room and fragments the plan.
- `dedupClosedRing` is the last gate before an `f2c::types::Cell`: a doubled vertex or mm-scale
  sliver makes boost::geometry reject the ring and the area silently drops (`coverage_planning.hpp:333-341`).
  `bufferRingOutward` never drops an obstacle, only its margin (`:1872-1880`).
- `planned_fraction` is a coarse strip-area estimate, not a coverage guarantee (`coverage_planning.hpp:53-57`).
- `.devcontainer/Dockerfile:177-186` only builds F2C **v2.0.0** into `/opt/fields2cover-200`; this
  package pins 3.0.0, so the devcontainer cannot build it without the v3 stage from `ros2/Dockerfile`.
- Stale in-code comments to ignore: `coverage_server.hpp:8-10`, `package.xml` description and
  `nav2_navigation_launch.py:234-237` still say the BT dispatches one segment per goal;
  `coverage_planning.hpp:18-19` and several `.cpp` comments still name MPPI (FTC is the controller);
  `coverage_server.cpp:102-104` and `coverage_planning.cpp:45-46,54-56` still cite a 12 s BT planning
  timeout (`PlanCoverageArea` waits 60 s); `coverage_planning.hpp:301-304` still says a join gap
  over ~0.6 m splits the sub-path (that length gate — and its named `kMaxMowJoinGapM` — is gone);
  `navigation.launch.py:448-449` cites a `clip_path_to_boundary` that exists nowhere.

## Generated & vendored — do not hand-edit

- `fields2cover.h` / `libFields2Cover.so` (v3.0 @ `884d895`) come from `/opt/fields2cover-300`, built by `ros2/Dockerfile` stage `fields2cover-v3-builder` and `ros2-ci.yml`; not in the tree.
- `mowgli_interfaces/action/plan_coverage.hpp` is rosidl-generated from `PlanCoverage.action`; GUI Go/TS message bindings are generated (`docs/claude/commands.md`).
- `ros2/src/opennav_coverage/` is a git submodule — only `opennav_coverage_msgs` is built; the server subpackages get build-time `COLCON_IGNORE` markers.
