# Per-control-cycle benchmark: following one long coverage sub-path

`cycle_bench` measures, as a function of the sub-path's pose count (1k … 400k at
the planner's 0.03 m spacing), what each consumer of the `FollowCoveragePath` goal
costs **every control cycle** (10 Hz):

| mode | what runs | robot |
|---|---|---|
| `server` | controller_server's own path bookkeeping, as Nav2 1.5.2 `ControllerServer` does it each cycle: `findPlanSegment` + `transformLocalPlan` of the selected `PathHandler` plugin, then `distance_from_path` + `calculate_path_length` of the tracking feedback | on the plan, 0.05 m/cycle |
| `ftc` | `FTCController::computeVelocityCommands`, LiDAR flags on, empty costmap (the `test_ftc_turn_fallback_controller` harness) | kinematic, closed loop |
| `checker` | `PathProgressGoalChecker::isGoalReached` | on the plan |

It is a benchmark, not a test: it is not built by the package. Every cycle runs
the real code; the reported figure is the median over the cycles.

## Build and run

From the repository root, with ROS Lyrical + Nav2 sourced and
`mowgli_nav2_plugins` installed into a workspace (`<ws>`):

```sh
source <ws>/install/setup.bash
cmake -S ros2/src/mowgli_nav2_plugins/test/bench -B build/cycle_bench -DCMAKE_BUILD_TYPE=Release
cmake --build build/cycle_bench -j2
export ROS_DOMAIN_ID=173 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
taskset -c 5 build/cycle_bench/cycle_bench server 200 nav2_controller::FeasiblePathHandler
taskset -c 5 build/cycle_bench/cycle_bench server 200 mowgli_nav2_plugins/LazyPrunePathHandler
taskset -c 5 build/cycle_bench/cycle_bench ftc 200
taskset -c 5 build/cycle_bench/cycle_bench checker 200
```

Columns (`server`): `setplan_ms` one-off cost of a new goal (handler `setPlan`
plus controller_server's own copy), `handler_us` path handler per cycle,
`pathlen_us` `calculate_path_length`, `search_us` `distance_from_path`,
`cycle_us` their sum.

`path_transport_bench` measures the one-off cost of moving such a path: deep
copy, CDR serialize/deserialize (`serialize`), and a publish to a second process
with the stack's plan QoS (`sub` in one shell, then `pub 5`; use the production
`CYCLONEDDS_URI=file://$PWD/docker/config/cyclonedds.xml`).

## Results (x86 i7-10700K, one core, machine load 20–40, 2026-11-09)

See `PERF_REPORT_PATH.md` at the repository root for the interleaved A/B and the
interpretation. In short: FTC (~8.2 ms) and the goal checker (~0.1 µs) do not
depend on the path length; controller_server's bookkeeping grows ~22 ns per
remaining pose with upstream's `FeasiblePathHandler` (8.9 ms at 400k poses).
