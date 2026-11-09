# Dense coverage path — transport and downstream handling: performance report

Branch `perf/coverage-path-transport`, 2026-11-09. **Stacked on the unmerged
`perf/coverage-planner` (track B, `e23c5e47d`)**; nothing here depends on track B's code.
Software evidence only: x86-64 benchmarks and unit tests. **Not measured on a Raspberry Pi and
not run on the robot.**

| | commit |
|---|---|
| test: per-cycle + transport benchmarks, e2e field-size breakdown | `de7d876b7` |
| perf: `LazyPrunePathHandler` + differential test + config + docs | `2816b4e44` |
| this report | last commit |

## Result

**The real headroom is per control cycle, not in the transport.** While a coverage sub-path is
followed, Nav2 1.5.2's own controller_server does path bookkeeping every cycle, and its cost grows
linearly with the poses still ahead: about 18 ns per pose on x86. At 400k poses that is
**7.2 ms per cycle**, on top of FTC's 8.2 ms. FTC and the goal checker themselves do **not**
depend on the path length.

`LazyPrunePathHandler` removes the larger part of it with **identical results**. That part is the
path handler's per-cycle `vector::erase`.

| remaining poses | controller_server bookkeeping, upstream | with LazyPrunePathHandler | ratio [95 % CI] |
|---|---|---|---|
| 1k | 18.8 µs | 13.5 µs | 1.39× [1.34–1.47] |
| 10k | 119 µs | 56 µs | 2.11× [2.08–2.15] |
| 50k | 798 µs | 276 µs | **2.89× [2.84–3.05]** |
| 100k | 1711 µs | 705 µs | 2.43× [2.38–2.49] |
| 200k | 3510 µs | 1528 µs | 2.30× [2.25–2.35] |
| 400k | 7213 µs | 3081 µs | **2.34× [2.31–2.38]** |

**The handler itself.**
- Wall time per cycle: 3948 µs → 14 µs at 400k poses. Over 1k–400k it stays flat at 8–14 µs.
- Instructions (callgrind, handler functions only, 300 cycles over the six sizes):
  **2.368 G → 18.8 M Ir, 126× fewer.**
- Peak RSS: unchanged (312.4–312.9 MB for both).

**What it does not fix.** The remainder is `calculate_path_length` in controller_server's tracking
feedback. It is upstream code, and no plugin can reach it (see "Upstream follow-ups").

**The one-off transport costs are real, but much smaller in total.** They are listed below with
numbers, as proposals. Only the BT move (proposal 3) is implemented, in a later commit; the
others change an interface or a topic's semantics.

## How big are the paths? (e2e, real `coverage_server`, shipped parameters)

Two real `coverage_server` processes and track B's C++ client, run with `E2E_SIZES=1`. The
fixtures are track B's realistic lawns; columns 4–6 are serialized sizes.

| field (rings) | poses | sub-paths | largest sub-path | result | `full_path` | segments | round trip − server |
|---|---|---|---|---|---|---|---|
| blob200_o5 (5) | 37k | 4 | 30k | 6.3 MB | 2.7 MB | 1.0 MB | ~0.1 s |
| blob600_o10 (5) | 115k | 7 | **80k** | 19.4 MB | 8.3 MB | 2.8 MB | ~0.2 s |
| ushape1200_o5 (5) | 239k | 2 | **223k** | 39.9 MB | 17.2 MB | 5.5 MB | ~0.4 s |
| blob2000_o0 (5) | 415k | 1 | **415k** | 69.0 MB | 29.9 MB | 9.2 MB | 1.2–2.3 s |
| blob2000_o20 (5) | 402k | 10 | 213k | 67.0 MB | 28.9 MB | 9.1 MB | 1.3–2.3 s |
| blob600_o10 (rings off) | 119k | 171 | 16k | 19.8 MB | 8.6 MB | 2.6 MB | ~0.2 s |
| blob2000_o20 (rings off) | 407k | 316 | 49k | 67.4 MB | 29.3 MB | 8.8 MB | 0.7–1.4 s |

- With the shipped five headland rings, one goal on an ordinary 600–1200 m² lawn is 80k–223k
  poses. That sets the per-cycle numbers above.
- `full_path` is 43–45 % of every result.
- Round-trip figures are medians of 3 interleaved runs on a loaded machine, so read them as ranges.

## Every consumer of the dense path, measured

x86 i7-10700K, one core (`taskset -c 5`), machine load 20–40.

| where | when | what | cost at 400k poses (x86) | scales with |
|---|---|---|---|---|
| controller_server, `FeasiblePathHandler::transformLocalPlan` | **every cycle (10 Hz)** | `erase(begin, closest)` moves every remaining pose | 3.9 ms (A/B median) | remaining poses |
| controller_server, `computeAndPublishVelocity` | **every cycle** | `calculate_path_length(current_path_, start_index_)` for the tracking feedback | 2.9–3.7 ms | remaining poses |
| FTC `computeVelocityCommands` | every cycle | lookahead, lattice horizon, windowed searches, pivot-corner binary search | 8.2 ms, **flat** 1k → 400k | — |
| `PathProgressGoalChecker::isGoalReached` | every cycle | bounded index window, arc-length prefix sums | 0.08 µs, **flat** | — |
| FTC `newPathReceived` | per dispatch | copy + corner scan + **latched viz republish** on `/FollowCoveragePath/global_plan` | 195–242 ms; **47 ms without the republish** | poses |
| controller_server `setPlannerPath` | per dispatch | 3 plan copies (handler ×2, `current_path_`) | 53 ms | poses |
| `nav_msgs/Path` CDR | per hop | serialize / deserialize (28.8 MB) vs deep copy (38 MB in memory) | 140 / 81 ms vs 16 ms | poses |
| DDS publish, 2 processes, production `cyclonedds.xml` on `lo` | per hop | `publish()` blocks the caller / delivered | 178 / 291 ms | poses |

Notes on the measurements:
- **FTC per cycle.** Measured closed loop on a kinematic robot with the LiDAR flags on and an
  empty costmap. The ~8 ms is the controller's own work and is independent of path length (most
  likely the lattice). That belongs to another track.
- **FTC resync.** Its carrot-resync recovery copies the current pivot-free leg, which is O(n),
  but only on a resync event, not every cycle.
- **Goal checker.** Its `projectToArcWindow` scans every segment, but only when the robot reaches
  the artificial search frontier. That does not happen at normal speeds (10 poses per call is
  allowed; 1.7 is driven).

**Hops per dispatch** of a FollowStrip unit:
1. FollowStrip publishes the unit on `/controller_server/FollowCoveragePath/global_plan` (latched)
   for the goal checker.
2. The BT's own subscription on that topic also receives and deserializes it, only to read
   `poses.front()`.
3. The `FollowPath` goal carries the same path to controller_server.
4. FTC republishes it on the viz-only latched topic.

That is ~3 serializations and ~3 deserializations of the unit, plus ~6 deep copies. On x86 it is
roughly 0.6–0.8 s of CPU per dispatch at 400k poses.

**Per plan**, the BT:
- receives the 67 MB result;
- copies `segments`, `full_path` and `drivable_subpaths` into `BTContext`;
- copies the sub-paths again for the resume fingerprint;
- publishes `full_path` on the latched `/coverage/full_plan`. `mqtt_bridge_node` turns that into a
  ~7 MB retained JSON message when MQTT is enabled.

**Why the per-cycle cost dominates.** One 400k-pose sub-path is ~12.5 km of driving. At 0.3 m/s
that is hours of 10 Hz cycles: 4.5 ms average × 10 Hz × ~40 000 s ≈ 30 CPU-minutes of bookkeeping
on x86. The one-off costs add up to seconds per plan or dispatch.

## The change: `LazyPrunePathHandler`

**What it is.** A `nav2_core::PathHandler` plugin and a subclass of upstream
`nav2_controller::FeasiblePathHandler`. It lives in its own small library,
`mowgli_lazy_prune_path_handler`.

**Why the result is identical.**
- Upstream erases the passed poses `[begin, closest)` every cycle. This handler instead records
  them as passed (`pruned_`), and erases them in bulk only when they are the larger half of the
  vector. Each pose is then moved at most once per halving: O(1) amortized.
- `findPlanSegment` and `transformLocalPlan` are upstream's code, verbatim, run on
  `[begin + pruned_, end)`.
- Parameters, `setPlan` (including the inversion/rotation constraint truncation),
  `getTransformedGoal` and the dynamic-parameter callbacks are inherited.
- Upstream's `enforce_path_inversion`/`rotation` branch reads and replaces the pruned plan, so it
  is given the compacted vector.
- Invariant (proof by induction over setPlan / findPlanSegment / transformLocalPlan): the logical
  sequence always equals upstream's pruned plan. Searches and the transform loop run the same code
  on the same elements; they mutate the same header fields and return the same values.
- Upstream's two physical-size checks agree with the logical size, because upstream never prunes
  below two poses. As a belt-and-braces guard, the handler compacts whenever fewer than two live
  poses would remain.

**Wiring.** `nav2_params_base.yaml` `PathHandler.plugin: mowgli_nav2_plugins/LazyPrunePathHandler`,
for both lanes, with the same parameters. `nav2_controller` moves from `test_depend` to `depend`;
it is already installed in the Docker image (`ros2/Dockerfile`).

**No change to:**
- any message, action, topic or parameter;
- FTC, the goal checker, FollowStrip or the BT;
- safety logic;
- the driven path. The planner and its output are untouched, and the controller sees the same
  transformed plan every cycle.

### Correctness evidence

**`test_lazy_prune_path_handler`** (6 gtests). The real upstream plugin and the lazy one run side
by side on one node, costmap and TF buffer, with a non-identity `map→odom`. Every cycle it
compares:
- the exception text, if any;
- the closest pose and the segment length;
- the transformed local plan;
- `getTransformedGoal`;
- the pruned plan itself: upstream's vector must equal the live tail of the lazy one.

The scenarios cover:
- serpentines with pivot twins, 6 seeds × 2500 cycles, with noise, stalls, back-drift, fast
  advance and localisation jumps;
- a closed ring, where start == end;
- a plan mostly outside the costmap, plus a robot 30 m away;
- replans mid-path;
- 1-, 2- and 3-pose plans, including the "0 poses" exception;
- an inversion cusp with `enforce_path_inversion` on. The test asserts that the branch fires at
  least 4 times, and that the lazy path really keeps poses behind in more than half the cycles.

**Mutation check.** Three plausible bugs were planted one at a time:
- searching from the physical begin — caught, 33 failures;
- corrupting the offset — caught, 15 failures;
- skipping the compaction before the constraint branch — caught, 8 failures.

A fourth mutant survived as predicted. It used the physical instead of the logical size in the
"keep two poses" check, which is equivalent (see the invariant above).

**Other checks:**
- `test_nav2_params.py`: 80 passed, including the new pin on the plugin in both overlays.
- `ament_lint_cmake`, `ament_xmllint`: clean.
- clang-format 18: applied.
- Not run here: the rest of `mowgli_nav2_plugins`' gtest suite, which is unaffected (no existing
  source changed except CMake and `package.xml`), and `ament_cppcheck` (the host's cppcheck 2.19
  is skipped by ament).

## Method and reproduction

- **Interleaved A/B.** 21 alternating runs per handler of
  `cycle_bench server 200 <plugin>` on core 5; median per size, with a bootstrap 95 % CI
  (4000 resamples) of the median ratio.
- **Instruction counts.** callgrind from the distro 3.26 `.deb`, extracted without root,
  `--toggle-collect` on `*PathHandler::findPlanSegment*` and `*PathHandler::transformLocalPlan*`.
- **Builds.** colcon in the worktree (`.ws/`), with the Nav2 `.deb`s extracted into a scratch
  root. `perf` is blocked on this host.

```sh
# packages (Release): mowgli_interfaces, mowgli_coverage, mowgli_nav2_plugins
colcon build --base-paths ros2/src/mowgli_interfaces ros2/src/mowgli_coverage \
  ros2/src/mowgli_nav2_plugins --build-base .ws/build --install-base .ws/install \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
source .ws/install/setup.bash
export ROS_DOMAIN_ID=173 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# per-cycle and transport benchmarks
cmake -S ros2/src/mowgli_nav2_plugins/test/bench -B .ws/bench && cmake --build .ws/bench -j2
taskset -c 5 .ws/bench/cycle_bench server 200 nav2_controller::FeasiblePathHandler
taskset -c 5 .ws/bench/cycle_bench server 200 mowgli_nav2_plugins/LazyPrunePathHandler
taskset -c 5 .ws/bench/cycle_bench ftc 200; taskset -c 5 .ws/bench/cycle_bench checker 200
taskset -c 5 .ws/bench/path_transport_bench serialize
export CYCLONEDDS_URI=file://$PWD/docker/config/cyclonedds.xml
taskset -c 5 .ws/bench/path_transport_bench sub & taskset -c 5 .ws/bench/path_transport_bench pub 5
# differential test
colcon test --packages-select mowgli_nav2_plugins --ctest-args -R test_lazy_prune_path_handler
#   (here the gtest binary was run directly from .ws/build/mowgli_nav2_plugins/, core 5)
python3 -m pytest -q ros2/src/mowgli_bringup/test/test_nav2_params.py
# field sizes (track B's harness; fixtures from make_bench_fixtures.py)
cmake -S ros2/src/mowgli_coverage/test/bench/e2e -B .ws/e2e && cmake --build .ws/e2e
E2E_SIZES=1 ros2/src/mowgli_coverage/test/bench/e2e/run_e2e.sh EXE EXE .ws/e2e/e2e_plan_coverage \
  5 3 5 fixtures/blob2000_o0_r5.json ...   # EXE = .ws/install/mowgli_coverage/lib/mowgli_coverage/mowgli_coverage
```

## Pi-scale estimate (not measured)

These loops stream memory: about 38 MB is touched per cycle at 400k poses. I therefore assume a
Pi 4 is at least ~3–5× slower than this core, and a Pi 5 ~2×. On that assumption, a Pi 4 at the
start of a 223k–400k-pose sub-path:

| | upstream | lazy |
|---|---|---|
| controller_server bookkeeping per cycle | ~11–36 ms | ~5–15 ms |
| at 50k poses | ~2–4 ms | ~1 ms |

Against the 100 ms cycle budget, upstream's share is ~10–35 %, next to FTC's own ~25–40 ms.
**HARDWARE_PENDING**: measure `cycle_bench server` on a Pi 4/5.

## Dead ends and what was not done

**Measured and found bounded, so not worth touching:**
- FTC per-cycle path handling;
- the goal checker's per-cycle search;
- FollowStrip's progress cursor (`advanceProgressCursor`, bounded by arc length);
- `makeCoveragePreview` (Douglas–Peucker, once per plan; the GUI gets only the decimated preview).

**Not reachable from a plugin:** `calculate_path_length` in controller_server's tracking feedback,
the remaining ~40–50 % of the bookkeeping (see "Upstream follow-ups").

**Proposals, not implemented**, because each changes an interface or a topic contract. Ranked by
measured benefit per unit of invasiveness:
1. **Skip FTC's viz-only republish on `/FollowCoveragePath/global_plan`** when nobody subscribes.
   - Saves ~150–195 ms per dispatch at 400k on x86, from 242 to 47 ms in `newPathReceived`. That
     time is in controller_server's execute thread before the first cycle, and inside the loop on
     a preemption.
   - Cost: a late-joining Foxglove/RViz loses the latched copy. A maintainer has to choose that.
     The topic is not consumed by anything in the repository (the goal checker reads
     `/controller_server/FollowCoveragePath/global_plan`).
2. **FollowStrip ignores its own latched publication** on
   `/controller_server/FollowCoveragePath/global_plan`.
   - Saves one 29 MB deserialization (~81 ms) and a 38 MB allocation in the BT per dispatch. It
     reads only `poses.front()` of FTC's rejoin message.
   - Needs `ignore_local_publications`. That would also drop same-process publishers, which
     `test_follow_strip_dig.cpp` uses, so the test needs rework. Moderate risk; not done.
3. **BT copies:** move the plan result into `BTContext` instead of copying, and stop copying the
   sub-paths again for the resume fingerprint.
   - **Half done, in a later commit:** `PlanCoverageArea` now moves `segments`, `full_path` and
     `drivable_subpaths` out of the action result, and everything after reads the context. The
     result is held by the goal handle until the next plan, so the copy kept every pose twice
     for the whole session. The short-lived `units` copy for the resume fingerprint is left as is.
     Checked: the file compiles (`g++ -fsyntax-only`) and clang-format 18 is clean. Not run: the
     BT unit tests (the package's other dependencies were not built in this worktree).
   - Saves ~38–115 MB of transient BT memory and a few tens of ms per plan, at 400k poses.
   - This is analysis from sizes only (`sizeof(PoseStamped)` = 96 B); memory was not measured in
     the BT.
4. **Interface: stop filling `PlanCoverage.full_path`.** It is the concatenation of
   `drivable_subpaths`, 43–45 % of the result.
   - Saves ~29 MB per plan, and roughly 0.3–1 s of serialization and transfer on x86 (~1–4 s
     estimated on a Pi 4).
   - It is a semantic change of a documented field. An old BT with a new server would publish an
     empty `/coverage/full_plan`, and `TransitToStrip` would skip its transit, because it checks
     `current_strip_path.poses.empty()`.
   - Compatibility plan: keep the field; have the BT build its `current_strip_path` and
     `/coverage/full_plan` from `drivable_subpaths` first; ship that; only then let the server
     leave `full_path` empty, behind a goal flag such as `omit_full_path` that an old client never
     sets. It needs `.action` codegen (`docs/claude/commands.md`). It is not justified while the
     per-plan cost stays at a few seconds on a Pi.

## Upstream follow-ups (Nav2, not done here: no upstream changes)

- **`FeasiblePathHandler::prunePlan`.** Use the same offset-and-bulk-erase scheme, or a
  `std::deque`. Exact, and about 10 lines. If upstream fixes it, this plugin can be deleted and the
  config pointed back at `nav2_controller::FeasiblePathHandler`.
- **`ControllerServer::computeAndPublishVelocity`.** Compute `remaining_path_length` from a prefix
  sum built once in `setPlannerPath`, instead of `calculate_path_length` every cycle. It is a
  feedback-only value, so a different summation order is acceptable there. This is the remaining
  ~3 ms per cycle at 400k poses.

## Risks

- **Binary coupling to upstream internals.** The plugin reads `FeasiblePathHandler`'s protected
  members (Nav2 1.5.2). An upstream change of those fails the build loudly. A behavioural change
  of upstream's algorithm (both overridden functions) would *not* fail the build. Instead the
  differential test, which compares against the live upstream plugin, would flag it.
- **Linking a plugin library.** `libmowgli_lazy_prune_path_handler.so` links
  `libfeasible_path_handler.so`, so loading it registers upstream's factory too. That is harmless
  while only one path handler is configured, which is the case here and the reason it is a
  separate library. Configuring *both* handlers in one controller_server could make class_loader
  attribute upstream's factory to our library.
- **Platforms.** ARM64 was not run. The code is plain C++ with no intrinsics, and the arithmetic is
  upstream's, unchanged.
- **Transit lane.** The transit lane (RPP) uses the same handler. Its paths are short, and its
  results are identical.

## Porting to `mowglinext/mowglinext`

The change applies on `dev` independently of track B: it touches only `mowgli_nav2_plugins`, one
line of `nav2_params_base.yaml`, `test_nav2_params.py` and two docs. The e2e-harness tweak
(`E2E_SIZES`, overridable `ROS_DOMAIN_ID`) lives in track B's files, so it should go with or
after track B.

I expect acceptance with discussion, because it is exact, test-pinned and a config-level swap. A
maintainer may reasonably prefer to fix this in Nav2 itself and avoid carrying a subclass. The
differential test then still serves as the check before switching back.

Physical acceptance (per-cycle CPU on a Pi under a real long sub-path) is **HARDWARE_PENDING**.
No motion or field test is needed beyond that, because the controller's inputs are unchanged.
