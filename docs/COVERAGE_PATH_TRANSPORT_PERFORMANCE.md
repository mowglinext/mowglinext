# Dense coverage path — transport and downstream handling: performance report

2026-11-09/10. Contributor document: optimization evidence and reproducible benchmarks.

Evidence kinds, kept apart:
- **Software:** x86-64 benchmarks (i7-10700K, one pinned core) and unit tests.
- **Hardware benchmark:** the same per-cycle benchmark run once on the robot's Raspberry Pi 5
  (below). It is a benchmark, not a mow.
- **Not done:** no mow, no motion and no field test with the new handler under the real stack.

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

**What these numbers are.** Each size is the median of 200 consecutive cycles starting at the
beginning of the plan, so the robot has advanced about 330 poses. They show the steady per-cycle
cost, not the compaction events (below).

**Compaction.** The lazy handler erases the passed prefix in bulk once it is more than half of the
vector. Over a whole sub-path that moves each pose at most once per halving (amortized O(1)), but
a single compaction costs one erase: at most about half of the upstream per-cycle erase at that
size, so at 400k poses roughly 2 ms on x86 and 6.5 ms on the Pi 5 (derived from the measured
upstream erase, not measured separately), once per halving of the remaining plan. With
`enforce_path_inversion` or `enforce_path_rotation` enabled the handler compacts every cycle, so
those modes keep upstream's cost; the shipped configuration enables neither.

**What it does not fix.** The remainder is `calculate_path_length` in controller_server's tracking
feedback. It is upstream code, and no plugin can reach it (see "Upstream follow-ups").

**The one-off transport costs are real, but much smaller in total.** They are listed below with
numbers, as proposals. Only the BT move (proposal 3) is implemented, partly; the
others change an interface or a topic's semantics.

## How big are the paths? (e2e, real `coverage_server`, shipped parameters)

Two real `coverage_server` processes and the e2e C++ client of the coverage-planner PR (#1114),
which is not part of this change, patched locally to print each field's serialized size
(`E2E_SIZES=1`, not included either). The fixtures are that PR's realistic lawns; columns 4–6 are
serialized sizes. The table is context for the per-cycle numbers, not something this change needs.

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
- FTC, the goal checker or FollowStrip;
- the BT, except that `PlanCoverageArea` moves the plan result into the BT context instead of
  copying it (see "Dead ends and what was not done", item 3);
- safety logic;
- the driven path. The planner and its output are untouched, and the controller sees the same
  transformed plan every cycle.

### Correctness evidence

**`test_lazy_prune_path_handler`** (9 gtests). The real upstream plugin and the lazy one run side
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
  least 4 times, and that the lazy path really keeps poses behind in more than half the cycles;
- a pivot corner with `enforce_path_rotation` on, the robot's yaw following the plan, across the
  rotation and into the next leg;
- an empty plan, and a robot pose in a frame TF does not know, followed by normal cycles: the same
  exceptions, and the same state afterwards;
- `reject_unit_path` with a one-pose plan.

**Mutation check** (run on the first six tests, before the last three were added). Three
plausible bugs were planted one at a time:
- searching from the physical begin — caught, 33 failures;
- corrupting the offset — caught, 15 failures;
- skipping the compaction before the constraint branch — caught, 8 failures.

A fourth mutant survived as predicted. It used the physical instead of the logical size in the
"keep two poses" check, which is equivalent (see the invariant above).

**Other checks:**
- `test_nav2_params.py`: 81 passed, including the new pin on the plugin in both overlays.
- `ament_lint_cmake`, `ament_xmllint`: clean.
- clang-format 18: applied.
- BT: `GetNextUnmowedAreaTest.SuccessfulPlanLandsOnTheContextIntact` serves a real result from a
  fake `plan_coverage` server and checks the context fields and transit goal; the plan/resume BT
  tests pass.
- The whole `mowgli_nav2_plugins` suite on top of `dev` (b5e46ed49): every test passes except
  `test_garden_costmap`, whose two `SelectiveClearInvalidatesAndRebuildsAllAuthorization` cases
  fail identically on an unmodified `dev` built the same way (18 of 20 cases pass there). It is
  unrelated to this change. The differential test was re-run 15 times pinned to one core after its
  fixture was made robust (Nav2's `Costmap2DROS::cleanup()` can hang when it cancels the executor
  thread before that thread has started spinning).
- Not run here: `ament_cppcheck` (the host's cppcheck 2.19 is skipped by ament).

## Method and reproduction

- **Interleaved A/B.** 21 alternating runs per handler of
  `cycle_bench server 200 <plugin>` on core 5; median per size, with a bootstrap 95 % CI
  (4000 resamples) of the median ratio.
- **Instruction counts.** callgrind from the distro 3.26 `.deb`, extracted without root,
  `--toggle-collect` on `*PathHandler::findPlanSegment*` and `*PathHandler::transformLocalPlan*`.
- **Builds.** colcon in a scratch workspace (`<ws>`), with the Nav2 `.deb`s extracted into a scratch
  root. `perf` is blocked on this host.

```sh
# packages (Release): mowgli_interfaces, mowgli_coverage, mowgli_nav2_plugins
colcon build --base-paths ros2/src/mowgli_interfaces ros2/src/mowgli_coverage \
  ros2/src/mowgli_nav2_plugins --build-base <ws>/build --install-base <ws>/install \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release
source <ws>/install/setup.bash
export ROS_DOMAIN_ID=173 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# per-cycle and transport benchmarks
cmake -S ros2/src/mowgli_nav2_plugins/test/bench -B <ws>/bench && cmake --build <ws>/bench -j2
taskset -c 5 <ws>/bench/cycle_bench server 200 nav2_controller::FeasiblePathHandler
taskset -c 5 <ws>/bench/cycle_bench server 200 mowgli_nav2_plugins/LazyPrunePathHandler
taskset -c 5 <ws>/bench/cycle_bench ftc 200; taskset -c 5 <ws>/bench/cycle_bench checker 200
taskset -c 5 <ws>/bench/path_transport_bench serialize
export CYCLONEDDS_URI=file://$PWD/docker/config/cyclonedds.xml
taskset -c 5 <ws>/bench/path_transport_bench sub & taskset -c 5 <ws>/bench/path_transport_bench pub 5
# differential test
colcon test --packages-select mowgli_nav2_plugins --ctest-args -R test_lazy_prune_path_handler
#   (here the gtest binary was run directly from <ws>/build/mowgli_nav2_plugins/, core 5)
python3 -m pytest -q ros2/src/mowgli_bringup/test/test_nav2_params.py
# field sizes: needs the e2e harness of #1114 plus the local E2E_SIZES patch (not in this change)
```

## Measured on the robot (Raspberry Pi 5, 2026-11-10)

Raspberry Pi 5 (4 cores, 8 GB, 2.4 GHz, `ondemand`), aarch64, the production stack running. The
benchmark was built and run in a separate throw-away container made from the robot's own image
`mowgli-ros2:pr991-58dcfca9-protocol7` (Nav2 and headers from the image, `mowgli_nav2_plugins`
rebuilt from the source of this change as it stood before rebasing onto `dev`: commit `5841a0488`
of the local branch, same plugin code): no network, `nice -n 19`, pinned to core 3, build in a
tmpfs. No running container, volume or file of the robot was touched. Same `cycle_bench` as on
x86; 3 interleaved repetitions of 200 cycles, median of the three shown. The raw output was not
kept; the table is transcribed from it.

`server` mode, µs per control cycle (`controller_server` path bookkeeping):

| poses ahead | upstream handler | upstream total | lazy handler | lazy total | total, x faster |
|---|---|---|---|---|---|
| 50k | 1247 | 2111 | 16 | 619 | 3.4 |
| 100k | 2952 | 5035 | 18 | 1334 | 3.8 |
| 200k | 6232 | 10052 | 20 | 2755 | 3.6 |
| 400k | 12984 | 19911 | 20 | 5492 | 3.6 |

- Against the 100 ms cycle: upstream is ~2 % at 50k and ~20 % at 400k; lazy is ~0.6 % and ~5.5 %.
  At 400k that is ~14 ms of every cycle saved, i.e. about 14 % of one core for as long as the
  sub-path is that long.
- Versus the x86 core: the handler is ~3.3× slower (13.0 vs 3.95 ms), `calculate_path_length`
  ~2.2× (6.7 vs 3.1 ms). That matches the memory-bandwidth argument. The lazy handler stays flat (10–20 µs).
- Flat in path length: FTC ~9.9 ms per cycle (x86 8.2 ms); the goal checker 0.13–0.32 µs.
- One-off at 400k: FTC `newPathReceived` 248 ms (x86 242 ms), `setplan` 45–79 ms, CDR serialize
  138 ms, deserialize 89 ms, deep copy 16 ms.

**Caveat on the noise.** `mowgli-ros2` was restarted (an explicit `docker restart`, not issued by
the benchmark) 20 s before the benchmark started, so the stack was booting on the other cores and the load average was
~9 (part of it a concurrent benchmark build). The three repetitions agree (lazy 5.49 / 5.50 / 5.49 ms at 400k;
upstream 20.7 / 19.4 / 19.9 ms), but the absolute numbers may be pessimistic. The 10k upstream rep
1 was 468 µs against 205 µs in rep 3, which is the only visible outlier. A rerun on an idle robot
would tighten them; it was not done. Throttling flags show undervoltage has occurred since boot
(`throttled=0x50000`), none at the time.

Upstream per-repetition totals at 400k (µs): 20653, 19386, 19911; lazy: 5490, 5502, 5492. At 50k:
upstream 3015, 2102, 2111; lazy 843, 619, 615.

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
   - **Half done in this change:** `PlanCoverageArea` now moves `segments`, `full_path` and
     `drivable_subpaths` out of the action result, and everything after reads the context. The
     result is held by the goal handle until the next plan, so the copy kept every pose twice
     for the whole session. The short-lived `units` copy for the resume fingerprint is left as is.
     Memory saved was not measured.
   - Estimated saving: ~38–115 MB of BT memory held for the session and a few tens of ms per plan
     at 400k poses. This is analysis from sizes only (`sizeof(PoseStamped)` = 96 B).
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
- **Platforms.** The per-cycle benchmark ran on a Pi 5 (above); the gtests were run on x86 only.
  The code is plain C++ with no intrinsics, and the arithmetic is upstream's, unchanged.
- **Transit lane.** The transit lane (RPP) uses the same handler. Its paths are short, and its
  results are identical.

## Scope and acceptance

This change is independent of the coverage-planner PR (#1114): it touches `mowgli_nav2_plugins`,
`PlanCoverageArea` (one move), one line of `nav2_params_base.yaml`, `test_nav2_params.py` and docs.
The e2e-harness tweaks (`E2E_SIZES`, overridable `ROS_DOMAIN_ID`) belong to that PR's files and are
not included.

A maintainer may reasonably prefer to fix this in Nav2 itself and avoid carrying a subclass
(upstream issue: ros-navigation/navigation2#6759). The differential test then still serves as the
check before switching back.

**Not tested:** a mow under the real stack with the new handler. The controller's inputs and
outputs are unchanged, so no motion test is needed to judge the code, but nothing here shows the
CPU saving during a real mow. Rollback is one line: set `PathHandler.plugin` in
`nav2_params_base.yaml` back to `nav2_controller::FeasiblePathHandler`. A useful first mow: compare
`controller_server` CPU on a sub-path of 100k+ poses with and without the handler.
