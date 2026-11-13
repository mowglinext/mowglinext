# FTC footprint sampling: measured local optimization

## Problem and result

FTC repeatedly rasterizes the chassis during its collision and lateral-offset planning cycle.
The old sampler converts every candidate cell centre from grid to world, back to grid, then reads
its cost. On normal free/inflated lawn those conversions repeat thousands of times per solve even
though almost every cost is below the blocking threshold. This consumes the controller's cycle
budget and holds the existing costmap lock longer.

The candidate first validates that the cell-centre round trip is the identity on **every x and y**
in the bounding box. With no zone guard it then reads the existing cost bytes directly and computes
world coordinates and the original polygon test only for candidate blocked cells. Active guards
and any failed round trip retain the scalar implementation. There is no cross-cycle cache,
resolution change, skipped footprint cell, altered threshold or new costmap allocation.

Primary result: complete `LatticeSolver` construction plus `Solve`, clear-lawn scenario with
varied headings, **5.939 → 1.613 ms CPU per call (72.8% reduction, 95% CI 67.4–79.2%)**.
Elapsed time is **5.941 → 1.614 ms (72.8%)**. Baseline CPU IQR is 4.683–7.143 ms;
candidate IQR is 1.290–1.749 ms. There are **50 calls per batch, 1,050 per arm per scenario**.
This is the full production solver, including its unchanged DP and memoization, rather than an
isolated byte-read microbenchmark. Controller TF, PID, ROS scheduling and robot behavior remain
outside this timing scope.

| Scenario | Baseline median [ms] | Candidate median [ms] | CPU reduction | Paired 95% CI |
|---|---:|---:|---:|---:|
| clear | 5.939 | 1.613 | 72.8% | 67.4 to 79.2% |
| post | 4.574 | 1.231 | 73.1% | 70.7 to 77.0% |
| far_wall | 3.948 | 1.631 | 58.7% | 55.9 to 66.5% |
| near_wall | 0.981 | 0.428 | 56.4% | 53.3 to 61.8% |
| clutter | 2.371 | 0.593 | 75.0% | 51.0 to 78.2% |
| zone | 2.920 | 0.862 | 70.5% | 67.5 to 74.3% |
| fine | 21.041 | 3.242 | 84.6% | 82.2 to 87.5% |
| solid | 0.163 | 0.165 | -1.6% | -13.7 to 15.2% |
| unknown | 0.184 | 0.187 | -1.8% | -10.7 to 5.6% |
| inflated | 5.496 | 1.594 | 71.0% | 68.0 to 77.6% |

## Independent verification and inspectable measurements

An independent reviewer rebuilt both arms and all 104 FTC regression tests, verified the original
source/library provenance and recomputed all original statistics exactly. On nine new seeds
(41000–41008), 25 calls per arm and scene, the clear-lawn primary metric was
**4.402 → 1.096 ms CPU (-75.1%, paired 95% CI 74.1–76.8%)**. Every new solver result matched.

**Clutter gains depend on the solver outcome mix.** The original 21 clutter seeds include seven
quickly wedged solves; the independent nine include five. The latter mix gives only a 4.8% median
reduction (95% CI 2.0–75.5%), while its feasible solves retain roughly 74–75% savings. A 50% gain
across arbitrary clutter seeds is not established. Clear-lawn and other scenario rows must be
interpreted at their stated workload, rather than as a uniform robot-wide CPU reduction.

The accompanying [paired timing data](2026-11-13-ftc-footprint-paired.csv) contains every original
and independent batch total, call count, seed, feasibility and result digest. Divide totals by
calls before taking medians; preserve pairs and keep the two campaigns separate. Digests cover
feasible/level/horizon/cost/offset bits and match within every pair. The standalone benchmark below
emits those result fields directly for independent comparisons.

## Correctness and tradeoffs

Every measured solver result matches the pristine baseline: feasible/wedged, degradation level,
horizon length, cost bits and every offset bit. Synthetic scenarios use the footprint, costmap
size and nominal lattice settings already exercised by `test_ftc_lattice_solver.cpp`: 240×240 cells,
0.05 m, 0.05 m station spacing, ±1 m offset, 0.30 m lead, 0.50 m reaction, 1 m minimum horizon.
The fine-grid compatibility case uses 0.025 m. No scenario or seed count was reduced.

The new scalar-oracle tests compare **21,216 queries**, covering single lethal cells throughout
rotated/off-grid footprints, concave shapes, translated and collapsed-centre grids, all five tested
thresholds including zero, active/transformed/deny-all guards, inflation and unknown costs.
The oracle is frozen from dev 6445b2c01. Existing obstacle, lattice, offset, turn-fallback and
contour-bypass suites pass: **104 tests**, including the recorded 2026-09-22 hedge wedge,
level-3 bare-chassis contact, single-station and trailing-body cases. The 55 obstacle/parity tests
also pass with AddressSanitizer and UndefinedBehaviorSanitizer, including leak detection.

Fully lethal/unknown grids show a **1.6–1.8% median CPU increase (about 3 microseconds)** with broad
intervals crossing zero. The fast path adds per-axis validation before early hits; this small
local cost is retained in exchange for the repeated clear/sparse-grid savings. A strict timing
regression in those cases has not been established. No functional mismatch is accepted.

A separate callgrind 3.26 run, collecting inside `Solve` at seed 20001, counts
**53.241 M → 12.173 M instructions per call (-77.1%)** on clear lawn. Fully lethal
maps increase from 1.710 M to 1.719 M (+0.50%), quantifying the small validation overhead
without scheduler noise. `instructions.json` and `callgrind.*` preserve these measurements.

The initial callgrind profile placed roughly half the measured instructions in footprint sampling
and coordinate conversion; the alternative of incrementally reusing shrinking-horizon DP tables
was deprioritized because it targets a smaller share. Unchecked direct indexing was rejected:
large map origins can make `worldToMap(mapToWorld(...))` differ from the original index.
The retained implementation checks that property and falls back instead of assuming it.

## Build and reproduce

From this branch's repository root in a sourced, dependency-complete, non-root ROS workspace,
create a pristine baseline worktree and build the **same benchmark source** against each arm:

```bash
repo=$PWD
baseline=$(mktemp -d)
git worktree add --detach "$baseline" 6445b2c014720124e68cd96395e44d1d78491522
base_build=$(mktemp -d)
candidate_build=$(mktemp -d)
bench="$repo/ros2/src/mowgli_nav2_plugins/test/bench"
cmake -S "$bench" -B "$base_build" -DCMAKE_BUILD_TYPE=Release \
  -DMOWGLI_PACKAGE_ROOT="$baseline/ros2/src/mowgli_nav2_plugins"
cmake -S "$bench" -B "$candidate_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$base_build" --parallel 1
cmake --build "$candidate_build" --parallel 1
taskset -c 3 "$base_build/ftc_solver_bench" clear 20000 50
taskset -c 3 "$candidate_build/ftc_solver_bench" clear 20000 50
```

Run paired seeds 20000–20020 with alternating A/B then B/A order for the original campaign.
Each process prints total CPU/elapsed nanoseconds, calls and exact result fields. Compare those
fields before accepting a pair; use the same core and dependencies for both arms. The companion
CSV preserves all measured totals for checking the reported statistics without private host scripts.
The standalone targets need installed ROS/vendor dependencies, but neither a whole-workspace build
nor a simulator. No physical robot or complete-controller latency is established by this benchmark.

## Changed files in this changeset

- `ros2/src/mowgli_nav2_plugins/src/obstacle_deviation.cpp`
- `ros2/src/mowgli_nav2_plugins/CMakeLists.txt`
- `ros2/src/mowgli_nav2_plugins/test/footprint_reference.hpp`
- `ros2/src/mowgli_nav2_plugins/test/test_footprint_raster_parity.cpp`
- `ros2/src/mowgli_nav2_plugins/test/bench/ftc_solver_bench.cpp`
- `ros2/src/mowgli_nav2_plugins/test/bench/CMakeLists.txt`
- `docs/performance/2026-11-13-ftc-footprint-paired.csv`
- `docs/performance/2026-11-13-ftc-footprint.md`

## Limits and next steps

These are code-level and replay results on x86-64. Raspberry Pi 5 CPU and complete-controller
latency, a physical mow and changes in path tracking are unmeasured. The implementation uses
portable C++20, existing Nav2 data and constant additional stack state. Safety decisions remain
with the unchanged guard, lethal-cell and polygon rules.

Next worthwhile work: replay complete FTC cycles on a Pi 5 under sensor load; profile the
remaining `SpanBlock`/offset-slot and shrinking-horizon DP work before changing it; measure
contour/turn fallback latency separately using the existing recorded-costmap replay harness.

## Baseline and measurement

The baseline is unchanged upstream `dev` **6445b2c014720124e68cd96395e44d1d78491522**, fetched on 2026-11-13.
Pristine production sources were extracted with `git archive` / `git show`; both arms compile the
same benchmark source against their own production sources. No prior local optimization branch
was cherry-picked. Earlier coverage RingIndex/nearest-first, map export/clustering, fusion-graph,
path-pruning, corridor-index and garden-costmap work was inventoried and excluded as duplicate work.
The existing Mowgli Simulation Farm and virtual-garden research were inspected; no Webots run or
image build was started, and no running service, robot, map or installed configuration was changed.

Host: Kuffe, Intel i7-10700K, x86-64, Ubuntu, kernel 7.0.0-30-generic, GCC 15.2.0,
ROS Lyrical, `-O3 -DNDEBUG -std=c++20`, CPU 3, nice 19, existing powersave governor unchanged.
Vendor libraries were reused read-only. The host was shared with independent CPU-heavy jobs,
so these measurements establish a local comparison, rather than a Raspberry Pi latency bound.

There are **21 paired batches**, with seeds **20000–20020** withheld until implementation was frozen.
Order alternates A/B, B/A. Inputs, seeds, compiler flags, libraries and core are identical within
pairs. Timings exclude scenario generation, process/node startup and one warm-up. They include
all measured production-call work and allocations. `CLOCK_THREAD_CPUTIME_ID` measures calling-thread
CPU; `steady_clock` measures elapsed time. Output comparisons run before accepting each pair.
The unit of variation is the batch mean per call; the table reports medians across batches.
IQR is across batches. Confidence intervals resample pairs 10,000 times and compare the ratio of
medians; they are descriptive of this shared-host experiment. These are not per-call p95 numbers.

Local evidence: `~/.cache/kuffe-agent/mowgli-evolution-20261113/evidence/`.
`final-ab.jsonl` contains every batch and exact output bits; `final-summary.json` contains all
medians, IQRs and paired confidence intervals. `final-ab.py`, `build.py`, `build-deskew.py`, `run.py`,
`build-config.json` and each binary's `provenance.json` preserve the exact commands and dependency
paths. `manifest.json` records source/library hashes and baseline identity. Logs and build products
are intentionally outside version control.
