# Authorized transit planner performance

The fixed garden-sized costmap and authorization contract in
[TRANSIT_AUTHORIZATION.md](TRANSIT_AUTHORIZATION.md) remain intact. These local
measurements do not establish Raspberry Pi 4 latency or explain the reported
robot backoff. Execution authorization remains tracked in
[#924](https://github.com/mowglinext/mowglinext/issues/924).

## Bottlenecks and retained changes

The original planner classified every grid cell against every polygon edge.
On a 1,562,500-cell garden with 750 boundary vertices, authorization preparation
took a median 2,132 ms in the initial local baseline. Row-intersection rasterization
with exact boundary-distance passes reduced that phase to 12.7 ms. Differential
regressions compare the new mask with the original classifier, including epsilon
seams, obstacle margins, translated coordinates and collapsed cell centres.

Search repeatedly copied each obstacle ring into a temporary vector and allocated
polygon/grid crossing buffers. Per-plan scratch buffers and a one-ring span
eliminate these copies. A separate allocation-instrumented run counted 338,173
to 100,019 main-thread `operator new` calls over 30 small-map plans, including
setup and post-plan validation. This is not a count of all process allocations.

Conservative bounding rejection skips far-away obstacle distance calculations
only after the original endpoint and exact crossing checks. An isolated same-core
comparison with 32 drawn obstacles measured median 2,477 to 1,235 ms and p95
2,734 to 1,468 ms. Continuous crossings and obstacle clearance remain checked.

A per-plan, negative-only connectivity preflight rejects separated polygon
bounds before rasterization and A*. Each ring contributes its existing boundary
tolerance. Overlapping bounds still require exact search and current obstacle
checks. No cache or verdict survives a plan or geometry invalidation.

`GridBased.max_planning_time` stays **8.0 seconds**. `BoundaryRecovery` and
costmap-layer update/clearing paths are unchanged. The full private costmap copy
was below 0.2 ms in the initial baseline. Full layer updates remain necessary to
remove expired marks and their inflation.

## Final paired comparison

Measured baseline algorithm: dev `9a91584d`, with profiling and the benchmark only.
Measured candidate: the profiling commit and four optimizations, clean-built with the final
seam regression verified before rebasing onto dev `49641786`. Candidate planner,
geometry, raster and test sources are byte-identical in the rebased runtime
`28a7bb13`; the only upstream plugin-package change was duplicate CMake entries.
This comparison does not measure the newer upstream motion/coverage changes.
Environment: Intel Core i7-10700K, x86_64, Docker Desktop/WSL2, ROS Lyrical,
GCC 15.2.0, Release/C++20, UID 1000. The container was limited to two CPUs,
4 GiB memory and no swap for both benchmark revisions.
Both revisions used image
`sha256:be2f6e79c62ec7a7ac7e36978ad2101ccf422c61993a0fee72dc8a9ac944833d`
and the same installed vendor dependencies. The repository's Nav2 source pin
is `a6354f3f39d12c6e5c3e323f8021c831752edb23`; installed `node_2d.hpp` SHA-256
is `6528540c65321d3e4fa8baa25a50cff967191e027f3ecbd46542ac70bee4d24e`.
The disposable container also installed `libhdf5-openmpi-dev` for exported
Smac dependency headers and GNU `time`; builds and tools ran as UID 1000.

Each row is 30 original plans followed by 30 candidate plans on CPU 14, with
identical geometry, endpoints and costs. Times below are **milliseconds**.
Median is the mean of samples 15/16; p95 is sorted sample 29/30.

| Case (cells) | Median before → after | p95 before → after | Maximum before → after |
|---|---:|---:|---:|
| Small (25,600) | 1.75 → 1.73 | 2.97 → 3.13 | 6.30 → 6.05 |
| Medium (390,625) | 13.29 → 8.67 | 18.85 → 13.27 | 20.40 → 13.27 |
| Large (1,562,500) | 42.51 → 23.68 | 53.71 → 35.13 | 53.98 → 37.26 |
| 750 vertices (1,562,500) | 2,387.82 → 78.53 | 3,022.35 → 114.47 | 3,532.53 → 116.33 |
| Long detour (1,375,000) | 1,329.24 → 1,382.58 | 2,127.15 → 2,128.07 | 2,619.98 → 2,400.09 |
| Disconnected (1,375,000) | 1,443.05 → 0.096 | 1,673.02 → 0.570 | 1,729.84 → 0.636 |
| 32 obstacles (390,625) | 3,544.90 → 1,646.63 | 4,946.85 → 2,406.98 | 5,094.06 → 3,111.43 |

The shared host showed scheduling variation. Small-map latency is comparable;
the slightly higher p95 does not establish a dedicated-machine tail bound.
The detour and obstacle-heavy cases remain above the sub-second target locally.
The detour is still A*-bound; the primary comparison does not show a latency
improvement for it. Targeted repeated pairs are recorded separately below.

Two additional 30-plan pairs for the detour gave median/p95/maximum:
1,839/2,164/2,286 → 1,751/2,683/2,771 ms, then
1,846/2,954/3,020 → 1,458/2,592/2,712 ms. Together with the primary pair,
these do not show a consistent detour regression or establish a tail-latency
improvement. The retained allocation changes reduce measured allocation count;
the strong latency gains are authorization preparation, obstacle-heavy search
and clearly disconnected areas. Pi measurements under sensor load remain required.

Median phase times for the final pair:

| Case | Snapshot | Authorization | A* search | Raw validation | Smoothing/validation |
|---|---:|---:|---:|---:|---:|
| 750 vertices, original | 0.169 | 2,333.055 | 76.449 | 3.946 | 4.131 |
| 750 vertices, candidate | 0.174 | 9.192 | 61.150 | 3.879 | 4.062 |
| Detour, original | 0.126 | 35.151 | 1,291.495 | 0.693 | 0.908 |
| Detour, candidate | 0.142 | 8.369 | 1,372.005 | 0.841 | 1.015 |
| 32 obstacles, original | 0.043 | 550.602 | 2,988.002 | 6.614 | 4.918 |
| 32 obstacles, candidate | 0.045 | 2.928 | 1,633.787 | 4.031 | 4.096 |

For disconnected polygons, candidate connectivity took 0.006 ms and performed
zero A* edge checks, instead of a median 2,103,753.5 checks. All other cases
retained identical edge counts and smoothing acceptance: 30/30 for the first
four cases, 15/30 for the detour, 0/30 for obstacles. Both directions retain
their success/failure and continuous-validity outcomes.

Summed process user CPU over all seven cases was **271.07 → 101.35 seconds**;
system CPU was **0.31 → 0.39 seconds**. These totals include setup, logging and
post-plan validation, unlike the planning latency table. Peak process RSS was
**111,836 → 113,656 KiB**, both at the detour; no swaps were reported. The new
temporary mask costs one byte per cell (1.56 MB for the largest test, at most
4 MB within the default garden budget), plus row crossings and reused scratch.
Nothing persists as an authorization cache.
GNU `time` reported 99% CPU for both detour and obstacle runs (approximately
one busy core); the original 750-vertex run was 100%, the candidate 90% including
process startup. The measurements quantify work reduction, not idle-system
capacity under concurrent sensing or localization.

Final plugin validation: **23/23 CTest targets**, including **300 enabled gtest cases**
(301 registered, one opt-in benchmark disabled),
the 14 layered-costmap integration cases with/without LiDAR, and package linters.
The 28 enabled authorization tests cover masks, gaps/seams, concavity, diagonal cuts,
obstacle margins/current costs, terminals/dock, invalidation/replacement,
cancellation and budget expiry. The prompt-disconnection regression fails with
the original plan function and passes with the candidate. Configuration tests:
**91 passed**, with structural configuration drift checks passing.

The complete CI package set built successfully (14 packages); all 13 packages
with registered tests passed. `colcon test-result` reported 3,304 test records,
zero errors/failures and 387 skipped records; this includes CTest/gtest wrapper
records, rather than 3,304 distinct executed cases. This validation used an
isolated Linux source copy, with LF `main_tree.xml` test data and the official
ROS package schema supplied through a local XML catalog. No linter was disabled.
The Windows checkout was preserved. `libmosquitto-dev` was installed only for
this full-workspace build, after the paired performance measurements.

## Reproduction

The opt-in `DISABLED_RepresentativePlanningBenchmark` exercises the real plugin
and pinned Smac search, with 30 plans per case alternating direction. Inputs
cover small/medium/large gardens, 750 vertices, a long navigation detour,
disconnected mowing areas and 32 obstacles. Successful paths undergo continuous
geometry/cost checks after each timed call; disconnected
plans must throw `NoValidPathCouldBeFound`.
Separate differential regressions compare raster masks and obstacle segment
clearance with the original implementations.

Build Release as the normal project user with the repository's ROS Lyrical vendor
dependencies. Run each case in a fresh process, serially on the same CPU, for both
revisions. Apply the profiling-only commit `fb86584a` to dev `9a91584d` to recreate
the measured baseline; an optional scenario selector in the harness is the only
additional baseline edit needed for per-case runs. Candidate source is available
at `28a7bb13`. Use matching headers, plugin library and
executable for each revision: mixing them can substitute template instantiations
and invalidate a comparison.

```bash
source /opt/ros/lyrical/setup.bash
source /opt/lyrical_vendor/local_setup.bash
source install/setup.bash # from the built ROS workspace
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=173
export MOWGLI_TRANSIT_BENCH_CASE=vertices750 # omit to run all seven
/usr/bin/time -v taskset -c 14 ./build/mowgli_nav2_plugins/test_authorized_transit \
  --gtest_also_run_disabled_tests \
  --gtest_filter='*RepresentativePlanningBenchmark'
```

The benchmark sets a **60-second test-only budget** to measure the original
algorithm without censoring slow cases. Production defaults are unchanged.
Resolution is 0.08 m, origin (-1.03, -1.03), substrate is free space with explicit
geometry obstacles, travel multiplier is 3, and unknown cells are forbidden.
There is no concurrent sensor/costmap update workload in these latency runs.
Debug logs report snapshot, anchors, connectivity, authorization, search,
raw validation, smoothing/validation, edge counts and smoothing acceptance.
They are suppressed at the default log level.

## Rejected experiment

Removing the full authorization mask preserved final validity but reduced
accepted smoothing on the long detour from 15/30 plans to 0/30 and increased raw
path fallback. That experiment was reverted; the retained raster produces the
original mask. No unsafe routing fallback, timeout increase or layer rewrite was
used to improve a benchmark.

## Raspberry Pi 4 and physical acceptance

**HARDWARE_REQUIRED.** Local tests do not establish Pi 4 performance, controller
tracking or recovery safety. Record exact baseline/candidate repository SHAs,
image digests, firmware binary hash, submodule gitlinks, robot unit, Pi RAM,
CPU governor, receiver/driver revisions, datum, configuration and map checksum.
Use identical saved inputs for both revisions. The Pi/robot unit, firmware and
receiver baseline have not been identified here; record them before this run.

First replay 30 plans in each direction on the Pi with motion physically
inhibited and blades removed. Cover the largest permitted grid, the detour,
narrow/touching boundaries, boundary terminals, dock anchors, disconnected areas,
drawn and sensor obstacles, geometry replacement, cancellation and timeout. Repeat
with and without LiDAR, including mark expiration and obstacle removal. Record
median/p95/max total and phase latency, CPU, peak RSS and swap. Record the
performance target separately: the benchmark's small/medium/large rectangular
gardens pass that target when p95 is below 1.0 second. Required pass criteria:
all valid plans finish within the unchanged 8-second budget, separated-bounds
cases return `NoValidPathCouldBeFound` with zero A* expansions,
authorization/clearance
checks pass, canceled/changed geometry emits no accepted stale result, and
expired obstacle costs are removed. Investigate misses rather than raise timeouts.

Then follow the blade-off, supervised procedure in
[TRANSIT_AUTHORIZATION.md](TRANSIT_AUTHORIZATION.md#physical-acceptance) on the same
recorded baseline. Verify emergency stop and collision protection, maintain an
exclusion zone, record actual motion and stop immediately on unexpected backoff
or protection failure. Plan replay alone does not validate robot motion.
