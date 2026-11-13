# LiDAR deskew history cursor: measured local optimization

## Problem and result

The deskew node linearly searches the IMU history from its beginning for every valid LiDAR ray.
A normal 720-ray scan queries near the newest part of a 0.5 s, 100 Hz history, repeatedly walking
most of the same approximately 50 samples. Optional translation compensation repeats the search
in wheel-velocity history.

A scan-local cursor now retains the first matching interval. For nondecreasing ray times, entries
already passed cannot be the first match later, including for an unordered history. Decreasing or
NaN query times reset the search. Endpoint precedence, first-match behavior, interpolation
arithmetic, missing-data counters, range/intensity rebinning and freshness gates are preserved.
Both cursors are discarded at the end of each callback; callback serialization keeps histories
immutable during their lifetime. No work is moved into sensor callbacks or another thread.

Primary result: actual `ScanDeskewNode::on_scan` callback, **69.096 → 19.427 µs CPU per scan
(71.9% reduction, 95% CI 71.3–74.3%)**, including scan copy, interpolation, rebinning and publish.
Elapsed time is **70.138 → 19.515 µs (72.2%)**. CPU IQR is 63.715–72.937 µs before and
17.569–19.555 µs after. There are **2,000 calls per batch, 42,000 per arm per scenario**.
At 10 Hz the primary saving is approximately **0.50 ms of one CPU core per second on this host**.
The percentage is substantial for this callback; the absolute whole-system saving is modest.
Whole-robot CPU, scan-matching accuracy and physical behavior are unmeasured.

| Scenario | Baseline median [µs] | Candidate median [µs] | CPU reduction | Paired 95% CI |
|---|---:|---:|---:|---:|
| rotation | 69.096 | 19.427 | 71.9% | 71.3 to 74.3% |
| linear | 162.810 | 66.805 | 59.0% | 56.0 to 62.9% |
| sparse | 26.551 | 8.982 | 66.2% | 64.5 to 67.9% |
| clockwise | 67.968 | 19.575 | 71.2% | 70.2 to 73.0% |
| reverse_time | 67.807 | 65.808 | 2.9% | -19.5 to 9.9% |
| start | 67.731 | 18.921 | 72.1% | 70.4 to 76.5% |

## Independent verification and inspectable measurements

An independent reviewer rebuilt both node arms and all three interpolation tests, verified the
baseline and recomputed the original statistics exactly. Nine fresh paired seeds (41000–41008),
2,000 callbacks per arm and scene, reproduce the rotation primary metric at
**41.057 → 11.590 µs CPU (-71.8%, paired 95% CI 70.9–72.9%)**. Every fresh range/intensity bit
and diagnostic counter matches, including the remaining five node benchmark scenarios.

The accompanying [paired timing data](2026-11-13-lidar-deskew-paired.csv) includes every original
and independent batch total, call count, seed and result digest. Divide totals by calls before
taking medians; preserve pairs and keep the two campaigns separate. Digests cover all range and
intensity bits plus diagnostic counters. The standalone benchmark emits those fields directly.

## Correctness and tradeoffs

The benchmark includes the actual pristine/candidate node source. Access macros expose private
members only in the benchmark translation unit; the installed production node is built normally.
Deterministic timestamps and a benchmark-only freshness limit prevent samples from ageing out
mid-measurement. Histories are populated through the node's real sensor callbacks. Both arms use
720 rays, the same approximately 50-sample histories and identical input arrays, including invalid
rays. Publish is timed; a real local subscriber captures output outside the timing window.
DDS uses a private domain (97), CycloneDDS and loopback only. No production parameter file is read
or altered. The optional benchmark is excluded from ordinary builds and is never installed.

All 126 measured node pairs have identical output range bits, intensity bits and diagnostic
counters. A further **111 held-out node pairs** cover seeds 30001/31007/32003/33001, ray counts
0/1/2/361/1080/1440, duplicate or unordered timestamps, empty histories, missing/stale IMU or wheel
input, zero scan increments, short/empty intensity arrays and reverse-time scans.
Three focused unit tests perform **2,160,094 scalar-oracle interpolation comparisons**, including
backwards/shuffled queries, duplicated/unsorted/NaN timestamps, endpoints and a new scan after
buffer trimming. The expected arithmetic is frozen from unchanged dev 6445b2c01.

Reverse-time queries intentionally restart the scalar search. The median change is 2.9% faster,
with a broad interval (-19.5 to 9.9%); no reliable gain or strict regression is established there.
Callgrind 3.26 counts **710,063 → 150,122 instructions per complete rotation callback
(-78.9%)** at seed 20001. Reverse-time callbacks increase from 711,340 to 717,668 instructions
(+0.89%), consistent with retaining the scalar search plus a small reset check. This deterministic
comparison bounds the added work in the timing-inconclusive case. `instructions.json` and
`callgrind.*` preserve the run boundaries and raw counts.

No fast path assumes sorted sensor arrival. A binary-search replacement was rejected because it
would introduce that assumption. Caching ground-filter trig coefficients remains a distinct,
unimplemented candidate that needs its own full-callback profile.

All three interpolation tests also pass with AddressSanitizer and UndefinedBehaviorSanitizer.

The real package CMake targets `scan_deskew_node`, `test_deskew_history_cursor` and
`scan_deskew_bench` build locally in Release, and the registered CTest passes. The node target
explicitly receives its new helper include path. No full image or robot deployment was performed.

## Build and reproduce

From this branch's repository root in a sourced, dependency-complete, non-root ROS workspace,
create a pristine baseline worktree and build the **same benchmark source** against each arm:

```bash
repo=$PWD
baseline=$(mktemp -d)
git worktree add --detach "$baseline" 6445b2c014720124e68cd96395e44d1d78491522
base_build=$(mktemp -d)
candidate_build=$(mktemp -d)
bench="$repo/ros2/src/mowgli_localization/test/bench"
cmake -S "$bench" -B "$base_build" -DCMAKE_BUILD_TYPE=Release \
  -DMOWGLI_PACKAGE_ROOT="$baseline/ros2/src/mowgli_localization"
cmake -S "$bench" -B "$candidate_build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$base_build" --parallel 1
cmake --build "$candidate_build" --parallel 1
export ROS_DOMAIN_ID=98 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo"/></Interfaces><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>'
taskset -c 3 "$base_build/scan_deskew_bench" rotation 20000 2000 720
taskset -c 3 "$candidate_build/scan_deskew_bench" rotation 20000 2000 720
```

Run paired seeds 20000–20020 with alternating A/B then B/A order for the original campaign.
Each process prints total CPU/elapsed nanoseconds, calls and exact result fields. Compare those
fields before accepting a pair; use the same core and dependencies for both arms. The companion
CSV preserves all measured totals for checking the reported statistics without private host scripts.
The standalone targets need installed ROS/vendor dependencies, but neither a whole-workspace build
nor a simulator. No physical robot or complete-controller latency is established by this benchmark.

## Changed files in this changeset

- `ros2/src/mowgli_localization/src/scan_deskew_node.cpp`
- `ros2/src/mowgli_localization/include/mowgli_localization/deskew_history_cursor.hpp`
- `ros2/src/mowgli_localization/CMakeLists.txt`
- `ros2/src/mowgli_localization/test/test_deskew_history_cursor.cpp`
- `ros2/src/mowgli_localization/test/bench/scan_deskew_bench.cpp`
- `ros2/src/mowgli_localization/test/bench/CMakeLists.txt`
- `docs/performance/2026-11-13-lidar-deskew-paired.csv`
- `docs/performance/2026-11-13-lidar-deskew.md`

## Limits and next steps

The cursor is O(rays + history) for increasing queries and adds two small stack objects, with no
new persistent cache, allocations or library dependencies. Functional equivalence is measured
on this host, including the full node output; no Raspberry Pi 5 timing or hardware evidence is
claimed. A Pi-side scan replay under sensor load is the next validation step for practical impact.
For more system-level CPU savings, profile the complete scan-filter and localization pipeline,
including the existing corridor-index work, before selecting another optimization.

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
