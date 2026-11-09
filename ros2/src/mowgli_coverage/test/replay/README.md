# Exact coverage planner replay (#823, #925)

This harness compiles `coverage_planning.cpp` itself against the real pinned
Fields2Cover library. It does not instantiate ROS nodes, send action goals,
connect to a robot, or read/write mission state. The optional `PlanningTrace`
records geometry without changing search, ordering, connector or safety decisions.
Normal planner calls do not request a trace.

## Build and run

Use a normal project user. The repository pin is F2C 3.0.0 at
`884d895b59192882476e986ba44ea9143a06a6a9`, built by `ros2/Dockerfile`.
The standalone build needs its normal development dependencies, plus GTest,
nlohmann-json, Python 3 and optionally `/usr/bin/time`. No Python packages are needed.

In an environment containing the pinned library:

```sh
source /opt/ros/lyrical/setup.bash
export LD_LIBRARY_PATH=/opt/fields2cover-300/lib:${LD_LIBRARY_PATH:-}
cmake -S ros2/src/mowgli_coverage/test/replay -B /tmp/coverage-release \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/opt/fields2cover-300
cmake --build /tmp/coverage-release --parallel 2
ctest --test-dir /tmp/coverage-release --output-on-failure
python3 ros2/src/mowgli_coverage/test/replay/run_replay.py \
  --binary /tmp/coverage-release/coverage_replay \
  --same-process 5 --processes 3 --output /tmp/coverage-results
```

Build types `Debug` and `RelWithDebInfo` use the same commands with separate build
directories. The normal package's ament suite runs `test_planning_trace`, the
Python runner/provenance tests and a 26-fixture replay matrix with two repetitions in each
of two fresh processes (104 replay runs). Larger stress runs use the commands
above. These are registered checks, not a statement that they have passed.
The standalone suite builds all five existing planner test files unchanged.

For an isolated container with no network or robot/service mounts:

```sh
docker build --target fields2cover-v3-builder -f ros2/Dockerfile \
  -t mowgli-f2c-replay:884d895 .
docker build -f ros2/src/mowgli_coverage/test/replay/Dockerfile \
  -t mowgli-planner-replay ros2/src/mowgli_coverage/test/replay
mkdir -p /tmp/coverage-work
# Mount the checkout read-only and give the non-root container user a writable
# output directory. Ownership should match your normal project UID/GID.
docker run --rm --network none --cpus 2 --memory 2g \
  --user "$(id -u):$(id -g)" -v "$PWD:/repo:ro" \
  -v /tmp/coverage-work:/work mowgli-planner-replay bash
```

Inside the container use `/repo/ros2/src/mowgli_coverage/test/replay` as the source
and `/work/build-release` and `/work/results` as the build/output directories.
The test-tool image installs packages as root; all compilation/testing is non-root.

## Input and output contract

The runner writes each complete JSON fixture before running the planner. Keep
that exact file, not rounded area/bbox/perimeter logs. Schema 1 has:

- `precision`: `float32` mirrors `geometry_msgs/Polygon` transport, `float64`
  isolates precision sensitivity. Transport conversion occurs before sanitization.
- `polygons`: list of polygons; each contains an outer ring followed by holes,
  each ring a list of `[x,y]` points in map-frame metres. Multiple polygons are
  planned as independent area requests. The thin-neck fixture separately tests
  several mainland cells within **one** planner request.
- `parameters`: all effective planner/connector knobs, in metres/radians, including
  obstacle growth, pivot sweep geometry, headland-turn limit, winding,
  perpendicular mode and optional start hint. Defaults in `run_replay.py` are
  fixture choices, not evidence of the #823 incident's configuration.

For a new report, start with one generated fixture and replace its geometry and
parameters with the exact transported goal and effective server values:

```sh
python3 ros2/src/mowgli_coverage/test/replay/run_replay.py \
  --binary /tmp/coverage-release/coverage_replay --fixture /path/to/report.json \
  --same-process 20 --processes 5 --output /tmp/report-replay
```

The runner compares exact IEEE-754 double bit patterns, structure and order. It
checks input transport/sanitization, safe cells, ring geometry, mainland cells,
actual per-cell AUTO argmins, raw/even/ordered swaths, connected subpaths before
final ordering, ordered execution geometry and server-equivalent quaternion
conversion. Stage names carry polygon and mainland-cell indices. A divergence
saves both snapshots and its first differing JSON location. Timing notes are
excluded. Full snapshots are compressed; summaries contain SHA-256 stage/output
hashes, pose/swath/ring/transit counts, path/endpoint-link lengths, connector
outcomes, planner/connector times and process peak RSS.
Use a fresh output directory for each comparison. A timeout or process failure
returns nonzero and may leave a partial report; never treat it as a completed run.

The diagnostic input hash binds transported and normalized geometry, all supplied
parameters and recorded build provenance. It is **not used by production**.
`followstrip_fingerprint` mirrors the existing position-only millimetre FNV hash;
the exact output hash additionally includes headings and subpath boundaries.
Changed-width, angle and geometry controls must change both input identity and
the existing fingerprint. Not every parameter change changes execution geometry,
and not every submillimetre change changes that quantized fingerprint.

## Baseline and performance comparison

Build the same harness against a separately extracted baseline implementation
and matching headers using `-DPLANNER_SOURCE=... -DPLANNER_INCLUDE=...`
and `-DREPLAY_BASELINE=ON`. If the baseline tests differ, also supply
`-DPLANNER_TEST_ROOT=...` with that revision's test sources and relative fixtures.
Then run baseline and candidate with `--no-trace`
using the same fixtures, dependency library, compiler and build type. This avoids
charging snapshot allocation to ordinary planner performance. Compare exact
output hashes, fingerprints and geometry metrics before interpreting timing/RSS.
RSS includes the process and JSON output; endpoint links are straight-line
distances, **not Nav2 routes or measured blade-off robot travel**.
For source archives, supply `-DREPLAY_REVISION=<hex commit>`; the build refuses
to invent a revision when Git metadata is absent. Source/header hashes and Git
revision refresh at build time, including incremental and packed-ref updates.

The dedicated `Coverage planner replay` workflow runs Release, Debug and
RelWithDebInfo on changes to this package (or by manual dispatch). It extracts
the tested synthetic merge's base parent (validated against the PR head), with
matching planner/header/tests/fixtures, freezes the 26 fixture files once and feeds
them to both implementations. Each configuration runs 15 untraced repetitions
per fixture for each implementation and 100 traced candidate repetitions per
fixture: 3,380 replay records, plus the standalone tests. Multi-polygon records
contain more than one field-planning call.

Artifacts named `coverage-planner-replay-<build type>` retain the exact fixtures,
compressed snapshots, summaries, comparison JSON/Markdown, build logs/flags,
dependency versions and library hashes for 14 days. Baseline and candidate must
match exact execution output and geometry metrics; traced execution coordinates
and quaternions must match the untraced candidate. Source identities are allowed
to differ. Runtime/RSS measurements are reported rather than gated by a noisy
percentage threshold. The job uses the existing pinned dependency cache; on a
cache hit its original compiler flags are unavailable and are reported as such.
The comparison verifies summaries against retained fixture/snapshot hashes and
rejects missing, null or invalid evidence. Its nine rejection/passivity tests
run in both the package and standalone suites.

To compare downloaded artifacts locally:

```sh
python3 ros2/src/mowgli_coverage/test/replay/compare_replay.py \
  --baseline-dir Release/results/baseline-no-trace \
  --candidate-dir Release/results/candidate-no-trace \
  --trace-dir Release/results/candidate-trace \
  --environment Release/environment/environment.json \
  --output-json /tmp/comparison.json --output-markdown /tmp/comparison.md \
  --require-identical
```

Record the F2C source commit, actual compiler flags (the pinned CMake
`ALLOW_PARALLELIZATION=ON` option alone does not define the C++ macro), library
SHA-256, compiler, GDAL/GEOS versions, architecture and repo/source hashes with
results. A version string alone cannot establish dependency provenance.

## Remaining acceptance evidence

The original #823 70-point boundary and effective parameters are unavailable in
the issues/repository/local report artifacts. Neither the 15-point recorded lawn,
the Isabey export nor the synthetic fixtures reproduce that incident. Repeated
deterministic results cannot prove all possible geometry/builds deterministic.
There is no evidence-backed production workaround in this change: no angle cache,
geometry canonicalization, changed mowing strategy or fingerprint bypass.

Integration with #718 may capture schema-1 inputs at the server's ingestion seam;
capture must include exact goal point values, all live effective parameters and
version/revision metadata. `coverage_server.cpp` is left to its current owner.
Charge/restart persistence, accepted plan retention, progress reset/remapping and
semantic completion remain #925/#931/#932 responsibilities. Planner determinism
alone does not settle them.

See [INVESTIGATION.md](INVESTIGATION.md) for the incident evidence, dependency
findings and baseline-specific validation evidence.

Physical charge/resume acceptance is **HARDWARE_REQUIRED**. The incident's exact
repo/image, firmware, submodule gitlinks, robot unit and receiver/driver baseline
are unknown. First capture that baseline and the exact replay input. On the same
robot with matching software/configuration, perform a supervised blade-disabled
plan/interruption/dock/resume and then a charge/restart replay. Pass requires
matching unchanged-input fingerprints and resuming at the saved cursor with
completed headlands retained; a changed-geometry/config control must reject stale
positional cursors. An operator and emergency stop must be present, the field
clear, localization/drive healthy, and normal firmware, collision and boundary
protections enabled. This harness provides software evidence only.

## Controlled floating-point replay

On an FMA-capable x86_64 host, use the same pinned library, exact fixture files
and byte-matching planner source/header for all variants. The 2026-10-12 experiment
used audited merge `6f9255585d5d531a101f29a2d3453c5290a403e3` (see the investigation's
source/header/library hashes). Extracting it to a separate directory keeps the
current checkout untouched:

```sh
mkdir -p /tmp/coverage-audited
git -c core.autocrlf=false archive 6f9255585d5d531a101f29a2d3453c5290a403e3 \
  ros2/src/mowgli_coverage ros2/src/mowgli_interfaces/include \
  | tar -x -C /tmp/coverage-audited
for variant in default fma contract-off; do
  flags=''
  case "$variant" in
    fma) flags='-mfma' ;;
    contract-off) flags='-mfma -ffp-contract=off' ;;
  esac
  cmake -S /tmp/coverage-audited/ros2/src/mowgli_coverage/test/replay \
    -B "/tmp/coverage-$variant" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/opt/fields2cover-300 -DCMAKE_CXX_FLAGS="$flags" \
    -DREPLAY_REVISION=6f9255585d5d531a101f29a2d3453c5290a403e3
  cmake --build "/tmp/coverage-$variant" --target coverage_replay --parallel 2
  python3 ros2/src/mowgli_coverage/test/replay/run_replay.py \
    --binary "/tmp/coverage-$variant/coverage_replay" --fixture /path/to/exact-report.json \
    --same-process 5 --processes 3 --output "/tmp/results-$variant"
done
```

Compare exact execution snapshots with matching fixtures; ignore build/timing
fields only for this controlled numerical comparison. The local 26-fixture
experiment found default and contraction-off outputs identical (26/26), while
FMA-on differed (26/26). Only 2/26 matched each archived CI counterpart; the local
dependency binary also had a different hash, so that comparison does not isolate
contraction. This is a demonstrated mechanism, not a confirmed CI or
original-incident cause.

Current standalone GNU builds use `-frecord-gcc-switches` for diagnostics only.
New workflow artifacts retain `coverage_replay`, `libreplay_planner.a`,
`effective-gcc-switches.txt`, disassembly, binary hashes, linked-library paths,
CPU information and the actual F2C shared library. Replay build fields also
record a SHA-256 of requested global/build-type C++ flags and compiler
FMA/AVX2/fast-math/evaluation macros. Requested flags and compiler macros do not
establish every dependency's effective flags; consult the retained binaries.
All identities remain offline diagnostics, with production resume validation
unchanged.
