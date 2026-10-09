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
Python runner tests and a 26-fixture replay matrix with two repetitions in each
of two fresh processes (104 planning calls). Larger stress runs use the commands
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
and `-DREPLAY_BASELINE=ON`. Then run baseline and candidate with `--no-trace`
using the same fixtures, dependency library, compiler and build type. This avoids
charging snapshot allocation to ordinary planner performance. Compare exact
output hashes, fingerprints and geometry metrics before interpreting timing/RSS.
RSS includes the process and JSON output; endpoint links are straight-line
distances, **not Nav2 routes or measured blade-off robot travel**.

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
findings and the validation status of this Draft.

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
