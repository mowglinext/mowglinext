# Automatic blackbox / flight recorder

Diagnostics → **Recordings** contains the automatic blackbox and the existing
manual full-rosbag recorder. Blackbox collection starts with the GUI backend,
independent of browser tabs. It only observes ROS telemetry; it publishes no
commands, calls no robot services and changes no robot state.

The default is enabled, 60 seconds of pre-event history and 15 seconds afterward.
Available memory, message rates and missing publishers can shorten that window.
The UI and snapshot metadata report actual observation coverage, rather than
claiming a complete requested window. A GUI/backend restart or power loss loses
unsaved history. Completed recordings survive in the existing shared maps volume.

## Architecture

The Go GUI backend reuses `RosProvider` and its Foxglove connection. Dedicated
subscriber mailboxes coalesce ordinary telemetry to 10 Hz; emergency and
diagnostic events are unthrottled. Existing wire-side decimation remains in place
for high-rate IMU/odometry/commands. The recorder adds no ROS package, DDS node,
container, service, dependency or firmware path.

`gui/pkg/blackbox` is a transport-independent collector with one bounded queue,
one rolling ring, one incident at a time, and at most one disk writer. Admission
takes a queue permit **before** copying a payload and never waits for collector,
configuration or storage locks. A full queue rejects the incoming observation;
the ring evicts its oldest observations at its byte/record/time limits. Capture
overflow preserves prehistory and drops additional observations with counters.
Repeated triggers join the active incident without extending its deadline.
There are at most eight bounded reasons; automatic incidents also have a global
cooldown. Manual save can bypass that cooldown, but cannot queue a second writer.

Capture records share immutable payloads with the rolling ring. Retention uses
Go's monotonic clock. Wall-clock receive time and monotonic offsets are exported,
and original ROS source timestamps remain inside telemetry. Receipt is not proof
of a new physical observation: missing/stale/repeated publisher stamps remain
inspectable and are never rewritten as acquisition timestamps.

Normal operation does not persist raw history or status polling. A trigger pins
available prehistory, collects the post-event window, then sends an immutable
capture to the disk worker. JSONL is written to an exclusive `.partial` file,
flushed and synced, then renamed atomically into the completed namespace. Only
completed names are listed. Restart removes recorder-owned partial files.
Writes, retention scans and fsync run outside the collector lock. Configuration
persistence is serialized with capture detection; failed settings saves leave
the previous configuration and history intact.
Startup with existing recordings and accepted retention changes apply pruning
asynchronously through the same single disk worker. Settings edits wait for
cleanup. Events during cleanup immediately pin one bounded RAM capture, preserving
the original trigger time and pre/post window; its disk write waits for cleanup
to finish. Additional events coalesce into that incident. Status changes from
`pruning` to `capturing` when an incident arrives; telemetry admission continues.
Pruning errors are visible and saving the same settings retries failed cleanup.

The existing session tracker, logs and other GUI features have their own writes;
the zero normal-history-write property is scoped to the blackbox, not the entire
GUI process.

## Recorded telemetry

The curated list follows `mow_session_monitor.py` and the provider topic map:

- raw map-frame fused odometry and covariance (`fusionRaw`), wheel odometry,
  wheel ticks and IMU;
- raw GNSS fix/covariance and receiver/RTK status;
- final commanded velocity and hardware-bridge applied velocity;
- hardware status (including blade command/RPM), power/charging state and emergency;
- high-level status, behavior-tree log, system/fusion diagnostics;
- dig events, localization mode and collision-monitor state.

`GET /api/tools/blackbox/status` lists these logical topic keys and last delivery
times. Missing topics are permitted; subscriptions wait for late publishers and
the existing bridge reconnect machinery handles publisher/container restarts.
Some backends do not produce every source. No raw scan, map, camera, planned path,
robot-description or complete configuration is captured. Those remain available
through the manual full-rosbag recorder; an extended blackbox profile is deferred.

## Automatic triggers

- Rising emergency activation/latch on `/hardware_bridge/emergency`, including
  its bounded producer reason. An already-active emergency on startup is captured
  with whatever history exists.
- A diagnostic status entering ERROR or STALE (`level >= 2`) in `/diagnostics`
  or `/fusion_graph/diagnostics`. Detector state is capped at 128 identities
  (topic, status name, hardware ID). Unknown sources beyond that cap are skipped
  and reported in `skipped_trigger_sources`, preventing repeated untracked errors
  from producing a capture storm.
- High-level emergency and entry into terminal failure states classified by the
  existing notification source: `DIG_OBSTRUCTION`, `NAV_TO_DOCK_FAILED`,
  `COVERAGE_FAILED_DOCKING`, `UNDOCK_FAILED`, `CHARGER_FAILED`,
  `CRITICAL_BATTERY_NAV_FAILED`.

Dig-event and collision telemetry are recorded, but do not directly trigger:
transient-local replay and normal obstacle avoidance should not create incidents
on their own. Localization degradation is captured if its producer reports a
diagnostic ERROR; no new threshold or safety decision is inferred here.
Unreported controller/navigation aborts need an existing producer event before
they can be supported. This PR does not modify those producers.

## Configuration and resource limits

Recorder settings are GUI feature configuration, stored at `blackbox.config` in
the existing bitcask database. They are independent of robot-operation YAML.
Basic controls are in Diagnostics; the API accepts the complete configuration.
Settings changes preserve admissible history and detector edge state. Disabling
clears volatile history and detaches the blackbox's subscriptions. Changes during
capture/write/pruning return HTTP 409; wait for completion. Saved files remain accessible.

- `enabled`: true by default.
- `pre_seconds`: 60, allowed 1–300; `post_seconds`: 15, allowed 1–60.
- `memory_bytes`: 16 MiB requested, allowed 4–64 MiB.
- `max_message_bytes`: 8192, allowed 256–16384; larger individual messages drop.
- `max_snapshots`: 20, allowed 1–200.
- `max_disk_bytes`: 256 MiB, allowed 4 MiB–4 GiB.
- `cooldown_seconds`: 60, allowed 1–3600, applied to automatic captures.
- `min_free_disk_bytes`: 32 MiB, allowed 0–1 GiB.

Linux reads actual `MemTotal`/`MemAvailable` and container cgroup v2/v1 memory
limits/usage. The effective budget is conservatively capped by 1/32 of total and
1/8 of available memory (with a 4 MiB floor). If probes are unavailable it caps
the requested budget at 8 MiB. No board model is used. The managed budget reserves
one quarter for fixed references/metadata/encoding, bounds the queue to 64
maximum-size payloads, then splits the remaining budget one third for history
and two thirds for the one capture. Each ring/capture also has a hard record cap
(32768 maximum), so tiny messages cannot create unbounded metadata.

At startup and every five seconds, known available memory below 32 MiB clears optional history
and pauses telemetry retention; trigger detection remains active. Recording
resumes when pressure clears. An incident already in progress keeps its bounded
capture; new incidents under pressure are metadata-only. The managed byte budget is **not** a process RSS/OOM guarantee: Go GC,
allocator slack, existing GUI/CDR/schema processing and upstream mailbox messages
have separate costs. Keep reserve memory for normal robot operation.

History capacity evictions and rejected/malformed/oversize observations have
separate counters; capacity evictions do not falsely count as lost pinned capture
records. A capture reports actual pre/post spans, lifetime rejected observations,
capture-local losses and interruption state. Status also reports the last
asynchronous snapshot-write duration. Coverage is a timestamp span, not
a guarantee that every topic delivered continuously for that span.

## Storage, export and failures

Default storage is `/ros2_ws/maps/blackbox` (override with `BLACKBOX_DIR` for
testing). It uses the same shared persistent volume as manual recordings, in a
separate namespace. Pruning only touches strictly named regular blackbox files;
manual rosbag directories are untouched. Earlier completed evidence is retained
until the replacement has been written, synced and atomically published. Pruning
then protects that new filename, including after a backward wall-clock adjustment.
`max_disk_bytes`/`max_snapshots` bound completed retention after cleanup. During
replacement there can be one additional partial or completed capture, individually
bounded by `max_disk_bytes`; transient managed disk use is at most twice that
limit when existing retention is satisfied. Startup or a reduced limit can briefly
exceed the new bound until asynchronous cleanup finishes. Disk reserve requires
space for the replacement without deleting old evidence; insufficient free space
rejects it and leaves earlier recordings downloadable.
If cleanup fails after publication, that complete snapshot is counted and remains
downloadable with a visible retention error. Once existing files exceed a limit,
further publication stops until cleanup or manual deletion restores the limit.
Retrying the same settings retries cleanup; repeated failures cannot accumulate
an unlimited sequence of additional recordings.
Disk reserve, serialization limit, write/flush/fsync errors become recorder status
errors; they never restart ROS or the robot. Only one writer can remain blocked.

Files are uncompressed `.jsonl` timelines, not replayable rosbag/MCAP. The first
line contains capture ID, trigger time/reasons, recorder configuration, available
GUI build revision/version, actual coverage and losses. Observation lines carry
topic, receive time, offset and JSON payload; the final summary marks completion.
Other component image/firmware revisions are not queried automatically and must
be recorded with the incident when comparing hardware baselines.

Endpoints reuse the existing `/api/tools` pattern:

- `GET /tools/blackbox/status` — status, sources, completed recordings.
- `PUT /tools/blackbox/config` — validated complete settings, bounded request body.
- `POST /tools/blackbox/save` — manual incident (HTTP 202; `accepted:false` when joined).
- `GET /tools/blackbox/download/:name` — streamed attachment, no whole-file allocation.
- `DELETE /tools/blackbox/:name` — completed recording removal.

Filename allowlisting, regular-file checks and `os.Root` containment guard file
operations. The existing GUI authentication/network-access model is unchanged.
No recorder upload or notification delivery is added. Exports include location
and diagnostic telemetry; inspect them before sharing. Full robot configuration
and credentials are deliberately not included.
Malformed/unavailable blackbox API responses remain local errors in its panel;
other Diagnostics controls continue working and polling can recover automatically.

`Recorder.Close(ctx)` marks an active snapshot interrupted and attempts to save
it; context expiry lets the caller stop waiting while the one writer finishes.
The existing GUI server has no graceful shutdown integration: a forced process
or container stop loses volatile/in-progress history; completed files survive.

## Reproducible validation

Run as the normal development user, never root:

```sh
cd gui
go test ./pkg/blackbox -count=1
go test ./pkg/api -run 'Blackbox|Rosbag|Topic' -count=1
go test ./pkg/providers -run 'RosProvider|RosSubscriber' -count=1
go test ./pkg/foxglove -count=1
go test ./...
go test -race ./pkg/blackbox ./pkg/api ./pkg/providers
go test ./pkg/blackbox -run '^$' -bench . -benchmem
go run ./cmd/blackbox-stress -duration 30m -hz 200 -enabled=false
go run ./cmd/blackbox-stress -duration 30m -hz 200 -dir /tmp/blackbox-recorder
go run ./cmd/blackbox-stress -duration 3m -hz 1000 -memory-mib 4
cd web
yarn install --frozen-lockfile
npx tsc --noEmit
yarn lint
yarn test
```

On Windows the full API package has pre-existing Linux-only file-ownership code.
The new HTTP handlers can still be executed unchanged using:

```sh
go test pkg/api/blackbox.go pkg/api/blackbox_test.go -count=1
GOOS=linux GOARCH=amd64 go test -c -o /tmp/gui-api.test ./pkg/api
GOOS=linux GOARCH=amd64 go build -o /tmp/mowgli-gui .
```

The synthetic stress command compares identical generated JSON telemetry with
and without the recorder and injects an emergency near the end. It reports CPU,
heap/runtime allocation, peak RSS, maximum admission time, normal disk I/O,
drop counts and saved capture metadata. It excludes Foxglove/CDR, DDS and normal
robot control; it must not be presented as ROS, simulation or hardware evidence.
See [performance evidence](BLACKBOX_PERFORMANCE.md) for measured results and gaps.

## Remaining target-system evidence: HARDWARE_REQUIRED

The outstanding acceptance question is whole-stack subscription/CPU/RSS impact
on a low-powered supported SBC. Local Windows core tests cannot settle it.

Baseline: record the PR commit, GUI and ROS image digests, firmware image,
both submodule gitlinks, SBC/OS/RAM/cgroup limits, robot unit and receiver/driver
revision. Compare the same baseline/config/workload with blackbox disabled and
enabled; do not compare different firmware or receiver revisions.

Procedure: with mower immobilized, blades disabled by the normal hardware safety
procedure and an operator able to remove power, run the ordinary ROS stack for
30 minutes per mode. Use deterministic synthetic publishers for the recorded
topics and emergency/diagnostic events; do not command wheels/blades. Measure
GUI/Foxglove/ROS CPU, peak RSS, disk I/O, telemetry delivery, snapshot latency and
existing hardware heartbeat/deadline metrics. Repeat with the 4 MiB setting and
the target's normal memory limit. Run disk-exhaustion injection in a disposable
quota/loop filesystem, never the production root volume.

Pass: bounded steady recorder memory/storage, no raw-history file writes before
trigger, downloadable valid pre/post capture, visible losses under pressure,
recovery after resource failure, and no attributable control/heartbeat deadline
miss or ROS lifecycle restart versus the disabled baseline. Stop immediately if
normal safety/heartbeat health deteriorates. No such target/hardware run has been
performed for this implementation.
