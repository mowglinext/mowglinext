# Blackbox validation and performance evidence

Measured on 2026-10-10 on Windows/amd64, Intel Core i7-10700K, Go 1.27.2.
This is transport-independent software evidence. No ROS mission, Webots run,
SBC benchmark or physical mower validation was performed.

## Sustained paired workload

Two separate processes ran the same JSON publisher workload for 30 minutes,
one without constructing a recorder and one with it. Target rate was 200 Hz
across eight synthetic diagnostic topics. The recorder requested 16 MiB,
received the conservative 8 MiB unknown-platform budget, and used a two-second
post-window to keep the synthetic fault near the end of the run.

| Measurement | Publisher baseline | Recorder enabled |
|---|---:|---:|
| Elapsed seconds | 1800.26 | 1800.00 |
| Emitted observations | 334596 | 338074 |
| CPU seconds | 31.86 | 36.42 |
| CPU, percent of one core | 1.770% | 2.023% |
| Peak process working set | 49.03 MB | 49.61 MB |
| Peak Go heap | 3.73 MB | 3.76 MB |
| Go heap at end | 2.44 MB | 2.95 MB |
| Peak Go runtime allocation | 16.98 MB | 16.99 MB |
| Normal recorder disk write bytes | n/a | 0 |
| Files before incident | n/a | 0 |

The observed CPU difference was 0.254 percentage points of one core. All
338074 ingress attempts were admitted. The synthetic rising emergency produced
one complete 819696-byte JSONL snapshot with 3747 observations, 18.13 seconds
of available prehistory, 2.00 seconds afterward and zero capture-local loss.
The requested 60-second history was shortened by the effective memory budget.

These runs shared a busy development host with builds/tests; they are not a
controlled comparison of complete GUI/ROS processes. Timer delivery achieved
approximately 186–188 Hz. Maximum measured ingress elapsed time was 3.447 s,
including host scheduling/page-fault stalls; this does **not** establish a
real-time latency bound. Admission has no blocking channel send or collector
lock, but allocation and the Go/OS scheduler can still delay the caller.

The sustained binary preceded review fixes to settings transitions, diagnostic
identity saturation, pressure capture allocation and counters. Its old
`dropped_messages` counter incorrectly included ring-capacity evictions
(334726 at shutdown); do not interpret that value as telemetry rejection.
The final code exposes `history_evictions` separately and has deterministic
regressions for each fix. Binary SHA-256 provenance:

```text
baseline D026E8A043100651252BB37991F93412DD6740716DA292F628805861FF6E5009
recorder A4A711BF8DAD5A6B72C11B432927B20F7223D843802FB7415A4E0341BFC50E86
```

## Reduced-memory follow-up

A later three-minute 4 MiB run targeted 1000 Hz on the same busy host. It emitted
73611 observations (timer delivery approximately 409 Hz), admitted 73294 and
reported 317 queue rejections. Peak heap was 3.68 MB, heap at end 1.78 MB,
peak working set 50.01 MB, and CPU 2.985% of one core. Normal disk writes/files
were both zero. Ring evictions were reported separately (71994).

The emergency saved a complete 408596-byte snapshot with 1878 observations,
4.45 seconds before and 1.98 seconds after, zero capture-local loss, and a
0.183-second asynchronous write. Maximum measured ingress elapsed time was
3.277 s under host contention. This is bounded-budget software stress, not a
claim that a lower-powered SBC has equivalent timing or that 1000 Hz was achieved.
This binary included the review fixes except the subsequent queued-message
revalidation on a settings-limit reduction; that boundary has its own regression.

A 60-second follow-up of the final reviewed source admitted all 9997 observations
with zero drops and zero normal disk writes. It saved 3729 observations with
17.40/2.00 seconds of coverage in an 808270-byte file; asynchronous write time
was 0.030 s. CPU was 1.484% of one core and peak heap 3.61 MB. Host scheduling
still affected maximum elapsed ingress time (0.101 s).

## Automated verification

- 24 recorder tests pass, including rolling expiry, manual/automatic pre/post
  capture, coalescing, concurrent ingress/triggers, missing publishers, payload
  timestamp discontinuities, oversize/invalid/burst telemetry, pressure/recovery,
  injected write failure, atomic publication failure, interrupted shutdown,
  restart cleanup, traversal/symlink rejection, retention and review regressions.
- Five unchanged-source HTTP integration tests pass: emergency and terminal
  behavior failure → capture → list/download/delete; manual capture/settings;
  malformed settings/storage failures; failed configuration persistence.
  Synthetic provider assertions confirm no robot publications/service calls.
- Existing Foxglove and focused ROS provider/subscriber/notification tests pass.
- Existing manual rosbag tests pass in a Windows source-isolated harness using
  production `rosbag.go`, its tests/types and the unchanged Linux-independent
  Docker helper functions. This is mocked Docker evidence, not a ROS bag run.
- Four panel tests and four locale parity tests pass. Desktop/mobile mocked API
  rendering was inspected. Typechecking and focused lint pass.
- Full Linux/amd64 API test binary cross-compiles; GUI builds for Linux/amd64
  and Linux/arm64. Cross-compilation does not execute Linux tests.
- Final-code ingress microbenchmark: admitted 2381 ns/op, 56 B/op, 2 allocations;
  saturated rejection 27.15 ns/op (mostly dropped calls, not normal throughput).
- Independent strong review found no remaining blockers or requested fixes and
  independently reran six focused regression groups successfully.
- GitHub's Linux `Go Tests (gui backend)` job passed the full `go test ./...`
  suite at implementation commit `2480a75c` (37 seconds). Final local frontend
  typechecking and full lint pass (zero errors, 897 existing warnings).
  A full Windows frontend run was stopped after prolonged host contention and
  failures in existing eslint-config/remote-access tests; it is not claimed as
  passing. The focused new panel/locale tests passed; Linux frontend CI remains
  the authoritative full-suite check.

Focused Linux recorder/API race execution is added to GUI CI. Its result and a
supported SBC/ROS runtime comparison remain merge prerequisites. Windows cannot
execute the full API suite because existing
file-ownership code uses Linux-only `syscall.Stat_t`; no C race toolchain, usable
local Linux Docker daemon or WSL runtime was available. Actual filesystem ENOSPC
and disk reserve probing require Linux: local tests inject write failures and
exercise serialization/storage-limit errors without filling the host disk.

## Reproduce

Use the commands in [BLACKBOX.md](BLACKBOX.md#reproducible-validation). Build the
stress command once, then run separate baseline/enabled processes with equivalent
host load and save the JSON output. Avoid comparing `go run` compiler overhead.
Record achieved rate, effective budget and actual coverage with every result.
For ROS/SBC acceptance, follow the explicit `HARDWARE_REQUIRED` baseline,
procedure, safety prerequisites and pass/fail criteria in that document.
