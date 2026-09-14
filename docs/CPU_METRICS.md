# CPU readings in Diagnostics

The Diagnostics page shows CPU temperature and usage in its summary badges and
CPU card. Temperature uses the existing Linux thermal-zone reading. Usage is
the percentage of the whole computer's CPU capacity in use across all cores:
one fully busy core on a four-core Pi is approximately 25%, not 100%.

The Go backend reads the aggregate `cpu` line in `/proc/stat` when the existing
diagnostics snapshot is requested. It calculates changes in busy time over total
time, treating idle and I/O wait as non-busy and excluding duplicated guest
columns. This is CPU utilization, not Linux load average. Counter meanings are
documented in the [Linux proc filesystem reference](https://docs.kernel.org/filesystems/proc.html#miscellaneous-kernel-statistics-in-proc-stat).

The normal page refresh is ten seconds. The first request establishes a baseline;
the following refresh can display usage. Requests within one second share the
same result, including concurrent browser tabs. A gap longer than 30 seconds
establishes a new baseline instead of displaying an average over an unattended
period. Missing/malformed counters, a counter decrease or no elapsed CPU ticks
produce an unavailable reading (`—`), not a false zero. A valid idle reading is
displayed as 0%.

`GET /api/diagnostics/snapshot` adds `system.cpu_usage` (number from 0 to 100, or
null). The existing `cpu_temperature` field and `/api/system/info` are unchanged.
Old backends without the new field are supported by the page's unavailable state.
CPU usage is informational: it does not change the overall health verdict or
control the mower.

On the standard Pi Docker deployment, `/proc/stat` exposes host CPU counters,
including work outside the GUI container. No extra mounts, privileges, packages,
shell commands, ROS topics or background monitoring processes are needed. Custom
environments that virtualize `/proc/stat` may report their virtual CPU view.

Tests: `cd gui && go test -race ./pkg/systemmetrics` covers aggregation, idle/busy
boundaries, guest/I/O-wait handling, reset/error recovery and concurrent sampling.
