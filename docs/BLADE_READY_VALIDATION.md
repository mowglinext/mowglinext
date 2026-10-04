# Coverage blade readiness

`FollowStrip` withholds each new `FollowCoveragePath` goal until the blade is
ready. This includes the first unit, every blade-off sub-path transit, and
dig/detour redispatches. Navigation transits still request blade OFF.

Readiness needs an accepted ON service request, `mow_enabled`, active blade ESC,
and finite RPM at or above `blade_ready_min_rpm` in distinct post-request blade
reports spanning 0.3 seconds. Both `blade_status_stamp` source age and monotonic
status delivery age must be at most 1 second. General `Status.stamp` updates or
cached republications do not establish blade readiness. The firmware's RPM
packet is an unsigned magnitude; forward and reverse use the same threshold.
If the blade stayed ON across two goals, the new handoff still requires new
distinct reports; it cannot inherit an earlier goal's readiness permission.

If no blade-controller report has ever arrived since the behavior node started,
the legacy timer waits `blade_spinup_delay_sec` after a successful ON response.
Once telemetry has been observed, missing, stale, inactive or low-RPM telemetry
never uses the timer. `blade_ready_timeout_sec` bounds the entire wait, including
service discovery/response; failure withholds the goal, requests OFF and leaves
the unit unbooked. Missing service discovery is retried without renewing this
deadline. Late service responses cannot release an abandoned gate.

Defaults are 1000 RPM, 1.5 seconds fallback, and 6 seconds timeout. The RPM
default matches the existing map-server mow-progress threshold; it is not a
field measurement of cutting readiness. Settings are available in Advanced
settings and require a ROS2 restart. Invalid/nonfinite settings are rejected;
the timeout must exceed the positive fallback delay and be at most 30 seconds.

`mowing_enabled=false` commissioning runs and explicit operator blade OFF retain
non-cutting coverage. Firmware interlocks remain authoritative. This change
does not add a controller motion hold or change the existing short-LiDAR-dropout
protocol, which retains an already-running follow goal. Gating the resumption
of motion for that retained goal needs a separate controller-side hold design.

## Automated evidence

`test_blade_ready` exercises acquisition identity, stability, invalid RPM,
clock jumps, delayed acknowledgement, fallback and timeout. Tick-level fake
Nav2/hardware tests in `test_follow_strip_dig` exercise first-unit and post-transit
dispatch, stalled blade failure without completion bookkeeping, dry run,
fallback and halt. These do not measure a physical blade or certify cut quality.

## Physical acceptance: HARDWARE_REQUIRED

Before running, record the exact PR head (`git rev-parse HEAD`), ROS image digest,
firmware binary hash and reported version/protocol, robot unit identifier,
blade assembly/direction, battery voltage and all three gate parameters. Retain
the current receiver/driver revision and localization configuration for the
moving test. No physical acceptance evidence is attached to this change.

Use the normal firmware interlocks. Work in an enclosed, supervised mowing
area clear of people, animals and loose objects, with a tested stop control.
Do not defeat lift/emergency guards or deliberately stall a mounted blade.
Test denied/stalled/absent-report cases with fake hardware or a controlled
bench setup with blade power physically isolated, never by jamming the blade.

Record `/hardware_bridge/status`, `/cmd_vel`, coverage-plan/action goals and
behavior logs with source timestamps. Run:

1. Start a first coverage unit with the robot already at its start. Then run
   a second unit requiring a blade-off transit. In both cases, verify transit
   OFF, arrival before ON, and no coverage goal/motion before fresh active RPM
   reports span 0.3 seconds above the configured threshold. Check that this
   threshold produces the intended cut from the very start of the swath.
2. Repeat with forward/reverse selection and a lower, permitted battery charge.
   Record observed spin-up times against the exact baseline above. Tune RPM and
   timeout explicitly if needed; do not generalize one unit's measurements.
3. With isolated fake/bench hardware, keep fresh low-RPM/inactive reports,
   replay a frozen blade stamp while general status keeps arriving, reject ON,
   and withhold its response. Each case must withhold coverage, request OFF and
   return failure within the configured timeout, without booking the unit.
4. With telemetry absent from node startup and ON acknowledged, verify no new
   coverage goal before the full fallback delay **after the response**. Then
   introduce telemetry and remove/replay it: timer permission must not return.
5. Halt/change command during spin-up; verify OFF and no late coverage goal.
   Verify commissioning dry run still follows the path without any ON request.

Acceptance passes only when the traces meet these criteria and the physical
cut-start check confirms the chosen RPM threshold on the recorded baseline.
Once all software prerequisites pass, the identified run above is
`HARDWARE_PENDING`; the physical run is still required before claiming field
validation.
