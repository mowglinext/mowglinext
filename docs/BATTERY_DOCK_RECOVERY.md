# Battery/dock recovery investigation — 2026-10-18

Read-only investigation of unit `automower`. No containers were restarted, no
motion or firmware commands were issued, and no changes were deployed.
Times below are CEST (UTC+02:00), unless a source filename explicitly contains `Z`.

## Baseline and limits

The implementation baseline is `dev` at `da1494f1`. The robot was running a
custom image combination, rather than an exact build of that dev commit:

- ROS image `mowgli-ros2:pr991-58dcfca9-protocol7`, digest
  `sha256:7ce4c5fbdd2279b79395e624084194c436f46d31c7f243991e7ac81fdf6fdfcb`.
  Image revision label: `58dcfca943077fe5dcc694963b503e2d6b3e173c`;
  hardware-source label: `a0950aa30c444c1f554712982ec095dd4fb0471d`.
- Installed `main_tree.xml` SHA256:
  `7c191f2d20d74da1de148e1e9e4f2206bf25c021a63e593c43102ad1a8fae35e`.
  Installed BT executable SHA256:
  `79dd5cf8e120bee5e9239a1318f6060395000bb8930f922f1d71e944c374d4de`.
  Image labels alone do not establish every installed source revision.
- GUI image `sha256:47806dcfe8eff62f598acfc5791f2764fd506a680a1db96c8d8141230978d080`,
  revision `39665a5a6b010d0b806fa7d25daf5378dc528b31`.
- Firmware reported `1.12.33`, protocol `7`, reset cause `BOR`.
  The flashed firmware binary hash was not acquired.
- GNSS image revision `56d96d538d90551d9ca42ae02c25b3b960fa620b`, integration
  label `2736d5c397391ffdca0b7bbf3c52e5357817b18e`. Receiver hardware/driver
  revisions were not independently measured; no GNSS performance claim is made.

Measurements apply only to this observed setup. Battery percentage is derived
from voltage; its rise during charging is not an independent capacity test.

## Confirmed timeline

- **Oct 17, 18:51:30–18:52:26:** the previous BT/docking logs show genuine
  docking attempts failing navigation to the staging pose (`903`). The final
  attempt was sent at 18:52:25.828 and aborted at 18:52:26.426. Earlier attempts
  also reached the dock but timed out waiting for charging; these do not prove
  the contact/power condition at the final shutdown.
- **18:52:26:** `coverage_resume.txt` was last modified. It contains
  `current_command 7`, `critical_dock_failure_latched 1`, area ID `88`, a resume
  cursor and three completed subpaths. This is retained progress, not proof
  that a mowing command remained active at shutdown.
- **Oct 18, 01:28:32–01:29:39:** existing capture
  `blackbox-20261017T232924.508764518Z-a54094f6e6c3c1e9.jsonl` shows a stopped
  failure status, battery around 24.43 V / 8.6–8.9% filtered status, charger
  disabled, charger voltage approximately zero and discharge current. The
  robot was not charging; the records do not establish its physical location.
- **02:05:35:** the previous host journal ends after repeated Pi undervoltage
  warnings, without a normal shutdown sequence in the inspected tail.
- **About 11:02:24:** current host boot, inferred from monotonic uptime and NTP
  synchronization at 11:03:18.723 with boot monotonic time 54.566 s. The RTC
  initially read 1970 and systemd substituted its saved 02:06 timestamp.
  Docker's displayed start time of 02:06:50 is therefore misleading.
- **11:04:22:** BT log reports restored progress with `auto_continue=false`,
  `stop_hold=false`. The current boot's docking log contains lifecycle startup
  only, with no dock/navigation attempts. Initial blackbox samples already
  show `is_charging=true`, about 1.2 A and a 33% voltage-derived battery status,
  while the published state is `CRITICAL_BATTERY_NAV_FAILED`.
- **11:04:43, 11:33:29, 11:36:16, 11:39:03, 11:42:14:** GUI JSON log records
  delivered `navFailed` notifications. These are five recorded deliveries,
  even though the original report mentioned three observed notifications.
- **11:33:02–11:33:23:** blackbox status changes from the saved failure to
  `EMERGENCY` and back. Hardware status, IMU and odometry ages exceed 10 s
  during the gap. Similar guard interruptions precede subsequent deliveries.
- **11:50:46 / 11:53:07:** passive subscriptions show battery 61.0% / 64.5%,
  charger 27.03 / 27.09 V, battery 26.81 / 27.37 V, current 1.19 / 1.23 A,
  charging enabled, blade disabled, ESC power off and zero sampled wheel
  velocity. Pi undervoltage warnings continue in the current boot.

Sources remain on the robot: `/root/.ros/log/` inside `mowgli-ros2`, Docker's
JSON logs, host journal, `/ros2_ws/maps/coverage_resume.txt`, and existing
`/ros2_ws/maps/blackbox/` captures. Temporary extracted evidence is outside Git;
raw logs and full captures are intentionally not committed.

## Cause and uncertainty

Today's navigation alerts describe yesterday's persisted critical-dock failure.
The retry gate takes priority even with charging confirmed, so that latch never
reaches the charge hold/healthy state. The notifier treats a return from a guard
status as a new failure after its 60 s cooldown. Neither ordinary unchanged
status republication nor container restarts explain these particular repeats.
Docker reports zero automatic restarts for the current ROS/GUI containers.

The initial failure was real, but today's repeats were not new navigation
failures. Boot did not automatically resume mowing. An unrelated defect on dev
also allowed the critical branch's recovery tail to undock an idle mower because
it lacked a command gate. DockRobot itself lacked an already-charging shortcut.

The battery drained while charging was absent and the critical failure was held.
The physical reason charging was absent, exact loss-of-power instant, and cause
of today's data gaps/undervoltage are not established by these records. The
observed state has no motion demand and is charging, but power quality remains
an unresolved hardware concern. Do not interpret the software patch as a power
supply or battery repair.

## Recovery policy

Confirmed charger contact takes priority over saved docking failure and cancels
an unnecessary active/pending docking goal. Battery charge holds survive guard
interruptions, and process restarts that continue the mow (see "Operator
control" below). Full-charge/current-taper or permitted manual
resume remains required before continuing an active mow. Recovery undocking
requires an active START command; idle/STOP preserve progress without departure.
Valid persisted mowing resumes remain supported.

Startup waits for the first usable live Power sample before any motion-capable
guard or mission branch runs. The sample's battery percentage and restored
charging intent are applied before releasing that barrier. Until then the
tree holds blade/wheel commands off and reports `WAITING_FOR_POWER`; default
100% battery and charger-off values cannot authorize heading seeding, docking
or undocking. This is a process-local startup barrier, not a new timeout or
runtime freshness policy.

### Operator control (review follow-up)

`battery_charge_pending` remembers that a battery-driven dock/charge is not
finished. It must never move the mower on its own, and an operator must always
be able to take the mower back. The rules:

- **Boot.** `applyRestoredCommandPolicy` (`coverage_persistence.cpp`, called by
  `behavior_tree_node.cpp` `init()`) keeps a saved pending flag only when the
  same snapshot auto-continues a mow (`current_command` START with resumable
  progress). Any other boot is IDLE (or a latched critical-charge STOP hold), and
  the flag is dropped and the snapshot re-saved after the first tick. Without
  this, an IDLE mower powered on off the charger drove itself to the dock with
  no command, no battery-still-low check and no localization readiness. A boot
  with a genuinely critical pack still docks through the live `IsBatteryLow`
  check, exactly as before this PR.
- **Tree gate.** In `CriticalBatteryDock` (`main_tree.xml`,
  `BatteryChargePendingAuthorized`) a pending flag enters the branch only with an
  active mow (`IsCommand 1`) or a battery still at/below `battery_low_pct`
  (`NeedsDocking`). This is defense in depth for a flag set at runtime; the
  enter-low / resume-at-full hysteresis of the charge hold itself is unchanged.
- **Operator commands.** `applyCommandToBatteryRecovery`
  (`battery_recovery_command.hpp`, called by `~/high_level_control` and
  `~/start_in_area`): HOME, RECORD (3/5/6), MANUAL_MOW and STOP clear the
  pending flag. Every non-STOP command clears the critical dock failure latch
  (a retry, as before). STOP never re-arms a dock: after a failed critical dock
  it converts the failure latch into the existing critical-charge STOP hold, so
  the critical branch steps aside and StopHold runs even with a still-critical
  pack, across restarts. The change is persisted on the next tick. START keeps
  the pending charge (the mow must still charge first). With the pack still
  critical, START, HOME, RECORD and MANUAL re-enter the critical branch through
  `IsBatteryLow`, which docks instead of mowing. A critical pack therefore still
  overrides manual driving and recording, as it did before this PR.
- **EndSession** clears the pending flag. Its keep-progress exit re-saves the
  snapshot, and a finished session removes it.
- **One failed low-battery dock.** BatteryGuard's DockRobot failure ends the
  run through `FailedCoverageDock`, which tries a second time and reports
  `NAV_TO_DOCK_FAILED` / `DOCK_NAV_FAILED` (unchanged since before this PR).
  Its `EndSession` now clears the pending flag, so the critical branch no longer
  makes a third attempt straight away. It comes back only once the pack is
  genuinely critical.

Residual: on a restart that continues a mow with a pending charge while off the
charger, the critical branch's DockRobot can run before Nav2 is active. If the
docking server is not up yet, the attempt fails and latches
`CRITICAL_BATTERY_NAV_FAILED` (the mower stays stopped). It does not move
without a planner.

Failure notifications are remembered by terminal reason across guard statuses
and GUI restarts. A concrete new attempt rearms them; confirmed charging also
resolves navigation incidents. Contact alone does not clear `CHARGER_FAILED`.
A distinct terminal failure still reports immediately. This replaces the failure
incident's time-only suppression without changing other event cooldowns.

## Software validation

On the development host as the normal project user, the behavior executable
built and 129 focused ROS tests passed: battery recovery 24 (16 new production
XML regressions), coverage persistence 30, manual resume 29, guard structure 11
and startup pause 35. Package cpplint also passes. The startup tests reproduce
the pre-fix heading motion with the power guard omitted, then delay initial
power for 200 ticks with startup readiness/waits already satisfied and assert
zero docking, heading-seed or backup commands. Charging and off-dock samples
then take their respective recovery paths. External actions and elapsed waits
are controlled; this does not establish physical robot or charger behavior.

The GUI `go test ./...` suite passed. All notification tests were rerun after
the independent review's dead-charger alert finding was fixed and passed.
Touched C++ passes clang-format 18, including changed lines against the
`origin/main` merge-base; XML parsing, gofmt and `git diff --check` pass.
The independent final safety review found no remaining blocker or should-fix.
Whole-workspace ROS build/CI, simulation and patched hardware runs were not
performed locally.

The first PR head's full ROS CI exposed an outdated startup-test probe (it
counted new reactive condition reads as replayed startup actions) and two
cpplint false positives on unnamed lambda arguments. The test fixture now uses
the real conditions with its replay assertions unchanged, and the lambda
arguments are named. Both failing checks pass locally; full CI for the updated
head must still run.

The operator-control follow-up was validated with a local Lyrical build of `mowgli_behavior`
(`ros:lyrical-ros-base`). `test_battery_critical_resume` passes all 33 tests (8 new). The full
package suite passes with 1027 tests and 0 failures, including cpplint, xmllint and copyright.
Before the follow-up, 15 of this PR's production-XML tests failed at the branch head, because the
fixture did not register the `SetStopReason` node merged from dev (#1016). The fixture now
registers the real node.

## Physical acceptance still required

`HARDWARE_REQUIRED`: power quality and end-to-end charging/motion behavior have
not been validated on the patched robot. No deployment or physical test is
authorized by this investigation.

Before a later approved run, record the exact PR commit, built image digests,
installed tree/executable hashes, firmware binary/version, submodule gitlinks,
unit identity and receiver/driver revisions. First inspect the supply/charging
path with the robot secured, blades physically made safe, drive movement
prevented, emergency stop accessible and an operator present. Measure supply
stability during charging and correlate any hardware-data gap with independent
power measurements; pass requires no unexplained undervoltage or data loss.

Then, only under separately approved motion conditions, verify an idle recovery
stays docked through full charge and a valid interrupted mow resumes only after
the charge gate clears. Capture power, high-level status and action goals; fail
on any docking goal while charging or any departure without a valid mission.
Do not deliberately deep-discharge the battery to reproduce this incident.
