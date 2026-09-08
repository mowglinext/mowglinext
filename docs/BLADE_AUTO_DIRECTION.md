# Automatic blade direction

Settings → Mowing → **Automatic blade direction** saves `blade_auto_reverse`
in `mowgli_robot.yaml`. It defaults to `false`. Restart ROS2 after changing it.
With it disabled, a new session defaults to direction 0; an explicit map-menu
choice still overrides it.

When enabled, the behavior tree randomly requests direction 0 or 1 on the first
blade-enable command of a session. Coverage and manual mowing share the choice.
Repeated enable commands, pause/resume, obstacle detours, area changes and
temporary charging stops retain it. The existing `EndSession` action clears it
after a completed or abandoned session; a temporary `ClearCommand` does not.
Stopping manual mowing in place is a pause, so resuming retains the choice too.
Random selection can repeat the previous direction; this is not strict alternation.

The choice lives in ROS2 memory, not the settings file or coverage resume file.
A ROS2 restart selects again on the next enable, including when resuming saved
coverage. Resetting coverage progress alone does not change blade direction.
Both command paths log the requested direction. The unsigned RPM feedback does
not identify the physical rotation direction.

## Direction display

Diagnostics → blade/motor telemetry and the dashboard blade tile show
**Requested blade direction**: Forward, Reverse, Off, or Unknown. The motor
entry in `/diagnostics` also includes `blade_requested_direction`.

This is the last blade command successfully written by the hardware bridge,
including automatic selection, map overrides, OFF requests and the dry-run
gate. It is not a firmware acknowledgement or a measured rotation direction.
RPM remains independent: the motor can still be spinning after an OFF request
or waiting to stop before a requested reversal. No clockwise/counterclockwise
claim is made. An idle map selection appears only when the tree permits the
corresponding ON request to reach the bridge.

The bridge clears the displayed request to Unknown on serial disconnect or
failed transmission. The GUI shows Unknown before telemetry, with an older
stack lacking the field, and after five seconds without status updates.
Updating this ROS Status message requires rebuilding all ROS consumers and
the GUI together; no firmware wire/protocol change or LFP patch is included.

The display explains the firmware requirement, including in a tap-accessible
dashboard popover. Older firmware may run forward even when Reverse was requested.
Mocked desktop/mobile screenshot evidence is attached to
[PR #558](https://github.com/mowglinext/mowglinext/pull/558), outside the product assets.

## Map blade controls

The map menu's **Blade forward**, **Blade backward** and **Blade off** actions
call `/behavior_tree_node/blade_control` through the GUI's `blade_control` API.
They share the session policy with coverage and manual mowing. An explicit
forward/backward choice replaces the random choice until `EndSession`, including
through pauses and temporary stops. Firmware performs the stopped reversal.

Blade off inhibits subsequent tree enables until forward/backward is selected,
an explicit mowing start is accepted, or the session ends. Play/Resume, Mow this
area, Mow next area and Manual Mowing clear only inhibition; they retain the
selected direction. Automatic continuation/recharge and repeated ticks do not
clear it. Blade off is a software mowing control, not isolation for handling
the machine. Choosing a direction cannot override the tree's OFF state:
while idle, transiting, docking or stopped by a guard, it selects the direction
for the next permitted enable. It does not start a mowing mode or move the mower.
These overrides are in memory and reset with ROS2, like the automatic choice.

OFF first attempts the tree latch, bounded to 250 ms, then sends an independent
hardware OFF with its own two-second budget. Missing service advertisement fails
immediately. Closing the browser does not cancel these bounded OFF attempts.
Coverage, manual mowing and operator requests share a ROS client/writer, so a
pending tree ON precedes the subsequent tree OFF. No future tick can re-enable
while inhibited. This does not assert global ordering across unrelated clients
or replace the firmware emergency stop.

The new `BladeControl` service reports policy acceptance, whether forwarding was
queued, and an explanatory message. The legacy BT `mower_control` endpoint is a
thin adapter for earlier combined test GUIs; the hardware `MowerControl` schema
is unchanged. OFF ignores its unused direction field and retains its inhibition
even during a hardware outage. Direction is selected before discovery/send so
an undelivered request and its retry retain the same session choice.

With a new GUI and older/disconnected ROS, ON has no direct hardware fallback and
explains the compatibility/connection problem. If hardware accepts OFF but the
tree latch is unavailable, the UI warns that the tree may re-enable it. If the
latch succeeds but direct OFF fails, it reports retained inhibition and an
unconfirmed hardware request. If both fail, it reports both failures. A queued
or accepted request never confirms physical stoppage. The separate manual-mode
Stop action retains its existing high-level STOP and direct OFF path.

The tick, operator service and explicit-start handlers share the node's default
MutuallyExclusive callback group. A production-wiring regression runs a real
tick under MultiThreadedExecutor and proves the operator callback cannot overlap
it. A future reentrant conversion must preserve command ownership and ordering.

## Lift recovery

With optional `lift_recovery_mode`, recovery preserves the latest requested
direction rather than assuming Forward. Repeated ON requests cannot bypass the
lift-clear delay. A protective OFF preserves desired direction, while a caller
OFF cancels pending recovery. A new lift restarts the delay; STOP/emergency,
dry-run suppression, failed transmission and serial reconnect invalidate delayed
recovery. Reconnect does not automatically revive an old enable request.

## Firmware and hardware requirement

This feature affects physical blade behavior. Enable it only with a blade
assembly suitable for both directions and firmware that implements the direction
request **and waits for the blade to stop before reversing**. Firmware remains
the sole blade safety authority; the host's dry-run gate and emergency handling
are unchanged. No host command bypasses firmware checks.

The upstream firmware at the base of this change (`2d45cab4`) ignores the blade
direction argument. This ROS2/GUI change alone does not add firmware reversal.
Standard 500 and 500B firmware support is proposed separately in
[PR #559](https://github.com/mowglinext/mowglinext/pull/559), including the stopped
reversal guard. That PR contains no LFP charging changes. Version numbers
alone are not a reliable capability check for custom builds. Merge/release #559
before #558, and flash compatible firmware before using reversal. This PR does
not invent a capability from the firmware version or change the STM32 protocol.

## Validation before enabling on a mower

Software tests capture real BT service requests against a fake hardware service:
both command paths retain reverse through OFF/ON, and session reset sends no
command. Operator-service tests also cover forward/reverse overrides, repeated
manual ticks, coverage stop/resume, OFF inhibition and idle/guard precedence.
API tests verify routing, rejected requests and the independent OFF path. GUI
tests click the three actual map menu actions. Launch/config tests cover the
default and toggle wiring.

**HARDWARE_REQUIRED for the review fixes.** Prior operator reports in
[the #559 test record](https://github.com/mowglinext/mowglinext/pull/559) establish
both physical directions on 500/.119 (standard firmware 1.9.205, source
`438eed5d084bd9d2ab49c878f5b7477518bbeef5`) and 500B/.118 (combined firmware
1.9.124, source `41c8d6d3addf4c84732cfad0b3cb7a51272a2ba5`). Recorded software was
ROS `f33f183d` and GUI `7e167b88`; controller revisions were not identified. These
reports do not validate the new review fixes, the standard 500B artifact or the
complete stopped-reversal guard.

For a new run, record the exact PR/combined-stack commits, container digests,
firmware source/hash, submodule gitlinks, board/controller revisions and lift
parameters. Remove blades, secure against travel, restore intended safety wiring
and have a verified independent stop with a supervising operator. Test idle OFF
followed by each explicit start, repeated enables, pause/recharge, reverse
lift-clear recovery, OFF during the delay, stopped reversal in both directions,
emergency cancellation and loss of feedback. Pass requires preserved direction,
no delayed restart after OFF, physical stop before opposite rotation, and no
forced restart with missing required feedback. Correlate serial/ROS logs with
physical observations; unsigned RPM cannot establish direction. No hardware
acceptance run or deployment is performed by these software checks.
