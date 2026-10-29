# Browser controllers — Part 1 of #999

Use a USB or Bluetooth controller connected to the **phone, tablet or computer
running the GUI**. Open that robot's **Map → Controller** panel. The GUI must
remain open, visible and focused. Native SDL2 / ROS 2 `joy`, headless operation,
robot-side Bluetooth management and standalone mapping files remain the required
follow-up Part 2 for the complete #999 scope.

For iPad, try the installed Home Screen web app (PWA), particularly if controller
buttons also navigate browser controls. One user reports a better driving
experience there; this is a recommendation, not a requirement or evidence that
PWA mode fixes input loss. Follow Apple's [Home Screen web app instructions](https://support.apple.com/guide/ipad/ipad8f1f7a29/ipados).
Keep the app visible and focused. All interlocks apply in both browser and PWA.

## Connect and configure

1. Pair using the client device's Bluetooth settings, or plug in USB. Open the
   GUI using the robot's local HTTP address or an existing trusted HTTPS access
   path. The GUI accepts either; Tailscale is optional. The browser must expose
   the Gamepad API on that address. If input is unavailable or blocked over HTTP,
   try HTTPS or a browser that supports it; the GUI cannot bypass browser policy.
2. Press a controller button so the browser exposes it. Select the controller
   explicitly; a newly connected controller never takes over automatically.
3. Test the live inputs. Standard-layout defaults are: axes 1/0 for drive/steer
   (both inverted), RB/R1 (button 5) for deadman, A/Cross (0) for STOP,
   Y/Triangle (3) for start recording, X/Square (2) for finish and B/Circle (1)
   for cancel. Browser button numbers are authoritative; labels vary by device.
4. Change axis/button selectors or choose **Bind input**, release everything,
   then press one button or move one stick. Assign unique buttons and axes.
   Button learning ignores resting axes. Axis learning measures movement from
   a stable resting position; an ambiguous gesture must return fully to that
   position before retrying. Configure dead zone, inversion and speed scale,
   then save. Unknown layouts
   require testing and saving before Enable becomes available.
5. Close configuration with **Done**, then tap **Enable for this robot** directly
   on the map. Done never opens a control session; activation stays unavailable
   until the panel has finished closing. The map's Enable/Disable button stays
   mounted and receives focus once on this explicit click, never continuously.
   After a fault, use the same map button to enable again without reopening settings.
   The hook rechecks the actual controller and mapping before each session.
6. Choose area recording or explicitly start Manual Mowing using the robot's
   controls. **Manual Mowing starts the blade.** After every mode change,
   center the sticks and release every mapped button, including the deadman.
   Press and hold the deadman with centered sticks, then move a stick. Maximum
   speeds match the touch joystick: 0.25 m/s and 0.6 rad/s, scaled down by the
   profile. A deflected stick on a new deadman press requires another neutral
   release before motion is accepted.

The map shows the mapped deadman and STOP buttons, configured maximum speed,
and the next required step. Standard mappings include common Xbox/PlayStation
button names alongside their browser indices. IDLE displays a mode instruction
instead of Ready; missing status, emergency and pending commands have distinct
messages. Opening Controller settings stops controller input.

Mappings are versioned, validated and saved separately from robot settings, in
this browser's local storage, per robot origin and controller layout/ID. They
survive page reloads but do not transfer between browsers, origins or devices.
Clearing browser storage removes them. The explicitly chosen controller layout
is remembered, but activation is never persisted. On the next visit, press a
controller button to expose it; a unique matching device is preselected using
its current slot. If multiple connected devices have the same ID/layout, choose
one explicitly. A remembered selection never opens a control session. An edited
or unknown mapping that cannot be saved remains ineligible for driving.

## Driving and recording

Driving uses the existing `/cmd_vel_teleop` path only in `RECORDING` or
`MANUAL_MOWING`. For blade-free driving, start an area recording through the GUI
or hold the mapped start button for 0.8 seconds while idle. To finish/save or
cancel/discard, stop, center the sticks, release the deadman and hold the mapped
button for 0.8 seconds. These call the existing `high_level_control` commands
3, 5 and 6. STOP calls command 8 immediately. If a recording request is still pending, STOP
is also reasserted after that request completes so a late start cannot win.

**Initially select Manual Mowing explicitly from the GUI; it runs the blade.**
During Manual Mowing, releasing the physical deadman sends zero and the existing
STOP command (8). The controller stays connected. The panel shows stopping,
paused and starting states. A pause becomes resumable only after STOP succeeds
and a subsequent fresh IDLE status arrives. At that pause, release all controls,
then press and hold the deadman again with centered sticks to request Manual
Mowing (7), including its blade. Holding deadman before the pause is ready does
not queue a restart. Keep the sticks centered until the command succeeds and a
subsequent MANUAL_MOWING status confirms the mode, then drive.

This permission belongs only to this controller's own manual pause. The mapped
STOP, **Stop Manual Mowing** map button, another mode selection, emergency,
disable, disconnect or input/status loss cancels it. Explicit STOP leaves the
controller connected but requires an explicit new Manual Mowing selection.
A new connection never inherits restart permission. Release during a pending
restart requests STOP immediately and again after that request settles, so a
late command cannot leave Manual Mowing selected. There is no blade override or
emergency reset in the controller. The existing map emergency stop acts immediately;
its reset remains separate. Recording deadman release stops driving while preserving recording.
The BT/map server retain all recording geometry, validation and persistence.
A successful command response acknowledges the command; it is not proof that a
valid polygon was saved. Check the resulting map.

## Stops and ownership

- Poll current `navigator.getGamepads()` state every 50 ms; never feed controller
  values into the touch joystick's cached-value repeater. The configuration
  preview refreshes only while its panel is open; closing it does not pause
  input reads or safety checks, and its hidden form is unmounted.
- Changing input ownership does not restart the recording trajectory stream or
  rebuild a map that has no trajectory to clear.
- Missing/disconnected device, malformed inputs, changed device identity,
  invalid profile, browser blur/hide/page exit or >200 ms poll gap revoke the
  session. Nonfinite, zero, negative or regressed device timestamps fail closed.
  `Gamepad.timestamp` marks input updates, not a periodic heartbeat: WebKit can
  use a different clock origin/unit from `performance.now()` and update it only
  when values change. Its ordering is checked independently from the browser's
  monotonic polling clock; a centered or steadily held control does not expire
  merely because its values remain unchanged. No timestamp is replaced with
  callback/poll time. See the [Gamepad timestamp contract](https://www.w3.org/TR/gamepad/#dom-gamepad-timestamp)
  and [WebKit's timestamp implementation](https://github.com/WebKit/WebKit/blob/main/Source/WebCore/Modules/gamepad/Gamepad.h).
- Loss of live GUI status delivery (>2.5 s on the monotonic browser clock)
  or emergency status revokes the controller session. An outstanding action
  suppresses motion and requires neutral rearming. This checks delivery
  liveness, not physical acquisition provenance. Robot-side interlocks remain
  authoritative.
- Movement is never queued by the browser, buffered for reconnect or replayed.
  A nonempty WebSocket send buffer terminates the session. Recovery requires
  explicit Enable, neutral inputs and a new deadman press.
- The backend's dedicated `/api/mowglinext/publish/gamepad` socket first sends
  `{type: "gamepadHello", challenge: <random 32-character hex>, version: 3}`. The current
  browser attempt must return `{type: "gamepadClaim", challenge: <same>, version: 3}` from a
  valid native-input poll within 300 ms of server challenge issuance. An HTTP
  upgrade alone reserves no ownership and performs no ROS subscription, zero
  publish or STOP. A canceled browser ignores a late Hello; invalid, expired or
  foreign claims close without touching robot control. Ownership and claim age
  are checked again together under the coordinator lock before adoption.
  After adoption, subscription and initial zero, the server sends
  `{type: "gamepadReady"}` to confirm transport setup, not physical actuation or
  the robot's mode. The browser waits for Ready before sending any
  velocity or controller action, and requires new neutral input without clearing
  timestamp ordering, poll history or terminal faults. The entire browser
  startup remains bounded at two seconds, including Ready. Handshake writes
  are bounded at 100 ms. This internal protocol requires matching GUI/backend
  builds. Version 3 requires physical deadman state and pause epochs, excluding cached
  clients with older lifecycle semantics; an older page fails closed and must be reloaded.
- After Ready, the socket accepts only bounded, finite teleop frames
  `{twist: <TwistStamped>, deadman: <boolean>, physicalDeadman: <boolean>,
  neutral: <boolean>, epoch: <nonnegative integer>}`. Effective `deadman` is the
  drive authorization; `physicalDeadman` preserves the actual button state even
  while a mode or neutral interlock blocks driving. `neutral` describes centered
  drive axes with mapped action buttons released (deadman is reported separately). The server sends bounded, serialized
  `{type: "gamepadControl", epoch, phase}` notices for `inactive`, `active`,
  `pausing`, `paused` and `resuming`. Ready starts at epoch 0, inactive; each
  phase change increments the epoch, and inactive revocation also increments it.
  An active notice includes `resumed: true` only for a successful, confirmed
  deadman resume. This preserves command provenance if intermediate phase
  notices are coalesced; initial activation and explicit GUI Manual Mowing do
  not carry it and still require normal neutral rearming.
  The browser echoes the latest epoch. A release/press from an older epoch cannot
  grant or consume a current pause capability. These are session identities,
  not proof of physical input freshness.
  A 300 ms monotonic session lease expires independently of the read/publish
  loop. Disconnect, malformed input and lease expiry close the session, send
  zero and request STOP if this session has observed Manual Mowing. Transient
  NULL/IDLE guard states retain that responsibility. A successful explicit mode
  command supersedes it; an uncorrelated status cannot erase explicit ownership.
  Each publish carries that accepted frame's fixed deadline through transport
  selection, relay writing and Foxglove advertisement/publication. Both writers
  recheck validity after acquiring their shared lock; an expired queued motion
  is discarded. Cleanup zero and STOP use independent contexts. Providers that
  cannot enforce this contract reject controller activation with HTTP 503.
  Receiving a frame checks the deadline under the same lock as renewal, and
  expiry checks/marks the session ended atomically. A delayed timer callback
  cannot let an already-expired session revive from late input.
  The lease starts immediately on adoption, including while subscription setup
  is blocked. Late status callbacks cannot alter an ended session; cleanup waits
  for registration to finish before removing it and releasing ownership.
- One controller session owns GUI teleop across tabs. While it owns the path,
  legacy touch publishers cannot send commands. Publishers predating a
  controller takeover cannot resume old queued input afterward. Cleanup retains
  ownership until zero, pending mode requests, STOP compensation and subscription
  cleanup complete. Relay writes are
  bounded at 100 ms. Other ROS teleop producers are outside GUI arbitration.
- Existing relay, `twist_mux`, hardware bridge and firmware safety remain in
  place. Teleop bypasses the Nav2 collision-monitor lane, as the existing touch
  joystick does: keep the robot in sight and its path clear.

Controller recording (3/5/6) and STOP (8) requests use the existing
`high_level_control` route with the internal `X-Mowgli-Gamepad-Session` header,
carrying this socket's Hello/Claim challenge. The server rejects retired,
unready or expired sessions with HTTP 409 and never logs the challenge. The
server owns admitted mode requests through bounded settlement, independently of
HTTP cancellation or page/component lifetime. STOP bypasses a pending request,
then reasserts STOP after it settles, including after failure. Concurrent STOP
requests and disconnect cleanup share that obligation. No retired browser sends
later compensation.

During a pending mode request, STOP compensation or session cleanup, conflicting
GUI requests (`high_level_control` except STOP, `start_in_area`, and
`coverage_clear_resume`) return HTTP 409 instead of entering a queue. Pending
requests also suppress controller velocity. A new socket cannot acquire ownership
until settlement and cleanup finish. Ordinary Manual Mowing (7) still works with
an idle, ready controller. An admitted Manual Mowing command retains its cleanup
obligation even if delayed/cached RECORDING status arrives. Only a successful
subsequent explicit mode command or final STOP supersedes that obligation;
clearing coverage progress does not select a mode. Once a GUI command establishes
mode ownership, uncorrelated status cannot erase or restore its cleanup obligation.
Status remains a mode hint for sessions with no explicit GUI mode command.
Untagged older GUI mode requests remain compatible:
when a controller owns the path they acquire the same action fence and cleanup
obligation; without an owner they retain request cancellation. Untagged STOP and
emergency requests remain recovery paths. This arbitration covers GUI routes,
not other ROS or MQTT producers.

STOP remains a best-effort existing behavior-service call, with a one-second
context per attempt (mode calls have a ten-second bound). Failure is reported;
settlement does not prove the robot stopped. Ownership is released after the
attempts settle, even if STOP fails, allowing an explicit recovery request.
A completed recording is preserved on disconnect; an in-flight recording must
settle and be compensated before a replacement controller can take over.
Waiting for a shared Foxglove writer can outlast that context, but cancellation
is checked after acquiring the writer and an expired call is not sent later.
The network write uses the earlier service/transport deadline. An already-sent
service request cannot be retracted. A dead GUI server, unavailable ROS service or
robot communication failure cannot guarantee immediate blade shutdown; zero
velocity alone does not switch off the Manual Mowing blade. The controller is
not a replacement for the physical emergency stop or firmware interlocks.

The browser API does not expose an independent physical input heartbeat. If the
browser/OS silently returns a frozen snapshot with `connected: true`, JavaScript
cannot distinguish that from a legitimately held control. Disconnect events,
failed/missing reads, invalid/regressed input, focus loss, polling interruption
and network loss are detected; a silent native input freeze is not guaranteed to
be detected. Physical controller-disconnect tests remain required for each
client/browser/controller combination. Browser input alone is not a safety-rated
deadman or emergency stop.

The startup claim proves a live post-upgrade browser attempt; it is not an
ongoing physical-input heartbeat or a per-frame network freshness guarantee.
The established-session lease measures validated frame **receipt**. Browser
`bufferedAmount === 0` does not exclude buffering in the OS or network, and
`close()` does not retract messages already sent; see the [WebSocket standard](https://websockets.spec.whatwg.org/#dom-websocket-bufferedamount).
A link that interrupts frame delivery for 300 ms must stop. The startup fix
prevents an unclaimed delayed connection from taking over a later manual mode;
it does not make a delayed/unavailable WiFi or tunnel path suitable for driving.

## Diagnosing an input stop

An input-loss warning includes the first specific cause, such as lost page focus,
a missing device, an invalid axis/button/timestamp or a measured polling gap.
Focus diagnostics distinguish activation, polling and a window blur event, and
include page visibility, the actual focus flag and the active element's tag.
Reopen **Controller** to read or copy that diagnostic; reopening preserves it,
and the next activation attempt clears it. When its close message reaches
the server, the GUI log also records the reason and polling gap. These are browser
observations, not proof of a physical Bluetooth disconnect. Record the client OS,
browser/controller versions and exact diagnostic when reporting a failure.

There is **no maximum session duration**. The 200 ms input polling deadline,
300 ms backend frame lease and 2.5 s status-delivery check apply continuously.
Following a stop, check the map's next-step message: `waitingForMode` requires
an explicit robot mode selection (a lost manual session requests STOP),
`releaseControls` requires centered sticks and all mapped buttons released,
`statusLost` requires fresh robot status, and `actionPending` requires the
outstanding command to settle. Retrying Enable does not discard a command with
an unknown outcome or restore blade-enabled Manual Mowing automatically.

The GUI now records a bounded diagnostic history automatically. Browser reports
include the web build ID, page origin (scheme/host/port), PWA flag, controller identity/mapping, mapped input
values, poll gaps, status age, mode, pending-command age, socket state/buffer,
and recent state/close/input-loss events. The browser retains 32 events across
activation attempts while the map stays mounted, uploads at most one report at
a time every five seconds while dirty, and attempts an immediate report after
a fault. Uploads time out independently after two seconds; failed reports can
be retried while this page stays open. No coordinates, credentials or map data
are collected. Diagnostics never renew the lease or issue robot commands.

Read **GET `/api/mowglinext/gamepad/diagnostics`** on the robot to inspect the
last eight browser reports and 64 backend lifecycle events, including lease
expiry, bootstrap claim/rejection, invalid mode/frame, transport errors, manual deadman release, STOP,
zero-write completion and ownership release/rejection. The browser attempt ID
matches the backend's `client` field. Backend causes also appear in GUI container
logs. GET remains readable during cleanup; POST is size-limited and same-origin.
This is temporary in-memory history, not a disk recording: a GUI restart clears
it, older entries are overwritten, and a fully offline/closed browser may not
deliver its last report. Capture it before restarting. For a supervised next
test, keep the PWA open after a fault and note the approximate time; an observer
can inspect this endpoint live without sending motion commands. A stalled
telemetry WebSocket handshake now retries its existing subscription connection;
the gamepad control connection still requires explicit Enable.

Compare browser event/snapshot times with server `receivedAt` and correlate
attempt IDs: network-delayed reports can arrive out of order. A late report is
not a new input observation. If packet delivery fails with healthy input polls
and focus, compare the direct LAN path with the tunnel/proxy path while the
robot is stationary, using the same build/profile. Do not increase the safety
deadline to conceal a failing link. Do not install GUI updates during mowing.

The direct activation, panel closing/focus handoff and map ownership churn are covered in software
tests. These tests cannot establish physical iPad/Safari/Bluetooth behavior.
Safety deadlines and timestamp-order validation are unchanged.

## Validation and physical acceptance

Automated coverage uses synthetic browser input, mocked robot commands and Go
WebSocket/provider tests. It does not prove any physical controller, transport,
blade or stopping-distance behavior. GUI CI runs both controller browser specs serially in Chromium and WebKit, including
change-only WebKit-style timestamps and configuration/drive handoff. The polling
deadline remains active in these tests.

**HARDWARE_PENDING** — software prerequisites and the procedure are defined;
physical USB/Bluetooth acceptance is outstanding. Baseline: record this PR's
exact tested commit, client OS/browser version, controller model/firmware and
transport, robot unit, hardware backend, ROS image/revision, firmware image and
external submodule gitlinks. Do not apply one baseline's results to another.

Safety prerequisites: first test with wheel/blade actuation physically isolated
according to the robot's service procedure. For ground driving, use a clear,
contained area, an observer at the physical emergency stop, the lowest speed
scale and the normal hardware interlocks. Test Manual Mowing blade shutdown
only under the project's safe blade test procedure, with no person in reach.

For each supported client/controller pair, including USB and Bluetooth and at
least two controller families:

1. Verify default/custom direction, dead zone and speed caps against live input;
   save/reload and test an unknown layout, conflicts and missing inputs.
2. Verify no motion on page load, enable with sticks deflected/deadman held,
   reconnect, controller selection, second-tab takeover attempt or configuration.
3. Drive in recording mode; release deadman, unplug USB, power off Bluetooth,
   interrupt browser polling, background the page, navigate away and interrupt the client
   network. Observe the command stream become zero/expire and wheels stop;
   measure command and physical stop latency separately. Software targets are
   the deadlines above plus the downstream watchdog/scheduling latency, not a
   promised physical stopping distance. Recording must remain active and must
   never save/discard merely because input was lost.
4. Start/finish/cancel deliberately. Each gesture sends one existing command;
   simultaneous buttons cannot save/discard. Confirm the expected map result.
5. Start Manual Mowing explicitly from the GUI. Release deadman with the sticks
   still deflected; verify wheels and blade stop while the controller stays
   connected. After the paused message, center/release all controls, then hold
   deadman again. Verify one restart, with no wheel motion before new mode
   confirmation. Repeat several cycles without reopening settings or enabling.
   Holding deadman early must never start later on its own. Release while a
   restart is pending and verify the final mode stays stopped. Verify explicit
   STOP, emergency/reset and another mode prevent deadman restart. Repeat with
   disconnect/network loss; these must stop and require explicit Enable and mode
   selection. A failed STOP or continuing blade fails acceptance; do not proceed
   to normal use.
6. Confirm fresh status/interlocks, physical emergency stop and touch controls
   still work after controller shutdown. Leave centered controls unchanged before
   enabling, hold a steady stick for several seconds and complete each 0.8-second
   recording gesture without further input changes. None may fail merely because
   the browser updates timestamps only when values change.

Keep observed timings and pass/fail results with the exact baseline above.
