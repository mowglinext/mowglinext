# Browser controllers — Part 1 of #999

Use a USB or Bluetooth controller connected to the **phone, tablet or computer
running the GUI**. Open that robot's **Map → Controller** panel. The GUI must
remain open, visible and focused. Native SDL2 / ROS 2 `joy`, headless operation,
robot-side Bluetooth management and standalone mapping files remain future work.

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
   Configure dead zone, inversion and speed scale, then save. Unknown layouts
   require testing and saving before Enable becomes available.
5. Enable for this robot, center the sticks and release every mapped button.
   Press and hold the deadman with centered sticks, then move a stick. Maximum
   speeds match the touch joystick: 0.25 m/s and 0.6 rad/s, scaled down by the
   profile. A deflected stick on a new deadman press requires another neutral
   release before motion is accepted.

Mappings are versioned, validated and saved separately from robot settings, in
this browser's local storage, per robot origin and controller layout/ID. They
survive page reloads but do not transfer between browsers, origins or devices.
Clearing browser storage removes them. Enable and the selected device are never
persisted. Unavailable storage leaves the profile unsaved and driving disabled.

## Driving and recording

Driving uses the existing `/cmd_vel_teleop` path only in `RECORDING` or
`MANUAL_MOWING`. For blade-free driving, start an area recording through the GUI
or hold the mapped start button for 0.8 seconds while idle. To finish/save or
cancel/discard, stop, center the sticks, release the deadman and hold the mapped
button for 0.8 seconds. These call the existing `high_level_control` commands
3, 5 and 6. STOP calls command 8 immediately. If a recording request is still pending, STOP
is also reasserted after that request completes so a late start cannot win.

The controller cannot start Manual Mowing or enable/reset blades or emergency
interlocks. **Manual Mowing must be started explicitly from the GUI and runs the
blade.** Deadman release during Manual Mowing ends the controller session and
requests the existing STOP command, so restarting requires another deliberate
GUI action. Recording deadman release stops driving while preserving recording.
The BT/map server retain all recording geometry, validation and persistence.
A successful command response acknowledges the command; it is not proof that a
valid polygon was saved. Check the resulting map.

## Stops and ownership

- Poll current `navigator.getGamepads()` state every 50 ms; never feed controller
  values into the touch joystick's cached-value repeater. The configuration
  preview refreshes only while its panel is open; closing it does not pause
  input reads or safety checks, and its hidden form is unmounted.
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
- Loss of live GUI status delivery (>2.5 s on the monotonic browser clock),
  emergency status or an outstanding action suppresses motion and requires neutral rearming. This checks delivery
  liveness, not physical acquisition provenance. Robot-side interlocks remain
  authoritative.
- Movement is never queued by the browser, buffered for reconnect or replayed.
  A nonempty WebSocket send buffer terminates the session. Recovery requires
  explicit Enable, neutral inputs and a new deadman press.
- The backend's dedicated `/api/mowglinext/publish/gamepad` socket accepts only
  bounded, finite teleop frames `{twist: <TwistStamped>, deadman: <boolean>}`.
  A 300 ms monotonic session lease expires independently of the read/publish
  loop. Disconnect, malformed input and lease expiry close the session, send
  zero and request STOP if this session has observed Manual Mowing. Transient
  NULL/IDLE guard states retain that responsibility; explicit RECORDING clears it.
- One controller session owns GUI teleop across tabs. While it owns the path,
  legacy touch publishers cannot send commands. Publishers predating a
  controller takeover cannot resume old queued input afterward. Cleanup retains
  ownership until zero and the STOP attempt complete. Relay writes are
  bounded at 100 ms. Other ROS teleop producers are outside GUI arbitration.
- Existing relay, `twist_mux`, hardware bridge and firmware safety remain in
  place. Teleop bypasses the Nav2 collision-monitor lane, as the existing touch
  joystick does: keep the robot in sight and its path clear.

STOP is a best-effort call to the existing behavior service, conditional on the
session’s observed manual ownership (which may initially be cached). It is not atomic with
another client's mode change. Its ROS service context is limited to one second;
existing Foxglove transport writes may outlast that context. A dead GUI server, unavailable ROS service or
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

## Validation and physical acceptance

Automated coverage uses synthetic browser input, mocked robot commands and Go
WebSocket/provider tests. It does not prove any physical controller, transport,
blade or stopping-distance behavior. GUI CI runs both controller browser specs serially, including
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
5. In explicitly GUI-started Manual Mowing, repeat release/disconnect/network
   loss; verify STOP is accepted and wheels and blade stop. A failed STOP or
   continuing blade fails acceptance; do not proceed to normal use.
6. Confirm fresh status/interlocks, physical emergency stop and touch controls
   still work after controller shutdown. Leave centered controls unchanged before
   enabling, hold a steady stick for several seconds and complete each 0.8-second
   recording gesture without further input changes. None may fail merely because
   the browser updates timestamps only when values change.

Keep observed timings and pass/fail results with the exact baseline above.
