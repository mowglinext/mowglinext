# GUI Lot 17 — Container restarts and error handling

## Baseline and scope

Branch `fix/gui-warnings-errors`, HEAD `38951375` (Lot 16 committed).
Preflight index empty, with only the preexisting MAVROS submodule worktree change.
Baseline: 143 ESLint warnings, zero errors; previous Vitest baseline 1,963.

Production edits are limited to `PowerMenu.tsx`, `utils/containers.ts`, and
`hooks/useContainerRestart.ts`. Three directly associated test files and this
report are added. No backend, ROS, firmware, GNSS protocol, dependency, generated
source, CSS or MAVROS changes. No staging, commit, push or system action.

## Contracts inspected and causes

- `gui/pkg/api/containers.go`, `types.go`, and `providers/docker.go`: list returns
  `{containers: [{id, names, labels, state}]}`; names/list can be null for nil Go
  slices, labels are string maps. Docker commands return HTTP 200 `OkResponse`
  (`{}` today, optional `ok` text), or an HTTP error containing `error` text.
  The provider calls the Docker SDK with the request context; a self-GUI restart
  can terminate that request before acknowledgement reaches the browser.
- `gui/pkg/api/system.go`: reboot/shutdown enter the **host** namespaces and call
  `systemctl reboot/poweroff`, then return the same acknowledgement or HTTP 500.
  HTTP acceptance does not prove that the host subsequently completed the action.
- Generated `Api.ts` (read only): non-2xx responses reject with the augmented
  Response; decoded bodies are in `response.error`. JSON parse errors on HTTP
  200 also populate `response.error` and can resolve rather than reject. Raw
  PowerMenu requests previously omitted `format: 'json'`, losing structured
  backend error details. Fetch transport failures reject independently.
- The four catch clauses treated thrown values as `any` and read `.message`
  directly (12 warnings). Container selection used `any` metadata (4 warnings).
  Generated API types do not validate JSON at runtime.
- The stack restarted non-GUI containers in parallel, then dispatched GUI last
  without observing rejection. Resolved error/invalid acknowledgements could
  also pass through the parallel branch. These paths now check acknowledgements.

Existing consumers were traced: Settings uses ROS2 + optional OpenMower sidecar;
Settings/Onboarding GNSS reconciliation uses its existing separate route and
skip-probe mode; Onboarding also has a standalone awaited GUI restart. Its
completion policy is unchanged. STM32 reboot still delegates to useMowerAction.

## Changes

Container metadata enters the decoder as `unknown`. It checks only targeting
fields: optional string IDs, string names and own string-valued labels. Missing
or null legacy metadata retains the empty/default behavior; malformed metadata
rejects before command selection. Unrelated metadata fields are ignored. Missing
containers remain distinct from a failed list request in their error details.
Name substring matching, stack prefixes, and exact OpenMower sidecar selection
are preserved. Labels cannot match inherited properties.

Commands require an HTTP-success JSON object acknowledgement with no `error`
field and an optional textual `ok`. A generated-client decode error cannot be
reported as command acceptance. Error extraction reuses `requestErrorMessage`;
non-text malformed errors fall back safely. HTTP error status is retained when
wrapping a resolved response error in an Error object.

Non-GUI stack restarts remain parallel; their acknowledgements must all succeed
before dispatching GUI. The GUI request stays detached so its own shutdown cannot
block reconnect observation. Its rejection is observed: HTTP errors report a
rejected request and end the blocking reconnect display; transport/decode loss
reports an unknown outcome and keeps availability observation. No command retry.

PowerMenu host requests explicitly decode and verify their acknowledgement.
Shutdown success notification requires that acknowledgement; a lost/malformed
response produces an unknown-outcome warning, without success or a reconnect
loop for a host intended to remain off. Reboot loss produces the same warning
while preserving reconnect observation. Existing contextual confirmations,
labels, danger styling, board action and blocking dialog settings are retained.
A synchronous shared guard prevents concurrent commands; a consumed confirmation
cannot dispatch its command again, including after completion. Reopening a new
confirmation for an explicit retry remains possible after a completed failure.
The 15-second reconnect GET interval is unchanged; overlapping GETs and polling
following unmount are prevented. These GETs never reissue a power command.

The restart hook retains its pending guard, probe timeout and optional no-probe
mode. HTTP rejection reports an error; other exceptions report unknown outcome.
A probe timeout also reports unknown status, without success/onSuccess. Existing
localized generic status/error text is reused within this strict file scope;
the timeout warning deliberately no longer claims that ROS2 failed to restart.
The first received status frame remains the existing delivery signal; the code
comment no longer calls it a fresh physical observation.

## Regression coverage

All commands use mocked transport. No Docker, robot or host command was executed.

- `utils/containers.test.ts` (21): name/label matching, optional/null metadata,
  malformed list/IDs/names/labels, unavailable Docker versus absent container,
  inherited label rejection, invalid acknowledgements, ROS-before-sidecar order,
  standalone GUI rejection, non-GUI completion before detached GUI dispatch,
  HTTP rejection preventing GUI dispatch, and observed GUI HTTP/transport loss.
- `hooks/useContainerRestart.test.tsx` (7): pending state and concurrent guard,
  acknowledgement followed by stream delivery, socket close, stream error/timeout,
  unknown and structured rejections, no false success, and no-probe mode.
- `components/PowerMenu.test.tsx` (12): contextual confirmation, Cancel without a
  request, explicit shutdown acknowledgement, repeated/concurrent confirmations,
  host HTTP error details, malformed acknowledgements, lost shutdown response,
  reboot reconnect observation/cleanup, and detached GUI HTTP/transport failure.

## Validation

- Focused Vitest: **40 passed**, 3 files.
- Full `yarn test --pool=threads --maxWorkers=2`: **2,003 passed** (40 added),
  plus the appearance registry test; 202.66 seconds.
- `yarn tsc --noEmit`: passed.
- `yarn build --outDir /tmp/gui-lot17-dist`: passed.
- ESLint: **143 → 127 warnings**, zero errors; all 16 targeted warnings removed.
- `yarn lint`: passed, 127 warnings and zero errors.
- `git diff --check`: passed; index remains empty.

| File | Before | After |
| --- | ---: | ---: |
| PowerMenu.tsx | 9 | 0 |
| utils/containers.ts | 4 | 0 |
| hooks/useContainerRestart.ts | 3 | 0 |
| Three new test files | 0 | 0 |

| ESLint rule | Before | After |
| --- | ---: | ---: |
| @typescript-eslint/no-explicit-any | 58 | 53 |
| @typescript-eslint/no-unsafe-assignment | 30 | 26 |
| @typescript-eslint/no-unsafe-member-access | 19 | 13 |
| @typescript-eslint/no-unsafe-argument | 8 | 8 |
| @typescript-eslint/no-unsafe-call | 4 | 3 |
| @typescript-eslint/no-unsafe-return | 3 | 3 |
| react-refresh/only-export-components | 11 | 11 |
| react-hooks/set-state-in-effect | 8 | 8 |
| react-hooks/purity | 2 | 2 |
| **Total** | **143** | **127** |

## Limits and separate follow-ups

The stack is **not transactional**. Parallel non-GUI commands may already have
restarted some containers when another command rejects. The GUI is not sent a
restart in that case; no frontend rollback is attempted. A reported HTTP error
is a server rejection report, not evidence that every system state is unchanged.

Backend follow-up: power and self-GUI command routes have no operation identity
or persistent result to resolve acknowledgement loss. `/system/info` returns
CPU temperature, with no boot/process generation. A successful reconnect GET
proves GUI availability only: the old process could answer before the requested
restart begins. The preserved auto-reload behavior must not be used as proof
of reboot completion. A separate backend contract should expose operation and
process/boot identity; extending a timeout would not solve this provenance gap.
Reproduce in a mocked or controlled maintenance setup by allowing the command
to take effect but dropping its response, or delaying host shutdown while the
old `/system/info` still returns 200. Acceptance would distinguish accepted,
rejected and unknown results and correlate a changed process/boot identity,
without resending commands. No such real host experiment was performed here.

Readiness WebSocket delivery is not proof of the requested restart, physical
telemetry freshness or robot safety. A response that never settles can still
leave a command pending; no new command deadline or implicit retry is introduced.
Standalone awaited GUI restart and Onboarding completion policy remain outside
this lot. Poll cleanup prevents later reload after unmount but does not cancel
an already submitted command. No hardware acceptance claim is made.

No Chromium tests were added: layout and input mechanics are unchanged; command,
confirmation and reconnect state transitions are covered with React and transport
mocks. The two previously recorded unrelated Playwright failures are not claimed
resolved. Existing large-chunk build notices and jsdom pseudo-element
getComputedStyle notices remain. Temporary logs and build output live in `/tmp`.
