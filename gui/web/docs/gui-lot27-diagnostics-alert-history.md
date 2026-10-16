# Stable, consultable diagnostic alerts — GUI Lot 27

## Baseline and scope

Branch `fix/gui-warnings-errors`, HEAD
`9105cb61e2d476f99142362b14f8e3b6f9ff3d70`. Initial index empty; the only
preexisting worktree modification was the independent MAVROS gitlink difference.
Baseline: 2,317 Vitest tests, 105 ESLint warnings, zero errors.

This change owns presentation lifetime only. The health engine, its admissibility
and freshness rules, decoded data, emergency assertions, GNSS observation/replay
barriers and all command/permission paths are unchanged. No backend, ROS,
submodule, dependency or generated file is changed. No robot command was run.

## Reproduction and cause

Previously Diagnostics built its Collapse directly from `health.alerts` and
rendered the entire Card as null when no alert remained. A valid WARN → OK
publication removed both the group and its mounted details. A later WARN created
another instance; changing severity also reordered groups. Consulting an alert
could therefore change the page beneath the operator.

A Chromium regression on the original production page opened a warning, sent an
ordered OK publication, and required the last message and a Resolved indicator
to remain visible. It failed at that assertion after the panel disappeared:
`/tmp/gui-lot27-red-browser.log`. The first harness attempt matched the same name
in the raw diagnostics table too; the corrected red run scopes to the Alerts Card.
No claim is made that the initial ambiguous locator reproduced the defect.

Additional mobile measurements found two genuine secondary reflows: the episode
header wrapped differently when its state label changed, and the health summary
above changed height as its cause and badge severity text changed. The retained
panel itself stayed mounted, but its position could still move by 20–34 pixels.
Evidence: `/tmp/gui-lot27-mobile-layout.log`. Measurements wait for fonts and
actual Collapse animations; they do not disable production animation or increase
timeouts to hide movement.

A final mixed-group regression also demonstrated that a retained resolved ERROR
could incorrectly label a group containing a current WARN as ERROR. The group
summary now computes severity only from entries in its displayed state; the
resolved ERROR remains inspectable on its own row. The current health verdict
was never affected. Red evidence: `/tmp/gui-lot27-group-severity-red.log`.

## Presentation model and policy

`advanceDiagnosticAlertHistory()` is a pure transition with a caller-supplied
browser monotonic clock. The page's existing one-second clock drives expiry;
there is no timeout per publication, render, alert or group. ROS timestamps are
not subtracted from this clock. Retention never renews diagnostic freshness.

| Observation admitted by the existing health model | Episode presentation |
| --- | --- |
| Current warning or error | Active; latest message, severity and individual values update immediately |
| Explicit admissible OK for the same name and source | Resolved; retain the previous non-OK message, level and values |
| Missing/malformed update, blocked replay, expired data or disconnected transport | State unverified; no inferred resolution |
| Valid new delivery after reconnect | Apply the existing health model's admissibility result; socket-open alone resolves nothing |
| WARN after confirmed OK | Update the same stable row with an incremented local episode number and a new minimum lifetime |
| Repeated 1 Hz warning | Update the existing episode; no duplicate row or restarted minimum lifetime |

A row's identity is the exact `(hardware_id, name)` tuple. A group's identity is
`(hardware_id, component-before-first-slash)`. Different declared sources do not
merge, and all individual names, values and messages remain inspectable. Missing
source identifiers are labelled as not provided, not replaced by a fictitious
receiver/robot identity. Episode numbers are local presentation counters, not
producer epochs or acquisition evidence. They reset after an entry has expired
and is later observed again.

The only way to mark an episode Resolved is an explicit level-0 observation from
the same source/name **absent from the current health alert projection**. Thus an
OK payload which the health model considers stale or blocked cannot resolve it.
An empty array, missing source, another source's OK and a transport reconnect
also cannot resolve it. Actual ROS STALE and the last reported WARN/ERROR remain
distinguishable from the separate unverified presentation state.

Every episode remains in the default view for at least **30 seconds from its
first appearance**, even if it resolves earlier. A reappearing resolved episode
starts another minimum period. Repeated publications and repeated OK do not
restart that deadline. After the minimum, closed inactive entries move out of
the default view but remain accessible with the translated Recent button.

Eligible recent history expires **five minutes after its ended/unverified
transition** and is limited to **100 entries**, keeping the most recently ended
eligible episodes. Current reports, episodes younger than 30 seconds and all
entries of an open group are protected. This deliberately is not a hard cap on
active or operator-pinned data: a burst can temporarily exceed 100, and a user
can keep an open group beyond five minutes. Collapse releases that protection;
expired entries then purge immediately. Hiding Recent never removes an open
group. Collapse all is an explicit local action, not an acknowledgement or clear
command to the robot.

This is a compact latest-episode history per diagnostic identity, not a durable
log of every transition or every raw ROS packet. It sees the admitted GUI stream
with its existing one-second diagnostics throttle; intermediate subsecond ROS
transitions coalesced upstream are not reconstructed. Leaving/reloading the page
ends this presentation session. Nothing is written to browser storage or sent to
an API. The bounded archive policy does not cap the producer's existing live
accumulator; unbounded active-source churn needs a separate protocol/resource
policy, not silent removal of current faults in this patch.

## UI stability and accessibility

- The Alerts Card is always mounted, including its empty state. The empty body
  reserves a row so zero ↔ one collapsed group does not remove the whole panel.
- Collapse keys and entry keys are stable; group order is first appearance, not
  changing severity order. Severity still updates immediately in its own label.
- Open groups pin their retained contents. Local controls expose recent entries
  and collapse groups; no automatic close, clear, save or robot operation occurs.
- Group headers retain normal Ant Design keyboard behavior. Source/name remain
  in the accessible label even when the visible header uses ellipsis. Details
  expose individual key/value evidence, including explicit zero text.
- Episode names and state/episode labels occupy separate lines so a state-label
  change does not unpredictably rewrap the message header on mobile.
- The existing health summary reserves the translated severity width in its
  badges using aria-hidden layout alternatives. Only the current label is
  exposed to assistive technology. Numeric width conventions stay unchanged.
- The current-cause region reserves three lines on mobile and two on desktop;
  longer current text remains scrollable and keyboard focusable under a translated
  region label. No cause or severity is removed or changed. This prevents a
  shorter OK cause from shifting the alert consultation below it.
- The desktop page now uses the same direct div/flex parent as mobile. This avoids
  an unnecessary wrapper change inside this page; the existing shell still owns
  whether the route itself remains mounted. Other sections retain their controls.

All new product states, controls and accessible labels are translated in FR/EN.
The existing global verdict is computed **before and independently of history**.
Resolved ERROR history cannot keep the verdict Critical; a current emergency
still immediately takes priority. The cause/history sizing is visual only.

## Regression coverage and validation

Model tests cover WARN → OK → WARN, 1 Hz repetition, ERROR → OK, immediate
severity changes, missing/malformed/stale/disconnected input, blocked OK,
reconnection recovery, source separation, stable ordering, immutability,
30-second visibility, five-minute expiry, the 100-entry eligible-history limit,
open-group protection and release. A health-engine integration regression proves
that retained ERROR can coexist with a current OK verdict and that emergency
still wins immediately.

DOM tests cover permanent empty-state mounting, controlled Collapse, the same
focused/open DOM elements across resolution and reappearance, individual zero
values, immediate ERROR styling, explicit Recent access/hiding, expiry during
consultation, collapse/purge, reconnect and source labels. The summary regression
checks its current cause is a labelled keyboard-focusable region and its reserved
severity alternatives are aria-hidden.

Chromium drives the real Diagnostics page in FR/EN at **390 and 1440 px**. It
checks WARN → OK → WARN → ERROR, a preserved open/focused group and message,
current verdict recovery to OK, immediate error presentation, actual socket
close/re-subscription, unverified state before new admissible OK, no implicit
writes in both mobile and desktop presentations.
After fonts/animations settle, WARN → OK panel height and vertical-position
changes must each remain below **2 px**. Viewport overflow is also checked.

A separate browser fixture supplies a deterministic presentation clock and 110
fictional episodes to the real panel. FR/EN cases verify the minimum period,
100-entry archive, explicit history consultation, keyboard Enter to open a group,
protection beyond six minutes, responsive resizing of the same panel instance,
and purge on explicit collapse. This clock is a
component input; it does not claim to test physical sensor age or replace the
page's real telemetry freshness policy. The fixture has no robot API.

Final validation on the delivered code:

| Check | Result |
| --- | --- |
| Focused Vitest, 8 files | **122 passed** |
| Full `yarn test --pool=threads --maxWorkers=2` | **2,340 passed in 195 files**, plus appearance registry test |
| Chromium, FR/EN, `--retries=0 --workers=2` | **16 passed**, 37.2 seconds |
| `yarn tsc --noEmit` | Passed |
| `yarn build --outDir /tmp/gui-lot27-dist` | Passed |
| `yarn lint` and complete ESLint JSON comparison | **105 → 105 warnings, zero errors** |
| `git diff --check` and separate new-file whitespace checks | Passed |

There are **23 additional Vitest tests** over baseline: 15 model cases, seven
panel cases and one summary accessibility/layout case. The last full run took
244.25 seconds for Vitest. The 16 browser cases comprise the six new history
cases, eight existing health cases and two unchanged GNSS-integrity cases.
This is a targeted Chromium run, not the entire Playwright suite or proof that
unrelated historical browser failures are fixed.

Logs: `/tmp/gui-lot27-focused-final.log`, `/tmp/gui-lot27-vitest-final.log`,
`/tmp/gui-lot27-browser-validated.log`, `/tmp/gui-lot27-tsc-final.log`,
`/tmp/gui-lot27-build-final.log`, `/tmp/gui-lot27-lint-final.log`.
Before/after ESLint inventories: `/tmp/gui-lot27-eslint-before.json` and
`/tmp/gui-lot27-eslint-final.json`. All 13 changed/new files have zero lint
warnings/errors; repository debt remains outside this change.

| ESLint rule | Before | After |
| --- | ---: | ---: |
| `@typescript-eslint/no-explicit-any` | 35 | 35 |
| `@typescript-eslint/no-unsafe-assignment` | 23 | 23 |
| `@typescript-eslint/no-unsafe-member-access` | 12 | 12 |
| `@typescript-eslint/no-unsafe-argument` | 8 | 8 |
| `@typescript-eslint/no-unsafe-call` | 3 | 3 |
| `@typescript-eslint/no-unsafe-return` | 3 | 3 |
| `react-refresh/only-export-components` | 11 | 11 |
| `react-hooks/set-state-in-effect` | 8 | 8 |
| `react-hooks/purity` | 2 | 2 |
| **Total** | **105** | **105** |

No console warning was suppressed, timeout increased, lint rule changed or
artificial cast added. The initial animation/locator/subscription harness
failures are documented above; later red cases correspond to actual layout and
mixed-group behavior and were corrected before this final acceptance run.

## Read-only GNSS investigation requested alongside Lot 27

Inspected commits:

- `19d99c471` — `feat(gnss): preserve explicit solution types`.
- `f56774ea4` — `fix(gui): validate GNSS telemetry and preserve fix authority`.
- `ffcb1ba2f` — `fix(gui): enforce GNSS observation provenance and replay safety`.

Static findings on this baseline:

1. The typed path retains explicit 2D/3D/DGPS categories (5/6/7): native bridge
   mapping → Go `transform.go` public-header pass-through (338–348), or its
   Universal compatibility mapping → `gnssTelemetry.ts` validation →
   `gpsStatus.ts` explicit display branches (42–47). No unconditional erasure
   of these explicit categories was found in those paths.
2. The diagnostics-only fallback is demonstrably less expressive.
   `gpsStatus.ts:281–289` maps `GPS.fix_status` NavSatStatus values 0/1/2 to
   generic `GPS_FIX`; its projection at 347–364 cannot infer dimension or RTK.
   The generic display branch at 48–53 returns UNKNOWN rather than fabricate
   2D/3D/DGPS/RTK. The Universal summary currently contributes `fix_valid`, not
   an explicit category through that projector. This is a precise information
   loss at fallback projection, not proof that the Rock 5B uses that path.
3. `19d99c471` added explicit categories and intentionally replaced the old
   generic “GPS_FIX 25%” presentation with UNKNOWN when dimension is absent.
4. `f56774ea4` introduced strict validation: an optional NaN/null could formerly
   reject an entire message. `ffcb1ba2f` fixed NaN → JSON null according to
   `value_flags`, without converting null to measured zero.
5. `ffcb1ba2f` also rejects contradictions and cross-source enrichment, and its
   hook suppresses invalid, expired or replayed observations. Those deliberate
   protections can look like category disappearance. After a typed frame has
   arrived, the hook does not silently reactivate a diagnostics fallback.

No GNSS corrective code or submodule file was changed. MAVROS checkout remains
`5328dd70d1fce34ee9e0c81ff3883b0260a712de`; the root gitlink remains
`4f1ac1410488b118d2563992df96d08de9730598`, whose object is unavailable locally.
Capabilities of the newer checkout cannot be attributed to that pinned revision
or a deployed image. Universal GNSS remains
`9a329cc216ed137d0e375a4cfe1f5c26e1dd52d0`. The prior installed Python overlay
mismatch is not repaired or claimed tested by this read-only investigation.

### HARDWARE_REQUIRED — locate the Rock 5B GNSS loss

No passive trace from that exact robot was supplied here. Before attributing its
symptom, record the robot/unit and Rock 5B identity, MowgliNext commit plus patch,
GUI/ROS image digests, firmware image, receiver/driver revision, deployed gitlinks,
configured producer and topic ownership. These are presently incomplete; no
hardware acceptance or deployment compatibility is claimed.

With blades disabled, motion prevented and the operator maintaining existing
safety controls, passively correlate the producer's original status, `/gps/status`,
the GUI WebSocket payload and the hook's admissibility/identity state over the
same interval. Include `fix_type`, validity/dimension, backend, flags, nonfinite
availability, observation sequence and receive/producer timestamps where the
real contract supplies them. Do not issue commands, clear state, change datum,
reconnect hardware or manufacture a fix just for this trace.

Pass/fail criterion: a category explicitly present and admissible at each upstream
stage must remain the same downstream. A justified rejection must identify the
first failing contract/invariant; a diagnostics-only source lacking category
information must stay unverified. Without these correlated observations, the
responsible stage in the reported physical case remains undetermined. Any GNSS
correction requires a separate lot and review.

### HARDWARE_REQUIRED — Rock 5B UI acceptance

The automated browser runs do not establish performance on the reported board.
Record its unit identity, actual OS/browser version, reviewed MowgliNext revision
and deployed GUI/ROS image digests, firmware and submodule revisions. With blades
disabled and motion prevented, passively inspect naturally occurring diagnostic
transitions without provoking hardware faults or issuing commands. Open a group
before WARN → OK, keep it open through further publications and any naturally
observed loss/recovery, and exercise keyboard/touch at the actual viewport.
Pass: the episode remains consultable for the specified minimum and while open,
its last report is preserved, lack of evidence is unverified, and the current
verdict changes independently without repeated panel movement or implicit writes.
This passive board/browser acceptance has not been performed here.

## Delivery and limits

- History is local, volatile and deliberately separate from safety/health state.
  It is not a backend log, hardware acknowledgement or acquisition identity.
- A further browser probe found that the existing `AppShell` switches whole
  render trees at its mobile breakpoint, remounting the route. As with navigation
  or reload, that ends this volatile history session. This is separate from ROS
  alert disappearance; changing application-wide route ownership or introducing
  cross-session/global robot state is deferred. Real-page tests cover each
  viewport independently; responsive retention is tested on the same mounted
  panel in the controlled fixture. No shell redesign is claimed.
- Full current data/message changes and new sources can legitimately add content.
  The no-jump checks cover stable-content state transitions in the stated widths
  and locales, not arbitrary unbounded diagnostic text in every browser.
- Current/open/minimum-lifetime entries are protected rather than dropped to
  enforce an artificial total cap; only eligible history is bounded to 100.
- The persistent panel and reserved health-summary cause space add steady layout
  space even with no current fault. Long causes remain accessible by scrolling.
- Existing Ant Design React compatibility notices, jsdom pseudo-element notices
  and build chunk-size warnings are not suppressed or confused with lint debt.
- Go tests are unnecessary for this frontend-only lot; no Go source changed.
  Browser APIs and WebSockets are mocked; no physical robot test was performed.

Review files:

- `gui/web/src/utils/diagnosticAlertHistory.ts` (new)
- `gui/web/src/utils/diagnosticAlertHistory.test.ts` (new)
- `gui/web/src/components/diagnostics/DiagnosticAlertsPanel.tsx` (new)
- `gui/web/src/components/diagnostics/DiagnosticAlertsPanel.test.tsx` (new)
- `gui/web/src/components/diagnostics/DiagnosticHealthSummary.tsx`
- `gui/web/src/components/diagnostics/DiagnosticHealthSummary.test.tsx`
- `gui/web/src/pages/DiagnosticsPage.tsx`
- `gui/web/src/i18n/locales/en.json`
- `gui/web/src/i18n/locales/fr.json`
- `gui/web/tests/e2e/diagnostics-alert-history.spec.ts` (new)
- `gui/web/tests/e2e/fixtures/diagnostic-alert-history.html` (new)
- `gui/web/tests/e2e/fixtures/diagnostic-alert-history.tsx` (new)
- `gui/web/docs/gui-lot27-diagnostics-alert-history.md` (new)

Full review diff: `/tmp/gui-lot27-final.diff`, including new files and excluding
submodules. Logs, browser artifacts and build outputs remain in `/tmp`. Nothing
is staged, committed or pushed. Wait for independent review and operator GO COMMIT.
