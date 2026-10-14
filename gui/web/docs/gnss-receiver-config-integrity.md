# GNSS receiver actions and runtime configuration integrity — Batch 9

Review baseline: `fix/gui-warnings-errors`, HEAD
`20920bf331c3316899ba1930e16d4cc1d51f4801`, after the user's merge of dev.
The initial worktree contained only the existing local MAVROS submodule
modification. Fresh ESLint JSON established **294 warnings, zero errors** before
changes. The previously reported Vitest baseline was 1,573; this batch does not
assume that count remained unchanged by the merge.

## Producer contract

The authoritative producers are `gui/pkg/api/gnss.go` and
`gui/pkg/api/gnss_runtime_config.go`. No Go handler or receiver command changed.

| Endpoint | Actual response and meaning |
| --- | --- |
| `POST /settings/gnss/plan` | Runs the plan tool without serial access. HTTP 200 carries explicit `success` and command results; a failed plan can still be HTTP 200. No receiver application is performed. |
| `POST /settings/gnss/apply` | Requires `confirm=true`. Loads saved configuration, may update runtime files and stop GPS, detects/validates baud, applies the profile and reconciles GPS after a successful stop. Operation failures and partial persistence/regeneration/restart failures can be HTTP 200. |
| `POST /settings/gnss/factory-reset-apply` | Requires `confirm_factory_reset=true`. Clears receiver configuration before reapplying the saved non-reset profile; partial failure does not imply rollback. GPS restoration is attempted after a successful stop, including failure outcomes. |
| `POST /settings/gnss/restart` | Synchronizes runtime configuration and reconciles the GPS service. Successful responses explicitly report operation success and restart attempted/succeeded. Errors can be error-only HTTP bodies after some work has occurred. |
| `GET /settings/gnss/runtime-config` | Resolves configuration from saved YAML, environment fallback and defaults, and inspects/globs local device paths. It does not query or attest the running receiver's serial communication. |

Action responses always have a boolean `success` in the current producer.
`partial_failure`, recovery and restart flags are often omitted when false.
Executions contain a string tool name, string command array, integer exit code
and boolean `success` derived from `exit_code == 0`; stdout/stderr are optional.
An earlier failed detection execution may precede successful manual recovery;
individual command failures therefore cannot replace the overall operation flag.

The apply detection branch sets `success` from tool exit status before checking
whether a baud was reported. Consequently, exit-zero output lacking detected
baud can pair `success=true` with `manual_baud_required=true`. The frontend gives
the explicit recovery requirement precedence and does not announce successful
application. This is code-path evidence, not an observed hardware outcome.

After a successful GPS stop, deferred restoration uses a cleanup context even
when the HTTP request was cancelled. A failed restore without an earlier error
sets partial failure, operation failure and restart error. With an earlier error,
the error-only HTTP response can hide detailed cleanup outcomes. The browser
cannot infer that no receiver or runtime-file changes occurred from such an error.

## Decoding and truthful outcomes

`gnssConfigContract.ts` accepts unknown bodies and validates only the fields
consumed by these UI flows. No cast supplies runtime authority. Missing/nonboolean
`success`, malformed strings/booleans, noninteger exit codes, malformed command
arrays and invalid optional fields reject the acknowledgement. Baud fields remain
strings: positive decimal baud values, legitimate empty values and execution
`auto` are distinguished, without numeric/null coercion or limiting detected
baud to the GUI's four manual options. Contradictory command-success/exit-code
pairs or positive restart confirmation with explicit restart error/no-attempt
are rejected.

Sparse compatible acknowledgements may omit action metadata, warnings,
executions and optional flags, but must explicitly state success or failure.
An action name, when supplied, must match the dispatched endpoint through the
explicit route-to-producer identifier mapping. In particular, the reset URL
segment `factory-reset-apply` maps to Go action `factory_reset_apply`. Command
entries, when supplied, must match the producer's required execution fields.
No missing completion flag becomes true or a measured zero.

One shared outcome function governs summary severity, result tag and notification:
explicit partial failure takes priority; operation failure, required manual baud
recovery and reported restart error cannot be promoted to success. An attempted
restart needs explicit positive confirmation to display confirmed restart;
missing confirmation is unavailable rather than silently converted to false.
Valid operation failure and partial failure retain their detailed results and
manual recovery controls. Plan success explicitly says it made no receiver
changes.

HTTP errors, missing data, invalid acknowledgements and network rejections clear
old result details and report **unconfirmed operation outcome**. They never
produce a successful result, automatic retry or a claim that the operation did
not occur. Failed prerequisites instead say that no GNSS request was dispatched.

## Side effects and lifecycle

Apply/factory-reset confirmation dialogs remain required, with their original
confirmation wire keys. Their explanatory text now matches the producer's
best-effort GPS restoration, including on receiver failure. No confirmation is
inferred from a save, a device path or a response's HTTP status.

A synchronous per-card lock covers prerequisite persistence and the action
request. Repeated clicks/confirmation callbacks cannot dispatch duplicate or
overlapping actions while that work is pending. `onPersistBeforeAction()` must
return exactly `true`; false, undefined or rejection prevents dispatch. Dirty
settings with no persistence prerequisite also cannot authorize an action.
Successful prerequisites are called once. The pending display distinguishes
saving prerequisites from waiting for the GNSS response.

Unmounting before prerequisite completion prevents GNSS dispatch. Responses
arriving after unmount do not update results or notify. Unmounting after dispatch
does not pretend to cancel or undo the server operation. No side-effecting request
is retried automatically, including malformed acknowledgements. Manual baud
selection changes only the requested recovery setting through its existing
callback; detected, verified, configuration and runtime baud remain separately
reported values.

## Runtime configuration authority

The hook retains its five-second poll and stable API dependency. Completion order
rejects older responses after a newer completion, including newer failures; it
does not discard every slow response merely because another poll has started.
Cleanup cancels UI acceptance and clears the timer. Failed/malformed refreshes
clear current metadata and expose an error.

The decoder preserves configured/environment/active/source strings, legitimate
empty strings, optional availability fields and device-presence booleans. Sparse
older payloads can omit values/availability, but an unrelated or empty envelope
is not a configuration response. Unknown source labels remain unknown; they do
not become default/config authority. Runtime baud text is configured metadata,
so it is not coerced into an acquired baud measurement. Unconsumed transport and
NTRIP projection fields are not treated as extra hardware capabilities.

The serial-device field now labels this projection **resolved configuration**.
When metadata is unavailable, the editable draft remains available, but its path
and a default baud are not displayed as reported active values. File existence
means only the producer's local stat observation, not connected GNSS hardware or
successful serial communication. Receiver-family/model telemetry derivation and
capability masks were not changed.

## Changed files

- `src/components/settings/GnssReceiverActionsCard.tsx` and new `GnssReceiverActionsCard.integrity.test.tsx`.
- `src/hooks/useGnssRuntimeConfig.ts` and new `useGnssRuntimeConfig.test.tsx`.
- New `src/utils/gnssConfigContract.ts` and `gnssConfigContract.test.ts`.
- `src/components/settings/gnssConfig.ts`: own-key GNSS settings projection retains unknown values instead of propagating any.
- `src/components/settings/GnssSerialDeviceConfigField.tsx` and its existing test: unavailable/provenance display and a typed hook mock.
- `src/i18n/locales/{en,fr}.json` and this report.

Existing action-card and settings-helper tests remain unchanged and are part of
focused validation. No GNSS driver, Universal GNSS, telemetry hook, ROS message,
RTCM route, firmware, hardware authorization, YAML persistence or MAVROS source
changed. All logs and build output are under `/tmp/gui-batch9-*`.

## Compatibility limits and deferred work

Explicit success is required for old/custom servers as well as current producers.
Previously tolerated empty bodies or malformed command results now fail visibly.
The client validates consumed fields, not arbitrary tool stdout/receiver protocol,
and does not infer acquisition timestamps, GNSS capabilities or physical freshness.
A malformed acknowledgement may arrive after the operation has already run.
Error-only producer responses do not establish which earlier side effects or
cleanup steps completed. GPS reconciliation success is the server-reported command
outcome, not confirmation of a new GNSS fix.

The lock is per mounted card; it supplies no cross-tab idempotency, server-wide
serialization or transaction across YAML, receiver commands and restart. Other
GNSS/map/navigation warning clusters need their own reviews and were not changed
for warning-count reduction. No physical commands or container restarts were
performed in tests. No hardware acceptance is claimed.

No commit, push, merge, rebase, reset or clean was performed. Existing local MAVROS
changes were preserved.

## Warning inventory and validation

Fresh post-merge ESLint JSON: **294 → 263 warnings (−31), zero errors before and
after**. All reported warnings are maintained code; generated exclusions and
existing generated-file suppressions are unchanged. No new suppression, cast to
any, rule change or warning-ceiling change was introduced.

| Rule | Before | After |
| --- | ---: | ---: |
| `@typescript-eslint/no-explicit-any` | 120 | 107 |
| `@typescript-eslint/no-unsafe-argument` | 35 | 34 |
| `@typescript-eslint/no-unsafe-assignment` | 57 | 55 |
| `@typescript-eslint/no-unsafe-call` | 6 | 4 |
| `@typescript-eslint/no-unsafe-member-access` | 48 | 38 |
| `@typescript-eslint/no-unsafe-return` | 6 | 3 |
| `react-hooks/purity` | 2 | 2 |
| `react-hooks/set-state-in-effect` | 9 | 9 |
| `react-refresh/only-export-components` | 11 | 11 |

| Changed warning cluster | Before | After |
| --- | ---: | ---: |
| `src/components/settings/GnssReceiverActionsCard.tsx` | 20 | 0 |
| `src/components/settings/GnssSerialDeviceConfigField.test.tsx` | 1 | 0 |
| `src/components/settings/gnssConfig.ts` | 4 | 0 |
| `src/hooks/useGnssRuntimeConfig.ts` | 6 | 0 |

The receiver actions card, runtime-config hook, settings projection helper and
serial-device test mock now have zero warnings. Newly added contracts/tests and
the directly related serial-device display have zero warnings. Other clusters
remain unchanged. Raw JSON inventories are outside version control at
`/tmp/gui-batch9-eslint-{before,after}.json`.

- Focused action/contract/runtime/helper/serial-device/locale checks: **89 tests passed across 7 files**.
- Full `yarn test`: appearance registry passed; **1,643 Vitest tests passed across 170 files**.
- TypeScript (`yarn tsc --noEmit`): passed.
- Frontend build (`yarn build --outDir /tmp/gui-batch9-dist`): passed, with the existing large-chunk advisory.
- `git diff --check`: passed.
- Go tests not rerun: no Go code or server contract changed.

Regressions cover plan/apply/reset/restart success, confirmation keys, HTTP error
with empty data, explicit failure/partial failure, malformed/missing success and
execution arrays, restart failure/unconfirmed completion, contradictory restart
flags, required manual baud recovery, failed/unconfirmed prerequisite saves,
duplicate confirmations/clicks, unmount before and after dispatch, network errors
after a successful result, manual selection without automatic retry, five-second
poll cadence, malformed/failed refreshes, older completions, unavailable device
metadata, sparse compatible responses and legitimate empty values. APIs and
container operations are mocked; no receiver command or restart is executed.

## Factory-reset contract review follow-up

Go `runFactoryResetApplyFlow` constructs its response with
`newGNSSActionResponse("factory_reset_apply", ...)`; the URL segment remains
`factory-reset-apply`. The initial frontend decoder and test fixture incorrectly
used the URL segment for both contracts. The decoder now separates route names
from producer action identifiers with an explicit four-entry map. No other
identifier is accepted, and reset identifiers are rejected for other routes.
Omitted action metadata remains compatible with the already documented sparse
acknowledgement contract.

Updated decoder and card tests use the literal Go-produced underscore identifier
for both success and partial failure, confirm the original URL/confirmation key,
and verify consistent notifications/result tags. Negative tests cover unrelated
producer identifiers, the incorrect hyphenated response identifier and reset
identifiers returned to other routes. Card production dispatch logic is unchanged.

Follow-up validation: **101 focused tests passed across 7 files**, TypeScript
passed, full ESLint JSON remained **263 warnings / zero errors**, and
`git diff --check` passed. The preceding full-suite/build results above are the
Batch 9 validation before this focused contract correction; they were not rerun
for this follow-up. Existing changes and the local MAVROS modification were
preserved. No commit or push was performed.
