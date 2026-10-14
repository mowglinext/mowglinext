# ROS 2 telemetry integrity and runtime validation

## Baseline and outcome

2026-10-14, branch `fix/gui-warnings-errors`, baseline commit
`0d2a10932e19c0935cb623387e0760dd32b3b681`. This follows the
[promise](async-promise-safety.md), [lifecycle](react-hooks-lifecycle-safety.md),
[type-boundary](typescript-data-boundaries.md) and
[persistence](config-persistence-integrity.md) investigations. The frontend was
clean initially; the existing modified MowgliMAVROS submodule was left untouched.
All task changes are within `gui/web`.

Initial/final ESLint JSON is local scratch under `/tmp/telemetry-lint-*.log`.
Counts include source and test findings, excluding existing generated-code
exemptions. No rule, suppression, warning ceiling, dependency or CI change was made.

| Rule | Initial | Final |
| --- | ---: | ---: |
| `@typescript-eslint/no-explicit-any` | 133 | 132 |
| `@typescript-eslint/no-unsafe-assignment` | 84 | 84 |
| `@typescript-eslint/no-unsafe-member-access` | 70 | 70 |
| `@typescript-eslint/no-unsafe-argument` | 39 | 38 |
| `@typescript-eslint/no-unsafe-call` | 6 | 6 |
| `@typescript-eslint/no-unsafe-return` | 11 | 9 |
| TypeScript safety total | 343 | 339 |
| `react-hooks/set-state-in-effect` | 9 | 9 |
| `react-hooks/purity` | 2 | 2 |
| `react-refresh/only-export-components` | 11 | 11 |
| All warnings | 365 | 361 |
| Errors | 0 | 0 |

Floating/misused promises and other hooks rules remain at zero. Two findings
were removed at the high-level message boundary and two by typing the existing
MowerStatus test mocks. Runtime integrity, rather than warning reduction, is the
purpose of this batch.

## Actual producer-to-consumer path

The GUI does not receive JSON text on the multiplex path:

1. ROS producers publish `/hardware_bridge/emergency`, `/hardware_bridge/status`
   and `/behavior_tree_node/high_level_status`, with authoritative contracts in
   `ros2/src/mowgli_interfaces/msg/{Emergency,Status,HighLevelStatus}.msg`.
2. Foxglove exposes CDR messages. The Go Foxglove/provider layer decodes them and
   `RosProvider.fanOut` caches JSON per logical topic. `Subscribe` can immediately
   replay the cache; the last listener's removal drops non-virtual topic caches.
3. `MultiplexRoute` encodes decoded objects as MessagePack `{topic,data}` frames.
   `topicSubscribeInterval` is unthrottled for all three topics.
4. `multiplexedSocket.ts` unpacks each frame once and dispatches `unknown` objects
   to ref-counted listeners. `useWS` passes those objects to committed callbacks.
   Its generic type parameter was previously mistaken for runtime validation.
5. The three status hooks now use a small validated wrapper over the existing
   `useWS` contract. `useTopic` and the transport implementation are unchanged.

Consumers include the dashboard, global status bar, live strip, diagnostics,
firmware handshake selectors, calibration helpers and the map's manual/recording
stream policy. The map continues to receive the existing stable high-level
`start(uri)` and `stop()` functions. Intentional stop retains its snapshot as
before; socket failure invalidates operational values.

## Contracts and decoders

`utils/statusTelemetry.ts` reuses generated ROS types/constants through `types/ros`;
no generated file or ROS interface was edited. Its complete known-field tables
are checked against the generated types at compile time. Runtime guards validate
plain/null-prototype objects, present booleans/strings, finite measurements,
int16 indices (including negative sentinels), uint8 domains, percentages and
normalized ROS stamps. Numeric high-level states 0–4, mower status 0/255, known
reset causes and blade request labels are checked explicitly.

Present null, undefined, wrong types, non-finite values and invalid numeric ranges
reject the frame. Valid but unsupported codes are distinguished from malformed
representations. No coercion or observed numeric/boolean default is introduced.
Sparse records, including `{}`, remain sparse: structural validity does not mean
that operational status is known. Backend extension fields and extensible
state/substate labels survive; they are not inferred to be new known numeric
states. Frontend clearance metadata is calculated separately and overwrites any
similarly named untrusted extension, rather than accepting it as a producer claim.

A routed invalid/unsupported topic payload removes operational/firmware fields instead of
keeping an old healthy sample as the current payload. Previously reported true
emergency, lift or dig warnings are conservatively retained when a bit is absent.
Repeated rejected frames of the same class reuse state instead of publishing
another snapshot. Decoding consists of a few dozen scalar checks per listener;
no large geometry/array decoding or new throttle is introduced.

## Emergency assertions, replay and clearance

Structural validity cannot establish emergency clearance. The contracts lack a
producer epoch and ordered acquisition sequence; high-level status has no stamp
at all. An apparently larger stamp is not a portable proof of a newer physical
observation. It would be unsafe to manufacture such a guarantee from browser
receipt time, compare unrelated clocks, or infer an epoch from reconnect.

The wrapper therefore stores **assertion history separately from latest payload
bits**. Once a valid frame reports active/latched emergency, that history is not
acknowledged cleared by an unstamped false bit, a replayed earlier frame, or a
larger publication stamp. After such a frame, status views show **“Emergency was
reported · clearing unverified”**, rather than declaring the observed assertion
cleared. High-level emergency history receives the same treatment. Invalid data,
socket drops and connection-open notifications cannot clear history.

This is historical display evidence, not an invented firmware latch. Latest
structurally valid payload bits remain available to existing command-authorization
consumers. The dashboard's action phase uses those original command inputs,
separately from historical warning emphasis. No command, rearm request, retry,
calibration or motor action is sent by validation or history tracking. Existing
physical/ROS safety authorities and command payloads remain unchanged.

**Limitation:** no current producer supplies a cross-backend trustworthy clearance
acknowledgment. Consequently, even a legitimate later clear cannot automatically
acknowledge this historical display warning under the current contract. History
is scoped to the mounted hook; it is not persisted across a real unmount, reload
or another tab. Reloading is not evidence of clearance. Initial cached false bits
also cannot prove that hardware is currently safe. Existing command guard inputs
remain unverified telemetry; this batch does not elevate them to an authorization
proof. A future producer/API change must supply source identity, epoch and ordered
observation semantics before verified automatic clearance can be implemented.

A false high-level bit no longer masks a hardware active/latched assertion in
status displays. Notifications retain active/latch distinctions. A rearm button
still makes an operator-initiated request; service acceptance is not a verified
physical clearance acknowledgment.

## Timestamp and freshness semantics

| Concept | Actual evidence | Treatment |
| --- | --- | --- |
| Physical acquisition | Not portably represented by these contracts | Never inferred from arrival or connection state |
| Native/OpenMower stamp | ROS host `now()` when processing LowLevel status packets | Preserved as producer-clock identity; not compared to browser time |
| MAVROS general status stamp | Readiness state's cached ROS receipt stamp | May repeat while the bridge republishes or updates other fields |
| MAVROS emergency stamp | `now()` for each safety-state projection/publication | Publication progress is not proof of new physical acquisition |
| High-level status | Cached snapshot, also periodically republished by the BT | No stamp, epoch or observation sequence; acquisition freshness unavailable |
| Browser receipt | `performance.now()` when a structurally valid frame arrives | `lastValidReceiptAt`; delivery evidence only |
| Identity change receipt | Browser time when a nonzero stamp identity changes | `stampChangedAt`; repeated identities do not advance it, even after reconnect |
| Connection | Shared socket open/connecting/closed and existing silence watchdog | Transport liveness only |

Negative ROS simulation stamps are valid; zero stamps remain unknown identity.
An identity change can also be a clock regression/reset, so it is deliberately
not called a newer acquisition. Invalid frames do not advance valid receipt or
identity metadata. Reconnect/open does not restore a valid payload; a subsequent
frame must pass validation, and a cached valid frame still does not prove freshness.

The existing 30-second transport silence watchdog is unchanged. It detects an
absence of subscribed continuous traffic collectively, not silence on one topic.
Other topics can keep the connection open while one producer stops. Undecodable
or unroutable transport envelopes never reach a topic decoder; those retain the
qualified reported snapshot rather than manufacturing a topic-specific invalidity. There is no
authoritative per-topic acquisition expiry for this batch, so none was invented.
Status views explicitly qualify retained/replayed values as **reported status,
freshness unverified**. This makes the uncertainty visible; it does not implement
physical stale-sample rejection or fix the missing producer provenance contract.

The investigated diagnostics accumulator has its existing 30-second receipt-age
contract; GNSS carries richer producer observation identity/age semantics; blade
and wheel display utilities already distinguish their specific identity and
receipt-age gates. Those different domains were not collapsed into this wrapper.
GNSS diagnostics/status hooks still contain unchecked assertions and are deferred.

## Display corrections and lifecycle

Missing status no longer synthesizes docked/idle in the global status bar or
ready/idle dashboard headlines. Missing emergency information is unknown rather
than “clear/no alerts.” Missing rain and hardware booleans display unknown, and
initializing mower status remains distinct from OK. Diagnostics retain real
critical verdicts from other known threats even if emergency information is absent.

Subscriptions remain multiplexed, unthrottled and unpaused for safety/status
messages. `useWS` retains its committed callback refs and listener cleanup; no
transport reconnection, message ordering, visibility policy, map control, GNSS,
battery calculation, geometry, backend selection or ROS safety implementation
was changed. No rules were suppressed and no new any/casts were introduced.

## Files and tests

Modified/added files under `gui/web`:

- `src/utils/statusTelemetry.ts`, `src/utils/statusTelemetry.test.ts`
- `src/hooks/useValidatedTelemetry.ts`, `src/hooks/useStatusTelemetry.test.tsx`
- `src/hooks/useEmergency.ts`, `src/hooks/useStatus.ts`, `src/hooks/useHighLevelStatus.ts`
- `src/components/MowerStatus.tsx`, `src/components/MowerStatus.test.tsx`
- `src/components/LiveStatusStrip.tsx`, `src/components/AppShell.tsx`
- `src/pages/MowgliNextPage.tsx`, `src/pages/DiagnosticsPage.tsx`
- `src/i18n/locales/en.json`, `src/i18n/locales/fr.json`
- `docs/telemetry-integrity.md`

86 new regression cases cover complete/sparse/backend-extension messages; null,
arrays/scalars; every known field's wrong types; NaN/Infinity/ranges; enum and stamp
validity; emergency retention through invalid/sparse frames; replayed false bits
following an assertion; reconnect and larger publication stamps; separate history
and command inputs; untrusted metadata collisions; valid structural recovery;
receipt versus identity changes; repeated rejection bailouts; stable callbacks,
one effective listener per hook and StrictMode teardown/remount. Status bar tests
cover unknown state, assertion precedence, unverified clearance, notification
failure preservation and absence of automatic rearm commands.

## Validation

Normal-user validation from `gui/web`:

- Focused transport/status/manual/map/action suites: **137 tests in 10 files PASS**.
- `yarn lint --format json`: **361 warnings, zero errors**; no category increased.
- `yarn build`: **PASS**, TypeScript and Vite production build. The existing
  large-chunk advisory remains.
- `yarn test`: **1,296 Vitest tests in 157 files PASS**, plus 11 separate Node
  appearance-registry tests.
- `git diff --check`: **PASS**.

Logs are ignored local scratch under `/tmp/telemetry-*`, not a versioned archive.
No robot, browser/mobile visual acceptance or power/hardware acceptance is claimed.

## Remaining work and compatibility

339 TypeScript safety warnings remain; the existing eleven hooks and eleven
Fast Refresh findings are unchanged. Recommended next work:

1. Separately review producer identity/epoch/observation sequence and backend cache
   provenance, including ROS/simulation clock reset behavior. Establish trustworthy
   ordering before implementing clearance acknowledgment or acquisition expiry.
2. GNSS topic/diagnostic decoding: retain receiver acquisition identities, fix-valid
   distinctions and optional capabilities; never infer RTK FIX from missing data.
3. Power/battery and motor boundaries: retain unavailable measurement markers and
   existing per-source stamps/age contracts; do not turn missing data into zero.
4. Backend identification: separate absent legacy metadata from explicitly
   unsupported producers before selecting backend-specific models/routes.

Native Mowgli and OpenMower publish status bundles from different controllers;
MAVROS readiness, timer-projected emergency and ESC reports have different
provenance and optional fields. Firmware compatibility remains a native-handshake
signal, not an autopilot health certificate. A malformed optional measurement can
invalidate a complete general-status frame; the resulting unknown state is
intentional and needs mocked backend fixtures during release review.

Recommended browser checks on an isolated mocked backend: active and latched
emergency followed by invalid/replayed/unstamped clear frames, valid but unverified
clearance after rearm, missing optional status fields, one silent topic while
others remain active, disconnect/reconnect with caches, StrictMode and map/manual
entry/exit, diagnostics unknown indicators, mobile status text wrapping, and no
repeated command issuance. Verified automatic clearance remains unresolved and
must be reviewed before claiming the display represents current physical safety.
No commit, push, merge or PR was created; changes remain uncommitted for review.
