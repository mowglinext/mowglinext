# Batch 7A — GNSS telemetry integrity

Software baseline: `dc65e2a8061df7c671aecd26cc8580917ca4aa1b` on
`fix/gui-warnings-errors`, plus this uncommitted patch. This batch concerns GUI
GNSS interpretation only. No receiver configuration, hardware routing, ROS
producer, MAVROS source, generated model or backend selection changed.

## Decoding boundary

`multiplexedSocket.ts` decodes MessagePack `{topic, data}` envelopes and delivers
the decoded `data` object. `useWS<unknown>` preserves that boundary for GNSS;
the old `payload as unknown as GnssStatus` assertion did not validate anything
and could not cause the surrounding catch to reject bad fields.

`gnssTelemetry.ts` validates the fields consumed by the GNSS views: plain object
shape, ROS header/stamp, receiver strings, boolean assertions, public fix/RTK/
baseline/correction-stream enum domains, uint32 masks, uint16 counters, finite
measurements and nonnegative magnitudes/ages. Absent optional properties remain
absent. Present null, undefined, numeric strings and wrong types are malformed.
Empty/header-only/extension-only frames do not establish a typed sample. Unknown
enum codes reject the frame and display unknown; they cannot select a valid fix.

Only consumed fields are retained. Unconsumed extensions, including the producer
quality score and observation sequence, are not used to manufacture display
percentages or acquisition identity. Unknown mask bits are retained as uint32
extensions but do not establish support for a known field. Current-value bits
must be a subset of supplied capabilities and must have their consumed fields.

MAVROS deliberately supplies NaN for unavailable float channels and its UI
quality score (`mavros_gnss_adapter/include/mavros_gnss_adapter/status_mapping.hpp`).
Consumed float NaN is omitted only when an explicit value mask marks that channel
unavailable. Nonfinite values claimed as available, infinities, strings and null
are rejected. Genuine reported zero values remain valid. Missing measurements
are never filled with zero.

## Fix categories and authority

The existing categories and percentages are retained:

| Evidence | Label | Percent |
| --- | --- | --- |
| Unknown, generic GPS, unavailable or dead reckoning | — | 0 |
| Explicit 2D fix | 2D | 20 |
| Explicit 3D fix | 3D | 40 |
| Explicit DGPS | DGPS | 60 |
| RTK float evidence | FLOAT | 80 |
| RTK fixed evidence | FIX | 100 |

`fix_valid=false` wins over contradictory positive codes. NO_FIX and dead
reckoning cannot be promoted by RTK mode. RTK mode alone does not establish valid
positioning: a positioning fix type or explicit `fix_valid=true` is required.
An available RTK mode can refine a generic positioning fix. Explicit RTK NONE
conflicting with an RTK fix type remains unknown. An unavailable/default RTK
scalar cannot refine a basic fix; an explicit RTK fix type still supplies its
own evidence, matching the public ROS helper's fix-type fallback semantics.

| Field/group | Precedence |
| --- | --- |
| Typed fix validity, fix type, RTK mode, receiver/backend identity | Typed sample only; diagnostics never fill positioning or identity around it |
| Explicit capability/value masks | Producer owns supported, unsupported and currently unavailable states; diagnostic masks are not OR-ed into them |
| Typed optional group, including explicit false/unknown | Preserved as a group; older diagnostics cannot fill around its negative assertion |
| Legacy typed core-only sample without masks or optional channels | Recognized correction-forwarding/MSM diagnostics may enrich missing groups |
| No typed frames delivered | Limited diagnostic-only fallback remains supported, including after reconnect |
| Typed stream delivered invalid/unsupported data or lost connection | Unknown display until another validated typed frame; diagnostics cannot resurrect the old fix |

A generated mask cannot express partial authority. Diagnostic enrichment is
therefore withheld when a mask-less typed sample already has optional channels:
adding a correction-only mask would incorrectly mark its typed RTK/channel
evidence unsupported. This conservative limitation preserves existing evidence
without inventing typed capabilities or borrowing another producer's position.

Diagnostic numeric strings are parsed completely, not with permissive
`parseInt`/`parseFloat` prefixes. NavSatFix diagnostic statuses 0/1/2 remain generic
GPS; SBAS/GBAS is not evidence of 3D or RTK. Explicit GPS NO_FIX or summary
`fix_valid=false` wins over contradictory positives. Missing diagnostic fields
remain missing. An MSM current-value group requires its three explicit boolean
states; absent counts/ages are not defaulted to zero. Correction forwarding
`inactive` cannot match `active`. Forwarding activity is not portable-RTK semantic
health or proof of a valid position.

## Producer contracts and limitations

- **Mowgli/native GNSS:** `sensors/gps/mowgli_gnss_bridge/src/universal_gnss_topic_bridge.cpp`
  projects Universal GNSS into `ros2/src/mowgli_interfaces/msg/GnssStatus.msg`.
  Generic FIX lacks dimensionality. Optional support/current values use masks;
  correction diagnostics are separately projected by the bridge's tracker.
- **MAVROS:** the adapter's public status mapping under
  `ros2/src/external/mowglimavros/ros2/src/mavros_gnss_adapter/` preserves mapped
  capability/value flags and marks unavailable floats with NaN. Its upstream
  transport does not supply UI quality or NTRIP/MSM truth. None is inferred here.
  This is an integration reference, not new validation of that submodule.
- **OpenMower:** `sensors/openmower/mowgli_openmower_bridge/src/openmower_bridge_node.cpp`
  consumes the shared `/gps/status`; the hardware backend does not establish a
  different rich GNSS contract. Hardware quality percentage is not a substitute
  for typed receiver evidence.
- **GUI compatibility adapter:** `gui/pkg/providers/transform.go:adaptGnssStatus`
  passes public header-bearing payloads through and maps legacy Universal stamp-
  bearing payloads. Browser validation follows that public snake-case contract.
  Information already discarded by an upstream mapping cannot be reconstructed.

`useValidatedTelemetry` has two small passive extensions needed here: an optional
identity reader (GNSS uses its existing header stamp rather than a top-level stamp)
and delivery-history metadata. Existing hooks retain their default identity
reader and behavior. Delivery history distinguishes a malformed typed producer
from a legacy producer that never delivers this topic, including reconnects.

Invalid frames and socket-open events do not update the last valid browser
monotonic receipt or publication-identity change time. Identical header replay
retains its original identity-change anchor, even through a malformed frame.
Reception of another valid frame can update **receipt** time. Neither reception
nor header progress proves physical GNSS acquisition. No new expiry window,
source epoch, acquisition timestamp or source-ordering guarantee was invented.

Diagnostic-only fallback remains a reported, freshness-unverified projection;
its accumulated entries have the existing diagnostic receipt/publication limits.
It does not provide detailed fix categories, physical acquisition freshness or
cross-producer ordering. Cached initial data, reordered distinct stamps, source
restarts, remounts and clock epochs remain ambiguous. Observation-sequence
guarantees and correction source identities remain producer-owned. No new
physical or hardware acceptance claim follows from these synthetic tests.

## Validation and modified files

Validation uses the normal project user in `gui/web`; logs and frontend build
outputs are outside version control under `/tmp/gui-7a`.

| Check | Result |
| --- | --- |
| Focused GNSS, hook, cards, readiness and mower-status suites | 156 tests across 7 files PASS |
| `yarn test` | 1,461 Vitest tests across 163 files PASS; appearance-registry check PASS |
| `yarn lint --format json` | 356 warnings before → 356 after; zero errors; every warning category unchanged |
| `tsc --noEmit` | PASS |
| `yarn build --outDir /tmp/gui-7a/dist` | PASS; existing large-chunk advisory remains |
| `git diff --check` | PASS |

The unsafe GNSS payload assertion was not an ESLint-reported warning, so runtime
validation does not lower the numeric warning debt. There are no new warnings or
lint suppressions: the existing 334 TypeScript warnings and 22 other warnings
remain. Focused regression coverage includes all five valid positioning categories,
explicit negatives, malformed/incomplete frames, unknown enums, optional NaN and
zero channels, typed/diagnostic contradictions, legacy diagnostic-only reconnect,
publication replay through invalid frames, socket-open behavior, stable
subscriptions, StrictMode cleanup and diagnostic timer teardown.

Modified/added files:

- `src/utils/gnssTelemetry.ts`, `src/utils/gnssTelemetry.test.ts`
- `src/utils/gpsStatus.ts`, `src/utils/gpsStatus.test.ts`
- `src/hooks/useGnssStatus.ts`, `src/hooks/useGnssStatus.test.tsx`
- `src/hooks/useValidatedTelemetry.ts`
- `src/components/gnss/GnssLiveStatusSummaryCard.tsx`
- `src/components/gnss/GnssLiveDiagnosticsCard.tsx`
- `src/components/onboarding/readinessChecks.ts`, `readinessChecks.test.ts`
- `src/pages/OnboardingPage.tsx`
- `docs/gnss-telemetry-integrity.md`

The existing MAVROS gitlink difference was left untouched. No commit, push or
merge was performed. Remaining warning debt and producer provenance/freshness
limitations are outside this batch.
