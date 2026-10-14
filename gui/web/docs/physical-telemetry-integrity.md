# Physical telemetry integrity — sixth frontend batch

## Baseline and result

2026-10-14, branch `fix/gui-warnings-errors`, baseline commit
`e802638d83763ddaba75f90e0480d955c14ff0aa`. This extends the
[operational telemetry investigation](telemetry-integrity.md) without modifying
its emergency assertion history, status validators, command inputs or safety
contracts. The frontend was initially clean. The pre-existing modified
MowgliMAVROS gitlink/checkout was left untouched; its inspected HEAD is
`5328dd70d1fce34ee9e0c81ff3883b0260a712de`.

| ESLint rule | Initial | Final |
| --- | ---: | ---: |
| `@typescript-eslint/no-explicit-any` | 132 | 130 |
| `@typescript-eslint/no-unsafe-assignment` | 84 | 83 |
| `@typescript-eslint/no-unsafe-member-access` | 70 | 70 |
| `@typescript-eslint/no-unsafe-argument` | 38 | 36 |
| `@typescript-eslint/no-unsafe-call` | 6 | 6 |
| `@typescript-eslint/no-unsafe-return` | 9 | 9 |
| TypeScript safety total | 339 | 334 |
| `react-hooks/set-state-in-effect` | 9 | 9 |
| `react-hooks/purity` | 2 | 2 |
| `react-refresh/only-export-components` | 11 | 11 |
| All warnings | 361 | 356 |
| Errors | 0 | 0 |

All other categories remain zero, including promise and dependency/ref warnings.
No lint ceiling, rule, suppression, dependency, generated contract or CI change.
Raw JSON/logs are local scratch under `/tmp/physical-*`.

## Actual producers and consumers

The path remains ROS message → Foxglove/CDR → Go JSON/cache → MessagePack multiplex
→ existing hook callbacks. No new topic, socket or producer rate was introduced.

| Measurement | Native Mowgli | MAVROS | OpenMower | Frontend consumers |
| --- | --- | --- | --- | --- |
| Power | `hardware_bridge_node.cpp` copies LowLevel battery/charger ADC values and contact bit into `Power`, stamped at host processing | `mavros_battery_observer/battery_observer_plugin.cpp` publishes `PowerState` projections; unavailable channels can be NaN | `openmower_bridge_lowlevel.cpp` reports LowLevel voltage/current with its explicit dock/contact-versus-relay semantics | `usePower`, dashboard, status bar, diagnostics, battery settings illustration; mirrored fleet data uses the battery calculation |
| Blade RPM/current | Native blade report packet populates cached blade fields and `blade_status_time_`; mainboard status carries them with a separate envelope stamp | Role-assigned ESC tracker projection requires observed/online/current-valid/RPM-valid data; zero RPM is a measured stop | xESC Mini parses VESC values; xESC 2040 derives RPM from hall-count changes; bridge carries direct-controller report age in `blade_status_stamp` | `useBladeTelemetry`, dashboard blades/anatomy and diagnostics |
| Blade winding temperature | Blade packet carries ADC `blade_temperature` from firmware; no ESC temperature is assigned in native Status | Winding temperature unavailable; zero default is not a winding measurement | Controller packet exposes motor and PCB temperature separately | Same blade projection and existing anatomy rendering |
| Wheel RPM | Calibrated ticks and direction from firmware | Scaled ESC observations projected into the same WheelTick model | xESC controller counters through bridge odometry | `useWheelRpm`; diagnostics also consumes raw `useWheelTicks` |
| Wheel ESC channels | No role-based wheel ESC measurements are exposed by this GUI contract | `mowgli_mavros_bridge/vesc_{left_wheel,right_wheel}` diagnostics supply source/role, validity flags, age and channels | No equivalent role diagnostics consumed by this view | `wheelControllerFromDiagnostics` and existing wheel field renderer |

Authoritative models reused: generated `Power`, `Status` and `WheelTick` through
`types/ros`, ROS `DiagnosticStatus`/KeyValue shape as modeled by `useDiagnostics`,
and existing backend producer mappings. `WheelTick.wheel_tick_factor` is **float32**,
not an integer: fractional calibrations such as 319.305 remain supported. Signed
ROS seconds remain legal for simulation. The inspected native firmware constructs
blade temperature from the ADC channel (`cpp_main.cpp` blade packet); no physical
sensor qualification is claimed from that source inspection.

## Corrections and root causes

### Power and battery

`physicalTelemetry.ts` validates object shape, normalized stamps, charger fields
and each measurement channel. Finite numeric channels remain numeric; missing,
null, string, NaN and infinite measurements are omitted, never converted to zero.
Negative voltage is invalid; finite signed current remains supported. Actual zero
voltage/current remains a reported channel. Sparse payloads and extension fields
survive. MAVROS can therefore retain valid traction voltage while dock voltage or
current is unavailable. A malformed envelope/identity/control field invalidates
the power payload rather than asserting a charger state.

`usePower` reuses the existing validated wrapper: invalid payloads and connection
loss remove readings; connection-open does not restore them. It retains one
multiplex subscription on the existing power key. No battery expiry deadline was
invented (see freshness limitations below).

Battery interpolation now accepts known finite percentages and fully parsed
numeric/legacy-string thresholds. Invalid percentages fall back to valid voltage.
Missing/nonpositive pack voltage or invalid/equal/inverted thresholds produce an
unknown estimate (NaN), and invalid percentages cannot receive a healthy severity.
The existing default thresholds and successful interpolation are unchanged.
All inspected percentage displays, including mirrored fleet rows, require both
positive finite pack voltage and a finite estimate. High-level percentage zero
still has no validity bit, so the existing voltage fallback is retained; a pack
at/below the empty threshold can still display a valid computed **0%**.

AntD Statistic defaults an undefined value to zero. Diagnostics previously passed
undefined for missing power channels, inadvertently displaying a measured zero.
`powerDisplayValue` now supplies an explicit dash. Existing tiny-noise clamping
is retained only for actual finite channels. Missing raw wheel counts/directions
also no longer render a fabricated zero tick count/direction.

### Blade and wheel measurements

The physical blade decoder validates each channel independently, then reuses the
unchanged Status decoder for the remaining fields. An unavailable MAVROS ESC
temperature no longer removes a simultaneously valid zero RPM/current reading in
the physical display. The safety/status hook's stricter whole-frame validation
is unchanged; it is not relaxed to accept malformed safety bits.

Native Status does not populate ESC temperature: its scalar default zero is now
unknown in the blade display. MAVROS winding temperature stays unknown. OpenMower
reported PCB/motor fields remain separate, including genuine zero readings. An
unidentified/unsupported backend does not establish either temperature capability.
These are reported fields; wiring/validity ambiguity in a producer cannot be
resolved by checking that a JavaScript number is finite.

Blade and wheel hooks previously asserted raw objects as generated interfaces.
They now decode shape/stamps/counters before use. Wheel counters are uint32,
validity masks use the four defined wheel bits, and fractional calibration is
preserved. Unknown direction codes do not establish a signed RPM. Existing
rollover, calibration-change, reset and two-sample calculations remain intact.

An invalid or stamp-less blade frame used to erase its last identity. Replaying
the earlier valid stamp could then restart its five-second display lifetime.
Rejected/stamp-less frames now publish unknown measurements **without resetting
that identity history**. Invalid wheel frames similarly preserve valid continuity
and identity age rather than making a cached sample new. A subsequent genuinely
advancing valid sample can recover, with the existing wheel pair/time gates.

### ESC diagnostics: justified shared change

`useDiagnostics` was assigning an any payload and merging unchecked entries. A
malformed values array could crash downstream consumers or be stamped as newly
received. Its shared boundary now validates ROS diagnostic fields and unique
string KeyValue pairs. A named malformed entry removes the old accumulated entry;
unroutable/malformed envelopes do not refresh valid entries. Other valid entries
in the frame still merge normally.

A valid DiagnosticArray header stamp is tracked as an opaque **publication
identity** per diagnostic name. Repeated/cached publication identities retain
their original browser receipt anchor, including after a malformed named update
has removed the entry. Thus a cached old `age_ms` cannot become recent merely
because the same publication was delivered again. Unknown/zero header identity
retains the legacy receipt-only behavior; it does not acquire a freshness proof.

The wheel ESC consumer additionally guards its entry shape, rejects permissive
numeric conversions such as hexadecimal/empty strings or unit-suffixed values,
and retains explicit validity flags. Negative magnitude RPM/voltage and invalid
unsigned failure/error counters are unknown. Zero RPM/current/temperature remains
valid when the relevant supported producer/validity contract supplies it.

The diagnostics accumulator is shared by GNSS/localizer/other diagnostic views,
which justifies validating the common message shape once there. Its one-second
throttle, accumulation by name, subscription ownership and cleanup remain
unchanged. GNSS logic and backend selection were not rewritten.

## Freshness, provenance and unresolved limits

The existing **5,000 ms motor display gate** and **30,000 ms diagnostic receipt
window** are unchanged. No new physical safety deadline was inferred from a
publication period or from firmware watchdogs.

- Blade report stamps and envelope stamps are compared only in their shared ROS
  clock. MAVROS's independent envelope can precede a newer blade report, which
  remains supported. Browser receipt/identity-change anchors use the existing
  browser clock; no ROS timestamp is replaced with it.
- Wheel speed uses two ROS stamps and encoder deltas; receipt-age display checks
  stay separate. Negative simulation time and clock-reset pair recovery are
  tested. Zero/unavailable blade report stamps remain unknown under the existing
  contract.
- ESC diagnostics combine producer-reported `age_ms` with elapsed time since the
  anchored publication receipt. A newly published diagnostic with an already-old
  acquisition age stays stale. Publication progress alone is not acquisition.
- Power stamps identify reported data but supply no portable acquisition epoch,
  sequence or expiry contract. While a connection remains open, an old power
  sample can remain as a **reported, freshness-unverified value**. A guessed
  five-second battery timeout was deliberately not borrowed from motor display.
  The previous status views' freshness qualification remains in place.

First cached samples, distinct older/reordered identities, publisher restarts and
ROS clock epochs remain ambiguous without producer identity/order. The same-stamp
replay fixes do not establish universal replay protection. OpenMower reconstructs
blade ROS stamps from controller steady age; clock mapping/jitter/reset semantics
remain producer-owned. Diagnostics without an authoritative header identity have
receipt-only liveness, and an initial cached diagnostic age cannot prove current
physical acquisition. History is local to a mounted hook, not a persisted
cross-session ledger. These limitations require producer/cache provenance work,
not additional deadlines.

Native wheel-controller electrical telemetry is not exposed here. OpenMower
controller temperature availability has no frontend per-channel validity bit;
unknown sensor wiring/default conventions require a producer contract and
hardware qualification. No missing channel was filled from another controller,
no raw ESC RPM was treated as calibrated ground/wheel speed, and blade current
was not confused with charge current.

## Tests and modified files

92 new cases cover power shape and partial channels, malformed/nonfinite/numeric
strings, valid zero readings, battery threshold parsing and unknown severity,
fractional encoder scale and uint32 domains, backend temperature capabilities,
malformed ESC diagnostics, unsigned/magnitude domains, cached publication identity,
old acquisition ages, invalid-frame replay without freshness renewal, simulation
clock reset, actual Statistic fallback rendering, and raw wheel formatting inputs.
Lifecycle tests use the actual hooks and shared subscribe API: stable subscriptions,
existing throttle, StrictMode teardown/remount, timer cleanup and connection loss.

Files modified/added under `gui/web`:

- `src/utils/physicalTelemetry.ts`, `src/utils/physicalTelemetry.test.ts`
- `src/utils/diagnosticTelemetry.ts`
- `src/utils/battery.ts`, `src/utils/battery.test.ts`
- `src/utils/motorTelemetry.ts`, `src/utils/motorTelemetry.test.ts`
- `src/utils/fleet.ts`, `src/utils/fleet.test.ts`
- `src/hooks/usePower.ts`, `src/hooks/useBladeTelemetry.ts`
- `src/hooks/useWheelRpm.ts`, `src/hooks/useWheelTicks.ts`
- `src/hooks/useDiagnostics.ts`, `src/hooks/usePhysicalTelemetry.test.tsx`
- `src/components/MowerStatus.tsx`, `src/components/PhysicalMeasurements.test.tsx`
- `src/pages/MowgliNextPage.tsx`, `src/pages/DiagnosticsPage.tsx`
- `docs/physical-telemetry-integrity.md`

No emergency/high-level/status validator, generated model, Go backend, ROS,
firmware, motor command, parameter write or safety-authority source was edited.

## Validation and review requirements

Normal project user, from `gui/web`:

| Check | Result |
| --- | --- |
| Focused measurement/display/consumer suites | 141 tests in 9 files PASS |
| `yarn lint --format json` | 356 warnings, zero errors; no category increased |
| `yarn build` | PASS; existing large-chunk advisory remains |
| `yarn test` | 1,388 Vitest tests across 161 files PASS; 11 separate appearance-registry tests PASS |
| `git diff --check` | PASS |

Recommended mocked browser checks before release: disconnected/partial power,
invalid threshold configuration, measured zero current/RPM, each backend's
unavailable temperatures, cached blade/tick stamps after malformed frames, ESC
cached diagnostics and missing header identity, and mobile unknown-value layout.
Verify normal charging/resume/manual controls alongside display changes; their
handlers, command payloads and authorization rules were not changed. An unknown
estimate is not authorization evidence; robot-side guards remain authoritative.

Physical qualification is **HARDWARE_REQUIRED**, outside this frontend batch.
Software baseline is the commit above plus these uncommitted changes; actual
robot unit, installed backend/firmware images, controller revisions and sensor
wiring are not established here and must be recorded before measurement. Use an
isolated bench with wheels raised, blade disarmed, mowing disabled and physical
STOP accessible; compare reported stopped RPM/current and temperatures/voltages
with the identified controllers' documented validity and an independent reference,
then interrupt telemetry and check unknown/stale displays. Pass requires matching
units/channel provenance, no unavailable channel shown as measured zero, no stale
reading renewed by replay, and no unintended motor/blade command. No hardware
acceptance is claimed from synthetic fixtures.

334 TypeScript warnings and the same 22 non-typing warnings remain. Next work:
producer epochs/observation validity and cache provenance, power acquisition
expiry, controller per-channel temperature validity, and the separate GNSS/backend
identity boundaries. No commit, push, merge or PR was made. The sixth batch remains
uncommitted for review.
