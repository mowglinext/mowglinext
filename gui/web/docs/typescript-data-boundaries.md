# TypeScript type safety and external data boundaries

## Baseline and first-batch outcome

2026-10-14, branch `fix/gui-warnings-errors`, baseline commit
`16a221aa5a8faa1642ed389aed801840d8cb8025`. The frontend was initially clean.
This follows [promise safety](async-promise-safety.md) and
[hooks/lifecycle safety](react-hooks-lifecycle-safety.md). The existing modified
`ros2/src/external/mowglimavros` checkout was left untouched.

ESLint JSON confirmed exactly 369 TypeScript safety findings across 71 source/test
files, within 391 warnings and zero errors. The first batch is intentionally
limited to shared request-error handling and GUI config/environment JSON.

| TypeScript rule | Initial | Final |
| --- | ---: | ---: |
| `no-explicit-any` | 144 | 133 |
| `no-unsafe-assignment` | 91 | 84 |
| `no-unsafe-member-access` | 77 | 70 |
| `no-unsafe-argument` | 40 | 39 |
| `no-unsafe-call` | 6 | 6 |
| `no-unsafe-return` | 11 | 11 |
| Total TypeScript safety warnings | 369 | 343 |
| All ESLint warnings | 391 | 365 |
| ESLint errors | 0 | 0 |

The remaining 22 non-typing warnings are unchanged: nine state-in-effect,
two purity, eleven Fast Refresh. Floating/misused promises remain zero;
dependency/ref/memoization/immutability warnings remain zero. No rule, warning
ceiling, dependency version, CI configuration or generated file was changed.

## Audit method and shared root causes

Initial raw output: `/tmp/gui-types-initial.json`; final output:
`/tmp/gui-types-final.json`. `/tmp/gui-types-audit.json` classifies every one of
369 findings by file, line, column, rule, data origins, shared cause, potential
runtime impact and fix complexity. These are local scratch files; the complete
file/count/provenance and initial-site inventory is retained below.

Classification uses file-level provenance groups. A file can mix multiple data
origins, and a warning is not proof that a runtime failure occurred. Generated
files and the existing third-party mode lint exemptions are outside this count;
absence of lint findings does not establish runtime validation. The audit also
identified asserted external payloads in already lint-clean code.

| Group | Shared cause | Why it matters | Typical fix |
| --- | --- | --- | --- |
| A | REST JSON or generic request response asserted as a domain model | Compile-time generics do not check a backend body; fields can be absent or wrong-typed | Read unknown, decode the actual endpoint contract, then publish state |
| E | Raw request rejection treated as Error or accessed through any | Generated client can reject with an HttpResponse/error body, losing the backend reason | Shared guarded string extraction; preserve operation-specific fallback/error UI |
| S | Dynamic settings/field callbacks use Record<string, any> | Boolean, string, numeric, null-deletion and backend-specific values leak between domains | Reuse SettingsValues/schema; validate writes at the domain boundary |
| W | ROS/WS frames cast to topic interfaces | Missing measurements can appear valid; malformed arrays/stamps/statuses can break derivation | Topic-specific decoders, optional values, finite checks and preserved producer/freshness identity |
| M | Mapbox Draw/GeoJSON adapters and feature-class conversion remain untyped | Bad event/geometry shape can affect selection, area identity and imports | Existing library types plus event/geometry guards; retain stable IDs and listener lifecycle |
| B | Legacy browser property outside standard Navigator typings | Narrow browser feature detection uses assertions | Guard the optional capability without changing detection semantics |
| T | Untyped mocks/fixtures and deliberate malformed inputs | Test implementation typing leaks or mocks misrepresent production contracts | Typed fixtures/mocks after runtime boundaries are corrected |

## Existing contracts and validation to reuse

- `src/api/Api.ts`: generated REST request/response types, including
  `ApiErrorResponse`, `ApiGetConfigResponse`, container metadata and schedules.
  Its request generics are static declarations; runtime JSON is not decoded.
  Do not hand-edit it. The generated `ApiGetSettingsResponse.settings` still
  contains any; correcting that at source requires a separately reviewed
  Swagger/code-generation change outside this frontend-only batch.
- `src/types/settings.generated.ts`: sparse optional YAML fields; unknown index
  values are intentional for extension keys. `gui/asserts/mower_config.schema.json`
  owns field domains/defaults; backend overlays own backend-specific defaults.
  Keep integer/float types, null deletion and dirty-key writes distinct.
- `src/types/ros.ts` / `ros.generated.ts`: optional message fields/constants.
  `GnssStatus`, `HighLevelStatus`, power, motor/ESC and map interfaces can be reused
  after runtime validation. Do not make optional fields mandatory via assertions.
- `useHardwareBackend.ts` and `constants/hardwareBackends.ts`: backend metadata
  and runtime parameter routes; route confirmation gates MAVROS/OpenMower writes.
  Existing normalization compatibility concerns are listed below.
- `useDriveTuning.ts` already declares job/report/rollback models; decode actual
  payloads into those models rather than introducing permissive replacements.
- `useGnssRuntimeConfig.ts`, GNSS response types in settings components, and
  diagnostic projections in `utils/gpsStatus.ts` already describe the receiver
  and optional GNSS capability fields. Preserve unsupported/unknown/false/true.
- `useBlackbox.ts` is an example of request<unknown> plus field-level runtime
  validation before storing a status. Its pattern is more useful than a generic
  cast-only request wrapper.
- `pages/map/utils/mapBackup.ts` validates imported backup polygons, finite points
  and corridor fields before mutating the editor; reuse that contract. Map feature
  classes/serialization in `types/map.ts` also preserve stable area identity.
- `utils/motorTelemetry.ts` uses finite checks, unknown display markers and receipt
  age. `utils/battery.ts` exposes hasBatteryReading. Consumers must preserve those
  gates instead of treating default numeric values as live measurements.

## External-boundary investigation

1. **GUI REST:** configuration, settings, generic commands, calibration reports,
   schedules, containers and error envelopes have distinct contracts. Go endpoints
   validate some write inputs but not every outgoing body is represented precisely
   by Swagger. Runtime response decoders must be endpoint specific.
2. **ROS/WS:** the multiplexer decodes msgpack and transports topic data;
   useWS/useTopic still assert raw payloads as T. This establishes neither field
   validity nor physical acquisition freshness. Backend CDR conversion supplies
   known producer shapes; cached/replayed/malformed data still needs consumer
   provenance and validity checks. A decoder must not reconnect streams or replace
   source acquisition identity with callback receipt time.
3. **Settings:** DB config strings and sparse ROS YAML settings are different
   domains. The new string decoder applies only to /config/keys; using it for YAML
   would reject legitimate booleans/numbers/nested values and null deletions.
4. **GNSS:** runtime-config responses are asserted after an any-based envelope
   unwrap. Receiver action responses mix execution state, warnings and errors.
   Future decoders must retain old receiver fields and unsupported optional
   capabilities, and never infer RTK FIX from absent/unknown status.
5. **Hardware backends:** native Mowgli firmware/PWM, MAVROS reporting/routing, and
   OpenMower LowLevel/xESC data are not interchangeable. Decode metadata before
   selecting a model/parameter route; never default an unsupported live backend to
   a known operational status merely to satisfy a type.
6. **Safety/measurements:** high-level status, power, motor telemetry, emergency
   bits and calibration results use partial message models. Rejecting a bad frame
   must not make an old sample indefinitely fresh; missing numbers remain unknown,
   not valid zero. Full byte/clock/producer handling is deferred, not generalized
   into this config decoder.
7. **Mapbox/GeoJSON:** Draw events are local library data; imported files and ROS
   map frames are separate external inputs. Typed GeoJSON alone does not verify
   finite coordinates, ring validity or area IDs. Preserve the existing whole-file
   backup validator, geometry-only selection retention and cleanup identities.

## First batch: shared API/JSON boundaries

### Validators and types

`utils/json.ts` adds `isJsonObject`, a shape-only guard for plain/null-prototype
JSON objects, and `decodeStringRecord`, which validates every own value and
returns a fresh string record. No coercion is performed. It rejects arrays,
scalars, null, undefined and nested/non-string values. Own special JSON keys such
as __proto__ are copied without prototype setter assignment. Empty documents and
empty strings are valid; strings "0" and "false" are not converted to measurements
or booleans.

`utils/configEnvironment.ts` decodes into the existing generated
`ApiGetConfigResponse`. tileUri is optional; absence or an empty string means no
local overlay. A present non-string is rejected. Unrelated future fields are
ignored rather than making the optional environment schema artificially closed.
This is shape/type validation, not new URL or robot policy.

`utils/errors.ts` removes assertions from its existing helpers. apiErrorMessage
now returns only an actual string. The new requestErrorMessage supports ordinary
Error, generated-client nested error envelopes, direct API error bodies and
textual message fields. Malformed numeric/object error fields are not coerced
into backend reasons. Existing errorMessage stringification and AbortError
recognition semantics are preserved.

### Consumer changes

- `useConfig`: state and writes use the endpoint's real string-map contract.
  Validate before POST and before storing read/write results. Decode write echoes
  **outside** the React updater, inside the existing try/catch, so malformed data
  cannot throw during rendering. A bad response reports failure and retains the
  previous state. A bad outbound JavaScript value is rejected before submission.
  Writes now normalize and propagate errors to the awaiting caller instead of
  notifying and resolving a failed write. This prevents the legacy settings
  caller from announcing success; existing map offset/bearing/settings callers
  already own their operation-specific catch/toast paths, so there is no duplicate
  notification. The mount-time key snapshot and fresh-language load Effect Event remain.
- `useEnv`: replace dynamic any state with generated environment type; decode
  before storing. Keep its empty no-overlay fallback and existing notification
  lifetime.
- `useRosbag`: reuse shared guarded request error extraction instead of an any
  assertion; preserve operation-specific fallback strings and original Error
  objects. Poll cadence, start/stop/delete ordering and busy cleanup are unchanged.
  Rosbag status payload decoding itself is a later domain-specific batch.
- `AsyncButton` / `AsyncDropDownButton`: use Promise<unknown>, unknown catch values
  and shared request-error extraction. Reuse AntD's menu event type instead of any.
  Actual callback invocation/order, loading reset, dropdown close and error toast
  behavior remain; generated-client rejections now show their actual textual
  backend reason. There is no command-body conversion or new success path.

No GNSS fix, robot status, backend identifier, command payload or ROS parameter
contract was changed. No telemetry sample was assigned a new numeric default.

## Initial findings by file, origin, impact and complexity

The six count columns below are compacted as
`any / assignment / member / argument / call / return`, in that order. Each file's
provenance can span several groups; impacts are potential static-analysis risks,
not measured robot failures. Source/test distinctions are explicit.

| File under gui/web | Counts | Data origins | Shared cause | Potential impact | Complexity |
| --- | --- | --- | --- | --- | --- |
| `src/components/AppShell.tsx` | 0 / 1 / 1 / 0 / 0 / 0 | REST JSON | A: unchecked JSON result/fields | Incorrect onboarding/schedule/version UI | Medium |
| `src/components/AsyncButton.tsx` | 1 / 1 / 1 / 0 / 0 / 0 | Rejected REST/actions; local callback/event contracts | E: untyped rejection/async result | Lost failure reason; loading/restart/action feedback | Low–medium |
| `src/components/AsyncDropDownButton.tsx` | 3 / 1 / 1 / 0 / 0 / 0 | Rejected REST/actions; local callback/event contracts | E: untyped rejection/async result | Lost failure reason; loading/restart/action feedback | Low–medium |
| `src/components/DrawControl.tsx` | 10 / 1 / 0 / 6 / 0 / 0 | Mapbox Draw/GeoJSON and local feature classes | M: untyped third-party events/geometry adapters | Incorrect selection/area geometry; editor exceptions | High |
| `src/components/MowerActions.tsx` | 2 / 0 / 1 / 1 / 0 / 0 | ROS-service REST bodies/responses/errors | A/E: open command/result records | Unsafe action input or invalid datum acceptance | High |
| `src/components/MowerStatus.test.tsx` | 0 / 0 / 0 / 0 / 0 / 2 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/components/PowerMenu.tsx` | 3 / 3 / 3 / 0 / 0 / 0 | REST container/system metadata and errors | A/E: untyped fields and failure reasons | Wrong resource match or incomplete failure feedback | Medium |
| `src/components/RobotComponentEditor.tsx` | 3 / 9 / 0 / 4 / 0 / 0 | Sparse settings; ROS geometry/pose; map editor | S/M/W: dynamic config and geometry adapters | Incorrect geometry/dock/calibration draft | High |
| `src/components/dashboard/DigEscalationBanner.test.tsx` | 0 / 0 / 0 / 0 / 0 / 1 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/components/settings/AdvancedSection.tsx` | 7 / 0 / 2 / 2 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/BatterySection.tsx` | 1 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/DockingSection.tsx` | 1 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/DriveMotorSection.test.tsx` | 1 / 1 / 0 / 0 / 0 / 0 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/components/settings/DriveMotorSection.tsx` | 7 / 25 / 22 / 1 / 0 / 1 | Drive REST reports/jobs/errors; backend-specific settings | A/S: structured reports leaking through dynamic records | Invalid calibration/apply state or parameter draft | High |
| `src/components/settings/GnssReceiverActionsCard.tsx` | 8 / 1 / 7 / 0 / 2 / 2 | GNSS sidecar REST status/actions/errors | A/E/S: asserted payloads and untyped errors | Wrong receiver status or destructive-action feedback | High |
| `src/components/settings/GnssSerialDeviceConfigField.test.tsx` | 0 / 0 / 0 / 0 / 0 / 1 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/components/settings/HardwareSection.tsx` | 2 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/LedsSection.test.tsx` | 2 / 0 / 0 / 0 / 0 / 0 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/components/settings/LedsSection.tsx` | 1 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/LocalizationSection.tsx` | 2 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/MowingSection.tsx` | 1 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/MqttSection.tsx` | 1 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/NavigationSection.tsx` | 1 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/NtripSection.tsx` | 1 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/ObstaclesSection.tsx` | 1 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/PositioningSection.tsx` | 4 / 1 / 1 / 0 / 0 / 0 | Sparse settings; GNSS datum service/telemetry | S/A/W: dynamic values and service responses | Incorrect datum/config synchronization | High |
| `src/components/settings/RainSection.tsx` | 1 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/SafetySection.tsx` | 1 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/SensorsSection.tsx` | 1 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/UniversalGnssAdvancedSettings.test.tsx` | 1 / 1 / 0 / 0 / 0 / 0 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/components/settings/UniversalGnssAdvancedSettings.tsx` | 1 / 0 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/settings/gnssConfig.ts` | 3 / 1 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/components/utils.tsx` | 4 / 2 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |
| `src/hooks/useBTLog.ts` | 1 / 1 / 0 / 0 / 0 / 0 | ROS topic frames through WebSocket/multiplexer | W: raw frames asserted as domain messages | Invalid telemetry/freshness or status UI | High |
| `src/hooks/useConfig.tsx` | 4 / 1 / 1 / 0 / 0 / 0 | GUI REST config string map/optional environment | A/S: config data and errors lack validation | Malformed config stored; invalid write or tile overlay | Low–medium |
| `src/hooks/useContainerRestart.ts` | 1 / 1 / 1 / 0 / 0 / 0 | Rejected REST/actions; local callback/event contracts | E: untyped rejection/async result | Lost failure reason; loading/restart/action feedback | Low–medium |
| `src/hooks/useDiagnostics.ts` | 1 / 1 / 0 / 0 / 0 / 0 | ROS topic frames through WebSocket/multiplexer | W: raw frames asserted as domain messages | Invalid telemetry/freshness or status UI | High |
| `src/hooks/useDriveTuning.ts` | 2 / 0 / 0 / 0 / 0 / 2 | Drive REST reports/jobs/errors; backend-specific settings | A/S: structured reports leaking through dynamic records | Invalid calibration/apply state or parameter draft | High |
| `src/hooks/useEnv.tsx` | 2 / 0 / 0 / 0 / 0 / 0 | GUI REST config string map/optional environment | A/S: config data and errors lack validation | Malformed config stored; invalid write or tile overlay | Low–medium |
| `src/hooks/useFusionGraphDiagnostics.ts` | 1 / 1 / 0 / 0 / 0 / 0 | ROS topic frames through WebSocket/multiplexer | W: raw frames asserted as domain messages | Invalid telemetry/freshness or status UI | High |
| `src/hooks/useGnssRuntimeConfig.ts` | 2 / 0 / 3 / 1 / 0 / 0 | GNSS sidecar REST status/actions/errors | A/E/S: asserted payloads and untyped errors | Wrong receiver status or destructive-action feedback | High |
| `src/hooks/useHighLevelStatus.ts` | 1 / 0 / 0 / 1 / 0 / 0 | ROS topic frames through WebSocket/multiplexer | W: raw frames asserted as domain messages | Invalid telemetry/freshness or status UI | High |
| `src/hooks/useIOSInstallPrompt.ts` | 1 / 0 / 1 / 0 / 0 / 0 | Browser navigator legacy property | B: nonstandard browser property assertion | Incorrect install-prompt eligibility | Low |
| `src/hooks/useImuYawCalibration.ts` | 1 / 2 / 2 / 0 / 0 / 0 | Calibration service/REST result | A/E: asserted result and unknown error | Invalid calibration result/feedback | High |
| `src/hooks/useInstalledVersions.ts` | 0 / 1 / 0 / 0 / 0 / 0 | REST JSON | A: unchecked JSON result/fields | Incorrect onboarding/schedule/version UI | Medium |
| `src/hooks/useRosbag.ts` | 1 / 2 / 3 / 0 / 0 / 0 | Rejected REST/actions; local callback/event contracts | E: untyped rejection/async result | Lost failure reason; loading/restart/action feedback | Low–medium |
| `src/hooks/useSettings.ts` | 9 / 7 / 1 / 1 / 0 / 0 | DB string configuration plus sparse YAML REST/schema | S/A: mixed value domains and generated any settings | Coercion/dirty-key/success state mistakes | High |
| `src/hooks/useWS.ts` | 0 / 0 / 0 / 1 / 0 / 0 | ROS topic frames through WebSocket/multiplexer | W: raw frames asserted as domain messages | Invalid telemetry/freshness or status UI | High |
| `src/pages/FleetPage.tsx` | 1 / 0 / 1 / 1 / 0 / 0 | Rejected REST/actions; local callback/event contracts | E: untyped rejection/async result | Lost failure reason; loading/restart/action feedback | Low–medium |
| `src/pages/MapPage.tsx` | 0 / 2 / 0 / 4 / 0 / 0 | ROS map/pose/obstacles plus Mapbox GeoJSON | W/M: frame and geometry boundaries | Invalid overlay/area identity or obstacle action | High |
| `src/pages/SchedulePage.tsx` | 0 / 2 / 3 / 0 / 0 / 0 | REST JSON | A: unchecked JSON result/fields | Incorrect onboarding/schedule/version UI | Medium |
| `src/pages/logBatcher.test.ts` | 0 / 0 / 0 / 0 / 0 / 1 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/pages/map/components/ImportOpenMowerModal.tsx` | 2 / 0 / 2 / 2 / 0 / 0 | File/import JSON, REST results, map features | A/M/E: imported payloads and error paths | Invalid import/edit state or lost failure feedback | High |
| `src/pages/map/components/MapToolbar.test.tsx` | 2 / 1 / 0 / 0 / 0 / 0 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/pages/map/components/MapToolbarMobile.test.tsx` | 2 / 1 / 0 / 0 / 0 / 1 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/pages/map/components/TrackedObstaclesPanel.tsx` | 3 / 1 / 3 / 1 / 0 / 0 | ROS map/pose/obstacles plus Mapbox GeoJSON | W/M: frame and geometry boundaries | Invalid overlay/area identity or obstacle action | High |
| `src/pages/map/hooks/useManualMode.test.ts` | 1 / 0 / 0 / 1 / 0 / 0 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/pages/map/hooks/useMapBearing.ts` | 1 / 1 / 1 / 0 / 0 / 0 | DB string config and write rejection | S/E: configuration sync and unknown errors | Incorrect map transform draft or save feedback | Medium |
| `src/pages/map/hooks/useMapEditHistory.test.ts` | 0 / 3 / 0 / 0 / 1 / 0 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/pages/map/hooks/useMapEditing.ts` | 3 / 7 / 0 / 2 / 0 / 0 | Mapbox Draw/GeoJSON and local feature classes | M: untyped third-party events/geometry adapters | Incorrect selection/area geometry; editor exceptions | High |
| `src/pages/map/hooks/useMapFiles.test.ts` | 1 / 2 / 5 / 0 / 0 / 0 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/pages/map/hooks/useMapFiles.ts` | 3 / 2 / 3 / 1 / 0 / 0 | File/import JSON, REST results, map features | A/M/E: imported payloads and error paths | Invalid import/edit state or lost failure feedback | High |
| `src/pages/map/hooks/useMapOffset.test.ts` | 6 / 0 / 0 / 6 / 0 / 0 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/pages/map/hooks/useMapOffset.ts` | 2 / 2 / 2 / 0 / 0 / 0 | DB string config and write rejection | S/E: configuration sync and unknown errors | Incorrect map transform draft or save feedback | Medium |
| `src/pages/map/hooks/useMapStreams.ts` | 7 / 0 / 0 / 0 / 0 / 0 | ROS map/pose/obstacles plus Mapbox GeoJSON | W/M: frame and geometry boundaries | Invalid overlay/area identity or obstacle action | High |
| `src/pages/map/hooks/useResetMowingProgress.test.ts` | 0 / 0 / 2 / 0 / 2 / 0 | Test mock/fixture | T: untyped mock, cast or deliberately malformed fixture | Indirect: incomplete regression coverage | Low |
| `src/types/map.ts` | 0 / 0 / 0 / 1 / 0 / 0 | Mapbox Draw/GeoJSON and local feature classes | M: untyped third-party events/geometry adapters | Incorrect selection/area geometry; editor exceptions | High |
| `src/utils/battery.ts` | 1 / 0 / 0 / 2 / 0 / 0 | Optional power/status telemetry plus settings | W/S: mixed numeric/string thresholds | Missing/invalid battery data displayed as measurement | High |
| `src/utils/containers.ts` | 1 / 0 / 2 / 0 / 1 / 0 | REST container/system metadata and errors | A/E: untyped fields and failure reasons | Wrong resource match or incomplete failure feedback | Medium |
| `src/utils/datumGps.ts` | 2 / 1 / 2 / 1 / 0 / 0 | ROS-service REST bodies/responses/errors | A/E: open command/result records | Unsafe action input or invalid datum acceptance | High |
| `src/utils/settingsSave.ts` | 4 / 1 / 0 / 0 / 0 / 0 | Sparse YAML settings/schema; UI callbacks | S: dynamic values/field callback contracts | Invalid setting drafts/writes; misleading field values | Medium–high |

## Initial finding locations

These locations are tied to the baseline commit, not the edited line numbers.
Each rule's complete initial locations are preserved here for subsequent batches.

| File | Initial lines by rule |
| --- | --- |
| `src/components/AppShell.tsx` | assign: 107; member: 108 |
| `src/components/AsyncButton.tsx` | any: 7; assign: 25; member: 25 |
| `src/components/AsyncDropDownButton.tsx` | any: 8,8,18; assign: 30; member: 30 |
| `src/components/DrawControl.tsx` | any: 63,75,76,77,78,79,80,81,85,90; assign: 69; arg: 76,77,78,79,80,81 |
| `src/components/MowerActions.tsx` | any: 25,108; member: 244; arg: 246 |
| `src/components/MowerStatus.test.tsx` | return: 16,24 |
| `src/components/PowerMenu.tsx` | any: 49,65,77; assign: 52,67,78; member: 52,67,78 |
| `src/components/RobotComponentEditor.tsx` | any: 124,125,250; assign: 247,248,249,309,329,372,373,374,375; arg: 292,804,839,982 |
| `src/components/dashboard/DigEscalationBanner.test.tsx` | return: 9 |
| `src/components/settings/AdvancedSection.tsx` | any: 12,24,43,93,93,106,106; member: 97,109; arg: 97,109 |
| `src/components/settings/BatterySection.tsx` | any: 14 |
| `src/components/settings/DockingSection.tsx` | any: 14 |
| `src/components/settings/DriveMotorSection.test.tsx` | any: 8; assign: 8 |
| `src/components/settings/DriveMotorSection.tsx` | any: 43,44,103,105,157,281,313; assign: 108,254,261,264,265,266,267,268,270,271,272,287,296,299,300,301,302,303,304,319,358,367,368,369,370; member: 158,158,159,160,264,265,266,267,268,270,271,272,277,282,299,300,301,302,303,304,309,314; arg: 158; return: 158 |
| `src/components/settings/GnssReceiverActionsCard.tsx` | any: 74,78,78,79,81,81,82,172; assign: 74; member: 74,78,78,79,81,81,82; call: 78,81; return: 79,82 |
| `src/components/settings/GnssSerialDeviceConfigField.test.tsx` | return: 11 |
| `src/components/settings/HardwareSection.tsx` | any: 21,22 |
| `src/components/settings/LedsSection.test.tsx` | any: 9,11 |
| `src/components/settings/LedsSection.tsx` | any: 13 |
| `src/components/settings/LocalizationSection.tsx` | any: 18,21 |
| `src/components/settings/MowingSection.tsx` | any: 60 |
| `src/components/settings/MqttSection.tsx` | any: 12 |
| `src/components/settings/NavigationSection.tsx` | any: 12 |
| `src/components/settings/NtripSection.tsx` | any: 27 |
| `src/components/settings/ObstaclesSection.tsx` | any: 30 |
| `src/components/settings/PositioningSection.tsx` | any: 32,38,72,87; assign: 72; member: 72 |
| `src/components/settings/RainSection.tsx` | any: 12 |
| `src/components/settings/SafetySection.tsx` | any: 11 |
| `src/components/settings/SensorsSection.tsx` | any: 12 |
| `src/components/settings/UniversalGnssAdvancedSettings.test.tsx` | any: 12; assign: 22 |
| `src/components/settings/UniversalGnssAdvancedSettings.tsx` | any: 20 |
| `src/components/settings/gnssConfig.ts` | any: 110,110,111; assign: 114 |
| `src/components/utils.tsx` | any: 6,9,62,66; assign: 63,67 |
| `src/hooks/useBTLog.ts` | any: 41; assign: 41 |
| `src/hooks/useConfig.tsx` | any: 12,13,23,45; assign: 26; member: 26 |
| `src/hooks/useContainerRestart.ts` | any: 100; assign: 101; member: 101 |
| `src/hooks/useDiagnostics.ts` | any: 43; assign: 43 |
| `src/hooks/useDriveTuning.ts` | any: 163,175; return: 172,184 |
| `src/hooks/useEnv.tsx` | any: 12,25 |
| `src/hooks/useFusionGraphDiagnostics.ts` | any: 49; assign: 49 |
| `src/hooks/useGnssRuntimeConfig.ts` | any: 51,54; member: 51,58,58; arg: 58 |
| `src/hooks/useHighLevelStatus.ts` | any: 11; arg: 11 |
| `src/hooks/useIOSInstallPrompt.ts` | any: 14; member: 14 |
| `src/hooks/useImuYawCalibration.ts` | any: 109; assign: 110,115; member: 110,112 |
| `src/hooks/useInstalledVersions.ts` | assign: 25 |
| `src/hooks/useRosbag.ts` | any: 37; assign: 37,38; member: 38,38,38 |
| `src/hooks/useSettings.ts` | any: 151,151,152,172,175,203,225,244,250; assign: 158,161,179,181,207,209,253; member: 253; arg: 156 |
| `src/hooks/useWS.ts` | arg: 87 |
| `src/pages/FleetPage.tsx` | any: 74; member: 75; arg: 75 |
| `src/pages/MapPage.tsx` | assign: 287,667; arg: 270,271,443,1341 |
| `src/pages/SchedulePage.tsx` | assign: 57,58; member: 539,539,539 |
| `src/pages/logBatcher.test.ts` | return: 52 |
| `src/pages/map/components/ImportOpenMowerModal.tsx` | any: 77,92; member: 78,93; arg: 78,93 |
| `src/pages/map/components/MapToolbar.test.tsx` | any: 8,13; assign: 9 |
| `src/pages/map/components/MapToolbarMobile.test.tsx` | any: 23,31; assign: 24; return: 31 |
| `src/pages/map/components/TrackedObstaclesPanel.tsx` | any: 100,100,102; assign: 105; member: 100,100,105; arg: 100 |
| `src/pages/map/hooks/useManualMode.test.ts` | any: 84; arg: 84 |
| `src/pages/map/hooks/useMapBearing.ts` | any: 44; assign: 45; member: 45 |
| `src/pages/map/hooks/useMapEditHistory.test.ts` | assign: 33,35,40; call: 33 |
| `src/pages/map/hooks/useMapEditing.ts` | any: 1019,1019,1063; assign: 822,833,834,835,836,837,838; arg: 1019,1063 |
| `src/pages/map/hooks/useMapFiles.test.ts` | any: 61; assign: 61,82; member: 77,83,84,112,267 |
| `src/pages/map/hooks/useMapFiles.ts` | any: 255,313,450; assign: 259,453; member: 259,314,453; arg: 314 |
| `src/pages/map/hooks/useMapOffset.test.ts` | any: 32,40,49,66,81,90; arg: 32,40,49,66,81,90 |
| `src/pages/map/hooks/useMapOffset.ts` | any: 35,49; assign: 36,50; member: 36,50 |
| `src/pages/map/hooks/useMapStreams.ts` | any: 130,146,158,169,259,267,306 |
| `src/pages/map/hooks/useResetMowingProgress.test.ts` | member: 57,79; call: 57,79 |
| `src/types/map.ts` | arg: 464 |
| `src/utils/battery.ts` | any: 43; arg: 49,50 |
| `src/utils/containers.ts` | any: 24; member: 25,26; call: 25 |
| `src/utils/datumGps.ts` | any: 26,27; assign: 27; member: 26,27; arg: 26 |
| `src/utils/settingsSave.ts` | any: 29,30,31,43; assign: 46 |

## Files modified in this batch

- `docs/typescript-data-boundaries.md`
- `src/components/AsyncButton.test.tsx`
- `src/components/AsyncButton.tsx`
- `src/components/AsyncDropDownButton.test.tsx`
- `src/components/AsyncDropDownButton.tsx`
- `src/hooks/useConfig.test.tsx`
- `src/hooks/useConfig.tsx`
- `src/hooks/useEnv.test.tsx`
- `src/hooks/useEnv.tsx`
- `src/hooks/useRosbag.test.ts`
- `src/hooks/useRosbag.ts`
- `src/utils/configEnvironment.ts`
- `src/utils/errors.test.ts`
- `src/utils/errors.ts`
- `src/utils/json.test.ts`
- `src/utils/json.ts`

## Regression tests and validation

58 new parameterized/regression cases cover non-object/non-string configuration
payloads, preserved empty string values, own special JSON keys, absent/empty/invalid
optional tileUri, unrelated future environment fields, valid and malformed error
envelopes, read failure, malformed write echoes retaining previous state, invalid
outbound JavaScript input never reaching POST, valid echo merging, request-error
notifications/loading, rosbag busy cleanup, and no success toast from the actual
legacy settings caller after a rejected or invalid configuration write. Tests model untyped payloads using
unknown or mocks, without adding any or unsafe assertions.

The focused boundary suites passed 65 tests; the final propagation/decoder/offset
batch passed 56 tests, including the two added settings-caller regressions. Existing command/status/map-offset/
settings-manager consumer suites passed 23 tests. Final normal-user validation:

- `yarn lint`: PASS, 365 warnings, zero errors; 343 TypeScript safety warnings.
- `yarn build`: PASS, TypeScript and Vite production build; existing advisory
  warning about large chunks remains.
- `yarn test`: PASS, 1,210 Vitest tests across 155 files, plus the separate Node
  appearance-registry suite.
- `git diff --check`: PASS.

No category regressed. The eleven unresolved hooks findings and eleven Fast
Refresh warnings remain exactly as before. Scratch logs/JSON are under
`/tmp/gui-types-*`; normal build outputs are ignored.

## Remaining typing debt and next batches

343 TypeScript safety warnings remain; this is not a whole-frontend decoder rollout.
Recommended order:

1. **Safety-critical topic boundaries and backend identity.** Decode emergency,
   status/power/GNSS/motor frames using existing message types and freshness
   ownership. Distinguish unsupported backend values from a genuinely missing
   legacy metadata endpoint before allowing backend-specific routes.
2. **Calibration/GNSS/drive API models.** Reuse job/report/receiver interfaces;
   validate actual success flags, optional capabilities, finite recommendations,
   applied report identity and failure states. Test native PWM, legacy/current
   MAVROS odometry reports and OpenMower xESC fields separately.
3. **Sparse settings read/write domains.** Propagate SettingsValues and schema
   value types through shared callbacks and serializers; validate known keys at
   write boundaries while retaining unknown extension keys, null tombstones,
   integer/float distinction, dirty-key updates and backend overrides.
4. **Mapbox/GeoJSON and import responses.** Type Draw events and response models,
   validate finite geometry/IDs before conversion, and extend existing validators
   without rebuilding or sanitizing away valid legacy map shapes.
5. **Other REST envelopes and test fixtures.** Containers/schedules/version data
   and error helpers can reuse this guarded-error pattern; clean mocks against
   actual contracts after production data origins are resolved.

## Backend-contract findings deferred for separate review

- `gui/pkg/api/config.go` skips failed DB reads, leaving the caller's empty string
  in its response. Missing key and stored empty value are indistinguishable.
  Preserve current empty-string compatibility here; representing absence/failure
  distinctly needs a backend/API change.
- The same write route ignores DB.Set failures and returns the original body.
  Validating an echo cannot prove persistence. The frontend now protects types,
  but must not claim storage verification. Per-key acknowledgments/transactional
  failures need backend review; no extra readback/retry was inserted here.
- Generated settings metadata still publishes Record<string, any>. Correcting
  Swagger/codegen source is outside this frontend-only scope; do not edit the
  generated client or introduce casts claiming that all unknown extension values
  satisfy known YAML fields.
- `normalizeHardwareBackend` currently maps unrecognized values to native Mowgli,
  and an existing test explicitly specifies that compatibility behavior. Future
  work must separate absent legacy metadata from an explicit unsupported producer.
  Adding a truthful unknown backend state and deciding route/UI compatibility
  needs separate cross-contract review, not a local cast or silent normalization.
- Command routes expose open records and inconsistent service result shapes.
  A shared typed command registry should be derived from authoritative service
  contracts, with versioned optional support, rather than imposing an invented
  catch-all interface. Robot command/parameter bodies were intentionally not
  generalized in this batch.

## Compatibility, risks and release checks

The changed config endpoints are GUI string-store/environment endpoints shared
by native Mowgli, MAVROS and OpenMower. They do not rewrite ROS YAML fields or
select firmware/motor routes. The environment response remains sparse; valid
empty URI and legacy absence remain no-overlay states.

A malformed body that was previously stored unchecked now generates the existing
load or caller-specific save-error notification. The write promise now rejects
rather than swallowing failures; every repository caller was inspected and already
catches write errors. This intentional failure-propagation correction prevents
false saved notifications; successful write sequencing is unchanged. A malformed post-write echo can be rejected after
the backend has already written; the previous GUI state remains visible, and the
backend persistence caveat above still applies. Unknown exception objects receive
generic failure text rather than invented success or a coerced error value.

Recommended mocked-browser checks: config/environment missing and wrong-typed
fields, dirty forms through rejected writes, valid map offset/bearing persistence,
no-overlay versus valid tile overlays, action rejection/loading recovery, and
rosbag start/stop failure. Later telemetry work must explicitly test missing data,
unknown backend/receiver states and every backend's optional fields.

No robot/field acceptance is claimed. No commit, push, merge or pull request was
created. All task edits remain within gui/web and are uncommitted for review.
