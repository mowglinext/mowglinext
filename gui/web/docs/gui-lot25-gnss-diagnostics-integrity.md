# GNSS status, fix quality and diagnostics — GUI Lot 25

## Baseline, scope and evidence

Branch `fix/gui-warnings-errors`, HEAD `00122e4f` (Lot 24 committed). Initial
index empty; only the preexisting MAVROS submodule modification was present.
Baseline: **2,175 Vitest tests, 105 ESLint warnings, zero errors**. No robot was
available or contacted. The historical robot screenshot/description is not
claimed reproduced at this HEAD: the software reproductions below establish
specific possible failure paths, not the deployed cause of that observation.

Production edits are GNSS frontend code plus a two-line observation-identity
copy in the existing Go Universal compatibility adapter. No command, protocol,
projection or transport policy changes. No ROS, sensor bridge, navigation, FusionGraph,
severity threshold, configuration persistence, receiver setting, firmware,
dependency or generated file is modified. Both external submodules are untouched.

External baselines matter here:

- MowgliNext gitlink MAVROS: `4f1ac1410488b118d2563992df96d08de9730598`.
- Preexisting local MAVROS checkout: `5328dd70d1fce34ee9e0c81ff3883b0260a712de`.
  The pinned gitlink object is not available in this local submodule object
  database. Its source could not be compared without fetching. Findings about
  the local adapter below apply to that checkout, not automatically the gitlink
  or an earlier robot image. No fetch or submodule update was performed.
- Universal GNSS checkout/gitlink: `9a329cc216ed137d0e375a4cfe1f5c26e1dd52d0`.

This is MowgliNext integration evidence. External implementation ownership stays
with the MAVROS and Universal GNSS repositories; no copied implementation or
submodule-specific audit artifacts are introduced here.

## Producer and deployment matrix

Current launch/install code takes precedence over older prose in `wiki/Sensors.md`
and `wiki/Deployment.md`. In particular, the latter's description that MAVROS
always forces the direct GNSS stack off is not the complete current source matrix.

| Configuration | Coordinates | Detailed status producer | Consequence |
| --- | --- | --- | --- |
| Mowgli + direct Universal | Receiver `/gps/fix` (`NavSatFix`) | `full_system.launch.py` starts `mowgli_gnss_bridge` for YAML `gnss_stack=universal`, `GNSS_SOURCE=direct`; `/universal_gnss_receiver/status` → public `/gps/status` | Public Mowgli enums/flags, not Universal ordinals |
| MAVROS hardware + direct Universal | Same direct receiver path | Same Mowgli-owned Universal bridge | Hardware backend does not identify the GNSS receiver |
| MAVROS + `GNSS_SOURCE=mavros` | Local checkout's `mavros_gnss_adapter` publishes `/gps/fix` | Local adapter consumes `/mavros/universal_gnss/<source>/status` and publishes Mowgli `/gps/status` when enabled | Adapter/plugin/image availability must be verified on the deployed baseline |
| `GNSS_STACK=disabled`, MAVROS coordinates available | May still have `/gps/fix` from the external MAVROS path | No Mowgli-owned direct bridge is launched; status requires the enabled external adapter | Coordinates do not prove `/gps/status` exists; no GUI-generated RTK replacement |
| OpenMower + direct Universal | Same direct receiver path | Same Mowgli-owned Universal bridge | OpenMower hardware bridge subscribes to GNSS status; it does not create detailed receiver status |
| No GNSS/external status producer | No guaranteed topic | None guaranteed | Unknown/unavailable is truthful |

Sources: `install/lib/gps.sh:configure_gps`,
`install/lib/config.sh:effective_gnss_{backend,stack}`,
`install/lib/compose.sh:build_compose_stack`,
`install/compose/docker-compose.{gps,gps-mavros,mavros}.yml`,
`ros2/src/mowgli_bringup/launch/full_system.launch.py`, and the local
`mavros_gnss_adapter/{src/gnss_adapter_plugin.cpp,include/mavros_gnss_adapter/status_mapping.hpp}`.
`GNSS_BACKEND`/`GNSS_STATUS_SOURCE` remain installer compatibility inputs;
actual direct launch gating uses YAML stack plus source mode. The external
Universal image runs the receiver/NTRIP runtime; orphaned `start_gps.sh` is not
assumed to describe the deployed launcher.

## Field provenance

| GUI information | ROS source / owner | MowgliNext bridge/transport | GUI consumer and limits |
| --- | --- | --- | --- |
| Latitude/longitude | `/gps/fix`, `sensor_msgs/NavSatFix`, direct receiver or external MAVROS adapter | `ros.go` logical `gps`; `transform.go:adaptGPS` projects lat/lon to AbsolutePose X/Y | `useGPS` → Diagnostics coordinates; never used to manufacture a detailed fix |
| Altitude | Same NavSatFix altitude | `adaptGPS` to AbsolutePose Z | Independent of detailed status; altitude/datum convention not changed |
| Fix valid | `/gps/status.fix_valid`, authoritative receiver/adapter | Header-shaped Mowgli message passes through `adaptGnssStatus` | Decoder → GNSS hook → `deriveGpsStatus`; explicit false wins |
| 2D/3D/DGPS | Explicit public `fix_type` | C++/Python bridge maps Universal enums symbolically; Go compatibility adapter has its own mapping | Exact categories only; generic GPS remains unknown dimension |
| RTK Float/Fixed | Public `fix_type`, optionally current `rtk_mode` | Capability and value bits mapped to Mowgli constants | No covariance, coordinates or NavSatStatus inference; contradictions become unknown |
| Horizontal accuracy | Public scalar with `CAP_HORIZONTAL_ACCURACY` | Universal bit **2** maps to Mowgli bit **8**; flags have the same meaning for capability/value masks | `displayHorizontalAccuracyM` uses availability and finite scalar; measured zero survives |
| Correction stream | Public correction status owned by bridge correction tracker; legacy named `/diagnostics` entries | Direct bridge tracks correction diagnostics independently of position; local MAVROS adapter does not supply this NTRIP/MSM truth | Explicit current channel only; fix quality does not establish correction flow |
| GNSS backend | Public `backend`, e.g. `universal`, `mavros_gps1` | Passed through and validated as string | GNSS source label, not hardware backend or receiver model |
| Receiver vendor/model | Public optional metadata | Direct bridge vendor comes from configured family; model/firmware are not supplied by its current status projection. Local MAVROS adapter leaves receiver metadata empty | Use published nonempty metadata only. Configured GUI family is no longer substituted into the receiver label |
| Position identity | Public `position_observation_sequence` copied from receiver | ROS header is retained too; non-position runtime updates may advance its stamp | Hook prioritizes nonzero sequence over runtime/header identity; no invented epoch |

The transport path is ROS CDR → `foxglove.DeserializeCDR` →
`sanitizeJSONValue` → JSON → provider adapter → multiplex MessagePack → `useWS`
unknown value → `decodeGnssStatus` → GNSS display state → pure classification/cards.
The dedicated subscription also originates from the same JSON payload. No
frontend/generated enum translation is applied to coordinates.

Mowgli public fix ordinals are No Fix=0, generic GPS=1, Float=2, Fixed=3,
Dead Reckoning=4, 2D=5, 3D=6, DGPS=7. Universal ordinals are Unknown=0,
No Fix=1, generic Fix=2, Float=3, Fixed=4, Dead Reckoning=5, 2D=6, 3D=7,
DGPS=8. They are not interchangeable. The inspected bridge mappings and existing
Go mapping tests use these distinctions; no bit/enum mapping defect was proven.

## Confirmed defects and red reproductions

Before production edits, new regressions failed for:

1. **Position sequence discarded** by the frontend schema; only the header
   identity was used, even though that stamp can advance without a new position.
2. **Cached Fixed resurrected on reconnect** when the same identifiable sample
   was replayed. Open/close handling cleared the display only until replay.
3. **Frozen identity stayed Fixed** while other fields/stamps continued arriving;
   individual-stream silence was also not expired while other multiplex topics
   remained active.
4. **Diagnostic fallback never expired** in the GNSS hook and could remain after
   reconnect without a new diagnostic publication.
5. **Legacy explicit horizontal zero hidden** solely because masks were absent.
6. **2D + available RTK Fixed** promoted to Fixed despite contradictory evidence.
7. **Typed MAVROS status enriched with Universal corrections** lacking receiver
   or acquisition correlation.
8. **Configured family displayed as detected receiver**, and an explicitly
   invalid fix could still show a green Fixed *RTK-mode* tag beside its No Fix tag.
9. **Distinct diagnostic receivers combined** into one projected status.
10. **Real wire sentinel rejected**: Go sanitizes ROS NaN to JSON null, whereas
    the frontend only recognized NaN as an unavailable sentinel. One unavailable
    optional channel rejected the entire otherwise valid frame, hiding fix,
    backend and accuracy together.

Additional pre-resumption red checks covered dropping an established sequence to
zero and navigation-only Fixed replay after dead reckoning
(`/tmp/gui-lot25-downgrade-red.log`, `/tmp/gui-lot25-dr-red.log`). Both guards were
already in the worktree when the user requested resumption.

Red logs: `/tmp/gui-lot25-red.log` (seven failures),
`/tmp/gui-lot25-cards-red.log` (two), `/tmp/gui-lot25-source-red.log` and
`/tmp/gui-lot25-null-wire-red.log` (one each). The last case is grounded in the
actual Go sanitizer, not an invented JSON convention.

`src/test/fixtures/gnss-mavros-sanitized.json` is shared by the frontend and Go
regressions. It contains public Fixed, RTK Fixed, capability mask 25 (1+8+16),
value mask 9 (1+8), horizontal accuracy **0**, unavailable vertical accuracy
**null**, unavailable MSM age **null**, empty receiver metadata and sequence 42.
The Go test serializes/deserializes a real CDR **field subset**, injects float32
NaNs and runs the production sanitizer, proving exact JSON equality with that
fixture. This is not claimed as an entire live MAVROS message capture. Chromium
then sends the same fixture through MessagePack to the actual Diagnostics page.

## Corrections and preserved contracts

- Accept NaN **or transport null** only for the existing numeric sentinel fields
  whose explicit value bit is off. Omit them; never coerce to zero. Null with an
  available bit, strings, negative accuracy, unknown enums or inconsistent masks
  remain invalid. Capability alone does not establish a measurement.
- Validate and retain safe integer position sequences. A nonzero sequence plus
  reported backend identifies observation progress; zero/absent sequence selects
  header compatibility identity. Values outside JavaScript's safe integer range
  are rejected rather than rounded. No generated model is edited.
- Give GNSS its own scoped lifecycle while retaining `useWS` start/stop and stable
  subscription callbacks. Replay of the last identity after loss, malformed data
  or No Fix cannot reinstate a positive status; sequence regression cannot reset
  the retained identity lifetime. Established sequences cannot be dropped to zero
  to obtain header-only freshness; dead reckoning also establishes a negative
  barrier. Explicit negatives may always clear the display.
- Reuse the existing **30-second GUI diagnostic delivery window**, with a
  one-second display tick, for absent/frozen GNSS display state. This is a GUI
  liveness policy, **not** the physical GNSS acquisition age, the ROS navigation
  stale threshold or a guarantee that the first received sample was acquired now.
  Browser monotonic elapsed time is never subtracted from a ROS timestamp.
- Diagnostic fallback remains available only before typed frames arrive. Filter
  stale diagnostic receipts and pre-disconnect publications. Cached diagnostic
  publication identity does not renew its receipt time. Typed frames, including
  explicit unsupported/negative fields, are never filled from unrelated diagnostic
  state. Diagnostic-only groups require a matching nonempty hardware identity
  when combined with another source; standalone recognized legacy channels remain
  supported and generic GPS does not become 3D/RTK.
- Reject contradictory basic/RTK or Float/Fixed evidence in classification and
  RTK-mode presentation. Preserve 20/40/60/80/100 display percentages. Explicit
  No Fix/dead reckoning cannot retain a positive RTK-mode tag. Unknown RTK is not
  mapped to None; no NTRIP state is derived from RTK quality.
- Legacy finite scalar values can be read when masks are absent; explicit masks
  still gate them. Advanced capability-driven panels retain their existing
  visibility rules. Configured receiver family no longer substitutes for runtime
  identity; missing metadata renders the existing translated unavailable label.
  No metadata is written back to persistent settings.

Older assertions were updated to reflect these explicit contract changes:
configuration `auto` is no longer a detected receiver; legacy typed status is no
longer enriched with uncorrelated diagnostics; and unidentified diagnostic
position sources no longer supply fields to each other. Standalone legacy GPS,
standalone correction diagnostics, same-receiver diagnostic enrichment, explicit
negative authority and typed unavailable masks retain separate regression checks.

No change to GPS coordinate conversion, physical acquisition, subscription
reconnect policy, GNSS configuration actions, navigation or safety authorization
was made. No implicit API write or hardware command was added.

## Resumption review — legacy replay barriers

The worktree and all untracked fixtures/tests were preserved. The most recent
pre-resumption full run actually passed **2,203** tests (the earlier report of
2,201 preceded the two sequence/dead-reckoning regressions).

Review then proved a remaining edge case with three new red tests: an unidentified
legacy positive frame could reinstate Fixed after a known disconnect, malformed
update or elapsed silence window. Another red test showed an older *different*
legacy header could bypass the exact-identity barrier after reconnect. Logs:
`/tmp/gui-lot25-unidentified-red.log` and `/tmp/gui-lot25-older-stamp-red.log`.

The GNSS hook now requires an identity to re-establish a positive state after
known loss/negative/expiry. For legacy stamped frames it retains a header
high-water mark and a loss barrier, rejecting equal or older publications until
a newer one arrives. Rejected messages cannot lower either the sequence/header
barrier or renew its lifetime. Explicit negatives still clear the display. Initial
legacy input remains supported; a valid newer identifiable publication recovers
normally. This is conservative ordering within one reported source, not a new
producer incarnation/robot identity or physical acquisition timestamp.

These four resume regressions pass; frontend validations were rerun after the
hook correction. The final transport review also proved that the historical
`adaptGnssStatus` branch recognizing a Universal `stamp` payload omitted its
observation sequence. Its new Go regression failed with expected 42/actual 0
(`/tmp/gui-lot25-go-sequence-red.log`). A raw uint64 field and assignment to the
existing public field now preserve it; no other field mapping changes. Go was
rerun after that correction. The normal Mowgli `header` branch still passes
through unchanged. Python's earlier blocked attempt remains applicable: neither
its code nor its installed overlay was modified. All original Lot 25 work and
both submodules were preserved.

## Final review — source round trips and replay

Before changing the hook, seven new focused cases were run against the existing
implementation. **Four failed, three passed** in
`/tmp/gui-lot25-source-roundtrip-red.log`. Both A=`universal` Fixed sequence 42 →
B=`mavros_gps1` (No Fix or Fixed) → A Fixed 42 restored Fixed incorrectly.
Reconnect/regression combined with a source switch also bypassed the barrier.
The new-source policy test likewise showed that a label change alone immediately
admitted a positive observation. These are software reproductions with mocks.

The hook now retains sequence and legacy-header high-water marks in a session-local
Map for every encountered source. On returning to a known source, it restores that
source’s barrier and requires identifiable progress: A42 stays blocked, A43 can
recover directly. Neither a lower sequence nor a source detour resets the maxima.
Repeated frames on the continuously active source retain the existing expiry
behavior; a source detour cannot renew a cached observation’s lifetime.

Policy for a never-seen source after an earlier typed frame: its first valid
frame establishes a baseline; it cannot establish a positive displayed fix until
an identifiable sequence/header progresses. An explicit negative may immediately
clear positioning. A label change is not an acquisition or producer restart.
The first frame of a new frontend session retains legacy reporting compatibility;
it still does not prove physical acquisition freshness. Sequence counters are not
compared across sources, and no UUID or producer restart epoch is invented.

After any named source has been seen, a valid frame lacking `backend` is withheld
rather than assigned to an anonymous producer or to the last named producer. It
preserves all known-source maxima, clears the display and requires attributable
progress to recover. A legacy session that has never received a named source
retains its existing anonymous sequence/header handling. Source labels remain
producer assertions, not authenticated receiver or robot identities.

Twelve added hook regressions cover the requested round trips, direct A43 recovery,
B’s own established progress, missing backend/identity, repeated switches through
three sources, reconnect plus regressed sequence, same-source recovery, legacy
header ordering and initial named legacy compatibility. Existing zero, optional
null/NaN flags, no cross-source enrichment and lifecycle tests remain in the
focused run. The FR/EN browser scenarios additionally verify source round-trip
rejection, missing-backend rejection, identifiable recovery and zero writes.

Only the hook, its tests, the existing mocked browser scenarios and this report
changed during this final review. No additional Go, producer, MAVROS or Universal
GNSS change was made. Final validation results follow. The final run logs are
`/tmp/gui-lot25-roundtrip-focused.log`, `-full.log`, `-browser.log`, `-tsc.log`,
`-lint.log` and `-build.log` (each using the same `gui-lot25-roundtrip` prefix).
The ESLint JSON inventory is `/tmp/gui-lot25-roundtrip-eslint.json`.

## Validation

- Focused Vitest: **158 tests passed**, five GNSS decoder/classification/hook/card
  files. Includes 2D/3D/DGPS/Float/Fixed, masked/unmasked zero, missing/malformed
  data after valid data, No Fix, contradictory RTK, expired and replayed identities,
  backend change, clean session/robot remount and StrictMode subscription cleanup.
- Full Vitest: **2,219 tests passed in 189 files**, plus the appearance registry
  test, using `yarn test --pool=threads --maxWorkers=2` (217.05 seconds for Vitest).
  This adds **44 frontend regressions** over the committed baseline.
- Focused Go CDR/sanitizer and provider adaptation tests passed; **full
  `go test ./...` passed**. The final compatibility adapter also preserves the existing observation field;
  its regression and the full Go suite were rerun after that two-line change.
  These are the preceding Go results (`/tmp/gui-lot25-go-final-reviewed.log`);
  Go was unchanged and was not rerun for the final frontend-only review.
- Chromium FR/EN: **2 passed**, `--retries=0 --workers=1`, 5.4 seconds. Actual
  Diagnostics coordinates with absent status, sanitized Fixed/zero accuracy,
  unknown correction state, explicit No Fix, cached Fixed refusal, new observation
  recovery, reported receiver and explicit correction activity, source round-trip
  replay and missing-backend rejection, malformed frame and zero writes. All REST and WebSocket traffic is mocked.
- `yarn tsc --noEmit`, `yarn lint` and
  `yarn build --outDir /tmp/gui-lot25-roundtrip-dist`: passed.
- Full ESLint JSON before/after: **105 → 105 warnings, zero errors**. No
  suppression, artificial cast, dependency/configuration or generated-file edit.
- Python bridge parity test was attempted but **could not start**: installed
  `/ros2_ws/install/universal_gnss_msgs/lib/python3.14/site-packages/universal_gnss_msgs/msg/_gnss_status.py`
  lacks `FIX_TYPE_2D_FIX`, present in the checked-out IDL. This is an installed
  overlay/interface mismatch, not proof of a new bridge source regression. No
  stub constants or rewritten fixtures were used to make it pass. There is no
  existing built `mowgli_gnss_bridge` test target here; no ROS build pass is claimed.
- Formatting and diff checks: `gofmt -l` returned no files; `git diff --check` passed,
  including separate whitespace checks for the new untracked files.

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

Existing jsdom pseudo-element and large-chunk build notices are not ESLint errors.
Browser coverage is targeted Chromium, not the full Playwright suite or physical
receiver behavior. No arbitrary sleeps, timeout increases or retry were used.

## Absence, rejection, masking and remaining ownership

The old all-missing observation has no exact deployment baseline, so its cause
cannot be assigned retroactively. The software evidence distinguishes:

- **Producer/topic absent:** possible in disabled/external configurations; inspect
  graph/type/launch/image before attributing it to React. No topic absence on the
  historical robot is proved by this repository audit.
- **Wrong message type:** the logical topic declares Mowgli `GnssStatus`, but
  `foxglove.Client.Subscribe` uses the advertised schema and intentionally ignores
  its compatibility `msgType` argument. The Go adapter recognizes legacy Universal
  `stamp` payloads and explicitly maps their different ordinals; it now also
  preserves their observation sequence. The configured type name is not an
  enforcement guarantee. No deployed type mismatch was demonstrated.
- **Field absent at publication:** direct status bridge does not supply a detected
  model/firmware; auto family can also yield empty vendor. Local MAVROS projection
  does not supply physical receiver identity or NTRIP/MSM status. These remain
  unavailable; GUI does not fill them from configuration or coordinates.
- **Field lost/rejected:** position sequence was dropped by the frontend and by
  the Go legacy Universal compatibility branch; unavailable NaN became transport
  null and caused whole-frame rejection. These paths are reproduced/fixed.
- **Masked field:** explicit absent value/capability still hides the channel;
  legacy explicit finite horizontal values are no longer hidden solely for
  lacking masks. No invalid/unsupported scalar is promoted.
- **Stale/replayed field:** local identity and delivery guards now hide it.
  First receipt and unstamped legacy messages cannot prove physical acquisition.
- **Presentation:** conflicting RTK tag and configured-family receiver fallback
  are corrected; coordinate-only input still reports detailed fix unknown.

Follow-up proposals belong to their source owners, not this GUI patch:

1. MAVROS: verify the **deployed** adapter/plugin/image and selected GPS source;
   if absent in that baseline, restore its authorized canonical status publisher
   in that repository. Do not add a generic NavSatFix-to-RTK shim here.
2. Universal GNSS / MAVROS: expose receiver identity through an authoritative
   runtime contract if required. The current direct status bridge has no observed
   model to propagate; a saved configured model is not detection evidence.
3. MAVROS correction integration: provide source-owned correction provenance if
   the runtime has it; empty adapter fields must not be filled by GUI assumptions.
4. Source incarnation: `position_observation_sequence` is process-local and has
   no restart/robot UUID. Sequence rollback is conservatively withheld until it
   exceeds the retained per-source high-water mark, or a new page session
   establishes its own baseline. Changing the reported source does not erase
   an already encountered source’s barrier. This can delay display after a real
   producer restart. An epoch/correlated snapshot contract belongs upstream;
   this patch invents none. Legacy zero/absent position sequence has only
   delivery/header compatibility guarantees. Unidentified initial frames cannot
   prove acquisition freshness; continuous unidentified cached traffic cannot be
   distinguished from genuine identical input. After a known loss, unidentified
   positive input is withheld until an identity is supplied. Clock rollback is
   withheld by the legacy header barrier until progress or a new page session.

The Fleet UI opens another robot in a separate page/origin (`FleetPage.tsx`);
there is no in-place API-host switch contract. New hook/session state starts
empty; backend updates replace the entire typed sample. Tests do not claim
cross-process ordering or a global robot identity supplied by ROS.

## Hardware acceptance — HARDWARE_REQUIRED

No robot is available. Remaining acceptance is whether a particular deployed
image/configuration supplies the topics and metadata seen by the operator.
Record exact MowgliNext commit including this patch, MAVROS gitlink **and deployed
image digest**, Universal GNSS receiver/driver/image revision, robot unit and
firmware image. Their values were not observed here and must not be inferred
from the development checkout.

Procedure: with blades disabled and motion prevented, inspect publisher counts
and types for `/gps/fix`, `/gps/status` and `/diagnostics`; passively record the
selected GNSS source, enum/flag/scalar/sequence/header and receiver metadata;
compare with FR/EN Diagnostics, including unavailable channels. Do not trigger
receiver reset/configuration, navigation, docking or hardware commands for this
check. Pass when every GUI category/value agrees with the **available** recorded
producer fields, no optional null becomes measured zero, missing status remains
unknown despite coordinates, and replay/no-fix cannot restore Fixed. This does
not establish physical positioning accuracy or navigation/blade-gate acceptance.

## Review files and delivery

- `gui/web/src/utils/gnssTelemetry.ts` and its test
- `gui/web/src/utils/gpsStatus.ts` and its test
- `gui/web/src/hooks/useGnssStatus.ts` and its test
- `gui/web/src/components/gnss/GnssLiveDiagnosticsCard.tsx` and its test
- `gui/web/src/components/gnss/GnssLiveStatusSummaryCard.tsx` and its test
- `gui/web/src/test/fixtures/gnss-mavros-sanitized.json` (new)
- `gui/pkg/foxglove/gnss_wire_integrity_test.go` (new)
- `gui/pkg/providers/transform.go` and its test
- `gui/web/tests/e2e/gnss-integrity.spec.ts` (new)
- `gui/web/docs/gui-lot25-gnss-diagnostics-integrity.md` (new)

Review diff: `/tmp/gui-lot25-final.diff`, including new files and excluding
MAVROS/Universal GNSS. Temporary evidence/build outputs stay in `/tmp`. Index
empty; no staging, commit, push, reset, stash or submodule modification.

Final-review complementary diff: `/tmp/gui-lot25-source-roundtrip.diff` (the hook,
its tests, existing browser scenarios and this report only). The complete
`/tmp/gui-lot25-final.diff` is refreshed with all untracked Lot 25 files. Both
are review artifacts, not staged changes. No GO COMMIT has been assumed.
