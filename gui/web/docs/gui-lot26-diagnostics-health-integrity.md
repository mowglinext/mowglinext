# Diagnostics health integrity — GUI Lot 26

## Baseline and boundary

Branch `fix/gui-warnings-errors`, HEAD `ffcb1ba2fa98` (Lot 25 committed).
The initial index was empty; the only preexisting worktree change was MAVROS.
Baseline: **2,219 Vitest tests, 105 ESLint warnings, zero errors**.
All changes remain unstaged and uncommitted. No robot was available or contacted.

This patch changes diagnostic presentation and its evidence metadata. It changes
no motion authorization, emergency command, blade/motor protection, navigation/BT,
FusionGraph calculation, ROS localizer, arming, firmware or configuration write.
An OK display is not permission to move. Existing emergency assertions remain
independent of latest payload bits and are never acknowledged cleared by a false
bit, a reconnect or a newer publication timestamp.

The Go extension is limited to the existing read-only snapshot: explicit Docker
and thermal-reading availability, existing deployment image-role metadata, and
`Cache-Control: no-store`. Existing numeric fields and response collections remain.
No service call or command was added.

## Reproductions before correction

At HEAD, `DiagnosticsPage` conflated missing information with danger or success:

- Missing GNSS satisfied `!gpsOk && !gpsWarn`, yielding Critical.
- A missing snapshot or empty container list satisfied `allContainersOk`.
- Missing CPU temperature became zero, then selected the green CPU badge.
- Any retained ROS alert yielded Warning, irrespective of its age or meaning.
- A generic container subtitle could explain a more important emergency-history
  condition. Separate component booleans did not constitute one verdict model.
- The battery calculator accepted a valid positive high-level percentage without
  voltage, but its display required voltage again and hid the known estimate.

Initial Chromium reproductions failed in both locales. The English CPU case
showed the reassuring green badge for an unavailable temperature; the unknown
verdict cases showed Critical. The French CPU locator also matched CPU usage and
was subsequently made unambiguous. Evidence: `/tmp/gui-lot26-red-browser.log`.
Two new Go route cases failed because `containers_available` was absent: success
with an empty list and a Docker query failure could not be distinguished
(`/tmp/gui-lot26-go-red.log`). No real Docker or ROS command was invoked.

Producer inspection then justified additional focused red tests:

- Four known monitoring availability ERRORs were presented as generic degradation,
  while a structured autonomous path-tracking ERROR lacked critical priority.
  **Five red cases**, `/tmp/gui-lot26-ros-red.log`.
- An earlier ordered-progression test incorrectly added a timestamp to
  HighLevelStatus, which has no such field. That test was not valid evidence
  about this producer. The follow-up below replaces it with the genuinely
  stamped Emergency producer and documents the actual HighLevelStatus defect.
- The analogous accumulated ROS and FusionGraph diagnostic publication cases
  could renew confidence after reconnect. **Two red cases**,
  `/tmp/gui-lot26-regressed-diagnostics-red.log`.
- An empty exception message could leave the snapshot error string falsy and
  make a failed read appear eligible again. **One red case**,
  `/tmp/gui-lot26-empty-error-red.log`; failures now have a nonempty fallback.
- Empty or unrecognized Docker state strings were treated as confirmed stopped
  core services. **Two red cases**, `/tmp/gui-lot26-unknown-container-red.log`;
  only known Docker stopped states now establish a stopped-service observation.
- A structured battery ERROR with valid positive voltage was not critical when
  another estimate was positive. **One red case**,
  `/tmp/gui-lot26-battery-alert-red.log`.

These are mocked software reproductions at the stated baseline, not evidence
that a particular deployed robot experienced each path.

## One verdict and a truth table

`deriveDiagnosticHealth()` is the shared display classifier. Its clocks and
inputs are explicit. It returns per-check levels/reasons/evidence, the primary
cause, available evidence, unknown checks, confirmed non-applicability and
critical checks. `DiagnosticHealthSummary` uses those same checks for the hero,
subtitle and badges. Unknown and non-applicable are neutral, with explicit text.
Original units, translated labels, fixed-width numeric spans and wrapping layout
are retained. The CPU statistic and configuration panel also respect availability.

Global priority is **Critical > Warning > Unknown > OK**. Emergency/history checks
come first among equal critical causes. A critical cause cannot be replaced by a
less serious subtitle. Warning can coexist with listed unknown checks, but those
unknown checks remain visible; they never become OK. All required checks must
be positive before the overall verdict can be OK.

| Evidence | Before | After |
| --- | --- | --- |
| Everything absent | Critical from missing GNSS; containers/CPU green | Unknown; missing checks neutral |
| GNSS absent, position present but quality unknown, or expired status | Critical | Unknown; coordinates do not establish quality |
| Explicit No Fix or dead reckoning | Critical | Warning: confirmed GNSS degradation, no invented global navigation failure |
| Valid 2D/3D/DGPS, including 3D without RTK information | Accuracy booleans could yield OK | Warning; category itself is unchanged |
| RTK Float, including degraded accuracy | Accuracy booleans could yield OK | Warning |
| RTK Fixed, accuracy unavailable | Warning | Unknown accuracy-dependent health |
| RTK Fixed with current accuracy ≤ 0.1 m, including zero | OK | OK for the GNSS check |
| Fixed accuracy > 0.1 m | Warning | Warning; threshold unchanged |
| Snapshot absent, pending initial read, HTTP/API failure, malformed or expired | Containers OK; CPU green zero | Unknown snapshot-dependent checks |
| Empty Docker result, including explicit successful empty list | Containers OK | Valid empty observation, insufficient evidence of required service health: Unknown |
| Current standard required services corroborated running | OK | OK |
| Unknown/empty Docker state string | Confirmed core failure | Unknown, including a neutral table row |
| Corroborated required core container stopped | Critical | Critical |
| Managed optional service stopped | Critical for any stopped container | Warning, unless explicitly disabled/non-applicable |
| Unrelated host container stopped or renamed old core backup | Could make whole robot Critical | Does not establish a required robot failure |
| CPU missing, nonfinite, malformed, or explicitly unavailable | Missing became green zero | Unknown |
| Modern explicitly available CPU zero | Hidden as an unavailable value | Real 0.0 °C, OK |
| Legacy CPU zero without availability metadata | Ambiguous green zero | Unknown; read failure and zero cannot be distinguished |
| CPU 55 / above 55 / 70 / above 70 °C | Mixed missing-value behavior | OK / Warning / Warning / Critical |
| Battery unavailable/expired | Could coexist with misleading all-good booleans | Unknown |
| Battery 0–20 / 21–50 / above 50 percent | Several views used different inclusive boundaries | Critical / Warning / OK, using existing canonical helper |
| Valid positive high-level percentage without voltage | Calculator knew it; display hid it | Displayed estimate; unavailable voltage stays unavailable |
| Fresh ROS WARN or unrelated/optional ERROR | Warning | Warning, with source details retained |
| Known ROS availability/age report | Warning or indiscriminate alarm | Unknown, while retaining its reported level and message |
| ROS STALE or expired entry | Could remain an active Warning indefinitely | Unknown, with last reported severity preserved |
| Confirmed required motor-temperature/battery ERROR | Only generic Warning unless another check caught it | Critical with structured evidence |
| Confirmed live tracking ERROR during autonomous operation | Generic Warning | Critical; no navigation threshold or decision changed |
| Active or latched canonical emergency | Emergency | Emergency, always critical |
| Canonical emergency history with unverified clearance | Could have mismatched cause text | Critical with the clearance cause, never normal |
| Configuration warnings based on legacy zero heuristics | Separate warning panel, possible global all-good | Global Unknown/configuration unverified; raw warnings remain visible |
| Explicitly disabled LiDAR | Could appear unavailable/faulty in ancillary views | Non-applicable check and anatomy label; raw diagnostics remain accessible |

Battery estimates keep the existing precedence and voltage calculation:
`computeBatteryPercent()` prefers a finite positive reported percentage, otherwise
uses finite positive voltage and configured endpoints. A bare reported zero has
no validity bit and retains its existing interpretation; a voltage-derived zero
is valid and critical. `hasBatteryReading()` still means a valid voltage reading,
not the validity of every percentage source. No battery threshold/helper formula
was changed. Statistic colors now match `getBatteryLevel()` at exactly 20/50.
A valid structured ROS battery ERROR can override the badge severity while the
reported percentage remains visible: both estimates are listed in the cause;
the GUI does not silently replace either value or recompute a new authority.

## Provenance and applicability

| Input | Actual producer/contract | Display handling |
| --- | --- | --- |
| Emergency/history | `/hardware_bridge/emergency`, `/mowgli/status`, existing validated hooks and assertion helpers | Canonical assertions outrank missing measurements; original command payloads retained |
| Operational context | `HighLevelStatus.state` numeric enum, validated hook | No inference from state/substate names |
| GNSS quality | Lot 25 `useGnssStatus`, `deriveGpsStatus`, availability masks and accuracy helper | Hook, decoder, fix categories, observation barriers and correction separation unchanged |
| Battery | Validated power/high-level streams plus existing settings and calculator | Only eligible current inputs enter the existing formula |
| Containers | Go snapshot → Docker `ContainerList(All:true)` | Success/failure availability is explicit; arbitrary host containers are not mandatory |
| CPU | Successful finite parse of `/sys/class/thermal/thermal_zone0/temp`, millidegrees divided by 1000 | Availability separate from number; zero remains representable |
| ROS diagnostics | Accumulated validated `/diagnostics` entries and producer publication stamps | Preserve message/reported level; ordered health evidence is separate from legacy receipt fields |
| Localizer | Read-only `/fusion_graph/diagnostics` | Autonomous context additionally requires current initialized localizer evidence; no graph calculation change |
| Configuration | Existing settings and snapshot cross-checks | Missing/ambiguous settings are not defaults or proof of disablement |

Standard deployment core recognition is deliberately narrow. The installer
Compose files declare `mowgli-ros2` and `mowgli-gui`; the updater's
`managedServices()` requires the ROS and GUI image roles. A required-core
conclusion needs both the standard name and matching
`garden.mowgli.update.image` metadata. The snapshot forwards that existing label,
not a new required-services inventory. A renamed old core backup is not a required
running service. Unknown/custom names or legacy snapshots without these
confirmations cannot prove the complete expected core set: health stays Unknown.
Optional GPS/MAVROS/LiDAR containers are not universally mandatory. A declared
LiDAR image family need not run when `lidar_enabled=false` is explicitly present;
an unlabelled unrelated host tool is not treated as a robot failure.

`gnss_stack=disabled` disables the direct bridge; it does **not** prove that all
GNSS is disabled, because `GNSS_SOURCE=mavros` can supply it independently.
There is therefore no GNSS Not-applicable inference from that setting alone.
A GNSS not-applicable contract would need authoritative runtime source/applicability
information. A reported fault is not hidden behind a configuration flag.
LiDAR configuration absence or a string/object instead of an explicit boolean is
not proof of disablement. Saved configuration does not prove that the next-restart
change is already active; contradictory live diagnostics remain visible.

## ROS meanings and operational context

Producer review covered `mowgli_monitoring/diagnostics_node.cpp`, its configuration,
`fusion_graph_node_setup_comms.cpp`, generated high-level states and the documented
high-level API. No producer code was changed.

The exact monitoring name/hardware-ID pairs for Hardware Bridge, IMU, LiDAR and
Odometry report input absence/age. Their non-OK level is not a measured hardware
failure. GPS is likewise unavailable when no status measurement exists or a
positive fix is old; a structured negative `fix_status` is a real No Fix warning.
For the historical `EKF Map` label, absence/age is separated from explicit
`flat_ok=false`/`z_drift_ok=false` degradation. That legacy name is not evidence
that the removed robot_localization EKF is deployed. Unknown external producers'
ERRORs remain visible warnings rather than guessed whole-robot failures.

Required temperature/battery ERROR recognition needs the known monitoring
name/hardware-ID and finite structured measurements. A battery report additionally
requires positive voltage and a finite percentage within 0–100; a default zero
voltage cannot invent a critical measured battery. Path-tracking ERROR promotion
needs the known controller diagnostic, a finite recent p95 value and confirmed
autonomous context. The existing ROS producer owns its thresholds; none are
copied into or altered by the GUI. Optional/unmapped ERRORs are not silently dropped.

The old global `no data since last update` text filter was traced to the installed
ROS `diagnostic_updater` C++ `TimeStampStatus::run()` and Python timestamp updater:
when no timestamp event was collected in the update interval, they emit WARN with
that text. It is not an exclusive ublox fault code or a universal benign condition.
The global substring filter is removed. Structured source/availability rules,
not text, determine health; the entry remains visible. A required ERROR containing
that phrase cannot disappear just because of its wording.

Operational policy uses numeric states:

- NULL/initializing or missing operational state: Unknown, not an inferred emergency.
- IDLE, including charging reported separately: no critical GNSS inference from absence.
- RECORDING and MANUAL_MOWING: preserve their actual states; GNSS absence is unknown,
  confirmed GNSS degradation remains visible, and no autonomous localization rule
  is inferred solely from their labels.
- AUTONOMOUS includes mowing, transit, recovery and returning home. It additionally
  checks localizer health, including a positive node count rather than trusting
  a level-OK `waiting init` publication. A current required localizer ERROR or
  structured active tracking ERROR is critical even when other measurements are
  unknown. This does not require GNSS unconditionally: FusionGraph can use other
  inputs. GNSS No Fix alone does not prove failure of that localization chain.
- Active emergency, retained latch or clearance history always wins. Disconnection
  removes current positive confidence, not emergency history.

These are display priorities, never replacements for the BT or firmware gates.
Onboarding readiness checks and their required/pass/pending semantics are unchanged.

## Freshness, identity and recovery

All expiry windows reuse the existing GUI **30-second** diagnostic delivery
window. This is not a new ROS safety threshold, receiver acquisition age or
physical-freshness claim. Producer age diagnostics retain their own configured
semantics. A one-second page tick updates expired presentation without new frames.

Where the producer supplies ordering, progression is checked before applying a
deadline. Unstamped reported context is explicitly distinguished below:

- ROS and FusionGraph health metadata retain per-reported-source stamp high-water
  marks in the hook session. Equal/older cached publications, source round trips
  and known disconnects cannot renew health confidence. A strictly progressing
  identifiable publication can recover. Missing identities after a known loss
  stay unconfirmed. The public receipt fields and retained payload contract remain
  available to existing consumers, including the Lot 25 GNSS fallback.
- Shared validated status hooks distinguish producer contracts explicitly. The
  real `Emergency.msg`, `Power.msg` and `Status.msg` contain `stamp`: their
  display-only high-water/progression barriers retain ordered publication
  checks. Equal/older publications, identity removal, invalid input and reconnect
  cannot lift a known barrier without identifiable progress. Initial unstamped
  legacy messages remain compatible; after known loss their ordering cannot be
  proved. Existing payload bits, accumulated assertions and command consumers
  are unchanged.
- `HighLevelStatus.msg` has **no stamp, header or sequence**. Its hook explicitly
  selects receipt liveness for the reported operational context. A structurally
  valid delivery restores that context after malformed input, disconnection or
  expiry, and expires 30 seconds after the last valid receipt. An open socket or
  an invalid frame alone never restores it. Continuous identical valid messages
  can keep the declared mode live, as required by the producer's one-second
  `republishHighLevelStatus()` behavior. They do not establish a newly acquired
  physical measurement or replay freedom. No timestamp, sequence, restart epoch
  or identity is fabricated. Ordering metadata stays absent even if an unknown
  extension happens to contain a timestamp. Emergency assertion history remains
  independent and cannot be cleared by this recovery.
- ROS and FusionGraph publication clock comparisons stay within the same reported
  producer. ROS time is never subtracted from browser time. Delivery/progression
  age uses wall time for the existing accumulated ROS entries, monotonic time for
  status/FusionGraph and the HTTP snapshot.
- Snapshot HTTP status/API error is checked even when the generated client does
  not throw. Used payload fields are decoded. No success is inferred from an
  empty/malformed body. An error remains unconfirmed during retry; previous data
  can be retained but cannot authorize a healthy snapshot badge.
- Identical snapshot timestamps do not renew its monotonic receipt lifetime;
  a regressed timestamp is rejected. Automatic background reads do not overlap
  an outstanding read (the backend's bounded read may exceed the poll period).
  Explicit refreshes retain latest-request ordering and unmount guards.

Timestamp ordering proves publication progress only. Legacy initial cached data,
republications with newly fabricated producer timestamps, incorrect firmware
measurements, producer clock rollback/incarnation and independent source identities
cannot all be resolved by this GUI. A conservative ordering barrier may delay
healthy display after a real producer restart until progress beyond its previous
maximum or a new hook session. It does not disable any robot operation.

`useDiagnostics` keeps its existing one-second throttle. Named malformed rows
cannot renew confidence; completely malformed arrays and FusionGraph frames
retain the prior valid sample without renewing its health window, in accordance
with their existing retention contract. Expired or unavailable ROS rows retain
the original message and reported numeric level beside the stale/unverified
presentation. Explicitly disabled LiDAR availability reports are labelled
non-applicable and remain inspectable; they do not demand that sensor be present.

## Compatibility and remaining anomalies

- Mowgli, MAVROS and OpenMower use the same GUI display rules and the existing
  canonical interfaces. Missing unsupported fields are not fabricated. Generic
  external diagnostics without known structured semantics are warnings or unknown,
  not assumed robot-critical faults. Per-deployment publisher/capability differences
  require passive hardware evidence; the GUI does not synthesize native telemetry
  to fill a different backend's gaps.
- Older snapshot servers remain readable, but cannot confirm Docker availability
  or the full standard core set without the new metadata. Legacy CPU zero remains
  ambiguous. Custom/manual deployments lack authoritative expected-membership
  metadata in this endpoint and can stay Unknown; no mandatory sidecar list is guessed.
- Existing Go cross-checks use zero-based datum/dock-yaw initialization heuristics.
  Explicit legitimate zero coordinates/yaw can therefore produce an ambiguous
  configuration warning. This patch does not rewrite that calibration contract:
  the global verdict treats those warnings as unverified configuration, not a
  confirmed malfunction, and preserves the producer's detailed warnings.
- The snapshot's existing `started_at` derives from Docker creation time, not an
  independently confirmed process start time. That field was not redefined.
- Onboarding's existing localizer checks use an 8-second receipt-time test,
  not the ordered health-publication evidence. They can therefore differ from
  this health verdict for replayed diagnostic data. Their pass/pending checks
  also gate Finish/apply, so they were audited and regression-tested but not
  rewritten in a display-only batch. A separate review should specify that
  readiness/persistence gate's freshness contract. The header's reported-state
  labels likewise keep their explicit freshness-unverified qualifier; they are
  not an independent all-good verdict.
- The former ROS summary name `EKF Map` remains historical wording. No removed
  EKF or alternative localizer is introduced. Renaming producers is separate work.
- The known map/dock transaction/correlation limits, unsupported GeoJSON holes,
  settings-payload work, GNSS category/provenance contracts and unrelated ESLint
  debt are outside this patch.
- The prior Python overlay mismatch remains outside this batch: installed
  Universal GNSS Python messages lack the checkout's `FIX_TYPE_2D_FIX`; no Python
  bridge pass or interface rebuild is claimed here.
- MAVROS root gitlink is `4f1ac1410488b118d2563992df96d08de9730598`, while its
  preexisting checkout remains `5328dd70d1fce34ee9e0c81ff3883b0260a712de`.
  Universal GNSS remains `9a329cc216ed137d0e375a4cfe1f5c26e1dd52d0`. No capability
  of the local MAVROS checkout is attributed to the unavailable pinned object
  or to a deployed image. Neither submodule was modified, staged or reset.

## Hardware acceptance — HARDWARE_REQUIRED

Software validation does not establish actual sensor availability or physical
health on any robot. Record the exact MowgliNext commit plus reviewed patch,
firmware image, robot unit, deployed image digests and both submodule revisions.
Those deployment/physical values were not observed here.

With blades disabled and unintended motion physically prevented, a qualified
operator should passively compare FR/EN Diagnostics with the actual ROS reports
(timestamped only where their message contracts provide it),
power/emergency/diagnostic messages, Docker state/role metadata and the actual
thermal-file reading. Use already recorded approved operational sessions to
compare startup, idle/charging, recording/manual and autonomous/recovery states;
do not issue motion, calibration, reset or configuration commands for this check.
Do not stop containers or unplug sensors on an active mower to manufacture cases.

Pass criteria: every displayed known value agrees with its eligible source;
missing/expired channels are neutral; explicit zero is preserved where its source
has availability; optional disabled LiDAR is not a failure; current confirmed
critical evidence and unverified emergency history win over missing information;
replayed data cannot restore a healthy verdict. A live autonomous acceptance run,
if later required, needs the project's separate operational safety procedure.
No physical freshness, positioning accuracy, emergency-clearance or blade/motor
safety acceptance is claimed by this batch.

## Validation before the blocking HighLevelStatus review

These runs preceded the correction documented at the end of this report.
The follow-up results below supersede frontend acceptance for the final patch.

- Focused final Vitest: **244 tests passed in 11 files**, including the verdict,
  presentation, snapshot, ordered diagnostic/status progression, battery, anatomy,
  header, unchanged GNSS and onboarding checks. Log:
  `/tmp/gui-lot26-focused-validated.log`.
- Full final `yarn test --pool=threads --maxWorkers=2`: **2,313 tests passed in
  193 files**, 231.10 seconds for Vitest, plus the appearance registry test.
  Baseline was 2,219: **94 additional frontend regressions**. Log:
  `/tmp/gui-lot26-vitest-validated.log`. Full runs made while adding deliberately
  failing regression cases are not counted as final acceptance results.
- Chromium: **8 passed**, `--retries=0 --workers=2`, 10.4 seconds. Six new health
  FR/EN cases plus both existing Lot 25 GNSS cases. They cover 390px mobile unknown
  evidence without overflow, missing CPU/container evidence, genuine healthy
  evidence, CPU-critical priority, return to Unknown for an empty Docker state,
  consistent neutral table/badge presentation and zero implicit writes. Log:
  `/tmp/gui-lot26-browser-validated.log`.
- `go test ./...` passed after the final Go changes
  (`/tmp/gui-lot26-go-full.log`). The three new snapshot/temperature test functions
  also passed **10 repetitions**, using a fake Docker provider and temporary
  thermal files. They distinguish successful empty Docker reads from failures,
  check label forwarding, explicit zero, malformed/NaN/infinite/empty file content,
  missing-file errors and backward-compatible numeric fields. Log:
  `/tmp/gui-lot26-go-repeated.log`. No ROS call is made by these new tests.
- `yarn tsc --noEmit` passed (`/tmp/gui-lot26-tsc-validated.log`).
- `yarn build --outDir /tmp/gui-lot26-dist` passed
  (`/tmp/gui-lot26-build-validated.log`).
- `yarn lint` passed: **105 warnings, zero errors**
  (`/tmp/gui-lot26-lint-validated.log`). The complete ESLint JSON inventories
  before and after have identical per-rule counts; no configuration change,
  suppression or artificial cast was used.
- `gofmt -l` returns no Go files. `git diff --check` and separate whitespace
  checks for all new files pass. No C++/ROS source was touched.

The initial browser harness selected both CPU temperature and CPU usage in French,
matched a refresh button without its accessible icon name, and counted the
existing POST-based `/config/keys/get` read as a write. Locators and request
classification were corrected; production behavior was not changed to satisfy
those harness errors. No arbitrary sleep, increased timeout, retry or console
masking was added. jsdom pseudo-element notices, the existing Ant Design v5 /
React 19 compatibility notice and the large-chunk build notice remain visible
and distinct from ESLint errors. This is targeted Chromium coverage, not a full
Playwright run; earlier unrelated Playwright failures are not claimed resolved.


| ESLint rule | Before | After |
| --- | ---: | ---: |
| `@typescript-eslint/no-explicit-any` | 35 | 35 |
| `@typescript-eslint/no-unsafe-argument` | 8 | 8 |
| `@typescript-eslint/no-unsafe-assignment` | 23 | 23 |
| `@typescript-eslint/no-unsafe-call` | 3 | 3 |
| `@typescript-eslint/no-unsafe-member-access` | 12 | 12 |
| `@typescript-eslint/no-unsafe-return` | 3 | 3 |
| `react-hooks/purity` | 2 | 2 |
| `react-hooks/set-state-in-effect` | 8 | 8 |
| `react-refresh/only-export-components` | 11 | 11 |
| **Total** | **105** | **105** |

## Exact review files and delivery

- `gui/pkg/api/diagnostics.go`
- `gui/pkg/api/diagnostics_health_test.go`
- `gui/web/docs/gui-lot26-diagnostics-health-integrity.md`
- `gui/web/src/components/MowerStatus.test.tsx`
- `gui/web/src/components/MowerStatus.tsx`
- `gui/web/src/components/RobotAnatomy.motorTelemetry.test.tsx`
- `gui/web/src/components/RobotAnatomy.tsx`
- `gui/web/src/components/diagnostics/DiagnosticHealthSummary.test.tsx`
- `gui/web/src/components/diagnostics/DiagnosticHealthSummary.tsx`
- `gui/web/src/hooks/useDiagnostics.health.test.ts`
- `gui/web/src/hooks/useDiagnostics.ts`
- `gui/web/src/hooks/useDiagnosticsSnapshot.test.ts`
- `gui/web/src/hooks/useDiagnosticsSnapshot.ts`
- `gui/web/src/hooks/useEmergency.ts`
- `gui/web/src/hooks/useFusionGraphDiagnostics.test.ts`
- `gui/web/src/hooks/useFusionGraphDiagnostics.ts`
- `gui/web/src/hooks/useHighLevelStatus.ts`
- `gui/web/src/hooks/usePower.ts`
- `gui/web/src/hooks/useStatusTelemetry.test.tsx`
- `gui/web/src/hooks/useValidatedTelemetry.ts`
- `gui/web/src/i18n/locales/en.json`
- `gui/web/src/i18n/locales/fr.json`
- `gui/web/src/pages/DiagnosticsPage.tsx`
- `gui/web/src/test/diagnosticHealthFixture.ts`
- `gui/web/src/utils/diagnosticHealth.test.ts`
- `gui/web/src/utils/diagnosticHealth.ts`
- `gui/web/src/utils/diagnosticTelemetry.ts`
- `gui/web/tests/e2e/diagnostics-health.spec.ts`

The complete review diff is `/tmp/gui-lot26-final.diff`, including all new files
and excluding both submodules. Source, tests and this integration report belong
to MowgliNext; temporary logs/build/browser outputs remain in `/tmp`.

Final index empty. Branch/HEAD unchanged. MAVROS and Universal GNSS preserved;
no staging, commit, push, reset, stash, clean, merge or rebase performed. Operator
review and GO COMMIT are still required before committing.


## Blocking review follow-up — real unstamped HighLevelStatus

The review found a genuine mismatch with the ROS contract. The shared hook's
ordered `publicationBlocked` barrier was applied to HighLevelStatus even though
this message cannot provide progression. After an invalid frame or disconnect,
valid real publications restored the raw payload but never restored diagnostic
context. In AUTONOMOUS, this suppressed the localizer check and downgraded a
confirmed structured Path Tracking ERROR to Warning. The earlier synthetic-stamp
HighLevelStatus tests concealed this defect; their ordering assertions now use
Emergency, whose ROS contract actually supplies a stamp.

Before changing production code, three new tests used every instance field of
`HighLevelStatus.msg`, with no timestamp. Reconnection recovery and invalid-frame
recovery **failed** (two red cases); expiry-only recovery already passed. Evidence:
`/tmp/gui-lot26-hls-red.log`. The same regressions now check restored operation,
required autonomous localizer visibility, Critical priority for a fresh structured
tracking ERROR, absent observation/publication identity and zero commands. A fourth
test covers continuous unchanged publications over 60 seconds, explicit zero
fields and emergency history preserved across recovery.

The policy is intentionally limited to **reported context liveness**. It does
not certify that state or battery measurements were reacquired. Silent replay
of an unstamped valid message cannot be detected. Refusing all subsequent
messages until a new frontend session would prevent legitimate recovery; making
up receipt-based observation identities would claim evidence the producer lacks.
The chosen policy accepts new validated delivery as a report, retains the visible
unverified-freshness qualification, and leaves emergency history, commands and
motion permissions untouched. A future producer identity/progression contract
could strengthen replay detection, but needs separate ROS design and approval;
no such protocol change is made here.

All four generic hook consumers were audited: only HighLevelStatus selects
receipt policy; Emergency, Power and Status remain ordered. `useGnssStatus` uses
the metadata type only and retains its separate per-source observation barriers.
The separate ROS/FusionGraph diagnostic consumers have a real DiagnosticArray
header timestamp and are unchanged. Their legacy missing-identity limitation
remains documented above. No timeout is offered as a substitute for identity.

Two additional real-page Chromium cases (FR/EN) use unstamped AUTONOMOUS messages
and a fresh structured Path Tracking ERROR. They verify invalid → valid recovery,
a genuine mock WebSocket close/reconnect, Critical priority after recovery, and
no configuration/robot writes. The existing healthy-page fixture also loses its
invented HighLevelStatus stamp. Expiration and continuous-delivery timing are
covered with the monotonic mocked clock in Vitest, not physical-acquisition claims.

Complementary source changes are limited to `useValidatedTelemetry.ts`,
`useHighLevelStatus.ts`, `diagnosticHealth.ts`, their existing status-hook tests,
the health Chromium test and this report. No Go, ROS, generated, emergency-bit,
GNSS or submodule change is added by this follow-up. Previous Python-overlay,
MAVROS-version and HARDWARE_REQUIRED limits above remain applicable.

Final follow-up validation:

- **248 focused Vitest tests passed in 11 files**
  (`/tmp/gui-lot26-hls-focused.log`).
- **2,317 Vitest tests passed in 193 files**, plus the separate appearance
  registry test; `yarn test --pool=threads --maxWorkers=2`, 228.53 seconds for
  Vitest (`/tmp/gui-lot26-hls-vitest.log`). Four additional tests cover the real
  unstamped contract; the three existing ordering tests now use Emergency.
- **10 Chromium tests passed**, FR/EN, `--retries=0 --workers=2`, 15.2 seconds:
  eight health cases and the two unchanged GNSS cases
  (`/tmp/gui-lot26-hls-browser.log`).
- TypeScript, frontend build and lint passed
  (`/tmp/gui-lot26-hls-tsc.log`, `/tmp/gui-lot26-hls-build.log`,
  `/tmp/gui-lot26-hls-lint.log`). Full ESLint JSON inventory remains **105
  warnings, zero errors**, with identical per-rule counts
  (`/tmp/gui-lot26-hls-eslint.json`).
- `git diff --check`, whitespace checks for new files and the Go formatting check
  pass. Go files are byte-for-byte unchanged since the pre-follow-up snapshot;
  the full Go result recorded above remains the applicable run and was not
  rerun for this frontend-only correction.
- Browser APIs/WebSockets are mocked; no robot command or hardware write ran.
  Existing compatibility, jsdom and large-chunk notices remain unsuppressed.

Complementary diff: `/tmp/gui-lot26-hls-recovery.diff` (six files). Complete
review diff, including new files and excluding submodules:
`/tmp/gui-lot26-final.diff`. Branch/HEAD unchanged, index empty, all Lot 26 changes
uncommitted. MAVROS and Universal GNSS preserved. No commit or push; GO COMMIT
remains with the operator.
