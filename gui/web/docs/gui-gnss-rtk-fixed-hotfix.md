# GNSS RTK Fixed classification — v1.6 GUI hotfix

## Baseline and scope

Branch `fix/gui-warnings-errors`, HEAD
`87003b0b1be59c16f2b38d0eb73836d5ccb94b05` (Lot 27 committed).
Baseline: 2,340 Vitest tests, 105 ESLint warnings, zero errors.
The independent MAVROS submodule modification is preserved; Universal GNSS is
unchanged. Nothing is staged, committed or pushed.

The only production change is in `src/utils/gpsStatus.ts`. No decoder, telemetry
hook, diagnostic health/history, ROS interface, bridge, producer, command,
dependency or configuration is changed. Existing percentages are display scores,
not probabilities or measurements of positioning accuracy.

## Reproduction and correction

The operator's Mowgli bridge observation contains `backend=universal`,
`fix_valid=true`, `fix_type=6`, `rtk_mode=3`, capabilities 260271615,
values 260074911, horizontal accuracy 0.014000000432133675 m and observation
sequence 326347. The original observation, sequence 306451 and approximately
0.014 m, is covered separately. Both masks expose RTK mode.

Before correction, the new contract suite reported **three failures and one
pass**: decoding passed, but Fixed classification, available Fixed mode and
3D/Float classification failed. Red evidence:
`/tmp/gui-gnss-rtk-red.log`. The decoder was not the cause.

`contradictoryRtk()` incorrectly placed the position dimension `3D_FIX` in the
same incompatible category as 2D and DGPS. Universal/u-blox legitimately reports
3D positioning and carrier resolution independently. Removing that one category
allows available `3D + Fixed` and `3D + Float`. A second narrow change retains a
valid 2D/DGPS base category when its RTK claim is incompatible, while suppressing
that claim in `currentRtkMode()`. Opposing explicit RTK classifications remain
unknown. No assertion of RTK comes from correction activity, centimetric accuracy,
coordinates or a changed backend name.

For valid positioning and an available RTK mode, the tested matrix is:

| Position type | Unknown (0) | None (1) | Float (2) | Fixed (3) |
| --- | --- | --- | --- | --- |
| No Fix (0) | No Fix | No Fix | No Fix | No Fix |
| Generic GPS (1) | Unknown | Unknown | Float | Fixed |
| RTK Float (2) | Float | Unknown | Float | Unknown |
| RTK Fixed (3) | Fixed | Unknown | Unknown | Fixed |
| Dead reckoning (4) | Unknown | Unknown | Unknown | Unknown |
| 2D (5) | 2D | 2D | 2D; RTK suppressed | 2D; RTK suppressed |
| 3D (6) | 3D | 3D | Float | Fixed |
| DGPS (7) | DGPS | DGPS | DGPS; RTK suppressed | DGPS; RTK suppressed |

`fix_valid=false` and explicit dead reckoning prevent positive RTK status.
Absent/masked RTK retains the base category; existing legacy explicit RTK fix
types retain their previous meaning. Malformed enums/masks still fail decoding.
Zero measurements and the optional unavailable `NaN → null` contract are unchanged.

## Passive observations on the two robots

Evidence below is limited to these units and deployed images on **2026-10-16**.
No restart, configuration change, install, update, motion, arming, navigation,
blade command or write API was executed. Version GETs and a read-only GNSS
WebSocket subscription were used; all browser acceptance tests use mocks.

The operator verified the changed Rock 5B ED25519 host fingerprint from a trusted
console. Tool SSH authentication still failed; no host verification was disabled
and no secret was entered into tool commands or saved. The operator reports
interactive SSH sessions, but no reusable tool control connection was found.
These direct tool observations therefore use the existing read-only GUI API,
not a claimed successful SSH login. ROS graph details are separately attributed
to the operator's terminal output.

At 08:50:35 UTC, `/api/system/versions` and `/web-build.json` reported identical
GUI and ROS deployments on both hosts:

| Component | Running image version/revision | Image digest |
| --- | --- | --- |
| GUI, both robots | fix-gui-warnings-errors; `9105cb61e2d476f99142362b14f8e3b6f9ff3d70` | `sha256:0ba75ccd27970d0db8c473f7cc17b68a7f56730ff9259cb4f9384a63bf3962a0` |
| ROS, both robots | fix-gui-warnings-errors; `20920bf331c3316899ba1930e16d4cc1d51f4801` | `sha256:e1571787b895c9c3a151feec502fb335a95a343d18d401aef861fb23e8508230` |
| GPS container, both robots | Universal GNSS v0.7.4-rc5; `9a329cc216ed137d0e375a4cfe1f5c26e1dd52d0` | `sha256:127103ef9e90c3fa47edb38c01797128f8c1e616ab4548a35fe135ff16349727` |
| MAVROS, Rock 5B | sidecar latest, version 26.04; revision not reported | `sha256:53f187470235055d8059f2b4a49162ceeae3d9dc91394912d70dd8da69d97090` |

The served web build ID on both is `d0b2b784-7a49-4557-9245-943d1b8bec6a`,
built 2026-10-15T19:10:01.245Z at GUI revision 9105cb61. This identifies served
assets, not the bundle already loaded in an operator's possibly cached browser.
Neither served build contains this hotfix. A running GPS container alone does
not prove it publishes the active position source.

At 08:52:30 UTC, read-only `/api/mowglinext/subscribe/gnssStatus` delivered three
distinct observation sequences from each robot. This route forwards `/gps/status`;
the client decoded its base64 JSON envelopes and sent no ROS/service commands.
The retained JSON fixture contains only consumed GNSS fields, not coordinates,
credentials or unrelated receiver data.

| Unit / producer evidence | `/gps/status` evidence | GUI processing | Display |
| --- | --- | --- | --- |
| Peuchmower2, 192.168.10.35: operator observed Universal GNSS fix 7, valid true, mode 3, sequence 326336; source masks 126086015 / 126026623 | Operator bridge sequence 326347; direct tool samples **328265 → 328266 → 328267**, backend universal, fix 6, mode 3, masks 260271615 / 260074911 | Real captured fields decode; hook accepts identifiable progression; old classifier rejects 3D/carrier pairing; corrected classifier returns Fixed | Operator reported Unknown before correction; local mocked FR/EN card, banner and health badge show **FIXED**, display score 100% |
| Rock 5B, 192.168.10.33: operator's ROS graph identifies the sole publisher `/mavros/mowgli_gnss`, reliable/volatile, depth 10; Universal status topic absent | Direct tool samples **58730 → 58731 → 58732**, backend mavros_gps2, fix 2, mode 2, masks 7455 / 7455; operator's later sample sequence 58820 agrees | Real captured fields decode; existing explicit Float classification retained | Operator reported FLOAT working; local classification remains **FLOAT**, display score 80% |

Both direct samples report horizontal accuracy 0.014000000432133675 m. This is a
receiver-reported number, not surveyed physical accuracy. The source and bridge
operator samples were taken at different times; their sequence difference is not
evidence of a translation error. Increasing producer sequences establish reported
identity progression, not independently verified physical acquisition or location.
Mowgli's publisher node names were not supplied or independently read; both source
and bridge topics were reported present by the operator.

`src/test/gnssBridgeFixedFixture.ts` preserves the exact operator subset. Additional
zero/false fields required by its value mask are explicitly labelled **synthetic
test scaffolding**, never hardware observations. Separately,
`src/test/fixtures/gnss-hotfix-passive-wire.json` contains actual whitelisted wire
fields from the passive reads, including unavailable nulls and real headers;
both captured frames pass decoding/classification without synthetic scaffolding.

Temporary sanitized observations remain in `/tmp/gui-gnss-passive-versions.json`
and `/tmp/gui-gnss-passive-status.json`. No submodule-specific audit is stored here;
this is a MowgliNext consumer/integration comparison. The deployed MAVROS image
does not expose a revision, so no capability is attributed to the pinned gitlink
or the independent local checkout from this image evidence.

## Regression coverage and validation

The contract tests exercise all 32 fix/mode pairs, availability masks, missing
mode, malformed data, No Fix, false validity, dead reckoning, both operator
observations and both real passive wire samples. The hook regression covers
expiry, repeated identity, reconnect, No Fix and recovery on progression using
the corrected 3D/Fixed classification. Existing Lot 25 source round-trip,
unknown-backend, legacy, zero/null and cross-source isolation cases are rerun.
No production hook or decoder is changed.

FR/EN rendering tests cover both GNSS cards. Four new Chromium cases cover
390/1440 px, real page/hook integration over mocked WebSocket, Fixed/Float,
masked RTK → 3D, No Fix/replay/progression, a new MAVROS source baseline and
Float recovery. They assert banner percentages and the diagnostic GNSS badge,
and no configuration/parameter/robot write. The MAVROS source-switch browser
fixture is synthetic gps1; the separate real captured witness is gps2.

Final validation on the frozen source/test tree:

- Focused Vitest: **292 passed in nine files**, 5.31 seconds, including the two
  real captured wire frames (`/tmp/gui-gnss-rtk-focused-final.log`).
- Full `yarn test --pool=threads --maxWorkers=2`: **2,396 passed in 197 files**,
  233.69 seconds for Vitest, plus the appearance registry test. Baseline 2,340;
  **56 additional tests** (`/tmp/gui-gnss-rtk-vitest-final.log`).
- Chromium: **20 passed**, FR/EN desktop/mobile, **no retry**, 39.8 seconds:
  four new hotfix cases, two Lot 25 GNSS cases, eight Lot 26 health cases and six
  Lot 27 alert-history cases. This is targeted coverage, not all Playwright.
- `yarn tsc --noEmit`: passed.
- `yarn build --outDir /tmp/gui-gnss-rtk-dist`: passed.
- `yarn lint`: passed. Global source JSON inventory: **105 → 105 warnings,
  zero errors**; no warning/error in changed source/tests.
- `git diff --check`, new-file whitespace checks and changed JSON parsing passed.
  No formatter is required for this TypeScript/JSON/Markdown-only change.
- An initial extra `eslint .` invocation reached non-source tooling without
  typed parser configuration and failed. The repository's actual `yarn lint`
  command and matching `eslint src -f json` both pass; no configuration was
  changed to accommodate the broader command.

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

## Remaining acceptance and limits

**HARDWARE_REQUIRED — corrected GUI deployment/display acceptance.** No patched
image was deployed and no corrected live robot display is claimed. The exact
observed deployment baseline is listed above; robot firmware, physical receiver
firmware and deployed MAVROS git revision were not obtained and must be recorded
before extending this evidence. Repository hotfix baseline is HEAD 87003b0b plus
the review diff; its eventual commit and deployed GUI digest must also be recorded.

After separately authorized deployment, with the robot stationary, blades
disabled and motion prevented, read the existing status topics and GUI stream
passively. Confirm progressive valid Universal fix 6/mode 3 with RTK availability
shows FIXED consistently in FR/EN card/banner; progressive MAVROS fix 2/mode 2
continues to show FLOAT. Observe naturally occurring No Fix/unavailable/stale
states if available; do not induce robot faults or issue commands for this check.
Pass requires agreement with the admissible source, no stale Fixed resurrection
and no implicit write. Ground-truth accuracy and mowing/docking acceptance are
separate physical questions.

The prior Python bridge-test overlay limit remains: installed ROS constants lack
`FIX_TYPE_2D_FIX` although the checkout IDL declares it. No Python pass is claimed,
no interface/overlay was changed and no ROS test is required by this GUI-only diff.
No Go code changed; full Go was not rerun for this hotfix. Existing build chunk,
Ant Design compatibility/Descriptions layout and jsdom pseudo-element notices
are not suppressed or represented as ESLint errors.

## Review files

- `gui/web/src/utils/gpsStatus.ts`
- `gui/web/src/utils/gpsStatus.rtkContract.test.ts` (new)
- `gui/web/src/utils/gnssTelemetry.test.ts`
- `gui/web/src/hooks/useGnssStatus.test.tsx`
- `gui/web/src/components/gnss/GnssRtkBridge.test.tsx` (new)
- `gui/web/src/test/gnssBridgeFixedFixture.ts` (new)
- `gui/web/src/test/fixtures/gnss-hotfix-passive-wire.json` (new)
- `gui/web/tests/e2e/gnss-rtk-bridge.spec.ts` (new)
- This report (new).

Complete review diff: `/tmp/gui-gnss-rtk-final.diff`, including new files and
excluding MAVROS. Index must remain empty. Independent review and GO COMMIT are
required before any staging, commit or push.

Final Git check: branch/HEAD match the baseline; index empty. MAVROS remains at
the preexisting local revision `5328dd70d1fce34ee9e0c81ff3883b0260a712de`, against
pinned gitlink `4f1ac1410488b118d2563992df96d08de9730598`; the unavailable pinned
object from the earlier audit is not a basis for capability claims. Universal
GNSS remains at its unchanged pinned revision
`9a329cc216ed137d0e375a4cfe1f5c26e1dd52d0`.
