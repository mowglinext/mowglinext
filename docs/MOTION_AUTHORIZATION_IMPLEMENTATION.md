# Motion authorization implementation and acceptance (#924)

This is a safety-critical, partial implementation of #924. It verifies FTC's
intended commands, checks the assumed forward response against mowing geometry,
and bounds EscapeStartBlocked. It does not establish the
stack-wide autonomous-motion invariant or physical containment.

## FollowStrip late-result ownership (2026-10-29)

The continuation found a separate real action-lifecycle defect: FollowStrip
reset and reused one outcome slot across successive coverage goals, and one
across its blade-off transits. An abandoned goal's callback still owned that
same slot. After halt and a new dispatch, a delayed old success could decide
the new nonterminal handle, marking an unfinished unit mowed or permitting
coverage before the new transit finished. Current fetched dev has the same
pattern; this is not retrospective attribution of the original boundary event.

Each FollowStrip follow/transit dispatch now allocates its own outcome slot,
matching the existing ownership of result-detail and feedback slots. Old
callbacks retain their old slot. Cancellation, deadlines, result precedence,
blade checks and completion criteria are unchanged. Two real FollowStrip tests
with controlled fake Nav2 servers reject the old cancellation, restart the
same tree, accept a new goal and then deliver the old result. Both are RED on
the previous production objects, with observed callback arrival and new-slot
contamination; both pass after the fix. Seven focused lifecycle tests pass in
17.736 s. Independent review and clang-format 18 checks pass. This is real ROS
action transport/component evidence, not a real ControllerServer or hardware
run. Source, binaries and XML are retained in `followstrip-late-result/` and
`followstrip-attempt-independent-review/`.

Standalone TransitToStrip and DetourAroundObstacle have analogous pre-existing
outcome-slot reuse, not exercised or changed by this FollowStrip-specific
repair. Their late-result schedules need separate focused investigation.

## Known-source TF knots (2026-10-29)

A separate actual-Node probe exposes an avoidable producer-side history error.
The node knows the consumed source estimate x=0.036 m at source time 100.400 s
and correctly publishes its forecast x=0.046 m at 100.500 s. However, querying
TF at the known source time still returns an older forecast x=0.042 m. The
6 mm difference is a source-estimate presentation error in this controlled
held-input model, not a measured physical error or proof about the old event.
All 25 newly emitted future predictions match the independent input model.

Both TF publisher modes now insert the captured source pose at its source
timestamp before the unchanged future forecast. Map/odom and odom/base samples
use one captured motion/anchor state; inline publication is paired in
PublishLocalOdom after timer anchor processing. The dedicated thread retains
exclusive TF ownership when enabled. Zero rounded lead emits one sample per
leg. Odometry messages, prediction equations, lead, noise and tolerances do not
change. Source backfill deliberately makes wire timestamp order nonmonotonic.

The persistent real-Node `test_tf_source_epoch.py` fails against the old producer
in both positive-lead modes; its zero-lead control passes. All three cases pass
against the repaired executable, checking source and forecast values, paired
TF batches, zero-lead duplicates and anchored map/base composition. The final
52-case Node fixture also passes after relinking only the changed publisher TU.
The final comment-corrected rebuild produces identical publisher, Node and test
binaries to the tested controls. Python flake8/pep257 and C++ changed-line
clang-format 18 pass. CMake configuration discovers the new isolated pytest;
the registered CTest passes against the exact validated cached executable.
This is focused cached compilation/registration evidence, not a clean workspace
build. Independent review covers the publisher and test/build registration.

Only the exact captured source knots are repaired. Older forecast knots can
remain between them, and already-consumed lookups cannot be revised. Arbitrary
interpolation accuracy, in-flight source-epoch reset races, atomic TF revision
identity and physical pose accuracy remain unqualified. Exact packets, old/new
binaries, controls and limits are in `producer-tf-epoch/`,
`producer-tf-epoch-green/` and `producer-tf-independent-review/`.

A combined current-Node/Core and current-FTC/goal-checker native component also
passes one precise straight stop with the unchanged 0.03 m XY request and
0.1 s TF lead. At first checker success/STOP the modeled native body is at
x=0.772644 m; real modeled wheel coast adds 0.012834 m and settles at
x=0.785478 m, 14.522 mm from the 0.80 m goal. There are 93 exact-rest sample
pairs and no later checker reversal. This directly constructed component uses
public initialization and the real goal/local-plan transform-header contract;
it does not run ControllerServer, BT or mux. It is a combined integration
control, not a failing-before comparison or hardware result. One failed
pre-motion executable-permission start is retained separately. Exact provenance
and the single valid attempt are in `native-precision-source-consumer-attempt02/`.

## Consumer evaluation epoch (2026-10-29)

Two focused production-consumer probes expose a different time error from an
incorrectly stamped producer. With supported x=0.76 at ROS time 100 and forecast
x=0.78 at 100.1, the actual goal checker accepts a 0.80 m goal with 0.03 m
tolerance while the supported current error is still 0.04 m. FTC can likewise
park using the future pose. Separately, a response sweep starting at forecast
x=0.72 rejects a request that fits from supported x=0.70: at 0.15 m/s the unchanged
0.6 s response reaches x=0.79 inside the x=0.80 boundary.

Consumers now select a supported TF epoch no later than the captured ROS time.
Past latest samples retain their existing behavior. Each nonzero FTC output
selects one complete map/base-chain timestamp for both final guards; STOP needs
no TF. Terminal decisions use the supported epoch, and a valid supplied checker
can continue the bounded approach only outside the effective distance cap.
The existing default/null/looser 100-command controls remain identical. Geometry,
coast bounds, speed settings, timeout and the existing strict-XY versus legacy-3D
distance rule remain unchanged.

The goal checker also recovers the supplied goal's original transform epoch
from the fourth argument's local-plan header, rather than substituting a topic
endpoint or comparing an old odom goal with a new map correction. The installed
Nav2 1.5.2 binary and matching upstream FeasiblePathHandler source confirm that
header provenance. XY-only checks continue to ignore only yaw. Missing required
TF or goal metadata holds completion without advancing progress. If no full
plan has ever identified its fixed frame and TF is future-dated, the old
no-plan proximity fallback cannot be evaluated safely: its watchdog reports the
missing context, and completion waits for the plan. The past-latest fallback
and recovery on plan arrival are tested.

Eight actual-FTC cases are RED on the previous object; **77 FTC cases pass** with
the final repair. The separate old-checker probe has two RED cases and a passing
past-latest control. Fresh coherent builds pass **14 goal-checker and 44
end-approach cases**, including moving anchors, asynchronous TF edges, missing
metadata/brackets and the missing-plan lifecycle. All 28 authorization cases
also pass after recompiling their fixture against the changed private API.
Seven existing focused turn-fallback cases pass with the same final FTC object.
Independent review pins all six changed C++ files. Evidence is retained in
`ftc-consumer-epoch/`, `terminal-reachability-review/` and
`consumer-epoch-independent-review/`.

The common selected time is not atomic TF revision identity: same-stamp edge
updates can still interleave. These tests do not establish actuator response,
downstream command ownership or complete mowing. The earlier native precise
stop, before this consumer repair, passes at 11.081 mm final physical error with
90 exact-rest epochs; it is not a native RED for this newly isolated software
defect. Its exact baseline and failed instrument starts remain in
`native-precise-stop/`.

One new controlled native cusp comparison with the final FTC passes the
unchanged y>=0.80 reference condition through actual coast and rest: minimum
y=0.8587798966 m, first zero applied twist at 3.30 s after denial at 3.14 s.
All 14 comparison inputs and the prior pre-anchor Node/Core/support artifacts
remain unchanged; this isolates the new FTC against that older component
baseline. The fixture uses a private captured-cursor seed and null goal checker.
It records 27 actual caller inputs, 26 returns and one geometry denial, without
new internal predicate logging. It is containment regression evidence, not a
complete current-stack run or corner traversal. The single attempt and exact
provenance are in `native-cusp-consumer-epoch/`.

## Constructive corner frontier (2026-10-29)

One fixed external route now preserves the entire original continuous 9 cm
required-line coverage, passes continuous body sweeps and required full-disc
pivots, and includes a proof for the unchanged 3 cm terminal endpoint band.
It replaces the response-rejected LSR with a fixed right arc and a secondary
visit to retained point 187. Sampled 0.15 m/s response checks pass through the
earliest covering endpoint. The shipped slow speed is 0.16 m/s; an actual
shipped-parameter command/measurement sequence is still required. A previously
permitted command can enter a STOP-only endpoint, so a denied nonzero endpoint
request alone is not an impossibility proof.

An external experimental copy of actual FTC and a dedicated checker now
completes the two-visit local maneuver in one controlled, current-only TF,
0.1 s zero-order-hold run. Both original retreat endpoints are reached within
5 mm; conservative debit stays below the existing 0.30 m ordinary allowance.
The first version refilled after only 0.54 m of actual forward travel because
the indexed carrot was ahead. A targeted experimental progress-credit change
refills only after 0.615 m of modeled qualifying forward travel, beyond the
unchanged 0.60 m requirement. It does not grant a new allowance per phase or
change legacy accounting. This attempt-local experiment is outside production.

The 162 actually executed positive-command intervals preserve all 34 original
continuous required lines at 9 cm when inserted into the specified nominal
prefix and **unexecuted** R-to-J/suffix context. The exact production planner
exports raw doubles and asserts the reconstructed old route vector matches;
both nominal and realized-local Difference are empty. This does not establish
actual full-route cutting, successor tracking or body/pivot permission at the
new rejoin. The prototype's R differs from the earlier static certificate.

Resume remains a concrete software blocker. Replaying the actual production
progress helper at 10 Hz against that successful local trace jumps to future
overlapping visits: by tick 30 the cursor is 52, beyond both retreat legs, before
the first reverse begins. A halt at tick 55 would therefore save the wrong
suffix under the inspected FollowStrip persistence/trim code. This is a helper
replay and source-derived halt consequence, not an executed geared FollowStrip
halt/reload. Five separately executed real FollowStrip tests with fake Nav2
servers pass their existing forward-path lifecycle controls. The experimental
controller/checker are direct executable objects, without plugin factories;
actual ControllerServer plus geared FollowStrip cancel/resume remains unrun.

No production planner/phase change or extraction is shipped. Exact evidence is
in `maneuver-seam-prototype/actual-progress/`, `realized-coverage-export/` and
`coverage-maneuver-contract/seam-followstrip/`. The static certificate remains
in `cusp-critical-terminal-topology/`; the earlier coverage/body GREEN but
response-RED LSR remains in `cusp-composite-original-spur/`.

## Precise terminal approach prerequisite (2026-10-29)

A separate actual-controller regression shows FTC ignores the supplied goal
checker's tighter XY tolerance. On a 0.80 m straight path with a 0.03 m checker,
the old controller stops at x=0.675 m. It cannot execute a precise terminal
coverage visit merely by selecting a stricter goal checker.

The repair uses a stack-local minimum of the existing configured cap and a
valid tighter requested XY tolerance. Existing final-carrot transition into
WAITING and its goal timeout remain intact; only that stricter terminal approach
can keep driving there. The same flag preserves forward body/response checks.
Null, default, looser, unavailable and invalid checker inputs retain the old
command sequence. No shipped tolerance, reverse limit or controller state budget
is enlarged. All 66 speed-limit/controller cases pass, including the preserved
stalled/overshot timeout, transient-obstacle restart and the shipped 0.093 m
base-link height with nonzero heading. Strict XY completion uses planar distance;
legacy/default completion retains its old behavior. Another 44 progress, 28
authorization and seven focused fallback cases pass with the final FTC overlay. Independent final review and additional guard evidence
are retained in `ftc-goal-tolerance/` and `ftc-goal-tolerance-repair/`.

A coverage-equivalent terminal spur plus the earlier LSR shortcut covers every
captured required line. Existing 0.30 m reverse geometry reaches a certified
full-disc pivot, but the route still needs explicit bounded phase execution and
certified rejoin/unit budget. These prerequisites and ideal-controller tests do
not establish native braking, a complete corner or physical containment.

## Automatic dock seed provenance (2026-10-29)

A newly reproduced production path invents a dock PosePrior after six seconds
with unknown hardware status. The actual Node adopts dock (5,8,pi) despite
having only GNSS (1,2) and no heading. Time cannot establish dock location.
Automatic dock seeding now requires positive charging evidence in the shared
helper and boot fallback. Hardware-status edge predicates use the previous
state; the new charging state is committed before the first guarded seed.
Known-charging/no-GNSS fallback, first charging callback, GNSS plus heading
without hardware status, and explicit operator initialization remain available.
When both charging and absolute position/heading are unknown, map TF remains
uninitialized; activation may wait rather than use an invented location.

The two unknown-status regressions fail on the old actual Node; both charging
controls pass. All **52 actual Node cases pass** after rebuilding the three
touched production translation units, and the real save-service fixture passes
against the linked production executable. Noise, COG gates, timeouts and
acceptance tolerances are unchanged. Independent review has no blocker; exact
source/binary pins and RED/GREEN logs are in `dock-seed-provenance/`.
This prevents a concrete bad-prior origin; general recovery from incorrect
authoritative seeds or corrupted gyro history remains unqualified.

## Delayed CI failure repair (2026-10-29)

The failed build/test notification for older 4ba70aad contains a successful
workspace build but three lint errors and a save-service fixture failure.
The fixture supplied wheel callbacks without IMU intervals, leaving its explicit
seed pending under the repaired source-time contract. Adding stamped zero-rate
IMU input preserves all original save/busy/executor-progress assertions and
10-second limits: the actual Node test is RED before and GREEN afterward.
Simulator docstring/import/blank-line fixes pass package-wide flake8/pep257 and
all 17 existing wheel regressions. No production behavior or acceptance limit
changes. Exact evidence is in `ci-4ba70aa-save-fixture/`.

## Delayed COG producer motion context (2026-10-29)

A separate actual-producer-to-Node experiment finds another false-flip cause.
Identical reverse GNSS fixes produce body yaw zero when delivered in sequence.
If processed after a newer forward wheel callback, the old producer uses that
latest sign and publishes three yaw-pi headings at the original fix stamps.
The actual Node accepts them and triggers erroneous recovery before their yaw
factors run, moving the previously correct heading to 1.665649789 rad. This is
an upstream context error, separate from historical factor association and the
still-unresolved genuine-flip prior problem.

The producer repair retains bounded, ordered wheel/gyro source history and uses
held context at the fix epoch. Every recorded direction/rest transition inside
the baseline invalidates it; baseline rotation is integrated in source time.
Missing context rejects explicitly instead of substituting a future cached rate.
Existing heading/noise equations, timing limits and gate thresholds stay intact.
This is a declared held-input model, not physical acquisition calibration.

Independent review also exposed two lifecycle requirements: a delayed heading
must carry already-observed intervening rotation into its stationary latch,
and stationary timer observations must participate in ROS-clock rewind detection.
A historical heading cannot become a current latch by erasing a completed turn;
a timer-only clock high-water cannot be ignored when the next input arrives.
Their exact discriminating fixtures and review findings are retained alongside
the producer/Node RED in `cog-delayed-direction-repro/` and the focused repair
validation in `cog-source-motion-validation/`.

The actual production executable builds and all **48 focused tests pass**:
7 history, 7 actual publisher, 15 existing yaw-math and 19 existing observation/
latch-timing cases. Independent review has no remaining finding. The same
safe-heading/recovery assertions fail against the old producer/actual Node.
A new combined producer/Node translation unit reached its 90 s compile cap;
it was stopped. The alternative new-producer capture uses identical callback
inputs and its complete stamp/quaternion/variance rows are byte-identical to
the retained correct control already consumed by the actual Node with zero
recoveries. This is deterministic input-equivalence composition, not a new
end-to-end Node rerun. Exact proof and preserved failed build attempts are in
`cog-delayed-direction-paired/`; no full mission or hardware run is claimed.

## Wheel-travel cancellation defect (2026-10-29)

A further public-Core regression finds an independent covariance defect: wheel
random-walk distance used the norm of signed accumulated displacement. A
forward/reverse pair inside one cadence edge therefore cancelled its travel
noise. Two 100 ms intervals at +0.2/-0.2 m/s produce process x variance 0.000020
in one 200 ms edge, versus 0.000120 in two 100 ms edges. The same physical
straight trajectory has identical endpoint and midpoint means; the x marginal
differs by 0.000100. Same-sign forward/reverse controls pass on the old Core.

The narrow repair accumulates each raw wheel translation magnitude separately
from signed motion. Bootstrap rebuilds that magnitude from only the retained,
clipped consumed intervals. Existing noise coefficients, creep floor, fault
releases, motion gates and safety tolerances stay intact. This restores the
claimed travel-distance random-walk rule; it does not establish calibrated
physical covariance or general nonlinear posterior cadence invariance.

Exact failing baseline inputs, test source, old ABI header, binary identities
and bounds are retained in `source-reversal-red/`. The identical final three new cases all fail on the old Core; its six existing
cases pass. After a coherent Core/actual Node rebuild, all **15 selected CTest
targets / 156 GTests pass**, including all nine wheel-noise cases. Independent
review has no blocker. Exact GREEN validation and review are recorded in
`source-reversal-green/` and `source-reversal-review.md`.

## Further cusp and heading-reconstruction evidence (2026-10-29)

The current planner, called with server-equivalent geometry, 0.55 m footprint
width and actual pivot limits, reproduces all 9,268 captured positions exactly
as the suffix of its 9,420-point route. The earlier bevel repair does not remove
the problematic link: 17.691661 mm with a 151.913636-degree heading reversal and
no pivot twin. Normal positive following at 0.15 m/s and at most 0.8 rad/s cannot
remain above y=0.8 from that approach. A clockwise body pivot fits the raw-hole
geometry, but inserting a pivot alone does not make its arrival safe under the
unchanged forward-response guard.

The remaining planner defect is a false continuous-path contract: when both the
fillet and the certified pivot fail at an internal ring corner, the corner is
still returned inside one driven unit. A candidate partitions residual corners
through the existing blade-off sub-path transit protocol, preserving finalized
directed geometry and ring order. Its three new tests are GREEN on the
candidate and RED against the old archive with the identical test object.
Review exposed two-pose fragments; collinear midpoint encoding and explicit
rejection of unrepresentable tiny fragments resolve that issue without
changing FTC's contracts.

That candidate is **not included**: it violates the unchanged Isabey maximum
of six sub-paths. The actual old output has four; eight added cuts produce
twelve. One is an 84.38-degree gentle corner, while seven exceed the existing
88-degree cusp threshold. Correcting that classification alone still gives
eleven paths. Every local minimum-radius fillet lacks the required adjacent
support. None of the six existing same-vertex Dubins loop variants succeeds at
those corners or at captured #950; every valid candidate leaves the clearance
boundary on its first moving sample. No split, pivot, radius, geometry or efficiency
assertion is relaxed to publish the candidate. Its complete patch and exact
files remain under `cusp-feasibility-frontier/unmerged-candidate-088ad8c9/`.

A further fixed neighborhood probe tests the existing six Dubins words at
radius 0.20 m, with supports 0.2/0.4/0.8 m before and after each of the eight
Isabey corners and captured #950: 486 variants. Both unchanged baselines first
pass continuous original-line coverage. Only one Isabey corner has a contained
replacement preserving global 9 cm mowing-line coverage; captured #950 has
none. Its best contained shortcut leaves 0.354825 m of required line uncovered.
One additional Isabey variant has unknown coverage because its sampling violates
the unchanged minimum-step contract; even qualifying it cannot close the route
budget. No missing coverage is waived. These individual centreline certificates
keep every other old pass unchanged and do not qualify combined replacements,
body sweeps or controller response.

New maneuver construction or an execution-contract decision is still required;
complete route execution and physical transit remain unqualified.

Only the include dependency repair is included from that candidate: the public
planner header names its actual geometry types, and implementation/test users
include their actual F2C algorithms directly. Unrelated route solvers and
OR-Tools headers no longer enter every planner client. A fresh serial Release
build passes all 12 unchanged coverage/replay targets at their original bounds,
including the six-path Isabey requirement. This is a build repair, not a cusp
repair. Earlier compiler timeouts and the rejected behavioral patch remain
preserved separately.

A new moving **accepted Core likelihood** experiment also rules out a simple
heading-prior replacement. It retains the same consumed controls, delayed
source unaries, nonzero antenna lever and noises in both gyro modes. Correcting
only the initial prior leaves wrong frozen acquisition likelihood; rebuilding
those factors through actual corrected-initialization replay restores the full
joint mean and covariance, including bias correlations. In that mutually
consistent fixture, subsequent actual prefix marginalizations and boundary/future
exact-once application agree with the untrimmed correct control. These are
controlled model results, not parity of the Node's admission gates or operational
recovery.

A follow-up uses actual learned nonzero bias and fixed conflicting observations.
Correct-initialization replay matches the independently instantiated control
exactly through two prefixes when both use the same construction and
relinearization schedule. Reusing the wrong run's prediction-bias references is
numerically different from causal raw replay, which must regenerate them and
rebuild endpoint factors as well as acquisition bridges. Raw gyro must enter
absolute-bias preintegration once. Exact equality to an untrimmed nonlinear
posterior does not generalize: the unchanged 1e-9 comparison remains RED, with
maximum mean difference 2.59e-4 after the second causal-model prefix. This is the
usual frozen-prefix approximation, not evidence of a new common-linearization
information-loss defect. The reconstruction contract must specify reference
policy and the rebase/relinearization schedule.

Production reconstruction requires the accepted history back to the affected
seed, including controls, source/cadence boundaries, gate outcomes, bias
references and observation payloads/noises/order. Current pruning removes old
edges and assimilated payloads; ID registries cannot reconstruct them. Copying
their contaminated separator or anchoring an already wrong moving XY at 5 mm
does not recover that information. No history-discarding recovery policy or
unbounded journal is silently introduced.

Exact new evidence: `~/.cache/pr950-resume-20261029/cusp-feasibility-frontier/`
and `moving-flip-reconstruction/`, plus `cusp-neighborhood-probe/` and
`moving-flip-bias-probe/`, including actual source/binary pins, raw planner
stages, keyed full-joint matrices and bounded experiment commands.

## Source-time pose and heading repairs (2026-10-29)

The actual Node's GNSS phase discriminator now passes all 20 controls, admitting
and applying all 18 valid fixes. Each perfect-motion final position stays within
1 micrometre of the oracle; immediate/delayed delivery agrees within 1 nanometre.
The retained old discriminator produced 14.814123 mm error by attaching epoch
1.24 s to node 1.14 s. No observation-dropping workaround is used. These are
software regressions, not attribution of the original 13.943731 mm event.

The production integration separates the integer timer schedule from the last
wholly consumed IMU source boundary. Wheel callbacks retain their existing gate
statistics/noise horizon; immutable consumed wheel/gyro controls define a shared
Expmap trajectory and its actual source epoch. Cadence gates are evaluated once,
and their translation/rotation verdicts and final process sigmas are frozen per
edge. Valid observations beyond the committed boundary wait for support instead
of forcing cadence nodes or being assigned to an older pose.

GNSS and COG/magnetometer factors share conditional acquisition states. The
frozen normalized **first-order** trajectory extension retains the original
endpoint likelihood, uses existing process diffusion, conditions on held wheel
controls, and handles multiple observations with their joint conditional law.
It is not exact sensor preintegration or a physical calibration of wheel noise.
Optional gyro sample identities cannot be split/reused across the supported
production intervals; unsupported repetitions fail explicitly. Observation
identity and same-time pose aliasing remain separate. Rebase retires complete
old edges and their acquisitions, retaining separator information. The Pose2
budget counts cadence and acquisition states and is checked by the existing
30-second maintenance timer; transient growth between checks remains possible.

TF and local/map odometry represent consumed source time plus only the configured
lead, propagated by Expmap. Timer delivery lag is not silently added to that
prediction. Anchors compare graph and DR at the same source epoch and refresh on
unary-only optimizer revisions. Internal numeric recentering keeps the published
odom SE2 gauge continuous. Source reset and loss of a graph snapshot invalidate
cached map/fast output; the optional display publisher respects its existing
200 ms prediction limit instead of relabeling an old baseline indefinitely.

Additional causal controls expose and repair heading/initialization defects:

- An absolute heading at T combined with GNSS at S was stamped as S, then replay
  integrated S-to-T rotation again. The actual old initializer gives seed yaw
  0.3 rather than 0.1 rad, followed by current yaw 0.5 rather than 0.3 rad.
  Stamped heading seeds now retime through supported same-epoch DR history.
- Antenna position was used as the initial base position despite a nonzero lever
  arm. The actual old initializer gives (1.217390206, 2.052359318) rather than
  (1, 2) m at yaw 0.7. Initialization now subtracts the rotated lever at the
  correct source heading; the original GNSS antenna measurement stays intact.
- Dynamic COG/magnetometer consumers discarded their source stamps. A
  producer-feasible delayed COG causes 1.063925 mm XY error through legacy
  `QueueYaw`; the source-aware factor returns 1.37e-15 m on identical inputs.
- Three distinct old straight COGs delivered after a correct half-turn trigger
  a false 180-degree recovery: actual Node yaw changes from pi to 0.128810002,
  a 3.012782651 rad error. Recovery now compares the observed source pose and
  applies a demonstrated heading offset to the latest state, preserving the
  intervening turn. Supported checks are processed before their own factors;
  a future check cannot block mature checks. Existing recovery thresholds,
  debounce, consistency, finite prior and minimum interval remain unchanged.
  The ROS-time cooldown is invalidated only on an actual clock rewind.

Explicit stationary `clear_graph` retains only a heading prior derived at a
verified consumed boundary and accepted input baselines. It discards graph
likelihood/bias and position evidence, preserves GNSS identity high-water state
so a cached fix cannot undo the clear, and waits for a genuinely new position.
Real IMU gaps and clock rewinds still discard that carried prior. Clear observes
the clock before taking its exception; it cannot carry an old source across a
rewind. Calibrated dock seeding retries only while charging remains confirmed
and a consumed source exists; it does not persist a dock command after undock.
Pose-only checkpoint loading has no live source provenance: the new source path
requires a fresh absolute seed, while legacy Core Load/Tick remains compatible.

Final reviewed-source validation passes **14 CTest targets / 145 GTest cases**:
48 actual Node cases, 79 Core cases and 18 trajectory/conditional component
cases. All 64 source hashes match before/after the bounded build/tests. Actual
old-source counterfactuals fail both bootstrap cases (six assertions), five of
seven clear/dock cases, and all three final-review regressions. The two already
passing clear controls remain recorded. No acceptance threshold or tolerance
is changed.

A separate physical true-flip criterion is still **failed**: the new temporal
recovery and its identical-trajectory finite-prior control agree, but both leave
posterior yaw 0.598188373 rather than physical 0.285 rad (17.94 degrees error).
The pre-existing `ForceAnchor` adds another prior while keeping the disproven
old heading prior. Actual factor ablation now establishes that cause: full
batch optimization leaves the error unchanged; replacing only the initial yaw
contribution with an XY-only prior gives physical yaw 0.285 within 5e-16 rad,
with all other factors retained. Correcting only the initial yaw mean gives
the same result. The fixture uses the shipped 0.05 rad initial sigma (2.8648
degrees) versus 1-degree recovery sigma. After actual prefix marginalization,
the identifiable initial prior is gone but its wrong information and posterior
remain in a separator. Deleting only a visible prior cannot repair that case.
Relative-oracle agreement is not physical recovery proof.
The independent design review rejects sigma tuning as a repair: an explicit
current-source recovery cut would discard historical likelihood/correlation;
preserving it requires selective lineage reconstruction, including marginalized
prefix information. That recovery-policy choice is not silently made here.
The exact existing behavior, failed physical criterion, and proposed lifecycle
are retained in `true-flip-astra-decision.md` and the final Node log.

The ablation also exposes a separate snapshot defect: `ForceAnchor` changes the
solver but leaves the published covariance stale, and anchoring an older live
node does not refresh the latest pose/revision. The narrow repair refreshes the
latest pose and iSAM2 marginal together, preserving source/cadence timestamps.
Both new regressions fail against the untouched baseline Core; all 11 rebase
cases and all 48 actual Node cases pass with the repaired Core. This retains
the existing iSAM2 linearization/covariance model, rather than adding a full
batch marginal computation on every dock anchor. It does not fix the wrong
heading-prior lineage above.

The bounded first-cusp component connects the actual Node, its actual published
TF and the real FTC controller. Both declared GNSS delays (0/60 ms) pass the
unchanged y >= 0.8 criterion, with minima 0.814573880930/0.814573896669 m.
All 15 distinct nonseed fixes per variant apply (13 interior, two endpoint),
and all 58 nonzero returns match successful same-command checks. Both following
requests deny at cursor 37 and the fixture applies ideal immediate zero. Exact
source/anchor/TF/command/plant tuples and a 101-file frozen subject are retained.
This is an ideal SE2 plant with explicit initialization/geometry/footprint and
50 ms production TF lead, not Webots-native truth, a mux/actuator experiment or
a full mission. It does not clear the original boundary failure.

A further single 15-second **native Webots + real Node DDS/timer** profile
reveals a distinct simulator plant defect. There are 387 exact source-epoch
map/DR pairs: maxima 16.206979 mm / 35.548900 mm, while DR yaw matches native
truth to 1.8e-15 rad. At source 2.520 s, 13.137979 mm is already between raw
wheel pose and native chassis pose; another 3.069 mm separates raw wheel from
Node DR. Actual N1 wheel intervals agree internally to 4.42e-14, so changing
velocity averaging again is not the answer. Native chassis moves immediately
at firmware velocity 0.215 m/s, while the real wheel controller ramps at its
unchanged 1 m/s² acceleration limit (first measured velocity 0.0186 m/s).
Shared command input was feeding two different plants. The measured-native-wheel
chassis repair now passes 74 selected tests (17 new); the identical new tests
fail 16 cases against the old plugin. Independent review also runs all 17 new
cases successfully. Controller limits, actuator code and command watchdogs stay
unchanged. The observation plugin reports measured residual braking instead of
fabricating instantaneous body zero; stationary zero/unchanged-pose assertions
remain at a new supported native epoch. No external stop acceptance test or
numerical tolerance changes.

One identical 15-second native GREEN profile completes in 31.027 seconds with
the same approved Node/core binaries and byte-identical observer/profile/seed/
analysis. Map maximum falls from 16.206979 to **0.418496 mm**, DR from 35.548900
to **0.333758 mm**. All 922 native producer packets reconstruct their measured
wheel distance/turn/rates exactly; GT SE2 increments agree within 6.43e-16.
Across 751 exactly paired intervals, raw wheel and native body increments
agree to arithmetic precision under the predeclared +20 ms header-phase
hypothesis. Same-header phase remains distinct, and two consumer hold-value
exceptions remain retained. No new pose acceptance threshold, fitted lag or
blanket all-errors-fixed claim is introduced.

The single native first-cusp component is **RED** with unchanged y >= 0.8:
minimum transformed reference-point y is 0.794580169403 m, 5.419831 mm outside.
All four runtime FTC obstacle/deviation parameter/config checks are true; source
period is 40 ms and TF lead 100 ms as shipped by the full simulation launch.
There are 32 accepted commands followed by denial at cursor 37 / ROS 3.86 s.
Exact native reference-point y at denial is still 0.803333450372 m; it first
crosses at 3.90 s and reaches its minimum at 4.02 s. Native applied twist first
reaches zero at 4.04 s; 48 stationary truth samples are retained. No ideal stop,
threshold adjustment or acceptance rerun follows the failure. This reproduces
a containment gap after the command-only certificate denies further motion;
the intended 100 ms command sweep is explicitly not a stopping certificate.
This is a component, not attribution of the old mission or a hardware result.

The narrow forward-response repair now has an empirical native **GREEN**:
one unchanged first-cusp component keeps reference y >= 0.8 throughout actual
motion and braking, minimum **0.871303315602 m**. After 24 nonzero commands it
denies at cursor 35 / ROS 3.02 s: the intended-command predicate still permits
the request, while the new response predicate rejects it. Native y at denial
is 0.883093136805 m; actual twist reaches zero 180 ms later and 45 stationary
samples follow. All 24 nonzero returns have actual successful response checks;
every helper's measured input matches the exact same-executor compute argument.
Fourteen profile/seed/plan/plant/configuration inputs and baseline Node/Core
artifacts are unchanged. The separate covariance repair is intentionally
excluded from this native comparison and validated with the Node tests above.

The response gate reuses the existing 0.5 s assumed coast-integral bound and
shared lethal-cell envelope builder. It checks the full reference envelope
against the mowing union and the body against raw holes, preserving legal
outer overhang, unconditional STOP and existing reverse/rotation contracts.
Geometry applies independently of obstacle/deviation flags. The old compiled
FTC fails both new expected-denial controls; **106/106 real-controller cases**
and **27/27 optimized critic cases** pass. Three unchanged 100 ms timing checks
fail in the retained unoptimized build, then pass at 5.39/32.72/12.73 ms with
optimized hot-source objects; no threshold or assertion changes. Independent
review finds no blocker in the response code. Native comparison build/run take
92.864/19.023 seconds under explicit hard/progress bounds.

This closes the demonstrated forward braking-tail gap in this component under
the existing response assumption. It stops before completing the sharp cusp;
it does not establish route completion, a calibrated physical response bound,
full controller-server velocity-cache qualification or original-event attribution.

The native run uses known stationary truth for explicit seed, zero TF lead,
no COG stimulus and the unchanged approved Node/core binaries. It completes in
31.028 seconds, with verified cleanup. Three pre-Node observer startup failures
are retained. Serialized CDR is reserialization of received decoded messages,
not original DDS wire bytes; metadata repr is not decoded publisher identity.
Fixed hold/phase replays identify compatible value families, not actual cached
wheel-message IDs or proof of callback ownership. Full results and exact source
windows are in `native-source-component/`; no second baseline profile was run.

Latest fetched `main` is **5ba9ad46** and `dev` is **112be1eb**. Since the
previous 95ab1b6d check, #1043 adds route-aware transit deadlines and credits
partial progress inside long unfinished coverage units; #1047 fixes the GUI
flash dialog. A separate low-battery docking-attempt change is reverted on dev.
The earlier #1041 dock-neighbour/fleet-mark and #1042 settings-save repairs are
also present. These do not replace the source-time, measured-wheel,
forward-response or snapshot repairs, nor remove the planner cusp. The new
behavior changes overlap the broader branch's coverage consumer and would need
their own integration validation before adopting latest dev. No base update is
performed; local evidence retains its exact 5d3845b1 integration baseline. The original capture still lacks accepted graph-node identities, optimizer/anchor revision history
and calibrated physical acquisition timing. Its retrospective seed-error
associations remain hypotheses. The new component retains actual accepted
command/response/consumed-pose tuples and native braking, plus source-registry
snapshots; it cannot recover missing optimizer history from the old mission.
Another uninstrumented E2E success cannot establish that attribution.

Evidence under `~/.cache/pr950-resume-20261029/`: `source-core/`,
`source-core-yaw/`, `source-core-query/`, `yaw-epoch/`, `yaw-repair/`,
`helper-reviewed/`, `source-epoch-node/`, `bootstrap-counterfactual-red/`,
`clear-counterfactual-red/`, `final-review-counterfactual-red/`,
`cog-flip-delay/`, `cusp-source-component/`, `native-source-component/`,
`native-source-component-green/`, `native-cusp-component/`, `wheel-plant-repair/`,
`true-flip-physical-oracle/`, `flip-prior-ablation/`, `anchor-snapshot-repair/`,
`geometry-response-repair/`, `native-cusp-response-green/` and
independent review reports. All earlier outputs remain preserved.

### Independently useful repair families

This is an assessment only; no extraction is performed and #950 stays Draft.
Each extracted result would need its own target-base focused validation rather
than inheriting a whole-branch CI result.

| Family | Dependency and tested limit |
| --- | --- |
| Wheel N1 interval configuration | Standalone simulator configuration plus real-upstream odometry/YAML test; retained Webots A/B. It does not repair source timestamps or the boundary event. |
| Native simulator observation identity | Keep applied-state producer, synthetic IMU/GNSS/status consumers and launch/test wiring together. 54 regressions and exact live component pairs; actuation dynamics remain separate. |
| Measured-wheel simulator body | Plugin, URDF contract and regressions belong together, depending on native GT/applied-IMU observation binding. 74 selected cases and native RED/GREEN support one wheel/motion authority; actual braking is reported, not a new physical-stop qualification. |
| Motion-input finite/order protection | The input guards and actual Node fixture can stand independently of GNSS retiming; 13 original Node cases passed. Later source-history consumers depend on these guards. |
| Rebase information and absolute gyro bias | Two independently causal Core repairs, with their tests/lifecycle pieces; source acquisition retention depends on information-preserving typed rebase. 52 focused cases passed before source integration. |
| COG producer source context | Producer/history/tests form a separate repair from GraphManager. Its pre-change producer, timing/math helpers and package CMake match latest fetched dev byte-for-byte; direct base application is plausible, but any extracted PR still needs its own validation. Retain delayed heading, completed-pivot latch and clock-epoch cases together. |
| Wheel travel-noise accounting | Four small production edits plus source/legacy/clipped-bootstrap regressions; all three new cases RED on the old Core, 15 selected targets / 156 cases GREEN after coherent ABI rebuild. Can be separated with its bootstrap source integration dependency retained; it does not repair true-flip lineage. |
| Post-anchor snapshot | Small Core/API repair with two baseline RED cases, 11 rebase and 48 Node GREEN cases; depends on existing source/revision semantics. It does not repair the invalid true-flip prior. |
| Automatic dock seed provenance | Keep the shared charging guard, callback ordering and timer fallback together. Two old-Node RED cases, two charging controls and 52 actual-Node GREEN cases support the narrow repair; general bad authoritative seed recovery remains separate. |
| FollowStrip result ownership | Two per-dispatch slot renewals and their real-class/fake-Nav2 regressions form a small separate repair. Two REDs and seven focused GREEN lifecycle cases; no dependency on the unshipped maneuver. It does not fix other BT nodes' outcome ownership or establish downstream cancellation. |
| FTC requested goal and consumer epoch | Keep the requested-distance approach, current terminal decision, goal transform provenance and final guard epoch repairs together. They depend on the selected Nav2 1.5.2 path-handler contract and existing geometry/response interfaces; extraction needs its own target-base validation. Software RED/GREEN does not establish a complete corner. |
| Public coverage header includes | Include-only dependency reduction has unchanged coverage/replay evidence; it is separable from the unshipped constructive-corner prototype. |
| GNSS/heading/source TF and recovery | Keep coherent consumed means, conditional states, source unaries, source publication/anchor, bootstrap and recovery lifecycle together. Individual API groundwork is not an independently exercised runtime repair. Frozen first-order and physical-provenance limits remain explicit. |
| FTC forward geometry response | Depends on existing raw geometry authorization and the assumed coast contract; shared envelope plus gate/tests belong together. 106 controller and 27 critic cases, plus one native containment GREEN. It stops before the cusp; route completion and calibrated physical containment remain unqualified. |
| FTC fixture startup and replay CI setup | Independently tested fixture/TF-probe and build-infrastructure repairs. They do not authorize motion or fix the original event. |

Dock dispatch, fresh detour acquisition and planner bevel repairs remain prior
standalone candidates with their existing software evidence. Keep each package's
source, dependencies and test wiring together; planner geometry changes still
need their own behavior review. None inherits physical containment or a completed
mission from these component results.

## Graph information and preintegration repairs (2026-10-29)

The active investigation continues independently of CI. The original boundary
capture still does not expose its accepted graph-node identities: a bounded
raw audit found ten fix/status pairs near the event but no graph diagnostics
or target-node records. No original-event attribution is inferred from receipt
time or from a later proxy node.

Two more production causes are reproduced and repaired:

- `RebaseISAM2` replaced all retained process factors with independent 5 cm
  priors. In the regression, an older correct GNSS update moved the unre-based
  current pose to 0.659706745 m, while the rebased graph stayed at 0.650000000 m.
  The rebase also attempted to cast scalar bias states to Pose2 outside its
  exception guard. It now retains nonlinear tail factors and typed values,
  marginalizes only the removed prefix into a Gaussian separator, and protects
  asynchronous worker ownership against cancellation/new-worker ABA. Pending
  data targeting a removed state abandons the rebase while keeping the already
  updated live graph. Nine new tests and seven existing focused cases pass;
  joint information is compared at the same linearization point. Removed
  history retains the standard fixed-lag linearization approximation.
- Optional gyro preintegration subtracted the prediction bias from its integral
  and then subtracted the absolute graph bias again. Four actual-core cases
  gave 0.18/0.23 rad instead of 0.20 rad after learning positive/negative bias.
  The factor now receives the raw integral and applies the absolute bias once;
  pose prediction retains its corrected rate. Pose-only reanchors/restores also
  need a valid bias predecessor before extending the random walk. Checkpoints
  now serialize canonical trajectory poses, matching the existing restore
  contract; transient scalar bias serialization had failed as an unregistered
  type. Historical bias is re-estimated against the new session's calibration.
  Reset also clears the prediction bias: a reused Reset+Load instance previously
  advanced 0.18 rad where a fresh load advanced 0.20 rad. The nonzero-bias
  regression now passes with identical behavior.

The representative rebase check builds 4,000 nodes with and without optional
preintegration, retains a 2,000-node window and advances another 20 nodes.
Both stay initialized with finite results and no printed mean shift. Rebase
wall times are 56/133 ms in this bounded test, peak child RSS about 51 MiB;
these are software measurements, not a robot throughput qualification.

Fresh core-only validation passes **9 CTest targets / 52 GTest cases**,
including seven preintegration/bias lifecycle cases and both persistence suites.
Local normal-node integration validation did
not complete: repeated bounded builds stopped in the unchanged communications
translation unit without a compiler diagnostic. Its flags match the prior
successful build; measured container peak was about 2.13 GiB with no cgroup
OOM, while the host experienced severe memory pressure. No larger blind
retry, compiler/configuration weakening or robot run is substituted for that
missing evidence. Hosted checks remain separate and do not pause this goal.

The next GNSS repair is now specified: a frozen first-order normalized
conditional trajectory bridge. Held wheel inputs are conditioned controls;
existing distance/fault sigmas describe process diffusion. A globally exact
raw-wheel measurement model is not a prerequisite. Multiple acquisition states
must share their joint conditional law; optional gyro sample error uses shared
identities within an edge. This supersedes the earlier implication that new
physical wheel-noise calibration is needed before software implementation.
It does not claim physically calibrated/conservative uncertainty or exact
sensor preintegration. Source-epoch/mean coherence is still required before
production integration; covariance alone cannot fix a falsely stamped pose.

The new source-history and conditional-bridge components have focused GTSAM
coverage and remain under integration. They are not treated as a production
GNSS repair or as clearance of the 14.814 mm phase discriminator. All existing
motion, freshness, geometry and acceptance tolerances remain unchanged.

Evidence: `~/.cache/pr950-resume-20261029/rebase-information/`,
`gyro-bias-preint/`, `core-final-reviewed/`, `core-repairs-validation/`, `source-history/`,
`conditional-bridge/`, `original-gnss-inventory/` and
`gnss-model-decision-astra.md`. Earlier reports and build outputs are retained.

## Motion-source input repair and GNSS covariance decision (2026-10-28)

The original boundary event and the 14.814 mm GNSS/node-epoch mechanism remain
open. A separate production-node discriminator now establishes and repairs
additional deterministic input faults without changing motion tolerances:

- Ordered input integrates 6 mm. Receiving IMU10.01 after IMU10.02 previously
  reopened the interval, so IMU10.04 produced 7.5 mm instead of 6 mm.
- A replayed wheel sample could replace the current cached rate before its
  timestamp was checked. The retained control moves −3 mm instead of +6 mm.
  Duplicate timestamps with altered payload could replace the cache too.
- A nonfinite gyro reaches Sophus and aborts the actual node (exit134);
  nonfinite wheel velocity makes DR coordinates nonfinite.

The node now checks finite used values, canonical nonnegative source stamps
and strictly increasing source identity before mutating cached rates,
integration baselines or graph inputs. Zero remains a valid simulation-clock
bootstrap epoch. A shared motion-clock barrier distinguishes an actual ROS
clock rewind from a late source message and invalidates both stream baselines
and cached rates together. An IMU-first rewind regression caught +3 mm of
old-wheel motion in the initial per-stream candidate; the final shared barrier
removes it while preserving a fresh wheel-first observation.

The registered `test_sensor_input_node` compiles the actual constructor and
callbacks, with an explicit controlled ROS clock and no executor-driven inputs.
Eight implementation translation units are shared with the production binary;
only the constructor/entry TU is compiled separately with the test entry
renamed. **All13 node cases pass** in a fresh build; the normal production
entry also links. The preceding focused build additionally passed GNSS timestamp,
GNSS freshness, factor and LiDAR-history suites (5/5 CTest targets total).
Changed C++ passes clang-format18; independent final review has no blocker.
This is input-state protection, not a full graph/reset, future-packet,
TF-cache, missing-interval or physical-containment guarantee.

Astra and independent Sol analysis reject a tempting GNSS unary-offset repair:
its motion noise is shared with existing process factors. An external actual
GTSAM prototype now passes **51 algebraic assertions** against independent
unique-source latent and Eigen references. Shared conditional acquisition
states preserve endpoint information before GNSS, joint means/covariances,
reversed insertion order and duplicate identity. A single noisy rate sample
split between two acquisitions retains its shared error, including an exactly
zero conditional variance case. The deliberately independent-retiming control
fails covariance equality. These are linear-Gaussian model tests, not calibrated
physical covariance or an implemented production GNSS repair.

The remaining causal path needs ordered source-interval history, SE2/bias
covariance transport and preservation of the active correlated tail across
rebase. Adding an ordinary cadence node at GNSS time is not a shortcut: it
consumes accumulators and changes slip/stationary/noise state. No such cadence,
GNSS-quality, prediction or safety-policy change is applied here.

Evidence is retained under `~/.cache/pr950-resume-20261028/gnss-repair/`:
`input-order-red/`, `epoch-cache-red/`, `input-order-green-final/` and
`final-motion-review.md` pin the repaired callback cases; `design-astra.md`,
`design-sol.md` and `conditional-prototype/REPORT.md` pin the GNSS decision.
Earlier reports, raw captures and build outputs remain preserved. No hardware
or full E2E run was performed.

## Seed error and simulator observation identity (2026-10-27)

The original **13.943731 mm** boundary failure remains open. Fetched `main`
and `dev` remain at `fe77a097` and `5d3845b1`; neither contains these repairs.
New evidence separates three mechanisms instead of assigning the residual to
prediction alone:

- In the retained call819 reconstruction, the joint N1/source-time control
  still leaves **+13.836143 mm** declared-time y error. Under two explicit
  truth-time associations, **+12.207160 mm / +14.558176 mm** is already in
  the actual published map seed. The post-seed DR contribution is only
  −0.022095 / −0.012954 mm. Original truth headers were wall time and no
  native step or `/clock` was retained; these partitions remain conditional.
- Reconstructing the longer pre-seed wheel history reproduces all 27 retained
  local states to 2.22e-16. Replacing N10 with N1 while freezing the original
  anchor makes the y error **6.210057 mm worse**. Original graph anchors were
  optimized against the original filtered inputs; this control cannot identify
  the reoptimized N1 result. The final captured anchor change lowers the
  selected seed by 0.492927 mm; earlier upward changes dominate. A calibrated
  GPS lever/yaw-only explanation has the wrong sign or insufficient magnitude.
- A fresh build of all production FusionGraphNode/core translation units and
  real GTSAM reproduces a distinct epoch-association defect with perfect data:
  straight motion at 0.15 m/s after rest, accurate GNSS at **1.24 s** attached
  to node **1.14 s**, produces **+14.814123 mm** map error at the next node.
  No-fix and exact-node controls stay near zero; an exact known-displacement
  oracle also removes the error. This proves that mechanism, not its share of
  the original event. Straight two-node time interpolation is unsafe: the
  throttled rest-to-start gap would invent 18.421053 mm motion at the actual
  stationary epoch. No such graph repair or observation dropping is applied.

A native-step Webots diagnostic also establishes an application-owned source
identity defect: the private plugin clock stamps truth with wall time, the
GNSS relay projects that truth but copies another raw GPS epoch, and synthetic
IMU pairs a cached command value with an independently delivered raw IMU epoch.
The narrow repair now stamps truth directly from native Webots time and emits
`/sim/applied_twist` for the exact interval ending at that same truth pose.
Synthetic IMU consumes that observation's value and epoch; generated planar
GNSS coordinates retain the selected truth epoch and antenna frame, paired
exactly with typed status and its lifetime sequence.

Clock eligibility never rewrites acquisition time. Invalid/future inputs cannot
advance accepted authority; rewind clears pending/accepted identity. The IMU
keeps ten ordered pending samples, matching DDS depth, and drains on clock
catch-up even without another input. Independent review found and reproduced
loss of a final timeout-zero in the first single-slot candidate; the ordered
queue fixes that RED case. Overflow deliberately drops oldest samples; this
is not a lossless delivery or arbitrary cross-generation replay guarantee.
Raw driver header phase and production TF/graph association remain separate.

One 15-second rest/straight/pivot/arc/stop minimal Webots profile completes in
30 seconds wall time under 180-second hard/45-second progress bounds.
All **766 captured truth/applied/IMU samples** pair exactly by native epoch;
gyro z equals that interval's applied yaw rate. Across 765 complete intervals,
GT yaw-increment residual is at most 5.45e-16 rad. All **73 GNSS fixes/statuses**
pair exactly with their selected truth; sequence is 1–73 and maximum antenna
projection error is 3.4e-8 m. No captured applied-to-IMU or raw-to-GNSS count
loss occurs. `/clock` is contiguous at 20 ms; eight stamps round 1 ns below
the native acquisition epoch, handled without changing observation headers.
This uses the pinned image's existing controllers plus source-hashed current
plugin/relays and current generated interfaces, UID1000 with network disabled;
it contains no FusionGraph, Nav2 or BT. During pivot, 18 zero-applied
samples are already present in the plant's consumed target; they are not hidden
by sensor synthesis. The known controller teardown segfault after SIGINT is
retained. An initial shell-setup failure before Webots startup is also retained.

Validation: **54 focused pytest cases pass**, including actual plugin step and
sensor callbacks, interval yaw transitions, replay, future ordering and rewind.
Retained old-source tests fail the native GT/selected-GT timestamp contracts
and applied-IMU source selection. The final-zero discriminator is RED on the
single-slot candidate and GREEN with the final queue. Five sensor/test files
pass ament_flake8; whole-file lint on the plugin/launch retains the same 185
pre-existing diagnostics as their exact HEAD baseline. No C++ changes are made.

Evidence is retained outside Git under
`~/.cache/pr950-resume-20261026/root-cause/`: `residual/` contains original-row
arithmetic and hashes; `real-node/logs/association.jsonl` is the actual-node
GNSS discriminator; `webots-phase/` contains the native diagnostic;
`applied-imu/` and `provenance-*.log` contain RED/GREEN and lint evidence;
`app-provenance-live/attempt-02/summary.{md,json}` links every component oracle
to the retained raw rows and source hashes.
The source snapshot is defined by retained hashes, not the image's old build.
No hardware or full mission acceptance is claimed.

The next software discriminator is a phase sweep of real production-node
GNSS against **integrated acquisition history**, including rest-to-start,
curves, delayed delivery, node throttling and reset. It must retain the actual
state epoch, wheel/gyro identities, graph node/optimizer snapshot and anchor
revision together. A correct repair must preserve valid off-node observations
without inventing rest-period motion, treating uncertain odometry as a known
lever arm, or increasing prediction/freshness tolerances. Once that local
contract passes, a bounded first-cusp replay with native truth can measure
its contribution to the original residual. More uninstrumented E2E runs
cannot settle that attribution.

The new native truth/applied-twist/IMU/GNSS wiring is one cohesive simulator
repair family, independently useful from motion authorization and N1 wheel
filtering. Its source and consumers must move together. Previously tested
wheel, FTC-fixture and CI families remain candidates for smaller PRs;
none is extracted here or inherits a boundary-containment pass.

## Producer-time contract and remaining blocker (2026-10-26)

All five required hosted checks pass on `651508c6`; the PR remains Draft.
The simulator wheel-interval repair and FTC fixture-startup repair are separate
from the original **13.943731 mm** boundary failure, which remains open.

A fresh, bounded component build extracts the current `OnImu` and
`TfBroadcastLoop` bodies unchanged. Their method/helper hashes still match the
earlier producer seam. The 18 normal timing cases again reproduce eight
timestamp/value mismatches; all emitted ages lie inside FTC's existing age
window. The mathematical timing control passes 18/18 but predicts through the
input lag; that result does not authorize a longer production horizon.

Eleven additional transition controls and two between-leg insertion controls
exercise the unchanged callbacks/publication method, real ROS clock, real TF
buffer and Nav2 latest-odom-pose-to-map query. They establish why neither
`now + lead` nor a substitution with `last_imu_stamp_ + lead` is a complete repair:

- First and rejected long-interval IMU callbacks advance the received timestamp
  without integrating pose. An out-of-order input from 10.0 to 9.99 seconds
  also rolls that baseline back; the next 10.01 input integrates 20 ms although
  only 10 ms is new. This is a controlled callback result, not evidence that
  the captured boundary event contained an out-of-order IMU.
- With integrated state at 10.02 seconds and clock at 11.02, production emits
  stamp 11.12 (age -0.1 seconds). A bounded source-plus-lead control emits
  stamp 10.12 (age 0.9 seconds). Both latest-time lookups succeed. FTC's existing
  age check would reject the latter by arithmetic; full FTC authorization was
  not executed in this producer seam.
- At unchanged stamp 10.12, an anchor revision changes the returned map x from
  1.024 to 1.074 m. Equal timestamps therefore do not establish unchanged state.
  A coordinated rebase preserves 1.074 m after both TF legs are inserted, but
  an explicit query between legs returns 0.070 or 2.078 m depending on order,
  with that same stamp. This is a controlled cache interleaving, not an
  observed production race or cause of the boundary event. Repeated ROS time
  can expose the same issue with the current publisher too.
- Reset-shaped field assignments retain old prediction rates and produce
  approximately 19.964 mm / 0.06 rad from a zeroed pose. The callback rewind
  control also carries state across clock generations. These controls do not
  invoke the real graph reset or rebase paths.

The graph/LiDAR sinks are inert and Pose2 is a stand-in; this evidence does not
partition optimizer history or validate a closed-loop repair. Independent
review confirms those limits. Each isolated build/run uses UID1000, the pinned
image `0fc53b7941fa`, no network/devices, 2 CPUs/2 GiB and a 180-second outer
deadline (60/90/30-second configure/build/execution stages). Source, control,
output and binary hashes are retained under
`~/.cache/pr950-resume-20261026/tf-time/seam/source-manifest.json`;
the inventory and independent review are in its parent directory.

### Exact remaining software questions

| Unknown | Next discriminating evidence |
| --- | --- |
| Which source intervals and wheel cache produced each accepted pose? | Record callback sequence/source stamp, accepted IMU interval, consumed wheel identity and the atomic DR pose/rate/source snapshot in the real producer. Received-input stamps alone are insufficient. |
| Which graph/anchor history accounts for the remaining +13.84 mm declared-time error? | Join graph-node creation inputs, optimizer snapshot and anchor revision to that DR snapshot, acquired truth and the accepted FTC tuple. The captured local seed does not recover that earlier history. |
| Which source-aligned publication survives reset, rewind and anchor changes? | Track integrated-state lineage separately from anchor revision; test real reset paths and consumer queries across both TF legs. Keep prediction bounded to the configured lead and retain existing freshness/motion assertions. |

No localization production change is selected from these controls. The next
run is a bounded producer/first-cusp capture with those identities and complete
signal provenance, using a fresh matching diagnostic build; former mission
overlays are unavailable. Another uninstrumented full mission would not resolve
these unknowns. Hardware is not required for this software discriminator.

### Smaller repair families, without extraction

The separately committed simulator one-sample repair (`1715574b`) is the
clearest standalone candidate: its upstream regression and short Webots
RED/GREEN qualify its observation contract, not mission containment. Replay-CI
dependency/mirror repairs are also separate from motion policy. The FTC fixture
startup barrier and exact TF-probe registration belong together; their tests
do not repair the upstream listener implementation. Dock-dispatch and fresh
detour/progress repairs have focused merged-dev tests but overlap newer dev
behavior and need their own scoped review. Shared geometry/sweep/escape work
depends on its message/producer substrate; the bevel repair depends on its
certification/replay evidence. None inherits a containment pass from green CI,
and no component has been extracted or marked ready.

## Simulator wheel-interval repair (2026-10-26)

Compared with fetched `main` `fe77a097` and `dev` `5d3845b1`, the PR's
dock-dispatch, fresh-detour-snapshot, simulator-geometry and bounded-bevel
repairs remain distinct. The investigation checkout fast-forwarded to PR head
`0caa2013`, including its dock-body execution exclusion, then integrated the
current `dev` at `5d3845b1`. Upstream #1008's firmware acquisition-time repair
is included; the physical USB/STM32 path is separate from the failed Webots
execution. No hardware result follows from that source integration.

The Webots diff-drive configuration now explicitly uses a one-sample velocity
window. The default ten-sample trailing mean paired a current wheel-odometry
pose/header with older velocity intervals. Fusion Graph consumes that velocity
as its current input, so the mean introduced an additional input delay during
acceleration, braking and turns. This repair retains joint-derived odometry
and the existing actuation model.

The new registered `test_wheel_odom_interval` drives the actual upstream
Odometry implementation using the shipped YAML, independently recovers body
Twist from pose increments, and compares both at the unchanged `1e-10`
tolerance. Before the repair, its configured case fails with 25 mismatched
intervals while the explicit ten-sample negative control passes. After the
repair both cases pass. The simulation package builds and installs; all four
focused CTest targets pass, including the firmware wheel-model, ground-truth
publisher and controller-remap checks.

Two new minimal Webots runs exercise rest, straight motion, pivot, arc and
stop with the same probe and oracle. The default-window run fails over 759
intervals (maximum forward-speed mismatch `0.08649 m/s`); the repaired run
passes over 758 intervals (maximum linear/angular mismatches `5.28e-15 m/s`
and `3.18e-14 rad/s`). Both finish their observations in approximately
30 seconds under independent progress and hard-deadline watchdogs. Both
retain the known driver segmentation fault during teardown; neither is a
clean-shutdown result. Execution uses UID1000 and the pinned simulation image
`0fc53b7941fa`, with current source launch/configuration and the image's
unchanged actuation source. This is fresh component evidence, not a rerun
using the unavailable older built overlay or a completed mowing mission.

Independent review found no blocking issue in this narrow repair. The
original boundary failure remains open: one-sample wheel velocity alone
does not explain the previously retained call819, its residual predicted-pose
error or earlier graph/anchor history. Source-time TF provenance remains a
separate producer defect. Geometry tolerances, safety horizons and acceptance
assertions remain unchanged; this simulator repair does not certify physical
behavior.


### Latest-dev integration

The three conflict files are reconciled with independent review. Docking
retains both the newer already-charging success paths and the PR's per-attempt
result ownership/cancellation draining. The imported cancellation regression
now also asserts that a successor is not dispatched before the old result is
drained; its successor-result and success assertions remain intact.

Corridor records retain dev's transactional preflight, strict unsigned IDs and
name normalization. A preflight failure preserves the known live area and
corridor records, while the existing planning-grid error gate keeps execution
geometry revoked across mask rebuilds. An error after area replacement starts
still discards partial state. A new integration regression verifies retained
live map data, revoked permission and recovery after a corrected reload.

Current interfaces were generated and installed from the merged source before
dependent builds. Both map targets, all four selected docking/coverage targets
and all five selected simulation targets pass (**11/11 CTest targets**).
The behavior group takes 111.35 seconds in total, within the 120-second per-test
and 240-second group bounds. This qualifies the focused merge seams; it does
not replace whole-stack CI or a source-aligned localization/containment run.

## Targeted first-cusp investigation (2026-10-21)

The original GPS01 **13.943731 mm** active-MOWING departure remains a failed
acceptance result. A new, bounded startup prefix reproduces the same failure
mode with **12.328622 mm** departure. It captures the previously missing
accepted FTC checks, actual returns, raw IMU, post-slip wheel odometry, TF,
ground truth and command lanes. This is one targeted reproduction, not a full
mowing/docking/charging mission or an exact replay of the old execution.

The checkout is `0af8f4f3`, with motion source `f44daf90`. The latest inspected
dev `708ef0f4` changes only firmware build configuration and the GUI Dockerfile
since the incorporated `da1494f1`; it adds no controller/localizer/simulator
change. All five required hosted checks passed on `0af8f4f3`, including all
three replay configurations. No product source, safety assertion, tolerance,
acceptance deadline or expected result changes in this investigation. External
diagnostic fixtures and a separate bounded capture deadline are identified as
such; they do not replace the unchanged mission acceptance criteria.

The run uses the previously qualified diagnostic-only FTC plugin, with its
matching dependencies, in the pinned ROS/Webots image. UID 1000, no network or
physical devices, read-only source/install/helper mounts, a 300-second outer
deadline and event-triggered cleanup bound the run. Actual plugin mapping and
hash, unchanged source/helper hashes, container exit 0 and owned-container
removal are recorded. The passive raw observer first passes an isolated real
message/header/footer smoke test; it retains received CDR rather than the old
action UUID projection. The older failed action-status comparison is not
declared repaired or needed to infer an unrecorded action UUID.

### Captured causal chain

Every position and quaternion of the new 9,420-pose full plan and 9,268-pose
event goal equals the respective historical captured path. The approximately
151.9136-degree reversal at event indices 37/38 is identical. This is the same
path/cusp input, with a different runtime sensor and command history.

The captured geometry and runtime footprint are replayed through the unchanged
native `Snapshot::permits`. All **808 retained checks** reproduce their actual
verdicts. All 821 retained call records individually validate: 792 nonzero
returns, 26 zero returns and three exceptions. **The trace is globally
INCOMPLETE**: shutdown leaves no footer and an `.incomplete` marker. The
unchanged whole-trace validator correctly exits 2. Individual complete records
and positive command matches do not prove absence of unrecorded trailing calls.
The separate raw observer has a footer with zero decoding errors.

| Call | Controller ROS time | Consumed map TF stamp / y | Actual decision |
|---|---:|---|---|
| 819 | 164.48 s | 164.58 s / 0.825296981982 m | Permits and returns v=0.15 m/s, w=-0.552295465825 rad/s, horizon 0.1 s |
| 820 | 164.58 s | 164.64 s / 0.816343690185 m | Permits and returns v=0.15 m/s, w=-0.613459508556 rad/s, horizon 0.1 s |
| 821 | 164.68 s | 164.76 s / 0.764642172099 m | Denies the requested rotation; throws without returning a command |

For calls 817–820, the exact source stamp and all six velocity components have
one matching message on each of `/cmd_vel_nav`, `/cmd_vel_monitored` and
`/cmd_vel`. The trace's `base_link` frame is explicitly rewritten to
`base_footprint` on publication; downstream messages agree with each other.
This is observed payload linkage, not an action lease or publisher ownership
claim from Cyclone's callback handle.

Ground truth acquired approximately 0.321 ms before call 819 starts is
`(2.5018545376021035, 0.8136933586739398, -1.3322143324865787)`.
Empirical host wall/monotonic alignment places its acquisition before that
call and the next simulator update after the call exits; the kinematic plant
holds this state between steps. This is stronger than selecting the nearest
receipt, but is not a physical clock-calibration guarantee. Substituting this
pose into the **same production predicate**, with the same captured geometry,
footprint and request, **rejects**: the nominal reference-point endpoint is
`y=0.799027790317`. The preceding sample permits and the following sample
rejects, so timing qualification matters. The actual consumed pose permits.

The first crossing, `y=0.799975848595`, is already received **during call 820,
before it returns or publishes**. Actual plant increments identify a retained
wheel payload from displacement and yaw; 46 consecutive increments match
recorded wheel velocities, with maximum transverse residual below 2.2e-16 m.
Travel from call 819's current state through first crossing is 0.0140145 m,
below the nominal 0.015 m command distance. The maximum departure,
`y=0.787671378399`, is already received before call 821 exits. Therefore neither
excess travel nor a nonzero hold after denial is needed to explain this event.
Immediate output at 164.68 s is zero on all three observed command lanes. A
later 164.8 s recovery command rotates at -0.25 rad/s; it is not an FTC return.
No denied rotation is treated as a returned command.

Earlier upward map/odom-anchor changes re-enable forward motion. Holding the
previous anchor fixed for call 817 changes its same-request predicate result
from permit to deny. The final downward anchor correction is approximately
32.774 mm, but is not necessary for call 821's rejection: the same odom pose
under call 820's anchor already rejects. Moving RTK-Fixed fixes have exact
payload ancestry in retained ground truth; a fabricated GPS displacement is
not needed. These controls isolate input-pose/anchor effects without changing
production motion policy.

### Recorded input mechanism and focused controls

Offline reconstruction from recorded local-odom seeds, actual IMU payloads and
wheel payloads reproduces calls 819–821's odom TF to floating-point precision
(x/y residual zero; maximum yaw residual 2.3e-16 rad). The matching wheel-cache
identities are inferred from that arithmetic, not acknowledged FusionGraph
callback order. An independent bounded search allows omitted IMU callbacks,
alternative seed source epochs and nearby wheel caches; it finds exactly one
matching sequence for each call within that explicit family. This strengthens
the inference without proving unrestricted callback ownership or ordering.
Each call uses its captured map/odom anchor unchanged.

Two input effects are demonstrated:

- Published wheel twist is a trailing ten-sample mean of instantaneous raw
  wheel-pose increments: 116 samples give forward-speed RMSE 1.6e-15 m/s.
  For call 819, the identified cached speed is 0.04557 m/s, versus instantaneous
  0.123225 m/s for that same wheel interval. Post-slip and raw twist agree in
  the critical window; the sampled DR slip veto is false. All 20 actual IMU
  samples at 164.00–164.38 s exceed its gyro-stillness threshold, excluding that
  veto as the cause of the earlier upward-anchor steps. After stopping, the
  veto correctly suppresses the filtered wheel-speed tail; disabling it is not
  a proposed repair.
- The reconstructed integrated IMU epochs are 164.46/164.52/164.64 s, while
  the consumed TF stamps are 164.58/164.64/164.76 s. Production predicts only
  the configured 100 ms although these state-to-stamp intervals are 120 ms.
  This is a 20 ms producer-time mismatch, distinct from the 20/60/40 ms state
  ages at controller entry. A fresh or future TF header does not establish
  the acquisition time of its represented state.

The following controls use the **unchanged production predicate**, captured
geometry/footprint/request and the same recorded local-state seed and anchor:

| Local reconstruction | Call 819 | Call 820 |
|---|---|---|
| Captured N10 speed, 100 ms prediction | Permit | Permit |
| Instantaneous wheel speed in post-seed DR steps and prediction | Permit | Deny |
| Source-aligned 120 ms prediction, captured speed | Permit | Deny |
| Both local changes | Deny | Deny |

For call 819 the joint control moves predicted map y from 0.825296981982 to
0.813811991578 m and changes permission to denial. Neither effect alone
changes that call's verdict. This supplies a concrete joint input mechanism;
it is **not** a reoptimized graph, a full repaired controller/plant run or proof
that either prospective product change alone repairs the mission. Earlier
controller controls also show that N10 filtering alone does not cross under
every history/TF phase: all six steady-moving-history controls pass.
The joint control still differs by approximately +13.84 mm from ground truth
observed at ROS time 164.58 s, the TF's declared time. Flipping permission does
not fully explain that declared-time pose error or make the pose accurate;
earlier state/anchor history and constant-velocity prediction remain material.

Separate minimal Webots world/driver/odometry controls confirm that explicitly
setting the external `velocity_rolling_window_size` to 1 restores same-interval
pose/twist agreement under the same 1e-10 oracle; the default N10 fails it.
No repository configuration was changed. An actual-method producer seam also
demonstrates the source-time defect with controlled delayed IMU: the original
timestamp/value contract fails 8/18 cases, while a source-aligned mathematical
control passes 18/18. That control does not authorize unbounded extrapolation
or establish safe stale-input behavior.

### Scope and remaining evidence

For this reproduction, the unchanged guard evaluates its captured inputs
consistently; the consumed pose overstates clearance, and a true-pose
substitution rejects the offending command. This does not exonerate every FTC
path or recover the missing accepted tuple from historical GPS01. It does not
justify a guessed margin or a relaxation of truth containment. The demonstrated
input-filter/time effects are upstream of FTC; changing localizer provenance or
the broader motion-authorization contract exceeds this PR's requested scope.
The original active-FTC acceptance failure remains open, rather than silently
being reclassified as resolved #924 work. This investigation adds no RPP,
output or localization policy, and selects no unverified production repair.

The next discriminating software experiment is a bounded producer trace with
actual callback/source identities, atomic DR state/velocity snapshots and
graph-node/anchor creation context, joined to the accepted FTC certificate.
That distinguishes remaining graph-history/anchor and executor-order effects
from the demonstrated local input mechanism before choosing a complete repair.
The recorded-seed controls cannot partition the optimizer's earlier history or
prove the prospective fixes' closed-loop behavior. Further uninstrumented full
missions would not answer that question. Hardware is a separate outstanding
acceptance requirement, not the blocker to this software experiment.

Evidence is retained outside Git under
`~/.cache/pr950-causal-20261021/prefix-run-2e748c5c52c8/`, including reproducible
analyses in `analysis/{ftc,chain,lead,wheel}`. Raw CDR SHA-256 is
`5dbd7688f5a1829a4e89ebb1e3e11dc566e14aa5114ac55c5a6b9324ea7be651`;
FTC trace SHA-256 is
`70ccf6f0603b255aada6e22bf1fcf0360de6df87ac0a6f1a18f91b34042dab2f`.
The earlier diagnostic failures and all full-mission failures remain retained.
No full regression rerun is claimed for this documentation-only increment.
PR #950 remains Draft, with motion acceptance blocked and physical acceptance
**HARDWARE_REQUIRED** under the exact-baseline procedure below.

## Bounded FTC diagnostic continuation (2026-10-18)

Latest fetched `dev` is `da1494f1e3a878383c86c2b5dbd518f4233dc330`,
incorporated by merge `a5b02e46`. Its change since the previous `e48cf12a`
baseline concerns GUI updater readiness. The `ros2/`, `sensors/`, `docker/`
and `.github/` trees remain byte-identical to pushed `f31f22dc`; motion source
remains `f44daf90`. All five required hosted checks passed on `f31f22dc`,
including Debug, Release and RelWithDebInfo replay. That hosted result does
not certify a later head or resolve the GPS01 containment failure below.

An external diagnostic-only FTC overlay was built against the unchanged,
previously attested thirteen-package installation. Its plugin and all its own
test executables were rebuilt together; this was not a pristine product build
or a new thirteen-package regression run. The overlay records actual accepted
and rejected predicate calls, consumed pose/geometry/footprint and final
returns without changing authorization policy. Independent source review and
controlled validation preceded the single bounded simulation attempt.

The complete controlled validation passes all 26 plugin CTest targets and all
28 selected MotionAuthorization cases. Independent capture compares 258
compute outcomes exactly with tracing off/on: 220 nonzero returns, 22 zero
returns and 16 exceptions. All 248 actual predicate checks replay through the
production C++ predicate, including first-check acceptance followed by
remaining-rotation rejection. Nine finite numeric bit controls and six invalid
controls pass. A private 4 KiB trace filesystem causes a recorded payload-write
failure; all 258 outcomes still match tracing off, while the incomplete
sidecar is retained and rejected. The specific OS errno was not captured.
These are controlled software checks, not mission or physical evidence.

The isolated observer control establishes exact command CDR/header identity
and a known producer GUID through its own Cyclone graph, the passive FastDDS
callback and a periodic endpoint inventory. The producer and controller retain
Cyclone. Foreign graph node names are placeholders in this setup; the known
test producer's identity does not establish arbitrary mission ownership.

The subsequent GPS diagnostic is **INCOMPLETE**. It reaches undocking,
planning a 9,420-pose path and transit, then the observer crashes while
serializing generated UUID byte elements. Liveness monitoring stops the run
in transit; no FTC compute call was captured. The trace retains an incomplete
sidecar and has no footer, so it cannot prove that no call occurred. The
interrupted canonical harness reports FAIL at approximately 36 seconds;
no full mowing/docking/charging cycle or target
event was obtained. Container exit is 1, no container OOM is recorded, and
the owned container is removed. Previously attested inputs remain unchanged.

The UUID conversion has an old-RED/new-GREEN control. A later isolated ROS
probe retains one correctly projected UUID/status message without observer
errors, but its exact action-status CDR comparison still fails. The sender's
comparison bytes were not retained, so a padding or transport explanation is
unproved. That validation remains **INCOMPLETE** and blocks the proposed
single repair repeat. No further mission was launched through the failed gate.
Earlier failed probes and the interrupted run remain evidence; no assertion,
expected geometry, tolerance, deadline or safety policy was relaxed.

This continuation adds no product motion change. It does not recover the
missing preceding accepted tuple from GPS01 or exonerate FTC. The original
13.944 mm active-FTC truth-reference crossing remains unresolved within this
PR's acceptance boundary. Broader RPP/transit/output/localization authorization
remains #924 work. Keep #950 Draft: passing CI and the earlier completed
GPS/LiDAR runs do not establish merge readiness. Physical validation remains
**HARDWARE_REQUIRED** under the exact-baseline procedure below.

## Scoped overnight verification (2026-10-17)

This checkpoint supersedes earlier readiness statements only for its exact
source and executions. The motion source is
`f44daf90da02bb49bf8be6fec48fccd9896cc2f5`; dev is
`e48cf12a4d73d1a56fdd9d5829f359946e3a6924`, incorporated by merge `01d75ffe`.
A remote check at 13:35 UTC on 2026-10-17 still identified that dev revision. PR #950 remains
Draft. This continuation addresses FTC/Escape authorization and the necessary
coverage replay repair. Broader RPP/transit, output and localization
authorization remain #924 work. Every earlier failed run remains evidence.

### In-scope repairs and failing-before controls

Turn-fallback retreat now has its own conservative ledger using measured
velocity or the preceding authorized command after caps/tapering. Six real
old-controller regressions reproduce selected-budget overshoot under frozen,
partial and accurate feedback at 100/500 ms holds. The corrected cases pass.
Completion and replanning use the actual retreat pose. Existing per-compute
charging also covers planner early returns, so boundary loss/restoration
cannot refill an active budget or freeze its existing abort deadline.

Physical reverse accounting now uses monotonic elapsed time. Four normal
old-controller controls pass while eight shortened-ROS-clock controls exceed
their unchanged travel budgets: approximately 0.10876 m for a single fallback
step and 0.63298 m for repeated ordinary escape against 0.10 m budgets. All
twelve pass with the repair. Active ROS-time simulations retain their time
domain; PID and pose-age clocks remain unchanged. These are held-command
software integrals, not measured robot travel or stopping/coast evidence.

The proposed final densified ring now undergoes continuous boundary/hole
containment after neighbouring ordinary fillets can be retuned. A real
regression involving a 20 micrometre hole fails five safety assertions before
the repair and passes afterwards. Rejection restores the entire exact legacy ring and
clears replacement provenance. Unchanged legacy rings are not newly certified:
retained split-field baseline rings themselves contain outside-working-boundary
chords, and rollback preserves those exact legacy defects.

Hosted replay on `2e466e63` exposed generic Dubins straight seams in
`multi_polygon`/`near_degenerate`: radial residuals of 10.627/11.617 nm exceeded
the unchanged 10 nm assertion. Bounded bevel interiors now follow one analytic
circle, preserving the legacy helper's selected tangent endpoints and radius.
Uniform sampling changes chord placement as well as the nanometre seam.
General Dubins and ordinary corner rounding remain unchanged.

Hosted Debug on `428a9dee` then retained two unsupported corners in
`near_degenerate`. Diagnostic-only heads `dcdcf0de` and `49bc04ee` proved that
all 21 legacy chooser radii were rejected before circular construction, despite
valid eligibility/trim gates. Native selects one radius on that same default
input; native therefore does not reproduce that platform-specific default
failure. The identical captured geometry at a fixed 0.4 m radius provides a
portable causal regression: compilation/linking succeed, all sixteen other
cases pass and only the new case fails on the old library. The final library
passes all seventeen production cases.

A bounded native debugger capture of that old fixed-radius case confirms
cancellation in the custom Dubins solver: the RSR squared-length expression
is algebraically a sum of two squares, yet its expanded evaluation gives
-8.881784197001252e-16 and rejects the word. The selected candidate length is
3.1412908725 m; the intended single circle is 0.6284191244 m and the unchanged
short-fillet limit is 1.7 m. The captured native discriminants and length are
actual observations. The exact selected word and first caller rejection guard
were not captured because the debugger script stopped on a conversion error;
that attempt did not complete the GTest. The separate old-library regression
above supplies the failing-test evidence. Hosted CI proves chooser rejection,
while its internal discriminants/word remain unobserved. No generic solver
change was introduced to repair other planning callers.

The fallback is confined to eligible microscopic bevels where the old helper
selected no fillet. It uses the same radius grid, minimum radius and 49% trim
limits, retaining strict local and final-ring continuous containment. A valid
legacy selection rejected by later containment is rejected, not retuned.
Separate oracle cases measure radius, tangency, direction, complete sweep and
continuous containment; deterministic sampling equality is an additional
constraint. Pristine-baseline ordinary rounding and downstream reconstruction,
exact non-ring state, fixture identity and real mutation controls remain in
force. No golden output, fixture, assertion, tolerance, or recovery deadline was weakened.

### Final software evidence

At clean `f44daf90`, the pinned simulation image builds all thirteen selected
packages. All 249 CTests and 2,367 actually executed GTests pass; the aggregate
report is 3,834 checks, zero errors/failures and 412 skips. All 261 XML files
match the fresh registration/results inventory, and all 1,876 installed-file
hashes were checked. The final install manifest SHA256 is
`b6eb4ead06fc9688a1b682e2be7cc11b93e0797de38be5e984663c0416c8e49f`.
FTC's entire sixty-case speed/budget/boundary suite also passes with Fast DDS.
Focused native Debug passes seventeen production, ten independent oracle and
four semantic cases. Configuration checking and fifteen host regressions pass;
those configuration sources are unchanged since that check.

Final native Release replay covers all 26 fixtures with fifteen plain runs and
100 traced runs each. All are stable, fixture-identical and trace-passive:
seven strict identities and nineteen independently certified local changes.
The strict identity diagnostic remains RED at 7/26 and is retained. Both
11-method real control suites pass. There are 25 CTest targets, 335 GTests and
62 Python unit methods, plus 22 actual control-method executions.

A future repaired-baseline control requires and produces 26 strict identities,
with zero changed-output certificates. There are 3,770 new records: 3,380 main
records plus 390 future-baseline records. The future comparison explicitly
reuses the 390 candidate and 2,600 traced main records. The combined audit
SHA256 is `9fa23b8646c036d16679c89100e16cb89a86976f3ea226ef5f737af3294b7077`.
The native Fields2Cover library is
`632a9a9e54ed84cb1029a097fdd73a066889a35eef6c891c0039316c1dd7091a`;
its original source/compiler flags are unknown. This does not reproduce the
hosted pinned-library environment. Historical three-configuration results at
`1c3d6f3e` and intermediate `428a9dee` executions remain separate.

Pinned cppcheck 2.18 on the three final production translation units remains
**NOT CLEAN**: all analyses exit 1 with 37/130/111 information/style findings,
including missing-include and bounded-analysis limits. No warning/error finding
is emitted. Diagnostic ID/severity/message multisets match the preceding run
exactly, ignoring line shifts; no new suppression was introduced. Earlier broad
analysis failures and skipped checks are not converted to static-analysis passes.

An independent cold-context safety/code review of the final repair series found
no introduced BLOCKER or SHOULD-FIX. That code review does not establish
simulation containment, hardware safety or overall merge readiness.

Hosted GitHub replay on exact `f44daf90` passes in Debug, Release and
RelWithDebInfo: [run 39018622393](https://github.com/mowglinext/mowglinext/actions/runs/39018622393).
Each completed job passes pristine-baseline 11/11 CTests, candidate 12/12 CTests
and the replay comparison; the primary completed logs were retained. Other
required PR checks passed or were explicitly skipped. This is actual hosted
evidence, separate from the native unknown-origin library results.

### Complete simulations and comparison limits

GPS-only canonical tracking uses `/odometry/filtered_map` against the nearest
segment of the full plan; canonical travel is accumulated from `/wheel_odom`.
These measurement paths are unchanged from dev. Their values are estimated-map
tracking and wheel-odometry travel, including stationary samples while a cached
MOWING phase persists. They are not ground-truth tracking or physical accuracy.
The LiDAR harness instead uses `/sim/ground_truth_pose`. The separately recorded
passive truth/command stream supplies the GPS containment observations below.

The first final-source LiDAR cycle completed with canonical/container exits
0/0: one unit completed, none skipped, 407.8 m total travel, 93.2% sampled area,
and IDLE_DOCKED/charging at 2568.8 s. Its seven phases include the complete
mowing cycle, obstacle handling, manual/HOME, recording/cancel and emergency
reset. The canonical tracking metric has median 7.2 mm, p95 66.0 mm and maximum
80.2 mm. Those values use minimum distance to any segment of the original
full plan, allowing a different nearby swath to hide active-goal error. The
last resumed 18-pose mowing-tail goal reports controller feedback maximum
0.575 m, mean 0.480 m and RMS 0.490 m over 50 samples. Passive GT recomputation
in its goal window gives approximately 0.587 m to that active tail versus
0.078 m to the full plan. It was accepted at 23.5% with 0.378 m remaining and
0.488 m endpoint error under the configured short-end rule. No raw feedback
samples locate its exact peak. The remaining tail is not proved covered;
Independent baseline review finds no introduced scoped defect in this chain:
trimming persists a target cursor before transit, failed transit leaves that
cursor, the immediate nearby retry bypasses transit, and the unchanged checker
accepts the short remainder. Those semantics and the full-plan nearest-swath
tracking alias predate this PR. Preserve them as a separate coverage/executor/
test-observability concern; no out-of-scope rule was changed here.

Informational SLAM-growth and idle-ratio checks remain FAIL. The original host
client exited 143 for an undetermined reason; its outer exit is absent.
The same container continued uninterrupted. A separate read-only waiter
recorded completion and checked all 1,876 installed hashes; it did not recreate
the missing outer result. This is a passing canonical/container cycle with an
interrupted host client, not a clean outer pipeline pass.

The full passive truth record contains 23,709 valid map-reference rows and
19,301 latest-RPM-active rows. All 506 outside rows lie south, at initial or
return dock/transit positions: 504 inactive, one unknown and **one active**.
Autonomous MOWING has zero active/outside rows. The all-mode active/outside
filter remains **FAIL** because of the post-dock manual feature described below.
Neither this sampled result nor canonical boundary PASS proves continuous
chassis containment. All 12,053 fix/status pairs join by exact ROS header stamp.
Status application sequences are unique and cover 1..12053; headers are
nonzero/nonfuture, and publisher GIDs/raw DDS timestamps are present. The
untruncated GNSS capture has 54,388 rows below its 64 MiB cap; optional DDS
sequences remain null.
Timestamp pairing is not direct shared observation identity. FTC telemetry
reached its separate 8 MiB cap. The first prospective Path recorder was attached
late and ended footerless, missing the beginning and final approximately 54 s;
its payloads remain useful partial receipt evidence only.

The second final-source LiDAR cycle completes normally with outer/canonical
exits 0/0, 410.4 m total travel, 94.9% sampled area and IDLE_DOCKED/charging at
2601.8 s. Its full truth record has 24,012 finite map rows, 19,361 latest-RPM-
active rows and zero active/outside rows. All 508 outside rows are south/dock-
related: 507 inactive and one unknown. Logs independently confirm another
approximately 100 ms manual-enable request while charging. The ten-hertz RPM
samples miss it; this does not resolve the first manual/output observation or
the charging-inhibit contract.
Informational SLAM-growth and idle-ratio failures remain. Its nearest-full-plan
median/p95/max are 7.7/69.7/87.6 mm, with the same aliasing limitation. Root's
post-run check rehashes all 1,876 installed files:
none differ, source remains clean `f44daf90`. Independent audit records the final
13-pose tail accepted at 8.3%, remaining 0.318 m and endpoint error 0.491 m;
active-goal feedback maximum/mean/RMS are 0.514/0.457/0.459 m. The unchanged
short-end/nearby-retry policy and nearest-full-plan aliasing again limit the
coverage claim. Its GNSS capture has 12,207 exact fix/status header-stamp pairs, a continuous
status sequence 1..12207, meaningful DDS GIDs/raw timestamps, nonzero/nonfuture
ROS headers, null optional DDS sequences and no output cap. FTC telemetry reaches
its separate 8 MiB cap. The prospective Path recorder begins about 38 s after
launch, preserves 33 unique CDRs with verified hashes and three publisher
identities, and ends footerless after exit 137. Its last receipt precedes
shutdown; do not call it complete history or consumed-path ownership. The first
full plan retained early does not provide historical reception proof.

The first final-candidate GPS run completed with outer/canonical exits 1/1
after the unchanged 3000 s deadline. Undocking/planning passed; mowing/docking
never completed. It travelled 160.7 m. Its tracking median/p95/maximum are
7.1/56.1/95.6 mm, with a late stationary interval after runtime interruption.
The cached MOWING dashboard is not evidence of a live BT after the abort.
Automatic HOME was unreached; this GPS harness makes no explicit fallback
HOME request after timeout. Its passive result is **FAIL**: one MOWING sample at GT stamp 1792235553.7118497 has latest RPM 3000,
truth (2.503302786,0.786056269), 13.944 mm south of the reference edge, and
latest command (0.15,-0.575614081). These latest receipts do not establish
controller ownership or consumed pose. Independent scoped review confirms
active FTC FOLLOWING at the crossing. The later rejected input at estimated
y=0.812688878 produces a 0.1 s endpoint y=0.797754754, consistent with rejection.
No introduced predicate or accounting bypass is proved. Missing preceding
accepted-check inputs/commands and consumed pose/geometry/output bindings
prevent exoneration. Preserve the failed active-FTC containment and unresolved
ownership links as #924 follow-up, without assuming previous commands valid.
The exact event-time 121-pose cusp window matches the prior 2e/428 pre-bevel
artifact; those PR heads do not establish whole-PR e48 geometry provenance or
explain the dynamics.
The completed truth record has 23,424 finite map rows, 8,616 with latest
received RPM active. Exactly one active/outside row is the event above. Total
outside rows are 335: 333 inactive, one unknown and one active. The maximum
truth-observer receipt gap is 142.199 s. All 1,876 installed files still match
the final manifest and source remains clean `f44daf90`.

Independent GNSS audit finds 11,979 unique header-paired fixes/statuses, with
nonzero/nonfuture headers, DDS GIDs/raw timestamps and null optional DDS
sequence numbers. Application sequences span 1..11980 with value 4886 missing;
this is not uninterrupted observation delivery. The 31,490-row, 32,195,841-byte
capture is below its 64 MiB cap. The prospective Path record retains twelve
unique CDRs plus fifteen duplicate/identical-republication rows, below its
limits; it ends footerless after observer exit 137. It cannot establish a
complete history or consumer/action identity.

A separate no-compile production-library probe ran the same reconstructed
request against pristine `e48cf12a` in the pinned isolated image. Runtime
before/after hashes and loaded-library identity pass; it completed in 1.563 s
and its owned container was removed. The first tooling attempt failed with
exit 127 because loader paths were incomplete; that failure and its cleanup
diagnostics remain retained. The corrected environment does not change product
code or request geometry.

The exact original 121-pose comparison remains FAIL: its window is absent from
e48's 9,371-pose output, compared with 9,420 candidate poses. A distinct narrower
comparison finds fifty exact full poses ending at the cusp, including its
predecessor and heading. Pristine dev already returns the approximately
151.9136-degree non-pivot reversal and reports its pivot sweep as not fitting.
The next sample differs by 0.146682 mm: a downstream bevel shortens a straight
support segment, and the unchanged 0.1 m densifier divides it into 66 rather
than 70 intervals. Ring 0 gains 49 points; the other four rings, connector
boundary and examined non-ring/swath stages are exact. This establishes that
the unsupported sharp reversal predates the PR for this request, while
preserving the real sampling changes and failed larger comparison. It does
not explain the physical simulation trajectory or exonerate accepted FTC
commands. The planner/executor contract and pose/check/output ownership remain
unresolved follow-up evidence.

The first f44 GPS runtime interruption is documented by the run logs and
coredump. GPS fix/status and scan receipts last arrived at 11:27:29.929 and
11:27:29.903 UTC, respectively; the resulting delivery gaps began before
the first recorded host OOM at 11:29:38 UTC. At 11:32:07 UTC, CycloneDDS
logged a `ddsi_spdp_unref_locator` discovery assertion; the run-owned coredump
records signal 6 (SIGABRT) in `behavior_tree_node`. Later OOM events overlap the
failure sequence, but OOM is not proven to have caused the delivery gaps or
abort. No evidence identifies FTC or EscapeStartBlocked as the aborting
component. The 13.944 mm active-FTC boundary failure described above predates
the interruption and remains **FAIL**. This continuation remains scoped to
FTC/Escape; broader #924 work is out of scope.

The common dev LiDAR control completed with outer/canonical exits 1/1.
It stalled near 79 m with the blades off; final totals are 15.7% area and 80.0 m.
The occupied-goal refusal at (2.68,1.12) is followed by repeated RPP transit
collision/patience and BackUp failures while advancing later transit targets.
These are NavigateToPose/FollowPath failures, separate from FTC CoveragePath.
A nearby world box is consistent with a collision concern only if frames align;
no synchronized goal/costmap/TF/footprint cell join proves the exact map cause.
All-zero dev simulated observation sequences are an independent provenance
confound, not a proved transit cause. Its canonical overall result is FAIL. Full truth has 28,803 finite map
rows, 4,711 latest-RPM-active rows and 579 active/outside rows in four MOWING
episodes, with a maximum of 78.067 mm north. HOME does not produce verified
charging; cycle, boundary, coverage and emergency-reset failures remain. An independent final audit
confirms the same truth pose after the last >1 mm movement, at wall stamp
1792232811.908. This includes 115.7 s in RETURNING_HOME (1061 rows) with zero
command/RPM and charging false. NAV_TO_DOCK_FAILED is followed by one
uncharged IDLE_DOCKED sample; it is not successful docking. GNSS has 14,623
header pairs; all application sequence values are 0, with meaningful DDS
metadata, nonzero/nonfuture headers and no output-cap truncation. FTC/boundary/
detour captures reach their own limits; the prospective Path recorder exits
137 and leaves a footerless file. Those partial streams cannot certify the
absence of later events. Cleanup's missing-PID warning runs under `set +e` and
does not replace the canonical failure.

The normalized common-harness dev GPS control completes with outer/canonical
exits **1/1**, not a successful cycle. UNDOCKING and PLANNING pass; MOWING
fails despite 1/1 booked paths and zero skips because BT reports
COVERAGE_INCOMPLETE. DOCKING never meets actual IDLE_DOCKED/charging acceptance.
The unchanged 3000 s deadline produces the overall FAIL (reported duration
3013 s), 365.6 m of wheel-odometry travel and estimated-map/full-plan tracking
median/p95/maximum 6.0/58.2/104.1 mm. The long stationary final interval is not
additional traversal evidence.

Full truth contains 27,497 finite map rows, 19,606 with latest synthetic RPM
active and 835 active/outside rows in six MOWING episodes, maximum 104.407 mm
south. Of 8,125 total outside rows, 7,289 are inactive and one unknown. Maximum
truth-observer receipt gap is 4.319658 s. Final truth is IDLE at
(-1.336487, 0.038781), command/RPM zero and charging=false. Brief uncharged
IDLE_DOCKED telemetry after NAV_TO_DOCK_FAILED does not establish docking.
All 1,876 stock installed hashes match clean dev e48cf12a, and all 53 frozen
external helper hashes remain unchanged.

Dock staging navigation succeeds, then initial perception times out with error
904 because no first detected_dock_pose arrives. Source/log triage identifies
a preexisting simulation integration incompatibility: the installed dev GNSS
relay emits observation sequence 0, which the unchanged production association
gate explicitly rejects. The dock detector requires a first Fixed absolute pose
before it starts publishing. During the dock window, 36 header-matched Fixed
fix/status pairs all have sequence 0; maximum observer pairing skew is 0.834 ms.
The association and detector sources are byte-identical in candidate and dev;
the candidate relay instead supplies nonzero identity. This supports suppression
of the first detection, but consumer callback arrivals and absolute/detected-pose
outputs were not captured. Complete runtime causality and additional TF/process
faults are not excluded. The separate COVERAGE_INCOMPLETE/blade-link stamping
failure is retained rather than treating booked paths as successful cutting.
No baseline producer, localizer, docking or harness assertion was changed.

Global host OOM kills at 12:45:57, 12:46:02 and 13:04:17 UTC materially confound
this baseline interval. These occur after the successful candidate GPS rerun.
They do not prove an OOM cause for any particular control/docking failure or
explain away the producer identity incompatibility. Combined normalization,
producer identity differences and host failures prevent a clean end-to-end
FTC-only comparison. No automatic rerun or safety-green inference follows.

The independent final audit confirms the full truth counts and 14,035 unique
header-paired GNSS fixes/statuses, all status application sequences zero.
Headers are nonzero/nonfuture; raw DDS metadata is retained and optional DDS
sequence fields are null. The 37,661,596-byte GNSS capture is below its cap.
Maximum fix/status receipt gaps are 8.445/8.446 s, overlapping recorded host OOM
without establishing causality. Actual RTK Fixed→Float→NoFix→Fixed receipts and
WAITING_FOR_RTK→MOWING recovery are observed. The prospective Path stream has
nine first-seen topic records, eight distinct CDR hashes and nine duplicate
receipts; every embedded payload hash validates. Its 25,461,816-byte stream is
below its cap but footerless after observer exit 137. Four goal windows have
paired global/local map captures; neither that evidence nor the harness supplies
a separate successful obstacle-traversal assertion. No obstacle PASS is claimed.

The final GPS rerun, `candidate-gps-fallback-final-02`, completes with host,
outer and canonical exits 0/0/0. All four required phases pass: 1/1 mowing path,
zero skipped, 364.2 m wheel-odometry travel and IDLE_DOCKED/charging at 2293.9 s.
Estimated-map/full-plan tracking median/p95/maximum are 6.0/53.0/85.8 mm.
The raw truth recorder's final cleanup sample instead says IDLE, zero command,
RPM 0 and charging=true; this does not replace the earlier docked-state evidence.

The actual log records WAITING_FOR_RTK at 174.0 s and a preflight/undock/
calibration/replan/transit return to MOWING over 203.0–207.8 s. The known 0.5 m
square keepout is promoted before START. Two FollowStrip detours, four
obstacle-clear messages, four recovery-stable messages and all eight requested
global/local costmap pairs are recorded. FTC also records a 0.30 m reverse-escape
exhaustion followed by obstacle abort and detour. These are observed recovery
and obstacle events; the GPS harness has no separate obstacle-pass assertion.

Independent full-stream audit confirms 21,133 finite map rows, 19,539 with
latest synthetic RPM active and zero active/outside rows. All 527 outside rows
are 526 inactive and one unknown. The maximum truth-observer receipt gap is
0.146438 s. A separate boundary-edge event is recorded during docking;
outside docking/transit observations are retained without adjudication against
a broader authorization policy. All 1,876 installed hashes still match clean
`f44daf90` after this run.

GNSS has 10,774 unique header-paired fixes/statuses, contiguous application
sequence 1..10774, one observed GID per topic, raw DDS timestamps, nonzero/
nonfuture ROS headers and null optional DDS sequence numbers. Maximum fix,
status and scan receipt gaps are 0.401324/0.402015/0.403677 s. Its 28,973,485-byte
capture is below its cap. All sixteen unique prospective Path CDR hashes and
sixteen duplicate-delivery rows validate; the 44,340,711-byte capture remains
footerless after exit 137. These are publication observations, not complete
consumer/action ownership.

The rerun has 47 resource samples, minimum available memory 19,158,765,568
bytes, maximum sampled memory PSI avg10 of zero and no selected kernel OOM
records during 12:04:50–12:43:40 UTC. The later baseline OOM events do not belong
to this interval. Its canonical result precedes planned SIGINT by approximately
2.019 s; the teardown failures described below still apply.

Repeatability is limited to these exact executions: two completed final-source
LiDAR cycles and one completed final-source GPS rerun, plus the failed first
GPS attempt. The clean sampled GPS rerun does not clear the earlier 13.944 mm
active-FTC failure, recover its missing accepted-check bindings or establish
continuous/physical containment. No general safety-stability claim follows.

All final missions use Webots R2025a, driver
`db4a77c84ee91445d39de9a631e5b57a888814e2`, and the isolated image
`sha256:0fc53b7941fabeed24dceaaa88097dcc7013072b3efe2e069495f8c1ea147f78`.
They run as UID 1000 with network isolation, localhost DDS and fake hardware,
using the unchanged worlds, assertions and 3000 s canonical deadline. The
GNSS quality profile is 180 s Fixed, 20 s Float, 10 s NoFix, then Fixed.

Teardown is **NOT CLEAN**. Both completed candidate LiDAR cycles, the final
candidate GPS rerun and both completed common-dev controls log a Webots
driver segmentation fault after the planned SIGINT. The GPS rerun and common-dev GPS control additionally
log Python threading/shutdown errors and a driver respawn after canonical
completion. These are retained teardown/integration failures, separate from
canonical and outer exit results. No introduced FTC/Escape cause is established;
no unrelated Webots/runtime repair is included in this scope.

The pre-existing LiDAR harness still subscribes to obsolete Twist velocity
interfaces, so its zero dashboard velocity and empty speed/time statistics are
unavailable observations, not stop evidence. Its obstacle predicate instead
requires ground-truth proximity, filtered scan detection and actual departure
travel; that predicate does not pass merely because velocity defaults to zero.
The independent stamped-command/wheel/truth record remains separate. The GPS
harness uses the current TwistStamped interface. No unrelated product change
was added to repair LiDAR dashboard statistics in this scoped continuation.

External observer v2 records publisher GIDs and raw DDS timestamps separately
from ROS header/callback clocks. Its actual isolated API smokes pass on both
overlays. Per-run output caps and pairing/identity checks must be audited before
claiming a complete capture. Null DDS sequence fields are retained honestly.
Header freshness and delivery metadata are not physical acquisition identity.

The final LiDAR run also has a bounded prospective Path receipt recorder,
attached after the mission started. Actual retained messages from all three
plan topics include exact CDR/JSON data and publisher GIDs/raw DDS timestamps.
Its late start cannot identify earlier event-time paths. Volatile versus
transient-local readers and duplicate-or-identical-republication records remain
explicit; publication receipt alone does not prove consumer/action ownership.
The capture has separate 64 MiB/120-path/3500 s limits and honestly incomplete
termination markers. Audit its actual cap/footer status before using it.
Passive boundary counts describe sampled map reference points paired with the
latest received blade RPM; they do not establish continuous chassis/footprint
containment or synchronized physical blade actuation.

Pristine dev `e48cf12a` GPS/LiDAR attempts failed before START because dev did
not forward the requested GNSS quality input. The first logger-compatible
normalized dev GPS attempt also FAILED: the old harness saw no path poses and
falsely reported zero-second docking; final truth was MOWING_COMPLETE, charging
false, and the actual docking goal was cancelled during shutdown. Its complete
truth capture has 812 latest-blade-active outside rows across six episodes,
maximum 92.714 mm south. Those failures remain in the record.

New controls are explicitly labelled normalized common-harness controls. They
use the exact current candidate E2E harness externally, including current path
interfaces and the GPS stamped-velocity interface, with stronger requirements
for complete paths, tracking,
all phases and actual IDLE_DOCKED/charging. Common scenario/quality forwarding,
datum and antenna geometry are normalized. Dev controllers, BT, Nav2,
localization, relay and world product code remain unchanged. All captured dev
position-observation sequence values were zero in the earlier normalized run;
relay identity/freshness semantics remain a comparison confound. These controls
are not pristine dev and cannot isolate every localization effect.

### Retained intermediate failures and causal limits

Intermediate `2e466e63` GPS passed its canonical cycle: 1/1 path, 364.6 m,
docked/charging at 2294.5 s, median tracking 5.7 mm. Its 21,119 map-reference
samples include 19,550 latest-blade-active samples with zero sampled active
outside episodes. Observer v1 has empty per-message DDS metadata. This does not
validate final geometry, continuous containment or historical failure causes.

Intermediate `428a9dee` GPS passed its canonical cycle: 1/1 path, 9,420 poses,
367.0 m, docked/charging at 2281.7 s; median tracking 5.5 mm, p95 51.2 mm,
maximum 94.7 mm. Eight complete detour map pairs and Float/NoFix recovery are
retained. The independent passive result is **FAIL**: two latest-blade-active
outside rows in one episode, maximum 20.163 mm south, among 20,973 valid map
rows. There are also 502 outside inactive rows and one unknown. Complete v2
capture has 10,727 fix/status pairs with nonzero/nonfuture headers, meaningful
GIDs/raw DDS timestamps, null DDS sequence fields and no cap truncation.
Invalid-fix rejections around NoFix remain recorded separately.

A reconstructed offline request exactly reproduces all 9,420 later-plan poses,
quaternions and duplicates using the actual `428a9dee` installed core. Only
then was the old `2e466e63` core compared. Both have the same 151.9136-degree
unpivoted cusp and identical 121-pose neighbourhood, already present in the
identical 42-point pre-bevel geometry before sampling. The neighbourhood hash is
`7853eacc39bbf6f6a13f67ebb87d068023747dd5a4bf213bbc9c3a024d5bd43b`.
No safe pivot fits. This is a pre-existing planner/executor contract concern,
not a new cusp introduced by the sampler. This later plan is not the crossing's
event-time path and cannot establish dynamic causality or exonerate that event.
Recent retained estimates were inside at the first truth crossing; FTC denied
its subsequent sweep approximately 120.7 ms later. No predicate bypass or
initial reverse-escape cause was established. Exact consumed pose and output
ownership remain unresolved.

The original complete LiDAR cycle's two truth-boundary failures, along with
the later GPS/LiDAR failures described below, remain failures. Budget repairs do not prove
those excursions eliminated. Out-of-scope RPP/transit/output authorization,
pose-versus-truth and recovery exceptions remain #924 follow-up. No guessed
pose margin, planner inset, simulator tuning or weakened pivot/collision guard
was introduced to obtain a pass.

### Remaining acceptance

A separately reviewed post-dock manual observation remains outside this PR's
FTC/Escape responsibility. One sample at truth (-0.27945, 0.00302) has latest
MANUAL_MOWING, charging=true, synthetic RPM=3000 and zero command, surrounded
by stationary samples reporting RPM0. Logs establish an approximately 100 ms
manual ON→HOME→OFF request pulse. The fake bridge directly latches the service
request and synthesizes RPM independently of charging; this is not measured
physical blade activation. Source comparison shows the relevant manual tree,
request/output paths and fake bridge unchanged from dev. The examined real
bridge/firmware gates also contain no explicit charging predicate; this is an
existing authorization-contract question for #924/manual-output/charging and
simulation fidelity. No new policy was implemented here and the sample was not
removed to make the passive filter pass. Physical acceptance of a manual request
while charging remains HARDWARE_REQUIRED: freeze unit/image/firmware/config,
remove blades, secure the chassis, verify emergency cutoff and safe charger
setup, and capture charge/contact, HL mode, ON/OFF, ESC activity and independently
observed motor rotation. The separate output-policy owner must specify the
required charging-inhibit contract before calling that check a pass.

Escape cancellation is proven for delivered geometry revocations. Its local
reader generation is not a producer epoch: an omitted/coalesced revoke-restore
or producer restart with identical content is not certified by that test.
Producer/pose/output identity remains #924 work; a timeout does not repair
observation ownership. The FTC geometry horizon is also not an actuator lease.

The reproducible FTC budget/accounting and bounded-bevel replay defects identified
in this continuation have causal regressions and verified scoped repairs.
Independent source reviews found no further supported introduced defect in the
reviewed changes. That finding does not establish mission safety acceptance:
the first final-source GPS run has an unresolved 13.944 mm active-FTC truth
crossing. Its preceding accepted checks and exact consumed pose/output ownership
were not captured. The guard is not exonerated, and this failure cannot be
classified confidently as entirely outside #950. A clean rerun or a preexisting
planner cusp cannot supply the missing proof.

This increment can be reviewed independently with its passing software evidence
and retained failures, but overall motion containment and merge readiness remain
unaccepted. Keep the PR Draft. Broader RPP/transit, producer/pose/output identity,
manual charging-policy and runtime integration follow-ups remain separate;
no unrelated policy repair was introduced to obtain green results. The next
software investigation of the active-FTC event needs joined accepted/rejected
check inputs, consumed geometry/pose identities and controller/output ownership;
any repair must first establish an in-scope causal defect. Do not infer that
remaining checks can be satisfied by further uninstrumented mission repetitions.

**HARDWARE_REQUIRED.** No robot motion, deployment, flash or field validation
was performed. The actual robot/ARM image, flashed firmware hash, unit/config,
GNSS receiver/driver revision and submodule baseline have not been acquired.
Before the physical procedure below, freeze those identities with the exact
motion commit. Remove blades, establish an exclusion zone, verify emergency
stop and supervise low-speed edge, concavity/hole, pivot, escape, transit,
docking/charging, cancellation and resume cases. Pass/fail must measure the
declared chassis/exception envelope, stopping/coast, blade-off preconditions
and approved recovery completion. Simulation and held-command integrals prove
none of those physical bounds. Clock mode must remain fixed per operation;
holds over 0.5 s are outside the command-budget claim. The separate 0.1 s
geometry horizon is unchanged and is not a downstream output lease.

## Scoped FTC/Escape follow-up (2026-10-16)

This checkpoint supersedes earlier results only for its exact source and runs.
Latest dev remains `29eaf625`; the earlier merge is retained. The continuation
changes FTC reverse-escape code/tests and two test-harness logging calls,
with matching documentation.
RPP, transit, output authorization, localization and simulator models are not
modified in this increment. The motion source for this checkpoint is
`c2e470b7950c98ba9ce07266192d8a548c23c30d`; subsequent test-harness logging
and evidence-only commits do not change the first-party controller binaries.

### In-scope defects and bounded repair

The real-controller regression reproduces a bookkeeping/output mismatch with
accurate feedback and ideal actuation: a 0.10 m escape at 0.15 m/s with 100 ms
holds issues 0.105 m before the old accumulator reports its capped 0.10 m.
With frozen velocity feedback, the old code can keep issuing reverse commands.
This is independent of pose/physics uncertainty and has a failing-before control.

A second review finding concerns planner early returns: with an active escape,
`confine_deviation_to_zone=true`, valid raw authorization geometry and TF, a
malformed global boundary costmap bypasses the helper that used to charge the
budget, while the controller continues returning reverse. A review control of
the initial budget-only repair, before boundary hardening, returns 0.145 m
against its 0.10 m budget after boundary loss/restoration. Charging now occurs
once per compute tick. A missing required guard holds zero while preserving the attempt and spent
budget; restoration cannot refill it. A further failing-before control proves
that boundary loss after budget exhaustion froze the existing abort deadline.
The wait now retains that deadline whether the escape is active or already
exhausted. Real-subscription tests cover both failures and clear-path recovery.

The outgoing command is capped by remaining distance using the existing 0.5 s
control-interval ceiling. Budget charging conservatively covers measured speed
and the last authorized request after external caps, before tapering. This can
hand recovery back early rather than spending the entire physical distance;
it is a maximum budget, not a promise of minimum travel. Logged escape
distance is conservatively charged budget, not measured physical travel.
Numeric accounting rounds consumption upward and available command speed toward zero. The existing
0.1 s geometry certificate, safety checks, external speed caps, retry/clear-hold
policy and all published assertions remain intact. No new control rate,
physical stopping margin, timeout or output lease is introduced. Holds longer
than 0.5 s, actuator response and inaccurate poses are not certified here.

### Classification of the recorded violations

The earlier west-edge event follows handoff to blade-off RPP after FTC refused
motion. The accepted planner path does not authorize RPP's executed arc;
residual motion and RPP contributions are not isolated by those samples.
Controller/output authorization and coherent outside-start/transit exceptions
remain #924 follow-up and are not implemented here.

The earlier east-edge event occurs during FTC reverse escape, before RPP.
The recorded FTC denial is consistent with its estimated input pose, which
remains inside while truth has crossed. The later drive-plant capture proves
joint/body divergence but does not isolate that original event. The newly
reproduced budget defects are fixed within scope; they are not asserted to
explain or eliminate either recorded truth crossing. No guessed pose margin,
planner inset or simulator tuning is used.

The new GPS attempt has one sampled blade-active north-edge episode: seven
rows, maximum 6.869 mm outside. At the first sampled exit, the preceding retained
TF estimate is still inside (Y=6.79395477 versus truth 6.80050263, 50.314 ms
receipt separation). This is not simultaneous acquisition. FTC later denies
its forward 0.1 s sweep at estimated Y=6.79998754; the computed arc endpoint
exits at Y=6.80112208. A brief reverse-and-turn wheel command appears about
0.819 s after the initial higher-rate crossing, followed by zero; downstream
ownership is unresolved and it is not an unmodified straight escape twist.
No new FTC predicate bypass or initial reverse-budget cause is demonstrated.
The GPS canonical harness does not report this truth-boundary criterion;
passive analysis exposes it separately. The excursion remains an unresolved
safety limitation, not a passing containment result.

The new LiDAR stationary handoff begins with the unchanged FTC pivot-sweep
obstacle hold/abort, then blade-off detour, RPP collision/patience failures and
Nav2 BackUp. Collision monitor stops the backup at truth (3.792446,1.233962).
This is not FTC reverse escape or EscapeStartBlocked; neither new budget
exhaustion nor a GNSS guard immediately triggers that handoff. Saved rounded
detour centers are below raw cost 253, so no occupied-center regression is
shown. The first rounded map goal (2.96,1.44) nevertheless lies inside the
unchanged nominal physical box footprint [2.75,3.25] x [1.25,1.75], while its
captured global center cost is 94; scene/global-center representation differs.
This is not proof that the corrected center predicate failed; post-dispatch
snapshots do not prove the exact pivot obstacle or full route viability. Pivot functions and detour/transit sources match `cfd45403`.
No new in-scope defect is established. The RPP/BackUp/costmap/output/recovery
work remains #924 follow-up, without weakening the pivot or collision guards.

### Coverage replay contract blocker

The actual Release and RelWithDebInfo jobs in CI run `38803346111` build and
pass baseline 9/9 and candidate 11/11 CTests, then fail the unchanged comparison
at `compare_replay.py`'s `all_planner_outputs_match` assertion under
`--require-identical`: 5/26 exact outputs match. Independent local rechecking
of the downloaded evidence returns exit 1; deterministic, fixture-equality,
trace-passivity and preserved-stage checks pass. The completed Debug job also
fails at that same final comparison step; its per-case artifacts were not independently re-audited in this continuation.

A stronger causal control executes the existing unchanged
`BufferedRectangleOuterRingHasSupportedFillets` regression against both sources
with the same pinned dependency library. Dev fails its required maximum-turn
bound (45.0578565 degrees versus <15) and tight-arc count (2 versus 0); the
repaired planner passes. Reproducing dev trajectories would undo an existing
required geometry improvement. No additional build/harness defect explains
these CI failures. Exact equality to that dev planner and preservation of the
repair cannot both hold. This needs an explicit maintainer compatibility-contract
or reviewed dev-baseline decision; no exemption, normalization, expected-result
edit, disabled gate or geometry rollback is made. The replay gate remains red.
The subsequent exact-source `c2e470b7`
[run 38811253896](https://github.com/mowglinext/mowglinext/actions/runs/38811253896)
also fails all three configurations at that final comparison step; coverage
planner source is unchanged by this FTC increment.

### Separate simulation GNSS-authority failure

Both new missions repeatedly trip the unchanged GNSS freshness guard after the
programmed quality outage has recovered. At fixed wall-log cutoffs
`1792101301.782909141` (GPS) and `1792101301.922098881` (LiDAR), GPS has 19
feed-stale transitions and LiDAR 13 stale plus one expected accuracy transition.
An explicit “receipt stamp is zero/future; authority denied” warning immediately
precedes all 19 GPS and 10 LiDAR stale transitions. The behavior tracker rejects
zero or consumer-future receipt evidence; low covariance and last-accepted
RTK-Fixed quality cannot override that invalid authority. It requests blade off,
stop and RTK wait, then resumes only through the existing healthy persistence.

The guard, freshness helper, relay, navsat converter, tree and launch source are
byte-identical to `cfd45403`; that earlier mission already contains this paired
warning/failure class. Equal event frequency or scheduling is not established.
Exact zero-versus-future condition, offending observation/publisher identity and
producer/consumer clock delta are not logged, so a clock-order, 1 ns conversion,
CPU-load or wheel/body explanation remains unproven. A direct FTC cause is not
evidenced. No timeout/tolerance workaround or GNSS subsystem change is made;
coherent producer/consumer evidence remains separate #924 follow-up. The bounded
classification and exact line pairs are in `verified/guard-causal-classification.md`
outside Git. These snapshot counts are not final mission verdicts.

### Fresh validation and remaining acceptance

Focused final-source validation passes 26/26 plugin CTests (395 GTests). The
42-case real-controller executable also passes on its first isolated FastDDS
control run, separate from the full CycloneDDS run. All five changed C++ files
pass whole-file and changed-line clang-format 18 checks, and whitespace checks
pass. Independent review closed the accounting and deadline findings with no
remaining supported blocker in the incremental code diff.

The final clean source builds all 13 selected first-party/tools packages and
passes **248/248 registered CTests and 2,314 GTests**. Aggregate colcon output is
**3,770 checks, 0 errors/failures, 407 skipped**; those skips are the installed
ament cppcheck wrapper, not 407 successful analyses. The unchanged prior
standalone cppcheck 2.18 failures remain separately disclosed below; no new
complete static-analysis pass is claimed. The final overlay also passes the
separate FastDDS 42/42 controller control. Build/test and both mission manifests
record source `c2e470b7`, its clean source fingerprint and the pinned runtime
image; the old successful workspace was copied independently as a cache seed,
then the current build and full tests ran again. The simulator underlay remains
source `28184b52`, separate from the current first-party overlay.

GitHub [ROS2 CI run 38811254004](https://github.com/mowglinext/mowglinext/actions/runs/38811254004)
passes on exact source `c2e470b7`: Lyrical build/test (protected legacy
check name `Build & Test (ROS2 kilted)`), config drift, formatting and its
report-only static-analysis job. That job success does not resolve the local
cppcheck skips/earlier standalone opt-in failures.

Fresh evidence is outside Git at
`/home/bruno/.cache/mowglinext-pr950-scoped-continuation/verified/`: the install
manifest, `suite-inventory.json`, build/test logs, `fastdds/`, and mission
provenance/results. Failing-before controls remain in the earlier cache
`/home/bruno/.cache/mowglinext-pr950/ftc-continuation/`. No evidence archive is
added to the repository.

Both full attempts retain their original worlds, scoring and 3000 s timeout.
They are **FAIL**, not stable successful end-to-end mowing validation:

| Scenario on motion source `c2e470b7` | Final result |
|---|---|
| GPS, physical box promoted to known keepout | Canonical/outer exit codes 1/1. Mowing FAIL after 1853.0 s, 0/1 paths, 0 wholly skipped; localization-guard re-dispatch budget exhausted. Undock 13.1 s, planning 27.7 s and fallback docking/charging 51.7 s PASS. Total 229.0 m. |
| LiDAR, physical box unmapped | Canonical/outer exit codes 1/1. Timeout at 3000 s, 0/1 paths, 30.6% measured coverage, 136.5 m. Undock 12.3 s, planning 52.4 s and obstacle encounter/detection/departure PASS. RPP/BackUp stall prevents completion. The timeout path then raises `AttributeError` on obsolete `RcutilsLogger.warn`, preventing HOME/docking and post-dock feature tests in this attempt. |

Closed passive traces contain 15,045 GPS samples (11,533 blade-active mowing;
seven outside in one episode) and 23,748 LiDAR samples (6,373 blade-active mowing;
zero outside). Both have zero sampled blade-active reference points in the
known hole and zero modeled body/box overlaps. Initial blade-zero corridor
samples are separate from blade-active excursions. These are sampled,
latest-receipt correlations, not continuous footprint/blade containment,
actuator synchronization or hardware evidence. The blocked LiDAR attempt does
not clear its two earlier recorded violations.

GPS FTC capture has four windows / 13,301 telemetry rows / 5,138,100 JSONL bytes and
no cap; LiDAR's four-window capture reaches its 8 MiB bound. A separate passive
LiDAR reference-exit observer (four 2 s-before/4 s-after windows, 2 MiB total) starts
later with a recorded source/admission manifest and closes normally at the
E2E exit with zero exit edges. The 32-pair map-observer cap remains intact;
later goals may lack map pairs.

The two obsolete `.warn()` calls are a test-tool compatibility defect, separate
from the RPP stall. The narrow repair uses the supported `.warning()` API and
preserves scoring, default timeout and controller code. Commit `7905ba1c`
changes exactly those two calls. An actual runtime API
smoke confirms `warning()` works and `warn` is absent. A separate LiDAR
failure-path probe uses the same clean `c2e470b7` controller overlay and pinned
runtime, with only that two-line harness mounted read-only. The override SHA256
is `7cc7c1bab93633da1d83e060c492b887950cb2a0e6ba9598bf494feaeae5839e`;
its bytes equal the committed harness. Only this diagnostic invocation sets
`E2E_MOWING_TIMEOUT_S=15`; the source default stays 3000 and all scoring remains
unchanged. The warning is emitted, HOME reaches IDLE_DOCKED/charging (15.7 s),
then manual mowing, area recording and emergency auto-reset phases pass. The
normal final report and both exit codes remain **FAIL / 1** for the deliberately
incomplete mowing/obstacle coverage. This validates the repaired failure path,
not a complete mowing mission or HOME from the earlier blocked pose. The full
3000 s LiDAR attempt was not repeated after this logging-only change. Evidence
and exact patch/override/overlay provenance are in `verified/timeout-compat/`
and `verified/missions/lidar-timeout-compat/`; Python compilation and final
whitespace checks pass.

This remains Draft. Software simulation, GitHub CI and physical robot evidence
are distinct. **HARDWARE_REQUIRED:** freeze the exact final ROS/ARM image,
firmware image/source, universal-GNSS gitlink/receiver/driver revision, robot unit
and configuration before the supervised blades-removed procedure in
[Physical acceptance](#physical-acceptance). The software candidate is ROS
`c2e470b7`, dev `29eaf625`, universal-GNSS
gitlink `9a329cc216ed137d0e375a4cfe1f5c26e1dd52d0` and firmware source tree
`a63eb82bbf6b6c0e73fb03883e340aabab6a2f7f`. The ARM image, flashed firmware,
robot unit and receiver hardware/driver revisions remain unvalidated; the
desktop image is not a validated robot baseline. Before accepting this escape
on hardware, measure the actual command-plus-STOP response against the existing
configured coast bound and confirm useful escape/recovery without refilling a
spent attempt or crossing the declared geometry; exceedance fails acceptance.
Use the blades-removed, exclusion-zone and verified-emergency-stop prerequisites
in that procedure. No merge, deployment, flash or hardware motion occurs.

## Ubuntu 26.04 continuation (2026-10-15)

This section supersedes earlier checkpoints for the changes and runs described
here. Older runs below retain their original baselines and limitations.

### Source and environment

The existing Draft PR #950 branch was advanced in a dedicated Linux worktree,
preserving the other checkouts. Merge `fe26f7cc` incorporates requested latest
`dev` `29eaf625a6a36f0ba151bd0076177b1bcd403218` into PR head `28184b52`;
the only conflict preserves both fresh GetCostmap fixtures and dev's isolated
blade-service executor lifecycle. No merge into `dev`, physical deployment or
firmware flash was performed.

The host is Ubuntu/Kubuntu 26.04.1 x86_64. Installed prerequisites include Docker
29.1.3, Compose 2.40.3, Buildx 0.30.1, ROS 2 Lyrical base/testing tools,
CycloneDDS, colcon/rosdep, compiler/CMake/Ninja, clang-format 18.1.8, and the
documented planner dependencies. Fields2Cover 3.0.0 is pinned to
`884d895b59192882476e986ba44ea9143a06a6a9` with the project's two missing-header
build repairs. Project builds, installation, formatters and tests run as UID
1000. Only system package/service configuration used sudo. Native development
configuration is opt-in; the user's shell configuration was preserved.

Full ROS builds reuse runtime digest
`sha256:d414afdcf766094ffc845738a5a2ec23ed4292dcba22d65c763813fe51374076`
(published source `28184b52`) as an underlay, with all first-party packages
freshly rebuilt. Webots R2025a and ROS driver
`db4a77c84ee91445d39de9a631e5b57a888814e2` are installed in the local simulation
image. Missions use the rebuilt first-party overlay, loopback-only DDS and
isolated Docker networking; no robot interface is connected. Source commit,
scenario/world/config hashes, live datum/lever-arm parameters and stationary
ground-truth alignment are recorded before START.

### Planner replay: reproduced contract conflict and infrastructure repairs

The unchanged strict comparator fails because the microscopic-bevel repair
changes actual execution trajectories. Dev independently rounds each tiny
bevel endpoint and cannot fit the configured minimum-radius fillet. The PR
reconstructs the supporting-edge intersection and emits a validated tangent
fillet. Execution coordinates, quaternions, lengths and fingerprints therefore
change in 21 of 26 unchanged fixtures. Removing that repair solely to recover
byte equality would restore the untrackable corner. This is not nondeterminism.

The standalone replay harness previously omitted the 11 ring-corner safety
GTests. Commit `7007929e` registers that existing suite, including it for a
baseline revision when present. The CI baseline export also omitted the new
private `ring_corner_rounding.hpp`; a later baseline containing the repair
would fail to compile. The export now takes source, headers and tests from the
same exact baseline Git tree. A baseline containing that private header is
compiled and tested as a regression control. Fixtures, comparisons, safety
assertions, process deadlines and `--require-identical` remain unchanged.

Release, Debug and RelWithDebInfo each pass baseline 9/9 CTests (141 GTests)
and candidate 11/11 CTests (156 GTests, including all 11 bevel safety cases).
Each configuration contains 390 baseline, 390 candidate and 2,600 traced
records: 3,380 per configuration, 10,140 total. All 26 cases are deterministic,
fixture-identical and trace-passive in every configuration. All three unchanged
comparisons return exit 1: 5/26 exact outputs match dev. Boundary/retained-swath
stages, primitive/join counts and transit counts/distances remain unchanged.
The synthetic future baseline export control passes 10/10 CTests. Compiled
planner/public-header hashes match final motion source `cfd45403` across
the matrix, despite differing build-time commit labels.

**The exact-output replay gate remains an unresolved merge blocker.** The
required trajectory repair and an unchanged requirement for exact dev
trajectory equality cannot both hold. This evidence does not independently
certify every changed swept footprint, and the PR remains Draft.

### Simulation defects fixed with failing-before regressions

Commit `e93f79d4` exposes `gps_quality_pattern` through the full-system launch.
The previously documented `simulate_gps_degradation` flag was never declared
and had no effect. Valid patterns now reach the existing single quality relay;
empty defaults remain fixed quality. Malformed, empty-segment, unknown-regime,
nonfinite, nonpositive and overflowing patterns fail explicitly at startup
instead of silently becoming RTK_FIXED. The unchanged old relay fails five
new cases (22 pass); the repaired suite passes all 27. Launch wiring fails
three cases before the repair and passes all five afterward. Ament lint passes.
NO_FIX here publishes invalid observations; it does not emulate transport
silence or a frozen physical receiver.

The first live LiDAR attempt exposed a real 0.30 m localization frame error:
ground truth reached `(4.56, 6.69)` outside the `x <= 4.50` lawn while Fusion
Graph reported approximately `(4.26063, 6.69523)`. The simulator/map adapters
used the garden datum, but navigation's `(0,0)` default made Fusion Graph
self-seed at the antenna's first fix, 0.30 m east of the chassis datum.
Subtracting the antenna lever arm then shifted the estimated chassis west.

Commit `c09d8fd1` forwards one complete validated garden datum/antenna snapshot
through simulation -> navigation -> COG and both Fusion Graph variants. The
override requires simulation time; empty overrides preserve physical robot
configuration, including nondefault datum/lever arms. Regression tests execute
the launch builders for both sensor modes, invalid bundles and rotated lever
arms. Both sensor modes fail against the old launch chain and pass after the
repair. A real ROS launch without an installed robot config verifies all four
parameters in COG and Fusion Graph and stationary ground-truth error 0.015 m.
Boundary assertions and all motion, collision, blade and recovery guards are
unchanged. Early runs aborted after isolating this defect remain failures or
incomplete diagnostic runs; none is counted as a completed mission.

### Detour target eligibility

The corrected LiDAR diagnostic run still records occupied transit goals and
bounded RPP collision holds/timeouts. A static policy mismatch is independently
reproduced: the old selector's lethal-only disc test permits target centers
with raw cost 253 or 255, while AuthorizedTransitPlanner rejects center costs
>=253. Two new public action/service regressions fail old production by
actually dispatching X=0.8; real Costmap2D reports the forbidden costs.

Commit `f8ae0408` adds a separate candidate-center predicate to the fresh
snapshot search. Unknown, inscribed, lethal and outside target centers are
rejected without creating obstacle evidence or altering lethal-only footprint
clearance. Every cell consistent with float32 wire-resolution rounding must be
eligible; binary32 midpoint bounds and outward division bounds avoid the
previous deferred adjacent-cell precision defect. Ambiguous otherwise-clear
centers can be rejected conservatively. The old unknown/sublethal clearance
and inscribed-boundary confirmation assertions remain unchanged. Both public
regressions pass afterward, choosing X=1.35, with blade off, one fresh request
and no completed-area credit. Additional tests use actual Costmap2D at
translated grid lines and adjacent double values. Independent strong review
finds no actionable finding.

This repairs the deterministic selector mismatch; it does not prove that the
specific live goals were already occupied at selection. The first recorded
rejection follows an initially accepted plan, so intervening map updates or
different local/global footprint checks remain possible. Planner, RPP and
collision guards retain authority, and later invalidation can still end
the task without crediting completion. Search, clearance, minimum-remnant and retry budgets are
unchanged; cumulative cutting-gap accounting remains open.

### Regression and mission results

On final motion source `cfd45403`, all 13 first-party/tools packages build in
Release with tests enabled and all 248 registered CTest entries pass. The
compiled suites contain 2,287 passing GTests. The aggregate is 3,743 checks,
zero errors/failures and 407 skips, all from the installed ament wrapper's
explicit cppcheck 2.x skip. This rerun includes the six new detour cases. An
earlier full run exposed a missing direct `<algorithm>` include; `cfd45403`
adds it and the unchanged registered lint check and entire suite then pass.
The prior baseline run had 247 registered entries/2,281 GTests, and the datum
increment's affected packages also passed separately. Clang-format 18
changed-line checks pass for all 44 changed ROS C++ paths; config drift passes.
Initial missing Git worktree metadata and schema-DNS failures are retained as
setup failures; corrected mounts and schema-enabled tests pass without source
bypasses. Source/header hashes confirm the replay matrix's planner is unchanged
at final motion source `cfd45403`.

A separate attempt to execute those skipped static-analysis checks uses pinned
Cppcheck 2.18.0 (`bd62c8ca7be207583c60002a5af50e5f1d12660b`) with ament's explicit
slow-version opt-in. It returns nonzero: 9/10 package CTests fail. The unchanged
registered commands produce 59 GoogleTest-macro parser errors and 134
branch-limit information diagnostics that ament treats as failures. The three
remaining semantic diagnostics are inherited and source-inspected false
positives: two ignore the packed, statically asserted 16-byte LlBladeStatus
wire layout in protocol tests (31/31 compiled cases pass); one models an owning
ROS string assignment as borrowed stack storage in calibration. The relevant
files are identical to dev. Localization's 169 compiled tests pass but do not
directly exercise that calibration callback. This analysis does not turn the
cppcheck run green; the failure and original skipped results remain distinct.
No suppression, filter or registered test was changed.

The complete corrected diagnostic missions use ROS source `e93f79d4` plus an
explicit sparse configuration containing the authoritative garden datum and
antenna values. These are distinct from the later automatic launch fix and
new recovery predicate. Both pass live datum/lever-arm and sensor-mode checks.

| Scenario | Captured result | Observed outcome |
|---|---|---|
| GPS-only `gps-frame-config-01` | canonical 0 / outer 0 | PASS, 2,278 s; undock 11.8 s, plan 37.7 s, mow 2,174.9 s, dock/charging 40.0 s; 1/1 paths, 0 wholly skipped, 373.6 m; estimated tracking median 4.6 mm, P95 51.2 mm, max 80.0 mm |
| LiDAR `lidar-frame-config-01` | canonical 1 / outer 1 | FAIL, 2,517 s; mowing fails after 2,425.5 s with no completed path; fallback docking/charging passes in 30.8 s; obstacle encounter/detection/departure passes, ground-truth coverage 94.3%, no boundary violation or <15 cm proximity event; manual/recording/emergency-reset phases pass |

The failed LiDAR cycle retains its safety verdict despite high measured area
coverage. Bounded collision aborts, detour transit failures and final resume
cursor 9718/9757 end in AREA_UNREACHABLE and COVERAGE_FAILED_DOCKING. No timeout,
short-tail tolerance or safety threshold is enlarged. The lack of a legacy
SLAM map and idle-ratio metrics are also reported as failing informational
checks by that harness; they are not upgraded to passes.

Final-source missions use `cfd45403`, the rebuilt 1,875-file first-party overlay,
automatic scenario geometry and no installed robot-config override. Both use
`gps_quality_pattern:='180,RTK_FIXED;20,RTK_FLOAT;10,NO_FIX;3000,RTK_FIXED'`.
The GPS-only scenario explicitly promotes polygon `(2.7,1.2)..(3.3,1.8)` around
the existing Webots box at `(3,1.5)` before START, then verifies it through
GetMowingArea. This exercises known-obstacle handling; it does not add blind
detection. The LiDAR scenario leaves that box unmapped. Both live sensor-mode
and datum checks pass, and both execute RTK_FLOAT -> NO_FIX -> RTK_FIXED with
blade-off guard holds and saved-progress recovery.

| Final scenario | Captured result | Observed outcome |
|---|---|---|
| GPS-only `gps-current-recovery-01` | canonical 0 / outer 0 | PASS, 2,350 s; undock 11.7 s, plan 37.1 s, mow 2,242.5 s, dock/charging 46.4 s; 1/1 paths, 0 wholly skipped, 364.2 m; estimated tracking median 5.1 mm, P95 49.3 mm, max 96.9 mm |
| LiDAR `lidar-current-recovery-01` | canonical 1 / outer 1 | FAIL, 2,591 s; mowing completes in 2,471.5 s, docking/charging passes in 58.3 s; ground-truth coverage 94.4% (51.0/54.0 m²), 406.9 m total; obstacle encounter/detection/departure and manual/recording/emergency-reset phases pass; two boundary violations remain |

The final GPS-only passive trace contains 21,051 samples, including 19,308
receipt-correlated blade-active MOWING samples. None of those active samples
places the reference point outside the lawn or inside the known obstacle;
none of the full trace's sampled modeled bodies overlaps the physical box.
One initial blade-off MOWING-labeled sample remains at the transit corridor,
before entry into the lawn; it is retained. These are sampled observations,
not continuous swept-containment or physical-blade certification.

The final LiDAR trace contains 23,189 samples, including 19,313
receipt-correlated blade-active MOWING samples. One active sample is
outside the lawn; no sampled modeled body overlaps the box. The unchanged
canonical boundary check records two violation episodes. Its legacy SLAM
map-growth and idle-ratio informational checks also fail. Measured coverage
still leaves 1,199 target cells (3.0 m²) uncovered; completion does not certify
every blade footprint or close the cumulative skipped-gap issue. The
passive costmap observer captures nine complete global/local pairs in the
GPS-only mission and 24 in LiDAR; goal coordinates come from rounded logs,
and each later service response is not the selector's original snapshot.

The final LiDAR run retains real ground-truth boundary failures. Independent
review establishes two distinct cases:

- At wall time 1792084603.388, truth remains 10.456 mm inside the west edge,
  while the estimated forward-stamped TF is already outside. FTC correctly
  rejects the command. The later blade-off RPP transit issues `(0.2,-0.941661)`
  at heading 114.91 degrees while truth is 6.10 mm inside; its initial world-X
  velocity is outward at about 0.084 m/s. Truth subsequently crosses by
  1.615 mm. RPP bypasses FTC's polygon command sweep; a legal plan does not
  authorize the controller's executed arc. Available samples do not isolate
  the residual-coast share from RPP's contribution.
- At wall time 1792084999.342, a separate FTC reverse-escape episode has final
  command `(-0.15,0)` and truth X=4.503416 (3.416 mm outside), before RPP starts
  at 4999.737. The corresponding TF X=4.498367 is still 1.633 mm inside;
  its proposed 0.1 s reverse sweep is correctly rejected. The sampled truth
  is X=4.506369 at the first recorded zero-command/blade-off sample
  (wall time 4999.480); source timing is not synchronized. Estimated
  nominal-horizon authorization has not established physical containment
  under pose and response uncertainty. The latest received 3,000 RPM sample
  is receipt-correlated, not synchronized proof of actual blade state at
  that instant.
  Nearby wheel odometry also under-reports the straight truth displacement
  and reports rotation absent from truth. The kinematic body consumes
  `/cmd_vel_wheels`, while wheel odometry integrates simulated joint
  positions. The evidence establishes model disagreement but lacks
  synchronized wheel-command/source timestamps to isolate its cause.

The datum is correct in these runs. The recorded FTC denials are consistent
with their estimated input poses; they do not establish truth containment.
Fixing RPP alone would not resolve the second case. Both are software/
model acceptance blockers; hardware-only wording must not hide them. A
conservative, measured pose/response model and coherent authorization of
transit/recovery exceptions remain necessary. No epsilon, planner inset,
noise setting, expected result, safety threshold or timeout is tuned to make
the tests pass.

A later subscribe-only drive-plant capture closes at its unchanged 600 s bound:
29,997 achievable commands, 29,406 joint states, 28,868 raw odometry and 29,505
truth messages. In a representative 33.90 s straight interval, command
integration is 6.60 m / -2.46 degrees, truth is 6.49 m / -2.43 degrees and raw
wheel odometry is 5.76 m / -42.84 degrees; joint deltas independently reproduce
the odometry yaw. This establishes later divergence between command-integrated
chassis motion and physical joint motion, not the cause of the earlier escape
or the internal Webots/ODE/ros2_control defect. Ground-truth headers use system
time, while command/joint/odometry headers use simulation time; analysis uses
monotonic receipt interpolation, not a direct join of incompatible timestamps.
The 60,179,219-byte trace has SHA-256
`efc3e288259f8fe9c81d3f54eff2316453f7c00f7f459a992d758d733f349301`;
exact receipt intervals, integration method and array-serialization caveats
are preserved in the cache's `drive-plant-live-01/analysis.json`. Neither motion
model was changed to hide the disagreement. This is not hardware evidence.

During the diagnostic GPS-only run, a positive GNSS receipt stamp was rejected
as future by consumers despite RTK_FIXED quality. Code review finds acquisition
stamps preserved in the relay and independent ROS-clock checks in consumers;
asynchronous per-process `/clock` delivery is the source-backed explanation,
not a captured schedule proof (consumer ROS time is not logged). The existing
guard revoked authority, disabled the blade, canceled motion, saved cursor
1346 and resumed it after recovery persistence. No clock tolerance or freshness
check was relaxed.

The no-LiDAR canonical harness measures estimated path tracking and cycle
progress; it does not establish blind-obstacle detection or independent blade
coverage. The LiDAR harness additionally uses ground truth for boundary,
coverage and obstacle checks. Supplemental passive traces record post-mux `/cmd_vel`
commands, truth, BT and charging; receipt correlation does not prove producer
identity, lossless delivery, physical freshness or continuous containment.
Webots' kinematic drive and synthetic charger cannot certify contact dynamics,
physical braking, blade safety or real charging.

Independent strong review found no new actionable source finding in these
continuation changes. The replay contract blocker and the broader unresolved
#924 acceptance below remain open. **HARDWARE_REQUIRED** continues to apply;
use the exact-baseline, supervised, blades-removed procedure in
[Physical acceptance](#physical-acceptance), including the final ROS/image,
firmware, GNSS driver/submodule/receiver, robot and configuration identities.

The hardware candidate software baseline is ROS motion source `cfd45403`,
base dev `29eaf625`, universal-GNSS gitlink
`9a329cc216ed137d0e375a4cfe1f5c26e1dd52d0` and firmware source tree
`a63eb82bbf6b6c0e73fb03883e340aabab6a2f7f`. No target ARM image, flashed firmware
image, named robot unit or receiver/driver hardware revision was supplied or
validated. These identities must be acquired and frozen before any field
measurement; the desktop runtime digest is not a tested robot image. Measure
the actual command-plus-STOP response against the configured
`FollowCoveragePath.obstacle_coast_time_bound_s` (default 0.5 s integral bound)
and fail acceptance if the measured response exceeds that assumption or crosses
the declared geometry. Do not reduce the configured bound to obtain a pass.
Unimplemented #924 command paths still prevent a stack-wide acceptance claim.

Temporary logs, source/build manifests, replay reports and mission traces are
outside version control under the task's local validation cache. They are not
CI or hardware evidence; the counts and limitations above are the retained
cross-package integration summary.

## Baselines and checkpoint

- Branch: `feat/924-motion-authorization`, isolated worktree `.motion-924`.
- Draft PR: [#950](https://github.com/mowglinext/mowglinext/pull/950), against
  `dev`; no merge, ready-for-review transition or physical deployment performed.
- Current base after the requested rebase: `dev`
  `50b1c361b1bf8d51c9ce95b64a7ab77f9efa9fc8`.
  The initial implementation and historical evidence used
  `9f39497f26f4759a0b26e601904710fb934beb65`; earlier results do not validate the
  rebased runtime. Fresh core integration results are recorded below;
  rebased mission evidence and its remaining limits are recorded below.
- Original #942 regression source: `ddd49d9f59e202c265d5aca5e0d1154b79a721af`.
  Its branch and Draft PR were not modified. The original assertions and
  scenarios were retained; additional tests and real map-publication fixtures
  supply the geometry required by the production interface.
- Initial authorization production checkpoint: `26926295`. Native checks passed:
  navigation 22/22, behavior 46/46 and map 12/12 CTest targets. Later changes
  record test latency, exercise automatic map startup and update documentation.
  Later production increments and their separate baselines are described below.
  Packaging checkpoint `0f66cd8c` adds Boost headers and visualization_msgs to
  the five thin sensor build dependency lists after CI exposed missing Boost.
  Test-only checkpoint `8671b3a4` verifies automatic permission publication;
  its expanded map suite passed under Fast DDS and CycloneDDS.
- #905 merged on 2026-10-13 and #906 on 2026-10-10. This branch retains its
  #905-compatible raw MarkerArray publication and segment logic. The requested
  rebase integrates current dev's planning-grid invalidation, dock datum
  identity, legal sharp-corner pivots and collision-monitor hold recovery.
  Conflict resolutions preserve the added mowing/navigation purpose labels,
  strict failed-load cleanup and final FTC authorization. Fresh validation is
  recorded below with the source-mount limitation and remaining mission gaps;
  no PR merge is claimed.

### Rebased software checks (2026-10-14)

The following table records ROS source baseline `daa6b402`. The later
recovery-acquisition increment has separate results below.

Fresh UID1000 Release builds against the new interfaces install compile and
install controllers, map, behavior and coverage. An independent integration
review found no concrete source blocker. Two new corner controls preserve
eligible pivots at actual vertices and retain supported fillets when pivots are
rejected. The paused-clock parameter-startup test now waits, within its existing
three-second deadline, for the nonempty wall-timer publication after current
dev's intentional initial revocation. Its validity and interior/exterior
permission assertions remain unchanged; a nonempty malformed snapshot still
fails. No original #942 assertion or scenario was changed.

| Package | CTest targets | GTest cases | Current result |
|---|---:|---:|---|
| Nav2 plugins | 26/26 | 368/368 | PASS; fresh installed plugin index selected |
| Map | 12/12 | 228/228 | PASS after startup-observation correction |
| Behavior | 54/55 on source mount | 582/582 | Mounted `cpplint` timeout; unchanged registered check passes on identical local source/config |
| Coverage | 16/16 | 156/156 | PASS, including both new pivot controls |

These are 1,334 passing GTest cases and 108/109 passing CTest entries on the
Windows source mount. The remaining registered `cpplint` check passes in 4.73 s
on a byte-identical builder-local package with the same ancestor `ros2/CPPLINT.cfg`
and unchanged 120-second timeout. This validates the check without claiming
the mounted suite passed 109/109. Earlier failed discovery, paused-clock startup,
14 structural XML assertions and both lint timeouts remain recorded. Windows
checkout CRLF bytes caused the structural XML comparisons to fail; restoring
only LF line endings leaves the Git blob and assertions unchanged and the
actual source/installed XML is compared. Clang-format18 whole-file and changed-line
checks pass. A bounded trace confirmed mount I/O wait during lint file reading.
An earlier local probe omitted the ancestor config and reported 94 warnings;
that probe did not use the repository's configured rules. Both probes and
timeouts remain recorded. No filters, assertions or timeouts were changed.
Standalone interfaces have no registered test suite here.

The isolated builder required the matching HDF5/OpenMPI development package
`1.14.6+repack-2` for current dev's Smac exported headers. Only system package
installation used root; all project builds/tests/installations used UID1000.
Actual C/C++ compilers remain GCC15.2.0; core runtime dependency checks report
no missing libraries and HDF5 serial runtime linkage. This modified SDK and its
package provenance must be distinguished from the earlier immutable image;
fresh mission runtime dependencies and source/install identities are recorded
below. No hardware validation has run.

### Rebased Webots mission evidence (2026-10-14)

LiDAR-enabled attempt28-02-01 executed the actual production stack at source
`daa6b402be48816820b858992abb8ac416cefad0` on the rebased dev above. All twelve
first-party runtime packages were rebuilt and installed as UID 1000; the pinned
prefix manifest covers 1,699 files. Webots R2025a and the external ROS driver,
rootfs and dependency bytes were recorded, including modified-driver status;
their complete source-image provenance remains unverified. The unchanged
canonical harness filename is `e2e_test_no_lidar.py`, but explicit config and
live checks establish that this run had LiDAR enabled.

The canonical harness reports PASS: undock 11.5 s, planning 38.5 s, mowing 2348.6 s,
dock/charging 44.0 s; total 2459 s. It reports 1/1 completed paths, 0 whole paths
skipped and 378.1 m. Nearest-path errors over 56,786 samples are median 7.1 mm,
P95 61 mm and maximum 80 mm. These establish observed software progress, not
continuous containment or complete blade coverage. FollowStrip explicitly
reports **260 poses / 11.4 m of planned path left unmowed by ten obstacle detours**.
A separate non-obstacle abort advances its blade-off resume cursor 152->161
poses (~0.80 m); later saved cursors 2305 and 7285 are recorded. Cycle PASS must
not be read as full coverage PASS, and retry spans must not simply be summed.

One outward near-boundary intended FTC command remains rejected, followed by
abort and blade-off transit. The final-output recorder was ready before START;
319,033 message receipts include 120,544 final wheel commands and one stamp-zero
raw rectangle/corridor snapshot, with no holes. A receipt-near zero wheel command
follows the denial log by 12.97 ms. Different source/receipt clocks and missing
owner identity prevent treating that interval as a stopping bound or cancellation
proof. Terminal/count consistency does not establish lossless delivery,
acquisition freshness, geometry revision/epoch, or continuous containment.

Canonical harness and inner cleanup record exit 0; the original host collector
records **null**. The authoritative outer Docker/GNU-timeout exit remains
unknown and must not be inferred from cleanup status. A separately reviewed
collector now rejects missing status before integer conversion and passes
real exit 0/31 and missing-status fault tests. That repair does not retroactively
validate the old outer status. Recorder shutdown tracebacks after its terminal
record and original prestart/setup failures remain retained.

GPS-only attempt28-03-01 launched the current fleet-protected no-LiDAR overlay
but stopped **before canonical START**, with captured outer exit 1. Direct
readiness saw collision_monitor active; the subsequent CLI lookup reported
`Node not found`. No crash/deactivation was logged, but the exact CLI discovery
cause is unconfirmed. No GPS-only mission or final-output recorder ran on that
attempt. A new isolated attempt uses fresh typed service responses for those
checks while preserving the active lifecycle and fleet/controller assertions.

The subsequent direct-service GPS-only attempt28-04-01 also stops before
START, with captured outer exit 29. Its raw collision-monitor log explicitly
reports `Failed to get parameters: PolygonStop.enabled`; requesting that
undeclared inactive-polygon setting makes the entire parameter batch empty.
This is a private validation-contract defect. The corrected probe requests
only active settings while retaining an exact active polygon list of
`[FootprintApproach]`, rejecting any extra PolygonStop, and retaining the
source/installed disabled-setting assertion. Active lifecycle, fleet source,
controller, geometry/datum and recorder guards are not relaxed. Test-only mock
service endpoints use a separate DDS domain from the real mission. No mission
result is inferred from either prestart failure or the helper transport tests.

GPS-only attempt28-05-01 actually starts the canonical harness at
2026-10-14T03:42:32Z (PID 2632), at the same runtime source HEAD `daa6b402`.
Its direct service gate confirms active collision_monitor ID 3 / label `active`,
six correctly typed active settings, the exact FootprintApproach/fleet_peers
lists, controller collision flags, disabled LiDAR anchor and enabled local/global
fleet protections. Datum, tool/fixture/actuation readiness and observers pass;
final-output recorder PID 2540 is ready before START. The canonical mission
then reports **PASS**, with an independently captured outer Docker exit 0:
undock 11.8 s, planning 40.8 s, mowing 2275.7 s, dock/charging 42.3 s; total
2390 s. It ends IDLE_DOCKED and charging, reports 1/1 completed paths, zero
wholly skipped paths and 371.3 m. Nearest-path tracking over 54,220 samples is
median 3.9 mm, P95 45.2 mm and maximum 80 mm. The full stack records 16 pivots,
no FTC authorization denial and no resume-cursor or unmowed-span accounting
message in this run. Four obstacle-recovery waits resume within the active
action. The goal checker accepts index 9740/9756 with 0.480 m remaining and
0.493 m endpoint error; those path-tail measurements are not a measured unmowed
area. This observed GPS-only pass is a positive functionality control; it does not establish physical blade coverage, blind
obstacle detection or continuous containment. The raw recorder ends with
305,978 message receipts, cap/timeout false and ready true; offline decoding
passes with matching terminal/topic counts. Receipt metadata does not prove
ownership, acquisition freshness or lossless delivery. Later documentation
commits do not change the running
ROS baseline `daa6b402` used for this completed mission.

LiDAR repeat attempt28-06-01, on the same ROS source baseline `daa6b402`,
reports **FAIL**, with authoritative outer exit 1 and no outer timeout. Readiness,
scan/Collision Monitor, signed catcher, datum and pre-START recorder gates pass.
Undock 11.4 s, planning 39.3 s and fallback dock/charging 41.1 s pass, but mowing
fails after 2464.2 s: 0/1 completed paths, zero wholly skipped; total 2570 s and
378.8 m. Median tracking error is 7.2 mm, P95 61.8 mm over 58,260 samples.
The final persisted cursor is 9738/9757. Four AREA_UNREACHABLE transitions lead
to COVERAGE_FAILED_DOCKING. The earlier LiDAR cycle PASS is therefore not stable
whole-mission acceptance. Its original missing outer exit remains unknown.

The terminal chain is a genuine FTC forward chassis/response-sweep lethal-cell
abort followed by a blade-off detour whose target the transit planner rejects
as GOAL_OCCUPIED. The last displayed suffix spans 0.539567 m; it cannot satisfy
the existing 2 m blocked-stretch skip or 0.60 m remembered-failure exclusion.
No safe alternative is established, so those policies and the authorizer remain
unchanged. Source-world ideal geometry is not a substitute for captured costmaps.

Independent review confirms a separate production recovery defect: FollowStrip
only consumes full global OccupancyGrid messages while the configured publisher
normally sends deltas after initial/geometry-change publication. Its cached grid
can therefore miss added or cleared obstacles. The observed stale-clear detour
is consistent with this defect, but exact causality for the last suffix is not
proved because raw costmaps were not recorded. The request-scoped GetCostmap
repair is implemented at `042329de`; its separate LiDAR mission is recorded below.
The failed repeat's 326,763 decoded CDR receipts have consistent terminal counts;
ownership, acquisition freshness, lossless delivery and continuous containment
remain unestablished.

### Recovery acquisition increment (`042329de`)

FollowStrip now requests a new full global GetCostmap response for each detour
decision. Pending acquisition preserves the failed action episode with the
blade off; a three-second steady-clock bound covers discovery and reply.
Halt, restart, fleet yield and unit changes remove the pending request, so a
late response cannot dispatch a replacement goal. Missing or malformed data
retains reached progress and leaves the unit incomplete instead of inventing a
clear path or advancing the non-obstacle cursor. Valid current data uses the
existing detour distance, clearance, attempt and blade-off transit policies.
No new velocity publisher or geometry-permission bypass is introduced.

The same public action/service regression fails on matching old production
source and headers: the retained-clear grid selects target X 0.8 m despite the
current obstacle; the required target is at least 1.5 m. The repaired production
path passes. Eighteen new cases cover add/remove, valid continuation, invalid
metadata, missing/delayed responses, halt/restart/fleet ownership, scan restoration
and exact reciprocal rounding at the cell-index limit. An independent strong
review found that rounding defect; it was corrected and its public-path test
passes. Existing near-end and rejoin controls retain their assertions and now
provide the real fresh-map service used in production.

Fresh UID1000 behavior results are **600/600 GTests**, zero disabled/error cases,
and **55/56 mounted CTest targets**. The sole mounted failure is the unchanged
120-second cpplint I/O timeout. Its registered check passes in 4.79 s on all
109 byte-identical package files with the exact ancestor CPPLINT.cfg; the
registered copyright check also passes. Earlier fixture/license diagnostics
remain recorded. Clang-format18 whole-file and changed-line checks pass, and
the new behavior install matches source headers with no missing ELF dependency.
The prior runtime prefixes for other packages are unchanged.

This repair establishes request ownership and validated response data. The
upstream service response is not an atomic costmap-generation certificate, an
execution permission, or a physical observation-freshness guarantee. Planner,
FTC, collision and firmware guards remain necessary. The completed missions
above retain baseline `daa6b402`; the new-source result follows separately.

### Current-source LiDAR mission (attempt29-02, 2026-10-14)

Source `042329de3cd78d1b233eb3d8448f4eed4b66ecef`, START documentation snapshot
`d674f6e5`, uses a fresh behavior install and eleven byte-identical prior package
prefixes. The complete twelve-prefix audit covers 1,700 files. The unchanged
canonical harness starts only after actual LiDAR/collision, datum, pose/tool,
actuation and recorder readiness. Attempt29-01 first stopped before START with
captured exit 38: the private observer used a reserved rclpy Node property.
Corrected actual node construction and two typed mock service checks pass;
that idle attempt has no mowing result. Failed artifacts remain preserved.

Fresh attempt29-02 records canonical **PASS and authoritative outer exit 0**,
without watchdog or timeout. It reaches IDLE_DOCKED and charging at 2554.8 s:
undock 12.0 s, planning 38.2 s, mowing 2446.5 s and dock 44.6 s. It drives 386.0 m;
60,970 nearest-path samples have median 6.2 mm, P95 62.7 mm and maximum 87.4 mm.
These demonstrate observed progress through the cycle, not full blade coverage.

Adverse functionality evidence remains: **15 detours, six transit timeouts and
13 failed-transit log messages**. A reproducible ledger sums 434 logged gap
poses and 17.39 m of rounded per-dispatch left-unmowed arc estimates; this is not
unique spatial unmowed area. Separate non-obstacle cursor advances and blocked
start skips also occur. Three actual FTC authorization denials are retained.
Near the end, the log reports skipping the rest/all one swath, then starts a
new pass at cursor 9741/9757 with 16 poses. The unchanged goal checker accepts
that short tail at 6.7% progress, 0.408 m remaining and 0.486 m endpoint error.
The final-pass summary therefore reports 1/1 completed and 0 skipped. Independent
strong review confirms expected per-pass retry semantics; earlier blade-off
gaps were not proved reclaimed. FollowStrip resets its local gap tally per
pass, and the published skipped counter is separate. Final 100% cursor progress
and unit completion must not be read as complete cutting coverage.

The sealed main CDR has 343,093 receipts and 267,646,613 bytes, 788,843 bytes below
the 256 MiB cap, with ready/terminal counts reconciled and no cap/timeout.
It includes 130,002 wheel-command receipts, demonstrating observation after
selection/shaping. It does not establish owner/action identity, lossless
delivery, acquisition freshness or continuous containment.

A separate bounded observer records actual typed GetCostmap request/response
CDRs: 254 global and 254 local replies, zero failed exchanges, integrity PASS,
29,117,384 bytes under 128 MiB. Both services answer before START; 253 later pairs
cover live cursor progress 90.017% through 99.836%. All 508 response payloads
also decode successfully. These independent requests are not correlated with
the grids consumed by individual FollowStrip decisions, and are not atomic
producer-generation or physical freshness certificates. Global/local observer
RPC medians are 1.143/0.976 ms (P95 2.454/1.942 ms). They are software reply timings,
not controller deadlines. Whole-container samples show 1.707-1.710 GiB and
557-613% multi-core CPU for Webots, ROS and observers together; no isolated
authorization overhead or identical-host baseline is measured.

### Current-source GPS-only mission (attempt29-03, 2026-10-14)

The strict direct no-LiDAR/fleet/FTC/collision runtime gate and both typed
costmap proofs pass before START on source `042329de` and documentation snapshot
`d674f6e5fc719cc2886663a64a92bea4e51ed398`. LiDAR consumers and map anchoring
are disabled; fleet, local-static, controller and collision protections remain
active. This mode does not establish blind-obstacle detection.

The canonical cycle **PASS and authoritative outer exit 0** are confirmed,
with no watchdog or outer timeout. The BT reaches `IDLE_DOCKED` and charging
at 2,362.8 s: undock 12.0 s, plan 37.8 s, mowing 2,259.8 s, docking 43.0 s.
Travel is 371.9 m. Tracking has 55,899 samples: mean 11.8 mm, median 4.4 mm,
P95 50.1 mm, maximum 79.9 mm. Three `WAITING_FOR_RTK` episodes return through
preflight, heading calibration, planning/transit and mowing. These are
simulated recovery results, not GNSS receiver or field validation.

Final status reports 1/1 completed paths, zero wholly skipped, no emergency
and 21,985 stamped cells. The 100% cursor and 54.9625 square metres of raster
cells do not establish the geometric residual on the 54-square-metre lawn.
The harness has no separate final geometric tail gate. Six observed plans
contain 9,757, 8,432, 6,666, 5,607, 1,194 and 138 poses; the stream carries no
action-owner identity and is not an independently certified execution cursor.
Complete blade coverage and continuous commanded containment remain unmeasured.

Typed costmap CDR integrity passes for 155 global and 155 local replies:
one pre-START proof and 154 final-ten-percent samples per service. Recording
uses 17,770,325 bytes below 128 MiB. Global/local observed reply medians are
1.290/1.172 ms; P95 3.046/3.722 ms. These independent replies do not establish
pair atomicity, producer generation, physical freshness or action alignment.
The main final-output capture is sealed at 245,089,864 bytes below 256 MiB;
its cap and timeout flags are false. A network-disabled UID1000 analyzer
decodes 313,172 receipts with matching terminal counts, including 119,044
wheel commands: 113,813 nonzero, 1,952 reverse and 355 rotation-only receipts.
The modeled post-firmware wheel extrema are 0.470578 m/s and 2.213331 rad/s;
these are not FTC source-command extrema or validated hardware limits.
Sampled ground-truth distance is 392.667 m, separate from the harness's
371.9 m and potentially including jumps. SHA-256
`15a90cc7e54f3a857fa24797e271b78a62043d7114fe11c4efd6dcf18e12858b`
matches before and after decoding. This establishes software receipts,
not continuous containment, action ownership, physical freshness or lossless
delivery. Earlier attempt28-05 retains source `daa6b402` as a separate baseline.

### CI integration checks and remaining planner gate

The [ROS CI run on documentation snapshot `00fa373f`](https://github.com/mowglinext/mowglinext/actions/runs/38539099057)
passes formatting, configuration drift and static analysis but fails the broad
test job: two new launch cases expected the old no-LiDAR collision-monitor
pass-through, and the simulator package reports three Python indent/import
lint errors. Commit `f434acd4` repairs those tests to require current dev's
active fleet approach polygon and peer source, and fixes formatting. All 19
affected Python regressions pass in the real ROS container as UID1000; the
unchanged simulation `ament_flake8` check passes all 16 files. The relay's AST
matches mission source `042329de`; its only runtime-source change is whitespace.
No production configuration, original #942 assertion or motion guard changes.
These local repairs are not a claim that the subsequent CI run is green.

The [planner replay run](https://github.com/mowglinext/mowglinext/actions/runs/38539099114)
fails Release, Debug and RelWithDebInfo and has a separate unresolved
strict-output gate. The retained RelWithDebInfo
artifact compares its actual synthetic merge `25f13a28` against base parent
`d125e924`: only **5/26 exact baseline outputs match**, while all **26/26**
cases are deterministic, use identical fixtures and preserve trace passivity.
Each case has 15 baseline, 15 untraced candidate and 100 traced repetitions.
An independent rerun of the unchanged comparison returns exit 1. All boundary
and kept-swath stages match in the retained snapshots, as do swath/ring/subpath
counts, join statistics, blade-off transits and endpoint-transit lengths.
Microscopic-bevel smoothing changes execution geometry, quaternions, pose
counts and path lengths. No coverage-package change separates `50b1c361` from
this newer CI base. This explains the exact-identity mismatch without proving
all changed trajectories safe or desirable. The gate remains a merge blocker;
no expected outputs, cases or `--require-identical` requirement are weakened.

This CI x86_64/GCC15.2/F2C3.0.0 comparison measures median per-case planner
time +0.580%, connector time +0.501%, process time 0% and RSS +0.105%.
Those measurements do not establish controller-loop or target ARM deadlines.

### Deferred recovery target and accounting gaps

Independent investigation reproduces another existing functionality mismatch:
a fresh raw 253/raw 255 target center can pass FollowStrip's preserved lethal-only
disc test, while AuthorizedTransitPlanner rejects center costs >=253. A private
central-cell candidate preserves body-only inflations and initially passes 21
fixture cases. Strong review finds an adjacent-cell precision defect: service
resolution is float32, planner resolution double. With origin (-1, -2), goal (0.8, 0)
and true resolution 0.1, actual Costmap2D::worldToMap selects (18, 20), wire metadata
selects (17, 19). Two real-API boundary comparisons fail the candidate, showing
both false clearance and rejection of a legitimately clear target.

The candidate is **not integrated**. Rejecting all ambiguous boundary cells
can regress legitimate recovery; a producer precision/eligibility contract and
positive boundary controls remain needed. The current planner retains its own
authoritative rejection; this is not a demonstrated motion bypass or proof of
exclusive mission causality. All failed probes are retained. Cumulative gap
accounting across retries is also open; neither the canonical pass nor these
probes establish complete coverage. Original #942 assertions remain unchanged.

No paired unmodified-dev run, repeated reliability, connected multi-area
coverage, hole/revocation mission, non-default docking, explicit HOME,
charge-resume, or physical acceptance is established by these new results.
Earlier failures remain pinned to their own source/runtime. No unique causal
attribution of improvement to the implementation versus the dev rebase is made.
Independent review confirms this evidence scope; the full #924 contract remains
incomplete. Private checkpoints/raw artifacts retain exact script/input hashes.

## Authorization policy

The reference point is the rear axle, `base_footprint`; `base_link` has the same
XY origin. FTC coverage commands keep that point in the recorded mowing union.
Blade-off Escape may use the recorded mowing, navigation and dock-corridor
union. A navigation corridor cannot grant FTC coverage permission. Body
extension beyond an outer boundary remains permitted by the axle policy and
bounded by the actual configured chassis. Raw forbidden holes and dock bodies
never permit overlap with the complete swept chassis. Their planner clearance
margins are not applied a second time to a complete chassis.

The shared verifier checks a constant outgoing Twist, including the entire
reference-point path and chassis sweep. Convex endpoint hulls padded by an
angular arc-to-chord bound enclose intermediate body positions. Analytic
circular-path intersections split axle arcs at polygon supporting lines;
every resulting interval is classified against the raw allowed union. Straight
paths split at polygon crossings, using exact binary-rational predicates for
ambiguous determinants and fractions. Strictly disjoint edge bounding boxes
and certified interval exclusions skip irrelevant edges without discarding
possible crossings. The point-on-boundary tolerance is 1e-8 m; it is not
localization uncertainty or permission to cross an obstacle.

Snapshots validate frame, marker purpose, topology, finite coordinates and
footprints before use. Limits are 512 markers, 32768 total points, absolute map
coordinates at most 1e6 m, and a finite 3-64-vertex chassis of radius at most
3 m. Content defines snapshot identity independently of cached publication
stamps. Empty or invalid snapshots revoke these consumers' permission. This
identity is not a producer epoch, edit sequence, map datum or action lease.

FTC's common final output gate caps and checks every nonzero return: nominal
tracking, deviation, reverse, turn fallback and rotation. It obtains a fresh
map-frame robot pose and the costmap's configured footprint. PRE/PIVOT/POST
states also validate the complete remaining directed rotation using the same
wrapped heading error as the controller. Each translational command verifies
0.1 s of intended execution, corresponding to the shipped 10 Hz controller.
The interval is not a final actuator lease or a guarantee against downstream
command retention. Zero commands bypass all geometry and pose prerequisites.
Raster costs can further reject commands; free or soft-cost cells cannot grant
polygon permission.

Escape consumes one existing fresh arming token, verifies blade-off feedback,
and moves opposite the validated last direction. Defaults remain 0.10 m/s,
0.40 m and 6 s; compiled ceilings remain 0.15 m/s, 0.60 m and 15 s. It validates
the complete straight configured envelope before starting, then checks current
geometry identity/generation, fresh pose, remaining sweep, measured/commanded
distance, 0.05 m lateral drift, 0.15 rad yaw drift and absolute monotonic timeout
every tick. A tick gap above 0.5 s ends the attempt. Each outgoing command also
certifies 2.5 s of intended held translation and tapers to remaining distance /
2.5 s. Full-speed budget accounting can end the nudge short of the configured
maximum. Recovery efficacy and firmware deadband behavior need further
integration and hardware evidence. Halt and rejection publish zero.

`motion_footprint` is an internal behavior-node parameter derived by both full
launch builders from the existing `chassis_footprint` helper, including its
existing 0.05 m navigation clearance. An ad-hoc node without the derived
footprint cannot perform Escape. No operator YAML setting is added. Existing
firmware emergency stop, blade/localization guards and collision processing
remain in their existing paths.

Map replacement revokes before fallible live mutation. Invalid reloads clear
partial replacement state and publish empty authorization; correcting the file
and reloading restores normal permission. A missing file before mutation
preserves unchanged live geometry. Rings, counts and indexed records are
validated strictly. All valid persisted and parameter holes are retained,
including coincident-centroid and duplicate rings. Orphan obstacle-array
entries reject startup; omitted trailing entries still mean no holes. Invalid
add-area requests reject without altering the live map.

## Motion-path acceptance matrix

| Producer / path | Command path and authorization in this slice | Evidence / remaining acceptance |
|---|---|---|
| FTC swaths, connectors, headlands, deviations | Raw mowing lattice critic plus `computeVelocityCommands` common final gate -> Nav2 `/cmd_vel_nav`; forward lattice output also checks closed lethal-cell response sweep | 26 command cases, including original 12 negative and 4 positive controls; 21 critic and 18 real-controller cases. Strong modeled obstacles safely abort; rebased LiDAR cycle passes with known unmowed spans; continuous containment remains OPEN |
| FTC PRE/PIVOT/POST and fallback reverse | Directed full rotation and final Twist sweep; existing costmap collision checks retained | Real-controller wheel-lag turn scenarios and complete footprint tests; passed on the final production source |
| Webots mission cycles | Actual rebased stack, audited twelve-prefix runtime, strict live mode checks and canonical E2E | Source `042329de`: LiDAR29-02 and GPS29-03 canonical/outer PASS; LiDAR has cutting gaps and final-pass counters, GPS has three handled RTK waits. Source `daa6b402`: GPS28-05 PASS and LiDAR28-06 FAIL remain separate baselines. Full blade coverage, continuous commanded containment, explicit HOME and charging-resume remain unverified |
| EscapeStartBlocked | Bound action instance -> `/cmd_vel_nav`; geometry, pose, blade, arming and budgets; rejection/halt zero | 17 production-path DDS/command cases; passed on the final production source |
| Map permission production | #905-compatible `/map_server_node/transit_geometry`; `area.text=mowing|navigation`; revocation before replacement | 30 startup/reload/add/parameter cases; passed on the final production source |
| Nav2 RPP/RotationShim transit and HOME | BT `NavigateToPose` -> Nav2 -> `/cmd_vel_nav` | Default staging transit completed in attempt 17. No new execution gate; #905 protects planning, not controller corner cutting. Explicit HOME interruption untested. OPEN |
| Nav2 BackUp, BT boundary recovery and undock | BT `/backup` and recovery action paths -> Nav2 -> `/cmd_vel_nav` | Existing behavior retained; no bounded geometry exception implemented here. OPEN |
| LiDAR collision catchers | Nav2 `/cmd_vel_nav` -> Collision Monitor -> `/cmd_vel_monitored`; signed forward/reverse/rotation regions with unchanged live-footprint guard | Rear-clear reverse trap reproduced and repaired; 27 isolated real-node cases and 10 registered integration tests pass. Continuous geometry authorization and complete LiDAR mission acceptance remain OPEN |
| Dock and dock calibration | `/dock_robot` and independent docking commands -> `/cmd_vel_docking` -> mux | One default simulated dock/charging cycle passed. No new execution gate; non-default distances, calibration and charge/resume remain unverified. OPEN |
| Motion heading seed / yaw / IMU / magnetometer calibration | Independent publishers, including `SeedYawFromMotion` -> `/cmd_vel_teleop` | Sharing the teleop lane does not make an autonomous calibration manual. OPEN |
| Dig recovery | `hardware_bridge_node::dig_monitor_tick` -> `send_cmd_vel_packet`, bypassing ROS mux | Existing limits retained; no geometry execution gate. OPEN |
| Selection, shaping, final motor packets | Collision monitor -> twist_mux -> bridge slew/deadband shaping -> firmware; bridge also has direct sources | FTC/Escape validate their intended source commands. No owner/action/revision lease after shaping or final-output integration evidence. OPEN |
| Manual/operator and STOP | Existing teleop/tuning/emergency lanes; firmware emergency latch remains authoritative | No new manual geometry clamp; unconditional source STOP preserved. Final arbitration/cancel ownership unverified |

Source maps: [navigation controllers](../ros2/src/mowgli_nav2_plugins/src/ftc_controller.cpp),
[behavior navigation/recovery](../ros2/src/mowgli_behavior/src/navigation_nodes.cpp),
[calibration](../ros2/src/mowgli_behavior/src/calibration_nodes.cpp),
[hardware bridge](../ros2/src/mowgli_hardware/src/hardware_bridge_node.cpp),
and [mux priorities/timeouts](../ros2/src/mowgli_bringup/config/twist_mux.yaml).

## Verification record

Original regression baseline was reproduced on exact dev plus unchanged #942
sources in a separate non-root ROS build: 12 FTC negative cases and the one
Escape negative case failed; four positive FTC controls passed.

Final-source navigation, behavior and map builds passed all **80 CTest targets**
(22 + 46 + 12), containing **968 GoogleTest cases** (263 + 484 + 221).
The focused suites contain 26 FTC command cases, 18 continuous
sweep cases, 17 Escape cases and 30 map startup/reload/add/parameter cases. All original
13 negative cases now pass and all four original positive FTC controls remain
green. Additional positive checks include real-FTC edge progress, 1000 legal
straight sweeps, 1000 curved headland sweeps around a hole, complete pivots,
connected polygon seams, and valid omitted trailing parameter entries.

The real FTC wheel-response fixture passed **100/100 scenario executions**:
20 repeats each of hedge U-turn recovery, fallback-disabled U-turn abort,
straight-wall abort, obstacle-during-recovery abort and no-LiDAR turn completion.
Each repeat contains two completions and three expected safe aborts, rather
than five completed missions. All executions report zero fixture lethal
samples. The unchanged dev/#942 baseline also passed all 100 executions of
those same existing fixtures. This establishes preserved fixture outcomes;
it does not measure full mission coverage or final actuator containment.

Fast DDS was used for the complete native suites. The four focused suites
(FTC, sweeps, Escape and map reload) additionally passed through CycloneDDS:
4/4 CTest targets originally containing 89 cases. The later expanded 30-case
map suite also passed under both implementations, with all 12 map-package
CTest targets rerun successfully. The two new cases exercise actual automatic
startup publication without `/map`: persisted geometry and the simulation's
parameter-defined lawn with a paused ROS clock. They neither invoke the
test-only mask rebuild nor substitute authorization evidence.

Host latency measurements below used production `26926295` and test-only
instrumentation `37af533e`. The dense-circle straight case previously averaged
226.411 ms before the certified edge exclusions, exceeding the shipped 100 ms
controller period. That comparison is to the prior implementation on the same
fixture, not an unmodified dev verifier: dev has no equivalent swept-geometry
verifier. No timing assertion is used to hide host scheduler variation.

| Geometry check | Samples | Mean | Largest observed check |
|---|---:|---:|---:|
| 32768-point circle, straight | 100 | 0.280 ms | 0.440 ms |
| Same circle, near-straight (`omega=1e-12`) | 100 | 3.85 ms | 5.17 ms |
| Same circle, curved (`omega=0.2`) | 100 | 0.845 ms | 1.393 ms |
| 32768-point rectangle, interior | 100 | 0.284 ms | 0.547 ms |
| Same rectangle, legal edge overhang | 100 | 0.225 ms | 0.350 ms |
| Complete 2-pi pivot with 49 holes | 3 | 10.45 ms | Not separately instrumented |

Repeated full-controller fixture means ranged from 0.033 ms (no-LiDAR overlay)
to 3.41 ms; the largest observed tick was 48.75 ms. These include existing
controller/costmap work. They are container/x86 host observations, not ARM
deadline guarantees or whole-stack CPU/memory overhead measurements. Across
20 runs, whole five-scenario GTest durations averaged 2280.9 ms on baseline and
2596.15 ms on the implementation. This includes setup/teardown and scheduler
variation; it is not isolated verifier CPU overhead or per-tick latency.

One additional paired process probe used `os.wait4` to obtain resource usage
for each actual test PID, avoiding inherited shell child-accounting. The five
controller scenarios passed again: baseline CPU user+system 2.441 s, peak RSS
58.6 MiB; implementation 2.959 s, peak RSS 60.1 MiB. The geometry-only 18-case
process passed with 0.824 s CPU and 32.2 MiB peak RSS. These single-process
samples include fixtures/frameworks, are sensitive to scheduling, and do not
isolate verifier overhead or predict a deployed stack's memory consumption.

Configuration/launch/URDF checks: **253 passed**, including both real full-system
launch builders with non-default chassis dimensions. These use lightweight
ROS launch adapters and execute the production builder code; they do not start
Nav2 or Webots. Firmware's 24 generated interface headers were checked, and Go
and TypeScript consumers regenerated with no drift in Linux under `LC_ALL=C`.
The additional existing OpenMower launch checks passed **11/11**. Repaired
sensor workflow YAML and all five explicit build dependency lists were checked.
Clean sensor CI at `0f66cd8c` passed GNSS (11 actual GoogleTest cases and
9 Python cases) and OpenMower (78 actual GoogleTest cases and 11 Python cases),
including newly generated interfaces and installed-consumer builds. OpenMower
amd64 and arm64 image builds and smoke tests also passed. The CI counter that
greps every XML `tests` attribute counts nested attributes twice; the case
counts here use GTest's actual executed-case output. The [full ROS workspace
Build & Test job](https://github.com/mowglinext/mowglinext/actions/runs/38014488887/job/114101679699)
also passed at test checkpoint `8671b3a4`: 14 source packages built and tested,
with colcon reporting 3107 tests, 0 errors, 0 failures and 369 skipped. That colcon total
includes framework/linter results and is not an additional GoogleTest count.
Despite the job's historical `ROS2 kilted` name, its actual setup is Lyrical.

Native builds use Docker image `be2f6e79c62e`, Ubuntu 26.04, ROS Lyrical,
`/opt/lyrical_vendor` and UID/GID 1000:1000. Generated message interfaces are
unchanged; source packages and added shared headers are rebuilt separately
against the image's cached generated-interface prefix. Full clean workspace,
coverage-planner package and physical hardware builds are not established by
those local focused checks. The separate CI workspace build covers the source
ROS packages, including coverage; it does not build firmware or prove Webots
missions or physical behavior. Local logs live in `.validation/`, excluded from Git.
Fresh final `mowgli_interfaces` CMake configuration also passed, finding
visualization_msgs and Boost 1.90.0. This validates configuration/export wiring,
not a new generated-message compilation or a clean installed-consumer build.

Independent Sol review found and prompted fixes for inward-arc rejection,
failed-reload revocation, unwrapped remaining rotation, shallow crossings,
malformed rings, count/index omissions, orphan parameter holes and unnecessary
exact arithmetic. Each has dedicated regressions. The final source review of
`26926295` found no remaining confirmed source blocker or should-fix in this
scoped FTC/Escape implementation. Its runtime conditions were checked by the
final native, transformed-seam, dense-polygon and repeated-controller runs.
The reviewer explicitly retained the full-#924 and hardware acceptance gaps.

## Simulation and resource limitations

An unchanged Webots E2E harness was executed, without a completed mission.
After Docker/WSL recovery and moving large
scratch artifacts to D:, Webots R2025a and the repository-pinned driver/control
revision `db4a77c84ee91445d39de9a631e5b57a888814e2` built and ran in a separate
bridge-networked container as UID 1000, ROS domain 170, with no robot devices,
host IPC or published ports. Xvfb, missing runtime libraries and Linux-native
Webots IPC were repaired in task-owned scratch; shared services were restored
after the user-authorized Docker restart and were not used for missions.

The current simulation launch/config/world/robot description and tested native
interfaces/navigation/behavior/map installations were overlaid on cached image
`be2f6e79c62e` (runtime provenance `df018f11`). Untouched components including
coverage and Nav2 still came from that cached runtime: this is a mixed-baseline
integration attempt, not a fully rebuilt release image. The configured
`main_mow` lawn loaded, robot-state publication initialized, and both
`joint_state_broadcaster` and `diffdrive_controller` reported active.

Earlier full-stack attempts failed readiness: docking exited with an undefined
`opennav_docking::Controller` symbol and Nav2 remained inactive. A subsequent
reliable transient-local probe delivered the real configured geometry from
the current map executable/library; `/map` is not required for this wall-timer
publication. Native startup tests establish that separately. The docking
loader failure was traced to case-insensitive Windows bind lookup: Webots'
`libController.so` shadowed ROS's `libcontroller.so`. Corrected loader order
resolves docking symbols while preserving Webots dependencies. Small Webots
controller libraries were staged in executable native `/tmp`, preserving
case-sensitive lookup and Linux IPC. Attempt 13 verified all eight navigation
lifecycle nodes, including docking, active; both drive controllers active;
and the genuine configured geometry delivered from the current map executable.
An initial CLI snapshot exhausted CycloneDDS participant indices; a later
snapshot returned all eight active states. Readiness used explicit node states,
not the generic lifecycle-manager log message.

The unchanged no-LiDAR E2E harness used its documented 3000-second budget,
accepted START, undocked and completed heading calibration. At 60 seconds it
reported approximately 1.0 m traveled, pose `(-0.96, 0.00)`, and zero coverage
path poses. The cached `df018f11` coverage executable generated a 9947-pose,
82-segment plan, then aborted with `double free or corruption (!prev)` and
exit code -6. Its missing heartbeat shut down the navigation group. No coverage
trajectory was followed, and no mowing/HOME/docking completion is claimed.
This mixed-baseline failure does not establish a regression in current
production coverage source; its cause remains unconfirmed.

The owned attempt-13 container was stopped below the 200 MB disk floor. Once
headroom recovered, current-source coverage was built on D: as UID 1000 against
the current interface overlay. All 8 focused CTest targets passed (128 GTests).
The build handoff identifies source `750eb4a1`; the runtime provenance file
does not independently record its Git SHA. The installed coverage executable
SHA-256 was `e0c4cc8409e2fd200d8029f30b9f73b08f92eecc4e2170a4f2ea48fb351fcc6b`.
Mapped-library evidence confirms the current coverage/interface/map overlays;
the schema-affected cached MQTT bridge was not launched. Other cached packages
still make this a mixed-baseline integration run.

Attempt 14h passed explicit readiness for all eight Nav2 lifecycle nodes,
coverage and both drive controllers, and received the real mowing polygon.
The unchanged no-LiDAR E2E harness ran its full default **3000-second budget**
in ROS domain 181. The mandatory session monitor started before START. The
read-only-rootfs task container disabled Docker duplicate logging and stored
the runner, session and pose logs on D:; it is stopped and removed.

Undocking and planning succeeded. Coverage produced 82 segments, 9947 poses
and a 383.8 m route. Four early FTC sweep denials aborted follow goals and
saved resume cursors 57, 118, 212 and 271; subsequent goals resumed and made
progress. The later saved cursor was 8157/9947 (82% of path poses, **not lawn
coverage**). At about 1920 seconds, charging became true during the follow
goal. ManualChargeGuard entered `MANUAL_CHARGING`, canceled motion and held
zero output while awaiting operator undocking. No override was injected.

The mission **did not complete**. At the full-budget timeout, the unchanged
harness additionally raised `AttributeError` from `RcutilsLogger.warn`, exiting
1. Its `Coverage path: 0 poses` metric was unusable despite the actual planned
and followed path; it also reported incompatible `/cmd_vel` QoS. The monitor
recorded 30421 samples over 3043.47 seconds, 319.66 m integrated wheel travel,
and final state `MANUAL_CHARGING`; the E2E harness reported about 313.7 m.
These distinct distance metrics have not been reconciled. Independent source
and receipt-time CSV assessment traced charging to the fake bridge's existing
axle-distance-to-origin rule: it asserts charging inside 0.3 m regardless of
heading or physical contact. At the rising edge, ground truth was approximately
`(-0.240,-0.150)` (0.283 m from origin), while fused pose was `(-0.548,-0.150)`.
Final truth remained about `(0.225,-0.156)` (0.274 m), preserving the charging
assertion and hold. The cached fake-bridge source has this same rule; battery
output was fixed at 28 V/full charge. This establishes a proximity-triggered
simulation hold, not a depleted battery or a new authorization failure. The
four FTC denials lack exact rejected-command evidence; full mowing is unverified.

All four last logged FTC states were FOLLOWING. Nearest receipt-time fused
poses lay 0.00075-0.00458 m inside the rectangular outer boundary, at different
edges. Their headings and preceding published commands are consistent with
perimeter overshoot, but those commands were previously accepted commands,
not the rejected Twist. The exact rejection pose, command and branch were not
logged. This cannot establish a verifier false positive or reconstruct its
precise sweep. No later authorization denial appeared. The smallest next
software investigation is one instrumented denial and deterministic replay,
not another blind full-mission run. Keep the execution invariant and existing
charging guard intact while investigating.

Ground-truth and fused-pose CSVs use different header time bases. Inspected
nearest receipt-time pairs differ by approximately 0.263-0.381 m, so raw truth,
fused map pose and wheel odometry must not be conflated. No boundary-violation
count is claimed without validating that alignment. Full attempt-14h logs
and monitor/pose data are retained under task-owned D: scratch. No physical
robot or shared-service mission was run.

Further verification uses a corrected simulation fixture: the same 54 m2 lawn
is shifted to `[-4.5,4.5] x [0.8,6.8]`, with a recorded navigation rectangle
`[-2.2,1.0] x [-1.0,1.8]` connecting origin dock/staging to the lawn. The
synthetic charging disk and every production charging/blade guard remain
unchanged. This geometry lives in the existing simulation scenario YAML;
real-robot map defaults are unaffected. First coverage poses remain on the
lawn, with blade-off Nav2 ingress rather than FTC coverage at the dock.

The no-LiDAR observer now uses the actual retained full-plan topic and stamped
post-mux command type. It requires observed undock/plan/mow/dock phases,
completed target paths without skips, actual docked charging, and the stated
median tracking threshold. `COVERAGE_INCOMPLETE` remains a failure even when
path counters later report completion. Timeout and signal termination preserve
the report's failure status. FTC rejection diagnostics record the actual
rejected pose/Twist and checked sweep; their geometry fingerprint is evidence
correlation only, not another permission identity.

Attempt 15 at `8fcb0176` executed the corrected no-LiDAR mission. Undock and
planning passed; the 9899-pose route progressed for 368.4 m, with median
tracking deviation 5.6 mm and p95 58.5 mm. Four early FTC denials were resumed.
The route reached its endpoint, but the real map reported 0% stamped coverage
and FollowStrip latched `COVERAGE_INCOMPLETE`. The harness correctly failed
mowing. Navigation to dock staging succeeded; docking then failed because no
external dock detection arrived. After over 60 s idle with zero command and no
charging, signal termination retained FAIL (exit 1). No guard was bypassed.
This is incomplete mission evidence, not preserved full-system functionality.
Logs show the simulator requested blade ON but reported an inactive blade
controller; production stamping correctly rejected it. GNSS projection also
reported an invalid-identity/provenance warning. Peak observed resource range was roughly
530-645% CPU and 1.3-1.45 GiB RSS; it is not isolated verifier overhead.

Targeted diagnosis confirmed two simulator producer defects. The fake bridge
always sent `mower_esc_status=0`, including during ON with 3000 RPM; it now
reports active only while ON. A real installed-process service/topic regression
fails against the old binary and passes OFF/ON/OFF against the correction, using
private test endpoints. Map-server verification remains unchanged.
The synthetic GNSS relay omitted `position_observation_sequence`; the current
production association explicitly rejects zero. The relay now pairs the exact
raw fix header with a nonzero sequence for each increasing raw acquisition plus
a new ground-truth generation. Duplicates, unset/future raw stamps, replay and
cached truth cannot refresh it. Both callbacks reset cached observations on ROS
clock rewind while retaining monotonic sequences.

All 60 simulator Python tests pass; 19 sweep tests include the four actual
attempt-15 rejected requests and original edge-overhang controls. The captured
requests are rejected by the real verifier; rotation-only counterparts remain
permitted. No authorization rule was relaxed. A stationary 60 s Webots probe
observed 262 fixes/261 zero-identity statuses with zero absolute poses or dock
detections before the relay correction. After correction it observed 260 fixes,
260 absolute poses and 599 real dock detections. This establishes the production
projection/detection connection, not successful physical docking or a completed
mission. Independent review found and resolved a clock-rewind regression.
Ground-truth headers currently use wall time while raw fixes use simulation
time; generation deduplication does not establish ground-truth acquisition-time
freshness. Receipt alignment and mixed cached packages remain limitations.

Attempt 16 at `7b894c84` passed undock/planning and entered MOWING with verified
blade activity. It advanced 21.7 m, but 19 warnings showed a missing
`map -> blade_link` transform. It was stopped early with FAIL (exit 1), rather
than waiting for a predictable unrecorded-coverage result. Stamped area was not
measured; HOME/docking were not exercised. Two FTC denials resumed movement.
The Webots URDF now includes the fixed cutting-tool frame using the simulator's
existing 0.18 m chassis centre and 0.093 m wheel radius, matching production
xacro construction and placing the tool 0.05 m above ground. Map stamping and
blade guards are unchanged. All 62 simulator Python tests pass, including real
robot-state-publisher TF and configured xacro geometry checks; the final two
tool-frame tests also pass with `ROS_DOMAIN_ID` unset. Independent review
confirmed publisher ownership/geometry and resolved a test-domain mismatch.
Further runs must verify the live map/tool TF and installed URDF before START,
then record actual map-server stamped cells during mowing. Full mission
completion remains pending.

Attempt 17 at `929e8378` passed the actual no-LiDAR mission cycle in 2539 s
(42.3 min), exit 0: undock 11.9 s, plan 46.4 s, mow 2412.5 s, dock 46.8 s,
ending `IDLE_DOCKED` and charging. The strict harness reported 1/1 complete
paths and zero wholly skipped paths, with no `COVERAGE_INCOMPLETE`. Final global
stamped raster was 21,712 cells / 54.28 m2; this is not an area-specific coverage
percentage. The existing area plausibility guard checks only 50% interior
stamping when a usable grid exists. Three authorization aborts caused approximately 0.8 m recovery cursor
advances with blade-off transit; those partial omissions do not increment the
whole-path skip counter. The goal checker also accepted the endpoint with
0.499 m remaining. Neighboring passes may overlap these spans; actual uncovered
area has not been established. This demonstrates a progressing mission cycle,
not every planned span executed blade-on or near-total physical lawn coverage.

The harness measured 366.6 m travel and median fused-pose nearest-path error
5.3 mm (p95 57.9 mm, 55,297 samples); these are not ground-truth tracking errors
or checks against the intended segment. Three short GNSS guard pauses recovered
automatically. The pre-START monitor recorded 23,417 samples. Live spot samples
were about 1.45-1.47 GiB and 500-600% CPU, not isolated authorization overhead.
The real dock action completed and final truth was within the simulator's
0.3 m charging circle. This does not validate physical dock contact. Logs also
show dock-prior/RTK discrepancies of 0.52-0.57 m, a final 0.59 m gauge reset,
eight failed periodic graph saves and a failed dock-arrival save. These runtime
diagnostics require follow-up; a healthy covariance verdict does not resolve
position disagreement. Independent evidence review confirmed the harness PASS
and these limits. LiDAR, explicit HOME interruption, charge/resume, revocation
and additional geometries still lack full-mission evidence.

LiDAR attempt 18 at `9fdcf8c3` stopped before START with a failed readiness
verdict. All base nodes/controllers/tool TF were ready, and raw/costmap/collision
scan streams had increasing nonzero acquisition stamps. The collision-monitor
bulk parameter response contained no values, so its effective guard configuration
was not verified. A startup LiDAR-OFF warning describes the absent installed
configuration's default; source review confirms explicit CLI overrides take
precedence, so that warning does not establish the active mode. The empty
response needs isolated parameter queries, including separating unused legacy
polygons from required active guards. No mission, LiDAR obstacle response or
coverage result is claimed. A fresh task-owned graph directory was writable as
UID 1000 and live graph save/autosave settings were correct; save success was
not exercised. The next isolated attempt uses an explicit sparse LiDAR config
and verifies actual guard parameters before authorizing START.

Attempt 19 at `8573be31` passed the live LiDAR/guard readiness checks, including
all required active polygons and independently queried parameters. Three setup
parameter tests passed; unused legacy polygon and PointCloud-only height keys
no longer mask required LaserScan guard settings. The subsequent mission was
stopped by a task-observer false positive: its 30 s timer armed on the umbrella
MOWING status before planning and blade-off transit. At termination transit was
still progressing, with no recorded mowing. Genuine blade ON and verified
stamping began later during serial cleanup after observers stopped. This is not
evidence of a mower refusal or successful LiDAR mission. Harness exit was 1;
runner/container exit was 31. Sensor streams were advancing, and 355 paired
nav/monitored commands matched, but no collision response was observed.

The next task-only observer arms from current mowing/path context plus fresh
advancing direct blade telemetry. It compares blade stamps against ROS clock,
uses monotonic liveness/deadlines, observes phase changes in callbacks, excludes
intentional OFF pauses and caps credit at previous evidence expiry. The same
30 active-second progress requirement and independent 60/3000 s limits remain.
Nineteen deterministic policy/real-ROS-adapter checks pass. Cleanup now stops
simulation execution before stopping evidence recorders. Production guards and
canonical harness assertions were not changed; LiDAR mission acceptance remains
pending.

LiDAR attempt 20 at `5cc78033` passed those readiness checks and completed the
undock, plan and mowing phases: 11.7, 38.1 and 2345.4 seconds respectively.
The strict whole-cycle verdict was **FAIL**, exit 1 at the unchanged 3000-second
limit. It observed 1/1 completed paths, no wholly skipped paths, 374.4 m travel
and 20,103 global stamped cells (50.2575 m²); these are not a physical coverage
percentage. Short blade-off resume-cursor advances remain a coverage limitation.
The corrected task watchdog passed with fresh blade-active grid growth and did
not interrupt the mission. Live collision-monitor logs show slowdown and
approach responses. Seven LiDAR TF future-extrapolation errors and transient
GNSS/pose freshness rejections occurred; guards stopped or paused motion.
Eight periodic graph saves succeeded with a private writable map directory.
An auxiliary collision-state recorder expected a nonexistent message header;
its failure limits recording and does not invalidate the separate readiness
probe or establish continuous collision-state evidence.

After MOWING_COMPLETE, a localization guard halted the pending dock dispatch.
The docking client canceled only an already-polled goal handle, allowing the
late-accepted goal to survive the halt. On restart, its terminal callback wrote
into the outcome slot reused by the replacement goal. Logs show staging
preemption, an old-goal abort, a second server docking attempt, overlapping Nav2
navigation and NAV_TO_DOCK_FAILED. Source inspection confirms both lifecycle
windows. A focused DockRobot/UndockRobot repair now gives every dispatch its
own acceptance/result state, cancels late acceptance after halt or destruction,
and waits for its previous action's terminal result before replacement. A
five-second steady-clock drain timeout refuses replacement and retains the old
identity; it does not establish that cancellation succeeded. Boundary exemption
is not enabled while draining. Callbacks no longer capture the destroyed BT
node, and goal-handle operations occur outside the dispatch mutex.

Four real-ROS cancellation regressions failed against the preceding production
code; all **14 new transport cases pass**, including normal instant success,
rejection, destruction, restart, rejected cancellation and concurrent result/halt.
The focused run passed **7/7 targets**. After correcting only CRLF in the scratch
XML copy, one complete behavior run passed **47/47 CTest targets**, including
42 GoogleTest targets with 498 cases and zero failures/errors. Its first
full run was 44/47; two XML structure failures then passed, while the unchanged
manual-charge debounce test failed twice in its complete target and passed alone
before passing in the final full batch. These earlier timing failures remain
repeatability limitations; no assertion or test source was weakened.

All 96 behavior build inputs match the checkout after CRLF normalization only.
The installed behavior binary SHA-256 is
`0025fbeccb5629805cd77000130077a5e7c42905c351c06b7e5adc1d3ee41d26`.
An independent source review found no additional confirmed defect within this
per-node repair. Cross-node action ownership, a rejected cancellation's one-shot
send and indefinite transport retention for a never-answered acceptance remain
limitations. This is not a geometry exception or a final-output cancellation
lease. LiDAR attempt 21 at `1b0f50a1` used this verified installed binary and
failed the strict cycle after about 1590 seconds. Undocking passed in 11.7 s
and the independent blade-active growth check passed (0 to 352 cells in
30.0166 s). It traveled 214.8 m and stamped 12,809 global cells (32.0225 m²),
but completed 0/1 paths. Following two localization-guard pauses, FTC reverse
and Nav2 BackUp/transit failed to make progress; three controller progress
failures and four AREA_UNREACHABLE transitions exhausted the five-attempt
budget. COVERAGE_FAILED_DOCKING began at 1583.6 s. The run was stopped after
at least 135 s without distance growth; fallback docking had only just begun,
so this run neither validates nor disproves completion of the docking repair.
Six FTC denials and three boundary warnings are retained. Path-error median
15.8 mm and p95 64.0 mm measure fused nearest-path tracking, not containment.

Investigation identified two separate defects. First, the sparse task config
omitted the scenario datum: fusion/COG self-seeded from the antenna's initial
fix while map/navsat used the world reference, producing about 0.30 m of
world-X displacement. All earlier mixed-runtime mission claims remain
bounded by that frame inconsistency; a passing cycle does not validate
physical map alignment. The next isolated fixture must provide and verify
the same explicit datum for every consumer.

Second, the installed Nav2 Collision Monitor 1.5.2 reproduced a rear-clear
reverse being stopped by two returns in the extra front catcher outside the
body. `approach` checks current polygon occupancy before projecting motion,
so the static front band can also freeze reverse. The production correction
uses supported `velocity_polygon` selection: exactly-zero linear commands
select the full padded body, negative commands select that body united with
a rear catcher, and positive commands select the unchanged original forward
band. Inclusive first-match ranges cover every finite signed velocity without
an epsilon gap. Launch derives all points from the same operator chassis,
three-decimal footprint and explicit existing 0.01 m local-costmap padding.
Two-point sensitivity, projection horizon/step, source/TF rejection and the
live three-point FootprintApproach guard remain enabled.

An initial 25-case proposal used raw overlay geometry. The subsequent actual
production-helper run passed **27/27 real installed-node cases**, including
positive reverse/turn progress, forward/rear and padded-body stops, curved and
corner sweeps, custom rounding/padding, zero padding, tiny signed commands,
stale scans and missing TF. Separate real Costmap2DROS probes confirmed default,
custom and zero padding. The checked-in registered node integration test passed
**10/10 tests** (nine motion tests plus shutdown) in 30.589 seconds; its outputs
must match current input stamps. Two parameter-generation cases execute the
production launch builder using action adapters and inspect its generated YAML
and footprint-rewrite argument; they do not launch the full ROS stack.
Independent review found no confirmed blocker in this focused correction.
The complete bringup Python/configuration batch passed **274/274 cases**.
Its first complete collection required explicit non-launch file selection;
five package-index setup failures then passed after loading the workspace and
fresh bringup install. Both new CMake-registered targets passed (2/2).

The two-point subpolygons, like the preceding static catcher, are regenerated
at bringup; they do not follow live footprint/padding changes. FootprintApproach
still follows the live body at its existing three-point threshold. This adds
no geometry exception or action lease, and docking's mux lane still bypasses
Collision Monitor. Helper work occurs at launch; selector/body-sweep overhead
and target-ARM deadlines are not measured. Retained attempt-21 scans omit beam
endpoints and cannot prove
the precise triggering returns. No full LiDAR mission PASS or stack-wide #924
acceptance is established.

Datum-aligned LiDAR attempt 22 at `c8c64e13` passed all live selector and
five-consumer datum checks before START (81 stationary receipt-aligned pairs,
maximum XY difference 38.5 mm, wrapped yaw difference zero). The strict mission
**failed after 385.8 seconds**: undock 12.0 s and planning 39.1 s passed;
mowing failed after 267.8 s with 0/1 paths complete and about 1% cursor progress.
Five attempts were exhausted. It traveled 29.5 m and stamped at most 759 global
cells (1.8975 m²), not a physical coverage percentage. The watchdog did not fire.
Fallback docking **and simulated charging passed in 50.7 s**, reaching
IDLE_DOCKED. This is useful docking functionality evidence, not a mowing-cycle
PASS or physical charger-contact test.

All 181 recorded reverse requests had same-stamp monitored outputs at their
requested -0.15 m/s. Successful mowing recovery was not established. Two FTC
denials occurred during FOLLOWING through rounded outer headland corners.
One reference pose started 1.198 mm outside the recorded max-X boundary; the
other started 0.629 mm inside min-Y but its proposed 0.1 s arc crossed it.
Those requests must remain rejected. Earlier translation/steering response and
residual wheel velocity require a short closed-loop reproduction before a
controller repair can be accepted. Receipt-aligned ground truth was still
inside at both denials; fused pose/projection differences cannot be treated as
proof of physical boundary crossing or permission to ignore an estimated pose.
Later RPP collision-ahead/controller-patience and repeated transit retries also
failed. Independent transit commands remain outside this slice's execution gate.

Raw collision-beam records were retained in attempt 22; the monitor still
records only path summaries, so exact full-plan replay is unavailable.
The mixed-runtime and clock/provenance limitations remain. A complete LiDAR
mowing cycle, a measured final-command containment result, and physical
acceptance remain outstanding.

A focused comparison reproduces a headland functionality problem through
the real FTC library, with identical geometry, TF, parameters and modeled wheel
response. Its representative legal rounded rectangle is not the unrecorded
attempt-22 full plan. Original `9f39497f` dev finishes after 726 model steps but
records 417 reference samples outside the rectangle (maximum X 4.50486 m,
minimum Y 0.79522 m). This branch aborts after 266 steps with no outside sample
before the abort; extending the model with zero-command coast records 39
outside samples and a final X of approximately 4.5001 m. Neither outcome passes
the combined progress-and-containment requirement. These are modeled controller
results, not a physical stopping-distance measurement.

An isolated brake/heading-alignment experiment completes the basic unbiased fixture
without a recorded crossing and preserves all 26 FTC authorization assertions.
It is **not included in production**: its early forecast assumes the fixture's
0.15 s response, a 0.04 s future-TF probe fails progress under its finite retry
budget, and signed 3 mm pose bias can leave the estimate inside while modeled
truth crosses. Earlier unbounded probes also exposed numerical-edge behavior.
Behavioral checks are not a robustness or full-mission PASS. The real Nav2
velocity argument has no acquisition timestamp; it supplies no freshness or
physical stopping guarantee. Exact full-plan recording has separately passed
a real ROS 9,899-pose transport and startup CLI check.

Strict LiDAR attempt 23, using the same reviewed runtime and the full-plan
recorder, **failed after 293.2 s**. Undocking (11.6 s), planning (40.3 s), and
fallback docking/charging (53.9 s) passed; mowing failed after 173.3 s, with
0/1 paths completed and 29.7 m total travel. The watchdog did not trigger.
Four complete plans were retained. All 9,899 initial reference points lie
inside the configured outer rectangle; this is not a complete trajectory or
footprint certificate. The actual first corners have roughly 45-degree heading
changes on either end of 3.586 mm and 3.606 mm chamfers, without coincident
planner pivot twins. They differ materially from the representative 0.2 m
radius fixture. Two FOLLOWING denials again rejected an outside-start pose and
an outward intended arc. A progress repair must now address the actual planner
and controller corner contract, preserve the existing body/hole gates, and be
validated against the captured path. No corrected full-mission PASS is claimed.

The production coverage planner now treats a microscopic headland bevel as
one corner only when a supported tangent fillet fits across its two supporting
edges. The four-point window admits ordinary edges from 0.1 mm to 10 mm,
requires consistent turn direction and forward tangent ordering, and bounds
the virtual intersection within 10 mm of both bevel vertices. That intersection
is never emitted alone. Existing minimum-radius and sampled geometry checks
remain; GEOS additionally checks every emitted chord and tangent connector
against the complete boundary and holes. A rejected proposal keeps the original
vertices. This changes sparse headland-ring smoothing only; the final
normalization, explicit pivot twins, FTC execution gate and chassis-overhang
policy retain their existing behavior.

A real Fields2Cover production-pipeline regression uses the configured ordinary
rectangle and the observed operation/headland widths, five passes and 0.20 m
minimum radius. The preceding source fails with a 45.0579-degree maximum
heading kink; the corrected outer ring has a 5.0555-degree maximum. Summaries
of the four inner rings are unchanged. The native original plan has 9,896
poses versus the captured mission's 9,899, so this is a reproduced planning
defect, not exact regeneration of that mission. All **138 coverage GoogleTest
cases pass**, including nine new seam controls and the new real-pipeline
regression. A complete non-root ROS coverage-package build and install passes
**9/9 CTest targets**, including those six GoogleTest targets and three linters;
the overlapping counts are not additional cases. Unchanged real-controller
FTC **26/26** and Escape **17/17** assertions also pass.

Independent strong review found no supported new production blocker in this
narrow increment. It identified an incomplete rectangle predicate in the
separate modeled-controller evidence. The corrected common oracle checks all
four bounds for truth, estimates and zero-command coast, with the original
numerical tolerances. Three direct oracle controls pass, including left/top
coast rejection. Both nine-case modeled prefix batches pass again: captured
vertices passed through the corrected ring seam (175 poses), and separately
regenerated production output (174 poses), each ending after the second corner.
All 18 cases finish with zero outside samples under that complete rectangle
oracle, including 40 additional coast steps. Cases cover no-LiDAR and an empty
real LiDAR costmap, TF leads through 0.08 s, response constants 0.10/0.15/0.30 s
and signed 3 mm XY biases. This is selected wheel-response model evidence,
not actual mission LiDAR, physical stopping distance or whole-route acceptance.
The raw-path and naive-pivot progress failures remain RED. The reviewer reran all
nine seam tests and the pipeline regression, and exercised 20,000 randomized
bevel inputs. At that checkpoint no corrected full mission had run; the
subsequent rebased cycle is recorded above. A separately
constructed beveled-field probe still has unsupported later 33.75-degree and
90-degree corners before and after this change; general corner feasibility is
not established. The inherited radius-retry routine also needs configuration
validation for nonfinite or extremely large radii; shipped finite radii were
used in the reported checks.

Webots attempt 24 stopped before E2E START: the unchanged Nav2 readiness gate
failed because `is_path_valid` was unavailable after its existing 10-second
wait. Copied coverage binary hashes and package-prefix checks passed, but live
mapping and the later datum/selector gates were not reached. No mission verdict
exists for that attempt. Its 24 startup resource samples are not mission or
isolated authorization-overhead measurements.

The first two attempt-25 launches also stopped before E2E START, from test-runner
encoding and helper-order defects. A partial service probe did not establish
the Nav2 failure's cause. Lead checks found and repaired an additional syntax
defect before the next launch; shell/Python/PowerShell syntax, encoding, helper
ordering and actual checksum accept/reject checks passed. The corrected third
launch passes Nav2/LiDAR readiness and verifies the exact live coverage executable
and mapped core library. The original readiness, geometry, datum and canonical
mission assertions remain in force. The earlier service failure's cause is
undetermined; passing readiness is not a full corrected mowing-cycle PASS.

That corrected LiDAR attempt 25-03 is now terminal **FAIL after 1,078 s**:
undocking 11.5 s and planning 39.8 s pass; mowing fails after 946.4 s with
0/1 paths completed and 124.6 m traveled. Repeated transit/recovery failures
exhaust the area attempts. The watchdog ends the run after
`COVERAGE_FAILED_DOCKING` persists for 60.64 s; fallback docking is not completed
before termination. Canonical harness exit is 1 and wrapper/watchdog exit is 31.
Nearest-path tracking passes with 19.2 mm median and 78.2 mm p95 over 22,319
samples, which does not establish command or physical containment.

The two FTC denials occur on the bottom straight after obstacle deviation,
and at a subsequent outside-mowing restart. The first intended 0.1 s arc
crosses the mowing boundary and must remain rejected. Between them, blade-off
RPP transit reports success near a point inside the navigation corridor but
outside the mowing polygon. This exposes an operation-handoff limitation;
it does not establish retained FTC output after abort. Lattice candidates
currently use the soft costmap boundary rather than raw mowing permission;
geometry-aware candidate selection was investigated in the increment below.

An additional observer retains 41,226 final simulated wheel commands, 7,762
footprints and one genuine cached raw geometry message. It starts after both
denials and therefore cannot verify their final-output window. The geometry
has float32 mowing/navigation vertices and stamp zero; no first-denial revision,
owner or acquisition freshness is established. All raw plans, scans, poses and
commands remain retained. The 108 resource samples peak at 764% host CPU and
1.716 GiB; these are whole-stack observations, not isolated verifier overhead
or ARM limits. The earlier startup failures remain recorded. No corrected
complete LiDAR cycle or hardware acceptance is claimed.

## Geometry-aware avoidance and forward obstacle sweep

FTC now applies raw mowing authorization during lattice selection as well as
at final command output. Every station, offset node and connecting chord must
keep the axle inside the mowing union; the complete chassis sweep must avoid
raw forbidden holes. These hard checks remain active during reaction grace
and horizon degradation. The critic uses the actual padded chassis and the
exact inverse of the transform that placed the plan in the local costmap.
Missing geometry or invalid transforms fail closed. Legal outer-body overhang,
existing recovery limits and the final raw command gate remain unchanged.

Selecting an inward skirt exposed a separate execution defect: the intended
path was legal, but actual modeled wheel/yaw response could sweep the body
through a lethal obstacle. Positive forward FOLLOWING lattice output therefore
also checks a complete reachable chassis envelope against closed lethal-254
cell squares. Rejection uses the existing obstacle hold/abort behavior. Soft
boundary-253 cells retain their existing policy; STOP remains unconditional.
Reverse, pure rotation, no-lattice and no-LiDAR obstacle paths retain their
existing guards rather than receiving this new forward-obstacle gate.

`FollowCoveragePath.obstacle_coast_time_bound_s` defaults to 0.5 seconds, with
finite startup and live bounds [0.01, 2.0]. Startup rejects an invalid value
explicitly. The envelope assumes no speed/yaw overshoot beyond proposed and
measured extrema during the 0.1-second command interval, then bounds remaining
absolute-speed path length and cumulative heading excursion by the configured
coast bound times those extrema at every subsequent time. First-order STOP
decay with tau at most that bound satisfies this model. This is an assumed
response contract, not physical stopping, localization uncertainty, transport
timing or downstream command-retention evidence. A smaller value requires a
validated response assumption; a larger value rejects more conservatively.

Immutable raw-edge indexing accelerates the same reference-point and chord
predicates; it does not simplify allowed geometry. Exact-Y parity preparation
is private to each decision, with an 8 MiB accounted cache budget and exact
predicate fallback on exhaustion. Geometry replacement/revocation installs
the new Snapshot before index preparation; delayed preparation cannot install
over a newer Snapshot. Content identity does not become a producer epoch or
action lease.

The original unsafe modeled obstacle result (27 lethal samples versus zero
before the candidate change) and the slow geometry cases are retained as
failures. After the repair, eight full-duration strong synthetic/captured-plan
prefix cases at response tau 0.15/0.5 seconds safely abort with zero lethal or
outside-reference samples, including 80 STOP-coast steps. They do not establish
obstacle traversal. An empty edge-overhang control completes in 228 ticks and
a small default obstacle skirt in 230 ticks; the five existing recovery scenes
also pass. Captured plan coordinates are real, while obstacles and response
are modeled; this is not exact first-denial costmap replay.

After the final startup guard, a fresh Release build passes 23/23 Nav2 CTest
targets (20 GTest executables containing 298 cases, plus three lint checks):
21 critic cases, 18 real-controller cases and all 26 FTC authorization cases.
The controller cases reject seven invalid startup values and accept both range
endpoints and the default, for both LiDAR configurations. Rebuilt Escape and
docking-dispatch targets against the same installed shared header pass 17/17
and 14/14 respectively. Both-overlay configuration checks pass 72/72; the drift
check reports structural fields in sync and three existing uncategorized GNSS
keys. Clang-format 18 whole-file and changed-line checks pass. These counts
overlap earlier evidence and must not be summed as independent suites.

Measured full index preparation plus a 51-station decision on this host fell
from approximately 1.7 seconds for a 32,768-point circle and 2.19 seconds for a
32,768-point comb to 14.40 ms and 31–39 ms respectively. Dense geometry with
30 holes takes 6.12 ms; nonidentity TF comb controls take 20–24 ms. Actual
less-than-100-ms assertions pass. Measurements exclude MarkerArray parsing and
are not ARM or generic hard-deadline evidence. Adversarial cache-budget
exhaustion passes predicate equivalence, but does not prove its decision latency.

At this pre-rebase checkpoint the last complete LiDAR mission was attempt
25-03 FAIL. The subsequent rebased cycle passes the canonical harness with
known unmowed spans and an unknown outer exit status, as recorded above.
General navigation-to-mowing handoff, worst-case deadlines, ownership/leases
and the other OPEN paths in the acceptance matrix remain owed. Hardware acceptance remains
`HARDWARE_REQUIRED` under the exact-baseline procedure below.

Available controller fixtures execute the real FTC with modeled wheel response
and simulated endpoints. They can establish command progress, expected aborts
and fixture collision checks. They do not constitute full mowing, connected
area, dock/undock/HOME/charging-resume missions or post-mux hardware-output tests.
One no-LiDAR mission cycle now passes; exact lawn coverage percentage,
final-command boundary violations and isolated authorization overhead remain
unmeasured.

The existing [OpenMower software hardware simulation](https://github.com/mowglinext/mowglinext/actions/runs/38010148929)
did execute on `0f66cd8c`: **178/179 checks passed** on its first attempt.
The failing `xesc_2040` 350 ms process-starvation scenario measured 0.224 m/s
instead of the unchanged 0.300 +/- 0.050 m/s progress requirement. Odometry
agreement, plausibility and board latch checks in that scenario passed. The
bridge/emulator/scenario sources are unchanged against the dev baseline; this
does not by itself explain the timing failure. The unchanged second attempt
passed **179/179 board/ESC checks and 10/10 stack seam checks**, with both
starvation scenes observing 0.299 m/s. The first unexplained miss is retained
as a repeatability limitation; no test, assertion or source was changed.
The preceding [dev simulation run](https://github.com/mowglinext/mowglinext/actions/runs/37965865987)
at `5733a399` passed 179/179 board/ESC checks and 10/10 stack seam checks;
the inspected bridge, simulation, hardware, message-schema and seam-launch
sources match this work's `9f39497f` dev baseline. It observed 0.300 and
0.299 m/s in the two starvation scenes. Its runner/runtime are separate evidence,
not an identical-host controlled comparison or an explanation of this failure.
The subsequent mux/bridge stack-integration script did not execute on the
first attempt because the board script returned failure. On the second attempt,
the real launch/mux/bridge moved emulated wheels through its teleop input and
stopped them after input release (970 ms observed, unchanged 1700 ms bound).
This emulates the OpenMower board/controllers
and charging/e-stop behavior; it is neither a Webots mowing mission nor evidence
for the FTC/Escape final-output invariant. No assertion was weakened.

## Unresolved safety and functionality acceptance

- Explicit producer epoch, revision and datum are absent. A map-server restart
  can leave cached permission, and depth-one DDS history can coalesce a
  revoke/restore transition. Escape generations cancel a delivered revoke,
  including restoration of identical content, but cannot detect an omitted one.
- FTC has no motion-owner/action lease. Source cancellation and geometry checks
  do not establish cancellation of a held, shaped or reselected downstream
  command. The final bridge may alter linear/angular proportions.
- Existing runtime obstacle-promotion centroid deduplication can report success
  while discarding a distinct candidate polygon. Pending-ID acceptance retains
  its polygon. This preexisting deferred gap prevents claiming protection of
  every successfully promoted forbidden region.
- Transit, docking, undocking, HOME, ingress, calibration and dig still require
  coherent bounded exceptions and final command authorization. One default
  no-LiDAR cycle demonstrates normal undock, ingress and return-to-dock;
  other variants, explicit HOME and charging-resume remain unverified.
- Worst-case union construction, many intersecting rings and target ARM
  controller deadlines remain unmeasured. Host microbenchmarks cannot certify
  real-time hardware behavior.

## Physical acceptance

**HARDWARE_REQUIRED.** Record this branch's final ROS commit/image, firmware
image/hash, GNSS submodule/driver/receiver revisions, robot unit and configuration
hashes before running. No robot motion, firmware flash or physical deployment
was performed.

Remove blades, establish an exclusion zone, verify emergency stop, and use a
supervised low-speed fixture. Exercise repeated legal edge passes, pivots,
holes, blocked-start escape, connected transit, non-default dock/undock,
HOME/charge/resume and geometry revocation. Pass only if approved operations
progress and measured chassis motion stays within the declared geometry and
operation envelope. Rejected motion must stop within a separately measured
physical bound, with no blade/localization/emergency regression. Software
constant-Twist tests cannot establish slip, physical stopping distance, blade
safety or hardware reliability. Completing the open software paths is a
prerequisite to accepting the full #924 invariant.
