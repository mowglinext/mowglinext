# fusion_graph Review — Holding <2 cm Absolute During RTK-Float
_2026-06-11 · 46-agent multi-agent review (wobpjur04) · 33 findings → 28 confirmed against code_

## 1. Verdict

**No. The current code cannot hold <2 cm absolute during an RTK-Float window, and on the deployed config it is not even close.** During Float the graph's only absolute-XY constraint is the GnssLeverArmFactor fed by the degraded `/gps/fix` (decimetre sigma, 0.2–1 m bias), fused at its *honest* sigma with a Huber kernel that — by construction — cannot reject a low-residual biased fix (`graph_manager.cpp:539-552`). The one mechanism designed to ride through Float (LiDAR scan-match + loop closure) is **disabled in the deployed config** (`mowgli_robot.yaml:60,106,109`) and there is no LiDAR container, so the only thing left between Float epochs is a wheel between-factor at `wheel_sigma_x=0.05 m/node` — pure dead reckoning with unbounded drift on an uncalibrated `ticks_per_meter=260` drivetrain (~+14% speed bias measured). The system as built degrades gracefully to *GPS-noise* (decimetre) during Float, not to 2 cm.

## 2. What is done wrong

### Float handling (the central blocker)
- **Float epochs fused at honest decimetre sigma, no global gate** — every non-Fixed fix falls through to `QueueGnss(mx,my,sigma,robust=true)` (`fusion_graph_node.cpp:982`) with `sigma=sqrt(0.5*(var_x+var_y))` raw from NavSatFix (`:893-895`). A 0.2–1 m biased Float fix at sigma~0.1–0.3 m out-weights the `wheel_sigma_x=0.05` chain and pulls base_footprint decimetres off truth within seconds.
- **Huber cannot reject a biased-but-low-residual Float fix** — Huber re-weights by *residual/sigma* (`graph_manager.cpp:545-552`); a 0.4 m bias at sigma 0.3 m is ~1.3σ, below `huber_k_gps=1.345`, so it stays fully weighted. The outlier-ness is in the *sigma magnitude*, not the residual — Huber never sees it.
- **Float gating exists ONLY inside `DockingApproachActive()`** — `fusion_graph_node.cpp:924-928`; true only when `/cmd_vel_docking` was non-zero in the last 1.0 s. During transit/mowing (where swath-tiling needs 2 cm) the gate is always false and Float flows straight in.
- **Wrong-fix gate is local-jump-only, blind to slow Float drift** — fires only on `jump > budget && wheel_dist < 0.02` against the *previous epoch* (`:868`). Float drifts a few cm/epoch while moving, so the AND is always false. No comparison against the dead-reckoning prediction.
- **GPS factor ignores `msg->header.stamp`** — fix attaches to `PoseKey(next_index_)` at `now_s` (`graph_manager.cpp:551-553`). ~90 ms serial latency at 0.5 m/s bakes 2.5–5 cm of velocity×latency error into every epoch.
- **Lever-arm couples yaw error into GPS XY residual** — `pred = X.t() + R(θ)*lever`, deployed `gps_x=0.3` (`factors.cpp:27-37`). 3.8° of (Float-grounded COG) yaw error → 2 cm antenna shift split into XY; the anchor becomes a yaw-to-XY error pump.
- **No managed Fixed→Float→Fixed snap-back** — `last_rtk_fixed_stamp_` only gates yield/skip; the first Fixed epoch is a tight unary that jumps the newest node, `t_map_odom_anchor` steps on node-index change (`:1782-1789`). Recovery is a discrete TF kink, not bounded convergence.

### Rebase (destroys the anchors it should preserve)
- **RebaseISAM2 re-priors EVERY kept node at isotropic 0.05/0.05/0.05** — `graph_manager.cpp:943,954`; the original RTK-Fixed GnssLeverArm factors (σ~3 mm) are thrown away. The graph's memory of where it truly was is floored at 5 cm. Same lossy re-prior in `RigidTransformAll` (`:1068`).
- **Rebase fires every 2000 nodes (~40–80 s), no Float guard** — the precise pre-Float anchor gets flattened to 5 cm *inside* the very Float window it should bridge.

### Dead reckoning (the Float-coast carrier)
- **`ticks_per_meter` static, no online calibration** — deployed 260 (`mowgli_robot.yaml:99`); 1% scale error = 7.5 cm/30 s of systematic along-track bias; ~+14% bias measured on this drivetrain.
- **Gyro bias EMA only updates while wheel-stationary; `use_imu_preint=false`** — thermal drift during a multi-minute pass uncompensated; 0.002 rad/s × 180 s = 20° yaw → 17 cm lateral at a 10 m lever.
- **Pivot mode inflates σ_x but never removes the phantom forward-vx bias** — `slip_detected` requires `|dtheta_gyro|<0.005` so genuine headland pivots never trigger zeroing; the documented +0.025 m/s pivot bias injects a few cm/pivot.
- **`wheel_sigma_y=0.005` hard-locks lateral motion** — real wet-grass/slope skid is structurally unrepresentable; with LiDAR off, cross-track drift diverges with *zero* covariance growth.
- **Chord-vs-arc constant-yaw integration** — `accum_.dx += vx*dt` holds yaw fixed across the node (`:115-117`); a per-turn systematic accumulating across headland segments.

### LiDAR (the designed carrier, switched off)
- **Scan-match + loop-closure disabled, no LiDAR container** — `sub_scan_` only created if `use_scan_matching_ || loop_closure_enabled_` (`:396-400`), so no scan is ever ingested. Removes the only drift-free absolute-XY mechanism.
- **Even when enabled, scan-between is RELATIVE-only** — `BetweenFactor` between consecutive nodes; only loop closure to an RTK-anchored historical node re-pins absolute XY.
- **Loop-closure window cuts off RTK-anchored history** — `window_cutoff = next_index_ - max_graph_nodes` (`:843,857`). LC can only attach to recent (already-Float-drifted) nodes; old Fixed-anchored nodes are both window-excluded *and* flattened by the rebase.
- **Scan-yield holds LiDAR off for 2 s after last Fixed** — at the Fixed→Float boundary, LiDAR stays off for 2 s of wheel-only drift before re-engaging.
- **No scan deskew + hard tf lookup** — single extrinsic at `msg->header.stamp`, no `time_increment`; 0.05 s lookup timeout drops whole scans on TF jitter.

### Observability (the Float blind spot)
- **Published covariance is the bare newest-node marginal** (`:1917-1925`) — reports σ≈cm while true error is decimetres; Nav2, the session-monitor `rtk_cov_check`, and the operator are all blind to the excursion.
- **GPS innovation is computed and discarded** — the one signal distinguishing coast-able bias from real DR drift is never surfaced.

### Architecture / restore
- **Home-grown iSAM2 + full-rebuild rebase is the wrong tool** — the rebase needs a detached worker + pending-replay + `rebase_in_progress_` race flag that already caused the D2 race and the 2026-06-09 OOM.
- **Restore-from-disk pins nodes at 0.01 with stale yaw** — autoload override checks XY distance only, fires only on Fixed off-dock, never validates yaw; a Float-window restart trusts a stale heading.

## 3. The core architectural truth

When GPS goes Float on the deployed config, **the only thing pinning absolute X/Y is the biased Float GPS itself**, fused at decimetre sigma. Between epochs, position is carried by a single relative wheel between-factor and yaw by an integrated gyro corrected only while stationary. COG/mag are yaw-only — zero position information, and COG is itself GPS-course-derived so it degrades *with* Float. **There is no drift-free absolute sensor in the loop.**

**Realistic accuracy during Float today is decimetre, tracking the GPS bias plus accumulating wheel/gyro drift — not 2 cm.** And it worsens with window length: every ~40–80 s a rebase flattens the last Fixed anchor to 5 cm.

**Is <2 cm during Float achievable without LiDAR? No — physically.** Dead reckoning on a differential mower over grass cannot hold 2 cm absolute for minutes; no tuning of wheel/gyro sigmas produces an absolute reference. With LiDAR off, the honest spec is **"bounded drift proportional to Float duration, fast re-convergence on Fixed return"** — and the code doesn't even deliver the clean re-convergence.

## 4. Prioritized fix path

1. **Measure Float duty-cycle first** (low). Log `carrSoln`/status transitions over real sessions. If Float is rare/short, the spec becomes "bounded drift + clean snap-back" and the heavy LiDAR work may be unjustified.
2. **Global Float coast-and-gate, out of the docking guard** (low–medium; biggest immediate win). In `OnGnss`, when `!rtk_fixed`: skip `QueueGnss` and coast, or hard-floor Float sigma to `max(reported, 0.5–1 m)`; add a Fixed/Float debounce. Moves Float error from "tracks GPS bias (decimetres)" to "bounded DR drift."
3. **Fix the lossy rebase / migrate to a fixed-lag smoother** (medium–high). Replace the isotropic 5 cm re-prior with per-node `marginalCovariance()`, or replace iSAM2+RebaseISAM2+RigidTransformAll with `gtsam::IncrementalFixedLagSmoother`. Preserves the RTK-Fixed mm anchor; removes OOM/race machinery.
4. **Honest in-window accuracy estimator** (low–medium). Publish a DR error budget (`wheel_σ_x × distance + gyro_bias × time` since last Fixed), expose time-since-Fixed + accumulated GPS innovation. Closes the blind spot.
5. **Time-align + de-bias the carrier signals** (medium). Attach GPS to the nearest-timestamp node; zero wheel translation in pivot mode; midpoint/arc integration; online `tpm` regressor during Fixed windows.
6. **Enable + validate LiDAR scan-matching + loop-closure** (high; the only true <2 cm path). Run the LiDAR container, enable the flags, add a **persistent RTK-anchored keyframe layer** excluded from window cutoff + rebase so LC can close back to Fixed-anchored truth during long Float; per-beam deskew; key scan-yield release off the Float flag not a 2 s timer.
7. **Managed Float→Fixed re-acquisition ramp** (low–medium). Rate-limit the first Fixed correction (inflate-then-tighten) so Nav2/FTC don't see a position step.

## 5. Measure first

- **Float duty-cycle + window-length distribution** — F9P `carrSoln`/NavSatStatus transitions across real sessions. Decides whether items 3 & 6 are justified.
- **Wheel scale error (`ticks_per_meter`)** — regress GPS displacement vs wheel distance during Fixed straight runs.
- **Gyro bias drift during motion** — integrated gyro yaw vs RTK-COG yaw over a Fixed window. Confirms whether `use_imu_preint=true` is needed.
- **Actual DR coast drift** — during a clean Fixed window, *artificially gate GPS off* for 30/60/180 s and measure departure from live Fixed. The empirical "how long can we coast?"
- **GPS innovation signature during real Float** — record GnssLeverArmFactor residual through observed Float windows. Distinguishes coast-able same-sign bias from random walk.
- **Published-cov vs ground-truth error gap** — log cov_xx alongside fusion↔Fixed displacement to quantify under-reporting.
