# Implementation Blueprint — RTK-Anchored Keyframe Map + Scan-to-Keyframe Absolute Factor
_fusion_graph · holding <2 cm during RTK-Float with LiDAR enabled · 2026-06-11_
_Synthesized from workflow `wf_5b81497d-6d9` (6 integration maps + 3 design proposals; judge/synthesis/verify phases re-derived in main loop after the run hit the account spend limit)._

## 1. Chosen architecture

**Two layers, added incrementally, default-OFF behind `use_keyframe_map`.**

- **Layer A — persistent RTK-anchored keyframe store** (`GraphManager::keyframes_`, a `std::map<uint64_t, Keyframe>` parallel to `scans_`). Each keyframe freezes `{abs_pose (the GPS-fused base_footprint pose at capture), scan_body, sigma_xy, stamp}`. It is **never an iSAM2 variable**, so `RebaseISAM2`'s isotropic `{0.05,0.05,0.05}` re-prior (`graph_manager.cpp:943,954`), `RigidTransformAll` (`:1068`), `PruneOldScans`, and the window cutoff **cannot reach it** — its ~3 mm precision survives by construction. This is the whole point: the prior review proved that a keyframe stored *as a graph node* is mathematically impossible to protect (the rebase loop reads only `kv.value` and discards every marginal), so a **separate store is the only viable exemption**.
- **Layer B — `ScanToKeyframeFactor`**, a unary `NoiseModelFactor1<Pose2>` on the current node. During Float, ICP-match the live scan to the nearest keyframe; because `kf.abs_pose` is a known ~3 mm constant, the match yields an **absolute** position constraint on `X_curr` (σ ≈ ICP quality), Huber-wrapped.

**Phase 2 (separate, later PR): migrate the core to `gtsam::IncrementalFixedLagSmoother`** to delete `RebaseISAM2`/`RigidTransformAll` and the 5 cm re-prior floor entirely (marginalize, don't re-prior). The keyframe layer does **not** depend on this — keyframes are exempt either way — but the FLS core lifts the *live trajectory nodes between keyframe pins* above the 5 cm floor and removes the OOM/D2-race machinery. Do it after the keyframe layer is field-validated so the two are independently bisectable.

Why this split: the judge lenses (gtsam-correctness, integration-cost, achieves-2cm) all favor the additive keyframe store as Phase 1 (smallest diff, no smoother risk, rebase-safe by construction) and treat the FLS swap as the higher-value-but-higher-risk follow-up.

## 2. Keyframe layer

```cpp
// graph_manager.hpp, next to scans_ (~line 713)
struct Keyframe {
  gtsam::Pose2 abs_pose;                       // FROZEN GPS-fused map pose
  std::vector<Eigen::Vector2d> scan_body;      // base_footprint-frame points
  double sigma_xy;                             // capture-time GPS sigma
  double stamp;
};
std::map<uint64_t, Keyframe> keyframes_;
uint64_t next_kf_id_ = 0;
```

New `GraphManager` API (all under `mu_`, mirroring `AttachScan`/`GetScan`/`FindNodesNearXY`):
- `AddKeyframe(abs_pose, scan_body, sigma_xy, stamp)` — insert; enforce `max_keyframes` by **spatial decimation** (skip if a keyframe already exists within `kf_spacing_m/2`).
- `FindKeyframesNearXY(x,y,max_dist,max_cand)` — clone of `FindNodesNearXY` (`:759-785`) but iterates `keyframes_` reading the **frozen** `kf.abs_pose` (no Bayes-tree `PoseAt`) and **without** the `window_cutoff` (`:842-844`).
- `GetKeyframe(id)` — returns a **value copy** so the matcher runs lock-free in `OnScan` (same contract as `GetScan`/`GetPose`).

**Capture trigger** (`fusion_graph_node.cpp:1677-1680`, right after `AttachScan`), gate on ALL:
1. `use_keyframe_map_`;
2. **RTK-Fixed and stable** — not just `status==Fixed` for one epoch but Fixed held for `kf_capture_debounce_s` (≥3 epochs). *[design-review addition — a single carrSoln Fixed flicker can otherwise freeze a slightly-off anchor that poisons every later match];*
3. latched `last_gps_sigma_ <= kf_capture_sigma_max_m` (~0.01 m), latched in `OnGnss` after the wrong-fix gate (`:895`);
4. moved `>= kf_spacing_m` (~0.5 m) since `last_kf_xy_`;
5. `curr_valid && scan.size() >= kf_min_inliers`, and **low angular rate** (don't freeze a smeared turning scan).

**Capture `out->pose`** (the just-Tick'd, GPS-fused pose), **never `OnGnss` `mx/my`** (raw lever-arm-offset antenna ENU) — this is the #1 correctness trap.

**Persistence**: new `<prefix>.keyframes` binary (magic `FGKF` + uint32 version), serialized in the anon namespace next to `SerializeScansBinary` (`:1197-1245`). Snapshot under `mu_` in Save phase-1, write lock-free phase-2 (the 2026-05-14 TF-stall lesson). Load after `.scans`, **without** the window cutoff; **missing file must not fail Load** (back-compat with existing `.graph/.scans/.meta` triples). Clear in `ResetLocked` (`:1279`).

## 3. Scan-to-keyframe factor

New `ScanToKeyframeFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2>` in `factors.{hpp,cpp}`, modeled on `GnssLeverArmFactor`. The keyframe pose is a **constant baked into the factor** (NOT a graph variable — keeps the constraint absolute, rebase-exempt, and avoids the unbounded-graph OOM).

**Recommended residual/Jacobian — the chain-rule form (proposal 2)**, because it auto-passes the central-difference harness and is hard to get wrong:
```cpp
gtsam::Vector ScanToKeyframeFactor::evaluateError(const gtsam::Pose2& X, OptMat H) const {
  gtsam::Matrix3 Hc, Hb, Hl;
  gtsam::Pose2 P    = X.compose(icp_delta_, H ? &Hc : nullptr);          // predicted current map pose
  gtsam::Pose2 errP = kf_abs_pose_.between(P, nullptr, H ? &Hb : nullptr);
  gtsam::Vector3 e  = gtsam::Pose2::Logmap(errP, H ? &Hl : nullptr);      // zero when P == kf_abs_pose_
  if (H) *H = Hl * Hb * Hc;
  return e;
}
```
where `icp_delta_` is the matched transform such that `X_current * icp_delta_ == kf_abs_pose_`. Provide an xy-only mode (loose `sigma_theta`) since COG/gyro already pin heading. *(Proposal 1's hand-rolled analytic Jacobian — the `GnssLeverArm` xy block with lever=0 plus a `[0,0,1]` yaw row — is a valid faster alternative; switch to it only if profiling on RK3588 shows the compose/Logmap chain is hot.)* Must pass `test_factors.cpp:48-75` central-difference to 1e-4 + a `ZeroErrorAtTrueValue` case **before** anything else is wired.

**Forming the constraint** (`OnScan`, a new branch parallel to scan-between `:1588-1668`, **before** `Tick` so it lands in the same node's update):
1. `pred = LatestSnapshot()->pose ∘ PeekAccumulator()` (same init as scan-between).
2. `cands = FindKeyframesNearXY(pred.x, pred.y, kf_search_radius_m≈10, K)`.
3. per kf: `res = Match(kf.scan_body /*source*/, curr_scan /*target*/, kf.abs_pose.between(pred) /*init*/)`. **Source = keyframe, target = current** — `ScanMatcher` returns `delta` s.t. `target = delta*source`.
4. reuse the **exact** guard block `:1609-1644` (`res.ok`, `rmse ≤ icp_max_rmse_m_`, sanity, **divergence-from-init**).
5. keep the single lowest-rmse keyframe; `icp_delta = res.delta`; `abs_meas = kf.abs_pose.compose(res.delta)`.
6. **Runtime mirror-guard** *[design-review addition]*: reject if `abs_meas` is farther than `icp_max_divergence_xy_m_` from `pred` — catches a swapped source/target or compose-vs-between that injects a *low-rmse mirror-image* factor Huber will **not** reject.
7. `QueueScanToKeyframe(kf.abs_pose, icp_delta, max(res.sigma_xy, kf_sigma_floor_m≈0.02), res.sigma_theta)`.

**Plumbing**: add `UnaryQueue::ScanToKeyframe` + `std::optional` member (`graph_manager.hpp:588-613`); `QueueScanToKeyframe` next to `QueueScanBetween`; consume in `CreateNodeLocked` after the scan-between block (`:565-574`), Huber-wrapped; `reset()` next to `queue_.scan_between.reset()` (`:648`). It rides `new_factors_` through `ApplyIsamUpdateLocked` (`:580`) — **never a separate `isam_.update`** — so it mirrors into `rebase_pending_*` during an in-flight rebase.

**Rate-limit** *[design-review addition]*: run the match at `kf_match_rate_hz` (~5 Hz), not the 50 Hz node rate — absolute correction doesn't need 50 Hz, and K ICP matches per 20 ms tick would overrun RK3588.

## 4. Rebase / precision preservation

- **By construction** (Phase 1): keyframes are a separate map, untouched by `RebaseISAM2`/`RigidTransformAll`/`PruneOldScans`/window cutoff. Live nodes stay floored at 5 cm, but each new node gets a **fresh mm-anchored** scan-to-keyframe factor — the windowed graph is continuously re-anchored *to* the keyframe map.
- **Gauge coupling (mandatory)**: `RigidTransformAll` shifts the live graph by `correction` at dock arrival; add — in the **same `mu_` section, ordered before** the D2 cleanup (`rebase_in_progress_=false` + pending clear, `:1096-1098`) — `for (auto& [id,kf] : keyframes_) kf.abs_pose = correction * kf.abs_pose;`. Otherwise the map and live graph desync and inject wrong absolute factors. Order matters: a late rebase swap must not revert the keyframe shift.
- **Phase 2 (FLS)**: replace `ISAM2`+`RebaseISAM2`+`RigidTransformAll` with `IncrementalFixedLagSmoother` (lag = `max_graph_nodes·node_period_s` ≈ 300 s); per-key timestamps; `smoother_.update(factors, values, keyTimestamps)` **marginalizes** old variables (Schur complement, information-preserving) instead of re-prioring to 5 cm. **Keep keyframes outside the smoother (Option A)** — do not make them smoother variables (Option B re-opens the OOM unless `max_keyframes`-capped). Keep the `ApplyIsamUpdateLocked` indeterminate-system `catch → ResetLocked` around `smoother_.update`.

## 5. Float engagement + degeneracy

**Engagement** (inverse of the existing yield, separate gate):
- **Engage the instant Fixed is lost.** Do **not** reuse the 2 s `scan_yield_timeout_s` (which blinds scan-between/LC at the boundary, `:1655-1696`). New `kf_engage_age_s ≈ 0`: engage when `!last_rtk_fixed_stamp_ || (now - *stamp) ≥ kf_engage_age_s`, plus a `prev_rtk_fixed_` transition latch at `:909` so there's no boundary hole.
- **Yield while Fixed is fresh** — skip the match+queue entirely (GPS ~3 mm beats ICP; double-counting absolute info is the over-constraint mode that OOM-killed LC 2026-06-09). Capture and engage are thus mutually exclusive: the map is only **written** under Fixed and only **read** under Float — no self-referential drift.

**Degeneracy (open lawn → DR, never a bad factor)** — four tiers:
- **T0 no candidate**: `FindKeyframesNearXY` empty → queue nothing, wheel+gyro DR carries. (The honest physical caveat: <2 cm only where LD19 sees structure.)
- **T1 ICP guards**: reuse `:1609-1644` (`!ok` = <30 inliers, `rmse>0.10`, sanity, divergence).
- **T2 structure check** *(new)*: require `inliers ≥ kf_min_inliers` (~40) AND a 2-direction observability test — eigenvalue ratio of the matched inlier-point scatter `< kf_min_aniso` (~0.05) rejected. Kills the **1-D wall-slide** (a single straight fence gives no along-wall constraint; ICP slides freely).
- **T3 robust kernel**: Huber/DCS wrap downweights a degenerate-but-passed match.

## 6. Deskew — reuse the existing node, add the missing translation term

**Correction (2026-06-11): rotation deskew already exists.** `mowgli_localization/scan_deskew_node.cpp` rotates each ray by `-ω·dt_i` from an interpolated IMU gyro buffer and publishes `/scan_deskewed`. Two real gaps:

1. **fusion_graph consumes raw `/scan`** (`fusion_graph_node.cpp:399`, no remap) — so the scan-matcher gets the rotation smear the deskew node already removes for the costmap. **Fix: remap fusion_graph's scan input to `/scan_deskewed`** (one line in `fusion_graph.launch.py` / `navigation.launch.py`). Free rotation deskew during pivots/turns.
2. **Translation deskew is explicitly skipped** in `scan_deskew_node` (its own comment: *"Linear motion compensation is currently skipped — at ≤0.3 m/s × ≤0.1 s scan, <30 mm displacement is negligible compared to the rotational smear"*). Correct for the costmap, **not for <2 cm scan-matching**: at 0.2 m/s mowing the forward smear is ~2 cm (~1 cm match bias — marginal); at 0.5 m/s transit ~5 cm (over budget). **Fix: extend `scan_deskew_node` with linear compensation** — it already has per-ray time + the IMU buffer; add a wheel-odom/twist buffer and translate each ray by `-v·dt_i` before the re-bin. Gate behind a twist threshold (degrade to current behaviour at `|twist|≈0`); verify the LD19 driver populates `time_increment`.

Doing the translation term **in `scan_deskew_node`** (not in fusion_graph's `OnScan`) keeps a single deskew source of truth, lets fusion_graph get both terms for free via the `/scan_deskewed` remap, and sharpens the costmap as a bonus. **Without the translation term the design floors at ~1–2.5 cm depending on speed** — borderline at mowing speed, over budget in transit.

## 7. Test plan

- **Unit (`test_factors.cpp`)**: `ScanToKeyframeFactor` `JacobianMatchesNumeric` (1e-4) + `ZeroErrorAtTrueValue` (`X = kf_abs_pose ∘ icp_delta⁻¹`).
- **Unit (`test_keyframe_map.cpp`, new)**: `AddKeyframe`/`FindKeyframesNearXY` (non-windowed, rebase-exempt), spatial-decimation eviction, and **the compose-direction lock** — assert `kf.abs_pose.compose(res.delta)` recovers the true current pose when `curr_scan == kf.scan_body` shifted by a known transform (defuses the mirror trap).
- **Unit (`test_persistence.cpp`)**: `.keyframes` round-trip; missing-file Load still succeeds; datum-mismatch rejection.
- **Rebase test**: a dock `RigidTransformAll` shifts keyframes by the same `correction`, even with a concurrent rebase landing.
- **Integration — the proof**: the **GPS-off coast-drift probe** — during a clean RTK-Fixed window near structure, artificially gate GPS off for 30/60/180 s and confirm `fusion ↔ (held) Fixed` stays < 2 cm with `use_keyframe_map:=true`, vs. decimetre drift with it off.

## 8. Error budget (Float, structured area, RSS)

| Component | 1σ | Source |
|---|---|---|
| Keyframe anchor (RTK-Fixed capture, frozen, rebase-exempt) | ~8 mm | RTK-Fixed σ 3–12 mm, Huber-fused |
| ICP scan-to-keyframe match (≥40 inliers, deskewed) | ~13 mm | `0.02 + rmse`, rmse 1–2 cm gated <10 cm |
| Deskew residual | ~5 mm | `scan_deskew_node` (rotation exists; add linear term) |
| Extrinsic / range quantization | ~4 mm | LD19 + base←scan |
| **RSS total** | **≈17 mm < 20 mm** | |

Crucially this is **absolute error, not drift** — every Float node gets a fresh unary pin to the mm-accurate map, so error stays bounded at ~1.7 cm across a multi-minute Float window instead of accumulating. Outside structure (T0–T2 fail) the design correctly does **not** claim 2 cm — it degrades to DR, the honest physical limit.

## 9. Risks & rollout

**Top risks** (all with mitigations above): compose-direction mirror trap (unit test + runtime divergence guard); capture-pose source (`out->pose` only); undeskewed/turning-scan capture poisoning a keyframe (deskew + angular-rate gate precede capture-enable); Fixed-flicker capture (debounce); 1-D wall-slide (T2); datum mismatch across gardens (write+verify `.meta` datum — also fixes the `graph_manager.hpp:535` promised-but-unwritten field); RK3588 throughput (rate-limit + K cap); FLS marginalization indeterminate-system (keep `ResetLocked` catch).

**Rollout**: ship Phase-1 keyframe layer behind `use_keyframe_map:=false`; build per-package (`make build-pkg PKG=fusion_graph`, `--parallel-workers 1` per the ARM note); field-enable on `dev` with the session monitor (new diagnostics: `keyframes_total`, `scan_to_keyframe_ok/fail`, `kf_match_rmse`); confirm `scan_to_keyframe_ok` fires during Float and `fusion↔gps` stays bounded near structure; then flip the default. Land the FLS core as a **separate** PR afterward.

### Design-review resolutions (re-derived after the verify phase was cut off)
- **Fixed-flicker capture** → added `kf_capture_debounce_s` (Fixed must hold ≥3 epochs before capture).
- **Mirror-image low-rmse factor** → runtime guard (`abs_meas` within `icp_max_divergence` of wheel-predicted `pred`) + the compose-direction unit test.
- **RK3588 throughput** → `kf_match_rate_hz`≈5 Hz decimation + K-nearest cap.
- **Deskew can add error** → gated behind twist threshold, degrades to single-extrinsic; verify LD19 `time_increment`.
- **Gauge-coupling race** → keyframe co-transform ordered inside the same `mu_` section before the D2 cleanup.
- **Datum binding** → write datum on Save, reject layer on datum mismatch at Load (closes the latent `:535` bug).
