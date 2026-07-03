// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphParams — tuning parameters for the GTSAM factor-graph localizer.
// Extracted from graph_manager.hpp to keep that header within the project's
// 600-line-per-file budget; included by graph_manager.hpp. Plain
// aggregate (no gtsam types) so it carries no heavy includes.

#pragma once

#include <cstdint>

namespace fusion_graph
{

struct GraphParams
{
  // Node creation cadence — one Pose2 per node_period_s of wall-clock.
  // 10 Hz default per the plan.
  double node_period_s = 0.1;

  // Wheel between-factor noise (sigmas, body-frame). Tight vy enforces
  // non-holonomic motion.
  double wheel_sigma_x = 0.05;  // m per node @ 10 Hz
  double wheel_sigma_y = 0.005;  // m per node — non-holo
  double wheel_sigma_theta = 0.01;  // rad per node

  // Gyro yaw between-factor noise (overrides wheel_sigma_theta when used).
  double gyro_sigma_theta = 0.005;  // rad per node — gyro is much
                                    // tighter than wheel-derived yaw.

  // GPS unary noise floor (when the message covariance is unrealistically
  // small).
  double gps_sigma_floor = 0.003;  // m — RTK-Fixed σ ~3 mm

  // Initial-pose prior noise — applied only at graph initialization.
  double prior_sigma_xy = 0.05;  // m
  double prior_sigma_theta = 0.05;  // rad

  // Huber kernel cutoff "k" for robustified factors. k is in σ-space:
  // residuals smaller than k σ stay quadratic, larger become linear.
  // GPS k=1.345 is the classic statistically-efficient default for
  // Gaussian inliers; yaw k=1.0 is tighter since mag bias is
  // heading-dependent and we want it pulled hard towards COG.
  double huber_k_gps = 1.345;
  double huber_k_yaw = 1.0;

  // GPS lever-arm in base_footprint frame (x forward, y left).
  double lever_arm_x = 0.0;
  double lever_arm_y = 0.0;

  // ── Datum (WGS84) ───────────────────────────────────────────────
  // Tags persisted maps so a keyframe map captured at one garden is
  // rejected at another (cross-site safety). (0,0) = unset → the
  // datum check is skipped (preserves self-seeded bootstrap reload).
  double datum_lat = 0.0;
  double datum_lon = 0.0;

  // ── RTK-anchored keyframe map ───────────────────────────────────
  // kf_spacing_m: minimum spacing between captured keyframes (spatial
  // decimation — a new keyframe within kf_spacing_m/2 of an existing
  // one is rejected). max_keyframes: hard cap on the stored map size
  // (0 = unbounded); oldest evicted when exceeded. The keyframe map is
  // the rebase-exempt absolute reference used to hold <2 cm during
  // RTK-Float; see graph_manager_keyframe.cpp.
  double kf_spacing_m = 0.5;
  uint64_t max_keyframes = 2000;

  // ── Performance ─────────────────────────────────────────────────
  // Recompute the per-tick marginal covariance only every Nth tick.
  // marginalCovariance is O(node_count) on the Bayes tree path and
  // dominates per-tick CPU once the graph passes ~3 k nodes; the
  // covariance value is consumed only by the diagnostics topic and
  // the published Odometry, neither of which need 10 Hz freshness.
  // Set to 1 to disable caching.
  int cov_update_every_n = 10;

  // iSAM2 relinearization throttle. 1 = relinearize every update
  // (max accuracy, max CPU). Higher values amortize Jacobian
  // recomputation across multiple updates with negligible accuracy
  // loss for our well-constrained Pose2 graph.
  int isam2_relinearize_skip = 5;

  // Sliding-window cap. RebaseISAM2 keeps only the most recent
  // `max_graph_nodes` pose nodes; everything older is dropped (its
  // accumulated constraints were already collapsed into the kept
  // nodes' priors during the rebase, so dropping them is loss-free
  // for the current estimate). Without this the graph grew unbounded
  // — observed 48,000+ nodes after a few sessions, which (a) blew up
  // iSAM2 marginal-covariance cost and pushed cov_xx to ~1 m, and (b)
  // kept stale far-away nodes anchoring the trajectory shape. 0 means
  // "no cap" (legacy behaviour: rebase keeps everything). At the 25 Hz
  // node rate, 3000 nodes ≈ 2 min of trajectory, which comfortably
  // covers a single mowing pass; LiDAR loop closures within that
  // window still function. Combined with isam2_rebase_every_nodes the
  // live graph oscillates in [max_graph_nodes, max_graph_nodes +
  // rebase_interval].
  uint64_t max_graph_nodes = 3000;

  // Stationary node-creation throttle. If the per-tick accumulator
  // shows ~zero motion (|dxy| < thresh AND |dtheta| < thresh), skip
  // node creation unless at least `stationary_node_period_s` has
  // elapsed since the last node. Prevents the graph from inflating
  // by 10 nodes/s while parked at the dock or paused mid-session.
  double stationary_motion_thresh_m = 0.02;  // m
  double stationary_motion_thresh_theta = 0.01;  // rad (~0.6°)
  double stationary_node_period_s = 5.0;  // 1 node / 5 s when still

  // Stationary detection (per-node wheel-accumulator thresholds). When
  // the wheel encoder reports motion strictly under all three thresholds
  // (|dx|, |dy|, |dtheta_wheel|), the BetweenFactor uses dtheta=0 with
  // stationary_sigma_theta — encoders cannot slip "into stationary", so
  // a zero wheel reading is taken as ground truth and the gyro bias
  // residual is suppressed regardless of its magnitude. (An earlier
  // iteration also required |dtheta_gyro| under stationary_thresh_theta;
  // on the live robot the gyro residual was ~10× the threshold, the AND
  // never fired, and yaw drift went the wrong way. Wheel-only is the
  // robust gate.) Set stationary_thresh_xy_m to a value smaller than
  // encoder noise per tick, and stationary_thresh_theta just above the
  // wheel-derived dtheta noise floor.
  double stationary_thresh_xy_m = 1.0e-3;  // 1 mm per node tick
  double stationary_thresh_theta = 2.0e-3;  // 0.11° per node tick (wheel noise floor)
  double stationary_sigma_theta = 1.0e-3;  // ≈ 0.057° BetweenFactor sigma when stationary

  // Pivot-mode wheel-translation downweight. During fast in-place
  // rotation the wheel encoders report a phantom forward vx in both
  // CW and CCW directions — measured 2026-05-14 at +0.021 m/s CW and
  // +0.026 m/s CCW under a 1 rad/s spin. The same-sign bias rules out
  // wheel-radius mismatch and points to one wheel under-magnituding
  // its backward rotation (motor deadband / encoder phase). With the
  // default wheel_sigma_x=0.05 m the wheel between-factor pulls
  // base_link forward by 0.2-0.4 m per spin, which Nav2 sees as path
  // deviation and re-plans on. When |per-tick gyro dtheta| crosses
  // pivot_gate_dtheta_rad, swap wheel_sigma_x for
  // pivot_wheel_sigma_x (effectively releasing the X constraint) so
  // GPS + scan-matching set XY. The gate scales with node_period_s
  // implicitly because dtheta = omega * dt; defaults are tuned for
  // 25 Hz (gate fires above ~0.3 rad/s) and remain reasonable for
  // 10 Hz (gate fires above ~0.12 rad/s).
  double pivot_gate_dtheta_rad = 0.012;  // rad per tick
  double pivot_wheel_sigma_x = 0.5;  // m — inflated sigma during pivot

  // Slip-veto thresholds (see Tick() implementation for rationale).
  // When the wheel-vs-gyro yaw delta disagreement exceeds
  // slip_residual_thresh_rad AND the gyro itself is below
  // slip_gyro_max_rad (i.e. the chassis isn't actually rotating much)
  // AND the wheel claims a non-trivial rotation, the BetweenFactor's
  // (dx, dy) is zeroed — wheels are skating and their translation is
  // a fiction. Defaults are tuned for 25 Hz nodes; thresholds are
  // in *per-tick* radians so the gate scales with node_period_s.
  //   slip_residual_thresh_rad = 0.01 (≈ 0.57° / tick = 14°/s @ 25 Hz)
  //   slip_gyro_max_rad        = 0.005 rad / tick (≈ 7°/s) — well
  //                              under any meaningful in-place pivot.
  //   slip_wheel_min_rad       = 0.005 rad / tick — wheel must be
  //                              claiming a non-trivial rotation for
  //                              the gate to apply.
  double slip_residual_thresh_rad = 0.01;
  double slip_gyro_max_rad = 0.005;
  double slip_wheel_min_rad = 0.005;

  // Stationary multi-source gate. The wheel-only gate (above) can be
  // tricked by encoders that report no motion while the robot is
  // hand-pushed / lifted — wheels free in mid-air read 0 ticks even
  // while base_link rotates. Cross-check against IMU gyro magnitude:
  // when wheel says stationary but |gyro_z| > stationary_gyro_thresh
  // (i.e. the robot is rotating despite still wheels), DON'T snap
  // dtheta to 0 — fall back to the gyro between-factor instead.
  //
  // 0.10 rad/s ≈ 5.7°/s sits comfortably above the live gyro residual
  // bias (~0.02-0.03 rad/s post-calibration drift) and below any
  // meaningful manual rotation (~0.5 rad/s is the slowest a human
  // intuitively pushes a robot). Tune up if false negatives appear
  // (robot rotated but treated as stationary); down if encoder noise
  // produces residual gyro samples on a truly parked robot.
  double stationary_gyro_thresh_rad_per_s = 0.10;

  // Full IMU preintegration with joint bias optimisation.
  //
  // When true, AddGyroDelta accumulates Σω·dt AND Σ(dt²·σ_gyro²) for
  // the variance, and CreateNodeLocked emits a ternary
  // GyroPreintFactor(X_prev, X_curr, bias_curr) along with a
  // random-walk BetweenFactor<double> on bias_{prev}→bias_{curr}.
  // The optimiser then solves for the trajectory AND the bias
  // jointly — same machinery GTSAM ships for the full Pose3 case,
  // tailored here to our planar Pose2 graph.
  //
  // When false (default), the existing pragmatic EMA bias estimator
  // remains active and the per-node BetweenFactor<Pose2> carries
  // yaw constraints as before.
  //
  // Set use_imu_preint=true in mowgli_robot.yaml to opt in. Keep
  // false until field-tested — the EMA path is well-validated and
  // the preint factor is new code.
  bool use_imu_preint = false;
  // Per-sample gyro_z noise sigma (rad/s). Used to propagate
  // covariance through the preintegration: σ²_preint = Σ (dt² · σ²).
  // Datasheet for the IMU on this robot gives ~0.01-0.02 rad/s noise
  // density; bias drift is handled separately by the bias state.
  double gyro_noise_density_rad_per_s = 0.015;
  // Bias random-walk noise (rad/s per √s). The bias state at node k
  // is constrained by a BetweenFactor<double>(bias_{k-1}, bias_k, 0,
  // σ = bias_rw · √dt). 0.001 rad/s/√s ≈ 0.06°/s per minute drift
  // tolerance — consistent with measured thermal drift on the WT901.
  double gyro_bias_rw_rad_per_s = 0.001;
  // Initial prior on the bias variable at graph start. Loose enough
  // that the first few observations dominate, tight enough to keep
  // iSAM2 well-conditioned. 0.05 rad/s ≈ 3°/s.
  double gyro_bias_prior_sigma_rad_per_s = 0.05;

  // Online gyro bias estimation (item #3, pragmatic variant).
  // hardware_bridge_node calibrates the gyro bias once at boot
  // (20 s window while docked) but the residual drifts with
  // temperature over the session — measured 0.025 rad/s after
  // 30 min of mowing on this robot. Without continuous re-estimation
  // the gyro between-factor accumulates the drift directly into the
  // graph's yaw estimate (8.5°/min @ 0.025 rad/s).
  //
  // Mechanism: when the wheel-only stationary gate fires (robot is
  // genuinely parked, encoders flat), EMA-update an estimate of the
  // gyro_z bias from the raw samples observed. The bias is then
  // SUBTRACTED from gyro samples in AddGyroDelta before integration
  // into the graph. When moving, the bias estimate is frozen.
  //
  // This is the pragmatic version of full IMU preintegration:
  // captures the dominant slow-drift bias term without requiring a
  // graph schema change (no bias state in the iSAM2 variables, just
  // a side-channel running EMA in graph_manager). Full preintegration
  // would let GTSAM optimise the bias jointly with the trajectory,
  // but on this robot the bias variation timescale (minutes) is much
  // slower than the graph optimisation cadence (20 ms) — the EMA
  // captures the same effect with two orders of magnitude less code.
  //
  // Set gyro_bias_estimation_enabled=false to disable entirely.
  bool gyro_bias_estimation_enabled = true;
  // EMA time constant for the bias estimate. 30 s = a few minutes of
  // stationary time to fully converge from a cold thermal state.
  double gyro_bias_ema_tau_s = 30.0;
  // Reject samples with |gyro_z| above this as "real motion despite
  // wheels saying stationary" — hand-pushed / lifted-spinning case.
  // Aligned with stationary_gyro_thresh_rad_per_s but kept separate
  // so the two gates can be tuned independently.
  double gyro_bias_max_sample_rad_per_s = 0.10;

  // Adaptive process-noise inflation on wheel σ_x.
  //
  // Diff-drive encoders report a body-frame translation per tick. If
  // one wheel slips (wet grass, slope, blade-jam recoil) the encoder
  // and the chassis disagree, but iSAM2 still treats the wheel
  // between-factor with the configured σ_x. The mismatch shows up as a
  // residual: dtheta_wheel ≠ dtheta_gyro when the angular motion was
  // identical, and the linear-axis equivalent (which we cannot observe
  // directly without an absolute sensor) is correlated with that yaw
  // residual on a diff-drive. We use |dtheta_wheel - dtheta_gyro| as a
  // proxy and inflate wheel_sigma_x when it exceeds a threshold.
  //
  // EMA-smoothed over recent ticks to avoid single-tick noise tripping
  // it; reverts to base sigma when residual decays. Disabled by setting
  // adaptive_noise_gain to 0.
  //
  // Gain interpretation: σ_x_eff = wheel_sigma_x + adaptive_noise_gain *
  // residual_ema. Default 10 (m per rad), tuned so a 0.1 rad slip event
  // (~6°) inflates σ_x from 0.05 to 1.05 m — effectively releases the X
  // constraint, similar to pivot_wheel_sigma_x.
  double adaptive_noise_enabled_gain = 10.0;
  // EMA time constant (seconds). 0.5 s = the residual settles within a
  // few ticks of slip onset; smaller values track faster but pick up
  // single-tick gyro noise.
  double adaptive_noise_ema_tau_s = 0.5;
  // Floor below which residual EMA is treated as zero (no inflation).
  // 0.005 rad sits at the typical gyro noise floor — anything below
  // this is dominated by sensor jitter, not real slip.
  double adaptive_noise_residual_floor_rad = 0.005;

  // ICP scan-match quality gates. Result is dropped if any of these
  // fail. Defaults are conservative — drop a few good matches in the
  // sparse-outdoor edge case rather than absorb a degenerate one,
  // because a single bad ICP delta corrupts iSAM2 trajectory for
  // many subsequent nodes.
  // icp_max_rmse_m: maximum acceptable RMS error over inliers (m).
  //   At our 50 Hz cadence with the 0.10 default, scans need ~3-10 cm
  //   of consistent matched-pair noise to be accepted — well within
  //   LiDAR distance accuracy on dock/tree/chassis features.
  // icp_max_delta_xy_m: per-node ICP delta (m) above which we treat
  //   the match as unphysical. At 50 Hz a 0.3 m delta implies ≥15 m/s
  //   robot velocity — impossible on a mower.
  // icp_max_delta_theta_rad: same idea, on rotation. 0.5 rad/tick at
  //   50 Hz = 25 rad/s, again physically impossible.
  // icp_max_divergence_xy_m / icp_max_divergence_theta_rad: maximum
  //   Mahalanobis-ish deviation of ICP result from its initial guess.
  //   When wheel+gyro init is good (per ICP init upgrade in same PR),
  //   ICP should refine it by mm — large divergences signal degenerate
  //   scenery (symmetric haie / pure-grass fields where any rotation
  //   has comparable score).
  double icp_max_rmse_m = 0.10;
  double icp_max_delta_xy_m = 0.30;
  double icp_max_delta_theta_rad = 0.50;
  double icp_max_divergence_xy_m = 0.15;
  double icp_max_divergence_theta_rad = 0.35;
};

}  // namespace fusion_graph
