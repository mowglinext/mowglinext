// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphManager — iSAM2 wrapper and sliding-window driver.
//
// Owns the factor graph, the values, and the per-tick logic:
//   1. Accumulate wheel twist + gyro_z between nodes.
//   2. On tick, create a new node X_k, add a between-factor from
//      X_{k-1} from the accumulated motion, and add any queued unary
//      factors (GPS, COG, mag).
//   3. Run iSAM2 update.
//   4. Return the latest optimized Pose2 + marginal covariance.
//
// The sliding window is implemented as a fixed-lag smoother (we keep
// the full graph but never reorder nodes older than the window). The
// plan called for explicit marginalization; that's a future cleanup —
// for now iSAM2's incremental Bayes-tree handles the cost adequately
// at our graph sizes (a few thousand nodes max per session).

#pragma once

#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

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

// What goes out to the publisher every tick.
struct TickOutput
{
  gtsam::Pose2 pose;
  Eigen::Matrix3d covariance;  // (x, y, theta) marginal
  uint64_t node_index;  // monotonically increasing
  double timestamp;  // ROS time, seconds
};

// Lightweight stats snapshot for diagnostics.
struct GraphStats
{
  uint64_t total_nodes = 0;  // # nodes created since boot
  uint64_t scans_attached = 0;  // # nodes with a scan
  uint64_t loop_closures = 0;  // # AddLoopClosure successes

  // Health counters (incremented by graph_manager; reset to 0 only on
  // process restart). Each counter buckets a specific rejection cause
  // so the diagnostics topic can show what's actually failing.
  uint64_t gps_rejects_wrongfix = 0;  // jump in /fix > thresh with stationary wheel
  uint64_t icp_rejects_rmse = 0;
  uint64_t icp_rejects_inliers = 0;  // ScanMatcher returned ok=false (min_inliers)
  uint64_t icp_rejects_sanity = 0;   // unphysical delta magnitude
  uint64_t icp_rejects_divergence = 0;  // result far from initial guess
  uint64_t stationary_hand_push = 0;  // wheel stationary but gyro disagrees
  uint64_t slip_veto = 0;  // ticks where wheel translation was vetoed by gyro
  // Adaptive process-noise telemetry. residual_ema_rad is the
  // current EMA-smoothed |dtheta_wheel - dtheta_gyro| (rad);
  // wheel_sigma_x_eff is the inflated σ_x actually used for the most
  // recent wheel between-factor.
  double residual_ema_rad = 0.0;
  double wheel_sigma_x_eff = 0.0;
  // Gyro bias telemetry. gyro_bias_z is the current estimate (rad/s)
  // that AddGyroDelta subtracts from raw samples; gyro_bias_updates
  // is the count of stationary samples that have contributed.
  double gyro_bias_z = 0.0;
  uint64_t gyro_bias_updates = 0;
};

class GraphManager
{
public:
  explicit GraphManager(const GraphParams& params);

  // Mutators — thread-safe (internal mutex). The node accepts inputs
  // from multiple ROS callbacks.

  // Wheel twist between samples — body-frame vx, vy, wz, dt.
  void AddWheelTwist(double vx, double vy, double wz, double dt);

  // Gyro yaw rate (rad/s) integrated with given dt.
  void AddGyroDelta(double wz, double dt);

  // GPS measurement (in map frame, datum-anchored). Cached and consumed
  // at next tick. sigma is per-axis; pass < 0 to use floor. When
  // `robust` is true, the noise model is wrapped in a Huber kernel —
  // appropriate for RTK-Float / single-fix samples where multipath
  // outliers can lie outside the reported covariance.
  void QueueGnss(double x, double y, double sigma_xy, bool robust = false);

  // Yaw observation (COG or mag). sigma_yaw is rad. `robust` should be
  // true for magnetometer yaw (uncalibrated / heading-dependent bias),
  // false for COG (gated on forward motion + RTK-Fixed).
  void QueueYaw(double yaw, double sigma_yaw, bool robust = false);

  // Scan-matching relative motion to apply at next node creation as a
  // BetweenFactor(X_{k-1}, X_k, delta, [sigma_xy, sigma_xy, sigma_theta]).
  // Applied in addition to the wheel between, both contribute under
  // their respective covariances.
  void QueueScanBetween(const gtsam::Pose2& delta, double sigma_xy, double sigma_theta);

  // Initial-pose seed. Required before the first tick if no GPS has
  // arrived yet — sets the prior on X_0. Must be called exactly once
  // (after Reset() it can be called again).
  // sigma_xy_override: when set, replaces the configured prior_sigma_xy
  // for this single Initialize call. Use it to seed with a tight prior
  // (~3 mm) when the seed comes from an RTK-Fixed GPS measurement so
  // the wheel between-factors can't drag the first few nodes off the
  // GPS-anchored origin.
  void Initialize(const gtsam::Pose2& X0,
                  double timestamp,
                  std::optional<double> sigma_xy_override = std::nullopt);

  // True once Initialize() has been called.
  bool IsInitialized() const
  {
    return initialized_;
  }

  // Tick: if at least node_period_s has elapsed since the last node,
  // create a new node + factors and run iSAM2. Returns the new tick
  // output, or nullopt if no node was created this call.
  std::optional<TickOutput> Tick(double now_s);

  // Read-only accessors (snapshot of current state).
  std::optional<TickOutput> LatestSnapshot() const;
  GraphStats Stats() const;

  // Count of pose ('x') variables currently live in the iSAM2 graph.
  // Distinct from GraphStats::total_nodes, which is the monotonic
  // next-index (never decreases). After a windowed RebaseISAM2 the
  // live count is capped at max_graph_nodes while total_nodes keeps
  // climbing. Exposed primarily for the sliding-window unit test.
  uint64_t LiveNodeCount() const;

  // Peek at the current per-tick wheel+gyro accumulator without
  // consuming it (Tick() resets the accumulator atomically). Used by
  // the ICP step to seed scan-matching with a non-identity initial
  // guess: composing wheel translation + gyro rotation since the
  // previous node creates a far better start than Pose2() for ICP's
  // brute-force NN search — especially under fast pivots where the
  // identity-init guess sends ICP looking 30°+ off true rotation.
  void PeekAccumulator(double& dx,
                       double& dy,
                       double& dtheta_gyro,
                       double& dtheta_wheel) const;

  // Health counters. fusion_graph_node calls these from its OnGnss
  // (wrong-fix detection) and OnTimer (ICP guard rails) paths. Each
  // call is mu_-locked and atomic.
  void RecordGpsRejectWrongFix();
  void RecordIcpRejectRmse();
  void RecordIcpRejectInliers();
  void RecordIcpRejectSanity();
  void RecordIcpRejectDivergence();

  // ── Visualization snapshots ─────────────────────────────────────
  // Optimized 2D pose for every variable currently in the iSAM2
  // estimate, keyed by node index. O(N) copy — call from a low-rate
  // viz timer, not the main tick.
  std::map<uint64_t, gtsam::Pose2> GetAllPoses() const;

  // Loop-closure edges accepted so far (prev_index, curr_index).
  // Bounded by loop_closures_added_; same memory life as scans_.
  std::vector<std::pair<uint64_t, uint64_t>> GetLoopClosureEdges() const;

  // ── Scan storage + loop closure ──────────────────────────────────
  //
  // Attach a scan (in body frame) to an existing node. Used for
  // future loop-closure searches and persisted to disk.
  void AttachScan(uint64_t node_index, const std::vector<Eigen::Vector2d>& scan_body);

  // Retrieve the scan stored at a node, or empty if none.
  std::vector<Eigen::Vector2d> GetScan(uint64_t node_index) const;

  // Lookup a node's optimized 2D pose (from current iSAM2 estimate).
  std::optional<gtsam::Pose2> GetPose(uint64_t node_index) const;

  // Find the K node indices closest to a query xy position
  // (Pose2.translation()), regardless of age. Used at cold boot to
  // narrow scan-matching candidates around dock_pose.
  std::vector<uint64_t> FindNodesNearXY(double x,
                                        double y,
                                        double max_dist_m,
                                        size_t max_candidates) const;

  // Force-anchor the current trajectory at `pose` by adding a tight
  // PriorFactor on the latest loaded node. Used after a successful
  // cold-boot scan-match relocalization. iSAM2 update happens
  // immediately; subsequent factors arrive on top.
  void ForceAnchor(uint64_t node_index,
                   const gtsam::Pose2& pose,
                   double sigma_xy,
                   double sigma_theta);

  // Find candidate node indices for loop closure: poses within
  // `max_dist_m` (xy plane) of `query_index`'s pose AND created
  // `min_age_s` seconds before now (so we don't loop-close to the
  // immediately preceding node, which is already constrained by the
  // wheel/scan between-factors).
  //
  // Returns at most `max_candidates` indices, sorted by ascending xy
  // distance to the query node.
  std::vector<uint64_t> FindLoopClosureCandidates(uint64_t query_index,
                                                  double max_dist_m,
                                                  double min_age_s,
                                                  size_t max_candidates) const;

  // ── Memory + compute bounding ───────────────────────────────────
  //
  // Drop per-node scans older than `max_age_nodes` to bound RAM. The
  // iSAM2 graph keeps the corresponding poses; only the scan blobs
  // (used for loop-closure candidates) are released.
  void PruneOldScans(uint64_t max_age_nodes);

  // Reset iSAM2 with the current optimized values as tight priors,
  // dropping every accumulated between/LC factor. Bounds the
  // per-tick update cost on long sessions where factor count grows
  // unbounded. Call periodically (e.g. every 2000 nodes); pose
  // estimates and the variable set are preserved.
  void RebaseISAM2();

  // Rigid-transform the entire trajectory by `correction` (applied as
  // X_k_new = correction * X_k for every Pose2 node). Relative
  // constraints between nodes (wheel between-factors, gyro between-
  // factors, scan_between, loop closures) are gauge-invariant under
  // this transform, so the graph topology and all LiDAR-derived
  // structure is preserved. The optimized solution is rebuilt with
  // priors at the shifted poses.
  //
  // Used at dock arrival when the latest node has accumulated drift
  // vs the operator-calibrated dock_pose: a "gauge reset" that snaps
  // the absolute frame so X_latest lands exactly on dock_pose,
  // without losing the persisted LiDAR scans / loop-closure structure
  // (which is what a Reset() would throw away). The latest node also
  // receives a tighter prior (5 mm / 0.3°) so future GPS factors take
  // longer to drift it back off the dock anchor.
  void RigidTransformAll(const gtsam::Pose2& correction, double latest_node_sigma_xy = 0.005,
                         double latest_node_sigma_theta = 0.005);

  // Add a loop-closure between-factor between two existing nodes.
  // delta is the relative Pose2 such that X_curr = X_prev * delta.
  // Triggers an iSAM2 update + returns the refreshed marginal pose
  // of the curr node.
  void AddLoopClosure(uint64_t prev_index,
                      uint64_t curr_index,
                      const gtsam::Pose2& delta,
                      double sigma_xy,
                      double sigma_theta);

  // ── Persistence ──────────────────────────────────────────────────
  //
  // Save the current graph + values + per-node scans to disk under
  // `prefix`:
  //   <prefix>.graph    -- gtsam factor graph + values (XML)
  //   <prefix>.scans    -- binary: per node, its body-frame scan
  //   <prefix>.meta     -- text: next_index, last_node_time_s, datum
  //
  // Idempotent; overwrites existing files. Returns false on I/O
  // error.
  bool Save(const std::string& prefix) const;

  // Load a previously-saved graph. The graph manager must NOT have
  // been initialized; on success, IsInitialized() becomes true and
  // next_index_ resumes after the highest loaded index.
  bool Load(const std::string& prefix);

  // Wipe all graph state (iSAM2, accumulators, scans, loop-closure
  // edges, queues, latest snapshot). After Reset() the manager is back
  // to its post-construction state — IsInitialized() returns false, and
  // a fresh Initialize() (or Load()) is required before any factor
  // input is accepted.
  // Used by the GUI / BT to start a clean session without restarting
  // the whole node (e.g. after relocating to a new garden).
  void Reset();

private:
  // Per-node accumulator for between-factors.
  struct Accumulator
  {
    double dx = 0.0;  // body-frame integration since last tick
    double dy = 0.0;
    double dtheta_wheel = 0.0;
    double dtheta_gyro = 0.0;
    double dt_total = 0.0;
    // Largest |gyro_z| (rad/s) seen this tick window. Used by the
    // stationary multi-source gate: when wheel says stationary but
    // this maximum exceeds stationary_gyro_thresh_rad_per_s, the
    // robot is being externally rotated (hand-pushed / lifted off
    // the ground) so don't snap dθ to 0.
    double max_abs_gyro_rad_per_s = 0.0;
    // IMU preintegration state (used when params_.use_imu_preint).
    // gyro_dt_total separately from dt_total because IMU may publish
    // at a different rate than wheel odom — we want strictly the
    // integrated gyro time horizon.
    double gyro_preint_dtheta = 0.0;
    double gyro_preint_dt = 0.0;
    double gyro_preint_variance = 0.0;  // Σ(dt_i² · σ_gyro²)
    void Reset()
    {
      *this = Accumulator{};
    }
  };

  // Cached unary observation queue.
  struct UnaryQueue
  {
    struct Gnss
    {
      gtsam::Vector2 xy;
      double sigma;
      bool robust;
    };
    struct Yaw
    {
      double yaw;
      double sigma;
      bool robust;
    };
    std::optional<Gnss> gnss;
    std::optional<Yaw> yaw;
    // Scan-matching between (delta, sigma_xy, sigma_theta). Applied
    // alongside the wheel between at the next node.
    struct ScanBetween
    {
      gtsam::Pose2 delta;
      double sigma_xy;
      double sigma_theta;
    };
    std::optional<ScanBetween> scan_between;
  };

  GraphParams params_;
  mutable std::mutex mu_;

  bool initialized_ = false;

  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph new_factors_;
  gtsam::Values new_values_;
  // Cached full estimate, refreshed lazily (only by callers that need
  // ALL nodes — viz markers + Save). Per-Tick / LC-search lookups go
  // straight to isam_.calculateEstimate<Pose2>(key), which is O(depth)
  // on the Bayes tree path rather than O(N) for the full extract.
  // estimate_dirty_ tells consumers the cache may be stale; call
  // RefreshEstimate() before iterating.
  mutable gtsam::Values current_estimate_;
  mutable bool estimate_dirty_ = true;
  // Helper: read one optimized Pose2 by node index. Returns pre-init
  // identity if iSAM2 doesn't know the key (early Tick paths).
  gtsam::Pose2 PoseAt(uint64_t idx) const;
  bool HasPoseAt(uint64_t idx) const;
  void RefreshEstimateLocked() const;

  uint64_t next_index_ = 0;  // index of the next node to create
  double last_node_time_s_ = 0.0;  // wall time of last created node

  Accumulator accum_;
  UnaryQueue queue_;

  std::optional<TickOutput> latest_;
  uint64_t loop_closures_added_ = 0;
  // How many ticks since the last marginalCovariance refresh; used to
  // throttle that O(N) call without losing covariance freshness on
  // the diagnostics + odom outputs.
  int ticks_since_cov_ = 0;
  std::vector<std::pair<uint64_t, uint64_t>> loop_closure_edges_;

  // Health counters surfaced via Stats(). All bumps go through the
  // Record*() mutators below so mu_ wraps them — Stats() makes a
  // single locked copy.
  uint64_t stats_gps_rejects_wrongfix_ = 0;
  uint64_t stats_icp_rejects_rmse_ = 0;
  uint64_t stats_icp_rejects_inliers_ = 0;
  uint64_t stats_icp_rejects_sanity_ = 0;
  uint64_t stats_icp_rejects_divergence_ = 0;
  uint64_t stats_hand_push_ = 0;
  uint64_t stats_slip_veto_ = 0;

  // Adaptive process-noise state.
  // residual_ema_ tracks the EMA-smoothed |dtheta_wheel - dtheta_gyro|
  // residual across ticks. Each Tick() updates it and reads back the
  // current σ_x inflation. Exposed via Stats() for diagnostics.
  double residual_ema_ = 0.0;
  double last_wheel_sigma_x_eff_ = 0.0;

  // Gyro bias estimation state (item #3 pragmatic). bias_ is the
  // current EMA-smoothed bias estimate (rad/s) subtracted from
  // incoming gyro samples in AddGyroDelta. bias_n_updates_ tracks
  // how many stationary samples have contributed — useful as a
  // diagnostic signal (bias converges in tens to hundreds of
  // samples depending on EMA τ and IMU rate).
  double gyro_bias_z_ = 0.0;
  uint64_t gyro_bias_updates_ = 0;

  // When use_imu_preint is true, this holds the latest optimised
  // bias value from iSAM2 (refreshed at every node creation). Used
  // as the linearisation point for the next preintegration window.
  double current_bias_estimate_ = 0.0;
  // Snapshot of "is the wheel-only stationary check currently
  // holding". Updated at each Tick from accum_ before the reset;
  // read by AddGyroDelta to gate EMA updates. Single-writer (Tick)
  // / single-reader (AddGyroDelta), both under mu_.
  bool wheel_stationary_now_ = false;

  // ── Async-rebase pipeline ───────────────────────────────────────
  // RebaseISAM2 rebuilds the iSAM2 Bayes tree from scratch with one
  // PriorFactor per existing variable. For a 50k-node graph that
  // update() call is ~1 s, and holding mu_ for that long stalls the
  // tick that publishes map→odom (observed 2026-05-14, caused
  // DockRobot to abort with `Transform data too old`). The fix is to
  // do the heavy iSAM2 rebuild OUTSIDE the lock: phase 1 snapshots
  // current_estimate_ under mu_; phase 2 builds the fresh iSAM2 on
  // that snapshot without holding mu_; phase 3 takes mu_ briefly to
  // replay anything Tick() added in the meantime and atomically swap
  // isam_. Tick() accumulates its post-snapshot factors/values into
  // rebase_pending_factors_ / rebase_pending_values_ when
  // rebase_in_progress_ is true.
  bool rebase_in_progress_ = false;
  gtsam::NonlinearFactorGraph rebase_pending_factors_;
  gtsam::Values rebase_pending_values_;

  // Scan storage. Map keeps memory bounded by the number of nodes
  // (we never delete; persistence drops everything to disk and a
  // reboot re-loads). Eigen::aligned_allocator is unnecessary for
  // Vector2d (8-byte alignment is fine on common targets).
  std::map<uint64_t, std::vector<Eigen::Vector2d>> scans_;

  // Helper — create a NoiseModel with diagonal sigmas.
  static gtsam::SharedNoiseModel MakeDiagonal(const std::vector<double>& sigmas);

  // Internal — actually creates the node and runs iSAM2. Caller must
  // hold mu_.
  TickOutput CreateNodeLocked(double now_s);

  // Internal — wrap isam_.update so any factors/values added while an
  // async rebase is in progress are also captured in the pending
  // buffer (replayed onto the fresh iSAM2 before the swap). Caller
  // must hold mu_.
  void ApplyIsamUpdateLocked(const gtsam::NonlinearFactorGraph& fg, const gtsam::Values& values);
};

}  // namespace fusion_graph
