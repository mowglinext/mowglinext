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
#include <iosfwd>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "fusion_graph/graph_params.hpp"
#include <Eigen/Core>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace fusion_graph
{

// Pose-variable key for node index i (symbol 'x'). Inline + namespace-scope so
// every graph_manager_*.cpp translation unit shares one definition (the class
// implementation is split across several TUs to stay within the file-size budget).
inline gtsam::Symbol PoseKey(uint64_t i)
{
  return gtsam::Symbol('x', i);
}

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

  // Scan-to-keyframe ABSOLUTE xy constraint to apply at next node creation as a
  // PoseTranslationPrior(X_curr, abs_xy). `abs_xy` is the map-frame position the
  // current node should have per an ICP match to a frozen keyframe. `robust`
  // wraps the noise model in Huber (a keyframe match on symmetric scenery can be
  // a gross outlier, like a wrong-fix GPS sample).
  void QueueScanToKeyframe(const gtsam::Vector2& abs_xy, double sigma_xy, bool robust = true);

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

  // ── RTK-anchored keyframe map ────────────────────────────────────
  //
  // A frozen keyframe: the absolute (map-frame) base_footprint pose at
  // RTK-Fixed capture time, plus the body-frame scan observed there.
  // Keyframes are NOT iSAM2 variables and live in a separate store, so
  // RebaseISAM2 / RigidTransformAll / PruneOldScans / the sliding-window
  // cutoff never touch them — their ~3 mm RTK precision survives by
  // construction. They are the absolute reference the scan-to-keyframe
  // factor uses to hold <2 cm during RTK-Float. Implemented in
  // graph_manager_keyframe.cpp.
  struct Keyframe
  {
    gtsam::Pose2 abs_pose;  // frozen GPS-fused map pose
    std::vector<Eigen::Vector2d> scan_body;  // base_footprint-frame points
  };

  // Capture a keyframe. `abs_pose` MUST be the GPS-fused node pose
  // (out->pose), never the raw antenna ENU. Enforces spatial decimation
  // (rejects within kf_spacing_m/2 of an existing keyframe) and the
  // max_keyframes cap. Returns the assigned id, or nullopt if decimated.
  std::optional<uint64_t> AddKeyframe(const gtsam::Pose2& abs_pose,
                                      const std::vector<Eigen::Vector2d>& scan_body);

  // Value-copy of a keyframe by id (lock-free use by the matcher), or
  // nullopt if the id is unknown.
  std::optional<Keyframe> GetKeyframe(uint64_t kf_id) const;

  // Up to `max_candidates` keyframe ids whose frozen abs_pose is within
  // `max_dist_m` (xy) of the query, sorted by ascending distance. No age
  // gate, no window cutoff — keyframes are the durable absolute map.
  std::vector<uint64_t> FindKeyframesNearXY(double x,
                                            double y,
                                            double max_dist_m,
                                            size_t max_candidates) const;

  // Number of keyframes currently stored (diagnostics).
  size_t KeyframeCount() const;

  // Drop the entire keyframe map. NOT called by the live-graph self-heal
  // reset (keyframes are reset-exempt) — only by an explicit operator
  // "delete maps" / clear_graph action.
  void ClearKeyframes();

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
  // Reset() body without taking mu_ — for callers that already hold the lock
  // (e.g. the iSAM2 indeterminate-system catch inside ApplyIsamUpdateLocked).
  void ResetLocked();

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
    // Scan-to-keyframe ABSOLUTE constraint: the pre-computed map-frame xy the
    // current node should have, derived from an ICP match to a frozen keyframe
    // (abs_xy = kf.abs_pose.compose(delta.inverse()).translation()). Applied as
    // a PoseTranslationPrior on X_curr — xy-only, so yaw stays owned by the
    // gyro/COG factors. Engaged only during RTK-Float (see fusion_graph_node).
    struct ScanToKeyframe
    {
      gtsam::Vector2 abs_xy;
      double sigma_xy;
      bool robust;
    };
    std::optional<ScanToKeyframe> scan_to_keyframe;
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
  // Count of iSAM2 indeterminate-system catches that triggered a graph
  // reset (instead of aborting the node). Nonzero = the graph hit an
  // ill-posed state and self-healed; investigate if it climbs.
  uint64_t stats_isam_resets_ = 0;

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

  // RTK-anchored keyframe map (see the public Keyframe API). Keyed by an
  // independent monotonic id because keyframes outlive the sliding pose
  // window and must not be coupled to next_index_ or the window cutoff.
  // Defined in graph_manager_keyframe.cpp; persisted in graph_manager.cpp.
  std::map<uint64_t, Keyframe> keyframes_;
  uint64_t next_keyframe_id_ = 0;

  // Helper — create a NoiseModel with diagonal sigmas.
  static gtsam::SharedNoiseModel MakeDiagonal(const std::vector<double>& sigmas);

  // Keyframe-map binary (de)serialization (defined in
  // graph_manager_keyframe.cpp). The <prefix>.keyframes file carries an
  // 'FGKF' magic + version header (unlike .scans) so it is self-describing.
  static void SerializeKeyframesBinary(std::ostream& os,
                                       const std::map<uint64_t, Keyframe>& keyframes);
  static bool DeserializeKeyframesBinary(std::istream& is, std::map<uint64_t, Keyframe>& keyframes);

  // Internal — actually creates the node and runs iSAM2. Caller must
  // hold mu_.
  // Returns std::nullopt when an ill-posed iSAM2 update forced a graph
  // ResetLocked() mid-node — the caller (Tick) must NOT publish a node
  // (the graph is empty; a node would carry the datum-origin pose).
  std::optional<TickOutput> CreateNodeLocked(double now_s);

  // Internal — wrap isam_.update so any factors/values added while an
  // async rebase is in progress are also captured in the pending
  // buffer (replayed onto the fresh iSAM2 before the swap). Caller
  // must hold mu_.
  // Returns false if the update hit an ill-posed system and triggered a
  // ResetLocked() — callers MUST bail (the graph is now empty/uninitialised;
  // continuing would publish a garbage origin pose).
  bool ApplyIsamUpdateLocked(const gtsam::NonlinearFactorGraph& fg, const gtsam::Values& values);
};

}  // namespace fusion_graph
