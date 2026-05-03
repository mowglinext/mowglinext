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
};

}  // namespace fusion_graph
