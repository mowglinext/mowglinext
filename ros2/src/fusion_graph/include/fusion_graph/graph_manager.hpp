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
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "fusion_graph/graph_params.hpp"
#include "fusion_graph/slip_window.hpp"
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

  // Health counters (incremented by graph_manager; reset to 0 only on
  // process restart). Each counter buckets a specific rejection cause
  // so the diagnostics topic can show what's actually failing.
  uint64_t gps_rejects_wrongfix = 0;  // jump in /fix > thresh with stationary wheel
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
  // outliers can lie outside the reported covariance. When `target_node`
  // is set, the factor is attached to that existing pose instead of the
  // node created by the next Tick(). Returns false if the requested node
  // is no longer present in the live graph.
  bool QueueGnss(double x,
                 double y,
                 double sigma_xy,
                 bool robust = false,
                 std::optional<uint64_t> target_node = std::nullopt);

  // Yaw observation (COG or mag). sigma_yaw is rad. `robust` should be
  // true for magnetometer yaw (uncalibrated / heading-dependent bias),
  // false for COG (gated on forward motion + RTK-Fixed).
  void QueueYaw(double yaw, double sigma_yaw, bool robust = false);

  // LiDAR map anchor: absolute XY (map frame) with its 2x2 covariance from the
  // particle filter. A historical target plus node_to_scan observes the scan-time
  // body origin. Every principal variance is floored; invalid covariance is rejected.
  void QueueLidarMapXy(const gtsam::Vector2& xy,
                       const Eigen::Matrix2d& cov,
                       bool robust = true,
                       std::optional<uint64_t> target = std::nullopt,
                       const gtsam::Vector2& node_to_scan = gtsam::Vector2::Zero(),
                       double expires_at = std::numeric_limits<double>::infinity());
  void ClearLidarObservations();
  uint64_t LidarAnchorFactorCount() const
  {
    return lidar_anchor_factors_;
  }

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

  // Newest live pose node whose graph timestamp is at or before
  // `timestamp_s`. Sensor callbacks use this to associate delayed stamped
  // observations with the state they measured instead of the state at
  // callback-delivery time. Returns nullopt before the oldest indexed live
  // node, for non-finite timestamps, or while uninitialized.
  std::optional<uint64_t> FindNodeAtOrBefore(double timestamp_s) const;

  // Count of pose ('x') variables currently live in the iSAM2 graph.
  // Distinct from GraphStats::total_nodes, which is the monotonic
  // next-index (never decreases). After a windowed RebaseISAM2 the
  // live count is capped at max_graph_nodes while total_nodes keeps
  // climbing. Exposed primarily for the sliding-window unit test.
  uint64_t LiveNodeCount() const;

  // GPS wrong-fix rejection counter; mutex-protected.
  void RecordGpsRejectWrongFix();

  // ── Visualization snapshots ─────────────────────────────────────
  // Optimized 2D pose for every variable currently in the iSAM2
  // estimate, keyed by node index. O(N) copy — call from a low-rate
  // viz timer, not the main tick.
  std::map<uint64_t, gtsam::Pose2> GetAllPoses() const;

  // Lookup a node's optimized 2D pose (from current iSAM2 estimate).
  std::optional<gtsam::Pose2> GetPose(uint64_t node_index) const;

  // Force-anchor the current trajectory at `pose` by adding a tight
  // PriorFactor on a live node. Used by dock and operator pose seeds. Update happens
  // immediately; subsequent factors arrive on top.
  void ForceAnchor(uint64_t node_index,
                   const gtsam::Pose2& pose,
                   double sigma_xy,
                   double sigma_theta);

  // Reset iSAM2 with the current optimized values as tight priors,
  // dropping every accumulated factor. Bounds the
  // per-tick update cost on long sessions where factor count grows
  // unbounded. Call periodically (e.g. every 2000 nodes); pose
  // estimates and the variable set are preserved.
  void RebaseISAM2();

  // Apply a map-frame correction to every live pose and rebuild their priors.
  // Used at dock arrival to align the trajectory with the calibrated dock pose.
  // The latest pose receives a tighter prior to retain that dock anchor.
  void RigidTransformAll(const gtsam::Pose2& correction,
                         double latest_node_sigma_xy = 0.005,
                         double latest_node_sigma_theta = 0.005);

  // ── Persistence ──────────────────────────────────────────────────
  //
  // Save the current optimized values and metadata to disk under
  // `prefix`:
  //   <prefix>.graph    -- gtsam values (XML)
  //   <prefix>.meta     -- text: next_index, last_node_time_s, datum
  //
  // Idempotent; overwrites existing files. Returns false on I/O
  // error.
  bool Save(const std::string& prefix) const;

  // Load a previously-saved graph. The graph manager must NOT have
  // been initialized; on success, IsInitialized() becomes true and
  // next_index_ resumes after the highest loaded index.
  bool Load(const std::string& prefix);

  // Wipe all graph state (iSAM2, accumulators, queues, latest snapshot).
  // After Reset() the manager is back to its post-construction state — IsInitialized() returns
  // false, and a fresh Initialize() (or Load()) is required before any factor input is accepted.
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
  uint64_t lidar_anchor_factors_ = 0;  // PoseTranslationPriors added from the LiDAR map anchor
  struct UnaryQueue
  {
    struct Gnss
    {
      gtsam::Vector2 xy;
      double sigma;
      bool robust;
      std::optional<uint64_t> target_node;
    };
    struct Yaw
    {
      double yaw;
      double sigma;
      bool robust;
    };
    std::optional<Gnss> gnss;
    std::optional<Yaw> yaw;
    // LiDAR map anchor: ABSOLUTE XY from the particle filter localising
    // against the occupancy grid, with its full 2x2 covariance (anisotropic:
    // a single wall constrains across, not along). XY-ONLY on purpose: a
    // LiDAR-derived yaw once flipped map→odom ~180° (2026-07-22), which is
    // why this prior is XY-only and heading stays with the gyro/COG factors.
    struct LidarMapXy
    {
      gtsam::Vector2 xy;
      Eigen::Matrix2d cov;
      bool robust;
      std::optional<uint64_t> target;
      gtsam::Vector2 node_to_scan;
      double expires_at;
    };
    std::optional<LidarMapXy> lidar_map_xy;
  };

  GraphParams params_;
  mutable std::mutex mu_;

  bool initialized_ = false;

  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph new_factors_;
  gtsam::Values new_values_;
  // Cached full estimate, refreshed lazily (only by callers that need
  // ALL nodes — viz markers + Save). Per-Tick lookups go
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
  // Creation times for recent live pose nodes, in chronological order.
  // The deque is capped alongside the graph window; FindNodeAtOrBefore also
  // skips entries already removed by an asynchronous rebase.
  std::deque<std::pair<double, uint64_t>> node_time_index_;

  Accumulator accum_;
  // Per-node (dθ_wheel, dθ_gyro) ring the slip veto integrates over
  // (issue #516) — sized from slip_window_s / node_period_s in the ctor.
  SlipWindow slip_window_;
  UnaryQueue queue_;

  std::optional<TickOutput> latest_;
  // How many ticks since the last marginalCovariance refresh; used to
  // throttle that O(N) call without losing covariance freshness on
  // the diagnostics + odom outputs.
  int ticks_since_cov_ = 0;

  // Health counters surfaced via Stats(). All bumps go through the
  // Record*() mutators below so mu_ wraps them — Stats() makes a
  // single locked copy.
  uint64_t stats_gps_rejects_wrongfix_ = 0;
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

  // Helper — create a NoiseModel with diagonal sigmas.
  static gtsam::SharedNoiseModel MakeDiagonal(const std::vector<double>& sigmas);

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
