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
// Periodic rebases marginalize removed history into a linear separator
// factor and retain the original nonlinear factors in the live window.

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

struct ConsumedMotionStep;
struct GraphMotionState;

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
  double timestamp;  // Schedule time, seconds
  int64_t source_stamp_ns = 0;
  uint64_t source_epoch = 0;
  uint64_t revision = 0;
};

// Lightweight stats snapshot for diagnostics.
struct GraphStats
{
  uint64_t total_nodes = 0;  // # nodes created since boot

  // Health counters (incremented by graph_manager; reset to 0 only on
  // process restart). Each counter buckets a specific rejection cause
  // so the diagnostics topic can show what's actually failing.
  uint64_t gps_rejects_wrongfix = 0;  // jump in /fix > thresh with stationary wheel
  // /fix samples withheld while LocalizationMonitorNode reports DEAD_RECKONING
  // (mowglinext#694) — its own /gps/fix + /gps/status pairing caught a payload
  // frozen at an identical position under a live, advancing receipt stamp,
  // which OnGnss's own receipt-stamp dedup cannot distinguish from genuine new
  // observations. A nonzero, climbing count without a matching drop in fix
  // rate means the receiver is delivering stale payloads under fresh stamps.
  uint64_t gps_rejects_dead_reckoning = 0;
  // /fix samples withheld by the direct payload-value comparison
  // (mowglinext#694, gps_stuck_gate.hpp) — wheel travel accumulated while
  // the reported lat/lon stayed bit-identical exceeded the stuck threshold.
  // Distinct from gps_rejects_dead_reckoning above: field data showed the
  // DEAD_RECKONING verdict never fired for the exact freeze this counter
  // catches, so the two counters tell you which mechanism actually acted.
  uint64_t gps_rejects_stuck_value = 0;
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
  double AddGyroDelta(double wz, double dt);

  // Source-time path: states end at wholly consumed integration boundaries.
  bool AddConsumedMotionStep(const ConsumedMotionStep& step);
  void ResetConsumedMotionEpoch(uint64_t epoch);
  void SetConsumedMotionBaseline(uint64_t epoch, int64_t source_ns);
  bool InitializeSource(const gtsam::Pose2& pose,
                        int64_t schedule_ns,
                        int64_t source_ns,
                        uint64_t epoch,
                        std::optional<double> sigma_xy = std::nullopt);
  std::optional<TickOutput> TickSource(int64_t schedule_ns, int64_t source_ns, uint64_t epoch);
  bool QueueGnssAtEpoch(double x,
                        double y,
                        double sigma,
                        bool robust,
                        int64_t source_ns,
                        uint64_t observation_id,
                        uint64_t epoch);
  std::optional<int64_t> OldestRetainedSourceNs(uint64_t epoch) const;
  // Read-only optimized acquisition pose or frozen first-order conditional
  // mean. Only committed, retained source support is eligible; no new value,
  // factor, observation identity, revision or cadence state is introduced.
  std::optional<gtsam::Pose2> EstimatePoseAtSource(uint64_t epoch, int64_t source_ns) const;
  uint64_t SourceGnssFactorCount() const;
  // Dynamic absolute headings retain source time and a separate identity
  // namespace from GNSS. COG/Mag callers distinguish their producer IDs.
  bool QueueYawAtEpoch(double yaw,
                       double sigma,
                       bool robust,
                       int64_t source_ns,
                       uint64_t observation_id,
                       uint64_t epoch);
  uint64_t SourceYawFactorCount() const;
  bool NeedsSourceRebase() const;

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

  // LiDAR map anchor: absolute scan-time XY (map frame) with its 2x2 covariance
  // from the particle filter. node_to_scan_map is the fixed map-frame displacement
  // from the target node to the scan time; it retimes the XY observation without
  // adding a yaw Jacobian. Every principal variance is floored; invalid covariance
  // is rejected.
  void QueueLidarMapXy(const gtsam::Vector2& xy,
                       const Eigen::Matrix2d& cov,
                       bool robust = true,
                       std::optional<uint64_t> target = std::nullopt,
                       const gtsam::Vector2& node_to_scan_map = gtsam::Vector2::Zero(),
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

  // Count of cadence ('x') and acquisition ('a') Pose2 states live in iSAM2.
  // Distinct from GraphStats::total_nodes, which is the monotonic
  // next-index (never decreases). After a windowed RebaseISAM2 the
  // live count is capped at max_graph_nodes while total_nodes keeps
  // climbing. Growth between maintenance checks is allowed. Exposed primarily
  // for the sliding-window unit test.
  uint64_t LiveNodeCount() const;

  // GPS wrong-fix rejection counter; mutex-protected.
  void RecordGpsRejectWrongFix();

  // GPS dead-reckoning-payload rejection counter; mutex-protected.
  void RecordGpsRejectDeadReckoning();

  // GPS stuck-payload-value rejection counter; mutex-protected.
  void RecordGpsRejectStuckValue();

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

  // Rebuild iSAM2, retaining the nonlinear factors in the live window and
  // marginalizing removed history at its current linearization point.
  // This preserves the retained joint information and process correlations.
  // Call periodically (e.g. every 2000 nodes) to bound graph history.
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
  //   <prefix>.checkpoint -- framed graph XML + metadata, one generation
  //   <prefix>.checkpoint.previous -- retained complete recovery generation
  //
  // Returns false on write/flush/sync/close/publication errors. After a
  // publication error either complete generation may be visible; the previous
  // remains recoverable. One writer per prefix, serialized by the node.
  bool Save(const std::string& prefix) const;

  // Load a previously-saved graph. The graph manager must NOT have
  // been initialized; on success, IsInitialized() becomes true and
  // next_index_ resumes after the highest loaded index. Tries the previous
  // complete generation if current is corrupt; legacy .graph/.meta are read
  // only when neither modern file exists.
  bool Load(const std::string& prefix);

  // Wipe all graph state (iSAM2, accumulators, queues, latest snapshot).
  // After Reset() the manager is back to its post-construction state — IsInitialized() returns
  // false, and a fresh Initialize() (or Load()) is required before any factor input is accepted.
  // Used by the GUI / BT to start a clean session without restarting
  // the whole node (e.g. after relocating to a new garden).
  void Reset();

private:
  friend class GraphManagerRebaseTestAccess;
  friend class GraphManagerSourceTestAccess;

  // Reset() body without taking mu_ — for callers that already hold the lock
  // (e.g. the iSAM2 indeterminate-system catch inside ApplyIsamUpdateLocked).
  void ResetLocked();
  void InitializeLocked(const gtsam::Pose2& pose, double timestamp, std::optional<double> sigma_xy);

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
    double gyro_preint_dtheta = 0.0;  // raw integral; factor applies absolute bias once
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
      gtsam::Vector2 node_to_scan_map;
      double expires_at;
    };
    std::optional<LidarMapXy> lidar_map_xy;
  };

  std::shared_ptr<GraphMotionState> motion_;
  gtsam::Pose2 SourceBetweenLocked(const gtsam::Pose2& fallback, bool translation, bool rotation);
  void FreezeSourceEdgeLocked(gtsam::Key left,
                              gtsam::Key right,
                              const gtsam::Pose2& previous,
                              const gtsam::Pose2& predicted,
                              const gtsam::Vector3& sigmas);
  void CommitSourceNodeLocked(TickOutput& output);
  bool AssimilateSourceObservationsLocked();
  void InvalidateSourceBridgesLocked();
  void PruneSourceStateLocked();
  std::map<gtsam::Key, uint64_t> AcquisitionOwnersLocked() const;

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
  uint64_t stats_gps_rejects_dead_reckoning_ = 0;
  uint64_t stats_gps_rejects_stuck_value_ = 0;
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
  // Snapshot factors/values under mu_, marginalize and rebuild outside mu_,
  // then replay post-snapshot updates and swap under mu_. Keeping the heavy
  // work off the lock prevents maintenance from stalling TF publication.
  // Reset/RigidTransform cancels by clearing the flag. Each new worker owns
  // a distinct generation so a cancelled worker cannot consume another's
  // pending buffer or overwrite its result.
  bool rebase_in_progress_ = false;
  uint64_t rebase_generation_ = 0;
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
