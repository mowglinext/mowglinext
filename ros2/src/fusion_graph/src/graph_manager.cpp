// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/graph_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <ios>
#include <sstream>

#include <gtsam/base/GenericValue.h>
#include <gtsam/base/serialization.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

// boost::serialization needs each polymorphic Value subtype registered
// in a TU before its container can round-trip through XML/binary. The
// only Value type our graph stores is GenericValue<Pose2>; the export
// must use the canonical GUID GTSAM uses internally so .graph files
// produced here can also be read by stock GTSAM tools.
#include <boost/serialization/export.hpp>
// cppcheck-suppress unknownMacro
BOOST_CLASS_EXPORT_GUID(gtsam::GenericValue<gtsam::Pose2>, "gtsam_GenericValue_Pose2")

#include "fusion_graph/factors.hpp"

namespace fusion_graph
{

namespace
{
constexpr unsigned char kPoseSym = 'x';

inline gtsam::Symbol PoseKey(uint64_t i)
{
  return gtsam::Symbol(kPoseSym, i);
}
}  // namespace

// ─────────────────────────────────────────────────────────────────────
// Internal: lazy estimate access
// ─────────────────────────────────────────────────────────────────────

bool GraphManager::HasPoseAt(uint64_t idx) const
{
  // valueExists is O(1); avoids the cost of try/catch on a missing key.
  return isam_.valueExists(PoseKey(idx));
}

gtsam::Pose2 GraphManager::PoseAt(uint64_t idx) const
{
  // O(depth) on the Bayes tree path. Much cheaper than the full
  // calculateEstimate() copy that returns ALL pose values.
  try
  {
    return isam_.calculateEstimate<gtsam::Pose2>(PoseKey(idx));
  }
  catch (const std::exception&)
  {
    return gtsam::Pose2();
  }
}

void GraphManager::RefreshEstimateLocked() const
{
  // O(N) full extraction. Only called by APIs that genuinely need
  // every pose: GetAllPoses (1 Hz viz markers), Save (manual /
  // periodic checkpoint), and FindLoopClosureCandidates fallback.
  if (!estimate_dirty_)
    return;
  current_estimate_ = isam_.calculateEstimate();
  estimate_dirty_ = false;
}

GraphManager::GraphManager(const GraphParams& params) : params_(params)
{
  gtsam::ISAM2Params p;
  // Gauss-Newton is faster than Dogleg here — graph is small (sliding
  // window ~600 nodes at 10 Hz × 60 s) and well-conditioned thanks to
  // the GPS unary prior on most nodes.
  p.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
  p.relinearizeThreshold = 0.05;
  p.relinearizeSkip = std::max(1, params_.isam2_relinearize_skip);
  isam_ = gtsam::ISAM2(p);
}

gtsam::SharedNoiseModel GraphManager::MakeDiagonal(const std::vector<double>& sigmas)
{
  gtsam::Vector v(sigmas.size());
  for (size_t i = 0; i < sigmas.size(); ++i)
    v[i] = sigmas[i];
  return gtsam::noiseModel::Diagonal::Sigmas(v);
}

void GraphManager::AddWheelTwist(double vx, double vy, double wz, double dt)
{
  if (dt <= 0.0)
    return;
  std::lock_guard<std::mutex> lock(mu_);
  // Body-frame integration. Yaw is integrated separately because gyro
  // is so much better than wheel-derived yaw on a differential drive.
  // We integrate wheel position assuming the yaw was constant over dt
  // — at 10 Hz nodes and < 0.5 rad/s rotation that's < 5 cm error which
  // the between-factor noise absorbs.
  accum_.dx += vx * dt;
  accum_.dy += vy * dt;
  accum_.dtheta_wheel += wz * dt;
  accum_.dt_total += dt;
}

void GraphManager::AddGyroDelta(double wz, double dt)
{
  if (dt <= 0.0)
    return;
  std::lock_guard<std::mutex> lock(mu_);
  accum_.dtheta_gyro += wz * dt;
}

void GraphManager::QueueGnss(double x, double y, double sigma_xy, bool robust)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (sigma_xy < params_.gps_sigma_floor)
    sigma_xy = params_.gps_sigma_floor;
  queue_.gnss = UnaryQueue::Gnss{gtsam::Vector2(x, y), sigma_xy, robust};
}

void GraphManager::QueueYaw(double yaw, double sigma_yaw, bool robust)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (sigma_yaw <= 0.0)
    sigma_yaw = 0.05;
  queue_.yaw = UnaryQueue::Yaw{yaw, sigma_yaw, robust};
}

void GraphManager::QueueScanBetween(const gtsam::Pose2& delta, double sigma_xy, double sigma_theta)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (sigma_xy <= 0.0)
    sigma_xy = 0.5;
  if (sigma_theta <= 0.0)
    sigma_theta = 0.1;
  queue_.scan_between = UnaryQueue::ScanBetween{delta, sigma_xy, sigma_theta};
}

void GraphManager::Initialize(const gtsam::Pose2& X0,
                              double timestamp,
                              std::optional<double> sigma_xy_override)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (initialized_)
    return;

  const double sigma_xy = sigma_xy_override.value_or(params_.prior_sigma_xy);

  auto prior_noise = MakeDiagonal({
      sigma_xy,
      sigma_xy,
      params_.prior_sigma_theta,
  });

  auto k0 = PoseKey(0);
  new_values_.insert(k0, X0);
  new_factors_.add(gtsam::PriorFactor<gtsam::Pose2>(k0, X0, prior_noise));

  isam_.update(new_factors_, new_values_);
  estimate_dirty_ = true;
  new_factors_.resize(0);
  new_values_.clear();

  next_index_ = 1;
  last_node_time_s_ = timestamp;
  initialized_ = true;

  TickOutput out;
  out.pose = X0;
  out.covariance = Eigen::Matrix3d::Identity() * (sigma_xy * sigma_xy);
  out.covariance(2, 2) = params_.prior_sigma_theta * params_.prior_sigma_theta;
  out.node_index = 0;
  out.timestamp = timestamp;
  latest_ = out;
}

std::optional<TickOutput> GraphManager::Tick(double now_s)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!initialized_)
    return std::nullopt;
  if (now_s - last_node_time_s_ < params_.node_period_s)
    return std::nullopt;

  // Stationary throttle: when the wheel + gyro accumulators show no
  // meaningful motion since the last node, drop the node period to
  // 1 / stationary_node_period_s. Stops the graph from inflating by
  // ~10 nodes/s while parked at the dock — both for memory bound
  // and to keep iSAM2 / LC search bounded.
  const double motion_xy_sq = accum_.dx * accum_.dx + accum_.dy * accum_.dy;
  const double abs_dtheta =
      std::abs(std::abs(accum_.dtheta_gyro) > 1e-9 ? accum_.dtheta_gyro : accum_.dtheta_wheel);
  const bool stationary =
      motion_xy_sq < params_.stationary_motion_thresh_m * params_.stationary_motion_thresh_m &&
      abs_dtheta < params_.stationary_motion_thresh_theta;
  if (stationary && now_s - last_node_time_s_ < params_.stationary_node_period_s)
  {
    return std::nullopt;
  }

  return CreateNodeLocked(now_s);
}

TickOutput GraphManager::CreateNodeLocked(double now_s)
{
  // Guard against next_index_ == 0: would underflow PoseKey(next_index_ - 1)
  // and crash GTSAM with "Symbol index is too large" when j wraps to 2^64-1.
  // Reachable historically when Load() restored an empty persisted graph
  // (next_index=0) and marked the manager initialized; both Save() and Load()
  // now refuse the empty case, but keep this defensive — the cost of the
  // check is one compare and the upside is no abort if a future code path
  // reintroduces the same hole.
  if (next_index_ == 0)
  {
    return latest_.value_or(TickOutput{});
  }

  // 1. Build the wheel between-factor: relative pose from X_{k-1} to X_k.
  //    Yaw selection rules:
  //    a. If the wheel encoder shows the robot stationary AND the gyro
  //       integrator hasn't seen a meaningful yaw change either, snap
  //       dtheta to 0 with a tight sigma. This suppresses the residual
  //       gyro bias (~0.01°/s mean post-hardware_bridge calibration)
  //       that otherwise accumulates ~0.4°/min of map-frame yaw drift
  //       while parked at the dock — measured 2026-05-03.
  //    b. Otherwise, prefer gyro: at speed the differential-drive yaw
  //       estimate is dominated by encoder slip and the gyro is strictly
  //       better. The wheel sigma_theta path only fires when no gyro
  //       sample arrived this tick (pre-cog seed window, IMU restart).
  const bool wheel_stationary =
      std::abs(accum_.dx) < params_.stationary_thresh_xy_m &&
      std::abs(accum_.dy) < params_.stationary_thresh_xy_m &&
      std::abs(accum_.dtheta_wheel) < params_.stationary_thresh_theta;
  const bool gyro_stationary =
      std::abs(accum_.dtheta_gyro) < params_.stationary_thresh_theta;

  double dtheta;
  double sigma_theta;
  if (wheel_stationary && gyro_stationary)
  {
    dtheta = 0.0;
    sigma_theta = params_.stationary_sigma_theta;
  }
  else if (std::abs(accum_.dtheta_gyro) > 1e-9)
  {
    dtheta = accum_.dtheta_gyro;
    sigma_theta = params_.gyro_sigma_theta;
  }
  else
  {
    dtheta = accum_.dtheta_wheel;
    sigma_theta = params_.wheel_sigma_theta;
  }

  const gtsam::Pose2 between(accum_.dx, accum_.dy, dtheta);

  const auto k_prev = PoseKey(next_index_ - 1);
  const auto k_curr = PoseKey(next_index_);

  // Predict X_k from current estimate of X_{k-1}, fall back to last
  // known pose if iSAM2 hasn't seen X_{k-1} yet (shouldn't happen).
  gtsam::Pose2 X_prev;
  if (HasPoseAt(next_index_ - 1))
  {
    X_prev = PoseAt(next_index_ - 1);
  }
  else
  {
    X_prev = latest_ ? latest_->pose : gtsam::Pose2();
  }
  const gtsam::Pose2 X_pred = X_prev.compose(between);
  new_values_.insert(k_curr, X_pred);

  // 2. Wheel between-factor.
  // sigma_theta was already selected above with the same wheel/gyro/
  // stationary logic that drove dtheta — reuse it here so both halves
  // of the BetweenFactor stay consistent.
  auto between_noise = MakeDiagonal({
      params_.wheel_sigma_x,
      params_.wheel_sigma_y,
      sigma_theta,
  });
  new_factors_.add(gtsam::BetweenFactor<gtsam::Pose2>(k_prev, k_curr, between, between_noise));

  // 3. Queued unary factors. Wrap in Huber when caller flagged the
  // measurement as outlier-prone (RTK-Float / single fix on GPS;
  // magnetometer on yaw).
  if (queue_.gnss)
  {
    gtsam::SharedNoiseModel noise = MakeDiagonal({
        queue_.gnss->sigma,
        queue_.gnss->sigma,
    });
    if (queue_.gnss->robust)
    {
      noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(
                                                    params_.huber_k_gps),
                                                noise);
    }
    new_factors_.add(GnssLeverArmFactor(
        k_curr, queue_.gnss->xy, gtsam::Vector2(params_.lever_arm_x, params_.lever_arm_y), noise));
  }
  if (queue_.yaw)
  {
    gtsam::SharedNoiseModel noise = MakeDiagonal({queue_.yaw->sigma});
    if (queue_.yaw->robust)
    {
      noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(
                                                    params_.huber_k_yaw),
                                                noise);
    }
    new_factors_.add(YawUnaryFactor(k_curr, queue_.yaw->yaw, noise));
  }
  if (queue_.scan_between)
  {
    auto noise = MakeDiagonal({
        queue_.scan_between->sigma_xy,
        queue_.scan_between->sigma_xy,
        queue_.scan_between->sigma_theta,
    });
    new_factors_.add(
        gtsam::BetweenFactor<gtsam::Pose2>(k_prev, k_curr, queue_.scan_between->delta, noise));
  }

  // 4. iSAM2 update. Mark the cached full estimate dirty — callers
  //    that need ALL poses (viz / Save / LC search fallback) will
  //    refresh on demand. Per-Tick / per-LC lookups go through
  //    PoseAt() / HasPoseAt() which are O(depth) on the Bayes tree.
  isam_.update(new_factors_, new_values_);
  estimate_dirty_ = true;
  new_factors_.resize(0);
  new_values_.clear();

  // 5. Marginal covariance — throttled. marginalCovariance is O(node
  //    count) on the Bayes tree path and dominates CPU once the graph
  //    passes a few thousand nodes. The value is only consumed by the
  //    diagnostics topic + published Odometry, neither of which needs
  //    10 Hz freshness — recomputing every Nth tick (default 10 → 1 Hz)
  //    keeps the displayed σ accurate without burning CPU on every
  //    Tick. Re-uses the previous tick's covariance when not due.
  Eigen::Matrix3d cov = Eigen::Matrix3d::Identity() * 1.0;
  ++ticks_since_cov_;
  const bool refresh_cov = ticks_since_cov_ >= std::max(1, params_.cov_update_every_n);
  if (refresh_cov)
  {
    try
    {
      cov = isam_.marginalCovariance(k_curr);
    }
    catch (const std::exception&)
    {
      // leave conservative default
    }
    ticks_since_cov_ = 0;
  }
  else if (latest_)
  {
    cov = latest_->covariance;
  }

  TickOutput out;
  out.pose = PoseAt(next_index_);
  out.covariance = cov;
  out.node_index = next_index_;
  out.timestamp = now_s;
  latest_ = out;

  // 6. Reset for next tick.
  ++next_index_;
  last_node_time_s_ = now_s;
  accum_.Reset();
  queue_.gnss.reset();
  queue_.yaw.reset();
  queue_.scan_between.reset();

  return out;
}

std::optional<TickOutput> GraphManager::LatestSnapshot() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return latest_;
}

GraphStats GraphManager::Stats() const
{
  std::lock_guard<std::mutex> lock(mu_);
  GraphStats s;
  s.total_nodes = next_index_;
  s.scans_attached = scans_.size();
  s.loop_closures = loop_closures_added_;
  return s;
}

// ─────────────────────────────────────────────────────────────────────
// Scan storage + loop closure
// ─────────────────────────────────────────────────────────────────────

void GraphManager::AttachScan(uint64_t node_index, const std::vector<Eigen::Vector2d>& scan)
{
  std::lock_guard<std::mutex> lock(mu_);
  scans_[node_index] = scan;
}

std::vector<Eigen::Vector2d> GraphManager::GetScan(uint64_t node_index) const
{
  std::lock_guard<std::mutex> lock(mu_);
  auto it = scans_.find(node_index);
  if (it == scans_.end())
    return {};
  return it->second;
}

std::optional<gtsam::Pose2> GraphManager::GetPose(uint64_t node_index) const
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!HasPoseAt(node_index))
    return std::nullopt;
  return PoseAt(node_index);
}

std::vector<uint64_t> GraphManager::FindNodesNearXY(double x,
                                                    double y,
                                                    double max_dist_m,
                                                    size_t max_candidates) const
{
  std::lock_guard<std::mutex> lock(mu_);
  std::vector<std::pair<double, uint64_t>> hits;
  hits.reserve(scans_.size());
  const double max_d2 = max_dist_m * max_dist_m;
  for (const auto& [idx, _] : scans_)
  {
    if (!HasPoseAt(idx))
      continue;
    const auto X = PoseAt(idx);
    const double dx = X.x() - x;
    const double dy = X.y() - y;
    const double d2 = dx * dx + dy * dy;
    if (d2 <= max_d2)
      hits.emplace_back(d2, idx);
  }
  std::sort(hits.begin(), hits.end());
  std::vector<uint64_t> out;
  out.reserve(std::min(max_candidates, hits.size()));
  for (size_t i = 0; i < hits.size() && out.size() < max_candidates; ++i)
    out.push_back(hits[i].second);
  return out;
}

void GraphManager::ForceAnchor(uint64_t node_index,
                               const gtsam::Pose2& pose,
                               double sigma_xy,
                               double sigma_theta)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (sigma_xy <= 0.0)
    sigma_xy = 0.05;
  if (sigma_theta <= 0.0)
    sigma_theta = 0.05;
  if (!HasPoseAt(node_index))
    return;
  auto noise = MakeDiagonal({sigma_xy, sigma_xy, sigma_theta});
  gtsam::NonlinearFactorGraph fg;
  fg.add(gtsam::PriorFactor<gtsam::Pose2>(PoseKey(node_index), pose, noise));
  isam_.update(fg, gtsam::Values());
  estimate_dirty_ = true;
  // Update latest_ snapshot so PublishOutputs sees the new pose.
  if (latest_ && latest_->node_index == node_index)
  {
    latest_->pose = PoseAt(node_index);
  }
}

std::vector<uint64_t> GraphManager::FindLoopClosureCandidates(uint64_t query_index,
                                                              double max_dist_m,
                                                              double min_age_s,
                                                              size_t max_candidates) const
{
  std::lock_guard<std::mutex> lock(mu_);
  std::vector<uint64_t> out;
  if (next_index_ == 0)
    return out;

  if (!HasPoseAt(query_index))
    return out;
  const auto Xq = PoseAt(query_index);

  // Per-node age proxy: nodes are created at node_period_s cadence,
  // so age_idx = (next - i) * node_period_s. Within ±10% of wall
  // clock, sufficient for the >30s gate.
  const double age_per_idx = params_.node_period_s;
  const auto cutoff_idx_diff = static_cast<uint64_t>(std::ceil(min_age_s / age_per_idx));

  // Linear scan over scans_ keys (== nodes with a stored scan, which
  // is what we want — no point loop-closing to a node without a
  // scan). PoseAt is O(depth) on the Bayes tree path, so this loop
  // is roughly O(scans_.size() · depth).
  std::vector<std::pair<double, uint64_t>> hits;
  hits.reserve(scans_.size());
  const double max_d2 = max_dist_m * max_dist_m;
  for (const auto& [idx, _] : scans_)
  {
    if (idx == query_index)
      continue;
    if (query_index - idx < cutoff_idx_diff)
      continue;
    if (!HasPoseAt(idx))
      continue;
    const auto X = PoseAt(idx);
    const double dx = X.x() - Xq.x();
    const double dy = X.y() - Xq.y();
    const double d2 = dx * dx + dy * dy;
    if (d2 <= max_d2)
      hits.emplace_back(d2, idx);
  }

  std::sort(hits.begin(), hits.end());
  for (size_t i = 0; i < hits.size() && out.size() < max_candidates; ++i)
    out.push_back(hits[i].second);
  return out;
}

void GraphManager::PruneOldScans(uint64_t max_age_nodes)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (next_index_ <= max_age_nodes)
    return;
  const uint64_t cutoff = next_index_ - max_age_nodes;
  auto it = scans_.begin();
  while (it != scans_.end())
  {
    if (it->first < cutoff)
      it = scans_.erase(it);
    else
      break;  // map is ordered by key, rest is newer
  }
}

void GraphManager::RebaseISAM2()
{
  std::lock_guard<std::mutex> lock(mu_);
  // Rebase needs the full pose set. This is one of the rare callers
  // where the O(N) extraction is unavoidable — we need every variable
  // to seed the fresh tree.
  RefreshEstimateLocked();
  if (current_estimate_.empty())
    return;

  // Build a fresh iSAM2 with the same parameters as the live one.
  gtsam::ISAM2Params p;
  p.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
  p.relinearizeThreshold = 0.05;
  p.relinearizeSkip = std::max(1, params_.isam2_relinearize_skip);
  gtsam::ISAM2 fresh(p);

  // Re-anchor every existing variable with a tight prior. The exact
  // sigma is a balance: too tight and future loop closures can't move
  // anything; too loose and iSAM2 wanders. 5 cm / 3° matches typical
  // RTK + COG noise floors and keeps the rebase non-destructive.
  gtsam::NonlinearFactorGraph fg;
  auto noise = MakeDiagonal({0.05, 0.05, 0.05});
  for (const auto& kv : current_estimate_)
  {
    fg.add(gtsam::PriorFactor<gtsam::Pose2>(kv.key, kv.value.cast<gtsam::Pose2>(), noise));
  }

  fresh.update(fg, current_estimate_);
  isam_ = std::move(fresh);
  estimate_dirty_ = true;

  // Loop-closure edges accumulated so far were collapsed into the
  // priors; reset the visualization list so future LCs are
  // distinguishable from the rebased history.
  loop_closure_edges_.clear();
}

void GraphManager::AddLoopClosure(uint64_t prev_index,
                                  uint64_t curr_index,
                                  const gtsam::Pose2& delta,
                                  double sigma_xy,
                                  double sigma_theta)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (sigma_xy <= 0.0)
    sigma_xy = 0.5;
  if (sigma_theta <= 0.0)
    sigma_theta = 0.1;

  if (!HasPoseAt(prev_index) || !HasPoseAt(curr_index))
    return;
  auto k_prev = PoseKey(prev_index);
  auto k_curr = PoseKey(curr_index);

  auto noise = MakeDiagonal({sigma_xy, sigma_xy, sigma_theta});
  gtsam::NonlinearFactorGraph fg;
  fg.add(gtsam::BetweenFactor<gtsam::Pose2>(k_prev, k_curr, delta, noise));

  isam_.update(fg, gtsam::Values());
  estimate_dirty_ = true;
  ++loop_closures_added_;
  loop_closure_edges_.emplace_back(prev_index, curr_index);
}

std::map<uint64_t, gtsam::Pose2> GraphManager::GetAllPoses() const
{
  std::lock_guard<std::mutex> lock(mu_);
  // Viz consumer: needs every node, so refresh the cached estimate.
  // Throttled by the caller (1 Hz markers) — not on the per-Tick path.
  RefreshEstimateLocked();
  std::map<uint64_t, gtsam::Pose2> out;
  for (const auto& kv : current_estimate_)
  {
    gtsam::Symbol s(kv.key);
    if (s.chr() != 'x')
      continue;
    out.emplace(static_cast<uint64_t>(s.index()), kv.value.cast<gtsam::Pose2>());
  }
  return out;
}

std::vector<std::pair<uint64_t, uint64_t>> GraphManager::GetLoopClosureEdges() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return loop_closure_edges_;
}

// ─────────────────────────────────────────────────────────────────────
// Persistence
// ─────────────────────────────────────────────────────────────────────
//
// We serialize the iSAM2 *result* — current_estimate_ + a summarized
// factor graph — rather than ISAM2 itself (whose internal Bayes-tree
// state is GTSAM-version-sensitive). A fresh boot rebuilds the Bayes
// tree by replaying a single PriorFactor on each node and re-adding
// the between-factors as we observe new ones.
//
// The on-disk format is a 3-tuple of files: .graph (XML, gtsam
// archive), .scans (binary, our own format), .meta (text key=value).

namespace
{

void SerializeScansBinary(const std::map<uint64_t, std::vector<Eigen::Vector2d>>& scans,
                          std::ostream& os)
{
  uint64_t n = scans.size();
  os.write(reinterpret_cast<const char*>(&n), sizeof(n));
  for (const auto& [idx, pts] : scans)
  {
    os.write(reinterpret_cast<const char*>(&idx), sizeof(idx));
    uint64_t m = pts.size();
    os.write(reinterpret_cast<const char*>(&m), sizeof(m));
    for (const auto& p : pts)
    {
      double xy[2] = {p.x(), p.y()};
      os.write(reinterpret_cast<const char*>(xy), sizeof(xy));
    }
  }
}

bool DeserializeScansBinary(std::istream& is,
                            std::map<uint64_t, std::vector<Eigen::Vector2d>>& scans)
{
  uint64_t n = 0;
  if (!is.read(reinterpret_cast<char*>(&n), sizeof(n)))
    return false;
  for (uint64_t i = 0; i < n; ++i)
  {
    uint64_t idx = 0, m = 0;
    if (!is.read(reinterpret_cast<char*>(&idx), sizeof(idx)))
      return false;
    if (!is.read(reinterpret_cast<char*>(&m), sizeof(m)))
      return false;
    std::vector<Eigen::Vector2d> pts;
    pts.reserve(m);
    for (uint64_t j = 0; j < m; ++j)
    {
      double xy[2];
      if (!is.read(reinterpret_cast<char*>(xy), sizeof(xy)))
        return false;
      pts.emplace_back(xy[0], xy[1]);
    }
    scans.emplace(idx, std::move(pts));
  }
  return true;
}

}  // namespace

void GraphManager::Reset()
{
  std::lock_guard<std::mutex> lock(mu_);
  // Rebuild iSAM2 with the same parameters used in the constructor —
  // GTSAM has no public clear() API.
  gtsam::ISAM2Params p;
  p.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
  p.relinearizeThreshold = 0.05;
  p.relinearizeSkip = std::max(1, params_.isam2_relinearize_skip);
  isam_ = gtsam::ISAM2(p);

  new_factors_ = gtsam::NonlinearFactorGraph();
  new_values_ = gtsam::Values();
  current_estimate_ = gtsam::Values();
  estimate_dirty_ = true;

  next_index_ = 0;
  last_node_time_s_ = 0.0;
  initialized_ = false;

  accum_.Reset();
  queue_ = UnaryQueue{};

  latest_.reset();
  loop_closures_added_ = 0;
  ticks_since_cov_ = 0;
  loop_closure_edges_.clear();
  scans_.clear();
}

bool GraphManager::Save(const std::string& prefix) const
{
  std::lock_guard<std::mutex> lock(mu_);
  // Refuse to persist an empty graph. An auto-save fired right after a
  // Reset() would otherwise overwrite the on-disk files with
  // next_index=0 / count=0; on next launch Load() restored that state,
  // marked initialized_, and the first Tick crashed with a Symbol-index
  // underflow. Keep whatever good state was on disk before the reset.
  if (!initialized_ || next_index_ == 0)
    return false;
  // Manual / on-checkpoint path. Always refreshes from iSAM2 — the
  // serialized state must reflect all factor updates since the last
  // RefreshEstimateLocked() call.
  RefreshEstimateLocked();
  try
  {
    std::ofstream graph_os(prefix + ".graph");
    if (!graph_os)
      return false;
    graph_os << gtsam::serializeXML(current_estimate_);
    graph_os.close();

    std::ofstream scans_os(prefix + ".scans", std::ios::binary);
    if (!scans_os)
      return false;
    SerializeScansBinary(scans_, scans_os);
    scans_os.close();

    std::ofstream meta_os(prefix + ".meta");
    if (!meta_os)
      return false;
    meta_os << "next_index=" << next_index_ << "\n";
    // Wall-clock seconds need ≥10 integer digits + a few fractional, so
    // default 6-digit iostream precision (1.7774e+09) silently corrupts
    // the timestamp. setprecision(15) is safe for double round-trip.
    meta_os << "last_node_time_s=" << std::fixed << std::setprecision(6) << last_node_time_s_
            << "\n";
    meta_os.close();
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "fusion_graph::Save: %s\n", e.what());
    return false;
  }
  return true;
}

bool GraphManager::Load(const std::string& prefix)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (initialized_)
    return false;

  gtsam::Values loaded_values;
  try
  {
    std::ifstream graph_is(prefix + ".graph");
    if (!graph_is)
      return false;
    std::stringstream buf;
    buf << graph_is.rdbuf();
    gtsam::deserializeXML(buf.str(), loaded_values);
  }
  catch (const std::exception&)
  {
    return false;
  }

  std::map<uint64_t, std::vector<Eigen::Vector2d>> loaded_scans;
  try
  {
    std::ifstream scans_is(prefix + ".scans", std::ios::binary);
    if (!scans_is)
      return false;
    if (!DeserializeScansBinary(scans_is, loaded_scans))
      return false;
  }
  catch (const std::exception&)
  {
    return false;
  }

  uint64_t next_idx = 0;
  double last_t = 0.0;
  try
  {
    std::ifstream meta_is(prefix + ".meta");
    if (!meta_is)
      return false;
    std::string line;
    while (std::getline(meta_is, line))
    {
      auto eq = line.find('=');
      if (eq == std::string::npos)
        continue;
      const std::string key = line.substr(0, eq);
      const std::string val = line.substr(eq + 1);
      if (key == "next_index")
        next_idx = std::stoull(val);
      else if (key == "last_node_time_s")
        last_t = std::stod(val);
    }
  }
  catch (const std::exception&)
  {
    return false;
  }

  // Refuse to restore a degenerate persisted state. With next_idx == 0
  // (or no values at all) marking initialized_ would let CreateNodeLocked
  // form PoseKey(next_idx - 1) and underflow into a 2^64-1 Symbol index
  // — GTSAM throws std::invalid_argument and the process aborts. Treat
  // this as "no graph on disk" so the node bootstraps from the next GPS
  // / set_pose seed.
  if (next_idx == 0 || loaded_values.empty())
    return false;

  // Re-seed iSAM2 with each loaded pose pinned by a tight prior; the
  // priors keep optimization stable as new wheel/GPS factors arrive.
  // The covariances are looser than the live priors so loop-closures
  // can still re-balance the loaded portion if it was inconsistent.
  auto pin_noise = MakeDiagonal({0.01, 0.01, 0.01});
  gtsam::NonlinearFactorGraph fg;
  for (const auto& key_value : loaded_values)
  {
    fg.add(gtsam::PriorFactor<gtsam::Pose2>(key_value.key,
                                            key_value.value.cast<gtsam::Pose2>(),
                                            pin_noise));
  }
  isam_.update(fg, loaded_values);
  estimate_dirty_ = true;

  scans_ = std::move(loaded_scans);
  next_index_ = next_idx;
  // Clamp loaded timestamp to "now" so a meta written with stale
  // precision (legacy iostream default, e.g. "1.7774e+09") doesn't
  // park last_node_time_s_ in the future, which would block Tick()
  // from creating any new node until wall-clock catches up.
  const double now_s =
      std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
  last_node_time_s_ = std::min(last_t, now_s);
  initialized_ = true;

  return true;
}

}  // namespace fusion_graph
