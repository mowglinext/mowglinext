// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphManager implementation — core: ctor, accumulators, queues, init, accessors. (The class
// implementation is split across several translation units to keep each file within the project's
// 600-line budget; all share graph_manager.hpp + the inline PoseKey().)

#include "fusion_graph/graph_manager.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <ios>
#include <sstream>

#include "fusion_graph/factors.hpp"
#include <gtsam/base/GenericValue.h>
#include <gtsam/base/serialization.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PriorFactor.h>

namespace fusion_graph
{

// PoseKey() is a shared inline in graph_manager.hpp (used across the split TUs).

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
  // periodic checkpoint).
  if (!estimate_dirty_)
    return;
  // gtsam::Values has no move assignment: swap instead of deep-copying the
  // freshly built estimate a second time.
  gtsam::Values fresh = isam_.calculateEstimate();
  current_estimate_.swap(fresh);
  estimate_dirty_ = false;
}

GraphManager::GraphManager(const GraphParams& params)
    : params_(params), slip_window_(SlipWindowNodes(params.slip_window_s, params.node_period_s))
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

  // Track the running maximum on the RAW sample for the multi-source
  // stationary gate (item #1). Done before bias subtraction so a
  // drifty bias can't mask a real manual rotation.
  const double abs_wz_raw = std::abs(wz);
  if (abs_wz_raw > accum_.max_abs_gyro_rad_per_s)
    accum_.max_abs_gyro_rad_per_s = abs_wz_raw;

  // Online gyro bias estimation (item #3, pragmatic). When the last
  // Tick() flagged the wheel-only stationary state AND this sample
  // is plausibly bias-only (magnitude under the manual-rotation
  // threshold), EMA-update the bias estimate.
  if (params_.gyro_bias_estimation_enabled && wheel_stationary_now_ &&
      abs_wz_raw < params_.gyro_bias_max_sample_rad_per_s)
  {
    const double tau = std::max(params_.gyro_bias_ema_tau_s, 1.0e-3);
    const double alpha = dt / (tau + dt);
    gyro_bias_z_ = (1.0 - alpha) * gyro_bias_z_ + alpha * wz;
    ++gyro_bias_updates_;
  }

  // Subtract the current bias estimate before integration. First few
  // seconds use bias=0 (offline cal from hardware_bridge has removed
  // the cold bias); EMA refines as temperature drifts.
  //
  // When use_imu_preint is true, we DON'T pre-subtract the EMA bias —
  // the graph's bias variable absorbs the residual via the
  // GyroPreintFactor. We still apply the latest bias_estimate_at_node
  // (stored in current_bias_estimate_) so the integrated ω is in the
  // right ballpark for iSAM2's linearisation point.
  const double bias_correction = params_.use_imu_preint
                                     ? current_bias_estimate_
                                     : (params_.gyro_bias_estimation_enabled ? gyro_bias_z_ : 0.0);
  const double wz_corrected = wz - bias_correction;
  accum_.dtheta_gyro += wz_corrected * dt;

  // Preintegration accumulation. Variance propagates as
  // Σ(dt² · σ_gyro²) — this is the noise on the integrated ω, not on
  // ω itself. Independent of which bias correction is applied above.
  if (params_.use_imu_preint)
  {
    accum_.gyro_preint_dtheta += wz_corrected * dt;
    accum_.gyro_preint_dt += dt;
    const double sigma = params_.gyro_noise_density_rad_per_s;
    accum_.gyro_preint_variance += dt * dt * sigma * sigma;
  }
}

bool GraphManager::QueueGnss(double x,
                             double y,
                             double sigma_xy,
                             bool robust,
                             std::optional<uint64_t> target_node,
                             uint64_t ingestion_token)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (target_node && !HasPoseAt(*target_node))
    return false;
  if (sigma_xy < params_.gps_sigma_floor)
    sigma_xy = params_.gps_sigma_floor;
  queue_.gnss =
      UnaryQueue::Gnss{gtsam::Vector2(x, y), sigma_xy, robust, target_node, ingestion_token};
  return true;
}

void GraphManager::QueueYaw(double yaw, double sigma_yaw, bool robust)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (sigma_yaw <= 0.0)
    sigma_yaw = 0.05;
  queue_.yaw = UnaryQueue::Yaw{yaw, sigma_yaw, robust};
}

void GraphManager::QueueLidarMapXy(const gtsam::Vector2& xy,
                                   const Eigen::Matrix2d& cov,
                                   bool robust,
                                   std::optional<uint64_t> target,
                                   const gtsam::Vector2& node_to_scan_map,
                                   double expires_at)
{
  std::lock_guard<std::mutex> lock(mu_);
  queue_.lidar_map_xy =
      UnaryQueue::LidarMapXy{xy, cov, robust, target, node_to_scan_map, expires_at};
}

void GraphManager::ClearLidarObservations()
{
  std::lock_guard<std::mutex> lock(mu_);
  queue_.lidar_map_xy.reset();
}

void GraphManager::Initialize(const gtsam::Pose2& X0,
                              double timestamp,
                              std::optional<double> sigma_xy_override)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (initialized_)
    return;

  InvalidateGnssSupportLocked();
  const double sigma_xy = sigma_xy_override.value_or(params_.prior_sigma_xy);

  auto prior_noise = MakeDiagonal({
      sigma_xy,
      sigma_xy,
      params_.prior_sigma_theta,
  });

  auto k0 = PoseKey(0);
  new_values_.insert(k0, X0);
  new_factors_.add(gtsam::PriorFactor<gtsam::Pose2>(k0, X0, prior_noise));

  // When IMU preintegration is on, seed the bias state at 0 with a
  // loose prior. iSAM2 refines it from the very first preint factor.
  if (params_.use_imu_preint)
  {
    using gtsam::Symbol;
    auto k_bias0 = Symbol('b', 0);
    new_values_.insert(k_bias0, 0.0);
    auto bias_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector1(params_.gyro_bias_prior_sigma_rad_per_s));
    new_factors_.add(gtsam::PriorFactor<double>(k_bias0, 0.0, bias_prior_noise));
  }

  if (!ApplyIsamUpdateLocked(new_factors_, new_values_))
    return;
  estimate_dirty_ = true;
  new_factors_.resize(0);
  new_values_.clear();

  next_index_ = 1;
  last_node_time_s_ = timestamp;
  node_time_index_.clear();
  node_time_index_.emplace_back(timestamp, 0);
  initialized_ = true;

  TickOutput out;
  out.pose = X0;
  out.covariance = Eigen::Matrix3d::Identity() * (sigma_xy * sigma_xy);
  out.covariance(2, 2) = params_.prior_sigma_theta * params_.prior_sigma_theta;
  out.node_index = 0;
  out.timestamp = timestamp;
  out.estimate_revision = ++estimate_revision_;
  out.graph_epoch = graph_epoch_;
  latest_ = out;
}

std::optional<uint64_t> GraphManager::FindNodeAtOrBefore(double timestamp_s) const
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!initialized_ || !std::isfinite(timestamp_s))
    return std::nullopt;

  for (auto it = node_time_index_.rbegin(); it != node_time_index_.rend(); ++it)
  {
    if (it->first <= timestamp_s && HasPoseAt(it->second))
      return it->second;
  }
  return std::nullopt;
}

std::optional<TickOutput> GraphManager::LatestSnapshot() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return latest_;
}

uint64_t GraphManager::LiveNodeCount() const
{
  std::lock_guard<std::mutex> lock(mu_);
  // The estimate holds exactly the linearization point's keys, so count those
  // instead of materialising every pose (1 Hz diagnostics). getDelta() keeps
  // iSAM2's deferred back-substitution on the schedule the old full refresh
  // gave it, so later estimates are bit-for-bit unchanged.
  if (estimate_dirty_)
    isam_.getDelta();
  uint64_t n = 0;
  for (const auto& kv : isam_.getLinearizationPoint())
  {
    if (gtsam::Symbol(kv.key).chr() == 'x')
      ++n;
  }
  return n;
}

GraphStats GraphManager::Stats() const
{
  std::lock_guard<std::mutex> lock(mu_);
  GraphStats s;
  s.total_nodes = next_index_;
  s.successful_updates = successful_updates_;
  s.updates_since_rebase = successful_updates_ - rebased_updates_;
  s.successful_rebases = successful_rebases_;
  s.factor_slots = isam_.getFactorsUnsafe().size();
  s.live_factors = isam_.getFactorsUnsafe().nrFactors();
  s.rebase_in_progress = rebase_in_progress_;
  s.gps_rejects_wrongfix = stats_gps_rejects_wrongfix_;
  s.gps_rejects_dead_reckoning = stats_gps_rejects_dead_reckoning_;
  s.gps_rejects_stuck_value = stats_gps_rejects_stuck_value_;
  s.stationary_hand_push = stats_hand_push_;
  s.slip_veto = stats_slip_veto_;
  s.residual_ema_rad = residual_ema_;
  s.wheel_sigma_x_eff = last_wheel_sigma_x_eff_;
  s.gyro_bias_z = gyro_bias_z_;
  s.gyro_bias_updates = gyro_bias_updates_;
  return s;
}

void GraphManager::RecordGpsRejectWrongFix()
{
  std::lock_guard<std::mutex> lock(mu_);
  ++stats_gps_rejects_wrongfix_;
}

void GraphManager::RecordGpsRejectDeadReckoning()
{
  std::lock_guard<std::mutex> lock(mu_);
  ++stats_gps_rejects_dead_reckoning_;
}

void GraphManager::RecordGpsRejectStuckValue()
{
  std::lock_guard<std::mutex> lock(mu_);
  ++stats_gps_rejects_stuck_value_;
}

// ─────────────────────────────────────────────────────────────────────
// Pose access and anchoring
// ─────────────────────────────────────────────────────────────────────

std::optional<gtsam::Pose2> GraphManager::GetPose(uint64_t node_index) const
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!HasPoseAt(node_index))
    return std::nullopt;
  return PoseAt(node_index);
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
  if (!ApplyIsamUpdateLocked(fg, gtsam::Values()))
  {
    return;  // ill-posed reset; latest_ is now null, nothing to anchor
  }
  estimate_dirty_ = true;
  InvalidateGnssSupportLocked();
  // A historical anchor can also revise the head. Publication tracks estimate
  // revisions, so no DR recapture or special invalidation is needed here.
  if (latest_)
    RefreshLatestLocked(latest_->node_index, latest_->timestamp);
}

std::map<uint64_t, gtsam::Pose2> GraphManager::GetAllPoses() const
{
  std::lock_guard<std::mutex> lock(mu_);
  // Viz consumer (1 Hz markers): needs every node. Retract the poses
  // straight into the map, exactly as calculateEstimate() does, rather than
  // building and caching a full gtsam::Values (one heap node per variable).
  std::map<uint64_t, gtsam::Pose2> out;
  if (!estimate_dirty_)
  {
    for (const auto& kv : current_estimate_)
    {
      gtsam::Symbol s(kv.key);
      if (s.chr() == 'x')
        out.emplace(static_cast<uint64_t>(s.index()), kv.value.cast<gtsam::Pose2>());
    }
    return out;
  }
  // iSAM2 keeps one delta per linearization-point variable and, in a GTSAM
  // built without TBB (as shipped), both maps are key-ordered: walk them in
  // lockstep instead of a lookup per key (the lookup remains the fallback).
  // Same rule as Values::retract: retract every pose that has a delta (even a
  // zero one, which also normalises -0.0), copy one that has none.
  const gtsam::VectorValues& delta = isam_.getDelta();
  auto d = delta.begin();
  for (const auto& kv : isam_.getLinearizationPoint())
  {
    const gtsam::Vector* dx = nullptr;
    if (d != delta.end() && d->first == kv.key)
      dx = &(d++)->second;
    else if (const auto it = delta.find(kv.key); it != delta.end())
      dx = &it->second;
    gtsam::Symbol s(kv.key);
    if (s.chr() != 'x')
      continue;
    const auto& pose = kv.value.cast<gtsam::Pose2>();
    out.emplace_hint(out.end(),
                     static_cast<uint64_t>(s.index()),
                     dx ? gtsam::traits<gtsam::Pose2>::Retract(pose, *dx) : pose);
  }
  return out;
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
// The on-disk files are .graph (XML values archive) and .meta (text metadata).

}  // namespace fusion_graph
