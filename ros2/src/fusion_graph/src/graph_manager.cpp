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
  const double bias_correction =
      params_.use_imu_preint ? current_bias_estimate_
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

void GraphManager::QueueScanToKeyframe(const gtsam::Vector2& abs_xy, double sigma_xy, bool robust)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (sigma_xy <= 0.0)
    sigma_xy = 0.1;
  queue_.scan_to_keyframe = UnaryQueue::ScanToKeyframe{abs_xy, sigma_xy, robust};
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

  // When IMU preintegration is on, seed the bias state at 0 with a
  // loose prior. iSAM2 refines it from the very first preint factor.
  if (params_.use_imu_preint)
  {
    using gtsam::Symbol;
    auto k_bias0 = Symbol('b', 0);
    new_values_.insert(k_bias0, 0.0);
    auto bias_prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
        gtsam::Vector1(params_.gyro_bias_prior_sigma_rad_per_s));
    new_factors_.add(
        gtsam::PriorFactor<double>(k_bias0, 0.0, bias_prior_noise));
  }

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

std::optional<TickOutput> GraphManager::LatestSnapshot() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return latest_;
}

uint64_t GraphManager::LiveNodeCount() const
{
  std::lock_guard<std::mutex> lock(mu_);
  const_cast<GraphManager*>(this)->RefreshEstimateLocked();
  uint64_t n = 0;
  for (const auto& kv : current_estimate_)
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
  s.scans_attached = scans_.size();
  s.loop_closures = loop_closures_added_;
  s.gps_rejects_wrongfix = stats_gps_rejects_wrongfix_;
  s.icp_rejects_rmse = stats_icp_rejects_rmse_;
  s.icp_rejects_inliers = stats_icp_rejects_inliers_;
  s.icp_rejects_sanity = stats_icp_rejects_sanity_;
  s.icp_rejects_divergence = stats_icp_rejects_divergence_;
  s.stationary_hand_push = stats_hand_push_;
  s.slip_veto = stats_slip_veto_;
  s.residual_ema_rad = residual_ema_;
  s.wheel_sigma_x_eff = last_wheel_sigma_x_eff_;
  s.gyro_bias_z = gyro_bias_z_;
  s.gyro_bias_updates = gyro_bias_updates_;
  return s;
}

void GraphManager::PeekAccumulator(double& dx,
                                   double& dy,
                                   double& dtheta_gyro,
                                   double& dtheta_wheel) const
{
  std::lock_guard<std::mutex> lock(mu_);
  dx = accum_.dx;
  dy = accum_.dy;
  dtheta_gyro = accum_.dtheta_gyro;
  dtheta_wheel = accum_.dtheta_wheel;
}

void GraphManager::RecordGpsRejectWrongFix()
{
  std::lock_guard<std::mutex> lock(mu_);
  ++stats_gps_rejects_wrongfix_;
}

void GraphManager::RecordIcpRejectRmse()
{
  std::lock_guard<std::mutex> lock(mu_);
  ++stats_icp_rejects_rmse_;
}
void GraphManager::RecordIcpRejectInliers()
{
  std::lock_guard<std::mutex> lock(mu_);
  ++stats_icp_rejects_inliers_;
}
void GraphManager::RecordIcpRejectSanity()
{
  std::lock_guard<std::mutex> lock(mu_);
  ++stats_icp_rejects_sanity_;
}
void GraphManager::RecordIcpRejectDivergence()
{
  std::lock_guard<std::mutex> lock(mu_);
  ++stats_icp_rejects_divergence_;
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
  if (!ApplyIsamUpdateLocked(fg, gtsam::Values()))
  {
    return;  // ill-posed reset; latest_ is now null, nothing to anchor
  }
  estimate_dirty_ = true;
  // Update latest_ snapshot so PublishOutputs sees the new pose.
  if (latest_ && latest_->node_index == node_index)
  {
    latest_->pose = PoseAt(node_index);
  }
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

}  // namespace fusion_graph
