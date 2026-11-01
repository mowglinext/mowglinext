// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphManager implementation — node creation: Tick + CreateNodeLocked. (The class implementation
// is split across several translation units to keep each file within the project's 600-line budget;
// all share graph_manager.hpp + the inline PoseKey().)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <ios>
#include <sstream>
#include <utility>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include "fusion_graph/lidar_covariance.hpp"
#include "process_window_policy.hpp"
#include "source_bootstrap_state.hpp"
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
std::optional<TickOutput> GraphManager::Tick(double now_s)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!initialized_ || (motion_ && motion_->bootstrap && motion_->bootstrap->provisional))
    return std::nullopt;
  // Self-heal a last_node_time_s_ parked in the FUTURE relative to the tick
  // clock. Load() clamps the restored timestamp to system_clock (wall time),
  // but Tick() is driven by the node clock — under use_sim_time those differ
  // by ~1.79e9 s, so the gate below would never fire and node creation would
  // freeze for the whole sim. Snapping down to now_s resumes the cadence and
  // is clock-source-agnostic (no-op on real hardware where they agree). The
  // timestamp index belongs to the old clock epoch, so rebuild it from the
  // latest live node rather than allowing new sensor stamps to match stale
  // entries after a bag loop / simulation reset.
  if (last_node_time_s_ > now_s)
  {
    queue_.lidar_map_xy.reset();
    last_node_time_s_ = now_s;
    node_time_index_.clear();
    if (latest_ && HasPoseAt(latest_->node_index))
      node_time_index_.emplace_back(now_s, latest_->node_index);
  }
  if (now_s - last_node_time_s_ < params_.node_period_s)
    return std::nullopt;

  // Stationary throttle: when the wheel + gyro accumulators show no
  // meaningful motion since the last node, drop the node period to
  // 1 / stationary_node_period_s. Stops the graph from inflating by
  // ~10 nodes/s while parked at the dock — both for memory bound
  // and to keep iSAM2 updates bounded.
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

std::optional<TickOutput> GraphManager::CreateNodeLocked(double now_s)
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

  // Select from a private policy snapshot. Source integration and factor
  // construction consume the selected values without advancing policy again.
  auto policy = SelectProcessWindowPolicy(params_,
                                          {accum_.dx,
                                           accum_.dy,
                                           accum_.wheel_travel_m,
                                           accum_.dtheta_wheel,
                                           accum_.dtheta_gyro,
                                           accum_.dt_total,
                                           accum_.max_abs_gyro_rad_per_s,
                                           last_node_time_s_,
                                           now_s},
                                          {slip_window_,
                                           residual_ema_,
                                           stats_hand_push_,
                                           stats_slip_veto_,
                                           wheel_stationary_now_,
                                           last_wheel_sigma_x_eff_});

  // Preserve the original adoption order around source integration: it can
  // throw before the residual EMA and effective sigma would have advanced.
  wheel_stationary_now_ = policy.state.wheel_stationary_now;
  if (policy.hand_push_detected)
    stats_hand_push_ = policy.state.hand_push_count;
  slip_window_ = std::move(policy.state.slip_window);
  if (policy.slip_detected)
    stats_slip_veto_ = policy.state.slip_veto_count;

  const auto between = SourceBetweenLocked(policy.fallback_mean,
                                           policy.wheel_translation_enabled,
                                           policy.gyro_rotation_enabled);

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

  // Adopt noise-policy state at its original point, after source integration
  // and the predicted value insertion have succeeded.
  if (params_.adaptive_noise_enabled_gain > 0.0)
    residual_ema_ = policy.state.residual_ema;
  last_wheel_sigma_x_eff_ = policy.state.last_wheel_sigma_x_eff;
  const double wheel_sigma_x_eff = policy.wheel_sigma_x;
  const double sigma_y_travel = policy.wheel_sigma_y;
  const double sigma_theta = policy.wheel_sigma_theta;

  auto between_noise = MakeDiagonal({
      wheel_sigma_x_eff,
      sigma_y_travel,
      sigma_theta,
  });
  new_factors_.add(gtsam::BetweenFactor<gtsam::Pose2>(k_prev, k_curr, between, between_noise));

  // ── IMU preintegration: ternary GyroPreintFactor + bias RW ──────
  // The preint factor adds a yaw constraint that depends on the
  // jointly-optimised bias variable, and the random-walk between-
  // factor links consecutive bias variables so iSAM2 can propagate
  // bias estimates through the trajectory.
  if (params_.use_imu_preint && accum_.gyro_preint_dt > 0.0)
  {
    using gtsam::Symbol;
    const auto k_bias_prev = Symbol('b', next_index_ - 1);
    const auto k_bias_curr = Symbol('b', next_index_);

    // Pose-only restores/reanchors intentionally discard historical bias
    // states. Establish the predecessor before extending its random walk.
    // A live reanchor keeps the current prediction estimate; a fresh load
    // starts from the configured cold-bias prior, not a prior-session sensor
    // calibration. Normal live intervals already have this key.
    if (!isam_->valueExists(k_bias_prev) && !new_values_.exists(k_bias_prev))
    {
      new_values_.insert(k_bias_prev, current_bias_estimate_);
      const auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
          gtsam::Vector1(params_.gyro_bias_prior_sigma_rad_per_s));
      new_factors_.add(
          gtsam::PriorFactor<double>(k_bias_prev, current_bias_estimate_, prior_noise));
    }

    // Insert the new bias variable initialised at the current best
    // estimate. iSAM2 will refine it on the next update.
    new_values_.insert(k_bias_curr, current_bias_estimate_);

    // Preint factor noise: sigma = √variance. Floor at a small value
    // to avoid a singular constraint when dt is tiny.
    const double sigma_preint = std::max(std::sqrt(accum_.gyro_preint_variance), 1e-4);
    auto preint_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(sigma_preint));
    new_factors_.add(GyroPreintFactor(k_prev,
                                      k_curr,
                                      k_bias_curr,
                                      accum_.gyro_preint_dtheta,
                                      accum_.gyro_preint_dt,
                                      preint_noise));

    // Bias random-walk between: bias_{k} = bias_{k-1} + N(0, σ_rw·√dt)
    const double sigma_bias_rw = params_.gyro_bias_rw_rad_per_s * std::sqrt(accum_.gyro_preint_dt);
    auto bias_rw_noise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(std::max(sigma_bias_rw, 1e-6)));
    new_factors_.add(gtsam::BetweenFactor<double>(k_bias_prev, k_bias_curr, 0.0, bias_rw_noise));
  }

  FreezeSourceEdgeLocked(k_prev,
                         k_curr,
                         X_prev,
                         X_pred,
                         gtsam::Vector3(wheel_sigma_x_eff, sigma_y_travel, sigma_theta));

  lidar_anchor_factors_ += AppendQueuedUnaryFactorsLocked(new_factors_, next_index_, now_s);

  // 4. iSAM2 update. Mark the cached full estimate dirty — callers
  //    that need ALL poses (viz / Save / LC search fallback) will
  //    refresh on demand. Per-Tick / per-LC lookups go through
  //    PoseAt() / HasPoseAt() which are O(depth) on the Bayes tree.
  if (!ApplyIsamUpdateLocked(new_factors_, new_values_))
  {
    // Graph was reset (ill-posed system). Don't publish a node this tick —
    // the manager is uninitialised; PoseAt(0) would return the datum origin
    // and teleport the robot. The next GPS/dock seed re-bootstraps.
    return std::nullopt;
  }
  estimate_dirty_ = true;
  new_factors_.resize(0);
  new_values_.clear();

  // 4b. Refresh the bias linearisation point from the new estimate
  //     when IMU preint is active. AddGyroDelta will subtract this
  //     from incoming samples until the next node creation.
  if (params_.use_imu_preint)
  {
    try
    {
      const auto k_bias = gtsam::Symbol('b', next_index_);
      current_bias_estimate_ = isam_->calculateEstimate<double>(k_bias);
    }
    catch (const std::exception&)
    {
      // Keep the previous estimate if iSAM2 hasn't materialised this
      // variable yet (shouldn't happen — we just inserted it).
    }
  }

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
      cov = isam_->marginalCovariance(k_curr);
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
  CommitSourceNodeLocked(out);
  latest_ = out;

  // 6. Reset for next tick.
  ++next_index_;
  last_node_time_s_ = now_s;
  node_time_index_.emplace_back(out.source_epoch || out.source_stamp_ns
                                    ? static_cast<double>(out.source_stamp_ns) * 1e-9
                                    : now_s,
                                out.node_index);
  const size_t time_index_cap =
      params_.max_graph_nodes > 0 ? static_cast<size_t>(params_.max_graph_nodes) : 10000U;
  while (node_time_index_.size() > std::max<size_t>(1U, time_index_cap))
    node_time_index_.pop_front();
  accum_.Reset();
  queue_.gnss.reset();
  queue_.yaw.reset();
  queue_.lidar_map_xy.reset();

  return out;
}

uint64_t GraphManager::AppendQueuedUnaryFactorsLocked(gtsam::NonlinearFactorGraph& factors,
                                                      uint64_t node_index,
                                                      double now_s) const
{
  uint64_t lidar_count = 0;
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
    const auto factor_key =
        queue_.gnss->target_node ? PoseKey(*queue_.gnss->target_node) : PoseKey(node_index);
    // A windowed asynchronous rebase may remove a historical target after
    // QueueGnss validated it. Drop that stale observation rather than silently
    // applying it to the current state at the wrong epoch.
    if (!queue_.gnss->target_node || HasPoseAt(*queue_.gnss->target_node))
    {
      factors.add(GnssLeverArmFactor(factor_key,
                                     queue_.gnss->xy,
                                     gtsam::Vector2(params_.lever_arm_x, params_.lever_arm_y),
                                     noise));
    }
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
    factors.add(YawUnaryFactor(PoseKey(node_index), queue_.yaw->yaw, noise));
  }
  if (queue_.lidar_map_xy)
  {
    const auto& q = *queue_.lidar_map_xy;
    const auto target = q.target.value_or(node_index);
    const auto cov = FloorLidarCovariance(q.cov, params_.lidar_anchor_sigma_floor_m);
    if (now_s <= q.expires_at && cov && q.xy.allFinite() && q.node_to_scan_map.allFinite() &&
        (target == node_index || HasPoseAt(target)))
    {
      gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Covariance(*cov);
      if (q.robust)
        noise =
            gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(1.345),
                                              noise);
      // Retiming to the graph node is done with a fixed map-frame displacement,
      // then the unary factor observes only node translation. Do not rotate the
      // offset by this node's yaw: a LiDAR map anchor must not observe heading.
      factors.add(gtsam::PoseTranslationPrior<gtsam::Pose2>(PoseKey(target),
                                                            q.xy - q.node_to_scan_map,
                                                            noise));
      ++lidar_count;
    }
  }

  return lidar_count;
}

}  // namespace fusion_graph
