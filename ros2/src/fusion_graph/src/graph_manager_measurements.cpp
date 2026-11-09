// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <algorithm>
#include <cmath>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include <gtsam/linear/NoiseModel.h>

namespace fusion_graph
{
void GraphManager::InvalidateGnssSupportLocked()
{
  ++graph_epoch_;
  applied_gnss_token_ = 0;
  applied_gnss_revision_ = 0;
  if (queue_.gnss)
    queue_.gnss->ingestion_token = 0;
  if (latest_)
  {
    latest_->graph_epoch = graph_epoch_;
    latest_->applied_gnss_token = 0;
    latest_->applied_gnss_revision = 0;
  }
}

void GraphManager::InvalidateGnssSupport()
{
  std::lock_guard<std::mutex> lock(mu_);
  InvalidateGnssSupportLocked();
}

uint64_t GraphManager::GraphEpoch() const
{
  return graph_epoch_.load(std::memory_order_acquire);
}

void GraphManager::AcknowledgeGnssLocked(uint64_t token)
{
  if (!latest_ || token == 0 || token <= applied_gnss_token_)
    return;
  applied_gnss_token_ = token;
  applied_gnss_revision_ = latest_->estimate_revision;
  latest_->applied_gnss_token = token;
  latest_->applied_gnss_revision = applied_gnss_revision_;
}

bool GraphManager::WheelStationaryLocked() const
{
  const double tick_scale = params_.node_period_s / kTunedNodePeriodS;
  return std::abs(accum_.dx) < params_.stationary_thresh_xy_m * tick_scale &&
         std::abs(accum_.dy) < params_.stationary_thresh_xy_m * tick_scale &&
         std::abs(accum_.dtheta_wheel) < params_.stationary_thresh_theta * tick_scale;
}

bool GraphManager::TrulyStationaryLocked() const
{
  return WheelStationaryLocked() &&
         !(accum_.max_abs_gyro_rad_per_s > params_.stationary_gyro_thresh_rad_per_s);
}

bool GraphManager::AddQueuedGnssFactorLocked(gtsam::NonlinearFactorGraph& factors,
                                             std::optional<uint64_t> new_node)
{
  if (!queue_.gnss)
    return false;
  const auto& q = *queue_.gnss;
  if (!q.target_node && !new_node)
    return false;  // preserve unstamped fixes for the next ordinary node
  if (q.target_node && !HasPoseAt(*q.target_node))
    return false;  // a rebase removed it; never redirect to a newer state
  gtsam::SharedNoiseModel noise = MakeDiagonal({q.sigma, q.sigma});
  if (q.robust)
    noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(
                                                  params_.huber_k_gps),
                                              noise);
  factors.add(GnssLeverArmFactor(PoseKey(q.target_node ? *q.target_node : *new_node),
                                 q.xy,
                                 gtsam::Vector2(params_.lever_arm_x, params_.lever_arm_y),
                                 noise));
  return true;
}

void GraphManager::ApplyQueuedGnssLocked()
{
  if (!queue_.gnss || !queue_.gnss->target_node)
    return;
  gtsam::NonlinearFactorGraph factors;
  const bool added = AddQueuedGnssFactorLocked(factors);
  const uint64_t token = queue_.gnss->ingestion_token;
  queue_.gnss.reset();  // consume once, including a target removed by rebase
  if (!added || !ApplyIsamUpdateLocked(factors, gtsam::Values()))
    return;
  estimate_dirty_ = true;
  // Historical factors can move the head too. No process state, other unary
  // queue, bias feedback or node epoch changes in this path.
  if (latest_)
  {
    RefreshLatestLocked(latest_->node_index, latest_->timestamp);
    AcknowledgeGnssLocked(token);
  }
}

void GraphManager::RefreshLatestLocked(uint64_t node_index, double timestamp)
{
  Eigen::Matrix3d cov = latest_ ? latest_->covariance : Eigen::Matrix3d::Identity();
  // The same bounded covariance cadence applies to ordinary and GNSS-only
  // updates. No empty optimizer updates merely to refresh this snapshot.
  if (++ticks_since_cov_ >= std::max(1, params_.cov_update_every_n))
  {
    cov = Eigen::Matrix3d::Identity();
    try
    {
      cov = isam_.marginalCovariance(PoseKey(node_index));
    }
    catch (const std::exception&)
    {
      // Retain the conservative default on marginal failure.
    }
    ticks_since_cov_ = 0;
  }
  latest_ = TickOutput{PoseAt(node_index),
                       cov,
                       node_index,
                       timestamp,
                       ++estimate_revision_,
                       graph_epoch_,
                       applied_gnss_token_,
                       applied_gnss_revision_};
}
}  // namespace fusion_graph
