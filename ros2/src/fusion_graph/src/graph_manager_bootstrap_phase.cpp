// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <utility>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include "source_bootstrap_state.hpp"
#include <gtsam/linear/NoiseModel.h>

namespace fusion_graph
{
bool GraphManager::SourceBootstrapContainsLocked(int64_t source_ns) const
{
  return motion_ && motion_->bootstrap && source_ns >= motion_->bootstrap->request.source_s_ns &&
         source_ns <= motion_->bootstrap->family->end_ns();
}

int64_t GraphManager::OldestSourceBoundaryLocked() const
{
  const auto& state = *motion_;
  int64_t oldest = initialized_ ? state.committed_ns : state.baseline_ns;
  if (!state.edges.empty())
    oldest = std::min(oldest, state.edges.front().bridge->edge().start_ns);
  if (state.bootstrap)
    oldest = std::min(oldest, state.bootstrap->request.source_s_ns);
  return oldest;
}

std::optional<TickOutput> GraphManager::PromoteSourceBootstrapLocked(int64_t schedule_ns)
{
  // Called only at the ordinary cadence/throttle decision, using the actual
  // public S..consumed accumulator. All failure paths leave the old joint live.
  try
  {
    const auto& state = *motion_;
    if (!state.bootstrap || !state.bootstrap->provisional || next_index_ != 1 ||
        state.pending_steps.empty() || state.consumed_ns <= state.committed_ns ||
        !new_factors_.empty() || !new_values_.empty())
      return std::nullopt;
    auto staged = std::make_shared<GraphMotionState>(state);
    auto phase = std::make_shared<SourceBootstrapState>(*state.bootstrap);
    auto& request = phase->request;
    request.promote = true;
    request.frontier_ns = state.consumed_ns;
    request.schedule_ns = schedule_ns;
    request.next_acquisition = state.next_acquisition;
    for (const auto& step : state.pending_steps)
      if (step.end_ns > phase->initial_end_ns)
        request.original_steps.push_back(step);
    request.suffix_policy_input = ProcessWindowPolicyInput{accum_.dx,
                                                           accum_.dy,
                                                           accum_.wheel_travel_m,
                                                           accum_.dtheta_wheel,
                                                           accum_.dtheta_gyro,
                                                           accum_.dt_total,
                                                           accum_.max_abs_gyro_rad_per_s,
                                                           last_node_time_s_,
                                                           static_cast<double>(schedule_ns) * 1e-9};
    for (const auto& observation : state.observations)
      if (observation.source_ns >= request.source_s_ns &&
          observation.source_ns <= state.consumed_ns)
      {
        request.additional_observations.push_back(observation);
        if (observation.yaw)
          ++staged->yaw_factors;
        else
          ++staged->gnss_factors;
      }
    const auto lidar_count =
        AppendQueuedUnaryFactorsLocked(request.extra_factors,
                                       1,
                                       static_cast<double>(schedule_ns) * 1e-9);
    auto candidate = BuildSourceBootstrapCandidate(request);
    if (!candidate.suffix_policy || candidate.frontier_key != PoseKey(1))
      return std::nullopt;
    const double bias = params_.use_imu_preint ? candidate.values.at<double>(gtsam::Symbol('b', 1))
                                               : current_bias_estimate_;
    phase->provisional = false;
    phase->family = std::move(candidate.family);
    phase->auxiliary_poses = std::move(candidate.auxiliary_poses);
    staged->bootstrap = std::move(phase);
    staged->next_acquisition = candidate.next_acquisition;
    staged->committed_ns = state.consumed_ns;
    staged->schedule_ns = schedule_ns;
    staged->last_committed_gyro = state.pending_steps.back().gyro.id;
    staged->pending_steps.clear();
    staged->pending_gyro_ids.clear();
    staged->validation.PruneBefore(state.epoch, state.consumed_ns);
    std::erase_if(staged->observations,
                  [&](const auto& observation)
                  {
                    return observation.source_ns <= state.consumed_ns;
                  });
    TickOutput snapshot;
    snapshot.pose = candidate.frontier_pose;
    snapshot.covariance = candidate.frontier_covariance;
    snapshot.node_index = 1;
    snapshot.timestamp = static_cast<double>(schedule_ns) * 1e-9;
    snapshot.source_stamp_ns = state.consumed_ns;
    snapshot.source_epoch = state.epoch;
    staged->revision = std::max(staged->revision, latest_->revision);
    snapshot.revision = ++staged->revision;
    auto times = node_time_index_;
    times.emplace_back(static_cast<double>(state.consumed_ns) * 1e-9, 1);
    const size_t cap = params_.max_graph_nodes > 0 ? params_.max_graph_nodes : 10000U;
    while (times.size() > std::max<size_t>(1, cap))
      times.pop_front();

    isam_.swap(candidate.isam);
    current_estimate_.swap(candidate.values);
    motion_.swap(staged);
    node_time_index_.swap(times);
    auto& policy = candidate.suffix_policy->state;
    slip_window_ = std::move(policy.slip_window);
    residual_ema_ = policy.residual_ema;
    stats_hand_push_ = policy.hand_push_count;
    stats_slip_veto_ = policy.slip_veto_count;
    wheel_stationary_now_ = policy.wheel_stationary_now;
    last_wheel_sigma_x_eff_ = policy.last_wheel_sigma_x_eff;
    current_bias_estimate_ = bias;
    lidar_anchor_factors_ += lidar_count;
    latest_ = snapshot;
    estimate_dirty_ = false;
    next_index_ = 2;
    last_node_time_s_ = snapshot.timestamp;
    ticks_since_cov_ = 0;
    accum_.Reset();
    queue_.gnss.reset();
    queue_.yaw.reset();
    queue_.lidar_map_xy.reset();
    return snapshot;
  }
  catch (const std::exception&)
  {
    return std::nullopt;
  }
}

bool GraphManager::AssimilateSourceBootstrapObservationsLocked(bool& updated)
{
  if (!motion_ || !motion_->bootstrap)
    return true;
  auto& state = *motion_;
  for (auto it = state.observations.begin(); it != state.observations.end();)
  {
    const auto& live = *state.bootstrap;
    if (it->source_ns < live.request.source_s_ns || it->source_ns > state.committed_ns ||
        it->source_ns > live.family->end_ns())
    {
      ++it;
      continue;
    }
    // Insert on an isolated copy. An unsupported chart or failed update must
    // not consume a key or mutate the live family's conditioning metadata.
    auto phase = std::make_shared<SourceBootstrapState>(live);
    phase->family = std::make_shared<SharedSourceProcessFamily>(*live.family);
    const auto current = isam_->calculateEstimate();
    SharedSourceProcessFamily::Insertion insertion;
    try
    {
      insertion = phase->family->Insert(it->source_ns,
                                        gtsam::Symbol('a', state.next_acquisition),
                                        &current);
    }
    catch (const UnsupportedRelativeConditional&)
    {
      ++it;
      continue;
    }
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    if (insertion.initial_value)
    {
      values.insert(insertion.key, *insertion.initial_value);
      factors.push_back(insertion.factor);
      phase->auxiliary_poses.emplace(insertion.key, it->source_ns);
    }
    auto noise = it->yaw ? MakeDiagonal({it->sigma}) : MakeDiagonal({it->sigma, it->sigma});
    if (it->robust)
      noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(
                                                    it->yaw ? params_.huber_k_yaw
                                                            : params_.huber_k_gps),
                                                noise);
    if (it->yaw)
      factors.emplace_shared<YawUnaryFactor>(insertion.key, *it->yaw, noise);
    else
      factors.emplace_shared<GnssLeverArmFactor>(
          insertion.key, it->xy, gtsam::Vector2(params_.lever_arm_x, params_.lever_arm_y), noise);
    if (phase->provisional)
      phase->request.additional_observations.push_back(*it);
    const bool allocated = insertion.initial_value.has_value();
    const bool yaw = it->yaw.has_value();
    if (!ApplyIsamUpdateLocked(factors, values))
      return false;
    state.bootstrap.swap(phase);
    if (allocated)
      ++state.next_acquisition;
    if (yaw)
      ++state.yaw_factors;
    else
      ++state.gnss_factors;
    updated = true;
    it = state.observations.erase(it);
  }
  return true;
}
}  // namespace fusion_graph
