// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <utility>

#include "fusion_graph/graph_manager.hpp"
#include "graph_heading_seed.hpp"
#include "source_bootstrap_state.hpp"

namespace fusion_graph
{
SourceBootstrapResult GraphManager::InitializeSourceObservationPair(
    int64_t schedule_ns, int64_t source_ns, uint64_t epoch, uint64_t gnss_id, uint64_t yaw_id)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (initialized_ || !motion_ || motion_->epoch != epoch || !motion_->baseline_set ||
      source_ns < motion_->baseline_ns || source_ns > motion_->consumed_ns)
    return SourceBootstrapResult::PendingProofUnavailable;
  const auto& state = *motion_;
  const auto position = std::find_if(state.observations.begin(),
                                     state.observations.end(),
                                     [&](const auto& observation)
                                     {
                                       return !observation.yaw && observation.id == gnss_id &&
                                              observation.source_ns == source_ns;
                                     });
  const auto heading = std::find_if(state.observations.begin(),
                                    state.observations.end(),
                                    [&](const auto& observation)
                                    {
                                      return observation.yaw && observation.id == yaw_id;
                                    });
  const auto admitted_position = state.observation_ids.find(gnss_id);
  const auto admitted_heading = state.yaw_observation_ids.find(yaw_id);
  if (position == state.observations.end() || heading == state.observations.end() ||
      admitted_position == state.observation_ids.end() || admitted_position->second != source_ns ||
      admitted_heading == state.yaw_observation_ids.end() ||
      admitted_heading->second != heading->source_ns || heading->source_ns < state.baseline_ns ||
      heading->source_ns > state.consumed_ns)
    return SourceBootstrapResult::PendingProofUnavailable;
  if (heading->source_ns == source_ns)
    return InitializeSourceObservationsLocked(schedule_ns, source_ns, epoch, gnss_id, yaw_id);

  try
  {
    if (!new_factors_.empty() || !new_values_.empty())
      return SourceBootstrapResult::Failed;
    auto staged = std::make_shared<GraphMotionState>(state);
    auto phase = std::make_shared<SourceBootstrapState>();
    auto seed = std::make_shared<HeadingSeedState>();
    phase->initial_end_ns = state.consumed_ns;
    auto& request = phase->request;
    request.params = params_;
    request.epoch = epoch;
    request.source_s_ns = source_ns;
    request.frontier_ns = state.consumed_ns;
    request.schedule_ns = request.initial_schedule_ns = schedule_ns;
    request.position = *position;
    request.heading = *heading;
    request.next_acquisition = state.next_acquisition;
    request.public_policy_baseline = {slip_window_,
                                      residual_ema_,
                                      stats_hand_push_,
                                      stats_slip_veto_,
                                      wheel_stationary_now_,
                                      last_wheel_sigma_x_eff_};
    // Capture whole physical records before preparing/clipping the live S tail.
    const int64_t first = std::min(source_ns, heading->source_ns);
    for (const auto& step : state.pending_steps)
      if (step.end_ns > first && step.start_ns < state.consumed_ns)
        request.original_steps.push_back(step);
    Accumulator accumulator;
    PrepareSourceInitializationLocked(*staged, accumulator, schedule_ns, source_ns);
    if (request.frontier_ns > source_ns)
      request.suffix_policy_input =
          ProcessWindowPolicyInput{accumulator.dx,
                                   accumulator.dy,
                                   accumulator.wheel_travel_m,
                                   accumulator.dtheta_wheel,
                                   accumulator.dtheta_gyro,
                                   accumulator.dt_total,
                                   accumulator.max_abs_gyro_rad_per_s,
                                   static_cast<double>(schedule_ns) * 1e-9,
                                   static_cast<double>(schedule_ns) * 1e-9};
    auto candidate = BuildSourceBootstrapCandidate(request);
    for (const auto& binding : candidate.raw_bindings)
    {
      if (!binding.yaw && binding.id == gnss_id)
        seed->source_position = binding.factor;
      if (binding.yaw && binding.id == yaw_id)
      {
        seed->prior = binding.factor;
        phase->live_birth_heading = binding;
      }
    }
    if (!seed->source_position || !seed->prior)
      return SourceBootstrapResult::Failed;
    request.preserved_primary_gnss = seed->source_position;
    request.prefix_left_key = candidate.prefix_key;
    phase->family = std::move(candidate.family);
    phase->auxiliary_poses = std::move(candidate.auxiliary_poses);
    phase->required_source_factors = candidate.isam->getFactorsUnsafe();
    staged->bootstrap = std::move(phase);
    staged->next_acquisition = candidate.next_acquisition;
    std::erase_if(staged->observations,
                  [&](const auto& observation)
                  {
                    return (observation.yaw && observation.id == yaw_id) ||
                           (!observation.yaw && observation.id == gnss_id);
                  });
    ++staged->gnss_factors;
    ++staged->yaw_factors;
    seed->heading_provenance = HeadingSeedState::HeadingProvenance::CoupledSourceObservations;
    seed->source_gnss_id = gnss_id;
    seed->source_heading_id = yaw_id;
    seed->source_heading_ns = heading->source_ns;
    seed->pose = candidate.source_s_pose;
    seed->epoch = epoch;
    seed->source_ns = source_ns;
    seed->source_bound = true;
    TickOutput snapshot;
    snapshot.pose = candidate.source_s_pose;
    snapshot.covariance = candidate.source_s_covariance;
    snapshot.node_index = 0;
    snapshot.timestamp = static_cast<double>(schedule_ns) * 1e-9;
    snapshot.source_stamp_ns = source_ns;
    snapshot.source_epoch = epoch;
    snapshot.revision = ++staged->revision;
    std::deque<std::pair<double, uint64_t>> times;
    times.emplace_back(static_cast<double>(source_ns) * 1e-9, 0);

    isam_.swap(candidate.isam);
    current_estimate_.swap(candidate.values);
    motion_.swap(staged);
    heading_seed_.swap(seed);
    node_time_index_.swap(times);
    accum_ = accumulator;
    latest_ = snapshot;
    estimate_dirty_ = false;
    next_index_ = 1;
    last_node_time_s_ = snapshot.timestamp;
    ticks_since_cov_ = 0;
    // Do not feed privately optimized b1(U) into the still-open S..V window.
    // Normal incoming IMU callbacks retain their existing EMA policy.
    initialized_ = true;
    return SourceBootstrapResult::Applied;
  }
  catch (const std::exception&)
  {
    return SourceBootstrapResult::Failed;
  }
}
}  // namespace fusion_graph
