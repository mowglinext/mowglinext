// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include "graph_heading_seed.hpp"
#include "graph_motion_state.hpp"
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PriorFactor.h>

namespace fusion_graph
{
// Shared with the legacy explicit-pose initialization. Stage on a copied motion
// state for observation initialization; all allocation precedes publication.
void GraphManager::PrepareSourceInitializationLocked(GraphMotionState& state,
                                                     Accumulator& accumulator,
                                                     int64_t schedule_ns,
                                                     int64_t source_ns) const
{
  state.committed_ns = source_ns;
  state.schedule_ns = schedule_ns;
  accumulator.Reset();
  while (!state.pending_steps.empty() && state.pending_steps.front().end_ns <= source_ns)
    state.pending_steps.pop_front();
  if (!state.pending_steps.empty())
    state.pending_steps.front().start_ns = source_ns;
  state.pending_gyro_ids.clear();
  for (const auto& step : state.pending_steps)
  {
    state.pending_gyro_ids.insert(step.gyro.id);
    AccumulateSourceStepLocked(accumulator, step);
  }
  state.validation.PruneBefore(state.epoch, source_ns);
}

void GraphManager::AccumulateSourceStepLocked(Accumulator& accumulator,
                                              const ConsumedMotionStep& step) const
{
  const double dt = static_cast<double>(step.end_ns - step.start_ns) * 1e-9;
  accumulator.dx += step.wheel.vx * dt;
  accumulator.dy += step.wheel.vy * dt;
  accumulator.wheel_travel_m += std::hypot(step.wheel.vx, step.wheel.vy) * dt;
  accumulator.dtheta_wheel += step.wheel.wz * dt;
  accumulator.dt_total += dt;
  accumulator.dtheta_gyro += (step.gyro.raw_z - step.bias_linearization) * dt;
  accumulator.max_abs_gyro_rad_per_s =
      std::max(accumulator.max_abs_gyro_rad_per_s, std::abs(step.gyro.raw_z));
  if (params_.use_imu_preint)
  {
    accumulator.gyro_preint_dtheta += step.gyro.raw_z * dt;
    accumulator.gyro_preint_dt += dt;
    accumulator.gyro_preint_variance +=
        dt * dt * params_.gyro_noise_density_rad_per_s * params_.gyro_noise_density_rad_per_s;
  }
}

SourceBootstrapResult GraphManager::InitializeSourceObservations(
    int64_t schedule_ns, int64_t source_ns, uint64_t epoch, uint64_t gnss_id, uint64_t yaw_id)
{
  std::lock_guard<std::mutex> lock(mu_);
  return InitializeSourceObservationsLocked(schedule_ns, source_ns, epoch, gnss_id, yaw_id);
}

SourceBootstrapResult GraphManager::InitializeSourcePositionWithRetimedHeading(
    int64_t schedule_ns, int64_t source_ns, uint64_t epoch, uint64_t gnss_id, double yaw)
{
  std::lock_guard<std::mutex> lock(mu_);
  return InitializeSourceObservationsLocked(
      schedule_ns, source_ns, epoch, gnss_id, std::nullopt, yaw);
}

SourceBootstrapResult GraphManager::InitializeSourceObservationsLocked(
    int64_t schedule_ns,
    int64_t source_ns,
    uint64_t epoch,
    uint64_t gnss_id,
    std::optional<uint64_t> yaw_id,
    double retimed_yaw)
{
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
                                      return yaw_id && observation.yaw &&
                                             observation.id == *yaw_id &&
                                             observation.source_ns == source_ns;
                                    });
  const auto admitted_position = state.observation_ids.find(gnss_id);
  if (position == state.observations.end() || admitted_position == state.observation_ids.end() ||
      admitted_position->second != source_ns)
    return SourceBootstrapResult::PendingProofUnavailable;
  if (yaw_id)
  {
    const auto admitted_heading = state.yaw_observation_ids.find(*yaw_id);
    if (heading == state.observations.end() ||
        admitted_heading == state.yaw_observation_ids.end() ||
        admitted_heading->second != source_ns)
      return SourceBootstrapResult::PendingProofUnavailable;
  }
  const double yaw = yaw_id ? *heading->yaw : retimed_yaw;
  const double yaw_sigma = yaw_id ? heading->sigma : params_.prior_sigma_theta;

  try
  {
    if (!new_factors_.empty() || !new_values_.empty())
      return SourceBootstrapResult::Failed;
    if (!position->xy.allFinite() || !std::isfinite(position->sigma) || position->sigma <= 0.0 ||
        !std::isfinite(yaw) || !std::isfinite(yaw_sigma) || yaw_sigma <= 0.0 ||
        !std::isfinite(params_.lever_arm_x) || !std::isfinite(params_.lever_arm_y))
      return SourceBootstrapResult::Failed;
    auto candidate_state = std::make_shared<GraphMotionState>(state);
    auto candidate_seed = std::make_shared<HeadingSeedState>();
    const gtsam::Vector2 lever(params_.lever_arm_x, params_.lever_arm_y);
    const gtsam::Rot2 rotation(yaw);
    const gtsam::Vector2 base = position->xy - rotation.rotate(lever);
    const gtsam::Pose2 initial_pose(base.x(), base.y(), yaw);
    gtsam::Values values;
    values.insert(PoseKey(0), initial_pose);
    auto position_noise = MakeDiagonal({position->sigma, position->sigma});
    auto heading_noise = MakeDiagonal({yaw_sigma});
    if (position->robust)
      position_noise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(params_.huber_k_gps), position_noise);
    if (yaw_id && heading->robust)
      heading_noise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(params_.huber_k_yaw), heading_noise);
    candidate_seed->source_position =
        std::make_shared<GnssLeverArmFactor>(PoseKey(0), position->xy, lever, position_noise);
    candidate_seed->prior = std::make_shared<YawUnaryFactor>(PoseKey(0), yaw, heading_noise);
    candidate_seed->heading_provenance =
        yaw_id ? HeadingSeedState::HeadingProvenance::RawSourceObservation
               : HeadingSeedState::HeadingProvenance::RetimedPrior;
    candidate_seed->source_gnss_id = gnss_id;
    candidate_seed->source_heading_id = yaw_id;
    if (yaw_id)
      candidate_seed->source_heading_ns = source_ns;
    candidate_seed->pose = initial_pose;
    candidate_seed->epoch = epoch;
    candidate_seed->source_ns = source_ns;
    candidate_seed->source_bound = true;
    gtsam::NonlinearFactorGraph factors;
    factors.push_back(candidate_seed->source_position);
    factors.push_back(candidate_seed->prior);
    if (params_.use_imu_preint)
    {
      values.insert(gtsam::Symbol('b', 0), 0.0);
      factors.emplace_shared<gtsam::PriorFactor<double>>(
          gtsam::Symbol('b', 0), 0.0, MakeDiagonal({params_.gyro_bias_prior_sigma_rad_per_s}));
    }
    gtsam::ISAM2Params parameters;
    parameters.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
    parameters.relinearizeThreshold = 0.05;
    parameters.relinearizeSkip = std::max(1, params_.isam2_relinearize_skip);
    auto fresh = std::make_unique<gtsam::ISAM2>(parameters);
    fresh->update(factors, values);
    auto estimate = fresh->calculateEstimate();
    TickOutput snapshot;
    snapshot.pose = fresh->calculateEstimate<gtsam::Pose2>(PoseKey(0));
    snapshot.covariance = fresh->marginalCovariance(PoseKey(0));
    snapshot.node_index = 0;
    snapshot.timestamp = static_cast<double>(schedule_ns) * 1e-9;
    snapshot.source_stamp_ns = source_ns;
    snapshot.source_epoch = epoch;
    if (!snapshot.pose.matrix().allFinite() || !snapshot.covariance.allFinite())
      return SourceBootstrapResult::Failed;
    Accumulator accumulator;
    PrepareSourceInitializationLocked(*candidate_state, accumulator, schedule_ns, source_ns);
    std::erase_if(candidate_state->observations,
                  [&](const auto& observation)
                  {
                    return observation.source_ns == source_ns &&
                           ((!observation.yaw && observation.id == gnss_id) ||
                            (yaw_id && observation.yaw && observation.id == *yaw_id));
                  });
    ++candidate_state->gnss_factors;
    if (yaw_id)
      ++candidate_state->yaw_factors;
    snapshot.revision = ++candidate_state->revision;
    std::deque<std::pair<double, uint64_t>> times;
    times.emplace_back(static_cast<double>(source_ns) * 1e-9, 0);

    isam_.swap(fresh);
    current_estimate_.swap(estimate);
    motion_.swap(candidate_state);
    heading_seed_.swap(candidate_seed);
    node_time_index_.swap(times);
    accum_ = accumulator;
    latest_ = snapshot;
    estimate_dirty_ = false;
    next_index_ = 1;
    last_node_time_s_ = snapshot.timestamp;
    ticks_since_cov_ = 0;
    current_bias_estimate_ = 0.0;
    initialized_ = true;
    return SourceBootstrapResult::Applied;
  }
  catch (const std::exception&)
  {
    // Once proof exists, failure is not permission to use the same data again
    // in a fabricated independent pose prior. Leave admitted observations pending.
    return SourceBootstrapResult::Failed;
  }
}
}  // namespace fusion_graph
