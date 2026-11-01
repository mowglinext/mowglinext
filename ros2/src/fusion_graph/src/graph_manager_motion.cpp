// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include "graph_heading_seed.hpp"
#include "graph_motion_state.hpp"
#include "source_bootstrap_state.hpp"
#include <gtsam/linear/NoiseModel.h>

namespace fusion_graph
{
namespace
{
int64_t Nanoseconds(double seconds)
{
  return static_cast<int64_t>(std::llround(seconds * 1e9));
}
}  // namespace

void GraphManager::SetConsumedMotionBaseline(uint64_t epoch, int64_t source_ns)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!motion_ && initialized_)
  {
    // Pose-only checkpoints contain no live acquisition epoch. Keep the file
    // intact, but require a new absolute seed rather than label an old pose
    // with the first current IMU timestamp.
    ResetLocked();
  }
  if (!motion_)
  {
    motion_ = std::make_shared<GraphMotionState>();
    motion_->epoch = epoch;
    motion_->validation.Reset(epoch);
  }
  if (motion_->epoch != epoch || motion_->baseline_set)
    return;
  motion_->baseline_set = true;
  motion_->baseline_ns = motion_->consumed_ns = source_ns;
}

void GraphManager::ResetConsumedMotionEpoch(uint64_t epoch)
{
  std::lock_guard<std::mutex> lock(mu_);
  // A missing integration interval cannot inherit an old absolute state with
  // a new timestamp. Require an absolute seed before publishing this epoch.
  ResetLocked();
  motion_ = std::make_shared<GraphMotionState>();
  motion_->epoch = epoch;
  motion_->validation.Reset(epoch);
}

bool GraphManager::AddConsumedMotionStep(const ConsumedMotionStep& step)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!motion_ || !motion_->baseline_set || step.epoch != motion_->epoch ||
      step.start_ns != motion_->consumed_ns)
    return false;
  if (params_.use_imu_preint &&
      ((motion_->last_committed_gyro && *motion_->last_committed_gyro == step.gyro.id) ||
       motion_->pending_gyro_ids.count(step.gyro.id)))
    // Production consumes each accepted IMU interval once, as a whole.
    // Repeated sample splitting needs a different endpoint preintegrator;
    // never claim its independent-call variance implements shared noise.
    return false;
  const auto status = motion_->validation.Append(step);
  if (status != MotionHistoryStatus::Ok)
    return status == MotionHistoryStatus::Duplicate;
  motion_->pending_steps.push_back(step);
  if (motion_->bootstrap && motion_->bootstrap->provisional)
    AccumulateSourceStepLocked(accum_, step);
  motion_->pending_gyro_ids.insert(step.gyro.id);
  motion_->consumed_ns = step.end_ns;
  return true;
}

bool GraphManager::InitializeSource(const gtsam::Pose2& pose,
                                    int64_t schedule_ns,
                                    int64_t source_ns,
                                    uint64_t epoch,
                                    std::optional<double> sigma_xy)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (initialized_ || !motion_ || motion_->epoch != epoch || !motion_->baseline_set ||
      source_ns < motion_->baseline_ns || source_ns > motion_->consumed_ns)
    return false;
  InitializeLocked(pose, static_cast<double>(schedule_ns) * 1e-9, sigma_xy);
  if (!initialized_)
    return false;
  auto& state = *motion_;
  // Preserve the legacy explicit-pose source/tail contract. The raw-observation
  // initializer stages this same bookkeeping before its atomic publication.
  PrepareSourceInitializationLocked(state, accum_, schedule_ns, source_ns);
  latest_->source_stamp_ns = source_ns;
  latest_->source_epoch = epoch;
  latest_->revision = ++state.revision;
  heading_seed_->epoch = epoch;
  heading_seed_->source_ns = source_ns;
  heading_seed_->source_bound = true;
  node_time_index_.clear();
  node_time_index_.emplace_back(static_cast<double>(source_ns) * 1e-9, latest_->node_index);
  return true;
}

std::optional<TickOutput> GraphManager::TickSource(int64_t schedule_ns,
                                                   int64_t source_ns,
                                                   uint64_t epoch)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!initialized_ || !motion_ || motion_->epoch != epoch || source_ns != motion_->consumed_ns ||
      !latest_ || latest_->source_epoch != epoch)
    return std::nullopt;
  try
  {
    auto& state = *motion_;
    if (schedule_ns < state.schedule_ns)
      return std::nullopt;  // The caller must explicitly begin the new clock epoch.
    const int64_t elapsed = schedule_ns - state.schedule_ns;
    const bool cadence = elapsed >= Nanoseconds(params_.node_period_s);
    const double xy_sq = accum_.dx * accum_.dx + accum_.dy * accum_.dy;
    const double yaw =
        std::abs(std::abs(accum_.dtheta_gyro) > 1e-9 ? accum_.dtheta_gyro : accum_.dtheta_wheel);
    const bool stationary =
        xy_sq < params_.stationary_motion_thresh_m * params_.stationary_motion_thresh_m &&
        yaw < params_.stationary_motion_thresh_theta;
    const bool stationary_due =
        !stationary || elapsed >= Nanoseconds(params_.stationary_node_period_s);
    std::optional<TickOutput> result;
    if (source_ns > state.committed_ns && cadence && stationary_due)
    {
      if (state.bootstrap && state.bootstrap->provisional)
        return PromoteSourceBootstrapLocked(schedule_ns);
      state.active_end = source_ns;
      state.active_schedule_ns = schedule_ns;
      result = CreateNodeLocked(static_cast<double>(schedule_ns) * 1e-9);
      if (!initialized_ || !motion_)
        return std::nullopt;
      motion_->active_end.reset();
    }
    const auto revision = latest_->revision;
    if (!AssimilateSourceObservationsLocked())
      return std::nullopt;
    if (result || latest_->revision != revision)
      return latest_;
    return std::nullopt;
  }
  catch (const std::exception& e)
  {
    std::fprintf(stderr,
                 "[fusion_graph] source-time update failed (%s), requiring re-seed.\n",
                 e.what());
    ResetLocked();
    return std::nullopt;
  }
}

gtsam::Pose2 GraphManager::SourceBetweenLocked(const gtsam::Pose2& fallback,
                                               bool translation,
                                               bool rotation)
{
  if (!motion_ || !motion_->active_end)
    return fallback;
  auto& state = *motion_;
  auto history = std::make_shared<MotionSourceHistory>(state.pending_steps.size());
  history->Reset(state.epoch);
  for (auto step : state.pending_steps)
  {
    step.wheel_translation_enabled = translation;
    step.gyro_rotation_enabled = rotation;
    if (history->Append(step) != MotionHistoryStatus::Ok)
      throw std::runtime_error("incoherent consumed edge");
  }
  const auto trajectory = history->Query(state.epoch, state.committed_ns, *state.active_end);
  if (trajectory.status != MotionHistoryStatus::Ok)
    throw std::runtime_error("uncovered consumed edge");
  state.active_history = std::move(history);
  return trajectory.mean;
}

void GraphManager::FreezeSourceEdgeLocked(gtsam::Key left,
                                          gtsam::Key right,
                                          const gtsam::Pose2& previous,
                                          const gtsam::Pose2& predicted,
                                          const gtsam::Vector3& sigmas)
{
  if (!motion_ || !motion_->active_end)
    return;
  auto& state = *motion_;
  FrozenProcessEdge edge;
  edge.left_key = left;
  edge.right_key = right;
  edge.source_epoch = state.epoch;
  edge.start_ns = state.committed_ns;
  edge.end_ns = *state.active_end;
  edge.endpoint_covariance = sigmas.array().square().matrix().asDiagonal();
  edge.reference_values.insert(left, previous);
  edge.reference_values.insert(right, predicted);
  edge.trajectory_query =
      FrozenProcessEdge::HistoryQuery(state.active_history, state.epoch, edge.start_ns);
  if (params_.use_imu_preint)
  {
    edge.bias_key = gtsam::Symbol('b', gtsam::Symbol(right).index());
    // Recorded prediction reference, never the subsequently optimized bias.
    const double bias = state.pending_steps.front().bias_linearization;
    for (const auto& step : state.pending_steps)
    {
      if (step.bias_linearization != bias)
        throw std::runtime_error("bias reference changed inside source edge");
      edge.gyro_rate_variances.emplace(step.gyro.id,
                                       params_.gyro_noise_density_rad_per_s *
                                           params_.gyro_noise_density_rad_per_s);
    }
    edge.reference_values.insert(*edge.bias_key, bias);
  }
  const auto mode = state.relative_conditionals ? ConditionalProcessBridge::Mode::RelativeErrors
                                                : ConditionalProcessBridge::Mode::FrozenAffine;
  state.edges.push_back({gtsam::Symbol(right).index(),
                         std::make_shared<ConditionalProcessBridge>(std::move(edge), mode),
                         {},
                         {}});
}

void GraphManager::CommitSourceNodeLocked(TickOutput& output)
{
  if (!motion_ || !motion_->active_end)
    return;
  auto& state = *motion_;
  state.committed_ns = *state.active_end;
  state.schedule_ns = state.active_schedule_ns;
  output.source_stamp_ns = state.committed_ns;
  output.source_epoch = state.epoch;
  state.revision = std::max(state.revision, latest_ ? latest_->revision : 0);
  output.revision = ++state.revision;
  state.last_committed_gyro = state.pending_steps.back().gyro.id;
  state.pending_steps.clear();
  state.pending_gyro_ids.clear();
  state.validation.PruneBefore(state.epoch, state.committed_ns);
  state.active_history.reset();
}

bool GraphManager::QueueGnssAtEpoch(double x,
                                    double y,
                                    double sigma,
                                    bool robust,
                                    int64_t source_ns,
                                    uint64_t observation_id,
                                    uint64_t epoch)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!motion_ || motion_->epoch != epoch || !motion_->baseline_set || !std::isfinite(x) ||
      !std::isfinite(y) || !std::isfinite(sigma))
    return false;
  auto& state = *motion_;
  if (state.observation_ids.count(observation_id))
    return false;
  const int64_t oldest = OldestSourceBoundaryLocked();
  if (source_ns < oldest || state.observations.size() >= 10000)
    return false;
  state.observations.push_back({{x, y},
                                std::max(sigma, params_.gps_sigma_floor),
                                robust,
                                source_ns,
                                observation_id,
                                std::nullopt});
  state.observation_ids.emplace(observation_id, source_ns);
  return true;
}

bool GraphManager::QueueYawAtEpoch(double yaw,
                                   double sigma,
                                   bool robust,
                                   int64_t source_ns,
                                   uint64_t observation_id,
                                   uint64_t epoch)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!motion_ || motion_->epoch != epoch || !motion_->baseline_set || !std::isfinite(yaw) ||
      !std::isfinite(sigma))
    return false;
  auto& state = *motion_;
  if (state.yaw_observation_ids.count(observation_id))
    return false;
  const int64_t oldest = OldestSourceBoundaryLocked();
  if (source_ns < oldest || state.observations.size() >= 10000)
    return false;
  // Match QueueYaw's legacy fallback; callers retain their producer-specific
  // COG floor and magnetometer covariance before reaching this API.
  if (sigma <= 0.0)
    sigma = 0.05;
  state.observations.push_back(
      {gtsam::Vector2::Zero(), sigma, robust, source_ns, observation_id, yaw});
  state.yaw_observation_ids.emplace(observation_id, source_ns);
  return true;
}

bool GraphManager::AssimilateSourceObservationsLocked()
{
  if (!motion_ || !latest_)
    return true;
  auto& state = *motion_;
  bool updated = false;
  if (!AssimilateSourceBootstrapObservationsLocked(updated))
    return false;
  for (auto it = state.observations.begin(); it != state.observations.end();)
  {
    if (it->source_ns > state.committed_ns)
    {
      ++it;
      continue;
    }
    // Supported-family observations left pending have an unsupported local
    // chart. Do not retire or retarget them through the ordinary edge path.
    if (SourceBootstrapContainsLocked(it->source_ns))
    {
      ++it;
      continue;
    }
    gtsam::Key key = 0;
    auto edge = std::find_if(state.edges.begin(),
                             state.edges.end(),
                             [&](const auto& entry)
                             {
                               return it->source_ns >= entry.bridge->edge().start_ns &&
                                      it->source_ns <= entry.bridge->edge().end_ns;
                             });
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    if (edge != state.edges.end())
    {
      const auto current = isam_->calculateEstimate();
      ConditionalProcessBridge::RelativeInsertion acquisition;
      const auto new_key = gtsam::Symbol('a', state.next_acquisition);
      if (edge->bridge->mode() == ConditionalProcessBridge::Mode::RelativeErrors)
      {
        try
        {
          acquisition = edge->bridge->InsertRelative(it->source_ns, new_key, &current);
        }
        catch (const UnsupportedRelativeConditional&)
        {
          // Keep this admitted observation pending. Another supported update
          // can change its conditioning state; never relabel it or reset the
          // graph merely because its current local chart cannot represent it.
          ++it;
          continue;
        }
      }
      else
      {
        const auto frozen = edge->bridge->Insert(it->source_ns, new_key, &current);
        acquisition = {frozen.key, frozen.initial_value, frozen.factor};
      }
      ++state.next_acquisition;
      key = acquisition.key;
      if (acquisition.initial_value)
      {
        values.insert(key, *acquisition.initial_value);
        factors.push_back(acquisition.factor);
        edge->acquisitions.emplace(key, it->source_ns);
        edge->conditionals.push_back({key, it->source_ns, acquisition.factor});
      }
    }
    else if (it->source_ns == state.committed_ns)
      key = PoseKey(latest_->node_index);
    else
    {
      // Retired or pre-seed observations are never retargeted to the head.
      it = state.observations.erase(it);
      continue;
    }
    gtsam::SharedNoiseModel noise =
        it->yaw ? MakeDiagonal({it->sigma}) : MakeDiagonal({it->sigma, it->sigma});
    if (it->robust)
      noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(
                                                    it->yaw ? params_.huber_k_yaw
                                                            : params_.huber_k_gps),
                                                noise);
    if (it->yaw)
      factors.add(YawUnaryFactor(key, *it->yaw, noise));
    else
      factors.add(GnssLeverArmFactor(
          key, it->xy, gtsam::Vector2(params_.lever_arm_x, params_.lever_arm_y), noise));
    if (!ApplyIsamUpdateLocked(factors, values))
      return false;
    if (it->yaw)
      ++state.yaw_factors;
    else
      ++state.gnss_factors;
    updated = true;
    it = state.observations.erase(it);
  }
  if (updated)
  {
    estimate_dirty_ = true;
    latest_->pose = PoseAt(latest_->node_index);
    latest_->covariance = isam_->marginalCovariance(PoseKey(latest_->node_index));
    state.revision = std::max(state.revision, latest_->revision);
    latest_->revision = ++state.revision;
  }
  return true;
}

bool GraphManager::NeedsSourceRebase() const
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!motion_ || (motion_->bootstrap && motion_->bootstrap->provisional) ||
      params_.max_graph_nodes == 0)
    return false;
  RefreshEstimateLocked();
  size_t count = 0;
  for (const auto& value : current_estimate_)
  {
    const char kind = gtsam::Symbol(value.key).chr();
    if (kind == 'x' || kind == 'a')
      ++count;
  }
  return count > params_.max_graph_nodes;
}

uint64_t GraphManager::SourceGnssFactorCount() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return motion_ ? motion_->gnss_factors : 0;
}

uint64_t GraphManager::SourceYawFactorCount() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return motion_ ? motion_->yaw_factors : 0;
}

std::optional<gtsam::Pose2> GraphManager::EstimatePoseAtSource(uint64_t epoch,
                                                               int64_t source_ns) const
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!initialized_ || !motion_ || motion_->epoch != epoch || !latest_ ||
      latest_->source_epoch != epoch || source_ns > motion_->committed_ns)
    return std::nullopt;
  try
  {
    if (source_ns == motion_->committed_ns)
      return isam_->calculateEstimate<gtsam::Pose2>(PoseKey(latest_->node_index));
    if (SourceBootstrapContainsLocked(source_ns))
      return motion_->bootstrap->family->ConditionalMean(source_ns, isam_->calculateEstimate());
    for (const auto& entry : motion_->edges)
    {
      const auto& edge = entry.bridge->edge();
      if (source_ns < edge.start_ns || source_ns > edge.end_ns)
        continue;
      if (!isam_->valueExists(edge.left_key) || !isam_->valueExists(edge.right_key))
        return std::nullopt;
      if (source_ns == edge.start_ns)
        return isam_->calculateEstimate<gtsam::Pose2>(edge.left_key);
      if (source_ns == edge.end_ns)
        return isam_->calculateEstimate<gtsam::Pose2>(edge.right_key);
      for (const auto& [key, time] : entry.acquisitions)
        if (time == source_ns)
          return isam_->calculateEstimate<gtsam::Pose2>(key);

      // Conditional insertion on a COPY computes the same initialization
      // without introducing a live acquisition or consuming its identity.
      // Its references/history are immutable and current values include the
      // optimized endpoints, previous acquisitions and optional bias state.
      auto conditional = *entry.bridge;
      const auto current = isam_->calculateEstimate();
      if (conditional.mode() == ConditionalProcessBridge::Mode::RelativeErrors)
        return conditional.RelativeInitialValue(source_ns, current);
      const auto query =
          conditional.Insert(source_ns, gtsam::Symbol('a', motion_->next_acquisition), &current);
      return query.initial_value;
    }
  }
  catch (const std::exception&)
  {
    // Unsupported/missing source state is not permission to compare an old
    // observation against the head or to mutate the live graph for a query.
  }
  return std::nullopt;
}

std::optional<int64_t> GraphManager::OldestRetainedSourceNs(uint64_t epoch) const
{
  std::lock_guard<std::mutex> lock(mu_);
  if (!motion_ || motion_->epoch != epoch || !motion_->baseline_set)
    return std::nullopt;
  return OldestSourceBoundaryLocked();
}

std::map<gtsam::Key, uint64_t> GraphManager::AcquisitionOwnersLocked() const
{
  std::map<gtsam::Key, uint64_t> result;
  if (motion_)
  {
    for (const auto& edge : motion_->edges)
      for (const auto& [key, time] : edge.acquisitions)
        result.emplace(key, edge.owner);
    if (motion_->bootstrap)
      for (const auto& [key, time] : motion_->bootstrap->auxiliary_poses)
        result.emplace(key, 1);
  }
  return result;
}

void GraphManager::PruneSourceStateLocked()
{
  if (!motion_)
    return;
  auto& state = *motion_;
  while (!state.edges.empty() && !isam_->valueExists(state.edges.front().bridge->edge().left_key))
    state.edges.pop_front();
  if (state.bootstrap)
  {
    bool retained = true;
    for (const auto& window : state.bootstrap->family->windows())
      retained = retained && isam_->valueExists(window.edge.left_key) &&
                 isam_->valueExists(window.edge.right_key) &&
                 (!window.edge.bias_key || isam_->valueExists(*window.edge.bias_key));
    for (const auto& [key, time] : state.bootstrap->auxiliary_poses)
      retained = retained && isam_->valueExists(key);
    if (!retained)
      state.bootstrap.reset();
  }
  const int64_t oldest = OldestSourceBoundaryLocked();
  std::erase_if(state.observation_ids,
                [&](const auto& id)
                {
                  return id.second < oldest;
                });
  std::erase_if(state.yaw_observation_ids,
                [&](const auto& id)
                {
                  return id.second < oldest;
                });
  std::erase_if(state.observations,
                [&](const auto& observation)
                {
                  return observation.source_ns < oldest;
                });
}

void GraphManager::InvalidateSourceBridgesLocked()
{
  heading_seed_.reset();
  if (!motion_)
    return;
  motion_->edges.clear();
  motion_->bootstrap.reset();
  // The rebuilt trajectory has no historical bridge references, but future
  // and boundary observations still describe the same physical map/source
  // epoch. Keep their admission and dedup identity through a live reanchor.
  const int64_t boundary = motion_->committed_ns;
  std::erase_if(motion_->observations,
                [boundary](const auto& observation)
                {
                  return observation.source_ns < boundary;
                });
  std::erase_if(motion_->observation_ids,
                [boundary](const auto& observation)
                {
                  return observation.second < boundary;
                });
  std::erase_if(motion_->yaw_observation_ids,
                [boundary](const auto& observation)
                {
                  return observation.second < boundary;
                });
  if (latest_)
  {
    motion_->revision = std::max(motion_->revision, latest_->revision);
    latest_->revision = ++motion_->revision;
  }
}
}  // namespace fusion_graph
