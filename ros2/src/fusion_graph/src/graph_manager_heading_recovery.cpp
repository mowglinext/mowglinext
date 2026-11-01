// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include "graph_heading_seed.hpp"
#include "graph_motion_state.hpp"
#include "source_bootstrap_state.hpp"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>

namespace fusion_graph
{
namespace
{
// Default pinned Pose2 coordinates separate Cartesian XY and yaw. Refuse a
// different linked chart rather than silently replacing a coupled Log prior.
bool SeedPositionCostIsCartesian(const HeadingSeedState& seed)
{
  for (const auto& delta : {gtsam::Vector3(0.3, -0.2, 1.1), gtsam::Vector3(-0.4, 0.1, -2.2)})
  {
    const gtsam::Pose2 pose(seed.pose.x() + delta.x(),
                            seed.pose.y() + delta.y(),
                            seed.pose.theta() + delta.z());
    gtsam::Values values;
    values.insert(PoseKey(0), pose);
    const auto* prior = dynamic_cast<const gtsam::NoiseModelFactor*>(seed.prior.get());
    if (!prior)
      return false;
    const auto error = prior->unwhitenedError(values);
    if (error.size() != 3 ||
        std::abs(error.head<2>().squaredNorm() - delta.head<2>().squaredNorm()) > 1e-12)
      return false;
  }
  return true;
}
}  // namespace

HeadingSeedToken GraphManager::OriginalHeadingSeed() const
{
  std::lock_guard<std::mutex> lock(mu_);
  return initialized_ && heading_seed_ && heading_seed_->source_bound ? heading_seed_ : nullptr;
}

HeadingRecoveryResult GraphManager::RecoverInvalidSeedHeading(
    const HeadingSeedToken& seed,
    bool seed_heading_invalid,
    const RecoveryYawObservation& observation)
{
  // Deliberately synchronous: every input mutator uses this mutex. Candidate
  // construction never touches live solver/bridges and cannot race admission
  // or discard an input arriving during construction. This rare explicit
  // operation does not require a second, incomplete mutation-generation scheme.
  std::lock_guard<std::mutex> lock(mu_);
  if (!seed_heading_invalid)
    return HeadingRecoveryResult::SeedNotInvalid;
  if (!std::isfinite(observation.yaw) || !std::isfinite(observation.sigma) ||
      observation.sigma <= 0.0)
    return HeadingRecoveryResult::InvalidObservation;
  if (!seed || seed != heading_seed_ || !seed->source_bound || !initialized_ || !motion_ ||
      !latest_)
    return HeadingRecoveryResult::SeedUnavailable;
  if (seed->heading_provenance == HeadingSeedState::HeadingProvenance::RetimedPrior)
    return HeadingRecoveryResult::UnsupportedFamily;
  const bool coupled =
      seed->heading_provenance == HeadingSeedState::HeadingProvenance::CoupledSourceObservations;
  const auto& state = *motion_;
  if (state.epoch != observation.epoch || seed->epoch != state.epoch ||
      latest_->source_epoch != state.epoch || observation.source_ns < seed->source_ns ||
      observation.source_ns > state.committed_ns || state.consumed_ns != state.committed_ns ||
      !state.pending_steps.empty() || !new_factors_.empty() || !new_values_.empty())
    return HeadingRecoveryResult::UnsupportedSource;
  if (!isam_->valueExists(PoseKey(0)))
    return HeadingRecoveryResult::UnsupportedPrefix;

  try
  {
    const auto original = isam_->getFactorsUnsafe();
    auto retired_heading = seed->prior;
    if (coupled)
    {
      if (!state.bootstrap || state.bootstrap->provisional ||
          !state.bootstrap->live_birth_heading || !seed->source_heading_id ||
          !seed->source_heading_ns)
        return HeadingRecoveryResult::UnsupportedFamily;
      const auto& phase = *state.bootstrap;
      const auto& birth = *phase.live_birth_heading;
      if (!birth.yaw || birth.id != *seed->source_heading_id ||
          birth.source_ns != *seed->source_heading_ns || birth.id != phase.request.heading.id ||
          birth.source_ns == seed->source_ns ||
          birth.source_ns != phase.request.heading.source_ns ||
          phase.request.epoch != state.epoch || phase.family->source_epoch() != state.epoch ||
          phase.request.source_s_ns != seed->source_ns ||
          phase.request.frontier_ns != phase.family->end_ns() ||
          phase.family->end_ns() > state.committed_ns || !birth.factor ||
          !dynamic_cast<const YawUnaryFactor*>(birth.factor.get()) ||
          birth.factor->keys().size() != 1 || birth.factor->keys().front() != birth.key ||
          !isam_->valueExists(birth.key) ||
          seed->source_position != phase.request.preserved_primary_gnss ||
          seed->source_gnss_id != phase.request.position.id ||
          phase.required_source_factors.empty())
        return HeadingRecoveryResult::UnsupportedFamily;
      for (const auto& factor : phase.required_source_factors)
      {
        if (!factor || std::count(original.begin(), original.end(), factor) != 1)
          return HeadingRecoveryResult::UnsupportedFamily;
        for (const auto key : factor->keys())
          if (!isam_->valueExists(key))
            return HeadingRecoveryResult::UnsupportedPrefix;
      }
      if (std::count(phase.required_source_factors.begin(),
                     phase.required_source_factors.end(),
                     birth.factor) != 1)
        return HeadingRecoveryResult::UnsupportedFamily;
      const auto acquired_birth = phase.auxiliary_poses.find(birth.key);
      bool birth_alias = acquired_birth != phase.auxiliary_poses.end() &&
                         acquired_birth->second == birth.source_ns;
      for (const auto& window : phase.family->windows())
      {
        if (!isam_->valueExists(window.edge.left_key) ||
            !isam_->valueExists(window.edge.right_key) ||
            (window.edge.bias_key && !isam_->valueExists(*window.edge.bias_key)))
          return HeadingRecoveryResult::UnsupportedPrefix;
        birth_alias =
            birth_alias ||
            (birth.key == window.edge.left_key && birth.source_ns == window.edge.start_ns) ||
            (birth.key == window.edge.right_key && birth.source_ns == window.edge.end_ns);
      }
      if (!birth_alias)
        return HeadingRecoveryResult::UnsupportedFamily;
      for (const auto& [key, time] : phase.auxiliary_poses)
        if (!isam_->valueExists(key))
          return HeadingRecoveryResult::UnsupportedPrefix;
      for (const auto& acquisition : phase.family->acquisitions())
      {
        const auto found = phase.auxiliary_poses.find(acquisition.key);
        if (found == phase.auxiliary_poses.end() || found->second != acquisition.source_ns)
          return HeadingRecoveryResult::UnsupportedFamily;
      }
      // Public retention may have pruned the pre-S birth ID. Its private
      // physical identity remains consumed and cannot become a new reading.
      if (observation.id == birth.id)
        return HeadingRecoveryResult::DuplicateObservation;
      retired_heading = birth.factor;
    }
    if (std::count(original.begin(), original.end(), retired_heading) != 1)
      return HeadingRecoveryResult::UnsupportedPrefix;
    if (seed->source_position)
    {
      if ((!coupled &&
           seed->heading_provenance != HeadingSeedState::HeadingProvenance::RawSourceObservation) ||
          !seed->source_heading_id ||
          std::count(original.begin(), original.end(), seed->source_position) != 1 ||
          state.observation_ids.count(seed->source_gnss_id) == 0 ||
          (!coupled && state.yaw_observation_ids.count(*seed->source_heading_id) == 0))
        return HeadingRecoveryResult::UnsupportedFamily;
    }
    else if (!SeedPositionCostIsCartesian(*seed) || !std::isfinite(seed->sigma_xy) ||
             seed->sigma_xy <= 0.0)
      return HeadingRecoveryResult::UnsupportedFamily;

    auto candidate_state = std::make_shared<GraphMotionState>(state);
    if (coupled)
    {
      candidate_state->bootstrap = std::make_shared<SourceBootstrapState>(*state.bootstrap);
      candidate_state->bootstrap->family =
          std::make_shared<SharedSourceProcessFamily>(*state.bootstrap->family);
    }
    if (state.yaw_observation_ids.count(observation.id))
    {
      const auto pending = std::find_if(candidate_state->observations.begin(),
                                        candidate_state->observations.end(),
                                        [&](const auto& value)
                                        {
                                          return value.yaw && value.id == observation.id;
                                        });
      if (pending == candidate_state->observations.end())
        return HeadingRecoveryResult::DuplicateObservation;
      if (pending->source_ns != observation.source_ns || *pending->yaw != observation.yaw ||
          pending->sigma != observation.sigma || pending->robust != observation.robust)
        return HeadingRecoveryResult::ObservationConflict;
      candidate_state->observations.erase(pending);
    }
    else
      candidate_state->yaw_observation_ids.emplace(observation.id, observation.source_ns);

    auto values = isam_->calculateEstimate();
    std::map<const gtsam::NonlinearFactor*, gtsam::NonlinearFactor::shared_ptr> replacements;
    for (auto& entry : candidate_state->edges)
    {
      if (entry.bridge->mode() != ConditionalProcessBridge::Mode::FrozenAffine ||
          entry.acquisitions.size() != entry.conditionals.size())
        return HeadingRecoveryResult::UnsupportedFamily;
      ConditionalProcessBridge original_family(entry.bridge->edge());
      auto rebuilt = std::make_shared<ConditionalProcessBridge>(
          entry.bridge->edge(), ConditionalProcessBridge::Mode::RelativeErrors);
      for (auto& registered : entry.conditionals)
      {
        const auto acquired = entry.acquisitions.find(registered.key);
        if (acquired == entry.acquisitions.end() || acquired->second != registered.source_ns ||
            !registered.factor || !values.exists(registered.key))
          return HeadingRecoveryResult::UnsupportedFamily;
        const auto old = original_family.Insert(registered.source_ns, registered.key);
        if (!old.factor || !old.factor->equals(*registered.factor, 1e-12))
          return HeadingRecoveryResult::UnsupportedFamily;
        const auto replacement =
            rebuilt->InsertRelative(registered.source_ns, registered.key, &values);
        if (!replacement.factor || replacement.key != registered.key ||
            !replacements.emplace(registered.factor.get(), replacement.factor).second)
          return HeadingRecoveryResult::UnsupportedFamily;
        registered.factor = replacement.factor;
      }
      entry.bridge = std::move(rebuilt);
    }

    gtsam::NonlinearFactorGraph factors;
    std::set<const gtsam::NonlinearFactor*> replaced;
    for (const auto& factor : original)
    {
      if (!factor || factor == retired_heading)
        continue;
      if (const auto found = replacements.find(factor.get()); found != replacements.end())
      {
        if (!replaced.insert(factor.get()).second)
          return HeadingRecoveryResult::UnsupportedFamily;
        factors.push_back(found->second);
      }
      else
      {
        if (dynamic_cast<const gtsam::LinearContainerFactor*>(factor.get()) ||
            dynamic_cast<const RelativeConditionalFactor*>(factor.get()))
          return HeadingRecoveryResult::UnsupportedPrefix;
        factors.push_back(factor);
      }
    }
    if (replaced.size() != replacements.size())
      return HeadingRecoveryResult::UnsupportedFamily;
    if (!seed->source_position)
      factors.emplace_shared<gtsam::PoseTranslationPrior<gtsam::Pose2>>(
          PoseKey(0), seed->pose.translation(), MakeDiagonal({seed->sigma_xy, seed->sigma_xy}));

    gtsam::Key recovery_key = 0;
    if (observation.source_ns == state.committed_ns)
      recovery_key = PoseKey(latest_->node_index);
    else if (coupled && observation.source_ns >= candidate_state->bootstrap->request.source_s_ns &&
             observation.source_ns <= candidate_state->bootstrap->family->end_ns())
    {
      auto& phase = *candidate_state->bootstrap;
      const auto inserted =
          phase.family->Insert(observation.source_ns,
                               gtsam::Symbol('a', candidate_state->next_acquisition),
                               &values);
      recovery_key = inserted.key;
      if (inserted.initial_value)
      {
        ++candidate_state->next_acquisition;
        values.insert(recovery_key, *inserted.initial_value);
        factors.push_back(inserted.factor);
        phase.required_source_factors.push_back(inserted.factor);
        phase.auxiliary_poses.emplace(recovery_key, observation.source_ns);
      }
    }
    else
    {
      auto edge = std::find_if(candidate_state->edges.begin(),
                               candidate_state->edges.end(),
                               [&](const auto& entry)
                               {
                                 return observation.source_ns >= entry.bridge->edge().start_ns &&
                                        observation.source_ns <= entry.bridge->edge().end_ns;
                               });
      if (edge == candidate_state->edges.end())
        return HeadingRecoveryResult::UnsupportedSource;
      const auto inserted =
          edge->bridge->InsertRelative(observation.source_ns,
                                       gtsam::Symbol('a', candidate_state->next_acquisition++),
                                       &values);
      recovery_key = inserted.key;
      if (inserted.initial_value)
      {
        values.insert(recovery_key, *inserted.initial_value);
        factors.push_back(inserted.factor);
        edge->acquisitions.emplace(recovery_key, observation.source_ns);
        edge->conditionals.push_back({recovery_key, observation.source_ns, inserted.factor});
      }
    }
    auto noise = MakeDiagonal({observation.sigma});
    if (observation.robust)
      noise = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(
                                                    params_.huber_k_yaw),
                                                noise);
    factors.emplace_shared<YawUnaryFactor>(recovery_key, observation.yaw, noise);
    if (coupled)
    {
      auto& phase = *candidate_state->bootstrap;
      gtsam::NonlinearFactorGraph lineage;
      for (const auto& factor : phase.required_source_factors)
        if (factor != retired_heading)
          lineage.push_back(factor);
      lineage.push_back(factors[factors.size() - 1]);
      phase.required_source_factors = std::move(lineage);
      phase.live_birth_heading.reset();
    }

    // Initial guess only: rotate the wrong posterior about its seed position.
    // Fixed map observations, seed XY and all process/bias likelihoods stay fixed.
    const double difference = observation.yaw - values.at<gtsam::Pose2>(recovery_key).theta();
    const double offset = std::atan2(std::sin(difference), std::cos(difference));
    const auto correction = gtsam::Pose2(seed->pose.x(), seed->pose.y(), offset)
                                .compose(gtsam::Pose2(-seed->pose.x(), -seed->pose.y(), 0.0));
    gtsam::Values initial;
    for (const auto& value : values)
    {
      const char kind = gtsam::Symbol(value.key).chr();
      if (kind == 'x' || kind == 'a')
        initial.insert(value.key, correction.compose(value.value.cast<gtsam::Pose2>()));
      else
        initial.insert(value.key, value.value);
    }
    const auto optimized = gtsam::LevenbergMarquardtOptimizer(factors, initial).optimize();
    gtsam::ISAM2Params parameters;
    parameters.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
    parameters.relinearizeThreshold = 0.05;
    parameters.relinearizeSkip = std::max(1, params_.isam2_relinearize_skip);
    auto fresh = std::make_unique<gtsam::ISAM2>(parameters);
    fresh->update(factors, optimized);
    auto estimate = fresh->calculateEstimate();
    if (coupled)
      for (const auto& value : estimate)
      {
        const bool bias = gtsam::Symbol(value.key).chr() == 'b';
        if (bias ? !std::isfinite(value.value.cast<double>())
                 : !value.value.cast<gtsam::Pose2>().matrix().allFinite())
          return HeadingRecoveryResult::Failed;
      }
    TickOutput snapshot = *latest_;
    snapshot.pose = fresh->calculateEstimate<gtsam::Pose2>(PoseKey(snapshot.node_index));
    snapshot.covariance = fresh->marginalCovariance(PoseKey(snapshot.node_index));
    if (!snapshot.pose.matrix().allFinite() || !snapshot.covariance.allFinite())
      return HeadingRecoveryResult::Failed;
    double bias = current_bias_estimate_;
    if (params_.use_imu_preint)
    {
      bias = fresh->calculateEstimate<double>(gtsam::Symbol('b', snapshot.node_index));
      if (!std::isfinite(bias))
        return HeadingRecoveryResult::Failed;
    }
    snapshot.revision = std::max(snapshot.revision, candidate_state->revision) + 1;
    candidate_state->revision = snapshot.revision;
    candidate_state->relative_conditionals = true;
    ++candidate_state->yaw_factors;

    // All allocations and solver queries completed before publication. Existing
    // source frontier, cadence, accumulators and unrelated pending IDs survive.
    isam_.swap(fresh);
    current_estimate_.swap(estimate);
    estimate_dirty_ = false;
    motion_.swap(candidate_state);
    latest_ = snapshot;
    current_bias_estimate_ = bias;
    ticks_since_cov_ = 0;
    heading_seed_.reset();
    ++rebase_generation_;
    rebase_in_progress_ = false;
    rebase_pending_factors_.resize(0);
    rebase_pending_values_.clear();
    return HeadingRecoveryResult::Applied;
  }
  catch (const UnsupportedRelativeConditional&)
  {
    return HeadingRecoveryResult::UnsupportedSource;
  }
  catch (const std::exception&)
  {
    // Do not use the live update/reset helper on an isolated candidate failure.
    return HeadingRecoveryResult::Failed;
  }
}
}  // namespace fusion_graph
