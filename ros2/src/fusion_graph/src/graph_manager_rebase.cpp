// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphManager implementation — rebase/transform: RebaseISAM2, RigidTransformAll,
// etc.. (The class implementation is split across several translation units to keep each file
// within the project's 600-line budget; all share graph_manager.hpp + the inline PoseKey().)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <ios>
#include <sstream>

#include "fusion_graph/factors.hpp"
#include "fusion_graph/graph_manager.hpp"
#include <gtsam/base/GenericValue.h>
#include <gtsam/base/serialization.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PoseTranslationPrior.h>
#include <gtsam/slam/PriorFactor.h>

namespace fusion_graph
{
void GraphManager::RebaseISAM2()
{
  uint64_t generation = 0;
  try
  {
    // Phase 1: snapshot immutable factors and typed values under the lock.
    // Updates to the live graph are mirrored until this worker commits.
    gtsam::Values estimate_snapshot;
    gtsam::NonlinearFactorGraph graph_snapshot;
    int relinearize_skip = 1;
    uint64_t cutoff_index = 0;
    {
      std::lock_guard<std::mutex> lock(mu_);
      if (rebase_in_progress_)
        return;
      RefreshEstimateLocked();
      if (current_estimate_.empty())
        return;
      estimate_snapshot = current_estimate_;
      graph_snapshot = isam_.getFactorsUnsafe();
      relinearize_skip = params_.isam2_relinearize_skip;
      if (params_.max_graph_nodes > 0 && next_index_ > params_.max_graph_nodes)
        cutoff_index = next_index_ - params_.max_graph_nodes;
      generation = ++rebase_generation_;
      rebase_in_progress_ = true;
      rebase_pending_factors_.resize(0);
      rebase_pending_values_.clear();
    }

    // Phase 2: retain the original nonlinear likelihood on the live window.
    // Marginalize ONLY factors touching removed variables. Using a posterior
    // marginal from the full graph here would count the retained factors twice.
    // The separator factor freezes the removed history at this linearization;
    // all retained wheel/gyro/GNSS factors can still relinearize normally.
    gtsam::KeyVector removed_keys;
    gtsam::KeySet removed;
    gtsam::Values kept_values;
    for (const auto& kv : estimate_snapshot)
    {
      const gtsam::Symbol symbol(kv.key);
      // Bias is the per-pose preintegration state. Other namespaces keep their
      // original type and factors; never assume every variable is a Pose2.
      if ((symbol.chr() == 'x' || symbol.chr() == 'b') && symbol.index() < cutoff_index)
      {
        removed_keys.push_back(kv.key);
        removed.insert(kv.key);
      }
      else
        kept_values.insert(kv.key, kv.value);
    }
    gtsam::NonlinearFactorGraph retained, prefix;
    for (const auto& factor : graph_snapshot)
    {
      if (!factor)
        continue;
      const bool touches_removed = std::any_of(factor->begin(),
                                               factor->end(),
                                               [&](gtsam::Key key)
                                               {
                                                 return removed.count(key);
                                               });
      (touches_removed ? prefix : retained).push_back(factor);
    }
    if (!removed_keys.empty())
    {
      const auto linear_prefix = prefix.linearize(estimate_snapshot);
      const auto separator = linear_prefix->eliminatePartialSequential(removed_keys).second;
      for (const auto& factor : *separator)
      {
        if (factor && !factor->empty())
          retained.emplace_shared<gtsam::LinearContainerFactor>(factor, estimate_snapshot);
      }
    }

    gtsam::ISAM2Params p;
    p.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
    p.relinearizeThreshold = 0.05;
    p.relinearizeSkip = std::max(1, relinearize_skip);
    gtsam::ISAM2 fresh(p);
    fresh.update(retained, kept_values);

    // Phase 3: only the worker that owns these mirrors may replay or clear them.
    // Reset/RigidTransform cancels via the flag. A subsequent worker increments
    // the generation, preventing an old worker from overwriting the new graph
    // or clearing the new worker's pending updates (the cancellation ABA race).
    std::lock_guard<std::mutex> lock(mu_);
    if (!rebase_in_progress_ || generation != rebase_generation_)
      return;
    for (const auto& factor : rebase_pending_factors_)
    {
      if (!factor)
        continue;
      for (const auto key : *factor)
      {
        if (!fresh.valueExists(key) && !rebase_pending_values_.exists(key))
        {
          // A delayed observation may have reached a soon-to-be-removed state
          // while phase 2 ran. It is already assimilated in the live graph;
          // abandon this rebase rather than drop or retarget the observation.
          rebase_pending_factors_.resize(0);
          rebase_pending_values_.clear();
          rebase_in_progress_ = false;
          return;
        }
      }
    }
    if (!rebase_pending_factors_.empty() || !rebase_pending_values_.empty())
      fresh.update(rebase_pending_factors_, rebase_pending_values_);
    isam_ = std::move(fresh);
    estimate_dirty_ = true;
    rebase_pending_factors_.resize(0);
    rebase_pending_values_.clear();
    rebase_in_progress_ = false;
  }
  catch (const std::exception& e)
  {
    // This is called on a detached maintenance thread: every GTSAM operation,
    // including typed-value extraction and marginalization, must be guarded.
    // A failed rebuild never alters the already-updated live graph.
    std::fprintf(stderr,
                 "[fusion_graph] iSAM2 rebase failed (%s) — abandoning "
                 "rebase, live graph untouched.\n",
                 e.what());
    std::lock_guard<std::mutex> lock(mu_);
    if (generation != 0 && generation == rebase_generation_ && rebase_in_progress_)
    {
      rebase_pending_factors_.resize(0);
      rebase_pending_values_.clear();
      rebase_in_progress_ = false;
    }
  }
}

bool GraphManager::ApplyIsamUpdateLocked(const gtsam::NonlinearFactorGraph& fg,
                                         const gtsam::Values& values)
{
  try
  {
    isam_.update(fg, values);
  }
  catch (const std::exception& e)
  {
    // The iSAM2 update failed fatally — most commonly an
    // IndeterminantLinearSystemException (underconstrained, e.g. a
    // stationary graph that lost its only absolute anchor), but we catch the
    // whole std::exception family because ANY unmatched throw out of update()
    // SIGABRTs the node and kills localization entirely (field 2026-05-29,
    // dock-bootstrap crash at x62). iSAM2 is left inconsistent, so the only
    // safe recovery is a full rebuild. After ResetLocked() IsInitialized()==
    // false and the next GPS/dock seed re-bootstraps cleanly. We are already
    // under mu_ here. Return false so the caller bails THIS tick — continuing
    // would publish a garbage origin pose from the now-empty graph.
    std::fprintf(stderr,
                 "[fusion_graph] iSAM2 update failed (%s) — resetting graph "
                 "for a clean re-seed instead of aborting the node.\n",
                 e.what());
    ++stats_isam_resets_;
    ResetLocked();
    return false;
  }
  if (rebase_in_progress_)
  {
    // Mirror everything onto the pending buffer so phase 3 of the
    // rebase can replay it on the fresh iSAM2 before the swap.
    rebase_pending_factors_.push_back(fg);
    rebase_pending_values_.insert(values);
  }
  return true;
}

void GraphManager::RigidTransformAll(const gtsam::Pose2& correction,
                                     double latest_node_sigma_xy,
                                     double latest_node_sigma_theta)
{
  std::lock_guard<std::mutex> lock(mu_);

  // Refresh the cached estimate so we have every variable.
  RefreshEstimateLocked();
  if (current_estimate_.empty())
    return;

  // Apply correction to every Pose2 node. Non-pose variables (e.g.
  // gyro bias) are gauge-invariant — copy them through unchanged.
  gtsam::Values transformed;
  uint64_t latest_idx_local = (next_index_ > 0) ? next_index_ - 1 : 0;
  auto latest_key = PoseKey(latest_idx_local);
  for (const auto& kv : current_estimate_)
  {
    gtsam::Symbol s(kv.key);
    if (s.chr() == 'x')
    {
      const gtsam::Pose2 X_old = kv.value.cast<gtsam::Pose2>();
      const gtsam::Pose2 X_new = correction * X_old;
      transformed.insert(kv.key, X_new);
    }
    else
    {
      transformed.insert(kv.key, kv.value);
    }
  }

  // Build a fresh iSAM2 with priors at the shifted poses. Loose σ
  // (5 cm / 3°) on the older nodes so future absolute observations can still
  // refine them; tight σ on the latest node so the dock anchor isn't
  // washed out by the next stream of GPS factors when the robot
  // undocks.
  gtsam::ISAM2Params p;
  p.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
  p.relinearizeThreshold = 0.05;
  p.relinearizeSkip = std::max(1, params_.isam2_relinearize_skip);
  gtsam::ISAM2 fresh(p);

  gtsam::NonlinearFactorGraph fg;
  auto loose_noise = MakeDiagonal({0.05, 0.05, 0.05});
  auto tight_noise = MakeDiagonal({std::max(latest_node_sigma_xy, 1.0e-4),
                                   std::max(latest_node_sigma_xy, 1.0e-4),
                                   std::max(latest_node_sigma_theta, 1.0e-4)});
  // Rebuild from POSE keys only. `transformed` may also carry per-node
  // gyro-bias variables ('b', double) when use_imu_preint is on; the loop below
  // adds no factor for them, and handing iSAM2 new variables that no new factor
  // references makes the update underconstrained (IndeterminantLinearSystem →
  // ResetLocked wipes the graph). Bias is re-estimated live, so drop it here —
  // mirroring what Load() does.
  gtsam::Values pose_values;
  for (const auto& kv : transformed)
  {
    gtsam::Symbol s(kv.key);
    if (s.chr() != 'x')
      continue;
    const auto noise = (kv.key == latest_key) ? tight_noise : loose_noise;
    fg.add(gtsam::PriorFactor<gtsam::Pose2>(kv.key, kv.value.cast<gtsam::Pose2>(), noise));
    pose_values.insert(kv.key, kv.value);
  }
  // SAFETY (SAFETY_REVIEW_2026-07-23 G-H4): this runs in the executor thread
  // via SeedFromDockPose — an unguarded GTSAM throw would propagate out of
  // the callback and kill the node. A degenerate rebuild self-heals into a
  // clean re-seed instead (same posture as ApplyIsamUpdateLocked).
  try
  {
    fresh.update(fg, pose_values);
  }
  catch (const std::exception& e)
  {
    std::fprintf(stderr,
                 "[fusion_graph] iSAM2 rigid-transform rebuild failed (%s) — "
                 "resetting graph for a clean re-seed instead of aborting the "
                 "node.\n",
                 e.what());
    ++stats_isam_resets_;
    ResetLocked();
    return;
  }
  isam_ = std::move(fresh);
  estimate_dirty_ = true;

  // Cancel any in-flight async rebase (D2 race, field 2026-06-10 dock walk).
  // RebaseISAM2 phase 2 builds its fresh tree WITHOUT the lock from a snapshot
  // of the PRE-transform poses, leaving rebase_in_progress_ true. If it lands
  // after this swap, its phase-3 check (RebaseISAM2 ~L965) sees the flag still
  // set, does NOT bail, and overwrites our just-applied rigid correction with
  // the stale snapshot — silently undoing the dock re-pin. Clearing the flag
  // and the pending buffers makes that worker discard its tree, exactly as
  // ResetLocked does for the same reason.
  rebase_in_progress_ = false;
  rebase_pending_factors_.resize(0);
  rebase_pending_values_.clear();

  // Update the latched latest_ snapshot so the next PublishOutputs
  // sees the transformed pose instead of the pre-transform one.
  if (latest_)
  {
    latest_->pose = correction * latest_->pose;
  }
}

}  // namespace fusion_graph
