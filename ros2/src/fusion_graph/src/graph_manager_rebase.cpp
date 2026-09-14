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
void GraphManager::RebaseISAM2()
{
  // Phase 1: snapshot under the lock. The heavy work (building the
  // fresh iSAM2 from N PriorFactors) is then done WITHOUT the lock
  // so per-tick Tick() can keep publishing TF — see the comment on
  // rebase_in_progress_ in graph_manager.hpp for the 2026-05-14
  // incident that motivated this. While we're outside the lock,
  // Tick / ForceAnchor go through
  // ApplyIsamUpdateLocked, which mirrors their factors+values into
  // rebase_pending_factors_ / rebase_pending_values_ so the fresh
  // iSAM2 catches up at phase 3.
  gtsam::Values estimate_snapshot;
  int relinearize_skip = 1;
  uint64_t cutoff_index = 0;
  {
    std::lock_guard<std::mutex> lock(mu_);
    if (rebase_in_progress_)
    {
      // Another rebase is already running; bail rather than racing.
      return;
    }
    RefreshEstimateLocked();
    if (current_estimate_.empty())
      return;
    estimate_snapshot = current_estimate_;
    relinearize_skip = params_.isam2_relinearize_skip;
    // Sliding-window cutoff: drop pose nodes older than this index.
    // Captured under the lock against the live next_index_ so the
    // window is measured from the newest node at snapshot time.
    if (params_.max_graph_nodes > 0 && next_index_ > params_.max_graph_nodes)
      cutoff_index = next_index_ - params_.max_graph_nodes;
    rebase_in_progress_ = true;
    rebase_pending_factors_.resize(0);
    rebase_pending_values_.clear();
  }

  // Phase 2: build the fresh iSAM2 with priors-from-snapshot. This is
  // the O(N) expensive call (~1 s for 50k nodes on this robot).
  // Runs without mu_, so Tick() is free to advance the live iSAM2.
  gtsam::ISAM2Params p;
  p.optimizationParams = gtsam::ISAM2GaussNewtonParams(0.001);
  p.relinearizeThreshold = 0.05;
  p.relinearizeSkip = std::max(1, relinearize_skip);
  gtsam::ISAM2 fresh(p);

  // Re-anchor every existing variable with a tight prior. The exact
  // sigma is a balance: too tight and future absolute observations can't move
  // anything; too loose and iSAM2 wanders. 5 cm / 3° matches typical
  // RTK + COG noise floors and keeps the rebase non-destructive.
  gtsam::NonlinearFactorGraph fg;
  auto noise = MakeDiagonal({0.05, 0.05, 0.05});
  gtsam::Values kept_values;
  for (const auto& kv : estimate_snapshot)
  {
    // Sliding-window drop: skip pose nodes older than the cutoff.
    // gtsam::Symbol::index() recovers the monotonic node index from
    // the key. Non-pose keys (if any) fall through the window check
    // unchanged. cutoff_index == 0 means "keep everything".
    const gtsam::Symbol s(kv.key);
    if (cutoff_index > 0 && s.chr() == 'x' && s.index() < cutoff_index)
      continue;
    fg.add(gtsam::PriorFactor<gtsam::Pose2>(kv.key, kv.value.cast<gtsam::Pose2>(), noise));
    kept_values.insert(kv.key, kv.value);
  }
  // SAFETY (SAFETY_REVIEW_2026-07-23 G-H4): this update runs on the detached
  // maintenance thread — an unguarded GTSAM throw here propagates out of the
  // thread lambda and std::terminate()s the whole node (dead localizer
  // mid-mow). A failed rebuild is recoverable: abandon the rebase and leave
  // the live isam_ untouched.
  try
  {
    fresh.update(fg, kept_values);
  }
  catch (const std::exception& e)
  {
    std::fprintf(stderr,
                 "[fusion_graph] iSAM2 rebase rebuild failed (%s) — abandoning "
                 "rebase, live graph untouched.\n",
                 e.what());
    std::lock_guard<std::mutex> lock(mu_);
    rebase_pending_factors_.resize(0);
    rebase_pending_values_.clear();
    rebase_in_progress_ = false;
    return;
  }

  // Phase 3: replay anything Tick / ForceAnchor
  // added while we were rebuilding, then atomically swap isam_.
  // The lock is held only for this replay (typically a handful of
  // factors/values — ms-scale), so the TF publisher unblocks quickly.
  {
    std::lock_guard<std::mutex> lock(mu_);
    if (!rebase_in_progress_)
    {
      // Reset() ran between phase 1 and phase 3 — the graph we
      // built is now stale (its priors reference a state that no
      // longer exists). Discard fresh, leave isam_ alone.
      return;
    }
    if (rebase_pending_factors_.size() > 0 || rebase_pending_values_.size() > 0)
    {
      // SAFETY (G-H4): same terminate risk as phase 2 — a throw during the
      // replay abandons the rebase (fresh is discarded, live isam_ kept).
      try
      {
        fresh.update(rebase_pending_factors_, rebase_pending_values_);
      }
      catch (const std::exception& e)
      {
        std::fprintf(stderr,
                     "[fusion_graph] iSAM2 rebase replay failed (%s) — abandoning "
                     "rebase, live graph untouched.\n",
                     e.what());
        rebase_pending_factors_.resize(0);
        rebase_pending_values_.clear();
        rebase_in_progress_ = false;
        return;
      }
    }
    isam_ = std::move(fresh);
    estimate_dirty_ = true;
    rebase_pending_factors_.resize(0);
    rebase_pending_values_.clear();
    rebase_in_progress_ = false;
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
