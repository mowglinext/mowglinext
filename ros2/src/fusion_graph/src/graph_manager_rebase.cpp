// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// GraphManager implementation — rebase/loop-closure/transform: RebaseISAM2, RigidTransformAll,
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
std::vector<uint64_t> GraphManager::FindLoopClosureCandidates(uint64_t query_index,
                                                              double max_dist_m,
                                                              double min_age_s,
                                                              size_t max_candidates) const
{
  std::lock_guard<std::mutex> lock(mu_);
  std::vector<uint64_t> out;
  if (next_index_ == 0)
    return out;

  if (!HasPoseAt(query_index))
    return out;
  const auto Xq = PoseAt(query_index);

  // Per-node age proxy: nodes are created at node_period_s cadence,
  // so age_idx = (next - i) * node_period_s. Within ±10% of wall
  // clock, sufficient for the >30s gate.
  const double age_per_idx = params_.node_period_s;
  const auto cutoff_idx_diff = static_cast<uint64_t>(std::ceil(min_age_s / age_per_idx));

  // Window bound: never offer a candidate older than the max_graph_nodes sliding
  // window. Its pose is (or is imminently) marginalized by the maintenance
  // rebase, so a loop-closure factor to it either fails or resurrects a node the
  // window is trying to drop — the mechanism behind the unbounded factor growth
  // that OOM-killed the node 2026-06-09 (an LC formed to node 2098 while the live
  // index was ~10138). scans_ are retained far longer than the pose window
  // (scan_retention_nodes), so the HasPoseAt filter alone did not bound this.
  // max_graph_nodes==0 disables the window → no extra bound.
  uint64_t window_cutoff = 0;
  if (params_.max_graph_nodes > 0 && next_index_ > params_.max_graph_nodes)
    window_cutoff = next_index_ - params_.max_graph_nodes;

  // Linear scan over scans_ keys (== nodes with a stored scan, which
  // is what we want — no point loop-closing to a node without a
  // scan). PoseAt is O(depth) on the Bayes tree path, so this loop
  // is roughly O(scans_.size() · depth).
  std::vector<std::pair<double, uint64_t>> hits;
  hits.reserve(scans_.size());
  const double max_d2 = max_dist_m * max_dist_m;
  for (const auto& [idx, _] : scans_)
  {
    if (idx == query_index)
      continue;
    if (idx < window_cutoff)
      continue;
    if (query_index - idx < cutoff_idx_diff)
      continue;
    if (!HasPoseAt(idx))
      continue;
    const auto X = PoseAt(idx);
    const double dx = X.x() - Xq.x();
    const double dy = X.y() - Xq.y();
    const double d2 = dx * dx + dy * dy;
    if (d2 <= max_d2)
      hits.emplace_back(d2, idx);
  }

  std::sort(hits.begin(), hits.end());
  for (size_t i = 0; i < hits.size() && out.size() < max_candidates; ++i)
    out.push_back(hits[i].second);
  return out;
}

void GraphManager::PruneOldScans(uint64_t max_age_nodes)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (next_index_ <= max_age_nodes)
    return;
  const uint64_t cutoff = next_index_ - max_age_nodes;
  auto it = scans_.begin();
  while (it != scans_.end())
  {
    if (it->first < cutoff)
      it = scans_.erase(it);
    else
      break;  // map is ordered by key, rest is newer
  }
}

void GraphManager::RebaseISAM2()
{
  // Phase 1: snapshot under the lock. The heavy work (building the
  // fresh iSAM2 from N PriorFactors) is then done WITHOUT the lock
  // so per-tick Tick() can keep publishing TF — see the comment on
  // rebase_in_progress_ in graph_manager.hpp for the 2026-05-14
  // incident that motivated this. While we're outside the lock,
  // Tick / ForceAnchor / AddLoopClosure go through
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
  // sigma is a balance: too tight and future loop closures can't move
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
  fresh.update(fg, kept_values);

  // Phase 3: replay anything Tick / ForceAnchor / AddLoopClosure
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
      fresh.update(rebase_pending_factors_, rebase_pending_values_);
    }
    isam_ = std::move(fresh);
    estimate_dirty_ = true;
    // Loop-closure edges accumulated so far were collapsed into the
    // priors; reset the visualization list so future LCs are
    // distinguishable from the rebased history.
    loop_closure_edges_.clear();
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
  // (5 cm / 3°) on the older nodes so future loop closures can still
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
  for (const auto& kv : transformed)
  {
    gtsam::Symbol s(kv.key);
    if (s.chr() != 'x')
      continue;
    const auto noise = (kv.key == latest_key) ? tight_noise : loose_noise;
    fg.add(gtsam::PriorFactor<gtsam::Pose2>(kv.key, kv.value.cast<gtsam::Pose2>(), noise));
  }
  fresh.update(fg, transformed);
  isam_ = std::move(fresh);
  estimate_dirty_ = true;
  // Loop-closure edges collapsed into priors during the rebuild.
  loop_closure_edges_.clear();

  // Co-transform the frozen keyframe map by the SAME correction so the absolute
  // keyframe constraints stay consistent with the rigidly-corrected live frame.
  // Keyframes are NOT iSAM2 variables (untouched by the pose loop above), so
  // without this the map and the live trajectory desync by `correction` and the
  // scan-to-keyframe factor would then drag the robot off-truth — a silent gauge
  // break. scan_body is body-frame and gauge-invariant; only abs_pose moves.
  // Ordered BEFORE the D2 cleanup below (same mu_ section) so a late async
  // rebase cannot revert the shift.
  for (auto& [id, kf] : keyframes_)
    kf.abs_pose = correction * kf.abs_pose;

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

void GraphManager::AddLoopClosure(uint64_t prev_index,
                                  uint64_t curr_index,
                                  const gtsam::Pose2& delta,
                                  double sigma_xy,
                                  double sigma_theta)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (sigma_xy <= 0.0)
    sigma_xy = 0.5;
  if (sigma_theta <= 0.0)
    sigma_theta = 0.1;

  if (!HasPoseAt(prev_index) || !HasPoseAt(curr_index))
    return;
  auto k_prev = PoseKey(prev_index);
  auto k_curr = PoseKey(curr_index);

  // Robust noise model on loop-closure between-factors (item #11).
  // Wraps the diagonal Gaussian in a Dynamic Covariance Scaling
  // (DCS) m-estimator. DCS smoothly downweights an LC whose
  // residual exceeds ~k·σ instead of letting a single bad LC
  // anchor the entire trajectory to a wrong place — even with the
  // upstream ICP guards (PR #233) and rmse acceptance gate, a
  // degenerate match can still squeak through on symmetric
  // outdoor scenery. DCS keeps inliers fully efficient (factor
  // weight ≈ 1 when residual is below k·σ) and decays the weight
  // quadratically beyond. Cheaper than PCM and well-validated in
  // the SLAM literature (Agarwal et al., "Robust Map Optimization
  // using Dynamic Covariance Scaling", ICRA 2013).
  //
  // DCS shape parameter Φ (kDcsPhi): residuals below √Φ are
  // unaffected; above, the loss switches from quadratic to
  // sub-quadratic. Φ = 1 is the classic value — equivalent to
  // saying "an LC residual of 1 σ is borderline acceptable".
  auto base_noise = MakeDiagonal({sigma_xy, sigma_xy, sigma_theta});
  constexpr double kDcsPhi = 1.0;
  auto robust_noise =
      gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::DCS::Create(kDcsPhi),
                                        base_noise);

  gtsam::NonlinearFactorGraph fg;
  fg.add(gtsam::BetweenFactor<gtsam::Pose2>(k_prev, k_curr, delta, robust_noise));

  if (!ApplyIsamUpdateLocked(fg, gtsam::Values()))
  {
    // Loop-closure factor triggered an ill-posed reset; the graph is now
    // empty — don't record the (now-meaningless) edge/count.
    return;
  }
  estimate_dirty_ = true;
  ++loop_closures_added_;
  loop_closure_edges_.emplace_back(prev_index, curr_index);
}

}  // namespace fusion_graph
