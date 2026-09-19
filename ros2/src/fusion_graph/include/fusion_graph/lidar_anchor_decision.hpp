// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0
//
// Pure control decisions for the LiDAR map anchor. Particle-filter updates,
// map access and graph mutation deliberately stay in FusionGraphNode.
#pragma once

namespace fusion_graph
{

enum class LidarAnchorSeedAction
{
  kNone,
  kResetFromFusedPose,
  kReseedFromPredictionPreservingReference,
};

enum class LidarAnchorReferenceAction
{
  kNone,
  kRefreshFromFusedPose,
};

struct LidarAnchorPreparationInput
{
  bool shadow = false;
  bool compute_run = false;
  bool compute_reseed = false;
  bool new_filter = false;
  bool reference_valid = false;
  bool shadow_reference_due = false;
};

struct LidarAnchorPreparationDecision
{
  bool run_filter = false;
  LidarAnchorSeedAction seed = LidarAnchorSeedAction::kNone;
  LidarAnchorReferenceAction reference = LidarAnchorReferenceAction::kNone;
};

inline LidarAnchorPreparationDecision DecideLidarAnchorPreparation(
    const LidarAnchorPreparationInput& input)
{
  if (!input.compute_run)
  {
    return {false,
            LidarAnchorSeedAction::kNone,
            input.shadow ? LidarAnchorReferenceAction::kRefreshFromFusedPose
                         : LidarAnchorReferenceAction::kNone};
  }

  if (input.compute_reseed || input.new_filter || !input.reference_valid)
  {
    const bool preserve = !input.shadow && input.reference_valid;
    return {true,
            preserve ? LidarAnchorSeedAction::kReseedFromPredictionPreservingReference
                     : LidarAnchorSeedAction::kResetFromFusedPose,
            LidarAnchorReferenceAction::kNone};
  }

  return {true,
          LidarAnchorSeedAction::kNone,
          input.shadow && input.shadow_reference_due
              ? LidarAnchorReferenceAction::kRefreshFromFusedPose
              : LidarAnchorReferenceAction::kNone};
}

struct LidarAnchorCandidateDecisionInput
{
  bool accepted = false;
  bool apply_eligible = false;
  double now_s = 0.0;
  double lost_since_s = -1.0;
  double reseed_after_s = 0.0;
};

struct LidarAnchorCandidateDecision
{
  bool apply = false;
  LidarAnchorSeedAction seed = LidarAnchorSeedAction::kNone;
  double next_lost_since_s = -1.0;
};

inline LidarAnchorCandidateDecision DecideLidarAnchorCandidate(
    const LidarAnchorCandidateDecisionInput& input)
{
  if (input.accepted)
    return {input.apply_eligible, LidarAnchorSeedAction::kNone, -1.0};

  const double lost_since_s = input.lost_since_s < 0.0 ? input.now_s : input.lost_since_s;
  if ((input.now_s - lost_since_s) >= input.reseed_after_s)
  {
    return {false, LidarAnchorSeedAction::kReseedFromPredictionPreservingReference, -1.0};
  }
  return {false, LidarAnchorSeedAction::kNone, lost_since_s};
}

template <typename ReferenceState>
ReferenceState LidarAnchorReferenceAfterSeed(LidarAnchorSeedAction action,
                                             const ReferenceState& before_seed,
                                             ReferenceState reset_by_seed)
{
  if (action == LidarAnchorSeedAction::kReseedFromPredictionPreservingReference)
  {
    reset_by_seed.pose = before_seed.pose;
    reset_by_seed.dead_reckoning_pose = before_seed.dead_reckoning_pose;
    reset_by_seed.timestamp_s = before_seed.timestamp_s;
    reset_by_seed.path_m = before_seed.path_m;
  }
  return reset_by_seed;
}

}  // namespace fusion_graph
