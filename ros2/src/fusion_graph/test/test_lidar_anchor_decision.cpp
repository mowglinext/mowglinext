// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0

#include "fusion_graph/lidar_anchor_decision.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{
struct ReferenceMemory
{
  int pose = 0;
  int dead_reckoning_pose = 0;
  int last_dead_reckoning_pose = 0;
  double timestamp_s = 0.0;
  double path_m = 0.0;
  bool valid = false;

  bool operator==(const ReferenceMemory&) const = default;
};
}  // namespace

TEST(LidarAnchorDecision, FirstSeedResetsReferenceFromFusedPose)
{
  fg::LidarAnchorPreparationInput input;
  input.compute_run = true;
  input.reference_valid = false;

  const auto decision = fg::DecideLidarAnchorPreparation(input);

  EXPECT_TRUE(decision.run_filter);
  EXPECT_EQ(decision.seed, fg::LidarAnchorSeedAction::kResetFromFusedPose);
  EXPECT_EQ(decision.reference, fg::LidarAnchorReferenceAction::kNone);

  const ReferenceMemory stale_reference{1, 2, 3, 4.0, 5.0, false};
  const ReferenceMemory fused_reference{6, 7, 8, 9.0, 0.0, true};
  EXPECT_EQ(fg::LidarAnchorReferenceAfterSeed(decision.seed, stale_reference, fused_reference),
            fused_reference);
}

TEST(LidarAnchorDecision, ShadowWithoutComputeRefreshesTrustedFusedReference)
{
  fg::LidarAnchorPreparationInput input;
  input.shadow = true;
  input.reference_valid = true;

  const auto decision = fg::DecideLidarAnchorPreparation(input);

  EXPECT_FALSE(decision.run_filter);
  EXPECT_EQ(decision.seed, fg::LidarAnchorSeedAction::kNone);
  EXPECT_EQ(decision.reference, fg::LidarAnchorReferenceAction::kRefreshFromFusedPose);
}

TEST(LidarAnchorDecision, ShadowComputeRefreshesReferenceOnlyWhenDue)
{
  fg::LidarAnchorPreparationInput input;
  input.shadow = true;
  input.compute_run = true;
  input.reference_valid = true;

  EXPECT_EQ(fg::DecideLidarAnchorPreparation(input).reference,
            fg::LidarAnchorReferenceAction::kNone);
  input.shadow_reference_due = true;
  EXPECT_EQ(fg::DecideLidarAnchorPreparation(input).reference,
            fg::LidarAnchorReferenceAction::kRefreshFromFusedPose);
}

TEST(LidarAnchorDecision, OutageTileSwapReseedPreservesReferenceTimestampAndPathBudget)
{
  fg::LidarAnchorPreparationInput input;
  input.compute_run = true;
  input.new_filter = true;
  input.reference_valid = true;

  const auto tile_swap = fg::DecideLidarAnchorPreparation(input);
  EXPECT_EQ(tile_swap.seed, fg::LidarAnchorSeedAction::kReseedFromPredictionPreservingReference);

  input.new_filter = false;
  input.compute_reseed = true;
  const auto compute_reseed = fg::DecideLidarAnchorPreparation(input);
  EXPECT_EQ(compute_reseed.seed,
            fg::LidarAnchorSeedAction::kReseedFromPredictionPreservingReference);

  const ReferenceMemory outage_reference{41, 73, 74, 125.5, 12.75, true};
  const ReferenceMemory reset_by_filter_seed{99, 100, 101, 140.0, 0.0, true};
  const auto after_reseed =
      fg::LidarAnchorReferenceAfterSeed(tile_swap.seed, outage_reference, reset_by_filter_seed);
  EXPECT_EQ(after_reseed.pose, outage_reference.pose);
  EXPECT_EQ(after_reseed.dead_reckoning_pose, outage_reference.dead_reckoning_pose);
  EXPECT_DOUBLE_EQ(after_reseed.timestamp_s, outage_reference.timestamp_s);
  EXPECT_DOUBLE_EQ(after_reseed.path_m, outage_reference.path_m);
  EXPECT_EQ(after_reseed.last_dead_reckoning_pose, reset_by_filter_seed.last_dead_reckoning_pose);
  EXPECT_TRUE(after_reseed.valid);
}

TEST(LidarAnchorDecision, RejectedCandidateReseedsOnlyAfterDwellAndPreservesReference)
{
  fg::LidarAnchorCandidateDecisionInput input;
  input.now_s = 10.0;
  input.lost_since_s = -1.0;
  input.reseed_after_s = 5.0;

  auto decision = fg::DecideLidarAnchorCandidate(input);
  EXPECT_EQ(decision.seed, fg::LidarAnchorSeedAction::kNone);
  EXPECT_DOUBLE_EQ(decision.next_lost_since_s, 10.0);

  input.now_s = 14.9;
  input.lost_since_s = decision.next_lost_since_s;
  decision = fg::DecideLidarAnchorCandidate(input);
  EXPECT_EQ(decision.seed, fg::LidarAnchorSeedAction::kNone);
  EXPECT_DOUBLE_EQ(decision.next_lost_since_s, 10.0);

  input.now_s = 15.0;
  input.lost_since_s = decision.next_lost_since_s;
  decision = fg::DecideLidarAnchorCandidate(input);
  EXPECT_EQ(decision.seed, fg::LidarAnchorSeedAction::kReseedFromPredictionPreservingReference);
  EXPECT_DOUBLE_EQ(decision.next_lost_since_s, -1.0);

  const ReferenceMemory outage_reference{7, 11, 12, 50.0, 8.5, true};
  const ReferenceMemory reset_by_filter_seed{20, 21, 22, 55.0, 0.0, true};
  const auto after_reseed =
      fg::LidarAnchorReferenceAfterSeed(decision.seed, outage_reference, reset_by_filter_seed);
  EXPECT_EQ(after_reseed.pose, outage_reference.pose);
  EXPECT_EQ(after_reseed.dead_reckoning_pose, outage_reference.dead_reckoning_pose);
  EXPECT_DOUBLE_EQ(after_reseed.timestamp_s, outage_reference.timestamp_s);
  EXPECT_DOUBLE_EQ(after_reseed.path_m, outage_reference.path_m);
  EXPECT_EQ(after_reseed.last_dead_reckoning_pose, reset_by_filter_seed.last_dead_reckoning_pose);
  EXPECT_TRUE(after_reseed.valid);
}

TEST(LidarAnchorDecision, AcceptedCandidateResetsRejectionDwellAndAppliesWhenEligible)
{
  fg::LidarAnchorCandidateDecisionInput input;
  input.accepted = true;
  input.apply_eligible = true;
  input.now_s = 20.0;
  input.lost_since_s = 10.0;
  input.reseed_after_s = 5.0;

  const auto decision = fg::DecideLidarAnchorCandidate(input);
  EXPECT_TRUE(decision.apply);
  EXPECT_EQ(decision.seed, fg::LidarAnchorSeedAction::kNone);
  EXPECT_DOUBLE_EQ(decision.next_lost_since_s, -1.0);
}

TEST(LidarAnchorDecision, AcceptedCandidateClearsDwellWithoutApplyingWhenIneligible)
{
  fg::LidarAnchorCandidateDecisionInput input;
  input.accepted = true;
  input.apply_eligible = false;
  input.now_s = 20.0;
  input.lost_since_s = 10.0;
  input.reseed_after_s = 5.0;

  const auto decision = fg::DecideLidarAnchorCandidate(input);
  EXPECT_FALSE(decision.apply);
  EXPECT_EQ(decision.seed, fg::LidarAnchorSeedAction::kNone);
  EXPECT_DOUBLE_EQ(decision.next_lost_since_s, -1.0);
}

TEST(LidarAnchorDecision, NoComputeDuringOutageDoesNothing)
{
  fg::LidarAnchorPreparationInput input;
  input.reference_valid = true;
  input.compute_reseed = true;
  input.new_filter = true;

  const auto decision = fg::DecideLidarAnchorPreparation(input);

  EXPECT_FALSE(decision.run_filter);
  EXPECT_EQ(decision.seed, fg::LidarAnchorSeedAction::kNone);
  EXPECT_EQ(decision.reference, fg::LidarAnchorReferenceAction::kNone);
}
