// Copyright 2026 MowgliNext contributors
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <limits>

#include "fusion_graph/lidar_anchor_validator.hpp"
#include <gtest/gtest.h>

using fusion_graph::DeadReckoningBudgetM;
using fusion_graph::LidarAnchorCandidate;
using fusion_graph::LidarAnchorValidatorParams;
using fusion_graph::LidarAnchorVerdict;
using fusion_graph::ValidateLidarAnchor;

namespace
{
LidarAnchorCandidate Good()
{
  LidarAnchorCandidate c;
  c.x = 3.0;
  c.y = 4.0;
  c.sigma_m = 0.08;
  c.hit_ratio = 0.92;
  c.hit_count = 250;
  c.dr_x = 3.05;
  c.dr_y = 3.97;
  c.dist_since_seed_m = 5.0;
  return c;
}
}  // namespace

TEST(LidarAnchorValidator, AcceptsAConsistentTightPlausibleEstimate)
{
  EXPECT_EQ(ValidateLidarAnchor(Good(), {}), LidarAnchorVerdict::kAccepted);
}

// The 2026-09-06 shape: 13 % of beams on known walls. Whatever the filter's
// covariance says, that estimate must not become a factor.
TEST(LidarAnchorValidator, RejectsTheLostFilterByScanConsistency)
{
  auto c = Good();
  c.hit_ratio = 0.13;
  c.sigma_m = 0.03;  // tight AND wrong — the classic particle-filter failure
  EXPECT_EQ(ValidateLidarAnchor(c, {}), LidarAnchorVerdict::kRejectedScore);
}

TEST(LidarAnchorValidator, RejectsAHighRatioOnTooFewBeams)
{
  auto c = Good();
  c.hit_ratio = 1.0;
  c.hit_count = 5;  // five beams on a wall is not a fix
  EXPECT_EQ(ValidateLidarAnchor(c, {}), LidarAnchorVerdict::kRejectedScore);
}

TEST(LidarAnchorValidator, RejectsAWideParticleCloud)
{
  auto c = Good();
  c.sigma_m = 0.9;
  EXPECT_EQ(ValidateLidarAnchor(c, {}), LidarAnchorVerdict::kRejectedSpread);
  c.sigma_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(ValidateLidarAnchor(c, {}), LidarAnchorVerdict::kRejectedSpread);
}

// Dead reckoning is the third witness: 0.3 m + 2 % of the distance driven.
TEST(LidarAnchorValidator, RejectsAnEstimateDeadReckoningCouldNotHaveDriftedTo)
{
  auto c = Good();
  c.dist_since_seed_m = 10.0;  // budget = 0.3 + 0.2 = 0.5 m
  EXPECT_NEAR(DeadReckoningBudgetM({}, 10.0), 0.5, 1e-9);
  c.dr_x = c.x + 0.45;
  EXPECT_EQ(ValidateLidarAnchor(c, {}), LidarAnchorVerdict::kAccepted);
  c.dr_x = c.x + 0.55;
  EXPECT_EQ(ValidateLidarAnchor(c, {}), LidarAnchorVerdict::kRejectedDeadReckoning);
}

TEST(LidarAnchorValidator, BudgetNeverShrinksBelowTheConstant)
{
  EXPECT_NEAR(DeadReckoningBudgetM({}, -3.0), 0.3, 1e-9);
  EXPECT_NEAR(DeadReckoningBudgetM({}, 0.0), 0.3, 1e-9);
}

TEST(LidarAnchorValidator, ScoreIsCheckedBeforeSpreadAndDeadReckoning)
{
  auto c = Good();
  c.hit_ratio = 0.1;
  c.sigma_m = 5.0;
  c.dr_x = 100.0;
  EXPECT_EQ(ValidateLidarAnchor(c, {}), LidarAnchorVerdict::kRejectedScore);
}
