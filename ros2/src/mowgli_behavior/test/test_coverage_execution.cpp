// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "mowgli_behavior/coverage_execution.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::accountCoverageAdvance;
using mowgli_behavior::accountCoverageSkip;
using mowgli_behavior::executedAreaPoses;
using mowgli_behavior::fullyCoveredUnitCount;
using mowgli_behavior::UnitCoverageExecution;
using mowgli_behavior::UnitCoverageExecutionMap;

TEST(CoverageExecution, SkippedTargetsAndReplayEarnNoCredit)
{
  UnitCoverageExecution execution;
  accountCoverageSkip(execution, 0, 100);
  accountCoverageSkip(execution, 100, 200);
  EXPECT_EQ(execution.accounted_frontier, 200u);
  EXPECT_EQ(execution.executed_poses, 0u);
  EXPECT_TRUE(execution.has_gap);

  accountCoverageAdvance(execution, 150);
  EXPECT_EQ(execution.executed_poses, 0u);
  accountCoverageAdvance(execution, 240);
  EXPECT_EQ(execution.executed_poses, 40u);
  accountCoverageAdvance(execution, 240);
  EXPECT_EQ(execution.executed_poses, 40u);
}

TEST(CoverageExecution, ReachedPrefixAndSuffixCountButTheirGapDoesNot)
{
  UnitCoverageExecution execution;
  accountCoverageAdvance(execution, 40);
  accountCoverageSkip(execution, 60, 160);
  accountCoverageAdvance(execution, 200);
  EXPECT_EQ(execution.accounted_frontier, 200u);
  EXPECT_EQ(execution.executed_poses, 100u);
  EXPECT_TRUE(execution.has_gap);
}

TEST(CoverageExecution, NormalTrimAndReplayDoNotCreateAGap)
{
  UnitCoverageExecution execution;
  accountCoverageSkip(execution, 60, 60);
  accountCoverageAdvance(execution, 40);
  accountCoverageAdvance(execution, 80);
  EXPECT_EQ(execution.executed_poses, 80u);
  EXPECT_FALSE(execution.has_gap);
}

TEST(CoverageExecution, ProcessedGapsAndMissingProvenanceAreNotFullyCovered)
{
  const std::set<std::size_t> processed{0, 1, 2};
  const UnitCoverageExecutionMap units{{0, {100, 100, false}}, {1, {100, 20, true}}};
  EXPECT_EQ(fullyCoveredUnitCount(processed, &units), 1u);
  EXPECT_EQ(fullyCoveredUnitCount(processed, &units, true), 0u);
  EXPECT_EQ(fullyCoveredUnitCount(processed, nullptr), 0u);
  EXPECT_EQ(executedAreaPoses(&units), 120u);
  EXPECT_EQ(executedAreaPoses(nullptr), 0u);
}

TEST(CoverageExecution, SkippingInsideAnAlreadyExecutedPrefixDoesNotInventANewGap)
{
  UnitCoverageExecution execution;
  accountCoverageAdvance(execution, 100);
  accountCoverageSkip(execution, 60, 80);
  EXPECT_EQ(execution.executed_poses, 100u);
  EXPECT_EQ(execution.accounted_frontier, 100u);
  EXPECT_FALSE(execution.has_gap);
  accountCoverageSkip(execution, 80, 120);
  EXPECT_EQ(execution.executed_poses, 100u);
  EXPECT_EQ(execution.accounted_frontier, 120u);
  EXPECT_TRUE(execution.has_gap);
}
