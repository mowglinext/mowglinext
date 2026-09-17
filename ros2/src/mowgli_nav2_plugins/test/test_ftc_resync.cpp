// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the bounded carrot resync (ftc_resync.hpp). ROS-free.

#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "mowgli_nav2_plugins/ftc_resync.hpp"

namespace
{

using mowgli_nav2_plugins::FindResyncIndex;
using Plan = std::vector<std::pair<double, double>>;

/// Two parallel 10 m passes 0.16 m apart joined at the far end, 0.05 m pose
/// spacing: the geometry of adjacent headland rings / swaths.
Plan TwoAdjacentPasses()
{
  Plan plan;
  for (int i = 0; i <= 200; ++i)
  {
    plan.emplace_back(i * 0.05, 0.0);
  }
  for (int i = 200; i >= 0; --i)
  {
    plan.emplace_back(i * 0.05, 0.16);
  }
  return plan;
}

TEST(FtcResync, NeverJumpsToTheAdjacentPassTheRobotIsStandingOn)
{
  // Arrange — carrot at x=1.0 on pass 0; the robot skirted 0.16 m sideways and
  // is now geometrically ON pass 1, whose poses are ~18 m further in path order.
  const Plan plan = TwoAdjacentPasses();
  const std::size_t current = 20;

  // Act
  const auto r = FindResyncIndex(plan, current, 1.0, 0.16, 4.0);

  // Assert
  EXPECT_LE(r.index, 200u) << "resync crossed onto the next pass";
  EXPECT_NEAR(plan[r.index].first, 1.0, 0.051);
}

TEST(FtcResync, ReanchorsForwardWithinTheWindow)
{
  const Plan plan = TwoAdjacentPasses();
  const auto r = FindResyncIndex(plan, 20, 2.5, 0.02, 4.0);
  EXPECT_EQ(r.index, 50u);
  EXPECT_NEAR(r.distance_m, 0.02, 1e-9);
}

TEST(FtcResync, ReanchorsBackwardWithinTheWindow)
{
  // A reverse-escape leaves the robot BEHIND the carrot.
  const Plan plan = TwoAdjacentPasses();
  const auto r = FindResyncIndex(plan, 60, 2.0, 0.0, 4.0);
  EXPECT_EQ(r.index, 40u);
}

TEST(FtcResync, APoseBeyondTheWindowIsNotConsideredEvenIfItIsTheNearest)
{
  const Plan plan = TwoAdjacentPasses();
  // Robot sits exactly on x=8.0 of pass 0; window only reaches x=3.0.
  const auto r = FindResyncIndex(plan, 20, 8.0, 0.0, 2.0);
  EXPECT_LE(r.index, 60u);
  EXPECT_GT(r.distance_m, 4.9) << "the caller must see it is still too far and abort";
}

TEST(FtcResync, ZeroWindowKeepsTheCurrentIndex)
{
  const Plan plan = TwoAdjacentPasses();
  EXPECT_EQ(FindResyncIndex(plan, 33, 9.0, 9.0, 0.0).index, 33u);
}

TEST(FtcResync, OutOfRangeIndexAndEmptyPlanAreSafe)
{
  const Plan plan = TwoAdjacentPasses();
  EXPECT_LT(FindResyncIndex(plan, 99999, 0.0, 0.16, 1.0).index, plan.size());
  EXPECT_EQ(FindResyncIndex({}, 5, 1.0, 1.0, 1.0).index, 0u);
}

}  // namespace
