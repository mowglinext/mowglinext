// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
/**
 * @file test_transit_avoidance.cpp
 * @brief Session failed-transit-target avoidance (transit_avoidance.hpp,
 *        issue #732): once a blade-off inter-unit transit to some target has
 *        failed, sendCurrentSwath must not repeat it on a later dispatch of
 *        the same area this session.
 *
 * Field 2026-09-21: a transit blocked by a LiDAR-observed, undrawn obstacle
 * was retried identically on every subsequent dispatch of the area, up to
 * kMaxAreaAttempts, each attempt burning the full transitDeadlineSec bound
 * plus nav2's own retry cycle, before the area was finally given up on.
 */

#include <vector>

#include "mowgli_behavior/transit_avoidance.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::FailedTransitTarget;
using mowgli_behavior::isKnownFailedTransit;
using mowgli_behavior::kFailedTransitMergeDistM;
using mowgli_behavior::kMaxSessionFailedTransits;
using mowgli_behavior::recordFailedTransit;

// ---------------------------------------------------------------------------
// isKnownFailedTransit
// ---------------------------------------------------------------------------

TEST(TransitAvoidanceZone, TargetWithinTheRadiusOfAFailedTargetIsKnown)
{
  const std::vector<FailedTransitTarget> targets = {{5.0, 5.0}};
  EXPECT_TRUE(isKnownFailedTransit(5.0, 5.0, targets, 0.30));
  EXPECT_TRUE(isKnownFailedTransit(5.2, 5.0, targets, 0.30));  // 0.2 m away
  EXPECT_FALSE(isKnownFailedTransit(6.0, 5.0, targets, 0.30));  // 1.0 m away
}

TEST(TransitAvoidanceZone, NoTargetsOrANonPositiveRadiusDisablesTheCheck)
{
  EXPECT_FALSE(isKnownFailedTransit(5.0, 5.0, {}, 0.30));
  const std::vector<FailedTransitTarget> targets = {{5.0, 5.0}};
  EXPECT_FALSE(isKnownFailedTransit(5.0, 5.0, targets, 0.0));
  EXPECT_FALSE(isKnownFailedTransit(5.0, 5.0, targets, -1.0));
}

// ---------------------------------------------------------------------------
// recordFailedTransit
// ---------------------------------------------------------------------------

TEST(TransitAvoidanceRecord, RepeatFailureAtTheSameTargetIsNotStoredTwice)
{
  std::vector<FailedTransitTarget> targets;
  targets = recordFailedTransit(targets, {5.0, 5.0});
  ASSERT_EQ(targets.size(), 1u);
  // 0.1 m away — inside kFailedTransitMergeDistM (0.30 m): merges, not a new entry.
  targets = recordFailedTransit(targets, {5.1, 5.0});
  EXPECT_EQ(targets.size(), 1u);
}

TEST(TransitAvoidanceRecord, DistinctTargetsAreAllKeptAndTheInputIsNotMutated)
{
  const std::vector<FailedTransitTarget> original = {{1.0, 1.0}};
  const std::vector<FailedTransitTarget> updated = recordFailedTransit(original, {10.0, 10.0});
  EXPECT_EQ(original.size(), 1u);  // input untouched — pure function
  ASSERT_EQ(updated.size(), 2u);
  EXPECT_TRUE(isKnownFailedTransit(1.0, 1.0, updated));
  EXPECT_TRUE(isKnownFailedTransit(10.0, 10.0, updated));
}

TEST(TransitAvoidanceRecord, OldestTargetIsDroppedOnceTheBoundIsReached)
{
  std::vector<FailedTransitTarget> targets;
  for (std::size_t i = 0; i < kMaxSessionFailedTransits; ++i)
  {
    // Spread targets 1 m apart so none merges with another.
    targets = recordFailedTransit(targets, {static_cast<double>(i) * 1.0, 0.0});
  }
  ASSERT_EQ(targets.size(), kMaxSessionFailedTransits);
  // The very first target (index 0, at x=0) is still present...
  EXPECT_TRUE(isKnownFailedTransit(0.0, 0.0, targets));
  // ...until one more failure pushes it out.
  targets =
      recordFailedTransit(targets, {static_cast<double>(kMaxSessionFailedTransits) * 1.0, 0.0});
  EXPECT_EQ(targets.size(), kMaxSessionFailedTransits);
  EXPECT_FALSE(isKnownFailedTransit(0.0, 0.0, targets));
}

TEST(TransitAvoidanceRecord, MergeToleranceDefaultsMatchIsKnownFailedTransitDefault)
{
  // Both helpers default to the SAME tolerance — recording a target and then
  // immediately checking it back with default arguments must agree, or a
  // caller that uses the defaults on both sides could silently disagree.
  std::vector<FailedTransitTarget> targets;
  targets = recordFailedTransit(targets, {5.0, 5.0});
  EXPECT_TRUE(isKnownFailedTransit(5.0 + kFailedTransitMergeDistM * 0.5, 5.0, targets));
  EXPECT_FALSE(isKnownFailedTransit(5.0 + kFailedTransitMergeDistM * 2.0, 5.0, targets));
}
