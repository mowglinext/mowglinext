// Copyright (C) 2024 Cedric <cedric@mowgli.dev>
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

// Unit tests for the geofence blade-kill classifier. Pure logic, no ROS —
// firmware has no polygon knowledge, so this is the ONLY geofence
// enforcement in the system.

#include <limits>

#include "mowgli_map/boundary_classifier.hpp"
#include <gtest/gtest.h>

namespace mm = mowgli_map;

TEST(ClassifyBoundary, InsideAnyAssertsNothing)
{
  int consecutive = 0;
  const auto c = mm::ClassifyBoundary(/*inside_any=*/true,
                                      /*min_edge_dist=*/999.0,
                                      /*soft_margin=*/0.20,
                                      /*lethal_margin=*/0.50,
                                      /*debounce_samples=*/3,
                                      consecutive);
  EXPECT_FALSE(c.soft);
  EXPECT_FALSE(c.lethal);
  EXPECT_EQ(consecutive, 0);
}

TEST(ClassifyBoundary, WithinSoftMarginAssertsNothing)
{
  // Outside every polygon but within the soft margin — not a violation.
  int consecutive = 0;
  const auto c = mm::ClassifyBoundary(false, /*min_edge_dist=*/0.10, 0.20, 0.50, 3, consecutive);
  EXPECT_FALSE(c.soft);
  EXPECT_FALSE(c.lethal);
  EXPECT_EQ(consecutive, 0);
}

TEST(ClassifyBoundary, LethalMarginAssertsImmediatelyWithNoDebounce)
{
  // 0.6 m outside, single sample: lethal fires on sample #1 — the blade
  // hazard cannot wait for a debounce window.
  int consecutive = 0;
  const auto c = mm::ClassifyBoundary(false, /*min_edge_dist=*/0.60, 0.20, 0.50, 3, consecutive);
  EXPECT_TRUE(c.lethal);
  // soft_outside is also true here (0.60 > 0.20 soft margin) but the
  // debounce counter has only reached 1 of 3 — soft stays unlatched even
  // though lethal already fired. This is the deliberate asymmetry.
  EXPECT_FALSE(c.soft);
  EXPECT_EQ(consecutive, 1);
}

TEST(ClassifyBoundary, SoftMarginRequiresConsecutiveSamplesBeforeAsserting)
{
  int consecutive = 0;
  const double min_edge_dist = 0.25;  // > soft (0.20), < lethal (0.50).
  const int debounce_samples = 3;

  // Samples 1 and 2: soft-outside but under the debounce threshold.
  auto c1 = mm::ClassifyBoundary(false, min_edge_dist, 0.20, 0.50, debounce_samples, consecutive);
  EXPECT_FALSE(c1.soft);
  EXPECT_FALSE(c1.lethal);
  EXPECT_EQ(consecutive, 1);

  auto c2 = mm::ClassifyBoundary(false, min_edge_dist, 0.20, 0.50, debounce_samples, consecutive);
  EXPECT_FALSE(c2.soft);
  EXPECT_EQ(consecutive, 2);

  // Sample 3 reaches the debounce threshold — soft latches.
  auto c3 = mm::ClassifyBoundary(false, min_edge_dist, 0.20, 0.50, debounce_samples, consecutive);
  EXPECT_TRUE(c3.soft);
  EXPECT_FALSE(c3.lethal);
  EXPECT_EQ(consecutive, 3);
}

TEST(ClassifyBoundary, SingleInsideSampleResetsDebounceCounter)
{
  // Reproduces the regression this debounce guards against: a single-tick
  // TF wobble (COG/mag shifting map→odom a few cm) must not carry over
  // partial debounce progress into the next outside streak.
  int consecutive = 0;
  const double outside_dist = 0.25;

  mm::ClassifyBoundary(false, outside_dist, 0.20, 0.50, 3, consecutive);
  mm::ClassifyBoundary(false, outside_dist, 0.20, 0.50, 3, consecutive);
  ASSERT_EQ(consecutive, 2);

  // One good sample (back inside) resets the counter.
  mm::ClassifyBoundary(true, 0.0, 0.20, 0.50, 3, consecutive);
  EXPECT_EQ(consecutive, 0);

  // Two more outside samples: still below threshold because the streak
  // restarted, not 4-of-5.
  auto c = mm::ClassifyBoundary(false, outside_dist, 0.20, 0.50, 3, consecutive);
  EXPECT_FALSE(c.soft);
  EXPECT_EQ(consecutive, 1);
}

TEST(ClassifyBoundary, ExactlyAtSoftMarginIsNotOutside)
{
  // min_edge_dist == soft_margin_m must NOT count as outside (strict > only,
  // mirrors GpsJumpImplausible's strict boundary convention).
  int consecutive = 0;
  const auto c = mm::ClassifyBoundary(false, /*min_edge_dist=*/0.20, 0.20, 0.50, 1, consecutive);
  EXPECT_FALSE(c.soft);
  EXPECT_EQ(consecutive, 0);
}

TEST(ClassifyBoundary, ExactlyAtLethalMarginIsNotLethal)
{
  int consecutive = 0;
  const auto c = mm::ClassifyBoundary(false, /*min_edge_dist=*/0.50, 0.20, 0.50, 1, consecutive);
  EXPECT_FALSE(c.lethal);
  // Still soft-outside (0.50 > 0.20) and debounce_samples=1 latches it.
  EXPECT_TRUE(c.soft);
}

TEST(ClassifyBoundary, DebounceCounterSaturatesInsteadOfOverflowing)
{
  int consecutive = std::numeric_limits<int>::max() - 1;
  const auto c = mm::ClassifyBoundary(false, 0.25, 0.20, 0.50, 3, consecutive);
  EXPECT_EQ(consecutive, std::numeric_limits<int>::max());
  EXPECT_TRUE(c.soft);
  // One more sample must not wrap around to a negative counter.
  const auto c2 = mm::ClassifyBoundary(false, 0.25, 0.20, 0.50, 3, consecutive);
  EXPECT_EQ(consecutive, std::numeric_limits<int>::max());
  EXPECT_TRUE(c2.soft);
}
