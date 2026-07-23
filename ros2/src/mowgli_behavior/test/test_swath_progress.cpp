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

// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_swath_progress.cpp
 * @brief Regression test for the GUI live mowing-progress scalars (Bug B).
 *
 * The GUI computes the mowing percentage as current_path_index / current_path,
 * fed by HighLevelStatus.completed_swaths / total_swaths, which are mirrored
 * from ctx.completed_swaths / ctx.total_swaths. These used to be written only at
 * the terminal branch of a completed area pass, so during mowing total_swaths
 * stayed 0 and the GUI showed no percentage. refreshSwathProgress now seeds them
 * at pass start and refreshes them on every swath boundary. These tests pin that
 * behaviour by driving the helper through a simulated pass with a plain
 * BTContext (no ROS, no action servers).
 */

#include <cstddef>
#include <set>

#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/coverage_nodes.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::BTContext;
using mowgli_behavior::coveragePercentFromCursor;
using mowgli_behavior::refreshSwathProgress;

namespace
{
constexpr uint32_t kArea0 = 0;
constexpr uint32_t kArea1 = 1;
}  // namespace

// At pass start with nothing mowed yet, total is the unit count (> 0) and
// completed is 0 — so current_path > 0 and the GUI ratio renders (0 %).
TEST(SwathProgress, PassStartSeedsTotalAndZeroCompleted)
{
  BTContext ctx;
  constexpr std::size_t kUnits = 5;

  refreshSwathProgress(ctx, kArea0, kUnits);

  EXPECT_EQ(ctx.total_swaths, 5);
  EXPECT_EQ(ctx.completed_swaths, 0);
  EXPECT_GT(ctx.total_swaths, 0);  // the exact condition the GUI gates on
}

// As each unit completes (recorded in area_completed_swaths, as FollowStrip
// does before advance()), a refresh makes completed_swaths climb, and at the end
// completed == total (100 %). total never drifts from the unit count.
TEST(SwathProgress, CompletedClimbsPerSwathAndEqualsTotalAtEnd)
{
  BTContext ctx;
  constexpr std::size_t kUnits = 4;

  refreshSwathProgress(ctx, kArea0, kUnits);
  EXPECT_EQ(ctx.completed_swaths, 0);

  for (std::size_t i = 0; i < kUnits; ++i)
  {
    ctx.area_completed_swaths[kArea0].insert(i);
    refreshSwathProgress(ctx, kArea0, kUnits);
    EXPECT_EQ(ctx.total_swaths, static_cast<int>(kUnits));
    EXPECT_EQ(ctx.completed_swaths, static_cast<int>(i + 1));
  }

  EXPECT_EQ(ctx.completed_swaths, ctx.total_swaths);  // 100 % at pass end
}

// Resuming an area that already had some units mowed seeds completed > 0 from
// the start (not a spurious 0 %), and total is that area's unit count.
TEST(SwathProgress, ResumeSeedsPartialCompleted)
{
  BTContext ctx;
  ctx.area_completed_swaths[kArea0] = {0, 1};  // two units already mowed
  constexpr std::size_t kUnits = 5;

  refreshSwathProgress(ctx, kArea0, kUnits);

  EXPECT_EQ(ctx.total_swaths, 5);
  EXPECT_EQ(ctx.completed_swaths, 2);
}

// Moving to the next area resets the scalars to THAT area's totals rather than
// leaking the previous area's end-of-pass values (multi-area case).
TEST(SwathProgress, ResetsAcrossAreas)
{
  BTContext ctx;

  // Area 0 finishes: 3/3 units mowed.
  ctx.area_completed_swaths[kArea0] = {0, 1, 2};
  refreshSwathProgress(ctx, kArea0, 3);
  ASSERT_EQ(ctx.total_swaths, 3);
  ASSERT_EQ(ctx.completed_swaths, 3);

  // Area 1 starts fresh with a different unit count — scalars must reflect
  // area 1, not the stale 3/3 from area 0.
  refreshSwathProgress(ctx, kArea1, 6);
  EXPECT_EQ(ctx.total_swaths, 6);
  EXPECT_EQ(ctx.completed_swaths, 0);
}

// ---------------------------------------------------------------------------
// coveragePercentFromCursor — the smooth pose-cursor progress (Bug B follow-up)
// ---------------------------------------------------------------------------

// A fresh pass starts at 0 % and an empty plan never divides by zero.
TEST(CoveragePercent, ZeroAtStartAndForEmptyPlan)
{
  EXPECT_FLOAT_EQ(coveragePercentFromCursor(0, 1000), 0.0f);
  EXPECT_FLOAT_EQ(coveragePercentFromCursor(0, 0), 0.0f);  // no poses → 0, no div0
  EXPECT_FLOAT_EQ(coveragePercentFromCursor(500, 0), 0.0f);  // guard dominates
}

// Driving the concatenated cursor forward yields a monotonically non-decreasing
// percentage that reaches ~100 at the final pose.
TEST(CoveragePercent, ClimbsMonotonicallyToOneHundred)
{
  constexpr std::size_t kTotal = 1000;
  float prev = -1.0f;
  for (std::size_t cursor = 0; cursor <= kTotal; cursor += 50)
  {
    const float pct = coveragePercentFromCursor(cursor, kTotal);
    EXPECT_GE(pct, prev);  // never goes backwards as the robot advances
    EXPECT_GE(pct, 0.0f);
    EXPECT_LE(pct, 100.0f);
    prev = pct;
  }
  EXPECT_FLOAT_EQ(coveragePercentFromCursor(kTotal, kTotal), 100.0f);
  EXPECT_FLOAT_EQ(coveragePercentFromCursor(kTotal / 2, kTotal), 50.0f);
}

// A cursor at/over the end clamps to exactly 100 (FTC can park a pose or two
// past the last tracked index; the readout must not exceed 100 %).
TEST(CoveragePercent, ClampsAtOneHundred)
{
  EXPECT_FLOAT_EQ(coveragePercentFromCursor(1000, 1000), 100.0f);
  EXPECT_FLOAT_EQ(coveragePercentFromCursor(1005, 1000), 100.0f);
}
