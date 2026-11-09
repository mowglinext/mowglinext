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

// GetNextUnmowedArea's no-progress budget (area_progress.hpp): a pass that got
// far into a long unfinished unit is progress, a pass that did not is not.

#include <map>
#include <optional>

#include "mowgli_behavior/area_progress.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::areaPassMadeProgress;
using mowgli_behavior::AreaProgressMark;
using mowgli_behavior::kAreaProgressMinPartialPoses;
using mowgli_behavior::minPartialProgressPoses;
using mowgli_behavior::partialUnitPoses;

TEST(AreaProgress, FirstDispatchAlwaysCounts)
{
  EXPECT_TRUE(areaPassMadeProgress(std::nullopt, AreaProgressMark{}, 100));
}

TEST(AreaProgress, AFinishedUnitAlwaysCounts)
{
  // Its partial entry is erased when the unit is booked mowed, so the sum drops.
  const AreaProgressMark last{1, 5000};
  EXPECT_TRUE(areaPassMadeProgress(last, AreaProgressMark{2, 0}, 100));
}

TEST(AreaProgress, ALongStretchInsideAnUnfinishedUnitCounts)
{
  // A large lawn drawn with few points: one ring unit + one swath unit. The
  // pass mowed 3000 poses (~90 m) of the swath unit and then stopped.
  const AreaProgressMark last{1, 0};
  EXPECT_TRUE(areaPassMadeProgress(last, AreaProgressMark{1, 3000}, 100));
}

TEST(AreaProgress, NoOrTooLittleGainDoesNotCount)
{
  const AreaProgressMark last{1, 3000};
  EXPECT_FALSE(areaPassMadeProgress(last, AreaProgressMark{1, 3000}, 100));
  EXPECT_FALSE(areaPassMadeProgress(last, AreaProgressMark{1, 3099}, 100));
  EXPECT_TRUE(areaPassMadeProgress(last, AreaProgressMark{1, 3100}, 100));
}

TEST(AreaProgress, ALostRecordIsNotProgress)
{
  // Fewer finished units than last time (the area's record was reset): no
  // credit, whatever the partial poses say — the rule from before.
  const AreaProgressMark last{3, 0};
  EXPECT_FALSE(areaPassMadeProgress(last, AreaProgressMark{2, 90000}, 100));
}

TEST(AreaProgress, PartialPosesSumEveryUnfinishedUnit)
{
  EXPECT_EQ(partialUnitPoses(nullptr), 0u);
  const std::map<std::size_t, std::size_t> units{{0u, 1200u}, {3u, 40u}};
  EXPECT_EQ(partialUnitPoses(&units), 1240u);
}

TEST(AreaProgress, TheMinimumGainScalesWithThePlanAndHasAFloor)
{
  // Small plan: the ~1 m floor.
  EXPECT_EQ(minPartialProgressPoses(0, 0.5f), kAreaProgressMinPartialPoses);
  EXPECT_EQ(minPartialProgressPoses(2000, 0.5f), kAreaProgressMinPartialPoses);
  // 12 km of path at 0.03 m: 0.5 % = 2000 poses (~60 m), so at most ~200
  // credited passes — a few metres a pass cannot keep the area alive forever.
  EXPECT_EQ(minPartialProgressPoses(400000, 0.5f), 2000u);
}
