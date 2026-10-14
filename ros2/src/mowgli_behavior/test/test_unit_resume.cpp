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
//
// Unit tests for the same-unit resume decision (unit_resume.hpp). ROS-free.

#include <set>

#include "gtest/gtest.h"
#include "mowgli_behavior/unit_resume.hpp"

namespace
{

using mowgli_behavior::DecideUnitResume;
using mowgli_behavior::nextUndoneUnit;
using mowgli_behavior::UnitResumeCfg;

TEST(UnitResume, ResumesTheSerpentineUnitCancelledNearItsStart)
{
  // Field 2026-09-17: 30 127-pose unit cancelled at pose 280, cursor stepped to 308.
  const auto d = DecideUnitResume(30127, 280, 308, 0);
  EXPECT_TRUE(d.resume);
  EXPECT_EQ(d.consecutive, 0u + 1u)
      << "280 poses is real progress: counter cleared, then one spent";
}

TEST(UnitResume, GivesUpAfterConsecutiveResumesWithoutProgress)
{
  const UnitResumeCfg cfg;
  std::size_t spent = 0;
  for (std::size_t i = 0; i < cfg.max_consecutive; ++i)
  {
    const auto d = DecideUnitResume(5000, 3, 30, spent, cfg);
    ASSERT_TRUE(d.resume) << "resume " << i;
    spent = d.consecutive;
  }
  EXPECT_FALSE(DecideUnitResume(5000, 3, 30, spent, cfg).resume)
      << "a deterministic re-abort loop must end";
}

TEST(UnitResume, RealProgressClearsTheCounter)
{
  const UnitResumeCfg cfg;
  const auto d = DecideUnitResume(5000, cfg.progress_reset_poses, 200, cfg.max_consecutive, cfg);
  EXPECT_TRUE(d.resume);
  EXPECT_EQ(d.consecutive, 1u);
}

TEST(UnitResume, ATinyRemainderIsNotWorthATransit)
{
  EXPECT_FALSE(DecideUnitResume(100, 70, 90, 0).resume);
}

TEST(UnitResume, CursorAtOrPastTheEndNeverResumes)
{
  EXPECT_FALSE(DecideUnitResume(100, 99, 100, 0).resume);
  EXPECT_FALSE(DecideUnitResume(100, 99, 250, 0).resume);
}

// The first unit a pass drives is the first one NOT recorded as mowed — also the rule
// PlanCoverageArea uses for its first transit (field 2026-10-12: it hopped to unit 7 and
// FollowStrip sent the robot back to unit 1).
TEST(NextUndoneUnit, SkipsOnlyUnitsRecordedAsMowed)
{
  EXPECT_EQ(nextUndoneUnit(0, 9, {}), 0u);
  EXPECT_EQ(nextUndoneUnit(0, 9, {0, 1}), 2u);
  EXPECT_EQ(nextUndoneUnit(0, 9, {3}), 0u) << "a later unit being done does not move the start";
  EXPECT_EQ(nextUndoneUnit(3, 9, {3, 4}), 5u);
}

TEST(NextUndoneUnit, ReturnsTheCountWhenNothingIsLeft)
{
  EXPECT_EQ(nextUndoneUnit(0, 3, {0, 1, 2}), 3u);
  EXPECT_EQ(nextUndoneUnit(3, 3, {}), 3u);
  EXPECT_EQ(nextUndoneUnit(0, 0, {}), 0u);
}

}  // namespace
