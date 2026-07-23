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
// Regression guard for resolveResumeLocation — the SHARED resume-cursor mapping
// used by both FollowStrip (trim the driven prefix, mark driven units done) and
// PlanCoverageArea (aim the blade-off transit at the resume point). If the two
// ever computed the resume point differently, the robot would transit to one
// place and then immediately re-transit to another ("arrive, wait for spin-up,
// then drive ~10 m off elsewhere"). This test pins the mapping so that failure
// mode cannot silently reopen.

#include "mowgli_behavior/coverage_nodes.hpp"

#include <gtest/gtest.h>

#include <vector>

#include "nav_msgs/msg/path.hpp"

namespace
{

// Build a sub-path with `n` poses at x=0,1,2,... (positions don't matter for the
// mapping — only the counts do).
nav_msgs::msg::Path makeUnit(std::size_t n)
{
  nav_msgs::msg::Path p;
  p.poses.resize(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    p.poses[i].pose.position.x = static_cast<double>(i);
  }
  return p;
}

std::size_t totalPoses(const std::vector<nav_msgs::msg::Path>& units)
{
  std::size_t t = 0;
  for (const auto& u : units)
  {
    t += u.poses.size();
  }
  return t;
}

}  // namespace

using mowgli_behavior::resolveResumeLocation;

// A cursor of 0 (never interrupted) is not resumable — mow fresh from pose 0.
TEST(ResolveResumeLocation, ZeroCursorIsFreshStart)
{
  const std::vector<nav_msgs::msg::Path> units{makeUnit(100)};
  const auto rl = resolveResumeLocation(units, 0, totalPoses(units));
  EXPECT_FALSE(rl.valid);
}

// A cursor within 2 poses of the very end means the path is effectively done —
// not worth resuming.
TEST(ResolveResumeLocation, NearEndCursorIsFreshStart)
{
  const std::vector<nav_msgs::msg::Path> units{makeUnit(100)};
  EXPECT_FALSE(resolveResumeLocation(units, 98, 100).valid);  // 98 + 2 >= 100
  EXPECT_FALSE(resolveResumeLocation(units, 99, 100).valid);
}

// The reproduction from the field logs: one big first sub-path, cursor at 216 →
// resume mid-unit 0 at local offset 216 (this is the pose TransitToStrip must aim
// at so FollowStrip doesn't re-transit).
TEST(ResolveResumeLocation, MidFirstUnitTrims)
{
  const std::vector<nav_msgs::msg::Path> units{makeUnit(13118)};
  const auto rl = resolveResumeLocation(units, 216, 13118);
  ASSERT_TRUE(rl.valid);
  EXPECT_EQ(rl.unit, 0u);
  EXPECT_EQ(rl.local, 216u);
}

// A cursor that lands inside a LATER sub-path: earlier units are fully driven
// (marked done by FollowStrip) and the landing unit is trimmed at the local
// offset.
TEST(ResolveResumeLocation, LandsInLaterUnit)
{
  const std::vector<nav_msgs::msg::Path> units{makeUnit(100), makeUnit(100), makeUnit(100)};
  const auto rl = resolveResumeLocation(units, 250, 300);
  ASSERT_TRUE(rl.valid);
  EXPECT_EQ(rl.unit, 2u);
  EXPECT_EQ(rl.local, 50u);
}

// A cursor exactly on a unit boundary resumes at the NEXT unit's front (local 0),
// not a mid-unit trim.
TEST(ResolveResumeLocation, UnitBoundarySnapsToFront)
{
  const std::vector<nav_msgs::msg::Path> units{makeUnit(100), makeUnit(100)};
  const auto rl = resolveResumeLocation(units, 100, 200);
  ASSERT_TRUE(rl.valid);
  EXPECT_EQ(rl.unit, 1u);
  EXPECT_EQ(rl.local, 0u);  // front of unit 1, no trim
}

// A near-boundary landing (within 2 poses of a unit's end) snaps to the front
// rather than leaving a 1-2 pose stub.
TEST(ResolveResumeLocation, NearUnitEndSnapsToFront)
{
  const std::vector<nav_msgs::msg::Path> units{makeUnit(100), makeUnit(100)};
  const auto rl = resolveResumeLocation(units, 99, 200);  // local 99, 99 + 2 >= 100
  ASSERT_TRUE(rl.valid);
  EXPECT_EQ(rl.unit, 0u);
  EXPECT_EQ(rl.local, 0u);
}

// A cursor past the end of the concatenation (stale / mismatched plan) is not
// resumable.
TEST(ResolveResumeLocation, CursorPastEndIsFreshStart)
{
  const std::vector<nav_msgs::msg::Path> units{makeUnit(100)};
  EXPECT_FALSE(resolveResumeLocation(units, 500, 100).valid);
}
