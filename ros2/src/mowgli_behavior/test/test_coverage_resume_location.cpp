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

#include "geometry_msgs/msg/pose_stamped.hpp"
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

// ---------------------------------------------------------------------------
// forwardSkipIndex — the #389 anti-stall step. On a non-obstacle FTC abort the
// resume cursor is stepped this far ahead so a re-dispatch does not land on the
// identical abort pose and re-abort forever. Poses are laid out at 0.1 m spacing
// on the x-axis (makeUnit sets x=i), so arc-length == 0.1 * pose count.
// ---------------------------------------------------------------------------

namespace
{
// makeUnit spaces poses 1.0 m apart; a finer spacing exercises the arc-length
// accumulation with more than one step per skip.
std::vector<geometry_msgs::msg::PoseStamped> makePoses(std::size_t n, double spacing)
{
  std::vector<geometry_msgs::msg::PoseStamped> v(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    v[i].pose.position.x = static_cast<double>(i) * spacing;
  }
  return v;
}
}  // namespace

using mowgli_behavior::forwardSkipIndex;

// The core reproduction: resuming at pose `from` and re-aborting there must move
// the cursor strictly forward, by at least the requested arc-length.
TEST(ForwardSkipIndex, StepsPastAbortByRequestedArcLength)
{
  const auto poses = makePoses(1000, 0.1);  // 0.1 m spacing
  // 0.8 m skip at 0.1 m spacing = 8 poses forward.
  EXPECT_EQ(forwardSkipIndex(poses, 100, 0.8), 108u);
  EXPECT_EQ(forwardSkipIndex(poses, 0, 0.8), 8u);
}

// Every call advances the index (breaks the deterministic re-abort loop).
TEST(ForwardSkipIndex, AlwaysAdvancesWhenRoomRemains)
{
  const auto poses = makePoses(1000, 0.1);
  EXPECT_GT(forwardSkipIndex(poses, 500, 0.8), 500u);
}

// When less than the skip distance remains, snap to the last pose (never past the
// end) so the resume cursor maps to the unit end / next unit, not out of range.
TEST(ForwardSkipIndex, SnapsToEndWhenRemainderShorterThanSkip)
{
  const auto poses = makePoses(1000, 0.1);
  EXPECT_EQ(forwardSkipIndex(poses, 995, 0.8), 999u);  // only 0.4 m left
  EXPECT_EQ(forwardSkipIndex(poses, 999, 0.8), 999u);  // already last pose
}

// Degenerate inputs return `from` unchanged (caller then persists the abort pose
// as before — no crash, no spurious skip).
TEST(ForwardSkipIndex, DegenerateInputsReturnFrom)
{
  const auto poses = makePoses(1000, 0.1);
  EXPECT_EQ(forwardSkipIndex(poses, 42, 0.0), 42u);    // non-positive skip
  EXPECT_EQ(forwardSkipIndex(poses, 42, -1.0), 42u);   // negative skip
  EXPECT_EQ(forwardSkipIndex(makePoses(1, 0.1), 0, 0.8), 0u);  // too short
  const std::vector<geometry_msgs::msg::PoseStamped> empty;
  EXPECT_EQ(forwardSkipIndex(empty, 0, 0.8), 0u);      // empty
}
