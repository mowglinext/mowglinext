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

#include <vector>

#include "mowgli_behavior/blocked_stretch.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::FailedTransitTarget;
using mowgli_behavior::resumePastBlockedStretch;

namespace
{

geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  return p;
}

// Straight line along +x, one pose every `step` metres.
std::vector<geometry_msgs::msg::PoseStamped> line(std::size_t n, double step = 0.05)
{
  std::vector<geometry_msgs::msg::PoseStamped> out;
  for (std::size_t i = 0; i < n; ++i)
  {
    out.push_back(pose(static_cast<double>(i) * step, 0.0));
  }
  return out;
}

}  // namespace

TEST(BlockedStretch, ResumesAtMinimumSkipWhenNothingIsBlocked)
{
  const auto poses = line(200);  // 9.95 m

  const auto idx = resumePastBlockedStretch(poses, 0, {}, 2.0, 0.6);

  ASSERT_TRUE(idx.has_value());
  EXPECT_NEAR(poses[*idx].pose.position.x, 2.0, 0.05 + 1e-9);
}

TEST(BlockedStretch, SkipsPosesNearARecordedFailedTarget)
{
  const auto poses = line(200);
  // A stuck point 2.2 m along: its 0.6 m disc covers 1.6 .. 2.8 m.
  const std::vector<FailedTransitTarget> blocked{{0.0, 0.0}, {2.2, 0.0}};

  const auto idx = resumePastBlockedStretch(poses, 0, blocked, 2.0, 0.6);

  ASSERT_TRUE(idx.has_value());
  EXPECT_GT(poses[*idx].pose.position.x, 2.8);
  EXPECT_LT(poses[*idx].pose.position.x, 2.9);
}

TEST(BlockedStretch, KeepsTheRestOfALongUnitInsteadOfDroppingIt)
{
  // The 2026-10-10 shape: a hairpin at the start of a long unit whose later
  // rings pass the same spot. Only the pinch is skipped; the rest stays.
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  for (int i = 0; i <= 40; ++i)  // out along y = 0 for 2 m
  {
    poses.push_back(pose(i * 0.05, 0.0));
  }
  for (int i = 40; i >= 0; --i)  // back along y = 0.16 (the next ring)
  {
    poses.push_back(pose(i * 0.05, 0.16));
  }
  for (int i = 1; i <= 400; ++i)  // then 20 m away from the pinch
  {
    poses.push_back(pose(-i * 0.05, 0.16));
  }
  const std::vector<FailedTransitTarget> blocked{{2.0, 0.0}};

  const auto idx = resumePastBlockedStretch(poses, 0, blocked, 2.0, 0.6);

  ASSERT_TRUE(idx.has_value());
  // The return leg next to the stuck point is skipped, nothing more.
  EXPECT_LT(poses[*idx].pose.position.x, 1.45);
  EXPECT_GT(poses[*idx].pose.position.x, 1.30);
  EXPECT_LT(*idx, poses.size() / 4);
}

TEST(BlockedStretch, ReturnsNulloptWhenTheWholeRemainderIsBlocked)
{
  const auto poses = line(60);  // 2.95 m
  const std::vector<FailedTransitTarget> blocked{{2.5, 0.0}};

  EXPECT_FALSE(resumePastBlockedStretch(poses, 0, blocked, 2.0, 0.6).has_value());
}

TEST(BlockedStretch, ReturnsNulloptWhenTheUnitIsShorterThanTheSkip)
{
  const auto poses = line(20);  // 0.95 m

  EXPECT_FALSE(resumePastBlockedStretch(poses, 0, {}, 2.0, 0.6).has_value());
  EXPECT_FALSE(resumePastBlockedStretch({}, 0, {}, 2.0, 0.6).has_value());
}

TEST(BlockedStretch, MeasuresTheSkipFromTheGivenStartPose)
{
  const auto poses = line(200);

  const auto idx = resumePastBlockedStretch(poses, 100, {}, 2.0, 0.6);

  ASSERT_TRUE(idx.has_value());
  EXPECT_EQ(*idx, 140u);
}
