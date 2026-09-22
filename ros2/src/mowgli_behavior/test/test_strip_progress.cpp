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
 * @file test_strip_progress.cpp
 * @brief FollowStrip's progress cursor must not jump to the neighbouring swath.
 *
 * Field 2026-09-22: a nearest-pose search over the next 400 poses put the
 * cursor on the ADJACENT serpentine swath (0.13 m away) whenever the robot was
 * more than half a spacing off its line — every obstacle skirt. The cursor is
 * monotonic, so it never came back: units were booked "reached 100 % → MOWED"
 * at ~80 % of FTC's own index and ~20 m² of lawn was never driven.
 */

#include <cmath>
#include <cstddef>
#include <vector>

#include "mowgli_behavior/strip_progress.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::advanceProgressCursor;
using mowgli_behavior::kMaxProgressAdvanceM;
using Poses = std::vector<geometry_msgs::msg::PoseStamped>;

namespace
{

constexpr double kSpacing = 0.13;  // shipped operation width
constexpr double kStep = 0.03;  // coverage pose spacing
constexpr double kSwathLen = 4.0;

void append(Poses& path, double x, double y)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1.0;
  path.push_back(p);
}

// `count` swaths along x, `kSpacing` apart in y, alternating direction, joined
// by pivot joins (a straight 0.13 m step between swath ends), like #716 plans.
Poses serpentine(int count)
{
  Poses path;
  const int n = static_cast<int>(std::round(kSwathLen / kStep));
  for (int s = 0; s < count; ++s)
  {
    const double y = s * kSpacing;
    for (int i = 0; i <= n; ++i)
    {
      const double x = (s % 2 == 0) ? i * kStep : kSwathLen - i * kStep;
      append(path, x, y);
    }
  }
  return path;
}

double yAt(const Poses& path, std::size_t i)
{
  return path[i].pose.position.y;
}

}  // namespace

TEST(StripProgress, AnOffsetRobotStaysOnItsOwnSwath)
{
  // Arrange: halfway along swath 0, skirting an obstacle 0.08 m toward swath 1
  // — nearer to swath 1's poses (0.05 m) than to its own (0.08 m).
  const Poses path = serpentine(4);
  const std::size_t mid = static_cast<std::size_t>(std::round(2.0 / kStep));
  const double rx = 2.0;
  const double ry = 0.08;

  // Act
  const std::size_t cursor = advanceProgressCursor(path, mid - 5, rx, ry);

  // Assert
  EXPECT_DOUBLE_EQ(yAt(path, cursor), 0.0) << "the cursor jumped onto the neighbouring swath";
  EXPECT_NEAR(path[cursor].pose.position.x, rx, kStep);
}

TEST(StripProgress, TheOldPoseCountWindowDidJump)
{
  // The pre-fix behaviour, for the record: a 400-pose (~12 m) window reaches the
  // pose of swath 1 beside the robot and takes it.
  const Poses path = serpentine(4);
  const std::size_t mid = static_cast<std::size_t>(std::round(2.0 / kStep));
  const std::size_t cursor = advanceProgressCursor(path, mid - 5, 2.0, 0.08, 400 * kStep);
  EXPECT_DOUBLE_EQ(yAt(path, cursor), kSpacing);
}

TEST(StripProgress, FollowsTheRobotAlongItsSwathAndRoundTheTurn)
{
  // Arrange: drive the path 0.07 m off to the swath-1 side of each swath (worse
  // than any real skirt that stays within one spacing), updating every 0.03 m.
  const Poses path = serpentine(3);
  std::size_t cursor = 0;
  std::size_t previous = 0;

  // Act + Assert: the cursor tracks the pose the robot is driving past, never
  // skipping ahead onto a later swath and never going back.
  for (std::size_t i = 0; i < path.size(); i += 1)
  {
    const double side = (yAt(path, i) < kSpacing * 1.5) ? +0.05 : -0.05;
    const double rx = path[i].pose.position.x;
    const double ry = yAt(path, i) + side;
    cursor = advanceProgressCursor(path, cursor, rx, ry);
    ASSERT_GE(cursor, previous) << "the cursor moved backwards at pose " << i;
    ASSERT_LE(cursor, i + static_cast<std::size_t>(std::ceil(kMaxProgressAdvanceM / kStep)) + 1)
        << "the cursor ran ahead of the robot at pose " << i;
    previous = cursor;
  }
  EXPECT_GE(cursor + 10, path.size()) << "the cursor did not reach the end of the unit";
}

TEST(StripProgress, NeverMovesBackwardsOrPastTheEnd)
{
  const Poses path = serpentine(2);
  const std::size_t last = path.size() - 1;
  EXPECT_EQ(advanceProgressCursor(path, 50, 0.0, 0.0), 50u);  // robot behind the cursor
  EXPECT_EQ(advanceProgressCursor(path, last + 20, 0.0, 0.0), last);
  EXPECT_EQ(advanceProgressCursor(Poses{}, 7, 0.0, 0.0), 7u);
}

// --- FTC turn fallback rejoins (mowgli_nav2_plugins/ftc_turn_fallback.hpp) ---

using mowgli_behavior::findControllerRejoin;

namespace
{

// Swath A along +x, a U-turn of radius r to the left, swath B back along -x,
// headings along the path (a pose's orientation is part of what a rejoin
// matches).
Poses uTurn(double length, double r)
{
  Poses path;
  const auto add = [&path](double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped p;
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.z = std::sin(yaw / 2.0);
    p.pose.orientation.w = std::cos(yaw / 2.0);
    path.push_back(p);
  };
  for (double x = 0.0; x < length - 1e-9; x += kStep)
  {
    add(x, 0.0, 0.0);
  }
  const int n = static_cast<int>(std::ceil(M_PI * r / kStep));
  for (int k = 0; k <= n; ++k)
  {
    const double a = -M_PI / 2.0 + M_PI * k / n;
    add(length + r * std::cos(a), r + r * std::sin(a), a + M_PI / 2.0);
  }
  for (double x = length - kStep; x >= -1e-9; x -= kStep)
  {
    add(x, 2.0 * r, M_PI);
  }
  return path;
}

std::size_t nearestIndex(const Poses& path, double x, double y)
{
  std::size_t best = 0;
  for (std::size_t i = 0; i < path.size(); ++i)
  {
    if (std::hypot(path[i].pose.position.x - x, path[i].pose.position.y - y) <
        std::hypot(path[best].pose.position.x - x, path[best].pose.position.y - y))
    {
      best = i;
    }
  }
  return best;
}

}  // namespace

TEST(StripProgress, ABoundedCursorCannotFollowATurnFallbackRejoin)
{
  // Why a rejoin needs its own signal. The robot was at x = 2.4 on swath A (the
  // cursor there); the fallback skipped the row end and the U-turn and rejoined
  // swath B abeam, 0.13 m away, then drives B to its end. The cursor never gets
  // past its own pose on A, which stays nearer to the robot than anything
  // within kMaxProgressAdvanceM ahead of it.
  const Poses path = uTurn(kSwathLen, kSpacing / 2.0);
  std::size_t cursor = nearestIndex(path, 2.4, 0.0);
  const std::size_t stuck = cursor;
  for (double x = 2.4; x >= 0.0; x -= kStep)
  {
    cursor = advanceProgressCursor(path, cursor, x, kSpacing);
  }
  EXPECT_LT(cursor, stuck + 3) << "the bounded search did follow the rejoin after all";
}

TEST(StripProgress, AControllerRejoinIsFoundOnlyAsTheExactPoseAhead)
{
  const Poses path = uTurn(kSwathLen, kSpacing / 2.0);
  const std::size_t cursor = nearestIndex(path, 2.4, 0.0);
  const std::size_t rejoin = nearestIndex(path, 2.4, kSpacing);
  ASSERT_GT(rejoin, cursor);

  // The exact pose FTC republished from: found.
  const auto found = findControllerRejoin(path, cursor, path[rejoin].pose);
  ASSERT_TRUE(found.has_value());
  EXPECT_EQ(*found, rejoin);

  // A pose merely NEAR one of the unit's — the return swath's position with
  // swath A's heading, or 1 mm off — is never taken for it.
  auto near = path[rejoin].pose;
  near.orientation = path[cursor].pose.orientation;
  EXPECT_FALSE(findControllerRejoin(path, cursor, near).has_value());
  near = path[rejoin].pose;
  near.position.x += 1e-3;
  EXPECT_FALSE(findControllerRejoin(path, cursor, near).has_value());

  // Never backwards, never further than the bound.
  EXPECT_FALSE(findControllerRejoin(path, rejoin, path[cursor].pose).has_value());
  EXPECT_FALSE(findControllerRejoin(path, cursor, path[rejoin].pose, 0.5).has_value());
}
