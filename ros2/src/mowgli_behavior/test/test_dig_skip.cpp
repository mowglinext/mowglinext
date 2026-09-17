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
 * @file test_dig_skip.cpp
 * @brief Session dig skip zones (dig_skip.hpp): which coverage poses are
 *        skipped around a recorded dig point, and when the bridge's reverse is
 *        considered finished.
 *
 * Field 2026-09-17: a dig at (-3.23, 11.01) was stamped as a pending keepout
 * 0.27 m from the robot; every transit was refused with START_OCCUPIED and the
 * mission died mid-lawn. The keepout is gone; this is what keeps the robot out
 * of the hole instead (issue #500: 3 dig latches in 18.4 s inside 0.13 m).
 */

#include <cmath>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_behavior/dig_skip.hpp"
#include <gtest/gtest.h>

namespace
{

using mowgli_behavior::DigPoint;
using mowgli_behavior::DigSettleCfg;
using mowgli_behavior::DigSettleState;
using mowgli_behavior::DigSettleStep;
using mowgli_behavior::DrivableRun;
using mowgli_behavior::insideDigZone;
using mowgli_behavior::nextDrivableRun;
using mowgli_behavior::recordDigPoint;

constexpr double kRadius = 0.60;
constexpr double kStep = 0.05;

/// Straight path along +x at height y, from x0 to x1, one pose every kStep.
std::vector<geometry_msgs::msg::PoseStamped> straightPath(double x0, double x1, double y = 0.0)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  const int n = static_cast<int>(std::round((x1 - x0) / kStep));
  for (int i = 0; i <= n; ++i)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.pose.position.x = x0 + kStep * i;
    p.pose.position.y = y;
    poses.push_back(p);
  }
  return poses;
}

double xAt(const std::vector<geometry_msgs::msg::PoseStamped>& poses, std::size_t i)
{
  return poses[i].pose.position.x;
}

}  // namespace

// ── insideDigZone ───────────────────────────────────────────────────────────

TEST(DigSkipZone, PoseWithinTheRadiusOfADigPointIsInside)
{
  const std::vector<DigPoint> digs{{-3.23, 11.01}};

  EXPECT_TRUE(insideDigZone(-3.23, 11.01, digs, kRadius));
  EXPECT_TRUE(insideDigZone(-3.23 + 0.59, 11.01, digs, kRadius));
  EXPECT_FALSE(insideDigZone(-3.23 + 0.61, 11.01, digs, kRadius));
}

TEST(DigSkipZone, NoDigPointsOrANonPositiveRadiusDisablesTheZones)
{
  EXPECT_FALSE(insideDigZone(0.0, 0.0, {}, kRadius));
  EXPECT_FALSE(insideDigZone(0.0, 0.0, {{0.0, 0.0}}, 0.0));
  EXPECT_FALSE(insideDigZone(0.0, 0.0, {{0.0, 0.0}}, -1.0));
}

// ── recordDigPoint ──────────────────────────────────────────────────────────

TEST(DigSkipRecord, RepeatDigAtTheSameHoleIsNotStoredTwice)
{
  // Issue #500: three latches inside 0.13 m are ONE hole.
  std::vector<DigPoint> digs;

  digs = recordDigPoint(digs, {1.00, 1.00});
  digs = recordDigPoint(digs, {1.05, 1.02});
  digs = recordDigPoint(digs, {1.00, 0.95});

  EXPECT_EQ(digs.size(), 1u);
}

TEST(DigSkipRecord, DistinctHolesAreAllKeptAndTheInputIsNotMutated)
{
  const std::vector<DigPoint> before{{0.0, 0.0}};

  const auto after = recordDigPoint(before, {3.0, 0.0});

  EXPECT_EQ(before.size(), 1u);
  ASSERT_EQ(after.size(), 2u);
  EXPECT_DOUBLE_EQ(after[1].x, 3.0);
}

TEST(DigSkipRecord, OldestPointIsDroppedOnceTheBoundIsReached)
{
  std::vector<DigPoint> digs;
  for (int i = 0; i < 5; ++i)
  {
    digs = recordDigPoint(digs, {static_cast<double>(i), 0.0}, 0.10, /*max_points=*/3);
  }

  ASSERT_EQ(digs.size(), 3u);
  EXPECT_DOUBLE_EQ(digs.front().x, 2.0);
  EXPECT_DOUBLE_EQ(digs.back().x, 4.0);
}

// ── nextDrivableRun ─────────────────────────────────────────────────────────

TEST(DigSkipRun, WithoutDigPointsTheWholeRemainderIsOneRun)
{
  const auto poses = straightPath(0.0, 10.0);

  const DrivableRun run = nextDrivableRun(poses, 7, {}, kRadius);

  EXPECT_EQ(run.start, 7u);
  EXPECT_EQ(run.end, poses.size());
}

TEST(DigSkipRun, RunStopsBeforeTheZoneOfADigAheadOnTheSwath)
{
  // Later unit of the session: the dig at x=5 was recorded earlier.
  const auto poses = straightPath(0.0, 10.0);
  const std::vector<DigPoint> digs{{5.0, 0.0}};

  const DrivableRun run = nextDrivableRun(poses, 0, digs, kRadius);

  EXPECT_EQ(run.start, 0u);
  ASSERT_LT(run.end, poses.size());
  ASSERT_GT(run.end, 0u);
  EXPECT_FALSE(insideDigZone(xAt(poses, run.end - 1), 0.0, digs, kRadius))
      << "the last mowed pose is outside the zone";
  EXPECT_TRUE(insideDigZone(xAt(poses, run.end), 0.0, digs, kRadius))
      << "the cut is the first pose inside";
  EXPECT_NEAR(xAt(poses, run.end), 5.0 - kRadius, kStep + 1e-6);
}

TEST(DigSkipRun, ResumeAfterADigStartsAtTheFirstPosePastTheZone)
{
  // The 2026-09-17 case: FTC reached the dig point, the bridge reversed the
  // robot 0.27 m. The resume must not be where the robot stands, nor the hole:
  // it is the first pose farther than the radius PAST the dig.
  const auto poses = straightPath(0.0, 10.0);
  const std::vector<DigPoint> digs{{3.2, 0.0}};
  const std::size_t reached = 60;  // x = 3.0, where the cursor froze

  const DrivableRun run = nextDrivableRun(poses, reached, digs, kRadius);

  ASSERT_LT(run.start, poses.size());
  ASSERT_GT(run.start, reached);
  EXPECT_FALSE(insideDigZone(xAt(poses, run.start), 0.0, digs, kRadius));
  EXPECT_TRUE(insideDigZone(xAt(poses, run.start - 1), 0.0, digs, kRadius))
      << "skip no more than necessary";
  EXPECT_NEAR(xAt(poses, run.start), 3.2 + kRadius, kStep + 1e-6) << "PAST the dig, not before";
  EXPECT_EQ(run.end, poses.size());
}

TEST(DigSkipRun, AdjacentSwathInsideTheRadiusIsSkippedTooButAFarOneIsNot)
{
  const std::vector<DigPoint> digs{{5.0, 0.0}};
  const auto neighbour = straightPath(0.0, 10.0, /*y=*/0.32);  // two swaths over
  const auto far_swath = straightPath(0.0, 10.0, /*y=*/0.80);

  const DrivableRun near_run = nextDrivableRun(neighbour, 0, digs, kRadius);
  const DrivableRun far_run = nextDrivableRun(far_swath, 0, digs, kRadius);

  EXPECT_LT(near_run.end, neighbour.size()) << "a swath 0.32 m from the hole still crosses it";
  EXPECT_EQ(far_run.end, far_swath.size()) << "a swath beyond the radius is mowed in full";
}

TEST(DigSkipRun, ShortRunBetweenTwoZonesIsNotWorthATransit)
{
  // Two holes 1.6 m apart leave 0.4 m of path between their zones: shorter
  // than kDigMinRunLengthM, so it is skipped with them.
  const auto poses = straightPath(0.0, 10.0);
  const std::vector<DigPoint> digs{{4.0, 0.0}, {5.6, 0.0}};
  const std::size_t from = 70;  // x = 3.5, inside the first zone

  const DrivableRun run = nextDrivableRun(poses, from, digs, kRadius);

  ASSERT_LT(run.start, poses.size());
  EXPECT_GT(xAt(poses, run.start), 5.6 + kRadius);
}

TEST(DigSkipRun, NothingDrivableLeftWhenTheRemainderLiesInsideAZone)
{
  const auto poses = straightPath(0.0, 5.0);
  const std::vector<DigPoint> digs{{4.8, 0.0}};
  const std::size_t from = 85;  // x = 4.25, already inside

  const DrivableRun run = nextDrivableRun(poses, from, digs, kRadius);

  EXPECT_TRUE(run.empty());
  EXPECT_EQ(run.start, poses.size());
}

TEST(DigSkipRun, SubMetreTailPastTheZoneIsDropped)
{
  const auto poses = straightPath(0.0, 5.0);
  const std::vector<DigPoint> digs{{4.0, 0.0}};  // zone ends at 4.6, tail = 0.4 m

  const DrivableRun run = nextDrivableRun(poses, 70, digs, kRadius);

  EXPECT_TRUE(run.empty());
}

TEST(DigSkipRun, FromPastTheEndYieldsAnEmptyRun)
{
  const auto poses = straightPath(0.0, 1.0);

  EXPECT_TRUE(nextDrivableRun(poses, poses.size(), {}, kRadius).empty());
  EXPECT_TRUE(nextDrivableRun(poses, poses.size() + 5, {{0.0, 0.0}}, kRadius).empty());
}

TEST(DigSkipRun, SparseLegacyPathIsJudgedByArcLengthNotPoseCount)
{
  // The joined-raw-segments fallback has two poses per 20 m segment. A pose
  // COUNT floor would discard the whole unit.
  std::vector<geometry_msgs::msg::PoseStamped> poses(2);
  poses[1].pose.position.x = 20.0;

  const DrivableRun run = nextDrivableRun(poses, 0, {{50.0, 50.0}}, kRadius);

  EXPECT_EQ(run.start, 0u);
  EXPECT_EQ(run.end, 2u);
}

// ── DigSettleStep ───────────────────────────────────────────────────────────

TEST(DigSettle, NotSettledWhileTheBridgeIsStillReversing)
{
  DigSettleState st;
  const DigSettleCfg cfg;
  double x = 0.0;

  bool settled = false;
  for (int i = 0; i < 25 && !settled; ++i)  // 2.5 s at 0.12 m/s
  {
    x -= 0.012;
    settled = DigSettleStep(st, cfg, 0.1, true, x, 0.0);
  }

  EXPECT_FALSE(settled);
}

TEST(DigSettle, SettledOnceTheRobotHasBeenStillForTheWindow)
{
  DigSettleState st;
  const DigSettleCfg cfg;
  double x = 0.0;
  for (int i = 0; i < 25; ++i)
  {
    x -= 0.012;
    ASSERT_FALSE(DigSettleStep(st, cfg, 0.1, true, x, 0.0));
  }

  int ticks = 0;
  while (!DigSettleStep(st, cfg, 0.1, true, x, 0.0))
  {
    ASSERT_LT(++ticks, 100);
  }

  EXPECT_NEAR(ticks * 0.1, cfg.still_window_s, 0.15);
}

TEST(DigSettle, AnInstantStillReadingRightAfterTheHardStopIsNotTheEndOfTheReverse)
{
  DigSettleState st;
  DigSettleCfg cfg;
  cfg.still_window_s = 0.2;  // even with a tiny window...

  bool settled = false;
  for (int i = 0; i < 5; ++i)  // 0.5 s < min_wait_s
  {
    settled = DigSettleStep(st, cfg, 0.1, true, 1.0, 1.0);
  }

  EXPECT_FALSE(settled) << "min_wait_s must hold the re-dispatch back";
}

TEST(DigSettle, HardBoundEndsTheWaitWithoutAnyPose)
{
  DigSettleState st;
  const DigSettleCfg cfg;

  int ticks = 0;
  while (!DigSettleStep(st, cfg, 0.1, /*have_pose=*/false, 0.0, 0.0))
  {
    ASSERT_LT(++ticks, 1000);
  }

  EXPECT_NEAR(ticks * 0.1, cfg.max_wait_s, 0.2);
}
