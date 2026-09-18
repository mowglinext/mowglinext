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
// Unit tests for the lateral-offset lattice planner (ftc_offset_lattice.hpp).
// ROS-free: obstacles are synthetic shapes in the path's Frenet frame
// (s along the path, d to the LEFT of it).

#include <cmath>
#include <cstddef>
#include <vector>

#include "gtest/gtest.h"
#include "mowgli_nav2_plugins/ftc_offset_lattice.hpp"

namespace
{

using mowgli_nav2_plugins::OffsetLatticeCfg;
using mowgli_nav2_plugins::OffsetLatticeStationSpacing;
using mowgli_nav2_plugins::PlanOffsetProfile;

constexpr double kBodyHalfWidth = 0.275;
constexpr double kBodyHalfLength = 0.35;

struct Box
{
  double s_min, s_max, d_min, d_max;
};

/// 3 m of path at the spacing the planner asks for (slope 1.0 -> 0.05 m).
std::vector<double> Stations(double length_m = 3.0)
{
  const double ds = OffsetLatticeStationSpacing(OffsetLatticeCfg{}.offset_step, 1.0);
  std::vector<double> s;
  for (double x = 0.0; x <= length_m + 1e-9; x += ds)
  {
    s.push_back(x);
  }
  return s;
}

/// Body (a box around the offset centre) against axis-aligned obstacle boxes.
auto BlockedBy(const std::vector<double>& stations, const std::vector<Box>& boxes)
{
  return [&stations, boxes](std::size_t i, double offset)
  {
    const double s = stations[i];
    for (const Box& b : boxes)
    {
      const bool overlap_s = s + kBodyHalfLength > b.s_min && s - kBodyHalfLength < b.s_max;
      const bool overlap_d = offset + kBodyHalfWidth > b.d_min && offset - kBodyHalfWidth < b.d_max;
      if (overlap_s && overlap_d)
      {
        return true;
      }
    }
    return false;
  };
}

TEST(FtcOffsetLattice, StaysOnTheLineWhenNothingIsInTheWay)
{
  const auto s = Stations();
  const auto r = PlanOffsetProfile(s, 0.0, 0, BlockedBy(s, {}));
  ASSERT_TRUE(r.feasible);
  EXPECT_DOUBLE_EQ(r.MaxAbsOffset(), 0.0);
  EXPECT_DOUBLE_EQ(r.cost, 0.0);
}

TEST(FtcOffsetLattice, GoesAroundATreeOnTheLineAndComesBack)
{
  // Arrange — 0.30 m trunk centred on the path at s = 1.5 m.
  const auto s = Stations();
  const std::vector<Box> tree{{1.35, 1.65, -0.15, 0.15}};

  // Act
  const auto r = PlanOffsetProfile(s, 0.0, 0, BlockedBy(s, tree));

  // Assert — skirts by just enough, and is back on the line at the horizon.
  ASSERT_TRUE(r.feasible);
  EXPECT_GE(r.MaxAbsOffset(), 0.15 + kBodyHalfWidth - 1e-9);
  EXPECT_LE(r.MaxAbsOffset(), 0.15 + kBodyHalfWidth + 0.05 + 1e-9)
      << "no more than one step of slack";
  EXPECT_DOUBLE_EQ(r.offsets.front(), 0.0);
  EXPECT_DOUBLE_EQ(r.offsets.back(), 0.0);
}

TEST(FtcOffsetLattice, ProfileNeverCollidesAndRespectsTheSlopeLimit)
{
  const auto s = Stations();
  const std::vector<Box> tree{{1.35, 1.65, -0.15, 0.15}};
  const auto blocked = BlockedBy(s, tree);
  const auto r = PlanOffsetProfile(s, 0.0, 0, blocked);
  ASSERT_TRUE(r.feasible);
  for (std::size_t i = 1; i < s.size(); ++i)
  {
    EXPECT_FALSE(blocked(i, r.offsets[i])) << "station " << i;
    EXPECT_LE(std::fabs(r.offsets[i] - r.offsets[i - 1]), OffsetLatticeCfg{}.offset_step + 1e-9);
  }
}

TEST(FtcOffsetLattice, PassesOnTheCheaperSideOfAnOffCentreObstacle)
{
  // Obstacle sits mostly to the LEFT: passing on the right needs less offset.
  const auto s = Stations();
  const std::vector<Box> obstacle{{1.35, 1.65, -0.05, 0.45}};
  const auto r = PlanOffsetProfile(s, 0.0, 0, BlockedBy(s, obstacle));
  ASSERT_TRUE(r.feasible);
  double extreme = 0.0;
  for (const double o : r.offsets)
  {
    extreme = std::fabs(o) > std::fabs(extreme) ? o : extreme;
  }
  EXPECT_LT(extreme, 0.0) << "should pass on the right (negative offset)";
}

TEST(FtcOffsetLattice, AvoidsThePocketTheNearestSideLeadsInto)
{
  // The right side is the smaller detour around the first obstacle, but a wall
  // further on closes it: a single-pose side test walks into the pocket, the
  // whole-profile search must go left.
  const auto s = Stations(4.0);
  const std::vector<Box> field{
      {1.00, 1.30, -0.05, 0.40},  // first obstacle, cheaper on the right
      {1.80, 2.10, -1.60, 0.10},  // wall closing the right-hand side
  };
  const auto r = PlanOffsetProfile(s, 0.0, 0, BlockedBy(s, field));
  ASSERT_TRUE(r.feasible);
  double extreme = 0.0;
  for (const double o : r.offsets)
  {
    extreme = std::fabs(o) > std::fabs(extreme) ? o : extreme;
  }
  EXPECT_GT(extreme, 0.0) << "must commit to the LEFT, the only side with an exit";
}

TEST(FtcOffsetLattice, HugsAHedgeRunningAlongTheLineInsteadOfGivingUp)
{
  // A hedge overhangs the rest of the window on the left by 0.10 m, from 0.5 m
  // ahead (seen at range, as the LiDAR does — not materialising under the body).
  const auto s = Stations();
  const std::vector<Box> hedge{{0.5, 10.0, kBodyHalfWidth - 0.10, 3.0}};
  const auto r = PlanOffsetProfile(s, 0.0, 0, BlockedBy(s, hedge));
  ASSERT_TRUE(r.feasible);
  EXPECT_NEAR(r.offsets.back(), -0.10, 1e-9) << "shift by exactly the overhang, no more";
  EXPECT_NEAR(r.MaxAbsOffset(), 0.10, 1e-9);
}

TEST(FtcOffsetLattice, ReportsInfeasibleWhenAWallSpansTheWholeLattice)
{
  const auto s = Stations();
  const std::vector<Box> wall{{1.4, 1.6, -5.0, 5.0}};
  EXPECT_FALSE(PlanOffsetProfile(s, 0.0, 0, BlockedBy(s, wall)).feasible);
}

TEST(FtcOffsetLattice, KeepsTheCommittedSideWhenTheOtherIsOnlyMarginallyCheaper)
{
  // Slightly cheaper on the right (-), but the previous plan committed LEFT.
  const auto s = Stations();
  const std::vector<Box> obstacle{{1.35, 1.65, -0.15, 0.20}};
  const auto blocked = BlockedBy(s, obstacle);
  const auto free_choice = PlanOffsetProfile(s, 0.0, 0, blocked);
  const auto committed = PlanOffsetProfile(s, 0.0, +1, blocked);
  ASSERT_TRUE(free_choice.feasible);
  ASSERT_TRUE(committed.feasible);
  double free_extreme = 0.0, committed_extreme = 0.0;
  for (std::size_t i = 0; i < s.size(); ++i)
  {
    free_extreme = std::fabs(free_choice.offsets[i]) > std::fabs(free_extreme)
                       ? free_choice.offsets[i]
                       : free_extreme;
    committed_extreme = std::fabs(committed.offsets[i]) > std::fabs(committed_extreme)
                            ? committed.offsets[i]
                            : committed_extreme;
  }
  EXPECT_LT(free_extreme, 0.0);
  EXPECT_GT(committed_extreme, 0.0) << "the soft latch must hold against a marginal gain";
}

TEST(FtcOffsetLattice, SwitchesSideWhenTheCommittedOneBecomesBlocked)
{
  const auto s = Stations();
  const std::vector<Box> field{{1.35, 1.65, -0.15, 5.0}};  // left fully closed
  const auto r = PlanOffsetProfile(s, 0.0, +1, BlockedBy(s, field));
  ASSERT_TRUE(r.feasible);
  EXPECT_LT(r.offsets[s.size() / 2], 0.0);
}

TEST(FtcOffsetLattice, StartsFromTheOffsetAlreadyAppliedAndBlendsBack)
{
  const auto s = Stations();
  const auto r = PlanOffsetProfile(s, 0.30, +1, BlockedBy(s, {}));
  ASSERT_TRUE(r.feasible);
  EXPECT_NEAR(r.offsets.front(), 0.30, 1e-9);
  EXPECT_NEAR(r.offsets.back(), 0.0, 1e-9);
  for (std::size_t i = 1; i < s.size(); ++i)
  {
    EXPECT_LE(r.offsets[i], r.offsets[i - 1] + 1e-9) << "monotonic return, no wobble";
  }
}

TEST(FtcOffsetLattice, StationZeroIsNeverCollisionChecked)
{
  // The robot is where it is: a lethal cell under the carrot's current offset
  // must not make the whole problem infeasible.
  const auto s = Stations();
  const auto blocked = [](std::size_t i, double)
  {
    return i == 0;
  };
  EXPECT_TRUE(PlanOffsetProfile(s, 0.0, 0, blocked).feasible);
}

TEST(FtcOffsetLattice, DegenerateInputsAreSafe)
{
  const auto never = [](std::size_t, double)
  {
    return false;
  };
  EXPECT_FALSE(PlanOffsetProfile({}, 0.0, 0, never).feasible);
  OffsetLatticeCfg bad;
  bad.offset_step = 0.0;
  EXPECT_FALSE(PlanOffsetProfile({0.0, 0.1}, 0.0, 0, never, bad).feasible);
  EXPECT_TRUE(PlanOffsetProfile({0.0}, 0.2, 0, never).feasible);
}

}  // namespace
