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
// Which swath angle and perimeter winding an area is planned with: the area's
// own override when it has one, the robot-wide setting otherwise — and a corrupt
// override never becomes a plan the operator did not ask for.

#include <limits>

#include "mowgli_behavior/area_coverage_lines.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::CoverageLineChoice;
using mowgli_behavior::ResolveCoverageLines;

namespace
{

mowgli_interfaces::msg::MapArea area(bool has_angle, double angle, bool has_dir, uint8_t dir)
{
  mowgli_interfaces::msg::MapArea a;
  a.has_mow_angle = has_angle;
  a.mow_angle_deg = angle;
  a.has_ring_direction = has_dir;
  a.ring_direction = dir;
  return a;
}

}  // namespace

TEST(AreaCoverageLinesChoice, AnAreaWithoutOverridesPlansWithTheRobotWideSettings)
{
  // The default MapArea — what every area saved before the fields existed is.
  const CoverageLineChoice c = ResolveCoverageLines(mowgli_interfaces::msg::MapArea{}, 37.0);
  EXPECT_DOUBLE_EQ(c.mow_angle_deg, 37.0);
  EXPECT_FALSE(c.override_ring_direction)
      << "must leave the winding to coverage_server's parameter";
}

TEST(AreaCoverageLinesChoice, AutoStaysAutoWhenTheAreaDoesNotOverride)
{
  EXPECT_DOUBLE_EQ(ResolveCoverageLines(mowgli_interfaces::msg::MapArea{}, -1.0).mow_angle_deg,
                   -1.0);
}

TEST(AreaCoverageLinesChoice, AnAreaAngleBeatsTheRobotWideOneIncludingAuto)
{
  EXPECT_DOUBLE_EQ(ResolveCoverageLines(area(true, 80.0, false, 0), 37.0).mow_angle_deg, 80.0);
  EXPECT_DOUBLE_EQ(ResolveCoverageLines(area(true, 80.0, false, 0), -1.0).mow_angle_deg, 80.0);
}

TEST(AreaCoverageLinesChoice, ZeroDegreesIsAnOverrideNotMissing)
{
  EXPECT_DOUBLE_EQ(ResolveCoverageLines(area(true, 0.0, false, 0), 37.0).mow_angle_deg, 0.0);
}

TEST(AreaCoverageLinesChoice, AnUnsetAngleIgnoresTheStoredNumber)
{
  EXPECT_DOUBLE_EQ(ResolveCoverageLines(area(false, 80.0, false, 0), 37.0).mow_angle_deg, 37.0);
}

TEST(AreaCoverageLinesChoice, AnAreaWindingIsAnExplicitOverride)
{
  for (int dir = 0; dir <= 2; ++dir)
  {
    const auto c = ResolveCoverageLines(area(false, 0.0, true, static_cast<uint8_t>(dir)), -1.0);
    EXPECT_TRUE(c.override_ring_direction) << "dir " << dir;
    EXPECT_EQ(c.ring_direction, dir);
  }
}

TEST(AreaCoverageLinesChoice, PlannerDefaultWindingIsAnOverrideToo)
{
  // 0 means "planner default", which differs from "use the robot-wide setting".
  const auto c = ResolveCoverageLines(area(false, 0.0, true, 0), -1.0);
  EXPECT_TRUE(c.override_ring_direction);
  EXPECT_EQ(c.ring_direction, 0);
}

TEST(AreaCoverageLinesChoice, AnUnsetWindingIgnoresTheStoredNumber)
{
  const auto c = ResolveCoverageLines(area(false, 0.0, false, 2), -1.0);
  EXPECT_FALSE(c.override_ring_direction);
}

TEST(AreaCoverageLinesChoice, ANegativeAreaAngleIsAnAutoOverride)
{
  // Pins one area to auto while the robot-wide angle is fixed.
  EXPECT_DOUBLE_EQ(ResolveCoverageLines(area(true, -1.0, false, 0), 37.0).mow_angle_deg, -1.0);
  EXPECT_DOUBLE_EQ(ResolveCoverageLines(area(true, -45.0, false, 0), 37.0).mow_angle_deg, -1.0);
  // Without the flag a negative stored value is just ignored.
  EXPECT_DOUBLE_EQ(ResolveCoverageLines(area(false, -1.0, false, 0), 37.0).mow_angle_deg, 37.0);
}

TEST(AreaCoverageLinesChoice, CorruptAnglesFallBackToTheRobotWideSetting)
{
  EXPECT_DOUBLE_EQ(ResolveCoverageLines(area(true, 180.0, false, 0), 37.0).mow_angle_deg, 37.0);
  EXPECT_DOUBLE_EQ(ResolveCoverageLines(area(true, 400.0, false, 0), 37.0).mow_angle_deg, 37.0);
  EXPECT_DOUBLE_EQ(
      ResolveCoverageLines(area(true, std::numeric_limits<double>::quiet_NaN(), false, 0), 37.0)
          .mow_angle_deg,
      37.0);
  EXPECT_DOUBLE_EQ(
      ResolveCoverageLines(area(true, std::numeric_limits<double>::infinity(), false, 0), 37.0)
          .mow_angle_deg,
      37.0);
}

TEST(AreaCoverageLinesChoice, ACorruptWindingFallsBackToTheLiveParameter)
{
  EXPECT_FALSE(ResolveCoverageLines(area(false, 0.0, true, 3), -1.0).override_ring_direction);
  EXPECT_FALSE(ResolveCoverageLines(area(false, 0.0, true, 255), -1.0).override_ring_direction);
}

TEST(AreaCoverageLinesChoice, OneCorruptValueDoesNotSpoilTheOther)
{
  const auto c = ResolveCoverageLines(area(true, 999.0, true, 1), 37.0);
  EXPECT_DOUBLE_EQ(c.mow_angle_deg, 37.0);
  EXPECT_TRUE(c.override_ring_direction);
  EXPECT_EQ(c.ring_direction, 1);
}

TEST(AreaCoverageLinesChoice, AnAreaWithoutAStartPointLetsThePlannerStartWhereItAlwaysDid)
{
  const auto c = ResolveCoverageLines(mowgli_interfaces::msg::MapArea{}, 37.0);
  EXPECT_FALSE(c.has_start_point);
  EXPECT_EQ(c.start_x, 0.0);
  EXPECT_EQ(c.start_y, 0.0);
}

TEST(AreaCoverageLinesChoice, AnAreasStartPointIsPassedThrough)
{
  auto a = area(false, 0.0, false, 0);
  a.has_start_point = true;
  a.start_x = 12.5;
  a.start_y = -3.0;
  const auto c = ResolveCoverageLines(a, -1.0);
  EXPECT_TRUE(c.has_start_point);
  EXPECT_DOUBLE_EQ(c.start_x, 12.5);
  EXPECT_DOUBLE_EQ(c.start_y, -3.0);
}

TEST(AreaCoverageLinesChoice, ZeroZeroIsARealStartPoint)
{
  auto a = area(false, 0.0, false, 0);
  a.has_start_point = true;
  EXPECT_TRUE(ResolveCoverageLines(a, -1.0).has_start_point);
}

TEST(AreaCoverageLinesChoice, AStoredStartWithoutItsFlagMeansNothing)
{
  auto a = area(false, 0.0, false, 0);
  a.start_x = 5.0;
  a.start_y = 6.0;
  EXPECT_FALSE(ResolveCoverageLines(a, -1.0).has_start_point);
}

TEST(AreaCoverageLinesChoice, ANonFiniteStartPointIsDropped)
{
  auto a = area(false, 0.0, false, 0);
  a.has_start_point = true;
  a.start_x = std::numeric_limits<double>::quiet_NaN();
  a.start_y = 1.0;
  EXPECT_FALSE(ResolveCoverageLines(a, -1.0).has_start_point);
  a.start_x = 1.0;
  a.start_y = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ResolveCoverageLines(a, -1.0).has_start_point);
}

TEST(AreaCoverageLinesChoice, TheStartPointIsIndependentOfTheAngleAndWinding)
{
  auto a = area(true, 80.0, true, 2);
  a.has_start_point = true;
  a.start_x = 1.0;
  a.start_y = 2.0;
  const auto c = ResolveCoverageLines(a, 37.0);
  EXPECT_DOUBLE_EQ(c.mow_angle_deg, 80.0);
  EXPECT_EQ(c.ring_direction, 2);
  EXPECT_TRUE(c.has_start_point);
}
