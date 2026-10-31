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
// Regression guards for coverage transits. Distance protects a far first unit;
// every later planner-produced sub-path requires a blade-off transit even when
// its first pose is nearby, because that boundary represents an intentional
// heading/obstacle discontinuity.

#include <cmath>

#include "mowgli_behavior/coverage_nodes.hpp"
#include "mowgli_interfaces/coverage_geometry.hpp"
#include <gtest/gtest.h>

TEST(CoverageTransitGap, ExecutionSideMatchesSharedConstant)
{
  EXPECT_DOUBLE_EQ(mowgli_behavior::FollowStrip::kSegmentTransitGap,
                   mowgli_interfaces::coverage_geometry::kSegmentTransitGapM);
}

TEST(CoverageTransitGap, ValueIsPinnedAndSane)
{
  // Pin the actual value (0.6 m) so a change here is a deliberate, reviewed
  // edit rather than an accidental one — and sanity-bound it: a gap this
  // large already relies on the connector-arc / relocation split described
  // in coverage_geometry.hpp, and adjacent swaths are ~one operation_width
  // (commonly ~0.16-0.20 m) apart, so the threshold must comfortably clear
  // that spacing without approaching a size that would swallow real
  // relocations as if they were ordinary turn-arounds.
  EXPECT_DOUBLE_EQ(mowgli_interfaces::coverage_geometry::kSegmentTransitGapM, 0.6);
  EXPECT_GT(mowgli_interfaces::coverage_geometry::kSegmentTransitGapM, 0.0);
  EXPECT_LT(mowgli_interfaces::coverage_geometry::kSegmentTransitGapM, 2.0);
}

TEST(CoverageTransitGap, EveryLaterSubPathTransitsBladeOff)
{
  constexpr double kAdjacentSwathGap = 0.16;
  EXPECT_FALSE(mowgli_behavior::coverageTransitRequired(kAdjacentSwathGap, false));
  EXPECT_TRUE(mowgli_behavior::coverageTransitRequired(kAdjacentSwathGap, true));
  EXPECT_TRUE(mowgli_behavior::coverageTransitRequired(0.0, true));
}

// 2026-09-10: a first unit that will be reached by a blade-off transit must not
// have the blade spun up on start — the START_OCCUPIED retry loop cycled the
// blade on/off on every pass. Only a directly-mowable first unit spins up.
TEST(CoverageTransitGap, BladeSpinsUpOnStartOnlyForADirectlyMowableFirstUnit)
{
  EXPECT_TRUE(mowgli_behavior::bladeSpinupBeforeFirstUnit(0.0));
  EXPECT_TRUE(mowgli_behavior::bladeSpinupBeforeFirstUnit(0.16));  // adjacent swath
  EXPECT_FALSE(mowgli_behavior::bladeSpinupBeforeFirstUnit(0.61));
  EXPECT_FALSE(mowgli_behavior::bladeSpinupBeforeFirstUnit(13.6));
}

// 2026-09-12: a 0.40 m sub-path-boundary transit spun on the spot for 164 s
// because nothing bounded it. The bound must be generous (Smac detours are
// much longer than the gap) but finite.
TEST(CoverageTransitGap, TransitDeadlineIsFiniteAndGenerous)
{
  EXPECT_DOUBLE_EQ(mowgli_behavior::transitDeadlineSec(0.40), 20.0);  // floor
  EXPECT_DOUBLE_EQ(mowgli_behavior::transitDeadlineSec(-1.0), 20.0);
  EXPECT_NEAR(mowgli_behavior::transitDeadlineSec(13.6), 151.0, 1e-9);
  EXPECT_LT(mowgli_behavior::transitDeadlineSec(0.40), 164.0);
}

// The transit planner only crosses the recorded areas, so the planned route can
// be far longer than the straight-line gap. The bound follows the route nav2
// reported — and only ever widens: a shorter route never shrinks the gap bound.
TEST(CoverageTransitGap, TransitDeadlineFollowsThePlannedRoute)
{
  using mowgli_behavior::transitDeadlineForRouteSec;
  using mowgli_behavior::transitDeadlineSec;
  // A 5 m gap that is a 60 m drive round the end of a long area edge: the gap
  // alone gave 65 s, i.e. 13 m at transit speed, and cancelled the detour.
  EXPECT_DOUBLE_EQ(transitDeadlineSec(5.0), 65.0);
  EXPECT_DOUBLE_EQ(transitDeadlineForRouteSec(5.0, 60.0), 615.0);
  // No route reported yet, or one shorter than the gap: the gap bound stands.
  EXPECT_DOUBLE_EQ(transitDeadlineForRouteSec(5.0, 0.0), transitDeadlineSec(5.0));
  EXPECT_DOUBLE_EQ(transitDeadlineForRouteSec(5.0, 3.0), transitDeadlineSec(5.0));
  // The 2026-09-12 spin case keeps its 20 s floor: its route is the 0.40 m gap.
  EXPECT_DOUBLE_EQ(transitDeadlineForRouteSec(0.40, 0.42), 20.0);
  // Garbage feedback can neither disable nor run away with the watchdog.
  EXPECT_DOUBLE_EQ(transitDeadlineForRouteSec(5.0, std::nan("")), transitDeadlineSec(5.0));
  EXPECT_DOUBLE_EQ(transitDeadlineForRouteSec(5.0, -10.0), transitDeadlineSec(5.0));
  EXPECT_DOUBLE_EQ(transitDeadlineForRouteSec(5.0, 1e9),
                   transitDeadlineSec(mowgli_behavior::kTransitRouteMaxM));
}

TEST(CoverageTransitGap, TransitRouteSlotKeepsTheLongestRoute)
{
  mowgli_behavior::TransitRouteSlot slot;
  EXPECT_DOUBLE_EQ(slot.Longest(), 0.0);
  slot.Record(42.0);  // first plan
  slot.Record(30.0);  // the robot drives along it: remaining shrinks
  EXPECT_DOUBLE_EQ(slot.Longest(), 42.0);
  slot.Record(55.0);  // a replan round something new is longer
  slot.Record(std::nan(""));
  EXPECT_DOUBLE_EQ(slot.Longest(), 55.0);
}

TEST(CoverageTransitGap, DistantFirstUnitStillTransits)
{
  EXPECT_TRUE(mowgli_behavior::coverageTransitRequired(0.61, false));
}
