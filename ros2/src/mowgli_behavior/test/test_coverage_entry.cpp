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
#include <limits>

#include "mowgli_behavior/coverage_entry.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::coverageEntryReached;

TEST(CoverageEntry, RequiresPositionAndHeadingTogether)
{
  EXPECT_TRUE(coverageEntryReached(1, 2, 0, 1, 2, 0, 0));
  EXPECT_TRUE(coverageEntryReached(0.029, 0, 0.299, 0, 0, 0, 0.29));
  EXPECT_FALSE(coverageEntryReached(0.28554, 0, 0, 0, 0, 0, 0));
  EXPECT_FALSE(coverageEntryReached(0.031, 0, 0, 0, 0, 0, 0));
  EXPECT_FALSE(coverageEntryReached(0, 0, 0.301, 0, 0, 0, 0));
  EXPECT_FALSE(coverageEntryReached(0.025, 0.025, 0, 0, 0, 0, 0));
  EXPECT_TRUE(coverageEntryReached(0, 0, 3.14, 0, 0, -3.14, 0));
}

TEST(CoverageEntry, MissingFreshnessAndNonFiniteValuesCannotAuthorizeEntry)
{
  EXPECT_FALSE(coverageEntryReached(0, 0, 0, 0, 0, 0, 0.301));
  EXPECT_FALSE(coverageEntryReached(0, 0, 0, 0, 0, 0, -0.01));
  for (double invalid :
       {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    EXPECT_FALSE(coverageEntryReached(invalid, 0, 0, 0, 0, 0, 0));
    EXPECT_FALSE(coverageEntryReached(0, invalid, 0, 0, 0, 0, 0));
    EXPECT_FALSE(coverageEntryReached(0, 0, invalid, 0, 0, 0, 0));
    EXPECT_FALSE(coverageEntryReached(0, 0, 0, invalid, 0, 0, 0));
    EXPECT_FALSE(coverageEntryReached(0, 0, 0, 0, invalid, 0, 0));
    EXPECT_FALSE(coverageEntryReached(0, 0, 0, 0, 0, invalid, 0));
    EXPECT_FALSE(coverageEntryReached(0, 0, 0, 0, 0, 0, invalid));
  }
}
