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

// SPDX-License-Identifier: GPL-3.0

#include <cmath>
#include <limits>
#include <utility>

#include "mowgli_behavior/undock_resume.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::kMinResumeUndockM;
using mowgli_behavior::remainingUndockDistance;
using mowgli_behavior::undockHeadingFromMotion;

TEST(UndockResume, ReversesOnlyWhatIsLeftOfTheUndock)
{
  // 0.6 m of a 1.5 m undock done → 0.9 m left, whatever the direction.
  EXPECT_NEAR(remainingUndockDistance(1.0, 2.0, 1.0, 2.6, 1.5), 0.9, 1e-9);
  EXPECT_NEAR(remainingUndockDistance(1.0, 2.0, 1.36, 1.52, 1.5), 0.9, 1e-9);
}

TEST(UndockResume, NotMovedYetMeansTheWholeReverse)
{
  EXPECT_NEAR(remainingUndockDistance(1.0, 2.0, 1.0, 2.0, 1.5), 1.5, 1e-9);
}

TEST(UndockResume, NothingLeftOnceTheReverseIsDoneOrOvershot)
{
  EXPECT_EQ(remainingUndockDistance(0.0, 0.0, 1.5, 0.0, 1.5), 0.0);
  EXPECT_EQ(remainingUndockDistance(0.0, 0.0, 4.0, 0.0, 1.5), 0.0);
}

TEST(UndockResume, ARemainderWithinGnssNoiseCountsAsDone)
{
  EXPECT_EQ(remainingUndockDistance(0.0, 0.0, 1.5 - kMinResumeUndockM / 2.0, 0.0, 1.5), 0.0);
  EXPECT_GT(remainingUndockDistance(0.0, 0.0, 1.5 - 2.0 * kMinResumeUndockM, 0.0, 1.5), 0.0);
}

TEST(UndockResume, InvalidInputsNeverAskForAReverse)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(remainingUndockDistance(0.0, 0.0, nan, 0.0, 1.5), 0.0);
  EXPECT_EQ(remainingUndockDistance(0.0, 0.0, 0.0, 0.0, nan), 0.0);
  EXPECT_EQ(remainingUndockDistance(0.0, 0.0, 0.0, 0.0, 0.0), 0.0);
  EXPECT_EQ(remainingUndockDistance(0.0, 0.0, 0.0, 0.0, -1.0), 0.0);
}

TEST(UndockHeading, AReverseUndockFacesAwayFromItsMotion)
{
  // Reversed due east -> the chassis faces west.
  EXPECT_NEAR(std::fabs(undockHeadingFromMotion(0.0, false)), M_PI, 1e-12);
  // Reversed due north -> faces south.
  EXPECT_NEAR(undockHeadingFromMotion(M_PI / 2.0, false), -M_PI / 2.0, 1e-12);
  // Matches the endpoint formula atan2(-dy, -dx) the node used before.
  for (const auto& [dx, dy] : {std::pair{1.0, 2.0}, std::pair{-3.0, 0.5}, std::pair{-0.2, -1.1}})
    EXPECT_NEAR(undockHeadingFromMotion(std::atan2(dy, dx), false), std::atan2(-dy, -dx), 1e-12);
}

TEST(UndockHeading, AForwardUndockFacesAlongItsMotion)
{
  // undock_forward: a reverse-entry dock is left driving forward.
  EXPECT_NEAR(undockHeadingFromMotion(0.0, true), 0.0, 1e-12);
  EXPECT_NEAR(undockHeadingFromMotion(M_PI / 2.0, true), M_PI / 2.0, 1e-12);
  EXPECT_NEAR(undockHeadingFromMotion(-2.5, true), -2.5, 1e-12);
  // Always wrapped into [-pi, pi].
  EXPECT_NEAR(undockHeadingFromMotion(3.0 * M_PI / 2.0, true), -M_PI / 2.0, 1e-12);
}
