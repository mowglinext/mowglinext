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

#include <limits>

#include "mowgli_behavior/undock_resume.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::kMinResumeUndockM;
using mowgli_behavior::remainingUndockDistance;

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
