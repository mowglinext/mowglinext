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
 * @file test_dock_stall.cpp
 * @brief The dock approach must be called blocked when it stands still, and only then.
 *
 * Field 2026-10-06: the mower pushed against the garage edge over the dock for 30 s,
 * 0.29 m short of the dock pose, until the approach timeout. See dock_stall.hpp.
 */

#include <cmath>

#include "mowgli_behavior/dock_stall.hpp"
#include <gtest/gtest.h>

using mowgli_behavior::DockStallDetector;
using mowgli_behavior::kDockStallMinMoveM;
using mowgli_behavior::kDockStallMinTurnRad;
using mowgli_behavior::kDockStallWindowS;

namespace
{
constexpr double kHz = 10.0;
}

TEST(DockStall, AnApproachThatKeepsMovingIsNeverStalled)
{
  DockStallDetector d;
  // 0.16 m/s (the graceful controller's slowest approach) for 20 s.
  for (int i = 0; i < 200; ++i)
  {
    const double t = i / kHz;
    EXPECT_FALSE(d.Update(t, 0.16 * t, 0.0, 0.0)) << "t=" << t;
  }
}

TEST(DockStall, StandingStillForTheWholeWindowIsAStall)
{
  DockStallDetector d;
  bool stalled = false;
  double stalled_at = -1.0;
  for (int i = 0; i < 100 && !stalled; ++i)
  {
    const double t = i / kHz;
    // Sub-centimetre jitter, as the fused pose has even when parked.
    stalled = d.Update(t, 0.43 + 0.004 * std::sin(t * 7.0), -0.47, 2.17);
    if (stalled)
    {
      stalled_at = t;
    }
  }
  EXPECT_TRUE(stalled);
  EXPECT_NEAR(stalled_at, kDockStallWindowS, 0.11);
}

TEST(DockStall, AMovingApproachThatThenHitsSomethingStallsOneWindowLater)
{
  DockStallDetector d;
  // Drives 2 s, then stops dead (the garage edge).
  for (int i = 0; i < 20; ++i)
  {
    EXPECT_FALSE(d.Update(i / kHz, 0.16 * (i / kHz), 0.0, 0.0));
  }
  const double x_stop = 0.16 * (19 / kHz);
  bool stalled = false;
  double stalled_at = 0.0;
  for (int i = 20; i < 120 && !stalled; ++i)
  {
    stalled = d.Update(i / kHz, x_stop, 0.0, 0.0);
    stalled_at = i / kHz;
  }
  ASSERT_TRUE(stalled);
  // The window starts at the LAST movement, not at the start of the approach.
  EXPECT_GT(stalled_at, 1.9 + kDockStallWindowS - 0.3);
  EXPECT_LT(stalled_at, 1.9 + kDockStallWindowS + 0.5);
}

TEST(DockStall, AnInPlaceAlignmentTurnIsNotAStall)
{
  DockStallDetector d;
  // The approach starts by rotating on the spot at ~0.75 rad/s for 8 s: no travel at all.
  for (int i = 0; i < 80; ++i)
  {
    const double t = i / kHz;
    EXPECT_FALSE(d.Update(t, 1.0, 1.0, 0.75 * t)) << "t=" << t;
  }
}

TEST(DockStall, ASlowCrawlJustAboveTheThresholdIsNotAStall)
{
  DockStallDetector d;
  // 0.02 m/s: 4 s of that is 8 cm, well over the 3 cm that counts as movement.
  for (int i = 0; i < 100; ++i)
  {
    const double t = i / kHz;
    EXPECT_FALSE(d.Update(t, 0.02 * t, 0.0, 0.0)) << "t=" << t;
  }
}

TEST(DockStall, ResetForgetsTheWindow)
{
  DockStallDetector d;
  for (int i = 0; i < 39; ++i)
  {
    EXPECT_FALSE(d.Update(i / kHz, 0.0, 0.0, 0.0));
  }
  d.Reset();  // e.g. the server left CONTROLLING for WAIT_FOR_CHARGE and came back
  // The old, almost-complete window must not carry over.
  for (int i = 40; i < 79; ++i)
  {
    EXPECT_FALSE(d.Update(i / kHz, 0.0, 0.0, 0.0)) << "i=" << i;
  }
  EXPECT_TRUE(d.Update(8.5, 0.0, 0.0, 0.0));
}

TEST(DockStall, TheTurnThresholdWrapsAcrossPlusMinusPi)
{
  DockStallDetector d;
  // 179.5 deg -> -179.5 deg is a 1 deg turn through +-pi, not a 359 deg one.
  EXPECT_FALSE(d.Update(0.0, 0.0, 0.0, 3.133));
  EXPECT_FALSE(d.Update(1.0, 0.0, 0.0, -3.133));
  EXPECT_FALSE(d.Update(2.0, 0.0, 0.0, 3.133));
  EXPECT_TRUE(d.Update(kDockStallWindowS + 0.1, 0.0, 0.0, -3.133));
}

TEST(DockStall, TheThresholdsAreTheDocumentedOnes)
{
  EXPECT_NEAR(kDockStallWindowS, 4.0, 1e-9);
  EXPECT_NEAR(kDockStallMinMoveM, 0.03, 1e-9);
  EXPECT_NEAR(kDockStallMinTurnRad, 0.10, 1e-9);
}
