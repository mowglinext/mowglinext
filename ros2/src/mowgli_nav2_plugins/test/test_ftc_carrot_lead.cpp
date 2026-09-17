// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the FTC carrot lead cap (ftc_carrot_lead.hpp). ROS-free.

#include "gtest/gtest.h"
#include "mowgli_nav2_plugins/ftc_carrot_lead.hpp"

namespace
{

using mowgli_nav2_plugins::CarrotLeadExceeded;
using mowgli_nav2_plugins::CarrotMaxLead;
using mowgli_nav2_plugins::kCarrotLeadFloorM;
using mowgli_nav2_plugins::kCarrotLeadHeadroom;

constexpr double kEps = 1e-9;

TEST(FtcCarrotLead, DerivesTheCapFromCruiseSpeedAndLongitudinalGain)
{
  // Shipped tuning: 0.20 m/s, kp_lon 1.0 -> steady lead 0.20 m, cap 0.30 m.
  EXPECT_NEAR(CarrotMaxLead(-1.0, 0.20, 1.0), kCarrotLeadHeadroom * 0.20, kEps);
}

TEST(FtcCarrotLead, DerivedCapStaysAboveTheSteadyStateLead)
{
  // Otherwise the robot could never reach cruise speed.
  for (const double kp : {0.5, 1.0, 2.0})
  {
    for (const double v : {0.10, 0.20, 0.30})
    {
      EXPECT_GT(CarrotMaxLead(0.0, v, kp), v / kp - kEps) << "v=" << v << " kp=" << kp;
    }
  }
}

TEST(FtcCarrotLead, DerivedCapIsFarBelowTheOldOneMetreLiteral)
{
  // The 2026-09-17 runaway reached 0.95 m under a 1.0 m cap.
  EXPECT_LT(CarrotMaxLead(-1.0, 0.20, 1.0), 0.5);
}

TEST(FtcCarrotLead, OperatorOverrideWins)
{
  EXPECT_NEAR(CarrotMaxLead(0.45, 0.20, 1.0), 0.45, kEps);
}

TEST(FtcCarrotLead, DegenerateGainOrSpeedFallsBackToTheFloorNotInfinity)
{
  EXPECT_NEAR(CarrotMaxLead(-1.0, 0.20, 0.0), kCarrotLeadFloorM, kEps);
  EXPECT_NEAR(CarrotMaxLead(-1.0, 0.0, 1.0), kCarrotLeadFloorM, kEps);
}

TEST(FtcCarrotLead, HighGainCannotPushTheCapUnderTheFloor)
{
  EXPECT_NEAR(CarrotMaxLead(-1.0, 0.20, 50.0), kCarrotLeadFloorM, kEps);
}

TEST(FtcCarrotLead, FreezesOnlyBeyondTheCap)
{
  EXPECT_FALSE(CarrotLeadExceeded(0.20, 0.30));
  EXPECT_FALSE(CarrotLeadExceeded(0.30, 0.30));
  EXPECT_TRUE(CarrotLeadExceeded(0.31, 0.30));
}

TEST(FtcCarrotLead, ACarrotBehindTheRobotNeverFreezes)
{
  // Negative x: the robot overshot; the carrot must be free to move on.
  EXPECT_FALSE(CarrotLeadExceeded(-0.40, 0.30));
}

}  // namespace
