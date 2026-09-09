// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <limits>

#include "fusion_graph/clear_graph_gate.hpp"
#include <gtest/gtest.h>

namespace fusion_graph
{

TEST(ClearGraphGate, RequiresKnownIdleState)
{
  EXPECT_FALSE(ClearGraphAllowed(false, true, 0.0, 0.0));
  EXPECT_FALSE(ClearGraphAllowed(true, false, 0.0, 0.0));
  EXPECT_TRUE(ClearGraphAllowed(true, true, 0.0, 0.0));
}

TEST(ClearGraphGate, RejectsMovingOrInvalidVelocity)
{
  EXPECT_TRUE(
      ClearGraphAllowed(true, true, kClearGraphMaxLinearSpeedMps, kClearGraphMaxAngularSpeedRadps));
  EXPECT_FALSE(ClearGraphAllowed(true, true, kClearGraphMaxLinearSpeedMps + 0.001, 0.0));
  EXPECT_FALSE(ClearGraphAllowed(true, true, 0.0, kClearGraphMaxAngularSpeedRadps + 0.001));
  EXPECT_FALSE(ClearGraphAllowed(true, true, std::numeric_limits<double>::quiet_NaN(), 0.0));
}

}  // namespace fusion_graph
