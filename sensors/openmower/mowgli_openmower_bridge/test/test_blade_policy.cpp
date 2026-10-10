// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_blade_policy.cpp
 * @brief Every gate that keeps the mow motor (and the wheels) from spinning
 *        when the firmware on a Mowgli board would have refused.
 */

#include "mowgli_openmower_bridge/blade_policy.hpp"
#include <gtest/gtest.h>

using namespace mowgli_openmower_bridge;  // NOLINT — test readability

namespace
{
BladeInputs Nominal()
{
  BladeInputs in;
  in.requested = true;
  in.mowing_enabled = true;
  in.emergency = false;
  in.ll_link_alive = true;
  in.hl_mode = HL_MODE_AUTONOMOUS;
  in.hl_status_fresh = true;
  in.update_maintenance = false;
  return in;
}
}  // namespace

TEST(BladePolicy, RunsOnlyWhenEveryGateOpens)
{
  EXPECT_TRUE(BladeMayRun(Nominal()));
  auto in = Nominal();
  in.hl_mode = HL_MODE_MANUAL_MOWING;
  EXPECT_TRUE(BladeMayRun(in));
}

TEST(BladePolicy, EachClosedGateStopsTheBlade)
{
  auto in = Nominal();
  in.requested = false;
  EXPECT_FALSE(BladeMayRun(in));
  in = Nominal();
  in.mowing_enabled = false;
  EXPECT_FALSE(BladeMayRun(in));
  in = Nominal();
  in.emergency = true;
  EXPECT_FALSE(BladeMayRun(in));
  in = Nominal();
  in.ll_link_alive = false;
  EXPECT_FALSE(BladeMayRun(in));
  in = Nominal();
  in.update_maintenance = true;
  EXPECT_FALSE(BladeMayRun(in));
  // A dead behavior_tree_node must not leave the blade running on its last word.
  in = Nominal();
  in.hl_status_fresh = false;
  EXPECT_FALSE(BladeMayRun(in));
  for (const uint8_t mode : {HL_MODE_NULL, HL_MODE_IDLE, HL_MODE_RECORDING})
  {
    in = Nominal();
    in.hl_mode = mode;
    EXPECT_FALSE(BladeMayRun(in)) << "mode " << static_cast<int>(mode);
  }
}

TEST(WheelPolicy, IdleAndEmergencyHoldTheWheels)
{
  EXPECT_TRUE(WheelsMayRun(false, true, HL_MODE_AUTONOMOUS, true));
  EXPECT_TRUE(WheelsMayRun(false, true, HL_MODE_RECORDING, true));
  EXPECT_TRUE(WheelsMayRun(false, true, HL_MODE_NULL, true));
  EXPECT_FALSE(WheelsMayRun(true, true, HL_MODE_AUTONOMOUS, true));
  EXPECT_FALSE(WheelsMayRun(false, false, HL_MODE_AUTONOMOUS, true));
  EXPECT_FALSE(WheelsMayRun(false, true, HL_MODE_IDLE, true));
  EXPECT_FALSE(WheelsMayRun(false, true, HL_MODE_AUTONOMOUS, false));
}
