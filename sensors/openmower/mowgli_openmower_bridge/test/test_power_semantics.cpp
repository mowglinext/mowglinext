// SPDX-License-Identifier: GPL-3.0
/**
 * @file test_power_semantics.cpp
 * @brief "On the dock" must come from the charge-contact voltage, never from
 *        the LowLevel charging bit (the charge relay). Rows taken from the
 *        v1 firmware's updateChargingEnabled() on tag v0.13.2 and branch v1-fw.
 */

#include <cmath>
#include <string>

#include "mowgli_hardware/ll_datatypes.hpp"
#include "mowgli_openmower_bridge/power_semantics.hpp"
#include <gtest/gtest.h>

using mowgli_hardware::STATUS_BIT_CHARGING;
using mowgli_hardware::STATUS_BIT_INITIALIZED;
using namespace mowgli_openmower_bridge;  // NOLINT

namespace
{
constexpr double kThr = kDefaultDockedChargeVoltage;
constexpr uint8_t kRelayOn = STATUS_BIT_INITIALIZED | STATUS_BIT_CHARGING;
constexpr uint8_t kRelayOff = STATUS_BIT_INITIALIZED;
}  // namespace

TEST(PowerSemantics, OldFirmwareRelayBitOffDockIsNotDocked)
{
  // v0.13.x: checkShouldCharge() is true at 0 V, so the relay bit reads 1
  // while mowing. Trusting it would zero /wheel_odom mid-lawn.
  EXPECT_TRUE(ChargeRelayOn(kRelayOn));
  EXPECT_FALSE(IsDocked(0.4f, kThr));
  EXPECT_EQ(Classify(IsDocked(0.4f, kThr), ChargeRelayOn(kRelayOn)), ChargeState::kUndocked);
}

TEST(PowerSemantics, DockedAndChargingOnEveryFirmware)
{
  EXPECT_EQ(Classify(IsDocked(29.1f, kThr), ChargeRelayOn(kRelayOn)), ChargeState::kCharging);
}

TEST(PowerSemantics, FullBatteryOnTheDockIsStillDocked)
{
  // Both firmwares open the relay once the battery reaches its cutoff; the
  // robot is still on the dock and must keep reading as such.
  EXPECT_FALSE(ChargeRelayOn(kRelayOff));
  EXPECT_TRUE(IsDocked(29.6f, kThr));
  EXPECT_EQ(Classify(IsDocked(29.6f, kThr), ChargeRelayOn(kRelayOff)),
            ChargeState::kDockedNotCharging);
}

TEST(PowerSemantics, ThresholdIsStrictAndNonFiniteIsUndocked)
{
  EXPECT_FALSE(IsDocked(10.0f, kThr));
  EXPECT_TRUE(IsDocked(10.01f, kThr));
  EXPECT_FALSE(IsDocked(std::nanf(""), kThr));
  EXPECT_FALSE(IsDocked(-30.0f, kThr));
}

TEST(PowerSemantics, ChargeCurrentIsZeroOffTheDock)
{
  // Field report: 0.05 A of sensor offset off the dock, relay on (v0.13).
  EXPECT_FLOAT_EQ(ReportedChargeCurrent(IsDocked(0.75f, kThr), 0.05f), 0.0f);
  EXPECT_FLOAT_EQ(ReportedChargeCurrent(IsDocked(29.1f, kThr), 0.85f), 0.85f);
  // A docked robot keeps its tail current: the charge-complete check needs it.
  EXPECT_FLOAT_EQ(ReportedChargeCurrent(true, 0.05f), 0.05f);
  EXPECT_FLOAT_EQ(ReportedChargeCurrent(true, std::nanf("")), 0.0f);
}

TEST(PowerSemantics, StatusStrings)
{
  EXPECT_EQ(std::string(ChargerStatusString(ChargeState::kCharging)), "charging");
  EXPECT_EQ(std::string(ChargerStatusString(ChargeState::kDockedNotCharging)),
            "docked, not charging");
  EXPECT_EQ(std::string(ChargerStatusString(ChargeState::kUndocked)), "idle");
}
