// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
/**
 * @file power_semantics.hpp
 * @brief What "on the dock" means on OpenMower v1 electronics.
 *
 * MowgliNext's Status.is_charging means ON THE DOCK (charger contacts live):
 * it gates the zero-odometry rule (Invariant 11), the BT's charging state, the
 * docking current (Invariant 12) and the at-rest IMU calibration.
 *
 * The LowLevel status bit 2 (STATUS_BIT_CHARGING) cannot carry that meaning.
 * In the v1 firmware it is `charging_allowed`, the charge RELAY, and it is
 * wrong in opposite directions on the two firmware generations in the field:
 *
 *   situation              v0.13.x firmware        current v1-fw branch
 *   mowing, off the dock   1  (checkShouldCharge   0
 *                             is true at 0 V)
 *   docked, charging       1                       1
 *   docked, battery full   0  (relay cut off)      0  (relay cut off)
 *
 * (OpenMower Firmware/LowLevel/src/main.cpp, updateChargingEnabled /
 * checkShouldCharge, at tag v0.13.2 and on branch v1-fw.) Trusting the bit
 * would zero /wheel_odom in the middle of the lawn on old firmware, and report
 * an undocked robot while it sits on a full-battery dock on both.
 *
 * OpenMower's own high level never used the bit for this either: mower_logic
 * sets is_charging = charge_voltage > 10.0 V. The voltage at the charging
 * contacts is present exactly when the robot is on a powered dock, whatever
 * the relay is doing, so that is the signal used here. Pure, no ROS.
 */

#pragma once

#include <cmath>
#include <cstdint>

#include "mowgli_hardware/ll_datatypes.hpp"

namespace mowgli_openmower_bridge
{

/// OpenMower mower_logic's own threshold (`charge_voltage > 10.0`).
constexpr double kDefaultDockedChargeVoltage = 10.0;

/// On a powered dock: the contacts carry the charger voltage.
[[nodiscard]] inline bool IsDocked(float v_charge, double threshold) noexcept
{
  return std::isfinite(v_charge) && static_cast<double>(v_charge) > threshold;
}

/// The charge relay as the firmware reports it — informational only.
[[nodiscard]] constexpr bool ChargeRelayOn(uint8_t status_bitmask) noexcept
{
  return (status_bitmask & mowgli_hardware::STATUS_BIT_CHARGING) != 0u;
}

enum class ChargeState
{
  kUndocked,
  kCharging,  ///< docked, relay closed
  kDockedNotCharging,  ///< docked, relay open (full battery or a cutoff)
};

[[nodiscard]] constexpr ChargeState Classify(bool docked, bool relay_on) noexcept
{
  if (!docked)
  {
    return ChargeState::kUndocked;
  }
  return relay_on ? ChargeState::kCharging : ChargeState::kDockedNotCharging;
}

[[nodiscard]] constexpr const char* ChargerStatusString(ChargeState s) noexcept
{
  switch (s)
  {
    case ChargeState::kCharging:
      return "charging";
    case ChargeState::kDockedNotCharging:
      return "docked, not charging";
    case ChargeState::kUndocked:
    default:
      return "idle";
  }
}

}  // namespace mowgli_openmower_bridge
