// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
/**
 * @file emergency_tracker.hpp
 * @brief Emergency bookkeeping between the host and the OpenMower LowLevel
 *        board — the same contract OpenMower's mower_comms_v1 implements.
 *
 * The LowLevel board owns the physical inputs (stop buttons, lift/tilt
 * halls) and a latch. The host can ASK it to engage the latch
 * (heartbeat.emergency_requested) or to release it
 * (heartbeat.emergency_release_requested); the board decides. While either
 * side reports an emergency every actuator is driven to zero by the bridge —
 * on OpenMower hardware the host is in the actuation path, exactly as with
 * OpenMower's own stack.
 *
 * Pure: no ROS, no time.
 */

#pragma once

#include <cstdint>
#include <string>

#include "mowgli_hardware/ll_datatypes.hpp"

namespace mowgli_openmower_bridge
{

struct EmergencyEvaluation
{
  bool active{false};  ///< a physical trigger is asserted right now
  bool latched{false};  ///< board latch or host request still in force
  bool heartbeat_request{false};  ///< set ll_heartbeat.emergency_requested
  bool heartbeat_release{false};  ///< set ll_heartbeat.emergency_release_requested
  std::string reason;
};

class EmergencyTracker
{
public:
  /// Host-side emergency (service call): engaged until RequestRelease().
  void RequestEmergency()
  {
    host_emergency_ = true;
    release_pending_ = false;
  }

  /// Host asks the board to drop its latch; keeps asking until it does.
  void RequestRelease()
  {
    host_emergency_ = false;
    release_pending_ = true;
  }

  /// Fold the latest ll_status emergency bitmask in and derive everything.
  [[nodiscard]] EmergencyEvaluation Evaluate(uint8_t ll_bitmask)
  {
    using mowgli_hardware::EMERGENCY_BIT_LATCH;
    using mowgli_hardware::EMERGENCY_BIT_LIFT;
    using mowgli_hardware::EMERGENCY_BIT_STOP;

    board_emergency_ = ll_bitmask != 0u;
    if (!board_emergency_)
    {
      // The release worked (or nothing was latched): stop asking.
      release_pending_ = false;
    }

    EmergencyEvaluation out{};
    out.active = (ll_bitmask & static_cast<uint8_t>(~EMERGENCY_BIT_LATCH)) != 0u;
    out.latched = (ll_bitmask & EMERGENCY_BIT_LATCH) != 0u || host_emergency_;
    out.heartbeat_request = host_emergency_ && !board_emergency_;
    out.heartbeat_release = release_pending_;

    if ((ll_bitmask & EMERGENCY_BIT_STOP) != 0u)
    {
      out.reason = "STOP button";
    }
    else if ((ll_bitmask & EMERGENCY_BIT_LIFT) != 0u)
    {
      out.reason = "Lift detected";
    }
    else if ((ll_bitmask & EMERGENCY_BIT_LATCH) != 0u)
    {
      out.reason = "Latched (press play button to release)";
    }
    else if (host_emergency_)
    {
      out.reason = "Software emergency stop";
    }
    return out;
  }

  /// True while any side reports an emergency — actuators must be zero.
  [[nodiscard]] bool is_emergency() const noexcept
  {
    return host_emergency_ || board_emergency_;
  }

  [[nodiscard]] bool host_emergency() const noexcept
  {
    return host_emergency_;
  }

private:
  bool host_emergency_{false};
  bool board_emergency_{false};
  bool release_pending_{false};
};

}  // namespace mowgli_openmower_bridge
