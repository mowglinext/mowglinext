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
  std::string reason;
};

/// What the next ll_heartbeat carries.
struct HeartbeatBits
{
  bool request{false};  ///< ll_heartbeat.emergency_requested
  bool release{false};  ///< ll_heartbeat.emergency_release_requested
};

class EmergencyTracker
{
public:
  /**
   * Heartbeats that carry one release request. More than one so a single
   * corrupted frame (the board drops it on CRC) does not swallow the reset;
   * few enough that a release can never linger.
   *
   * Why it must not linger: the v1 firmware clears its latch on ANY release
   * bit, then re-latches on the next loop if a stop/lift trigger is still
   * asserted. A release kept pending "until the board clears" (upstream
   * mower_comms_v1's behaviour) therefore stays armed while an operator holds
   * the stop button, and drops the latch by itself the moment the button is
   * let go — the robot is free to move with no fresh confirmation. MowgliNext
   * sends a release as a one-shot on the STM32 path; this keeps that rule.
   */
  static constexpr int kReleaseHeartbeats = 3;

  /// Host-side emergency (service call): asserted until RequestRelease().
  void RequestEmergency()
  {
    host_emergency_ = true;
    release_heartbeats_left_ = 0;
  }

  /// Host asks the board to drop its latch, on the next few heartbeats only.
  void RequestRelease()
  {
    host_emergency_ = false;
    release_heartbeats_left_ = kReleaseHeartbeats;
  }

  /// Fold the latest ll_status emergency bitmask in.
  [[nodiscard]] EmergencyEvaluation Evaluate(uint8_t ll_bitmask)
  {
    using mowgli_hardware::EMERGENCY_BIT_LATCH;
    using mowgli_hardware::EMERGENCY_BIT_LIFT;
    using mowgli_hardware::EMERGENCY_BIT_STOP;

    board_emergency_ = ll_bitmask != 0u;
    const bool trigger_active = (ll_bitmask & static_cast<uint8_t>(~EMERGENCY_BIT_LATCH)) != 0u;
    if (!board_emergency_ || trigger_active)
    {
      // Either the release worked, or the board refused it because a
      // trigger is still asserted. In both cases stop asking: a refused
      // release must be requested again once the trigger is gone.
      release_heartbeats_left_ = 0;
    }

    EmergencyEvaluation out{};
    out.active = trigger_active;
    out.latched = (ll_bitmask & EMERGENCY_BIT_LATCH) != 0u || host_emergency_;
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

  /// Bits for the heartbeat about to be sent; consumes one release slot.
  [[nodiscard]] HeartbeatBits NextHeartbeat()
  {
    HeartbeatBits bits{};
    // Re-asserted on every heartbeat while the host wants it, so a latch the
    // board dropped for any other reason is restored within one heartbeat.
    bits.request = host_emergency_;
    if (release_heartbeats_left_ > 0)
    {
      bits.release = true;
      --release_heartbeats_left_;
    }
    return bits;
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
  int release_heartbeats_left_{0};
};

}  // namespace mowgli_openmower_bridge
