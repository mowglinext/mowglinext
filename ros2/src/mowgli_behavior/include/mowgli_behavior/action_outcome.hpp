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
//
// Terminal verdict for an rclcpp_action goal driven from a BT node.
//
// A BT action node cannot rely on polling ClientGoalHandle::get_status()
// alone. The client only learns a goal's status from the action's status
// TOPIC, and it drops status messages for goals it does not know about yet —
// so when a server finishes a goal in the same instant it accepts it, the
// terminal status races ahead of the client registering the handle and is
// thrown away. No further status is ever published for that goal, the poll
// returns ACCEPTED forever, and the node stays RUNNING for good.
//
// Field, 2026-09-17: the operator put the robot on the dock by hand, the BT's
// critical-battery branch sent a DockRobot goal, and opennav_docking answered
// "Robot is already docked and/or charging, no need to dock" in the same
// second. DockRobot then sat RUNNING for 11.5 hours; because a BT Sequence
// resumes at its RUNNING child, IsBatteryLow was never re-evaluated and the
// robot still reported CRITICAL_BATTERY_DOCKING on a full pack.
//
// The fix is to also ask for the RESULT. Setting a result callback makes
// rclcpp_action request it as soon as the goal is accepted, which cannot lose
// the race, and the callback is then the authoritative terminal signal. This
// header holds the thread-safe slot it writes into and the pure rule that
// merges it with the polled status, so every action node can keep its existing
// switch on action_msgs GoalStatus constants unchanged.

#pragma once

#include <cstdint>
#include <mutex>
#include <optional>

namespace mowgli_behavior
{

/// How a goal ended, as reported by the result callback. Mirrors
/// rclcpp_action::ResultCode without depending on it, so the rule below stays
/// unit-testable without a ROS context.
enum class GoalOutcome : std::uint8_t
{
  kSucceeded,
  kAborted,
  kCanceled,
};

/// action_msgs/msg/GoalStatus constants, repeated here so this header needs no
/// ROS include. Static-asserted against the real message at the call sites.
inline constexpr std::int8_t kGoalStatusUnknown = 0;
inline constexpr std::int8_t kGoalStatusAccepted = 1;
inline constexpr std::int8_t kGoalStatusExecuting = 2;
inline constexpr std::int8_t kGoalStatusCanceling = 3;
inline constexpr std::int8_t kGoalStatusSucceeded = 4;
// CANCELED is 5 and ABORTED is 6 — not the other way round. The static_asserts
// in docking_nodes.cpp caught exactly that mistake here.
inline constexpr std::int8_t kGoalStatusCanceled = 5;
inline constexpr std::int8_t kGoalStatusAborted = 6;

inline bool IsTerminalGoalStatus(const std::int8_t status)
{
  return status == kGoalStatusSucceeded || status == kGoalStatusAborted ||
         status == kGoalStatusCanceled;
}

/// Written by the result callback (an executor thread), read by the BT tick.
class ActionOutcomeSlot
{
public:
  void Reset()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    outcome_.reset();
  }

  void Record(const GoalOutcome outcome)
  {
    std::lock_guard<std::mutex> lk(mutex_);
    // First verdict wins: a late duplicate must not reopen a decided goal.
    if (!outcome_.has_value())
    {
      outcome_ = outcome;
    }
  }

  std::optional<GoalOutcome> Get() const
  {
    std::lock_guard<std::mutex> lk(mutex_);
    return outcome_;
  }

private:
  mutable std::mutex mutex_;
  std::optional<GoalOutcome> outcome_;
};

/// The status a BT node should act on: the polled one while it still carries
/// information, otherwise whatever the result callback recorded.
///
/// A polled TERMINAL status always wins — it is the same verdict the callback
/// would give and it is already visible to the whole node. A recorded outcome
/// only fills the gap where the poll is stuck on a non-terminal value.
inline std::int8_t ResolveGoalStatus(const std::int8_t polled,
                                     const std::optional<GoalOutcome>& recorded)
{
  if (IsTerminalGoalStatus(polled) || !recorded.has_value())
  {
    return polled;
  }
  switch (*recorded)
  {
    case GoalOutcome::kSucceeded:
      return kGoalStatusSucceeded;
    case GoalOutcome::kAborted:
      return kGoalStatusAborted;
    case GoalOutcome::kCanceled:
      return kGoalStatusCanceled;
  }
  return polled;
}

}  // namespace mowgli_behavior
