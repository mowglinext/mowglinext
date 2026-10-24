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
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/stop_reason.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mowgli_behavior
{

/// COMMAND_GOTO (HighLevelControl.srv). Kept here so the ~/goto_pose handler
/// and the tests share one definition.
constexpr uint8_t kCommandGoto = 9;

struct GotoRequestDecision
{
  bool accepted{false};
  std::string reason;
};

/// Battery levels a GOTO must not start below — the same values the tree's
/// GotoLowBatteryGuard trips on (battery_low_pct, battery_critical_pct and
/// battery_critical_voltage on the blackboard).
struct GotoBatteryLimits
{
  float low_pct{20.0f};
  float critical_pct{10.0f};
  float critical_voltage{0.0f};  // 0 disables the voltage gate, as in IsBatteryLow
};

/// A battery reading older than this is not trusted for a GOTO start — the
/// same 2 s staleness bound IsEmergency applies to the emergency topic.
/// /hardware_bridge/power arrives at 4 Hz on the robot, 10 Hz in sim.
constexpr std::chrono::seconds kGotoMaxPowerAge{2};

/// goto_timeout_sec: 0 disables the limit, otherwise a positive finite number
/// of seconds. Anything else (negative, NaN, Inf) is a configuration error.
inline bool isValidGotoTimeout(double seconds)
{
  return std::isfinite(seconds) && seconds >= 0.0;
}

/// Decision behind the ~/goto_pose service (maintenance is checked by the caller).
///
/// A GOTO only starts from a settled, undocked rest state; anything else is
/// refused and leaves the context untouched:
///   - "invalid_pose": x, y or yaw is NaN/Inf.
///   - "busy": a mission is active (a GOTO included — no in-flight target
///     replacement, the caller sends COMMAND_STOP then a new request), or
///     current_command is idle (0) / stop (8) but the tree is not actually
///     parked in its rest branch (ctx.tree_at_rest, from the last tick) or
///     has not published IDLE — e.g. a critical-battery dock or charge hold
///     running with command 0, or a root guard handler (sensor fault,
///     emergency) that holds the tree without publishing a new state.
///   - "critical_dock_failure" / "critical_charge_hold": a CriticalBatteryDock
///     latch is set. These are cleared only by an operator command on
///     ~/high_level_control or ~/start_in_area; a GOTO request never clears
///     them.
///   - "battery_charge_pending": a battery-driven dock/charge is unfinished
///     (e.g. a charge hold halted by a root guard, then the robot set down
///     off the dock). STOP or HOME releases it; a GOTO request never does.
///   - "docked": on the charger, or an undock reverse is unfinished. GOTO has
///     no undock sequence of its own; driving forward from there would push
///     into the station.
///   - "heading_not_calibrated": no heading seed this session (the mowing
///     start drives 1 m forward to get one, GOTO does not).
///   - "localization_degraded": the latch LocalizationGuard reads is set
///     (exempted at rest, so it would only pause the GOTO right after start).
///   - "battery_unknown": no valid filtered battery voltage yet, or the last
///     valid sample is older than kGotoMaxPowerAge — battery_percent then
///     still holds its 100 % default or a frozen value.
///   - "battery_low": below the level at which GotoLowBatteryGuard would
///     immediately send the robot back to the dock.
/// Otherwise the map-frame target is stored and current_command becomes
/// COMMAND_GOTO, together, under the context mutex.
inline GotoRequestDecision acceptGotoRequest(BTContext& ctx,
                                             double x,
                                             double y,
                                             double yaw,
                                             const GotoBatteryLimits& battery,
                                             const builtin_interfaces::msg::Time& stamp,
                                             std::chrono::steady_clock::time_point steady_now)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw))
  {
    return {false, "invalid_pose"};
  }
  std::lock_guard<std::mutex> lock(ctx.context_mutex);
  const bool rest_command = ctx.current_command == 0 || ctx.current_command == 8;
  const std::string& state = ctx.last_high_level_status.state_name;
  const bool rest_state = state == "IDLE" || state == "GOTO_REACHED" || state == "GOTO_FAILED";
  if (!rest_command || !rest_state || !ctx.tree_at_rest)
  {
    return {false, "busy"};
  }
  if (ctx.critical_dock_failure_latched)
  {
    return {false, "critical_dock_failure"};
  }
  if (ctx.critical_charge_stop_latched)
  {
    return {false, "critical_charge_hold"};
  }
  if (ctx.battery_charge_pending)
  {
    return {false, "battery_charge_pending"};
  }
  if (ctx.latest_power.charger_enabled || ctx.undock_start_recorded)
  {
    return {false, "docked"};
  }
  if (!ctx.yaw_seeded_this_session)
  {
    return {false, "heading_not_calibrated"};
  }
  if (ctx.localization_degraded)
  {
    return {false, "localization_degraded"};
  }
  if (ctx.battery_voltage_filtered <= 0.0f ||
      ctx.last_power_time == std::chrono::steady_clock::time_point{} ||
      steady_now - ctx.last_power_time > kGotoMaxPowerAge)
  {
    return {false, "battery_unknown"};
  }
  const bool voltage_critical = battery.critical_voltage > 0.0f &&
                                ctx.battery_voltage_filtered > 0.0f &&
                                ctx.battery_voltage_filtered < battery.critical_voltage;
  if (ctx.battery_percent <= battery.low_pct || ctx.battery_percent < battery.critical_pct ||
      voltage_critical)
  {
    return {false, "battery_low"};
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);
  // An accepted GOTO is an operator command like any other: fence fleet
  // ownership, so ~/fleet_yield stops reporting held=true while the robot
  // drives. Last step before accepting: no refusal can follow the revoke.
  ctx.fleet_yield.revoke();
  // A new mission: the reason the previous one ended no longer describes the
  // mower, as for every other accepted command except STOP.
  clearStopReason(ctx);
  ctx.goto_target_pose = pose;
  ctx.current_command = kCommandGoto;
  return {true, ""};
}

/// Whether the tree is parked in a rest branch, read from the tree's own node
/// statuses (BTContext::tree_at_rest) — never from the published state name,
/// which can stay "IDLE" while a root guard handler holds the tree.
///
/// At rest after a tick means: MainLogic's IdleSequence or StopHoldSequence is
/// RUNNING (every root guard passed and MainLogic resolved to a rest branch),
/// or the tick SUCCEEDED by completing one of them (the end of a rest cycle)
/// or GotoSequence (whose only SUCCESS exits are GOTO_REACHED / GOTO_FAILED;
/// the next tick starts IdleSequence, so a request in between — chaining —
/// must not be refused). The branch that completed is identified by the
/// status-change notifications of those nodes during the tick, so any other
/// branch completing never counts, whatever was published last.
///
/// beforeTick() / afterTick() wrap every tickOnce(), on the tick thread. A
/// custom tree without these branches is never at rest, so a GOTO is never
/// accepted there; missingBranches() names what is missing.
class GotoRestObserver
{
public:
  explicit GotoRestObserver(BT::Tree& tree)
  {
    tree.applyVisitor(
        [this](BT::TreeNode* node)
        {
          const bool rest_branch =
              node->name() == "IdleSequence" || node->name() == "StopHoldSequence";
          if (!rest_branch && node->name() != "GotoSequence")
          {
            return;
          }
          found_.push_back(node->name());
          if (rest_branch)
          {
            rest_nodes_.push_back(node);
          }
          subscribers_.push_back(node->subscribeToStatusChange(
              [this](BT::TimePoint, const BT::TreeNode&, BT::NodeStatus, BT::NodeStatus status)
              {
                if (status == BT::NodeStatus::SUCCESS)
                {
                  completed_this_tick_ = true;
                }
              }));
        });
  }

  GotoRestObserver(const GotoRestObserver&) = delete;
  GotoRestObserver& operator=(const GotoRestObserver&) = delete;

  void beforeTick()
  {
    completed_this_tick_ = false;
  }

  bool afterTick(BT::NodeStatus root_status) const
  {
    for (const BT::TreeNode* node : rest_nodes_)
    {
      if (node->status() == BT::NodeStatus::RUNNING)
      {
        return true;
      }
    }
    return root_status == BT::NodeStatus::SUCCESS && completed_this_tick_;
  }

  std::size_t restBranchCount() const
  {
    return rest_nodes_.size();
  }

  /// The branches this observer (and so ~/goto_pose) relies on that the tree
  /// lacks. Checked once after loading the tree, so a tree that renamed or
  /// dropped one of them is reported explicitly instead of every GOTO being
  /// refused as "busy" without explanation.
  std::vector<std::string> missingBranches() const
  {
    std::vector<std::string> missing;
    for (const char* name : {"IdleSequence", "StopHoldSequence", "GotoSequence"})
    {
      if (std::find(found_.begin(), found_.end(), name) == found_.end())
      {
        missing.emplace_back(name);
      }
    }
    return missing;
  }

private:
  std::vector<std::string> found_;
  std::vector<const BT::TreeNode*> rest_nodes_;
  std::vector<BT::TreeNode::StatusChangeSubscriber> subscribers_;
  bool completed_this_tick_{false};
};

/// Drops the GOTO target once COMMAND_GOTO is no longer active (reached,
/// failed, STOP/HOME, maintenance, ...). Called at the top of every tick with
/// the context mutex held, so a target can only ever be read by the GOTO that
/// stored it. Race-free with ~/goto_pose: the service and the tick share the
/// node's MutuallyExclusive default callback group, and a request accepted
/// between two ticks sets the target and current_command=9 together, so this
/// sees 9 and keeps the new target.
inline void dropStaleGotoTargetLocked(BTContext& ctx)
{
  if (ctx.current_command != kCommandGoto)
  {
    ctx.goto_target_pose.reset();
  }
}

}  // namespace mowgli_behavior
