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

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace mowgli_behavior
{

/// Cancel an action goal from a halt / cleanup path WITHOUT ever throwing.
///
/// Once a goal is terminal and its result has been delivered, rclcpp_action's
/// client forgets the handle and async_cancel_goal() throws
/// UnknownGoalHandleError ("Goal handle is not known to this client"). A throw
/// out of onHalted() skips the rest of the halt (the handle reset, and in
/// NavigateInsideBoundary the keepout-filter restore) and leaves the node
/// RUNNING, so its parent halts it again on the next tick and it throws again.
/// Field 2026-09-21/22: the robot reached the charger while DockRobot's goal
/// finished, HomeOrAlreadyDocked halted DockRobot, and the tree then threw
/// 73 840 times at 10 Hz for 10 h, frozen in RETURNING_HOME.
///
/// Returns true when a cancel request was sent. A goal that is already gone
/// needs no cancel — that is the normal end of a race, not an error.
template <typename ActionT>
bool cancelGoalQuietly(const std::shared_ptr<rclcpp_action::Client<ActionT>>& client,
                       const std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>>& handle,
                       const rclcpp::Logger& logger,
                       const char* who) noexcept
{
  if (!client || !handle)
  {
    return false;
  }
  try
  {
    client->async_cancel_goal(handle);
    return true;
  }
  catch (const rclcpp_action::exceptions::UnknownGoalHandleError&)
  {
    RCLCPP_INFO(logger, "%s: goal had already finished — nothing to cancel", who);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_WARN(logger, "%s: cancel failed: %s", who, ex.what());
  }
  return false;
}

}  // namespace mowgli_behavior
