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

#include <atomic>
#include <chrono>
#include <exception>
#include <functional>
#include <future>
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

/// Shared between whoever sends an action goal and that goal's response callback.
///
/// A goal that has been SENT but not yet ANSWERED has no handle, so nothing can cancel it:
/// the server accepts it a moment later and the sender, which has meanwhile moved on, never
/// learns of it. Field 2026-10-09: FollowStrip dispatched a blade-off NavigateToPose transit
/// and booked the same unit as finished 77 ms later; bt_navigator accepted the orphaned goal
/// and then refused every other navigation request for seven minutes — new transits, the
/// dock approach — until the orphan gave up by itself, leaving the mower idle in the field.
struct GoalAbandonment
{
  std::atomic<bool> abandoned{false};
};

/// Goal-response callback for SendGoalOptions: cancels the goal the instant the server accepts
/// it if its sender abandoned it in the meantime. Holds the client weakly and never throws.
template <typename ActionT>
std::function<void(typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr)>
cancelWhenAbandoned(const std::shared_ptr<rclcpp_action::Client<ActionT>>& client,
                    const std::shared_ptr<GoalAbandonment>& abandonment)
{
  return [weak_client = std::weak_ptr<rclcpp_action::Client<ActionT>>(client),
          abandonment](typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr handle)
  {
    if (!handle || !abandonment || !abandonment->abandoned.load())
    {
      return;  // rejected, or still wanted
    }
    if (auto locked = weak_client.lock())
    {
      try
      {
        locked->async_cancel_goal(handle);
      }
      catch (const std::exception&)
      {
        // Already finished: nothing left to cancel.
      }
    }
  };
}

/// Give up on a goal for good, whether or not the server has answered it yet.
///
/// Marks it abandoned (so cancelWhenAbandoned() cancels it on acceptance), and cancels it now
/// when its handle is known — taken from `handle`, or from `future` when the answer has
/// arrived but nobody picked it up. Returns true when a cancel request was sent.
template <typename ActionT>
bool abandonGoal(
    const std::shared_ptr<rclcpp_action::Client<ActionT>>& client,
    std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>> handle,
    const std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>>>& future,
    const std::shared_ptr<GoalAbandonment>& abandonment,
    const rclcpp::Logger& logger,
    const char* who) noexcept
{
  if (abandonment)
  {
    abandonment->abandoned.store(true);
  }
  if (!handle && future.valid() &&
      future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    try
    {
      handle = future.get();
    }
    catch (const std::exception&)
    {
      handle.reset();
    }
  }
  return cancelGoalQuietly(client, handle, logger, who);
}

/// Cancel every goal on an action server, from a recovery path, WITHOUT ever throwing.
/// Used when the server refused a goal: if it is busy with a goal nobody owns any more, this
/// frees it for the next attempt.
template <typename ActionT>
void cancelAllGoalsQuietly(const std::shared_ptr<rclcpp_action::Client<ActionT>>& client,
                           const rclcpp::Logger& logger,
                           const char* who) noexcept
{
  if (!client)
  {
    return;
  }
  try
  {
    client->async_cancel_all_goals();
    RCLCPP_WARN(logger,
                "%s: goal refused — cancelled any stale goal the server is still busy with",
                who);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_WARN(logger, "%s: could not cancel stale goals: %s", who, ex.what());
  }
}

}  // namespace mowgli_behavior
