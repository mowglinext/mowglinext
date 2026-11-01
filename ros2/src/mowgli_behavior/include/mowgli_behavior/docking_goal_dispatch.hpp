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

#include <memory>
#include <mutex>
#include <optional>
#include <utility>

#include "mowgli_behavior/action_outcome.hpp"
#include "mowgli_behavior/cancel_goal.hpp"

namespace mowgli_behavior
{

/// One docking action's lifetime, shared with callbacks that can outlive its BT node.
/// Acceptance and retirement are serialized: whichever runs second cancels the
/// accepted goal. Each dispatch has its own result and feedback identity.
template <typename ActionT>
class DockingGoalDispatch
{
public:
  using Client = rclcpp_action::Client<ActionT>;
  using Handle = rclcpp_action::ClientGoalHandle<ActionT>;
  struct Snapshot
  {
    bool responded;
    typename Handle::SharedPtr handle;
    std::optional<GoalOutcome> outcome;
  };

  DockingGoalDispatch(const typename Client::SharedPtr& client, rclcpp::Logger logger)
      : client_(client), logger_(std::move(logger))
  {
  }

  void Accept(const typename Client::SharedPtr& client, const typename Handle::SharedPtr& handle)
  {
    bool cancel = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      responded_ = true;
      accepted_ = static_cast<bool>(handle);
      cancel = retired_ && handle;
      if (!retired_)
      {
        handle_ = handle;
      }
    }
    if (cancel)
    {
      cancelGoalQuietly(client, handle, logger_, "DockingGoalDispatch");
    }
  }

  void Retire()
  {
    typename Handle::SharedPtr handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      retired_ = true;
      handle = std::move(handle_);
    }
    // Never acquire the goal-handle mutex while holding ours: rclcpp_action
    // invokes the result callback with its goal-handle mutex already held.
    if (handle)
    {
      const auto status = handle->get_status();
      if (IsTerminalGoalStatus(status))
      {
        Record(status == kGoalStatusSucceeded  ? GoalOutcome::kSucceeded
               : status == kGoalStatusCanceled ? GoalOutcome::kCanceled
                                               : GoalOutcome::kAborted);
      }
    }
    cancelGoalQuietly(client_.lock(), handle, logger_, "DockingGoalDispatch");
  }

  void Record(GoalOutcome outcome)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Retired goals still report completion so a restart can wait for their
    // server-side cancellation to finish before sending another goal.
    if (!outcome_)
    {
      outcome_ = outcome;
    }
  }

  bool Finished() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return outcome_.has_value() || (responded_ && !accepted_);
  }

  Snapshot Get() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return {responded_, handle_, outcome_};
  }

  bool FeedbackChanged(uint16_t state)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (retired_)
    {
      return false;
    }
    const auto previous = feedback_state_;
    feedback_state_ = state;
    return state != previous;
  }

private:
  mutable std::mutex mutex_;
  std::weak_ptr<Client> client_;
  rclcpp::Logger logger_;
  bool responded_ = false;
  bool accepted_ = false;
  bool retired_ = false;
  typename Handle::SharedPtr handle_;
  std::optional<GoalOutcome> outcome_;
  uint16_t feedback_state_ = 0;
};

}  // namespace mowgli_behavior
