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

// SPDX-License-Identifier: GPL-3.0
/**
 * @file goto_action.hpp
 * @brief The ~/goto action server (mowgli_interfaces/action/Goto).
 *
 * The action runs the same mission as ~/goto_pose: a goal is admitted by the
 * caller's admission function (acceptGotoRequest plus the node-level checks)
 * and then executed by GotoSequence. The server only observes the tree: after
 * every tick, afterTick() decides from the context whether the active goal has
 * ended (evaluateGotoMission) and finishes it exactly once.
 *
 * Threading: every callback of this class (goal, cancel, accepted, /wheel_odom)
 * and afterTick() run in the node's default MutuallyExclusive callback group,
 * the group of the tick timer and of ~/high_level_control, so they never run
 * concurrently with a tick or with another command.
 */

#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_behavior/battery_recovery_command.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/goto_outcome.hpp"
#include "mowgli_behavior/goto_request.hpp"
#include "mowgli_interfaces/action/goto.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"

namespace mowgli_behavior
{

inline constexpr uint8_t kCommandStop = 8;  // HighLevelControl::Request::COMMAND_STOP

/// Context snapshot taken after a tick, for an active ~/goto goal.
struct GotoMissionInput
{
  uint8_t command{0};
  std::string outcome;  ///< BTContext::goto_outcome
  std::string state_name;  ///< last published HighLevelStatus state_name
  bool cancel_requested{false};
  double elapsed_s{0.0};  ///< since the goal was admitted, monotonic
  double deadline_s{0.0};  ///< goto_deadline_sec, 0 = none
};

enum class GotoMissionStep
{
  kContinue,  ///< still running or paused
  kSettle,  ///< target reached: wait for standstill, then succeed
  kFinish,  ///< ended: report the result
  kStopAndFinish,  ///< end it with COMMAND_STOP, then report the result
};

struct GotoMissionDecision
{
  GotoMissionStep step{GotoMissionStep::kContinue};
  uint16_t error_code{0};
  std::string error_msg;
};

/// Decides what happens to an active ~/goto goal after a tick. Order:
///   1. an outcome recorded by the tree (SetGotoOutcome) ends the goal with it;
///   2. a client cancellation stops the GOTO (unless it already ended);
///   3. current_command no longer COMMAND_GOTO without an outcome: another
///      command, maintenance, CHARGER_FAILED... ended it (INTERRUPTED);
///   4. critical-battery handling holds the tree (CRITICAL_BATTERY_* state):
///      it owns the robot from here, the goal ends without touching it;
///   5. past goto_deadline_sec: the GOTO is stopped (DEADLINE).
/// Pauses by a root guard keep command 9 and no outcome, so they continue.
inline GotoMissionDecision evaluateGotoMission(const GotoMissionInput& in)
{
  using Result = mowgli_interfaces::action::Goto::Result;
  if (!in.outcome.empty())
  {
    if (in.outcome == kGotoOutcomeReached)
    {
      return {GotoMissionStep::kSettle, Result::NONE, ""};
    }
    if (in.outcome == kGotoOutcomeFailed)
    {
      return {GotoMissionStep::kFinish,
              Result::FAILED,
              "navigation failed or a run exceeded goto_timeout_sec"};
    }
    if (in.outcome == kGotoOutcomeLowBatteryDocked)
    {
      return {GotoMissionStep::kFinish, Result::LOW_BATTERY_DOCKED, "docked on low battery"};
    }
    if (in.outcome == kGotoOutcomeLowBatteryDockFailed)
    {
      return {GotoMissionStep::kFinish,
              Result::LOW_BATTERY_DOCK_FAILED,
              "low battery, docking failed"};
    }
    return {GotoMissionStep::kFinish, Result::CRITICAL_BATTERY, in.outcome};
  }
  if (in.cancel_requested)
  {
    return {in.command == kCommandGoto ? GotoMissionStep::kStopAndFinish : GotoMissionStep::kFinish,
            Result::CANCELED,
            "canceled by the client"};
  }
  if (in.command != kCommandGoto)
  {
    return {GotoMissionStep::kFinish,
            Result::INTERRUPTED,
            "command=" + std::to_string(in.command) + " state=" + in.state_name};
  }
  if (in.state_name.rfind("CRITICAL_BATTERY", 0) == 0)
  {
    return {GotoMissionStep::kFinish, Result::CRITICAL_BATTERY, in.state_name};
  }
  if (in.deadline_s > 0.0 && in.elapsed_s >= in.deadline_s)
  {
    return {GotoMissionStep::kStopAndFinish, Result::DEADLINE, "goto_deadline_sec exceeded"};
  }
  return {};
}

/// Standstill test for a reached target: the wheels report less than these
/// speeds, from a fresh /wheel_odom sample, for kGotoSettleHold.
inline constexpr double kGotoSettleMaxLinear = 0.02;  // m/s
inline constexpr double kGotoSettleMaxAngular = 0.05;  // rad/s
inline constexpr std::chrono::milliseconds kGotoSettleHold{500};
inline constexpr std::chrono::milliseconds kGotoSettleTimeout{3000};
inline constexpr std::chrono::milliseconds kGotoOdomMaxAge{500};
/// A reported pose older than this is marked invalid.
inline constexpr double kGotoPoseMaxAgeSec = 1.0;

class GotoActionServer
{
public:
  using Goto = mowgli_interfaces::action::Goto;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Goto>;
  using Admit = std::function<GotoRequestDecision(double x, double y, double yaw)>;

  GotoActionServer(rclcpp::Node& owner,
                   const std::shared_ptr<BTContext>& context,
                   Admit admit,
                   double deadline_s,
                   std::chrono::seconds result_timeout = std::chrono::hours(1))
      : node_(owner), context_(context), admit_(std::move(admit)), deadline_s_(deadline_s)
  {
    auto group = owner.get_node_base_interface()->get_default_callback_group();
    rcl_action_server_options_t options = rcl_action_server_get_default_options();
    options.result_timeout.nanoseconds =
        std::chrono::duration_cast<std::chrono::nanoseconds>(result_timeout).count();
    server_ = rclcpp_action::create_server<Goto>(
        &owner,
        "~/goto",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Goto::Goal>)
        {
          // Admission needs the tick-thread view of the context: done in
          // handleAccepted, which reports a refusal as an aborted goal.
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<GoalHandle> handle)
        {
          return handleCancel(handle);
        },
        [this](const std::shared_ptr<GoalHandle> handle)
        {
          handleAccepted(handle);
        },
        options,
        group);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = group;
    wheel_odom_sub_ = owner.create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odom",
        rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
          wheel_linear_ = msg->twist.twist.linear.x;
          wheel_angular_ = msg->twist.twist.angular.z;
          wheel_stamp_ = std::chrono::steady_clock::now();
        },
        sub_options);
  }

  GotoActionServer(const GotoActionServer&) = delete;
  GotoActionServer& operator=(const GotoActionServer&) = delete;

  /// After every tick, on the tick thread. goto_running: GotoSequence is
  /// RUNNING after the tick (false while a root guard or the battery logic
  /// holds the tree).
  void afterTick(bool goto_running)
  {
    if (!active_)
    {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (settle_since_)
    {
      settle(now);
      return;
    }
    GotoMissionDecision decision;
    std::string state_name;
    {
      std::lock_guard<std::mutex> lock(context_->context_mutex);
      GotoMissionInput in;
      in.command = context_->current_command;
      in.outcome = context_->goto_outcome;
      in.state_name = context_->last_high_level_status.state_name;
      in.cancel_requested = cancel_requested_;
      in.elapsed_s = std::chrono::duration<double>(now - admitted_at_).count();
      in.deadline_s = deadline_s_;
      decision = evaluateGotoMission(in);
      state_name = in.state_name;
      if (decision.step == GotoMissionStep::kStopAndFinish)
      {
        context_->current_command = kCommandStop;
        applyCommandToBatteryRecovery(*context_, kCommandStop);
      }
    }
    switch (decision.step)
    {
      case GotoMissionStep::kContinue:
        publishFeedback(goto_running, state_name, now);
        return;
      case GotoMissionStep::kSettle:
        settle_since_ = now;
        standstill_since_.reset();
        settle(now);
        return;
      case GotoMissionStep::kFinish:
      case GotoMissionStep::kStopAndFinish:
        finish(decision.error_code, decision.error_msg, false);
        return;
    }
  }

  /// Ends an active goal with SHUTDOWN. Called when the node shuts down.
  void shutdown()
  {
    if (active_)
    {
      finish(Goto::Result::SHUTDOWN, "behavior tree node shutting down", false);
    }
  }

  bool active() const
  {
    return static_cast<bool>(active_);
  }

private:
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>& handle)
  {
    if (!active_ || active_->get_goal_id() != handle->get_goal_id())
    {
      return rclcpp_action::CancelResponse::REJECT;
    }
    // Applied by the next afterTick(); a reached target that is settling ends
    // with its result.
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle>& handle)
  {
    const auto goal = handle->get_goal();
    auto result = std::make_shared<Goto::Result>();
    result->error_code = Goto::Result::REFUSED;
    if (active_)
    {
      result->error_msg = "busy";
      fillPose(*result);
      handle->abort(result);
      return;
    }
    const GotoRequestDecision decision = admit_(goal->x, goal->y, goal->yaw);
    if (!decision.accepted)
    {
      result->error_msg = decision.reason;
      fillPose(*result);
      RCLCPP_WARN(node_.get_logger(), "GotoAction: refused (%s)", decision.reason.c_str());
      handle->abort(result);
      return;
    }
    active_ = handle;
    target_x_ = goal->x;
    target_y_ = goal->y;
    admitted_at_ = std::chrono::steady_clock::now();
    cancel_requested_ = false;
    settle_since_.reset();
    standstill_since_.reset();
    last_feedback_ = {};
    RCLCPP_INFO(node_.get_logger(),
                "GotoAction: accepted x=%.2f y=%.2f yaw=%.2f",
                goal->x,
                goal->y,
                goal->yaw);
  }

  void settle(std::chrono::steady_clock::time_point now)
  {
    const bool fresh = wheel_stamp_ && now - *wheel_stamp_ <= kGotoOdomMaxAge;
    const bool still = fresh && std::abs(wheel_linear_) < kGotoSettleMaxLinear &&
                       std::abs(wheel_angular_) < kGotoSettleMaxAngular;
    if (!still)
    {
      standstill_since_.reset();
    }
    else if (!standstill_since_)
    {
      standstill_since_ = now;
    }
    if (standstill_since_ && now - *standstill_since_ >= kGotoSettleHold)
    {
      finish(Goto::Result::NONE, "", true);
    }
    else if (now - *settle_since_ >= kGotoSettleTimeout)
    {
      finish(Goto::Result::NONE, "no standstill reported", false);
    }
  }

  void publishFeedback(bool goto_running,
                       const std::string& state_name,
                       std::chrono::steady_clock::time_point now)
  {
    if (last_feedback_ != std::chrono::steady_clock::time_point{} &&
        now - last_feedback_ < std::chrono::seconds(1))
    {
      return;
    }
    last_feedback_ = now;
    auto feedback = std::make_shared<Goto::Feedback>();
    feedback->phase = goto_running ? Goto::Feedback::PHASE_ACTIVE : Goto::Feedback::PHASE_PAUSED;
    feedback->state_name = state_name;
    feedback->current_pose_valid = lookupPose(feedback->current_pose);
    if (feedback->current_pose_valid)
    {
      feedback->distance_remaining =
          static_cast<float>(std::hypot(target_x_ - feedback->current_pose.pose.position.x,
                                        target_y_ - feedback->current_pose.pose.position.y));
    }
    active_->publish_feedback(feedback);
  }

  void finish(uint16_t error_code, const std::string& error_msg, bool settled)
  {
    auto handle = std::move(active_);
    settle_since_.reset();
    auto result = std::make_shared<Goto::Result>();
    result->error_code = error_code;
    result->error_msg = error_msg;
    result->settled = settled;
    fillPose(*result);
    RCLCPP_INFO(node_.get_logger(),
                "GotoAction: ended (code %u%s%s)",
                error_code,
                error_msg.empty() ? "" : ": ",
                error_msg.c_str());
    if (error_code == Goto::Result::NONE)
    {
      handle->succeed(result);
    }
    else if (error_code == Goto::Result::CANCELED && handle->is_canceling())
    {
      handle->canceled(result);
    }
    else
    {
      handle->abort(result);
    }
  }

  void fillPose(Goto::Result& result)
  {
    result.final_pose_valid = lookupPose(result.final_pose);
  }

  /// map → base_link, valid only when the transform is recent and localization
  /// is not degraded.
  bool lookupPose(geometry_msgs::msg::PoseStamped& pose)
  {
    pose.header.frame_id = "map";
    std::shared_ptr<tf2_ros::Buffer> buffer;
    bool degraded = false;
    {
      std::lock_guard<std::mutex> lock(context_->context_mutex);
      buffer = context_->tf_buffer;
      degraded = context_->localization_degraded;
    }
    if (!buffer)
    {
      return false;
    }
    try
    {
      const auto tf = buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
      pose.header.stamp = tf.header.stamp;
      pose.pose.position.x = tf.transform.translation.x;
      pose.pose.position.y = tf.transform.translation.y;
      pose.pose.position.z = tf.transform.translation.z;
      pose.pose.orientation = tf.transform.rotation;
      const double age =
          (node_.now() - rclcpp::Time(tf.header.stamp, node_.now().get_clock_type())).seconds();
      return !degraded && age <= kGotoPoseMaxAgeSec;
    }
    catch (const tf2::TransformException&)
    {
      return false;
    }
  }

  rclcpp::Node& node_;
  std::shared_ptr<BTContext> context_;
  Admit admit_;
  double deadline_s_;
  rclcpp_action::Server<Goto>::SharedPtr server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;

  std::shared_ptr<GoalHandle> active_;
  double target_x_{0.0};
  double target_y_{0.0};
  std::chrono::steady_clock::time_point admitted_at_;
  bool cancel_requested_{false};
  std::optional<std::chrono::steady_clock::time_point> settle_since_;
  std::optional<std::chrono::steady_clock::time_point> standstill_since_;
  std::chrono::steady_clock::time_point last_feedback_;

  double wheel_linear_{0.0};
  double wheel_angular_{0.0};
  std::optional<std::chrono::steady_clock::time_point> wheel_stamp_;
};

}  // namespace mowgli_behavior
