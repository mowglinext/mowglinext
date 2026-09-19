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

#include "mowgli_behavior/utility_nodes.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

namespace
{

/// Spin the node's executor for up to timeout_ms to wait for a service.
bool waitForService(rclcpp::ClientBase::SharedPtr client,
                    const rclcpp::Node::SharedPtr& node,
                    int timeout_ms = 1000)
{
  if (client->wait_for_service(std::chrono::milliseconds(timeout_ms)))
  {
    return true;
  }
  RCLCPP_WARN(node->get_logger(), "Service '%s' not available", client->get_service_name());
  return false;
}

}  // namespace

// ---------------------------------------------------------------------------
// SetMowerEnabled
// ---------------------------------------------------------------------------

BT::NodeStatus SetMowerEnabled::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  auto res = getInput<bool>("enabled");
  if (!res)
  {
    RCLCPP_ERROR(ctx->node->get_logger(),
                 "SetMowerEnabled: missing required port 'enabled': %s",
                 res.error().c_str());
    return BT::NodeStatus::FAILURE;
  }
  const bool enabled = res.value();

  if (!client_)
  {
    client_ = ctx->node->create_client<mowgli_interfaces::srv::MowerControl>(
        "/hardware_bridge/mower_control");
  }

  if (!waitForService(client_, ctx->node))
  {
    // In simulation, no hardware bridge is running — proceed gracefully.
    RCLCPP_WARN(ctx->node->get_logger(),
                "SetMowerEnabled: hardware service unavailable, continuing (simulation mode)");
    return BT::NodeStatus::SUCCESS;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::MowerControl::Request>();
  request->mow_enabled = enabled ? 1u : 0u;
  request->mow_direction = 0u;

  // Fire-and-forget: the firmware is the safety authority for the blade.
  // It has its own lift/tilt/emergency checks and will refuse or stop the
  // blade regardless of what ROS2 requests.
  auto future = client_->async_send_request(request);
  (void)future;

  RCLCPP_INFO(ctx->node->get_logger(),
              "SetMowerEnabled: requested mow_enabled=%s",
              enabled ? "true" : "false");

  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// WaitForDuration
// ---------------------------------------------------------------------------

BT::NodeStatus WaitForDuration::onStart()
{
  double duration_sec = 1.0;
  if (auto res = getInput<double>("duration_sec"))
  {
    duration_sec = res.value();
  }

  duration_ = std::chrono::duration<double>(duration_sec);
  start_time_ = std::chrono::steady_clock::now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForDuration::onRunning()
{
  const auto elapsed = std::chrono::steady_clock::now() - start_time_;
  return elapsed >= duration_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
}

void WaitForDuration::onHalted()
{
  // Nothing to clean up; the timer is purely in-process.
}

// ---------------------------------------------------------------------------
// WaitForGpsFix
// ---------------------------------------------------------------------------

BT::NodeStatus WaitForGpsFix::onStart()
{
  double timeout_sec = 20.0;
  if (auto res = getInput<double>("timeout_sec"))
  {
    timeout_sec = res.value();
  }
  min_fix_type_ = 2;
  if (auto res = getInput<int>("min_fix_type"))
  {
    min_fix_type_ = res.value();
  }

  timeout_ = std::chrono::duration<double>(timeout_sec);
  start_time_ = std::chrono::steady_clock::now();

  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");
  uint8_t current_fix = 0;
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    current_fix = ctx->gps_fix_type;
  }

  if (static_cast<int>(current_fix) >= min_fix_type_)
  {
    RCLCPP_INFO(ctx->node->get_logger(),
                "WaitForGpsFix: already at fix_type=%u (>= %d), proceeding",
                current_fix,
                min_fix_type_);
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(ctx->node->get_logger(),
              "WaitForGpsFix: waiting up to %.1fs for fix_type >= %d (current=%u)",
              timeout_sec,
              min_fix_type_,
              current_fix);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForGpsFix::onRunning()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  uint8_t current_fix = 0;
  {
    std::lock_guard<std::mutex> lock(ctx->context_mutex);
    current_fix = ctx->gps_fix_type;
  }

  if (static_cast<int>(current_fix) >= min_fix_type_)
  {
    const auto waited = std::chrono::steady_clock::now() - start_time_;
    const double waited_sec = std::chrono::duration<double>(waited).count();
    RCLCPP_INFO(ctx->node->get_logger(),
                "WaitForGpsFix: fix_type=%u reached after %.1fs",
                current_fix,
                waited_sec);
    return BT::NodeStatus::SUCCESS;
  }

  const auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed >= timeout_)
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "WaitForGpsFix: timeout after %.1fs (last fix_type=%u < %d) — "
                "proceeding at degraded GPS quality",
                std::chrono::duration<double>(timeout_).count(),
                current_fix,
                min_fix_type_);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void WaitForGpsFix::onHalted()
{
  // Nothing to clean up.
}

// ---------------------------------------------------------------------------
// SaveObstacles
// ---------------------------------------------------------------------------

BT::NodeStatus SaveObstacles::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!client_)
  {
    client_ = ctx->node->create_client<std_srvs::srv::Trigger>("/obstacle_tracker/save_obstacles");
  }

  if (!client_->service_is_ready())
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "SaveObstacles: service unavailable, skipping (no obstacle tracker running)");
    return BT::NodeStatus::SUCCESS;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client_->async_send_request(request);

  RCLCPP_INFO(ctx->node->get_logger(), "SaveObstacles: save request sent");
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// ResetEmergency
// ---------------------------------------------------------------------------

BT::NodeStatus ResetEmergency::tick()
{
  auto ctx = config().blackboard->get<std::shared_ptr<BTContext>>("context");

  if (!client_)
  {
    client_ = ctx->node->create_client<mowgli_interfaces::srv::EmergencyStop>(
        "/hardware_bridge/emergency_stop");
  }

  if (!waitForService(client_, ctx->node))
  {
    RCLCPP_WARN(ctx->node->get_logger(),
                "ResetEmergency: emergency_stop service unavailable, continuing");
    return BT::NodeStatus::SUCCESS;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::EmergencyStop::Request>();
  request->emergency = 0u;  // Release emergency

  // Fire-and-forget: the firmware is the safety authority and will only
  // clear the latch if no physical trigger (lift/stop) is still asserted.
  auto future = client_->async_send_request(request);
  (void)future;

  RCLCPP_INFO(ctx->node->get_logger(), "ResetEmergency: emergency release requested");

  return BT::NodeStatus::SUCCESS;
}

}  // namespace mowgli_behavior
