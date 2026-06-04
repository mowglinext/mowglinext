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
 * @file localization_monitor_node.cpp
 * @brief Localization quality monitor implementation.
 *
 * Mode decision logic (evaluated every publish_rate_ Hz):
 *
 *   RTK_FIXED      ← GPS fresh AND rtk_fixed
 *   RTK_FLOAT      ← GPS fresh AND rtk_active (float or DGPS)
 *   GPS_ONLY       ← GPS fresh (unaugmented)
 *   DEAD_RECKONING ← everything else
 *
 * The mode string is also logged at INFO level whenever it changes so the
 * transition is visible in `ros2 topic echo` and in bag files.
 */

#include "mowgli_localization/localization_monitor_node.hpp"

#include <chrono>
#include <string>

#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace mowgli_localization
{

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

LocalizationMonitorNode::LocalizationMonitorNode(const rclcpp::NodeOptions& options)
    : Node("localization_monitor", options)
{
  declare_parameters();
  create_publishers();
  create_subscribers();
  create_timer();

  RCLCPP_INFO(get_logger(),
              "LocalizationMonitorNode started — gps_timeout=%.1f s, "
              "pose_timeout=%.1f s, publish_rate=%.1f Hz",
              gps_timeout_,
              pose_timeout_,
              publish_rate_);
}

// ---------------------------------------------------------------------------
// Initialisation helpers
// ---------------------------------------------------------------------------

void LocalizationMonitorNode::declare_parameters()
{
  gps_timeout_ = declare_parameter<double>("gps_timeout", 2.0);
  pose_timeout_ = declare_parameter<double>("pose_timeout", 0.5);
  publish_rate_ = declare_parameter<double>("publish_rate", 10.0);
  mode_debounce_sec_ = declare_parameter<double>("mode_debounce_sec", 1.0);
}

void LocalizationMonitorNode::create_publishers()
{
  // Latch-like behaviour for string topic: use transient_local so late
  // subscribers always receive the current mode.
  const auto latched_qos = rclcpp::QoS(1).transient_local();

  mode_pub_ = create_publisher<std_msgs::msg::String>("/mowgli/localization/mode", latched_qos);
  mode_id_pub_ =
      create_publisher<std_msgs::msg::Int32>("/mowgli/localization/mode_id", latched_qos);
}

void LocalizationMonitorNode::create_subscribers()
{
  wheel_odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>("/wheel_odom",
                                                   rclcpp::QoS(10),
                                                   [this](
                                                       nav_msgs::msg::Odometry::ConstSharedPtr msg)
                                                   {
                                                     on_wheel_odom(msg);
                                                   });

  abs_pose_sub_ = create_subscription<mowgli_interfaces::msg::AbsolutePose>(
      "/gps/absolute_pose",
      rclcpp::QoS(10),
      [this](mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
      {
        on_absolute_pose(msg);
      });
}

void LocalizationMonitorNode::create_timer()
{
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));

  publish_timer_ = create_wall_timer(period_ns,
                                     [this]()
                                     {
                                       on_publish_timer();
                                     });
}

// ---------------------------------------------------------------------------
// Subscription callbacks
// ---------------------------------------------------------------------------

void LocalizationMonitorNode::on_wheel_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  last_wheel_odom_stamp_ = msg->header.stamp;
}

void LocalizationMonitorNode::on_absolute_pose(
    mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg)
{
  using Flags = mowgli_interfaces::msg::AbsolutePose;

  last_gps_stamp_ = msg->header.stamp;

  gps_rtk_fixed_ = (msg->flags & Flags::FLAG_GPS_RTK_FIXED) != 0u;
  gps_rtk_active_ = gps_rtk_fixed_ || ((msg->flags & Flags::FLAG_GPS_RTK_FLOAT) != 0u) ||
                    ((msg->flags & Flags::FLAG_GPS_RTK) != 0u);
}

// ---------------------------------------------------------------------------
// Timer callback
// ---------------------------------------------------------------------------

void LocalizationMonitorNode::on_publish_timer()
{
  const LocalizationMode instant = evaluate_mode();

  // Hysteresis (mode_debounce_sec_): only commit a changed mode once the new
  // value has persisted continuously for the debounce window. This filters the
  // spurious per-epoch RTK Fixed<->Float flicker (the receiver's carrSoln can
  // toggle every epoch during motion while position σ stays sub-cm) so the
  // published localization mode — and anything gating on it — does not flap.
  // mode_debounce_sec_ <= 0 disables the hysteresis (commit immediately).
  const rclcpp::Time now = this->now();
  if (instant == stable_mode_)
  {
    // Back to the committed mode — cancel any pending change.
    pending_mode_ = stable_mode_;
  }
  else if (instant != pending_mode_)
  {
    // New candidate — start its dwell timer.
    pending_mode_ = instant;
    pending_since_ = now;
  }
  else if (mode_debounce_sec_ <= 0.0 || (now - pending_since_).seconds() >= mode_debounce_sec_)
  {
    // Candidate has dwelled long enough — commit it.
    RCLCPP_INFO(get_logger(),
                "Localization mode change: %s → %s",
                mode_to_string(stable_mode_).c_str(),
                mode_to_string(instant).c_str());
    stable_mode_ = instant;
  }

  std_msgs::msg::String mode_msg;
  mode_msg.data = mode_to_string(stable_mode_);
  mode_pub_->publish(mode_msg);

  std_msgs::msg::Int32 id_msg;
  id_msg.data = static_cast<int32_t>(stable_mode_);
  mode_id_pub_->publish(id_msg);
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

LocalizationMode LocalizationMonitorNode::evaluate_mode() const
{
  const bool gps_fresh = is_fresh(last_gps_stamp_, gps_timeout_);

  // Ordered from best to worst; first matching rule wins.

  if (gps_fresh && gps_rtk_fixed_)
  {
    return LocalizationMode::RTK_FIXED;
  }

  if (gps_fresh && gps_rtk_active_)
  {
    return LocalizationMode::RTK_FLOAT;
  }

  if (gps_fresh)
  {
    return LocalizationMode::GPS_ONLY;
  }

  return LocalizationMode::DEAD_RECKONING;
}

std::string LocalizationMonitorNode::mode_to_string(const LocalizationMode mode)
{
  switch (mode)
  {
    case LocalizationMode::RTK_FIXED:
      return "RTK_FIXED";
    case LocalizationMode::RTK_FLOAT:
      return "RTK_FLOAT";
    case LocalizationMode::GPS_ONLY:
      return "GPS_ONLY";
    case LocalizationMode::DEAD_RECKONING:
      return "DEAD_RECKONING";
    default:
      return "UNKNOWN";
  }
}

bool LocalizationMonitorNode::is_fresh(const rclcpp::Time last_stamp,
                                       const double timeout_sec) const
{
  // A stamp of {0,0} means we have never received a message.
  if (last_stamp.nanoseconds() == 0)
  {
    return false;
  }

  const rclcpp::Duration age = now() - last_stamp;
  return age.seconds() < timeout_sec;
}

}  // namespace mowgli_localization

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_localization::LocalizationMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
