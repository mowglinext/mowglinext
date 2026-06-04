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
 * @file localization_monitor_node.hpp
 * @brief Localization quality monitor node.
 *
 * Watches the availability and quality of each localization source and
 * publishes a human-readable mode string that the behaviour tree and safety
 * monitors can use to gate mowing operations.
 *
 * Localization modes (ordered best to worst):
 *
 *   "RTK_FIXED"       – RTK fixed. ~2 cm absolute.
 *   "RTK_FLOAT"       – RTK float / DGPS. ~5–20 cm.
 *   "GPS_ONLY"        – Single / unaugmented GPS.
 *   "DEAD_RECKONING"  – All sources degraded / timed out. Return to dock.
 *
 * Subscribed topics:
 *   /wheel_odom               nav_msgs/msg/Odometry       (wheel odometry alive-check)
 *   /gps/absolute_pose        mowgli_interfaces/msg/AbsolutePose
 *
 * Published topics:
 *   /localization/mode        std_msgs/msg/String   (latched, 10 Hz)
 *   /localization/mode_id     std_msgs/msg/Int32    (integer enum for BT use)
 *
 * Parameters:
 *   gps_timeout    (double, default 2.0 s) – declare GPS stale after this gap.
 *   pose_timeout   (double, default 0.5 s) – declare wheel odom stale.
 *   publish_rate   (double, default 10.0 Hz)
 */

#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "mowgli_interfaces/msg/absolute_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace mowgli_localization
{

/// Integer IDs for each localization mode (published on /localization/mode_id).
enum class LocalizationMode : int32_t
{
  DEAD_RECKONING = 0,
  GPS_ONLY = 1,
  RTK_FLOAT = 2,
  RTK_FIXED = 3,
};

class LocalizationMonitorNode : public rclcpp::Node
{
public:
  explicit LocalizationMonitorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~LocalizationMonitorNode() override = default;

private:
  // ---------------------------------------------------------------------------
  // Initialisation helpers
  // ---------------------------------------------------------------------------
  void declare_parameters();
  void create_publishers();
  void create_subscribers();
  void create_timer();

  // ---------------------------------------------------------------------------
  // Callbacks
  // ---------------------------------------------------------------------------
  void on_wheel_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void on_absolute_pose(mowgli_interfaces::msg::AbsolutePose::ConstSharedPtr msg);

  // ---------------------------------------------------------------------------
  // Timer callback – evaluates state and publishes
  // ---------------------------------------------------------------------------
  void on_publish_timer();

  // ---------------------------------------------------------------------------
  // Internal helpers
  // ---------------------------------------------------------------------------

  /**
   * @brief Evaluate the current localization mode from source freshness / quality.
   */
  LocalizationMode evaluate_mode() const;

  /**
   * @brief Convert a LocalizationMode enum to its string representation.
   */
  static std::string mode_to_string(LocalizationMode mode);

  /**
   * @brief Return true if the given time_point is within timeout seconds of now.
   */
  bool is_fresh(rclcpp::Time last_stamp, double timeout_sec) const;

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------
  double gps_timeout_{2.0};
  double pose_timeout_{0.5};
  double publish_rate_{10.0};
  /// Hysteresis: a changed localization mode must persist at least this long
  /// before it is committed/published. Filters the spurious per-epoch RTK
  /// Fixed<->Float flicker (the receiver's carrSoln can toggle every epoch
  /// during motion while the position stays sub-cm), which otherwise flaps
  /// the published mode. 0 disables the debounce.
  double mode_debounce_sec_{1.0};

  // ---------------------------------------------------------------------------
  // Source state
  // ---------------------------------------------------------------------------
  rclcpp::Time last_wheel_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gps_stamp_{0, 0, RCL_ROS_TIME};

  /// True when the last GPS message carried an RTK-fixed or RTK-float flag.
  bool gps_rtk_active_{false};
  /// True when fixed (vs. float).
  bool gps_rtk_fixed_{false};

  // ---------------------------------------------------------------------------
  // Debounce state (committed mode published; a differing instantaneous mode
  // must persist >= mode_debounce_sec_ before it is committed).
  // ---------------------------------------------------------------------------
  LocalizationMode stable_mode_{LocalizationMode::DEAD_RECKONING};
  LocalizationMode pending_mode_{LocalizationMode::DEAD_RECKONING};
  rclcpp::Time pending_since_{0, 0, RCL_ROS_TIME};

  // ---------------------------------------------------------------------------
  // ROS handles
  // ---------------------------------------------------------------------------
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_id_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<mowgli_interfaces::msg::AbsolutePose>::SharedPtr abs_pose_sub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace mowgli_localization
