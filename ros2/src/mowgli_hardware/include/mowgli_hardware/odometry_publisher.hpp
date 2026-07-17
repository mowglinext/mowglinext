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
 * @file odometry_publisher.hpp
 * @brief Node-owned helper: turns STM32 LlOdometry wire packets into
 *        ~/wheel_ticks (per-packet diagnostics) and ~/wheel_odom
 *        (aggregated, ~20 Hz) publishes.
 *
 * Extracted from hardware_bridge_node's former handle_odometry() as part of
 * the god-node breakup (2616-line TU, see CLAUDE.md task board #11).
 * Behavior-preserving: the packet-handling logic below is a verbatim port —
 * see the .cpp for the detailed rationale comments (16-bit encoder-wrap
 * recovery, tick-spike rejection, 50 ms aggregation window, dock-charging
 * force-zero + covariance policy).
 */

#pragma once

#include <cstdint>

#include "mowgli_hardware/clock_fit.hpp"
#include "mowgli_hardware/ll_datatypes.hpp"
#include "mowgli_interfaces/msg/wheel_tick.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_hardware
{

class OdometryPublisher
{
public:
  /// Creates its own ~/wheel_odom and ~/wheel_ticks publishers on @p node.
  explicit OdometryPublisher(rclcpp::Node& node);

  /// Drop in-flight tick/aggregation state. Call on serial (re)connect —
  /// mirrors hardware_bridge_node's reset_serial_dependent_state().
  void reset();

  /**
   * @brief Process one decoded LlOdometry packet; publishes WheelTick every
   *        call and Odometry once per ~50 ms aggregation window.
   *
   * @param pkt             Decoded LlOdometry wire packet.
   * @param ticks_per_meter Live-tunable encoder scale (ROS param, may
   *                        change between calls).
   * @param wheel_track     Live-tunable wheel separation (ROS param, may
   *                        change between calls).
   * @param is_charging     True while dock charging contacts are live —
   *                        forces the published velocity (and its
   *                        covariance) to reflect certain-stationary.
   */
  void handle_packet(const LlOdometry& pkt, double ticks_per_meter, double wheel_track,
                      bool is_charging);

  /// True once the most recent aggregation window saw zero net wheel ticks.
  [[nodiscard]] bool wheels_stationary() const
  {
    return wheels_stationary_;
  }

private:
  rclcpp::Node& node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_wheel_odom_;
  rclcpp::Publisher<mowgli_interfaces::msg::WheelTick>::SharedPtr pub_wheel_ticks_;

  HostFirmwareClockFit odom_clock_fit_;

  bool odom_initialized_{false};
  int32_t prev_left_ticks_{0};
  int32_t prev_right_ticks_{0};
  int32_t odom_acc_delta_left_{0};
  int32_t odom_acc_delta_right_{0};
  uint32_t odom_acc_dt_ms_{0};
  bool wheels_stationary_{true};

  // Per-wheel cumulative-magnitude tick counters + last direction (for
  // WheelTick). Magnitude is monotonic-up; direction is 1=fwd/0=rev.
  uint32_t wheel_ticks_mag_left_{0};
  uint32_t wheel_ticks_mag_right_{0};
  uint8_t wheel_dir_left_{1};
  uint8_t wheel_dir_right_{1};

  // Debug log throttle (mirrors the original's `static int` local).
  int odom_debug_count_{0};
};

}  // namespace mowgli_hardware
