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
 * @file diagnostics_node.hpp
 * @brief DiagnosticsNode: aggregates robot health information and publishes
 *        standard ROS2 diagnostic_msgs at a configurable rate.
 *
 * Monitored subsystems:
 *   - Hardware Bridge   (freshness of /status)
 *   - Emergency System  (/emergency active/latched state)
 *   - Battery           (/power voltage + percentage estimate)
 *   - IMU               (freshness of /imu/data_raw)
 *   - LiDAR             (freshness of /scan)
 *   - GPS               (/gps/fix quality + satellite count)
 *   - Odometry          (freshness of /wheel_odom)
 *   - EKF Map           (/odometry/filtered_map: rate, position, orientation, z-drift, flat check)
 *   - Motors            (ESC temperatures from /status)
 */

#ifndef MOWGLI_MONITORING__DIAGNOSTICS_NODE_HPP_
#define MOWGLI_MONITORING__DIAGNOSTICS_NODE_HPP_

#include <chrono>
#include <optional>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "mowgli_interfaces/msg/emergency.hpp"
#include "mowgli_interfaces/msg/power.hpp"
#include "mowgli_interfaces/msg/status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace mowgli_monitoring
{

/// Shorthand for the diagnostic level constants.
using DiagLevel = diagnostic_msgs::msg::DiagnosticStatus;

/**
 * @brief Convenience struct that captures the last-received snapshot of each
 *        monitored subsystem together with its reception timestamp.
 *
 * All timestamps use the node's clock so that tests can inject a simulated
 * clock without touching wall time.
 */
struct DiagnosticsState
{
  // Hardware Bridge / Status
  std::optional<mowgli_interfaces::msg::Status> last_status{};
  rclcpp::Time last_status_time{0, 0, RCL_ROS_TIME};

  // Emergency
  std::optional<mowgli_interfaces::msg::Emergency> last_emergency{};

  // Power / Battery
  std::optional<mowgli_interfaces::msg::Power> last_power{};

  // IMU
  rclcpp::Time last_imu_time{0, 0, RCL_ROS_TIME};
  bool imu_ever_received{false};

  // LiDAR
  rclcpp::Time last_scan_time{0, 0, RCL_ROS_TIME};
  bool scan_ever_received{false};

  // Odometry (wheel)
  rclcpp::Time last_odom_time{0, 0, RCL_ROS_TIME};
  bool odom_ever_received{false};

  // robot_localization (ekf_map_node output)
  std::optional<nav_msgs::msg::Odometry> last_fusion_odom{};
  rclcpp::Time last_fusion_time{0, 0, RCL_ROS_TIME};
  bool fusion_ever_received{false};
  uint64_t fusion_msg_count{0};
  rclcpp::Time fusion_count_window_start{0, 0, RCL_ROS_TIME};
  uint64_t fusion_count_window{0};
  double fusion_rate_hz{0.0};

  // GPS
  std::optional<sensor_msgs::msg::NavSatFix> last_gps{};
  rclcpp::Time last_gps_time{0, 0, RCL_ROS_TIME};
  bool gps_ever_received{false};
};

/**
 * @brief Classifies a data-freshness duration against warn/error thresholds.
 *
 * @param age_sec   Age of the last received sample in seconds.
 * @param never     True when no sample has ever been received.
 * @param warn_sec  Age above which the status becomes WARN.
 * @param error_sec Age above which the status becomes ERROR.
 * @return One of DiagLevel::{OK, WARN, ERROR}.
 */
uint8_t classify_freshness(double age_sec, bool never, double warn_sec, double error_sec);

/**
 * @brief Classifies a battery percentage against configurable thresholds.
 *
 * @param percentage  Battery percentage in [0, 100].
 * @param warn_pct    Percentage below which the status becomes WARN.
 * @param error_pct   Percentage below which the status becomes ERROR.
 * @return One of DiagLevel::{OK, WARN, ERROR}.
 */
uint8_t classify_battery(double percentage, double warn_pct, double error_pct);

/**
 * @brief Classifies a temperature reading against configurable thresholds.
 *
 * @param temp_c     Temperature in degrees Celsius.
 * @param warn_c     Temperature above which status becomes WARN.
 * @param error_c    Temperature above which status becomes ERROR.
 * @return One of DiagLevel::{OK, WARN, ERROR}.
 */
uint8_t classify_temperature(double temp_c, double warn_c, double error_c);

/**
 * @brief Converts a numeric diagnostic level to its human-readable name.
 */
std::string level_name(uint8_t level);

/**
 * @brief DiagnosticsNode
 *
 * Aggregates robot health and publishes /diagnostics at 1 Hz (configurable).
 * All subscriptions use best-effort, sensor-data QoS where appropriate.
 *
 * Parameters
 * ----------
 * publish_rate         double  1.0    Hz — diagnostics publish rate
 * freshness_warn_sec   double  5.0    s  — default freshness WARN threshold
 * freshness_error_sec  double  10.0   s  — default freshness ERROR threshold
 * battery_warn_pct     double  20.0   %  — battery WARN level
 * battery_error_pct    double  10.0   %  — battery ERROR level
 * motor_temp_warn_c    double  60.0   °C — ESC/motor WARN temperature
 * motor_temp_error_c   double  80.0   °C — ESC/motor ERROR temperature
 * lidar_enabled        bool    false     — gate the LiDAR /scan health check; when
 *                                          false, report OK "LiDAR disabled"
 */
class DiagnosticsNode : public rclcpp::Node
{
public:
  explicit DiagnosticsNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~DiagnosticsNode() override = default;

  // Public only for unit-test access — do not call from production code.
  diagnostic_msgs::msg::DiagnosticStatus check_hardware_bridge(const rclcpp::Time& now) const;
  diagnostic_msgs::msg::DiagnosticStatus check_emergency() const;
  diagnostic_msgs::msg::DiagnosticStatus check_battery() const;
  diagnostic_msgs::msg::DiagnosticStatus check_imu(const rclcpp::Time& now) const;
  diagnostic_msgs::msg::DiagnosticStatus check_lidar(const rclcpp::Time& now) const;
  diagnostic_msgs::msg::DiagnosticStatus check_gps(const rclcpp::Time& now) const;
  diagnostic_msgs::msg::DiagnosticStatus check_odometry(const rclcpp::Time& now) const;
  diagnostic_msgs::msg::DiagnosticStatus check_fusion(const rclcpp::Time& now) const;
  diagnostic_msgs::msg::DiagnosticStatus check_motors() const;

  /// Read-only access to the internal state snapshot (for tests).
  const DiagnosticsState& state() const
  {
    return state_;
  }

private:
  // ---- Initialisation helpers -----------------------------------------------

  void declare_parameters();
  void create_subscriptions();
  void create_publishers();
  void create_timer();

  // ---- Subscription callbacks -----------------------------------------------

  void on_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg);
  void on_emergency(mowgli_interfaces::msg::Emergency::ConstSharedPtr msg);
  void on_power(mowgli_interfaces::msg::Power::ConstSharedPtr msg);
  void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
  void on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void on_fusion_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void on_gps(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

  // ---- Timer callback -------------------------------------------------------

  void publish_diagnostics();

  // ---- Helpers --------------------------------------------------------------

  /// Build a KeyValue pair.
  static diagnostic_msgs::msg::KeyValue kv(const std::string& key, const std::string& value);

  /// Format a float with a fixed number of decimal places.
  static std::string fmt_float(double value, int decimals = 2);

  // ---- ROS2 interfaces ------------------------------------------------------

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pub_diagnostics_;

  rclcpp::Subscription<mowgli_interfaces::msg::Status>::SharedPtr sub_status_;
  rclcpp::Subscription<mowgli_interfaces::msg::Emergency>::SharedPtr sub_emergency_;
  rclcpp::Subscription<mowgli_interfaces::msg::Power>::SharedPtr sub_power_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_fusion_odom_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ---- Parameters -----------------------------------------------------------

  double publish_rate_{1.0};
  double freshness_warn_sec_{5.0};
  double freshness_error_sec_{10.0};
  double battery_warn_pct_{20.0};
  double battery_error_pct_{10.0};
  double motor_temp_warn_c_{60.0};
  double motor_temp_error_c_{80.0};
  bool lidar_enabled_{false};

  // ---- State snapshot -------------------------------------------------------

  DiagnosticsState state_;
};

}  // namespace mowgli_monitoring

#endif  // MOWGLI_MONITORING__DIAGNOSTICS_NODE_HPP_
