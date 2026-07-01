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
 * @file diagnostics_node.cpp
 * @brief Implementation of DiagnosticsNode — robot health aggregator.
 */

#include "mowgli_monitoring/diagnostics_node.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace mowgli_monitoring
{

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Free-function utilities (declared in the header, tested directly)
// ---------------------------------------------------------------------------

uint8_t classify_freshness(double age_sec, bool never, double warn_sec, double error_sec)
{
  if (never || age_sec >= error_sec)
  {
    return DiagLevel::ERROR;
  }
  if (age_sec >= warn_sec)
  {
    return DiagLevel::WARN;
  }
  return DiagLevel::OK;
}

uint8_t classify_battery(double percentage, double warn_pct, double error_pct)
{
  if (percentage <= error_pct)
  {
    return DiagLevel::ERROR;
  }
  if (percentage <= warn_pct)
  {
    return DiagLevel::WARN;
  }
  return DiagLevel::OK;
}

uint8_t classify_temperature(double temp_c, double warn_c, double error_c)
{
  if (temp_c >= error_c)
  {
    return DiagLevel::ERROR;
  }
  if (temp_c >= warn_c)
  {
    return DiagLevel::WARN;
  }
  return DiagLevel::OK;
}

std::string level_name(uint8_t level)
{
  switch (level)
  {
    case DiagLevel::OK:
      return "OK";
    case DiagLevel::WARN:
      return "WARN";
    case DiagLevel::ERROR:
      return "ERROR";
    case DiagLevel::STALE:
      return "STALE";
    default:
      return "UNKNOWN";
  }
}

// ---------------------------------------------------------------------------
// DiagnosticsNode
// ---------------------------------------------------------------------------

DiagnosticsNode::DiagnosticsNode(const rclcpp::NodeOptions& options)
    : Node("diagnostics_node", options)
{
  declare_parameters();
  create_subscriptions();
  create_publishers();
  create_timer();

  RCLCPP_INFO(get_logger(),
              "DiagnosticsNode started. publish_rate=%.1f Hz  "
              "freshness_warn=%.1fs  freshness_error=%.1fs",
              publish_rate_,
              freshness_warn_sec_,
              freshness_error_sec_);
}

// ---------------------------------------------------------------------------
// Initialisation helpers
// ---------------------------------------------------------------------------

void DiagnosticsNode::declare_parameters()
{
  publish_rate_ = declare_parameter<double>("publish_rate", 1.0);
  freshness_warn_sec_ = declare_parameter<double>("freshness_warn_sec", 5.0);
  freshness_error_sec_ = declare_parameter<double>("freshness_error_sec", 10.0);
  battery_warn_pct_ = declare_parameter<double>("battery_warn_pct", 20.0);
  battery_error_pct_ = declare_parameter<double>("battery_error_pct", 10.0);
  motor_temp_warn_c_ = declare_parameter<double>("motor_temp_warn_c", 60.0);
  motor_temp_error_c_ = declare_parameter<double>("motor_temp_error_c", 80.0);
  lidar_enabled_ = declare_parameter<bool>("lidar_enabled", false);

  // Clamp publish_rate to [0.1, 100.0] Hz to prevent zero-division.
  if (publish_rate_ < 0.1 || publish_rate_ > 100.0)
  {
    RCLCPP_WARN(get_logger(),
                "publish_rate=%.2f is out of range [0.1, 100.0]. Clamping to 1.0 Hz.",
                publish_rate_);
    publish_rate_ = 1.0;
  }
}

void DiagnosticsNode::create_subscriptions()
{
  // Hardware status — use sensor QoS (best-effort, keep-last-1).
  const auto sensor_qos = rclcpp::SensorDataQoS();

  sub_status_ = create_subscription<mowgli_interfaces::msg::Status>(
      "/hardware_bridge/status",
      10,
      [this](mowgli_interfaces::msg::Status::ConstSharedPtr msg)
      {
        on_status(msg);
      });

  sub_emergency_ = create_subscription<mowgli_interfaces::msg::Emergency>(
      "/hardware_bridge/emergency",
      10,
      [this](mowgli_interfaces::msg::Emergency::ConstSharedPtr msg)
      {
        on_emergency(msg);
      });

  sub_power_ = create_subscription<mowgli_interfaces::msg::Power>(
      "/hardware_bridge/power",
      10,
      [this](mowgli_interfaces::msg::Power::ConstSharedPtr msg)
      {
        on_power(msg);
      });

  sub_imu_ =
      create_subscription<sensor_msgs::msg::Imu>("/imu/data",
                                                 sensor_qos,
                                                 [this](sensor_msgs::msg::Imu::ConstSharedPtr msg)
                                                 {
                                                   on_imu(msg);
                                                 });

  sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      sensor_qos,
      [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
      {
        on_scan(msg);
      });

  sub_odom_ =
      create_subscription<nav_msgs::msg::Odometry>("/wheel_odom",
                                                   sensor_qos,
                                                   [this](
                                                       nav_msgs::msg::Odometry::ConstSharedPtr msg)
                                                   {
                                                     on_odom(msg);
                                                   });

  sub_fusion_odom_ =
      create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered_map",
                                                   sensor_qos,
                                                   [this](
                                                       nav_msgs::msg::Odometry::ConstSharedPtr msg)
                                                   {
                                                     on_fusion_odom(msg);
                                                   });

  sub_gps_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix",
      sensor_qos,
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
      {
        on_gps(msg);
      });
}

void DiagnosticsNode::create_publishers()
{
  pub_diagnostics_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
}

void DiagnosticsNode::create_timer()
{
  const auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
  timer_ = create_wall_timer(period_ms,
                             [this]()
                             {
                               publish_diagnostics();
                             });
}

// ---------------------------------------------------------------------------
// Subscription callbacks
// ---------------------------------------------------------------------------

void DiagnosticsNode::on_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg)
{
  state_.last_status = *msg;
  state_.last_status_time = now();
}

void DiagnosticsNode::on_emergency(mowgli_interfaces::msg::Emergency::ConstSharedPtr msg)
{
  state_.last_emergency = *msg;
}

void DiagnosticsNode::on_power(mowgli_interfaces::msg::Power::ConstSharedPtr msg)
{
  state_.last_power = *msg;
}

void DiagnosticsNode::on_imu(sensor_msgs::msg::Imu::ConstSharedPtr /*msg*/)
{
  state_.last_imu_time = now();
  state_.imu_ever_received = true;
}

void DiagnosticsNode::on_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr /*msg*/)
{
  state_.last_scan_time = now();
  state_.scan_ever_received = true;
}

void DiagnosticsNode::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr /*msg*/)
{
  state_.last_odom_time = now();
  state_.odom_ever_received = true;
}

void DiagnosticsNode::on_fusion_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const auto t = now();
  state_.last_fusion_odom = *msg;
  state_.last_fusion_time = t;
  state_.fusion_ever_received = true;
  state_.fusion_msg_count++;

  // Compute rate over a 5-second sliding window
  const double window = (t - state_.fusion_count_window_start).seconds();
  state_.fusion_count_window++;
  if (window >= 5.0)
  {
    state_.fusion_rate_hz = static_cast<double>(state_.fusion_count_window) / window;
    state_.fusion_count_window = 0;
    state_.fusion_count_window_start = t;
  }
}

void DiagnosticsNode::on_gps(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  state_.last_gps = *msg;
  state_.last_gps_time = now();
  state_.gps_ever_received = true;
}

// ---------------------------------------------------------------------------
// Timer callback
// ---------------------------------------------------------------------------

void DiagnosticsNode::publish_diagnostics()
{
  const rclcpp::Time t = now();

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = t;

  array.status.push_back(check_hardware_bridge(t));
  array.status.push_back(check_emergency());
  array.status.push_back(check_battery());
  array.status.push_back(check_imu(t));
  array.status.push_back(check_lidar(t));
  array.status.push_back(check_gps(t));
  array.status.push_back(check_odometry(t));
  array.status.push_back(check_fusion(t));
  array.status.push_back(check_motors());

  pub_diagnostics_->publish(array);
}

// ---------------------------------------------------------------------------
// Diagnostic check functions
// ---------------------------------------------------------------------------

diagnostic_msgs::msg::DiagnosticStatus DiagnosticsNode::check_hardware_bridge(
    const rclcpp::Time& now) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "Hardware Bridge";
  status.hardware_id = "mowgli/hardware_bridge";

  if (!state_.last_status.has_value())
  {
    status.level = DiagLevel::ERROR;
    status.message = "No status received";
    return status;
  }

  const double age_sec = (now - state_.last_status_time).seconds();

  status.level = classify_freshness(age_sec, false, freshness_warn_sec_, freshness_error_sec_);
  status.message = "Last status age: " + fmt_float(age_sec, 1) + "s";

  status.values.push_back(kv("age_sec", fmt_float(age_sec, 2)));
  status.values.push_back(kv("mower_status", std::to_string(state_.last_status->mower_status)));
  status.values.push_back(kv("is_charging", state_.last_status->is_charging ? "true" : "false"));
  status.values.push_back(kv("mow_enabled", state_.last_status->mow_enabled ? "true" : "false"));

  return status;
}

diagnostic_msgs::msg::DiagnosticStatus DiagnosticsNode::check_emergency() const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "Emergency System";
  status.hardware_id = "mowgli/emergency";

  if (!state_.last_emergency.has_value())
  {
    status.level = DiagLevel::WARN;
    status.message = "No emergency message received yet";
    return status;
  }

  const auto& e = *state_.last_emergency;

  if (e.active_emergency)
  {
    status.level = DiagLevel::ERROR;
    status.message = "EMERGENCY ACTIVE: " + e.reason;
  }
  else if (e.latched_emergency)
  {
    status.level = DiagLevel::WARN;
    status.message = "Emergency latched (not yet cleared): " + e.reason;
  }
  else
  {
    status.level = DiagLevel::OK;
    status.message = "No emergency";
  }

  status.values.push_back(kv("active", e.active_emergency ? "true" : "false"));
  status.values.push_back(kv("latched", e.latched_emergency ? "true" : "false"));
  status.values.push_back(kv("reason", e.reason));

  return status;
}

diagnostic_msgs::msg::DiagnosticStatus DiagnosticsNode::check_battery() const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "Battery";
  status.hardware_id = "mowgli/battery";

  if (!state_.last_power.has_value())
  {
    status.level = DiagLevel::WARN;
    status.message = "No power message received yet";
    return status;
  }

  const auto& p = *state_.last_power;

  // The hardware does not expose a percentage directly; derive it from the
  // known LiPo 4S cell voltage range (full: 16.8V, empty: 12.0V).
  constexpr double kVFull = 16.8;
  constexpr double kVEmpty = 12.0;
  const double voltage = static_cast<double>(p.v_battery);

  double percentage = 100.0 * (voltage - kVEmpty) / (kVFull - kVEmpty);
  percentage = std::max(0.0, std::min(100.0, percentage));

  status.level = classify_battery(percentage, battery_warn_pct_, battery_error_pct_);
  status.message = fmt_float(percentage, 0) + "% (" + fmt_float(voltage, 2) + " V)";

  status.values.push_back(kv("voltage_v", fmt_float(voltage, 3)));
  status.values.push_back(kv("charge_voltage_v", fmt_float(static_cast<double>(p.v_charge), 3)));
  status.values.push_back(
      kv("charge_current_a", fmt_float(static_cast<double>(p.charge_current), 3)));
  status.values.push_back(kv("percentage", fmt_float(percentage, 1)));
  status.values.push_back(kv("charger_enabled", p.charger_enabled ? "true" : "false"));
  status.values.push_back(kv("charger_status", p.charger_status));

  return status;
}

diagnostic_msgs::msg::DiagnosticStatus DiagnosticsNode::check_imu(const rclcpp::Time& now) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "IMU";
  status.hardware_id = "mowgli/imu";

  if (!state_.imu_ever_received)
  {
    status.level = DiagLevel::ERROR;
    status.message = "No IMU data received";
    return status;
  }

  const double age_sec = (now - state_.last_imu_time).seconds();
  status.level = classify_freshness(age_sec, false, freshness_warn_sec_, freshness_error_sec_);
  status.message = "Last IMU age: " + fmt_float(age_sec, 1) + "s";
  status.values.push_back(kv("age_sec", fmt_float(age_sec, 2)));

  return status;
}

diagnostic_msgs::msg::DiagnosticStatus DiagnosticsNode::check_lidar(const rclcpp::Time& now) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "LiDAR";
  status.hardware_id = "mowgli/lidar";

  // When the LiDAR is intentionally disabled (GPS-only operation) there is no
  // /scan publisher, so don't raise a spurious "no scan" error.
  if (!lidar_enabled_)
  {
    status.level = DiagLevel::OK;
    status.message = "LiDAR disabled";
    return status;
  }

  if (!state_.scan_ever_received)
  {
    status.level = DiagLevel::ERROR;
    status.message = "No LiDAR scan received";
    return status;
  }

  const double age_sec = (now - state_.last_scan_time).seconds();
  status.level = classify_freshness(age_sec, false, freshness_warn_sec_, freshness_error_sec_);
  status.message = "Last scan age: " + fmt_float(age_sec, 1) + "s";
  status.values.push_back(kv("age_sec", fmt_float(age_sec, 2)));

  return status;
}

diagnostic_msgs::msg::DiagnosticStatus DiagnosticsNode::check_gps(const rclcpp::Time& now) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "GPS";
  status.hardware_id = "mowgli/gps";

  if (!state_.gps_ever_received || !state_.last_gps.has_value())
  {
    status.level = DiagLevel::WARN;
    status.message = "No GPS fix received";
    return status;
  }

  const auto& gps = *state_.last_gps;
  const double age_sec = (now - state_.last_gps_time).seconds();

  // Fix status codes from sensor_msgs/NavSatStatus:
  //   STATUS_NO_FIX = -1, STATUS_FIX = 0, STATUS_SBAS_FIX = 1, STATUS_GBAS_FIX = 2
  const bool has_fix = gps.status.status >= 0;

  if (!has_fix)
  {
    status.level = DiagLevel::WARN;
    status.message = "GPS: no fix";
  }
  else
  {
    status.level = classify_freshness(age_sec, false, freshness_warn_sec_, freshness_error_sec_);
    status.message =
        "GPS fix OK  lat=" + fmt_float(gps.latitude, 9) + "  lon=" + fmt_float(gps.longitude, 9);
  }

  status.values.push_back(kv("fix_status", std::to_string(gps.status.status)));
  status.values.push_back(kv("service", std::to_string(gps.status.service)));
  status.values.push_back(kv("latitude", fmt_float(gps.latitude, 9)));
  status.values.push_back(kv("longitude", fmt_float(gps.longitude, 9)));
  status.values.push_back(kv("altitude_m", fmt_float(gps.altitude, 2)));
  status.values.push_back(kv("age_sec", fmt_float(age_sec, 2)));

  return status;
}

diagnostic_msgs::msg::DiagnosticStatus DiagnosticsNode::check_odometry(
    const rclcpp::Time& now) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "Odometry";
  status.hardware_id = "mowgli/odometry";

  if (!state_.odom_ever_received)
  {
    status.level = DiagLevel::ERROR;
    status.message = "No odometry received";
    return status;
  }

  const double age_sec = (now - state_.last_odom_time).seconds();
  status.level = classify_freshness(age_sec, false, freshness_warn_sec_, freshness_error_sec_);
  status.message = "Last odometry age: " + fmt_float(age_sec, 1) + "s";
  status.values.push_back(kv("age_sec", fmt_float(age_sec, 2)));

  return status;
}

diagnostic_msgs::msg::DiagnosticStatus DiagnosticsNode::check_fusion(const rclcpp::Time& now) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "EKF Map";
  status.hardware_id = "robot_localization/filtered_map";

  if (!state_.fusion_ever_received)
  {
    status.level = DiagLevel::ERROR;
    status.message = "No /odometry/filtered_map received";
    return status;
  }

  const double age_sec = (now - state_.last_fusion_time).seconds();
  status.level = classify_freshness(age_sec, false, freshness_warn_sec_, freshness_error_sec_);

  const auto& odom = *state_.last_fusion_odom;
  const auto& pos = odom.pose.pose.position;
  const auto& q = odom.pose.pose.orientation;

  // Extract roll, pitch, yaw from quaternion
  const double sinr = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  const double roll_deg = std::atan2(sinr, cosr) * 180.0 / M_PI;

  const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
  const double pitch_deg =
      (std::abs(sinp) >= 1.0 ? std::copysign(90.0, sinp) : std::asin(sinp) * 180.0 / M_PI);

  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  const double yaw_deg = std::atan2(siny, cosy) * 180.0 / M_PI;

  // Check flat constraint (roll/pitch should be near zero for ground robot)
  const bool flat_ok = std::abs(roll_deg) < 5.0 && std::abs(pitch_deg) < 5.0;
  // Check z-drift (should be near zero in odom frame)
  const bool z_ok = std::abs(pos.z) < 2.0;

  if (status.level == DiagLevel::OK)
  {
    if (!flat_ok)
    {
      status.level = DiagLevel::WARN;
      status.message = "Roll/pitch drift detected";
    }
    else if (!z_ok)
    {
      status.level = DiagLevel::WARN;
      status.message = "Z-drift: " + fmt_float(pos.z, 2) + "m";
    }
    else
    {
      status.message = "OK (" + fmt_float(state_.fusion_rate_hz, 1) + " Hz)";
    }
  }
  else
  {
    status.message = "Stale (" + fmt_float(age_sec, 1) + "s ago)";
  }

  status.values.push_back(kv("rate_hz", fmt_float(state_.fusion_rate_hz, 1)));
  status.values.push_back(kv("age_sec", fmt_float(age_sec, 2)));
  status.values.push_back(kv("x_m", fmt_float(pos.x, 3)));
  status.values.push_back(kv("y_m", fmt_float(pos.y, 3)));
  status.values.push_back(kv("z_m", fmt_float(pos.z, 3)));
  status.values.push_back(kv("roll_deg", fmt_float(roll_deg, 2)));
  status.values.push_back(kv("pitch_deg", fmt_float(pitch_deg, 2)));
  status.values.push_back(kv("yaw_deg", fmt_float(yaw_deg, 2)));
  status.values.push_back(kv("flat_ok", flat_ok ? "true" : "false"));
  status.values.push_back(kv("z_drift_ok", z_ok ? "true" : "false"));
  status.values.push_back(kv("msg_count", std::to_string(state_.fusion_msg_count)));

  return status;
}

diagnostic_msgs::msg::DiagnosticStatus DiagnosticsNode::check_motors() const
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "Motors";
  status.hardware_id = "mowgli/motors";

  if (!state_.last_status.has_value())
  {
    status.level = DiagLevel::WARN;
    status.message = "No status message received";
    return status;
  }

  const auto& s = *state_.last_status;

  const double esc_temp = static_cast<double>(s.mower_esc_temperature);
  const double motor_temp = static_cast<double>(s.mower_motor_temperature);

  // Take the worst level across both temperature readings.
  const uint8_t esc_level = classify_temperature(esc_temp, motor_temp_warn_c_, motor_temp_error_c_);
  const uint8_t motor_level =
      classify_temperature(motor_temp, motor_temp_warn_c_, motor_temp_error_c_);
  const uint8_t worst = std::max(esc_level, motor_level);

  status.level = worst;

  if (worst == DiagLevel::ERROR)
  {
    status.message = "Motor/ESC temperature critical";
  }
  else if (worst == DiagLevel::WARN)
  {
    status.message = "Motor/ESC temperature elevated";
  }
  else
  {
    status.message = "Motors OK";
  }

  status.values.push_back(kv("esc_temperature_c", fmt_float(esc_temp, 1)));
  status.values.push_back(kv("motor_temperature_c", fmt_float(motor_temp, 1)));
  status.values.push_back(kv("mower_esc_status", std::to_string(s.mower_esc_status)));
  status.values.push_back(
      kv("mower_esc_current_a", fmt_float(static_cast<double>(s.mower_esc_current), 2)));
  status.values.push_back(kv("mower_rpm", fmt_float(static_cast<double>(s.mower_motor_rpm), 0)));

  return status;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

diagnostic_msgs::msg::KeyValue DiagnosticsNode::kv(const std::string& key, const std::string& value)
{
  diagnostic_msgs::msg::KeyValue pair;
  pair.key = key;
  pair.value = value;
  return pair;
}

std::string DiagnosticsNode::fmt_float(double value, int decimals)
{
  // Stack-allocated buffer; no dynamic allocation for this hot path.
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.*f", decimals, value);
  return std::string{buf};
}

}  // namespace mowgli_monitoring
