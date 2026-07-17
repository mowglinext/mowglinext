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
//
// Verbatim port of hardware_bridge_node's former handle_odometry() — see
// odometry_publisher.hpp for the extraction rationale. Every comment below
// that isn't marked otherwise is carried over unchanged from the original
// inline method; this file does not change behaviour.

#include "mowgli_hardware/odometry_publisher.hpp"

#include <cmath>
#include <cstdlib>

namespace mowgli_hardware
{

OdometryPublisher::OdometryPublisher(rclcpp::Node& node) : node_(node)
{
  pub_wheel_odom_ =
      node_.create_publisher<nav_msgs::msg::Odometry>("~/wheel_odom", rclcpp::QoS(10));
  // Per-wheel encoder ticks (diagnostics — GUI "Per-Wheel Encoders" panel).
  // 2-wheel diff-drive maps to RL/RR only. Remapped to /wheel_ticks in the
  // launch file (the GUI bridge subscribes to /wheel_ticks).
  pub_wheel_ticks_ =
      node_.create_publisher<mowgli_interfaces::msg::WheelTick>("~/wheel_ticks", rclcpp::QoS(10));
}

void OdometryPublisher::reset()
{
  odom_initialized_ = false;
  prev_left_ticks_ = 0;
  prev_right_ticks_ = 0;
  odom_acc_delta_left_ = 0;
  odom_acc_delta_right_ = 0;
  odom_acc_dt_ms_ = 0;
  wheels_stationary_ = true;
}

void OdometryPublisher::handle_packet(const LlOdometry& pkt, double ticks_per_meter,
                                       double wheel_track, bool is_charging)
{
  // Signed tick deltas since last firmware packet (polarity = direction).
  int32_t d_left = pkt.left_ticks - prev_left_ticks_;
  int32_t d_right = pkt.right_ticks - prev_right_ticks_;
  prev_left_ticks_ = pkt.left_ticks;
  prev_right_ticks_ = pkt.right_ticks;

  if (!odom_initialized_)
  {
    odom_initialized_ = true;
    return;
  }

  // 16-bit unsigned-counter wraparound recovery. Firmware packets carry
  // int32_t left_ticks / right_ticks but the underlying motor-controller
  // encoder counter is 16-bit and wraps 0xFFFF↔0x0000. After a wrap a
  // raw subtraction produces a delta of ±65535 (with small ±N noise from
  // the actual motion that occurred during the wrap), which is the
  // signature we observe ("dL=-65528", "dR=65535" etc.). Unwrap any
  // delta whose magnitude is closer to 65536 than to 0 by adding/
  // subtracting 65536 — this recovers the true small physical delta
  // instead of dropping the packet and losing position information.
  auto unwrap_16bit = [](int32_t d)
  {
    if (d > 32768)
      return d - 65536;
    if (d < -32768)
      return d + 65536;
    return d;
  };
  d_left = unwrap_16bit(d_left);
  d_right = unwrap_16bit(d_right);

  // Sanity-clamp residual implausible deltas. After wrap-recovery the
  // remaining oversize deltas are firmware glitches (e.g. motor-controller
  // encoder reset on direction change not in lockstep with the firmware's
  // own prev tracking). Drop those — at 21 ms packet period and a hard
  // 2 m/s upper bound the physical max is ~13 ticks; 100 leaves margin.
  constexpr int32_t kTickSpikeLimit = 100;
  if (std::abs(d_left) > kTickSpikeLimit || std::abs(d_right) > kTickSpikeLimit)
  {
    RCLCPP_WARN_THROTTLE(node_.get_logger(),
                         *node_.get_clock(),
                         2000,
                         "Dropping residual wheel tick spike: dL=%d dR=%d (limit=%d).",
                         d_left,
                         d_right,
                         kTickSpikeLimit);
    d_left = 0;
    d_right = 0;
  }

  // ----- Per-wheel WheelTick (diagnostics: GUI "Per-Wheel Encoders") -----
  // Published every firmware packet (~47 Hz). The mower is 2-wheel diff-drive,
  // so only the Rear-Left / Rear-Right slots are valid (left→RL, right→RR,
  // matching the wheel_odometry_node convention); Front-L/R stay invalid.
  // WheelTick wants MONOTONIC-up magnitude counts plus a direction byte
  // (consumers do (cur-prev)*sign(direction)), whereas the firmware sends
  // signed cumulative ticks — so accumulate |delta| and derive the direction
  // from the delta sign (holding the last direction through a zero delta).
  wheel_ticks_mag_left_ += static_cast<uint32_t>(std::abs(d_left));
  wheel_ticks_mag_right_ += static_cast<uint32_t>(std::abs(d_right));
  if (d_left > 0)
    wheel_dir_left_ = 1u;
  else if (d_left < 0)
    wheel_dir_left_ = 0u;
  if (d_right > 0)
    wheel_dir_right_ = 1u;
  else if (d_right < 0)
    wheel_dir_right_ = 0u;

  mowgli_interfaces::msg::WheelTick wt{};
  wt.stamp = node_.now();
  wt.wheel_tick_factor = static_cast<float>(ticks_per_meter);
  wt.valid_wheels = mowgli_interfaces::msg::WheelTick::WHEEL_VALID_RL |
                    mowgli_interfaces::msg::WheelTick::WHEEL_VALID_RR;
  wt.wheel_direction_rl = wheel_dir_left_;
  wt.wheel_ticks_rl = wheel_ticks_mag_left_;
  wt.wheel_direction_rr = wheel_dir_right_;
  wt.wheel_ticks_rr = wheel_ticks_mag_right_;
  pub_wheel_ticks_->publish(wt);

  // ----- Aggregate firmware packets into ~10 Hz wheel_odom publishes -----
  // Firmware packets arrive at ~47 Hz (every ~21 ms). At slow speeds this
  // gives only 0-3 ticks per window, so single-tick encoder noise (1 tick
  // = ~167 mm/s over 21 ms!) gets amplified into phantom velocity spikes
  // that robot_localization trusts thanks to the tight wheel covariance. Sum 5
  // packets (~100 ms, ~15 ticks at 0.5 m/s) so the velocity denominator
  // grows and single-tick noise collapses to ~7 % relative error.
  odom_acc_delta_left_ += d_left;
  odom_acc_delta_right_ += d_right;
  odom_acc_dt_ms_ += pkt.dt_millis;

  // 50 ms aggregation → ~20 Hz /wheel_odom. Tested: 33 ms (30 Hz)
  // saturated the EKF on this ARM CPU and produced "Failed to meet
  // update rate" errors on every cycle. 50 ms is twice the GPS rate
  // and twice the controller rate — sufficient for closed-loop
  // velocity control without choking the filter.
  static constexpr uint32_t kAggregateMs = 50;
  if (odom_acc_dt_ms_ < kAggregateMs)
  {
    return;
  }

  // Smoothed timestamp via the clock fitter. We feed it the
  // aggregated dt_ms (i.e. the total firmware time spanned by the
  // 5 packets we just folded together), so the fitter's virtual
  // firmware clock advances in sync with the published rate.
  const auto stamp = odom_clock_fit_.Ingest(odom_acc_dt_ms_, node_.now());
  const int32_t acc_d_left = odom_acc_delta_left_;
  const int32_t acc_d_right = odom_acc_delta_right_;
  const uint32_t acc_dt_ms = odom_acc_dt_ms_;
  odom_acc_delta_left_ = 0;
  odom_acc_delta_right_ = 0;
  odom_acc_dt_ms_ = 0;

  wheels_stationary_ = (acc_d_left == 0 && acc_d_right == 0);

  // Debug: log the aggregated window periodically.
  if (++odom_debug_count_ % 10 == 0)
  {
    RCLCPP_INFO(node_.get_logger(),
                "Odom: acc_dL=%d acc_dR=%d acc_dt=%u ms  (cum L=%d R=%d)",
                acc_d_left,
                acc_d_right,
                acc_dt_ms,
                pkt.left_ticks,
                pkt.right_ticks);
  }

  const double dt_sec = static_cast<double>(acc_dt_ms) / 1000.0;
  const double d_left_m = static_cast<double>(acc_d_left) / ticks_per_meter;
  const double d_right_m = static_cast<double>(acc_d_right) / ticks_per_meter;
  double vx = (d_left_m + d_right_m) * 0.5 / dt_sec;
  double vyaw = (d_right_m - d_left_m) / wheel_track / dt_sec;

  auto msg = nav_msgs::msg::Odometry{};
  msg.header.stamp = stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  // Force zero whenever the dock contacts are live. Charging current
  // proves the robot is mechanically anchored to the dock — the motors
  // cannot have moved it regardless of BT state. Previous narrower
  // condition (charging AND mode ∈ {NULL, IDLE}) missed the transient
  // RETURNING_HOME / end-of-mission states, during which the BT is
  // still mode=AUTONOMOUS(2) but the robot has already re-docked and
  // is physically stationary. Without the zero constraint, gyro_z
  // bias (~0.01 rad/s on the WT901) integrates into fusion yaw at
  // ~30°/min, which then corrupts the fused heading estimate and
  // manifests as a slowly-rotating robot icon while on the dock.
  //
  // Edge case: the charger bit can briefly stay high during a BackUp
  // undock before the contacts separate. That moment is handled by
  // the UndockRobot action, not by the wheel-odom path, so zeroing
  // for those ~100 ms is harmless.
  const bool force_zero = is_charging;
  if (force_zero)
  {
    vx = 0.0;
    vyaw = 0.0;
  }

  msg.twist.twist.linear.x = vx;
  msg.twist.twist.angular.z = vyaw;

  // Covariance: force_zero → very tight (we're certain we're not moving).
  // Otherwise: linear vel σ = 0.1 m/s, yaw rate σ = 0.03 rad/s (tight
  // wheel trust — calibrated drivetrain, dominated by grass slip ~3%).
  const double vel_var = force_zero ? 1e-6 : 0.01;
  msg.twist.covariance[0] = vel_var;  // vx variance
  // Non-holonomic constraint: diff-drive can't slide sideways. Tight
  // variance on VY=0 tells robot_localization to treat this as a hard constraint;
  // leaving at 1e6 ("unknown") lets GPS+IMU noise accumulate as apparent
  // lateral drift during outdoor runs.
  msg.twist.covariance[7] = 1e-4;  // vy (enforce VY = 0)
  msg.twist.covariance[14] = 1e6;  // vz - unknown
  msg.twist.covariance[21] = 1e6;  // wx - unknown
  msg.twist.covariance[28] = 1e6;  // wy - unknown
  msg.twist.covariance[35] = force_zero ? 1e-6 : 9e-4;  // wz variance

  pub_wheel_odom_->publish(msg);
}

}  // namespace mowgli_hardware
