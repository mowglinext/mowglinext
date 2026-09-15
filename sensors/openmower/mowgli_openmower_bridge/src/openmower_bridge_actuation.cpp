// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0
//
// The 50 Hz control tick: poll the motor controllers, derive wheel odometry,
// close the wheel velocity loops, gate the blade, feed the LowLevel heartbeat.

#include <algorithm>
#include <cmath>

#include "mowgli_hardware/cmd_vel_slew.hpp"
#include "mowgli_interfaces/update_maintenance.hpp"
#include "mowgli_openmower_bridge/blade_policy.hpp"
#include "mowgli_openmower_bridge/openmower_bridge_node.hpp"

namespace mowgli_openmower_bridge
{

double OpenMowerBridgeNode::motor_sign(std::size_t motor) const
{
  if (motor == kLeft)
  {
    return left_motor_inverted_ ? -1.0 : 1.0;
  }
  if (motor == kRight)
  {
    return right_motor_inverted_ ? -1.0 : 1.0;
  }
  return 1.0;
}

bool OpenMowerBridgeNode::drive_links_connected() const
{
  return motors_[kLeft] && motors_[kRight] && motors_[kLeft]->telemetry().connected &&
         motors_[kRight]->telemetry().connected;
}

std::string OpenMowerBridgeNode::motor_summary() const
{
  std::string out = xesc_type_;
  static const char* const kNames[] = {"L", "R", "M"};
  for (std::size_t m = 0; m < motors_.size(); ++m)
  {
    if (!motors_[m])
    {
      continue;
    }
    const auto& t = motors_[m]->telemetry();
    out += ' ';
    out += kNames[m];
    out += t.connected ? "=ok" : "=down";
    if (t.connected)
    {
      out += '(' + std::to_string(t.fw_major) + '.' + std::to_string(t.fw_minor) + ')';
    }
  }
  return out;
}

void OpenMowerBridgeNode::control_tick()
{
  const SteadyClock::time_point now_steady = SteadyClock::now();
  double dt_s = 1.0 / control_rate_hz_;
  if (last_control_tick_.time_since_epoch().count() != 0)
  {
    dt_s = std::chrono::duration<double>(now_steady - last_control_tick_).count();
    dt_s = std::clamp(dt_s, 0.5 / control_rate_hz_, 5.0 / control_rate_hz_);
  }
  last_control_tick_ = now_steady;
  ++control_ticks_;

  for (auto& motor : motors_)
  {
    if (motor)
    {
      motor->Poll(now_steady);
    }
  }

  const bool drives_ok = drive_links_connected();
  if (drives_ok != motors_were_connected_)
  {
    motors_were_connected_ = drives_ok;
    if (drives_ok)
    {
      RCLCPP_INFO(get_logger(), "Drive controllers connected: %s", motor_summary().c_str());
      odometry_.Reset();
      wheel_ticks_primed_ = {false, false};
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Drive controller lost: %s", motor_summary().c_str());
    }
  }

  if (drives_ok)
  {
    update_odometry(dt_s);
  }

  const bool ll_alive = lowlevel_alive();
  const bool emergency = emergency_.is_emergency();
  const bool cmd_fresh =
      cmd_vel_stamp_.nanoseconds() != 0 && (now() - cmd_vel_stamp_).seconds() <= cmd_vel_timeout_s_;
  const bool wheels_allowed = WheelsMayRun(emergency, ll_alive, current_mode_, cmd_fresh);
  drive_wheels(dt_s, wheels_allowed && drives_ok);

  const bool hl_fresh = hl_status_stamp_.nanoseconds() != 0 &&
                        (now() - hl_status_stamp_).seconds() <= high_level_status_timeout_s_;
  const bool blade_allowed = BladeMayRun(BladeInputs{mow_requested_,
                                                     mowing_enabled_,
                                                     emergency,
                                                     ll_alive,
                                                     current_mode_,
                                                     hl_fresh,
                                                     mowgli_interfaces::updateMaintenanceActive()});
  drive_blade(blade_allowed);

  // Heartbeat + config handshake ride on the control tick so the LowLevel
  // board sees a steady cadence tied to actuation, as with mower_comms_v1.
  const auto ticks_per_heartbeat =
      static_cast<std::size_t>(std::max(1.0, std::round(control_rate_hz_ / heartbeat_rate_hz_)));
  if (control_ticks_ % ticks_per_heartbeat == 0u)
  {
    send_heartbeat();
  }
  service_config_handshake();
}

void OpenMowerBridgeNode::update_odometry(double dt_s)
{
  std::array<int64_t, 2> ticks{};
  for (std::size_t w = 0; w < 2u; ++w)
  {
    ticks[w] = static_cast<int64_t>(motor_sign(w)) * motors_[w]->telemetry().signed_ticks;
  }

  // Per-wheel speed (for the velocity loops) and the diagnostic WheelTick.
  std::array<int64_t, 2> deltas{};
  for (std::size_t w = 0; w < 2u; ++w)
  {
    if (!wheel_ticks_primed_[w])
    {
      wheel_ticks_primed_[w] = true;
      prev_wheel_ticks_[w] = ticks[w];
      wheel_speed_[w].Reset();
      continue;
    }
    deltas[w] = ticks[w] - prev_wheel_ticks_[w];
    prev_wheel_ticks_[w] = ticks[w];
    (void)wheel_speed_[w].Update(deltas[w], ticks_per_meter_, dt_s);
    wheel_tick_magnitude_[w] += static_cast<uint32_t>(std::abs(deltas[w]));
    if (deltas[w] > 0)
    {
      wheel_tick_direction_[w] = 1u;
    }
    else if (deltas[w] < 0)
    {
      wheel_tick_direction_[w] = 0u;
    }
  }

  mowgli_interfaces::msg::WheelTick wt{};
  wt.stamp = now();
  wt.wheel_tick_factor = static_cast<float>(ticks_per_meter_);
  wt.valid_wheels = mowgli_interfaces::msg::WheelTick::WHEEL_VALID_RL |
                    mowgli_interfaces::msg::WheelTick::WHEEL_VALID_RR;
  wt.wheel_direction_rl = wheel_tick_direction_[kLeft];
  wt.wheel_ticks_rl = wheel_tick_magnitude_[kLeft];
  wt.wheel_direction_rr = wheel_tick_direction_[kRight];
  wt.wheel_ticks_rr = wheel_tick_magnitude_[kRight];
  pub_wheel_ticks_->publish(wt);

  const auto sample = odometry_.Update(ticks[kLeft], ticks[kRight], dt_s);
  if (!sample)
  {
    return;
  }

  nav_msgs::msg::Odometry msg{};
  msg.header.stamp =
      odom_clock_fit_.Ingest(static_cast<uint32_t>(std::lround(sample->dt_s * 1000.0)), now());
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";
  double vx = sample->vx_mps;
  double vyaw = sample->vyaw_radps;
  // Invariant 11: zero while the charger contacts are live.
  const bool force_zero = is_charging_;
  if (force_zero)
  {
    vx = 0.0;
    vyaw = 0.0;
  }
  msg.twist.twist.linear.x = vx;
  msg.twist.twist.angular.z = vyaw;
  const double vel_var = force_zero ? 1e-6 : 0.01;
  msg.twist.covariance[0] = vel_var;
  msg.twist.covariance[7] = 1e-4;  // non-holonomic: vy is zero
  msg.twist.covariance[14] = 1e6;
  msg.twist.covariance[21] = 1e6;
  msg.twist.covariance[28] = 1e6;
  msg.twist.covariance[35] = force_zero ? 1e-6 : 9e-4;
  pub_wheel_odom_->publish(msg);
}

void OpenMowerBridgeNode::drive_wheels(double dt_s, bool wheels_allowed)
{
  double target_vx = 0.0;
  double target_wz = 0.0;
  if (wheels_allowed)
  {
    target_vx = cmd_vx_;
    target_wz = cmd_wz_;
  }
  else if (last_wheels_allowed_)
  {
    // Edge into a stop: log once why the wheels are held.
    RCLCPP_INFO_THROTTLE(get_logger(),
                         *get_clock(),
                         5000,
                         "Wheels held: emergency=%s lowlevel=%s mode=%u cmd_fresh=%s",
                         emergency_.is_emergency() ? "yes" : "no",
                         lowlevel_alive() ? "alive" : "down",
                         current_mode_,
                         (now() - cmd_vel_stamp_).seconds() <= cmd_vel_timeout_s_ ? "yes" : "no");
  }
  last_wheels_allowed_ = wheels_allowed;

  applied_vx_ = mowgli_hardware::limit_motion_command_slew(
      target_vx, applied_vx_, dt_s, lin_accel_limit_, lin_decel_limit_);
  applied_wz_ = mowgli_hardware::limit_motion_command_slew(
      target_wz, applied_wz_, dt_s, ang_accel_limit_, ang_decel_limit_);
  publish_cmd_vel_applied(applied_vx_, applied_wz_);

  const WheelSpeeds targets = SplitTwist(applied_vx_, applied_wz_, wheel_track_);
  const std::array<double, 2> target_mps{targets.left_mps, targets.right_mps};
  for (std::size_t w = 0; w < 2u; ++w)
  {
    if (!motors_[w])
    {
      continue;
    }
    const double duty = wheel_loops_[w].Update(target_mps[w], wheel_speed_[w].value(), dt_s);
    motors_[w]->SendDuty(motor_sign(w) * duty);
  }
}

void OpenMowerBridgeNode::drive_blade(bool blade_allowed)
{
  if (!motors_[kMow])
  {
    blade_running_ = false;
    return;
  }
  // OpenMower convention (mower_comms_v1): direction 1 = +duty, 0 = -duty.
  const double duty = blade_allowed ? (mow_direction_ != 0u ? blade_duty_ : -blade_duty_) : 0.0;
  motors_[kMow]->SendDuty(duty);
  if (blade_allowed != blade_running_)
  {
    RCLCPP_INFO(get_logger(), "Blade %s", blade_allowed ? "ON" : "OFF");
  }
  blade_running_ = blade_allowed;
  if (motors_[kMow]->telemetry().has_status)
  {
    const auto age = SteadyClock::now() - motors_[kMow]->telemetry().last_status;
    blade_status_stamp_ = now() - rclcpp::Duration(age);
  }
}

void OpenMowerBridgeNode::publish_cmd_vel_applied(double vx, double wz)
{
  // Only on change: a 50 Hz stream of identical zeros is noise in a bag.
  if (vx == published_vx_ && wz == published_wz_)
  {
    return;
  }
  published_vx_ = vx;
  published_wz_ = wz;
  geometry_msgs::msg::TwistStamped msg{};
  msg.header.stamp = now();
  msg.header.frame_id = "base_link";
  msg.twist.linear.x = vx;
  msg.twist.angular.z = wz;
  pub_cmd_vel_applied_->publish(msg);
}

}  // namespace mowgli_openmower_bridge
