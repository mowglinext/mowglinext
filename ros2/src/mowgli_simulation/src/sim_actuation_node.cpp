// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// sim_actuation_node — SIMULATION ONLY.
//
// The ideal Webots diff_drive_controller applies any commanded velocity
// perfectly, so the sim CANNOT reproduce the real robot's actuation dynamics —
// specifically the firmware STATIC-FRICTION DEADBAND on wheel motion and the
// host closed-loop angular-rate PI (mowgli_hardware/angular_rate_controller.hpp)
// that winds up against it. That deadband↔PI interaction is the physical origin
// of the near-deadband angular LIMIT CYCLE seen on hardware: the slow left-right
// weave while mowing and the left-right hunt at the dock. Without it the sim
// mows unrealistically clean (rms_wz≈0.05) and any A/B of the fix is meaningless.
//
// This node inserts the missing physics between the nav command and the wheels:
//
//   /cmd_vel (nav twist_mux output, matches the real robot's /cmd_vel)
//     → [angular-rate PI, the REAL header, if angular_rate_loop_enabled]
//       → [static-friction deadband on the actual wheel rotation]
//         → /cmd_vel_wheels (the Webots diff_drive input)
//
// Gyro feedback comes from /imu/data (which already carries sim_imu_noise), so
// the whole loop — nav → PI → deadband → chassis → IMU → localizer → nav — is
// closed exactly as on hardware. A/B the fix at runtime with:
//   ros2 param set /sim_actuation angular_rate_loop_enabled false   # passthrough
//   ros2 param set /sim_actuation angular_rate_ki 0.5               # soften windup
// and observe /cmd_vel (the nav command weave) collapse or not.
//
// Safety: SIM ONLY. Reads /cmd_vel + /imu/data, publishes one wheel-command
// topic. Never runs on real hardware (the real hardware_bridge owns this loop).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "mowgli_hardware/angular_rate_controller.hpp"

namespace mowgli_simulation
{
namespace mh = mowgli_hardware;

class SimActuationNode : public rclcpp::Node
{
public:
  SimActuationNode() : rclcpp::Node("sim_actuation")
  {
    // ── Angular-rate PI (mirror hardware_bridge_node defaults) — the loop
    //    under A/B test. Disable → passthrough (the nav command reaches the
    //    deadband directly, no windup).
    loop_enabled_ = declare_parameter<bool>("angular_rate_loop_enabled", true);
    ar_.kff = declare_parameter<double>("angular_rate_kff", 1.0);
    ar_.kp = declare_parameter<double>("angular_rate_kp", 0.4);
    ar_.ki = declare_parameter<double>("angular_rate_ki", 2.0);
    ar_.max_cmd = declare_parameter<double>("angular_rate_max_cmd", 1.5);
    ar_.integral_max = declare_parameter<double>("angular_rate_integral_max", 1.5);
    ar_.target_lp_tau = declare_parameter<double>("angular_rate_target_lp_tau", 0.2);
    ar_.min_target = declare_parameter<double>("angular_rate_min_target", 1.0e-3);

    // ── Firmware static-friction deadband on the ACTUAL wheel motion. Below
    //    wz_break_free (from rest) the chassis will not rotate; once rotating it
    //    keeps rotating down to wz_keep_move (kinetic hysteresis). This is the
    //    nonlinearity the PI winds up against. Forward: |vx| < min_linear_vel is
    //    clamped to 0 (matches hardware_bridge min_linear_vel_).
    deadband_enabled_ = declare_parameter<bool>("deadband_enabled", true);
    wz_break_free_ = declare_parameter<double>("wz_break_free", 0.18);
    wz_keep_move_ = declare_parameter<double>("wz_keep_move", 0.08);
    min_linear_vel_ = declare_parameter<double>("min_linear_vel", 0.05);

    control_hz_ = declare_parameter<double>("control_hz", 50.0);

    pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel_wheels", rclcpp::SystemDefaultsQoS());
    sub_cmd_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m) {
          last_vx_ = m->twist.linear.x;
          last_wz_ = m->twist.angular.z;
          last_cmd_stamp_ = this->now();
        });
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
          measured_wz_ = m->angular_velocity.z;
        });

    last_tick_ = this->now();
    last_cmd_stamp_ = this->now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_hz_), [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
                "sim_actuation: PI %s (ki=%.2f), deadband %s (break=%.2f keep=%.2f, "
                "min_vx=%.2f) — /cmd_vel -> /cmd_vel_wheels",
                loop_enabled_ ? "ON" : "OFF(passthrough)", ar_.ki,
                deadband_enabled_ ? "ON" : "OFF", wz_break_free_, wz_keep_move_,
                min_linear_vel_);
  }

private:
  void tick()
  {
    const rclcpp::Time t = this->now();
    double dt = (t - last_tick_).seconds();
    last_tick_ = t;
    if (dt <= 0.0 || dt > 0.5)
    {
      dt = 1.0 / control_hz_;
    }

    // Stale command (nav stopped publishing) → stop.
    double target_vx = last_vx_;
    double target_wz = last_wz_;
    if ((t - last_cmd_stamp_).seconds() > 0.5)
    {
      target_vx = 0.0;
      target_wz = 0.0;
    }

    // Angular-rate PI (the REAL header — the loop under test) or passthrough.
    const double cmd_wz =
        loop_enabled_
            ? mh::compute_angular_rate_cmd(target_wz, measured_wz_, dt, ar_, ar_state_)
            : target_wz;

    // Static-friction deadband on the actual wheel rotation (hysteresis).
    double wheel_wz = cmd_wz;
    if (deadband_enabled_)
    {
      const double mag = std::abs(cmd_wz);
      if (!rotating_)
      {
        if (mag >= wz_break_free_)
        {
          rotating_ = true;
        }
        else
        {
          wheel_wz = 0.0;
        }
      }
      else if (mag < wz_keep_move_)
      {
        rotating_ = false;
        wheel_wz = 0.0;
      }
    }

    // Forward static-friction deadband.
    const double wheel_vx =
        (deadband_enabled_ && std::abs(target_vx) < min_linear_vel_) ? 0.0 : target_vx;

    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = t;
    out.header.frame_id = "base_link";
    out.twist.linear.x = wheel_vx;
    out.twist.angular.z = wheel_wz;
    pub_->publish(out);
  }

  // ── Params
  bool loop_enabled_ = true;
  bool deadband_enabled_ = true;
  mh::AngularRateParams ar_;
  double wz_break_free_ = 0.18;
  double wz_keep_move_ = 0.08;
  double min_linear_vel_ = 0.05;
  double control_hz_ = 50.0;

  // ── State
  mh::AngularRateState ar_state_;
  double last_vx_ = 0.0;
  double last_wz_ = 0.0;
  double measured_wz_ = 0.0;
  bool rotating_ = false;
  rclcpp::Time last_cmd_stamp_;
  rclcpp::Time last_tick_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mowgli_simulation

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mowgli_simulation::SimActuationNode>());
  rclcpp::shutdown();
  return 0;
}
