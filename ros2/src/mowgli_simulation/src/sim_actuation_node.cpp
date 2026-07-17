// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// sim_actuation_node — SIMULATION ONLY.
//
// The ideal Webots diff_drive_controller applies any commanded velocity
// perfectly, so the sim CANNOT reproduce the real robot's actuation dynamics —
// specifically the firmware's PER-WHEEL PWM static-friction stiction (two
// independent linear-velocity PIs, no chassis-level angular loop at all — see
// firmware_wheel_model.hpp for the full pipeline) and the host closed-loop
// angular-rate PI (mowgli_hardware/angular_rate_controller.hpp) that winds up
// against it. That stiction↔PI interaction is the physical origin of the
// near-deadband angular LIMIT CYCLE seen on hardware: the slow left-right
// weave while mowing and the left-right hunt at the dock. Without it the sim
// mows unrealistically clean (rms_wz≈0.05) and any A/B of the fix is meaningless.
//
// This node inserts the missing physics between the nav command and the wheels:
//
//   /cmd_vel (nav twist_mux output, matches the real robot's /cmd_vel)
//     → [angular-rate PI, the REAL header, if angular_rate_loop_enabled]
//       → [per-wheel firmware motor model: inverse kinematics, two PIs,
//          per-wheel static/kinetic PWM stiction, forward kinematics]
//         → /cmd_vel_wheels (the Webots diff_drive input)
//
// Gyro feedback comes from /imu/data (which already carries sim_imu_noise), so
// the whole loop — nav → PI → per-wheel model → chassis → IMU → localizer →
// nav — is closed exactly as on hardware. A/B the fix at runtime with:
//   ros2 param set /sim_actuation angular_rate_loop_enabled false   # passthrough
//   ros2 param set /sim_actuation angular_rate_ki 0.5               # soften windup
// and observe /cmd_vel (the nav command weave) collapse or not.
//
// The per-wheel model here MUST be kept in lockstep with the identical Python
// copy in mowgli_simulation/kinematic_drive.py (which drives the Webots body
// teleport + IMU from the same raw /cmd_vel) — see firmware_wheel_model.hpp.
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
#include "mowgli_simulation/firmware_wheel_model.hpp"

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

    // ── Per-wheel firmware motor model (see firmware_wheel_model.hpp): two
    //    independent linear-velocity PIs, each fighting its own PWM
    //    static/kinetic stiction. This is the nonlinearity the host PI winds
    //    up against. Forward: |vx| < min_linear_vel is clamped to 0 (matches
    //    hardware_bridge min_linear_vel_).
    deadband_enabled_ = declare_parameter<bool>("deadband_enabled", true);
    wheel_.wheel_separation = declare_parameter<double>("wheel_separation", 0.325);
    wheel_.max_mps = declare_parameter<double>("firmware_max_mps", 0.5);
    wheel_.pwm_per_mps = declare_parameter<double>("firmware_pwm_per_mps", 300.0);
    wheel_.pwm_max = declare_parameter<double>("firmware_pwm_max", 255.0);
    wheel_.deadband_pwm_static =
        declare_parameter<double>("firmware_deadband_pwm_static", 40.0);
    wheel_.deadband_pwm_kinetic =
        declare_parameter<double>("firmware_deadband_pwm_kinetic", 30.0);
    wheel_.pi_kp_pwm_per_mps = declare_parameter<double>("firmware_pi_kp_pwm_per_mps", 30.0);
    wheel_.pi_ki_pwm_per_mps_s =
        declare_parameter<double>("firmware_pi_ki_pwm_per_mps_s", 5000.0);
    wheel_.pi_int_max_pwm = declare_parameter<double>("firmware_pi_int_max_pwm", 100.0);
    wheel_.pi_hold_thresh_mps =
        declare_parameter<double>("firmware_pi_hold_thresh_mps", 0.02);
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
                "sim_actuation: PI %s (ki=%.2f), per-wheel model %s (static=%.0f "
                "kinetic=%.0f PWM, min_vx=%.2f) — /cmd_vel -> /cmd_vel_wheels",
                loop_enabled_ ? "ON" : "OFF(passthrough)", ar_.ki,
                deadband_enabled_ ? "ON" : "OFF", wheel_.deadband_pwm_static,
                wheel_.deadband_pwm_kinetic, min_linear_vel_);
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

    // Sub-deadband forward-velocity guard (host-side, matches hardware_bridge
    // min_linear_vel_ — NOT the per-wheel motor stiction below).
    const double cmd_vx = (std::abs(target_vx) < min_linear_vel_) ? 0.0 : target_vx;

    // Per-wheel firmware motor model: inverse kinematics → two independent
    // PIs in PWM space → per-wheel static/kinetic stiction → forward
    // kinematics, giving the ACHIEVABLE body twist. Disabled → passthrough
    // (ideal-actuation baseline).
    double wheel_vx = cmd_vx;
    double wheel_wz = cmd_wz;
    if (deadband_enabled_)
    {
      step_firmware_wheel_model(cmd_vx, cmd_wz, dt, wheel_, wheel_state_, wheel_vx, wheel_wz);
    }

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
  FirmwareWheelModelParams wheel_;
  double min_linear_vel_ = 0.05;
  double control_hz_ = 50.0;

  // ── State
  mh::AngularRateState ar_state_;
  FirmwareWheelState wheel_state_;
  double last_vx_ = 0.0;
  double last_wz_ = 0.0;
  double measured_wz_ = 0.0;
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
