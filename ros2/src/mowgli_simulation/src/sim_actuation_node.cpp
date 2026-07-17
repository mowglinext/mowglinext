// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// sim_actuation_node — SIMULATION ONLY.
//
// The ideal Webots diff_drive_controller applies any commanded velocity
// perfectly, so the sim CANNOT reproduce the real robot's actuation dynamics —
// specifically the firmware's PER-WHEEL PWM static-friction stiction (two
// independent linear-velocity PIs — see firmware_wheel_model.hpp for the full
// pipeline). Without it the sim mows unrealistically clean and any A/B of a
// drive-loop fix is meaningless.
//
// This node inserts the missing physics between the nav command and the wheels:
//
//   /cmd_vel (nav twist_mux output, matches the real robot's /cmd_vel)
//     → [per-wheel firmware motor model: inverse kinematics, two PIs,
//        per-wheel static/kinetic PWM stiction, forward kinematics]
//       → /cmd_vel_wheels (the Webots diff_drive input)
//
// 2026-07-17 Option C (task #34): wz used to pass through a host-side
// closed-loop angular-rate PI here (mirroring hardware_bridge_node's Option B
// loop, task #24) before reaching the per-wheel model — that stage is
// REMOVED. The yaw-rate loop moved into FIRMWARE (task #33); on real
// hardware, hardware_bridge now sends wz straight through, so this sim node
// does the same — wz reaches the per-wheel model unshaped, matching the new
// host behaviour. The per-wheel model's two independent linear-velocity PIs
// (still simulated below) have no chassis-level angular loop of their own,
// same as before; if the firmware's new yaw loop needs sim fidelity, that
// would be a NEW stage modeling task #33's loop specifically, not a revival
// of this one — deferred pending Firmware-2's #33 interface.
//
// The per-wheel model is purely translational — no /imu/data subscription is
// needed anymore (the removed angular-rate PI was the only gyro-feedback
// consumer here). A future firmware-yaw-loop fidelity stage would re-add it.
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

#include "mowgli_simulation/firmware_wheel_model.hpp"

namespace mowgli_simulation
{

class SimActuationNode : public rclcpp::Node
{
public:
  SimActuationNode() : rclcpp::Node("sim_actuation")
  {
    // ── Per-wheel firmware motor model (see firmware_wheel_model.hpp): two
    //    independent linear-velocity PIs, each with its own PWM static/
    //    kinetic stiction. Forward: |vx| < min_linear_vel is clamped to 0
    //    (matches hardware_bridge min_linear_vel_).
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
    last_tick_ = this->now();
    last_cmd_stamp_ = this->now();
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / control_hz_), [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
                "sim_actuation: per-wheel model %s (static=%.0f "
                "kinetic=%.0f PWM, min_vx=%.2f), wz passthrough (Option C, "
                "firmware owns the yaw loop) — /cmd_vel -> /cmd_vel_wheels",
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

    // wz passthrough (Option C, task #34) — the yaw-rate loop runs in
    // firmware now, so the host (and this sim node, mirroring it) sends the
    // commanded rate straight through with no shaping.
    const double cmd_wz = target_wz;

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
  bool deadband_enabled_ = true;
  FirmwareWheelModelParams wheel_;
  double min_linear_vel_ = 0.05;
  double control_hz_ = 50.0;

  // ── State
  FirmwareWheelState wheel_state_;
  double last_vx_ = 0.0;
  double last_wz_ = 0.0;
  rclcpp::Time last_cmd_stamp_;
  rclcpp::Time last_tick_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_;
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
