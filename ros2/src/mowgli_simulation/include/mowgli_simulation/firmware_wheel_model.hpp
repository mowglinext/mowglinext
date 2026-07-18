// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// firmware_wheel_model — per-wheel mirror of the real firmware's motor loop
// (firmware/stm32/ros_usbnode/src/ros/ros_custom/cpp_main.cpp::on_cmd_vel +
// motors_handler), for use by sim_actuation_node.
//
// WHY per-wheel, not a lumped chassis model: the real firmware has NO
// chassis-level angular loop and NO angular deadband. It runs TWO
// independent linear-velocity PIs (firmware/stm32/ros_usbnode/include/pid.hpp,
// vendored PX4 PID), one per wheel, each fighting its own PWM
// static-friction stiction. A lumped "chassis won't rotate below
// wz_break_free" model cannot represent per-wheel asymmetry (grass load,
// battery sag, one wheel over an edge...) and its thresholds have no
// firmware counterpart. This header instead mirrors the real pipeline:
//
//   1. Diff-drive inverse kinematics: (vx, wz) -> (left_target, right_target)
//      — mirrors cpp_main.cpp::on_cmd_vel, board.h WHEEL_BASE.
//   2. Saturate each wheel to +-max_mps (board.h MAX_MPS).
//   3. Integrator reset on direction reversal / stop-to-go, exactly as
//      motors_handler does, so windup doesn't drive a decelerating wheel
//      backwards.
//   4. PI in PWM space: PWM = target*pwm_per_mps (feedforward) + Kp*err + I.
//   5. Saturate to +-pwm_max (PAC5210 magnitude).
//   6. Static/kinetic PWM deadband with hysteresis — a stalled wheel needs
//      deadband_pwm_static to break free; a moving wheel keeps rolling down
//      to deadband_pwm_kinetic. Below either, the wheel reports zero
//      velocity, exactly like the real motor + PAC5210.
//   7. Force PWM=0 on a stopped-target wheel once |actual| < hold_thresh_mps,
//      matching the real firmware's residual-hum suppression.
//   8. Forward kinematics recombine (left_actual, right_actual) into the
//      ACHIEVABLE body twist — this is what actually reached the chassis.
//
// Ported from (and MUST be kept in lockstep with) the identical Python model
// in ros2/src/mowgli_simulation/mowgli_simulation/kinematic_drive.py::
// __simulate_firmware_motor_model, which drives the Webots body teleport +
// IMU from raw /cmd_vel every physics tick. This C++ copy runs on the same
// raw /cmd_vel (after the host angular-rate PI, matching what the real
// firmware receives over the wire) to produce /cmd_vel_wheels for the ideal
// ros2_control diff_drive_controller, so /wheel_odom_raw reflects the same
// actuation dynamics the body pose and IMU already do. See
// sim_actuation_node.cpp for the rest of the pipeline and
// sim_full_system.launch.py for why /cmd_vel_wheels exists as a topic
// separate from /cmd_vel.
//
// Default gains mirror firmware/stm32/ros_usbnode/{include/board.h,
// src/ros/ros_custom/cpp_main.cpp} — coordinate changes with Firmware.
//
// Pure / ROS-free so it is unit-testable; all state is caller-owned.

#ifndef MOWGLI_SIMULATION__FIRMWARE_WHEEL_MODEL_HPP_
#define MOWGLI_SIMULATION__FIRMWARE_WHEEL_MODEL_HPP_

#include <algorithm>
#include <cmath>

namespace mowgli_simulation
{

struct FirmwareWheelModelParams
{
  double wheel_separation = 0.325;  ///< m, track width (board.h WHEEL_BASE).
  double max_mps = 0.5;  ///< per-wheel saturation (board.h MAX_MPS).
  double pwm_per_mps = 300.0;  ///< feedforward gain (board.h PWM_PER_MPS).
  double pwm_max = 255.0;  ///< PAC5210 magnitude clamp.
  double deadband_pwm_static = 40.0;  ///< breakaway PWM from rest (measured, ~PWM 40).
  double deadband_pwm_kinetic = 30.0;  ///< kinetic hold-on threshold (hysteresis).
  double pi_kp_pwm_per_mps = 30.0;  ///< WHEEL_PI_KP_PWM_PER_MPS.
  double pi_ki_pwm_per_mps_s = 5000.0;  ///< WHEEL_PI_KI_PWM_PER_MPS_S.
  double pi_int_max_pwm = 100.0;  ///< WHEEL_PI_INT_MAX_PWM.
  double pi_hold_thresh_mps = 0.02;  ///< |actual|<this and target==0 -> force PWM=0.
};

struct FirmwareWheelState
{
  double left_actual_mps = 0.0;
  double right_actual_mps = 0.0;
  double left_pi_int_pwm = 0.0;
  double right_pi_int_pwm = 0.0;
  double prev_left_target_mps = 0.0;
  double prev_right_target_mps = 0.0;
};

namespace detail
{

inline bool wheel_needs_integrator_reset(double target, double prev)
{
  return (target * prev < 0.0) || (target == 0.0 && prev != 0.0);
}

// One wheel's PI-in-PWM-space + static/kinetic stiction. Updates `actual_mps`,
// `pi_int_pwm` and `prev_target_mps` in place to the new state after one dt.
inline void step_wheel(double target_mps,
                       double& actual_mps,
                       double& pi_int_pwm,
                       double& prev_target_mps,
                       double dt,
                       const FirmwareWheelModelParams& p)
{
  target_mps = std::clamp(target_mps, -p.max_mps, p.max_mps);

  if (wheel_needs_integrator_reset(target_mps, prev_target_mps))
  {
    pi_int_pwm = 0.0;
  }
  prev_target_mps = target_mps;

  const double err = target_mps - actual_mps;
  pi_int_pwm = std::clamp(pi_int_pwm + p.pi_ki_pwm_per_mps_s * err * dt,
                          -p.pi_int_max_pwm,
                          p.pi_int_max_pwm);

  double pwm = target_mps * p.pwm_per_mps + p.pi_kp_pwm_per_mps * err + pi_int_pwm;
  pwm = std::clamp(pwm, -p.pwm_max, p.pwm_max);

  // Residual-hum suppression: a wheel commanded to stop that has already
  // settled near zero gets a hard PWM=0 rather than dithering on the PI.
  if (target_mps == 0.0 && std::abs(actual_mps) < p.pi_hold_thresh_mps)
  {
    pwm = 0.0;
  }

  // Static/kinetic deadband, evaluated against the PRE-update actual speed
  // (a stalled wheel needs the higher static threshold to break free; a
  // wheel already rolling only needs to clear the lower kinetic one).
  const bool was_stalled = (actual_mps == 0.0);
  const double deadband = was_stalled ? p.deadband_pwm_static : p.deadband_pwm_kinetic;
  actual_mps = (std::abs(pwm) < deadband) ? 0.0 : pwm / p.pwm_per_mps;
}

}  // namespace detail

/// Project a (cmd_vx, cmd_wz) body twist through the per-wheel firmware
/// motor model for one tick and return the ACHIEVABLE body twist.
///
/// \param cmd_vx forward velocity command (m/s).
/// \param cmd_wz yaw-rate command (rad/s), matching what the real firmware
///               receives over the wire (Option C, task #34: the host sends
///               this straight through — the yaw-rate loop runs in firmware,
///               not on the host, so there is no host-side shaping stage).
/// \param dt          seconds since the previous call.
/// \param p           firmware-mirrored gains / limits.
/// \param st          caller-owned per-wheel PI + stiction state.
/// \param[out] achievable_vx forward velocity the chassis actually achieves.
/// \param[out] achievable_wz yaw rate the chassis actually achieves.
inline void step_firmware_wheel_model(double cmd_vx,
                                      double cmd_wz,
                                      double dt,
                                      const FirmwareWheelModelParams& p,
                                      FirmwareWheelState& st,
                                      double& achievable_vx,
                                      double& achievable_wz)
{
  const double half_track = p.wheel_separation * 0.5;
  const double left_target = cmd_vx - cmd_wz * half_track;
  const double right_target = cmd_vx + cmd_wz * half_track;

  detail::step_wheel(
      left_target, st.left_actual_mps, st.left_pi_int_pwm, st.prev_left_target_mps, dt, p);
  detail::step_wheel(
      right_target, st.right_actual_mps, st.right_pi_int_pwm, st.prev_right_target_mps, dt, p);

  achievable_vx = 0.5 * (st.left_actual_mps + st.right_actual_mps);
  achievable_wz = (st.right_actual_mps - st.left_actual_mps) / p.wheel_separation;
}

}  // namespace mowgli_simulation

#endif  // MOWGLI_SIMULATION__FIRMWARE_WHEEL_MODEL_HPP_
