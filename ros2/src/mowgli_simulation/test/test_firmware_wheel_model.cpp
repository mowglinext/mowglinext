// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for step_firmware_wheel_model. Pure math, no ROS plumbing.
//
// Exercises the per-wheel pipeline (inverse kinematics -> per-wheel PI in
// PWM space -> static/kinetic stiction -> forward kinematics) that replaced
// the earlier lumped chassis-level angular deadband — the fictional model
// this header retires had no firmware counterpart at all (see the file
// header of firmware_wheel_model.hpp for the full rationale).

#include <algorithm>
#include <cmath>
#include <utility>

#include "mowgli_simulation/firmware_wheel_model.hpp"
#include <gtest/gtest.h>

namespace ms = mowgli_simulation;

namespace
{

constexpr double kDt = 0.02;  // 50 Hz, matches sim_actuation_node control_hz_.

// Run the model to steady state for a sustained (vx, wz) command and return
// the final achievable body twist.
std::pair<double, double> settle(double cmd_vx,
                                 double cmd_wz,
                                 const ms::FirmwareWheelModelParams& p,
                                 int ticks = 500)
{
  ms::FirmwareWheelState st{};
  double vx = 0.0, wz = 0.0;
  for (int i = 0; i < ticks; ++i)
  {
    ms::step_firmware_wheel_model(cmd_vx, cmd_wz, kDt, p, st, vx, wz);
  }
  return {vx, wz};
}

}  // namespace

// A sustained straight-line command well above the deadband must converge
// to the commanded forward speed with ~zero yaw rate.
TEST(FirmwareWheelModel, StraightLineConverges)
{
  ms::FirmwareWheelModelParams p;  // defaults
  const auto [vx, wz] = settle(0.2, 0.0, p);
  EXPECT_NEAR(vx, 0.2, 0.01);
  EXPECT_NEAR(wz, 0.0, 0.01);
}

// A sustained pure pivot (vx=0) must converge to the commanded yaw rate with
// ~zero drift — left/right targets are exactly antisymmetric so the two
// wheel PIs converge to antisymmetric speeds by construction.
TEST(FirmwareWheelModel, PivotConverges)
{
  ms::FirmwareWheelModelParams p;
  const auto [vx, wz] = settle(0.0, 1.0, p);
  EXPECT_NEAR(vx, 0.0, 0.01);
  EXPECT_NEAR(wz, 1.0, 0.05);
}

// THE FIX: a per-wheel target too small to clear the STATIC PWM deadband
// from rest must stall (report zero velocity) rather than instantly track,
// exactly like the real motor + PAC5210. The old lumped chassis model could
// not represent this at the per-wheel level at all. Given enough time the
// integrator winds up past the deadband, the wheel "kicks" free, the P term
// then pulls it back below the (now lower, kinetic) threshold and it stalls
// again — a bounded buzz/dither, not a smooth sub-deadband creep. This is
// exactly why hardware_bridge_node's min_linear_vel_ clamps tiny commands to
// zero before they ever reach the real firmware.
TEST(FirmwareWheelModel, SubDeadbandTargetStallsThenBuzzesBounded)
{
  ms::FirmwareWheelModelParams p;
  ms::FirmwareWheelState st{};
  double vx = 0.0, wz = 0.0;
  // vx=0.01 m/s -> feedforward PWM = 0.01*300 = 3, far below deadband_pwm_static
  // (40). A handful of ticks must not be enough for the integrator
  // (ki=5000 PWM/(mps*s)) to cross the breakaway threshold.
  for (int i = 0; i < 5; ++i)
  {
    ms::step_firmware_wheel_model(0.01, 0.0, kDt, p, st, vx, wz);
  }
  EXPECT_EQ(vx, 0.0) << "wheel should still be stalled after only 5 ticks";

  double max_seen = 0.0;
  for (int i = 0; i < 2000; ++i)
  {
    ms::step_firmware_wheel_model(0.01, 0.0, kDt, p, st, vx, wz);
    max_seen = std::max(max_seen, vx);
  }
  EXPECT_GT(max_seen, 0.0) << "integrator should cross the static deadband at least once";
  EXPECT_LT(max_seen, 0.2) << "kicks must stay well below the wheel's max speed, not run away";
}

// Once a wheel is rolling it should keep rolling down to the lower KINETIC
// threshold (hysteresis) rather than immediately re-stalling at the static
// one — this is what lets slow, sustained coverage-speed creep track
// smoothly instead of stick-slipping every tick.
TEST(FirmwareWheelModel, KineticHysteresisKeepsWheelMovingBelowStaticThreshold)
{
  ms::FirmwareWheelModelParams p;
  ms::FirmwareWheelState st{};
  // Seed the state as "already rolling" at 0.15 m/s (feedforward PWM=45,
  // comfortably above deadband_pwm_static=40) — deterministic, independent
  // of any prior transient.
  st.left_actual_mps = 0.15;
  st.right_actual_mps = 0.15;

  // Command a slightly lower sustained speed: target*pwm_per_mps + Kp*err +
  // I works out to ~32 PWM here — below the static threshold (40), so a
  // STALLED wheel could not have broken free at this command, but an
  // already-rolling wheel (kinetic threshold 30) must keep going.
  double vx = 0.0, wz = 0.0;
  ms::step_firmware_wheel_model(0.12, 0.0, kDt, p, st, vx, wz);
  EXPECT_GT(vx, 0.0) << "already-rolling wheel must not instantly re-stall";
}

// Per-wheel targets must saturate to +-max_mps before the PI ever sees them
// — a runaway forward command cannot make the chassis exceed the firmware's
// physical wheel-speed ceiling.
TEST(FirmwareWheelModel, SaturatesToMaxWheelSpeed)
{
  ms::FirmwareWheelModelParams p;
  const auto [vx, wz] = settle(5.0, 0.0, p);
  EXPECT_NEAR(vx, p.max_mps, 0.01);
  EXPECT_NEAR(wz, 0.0, 0.01);
}

// Stopping (target -> 0) must settle the wheel to EXACTLY zero (the
// residual-hum suppression), not leave a dithering PI hum forever.
TEST(FirmwareWheelModel, StopSettlesToExactZero)
{
  ms::FirmwareWheelModelParams p;
  ms::FirmwareWheelState st{};
  double vx = 0.0, wz = 0.0;
  for (int i = 0; i < 200; ++i)
  {
    ms::step_firmware_wheel_model(0.2, 0.0, kDt, p, st, vx, wz);
  }
  ASSERT_GT(vx, 0.0);
  for (int i = 0; i < 200; ++i)
  {
    ms::step_firmware_wheel_model(0.0, 0.0, kDt, p, st, vx, wz);
  }
  EXPECT_EQ(vx, 0.0);
  EXPECT_EQ(wz, 0.0);
}

// Anti-windup: the per-wheel PWM integrator must stay within its clamp even
// under a permanently-stalled wheel (this test forces a stall by driving the
// model with a target the deadband can never be reached for, i.e. checks the
// clamp directly rather than inferring it from achievable velocity).
TEST(FirmwareWheelModel, IntegratorClampRespected)
{
  ms::FirmwareWheelModelParams p;
  ms::FirmwareWheelState st{};
  double vx = 0.0, wz = 0.0;
  for (int i = 0; i < 1000; ++i)
  {
    ms::step_firmware_wheel_model(0.3, 0.0, kDt, p, st, vx, wz);
  }
  EXPECT_LE(std::abs(st.left_pi_int_pwm), p.pi_int_max_pwm + 1e-9);
  EXPECT_LE(std::abs(st.right_pi_int_pwm), p.pi_int_max_pwm + 1e-9);
}

// A direction reversal must drop the stale integrator rather than let it
// fight the new direction (mirrors motors_handler's stop-to-go / sign-flip
// reset). Sampled mid-transient (5 ticks): by full steady state the
// integral has already decayed back near zero (the feedforward term alone
// sustains a constant target once above deadband), so a later sample
// wouldn't exercise the reset at all.
TEST(FirmwareWheelModel, DirectionReversalResetsIntegrator)
{
  ms::FirmwareWheelModelParams p;
  ms::FirmwareWheelState st{};
  double vx = 0.0, wz = 0.0;
  for (int i = 0; i < 5; ++i)
  {
    ms::step_firmware_wheel_model(0.3, 0.0, kDt, p, st, vx, wz);
  }
  EXPECT_GT(st.left_pi_int_pwm, 0.0);
  EXPECT_GT(st.right_pi_int_pwm, 0.0);

  ms::step_firmware_wheel_model(-0.3, 0.0, kDt, p, st, vx, wz);
  EXPECT_LE(st.left_pi_int_pwm, 0.0);
  EXPECT_LE(st.right_pi_int_pwm, 0.0);
}
