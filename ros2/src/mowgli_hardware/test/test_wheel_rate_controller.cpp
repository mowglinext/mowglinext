// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for compute_wheel_pwm. Pure math, no ROS plumbing.
//
// The controller is the host-side replacement for the firmware wheel PI. The
// decisive tests drive it against a SIMULATED firmware/drivetrain (a static-
// friction deadband plus a PWM-per-(m/s) gain) and assert (1) the closed loop
// converges the measured wheel speed onto the target even when the feedforward
// seed is wrong, and (2) online calibration folds the steady-state correction
// into the feedforward so the integral unloads to ~0.

#include <cmath>

#include "mowgli_hardware/wheel_rate_controller.hpp"
#include <gtest/gtest.h>

namespace mh = mowgli_hardware;

namespace
{

// Crude drivetrain model: signed PWM → wheel velocity. Below the static-
// friction deadband db_phys the wheel doesn't move; above it, velocity rises
// linearly at 1/kff_phys (so the PWM needed for speed v is kff_phys*v + db_phys).
// Saturates at a chassis max. Not a physical fit — just enough to exercise the
// loop's convergence and the calibrator.
double sim_wheel_velocity(double pwm, double kff_phys, double db_phys)
{
  const double s = (pwm < 0.0) ? -1.0 : 1.0;
  const double a = std::abs(pwm);
  if (a <= db_phys)
  {
    return 0.0;  // sub-deadband: buzz, no motion
  }
  double v = (a - db_phys) / kff_phys;
  const double vmax = 0.6;
  if (v > vmax)
  {
    v = vmax;
  }
  return s * v;
}

constexpr double kDt = 0.021;  // ~47 Hz firmware odom packet cadence

// Run the loop to steady state against the sim drivetrain. Returns the final
// measured speed; leaves `st` holding the converged state for inspection.
double settle(double target, double kff_phys, double db_phys, const mh::WheelRateParams& p,
              mh::WheelRateState& st, int ticks)
{
  double measured = 0.0;
  for (int i = 0; i < ticks; ++i)
  {
    const double pwm = mh::compute_wheel_pwm(target, measured, kDt, p, st);
    measured = sim_wheel_velocity(pwm, kff_phys, db_phys);
  }
  return measured;
}

}  // namespace

// Closed-loop convergence: even with the feedforward seed matching the plant,
// the loop must hold the measured speed on target across the operating range,
// including reverse.
TEST(WheelRateController, ConvergesAcrossSpeedRange)
{
  mh::WheelRateParams p;  // seeds kff=300, deadband=40 — equal to plant below
  for (double target : {0.10, 0.15, 0.20, 0.30, 0.45, -0.20, -0.35})
  {
    mh::WheelRateState st{};
    const double measured = settle(target, 300.0, 40.0, p, st, 600);
    EXPECT_NEAR(measured, target, 0.01) << "target " << target << " settled at " << measured;
  }
}

// Deadband bridging: with the feedforward DELIBERATELY undershooting (no
// feedforward deadband, low slope) and adaptation OFF, the integrator alone
// must ramp the PWM past the plant's static friction and hold the target — the
// exact property the firmware integrator used to provide.
TEST(WheelRateController, IntegratorBridgesDeadband)
{
  mh::WheelRateParams p;
  p.adapt_enabled = false;
  p.kff_init = 150.0;     // half the true slope
  p.deadband_init = 0.0;  // no feedforward deadband at all
  mh::WheelRateState st{};
  const double measured = settle(0.20, 300.0, 40.0, p, st, 800);
  EXPECT_NEAR(measured, 0.20, 0.01) << "integrator failed to bridge deadband (got " << measured
                                    << ")";
}

// THE CALIBRATION FIX: with the seed wrong (plant needs more PWM than the seed
// feedforward supplies), online adaptation must fold the steady-state
// correction into the feedforward so that (a) the speed still tracks, (b) the
// feedforward alone ≈ the required PWM, and (c) the integral unloads to ~0.
TEST(WheelRateController, OnlineCalibrationLearnsFeedforward)
{
  const double kff_phys = 380.0, db_phys = 55.0;
  const double target = 0.30;
  const double required_pwm = kff_phys * target + db_phys;  // = 169

  mh::WheelRateParams p;  // adaptation on by default
  mh::WheelRateState st{};
  const double measured = settle(target, kff_phys, db_phys, p, st, 4000);

  EXPECT_NEAR(measured, target, 0.01);
  // Feedforward alone should now reproduce the required PWM (the loop no longer
  // leans on the integral). The deadband/slope split is underdetermined from a
  // single operating point, so assert the COMBINED feedforward, not each term.
  const double ff_only = st.kff * target + st.deadband;
  EXPECT_NEAR(ff_only, required_pwm, 8.0) << "feedforward did not learn (ff=" << ff_only << ")";
  // Integral unloaded → small.
  EXPECT_LT(std::abs(st.integral), 15.0) << "integral not unloaded (" << st.integral << ")";
  // And it actually moved off the seed (learning happened).
  EXPECT_GT(st.kff, p.kff_init + 1.0);
}

// With adaptation disabled the learned feedforward must stay frozen at the
// seeds even though the loop is actively bridging via the integrator.
TEST(WheelRateController, NoAdaptationWhenDisabled)
{
  mh::WheelRateParams p;
  p.adapt_enabled = false;
  mh::WheelRateState st{};
  settle(0.30, 380.0, 55.0, p, st, 2000);
  EXPECT_DOUBLE_EQ(st.kff, p.kff_init);
  EXPECT_DOUBLE_EQ(st.deadband, p.deadband_init);
}

// Direction reversal must drop the opposite-direction integral and settle on
// the new (negative) target.
TEST(WheelRateController, SignFlipDropsStaleIntegral)
{
  mh::WheelRateParams p;
  p.adapt_enabled = false;  // isolate the PI behaviour from learning
  mh::WheelRateState st{};
  double measured = settle(0.30, 300.0, 40.0, p, st, 300);
  EXPECT_GT(st.integral, -1.0);  // wound up non-negative for a forward target
  for (int i = 0; i < 600; ++i)
  {
    measured = sim_wheel_velocity(mh::compute_wheel_pwm(-0.30, measured, kDt, p, st), 300.0, 40.0);
  }
  EXPECT_LE(st.integral, 0.0);
  EXPECT_NEAR(measured, -0.30, 0.01);
}

// Zero / dust target → exactly zero output and the integrator is cleared.
TEST(WheelRateController, ZeroTargetStopsAndResets)
{
  mh::WheelRateParams p;
  mh::WheelRateState st{};
  st.kff = 300.0;  // pretend already seeded
  st.deadband = 40.0;
  st.integral = 80.0;  // pretend a prior drive wound this up
  st.last_target = 0.3;
  EXPECT_DOUBLE_EQ(mh::compute_wheel_pwm(0.0, 0.2, kDt, p, st), 0.0);
  EXPECT_DOUBLE_EQ(st.integral, 0.0);
  EXPECT_DOUBLE_EQ(mh::compute_wheel_pwm(5.0e-4, 0.0, kDt, p, st), 0.0);
}

// Anti-windup: a permanently stalled wheel (sim never moves) must not let the
// integral or the emitted PWM grow without bound.
TEST(WheelRateController, AntiWindupClampsStalledOutput)
{
  mh::WheelRateParams p;
  mh::WheelRateState st{};
  double pwm = 0.0;
  for (int i = 0; i < 2000; ++i)
  {
    pwm = mh::compute_wheel_pwm(0.30, 0.0 /* never moves */, kDt, p, st);
  }
  EXPECT_LE(std::abs(pwm), p.max_pwm + 1e-9);
  EXPECT_LE(std::abs(st.integral), p.integral_max + 1e-9);
}

// First call lazily seeds the learned feedforward from the params.
TEST(WheelRateController, SeedsFeedforwardFromParamsOnFirstCall)
{
  mh::WheelRateParams p;
  p.kff_init = 275.0;
  p.deadband_init = 33.0;
  mh::WheelRateState st{};  // kff == 0 sentinel
  mh::compute_wheel_pwm(0.20, 0.0, kDt, p, st);
  EXPECT_DOUBLE_EQ(st.kff, 275.0);
  EXPECT_DOUBLE_EQ(st.deadband, 33.0);
}
