// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for compute_angular_rate_cmd. Pure math, no ROS plumbing.
//
// The decisive test drives the controller against a SIMULATED firmware that
// reproduces the on-robot 2026-05-27 measurements (soft deadband + ~0.7
// nonlinear gain) and asserts the closed loop converges the measured yaw rate
// onto the target — the property every open-loop amplitude hack lacked.

#include <cmath>

#include "mowgli_hardware/angular_rate_controller.hpp"
#include <gtest/gtest.h>

namespace mh = mowgli_hardware;

namespace
{

// Crude model of the firmware PWM→yaw-rate response measured on-robot:
// a soft deadband around 0.12 rad/s of command, then a ~0.72 gain. Saturates
// at the chassis max. Good enough to exercise the loop's convergence, not a
// physical fit.
double sim_firmware_rate(double cmd)
{
  const double s = (cmd < 0.0) ? -1.0 : 1.0;
  const double a = std::abs(cmd);
  const double deadband = 0.12;
  if (a <= deadband)
  {
    return 0.0;  // sub-deadband: buzz, no rotation
  }
  double rate = 0.72 * (a - deadband);
  const double max_rate = 1.2;
  if (rate > max_rate)
  {
    rate = max_rate;
  }
  return s * rate;
}

// Run the loop to steady state against the sim firmware and return the final
// measured rate. dt = 0.05 s (20 Hz cmd_vel), 400 ticks = 20 s.
double settle(double target, const mh::AngularRateParams& p, int ticks = 400)
{
  mh::AngularRateState st{};
  double measured = 0.0;
  for (int i = 0; i < ticks; ++i)
  {
    const double cmd = mh::compute_angular_rate_cmd(target, measured, 0.05, p, st);
    measured = sim_firmware_rate(cmd);
  }
  return measured;
}

}  // namespace

// THE FIX: against a nonlinear-gain + deadband firmware, the closed loop must
// converge the MEASURED rate onto the target — at every operating point,
// including the sub-deadband commands that open-loop passthrough left at
// ~36 % and the pulse approach left at ~17 %.
TEST(AngularRateController, ConvergesAcrossNonlinearCurve)
{
  mh::AngularRateParams p;  // defaults
  for (double target : {0.10, 0.15, 0.20, 0.30, 0.40, -0.20, -0.35})
  {
    const double measured = settle(target, p);
    EXPECT_NEAR(measured, target, 0.02)
        << "target " << target << " settled at " << measured;
  }
}

// The command the loop emits to achieve a sub-deadband target must be BOOSTED
// above the raw target (that is the whole point — the firmware needs more than
// the target to overcome its deadband+gain), but must not run away.
TEST(AngularRateController, BoostsSubDeadbandCommandWithinLimits)
{
  mh::AngularRateParams p;
  mh::AngularRateState st{};
  double measured = 0.0;
  double last_cmd = 0.0;
  for (int i = 0; i < 400; ++i)
  {
    last_cmd = mh::compute_angular_rate_cmd(0.15, measured, 0.05, p, st);
    measured = sim_firmware_rate(last_cmd);
  }
  // To get 0.15 actual out of a (0.72 gain, 0.12 deadband) firmware the
  // command must be ~ 0.15/0.72 + 0.12 ≈ 0.33 — clearly above the raw target.
  EXPECT_GT(last_cmd, 0.15);
  EXPECT_LE(std::abs(last_cmd), p.max_cmd);
}

// Zero / dust target → exactly zero output and the integrator is cleared so
// no residual creep carries into the next command.
TEST(AngularRateController, ZeroTargetStopsAndResets)
{
  mh::AngularRateParams p;
  mh::AngularRateState st{};
  st.integral = 0.9;  // pretend a prior spin wound this up
  st.last_target = 0.4;
  EXPECT_DOUBLE_EQ(mh::compute_angular_rate_cmd(0.0, 0.3, 0.05, p, st), 0.0);
  EXPECT_DOUBLE_EQ(st.integral, 0.0);
  EXPECT_DOUBLE_EQ(mh::compute_angular_rate_cmd(5.0e-4, 0.0, 0.05, p, st), 0.0);
}

// A sustained direction reversal must drop the opposite-spin integrator and
// settle on the new (negative) target. With the target low-pass the filtered
// target crosses zero a few ticks after the raw command flips, so the reset
// happens slightly later than the first reverse tick — but the steady state
// must be correct.
TEST(AngularRateController, SignFlipDropsStaleIntegral)
{
  mh::AngularRateParams p;
  mh::AngularRateState st{};
  double measured = 0.0;
  // Wind up positive.
  for (int i = 0; i < 200; ++i)
  {
    measured = sim_firmware_rate(mh::compute_angular_rate_cmd(0.3, measured, 0.05, p, st));
  }
  EXPECT_GT(st.integral, 0.0);
  // Sustained reverse: the filtered target crosses zero within a few tau and
  // the integrator must end up non-positive, then settle on the negative.
  for (int i = 0; i < 400; ++i)
  {
    measured = sim_firmware_rate(mh::compute_angular_rate_cmd(-0.3, measured, 0.05, p, st));
  }
  EXPECT_LE(st.integral, 0.0);
  EXPECT_NEAR(measured, -0.3, 0.02);
}

// THE DOCKING FIX: a dithering target (small left-right alternating
// corrections around a small net intent, like the graceful dock controller)
// must NOT produce wild left-right output pulsing. The target low-pass
// extracts the net intent so the emitted command is smooth and the measured
// rate tracks the NET target — not the jitter. Without the filter the
// deadband + sign-flip resets lose the net rotation and pulse the output.
TEST(AngularRateController, DitherTargetIsSmoothed)
{
  mh::AngularRateParams p;  // tau=0.2 default
  mh::AngularRateState st{};
  double measured = 0.0;
  double sum = 0.0, sumsq = 0.0, meas_sum = 0.0;
  int n = 0;
  // Net +0.10 rad/s with +/-0.15 sinusoidal jitter at 1.5 Hz.
  for (int i = 0; i < 600; ++i)
  {
    const double t = i * 0.05;
    const double target = 0.10 + 0.15 * std::sin(2.0 * M_PI * 1.5 * t);
    const double out = mh::compute_angular_rate_cmd(target, measured, 0.05, p, st);
    measured = sim_firmware_rate(out);
    if (i >= 200)  // steady-state window
    {
      sum += out;
      sumsq += out * out;
      meas_sum += measured;
      ++n;
    }
  }
  const double mean = sum / n;
  const double var = sumsq / n - mean * mean;
  const double stddev = std::sqrt(std::max(var, 0.0));
  const double meas_mean = meas_sum / n;
  // Output pulsing must be modest (raw dither amplitude is 0.15; the filtered
  // output stddev should be well under half that).
  EXPECT_LT(stddev, 0.07) << "output still pulsing (stddev " << stddev << ")";
  // And the mean measured rate must track the NET +0.10, not collapse to ~0
  // as it does without the filter (deadband eats the unfiltered dither).
  EXPECT_NEAR(meas_mean, 0.10, 0.03);
}

// Anti-windup: a permanently stalled chassis (sim returns 0 always) must not
// let the integrator — or the emitted command — grow without bound.
TEST(AngularRateController, AntiWindupClampsStalledOutput)
{
  mh::AngularRateParams p;
  mh::AngularRateState st{};
  double cmd = 0.0;
  for (int i = 0; i < 1000; ++i)
  {
    cmd = mh::compute_angular_rate_cmd(0.3, 0.0 /* never moves */, 0.05, p, st);
  }
  EXPECT_LE(std::abs(cmd), p.max_cmd + 1e-9);
  EXPECT_LE(std::abs(st.integral), p.integral_max + 1e-9);
}

// Passthrough sanity: with all gains neutralised (kff=1, kp=ki=0) and the
// target low-pass disabled (tau=0), the command equals the target — confirms
// the feed-forward path is wired right.
TEST(AngularRateController, FeedForwardOnlyEqualsTarget)
{
  mh::AngularRateParams p;
  p.kp = 0.0;
  p.ki = 0.0;
  p.kff = 1.0;
  p.target_lp_tau = 0.0;  // disable filter so a single call equals the target
  mh::AngularRateState st{};
  EXPECT_NEAR(mh::compute_angular_rate_cmd(0.25, 0.1, 0.05, p, st), 0.25, 1e-9);
}
