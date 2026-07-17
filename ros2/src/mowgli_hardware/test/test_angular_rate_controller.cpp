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
//
// ki is set explicitly here: this test exercises the INTEGRAL mechanism,
// which is still available (config-controlled) but is no longer the
// production default. 2026-07-17 WEAVE FIX (Option B, task #24): the
// integrator was found to fight the per-wheel firmware PIs and cause a
// left-right weave, so the shipped default is now ki=0 (feedforward-
// dominant, see DefaultsAreFeedforwardDominantNoIntegrator below) —
// UNVALIDATED ON HARDWARE, see angular_rate_controller.hpp.
TEST(AngularRateController, ConvergesAcrossNonlinearCurve)
{
  mh::AngularRateParams p;
  p.ki = 2.0;  // exercise the integral mechanism explicitly (see above)
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
//
// ki explicit — see ConvergesAcrossNonlinearCurve above for why (the
// production default is now ki=0, task #24).
TEST(AngularRateController, BoostsSubDeadbandCommandWithinLimits)
{
  mh::AngularRateParams p;
  p.ki = 2.0;
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
// ki explicit — see ConvergesAcrossNonlinearCurve above for why (the
// production default is now ki=0, task #24; with no integrator there is no
// stale integral to drop).
TEST(AngularRateController, SignFlipDropsStaleIntegral)
{
  mh::AngularRateParams p;
  p.ki = 2.0;
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
// ki explicit — see ConvergesAcrossNonlinearCurve above for why (the
// production default is now ki=0, task #24). Without the integrator the
// feedforward-only loop has steady-state error against this deadband model,
// so it would not track the net target closely enough for this assertion.
TEST(AngularRateController, DitherTargetIsSmoothed)
{
  mh::AngularRateParams p;  // tau=0.2 default
  p.ki = 2.0;
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
//
// ki explicit — see ConvergesAcrossNonlinearCurve above for why (the
// production default is now ki=0, task #24; with no integrator this clamp
// is not exercised, so the mechanism is tested directly here instead).
TEST(AngularRateController, AntiWindupClampsStalledOutput)
{
  mh::AngularRateParams p;
  p.ki = 2.0;
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

// 2026-07-17 WEAVE FIX (Option B, task #24, USER-APPROVED sign-off on the
// task #8 design proposal). *** UNVALIDATED ON HARDWARE — see the header
// comment in angular_rate_controller.hpp for the required bench + blade-off
// validation sequence before any blade-on use. ***
//
// This locks in what the SHIPPED DEFAULTS actually do now: ki=0 removes the
// host integrator from the 3-loop cascade (FTC heading PID -> this loop ->
// per-wheel firmware PIs) identified as the weave's root cause, and kff is
// refit toward the measured inverse gain (~1.35) to do the deadband + gain
// compensation open-loop instead. Two consequences to keep visible in tests:
//   1. the integral state must stay inert (never grows) under a sustained
//      tracking error — the removed loop really is gone;
//   2. unlike the ki>0 mechanism exercised above, this does NOT drive
//      steady-state error to zero — that tradeoff (stability over exact
//      tracking) is the whole point of Option B.
TEST(AngularRateController, DefaultsAreFeedforwardDominantNoIntegrator)
{
  mh::AngularRateParams p;  // production defaults
  EXPECT_DOUBLE_EQ(p.ki, 0.0);
  EXPECT_NEAR(p.kff, 1.35, 1e-9);
  mh::AngularRateState st{};
  double measured = 0.0;
  for (int i = 0; i < 400; ++i)
  {
    measured = sim_firmware_rate(mh::compute_angular_rate_cmd(0.3, measured, 0.05, p, st));
  }
  // No integrator: the integral state never leaves zero regardless of a
  // sustained tracking error.
  EXPECT_DOUBLE_EQ(st.integral, 0.0);
  // Feedforward + a small kp get close but do not close the gap exactly.
  EXPECT_NEAR(measured, 0.3, 0.1);
  EXPECT_LT(measured, 0.3);  // steady-state error is real, not eliminated
}
