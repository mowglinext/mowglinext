// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Regression tests for the adaptive process-noise inflation on wheel
// σ_x. Motivating failure mode: encoders over-report distance under
// slip (wet grass, slope, blade-jam recoil); without inflating σ_x
// dynamically, iSAM2's wheel between-factor pins the trajectory to
// the bogus delta until a downstream sensor (GPS / ICP) corrects.
//
// The mechanism observed: |dtheta_wheel - dtheta_gyro| is a per-tick
// proxy for slip. We EMA-smooth it and use `gain * net_residual` (in
// metres per radian) as a per-node FLOOR under the baseline σ_x.
//
// Updated 2026-08-24 (issue #491): the σ_x baseline is no longer a fixed
// per-node constant — it is a distance-scaled random walk,
// σ = k·√(step_m + creep·dt). The adaptive term deliberately stayed a
// per-node absolute release (it models a wheel FAULT, not a distance-driven
// random walk), so it is now applied as max(travel, release) instead of
// baseline + release. The "baseline" the tests below compare against is
// therefore ExpectedTravelSigmaX() rather than a literal 0.05.
//
// Tests pin four behaviours:
//   1. Matched wheel + gyro (no slip)         → σ_x stays at the travel
//                                                baseline.
//   2. Sustained wheel↔gyro disagreement (slip) → σ_x inflates.
//   3. Slip event followed by quiet           → EMA decays back, σ_x
//                                                returns to the travel
//                                                baseline.
//   4. gain = 0                               → no inflation at all.

#include <cmath>
#include <cstdio>

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{

// Defaults mirror the YAML so failures bisect to logic, not params.
fg::GraphParams MakeParams()
{
  fg::GraphParams gp;
  gp.node_period_s = 0.1;
  gp.wheel_sigma_x_per_sqrt_m = 0.05;
  gp.wheel_sigma_y_per_sqrt_m = 0.005;
  gp.wheel_sigma_theta = 0.01;
  gp.gyro_sigma_theta = 0.005;
  gp.stationary_thresh_xy_m = 1.0e-3;
  gp.stationary_thresh_theta = 2.0e-3;
  gp.stationary_sigma_theta = 1.0e-3;
  gp.stationary_node_period_s = 0.0;
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  gp.adaptive_noise_enabled_gain = 10.0;
  gp.adaptive_noise_ema_tau_s = 0.5;
  gp.adaptive_noise_residual_floor_rad = 0.005;
  gp.wheel_creep_speed_mps = 0.04;
  return gp;
}

// The no-fault σ_x for a node that advanced `step_m` metres over `dt`
// seconds. Mirrors the model in CreateNodeLocked so a params change here
// stays in step with the code under test.
double ExpectedTravelSigmaX(double step_m, double dt)
{
  const auto gp = MakeParams();
  return gp.wheel_sigma_x_per_sqrt_m * std::sqrt(step_m + gp.wheel_creep_speed_mps * dt);
}

// These tests tick at 0.1 s against node_period_s = 0.1, a comparison that
// drifts in binary float, so a node absorbs ONE OR TWO wheel samples. With
// σ_x now scaled by the step, that makes the exact value ambiguous — assert
// the band the travel model allows instead. The old per-node constant
// (0.05 m) sits an order of magnitude above the whole band either way, so the
// regression this pins is not weakened.
void ExpectTravelBaseline(double sigma_x_eff, double step_per_sample_m, double dt_per_sample)
{
  const double lo = ExpectedTravelSigmaX(step_per_sample_m, dt_per_sample);
  const double hi = ExpectedTravelSigmaX(2.0 * step_per_sample_m, 2.0 * dt_per_sample);
  EXPECT_GE(sigma_x_eff, lo - 1.0e-9);
  EXPECT_LE(sigma_x_eff, hi + 1.0e-9);
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────
// 1. No slip — wheel and gyro agree, σ_x must stay at baseline.
// ─────────────────────────────────────────────────────────────────────
TEST(AdaptiveNoise, NoSlipKeepsBaselineSigma)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  constexpr int kTicks = 50;  // 5 s @ 10 Hz
  constexpr double kDt = 0.1;
  constexpr double kVx = 0.20;
  constexpr double kWz = 0.05;  // gentle turn, well above pivot gate

  for (int i = 0; i < kTicks; ++i)
  {
    gm.AddWheelTwist(kVx, 0.0, kWz, kDt);
    // Gyro matches wheel exactly — no residual.
    gm.AddGyroDelta(kWz, kDt);
    gm.Tick(kDt * (i + 1));
  }

  auto stats = gm.Stats();
  std::printf("[AdaptiveNoise] no-slip: residual_ema=%.6f rad, σ_x_eff=%.4f m\n",
              stats.residual_ema_rad,
              stats.wheel_sigma_x_eff);

  // Residual EMA must stay under the floor (no inflation kicks in).
  EXPECT_LT(stats.residual_ema_rad, 0.005);
  // σ_x_eff must sit on the travel-scaled baseline — adaptive gain ×
  // (residual − floor) is 0, so the release floor is 0 and
  // max(travel, 0) == travel.
  ExpectTravelBaseline(stats.wheel_sigma_x_eff, kVx * kDt, kDt);
}

// ─────────────────────────────────────────────────────────────────────
// 2. Slip — wheel reports a forward rotation that the gyro doesn't see.
//    The residual EMA must climb, and σ_x must inflate proportionally.
// ─────────────────────────────────────────────────────────────────────
TEST(AdaptiveNoise, SlipInflatesSigma)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  constexpr int kTicks = 50;
  constexpr double kDt = 0.1;
  // Wheel says we're rotating at 0.3 rad/s. Gyro reports 0.0 — the
  // robot is slipping on a slope and one encoder is spinning free.
  constexpr double kWheelWz = 0.30;
  constexpr double kGyroWz = 0.0;

  for (int i = 0; i < kTicks; ++i)
  {
    gm.AddWheelTwist(0.0, 0.0, kWheelWz, kDt);
    gm.AddGyroDelta(kGyroWz, kDt);
    gm.Tick(kDt * (i + 1));
  }

  auto stats = gm.Stats();
  std::printf("[AdaptiveNoise] slip: residual_ema=%.4f rad, σ_x_eff=%.4f m\n",
              stats.residual_ema_rad,
              stats.wheel_sigma_x_eff);

  // The raw per-tick wheel↔gyro disagreement is |kWheelWz - kGyroWz| × kDt
  // = 0.03 rad, but the residual the graph actually tracks settles higher:
  // with the robot XY-stationary (vx=0) under a sustained wheel-only
  // rotation, the gyro-bias estimator pulls wz_corrected slightly negative,
  // so the steady-state EMA lands near 0.055 rad (measured), not 0.03. The
  // point of this test is "slip inflates σ_x meaningfully", so the bounds
  // are deliberately loose around the observed value.
  EXPECT_GT(stats.residual_ema_rad, 0.020);
  EXPECT_LT(stats.residual_ema_rad, 0.070);

  // σ_x_eff = max(travel, gain × (residual − floor)) ≈ 10 × 0.05 ≈ 0.5 m —
  // the release dominates the ~3 mm travel term by two orders of magnitude.
  // Wide bounds — this just checks "inflated meaningfully".
  EXPECT_GT(stats.wheel_sigma_x_eff, 0.15);
  EXPECT_LT(stats.wheel_sigma_x_eff, 0.65);
}

// ─────────────────────────────────────────────────────────────────────
// 3. Slip event followed by recovery — σ_x must decay back near
//    baseline once the EMA bleeds out.
// ─────────────────────────────────────────────────────────────────────
TEST(AdaptiveNoise, EmaDecaysAfterSlipEnds)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  constexpr double kDt = 0.1;

  // 2 s of slip: σ_x ramps up.
  for (int i = 0; i < 20; ++i)
  {
    gm.AddWheelTwist(0.0, 0.0, 0.30, kDt);
    gm.AddGyroDelta(0.0, kDt);
    gm.Tick(kDt * (i + 1));
  }
  auto peak = gm.Stats();
  std::printf("[AdaptiveNoise] peak slip: residual=%.4f rad, σ_x_eff=%.4f m\n",
              peak.residual_ema_rad,
              peak.wheel_sigma_x_eff);
  ASSERT_GT(peak.wheel_sigma_x_eff, 0.15);

  // 5 s of recovery: matched wheel+gyro at zero. EMA should decay
  // toward zero with τ=0.5 s — 10 τ = essentially zero.
  for (int i = 0; i < 50; ++i)
  {
    gm.AddWheelTwist(0.0, 0.0, 0.0, kDt);
    gm.AddGyroDelta(0.0, kDt);
    gm.Tick(kDt * (20 + i + 1));
  }
  auto recovered = gm.Stats();
  std::printf("[AdaptiveNoise] recovered: residual=%.6f rad, σ_x_eff=%.4f m\n",
              recovered.residual_ema_rad,
              recovered.wheel_sigma_x_eff);

  // 10 τ of zero residual collapses the EMA to << floor; σ_x returns to the
  // travel baseline for a zero-length step (creep floor only).
  EXPECT_LT(recovered.residual_ema_rad, 0.001);
  ExpectTravelBaseline(recovered.wheel_sigma_x_eff, 0.0, kDt);
}

// ─────────────────────────────────────────────────────────────────────
// 4. Disabling adaptive noise (gain = 0) — even with slip, σ_x must
//    stay at baseline. Pins the bypass path so we can disable in
//    production via yaml without code changes.
// ─────────────────────────────────────────────────────────────────────
TEST(AdaptiveNoise, GainZeroDisablesAdaptation)
{
  auto gp = MakeParams();
  gp.adaptive_noise_enabled_gain = 0.0;
  fg::GraphManager gm(gp);
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  constexpr double kDt = 0.1;
  for (int i = 0; i < 50; ++i)
  {
    gm.AddWheelTwist(0.0, 0.0, 0.30, kDt);
    gm.AddGyroDelta(0.0, kDt);
    gm.Tick(kDt * (i + 1));
  }
  auto stats = gm.Stats();
  std::printf("[AdaptiveNoise] gain=0: residual=%.4f rad, σ_x_eff=%.4f m\n",
              stats.residual_ema_rad,
              stats.wheel_sigma_x_eff);

  // EMA still tracks (it's a passive measurement), but σ_x_eff must sit on
  // the travel baseline — gain=0 short-circuits inflation. The wheels claim
  // rotation only (vx = 0), so the step is zero and only the creep floor
  // contributes.
  ExpectTravelBaseline(stats.wheel_sigma_x_eff, 0.0, kDt);
}
