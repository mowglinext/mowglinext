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
// proxy for slip. We EMA-smooth it and add `gain * net_residual` (in
// metres per radian) on top of the configured σ_x baseline.
//
// Tests pin three behaviours:
//   1. Matched wheel + gyro (no slip)         → σ_x stays at baseline.
//   2. Sustained wheel↔gyro disagreement (slip) → σ_x inflates.
//   3. Slip event followed by quiet           → EMA decays back, σ_x
//                                                returns near baseline.

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
  gp.wheel_sigma_x = 0.05;
  gp.wheel_sigma_y = 0.005;
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
  return gp;
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
  // σ_x_eff must equal the configured wheel_sigma_x within a small
  // numeric epsilon — adaptive gain × (residual − floor) must be 0.
  EXPECT_NEAR(stats.wheel_sigma_x_eff, 0.05, 1.0e-6);
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

  // Per-tick residual is |kWheelWz - kGyroWz| × kDt = 0.03 rad. After
  // many τ time constants of accumulation the EMA should settle near
  // 0.03 rad. Allow 20% tolerance for the τ=0.5 s settling response
  // over 5 s of input.
  EXPECT_GT(stats.residual_ema_rad, 0.020);
  EXPECT_LT(stats.residual_ema_rad, 0.040);

  // σ_x_eff = baseline + gain × (residual − floor) ≈ 0.05 + 10 × 0.025 = 0.30 m
  // Wide bounds — this just checks "inflated meaningfully".
  EXPECT_GT(stats.wheel_sigma_x_eff, 0.15);
  EXPECT_LT(stats.wheel_sigma_x_eff, 0.50);
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

  // 10 τ of zero residual collapses the EMA to << floor; σ_x returns
  // to the configured baseline (0.05).
  EXPECT_LT(recovered.residual_ema_rad, 0.001);
  EXPECT_NEAR(recovered.wheel_sigma_x_eff, 0.05, 1.0e-6);
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

  // EMA still tracks (it's a passive measurement), but σ_x_eff
  // must equal the baseline — the gain=0 short-circuits inflation.
  EXPECT_NEAR(stats.wheel_sigma_x_eff, 0.05, 1.0e-6);
}
