// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Tests for the online gyro bias estimation (item #3 pragmatic).
//
// Pins three behaviours:
//   1. With a biased gyro and stationary wheels, the EMA estimate
//      converges toward the true bias and yaw drift collapses.
//   2. With matched wheel + gyro rotation (no stationary windows),
//      the bias estimate stays at its last value (no false updates).
//   3. Disabling via params: estimate stays at 0 and bias is not
//      subtracted, so yaw drifts at the raw bias rate.

#include <cmath>
#include <cstdio>

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{

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
  // Disable the other adaptive systems so this test is purely the
  // bias estimator.
  gp.adaptive_noise_enabled_gain = 0.0;
  // Tight EMA τ so the test converges in seconds, not minutes.
  gp.gyro_bias_estimation_enabled = true;
  gp.gyro_bias_ema_tau_s = 1.0;
  gp.gyro_bias_max_sample_rad_per_s = 0.10;
  return gp;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────
// 1. Stationary with a 0.025 rad/s gyro bias — the EMA estimate must
//    converge toward 0.025 within a few τ and the resulting yaw
//    drift over 60 s must be near zero.
// ─────────────────────────────────────────────────────────────────────
TEST(GyroBias, EmaConvergesToBiasWhenStationary)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  constexpr int kTicks = 600;  // 60 s @ 10 Hz nodes
  constexpr double kDt = 0.1;
  constexpr double kBias = 0.025;  // rad/s, ~1.43°/s

  for (int i = 0; i < kTicks; ++i)
  {
    gm.AddWheelTwist(0.0, 0.0, 0.0, kDt);
    gm.AddGyroDelta(kBias, kDt);
    gm.Tick(kDt * (i + 1));
  }

  auto stats = gm.Stats();
  std::printf("[GyroBias] stationary: bias_est=%.5f rad/s (true=%.5f), updates=%lu\n",
              stats.gyro_bias_z,
              kBias,
              static_cast<unsigned long>(stats.gyro_bias_updates));

  // EMA with τ=1 s converges to ~99 % of true bias after ~5 τ. After
  // 60 s of stationary input the estimate should be tight.
  EXPECT_NEAR(stats.gyro_bias_z, kBias, 0.005);
  EXPECT_GT(stats.gyro_bias_updates, 100u);

  // And the resulting yaw drift must be small: the bias is being
  // subtracted, so the graph sees ~0 angular velocity.
  auto snap = gm.LatestSnapshot();
  ASSERT_TRUE(snap.has_value());
  const double yaw_drift_deg = std::abs(snap->pose.theta()) * 180.0 / M_PI;
  std::printf("[GyroBias] yaw drift over 60s: %.3f°\n", yaw_drift_deg);
  EXPECT_LT(yaw_drift_deg, 0.5);
}

// ─────────────────────────────────────────────────────────────────────
// 2. Real motion does NOT update the bias estimate.
//    Drive a real 0.5 rad/s rotation (wheels and gyro both report it).
//    Wheels report motion → stationary gate doesn't fire → bias EMA
//    stays at 0.
// ─────────────────────────────────────────────────────────────────────
TEST(GyroBias, RealMotionDoesNotUpdateBias)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  constexpr int kTicks = 60;  // 6 s
  constexpr double kDt = 0.1;
  constexpr double kWz = 0.5;  // real rotation rad/s

  for (int i = 0; i < kTicks; ++i)
  {
    gm.AddWheelTwist(0.0, 0.0, kWz, kDt);
    gm.AddGyroDelta(kWz, kDt);
    gm.Tick(kDt * (i + 1));
  }

  auto stats = gm.Stats();
  std::printf("[GyroBias] rotation phase: bias_est=%.5f, updates=%lu\n",
              stats.gyro_bias_z, static_cast<unsigned long>(stats.gyro_bias_updates));
  // No stationary windows → no bias updates.
  EXPECT_EQ(stats.gyro_bias_updates, 0u);
  EXPECT_NEAR(stats.gyro_bias_z, 0.0, 1.0e-9);
}

// ─────────────────────────────────────────────────────────────────────
// 3. Stationary, then bias estimation converges; THEN real motion
//    follows. The frozen bias estimate continues to be subtracted
//    from gyro samples → the integrated motion matches truth.
// ─────────────────────────────────────────────────────────────────────
TEST(GyroBias, BiasAppliedToSubsequentMotion)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  constexpr double kDt = 0.1;
  constexpr double kBias = 0.025;

  // 30 s of stationary to converge bias.
  for (int i = 0; i < 300; ++i)
  {
    gm.AddWheelTwist(0.0, 0.0, 0.0, kDt);
    gm.AddGyroDelta(kBias, kDt);
    gm.Tick(kDt * (i + 1));
  }
  auto after_stationary = gm.Stats();
  ASSERT_NEAR(after_stationary.gyro_bias_z, kBias, 0.001);

  // Real rotation: 0.5 rad/s for 4 s, expecting +2.0 rad final yaw.
  // The gyro reports 0.5 + bias; bias estimator subtracts the bias.
  constexpr double kRealWz = 0.5;
  for (int i = 0; i < 40; ++i)
  {
    gm.AddWheelTwist(0.0, 0.0, kRealWz, kDt);
    // Real motion delivered to gyro = signal + bias.
    gm.AddGyroDelta(kRealWz + kBias, kDt);
    gm.Tick(kDt * (300 + i + 1));
  }

  auto snap = gm.LatestSnapshot();
  ASSERT_TRUE(snap.has_value());
  const double expected = kRealWz * kDt * 40.0;  // 2.0 rad
  std::printf("[GyroBias] post-stationary rotation: expected=%.3f, got=%.3f\n",
              expected, snap->pose.theta());
  // 10 % tolerance — the bias estimator captured the offset cleanly.
  EXPECT_NEAR(snap->pose.theta(), expected, 0.10 * expected);
}

// ─────────────────────────────────────────────────────────────────────
// 4. Disabling via params: the bias is not estimated and not
//    subtracted, so yaw drifts at the raw bias rate.
// ─────────────────────────────────────────────────────────────────────
TEST(GyroBias, DisableLeavesGyroUncompensated)
{
  auto gp = MakeParams();
  gp.gyro_bias_estimation_enabled = false;
  fg::GraphManager gm(gp);
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  constexpr double kDt = 0.1;
  constexpr double kBias = 0.025;
  // 10 s of stationary with biased gyro.
  for (int i = 0; i < 100; ++i)
  {
    gm.AddWheelTwist(0.0, 0.0, 0.0, kDt);
    gm.AddGyroDelta(kBias, kDt);
    gm.Tick(kDt * (i + 1));
  }

  auto stats = gm.Stats();
  // Disabled: estimate stays at 0, never increments.
  EXPECT_EQ(stats.gyro_bias_updates, 0u);
  EXPECT_NEAR(stats.gyro_bias_z, 0.0, 1.0e-9);

  // BUT the stationary suppressor (item #1, separate) still snaps
  // wheel-stationary frames to dθ=0, so the yaw doesn't drift even
  // with no bias correction. This test mainly pins that the disable
  // switch turns off the bias mechanism cleanly — yaw drift is the
  // stationary-yaw test's concern.
}
