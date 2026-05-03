// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Regression tests for the stationary-yaw drift suppressor in
// GraphManager::CreateNodeLocked. Motivating failure (measured on the
// robot 2026-05-03 with the dock charging, RPM=0, encoders idle):
//
//   ekf_odom_node (odom-frame)   yaw drift = -0.033°/min   (correct)
//   fusion_graph_node (map-frame) yaw drift = +0.43°/min    (~13× worse)
//
// The map-frame factor graph was selecting the wheel/gyro yaw delta
// with a single ternary on |dtheta_gyro| > 1e-9. The residual gyro
// bias (~0.011°/s mean post-hardware_bridge calibration) integrates
// to ~0.0011° per node tick and always trips that gate, so every
// BetweenFactor got the gyro delta even when the wheel encoder
// unambiguously reported "no motion". iSAM2 then propagated the small
// biased delta across nodes, giving the observed drift.
//
// These tests pin the post-fix behaviour:
//   1. With encoders idle and a realistic gyro-bias drift the graph
//      yaw must stay within ~0.05° over a 60 s parked window (pre-fix
//      ≈ 0.66°).
//   2. With matching wheel + gyro rotation the graph still tracks the
//      input rotation (the stationary path only kicks in when both
//      sources agree the robot is at rest).

#include <cmath>
#include <cstdio>

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{

// Reasonable-but-not-tightest defaults. Mirrors the YAML so test
// failures bisect to graph_manager.cpp logic, not param differences.
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
  // Bypass the node-creation throttle so every Tick produces a node —
  // this is a yaw-drift test, not a cadence test.
  gp.stationary_node_period_s = 0.0;
  gp.stationary_motion_thresh_m = 0.0;
  gp.stationary_motion_thresh_theta = 0.0;
  return gp;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────
// 60 s of stationary operation with a 0.011°/s residual gyro bias
// (a typical post-calibration value from hardware_bridge_node). The
// pre-fix code drifted ~0.66°; the suppressor must hold yaw to within
// 0.05°.
// ─────────────────────────────────────────────────────────────────────
TEST(StationaryYaw, GyroBiasDoesNotDriftMapYaw)
{
  fg::GraphManager gm(MakeParams());
  // Initialize at a non-zero pose so the test isn't accidentally
  // hitting any "near-origin" early-exit path in iSAM2 / Pose2.
  const gtsam::Pose2 X0(1.0, 2.0, 0.5);
  gm.Initialize(X0, 0.0);

  // 600 ticks × 0.1 s = 60 s. 0.011°/s ≈ 0.0001920 rad/s gyro bias.
  constexpr int kTicks = 600;
  constexpr double kDt = 0.1;
  constexpr double kGyroBias = 0.0001920;  // rad/s, ~0.011°/s

  for (int i = 0; i < kTicks; ++i)
  {
    // Encoders flat — robot is parked.
    gm.AddWheelTwist(0.0, 0.0, 0.0, kDt);
    // Gyro reports the residual bias.
    gm.AddGyroDelta(kGyroBias, kDt);
    gm.Tick(kDt * (i + 1));
  }

  auto snap = gm.LatestSnapshot();
  ASSERT_TRUE(snap.has_value());
  const double yaw_drift_rad = std::abs(snap->pose.theta() - X0.theta());
  const double yaw_drift_deg = yaw_drift_rad * 180.0 / M_PI;

  std::printf("[StationaryYaw] 60 s parked, bias=%.4f °/s, drift=%.3f° (%.5f rad)\n",
              kGyroBias * 180.0 / M_PI,
              yaw_drift_deg,
              yaw_drift_rad);

  // Hard ceiling: 0.05° over 60 s. Pre-fix value was ~0.66° in the
  // same setup, so this gives ~13× headroom.
  EXPECT_LT(yaw_drift_deg, 0.05);
}

// ─────────────────────────────────────────────────────────────────────
// Real rotation must still be tracked: the suppressor is only allowed
// to fire when BOTH the wheel encoders AND the gyro agree the robot
// is at rest. Drive a 0.5 rad/s rotation for 6 s and verify the graph
// yaw lands within ~10% of the expected angle.
// ─────────────────────────────────────────────────────────────────────
TEST(StationaryYaw, RotationStillTracked)
{
  fg::GraphManager gm(MakeParams());
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);

  constexpr int kTicks = 60;  // 6 s @ 10 Hz
  constexpr double kDt = 0.1;
  constexpr double kWz = 0.5;  // rad/s

  for (int i = 0; i < kTicks; ++i)
  {
    gm.AddWheelTwist(0.0, 0.0, kWz, kDt);
    gm.AddGyroDelta(kWz, kDt);
    gm.Tick(kDt * (i + 1));
  }

  auto snap = gm.LatestSnapshot();
  ASSERT_TRUE(snap.has_value());
  const double expected = kWz * kDt * kTicks;  // 3.0 rad
  const double got = snap->pose.theta();
  // Pose2 wraps to (-pi, pi]; expected 3.0 rad is inside that range.
  std::printf("[StationaryYaw] 6 s @ %.2f rad/s: expected=%.3f rad, got=%.3f rad\n",
              kWz,
              expected,
              got);

  // 10 % tolerance — between-factor noise + iSAM2 relinearization can
  // pull a few percent. The point is to catch a regression where the
  // suppressor wrongly zeroes real motion.
  EXPECT_NEAR(got, expected, 0.1 * expected);
}
