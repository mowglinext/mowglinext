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
// First iteration (PR #161) gated the suppressor on
// wheel_stationary AND gyro_stationary. On-robot post-deploy:
//
//   fusion_graph_node yaw drift = -4.28°/min (worse than no fix at all)
//
// Live IMU /imu/data publishes wz ≈ -0.023 rad/s (-1.32°/s) at rest on
// this robot — hardware_bridge calibration neutralises the cold bias
// but not the thermal drift — so |dtheta_gyro| was always ~10× above
// any sensible stationary_thresh_theta, the AND fell through, and the
// gyro path ran like before. Worse, the original test used a
// 0.0002 rad/s bias which masked the AND-gate hiding bug entirely.
//
// Second iteration (this file): drop the gyro_stationary check.
// Encoders cannot slip "into" stationary — when |dx|, |dy|,
// |dtheta_wheel| are all below thresholds the robot is genuinely
// parked, so trust the wheel and snap dtheta=0 regardless of gyro
// noise.
//
// These tests pin the post-fix behaviour:
//   1. With encoders idle and a realistic 0.025 rad/s gyro bias (the
//      live residual order of magnitude) the graph yaw must stay
//      within ~0.05° over a 60 s parked window. The previous test's
//      0.0002 rad/s bias did NOT exercise the AND gate; this version
//      would have failed against the PR #161 code.
//   2. With matching wheel + gyro rotation the graph still tracks the
//      input rotation (the stationary path only kicks in when the
//      wheel encoder reports no motion).

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
// 60 s of stationary operation with a 0.025 rad/s (~1.43°/s) residual
// gyro bias — the order of magnitude actually seen on /imu/data after
// hardware_bridge calibration on this robot. Crucially this is well
// above stationary_thresh_theta (2 mrad/tick → 20 mrad/s @ 10 Hz), so
// any AND-gated suppressor (PR #161) would fail the test. The wheel-
// only suppressor must hold yaw to within 0.05°.
// ─────────────────────────────────────────────────────────────────────
TEST(StationaryYaw, GyroBiasDoesNotDriftMapYaw)
{
  fg::GraphManager gm(MakeParams());
  // Initialize at a non-zero pose so the test isn't accidentally
  // hitting any "near-origin" early-exit path in iSAM2 / Pose2.
  const gtsam::Pose2 X0(1.0, 2.0, 0.5);
  gm.Initialize(X0, 0.0);

  // 600 ticks × 0.1 s = 60 s. 0.025 rad/s ≈ 1.43°/s gyro bias —
  // matches the live residual measured on /imu/data with the robot
  // parked at the dock.
  constexpr int kTicks = 600;
  constexpr double kDt = 0.1;
  constexpr double kGyroBias = 0.025;  // rad/s, ~1.43°/s

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

  // Hard ceiling: 0.05° over 60 s. With the AND-gated PR #161 code
  // this test would drift ~86° (60 s × 1.43°/s) — orders of magnitude
  // out — so the bound is a clean regression catch.
  EXPECT_LT(yaw_drift_deg, 0.05);
}

// ─────────────────────────────────────────────────────────────────────
// Real rotation must still be tracked: the suppressor only fires when
// the wheel encoders report no motion. Drive a 0.5 rad/s rotation for
// 6 s (wheels + gyro both report it) and verify the graph yaw lands
// within ~10% of the expected angle.
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
