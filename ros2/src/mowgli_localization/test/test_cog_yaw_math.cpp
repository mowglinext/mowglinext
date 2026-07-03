// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// test_cog_yaw_math.cpp
//
// Unit tests for the COG-to-body-heading algebra in cog_yaw_math.hpp.
//
// WHY: the on-robot 2026-05-27 reverse+rotate teleop (commanded vx=-0.2,
// ωz=+0.3; IMU gyro steady +0.493 rad/s = ground-truth CCW) showed the
// fused map-frame yaw diverging the WRONG way (integrated to -218°) while
// the chassis physically rotated +112° CCW. Root cause: cog_to_imu applied
// the forward-form lever-arm correction during reverse motion, so the
// published COG yaw was ~73° off the true body heading and fought the gyro
// between-factor in fusion_graph.
//
// These tests synthesise a GPS displacement baseline by integrating the
// antenna world-frame path for a known body heading + turn rate, then assert
// compute_cog_body_yaw() recovers the true body heading (NOT ~180°+ off) in
// both forward and reverse, with arbitrary rotation sign / lever arm.

#include <cmath>
#include <tuple>

#include <gtest/gtest.h>

#include "mowgli_localization/cog_yaw_math.hpp"

using mowgli_localization::compute_cog_body_yaw;
using mowgli_localization::wrap_angle;

namespace
{
constexpr double kRad = M_PI / 180.0;

// Smallest signed angular difference a-b, wrapped to (-pi, pi].
double ang_diff(double a, double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

// Integrate the GPS antenna's world-frame path over a baseline of duration
// dt at constant turn rate omega, with the body ending at heading psi_end.
// Returns the (dx, dy) displacement that cog_to_imu would observe between
// the anchor sample and the current sample.
//
// Antenna body-frame velocity = (vx - omega*ry, omega*rx) with SIGNED vx.
std::tuple<double, double> simulate_baseline(double vx,
                                             double omega,
                                             double rx,
                                             double ry,
                                             double psi_end,
                                             double dt)
{
  const double vant_bx = vx - omega * ry;
  const double vant_by = omega * rx;
  const double psi_anchor = psi_end - omega * dt;
  const int n = 4000;
  double x = 0.0;
  double y = 0.0;
  for (int i = 0; i < n; ++i)
  {
    const double psi = psi_anchor + omega * dt * (i + 0.5) / n;
    const double wvx = vant_bx * std::cos(psi) - vant_by * std::sin(psi);
    const double wvy = vant_bx * std::sin(psi) + vant_by * std::cos(psi);
    x += wvx * (dt / n);
    y += wvy * (dt / n);
  }
  return {x, y};
}

// Run the full estimator over a simulated baseline and return the recovered
// body yaw. lever arm rx/ry default to the YardForce 500 mount.
double recover_yaw(double vx, double omega, double psi_end, double rx = 0.30, double ry = 0.0,
                   double dt = 0.5)
{
  const auto [dx, dy] = simulate_baseline(vx, omega, rx, ry, psi_end, dt);
  const int wheel_sign = (vx >= 0.0) ? 1 : -1;
  // The node uses omega_avg = 0.5*(wheel_omega + gyro_z); both equal omega here.
  return compute_cog_body_yaw(dx, dy, wheel_sign, omega, dt, vx, rx, ry);
}
}  // namespace

// ---------------------------------------------------------------------------
// Forward motion must be unchanged (regression guard for the unaffected path)
// ---------------------------------------------------------------------------

TEST(CogYawMath, ForwardStraightRecoversHeading)
{
  EXPECT_NEAR(ang_diff(recover_yaw(0.20, 0.0, 0.0), 0.0), 0.0, 0.1 * kRad);
  EXPECT_NEAR(ang_diff(recover_yaw(0.20, 0.0, 0.7), 0.7), 0.0, 0.1 * kRad);
}

TEST(CogYawMath, ForwardTurningRecoversHeading)
{
  // Forward + CCW and forward + CW, both should land on the true heading.
  EXPECT_NEAR(ang_diff(recover_yaw(0.20, 0.493, 0.0), 0.0), 0.0, 0.1 * kRad);
  EXPECT_NEAR(ang_diff(recover_yaw(0.30, -0.30, 1.2), 1.2), 0.0, 0.1 * kRad);
}

// ---------------------------------------------------------------------------
// Reverse motion — the bug
// ---------------------------------------------------------------------------

TEST(CogYawMath, ReverseStraightRecoversHeading)
{
  EXPECT_NEAR(ang_diff(recover_yaw(-0.20, 0.0, 0.0), 0.0), 0.0, 0.1 * kRad);
  EXPECT_NEAR(ang_diff(recover_yaw(-0.20, 0.0, -1.0), -1.0), 0.0, 0.1 * kRad);
}

// The 2026-05-27 incident: reverse (vx=-0.2 m/s) + CCW rotation
// (gyro +0.493 rad/s). The recovered yaw must equal the true body heading,
// NOT be ~73° off (the pre-fix forward-form lever-arm bug) and certainly not
// ~180° off.
TEST(CogYawMath, ReverseCcwRecoversHeadingNotOffBy73Deg)
{
  const double psi_true = 0.0;
  const double recovered = recover_yaw(-0.20, 0.493, psi_true);
  EXPECT_NEAR(ang_diff(recovered, psi_true), 0.0, 2.0 * kRad);
  // And it must NOT be the pre-fix wrong answer (~-73°).
  EXPECT_GT(std::abs(ang_diff(recovered, -73.0 * kRad)), 60.0 * kRad);
}

TEST(CogYawMath, ReverseCwRecoversHeading)
{
  // Reverse + CW rotation at a non-zero true heading.
  EXPECT_NEAR(ang_diff(recover_yaw(-0.20, -0.40, -2.5), -2.5), 0.0, 2.0 * kRad);
}

// The recovered reverse yaw must agree with the gyro direction (sign of the
// rotation), i.e. it must not run off the opposite way as observed on-robot.
TEST(CogYawMath, ReverseYawAgreesWithGyroSign)
{
  // True heading swept from psi_anchor (=psi_end - omega*dt) up to psi_end.
  // For CCW (omega>0) the recovered current heading must exceed the anchor.
  const double dt = 0.5;
  const double omega = 0.493;
  const double psi_end = 0.4;
  const double psi_anchor = psi_end - omega * dt;
  const double recovered = recover_yaw(-0.20, omega, psi_end, 0.30, 0.0, dt);
  EXPECT_GT(ang_diff(recovered, psi_anchor), 0.0);  // moved CCW from anchor
  EXPECT_NEAR(ang_diff(recovered, psi_end), 0.0, 2.0 * kRad);
}

// ---------------------------------------------------------------------------
// Broad sweep — forward & reverse, both rotation signs, lateral lever arm,
// arbitrary true heading. Catches any wrap / quadrant regression.
// ---------------------------------------------------------------------------

TEST(CogYawMath, SweepRecoversHeadingWithinDegrees)
{
  const double vxs[] = {0.15, 0.20, 0.30, -0.15, -0.20, -0.30};
  const double omegas[] = {-0.49, -0.30, -0.10, -0.02, 0.0, 0.02, 0.10, 0.30, 0.49};
  const double rys[] = {0.0, 0.05, -0.05};
  const double psis[] = {0.0, 0.7, -1.2, 2.5};
  for (double vx : vxs)
    for (double omega : omegas)
      for (double ry : rys)
        for (double psi : psis)
        {
          const double recovered = recover_yaw(vx, omega, psi, 0.30, ry);
          EXPECT_NEAR(ang_diff(recovered, psi), 0.0, 2.0 * kRad)
              << "vx=" << vx << " omega=" << omega << " ry=" << ry << " psi=" << psi;
        }
}

// ---------------------------------------------------------------------------
// Lever-arm-sweep dominance gate. The failure that drove the wrong-direction
// docking (2026-05-27): slow in-place alignment pivots (0.1-0.3 rad/s) sweep
// the antenna faster than the chassis translates, so the COG must be rejected.
// ---------------------------------------------------------------------------

TEST(CogSweepGate, RejectsSlowInPlacePivot)
{
  // Dock alignment: 0.3 m lever, 0.3 rad/s pivot, ~no real translation.
  // sweep = 0.3*0.3 = 0.09 m/s >> ~0 → must reject.
  EXPECT_TRUE(mowgli_localization::cog_sweep_dominates(0.3, 0.30, 0.02, 1.0));
  // Even the phantom-vx case (0.025 m/s) is dominated by the 0.09 sweep.
  EXPECT_TRUE(mowgli_localization::cog_sweep_dominates(0.3, 0.30, 0.025, 1.0));
}

TEST(CogSweepGate, AcceptsGentleCurve)
{
  // Driving forward 0.3 m/s while turning 0.2 rad/s: sweep = 0.06 < 0.3 →
  // COG is a valid heading, accept.
  EXPECT_FALSE(mowgli_localization::cog_sweep_dominates(0.2, 0.30, 0.30, 1.0));
}

TEST(CogSweepGate, AcceptsStraightDriving)
{
  // No rotation → sweep 0, always accept regardless of speed.
  EXPECT_FALSE(mowgli_localization::cog_sweep_dominates(0.0, 0.30, 0.20, 1.0));
}

TEST(CogSweepGate, BoundaryAtRatio)
{
  // sweep == ratio*vx is NOT dominant (strict >). 0.1*0.3=0.03 vs 1.0*0.03.
  EXPECT_FALSE(mowgli_localization::cog_sweep_dominates(0.1, 0.30, 0.03, 1.0));
  // Nudge omega up → now dominant.
  EXPECT_TRUE(mowgli_localization::cog_sweep_dominates(0.11, 0.30, 0.03, 1.0));
}

// ── Stationary-latch staleness accumulator ──────────────────────────────
// cog_latch_rotation_increment() deadbands the per-sample rotation so a long
// stationary dwell's gyro noise never accumulates into a false latch
// invalidation, while a real in-place pivot trips the threshold quickly.
// Field 2026-06-20: a stale post-pivot latch republished a heading 180° wrong
// and teleported the fused yaw.

TEST(CogLatchStaleness, NoiseBelowDeadbandDoesNotAccumulate)
{
  // 0.03 rad/s gyro noise, under a 0.05 deadband, integrated at 100 Hz for a
  // full 10 minutes of standing still — must accumulate exactly zero.
  double acc = 0.0;
  const double deadband = 0.05;
  const double dt = 0.01;
  for (int i = 0; i < 100 * 600; ++i)
  {
    acc += mowgli_localization::cog_latch_rotation_increment(0.03, dt, deadband);
  }
  EXPECT_DOUBLE_EQ(acc, 0.0);
}

TEST(CogLatchStaleness, InPlacePivotTripsThreshold)
{
  // A 180° in-place pivot at 0.5 rad/s (well over deadband). By a quarter turn
  // the accumulator is past the 0.26 rad staleness threshold, and by the end
  // it has integrated ~the full half-turn magnitude.
  double acc = 0.0;
  const double deadband = 0.05;
  const double threshold = 0.26;
  const double omega = 0.5;
  const double dt = 0.01;
  const int n = static_cast<int>((M_PI / omega) / dt);  // samples for a half turn
  bool tripped_before_quarter_turn = false;
  for (int i = 0; i < n; ++i)
  {
    acc += mowgli_localization::cog_latch_rotation_increment(omega, dt, deadband);
    if (i == n / 4 && acc > threshold)
    {
      tripped_before_quarter_turn = true;
    }
  }
  EXPECT_TRUE(tripped_before_quarter_turn);
  EXPECT_NEAR(acc, M_PI, 0.05);
}

TEST(CogLatchStaleness, NonPositiveDtContributesNothing)
{
  // Guards the out-of-order / first-sample dt path.
  EXPECT_DOUBLE_EQ(mowgli_localization::cog_latch_rotation_increment(1.0, 0.0, 0.05), 0.0);
  EXPECT_DOUBLE_EQ(mowgli_localization::cog_latch_rotation_increment(1.0, -0.1, 0.05), 0.0);
}

TEST(CogLatchStaleness, AtDeadbandIsNotAccumulated)
{
  // Strict > : a sample exactly at the deadband is treated as noise.
  EXPECT_DOUBLE_EQ(mowgli_localization::cog_latch_rotation_increment(0.05, 0.01, 0.05), 0.0);
  // Just above accumulates.
  EXPECT_GT(mowgli_localization::cog_latch_rotation_increment(0.051, 0.01, 0.05), 0.0);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
