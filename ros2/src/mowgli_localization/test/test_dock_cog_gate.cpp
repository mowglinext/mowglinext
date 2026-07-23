// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// test_dock_cog_gate.cpp
//
// Unit tests for the one-click dock-calibration COG-coherence gate
// (dock_cog_gate.hpp). RESOLUTION A: the dock yaw is circular_mean of the
// /imu/cog_heading BODY-heading samples with NO +pi (the topic is already the
// reverse-aware chassis heading), cross-checked against the INDEPENDENT
// reversed GPS travel bearing.

#include <cmath>
#include <vector>

#include "mowgli_localization/dock_cog_gate.hpp"
#include <gtest/gtest.h>

using mowgli_localization::circular_mean;
using mowgli_localization::circular_std;
using mowgli_localization::DockCogReason;
using mowgli_localization::evaluate_dock_cog_gate;

namespace
{
constexpr double kRad = M_PI / 180.0;

// Default thresholds (mirror the mowgli_robot.yaml template defaults).
constexpr std::size_t kMinSamples = 8;
constexpr double kMaxStd = 3.0 * kRad;  // ~3°
constexpr double kMaxBearingErr = 6.0 * kRad;  // ~6°
constexpr double kMinDisp = 0.5;  // m

double ang_diff(double a, double b)
{
  return std::atan2(std::sin(a - b), std::cos(a - b));
}

// Build a straight-reverse sample set for a robot whose BODY heading is
// `heading` with a little COG noise, travelling 2 m in reverse (so the GPS
// displacement points along heading-pi).
std::vector<double> body_headings(double heading, int n, double noise = 0.0)
{
  std::vector<double> v;
  v.reserve(n);
  for (int i = 0; i < n; ++i)
  {
    v.push_back(heading + ((i % 2 == 0) ? noise : -noise));
  }
  return v;
}

// Net GPS displacement of a 2 m straight reverse at body `heading`
// (travel bearing = heading - pi).
std::pair<double, double> reverse_gps(double heading, double dist = 2.0)
{
  const double travel = heading - M_PI;
  return {dist * std::cos(travel), dist * std::sin(travel)};
}
}  // namespace

// ── circular_mean / circular_std ────────────────────────────────────────────

TEST(CircularStats, MeanOfClusteredAngles)
{
  EXPECT_NEAR(ang_diff(circular_mean({0.1, 0.2, 0.3}), 0.2), 0.0, 1e-9);
}

TEST(CircularStats, MeanHandlesPiWrap)
{
  // Straddling +/-pi: mean must land near pi, not near 0.
  const double m = circular_mean({3.0, -3.0});
  EXPECT_NEAR(std::abs(m), M_PI, 0.15);
}

TEST(CircularStats, StdZeroForIdentical)
{
  EXPECT_NEAR(circular_std({0.5, 0.5, 0.5, 0.5}), 0.0, 1e-9);
}

TEST(CircularStats, StdGrowsWithSpread)
{
  const double tight = circular_std(body_headings(0.5, 20, 1.0 * kRad));
  const double loose = circular_std(body_headings(0.5, 20, 10.0 * kRad));
  EXPECT_LT(tight, loose);
  EXPECT_LT(tight, 3.0 * kRad);
  EXPECT_GT(loose, 3.0 * kRad);
}

// ── dock_yaw = circular_mean, NO +pi (Resolution A) ─────────────────────────

TEST(DockCogGate, DockYawIsBodyHeadingMeanNoPiOffset)
{
  const double heading = 0.5;  // rad, e.g. dock faces ~29° ENU
  const auto samples = body_headings(heading, 20, 0.5 * kRad);
  const auto [dx, dy] = reverse_gps(heading);

  const auto r =
      evaluate_dock_cog_gate(samples, dx, dy, kMinSamples, kMaxStd, kMaxBearingErr, kMinDisp);

  EXPECT_TRUE(r.coherent);
  EXPECT_EQ(r.reason, DockCogReason::OK);
  // dock_yaw must equal the body-heading mean, NOT heading+pi.
  EXPECT_NEAR(ang_diff(r.dock_yaw_rad, heading), 0.0, 1.0 * kRad);
  EXPECT_GT(std::abs(ang_diff(r.dock_yaw_rad, heading + M_PI)), 3.0);  // definitely not +pi
}

TEST(DockCogGate, CoherentAcrossHeadingsIncludingWrap)
{
  for (double heading : {0.0, 0.7, -1.2, 2.5, 3.0, -3.0})
  {
    const auto samples = body_headings(heading, 20, 0.5 * kRad);
    const auto [dx, dy] = reverse_gps(heading);
    const auto r =
        evaluate_dock_cog_gate(samples, dx, dy, kMinSamples, kMaxStd, kMaxBearingErr, kMinDisp);
    EXPECT_TRUE(r.coherent) << "heading=" << heading;
    EXPECT_NEAR(ang_diff(r.dock_yaw_rad, heading), 0.0, 1.0 * kRad) << "heading=" << heading;
  }
}

// ── failure modes → retry ───────────────────────────────────────────────────

TEST(DockCogGate, RejectsHighStd)
{
  const double heading = 0.5;
  const auto samples = body_headings(heading, 20, 10.0 * kRad);  // ~10° jitter
  const auto [dx, dy] = reverse_gps(heading);
  const auto r =
      evaluate_dock_cog_gate(samples, dx, dy, kMinSamples, kMaxStd, kMaxBearingErr, kMinDisp);
  EXPECT_FALSE(r.coherent);
  EXPECT_EQ(r.reason, DockCogReason::HIGH_STD);
}

TEST(DockCogGate, RejectsBearingMismatch)
{
  // Steady COG at 0.5, but GPS says the robot actually reversed along a
  // heading of 1.5 (e.g. RTK not truly fixed) → mismatch.
  const double cog_heading = 0.5;
  const auto samples = body_headings(cog_heading, 20, 0.3 * kRad);
  const auto [dx, dy] = reverse_gps(1.5);
  const auto r =
      evaluate_dock_cog_gate(samples, dx, dy, kMinSamples, kMaxStd, kMaxBearingErr, kMinDisp);
  EXPECT_FALSE(r.coherent);
  EXPECT_EQ(r.reason, DockCogReason::BEARING_MISMATCH);
}

TEST(DockCogGate, RejectsInsufficientDisplacement)
{
  const double heading = 0.5;
  const auto samples = body_headings(heading, 20, 0.3 * kRad);
  const auto [dx, dy] = reverse_gps(heading, 0.2);  // only 0.2 m < 0.5 m gate
  const auto r =
      evaluate_dock_cog_gate(samples, dx, dy, kMinSamples, kMaxStd, kMaxBearingErr, kMinDisp);
  EXPECT_FALSE(r.coherent);
  EXPECT_EQ(r.reason, DockCogReason::INSUFFICIENT_DISPLACEMENT);
}

TEST(DockCogGate, RejectsInsufficientSamples)
{
  const double heading = 0.5;
  const auto samples = body_headings(heading, 3, 0.3 * kRad);  // < kMinSamples
  const auto [dx, dy] = reverse_gps(heading);
  const auto r =
      evaluate_dock_cog_gate(samples, dx, dy, kMinSamples, kMaxStd, kMaxBearingErr, kMinDisp);
  EXPECT_FALSE(r.coherent);
  EXPECT_EQ(r.reason, DockCogReason::INSUFFICIENT_SAMPLES);
}

// Displacement is checked before sample count: a stationary robot with plenty
// of samples still fails on displacement (the actionable message).
TEST(DockCogGate, DisplacementCheckedBeforeSamples)
{
  const auto samples = body_headings(0.5, 50, 0.3 * kRad);
  const auto r =
      evaluate_dock_cog_gate(samples, 0.01, 0.0, kMinSamples, kMaxStd, kMaxBearingErr, kMinDisp);
  EXPECT_EQ(r.reason, DockCogReason::INSUFFICIENT_DISPLACEMENT);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
