// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for the dead-IMU detector. Pure logic, no ROS — mirrors
// test_dig_detector.cpp. The scenario under test is the 2026-09-02 field
// failure: a hung WT901 bus streaming all-zero accel + gyro at 90 Hz.

#include <limits>

#include "mowgli_hardware/imu_liveness.hpp"
#include <gtest/gtest.h>

namespace mh = mowgli_hardware;

namespace
{
constexpr double kG = 9.81;

// Feed `n` samples of the given deadness through the tracker, returning the
// final update (a fresh state each step — the tracker never mutates input).
mh::ImuLivenessUpdate Feed(mh::ImuLivenessState st, bool sample_dead, int n)
{
  mh::ImuLivenessUpdate last{st, false, false};
  for (int i = 0; i < n; ++i)
  {
    last = mh::UpdateImuLiveness(last.state, sample_dead);
  }
  return last;
}
}  // namespace

// ── IsImuSampleDead ──────────────────────────────────────────────────────────

TEST(IsImuSampleDead, AllZeroSampleIsDead)
{
  EXPECT_TRUE(mh::IsImuSampleDead(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
}

TEST(IsImuSampleDead, LevelGravityIsAlive)
{
  EXPECT_FALSE(mh::IsImuSampleDead(0.0, 0.0, kG, 0.0, 0.0, 0.0));
}

TEST(IsImuSampleDead, TiltedGravityIsAlive)
{
  // 45° mounting tilt: magnitude is still g.
  const double c = kG / std::sqrt(2.0);
  EXPECT_FALSE(mh::IsImuSampleDead(c, 0.0, c, 0.0, 0.0, 0.0));
}

TEST(IsImuSampleDead, ZeroGyroAloneIsNotDead)
{
  // A quiet chip at rest reports gyro ~ 0 — that is normal, not a hang.
  EXPECT_FALSE(mh::IsImuSampleDead(0.05, -0.02, kG, 0.0, 0.0, 0.0));
}

TEST(IsImuSampleDead, ZeroAccelWithLiveGyroIsStillDead)
{
  // Accel alone decides: gravity is never zero.
  EXPECT_TRUE(mh::IsImuSampleDead(0.0, 0.0, 0.0, 0.1, -0.2, 0.3));
}

TEST(IsImuSampleDead, ThresholdBoundary)
{
  EXPECT_TRUE(mh::IsImuSampleDead(0.0, 0.0, mh::kMinPlausibleAccelMps2 - 1e-9, 0.0, 0.0, 0.0));
  EXPECT_FALSE(mh::IsImuSampleDead(0.0, 0.0, mh::kMinPlausibleAccelMps2, 0.0, 0.0, 0.0));
}

TEST(IsImuSampleDead, NanAccelIsDead)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(mh::IsImuSampleDead(nan, 0.0, kG, 0.0, 0.0, 0.0));
}

// ── UpdateImuLiveness ────────────────────────────────────────────────────────

TEST(UpdateImuLiveness, StartsAlive)
{
  const mh::ImuLivenessState st{};
  EXPECT_FALSE(st.dead);
  EXPECT_EQ(st.consecutive_dead, 0);
}

TEST(UpdateImuLiveness, StaysAliveBelowThreshold)
{
  const auto u = Feed(mh::ImuLivenessState{}, true, mh::kImuDeadSampleThreshold - 1);
  EXPECT_FALSE(u.state.dead);
  EXPECT_FALSE(u.became_dead);
  EXPECT_EQ(u.state.consecutive_dead, mh::kImuDeadSampleThreshold - 1);
}

TEST(UpdateImuLiveness, FlipsDeadExactlyAtThreshold)
{
  const auto before = Feed(mh::ImuLivenessState{}, true, mh::kImuDeadSampleThreshold - 1);
  const auto at = mh::UpdateImuLiveness(before.state, true);
  EXPECT_TRUE(at.state.dead);
  EXPECT_TRUE(at.became_dead);
  EXPECT_FALSE(at.became_alive);
}

TEST(UpdateImuLiveness, TransitionFlagFiresOnlyOnce)
{
  const auto dead = Feed(mh::ImuLivenessState{}, true, mh::kImuDeadSampleThreshold);
  ASSERT_TRUE(dead.became_dead);
  const auto later = mh::UpdateImuLiveness(dead.state, true);
  EXPECT_TRUE(later.state.dead);
  EXPECT_FALSE(later.became_dead);
  EXPECT_FALSE(later.became_alive);
}

TEST(UpdateImuLiveness, RunCounterIsCappedWhileDead)
{
  // A bus hung for hours must not overflow the counter.
  const auto u = Feed(mh::ImuLivenessState{}, true, mh::kImuDeadSampleThreshold * 10);
  EXPECT_TRUE(u.state.dead);
  EXPECT_EQ(u.state.consecutive_dead, mh::kImuDeadSampleThreshold);
}

TEST(UpdateImuLiveness, FirstLiveSampleRevives)
{
  const auto dead = Feed(mh::ImuLivenessState{}, true, mh::kImuDeadSampleThreshold * 2);
  ASSERT_TRUE(dead.state.dead);
  const auto alive = mh::UpdateImuLiveness(dead.state, false);
  EXPECT_FALSE(alive.state.dead);
  EXPECT_TRUE(alive.became_alive);
  EXPECT_FALSE(alive.became_dead);
  EXPECT_EQ(alive.state.consecutive_dead, 0);
}

TEST(UpdateImuLiveness, ReviveFlagDoesNotFireWhenAlreadyAlive)
{
  const auto u = mh::UpdateImuLiveness(mh::ImuLivenessState{}, false);
  EXPECT_FALSE(u.became_alive);
  EXPECT_FALSE(u.became_dead);
}

TEST(UpdateImuLiveness, SingleLiveSampleResetsTheRun)
{
  // 44 dead, 1 live, 44 dead: never reaches the 45-sample threshold.
  auto u = Feed(mh::ImuLivenessState{}, true, mh::kImuDeadSampleThreshold - 1);
  u = mh::UpdateImuLiveness(u.state, false);
  u = Feed(u.state, true, mh::kImuDeadSampleThreshold - 1);
  EXPECT_FALSE(u.state.dead);
}

TEST(UpdateImuLiveness, DoesNotMutateInput)
{
  const mh::ImuLivenessState prev{3, false};
  (void)mh::UpdateImuLiveness(prev, true);
  EXPECT_EQ(prev.consecutive_dead, 3);
  EXPECT_FALSE(prev.dead);
}

// ── IsCalibrationPlausible ───────────────────────────────────────────────────

TEST(IsCalibrationPlausible, HealthyDockedWindowIsPlausible)
{
  // Real WT901 at rest: |a| ~ g, gyro variance ~1e-5 rad²/s².
  EXPECT_TRUE(mh::IsCalibrationPlausible(9.79, 1.2e-5, 0.9e-5, 1.1e-5));
}

TEST(IsCalibrationPlausible, AllZeroWindowIsRejected)
{
  // The 2026-09-02 case: 200 samples of exact zeros.
  EXPECT_FALSE(mh::IsCalibrationPlausible(0.0, 0.0, 0.0, 0.0));
}

TEST(IsCalibrationPlausible, LowMeanAccelIsRejectedEvenWithGyroNoise)
{
  EXPECT_FALSE(mh::IsCalibrationPlausible(2.99, 1e-5, 1e-5, 1e-5));
}

TEST(IsCalibrationPlausible, SilentGyroIsRejectedEvenWithGravity)
{
  EXPECT_FALSE(mh::IsCalibrationPlausible(9.81, 0.0, 0.0, 0.0));
}

TEST(IsCalibrationPlausible, OneNoisyGyroAxisIsEnough)
{
  EXPECT_TRUE(mh::IsCalibrationPlausible(9.81, 0.0, 0.0, 1e-7));
}

TEST(IsCalibrationPlausible, AccelBoundary)
{
  EXPECT_TRUE(mh::IsCalibrationPlausible(mh::kMinPlausibleAccelMps2, 1e-5, 1e-5, 1e-5));
}

TEST(IsCalibrationPlausible, NanMeanAccelIsRejected)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mh::IsCalibrationPlausible(nan, 1e-5, 1e-5, 1e-5));
}

// ── IsDeadSensorCovariance ───────────────────────────────────────────────────

TEST(IsDeadSensorCovariance, AllFiveZeroIsDead)
{
  EXPECT_TRUE(mh::IsDeadSensorCovariance(0.0, 0.0, 0.0, 0.0, 0.0));
}

TEST(IsDeadSensorCovariance, AnyNonZeroIsAlive)
{
  EXPECT_FALSE(mh::IsDeadSensorCovariance(1e-4, 0.0, 0.0, 0.0, 0.0));
  EXPECT_FALSE(mh::IsDeadSensorCovariance(0.0, 1e-4, 0.0, 0.0, 0.0));
  EXPECT_FALSE(mh::IsDeadSensorCovariance(0.0, 0.0, 1e-6, 0.0, 0.0));
  EXPECT_FALSE(mh::IsDeadSensorCovariance(0.0, 0.0, 0.0, 1e-6, 0.0));
  EXPECT_FALSE(mh::IsDeadSensorCovariance(0.0, 0.0, 0.0, 0.0, 1e-6));
}

TEST(IsDeadSensorCovariance, HealthyPersistedFileIsAlive)
{
  EXPECT_FALSE(mh::IsDeadSensorCovariance(2.1e-4, 1.9e-4, 1.2e-5, 0.9e-5, 1.1e-5));
}
