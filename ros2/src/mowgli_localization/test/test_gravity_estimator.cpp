// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <cmath>
#include <limits>

#include "gtest/gtest.h"
#include "mowgli_localization/gravity_estimator.hpp"

using mowgli_localization::GravityEstimator;
using mowgli_localization::GravityEstimatorAction;
using mowgli_localization::GravityEstimatorConfig;
using mowgli_localization::GravityVector;
using mowgli_localization::kStandardGravityMs2;

namespace
{
double magnitude(const GravityVector& v)
{
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

GravityVector tilted_vector(double magnitude_ms2, double tilt_rad)
{
  return GravityVector{magnitude_ms2 * std::sin(tilt_rad), 0.0, magnitude_ms2 * std::cos(tilt_rad)};
}
}  // namespace

TEST(GravityEstimator, NormalGravityAndSmallTiltAreAccepted)
{
  GravityEstimator estimator;
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0),
            GravityEstimatorAction::SEEDED);
  EXPECT_EQ(estimator.update(GravityVector{-1.7, 0.0, 9.66}, 0.1),
            GravityEstimatorAction::ACCEPTED);
  EXPECT_TRUE(estimator.has_direction());
  EXPECT_NEAR(magnitude(estimator.direction()), 1.0, 1e-12);
  EXPECT_LT(estimator.direction().x, 0.0);
}

TEST(GravityEstimator, SingleTransientDoesNotReplaceBaseline)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1),
            GravityEstimatorAction::REJECTED);
  EXPECT_TRUE(estimator.has_candidate());
  EXPECT_NEAR(estimator.direction().z, 1.0, 1e-12);
}

TEST(GravityEstimator, StartupOffsetReseedsBeforeBecomingUsable)
{
  GravityEstimator estimator;
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1),
            GravityEstimatorAction::REJECTED);
  EXPECT_FALSE(estimator.has_direction());
  for (int i = 1; i < 50; ++i)
  {
    EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1 + i * 0.1),
              GravityEstimatorAction::REJECTED);
  }
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 5.1),
            GravityEstimatorAction::RESEEDED);
  EXPECT_TRUE(estimator.has_direction());
  EXPECT_NEAR(estimator.baseline_magnitude_ms2(), 13.09, 1e-12);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 5.2),
            GravityEstimatorAction::ACCEPTED);
}

TEST(GravityEstimator, StableOffsetReseedsAndContinuesAcceptingNewBaseline)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1),
            GravityEstimatorAction::REJECTED);
  for (int i = 1; i < 50; ++i)
  {
    EXPECT_EQ(estimator.update(GravityVector{0.1, -0.1, 13.09}, 0.1 + i * 0.1),
              GravityEstimatorAction::REJECTED);
  }
  EXPECT_EQ(estimator.update(GravityVector{0.1, -0.1, 13.09}, 5.1),
            GravityEstimatorAction::RESEEDED);
  EXPECT_NEAR(estimator.previous_baseline_magnitude_ms2(), kStandardGravityMs2, 1e-12);
  // The first candidate is exactly vertical; re-seed uses the full candidate
  // average rather than the final sample's direction.
  const double mean_x = 50.0 * 0.1 / 51.0;
  const double mean_y = -mean_x;
  EXPECT_NEAR(estimator.direction().x,
              mean_x / std::sqrt(mean_x * mean_x + mean_y * mean_y + 13.09 * 13.09),
              1e-12);
  EXPECT_NEAR(magnitude(estimator.direction()), 1.0, 1e-12);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 5.2),
            GravityEstimatorAction::ACCEPTED);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 5.3),
            GravityEstimatorAction::ACCEPTED);
  EXPECT_TRUE(estimator.has_direction());
  EXPECT_NEAR(magnitude(estimator.direction()), 1.0, 1e-12);
}

TEST(GravityEstimator, CandidateInconsistencyPreventsReseed)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  for (int i = 1; i <= 70; ++i)
  {
    const GravityVector sample =
        (i % 2 == 0) ? GravityVector{0.0, 0.0, 13.09} : GravityVector{1.0, 0.0, 13.09};
    EXPECT_EQ(estimator.update(sample, i * 0.1), GravityEstimatorAction::REJECTED);
  }
  EXPECT_NEAR(estimator.direction().z, 1.0, 1e-12);
}

TEST(GravityEstimator, SlowlyDriftingCandidateCannotFollowItsOwnMean)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  for (int i = 0; i <= 60; ++i)
  {
    const double x = 0.02 * i;
    EXPECT_EQ(estimator.update(GravityVector{x, 0.0, 13.09}, 0.1 * i),
              GravityEstimatorAction::REJECTED);
  }
  EXPECT_NEAR(estimator.baseline_magnitude_ms2(), kStandardGravityMs2, 1e-12);
}

TEST(GravityEstimator, ObviousMotionAndInvalidSamplesAreRejectedAndResetCandidate)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1);
  EXPECT_TRUE(estimator.has_candidate());
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 20.0}, 0.2), GravityEstimatorAction::INVALID);
  EXPECT_FALSE(estimator.has_candidate());
  EXPECT_EQ(estimator.update(GravityVector{std::numeric_limits<double>::quiet_NaN(), 0.0, 9.8},
                             0.3),
            GravityEstimatorAction::INVALID);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 0.0}, 0.4), GravityEstimatorAction::INVALID);
}

TEST(GravityEstimator, AcceptedSampleClearsRejectedRunAndRecoveryWorks)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.2),
            GravityEstimatorAction::ACCEPTED);
  EXPECT_FALSE(estimator.has_candidate());

  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.3),
            GravityEstimatorAction::REJECTED);
  GravityEstimatorAction action = GravityEstimatorAction::REJECTED;
  for (int i = 1; i <= 50; ++i)
    action = estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.3 + i * 0.1);
  EXPECT_EQ(action, GravityEstimatorAction::RESEEDED);
}

TEST(GravityEstimator, GapDoesNotCountTowardReseedDuration)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1),
            GravityEstimatorAction::REJECTED);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 6.1),
            GravityEstimatorAction::REJECTED);
  EXPECT_NEAR(estimator.baseline_magnitude_ms2(), kStandardGravityMs2, 1e-12);
}

TEST(GravityEstimator, SamplesAtMaximumGapReseedAfterContinuousWindow)
{
  GravityEstimatorConfig config;
  config.candidate_max_gap_s = 0.5;
  GravityEstimator estimator(config);
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);

  for (int i = 0; i < 10; ++i)
  {
    EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.5 * i),
              GravityEstimatorAction::REJECTED);
  }
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 5.0),
            GravityEstimatorAction::RESEEDED);
}

TEST(GravityEstimator, EqualTimestampsDoNotAdvanceReseedDuration)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  for (int i = 0; i < 100; ++i)
  {
    EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 1.0),
              GravityEstimatorAction::REJECTED);
  }
  EXPECT_NEAR(estimator.baseline_magnitude_ms2(), kStandardGravityMs2, 1e-12);
}

TEST(GravityEstimator, ClockRollbackRestartsCandidateWindow)
{
  GravityEstimatorConfig config;
  config.candidate_max_gap_s = 0.5;
  GravityEstimator estimator(config);
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);

  for (int i = 0; i <= 6; ++i)
  {
    EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.5 * i),
              GravityEstimatorAction::REJECTED);
  }
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 1.0),
            GravityEstimatorAction::REJECTED);
  for (int i = 3; i <= 10; ++i)
  {
    EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.5 * i),
              GravityEstimatorAction::REJECTED);
  }
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 5.5),
            GravityEstimatorAction::REJECTED);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 6.0),
            GravityEstimatorAction::RESEEDED);
}

TEST(GravityEstimator, GapPartwayThroughRequiresFreshFullWindow)
{
  GravityEstimatorConfig config;
  config.candidate_max_gap_s = 0.5;
  GravityEstimator estimator(config);
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);

  for (int i = 0; i <= 6; ++i)
  {
    EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.5 * i),
              GravityEstimatorAction::REJECTED);
  }
  for (int i = 8; i < 18; ++i)
  {
    EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.5 * i),
              GravityEstimatorAction::REJECTED);
  }
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 9.0),
            GravityEstimatorAction::RESEEDED);
}

TEST(GravityEstimator, NonFiniteTimestampClearsCandidate)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.1);
  EXPECT_TRUE(estimator.has_candidate());
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09},
                             std::numeric_limits<double>::infinity()),
            GravityEstimatorAction::INVALID);
  EXPECT_FALSE(estimator.has_candidate());
}

TEST(GravityEstimator, PlausibilityEnvelopeIncludesOnlyItsExactBoundaries)
{
  GravityEstimator lower_boundary;
  EXPECT_EQ(lower_boundary.update(GravityVector{0.0, 0.0, 0.5 * kStandardGravityMs2}, 0.0),
            GravityEstimatorAction::REJECTED);
  EXPECT_EQ(lower_boundary.update(GravityVector{0.0, 0.0, 0.5 * kStandardGravityMs2 - 1e-6}, 0.1),
            GravityEstimatorAction::INVALID);

  GravityEstimator upper_boundary;
  EXPECT_EQ(upper_boundary.update(GravityVector{0.0, 0.0, 1.5 * kStandardGravityMs2}, 0.0),
            GravityEstimatorAction::REJECTED);
  EXPECT_EQ(upper_boundary.update(GravityVector{0.0, 0.0, 1.5 * kStandardGravityMs2 + 1e-6}, 0.1),
            GravityEstimatorAction::INVALID);
}

TEST(GravityEstimator, SafeTiltedOffsetCanReseed)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  const GravityVector candidate = tilted_vector(13.09, 0.4);

  for (int i = 0; i < 10; ++i)
  {
    EXPECT_EQ(estimator.update(candidate, 0.5 * i), GravityEstimatorAction::REJECTED);
  }
  EXPECT_EQ(estimator.update(candidate, 5.0), GravityEstimatorAction::RESEEDED);
  EXPECT_NEAR(estimator.baseline_magnitude_ms2(), 13.09, 1e-12);
  EXPECT_NEAR(estimator.direction().x, std::sin(0.4), 1e-12);
  EXPECT_NEAR(estimator.direction().z, std::cos(0.4), 1e-12);
}

TEST(GravityEstimator, MaximumReseedTiltBoundaryIsAllowedButGreaterTiltIsRejected)
{
  GravityEstimator at_boundary;
  at_boundary.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  const GravityVector boundary =
      tilted_vector(13.09, mowgli_localization::kDefaultMaxReseedTiltRad);
  for (int i = 0; i < 10; ++i)
  {
    EXPECT_EQ(at_boundary.update(boundary, 0.5 * i), GravityEstimatorAction::REJECTED);
  }
  EXPECT_EQ(at_boundary.update(boundary, 5.0), GravityEstimatorAction::RESEEDED);

  GravityEstimator above_boundary;
  above_boundary.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  const GravityVector just_above =
      tilted_vector(13.09, mowgli_localization::kDefaultMaxReseedTiltRad + 1e-3);
  for (int i = 0; i <= 10; ++i)
  {
    EXPECT_EQ(above_boundary.update(just_above, 0.5 * i), GravityEstimatorAction::REJECTED);
  }
  EXPECT_NEAR(above_boundary.baseline_magnitude_ms2(), kStandardGravityMs2, 1e-12);
  EXPECT_NEAR(above_boundary.direction().x, 0.0, 1e-12);
  EXPECT_NEAR(above_boundary.direction().z, 1.0, 1e-12);
  EXPECT_FALSE(above_boundary.has_candidate());
}

TEST(GravityEstimator, ExcessiveTiltNeverReseedsAndEachAttemptNeedsAFullWindow)
{
  GravityEstimator estimator;
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);
  const GravityVector unsafe = tilted_vector(13.09, 1.0);

  for (int i = 0; i <= 10; ++i)
  {
    EXPECT_EQ(estimator.update(unsafe, 0.5 * i), GravityEstimatorAction::REJECTED);
  }
  EXPECT_FALSE(estimator.has_candidate());

  // The refused candidate was cleared: a new run has not inherited its time.
  for (int i = 1; i <= 10; ++i)
  {
    EXPECT_EQ(estimator.update(unsafe, 5.0 + 0.5 * i), GravityEstimatorAction::REJECTED);
  }
  EXPECT_TRUE(estimator.has_candidate());
  EXPECT_EQ(estimator.update(unsafe, 10.5), GravityEstimatorAction::REJECTED);
  EXPECT_FALSE(estimator.has_candidate());

  // Continue well beyond five seconds: every completed unsafe attempt stays rejected.
  for (int i = 1; i <= 20; ++i)
  {
    EXPECT_EQ(estimator.update(unsafe, 10.5 + 0.5 * i), GravityEstimatorAction::REJECTED);
  }
  EXPECT_NEAR(estimator.baseline_magnitude_ms2(), kStandardGravityMs2, 1e-12);
  EXPECT_NEAR(estimator.direction().x, 0.0, 1e-12);
  EXPECT_NEAR(estimator.direction().z, 1.0, 1e-12);
}

TEST(GravityEstimator, ElapsedTimeAloneCannotReseedWithPermissiveGapConfiguration)
{
  GravityEstimatorConfig config;
  config.candidate_max_gap_s = 5.0;
  config.reseed_after_s = 5.0;
  GravityEstimator estimator(config);
  estimator.update(GravityVector{0.0, 0.0, kStandardGravityMs2}, 0.0);

  // Elapsed time and the configured gap alone would allow two isolated samples.
  // Such a permissive configuration disables recovery instead.
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.0),
            GravityEstimatorAction::REJECTED);
  EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 5.0),
            GravityEstimatorAction::REJECTED);
  EXPECT_NEAR(estimator.baseline_magnitude_ms2(), kStandardGravityMs2, 1e-12);
}

TEST(GravityEstimator, InvalidReseedTimingConfigurationCannotAdoptCandidate)
{
  const double invalid_values[] = {0.0,
                                   -0.5,
                                   std::numeric_limits<double>::infinity(),
                                   std::numeric_limits<double>::quiet_NaN()};
  for (const double invalid_gap : invalid_values)
  {
    GravityEstimatorConfig config;
    config.candidate_max_gap_s = invalid_gap;
    GravityEstimator estimator(config);
    for (int i = 0; i <= 20; ++i)
    {
      EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.5 * i),
                GravityEstimatorAction::REJECTED);
    }
    EXPECT_NEAR(estimator.baseline_magnitude_ms2(), kStandardGravityMs2, 1e-12);
  }

  for (const double invalid_duration : invalid_values)
  {
    GravityEstimatorConfig config;
    config.reseed_after_s = invalid_duration;
    GravityEstimator estimator(config);
    for (int i = 0; i <= 20; ++i)
    {
      EXPECT_EQ(estimator.update(GravityVector{0.0, 0.0, 13.09}, 0.5 * i),
                GravityEstimatorAction::REJECTED);
    }
    EXPECT_NEAR(estimator.baseline_magnitude_ms2(), kStandardGravityMs2, 1e-12);
  }
}
