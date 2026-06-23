// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/gnss_mobile_gate.hpp"
#include <gtest/gtest.h>

namespace fusion_graph
{

TEST(GnssMobileGate, AcceptsNominalForwardMotion)
{
  const auto metrics = EvaluateGnssMobileGate(
      0.20, 0.20, 0.040, 0.040, 0.045, 0.002, 0.014, 0.0, GnssMobileGateParams{2.0, 0.01, 2.0});

  EXPECT_DOUBLE_EQ(metrics.expected_motion_m, 0.040);
  EXPECT_NEAR(metrics.delta_gps_m, 0.045044, 1e-6);
  EXPECT_EQ(metrics.decision, GnssMobileGateDecision::kAccepted);
  EXPECT_LT(metrics.innovation_m, metrics.allowed_delta_m);
  EXPECT_LT(metrics.lateral_innovation_m, 0.01);
}

TEST(GnssMobileGate, FallsBackToCommandedMotionWhenWheelIsAbsent)
{
  const auto metrics = EvaluateGnssMobileGate(
      0.50, 0.30, 0.15, 0.0, 0.152, 0.0, 0.010, 0.0, GnssMobileGateParams{2.0, 0.01, 2.0});

  EXPECT_DOUBLE_EQ(metrics.cmd_delta_m, 0.15);
  EXPECT_DOUBLE_EQ(metrics.expected_motion_m, 0.15);
  EXPECT_EQ(metrics.decision, GnssMobileGateDecision::kAccepted);
}

TEST(GnssMobileGate, DownweightsModerateForwardOutlier)
{
  const auto metrics = EvaluateGnssMobileGate(
      0.20, 0.20, 0.040, 0.040, 0.100, 0.0, 0.014, 0.0, GnssMobileGateParams{2.0, 0.01, 2.0});

  EXPECT_EQ(metrics.decision, GnssMobileGateDecision::kDownweighted);
  EXPECT_GT(metrics.innovation_m, 0.03);
  EXPECT_LT(metrics.lateral_innovation_m, 1e-9);
}

TEST(GnssMobileGate, RejectsLargeLateralOutlier)
{
  const auto metrics = EvaluateGnssMobileGate(
      0.20, 0.20, 0.040, 0.040, 0.040, 0.100, 0.014, 0.0, GnssMobileGateParams{2.0, 0.01, 2.0});

  EXPECT_EQ(metrics.decision, GnssMobileGateDecision::kRejected);
  EXPECT_GT(metrics.lateral_innovation_m, metrics.allowed_delta_m);
}

TEST(GnssMobileGate, UsesAcceptedWindowMotionNotRawCadenceForCommandFallback)
{
  const auto metrics = EvaluateGnssMobileGate(
      0.20, 0.30, 0.60, 0.0, 0.610, 0.0, 0.010, 0.0, GnssMobileGateParams{2.0, 0.01, 2.0});

  EXPECT_DOUBLE_EQ(metrics.cmd_delta_m, 0.60);
  EXPECT_DOUBLE_EQ(metrics.expected_motion_m, 0.60);
  EXPECT_EQ(metrics.decision, GnssMobileGateDecision::kAccepted);
}

}  // namespace fusion_graph
