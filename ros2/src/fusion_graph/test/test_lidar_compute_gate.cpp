// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <limits>

#include "fusion_graph/lidar_compute_gate.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

TEST(LidarComputeGate, FreshCalibrationHasBoundedDutyCycleAndReseedsEachBurst)
{
  fg::LidarComputeGate gate;
  auto first = gate.Step(100.0, 0.1, true, true);
  EXPECT_TRUE(first.run);
  EXPECT_TRUE(first.reseed);
  auto next = gate.Step(100.25, 0.1, true, true);
  EXPECT_TRUE(next.run);
  EXPECT_FALSE(next.reseed);
  EXPECT_FALSE(gate.Step(110.0, 0.1, true, true).run);
  EXPECT_FALSE(gate.Step(159.9, 0.1, true, true).run);
  auto burst = gate.Step(160.0, 0.1, true, true);
  EXPECT_TRUE(burst.run);
  EXPECT_TRUE(burst.reseed);
  EXPECT_FALSE(gate.Step(170.0, 0.1, true, true).run);
}

TEST(LidarComputeGate, LimitsRateAcrossPhaseTransitionsAndInvalidScans)
{
  fg::LidarComputeGate gate;
  EXPECT_TRUE(gate.Step(100.0, 0.1, true, true).run);
  EXPECT_FALSE(gate.Step(100.1, 20.0, true, true).run);
  EXPECT_FALSE(gate.Step(100.25, 20.0, true, false).run);
  auto anchoring = gate.Step(100.3, 20.0, true, true);
  EXPECT_TRUE(anchoring.run);
  EXPECT_TRUE(anchoring.reseed);
  EXPECT_FALSE(gate.Step(100.4, 20.0, true, true).run);
  EXPECT_FALSE(gate.Step(100.55, 20.0, true, true).reseed);
}

TEST(LidarComputeGate, StartsFiveSecondsBeforeApplicationAndStopsWhenFixedReturns)
{
  fg::LidarComputeGate gate;
  EXPECT_FALSE(gate.Step(100.0, 2.0, true, true, false, false).run);
  EXPECT_FALSE(gate.Step(112.0, 14.0, true, true, false, false).run);
  auto warmup = gate.Step(113.0, 15.0, true, true, false, false);
  EXPECT_TRUE(warmup.run);
  EXPECT_TRUE(warmup.reseed);
  EXPECT_TRUE(gate.Step(113.25, 15.25, true, true, false, false).run);
  EXPECT_FALSE(gate.Step(114.0, 0.1, true, true, false, false).run);
  EXPECT_FALSE(gate.Step(115.0, 1.1, true, true, false, false).run);
}

TEST(LidarComputeGate, ShadowRunsBeyondCalibrationBurstButStillYieldsToRateLimit)
{
  fg::LidarComputeGate gate;
  EXPECT_TRUE(gate.Step(100.0, 0.1, true, true, true, false).run);
  EXPECT_TRUE(gate.Step(111.0, 0.1, true, true, true, false).run);
  EXPECT_FALSE(gate.Step(111.1, 0.1, true, true, true, false).run);
  EXPECT_FALSE(gate.Step(112.0, 2.0, true, true, true, false).run);
}

TEST(LidarComputeGate, RewindResetsEpochAndMissingStructureRequiresReseed)
{
  fg::LidarComputeGate gate;
  EXPECT_TRUE(gate.Step(100.0, 0.1, true, true).run);
  EXPECT_FALSE(gate.Step(100.25, 0.1, false, true).run);
  EXPECT_TRUE(gate.Step(100.3, 0.1, true, true).reseed);
  auto rewind = gate.Step(10.0, 0.1, true, true);
  EXPECT_TRUE(rewind.run);
  EXPECT_TRUE(rewind.reseed);
  EXPECT_FALSE(gate.Step(20.0, 0.1, true, true).run);
}

TEST(LidarComputeGate, RejectsInvalidTimesAndDoesNotSpendBudget)
{
  fg::LidarComputeGate gate;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(gate.Step(nan, 0.1, true, true).run);
  EXPECT_FALSE(gate.Step(1.0, nan, true, true).run);
  EXPECT_FALSE(gate.Step(-1.0, 0.1, true, true).run);
  EXPECT_FALSE(gate.Step(1.0, -1.0, true, true).run);
  EXPECT_FALSE(gate.Step(1.0, 0.1, true, false).run);
  EXPECT_TRUE(gate.Step(1.0, 0.1, true, true).run);
  EXPECT_FALSE(gate.Step(1.0, 0.1, true, true).run);
}

TEST(LidarComputeGate, RespectsEngageThresholdWhenApplicationHasNoDelay)
{
  fg::LidarComputeGateParams p;
  p.apply_age_s = 0.0;
  fg::LidarComputeGate gate(p);
  EXPECT_FALSE(gate.Step(100.0, 1.0, true, true, false, false).run);
  EXPECT_TRUE(gate.Step(100.25, 1.01, true, true, false, false).run);
}

TEST(LidarComputeGate, InvalidConfigurationFailsClosed)
{
  fg::LidarComputeGateParams p;
  p.max_rate_hz = 0.0;
  EXPECT_FALSE(fg::LidarComputeGate(p).Step(100.0, 20.0, true, true).run);
  p.max_rate_hz = 5.0;
  p.calibration_burst_s = 61.0;
  EXPECT_FALSE(fg::LidarComputeGate(p).Step(100.0, 20.0, true, true).run);
}
