// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/lidar_map_anchor_gate.hpp"
#include <gtest/gtest.h>

using fusion_graph::LidarAnchorState;
using fusion_graph::LidarMapAnchorGate;

TEST(LidarMapAnchorGate, DisabledDoesNothing)
{
  LidarMapAnchorGate g(false, 0.3, 0.5);
  const auto d = g.Step(100.0, true, 1.0);
  EXPECT_EQ(d.state, LidarAnchorState::kDisabled);
  EXPECT_FALSE(d.insert_scan);
}

// Fresh RTK-Fixed: the pose is trusted, so scans build the map and no anchor
// factor is produced — the GPS already pins the pose.
TEST(LidarMapAnchorGate, FreshFixedMapsAndNeverAnchors)
{
  LidarMapAnchorGate g(true, 0.3, 0.5);
  const auto d = g.Step(0.1, true, 1.0);
  EXPECT_EQ(d.state, LidarAnchorState::kMapping);
  EXPECT_TRUE(d.insert_scan);
}

// Insertion is rate-limited: a 10 Hz LiDAR need not write every scan.
TEST(LidarMapAnchorGate, InsertionIsRateLimited)
{
  LidarMapAnchorGate g(true, 0.3, 0.5);
  EXPECT_TRUE(g.Step(0.0, true, 1.0).insert_scan);
  EXPECT_FALSE(g.Step(0.0, true, 1.2).insert_scan);
  EXPECT_FALSE(g.Step(0.0, true, 1.4).insert_scan);
  EXPECT_TRUE(g.Step(0.0, true, 1.5).insert_scan);
}

// Stale Fixed with nothing in the map yet: stay honest, no factor.
TEST(LidarMapAnchorGate, StaleWithoutMapWaits)
{
  LidarMapAnchorGate g(true, 0.3, 0.5);
  const auto d = g.Step(5.0, false, 1.0);
  EXPECT_EQ(d.state, LidarAnchorState::kWaitingForMap);
  EXPECT_FALSE(d.insert_scan);
}

// An outage freezes learning; subsequent stale scans retain that state.
TEST(LidarMapAnchorGate, OutageFreezesLearning)
{
  LidarMapAnchorGate g(true, 0.3, 0.5);
  g.Step(0.0, true, 1.0);
  const auto first = g.Step(1.0, true, 2.0);
  EXPECT_EQ(first.state, LidarAnchorState::kAnchoring);
  EXPECT_FALSE(first.insert_scan);
  const auto second = g.Step(2.0, true, 3.0);
  EXPECT_EQ(second.state, LidarAnchorState::kAnchoring);
  EXPECT_FALSE(second.insert_scan);
}

// Fresh Fixed resumes learning only after the dwell; a later outage freezes it.
TEST(LidarMapAnchorGate, ReturnToFixedResumesLearningAfterDwell)
{
  LidarMapAnchorGate g(true, 0.3, 0.5, /*disengage_dwell_s=*/1.0);
  g.Step(1.0, true, 1.0);
  EXPECT_EQ(g.state(), LidarAnchorState::kAnchoring);
  const auto still = g.Step(0.0, true, 1.5);  // fresh, but only for 0 s so far
  EXPECT_EQ(still.state, LidarAnchorState::kAnchoring);
  const auto back = g.Step(0.0, true, 2.6);  // fresh for 1.1 s ≥ dwell
  EXPECT_EQ(back.state, LidarAnchorState::kMapping);
  EXPECT_FALSE(still.insert_scan);
  EXPECT_TRUE(back.insert_scan);
  const auto again = g.Step(1.0, true, 3.0);
  EXPECT_EQ(again.state, LidarAnchorState::kAnchoring);
  EXPECT_FALSE(again.insert_scan);
}

// Intermittent fresh samples cannot restart learning during an outage.
TEST(LidarMapAnchorGate, IntermittentFreshSamplesDoNotResumeMapping)
{
  LidarMapAnchorGate g(true, 1.0, 0.5, 1.0);
  g.Step(1.05, true, 0.0);
  for (int i = 1; i < 100; ++i)
  {
    const double age = (i % 7 == 6) ? 1.05 : 0.28;
    const auto d = g.Step(age, true, 0.04 * i);
    EXPECT_EQ(d.state, LidarAnchorState::kAnchoring);
    EXPECT_FALSE(d.insert_scan);
  }
}

// A single stale sample while the dwell is counting down resets it: Fixed
// must be fresh CONTINUOUSLY to disengage.
TEST(LidarMapAnchorGate, StaleSampleResetsTheDisengageDwell)
{
  LidarMapAnchorGate g(true, 0.3, 0.5, 1.0);
  g.Step(1.0, true, 0.0);
  g.Step(0.0, true, 0.5);  // fresh since 0.5
  g.Step(1.0, true, 1.0);  // stale again: dwell resets
  const auto d = g.Step(0.0, true, 1.4);  // fresh again since 1.4, not since 0.5
  EXPECT_EQ(d.state, LidarAnchorState::kAnchoring);
  EXPECT_EQ(g.Step(0.0, true, 2.5).state, LidarAnchorState::kMapping);
}
