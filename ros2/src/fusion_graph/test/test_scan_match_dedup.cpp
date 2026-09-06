// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include "fusion_graph/scan_match_dedup.hpp"
#include <gtest/gtest.h>

using fusion_graph::ScanMatchDedupGate;

TEST(ScanMatchDedup, FirstCallAlwaysMatches)
{
  ScanMatchDedupGate g;
  EXPECT_TRUE(g.ShouldMatch(0, 0));
  EXPECT_TRUE(g.ShouldMatch(7, 3));
}

// The measured case: 25 Hz timer, ~10 Hz scans. Between two scans the pair
// (scan, prev-node) is unchanged and the previous match succeeded — do not
// realign the same two point clouds again.
TEST(ScanMatchDedup, SkipsRepeatOfASuccessfulMatchOnUnchangedInputs)
{
  ScanMatchDedupGate g;
  ASSERT_TRUE(g.ShouldMatch(1, 0));
  g.RecordOutcome(1, 0, true);
  EXPECT_FALSE(g.ShouldMatch(1, 0));
  EXPECT_FALSE(g.ShouldMatch(1, 0));
}

// Conservative: a failed match may be retried on the next tick, because the
// odometry-driven init guess drifts and a guard rail that tripped once may
// clear.
TEST(ScanMatchDedup, RetriesAfterAFailedMatch)
{
  ScanMatchDedupGate g;
  ASSERT_TRUE(g.ShouldMatch(1, 0));
  g.RecordOutcome(1, 0, false);
  EXPECT_TRUE(g.ShouldMatch(1, 0));
}

TEST(ScanMatchDedup, MatchesAgainWhenANewScanArrives)
{
  ScanMatchDedupGate g;
  g.RecordOutcome(1, 0, true);
  EXPECT_FALSE(g.ShouldMatch(1, 0));
  EXPECT_TRUE(g.ShouldMatch(2, 0));
}

// A new graph node stores a new prev_node_scan_: the same scan must now be
// aligned against a DIFFERENT target, so it is a new measurement.
TEST(ScanMatchDedup, MatchesAgainWhenThePrevNodeScanChanges)
{
  ScanMatchDedupGate g;
  g.RecordOutcome(5, 2, true);
  EXPECT_FALSE(g.ShouldMatch(5, 2));
  EXPECT_TRUE(g.ShouldMatch(5, 3));
}

// Regression shape for the field measurement: 25 ticks, 10 distinct scans,
// no new node, every match succeeds → exactly 10 ICPs, not 25.
TEST(ScanMatchDedup, OneIcpPerDistinctScanWhenStationary)
{
  ScanMatchDedupGate g;
  int icps = 0;
  for (int tick = 0; tick < 25; ++tick)
  {
    const uint64_t scan_seq =
        static_cast<uint64_t>(tick * 10 / 25);  // 10 Hz scans under a 25 Hz timer
    if (g.ShouldMatch(scan_seq, 0))
    {
      ++icps;
      g.RecordOutcome(scan_seq, 0, true);
    }
  }
  EXPECT_EQ(icps, 10);
}
