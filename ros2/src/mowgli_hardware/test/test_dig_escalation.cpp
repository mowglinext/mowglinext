// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for repeat-dig escalation. Pure logic, no ROS — mirrors
// test_dig_detector.cpp.
//
// The headline cases replay the REAL latch positions and timestamps from the
// instrumented capture in issue #500 (2026-09-04), so the shipped defaults are
// pinned against the field data they were chosen from rather than against
// invented numbers.

#include <cstddef>
#include <vector>

#include "mowgli_hardware/dig_escalation.hpp"
#include <gtest/gtest.h>

namespace mh = mowgli_hardware;

namespace
{
// The 2026-09-04 capture, in order. The first three latches are isolated and
// genuine (metres apart, minutes apart); the last five are one obstruction —
// five latches in 23 s inside a 6 cm square around (-5.39, 11.85).
const std::vector<mh::DigLatch> kFieldLatches = {
    {-9.03, 12.82, 10957.8},  // isolated, genuine
    {-4.74, 6.72, 11171.8},  // isolated, genuine
    {-4.39, 12.59, 11496.6},  // isolated, genuine (1.2 m from the cluster)
    {-5.39, 11.86, 11517.1},  // cluster 1/5
    {-5.38, 11.84, 11522.8},  // cluster 2/5
    {-5.40, 11.87, 11528.6},  // cluster 3/5 <- must escalate here
    {-5.39, 11.84, 11534.4},  // cluster 4/5
    {-5.37, 11.80, 11540.0},  // cluster 5/5
};

// Replay a latch sequence through the shipped defaults, returning the index of
// the first latch that escalates (or -1 if none does).
int FirstEscalatingIndex(const std::vector<mh::DigLatch>& latches,
                         const mh::DigEscalationCfg& cfg = {})
{
  mh::DigLatchHistory history;
  for (std::size_t i = 0; i < latches.size(); ++i)
  {
    const mh::DigLatch& latch = latches[i];
    const bool escalate = mh::ShouldEscalate(cfg, history, latch.x, latch.y, latch.t);
    history = mh::DigRecordLatch(history, latch.x, latch.y, latch.t, cfg.window_s);
    if (escalate)
    {
      return static_cast<int>(i);
    }
  }
  return -1;
}
}  // namespace

// ── Field data ──────────────────────────────────────────────────────────────

TEST(DigEscalation, FieldCaptureEscalatesOnThirdClusterLatch)
{
  // Index 5 is t=11528.6, the third latch of the (-5.39, 11.85) cluster —
  // ~11 s into an episode that otherwise ran for 5.5 more minutes.
  EXPECT_EQ(FirstEscalatingIndex(kFieldLatches), 5);
}

TEST(DigEscalation, IsolatedFieldLatchesNeverEscalate)
{
  // The three genuine, well-separated digs from the same run, on their own.
  const std::vector<mh::DigLatch> isolated(kFieldLatches.begin(), kFieldLatches.begin() + 3);

  EXPECT_EQ(FirstEscalatingIndex(isolated), -1);
}

TEST(DigEscalation, ClusterSpreadBeyondTheWindowDoesNotEscalate)
{
  // Same three positions as the cluster, but one every 90 s: the robot left
  // and came back much later, which is not the runaway loop.
  const std::vector<mh::DigLatch> spread = {
      {-5.39, 11.86, 100.0},
      {-5.38, 11.84, 190.0},
      {-5.40, 11.87, 280.0},
  };

  EXPECT_EQ(FirstEscalatingIndex(spread), -1);
}

// ── Core behaviour ──────────────────────────────────────────────────────────

TEST(DigEscalation, ThirdLatchInsideRadiusAndWindowEscalates)
{
  mh::DigLatchHistory history;
  history = mh::DigRecordLatch(history, 0.0, 0.0, 0.0, 60.0);
  history = mh::DigRecordLatch(history, 0.10, 0.0, 5.0, 60.0);

  EXPECT_TRUE(mh::ShouldEscalate(history, 0.20, 0.0, 10.0, 0.50, 60.0, 3));
}

TEST(DigEscalation, SecondLatchIsNotEnough)
{
  mh::DigLatchHistory history;
  history = mh::DigRecordLatch(history, 0.0, 0.0, 0.0, 60.0);

  EXPECT_FALSE(mh::ShouldEscalate(history, 0.10, 0.0, 5.0, 0.50, 60.0, 3));
}

TEST(DigEscalation, LatchesJustOutsideTheRadiusDoNotCount)
{
  mh::DigLatchHistory history;
  history = mh::DigRecordLatch(history, 0.0, 0.0, 0.0, 60.0);
  history = mh::DigRecordLatch(history, 0.60, 0.0, 5.0, 60.0);

  // Both prior latches are >0.50 m from (1.20, 0), so the count is 1.
  EXPECT_FALSE(mh::ShouldEscalate(history, 1.20, 0.0, 10.0, 0.50, 60.0, 3));
}

TEST(DigEscalation, CountsAreInclusiveOfTheCurrentLatch)
{
  mh::DigLatchHistory history;
  history = mh::DigRecordLatch(history, 0.0, 0.0, 0.0, 60.0);

  EXPECT_EQ(mh::DigNearbyLatchCount(history, 0.10, 0.0, 5.0, 0.50, 60.0), 2);
}

TEST(DigEscalation, LatchesOlderThanTheWindowDoNotCount)
{
  mh::DigLatchHistory history;
  // Recorded with a long retention so pruning does not remove them; the
  // window argument to ShouldEscalate is what must exclude them.
  history = mh::DigRecordLatch(history, 0.0, 0.0, 0.0, 1.0e6);
  history = mh::DigRecordLatch(history, 0.05, 0.0, 1.0, 1.0e6);

  EXPECT_FALSE(mh::ShouldEscalate(history, 0.05, 0.0, 500.0, 0.50, 60.0, 3));
}

// ── Pruning / bounding ──────────────────────────────────────────────────────

TEST(DigEscalation, PruningDropsEntriesOlderThanTheWindow)
{
  mh::DigLatchHistory history;
  history = mh::DigRecordLatch(history, 0.0, 0.0, 0.0, 60.0);
  history = mh::DigRecordLatch(history, 0.0, 0.0, 30.0, 60.0);
  ASSERT_EQ(history.size(), 2u);

  // At t=80 the t=0 entry is 80 s old and drops; t=30 (50 s old) survives.
  history = mh::DigRecordLatch(history, 0.0, 0.0, 80.0, 60.0);
  ASSERT_EQ(history.size(), 2u);
  EXPECT_DOUBLE_EQ(history.front().t, 30.0);  // the t=0 entry was pruned
  EXPECT_DOUBLE_EQ(history.back().t, 80.0);

  // Push past the window entirely: everything older than 60 s is gone.
  history = mh::DigRecordLatch(history, 0.0, 0.0, 200.0, 60.0);
  ASSERT_EQ(history.size(), 1u);
  EXPECT_DOUBLE_EQ(history.front().t, 200.0);
}

TEST(DigEscalation, HistoryStaysBoundedUnderRelentlessLatching)
{
  mh::DigLatchHistory history;
  for (int i = 0; i < 500; ++i)
  {
    // Same spot, same instant: age-pruning can never drop any of these, so
    // only the hard cap bounds the structure.
    history = mh::DigRecordLatch(history, 0.0, 0.0, 0.0, 60.0);
  }

  EXPECT_EQ(history.size(), mh::kDigLatchHistoryMax);
}

// ── Disable sentinels ───────────────────────────────────────────────────────

TEST(DigEscalation, MinCountOfOneOrLessDisables)
{
  mh::DigLatchHistory history;
  history = mh::DigRecordLatch(history, 0.0, 0.0, 0.0, 60.0);
  history = mh::DigRecordLatch(history, 0.0, 0.0, 5.0, 60.0);

  EXPECT_FALSE(mh::ShouldEscalate(history, 0.0, 0.0, 10.0, 0.50, 60.0, 1));
  EXPECT_FALSE(mh::ShouldEscalate(history, 0.0, 0.0, 10.0, 0.50, 60.0, 0));
  EXPECT_FALSE(mh::ShouldEscalate(history, 0.0, 0.0, 10.0, 0.50, 60.0, -1));
}

TEST(DigEscalation, NonPositiveRadiusDisables)
{
  mh::DigLatchHistory history;
  history = mh::DigRecordLatch(history, 0.0, 0.0, 0.0, 60.0);
  history = mh::DigRecordLatch(history, 0.0, 0.0, 5.0, 60.0);

  EXPECT_FALSE(mh::ShouldEscalate(history, 0.0, 0.0, 10.0, 0.0, 60.0, 3));
  EXPECT_FALSE(mh::ShouldEscalate(history, 0.0, 0.0, 10.0, -1.0, 60.0, 3));
}

TEST(DigEscalation, NonPositiveWindowDisables)
{
  mh::DigLatchHistory history;
  history = mh::DigRecordLatch(history, 0.0, 0.0, 0.0, 60.0);
  history = mh::DigRecordLatch(history, 0.0, 0.0, 5.0, 60.0);

  EXPECT_FALSE(mh::ShouldEscalate(history, 0.0, 0.0, 10.0, 0.50, 0.0, 3));
}

TEST(DigEscalation, EnabledReportsTheDisableSentinels)
{
  EXPECT_TRUE(mh::DigEscalationEnabled(mh::DigEscalationCfg{}));

  mh::DigEscalationCfg cfg;
  cfg.min_count = 1;
  EXPECT_FALSE(mh::DigEscalationEnabled(cfg));

  cfg = mh::DigEscalationCfg{};
  cfg.radius_m = 0.0;
  EXPECT_FALSE(mh::DigEscalationEnabled(cfg));

  cfg = mh::DigEscalationCfg{};
  cfg.window_s = 0.0;
  EXPECT_FALSE(mh::DigEscalationEnabled(cfg));
}

TEST(DigEscalation, ShippedDefaultsMatchTheDocumentedValues)
{
  const mh::DigEscalationCfg cfg;

  EXPECT_DOUBLE_EQ(cfg.radius_m, 0.50);
  EXPECT_DOUBLE_EQ(cfg.window_s, 60.0);
  EXPECT_EQ(cfg.min_count, 3);
}
