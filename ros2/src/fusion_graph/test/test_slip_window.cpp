// Windowed wheel-vs-gyro slip veto (issue #516).
//
// The per-node veto could not tell one encoder tick of L/R asymmetry
// (≈ 0.011 rad of wheel dθ in a 40 ms frame) from genuine slip (≈ 0.012 rad
// per frame, sustained for seconds) — the difference is the PERSISTENCE, not
// the per-frame magnitude. These tests pin the window behaviour: jitter never
// trips it, sustained slip does, a coordinated turn does not, a cold window
// never vetoes, and the flag clears within one window once the slip stops.

#include <cstddef>

#include "fusion_graph/slip_window.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{
// Shipped per-node thresholds (fusion_graph.yaml, tuned at 25 Hz).
constexpr double kResidual = 0.01;
constexpr double kGyroMax = 0.005;
constexpr double kWheelMin = 0.005;
// One encoder tick of L/R asymmetry in one 40 ms frame (2026-08-24 note:
// 1 / 280.44 / 0.325 = 0.01097 rad).
constexpr double kTickRad = 0.01097;
// Genuine slip per frame: ~0.3 rad/s wheel yaw rate x 0.04 s.
constexpr double kSlipRad = 0.012;
constexpr std::size_t kWindow = 12;  // 0.5 s @ 25 Hz

bool Detected(const fg::SlipWindow& w)
{
  return w.Detected(kResidual, kGyroMax, kWheelMin);
}
}  // namespace

TEST(SlipWindow, WindowNodesRoundsAndFloorsAtOne)
{
  EXPECT_EQ(fg::SlipWindowNodes(0.5, 0.04), 12u);
  EXPECT_EQ(fg::SlipWindowNodes(0.5, 0.02), 25u);
  EXPECT_EQ(fg::SlipWindowNodes(0.5, 0.1), 5u);
  // Disabled / degenerate → a window of one node (the old per-node gate).
  EXPECT_EQ(fg::SlipWindowNodes(0.0, 0.04), 1u);
  EXPECT_EQ(fg::SlipWindowNodes(0.01, 0.04), 1u);
  EXPECT_EQ(fg::SlipWindowNodes(0.5, 0.0), 1u);
  EXPECT_EQ(fg::SlipWindowNodes(-1.0, 0.04), 1u);
}

TEST(SlipWindow, PureRuleMatchesPerNodeGateAtOneNode)
{
  // The 2026-05-27 signature on a single node: wheel 0.03, gyro 0.002.
  EXPECT_TRUE(fg::SlipDetectedOverWindow(0.03, 0.002, 1, kResidual, kGyroMax, kWheelMin));
  // Coordinated turn: wheel and gyro agree.
  EXPECT_FALSE(fg::SlipDetectedOverWindow(0.03, 0.03, 1, kResidual, kGyroMax, kWheelMin));
  // Empty window never vetoes.
  EXPECT_FALSE(fg::SlipDetectedOverWindow(1.0, 0.0, 0, kResidual, kGyroMax, kWheelMin));
}

TEST(SlipWindow, AlternatingTickJitterIsNeverDetected)
{
  fg::SlipWindow w(kWindow);
  for (int i = 0; i < 200; ++i)
  {
    w.Push((i % 2 == 0) ? kTickRad : -kTickRad, 0.0);
    EXPECT_FALSE(Detected(w)) << "frame " << i;
  }
}

TEST(SlipWindow, TwoTickBurstsOnStraightsAreNotDetected)
{
  // The #516 failure: an occasional 2-tick frame on a straight swath trips
  // the per-node gate (0.022 > 0.01). Sparse bursts must not trip the window.
  fg::SlipWindow w(kWindow);
  for (int i = 0; i < 200; ++i)
  {
    const double wheel = (i % 10 == 0) ? 2.0 * kTickRad : ((i % 2 == 0) ? kTickRad : -kTickRad);
    w.Push(wheel, 0.0);
    EXPECT_FALSE(Detected(w)) << "frame " << i;
  }
}

TEST(SlipWindow, SustainedSlipIsDetectedOnceWindowIsFull)
{
  fg::SlipWindow w(kWindow);
  for (std::size_t i = 0; i < kWindow - 1; ++i)
  {
    w.Push(kSlipRad, 0.0);
    EXPECT_FALSE(Detected(w)) << "warm-up frame " << i;
  }
  w.Push(kSlipRad, 0.0);
  EXPECT_TRUE(Detected(w));
  for (int i = 0; i < 50; ++i)
  {
    w.Push(kSlipRad, 0.0);
    EXPECT_TRUE(Detected(w)) << "sustained frame " << i;
  }
}

TEST(SlipWindow, CoordinatedTurnIsNotDetected)
{
  fg::SlipWindow w(kWindow);
  for (int i = 0; i < 100; ++i)
  {
    w.Push(0.02, 0.02);
    EXPECT_FALSE(Detected(w)) << "frame " << i;
  }
}

TEST(SlipWindow, ColdWindowNeverVetoesEvenOnHugeResidual)
{
  fg::SlipWindow w(kWindow);
  for (std::size_t i = 0; i < kWindow - 1; ++i)
  {
    w.Push(0.5, 0.0);
    EXPECT_FALSE(Detected(w)) << "frame " << i;
  }
}

TEST(SlipWindow, MinNodesBelowWindowArmsEarlier)
{
  fg::SlipWindow w(kWindow, 4);
  w.Push(kSlipRad, 0.0);
  w.Push(kSlipRad, 0.0);
  w.Push(kSlipRad, 0.0);
  EXPECT_FALSE(Detected(w));
  w.Push(kSlipRad, 0.0);
  EXPECT_TRUE(Detected(w));
}

TEST(SlipWindow, ClearsWithinOneWindowAfterSlipStops)
{
  fg::SlipWindow w(kWindow);
  for (std::size_t i = 0; i < 3 * kWindow; ++i)
    w.Push(kSlipRad, 0.0);
  ASSERT_TRUE(Detected(w));
  // Slip stops: wheels agree with the gyro (both still).
  std::size_t frames_to_clear = 0;
  while (Detected(w))
  {
    w.Push(0.0, 0.0);
    ++frames_to_clear;
    ASSERT_LE(frames_to_clear, kWindow) << "veto did not clear within one window";
  }
  EXPECT_LE(frames_to_clear, kWindow);
  EXPECT_EQ(w.Size(), kWindow);  // eviction keeps the ring at capacity
}

TEST(SlipWindow, WindowOfOneReproducesPerNodeGate)
{
  fg::SlipWindow w(1);
  w.Push(0.03, 0.002);
  EXPECT_TRUE(Detected(w));
  w.Push(2.0 * kTickRad, 0.0);  // per-node gate trips on 2-tick jitter
  EXPECT_TRUE(Detected(w));
  w.Push(0.0, 0.0);
  EXPECT_FALSE(Detected(w));
  w.Clear();
  EXPECT_EQ(w.Size(), 0u);
  EXPECT_FALSE(Detected(w));
}
