// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Performance benchmarks for fusion_graph_core. Single-threaded
// micro-benchmarks targeting the per-Tick hot path that dominates
// fusion_graph_node CPU when the graph passes ~3 k nodes.
//
// Each test prints a one-line summary so regressions in CI logs are
// grep-able. They also assert loose ceilings so a major regression
// (e.g. accidentally O(N²) relinearization) fails the test.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <random>
#include <vector>

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>

namespace fg = fusion_graph;

namespace
{

template <typename F>
double TimeMillis(F&& fn)
{
  const auto t0 = std::chrono::steady_clock::now();
  fn();
  const auto t1 = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────
// Baseline: pure node creation through Tick.
// Measures iSAM2 update + calculateEstimate cost as N grows.
// Expected: ~constant per-tick once tree is warm. Sub-linear growth
// with N is OK; super-linear is a red flag.
// ─────────────────────────────────────────────────────────────────────
TEST(Perf, BareTickThroughput)
{
  fg::GraphParams gp;
  gp.cov_update_every_n = 10;
  gp.isam2_relinearize_skip = 5;
  fg::GraphManager gm(gp);
  gm.Initialize(gtsam::Pose2(), 0.0);

  constexpr int kN = 2000;
  std::vector<double> per_tick_ms;
  per_tick_ms.reserve(kN);

  for (int i = 0; i < kN; ++i)
  {
    // Simulate 0.1 s of straight-line motion @ 1 m/s + tight RTK GPS.
    gm.AddWheelTwist(1.0, 0.0, 0.0, 0.1);
    gm.AddGyroDelta(0.0, 0.1);
    gm.QueueGnss(0.1 * (i + 1), 0.0, 0.005);
    const double now_s = 0.1 * (i + 1);
    double dt = TimeMillis(
        [&]
        {
          gm.Tick(now_s);
        });
    per_tick_ms.push_back(dt);
  }

  std::sort(per_tick_ms.begin(), per_tick_ms.end());
  const double med = per_tick_ms[kN / 2];
  const double p95 = per_tick_ms[kN * 95 / 100];
  const double last100_avg =
      std::accumulate(per_tick_ms.end() - 100, per_tick_ms.end(), 0.0) / 100.0;
  std::printf(
      "[Perf] BareTickThroughput N=%d  median=%.2f ms  p95=%.2f ms  "
      "last100_avg=%.2f ms\n",
      kN,
      med,
      p95,
      last100_avg);

  // Loose ceiling: median tick under 30 ms even on slow ARM. If we
  // regress to >30 ms median the 10 Hz publish target is unreachable.
  EXPECT_LT(med, 30.0);
}
