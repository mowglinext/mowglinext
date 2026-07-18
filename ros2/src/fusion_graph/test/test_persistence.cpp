// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Regression tests for GraphManager::Save / Load. The motivating
// failure: a Reset() (Clear-graph service) followed by an auto-save
// (RECORDING-exit or dock-arrival) wrote an empty graph to disk
// (next_index=0, count=0). On next launch, Load() restored that state
// and marked the manager initialized. The first Tick then formed
// PoseKey(next_index_ - 1) → underflow to 2^64-1 → GTSAM
// std::invalid_argument → process abort.

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "fusion_graph/graph_manager.hpp"
#include <gtest/gtest.h>
#include <unistd.h>

namespace fg = fusion_graph;
namespace fs = std::filesystem;

namespace
{
std::string TempPrefix(const char* tag)
{
  auto base = fs::temp_directory_path() /
              (std::string("fg_test_") + tag + "_" + std::to_string(::getpid()));
  return base.string();
}

void CleanupPrefix(const std::string& prefix)
{
  std::error_code ec;
  fs::remove(prefix + ".graph", ec);
  fs::remove(prefix + ".scans", ec);
  fs::remove(prefix + ".meta", ec);
}
}  // namespace

// Reset() leaves the manager uninitialized with next_index_ == 0. Any
// auto-save fired in that window must NOT touch on-disk files; otherwise
// the next launch resurrects the poisoned state.
TEST(Persistence, SaveAfterResetIsRefused)
{
  const auto prefix = TempPrefix("save_after_reset");
  CleanupPrefix(prefix);

  fg::GraphManager gm(fg::GraphParams{});
  gm.Initialize(gtsam::Pose2(0.0, 0.0, 0.0), 0.0);
  ASSERT_TRUE(gm.Save(prefix));  // baseline: a real graph saves
  gm.Reset();

  EXPECT_FALSE(gm.Save(prefix));  // empty graph must be refused
  CleanupPrefix(prefix);
}

// Symmetric on the read side: even if a stale empty file pair survives
// (e.g. written by a pre-fix binary), Load() must refuse to mark the
// manager initialized — otherwise CreateNodeLocked's PoseKey(j-1) with
// j=0 underflows the GTSAM Symbol index check.
TEST(Persistence, LoadOfEmptyMetaIsRefused)
{
  const auto prefix = TempPrefix("load_empty");
  CleanupPrefix(prefix);

  // Hand-craft the empty file triple a pre-fix binary would produce.
  {
    std::ofstream g(prefix + ".graph");
    g << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\" ?>\n"
         "<!DOCTYPE boost_serialization>\n"
         "<boost_serialization signature=\"serialization::archive\" version=\"19\">\n"
         "<data class_id=\"0\" tracking_level=\"0\" version=\"0\">\n"
         "  <values_ class_id=\"1\" tracking_level=\"0\" version=\"0\">\n"
         "    <count>0</count>\n"
         "    <item_version>0</item_version>\n"
         "  </values_>\n"
         "</data>\n"
         "</boost_serialization>\n";
  }
  {
    std::ofstream s(prefix + ".scans", std::ios::binary);
    const uint64_t zero = 0;
    s.write(reinterpret_cast<const char*>(&zero), sizeof(zero));
  }
  {
    std::ofstream m(prefix + ".meta");
    m << "next_index=0\nlast_node_time_s=0.000000\n";
  }

  fg::GraphManager gm(fg::GraphParams{});
  EXPECT_FALSE(gm.Load(prefix));
  EXPECT_FALSE(gm.IsInitialized());

  // The bug repro: with the previous code, IsInitialized() was true
  // here and the next Tick aborted. Confirm Tick is a no-op now.
  EXPECT_FALSE(gm.Tick(1.0).has_value());

  CleanupPrefix(prefix);
}

// Healthy round-trip is unchanged.
TEST(Persistence, NonEmptyRoundTrip)
{
  const auto prefix = TempPrefix("roundtrip");
  CleanupPrefix(prefix);

  {
    fg::GraphManager gm(fg::GraphParams{});
    gm.Initialize(gtsam::Pose2(1.0, 2.0, 0.5), 0.0);
    // dx = vx*dt = 0.05 m must exceed the default stationary_motion_thresh_m
    // (0.02 m); otherwise Tick() is throttled and returns nullopt with the
    // out-of-the-box GraphParams{} this test uses.
    gm.AddWheelTwist(0.5, 0.0, 0.0, 0.1);
    auto out = gm.Tick(0.2);
    ASSERT_TRUE(out.has_value());
    EXPECT_TRUE(gm.Save(prefix));
  }
  {
    fg::GraphManager gm(fg::GraphParams{});
    EXPECT_TRUE(gm.Load(prefix));
    EXPECT_TRUE(gm.IsInitialized());
  }

  CleanupPrefix(prefix);
}
