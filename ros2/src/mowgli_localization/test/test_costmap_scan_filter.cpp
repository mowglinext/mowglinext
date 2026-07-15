// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// test_costmap_scan_filter.cpp — unit tests for the static filter
// helper in costmap_scan_filter_node. Drives the radial blanking
// directly without instantiating a node, so this stays a pure-C++ test.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <vector>

#include "gtest/gtest.h"
#include "sensor_msgs/msg/laser_scan.hpp"

// The filter logic now lives in mowgli_localization/scan_filters.hpp (shared
// with the node — the "refactor into a shared header" this file's old
// hand-mirrored copies asked for once the sector-limited dock blank became a
// third pass). The aliases below keep the historical test names while
// exercising the exact deployed functions.
#include "mowgli_localization/scan_filters.hpp"

namespace mowgli_localization
{
using Vec3ForTest = Vec3;
using GroundFilterConfigForTest = GroundFilterConfig;

inline void apply_ground_filter_for_test(sensor_msgs::msg::LaserScan& io,
                                         const GroundFilterConfigForTest& cfg,
                                         const std::optional<Vec3>& up_in_imu)
{
  apply_ground_filter(io, cfg, up_in_imu);
}

/// Legacy call shape: single radial blank at every bearing == the new
/// filter_scan with a full-circle (2π) dock sector and no chassis blank.
inline sensor_msgs::msg::LaserScan filter_scan_for_test(const sensor_msgs::msg::LaserScan& in,
                                                        double dock_blank_range,
                                                        bool blank_active)
{
  return filter_scan(in, 0.0, dock_blank_range, blank_active, 2.0 * M_PI, 0.0);
}

/// Build the up-in-IMU vector for a robot pitched `pitch_rad` (positive
/// = nose-down) at rest. accel = (-g·sin θ, 0, +g·cos θ); up = accel/|g|.
inline Vec3ForTest up_from_pitch_rad(double pitch_rad)
{
  return Vec3ForTest{-std::sin(pitch_rad), 0.0, std::cos(pitch_rad)};
}
}  // namespace mowgli_localization

namespace
{

sensor_msgs::msg::LaserScan make_scan(const std::vector<float>& ranges)
{
  sensor_msgs::msg::LaserScan s;
  s.angle_min = -1.57f;
  s.angle_max = 1.57f;
  s.angle_increment = 3.14f / std::max<size_t>(1, ranges.size() - 1);
  s.range_min = 0.05f;
  s.range_max = 12.0f;
  s.ranges = ranges;
  return s;
}

}  // namespace

TEST(CostmapScanFilter, PassThroughWhenInactive)
{
  auto in = make_scan({0.10f, 0.30f, 0.65f, 1.0f, 5.0f});
  auto out = mowgli_localization::filter_scan_for_test(in, 0.70, false);
  ASSERT_EQ(out.ranges.size(), in.ranges.size());
  for (size_t i = 0; i < in.ranges.size(); ++i)
    EXPECT_FLOAT_EQ(out.ranges[i], in.ranges[i]);
}

TEST(CostmapScanFilter, BlanksReturnsBelowThresholdWhenActive)
{
  auto in = make_scan({0.10f, 0.30f, 0.69f, 0.71f, 1.0f, 5.0f});
  auto out = mowgli_localization::filter_scan_for_test(in, 0.70, true);
  ASSERT_EQ(out.ranges.size(), in.ranges.size());
  EXPECT_FALSE(std::isfinite(out.ranges[0]));
  EXPECT_FALSE(std::isfinite(out.ranges[1]));
  EXPECT_FALSE(std::isfinite(out.ranges[2]));
  EXPECT_FLOAT_EQ(out.ranges[3], 0.71f);
  EXPECT_FLOAT_EQ(out.ranges[4], 1.0f);
  EXPECT_FLOAT_EQ(out.ranges[5], 5.0f);
}

TEST(CostmapScanFilter, LeavesNonFiniteValuesAlone)
{
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  auto in = make_scan({inf, nan, 0.20f, 2.0f});
  auto out = mowgli_localization::filter_scan_for_test(in, 0.70, true);
  ASSERT_EQ(out.ranges.size(), in.ranges.size());
  EXPECT_FALSE(std::isfinite(out.ranges[0]));  // was inf
  EXPECT_TRUE(std::isnan(out.ranges[1]));      // NaN preserved
  EXPECT_FALSE(std::isfinite(out.ranges[2]));  // 0.20 < 0.70 → +inf
  EXPECT_FLOAT_EQ(out.ranges[3], 2.0f);
}

TEST(CostmapScanFilter, ThresholdBoundaryIsExclusive)
{
  // A return exactly equal to the threshold should NOT be blanked
  // (filter uses `r < threshold`, not `<=`).
  auto in = make_scan({0.70f});
  auto out = mowgli_localization::filter_scan_for_test(in, 0.70, true);
  EXPECT_FLOAT_EQ(out.ranges[0], 0.70f);
}

// ─────────────────────────────────────────────────────────────────────────
// Ground filter (IMU-aware slope tolerance)
// ─────────────────────────────────────────────────────────────────────────

namespace
{
sensor_msgs::msg::LaserScan make_forward_only_scan(float range_m)
{
  // Single beam pointing along +X (α=0). Easiest to reason about
  // because cos α = 1, sin α = 0 → z_dir = 2·(qx·qz − qw·qy).
  sensor_msgs::msg::LaserScan s;
  s.angle_min = 0.0f;
  s.angle_max = 0.0f;
  s.angle_increment = 0.0f;
  s.range_min = 0.05f;
  s.range_max = 12.0f;
  s.ranges = {range_m};
  return s;
}
}  // namespace

TEST(CostmapScanFilterGround, NoOpWhenDisabled)
{
  // Even with a steep nose-down pitch, disabled filter must not touch ranges.
  auto in = make_forward_only_scan(2.0f);
  mowgli_localization::GroundFilterConfigForTest cfg{
      false, 0.08, 1.5, 0.22};
  std::optional<mowgli_localization::Vec3ForTest> u =
      mowgli_localization::up_from_pitch_rad(0.30);  // ~17° nose-down
  mowgli_localization::apply_ground_filter_for_test(in, cfg, u);
  EXPECT_FLOAT_EQ(in.ranges[0], 2.0f);
}

TEST(CostmapScanFilterGround, NoOpWhenNoImu)
{
  // Filter enabled but no IMU sample → pass-through (failsafe).
  auto in = make_forward_only_scan(2.0f);
  mowgli_localization::GroundFilterConfigForTest cfg{
      true, 0.08, 1.5, 0.22};
  std::optional<mowgli_localization::Vec3ForTest> u;  // empty
  mowgli_localization::apply_ground_filter_for_test(in, cfg, u);
  EXPECT_FLOAT_EQ(in.ranges[0], 2.0f);
}

TEST(CostmapScanFilterGround, FlatGroundReturnPassesThroughOnLevelRobot)
{
  // Level robot — up vector is +Z, beam Z component is 0. Forward beam
  // at 2 m projects to Z = 0.22 + 2·0 = 0.22 m, in [0.08, 1.5] → kept.
  auto in = make_forward_only_scan(2.0f);
  mowgli_localization::GroundFilterConfigForTest cfg{
      true, 0.08, 1.5, 0.22};
  std::optional<mowgli_localization::Vec3ForTest> u =
      mowgli_localization::Vec3ForTest{0.0, 0.0, 1.0};  // level
  mowgli_localization::apply_ground_filter_for_test(in, cfg, u);
  EXPECT_FLOAT_EQ(in.ranges[0], 2.0f);
}

TEST(CostmapScanFilterGround, GroundReturnFilteredOnNoseDownSlope)
{
  // Robot pitched nose-down 10°. Forward beam at 2 m: z_dir = -sin(10°) =
  // -0.174 → return Z = 0.22 + 2·(-0.174) = -0.127 m, well below 0.08 m
  // floor → must be filtered to +inf.
  auto in = make_forward_only_scan(2.0f);
  // min_ground_run=1: this test exercises the per-beam z-projection math, not
  // the cluster guard (a single-beam scan has no run length to speak of).
  mowgli_localization::GroundFilterConfigForTest cfg{true, 0.08, 1.5, 0.22, 0.0, 1};
  const double pitch_rad = 10.0 * M_PI / 180.0;  // nose-down (positive in URDF Y rotation)
  std::optional<mowgli_localization::Vec3ForTest> u =
      mowgli_localization::up_from_pitch_rad(pitch_rad);
  mowgli_localization::apply_ground_filter_for_test(in, cfg, u);
  EXPECT_FALSE(std::isfinite(in.ranges[0]));
}

TEST(CostmapScanFilterGround, NearObstacleSurvivesNoseDownSlope)
{
  // Same 10° nose-down pitch but the return is at 0.5 m. Z = 0.22 +
  // 0.5·(-0.174) = 0.133 m, still above 0.08 floor → keep as obstacle.
  auto in = make_forward_only_scan(0.5f);
  mowgli_localization::GroundFilterConfigForTest cfg{
      true, 0.08, 1.5, 0.22};
  const double pitch_rad = 10.0 * M_PI / 180.0;
  std::optional<mowgli_localization::Vec3ForTest> u =
      mowgli_localization::up_from_pitch_rad(pitch_rad);
  mowgli_localization::apply_ground_filter_for_test(in, cfg, u);
  EXPECT_FLOAT_EQ(in.ranges[0], 0.5f);
}

TEST(CostmapScanFilterGround, OverheadReturnFilteredOnLevelRobot)
{
  // Lift the LIDAR origin by 1.4 m so a 2 m forward beam on a level robot
  // would project to Z = 1.4 m, which is below max_obstacle_z_m (1.5).
  // Push the LIDAR origin to 1.55 m: a 2 m return projects to Z = 1.55 m
  // (level robot), above the 1.5 m ceiling → filtered.
  auto in = make_forward_only_scan(2.0f);
  mowgli_localization::GroundFilterConfigForTest cfg{
      true, 0.08, 1.5, 1.55};
  std::optional<mowgli_localization::Vec3ForTest> u =
      mowgli_localization::Vec3ForTest{0.0, 0.0, 1.0};  // level
  mowgli_localization::apply_ground_filter_for_test(in, cfg, u);
  EXPECT_FALSE(std::isfinite(in.ranges[0]));
}

namespace
{
// Single beam at an explicit LIDAR-frame angle α. Lets the mount-yaw
// tests place a return on the forward/rear half of the LIDAR ring.
sensor_msgs::msg::LaserScan make_single_beam_at(float alpha_rad, float range_m)
{
  sensor_msgs::msg::LaserScan s;
  s.angle_min = alpha_rad;
  s.angle_max = alpha_rad;
  s.angle_increment = 0.0f;
  s.range_min = 0.05f;
  s.range_max = 12.0f;
  s.ranges = {range_m};
  return s;
}
}  // namespace

TEST(CostmapScanFilterGround, MountYawPiFiltersForwardGroundReturn)
{
  // 180°-rotated LIDAR mount (lidar_mount_yaw = π): the beam pointing
  // FORWARD in the robot/base frame sits at LIDAR angle α = π. On a 10°
  // nose-down slope that forward ground return at 2 m must be filtered.
  // ψ = π + π ≡ 0 → z_dir = u.x = -sin(10°) → Z = 0.22 - 0.35 < 0.08.
  const double pitch_rad = 10.0 * M_PI / 180.0;
  auto in = make_single_beam_at(static_cast<float>(M_PI), 2.0f);
  // min_ground_run=1: per-beam mount-yaw math test (see note above).
  mowgli_localization::GroundFilterConfigForTest cfg{true, 0.08, 1.5, 0.22, M_PI, 1};
  auto u = mowgli_localization::up_from_pitch_rad(pitch_rad);
  mowgli_localization::apply_ground_filter_for_test(in, cfg, u);
  EXPECT_FALSE(std::isfinite(in.ranges[0]));
}

// ─────────────────────────────────────────────────────────────────────────
// Cluster guard (SAFETY): on a slope a real vertical obstacle and sloped
// ground both project below min_obstacle_z, so a 2-D filter can't tell them
// apart per-beam. Ground forms a LONG contiguous arc; an obstacle subtends
// only a few beams. The run-length guard strips the long ground arc but keeps
// the short obstacle cluster so the costmap/collision path still sees it.
// ─────────────────────────────────────────────────────────────────────────
TEST(CostmapScanFilterGround, ShortObstacleClusterSurvivesLongGroundRunOnSlope)
{
  // All beams forward (uniform z_dir), nose-down 10° so any finite 2 m return
  // projects to ~-0.13 m → ground-classified. The realistic obstacle geometry:
  // a vertical obstacle occupies a few bearings (returns at ~2 m, which the
  // slope mis-projects below the floor), while the bearings around it see sky /
  // no return (inf) — so the obstacle is a SHORT ground-classified run isolated
  // by non-returns, NOT contiguous with the wide ground sweep.
  const float inf = std::numeric_limits<float>::infinity();
  std::vector<float> ranges(30, inf);
  for (int i = 0; i <= 14; ++i)
    ranges[i] = 2.0f;  // long ground arc: 15 beams >= min_ground_run(8) → stripped
  // beams [15..19] = inf (no return) → breaks the run
  ranges[20] = 2.0f;
  ranges[21] = 2.0f;
  ranges[22] = 2.0f;  // 3-beam obstacle cluster (< 8) isolated by inf → kept
  auto in = make_scan(ranges);
  in.angle_min = 0.0f;  // all beams forward so z_dir is uniform (pure run-length test)
  in.angle_max = 0.0f;
  in.angle_increment = 0.0f;
  mowgli_localization::GroundFilterConfigForTest cfg{true, 0.08, 1.5, 0.22, 0.0, 8};
  auto u = mowgli_localization::up_from_pitch_rad(10.0 * M_PI / 180.0);
  mowgli_localization::apply_ground_filter_for_test(in, cfg, u);
  // The long 2.0 m ground arc is stripped...
  EXPECT_FALSE(std::isfinite(in.ranges[0]));
  EXPECT_FALSE(std::isfinite(in.ranges[14]));
  // ...but the isolated short cluster survives (kept as a possible obstacle).
  EXPECT_FLOAT_EQ(in.ranges[20], 2.0f);
  EXPECT_FLOAT_EQ(in.ranges[21], 2.0f);
  EXPECT_FLOAT_EQ(in.ranges[22], 2.0f);
}

TEST(CostmapScanFilterGround, MountYawPiKeepsRearBeam)
{
  // Same π mount + nose-down: the beam at LIDAR α = 0 points to the REAR
  // in base, which tilts UP on a nose-down robot, so a 2 m return there
  // is not ground. ψ = 0 + π = π → z_dir = -u.x = +sin(10°) → Z rises,
  // stays in band → kept.
  const double pitch_rad = 10.0 * M_PI / 180.0;
  auto in = make_single_beam_at(0.0f, 2.0f);
  mowgli_localization::GroundFilterConfigForTest cfg{true, 0.08, 1.5, 0.22, M_PI};
  auto u = mowgli_localization::up_from_pitch_rad(pitch_rad);
  mowgli_localization::apply_ground_filter_for_test(in, cfg, u);
  EXPECT_FLOAT_EQ(in.ranges[0], 2.0f);
}

TEST(CostmapScanFilterGround, UnaccountedMountYawInvertsFilter)
{
  // Regression guard: with the mount yaw left at 0 (the old bug) on a
  // π-mounted robot, the forward ground return at LIDAR α = π is NOT
  // filtered — ψ = π → z_dir = -u.x = +sin(10°) → Z rises above the
  // floor → phantom obstacle survives. This is exactly the slope failure
  // the lidar_mount_yaw plumbing fixes; the assertion documents the
  // wrong behaviour so a future refactor can't silently reintroduce it.
  const double pitch_rad = 10.0 * M_PI / 180.0;
  auto in = make_single_beam_at(static_cast<float>(M_PI), 2.0f);
  // mount yaw NOT applied (the old bug)
  mowgli_localization::GroundFilterConfigForTest cfg{true, 0.08, 1.5, 0.22, 0.0};
  auto u = mowgli_localization::up_from_pitch_rad(pitch_rad);
  mowgli_localization::apply_ground_filter_for_test(in, cfg, u);
  EXPECT_FLOAT_EQ(in.ranges[0], 2.0f);  // bug: ground return survives
}

TEST(CostmapScanFilterGround, NonFiniteRangesUntouched)
{
  // Inf and NaN beams must remain non-finite regardless of filter logic.
  const float inf = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  sensor_msgs::msg::LaserScan in;
  in.angle_min = 0.0f;
  in.angle_max = static_cast<float>(M_PI);
  in.ranges = {inf, nan, 2.0f};
  in.angle_increment = static_cast<float>(M_PI / 2.0);
  in.range_min = 0.05f;
  in.range_max = 12.0f;
  mowgli_localization::GroundFilterConfigForTest cfg{
      true, 0.08, 1.5, 0.22};
  std::optional<mowgli_localization::Vec3ForTest> u =
      mowgli_localization::up_from_pitch_rad(0.30);
  mowgli_localization::apply_ground_filter_for_test(in, cfg, u);
  EXPECT_FALSE(std::isfinite(in.ranges[0]));
  EXPECT_TRUE(std::isnan(in.ranges[1]));
}

// ─────────────────────────────────────────────────────────────────────────────
// Sector-limited dock blank (dock_blank_sector_deg). The dock/canopy sits
// FORWARD of a nose-in-docked robot; keeping REAR beams live during the
// charge/post-undock window is what lets the undock BackUp collision check
// and the calibration rear guard see a real obstacle behind the robot.
// ─────────────────────────────────────────────────────────────────────────────

namespace
{

/// Full-circle 5-beam scan: bearings -π, -π/2, 0, +π/2, +π at LIDAR index
/// angles (yaw rotation applied by the filter under test).
sensor_msgs::msg::LaserScan make_scan_360(const std::vector<float>& ranges)
{
  sensor_msgs::msg::LaserScan s;
  s.angle_min = static_cast<float>(-M_PI);
  s.angle_max = static_cast<float>(M_PI);
  s.angle_increment = static_cast<float>(2.0 * M_PI / (ranges.size() - 1));
  s.range_min = 0.05f;
  s.range_max = 12.0f;
  s.ranges = ranges;
  return s;
}

constexpr double kSector220 = 220.0 * M_PI / 180.0;

}  // namespace

TEST(CostmapScanFilterSector, RearReturnSurvivesForwardBlanked)
{
  // Beams at -π, -π/2, 0, +π/2, +π — all at 0.5 m (< 0.70 dock range).
  auto in = make_scan_360({0.5f, 0.5f, 0.5f, 0.5f, 0.5f});
  auto out = mowgli_localization::filter_scan(in,
                                              /*chassis*/ 0.0,
                                              /*dock*/ 0.70,
                                              /*dock_active*/ true,
                                              kSector220,
                                              /*lidar_yaw*/ 0.0);
  // Forward (index 2, ψ=0) and sides (±90° < 110° half-sector) blanked.
  EXPECT_FALSE(std::isfinite(out.ranges[2])) << "forward return must be blanked";
  EXPECT_FALSE(std::isfinite(out.ranges[1])) << "-90° return inside the 220° sector";
  EXPECT_FALSE(std::isfinite(out.ranges[3])) << "+90° return inside the 220° sector";
  // Rear (±π, |ψ|=180° > 110°) stays LIVE — the undock BackUp's view.
  EXPECT_FLOAT_EQ(out.ranges[0], 0.5f) << "rear return must survive the sector blank";
  EXPECT_FLOAT_EQ(out.ranges[4], 0.5f) << "rear return must survive the sector blank";
}

TEST(CostmapScanFilterSector, FullCircleSectorIsLegacyBlank)
{
  auto in = make_scan_360({0.5f, 0.5f, 0.5f, 0.5f, 0.5f});
  auto out = mowgli_localization::filter_scan(in, 0.0, 0.70, true, 2.0 * M_PI, 0.0);
  for (const auto& r : out.ranges)
  {
    EXPECT_FALSE(std::isfinite(r)) << "360° sector must blank every bearing (legacy)";
  }
}

TEST(CostmapScanFilterSector, MountYawRotatesSectorToBaseForward)
{
  // 180°-rotated mount (this robot): LIDAR index angle ±π points BASE
  // forward. With lidar_yaw=π the blank must land on index ±π beams and
  // spare the index-0 beam (base rear).
  auto in = make_scan_360({0.5f, 0.5f, 0.5f, 0.5f, 0.5f});
  auto out = mowgli_localization::filter_scan(in, 0.0, 0.70, true, kSector220, M_PI);
  EXPECT_FALSE(std::isfinite(out.ranges[0])) << "index -π beam is base-forward → blanked";
  EXPECT_FALSE(std::isfinite(out.ranges[4])) << "index +π beam is base-forward → blanked";
  EXPECT_FLOAT_EQ(out.ranges[2], 0.5f) << "index 0 beam is base-rear → live";
}

TEST(CostmapScanFilterSector, ChassisBlankStaysRadial)
{
  // Chassis self-return blank has no sector: a rear return inside
  // chassis_blank_range is blanked even though the dock sector spares it,
  // and even with the dock blank inactive.
  auto in = make_scan_360({0.2f, 5.0f, 0.2f, 5.0f, 0.2f});
  auto out = mowgli_localization::filter_scan(in,
                                              /*chassis*/ 0.3,
                                              /*dock*/ 0.70,
                                              /*dock_active*/ false,
                                              kSector220,
                                              0.0);
  EXPECT_FALSE(std::isfinite(out.ranges[0])) << "rear chassis self-return blanked";
  EXPECT_FALSE(std::isfinite(out.ranges[2])) << "forward chassis self-return blanked";
  EXPECT_FALSE(std::isfinite(out.ranges[4])) << "rear chassis self-return blanked";
  EXPECT_FLOAT_EQ(out.ranges[1], 5.0f);
  EXPECT_FLOAT_EQ(out.ranges[3], 5.0f);
}

TEST(CostmapScanFilterSector, InactiveDockBlankLeavesNearReturns)
{
  auto in = make_scan_360({0.5f, 0.5f, 0.5f, 0.5f, 0.5f});
  auto out = mowgli_localization::filter_scan(in, 0.0, 0.70, false, kSector220, 0.0);
  for (const auto& r : out.ranges)
  {
    EXPECT_FLOAT_EQ(r, 0.5f) << "no blank outside the charge/post-undock window";
  }
}
