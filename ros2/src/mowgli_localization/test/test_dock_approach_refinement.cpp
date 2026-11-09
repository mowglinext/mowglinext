// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Pure tests of the gps_dock_detection final-approach refinement
// (dock_approach_refinement.hpp): the GPS track fit of the map→odom yaw, its
// blending and gates, the approach episode and the freeze radius.

#include <cmath>
#include <random>

#include "mowgli_localization/dock_approach_refinement.hpp"
#include <gtest/gtest.h>

namespace ml = mowgli_localization;

namespace
{
constexpr double kDeg = M_PI / 180.0;

// p_map = R(delta)·p_odom + t
ml::Vec2 odom_to_map(const ml::Vec2& p, double delta, const ml::Vec2& t)
{
  const ml::Vec2 r = ml::rotate(p, delta);
  return {r.x + t.x, r.y + t.y};
}

ml::Vec2 map_to_odom(const ml::Vec2& p, double delta, const ml::Vec2& t)
{
  return ml::rotate({p.x - t.x, p.y - t.y}, -delta);
}
}  // namespace

TEST(DockInOdom, MatchesTheTrueTransformWhenDeltaIsRight)
{
  const double delta = 0.7;
  const ml::Vec2 t{3.0, -2.0};
  const ml::PlanarPose dock_map{6.1, 2.6, -0.886};
  const ml::Vec2 anchor_odom{1.0, 0.5};
  const ml::Vec2 anchor_map = odom_to_map(anchor_odom, delta, t);

  const ml::PlanarPose got = ml::dock_in_odom(anchor_odom, anchor_map, dock_map, delta);

  const ml::Vec2 expected = map_to_odom({dock_map.x, dock_map.y}, delta, t);
  EXPECT_NEAR(got.x, expected.x, 1e-9);
  EXPECT_NEAR(got.y, expected.y, 1e-9);
  EXPECT_NEAR(got.yaw, ml::wrap_to_pi(dock_map.yaw - delta), 1e-9);
}

TEST(TrackRotationFit, NeedsThreePairs)
{
  ml::TrackRotationFit fit;
  fit.add({0, 0}, {0, 0});
  fit.add({1, 0}, {1, 0});
  EXPECT_FALSE(fit.fit().valid);
}

TEST(TrackRotationFit, RecoversTheRotationOfANoisyStraightTrack)
{
  const double delta = -0.65;
  const ml::Vec2 t{5.0, 1.0};
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, 0.01);  // 1 cm RTK-Fixed
  ml::TrackRotationFit fit;
  // 0.8 m straight forward track at 5 Hz, 0.17 m/s.
  for (int i = 0; i <= 24; ++i)
  {
    const ml::Vec2 odom{0.034 * i, 0.0};
    ml::Vec2 map = odom_to_map(odom, delta, t);
    map.x += noise(rng);
    map.y += noise(rng);
    fit.add(odom, map);
  }
  const ml::TrackFitResult r = fit.fit();
  ASSERT_TRUE(r.valid);
  EXPECT_NEAR(r.delta, delta, 2.0 * kDeg);
  EXPECT_NEAR(r.extent_m, 0.816, 1e-6);
  EXPECT_LT(r.rms_m, 0.02);
}

TEST(TrackRotationFit, DropsTheOldestPairsBeyondTheCap)
{
  ml::TrackFitConfig cfg;
  cfg.max_pairs = 5;
  ml::TrackRotationFit fit(cfg);
  for (int i = 0; i < 9; ++i)
  {
    fit.add({0.1 * i, 0}, {0.1 * i, 0});
  }
  EXPECT_EQ(fit.size(), 5u);
  EXPECT_NEAR(fit.fit().extent_m, 0.4, 1e-9);
}

namespace
{
ml::TrackFitResult result(double delta, double extent, double rms = 0.01)
{
  ml::TrackFitResult r;
  r.valid = true;
  r.delta = delta;
  r.extent_m = extent;
  r.rms_m = rms;
  r.pairs = 10;
  return r;
}
}  // namespace

TEST(BlendTrackDelta, KeepsTheTfBelowTheMinimumTravel)
{
  const ml::TrackFitConfig cfg;
  const ml::TrackBlend b = ml::blend_track_delta(0.5, result(0.55, 0.2), cfg);
  EXPECT_EQ(b.state, ml::TrackBlendState::TF_ONLY);
  EXPECT_DOUBLE_EQ(b.delta, 0.5);
}

TEST(BlendTrackDelta, RampsLinearlyThenReplacesTheTf)
{
  const ml::TrackFitConfig cfg;  // 0.30 → 0.60 m
  const double tf = 0.5;
  const double fit = tf + 4.0 * kDeg;

  const ml::TrackBlend half = ml::blend_track_delta(tf, result(fit, 0.45), cfg);
  EXPECT_EQ(half.state, ml::TrackBlendState::BLENDING);
  EXPECT_NEAR(half.weight, 0.5, 1e-9);
  EXPECT_NEAR(half.delta, tf + 2.0 * kDeg, 1e-9);

  const ml::TrackBlend full = ml::blend_track_delta(tf, result(fit, 0.9), cfg);
  EXPECT_EQ(full.state, ml::TrackBlendState::TRACK);
  EXPECT_NEAR(full.delta, fit, 1e-9);
}

TEST(BlendTrackDelta, BlendsAcrossTheWrapAround)
{
  const ml::TrackFitConfig cfg;
  const double tf = M_PI - 1.0 * kDeg;
  const double fit = -M_PI + 1.0 * kDeg;  // 2° away, across ±π
  const ml::TrackBlend b = ml::blend_track_delta(tf, result(fit, 1.0), cfg);
  EXPECT_EQ(b.state, ml::TrackBlendState::TRACK);
  EXPECT_NEAR(ml::wrap_to_pi(b.delta - fit), 0.0, 1e-9);
}

TEST(BlendTrackDelta, RejectsAFitFarFromTheTf)
{
  const ml::TrackFitConfig cfg;  // 10°
  const ml::TrackBlend b = ml::blend_track_delta(0.5, result(0.5 + 15.0 * kDeg, 1.0), cfg);
  EXPECT_EQ(b.state, ml::TrackBlendState::REJECTED);
  EXPECT_DOUBLE_EQ(b.delta, 0.5);
}

TEST(BlendTrackDelta, RejectsANoisyFit)
{
  const ml::TrackFitConfig cfg;  // 0.04 m
  const ml::TrackBlend b = ml::blend_track_delta(0.5, result(0.52, 1.0, 0.08), cfg);
  EXPECT_EQ(b.state, ml::TrackBlendState::REJECTED);
  EXPECT_DOUBLE_EQ(b.delta, 0.5);
}

TEST(ApproachEpisode, ForwardCommandsOnlyAndTimesOut)
{
  ml::ApproachEpisode ep(1.0);
  EXPECT_FALSE(ep.active(0.0));
  ep.on_command(-0.15, 0.0);  // retry return / calibration reverse leg
  EXPECT_FALSE(ep.active(0.1));
  ep.on_command(0.16, 1.0);
  EXPECT_TRUE(ep.active(1.5));
  EXPECT_TRUE(ep.active(2.0));
  EXPECT_FALSE(ep.active(2.01));
}

TEST(ShouldFreeze, InsideTheRadiusOnlyAndDisabledAtZero)
{
  const ml::PlanarPose dock{1.0, 0.0, 0.0};
  EXPECT_TRUE(ml::should_freeze({0.6, 0.0}, dock, 0.5));
  EXPECT_FALSE(ml::should_freeze({0.4, 0.0}, dock, 0.5));
  EXPECT_FALSE(ml::should_freeze({0.99, 0.0}, dock, 0.0));
}

// The case the refinement exists for: the fused heading (hence the TF Δ) is 3°
// off during the approach. The TF-only target heading carries the whole error;
// the track fit brings it back once the robot has driven its runway.
TEST(DockApproachRefinement, TrackFitRemovesAFusedHeadingError)
{
  const double true_delta = 0.9;
  const double tf_delta = true_delta + 3.0 * kDeg;
  const ml::Vec2 t{4.0, -1.0};
  const ml::PlanarPose dock_map{6.1, 2.66, -0.886};
  const ml::Vec2 dock_odom_true = map_to_odom({dock_map.x, dock_map.y}, true_delta, t);
  const double dock_yaw_odom_true = ml::wrap_to_pi(dock_map.yaw - true_delta);

  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.01);
  ml::TrackRotationFit fit;
  ml::TrackBlend blend;
  // Drive the true dock axis from 1.5 m out towards the dock.
  for (int i = 0; i <= 25; ++i)
  {
    const double back = 1.5 - 0.034 * i;
    const ml::Vec2 antenna_odom{dock_odom_true.x - back * std::cos(dock_yaw_odom_true),
                                dock_odom_true.y - back * std::sin(dock_yaw_odom_true)};
    ml::Vec2 antenna_map = odom_to_map(antenna_odom, true_delta, t);
    antenna_map.x += noise(rng);
    antenna_map.y += noise(rng);
    fit.add(antenna_odom, antenna_map);
    blend = ml::blend_track_delta(tf_delta, fit.fit(), fit.config());
  }

  ASSERT_EQ(blend.state, ml::TrackBlendState::TRACK);
  const ml::PlanarPose tf_only = ml::dock_in_odom(dock_odom_true,
                                                  odom_to_map(dock_odom_true, true_delta, t),
                                                  dock_map,
                                                  tf_delta);
  const ml::PlanarPose refined = ml::dock_in_odom(dock_odom_true,
                                                  odom_to_map(dock_odom_true, true_delta, t),
                                                  dock_map,
                                                  blend.delta);
  EXPECT_NEAR(std::abs(ml::wrap_to_pi(tf_only.yaw - dock_yaw_odom_true)), 3.0 * kDeg, 1e-9);
  EXPECT_LT(std::abs(ml::wrap_to_pi(refined.yaw - dock_yaw_odom_true)), 1.0 * kDeg);
}
