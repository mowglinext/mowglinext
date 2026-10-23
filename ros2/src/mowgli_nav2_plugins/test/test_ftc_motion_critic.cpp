// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later

#include <chrono>
#include <cmath>
#include <iostream>
#include <random>

#include "mowgli_interfaces/motion_authorization.hpp"
#include "mowgli_nav2_plugins/ftc_forward_sweep.hpp"
#include "mowgli_nav2_plugins/ftc_motion_critic.hpp"
#include "mowgli_nav2_plugins/ftc_motion_geometry_index.hpp"
#include <gtest/gtest.h>

namespace
{
namespace motion = mowgli_interfaces::motion;
namespace mn = mowgli_nav2_plugins;
using Pose = geometry_msgs::msg::PoseStamped;

Pose P(double x, double y, double yaw = 0)
{
  Pose p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2);
  p.pose.orientation.w = std::cos(yaw / 2);
  return p;
}

void Add(visualization_msgs::msg::MarkerArray& message,
         const motion::Ring& ring,
         const std::string& kind = "mowing")
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.ns = kind == "obstacle" ? "obstacle" : "area";
  marker.text = kind == "obstacle" ? "" : kind;
  marker.id = static_cast<int>(message.markers.size());
  marker.type = marker.LINE_STRIP;
  marker.action = marker.ADD;
  marker.pose.orientation.w = 1;
  for (const auto& xy : ring)
  {
    geometry_msgs::msg::Point p;
    p.x = xy.x;
    p.y = xy.y;
    marker.points.push_back(p);
  }
  message.markers.push_back(marker);
}

motion::Ring Body(double size = .1)
{
  return {{-size, -size}, {size, -size}, {size, size}, {-size, size}};
}
visualization_msgs::msg::MarkerArray Lawn()
{
  visualization_msgs::msg::MarkerArray message;
  Add(message, {{-4.5, .8}, {4.5, .8}, {4.5, 6.8}, {-4.5, 6.8}});
  Add(message, {{-2.2, -1}, {1, -1}, {1, 1.8}, {-2.2, 1.8}}, "navigation");
  return message;
}

TEST(FtcMotionCritic, RawOuterPermissionAllowsBodyOverhangAndInwardSkirt)
{
  auto critic = mn::MakeMotionLatticeCritic(motion::Snapshot::parse(Lawn()), Body(.3), {0, 0, 0});
  EXPECT_FALSE(critic(P(1, .8, M_PI), P(0, .8, M_PI)));
  EXPECT_FALSE(critic(P(1, .8, M_PI), P(.9, .85, M_PI)));
  EXPECT_TRUE(critic(P(1, .8, M_PI), P(.9, .75, M_PI)));
  EXPECT_TRUE(critic(P(-.79, .7623, M_PI), P(-.81, .7623, M_PI)))
      << "Navigation-only region is not mowing permission";
}

TEST(FtcMotionCritic, AppliesCostmapToMapTransformBeforeTestingPermission)
{
  auto critic =
      mn::MakeMotionLatticeCritic(motion::Snapshot::parse(Lawn()), Body(), {1, .8, M_PI / 2});
  EXPECT_FALSE(critic(P(0, 0), P(.1, 0)));
  EXPECT_TRUE(critic(P(-.1, 0), P(-.2, 0)));
}

TEST(FtcMotionCritic, MissingAndInvalidGeometryAlwaysReject)
{
  for (auto snapshot : {std::shared_ptr<const motion::Snapshot>{}, motion::Snapshot::parse({})})
  {
    auto critic = mn::MakeMotionLatticeCritic(snapshot, Body(), {0, 0, 0});
    EXPECT_TRUE(critic(P(0, 1), P(.1, 1)));
  }
  auto bad = mn::MakeMotionLatticeCritic(motion::Snapshot::parse(Lawn()), {}, {0, 0, 0});
  EXPECT_TRUE(bad(P(0, 1), P(.1, 1)));
  auto critic = mn::MakeMotionLatticeCritic(motion::Snapshot::parse(Lawn()), Body(), {0, 0, 0});
  auto invalid = P(0, 1);
  invalid.pose.orientation.w = 0;
  EXPECT_TRUE(critic(invalid, P(.1, 1)));
}

TEST(FtcMotionCritic, FullChordRejectsConcavityBetweenAuthorizedEndpoints)
{
  visualization_msgs::msg::MarkerArray message;
  Add(message, {{0, 0}, {3, 0}, {3, 3}, {2, 3}, {2, 1}, {1, 1}, {1, 3}, {0, 3}});
  auto critic = mn::MakeMotionLatticeCritic(motion::Snapshot::parse(message), Body(.01), {0, 0, 0});
  EXPECT_FALSE(critic(P(.5, 2), P(.5, 2)));
  EXPECT_FALSE(critic(P(2.5, 2), P(2.5, 2)));
  EXPECT_TRUE(critic(P(.5, 2), P(2.5, 2)));
}

TEST(FtcMotionCritic, ThinHoleBetweenEndpointBodiesCannotBeSkipped)
{
  auto message = Lawn();
  Add(message, {{.04999, 1.999}, {.05001, 1.999}, {.05001, 2.001}, {.04999, 2.001}}, "obstacle");
  auto critic = mn::MakeMotionLatticeCritic(motion::Snapshot::parse(message), Body(.01), {0, 0, 0});
  EXPECT_FALSE(critic(P(0, 2), P(0, 2)));
  EXPECT_FALSE(critic(P(.1, 2), P(.1, 2)));
  EXPECT_TRUE(critic(P(0, 2), P(.1, 2)));
}

TEST(FtcMotionCritic, RotationBetweenClearEndpointBodiesStillChecksFullSweep)
{
  auto message = Lawn();
  Add(message, {{.30, 2.35}, {.40, 2.35}, {.40, 2.45}, {.30, 2.45}}, "obstacle");
  motion::Ring body{{-.17, -.275}, {.53, -.275}, {.53, .275}, {-.17, .275}};
  auto critic = mn::MakeMotionLatticeCritic(motion::Snapshot::parse(message), body, {0, 0, 0});
  EXPECT_FALSE(critic(P(0, 2), P(0, 2)));
  EXPECT_FALSE(critic(P(0, 2, M_PI / 2), P(0, 2, M_PI / 2)));
  EXPECT_TRUE(critic(P(0, 2), P(0, 2, M_PI / 2)));
}

TEST(FtcMotionCritic, CapturedOutwardTwistRemainsDeniedByTheFinalCertificate)
{
  auto snapshot = motion::Snapshot::parse(Lawn());
  motion::Ring body{{-.18, -.285}, {.54, -.285}, {.54, .285}, {-.18, .285}};
  EXPECT_FALSE(snapshot->permits({-.2078129601192531, .80069224232974601, -3.0830393738761721},
                                 body,
                                 .2,
                                 -.25943304604922168,
                                 .1,
                                 true));
  EXPECT_FALSE(snapshot->permits({-.789994, .7623277, 3.102}, body, .16, -.05578, .1, true));
}

TEST(FtcMotionCritic, StationZeroAndCostmapGraceCannotEraseRawGeometry)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, .05, -5, -5, 0);
  auto critic = mn::MakeMotionLatticeCritic(motion::Snapshot::parse(Lawn()), Body(), {0, 0, 0});
  mn::LatticeSolverCfg cfg;
  cfg.lattice.max_offset = .2;
  cfg.reaction_m = 0;
  cfg.lead_m = .3;
  std::vector<Pose> poses{P(0, .8, M_PI), P(-.05, .8, M_PI), P(-.1, .8, M_PI)};
  mn::LatticeSolver solver(costmap, {}, {}, poses, 0, false, cfg, critic);
  EXPECT_EQ(solver.SpanBlock(1, .05, 0, 10), mn::LatticeBlock::kGeometry);
  EXPECT_FALSE(solver.Solve(.05, 1, 0).plan.feasible);
  EXPECT_TRUE(solver.Solve(0, 0, 0).plan.feasible);
  EXPECT_EQ(solver.PoseBlock(1, -.05), mn::LatticeBlock::kFree);
}

TEST(FtcMotionCritic, NewSolverCannotReuseOldSnapshotPermission)
{
  auto before = mn::MakeMotionLatticeCritic(motion::Snapshot::parse(Lawn()), Body(), {0, 0, 0});
  auto after = mn::MakeMotionLatticeCritic(motion::Snapshot::parse({}), Body(), {0, 0, 0});
  EXPECT_FALSE(before(P(0, 1), P(.1, 1)));
  EXPECT_TRUE(after(P(0, 1), P(.1, 1)));
}

TEST(FtcMotionCritic, NonGridStartPreservesThePhysicalPoseAndFirstEdgeSlope)
{
  nav2_costmap_2d::Costmap2D costmap(200, 200, .05, -5, -5, 0);
  auto critic = mn::MakeMotionLatticeCritic(motion::Snapshot::parse(Lawn()), Body(), {0, 0, 0});
  std::vector<Pose> poses{P(0, .79), P(.05, .79), P(.1, .79)};
  mn::LatticeSolver solver(costmap, {}, {}, poses, 0, false, {}, critic);
  const auto solution = solver.Solve(.02, 0, 0);
  ASSERT_TRUE(solution.plan.feasible);
  EXPECT_DOUBLE_EQ(solution.plan.offsets[0], .02);
  for (std::size_t i = 1; i < solution.plan.offsets.size(); ++i)
  {
    EXPECT_LE(std::abs(solution.plan.offsets[i] - solution.plan.offsets[i - 1]), .05 + 1e-12);
    EXPECT_GE(poses[i].pose.position.y + solution.plan.offsets[i], .8 - 1e-8);
  }
}

TEST(FtcMotionCritic, SubNanometreNonGridOffsetCannotReuseAFreeRawHolePose)
{
  auto message = Lawn();
  Add(message,
      {{-.01, .90000000025}, {.01, .90000000025}, {.01, .90001}, {-.01, .90001}},
      "obstacle");
  auto snapshot = motion::Snapshot::parse(message);
  ASSERT_TRUE(snapshot->valid);
  auto critic = mn::MakeMotionLatticeCritic(snapshot, Body(.1), {0, 0, 0});
  nav2_costmap_2d::Costmap2D costmap(200, 200, .05, -5, -5, 0);
  mn::LatticeSolver solver(costmap, {}, {}, {P(0, .8), P(.05, .8)}, 0, false, {}, critic);
  EXPECT_EQ(solver.PoseBlock(0, 0), mn::LatticeBlock::kFree);
  EXPECT_EQ(solver.PoseBlock(0, 5e-10), mn::LatticeBlock::kGeometry);
  EXPECT_FALSE(solver.Solve(5e-10, 0, 0).plan.feasible);
}

TEST(FtcMotionCritic, CompactedRawRingRetainsThinAxisAlignedNotchesAndHoles)
{
  visualization_msgs::msg::MarkerArray message;
  Add(message,
      {{0, 0},
       {.5, 0},
       {1, 0},
       {2, 0},
       {3, 0},
       {3, 1},
       {3, 2},
       {2, 2},
       {2, .99999},
       {1.99999, .99999},
       {1.99999, 2},
       {0, 2},
       {0, 1}});
  Add(message, {{.49, .49}, {.51, .49}, {.51, .51}, {.49, .51}}, "obstacle");
  auto snapshot = motion::Snapshot::parse(message);
  ASSERT_TRUE(snapshot->valid);
  auto critic = mn::MakeMotionLatticeCritic(snapshot, Body(.001), {0, 0, 0});
  for (double y : {.1, .9, 1., 1.5})
  {
    const auto a = P(1.5, y), b = P(2.5, y);
    EXPECT_EQ(!critic(a, b), snapshot->mowing.segmentAuthorized({1.5, y}, {2.5, y}));
  }
  EXPECT_TRUE(critic(P(.4, .5), P(.6, .5)));
  EXPECT_EQ(snapshot->mowing.allowed[0].size(), 13u) << "The immutable raw snapshot is unchanged";
}

TEST(FtcMotionCritic, DenseGeometryProfileStaysWithinHostControlBudget)
{
  visualization_msgs::msg::MarkerArray message;
  motion::Ring ring;
  constexpr int n = 128;
  for (int k = 0; k < n; ++k)
    ring.push_back({-4.5 + 9. * k / n, .8});
  for (int k = 0; k < n; ++k)
    ring.push_back({4.5, .8 + 6. * k / n});
  for (int k = 0; k < n; ++k)
    ring.push_back({4.5 - 9. * k / n, 6.8});
  for (int k = 0; k < n; ++k)
    ring.push_back({-4.5, 6.8 - 6. * k / n});
  Add(message, ring);
  for (int k = 0; k < 30; ++k)
  {
    double x = -4 + double(k) * .25;
    Add(message, {{x, 5}, {x + .05, 5}, {x + .05, 5.05}, {x, 5.05}}, "obstacle");
  }
  auto snapshot = motion::Snapshot::parse(message);
  ASSERT_TRUE(snapshot->valid);
  nav2_costmap_2d::Costmap2D costmap(200, 200, .05, -5, -5, 0);
  std::vector<Pose> poses;
  for (int k = 0; k < 51; ++k)
    poses.push_back(P(2 - .05 * k, .8, M_PI));
  double worst = 0;
  for (int iteration = 0; iteration < 5; ++iteration)
  {
    auto start = std::chrono::steady_clock::now();
    auto critic = mn::MakeMotionLatticeCritic(snapshot, Body(.3), {0, 0, 0});
    mn::LatticeSolver solver(costmap, {}, {}, poses, 0, false, {}, critic);
    ASSERT_TRUE(solver.Solve(0, 0, 0).plan.feasible);
    double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    worst = std::max(worst, ms);
  }
  std::cout << "Dense geometry 512 outer vertices, 30 raw holes, 51 stations host worst_ms="
            << worst << '\n';
  EXPECT_LT(worst, 100) << "10Hz host budget only, not an ARM timing claim";
}

TEST(FtcMotionCritic, MaximumPointOrdinaryRectangleProfileHostBudget)
{
  visualization_msgs::msg::MarkerArray message;
  motion::Ring ring;
  constexpr int n = 8192;
  for (int k = 0; k < n; ++k)
    ring.push_back({-4.5 + 9. * k / n, .8});
  for (int k = 0; k < n; ++k)
    ring.push_back({4.5, .8 + 6. * k / n});
  for (int k = 0; k < n; ++k)
    ring.push_back({4.5 - 9. * k / n, 6.8});
  for (int k = 0; k < n; ++k)
    ring.push_back({-4.5, 6.8 - 6. * k / n});
  Add(message, ring);
  auto snapshot = motion::Snapshot::parse(message);
  ASSERT_TRUE(snapshot->valid);
  ASSERT_EQ(snapshot->mowing.allowed[0].size(), 32768u);
  nav2_costmap_2d::Costmap2D costmap(200, 200, .05, -5, -5, 0);
  std::vector<Pose> poses;
  for (int k = 0; k < 51; ++k)
    poses.push_back(P(2 - .05 * k, .8, M_PI));
  auto start = std::chrono::steady_clock::now();
  auto critic = mn::MakeMotionLatticeCritic(snapshot, Body(.3), {0, 0, 0});
  mn::LatticeSolver solver(costmap, {}, {}, poses, 0, false, {}, critic);
  ASSERT_TRUE(solver.Solve(0, 0, 0).plan.feasible);
  const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  std::cout << "Maximum ordinary geometry 32768 points, 51 stations full DP host ms=" << ms << '\n';
  EXPECT_LT(ms, 100) << "Host only; arbitrary nonredundant geometry remains unbounded";
  EXPECT_TRUE(critic(P(0, .8), P(.1, .7)));
  EXPECT_FALSE(critic(P(0, .8), P(.1, .9)));
}

TEST(FtcMotionCritic, MaximumNonredundantCircleRecordsTheHostBudgetLimit)
{
  visualization_msgs::msg::MarkerArray message;
  motion::Ring ring;
  for (int k = 0; k < 32768; ++k)
  {
    double angle = 2 * M_PI * k / 32768;
    ring.push_back({2 * std::cos(angle), 3 + 2 * std::sin(angle)});
  }
  Add(message, ring);
  auto snapshot = motion::Snapshot::parse(message);
  ASSERT_TRUE(snapshot->valid);
  nav2_costmap_2d::Costmap2D costmap(200, 200, .05, -5, -5, 0);
  std::vector<Pose> poses;
  for (int k = 0; k < 51; ++k)
    poses.push_back(P(1.25 - .05 * k, 3, M_PI));
  const auto start = std::chrono::steady_clock::now();
  auto critic = mn::MakeMotionLatticeCritic(snapshot, Body(.3), {0, 0, 0});
  mn::LatticeSolver solver(costmap, {}, {}, poses, 0, false, {}, critic);
  ASSERT_TRUE(solver.Solve(0, 0, 0).plan.feasible);
  const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
  std::cout << "NONREDUNDANT32768 full DP host ms=" << ms << " within_10Hz_budget=" << (ms < 100)
            << '\n';
  EXPECT_LT(ms, 100) << "Complete preparation + DP must meet the 10Hz HOST budget";
}

TEST(FtcMotionCritic, IndexedPointAndChordMatchRawPredicatesOnAdversarialRings)
{
  const std::vector<motion::Ring> rings{
      {{0, 0}, {3, 0}, {3, 3}, {2, 3}, {2, 1}, {1, 1}, {1, 3}, {0, 3}},
      {{900000, 900000}, {900001, 900000}, {900001, 900001}, {900000, 900001}},
      {{-1, -1}, {1, -1 + 1e-12}, {1, 1}, {-1, 1}},
      {{-3, -3}, {-2, -3}, {-2, -2}, {-3, -2}}};
  for (const auto& ring : rings)
  {
    motion::Geometry raw;
    raw.allowed = {ring};
    mn::MotionRegionIndex index(raw.allowed);
    const double ox = ring.front().x, oy = ring.front().y;
    std::mt19937_64 random(924);
    std::uniform_real_distribution<double> sample(-.1, 3.1);
    for (int i = 0; i < 500; ++i)
    {
      const motion::Point a{ox + sample(random), oy + sample(random)};
      const motion::Point b{ox + sample(random), oy + sample(random)};
      EXPECT_EQ(index.authorized(a), raw.authorized(a));
      EXPECT_EQ(index.segmentAuthorized(a, b), raw.segmentAuthorized(a, b));
    }
    for (const auto& v : ring)
      for (double dx : {-1.1e-8, -1e-8, -.9e-8, 0., .9e-8, 1e-8, 1.1e-8})
        for (double dy : {-1.1e-8, -1e-8, -.9e-8, 0., .9e-8, 1e-8, 1.1e-8})
        {
          const motion::Point p{v.x + dx, v.y + dy};
          EXPECT_EQ(index.authorized(p), raw.authorized(p)) << p.x << ',' << p.y;
        }
  }
  motion::Geometry raw;
  raw.allowed = rings;
  mn::MotionRegionIndex union_index(rings);
  EXPECT_EQ(union_index.segmentAuthorized({.5, 2}, {2.5, 2}),
            raw.segmentAuthorized({.5, 2}, {2.5, 2}));
}

TEST(FtcMotionCritic, PreparedContentReuseCannotBorrowDifferentSnapshotPermission)
{
  auto original = motion::Snapshot::parse(Lawn());
  auto prepared = std::make_shared<mn::MotionRegionIndex>(original);
  auto identical = motion::Snapshot::parse(Lawn());
  EXPECT_TRUE(prepared->matches(identical.get()));
  auto changed = Lawn();
  changed.markers[0].points[0].y = 2;
  changed.markers[0].points[1].y = 2;
  auto next = motion::Snapshot::parse(changed);
  ASSERT_TRUE(next->valid);
  EXPECT_FALSE(prepared->matches(next.get()));
  auto critic = mn::MakeMotionLatticeCritic(next, Body(), {0, 0, 0}, prepared);
  EXPECT_TRUE(critic(P(0, 1), P(.1, 1)));
  EXPECT_FALSE(critic(P(0, 3), P(.1, 3)));
  EXPECT_FALSE(prepared->matches(motion::Snapshot::parse({}).get()));
}

TEST(FtcMotionCritic, DecisionExactYCachedParityMatchesRawAtCrossingsAndBoundaries)
{
  motion::Ring ring{{-4.5, .8}, {4.5, .8}, {4.5, 1.8}};
  constexpr int teeth = 513;
  for (int k = 0; k < teeth; ++k)
  {
    const double x = 4.5 - 9. * k / teeth, dx = 9. / teeth;
    ring.push_back({x - .2 * dx, 1.8});
    ring.push_back({x - .2 * dx, 2.8});
    ring.push_back({x - .8 * dx, 2.8});
    ring.push_back({x - .8 * dx, 1.8});
  }
  ring.push_back({-4.5, 1.8});
  motion::Geometry raw;
  raw.allowed = {ring};
  mn::MotionRegionIndex index(raw.allowed);
  auto query = index.newQuery();
  for (double y : {1.8 - 1e-8, 1.8, 1.8 + 1e-8, 2., 2.8 - 1e-8, 2.8, 2.8 + 1e-8})
    for (const auto& vertex : ring)
      for (double dx : {-1e-8, 0., 1e-8})
      {
        const motion::Point p{vertex.x + dx, y};
        EXPECT_EQ(query->authorized(p), raw.authorized(p));
      }
  std::mt19937_64 random(27);
  std::uniform_real_distribution<double> x(-4.5, 4.5);
  for (int i = 0; i < 150; ++i)
  {
    const motion::Point a{x(random), 2}, b{x(random), 2};
    EXPECT_EQ(query->segmentAuthorized(a, b), raw.segmentAuthorized(a, b));
  }
  // A second decision has independent preparation, including its exact Y key.
  EXPECT_EQ(index.newQuery()->authorized({0, 2}), query->authorized({0, 2}));
}

TEST(FtcMotionCritic, ForwardSweepUsesClosedCellSquaresAndMeasuredCoast)
{
  nav2_costmap_2d::Costmap2D grid(200, 200, .05, -5, -5, 0);
  geometry_msgs::msg::Twist command, measured;
  command.linear.x = .2;
  measured.linear.x = .2;
  unsigned x, y;
  ASSERT_TRUE(grid.worldToMap(.275, 0, x, y));
  grid.setCost(x, y, 254);
  EXPECT_FALSE(mn::ForwardSweepBlocked(grid, Body(), {0, 0, 0}, command, measured, .1, 0));
  EXPECT_TRUE(mn::ForwardSweepBlocked(grid, Body(), {0, 0, 0}, command, measured, .1, 1));
  command.linear.x = .02;
  measured.linear.x = .3;
  EXPECT_TRUE(mn::ForwardSweepBlocked(grid, Body(), {0, 0, 0}, command, measured, .1, .5));
  command = geometry_msgs::msg::Twist{};
  measured.linear.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(mn::ForwardSweepBlocked(grid, {}, {NAN, NAN, NAN}, command, measured, NAN, NAN));
}

TEST(FtcMotionCritic, DecisionCacheBudgetFallbackRetainsSlantedRawPermission)
{
  motion::Ring ring{{-4.5, .8}, {4.5, .8}, {4.5, 1.8}};
  constexpr int teeth = 513;
  for (int k = 0; k < teeth; ++k)
  {
    const double x = 4.5 - 9. * k / teeth, dx = 9. / teeth;
    ring.push_back({x - .2 * dx, 1.8});
    ring.push_back({x - .2 * dx, 2.8});
    ring.push_back({x - .8 * dx, 2.8});
    ring.push_back({x - .8 * dx, 1.8});
  }
  ring.push_back({-4.5, 1.8});
  for (auto& p : ring)
    p.x += .13 * p.y;  // Genuine non-axis parity crossings, no simplification.
  motion::Geometry raw;
  raw.allowed = {ring};
  mn::MotionRegionIndex index(raw.allowed);
  auto query = index.newQuery();
  // 1100 exact distinct Ys x 1026 crossings exceed the 8MiB session budget;
  // both cached and original-predicate fallback must preserve permission.
  for (int i = 0; i < 1100; ++i)
  {
    const motion::Point p{-.4 + .001 * i, 2 + .0001 * i};
    EXPECT_EQ(query->authorized(p), raw.authorized(p));
  }
}

TEST(FtcMotionCritic, ForwardSweepRotationTailAndOutOfGridFailClosed)
{
  nav2_costmap_2d::Costmap2D grid(200, 200, .05, -5, -5, 0);
  geometry_msgs::msg::Twist command, measured;
  command.linear.x = .02;
  measured.angular.z = .8;
  unsigned x, y;
  ASSERT_TRUE(grid.worldToMap(.4, .35, x, y));
  grid.setCost(x, y, 254);
  motion::Ring body{{-.17, -.275}, {.53, -.275}, {.53, .275}, {-.17, .275}};
  EXPECT_TRUE(mn::ForwardSweepBlocked(grid, body, {0, 0, 0}, command, measured, .1, .5));
  EXPECT_TRUE(mn::ForwardSweepBlocked(grid, body, {4.8, 0, 0}, command, measured, .1, .5));
  grid.setCost(x, y, 253);
  EXPECT_FALSE(mn::ForwardSweepBlocked(grid, body, {0, 0, 0}, command, measured, .1, .5));
}
}  // namespace
