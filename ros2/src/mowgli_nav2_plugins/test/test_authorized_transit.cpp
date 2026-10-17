// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "mowgli_nav2_plugins/authorized_transit_planner.hpp"
#include "mowgli_nav2_plugins/transit_raster.hpp"
#include <gtest/gtest.h>

namespace mowgli_nav2_plugins
{
using transit::Geometry;
using transit::Point;
using transit::Ring;
Ring rectangle(double x0, double y0, double x1, double y1)
{
  return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}
TEST(TransitGeometry, ClosedUnionAcceptsSeamsButRejectsThinGapsAndDiagonalCuts)
{
  Geometry geometry;
  geometry.allowed = {rectangle(-1, -1, 0, 1), rectangle(0, -1, 1, 1)};
  EXPECT_TRUE(geometry.segmentAuthorized({-0.8, 0}, {0.8, 0}));
  EXPECT_TRUE(geometry.segmentAuthorized({-1, -1}, {1, -1}));
  geometry.allowed[1] = rectangle(0.000001, -1, 1, 1);
  EXPECT_FALSE(geometry.segmentAuthorized({-0.8, 0}, {0.8, 0}));
  geometry.allowed = {rectangle(0, 0, 1, 0.4), rectangle(0.6, 0, 1, 1)};
  EXPECT_FALSE(geometry.segmentAuthorized({0.5, 0.35}, {0.65, 0.5}));
}
TEST(TransitGeometry, TerminalExceptionsAreBoundedAndTiedToOneArea)
{
  Geometry geometry;
  geometry.allowed = {rectangle(-4, 0, -0.2, 10), rectangle(0.2, 0, 4, 10)};
  EXPECT_TRUE(geometry.terminal({-0.1, 1}, {-0.25, 1}, 0.15));
  EXPECT_FALSE(geometry.terminal({0, 1}, {-0.25, 1}, 0.15));
  EXPECT_FALSE(geometry.terminal({-0.1, 1}, {0.25, 1}, 0.15));
  EXPECT_FALSE(geometry.terminal({-0.1, -0.3}, {-0.25, 0.1}, 0.15));
}
TEST(TransitGeometry, SharedNearestBoundaryMatchesTheSinglePolygonUnion)
{
  Geometry single;
  single.allowed = {rectangle(-1, 0, 1, 1)};
  for (const auto& rings :
       std::vector<std::vector<Ring>>{{rectangle(-1, 0, 0, 1), rectangle(0, 0, 1, 1)},
                                      {rectangle(-1, 0, 0.2, 1), rectangle(-0.2, 0, 1, 1)}})
  {
    Geometry geometry;
    geometry.allowed = rings;
    for (const Point anchor : {Point{-0.04, 0.96}, Point{0, 0.96}, Point{0.04, 0.96}})
    {
      EXPECT_TRUE(single.terminal({0, 1.1}, anchor, 0.15));
      EXPECT_TRUE(geometry.terminal({0, 1.1}, anchor, 0.15));
      EXPECT_FALSE(geometry.terminal({0, 1.16}, anchor, 0.15));
    }
  }
}
TEST(TransitGeometry, DistinctNearestBoundariesRemainAmbiguousEvenWhenJoinedElsewhere)
{
  Geometry geometry;
  geometry.allowed = {rectangle(-1, 0, -0.1, 1), rectangle(0.1, 0, 1, 1)};
  for (bool connected : {false, true})
  {
    if (connected)
      geometry.allowed.push_back(rectangle(-1, 0, 1, 0.2));
    EXPECT_FALSE(geometry.terminal({0, 0.8}, {-0.14, 0.8}, 0.15));
    EXPECT_FALSE(geometry.terminal({0, 0.8}, {0.14, 0.8}, 0.15));
  }
  // The same U-shaped union represented by one ring has the same ambiguity.
  geometry.allowed = {
      {{-1, 0}, {1, 0}, {1, 1}, {0.1, 1}, {0.1, 0.2}, {-0.1, 0.2}, {-0.1, 1}, {-1, 1}}};
  EXPECT_FALSE(geometry.terminal({0, 0.8}, {-0.14, 0.8}, 0.15));
  EXPECT_FALSE(geometry.terminal({0, 0.8}, {0.14, 0.8}, 0.15));
}
TEST(TransitGeometry, ObstaclesRejectWholeSegmentsIncludingMargin)
{
  Geometry geometry;
  geometry.blocked = {rectangle(-0.001, -1, 0.001, 1)};
  geometry.obstacle_margin = 0.1;
  EXPECT_FALSE(geometry.segmentClear({-2, 0}, {2, 0}));
  EXPECT_FALSE(geometry.segmentClear({-2, 1.05}, {2, 1.05}));
  EXPECT_TRUE(geometry.segmentClear({-2, 1.2}, {2, 1.2}));
}

TEST(TransitGeometry, RasterMatchesOriginalCellClassifier)
{
  for (double shift : {0.0, 1048576.07, -1048576.07})
    for (double resolution : {0.05, 0.08})
      for (double seam : {-0.000001, 0.0, 0.000001})
      {
        const transit::RasterGrid grid{80, 64, resolution, shift - 1.03, shift - 1.03};
        Geometry geometry;
        geometry.allowed = {rectangle(shift - 0.8, shift - 0.8, shift + 0.4, shift + 2),
                            rectangle(shift + 0.4 + seam, shift - 0.8, shift + 3, shift + 0.4),
                            {{shift + 0.4, shift + 0.4},
                             {shift + 3, shift + 0.4},
                             {shift + 3, shift + 2.5},
                             {shift + 2.5, shift + 2.5},
                             {shift + 2.5, shift + 0.8},
                             {shift + 0.4, shift + 0.8}}};
        geometry.blocked = {rectangle(shift, shift, shift + 0.2, shift + 0.2),
                            {{shift + 1, shift + 0.5},
                             {shift + 1.4, shift + 1},
                             {shift + 0.6, shift + 1}}};
        geometry.blocked_margins = {0, 0.13};
        const auto mask = transit::rasterAuthorization(geometry, grid, [] {});
        ASSERT_EQ(mask.size(), grid.width * grid.height);
        for (unsigned y = 0; y < grid.height; ++y)
          for (unsigned x = 0; x < grid.width; ++x)
          {
            const auto p = grid.centre(x, y);
            ASSERT_EQ(bool(mask[y * grid.width + x]), geometry.authorized(p) && geometry.clear(p))
                << "shift=" << shift << " resolution=" << resolution << " seam=" << seam
                << " cell=" << x << "," << y;
          }
      }
}

TEST(TransitGeometry, RasterPreservesEpsilonBoundaryAndObstacleMargin)
{
  const transit::RasterGrid grid{10, 10, 0.08, 0, 0};
  for (double offset :
       {-2 * transit::kEpsilon, -transit::kEpsilon, 0.0, transit::kEpsilon, 2 * transit::kEpsilon})
  {
    Geometry geometry;
    geometry.allowed = {rectangle(0.04 + offset, 0.04 + offset, 0.68 + offset, 0.68 + offset)};
    geometry.blocked = {rectangle(0.4 + offset, 0.4 + offset, 0.6, 0.6)};
    geometry.blocked_margins = {0.12};
    const auto mask = transit::rasterAuthorization(geometry, grid, [] {});
    for (unsigned y = 0; y < grid.height; ++y)
      for (unsigned x = 0; x < grid.width; ++x)
      {
        const auto p = grid.centre(x, y);
        EXPECT_EQ(bool(mask[y * grid.width + x]), geometry.authorized(p) && geometry.clear(p));
      }
  }
}

TEST(TransitGeometry, RasterMatchesCollapsedCellCentresAtExtremeOrigin)
{
  for (double shift : {4503599627370496.0, -4503599627370496.0})
  {
    const transit::RasterGrid grid{80, 80, 0.08, shift, shift};
    Geometry geometry;
    geometry.allowed = {rectangle(shift, shift, shift + 8, shift + 8)};
    geometry.blocked = {rectangle(shift + 2, shift + 2, shift + 4, shift + 4)};
    const auto mask = transit::rasterAuthorization(geometry, grid, [] {});
    for (unsigned y = 0; y < grid.height; ++y)
      for (unsigned x = 0; x < grid.width; ++x)
      {
        const auto p = grid.centre(x, y);
        EXPECT_EQ(bool(mask[y * grid.width + x]), geometry.authorized(p) && geometry.clear(p));
      }
  }
}

TEST(TransitGeometry, ReusedCrossingBuffersDoNotRetainPreviousEdges)
{
  Geometry geometry;
  geometry.allowed = {rectangle(-1, -1, 0, 1), rectangle(0.000001, -1, 1, 1)};
  geometry.blocked = {rectangle(-0.01, -0.2, 0.01, 0.2)};
  geometry.blocked_margins = {0.1};
  std::vector<double> scratch{0.25, 0.5, 0.75};
  for (unsigned iteration = 0; iteration < 10; ++iteration)
  {
    EXPECT_FALSE(geometry.segmentAuthorized({-0.8, 0.8}, {0.8, 0.8}, scratch));
    EXPECT_TRUE(geometry.segmentAuthorized({-0.8, 0.8}, {-0.2, 0.8}, scratch));
    EXPECT_FALSE(geometry.segmentClear({-0.8, 0}, {0.8, 0}, scratch));
    EXPECT_TRUE(geometry.segmentClear({-0.8, 0.8}, {0.8, 0.8}, scratch));
  }
}

TEST(TransitGeometry, RemoteObstacleRejectionMatchesOriginalSegmentClearance)
{
  const auto original_clear = [](const Geometry& geometry, Point a, Point b)
  {
    if (!geometry.clear(a) || !geometry.clear(b))
      return false;
    for (std::size_t index = 0; index < geometry.blocked.size(); ++index)
    {
      const auto& ring = geometry.blocked[index];
      if (transit::cuts(a, b, {ring}).size() > 2)
        return false;
      for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++)
        if (std::min({transit::edgeDistance(a, ring[j], ring[i]),
                      transit::edgeDistance(b, ring[j], ring[i]),
                      transit::edgeDistance(ring[j], a, b),
                      transit::edgeDistance(ring[i], a, b)}) <=
            geometry.margin(index) + transit::kEpsilon)
          return false;
    }
    return true;
  };
  for (double shift : {0.0, 1048576.07, 4503599627370496.0})
  {
    Geometry geometry;
    for (unsigned i = 0; i < 16; ++i)
    {
      geometry.blocked.push_back(rectangle(
          shift + i * 0.3, shift + i * 0.13, shift + i * 0.3 + 0.17, shift + i * 0.13 + 0.21));
      geometry.blocked_margins.push_back((i % 3) * 0.05);
    }
    for (unsigned i = 0; i < 200; ++i)
    {
      const Point a{shift - 1 + (i % 19) * 0.4, shift - 1 + (i % 13) * 0.3};
      const Point b{shift - 1 + (i % 17) * 0.35, shift - 1 + (i % 11) * 0.4};
      EXPECT_EQ(geometry.segmentClear(a, b), original_clear(geometry, a, b));
      EXPECT_EQ(geometry.segmentClear(b, a), original_clear(geometry, b, a));
    }
  }
  Geometry near_parallel;
  near_parallel.blocked = {rectangle(0.02, 0.0000001, 0.06, 0.1)};
  // Preserve even the original near-collinear cut tolerance, not just metric
  // distance: a naive bounding rejection before cuts would accept this edge.
  EXPECT_FALSE(near_parallel.segmentClear({0, 0}, {0.08, 0}));
  EXPECT_FALSE(near_parallel.segmentClear({0.08, 0}, {0, 0}));
}

TEST(TransitGeometry, BoundsConnectivityIsOnlyANegativeCheck)
{
  Geometry geometry;
  geometry.allowed = {rectangle(-4, 0, -0.2, 10), rectangle(0.2, 0, 4, 10)};
  EXPECT_FALSE(geometry.boundsMayConnect({-1, 1}, {1, 1}, [] {}));
  geometry.allowed.push_back(rectangle(-1, 8, 1, 9));
  EXPECT_TRUE(geometry.boundsMayConnect({-1, 1}, {1, 1}, [] {}));
  EXPECT_TRUE(geometry.boundsMayConnect({1, 1}, {-1, 1}, [] {}));
  geometry.allowed = {rectangle(-1, 0, 0, 1), rectangle(0, 0, 1, 1)};
  EXPECT_TRUE(geometry.boundsMayConnect({-0.5, 0.5}, {0.5, 0.5}, [] {}));
  geometry.allowed[1] = rectangle(1.5 * transit::kEpsilon, 0, 1, 1);
  EXPECT_TRUE(geometry.segmentAuthorized({-0.5, 0.5}, {0.5, 0.5}));
  EXPECT_TRUE(geometry.boundsMayConnect({-0.5, 0.5}, {0.5, 0.5}, [] {}));
  // Disconnected triangles with overlapping boxes must go through exact A*.
  geometry.allowed = {{{0, 0}, {2, 0}, {0, 2}}, {{2, 2}, {0.9, 2}, {2, 0.9}}};
  EXPECT_TRUE(geometry.boundsMayConnect({0.2, 0.2}, {1.8, 1.8}, [] {}));
}

class AuthorizedTransitPlannerTest : public testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    node_ = std::make_shared<nav2::LifecycleNode>("authorized_transit_test");
    node_->declare_parameter("GridBased.tolerance", 0.0);
    node_->declare_parameter("GridBased.max_planning_time", 2.0);
    auto options = rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("plugins", std::vector<std::string>{}),
         rclcpp::Parameter("filters", std::vector<std::string>{}),
         rclcpp::Parameter("global_frame", "map"),
         rclcpp::Parameter("rolling_window", false)});
    options.arguments({"--ros-args", "-r", "__node:=transit_test_map"});
    map_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    ASSERT_EQ(map_->on_configure(rclcpp_lifecycle::State()), nav2::CallbackReturn::SUCCESS);
    map_->getCostmap()->resizeMap(160, 160, 0.08, -6.03, -1.03);
    std::fill_n(map_->getCostmap()->getCharMap(), 160 * 160, 127);
    planner_.configure(node_, "GridBased", nullptr, map_);
    planner_.activate();
  }
  void TearDown() override
  {
    planner_.deactivate();
    planner_.cleanup();
    map_->on_cleanup(rclcpp_lifecycle::State());
  }
  void geometry(const std::vector<Ring>& rings,
                const std::vector<Ring>& obstacles = {},
                double obstacle_margin = 0)
  {
    auto message = std::make_shared<visualization_msgs::msg::MarkerArray>();
    for (std::size_t i = 0; i < rings.size() + obstacles.size(); ++i)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.ns = i < rings.size() ? "area" : "obstacle";
      marker.scale.x = i < rings.size() ? 0 : obstacle_margin;
      marker.action = marker.ADD;
      for (const auto& p : i < rings.size() ? rings[i] : obstacles[i - rings.size()])
      {
        geometry_msgs::msg::Point point;
        point.x = p.x;
        point.y = p.y;
        marker.points.push_back(point);
      }
      message->markers.push_back(marker);
    }
    planner_.onGeometry(message);
  }
  geometry_msgs::msg::PoseStamped pose(double x, double y)
  {
    geometry_msgs::msg::PoseStamped result;
    result.header.frame_id = "map";
    result.pose.position.x = x;
    result.pose.position.y = y;
    result.pose.orientation.w = 1;
    return result;
  }
  nav_msgs::msg::Path plan(double x0, double y0, double x1, double y1)
  {
    return planner_.createPlan(pose(x0, y0),
                               pose(x1, y1),
                               {},
                               []
                               {
                                 return false;
                               });
  }
  bool authorized(const nav_msgs::msg::Path& path)
  {
    return planner_.pathValid(path, planner_.geometry_);
  }
  nav_msgs::msg::Path unrestrictedPlan()
  {
    nav2_smac_planner::SmacPlanner2D baseline;
    baseline.configure(node_, "Baseline", nullptr, map_);
    baseline.activate();
    auto path = baseline.createPlan(pose(-1.2, 1),
                                    pose(1.2, 1),
                                    {},
                                    []
                                    {
                                      return false;
                                    });
    baseline.deactivate();
    baseline.cleanup();
    return path;
  }
  std::shared_ptr<nav2::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  AuthorizedTransitPlanner planner_;
};

TEST_F(AuthorizedTransitPlannerTest, OverlappingSlackUsesAuthorizedDetourAfterSmoothing)
{
  geometry({rectangle(-4, 0, -0.2, 10), rectangle(0.2, 0, 4, 10), rectangle(-1, 8, 1, 9)});
  const auto before = unrestrictedPlan();
  EXPECT_LT(before.poses.size(), 40U);
  const auto path = plan(-1.2, 1, 1.2, 1);
  EXPECT_TRUE(authorized(path));
  double length = 0;
  for (std::size_t i = 1; i < path.poses.size(); ++i)
    length += std::hypot(path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
                         path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
  EXPECT_GT(length, 14);
  EXPECT_LT(length, 20);
  // The private grid must not make shared coverage/body slack lethal.
  EXPECT_EQ(map_->getCostmap()->getCost(75, 25), 127);
}
TEST_F(AuthorizedTransitPlannerTest, DisconnectedAreasCannotCrossSlackEvenBetweenOutsideTerminals)
{
  geometry({rectangle(-4, 0, -0.2, 10), rectangle(0.2, 0, 4, 10)});
  EXPECT_THROW(plan(-1.2, 1, 1.2, 1), nav2_core::NoValidPathCouldBeFound);
  EXPECT_THROW(plan(-0.1, 1, 0.1, 1), nav2_core::NoValidPathCouldBeFound);
}
TEST_F(AuthorizedTransitPlannerTest, BoundaryAndOutsideTerminalsRemainPlannableOnAdverseGrid)
{
  geometry({rectangle(-4, 0, 4, 10)});
  EXPECT_NO_THROW(plan(-4, 0, 4, 10));
  EXPECT_NO_THROW(plan(-4.1, 1, 4.1, 9));
  EXPECT_THROW(plan(-4.3, 1, 0, 9), nav2_core::NoValidPathCouldBeFound);
}
TEST_F(AuthorizedTransitPlannerTest, OutsideSharedBoundaryTerminalsRemainPlannableOnAdverseGrid)
{
  for (const auto& rings :
       std::vector<std::vector<Ring>>{{rectangle(-1, 0, 0, 1), rectangle(0, 0, 1, 1)},
                                      {rectangle(-1, 0, 0.2, 1), rectangle(-0.2, 0, 1, 1)}})
  {
    geometry(rings);
    for (bool outside_start : {true, false})
    {
      const auto path = outside_start ? plan(0, 1.1, 0.7, 0.5) : plan(0.7, 0.5, 0, 1.1);
      ASSERT_GT(path.poses.size(), 2U);
      const auto& anchor = path.poses[outside_start ? 1 : path.poses.size() - 2].pose.position;
      Geometry exact;
      exact.allowed = rings;
      EXPECT_TRUE(exact.terminal({0, 1.1}, {anchor.x, anchor.y}, 0.15));
      EXPECT_LE(transit::distance({0, 1.1}, {anchor.x, anchor.y}), 0.30);
      auto interior = path;
      interior.poses.erase(interior.poses.begin());
      interior.poses.pop_back();
      EXPECT_TRUE(authorized(interior));
    }
  }
}
TEST_F(AuthorizedTransitPlannerTest, EquidistantGapTerminalsRemainRejected)
{
  geometry({rectangle(-1, 0, -0.1, 1), rectangle(0.1, 0, 1, 1)});
  EXPECT_THROW(plan(0, 0.8, -0.7, 0.5), nav2_core::NoValidPathCouldBeFound);
  EXPECT_THROW(plan(0.7, 0.5, 0, 0.8), nav2_core::NoValidPathCouldBeFound);
  geometry({rectangle(-1, 0, -0.1, 1), rectangle(0.1, 0, 1, 1), rectangle(-1, 0, 1, 0.2)});
  EXPECT_THROW(plan(0, 0.8, -0.7, 0.5), nav2_core::NoValidPathCouldBeFound);
}
TEST_F(AuthorizedTransitPlannerTest, NarrowNavigationAndClosedSeamsRemainUsable)
{
  geometry({rectangle(-4, 0, -0.2, 4), rectangle(0.2, 0, 4, 4), rectangle(-0.2, 1.91, 0.2, 2.09)});
  EXPECT_TRUE(authorized(plan(-1, 2, 1, 2)));
}
TEST_F(AuthorizedTransitPlannerTest, DockCorridorAndObstacleCostsArePreserved)
{
  geometry({rectangle(-4, 0, 0, 4), rectangle(0, 1.5, 2, 2.5)}, {rectangle(-0.2, 1.7, 0.2, 2.3)});
  EXPECT_TRUE(authorized(plan(-1, 2, 1, 2)));
  for (unsigned y = 0; y < 160; ++y)
    for (unsigned x = 0; x < 160; ++x)
    {
      double wx, wy;
      map_->getCostmap()->mapToWorld(x, y, wx, wy);
      if (wx > -0.1 && wx < 0.1)
        map_->getCostmap()->setCost(x, y, 254);
    }
  EXPECT_THROW(plan(-1, 2, 1, 2), nav2_core::NoValidPathCouldBeFound);
}
TEST_F(AuthorizedTransitPlannerTest, MissingAndInvalidatedGeometryFailClosed)
{
  EXPECT_THROW(plan(-1, 1, 1, 1), nav2_core::NoValidPathCouldBeFound);
  geometry({rectangle(-4, 0, 4, 10)});
  EXPECT_NO_THROW(plan(-1, 1, 1, 1));
  geometry({});
  EXPECT_THROW(plan(-1, 1, 1, 1), nav2_core::NoValidPathCouldBeFound);
  geometry({rectangle(-4, 0, 4, 10)});
  EXPECT_NO_THROW(plan(-1, 1, 1, 1));
}
TEST_F(AuthorizedTransitPlannerTest, MapChangeDuringPlanningDiscardsResult)
{
  geometry({rectangle(-4, 0, 4, 10)});
  int checks = 0;
  EXPECT_THROW(planner_.createPlan(pose(-1, 1),
                                   pose(1, 1),
                                   {},
                                   [&]
                                   {
                                     if (++checks == 10)
                                     {
                                       geometry({});
                                     }
                                     return false;
                                   }),
               nav2_core::NoValidPathCouldBeFound);
}

TEST_F(AuthorizedTransitPlannerTest, CancellationAndTotalBudgetCoverPreprocessing)
{
  geometry({rectangle(-4, 0, 4, 10)});
  EXPECT_THROW(planner_.createPlan(pose(-1, 1),
                                   pose(1, 1),
                                   {},
                                   []
                                   {
                                     return true;
                                   }),
               nav2_core::PlannerCancelled);
  node_->set_parameter(rclcpp::Parameter("GridBased.max_planning_time", 0.000001));
  EXPECT_THROW(plan(-1, 1, 1, 1), nav2_core::PlannerTimedOut);
}

TEST_F(AuthorizedTransitPlannerTest, TranslatedGardenUsesExactSearchCellCentres)
{
  constexpr double shift = 1048576.07;
  map_->getCostmap()->resizeMap(160, 160, 0.08, shift - 6.03, shift - 1.03);
  std::fill_n(map_->getCostmap()->getCharMap(), 160 * 160, 127);
  geometry({rectangle(shift - 4, shift, shift - 0.2, shift + 10),
            rectangle(shift + 0.2, shift, shift + 4, shift + 10),
            rectangle(shift - 1, shift + 8, shift + 1, shift + 9)});
  EXPECT_TRUE(authorized(plan(shift - 1.2, shift + 1, shift + 1.2, shift + 1)));
}

TEST_F(AuthorizedTransitPlannerTest, RejectedParameterUpdatesDoNotChangeSearchGeometry)
{
  geometry({rectangle(-4, 0, 4, 10)});
  EXPECT_FALSE(node_->set_parameter(rclcpp::Parameter("GridBased.tolerance", 0.2)).successful);
  EXPECT_FALSE(
      node_->set_parameter(rclcpp::Parameter("GridBased.downsample_costmap", true)).successful);
  EXPECT_DOUBLE_EQ(node_->get_parameter("GridBased.tolerance").as_double(), 0.0);
  EXPECT_FALSE(node_->get_parameter("GridBased.downsample_costmap").as_bool());
  EXPECT_NO_THROW(plan(-4, 0, 4, 10));
  geometry({rectangle(-4, 0, -0.2, 10), rectangle(0.2, 0, 4, 10)});
  EXPECT_THROW(plan(-1.2, 1, 1.2, 1), nav2_core::NoValidPathCouldBeFound);
}

TEST_F(AuthorizedTransitPlannerTest, ConcaveCorridorRemainsAuthorizedInBothDirections)
{
  geometry({{{-3, 0}, {3, 0}, {3, 4}, {2.5, 4}, {2.5, 0.5}, {-3, 0.5}}});
  EXPECT_TRUE(authorized(plan(-2, 0.25, 2.75, 3.8)));
  EXPECT_TRUE(authorized(plan(2.75, 3.8, -2, 0.25)));
}

TEST_F(AuthorizedTransitPlannerTest, NewGeometryObstacleKeepsMarginBeforeRasterCatchesUp)
{
  geometry({rectangle(-4, 0, 4, 10)});
  EXPECT_TRUE(authorized(plan(-1, 5, 1, 5)));
  // The shared raster is deliberately unchanged when the new snapshot arrives.
  geometry({rectangle(-4, 0, 4, 10)}, {rectangle(-0.2, 4, 0.2, 6)}, 0.25);
  EXPECT_TRUE(authorized(plan(-1, 5, 1, 5)));
  EXPECT_TRUE(authorized(plan(1, 5, -1, 5)));
  EXPECT_THROW(plan(-0.4, 5, 1, 5), nav2_core::StartOccupied);
  EXPECT_THROW(plan(1, 5, -0.4, 5), nav2_core::GoalOccupied);
}

TEST_F(AuthorizedTransitPlannerTest, CurrentSensorCostsPreventDiagonalCornerCutting)
{
  map_->getCostmap()->resizeMap(20, 20, 0.08, 0, 0);
  std::fill_n(map_->getCostmap()->getCharMap(), 400, 0);
  geometry({rectangle(0, 0, 1.6, 1.6)});
  for (const unsigned char cost : {253, 254, 255})
  {
    map_->getCostmap()->setCost(10, 9, cost);
    for (bool reverse : {false, true})
    {
      const auto path = reverse ? plan(0.84, 0.84, 0.76, 0.76) : plan(0.76, 0.76, 0.84, 0.84);
      EXPECT_TRUE(authorized(path));
      double length = 0;
      for (std::size_t i = 1; i < path.poses.size(); ++i)
        length += transit::distance({path.poses[i - 1].pose.position.x,
                                     path.poses[i - 1].pose.position.y},
                                    {path.poses[i].pose.position.x, path.poses[i].pose.position.y});
      EXPECT_GT(length, 0.14);  // the direct diagonal would be only 0.113 m
    }
  }
  map_->getCostmap()->setCost(10, 9, 0);
  EXPECT_TRUE(authorized(plan(0.76, 0.76, 0.84, 0.84)));
}

TEST_F(AuthorizedTransitPlannerTest, DisconnectedBoundsFailBeforeGridWorkAndRecheckNewGeometry)
{
  for (bool reverse : {false, true})
  {
    geometry({rectangle(-4, 0, -0.2, 10), rectangle(0.2, 0, 4, 10)});
    unsigned checks = 0;
    EXPECT_THROW(planner_.createPlan(pose(reverse ? 1.2 : -1.2, 1),
                                     pose(reverse ? -1.2 : 1.2, 1),
                                     {},
                                     [&]
                                     {
                                       return ++checks > 20;
                                     }),
                 nav2_core::NoValidPathCouldBeFound);
    EXPECT_LT(checks, 20U);
    geometry({rectangle(-4, 0, -0.2, 10), rectangle(0.2, 0, 4, 10), rectangle(-1, 8, 1, 9)});
    EXPECT_TRUE(authorized(reverse ? plan(1.2, 1, -1.2, 1) : plan(-1.2, 1, 1.2, 1)));
  }
  geometry({{{0, 0}, {2, 0}, {0, 2}}, {{2, 2}, {0.9, 2}, {2, 0.9}}});
  EXPECT_THROW(plan(0.2, 0.2, 1.8, 1.8), nav2_core::NoValidPathCouldBeFound);
  EXPECT_THROW(plan(1.8, 1.8, 0.2, 0.2), nav2_core::NoValidPathCouldBeFound);
}

TEST_F(AuthorizedTransitPlannerTest, BothRingBoundaryTolerancesRemainCrossable)
{
  map_->getCostmap()->resizeMap(30, 20, 0.08, 0, 0);
  std::fill_n(map_->getCostmap()->getCharMap(), 600, 0);
  geometry({rectangle(0, 0, 1, 1), rectangle(1 + 1.5 * transit::kEpsilon, 0, 2, 1)});
  EXPECT_TRUE(authorized(plan(0.52, 0.52, 1.48, 0.52)));
  EXPECT_TRUE(authorized(plan(1.48, 0.52, 0.52, 0.52)));
}

// Opt-in real-planner benchmark, excluded from ordinary test runs. Run with
// --gtest_also_run_disabled_tests --gtest_filter=*RepresentativePlanningBenchmark.
// Each case uses the same geometry, costs and endpoints in both directions.
TEST_F(AuthorizedTransitPlannerTest, DISABLED_RepresentativePlanningBenchmark)
{
  ASSERT_EQ(rcutils_logging_set_logger_level(node_->get_logger().get_name(),
                                             RCUTILS_LOG_SEVERITY_DEBUG),
            RCUTILS_RET_OK);
  node_->set_parameter(rclcpp::Parameter("GridBased.max_planning_time", 60.0));
  node_->set_parameter(rclcpp::Parameter("GridBased.cost_travel_multiplier", 3.0));
  node_->set_parameter(rclcpp::Parameter("GridBased.allow_unknown", false));
  struct Scenario
  {
    const char* name;
    unsigned width, height;
    std::vector<Ring> areas, obstacles;
    Point start, goal;
    bool connected{true};
  };
  Ring detailed;
  for (unsigned i = 0; i < 750; ++i)
  {
    const double angle = i * 2 * std::acos(-1.0) / 750;
    const double radius = 35 + 2 * std::sin(7 * angle);
    detailed.push_back({45 + radius * std::cos(angle), 45 + radius * std::sin(angle)});
  }
  const auto lawns = std::vector<Ring>{rectangle(0, 0, 45, 75), rectangle(55, 0, 100, 75)};
  auto detour = lawns;
  detour.push_back(rectangle(40, 70, 60, 75));
  std::vector<Ring> obstacles;
  for (unsigned y = 0; y < 4; ++y)
    for (unsigned x = 0; x < 8; ++x)
      obstacles.push_back(rectangle(6 + x * 4, 5 + y * 9, 7 + x * 4, 8 + y * 9));
  const std::vector<Scenario> scenarios{
      {"small", 160, 160, {rectangle(0, 0, 10, 10)}, {}, {1, 1}, {9, 9}},
      {"medium", 625, 625, {rectangle(0, 0, 45, 45)}, {}, {1, 1}, {44, 44}},
      {"large", 1250, 1250, {rectangle(0, 0, 95, 95)}, {}, {1, 1}, {94, 94}},
      {"vertices750", 1250, 1250, {detailed}, {}, {20, 45}, {70, 45}},
      {"detour", 1375, 1000, detour, {}, {40, 1}, {60, 1}},
      {"disconnected", 1375, 1000, lawns, {}, {40, 1}, {60, 1}, false},
      {"obstacles32", 625, 625, {rectangle(0, 0, 45, 45)}, obstacles, {1, 1}, {44, 44}}};
  for (const auto& scenario : scenarios)
  {
    if (const char* filter = std::getenv("MOWGLI_TRANSIT_BENCH_CASE");
        filter && std::strcmp(filter, scenario.name) != 0)
      continue;
    map_->getCostmap()->resizeMap(scenario.width, scenario.height, 0.08, -1.03, -1.03);
    std::fill_n(map_->getCostmap()->getCharMap(), scenario.width * scenario.height, 0);
    geometry(scenario.areas, scenario.obstacles);
    std::vector<double> times;
    for (unsigned iteration = 0; iteration < 30; ++iteration)
    {
      const auto a = iteration % 2 ? scenario.goal : scenario.start;
      const auto b = iteration % 2 ? scenario.start : scenario.goal;
      const auto begin = std::chrono::steady_clock::now();
      nav_msgs::msg::Path path;
      bool reached = false;
      try
      {
        path = plan(a.x, a.y, b.x, b.y);
        reached = true;
      }
      catch (const nav2_core::NoValidPathCouldBeFound&)
      {
      }
      times.push_back(
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin)
              .count());
      ASSERT_EQ(reached, scenario.connected) << scenario.name;
      if (reached)
      {
        ASSERT_TRUE(authorized(path)) << scenario.name;
      }
    }
    std::sort(times.begin(), times.end());
    std::cout << "TRANSIT_BENCH " << scenario.name << " cells=" << scenario.width * scenario.height
              << " median_ms=" << (times[14] + times[15]) / 2 << " p95_ms=" << times[28]
              << " max_ms=" << times.back() << std::endl;
  }
}
}  // namespace mowgli_nav2_plugins
