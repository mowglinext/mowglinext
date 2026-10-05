// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
#include "mowgli_nav2_plugins/authorized_transit_planner.hpp"
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
TEST(TransitGeometry, ObstaclesRejectWholeSegmentsIncludingMargin)
{
  Geometry geometry;
  geometry.blocked = {rectangle(-0.001, -1, 0.001, 1)};
  geometry.obstacle_margin = 0.1;
  EXPECT_FALSE(geometry.segmentClear({-2, 0}, {2, 0}));
  EXPECT_FALSE(geometry.segmentClear({-2, 1.05}, {2, 1.05}));
  EXPECT_TRUE(geometry.segmentClear({-2, 1.2}, {2, 1.2}));
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
  void geometry(const std::vector<Ring>& rings, const std::vector<Ring>& obstacles = {})
  {
    auto message = std::make_shared<visualization_msgs::msg::MarkerArray>();
    for (std::size_t i = 0; i < rings.size() + obstacles.size(); ++i)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.ns = i < rings.size() ? "area" : "obstacle";
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
}  // namespace mowgli_nav2_plugins
