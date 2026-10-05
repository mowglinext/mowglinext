// Copyright (C) 2026 MowgliNext contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <memory>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_smac_planner/smac_planner_2d.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "mowgli_nav2_plugins/garden_keepout_layer.hpp"
#include <gtest/gtest.h>

namespace
{
using Mask = nav_msgs::msg::OccupancyGrid;
using Pose = geometry_msgs::msg::PoseStamped;
Pose pose(double x, double y)
{
  Pose p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.w = 1;
  return p;
}

std::shared_ptr<Mask> mask(double ox, double oy, unsigned w, unsigned h)
{
  auto m = std::make_shared<Mask>();
  m->header.frame_id = "map";
  m->info.resolution = 0.05;
  m->info.width = w;
  m->info.height = h;
  m->info.origin.position.x = ox;
  m->info.origin.position.y = oy;
  m->info.origin.orientation.w = 1;
  m->data.assign(size_t{w} * h, 100);
  return m;
}
void carve(Mask& m, double x0, double y0, double x1, double y1, int8_t cost = 0)
{
  for (unsigned y = 0; y < m.info.height; ++y)
    for (unsigned x = 0; x < m.info.width; ++x)
    {
      const double wx = m.info.origin.position.x + (x + .5) * m.info.resolution;
      const double wy = m.info.origin.position.y + (y + .5) * m.info.resolution;
      if (wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1)
        m.data[size_t{y} * m.info.width + x] = cost;
    }
}

class GardenCostmapTest : public ::testing::TestWithParam<bool>
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
  void SetUp() override
  {
    auto options = rclcpp::NodeOptions().parameter_overrides(std::vector<rclcpp::Parameter>{
        {"plugins", std::vector<std::string>{"sensor", "inflation", "keepout"}},
        {"sensor.plugin", "mowgli_nav2_plugins::RecentObstacleLayer"},
        {"sensor.observation_sources", ""},
        {"sensor.footprint_clearing_enabled", false},
        {"inflation.plugin", "nav2_costmap_2d::InflationLayer"},
        {"inflation.inflation_radius", 0.68},
        {"keepout.plugin", "mowgli_nav2_plugins::GardenKeepoutLayer"},
        {"global_frame", "map"},
        {"robot_base_frame", "base_footprint"},
        {"rolling_window", false},
        {"resolution", .08},
        {"width", 1},
        {"height", 1},
        {"track_unknown_space", false}});
    options.arguments({"--ros-args", "-r", "__ns:=/garden_test", "-r", "__node:=garden_test"});
    costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    ASSERT_EQ(costmap->on_configure(rclcpp_lifecycle::State()), nav2::CallbackReturn::SUCCESS);
    auto* plugins = costmap->getLayeredCostmap()->getPlugins();
    obstacles = std::dynamic_pointer_cast<mowgli_nav2_plugins::RecentObstacleLayer>((*plugins)[0]);
    keepout = std::dynamic_pointer_cast<mowgli_nav2_plugins::GardenKeepoutLayer>((*plugins)[2]);
    ASSERT_TRUE(obstacles && keepout);
    planner_node = std::make_shared<nav2::LifecycleNode>("garden_planner");
    planner_node->declare_parameter("GridBased.smoother.max_its", 0);
    planner_node->declare_parameter("GridBased.max_planning_time", 5.0);
    planner.configure(planner_node, "GridBased", costmap->getTfBuffer(), costmap);
  }
  void TearDown() override
  {
    planner.cleanup();
    keepout.reset();
    obstacles.reset();
    costmap->on_cleanup(rclcpp_lifecycle::State());
    costmap.reset();
    planner_node.reset();
  }
  void apply(const std::shared_ptr<Mask>& m)
  {
    keepout->receiveMask(m);
    costmap->getLayeredCostmap()->updateMap(0, 0, 0);
    costmap->getLayeredCostmap()->updateMap(0, 0, 0);
  }
  unsigned char at(double x, double y)
  {
    unsigned mx, my;
    auto* m = costmap->getCostmap();
    EXPECT_TRUE(m->worldToMap(x, y, mx, my));
    return m->getCost(mx, my);
  }
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<mowgli_nav2_plugins::GardenKeepoutLayer> keepout;
  std::shared_ptr<mowgli_nav2_plugins::RecentObstacleLayer> obstacles;
  nav2::LifecycleNode::SharedPtr planner_node;
  nav2_smac_planner::SmacPlanner2D planner;
};

TEST_P(GardenCostmapTest, DistantGoalAndDetourAvailableFromBothStarts)
{
  auto m = mask(-5, -5, 1700, 1000);  // x to 80 m, y to 45 m
  carve(*m, -2, -2, 2, 2);
  carve(*m, 70, -2, 74, 2);
  carve(*m, 0, 0, 1, 40);
  carve(*m, 0, 39, 72, 40);
  carve(*m, 71, 0, 72, 40);
  apply(m);
  EXPECT_DOUBLE_EQ(costmap->getCostmap()->getResolution(), .08);
  const double origin = costmap->getCostmap()->getOriginX();
  for (const auto& ends : std::vector<std::pair<Pose, Pose>>{{pose(.5, 0), pose(71.5, 0)},
                                                             {pose(71.5, 0), pose(.5, 0)}})
  {
    const auto path = planner.createPlan(ends.first,
                                         ends.second,
                                         {},
                                         []
                                         {
                                           return false;
                                         });
    ASSERT_FALSE(path.poses.empty());
    EXPECT_TRUE(std::any_of(path.poses.begin(),
                            path.poses.end(),
                            [](const auto& p)
                            {
                              return p.pose.position.y > 35;
                            }));
  }
  costmap->getLayeredCostmap()->updateMap(72, 0, 0);
  EXPECT_DOUBLE_EQ(costmap->getCostmap()->getOriginX(), origin);
}

TEST_P(GardenCostmapTest, NarrowConnectionAndSoftBoundaryCostsSurvive)
{
  auto m = mask(-5, -5, 1000, 200);
  carve(*m, -2, -2, 2, 2);
  carve(*m, 40, -2, 42, 2);
  carve(*m, 1, -.25, 41, .25);
  carve(*m, -2, 1, 2, 2, 50);
  apply(m);
  EXPECT_EQ(at(0, 1.5), 127);
  EXPECT_EQ(at(20, 1), nav2_costmap_2d::LETHAL_OBSTACLE);
  ASSERT_FALSE(planner
                   .createPlan(pose(0, 0),
                               pose(41, 0),
                               {},
                               []
                               {
                                 return false;
                               })
                   .poses.empty());
}

TEST_P(GardenCostmapTest, StartupResizeAndEditStayClosedUntilApplied)
{
  EXPECT_FALSE(keepout->isCurrent());
  EXPECT_EQ(at(.2, .2), nav2_costmap_2d::LETHAL_OBSTACLE);
  auto m = mask(-5, -5, 400, 200);
  carve(*m, -2, -2, 12, 2);
  keepout->receiveMask(m);
  costmap->getLayeredCostmap()->updateMap(0, 0, 0);
  EXPECT_FALSE(keepout->isCurrent());
  EXPECT_EQ(at(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap->getLayeredCostmap()->updateMap(0, 0, 0);
  EXPECT_TRUE(keepout->isCurrent());
  EXPECT_EQ(at(0, 0), 0);
  keepout->receiveMask(std::make_shared<Mask>());  // map edit invalidation
  EXPECT_EQ(at(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap->getLayeredCostmap()->updateMap(0, 0, 0);
  EXPECT_FALSE(keepout->isCurrent());
  auto edit = mask(-5, -5, 400, 200);  // identical geometry, changed boundary
  carve(*edit, 5, -2, 12, 2);
  apply(edit);
  EXPECT_EQ(at(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(at(6, 0), 0);
  // Recovery clearing cannot erase authorization.
  keepout->reset();
  EXPECT_EQ(at(6, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap->getLayeredCostmap()->updateMap(0, 0, 0);
  EXPECT_EQ(at(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_P(GardenCostmapTest, InvalidAndOversizedMasksFailClosedWithoutAllocation)
{
  auto m = mask(-5, -5, 400, 200);
  carve(*m, -2, -2, 12, 2);
  apply(m);
  const unsigned width = costmap->getCostmap()->getSizeInCellsX();
  auto excessive = mask(-5, -5, 1, 1);
  excessive->info.resolution = 1000;  // > budget without large input allocation
  apply(excessive);
  EXPECT_EQ(costmap->getCostmap()->getSizeInCellsX(), width);
  EXPECT_FALSE(keepout->isCurrent());
  EXPECT_EQ(at(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  auto invalid = std::make_shared<Mask>(*m);
  invalid->data[0] = -1;
  apply(invalid);
  EXPECT_FALSE(keepout->isCurrent());
  apply(m);
  EXPECT_TRUE(keepout->isCurrent());
}

TEST_P(GardenCostmapTest, ExplicitRecoveryTogglePreservesGeometryAndReenablesBoundary)
{
  EXPECT_TRUE(
      costmap->get_service_names_and_types().contains("/garden_test/keepout/toggle_filter"));
  EXPECT_FALSE(keepout->setFilterEnabled(false));
  auto m = mask(-5, -5, 400, 200);
  carve(*m, -2, -2, 12, 2);
  apply(m);
  ASSERT_TRUE(keepout->setFilterEnabled(false));
  costmap->getLayeredCostmap()->updateMap(0, 0, 0);
  EXPECT_EQ(at(-3, 0), 0);  // explicit ingress starts beyond the body-slack band
  ASSERT_FALSE(planner
                   .createPlan(pose(-3, 0),
                               pose(0, 0),
                               {},
                               []
                               {
                                 return false;
                               })
                   .poses.empty());
  EXPECT_TRUE(keepout->setFilterEnabled(true));
  EXPECT_EQ(at(-3, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap->getLayeredCostmap()->updateMap(0, 0, 0);
  EXPECT_EQ(at(-3, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(keepout->setFilterEnabled(false));
  keepout->receiveMask(std::make_shared<Mask>());
  costmap->getLayeredCostmap()->updateMap(0, 0, 0);
  EXPECT_FALSE(keepout->setFilterEnabled(false));
  EXPECT_FALSE(keepout->isCurrent());
  EXPECT_EQ(at(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  apply(m);
  EXPECT_EQ(at(-3, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_P(GardenCostmapTest, RemovedSensorMarksAndTheirInflationDoNotPersist)
{
  auto m = mask(-5, -5, 400, 200);
  carve(*m, -2, -2, 12, 2);
  // A drawn obstacle is independent of either sensor source.
  carve(*m, 9, -.3, 9.5, .3, 100);
  apply(m);
  sensor_msgs::msg::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  *x = 2;
  *y = 0;
  *z = .3;
  geometry_msgs::msg::Point origin;
  nav2_costmap_2d::Observation obs(origin, cloud, 3, 0, 8, 0);
  // LiDAR has clearing rays; the no-LiDAR fleet cloud is marking-only.
  obstacles->addStaticObservation(std::move(obs), true, GetParam());
  costmap->getLayeredCostmap()->updateMap(0, 0, 0);
  EXPECT_EQ(at(2, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(at(2.3, 0), 0);
  obstacles->clearStaticObservations(true, true);
  costmap->getLayeredCostmap()->updateMap(0, 0, 0);
  EXPECT_EQ(at(2, 0), 0);
  EXPECT_EQ(at(2.3, 0), 0);
  EXPECT_EQ(at(9.2, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

INSTANTIATE_TEST_SUITE_P(LidarAndNoLidar, GardenCostmapTest, ::testing::Bool());
}  // namespace
