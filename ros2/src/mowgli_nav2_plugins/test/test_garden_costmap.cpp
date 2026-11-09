// Copyright (C) 2026 MowgliNext contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>

#include <nav2_costmap_2d/clear_costmap_service.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_smac_planner/smac_planner_2d.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "costmap_test_support.hpp"
#include "mowgli_nav2_plugins/authorized_transit_planner.hpp"
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
        {"plugins", std::vector<std::string>{"sensor", "inflation"}},
        {"filters", std::vector<std::string>{"keepout"}},
        {"sensor.plugin", "mowgli_nav2_plugins::RecentObstacleLayer"},
        {"sensor.observation_sources", ""},
        {"sensor.footprint_clearing_enabled", false},
        {"inflation.plugin", "nav2_costmap_2d::InflationLayer"},
        {"inflation.inflation_radius", 0.68},
        {"inflation.cost_scaling_factor", 7.0},
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
    keepout = std::dynamic_pointer_cast<mowgli_nav2_plugins::GardenKeepoutLayer>(
        (*costmap->getLayeredCostmap()->getFilters())[0]);
    ASSERT_TRUE(obstacles && keepout);
    planner_node = std::make_shared<nav2::LifecycleNode>("garden_planner");
    planner_node->declare_parameter("GridBased.smoother.max_its", 0);
    planner_node->declare_parameter("GridBased.max_planning_time", 5.0);
    planner.configure(planner_node, "GridBased", costmap->getTfBuffer(), costmap);
    planner_node->declare_parameter("Authorized.tolerance", 0.0);
    planner_node->declare_parameter("Authorized.max_planning_time", 5.0);
    authorized.configure(planner_node, "Authorized", costmap->getTfBuffer(), costmap);
    geometry_node = std::make_shared<rclcpp::Node>("garden_geometry");
    geometry_pub = geometry_node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/map_server_node/transit_geometry", rclcpp::QoS(1).transient_local());
  }
  void TearDown() override
  {
    planner.cleanup();
    authorized.cleanup();
    geometry_pub.reset();
    geometry_node.reset();
    keepout.reset();
    obstacles.reset();
    mowgli_nav2_plugins_test::WaitForTfListenerSpinning(*costmap);
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
  void observation(double px, double py)
  {
    obstacles->clearStaticObservations(true, true);
    sensor_msgs::msg::PointCloud2 cloud;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(1);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    *x = px;
    *y = py;
    *z = .3;
    geometry_msgs::msg::Point origin;
    obstacles->addStaticObservation(nav2_costmap_2d::Observation(origin, cloud, 10, 0, 10, 0),
                                    true,
                                    GetParam());
  }
  unsigned updateAndCompareFullRebuild()
  {
    auto* layered = costmap->getLayeredCostmap();
    layered->updateMap(0, 0, 0);
    auto* grid = costmap->getCostmap();
    const size_t cells = size_t{grid->getSizeInCellsX()} * grid->getSizeInCellsY();
    const std::vector<unsigned char> bounded(grid->getCharMap(), grid->getCharMap() + cells);
    unsigned x0, xn, y0, yn;
    layered->getBounds(&x0, &xn, &y0, &yn);
    keepout->reset();
    layered->updateMap(0, 0, 0);
    EXPECT_EQ(bounded, std::vector<unsigned char>(grid->getCharMap(), grid->getCharMap() + cells));
    return (xn - x0) * (yn - y0);
  }
  unsigned char at(double x, double y)
  {
    unsigned mx, my;
    auto* m = costmap->getCostmap();
    EXPECT_TRUE(m->worldToMap(x, y, mx, my));
    return m->getCost(mx, my);
  }
  void geometry(const std::vector<mowgli_nav2_plugins::transit::Ring>& rings)
  {
    visualization_msgs::msg::MarkerArray snapshot;
    for (const auto& ring : rings)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.ns = "area";
      marker.action = marker.ADD;
      for (const auto& p : ring)
      {
        geometry_msgs::msg::Point point;
        point.x = p.x;
        point.y = p.y;
        marker.points.push_back(point);
      }
      snapshot.markers.push_back(marker);
    }
    for (int i = 0; i < 200 && geometry_pub->get_subscription_count() == 0; ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ASSERT_GT(geometry_pub->get_subscription_count(), 0U);
    geometry_pub->publish(snapshot);
    // This lifecycle node's only subscription is the planner's geometry input.
    // Deliver the real DDS snapshot, including an empty invalidation.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(planner_node->get_node_base_interface());
    executor.spin_once(std::chrono::seconds(2));
  }
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap;
  std::shared_ptr<mowgli_nav2_plugins::GardenKeepoutLayer> keepout;
  std::shared_ptr<mowgli_nav2_plugins::RecentObstacleLayer> obstacles;
  nav2::LifecycleNode::SharedPtr planner_node;
  nav2_smac_planner::SmacPlanner2D planner;
  mowgli_nav2_plugins::AuthorizedTransitPlanner authorized;
  rclcpp::Node::SharedPtr geometry_node;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr geometry_pub;
};

TEST_P(GardenCostmapTest, AuthorizedPlannerUsesDistantDetourAndRejectsDisconnectedSlack)
{
  using namespace mowgli_nav2_plugins::transit;
  const auto rectangle = [](double x0, double y0, double x1, double y1) -> Ring
  {
    return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
  };
  Geometry exact;
  exact.allowed = {rectangle(-2, -2, 2, 2),
                   rectangle(70, -2, 74, 2),
                   rectangle(0, 0, 1, 40),
                   rectangle(0, 39, 72, 40),
                   rectangle(71, 0, 72, 40)};
  auto m = mask(-5, -5, 1700, 1000);
  // Soft costs throughout reproduce the permissive body-slack substrate;
  // exact geometry must still force the long connected detour.
  std::fill(m->data.begin(), m->data.end(), 50);
  apply(m);
  geometry(exact.allowed);
  const auto same_cell = authorized.createPlan(pose(.51, .1),
                                               pose(.52, .1),
                                               {},
                                               []
                                               {
                                                 return false;
                                               });
  ASSERT_FALSE(same_cell.poses.empty());
  EXPECT_THROW(authorized.createPlan(pose(35, 0),
                                     pose(35.01, 0),
                                     {},
                                     []
                                     {
                                       return false;
                                     }),
               nav2_core::NoValidPathCouldBeFound);
  for (const auto& ends : std::vector<std::pair<Pose, Pose>>{{pose(.5, 0), pose(71.5, 0)},
                                                             {pose(71.5, 0), pose(.5, 0)}})
  {
    const auto path = authorized.createPlan(ends.first,
                                            ends.second,
                                            {},
                                            []
                                            {
                                              return false;
                                            });
    ASSERT_GT(path.poses.size(), 2U);
    EXPECT_TRUE(std::any_of(path.poses.begin(),
                            path.poses.end(),
                            [](const auto& p)
                            {
                              return p.pose.position.y > 35;
                            }));
    for (std::size_t i = 1; i < path.poses.size(); ++i)
    {
      const auto& a = path.poses[i - 1].pose.position;
      const auto& b = path.poses[i].pose.position;
      EXPECT_TRUE(exact.segmentAuthorized({a.x, a.y}, {b.x, b.y}));
    }
  }
  exact.allowed.resize(2);  // unchanged raster costs cannot authorize a shortcut
  geometry(exact.allowed);
  EXPECT_THROW(authorized.createPlan(pose(.5, 0),
                                     pose(71.5, 0),
                                     {},
                                     []
                                     {
                                       return false;
                                     }),
               nav2_core::NoValidPathCouldBeFound);
  geometry({});
  EXPECT_THROW(authorized.createPlan(pose(.5, 0),
                                     pose(.8, 0),
                                     {},
                                     []
                                     {
                                       return false;
                                     }),
               nav2_core::NoValidPathCouldBeFound);
}

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

TEST_P(GardenCostmapTest, MaskValuesKeepNav2CostQuantization)
{
  auto m = mask(0, 0, 101, 1);
  m->info.resolution = 0.08;
  for (int value = 0; value <= 100; ++value)
    m->data[value] = static_cast<int8_t>(value);
  apply(m);
  ASSERT_TRUE(keepout->isCurrent());
  for (int value = 0; value <= 100; ++value)
    EXPECT_EQ(costmap->getCostmap()->getCost(value, 0),
              static_cast<unsigned char>(std::lround(value * 254.0 / 100.0)))
        << "mask value " << value;
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

TEST_P(GardenCostmapTest, BoundedUpdatesPreserveMaskAndOldInflationExtent)
{
  auto m = mask(-10, -10, 400, 400);
  carve(*m, -9, -9, 9, 9);
  carve(*m, 2.5, -2, 3, 2, 100);
  apply(m);
  observation(1.5, 0);
  EXPECT_LT(updateAndCompareFullRebuild(), 10000U);
  // ceil(0.68 / 0.08) reaches nine cells, beyond continuous radius bounds.
  EXPECT_GT(at(2.2, 0), 0);
  EXPECT_EQ(at(2.4, 0), 0);  // the retained drawn wall must not be inflated
  observation(-5, 0);
  updateAndCompareFullRebuild();
  EXPECT_EQ(at(1.5, 0), 0);
  EXPECT_EQ(at(2.2, 0), 0);
  EXPECT_EQ(at(-5, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  obstacles->clearStaticObservations(true, true);
  updateAndCompareFullRebuild();
  EXPECT_EQ(at(-5, 0), 0);
  EXPECT_EQ(at(-4.3, 0), 0);
  EXPECT_EQ(at(2.7, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_P(GardenCostmapTest, SelectiveClearInvalidatesAndRebuildsAllAuthorization)
{
  auto m = mask(-10, -10, 400, 400);
  carve(*m, -9, -9, 9, 9);
  apply(m);
  nav2_costmap_2d::ClearCostmapService clearing(costmap, *costmap);
  ASSERT_TRUE(clearing.clearEntirely({"sensor"}));
  EXPECT_FALSE(keepout->isCurrent());
  EXPECT_EQ(at(9.5, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap->getLayeredCostmap()->updateMap(0, 0, 0);
  EXPECT_TRUE(keepout->isCurrent());
  EXPECT_EQ(at(9.5, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(at(0, 0), 0);
  costmap->resetLayers();
  EXPECT_FALSE(keepout->isCurrent());
  EXPECT_EQ(at(0, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap->getLayeredCostmap()->updateMap(0, 0, 0);
  EXPECT_TRUE(keepout->isCurrent());
  EXPECT_EQ(at(9.5, 0), nav2_costmap_2d::LETHAL_OBSTACLE);
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
