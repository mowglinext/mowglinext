// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "costmap_test_support.hpp"
#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>
#include <rcutils/logging.h>

namespace mowgli_nav2_plugins
{
struct FTCSpeedLimitTestAccess
{
  static void RequiredPivot(FTCController& c)
  {
    c.current_state_ = FTCController::PlannerState::PIVOT;
    c.current_index_ = 1;
    tf2::fromMsg(c.global_plan_[1].pose, c.current_control_point_);
  }
  static std::optional<double> CheckedTurn(const FTCController& c)
  {
    return c.checked_corner_turn_;
  }
  static double GeometricError(const FTCController& c)
  {
    return c.angle_error_;
  }

  static void AngularHistory(FTCController& c, double integral, double previous)
  {
    c.i_angle_error_ = integral;
    c.last_angle_error_ = previous;
    c.d_angle_filt_ = 0;
  }
  static void OscillatingHistory(FTCController& c)
  {
    c.failure_detector_.setBufferLength(4);
    for (int i = 0; i < 4; ++i)
      c.failure_detector_.update(0, i % 2 ? .008 : -.008, .3, .3, .8, .1, .1);
  }
  static void Geometry(FTCController& controller,
                       const visualization_msgs::msg::MarkerArray& message)
  {
    std::lock_guard lock(controller.corner_geometry_mutex_);
    controller.corner_geometry_ =
        std::make_shared<const CornerGeometry>(CornerGeometry::parse(message));
  }

  static bool TryGeometry(FTCController& controller,
                          const visualization_msgs::msg::MarkerArray& message)
  {
    std::unique_lock lock(controller.corner_geometry_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
      return false;
    controller.corner_geometry_ =
        std::make_shared<const CornerGeometry>(CornerGeometry::parse(message));
    return true;
  }

  static int State(const FTCController& controller)
  {
    return static_cast<int>(controller.current_state_);
  }

  static std::size_t Index(const FTCController& controller)
  {
    return controller.current_index_;
  }

  static bool TurnFallbackReversing(const FTCController& controller)
  {
    return controller.turnFallbackReversing();
  }

  static bool PreRotating(const FTCController& controller)
  {
    return controller.current_state_ == FTCController::PlannerState::PRE_ROTATE;
  }

  static bool Crashed(const FTCController& controller)
  {
    return controller.is_crashed_;
  }

  static bool ReverseEscaping(const FTCController& controller)
  {
    return controller.reverse_escape_active_;
  }

  static bool ObstacleWaiting(const FTCController& controller)
  {
    return controller.obstacle_waiting_;
  }

  static bool Stalled(const FTCController& controller)
  {
    return controller.is_stalled_;
  }

  static void ApproachFixture(FTCController& controller, double body_distance, bool second = false)
  {
    controller.current_state_ = FTCController::PlannerState::FOLLOWING;
    controller.current_index_ =
        second ? controller.pivot_corners_.back() : controller.pivot_corners_.front();
    tf2::fromMsg(controller.global_plan_[controller.current_index_].pose,
                 controller.current_control_point_);
    controller.local_control_point_.setIdentity();
    controller.local_control_point_.translation().x() = body_distance;
  }

  static std::pair<double, double> ApproachLimits(const FTCController& controller)
  {
    return {*controller.pivot_approach_speed_limit_, *controller.pivot_approach_response_limit_};
  }

  static void ApproachStateTick(FTCController& controller)
  {
    (void)controller.update_planner_state();
  }

  static bool ApproachTimerArmed(const FTCController& controller)
  {
    return controller.pivot_approach_start_.has_value();
  }

  static bool SpliceApproachFallback(FTCController& controller)
  {
    TurnFallbackPlan plan;
    plan.rejoin = 21;
    plan.connector_m = 0.925;
    return controller.spliceTurnFallback(plan, 0);
  }

  static void LeaveFallbackInitialPivot(FTCController& controller)
  {
    controller.current_state_ = FTCController::PlannerState::FOLLOWING;
    controller.current_index_ = 1;
    tf2::fromMsg(controller.global_plan_[1].pose, controller.current_control_point_);
    controller.local_control_point_.setIdentity();
    controller.local_control_point_.translation().x() = 0.925;
  }
};
}  // namespace mowgli_nav2_plugins

namespace
{
using Pose = geometry_msgs::msg::PoseStamped;
constexpr char kPlugin[] = "FollowCoveragePath";

Pose MakePose(double x, double y = 0.0, double yaw = 0.0)
{
  Pose pose;
  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

nav_msgs::msg::Path StraightPlan(double first = 0.0)
{
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (double x = first; x <= 6.0; x += 0.05)
  {
    plan.poses.push_back(MakePose(x));
  }
  return plan;
}

visualization_msgs::msg::MarkerArray MowingBox(double x0 = -5,
                                               double x1 = 15,
                                               double y0 = -5,
                                               double y1 = 15)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.ns = "area";
  m.text = "mowing";
  m.type = m.LINE_STRIP;
  m.pose.orientation.w = 1;
  for (const auto& xy :
       std::vector<std::pair<double, double>>{{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}, {x0, y0}})
  {
    geometry_msgs::msg::Point p;
    p.x = xy.first;
    p.y = xy.second;
    m.points.push_back(p);
  }
  visualization_msgs::msg::MarkerArray result;
  result.markers.push_back(m);
  return result;
}

TEST(CornerGeometry, RolesAndCompleteReplacementFailClosed)
{
  using Geometry = mowgli_nav2_plugins::CornerGeometry;
  auto m = MowingBox(-3, -1, -1, 1);
  EXPECT_TRUE(Geometry::parse(m).valid);
  auto outer = MowingBox(-10, 10, -10, 10).markers.front();
  outer.id = 1;
  outer.text = "navigation";
  m.markers.push_back(outer);
  auto g = Geometry::parse(m);
  EXPECT_TRUE(g.mowing.authorized({-2, 0}));
  EXPECT_FALSE(g.mowing.authorized({0, 0}));
  m.markers.front().text = "navigation";
  EXPECT_FALSE(Geometry::parse(m).valid);
  m.markers.front().text = "";
  EXPECT_FALSE(Geometry::parse(m).valid);
  m.markers.front().text = "mowing";
  m.markers.front().header.frame_id = "odom";
  EXPECT_FALSE(Geometry::parse(m).valid);
  m.markers.front().header.frame_id = "map";
  m.markers.front().points[0].x = NAN;
  EXPECT_FALSE(Geometry::parse(m).valid);
  EXPECT_FALSE(Geometry::parse({}).valid);
  m = MowingBox();
  m.markers[0].points = {m.markers[0].points[0],
                         m.markers[0].points[2],
                         m.markers[0].points[1],
                         m.markers[0].points[3]};
  EXPECT_FALSE(Geometry::parse(m).valid);
}

TEST(CornerGeometry, ProductionDockSnapshotKeepsRawStationGeometry)
{
  using namespace mowgli_nav2_plugins;
  const transit::Ring body{{.54, .285}, {.54, -.285}, {-.18, -.285}, {-.18, .285}};

  auto snapshot = MowingBox(-3, -1, -1, 1);
  snapshot.markers.front().text = "mowing";
  auto navigation = MowingBox(0, 2, -1, 1).markers.front();
  navigation.id = 1;
  navigation.text = "navigation";
  snapshot.markers.push_back(navigation);

  auto corridor = MowingBox(-3.8, -3.5, -.2, .2).markers.front();
  corridor.id = 2;
  corridor.ns = "dock_corridor";
  corridor.scale.x = .4;
  corridor.scale.y = .65;
  snapshot.markers.push_back(corridor);

  auto dock = MowingBox(-2.05, -1.9, -.2, .2).markers.front();
  dock.id = 3;
  dock.ns = "dock_body";
  dock.scale.x = .4;
  dock.scale.y = .65;
  snapshot.markers.push_back(dock);

  const auto geometry = CornerGeometry::parse(snapshot);
  ASSERT_TRUE(geometry.valid);
  EXPECT_TRUE(geometry.mowing.authorized({-2.7, 0}));
  EXPECT_FALSE(geometry.mowing.authorized({.5, 0})) << "navigation area alone is not mowing";
  EXPECT_FALSE(geometry.mowing.authorized({-3.65, 0})) << "dock corridor grants no mowing";

  EXPECT_FALSE(geometry.rotationClear({-2.45, 0}, 0, .5, body))
      << "the raw station intersects the filled chassis";
  EXPECT_FALSE(geometry.stoppingBudget({-2.45, 0}, .02, body));
  const auto budget = geometry.stoppingBudget({-2.7, 0}, .02, body);
  ASSERT_TRUE(budget);
  EXPECT_DOUBLE_EQ(*budget, .02)
      << "raw body clearance is about 39 mm; the transit margin is not applied again";
}

TEST(CornerGeometry, PeerDockSnapshotKeepsMowingAvailableAndRawBodyForbidden)
{
  using namespace mowgli_nav2_plugins;
  const transit::Ring body{{.54, .285}, {.54, -.285}, {-.18, -.285}, {-.18, .285}};
  auto snapshot = MowingBox(-3, -1, -1, 1);
  auto peer = MowingBox(-2.05, -1.9, -.2, .2).markers.front();
  peer.id = 1;
  peer.ns = "peer_dock";
  peer.scale.x = .4;
  snapshot.markers.push_back(peer);

  const auto geometry = CornerGeometry::parse(snapshot);
  ASSERT_TRUE(geometry.valid) << "the map server's peer station is a supported forbidden ring";
  EXPECT_TRUE(geometry.mowing.authorized({-2.7, 0}));
  EXPECT_FALSE(geometry.mowing.clear({-2, 0}));
  EXPECT_FALSE(geometry.rotationClear({-2.45, 0}, 0, .5, body));
  EXPECT_FALSE(geometry.stoppingBudget({-2.45, 0}, .02, body));
  const auto budget = geometry.stoppingBudget({-2.7, 0}, .02, body);
  ASSERT_TRUE(budget);
  EXPECT_DOUBLE_EQ(*budget, .02) << "transit's axle margin must not be applied to the body twice";

  snapshot.markers.back().ns = "unknown_station";
  EXPECT_FALSE(CornerGeometry::parse(snapshot).valid);
}

TEST(CornerGeometry, UnionCanCoverASeamButNavigationCannotCoverAGap)
{
  using Geometry = mowgli_nav2_plugins::CornerGeometry;
  auto m = MowingBox(-3, -2, -1, 1);
  auto second = MowingBox(-2, -1, -1, 1).markers[0];
  second.id = 1;
  m.markers.push_back(second);
  EXPECT_TRUE(Geometry::parse(m).mowing.segmentAuthorized({-2.5, 0}, {-1.5, 0}));
  m.markers[1].text = "navigation";
  EXPECT_FALSE(Geometry::parse(m).mowing.segmentAuthorized({-2.5, 0}, {-1.5, 0}));
}

TEST(CornerGeometry, WholeRawBodyArcIncludesThinHolesBetweenEndpointBodies)
{
  auto m = MowingBox();
  const mowgli_nav2_plugins::transit::Ring body{{.53, .275},
                                                {.53, -.275},
                                                {-.17, -.275},
                                                {-.17, .275}};
  auto g = mowgli_nav2_plugins::CornerGeometry::parse(m);
  EXPECT_TRUE(g.rotationClear({0, 0}, 0, 1, body));
  const double x = .53 * std::cos(.5) - .275 * std::sin(.5);
  const double y = .53 * std::sin(.5) + .275 * std::cos(.5);
  auto hole = MowingBox(x - .0001, x + .0001, y - .0001, y + .0001).markers.front();
  hole.ns = "obstacle";
  hole.text = "";
  m.markers.push_back(hole);
  g = mowgli_nav2_plugins::CornerGeometry::parse(m);
  EXPECT_TRUE(g.rotationClear({0, 0}, 0, 0, body));
  EXPECT_TRUE(g.rotationClear({0, 0}, 1, 0, body));
  EXPECT_FALSE(g.rotationClear({0, 0}, 0, 1, body));
  m.markers.back() = MowingBox(-.02, .02, -.02, .02).markers.front();
  m.markers.back().ns = "dock_body";
  EXPECT_FALSE(mowgli_nav2_plugins::CornerGeometry::parse(m).rotationClear({0, 0}, 0, 0, body));
}

TEST(CornerGeometry, DirectedOppositeArcPreservesTheWholeIntervalAcrossPiAndReflection)
{
  using namespace mowgli_nav2_plugins;
  const transit::Ring body{{.54, .285}, {.54, -.285}, {-.18, -.285}, {-.18, .285}};
  for (double reflection : {-1.0, 1.0})
    for (double phase : {0.0, M_PI})
    {
      auto m = MowingBox(-3, -1, -1, 1);
      auto hole = MowingBox(-2.125, -1.875, .375, .625).markers.front();
      hole.ns = "obstacle";
      m.markers.push_back(hole);
      const auto transform = [reflection, phase](double x, double y)
      {
        x *= reflection;
        return transit::Point{x * std::cos(phase) - y * std::sin(phase),
                              x * std::sin(phase) + y * std::cos(phase)};
      };
      for (auto& marker : m.markers)
        for (auto& p : marker.points)
        {
          const auto q = transform(p.x, p.y);
          p.x = q.x;
          p.y = q.y;
        }
      const auto axle = transform(-2.4065665196399597, .9982199846917013);
      const double yaw =
          (reflection > 0 ? -.002998904331640382 : M_PI + .002998904331640382) + phase;
      const double short_turn = reflection * -2.45137035628537;
      const double long_turn = short_turn - std::copysign(2 * M_PI, short_turn);
      auto geometry = CornerGeometry::parse(m);
      EXPECT_FALSE(geometry.rotationClear(axle, yaw, short_turn, body));
      EXPECT_TRUE(geometry.rotationClear(axle, yaw, long_turn, body));
      EXPECT_FALSE(
          geometry.rotationClear(axle, yaw, long_turn + std::copysign(2 * M_PI, long_turn), body));
      // This obstacle lies on the long arc only. Wrapping would miss it.
      auto long_only = MowingBox(-2.70, -2.68, 1.53, 1.55).markers.front();
      long_only.ns = "obstacle";
      for (auto& p : long_only.points)
      {
        const auto q = transform(p.x, p.y);
        p.x = q.x;
        p.y = q.y;
      }
      m.markers.erase(m.markers.begin() + 1);  // remove the short-only raw hole
      m.markers.push_back(long_only);
      geometry = CornerGeometry::parse(m);
      EXPECT_TRUE(geometry.rotationClear(axle, yaw, short_turn, body));
      EXPECT_FALSE(geometry.rotationClear(axle, yaw, long_turn, body));
    }
}

TEST(CornerGeometry, StoppingNeighborhoodRequiresOneStrictMowingInterior)
{
  using Geometry = mowgli_nav2_plugins::CornerGeometry;
  const mowgli_nav2_plugins::transit::Ring body{{.53, .275},
                                                {.53, -.275},
                                                {-.17, -.275},
                                                {-.17, .275}};
  auto m = MowingBox(-1, 1, -1, 1);
  auto g = Geometry::parse(m);
  EXPECT_TRUE(g.stoppingNeighborhoodClear({0, 0}, .2, body));
  EXPECT_FALSE(g.stoppingNeighborhoodClear({0, 0}, 1, body));
  EXPECT_FALSE(g.stoppingNeighborhoodClear({1, 0}, 0, body));
  EXPECT_FALSE(g.stoppingNeighborhoodClear({0, 0}, NAN, body));
  EXPECT_FALSE(g.stoppingNeighborhoodClear({0, 0}, -.1, body));
  EXPECT_FALSE(g.stoppingNeighborhoodClear({NAN, 0}, .1, body));
  EXPECT_FALSE(g.stoppingNeighborhoodClear({0, 0}, .1, {}));
  EXPECT_FALSE(g.stoppingNeighborhoodClear({0, 0}, .1, {{0, 0}, {1, 1}, {0, 1}, {1, 0}}));
  EXPECT_FALSE(g.stoppingNeighborhoodClear({0, 0}, .1, {{0, 0}, {INFINITY, 0}, {0, 1}}));
  // Outer containment belongs to the axle, so body overhang alone is allowed.
  EXPECT_TRUE(
      Geometry::parse(MowingBox(-.1, .1, -.1, .1)).stoppingNeighborhoodClear({0, 0}, .05, body));
  m = MowingBox(-1, 0, -1, 1);
  m.markers.push_back(MowingBox(0, 1, -1, 1).markers.front());
  g = Geometry::parse(m);
  EXPECT_TRUE(g.mowing.authorized({0, 0}));
  EXPECT_FALSE(g.stoppingNeighborhoodClear({0, 0}, .05, body));
  m = MowingBox();
  m.markers.front().text = "navigation";
  EXPECT_FALSE(Geometry::parse(m).stoppingNeighborhoodClear({0, 0}, .05, body));
  EXPECT_FALSE(Geometry::parse({}).stoppingNeighborhoodClear({0, 0}, .05, body));
}

TEST(CornerGeometry, StoppingNeighborhoodIncludesRawHolesAndFilledBodyAtEveryYaw)
{
  using Geometry = mowgli_nav2_plugins::CornerGeometry;
  const mowgli_nav2_plugins::transit::Ring body{{.5, 0}, {0, .25}, {-.2, 0}};
  auto m = MowingBox(-2, 2, -2, 2);
  auto hole = MowingBox(.6, .601, -.001, .001).markers.front();
  hole.ns = "obstacle";
  m.markers.push_back(hole);
  // Exact expanded-radius equality refuses. This thin ring misses both the
  // current body and axle ray, but lies in the any-yaw filled-body neighborhood.
  EXPECT_FALSE(Geometry::parse(m).stoppingNeighborhoodClear({0, 0}, .1, body));
  m.markers.back().ns = "dock_body";
  EXPECT_FALSE(Geometry::parse(m).stoppingNeighborhoodClear({0, 0}, .1, body));
  m.markers.back() = MowingBox(-.01, .01, -.01, .01).markers.front();
  m.markers.back().ns = "obstacle";
  EXPECT_FALSE(Geometry::parse(m).stoppingNeighborhoodClear({0, 0}, .1, body));
  m.markers.back() = MowingBox(1, 1.01, 1, 1.01).markers.front();
  m.markers.back().ns = "obstacle";
  EXPECT_TRUE(Geometry::parse(m).stoppingNeighborhoodClear({0, 0}, .1, body));
}

TEST(CornerGeometry, StoppingBudgetCertifiesClearanceAndContainingRingChoice)
{
  using Geometry = mowgli_nav2_plugins::CornerGeometry;
  const mowgli_nav2_plugins::transit::Ring body{{.5, 0}, {0, .25}, {-.2, 0}};
  const auto check = [&](const Geometry& g, mowgli_nav2_plugins::transit::Point axle, double r)
  {
    const auto budget = g.stoppingBudget(axle, r, body);
    EXPECT_TRUE(budget);
    if (budget)
    {
      EXPECT_GT(*budget, 0);
      EXPECT_LE(*budget, r);
      EXPECT_TRUE(g.stoppingNeighborhoodClear(axle, *budget, body));
    }
    return budget;
  };
  auto g = Geometry::parse(MowingBox(-1, 1, -1, 1));
  ASSERT_TRUE(check(g, {0, 0}, .2));
  EXPECT_DOUBLE_EQ(*g.stoppingBudget({0, 0}, .2, body), .2);
  auto budget = check(g, {0, .99}, .02);
  ASSERT_TRUE(budget);
  EXPECT_LT(*budget, .01);
  EXPECT_GT(*budget, .009);
  // Actual positive 1.4 mm clearance remains a strict disk certificate.
  budget = check(g, {0, 1 - .0014}, .02);
  ASSERT_TRUE(budget);
  EXPECT_LT(*budget, .0014);
  EXPECT_FALSE(g.stoppingBudget({0, 1}, .02, body));
  EXPECT_FALSE(g.stoppingBudget({0, 1.01}, .02, body));
  auto m = MowingBox(-1, 1, -1, 1);
  m.markers.push_back(MowingBox(-2, 2, -2, 2).markers.front());
  budget = check(Geometry::parse(m), {0, 0}, 1.5);
  ASSERT_TRUE(budget);
  EXPECT_DOUBLE_EQ(*budget, 1.5);
  m.markers.back().text = "navigation";
  budget = check(Geometry::parse(m), {0, 0}, 1.5);
  ASSERT_TRUE(budget);
  EXPECT_LT(*budget, 1);
  m = MowingBox(-2, 0, -1, 1);
  m.markers.push_back(MowingBox(0, 2, -1, 1).markers.front());
  EXPECT_FALSE(Geometry::parse(m).stoppingBudget({0, 0}, .02, body));
}

TEST(CornerGeometry, StoppingBudgetRawBodyRoomAndFloatingPointRefusal)
{
  using Geometry = mowgli_nav2_plugins::CornerGeometry;
  const mowgli_nav2_plugins::transit::Ring body{{.5, 0}, {0, .25}, {-.2, 0}};
  auto m = MowingBox(-3, 3, -3, 3);
  auto hole = MowingBox(1.5, 1.501, -.001, .001).markers.front();
  hole.ns = "obstacle";
  m.markers.push_back(hole);
  auto g = Geometry::parse(m);
  EXPECT_FALSE(g.stoppingNeighborhoodClear({0, 0}, 1.2, body));
  const auto budget = g.stoppingBudget({0, 0}, 1.2, body);
  ASSERT_TRUE(budget);
  EXPECT_LT(*budget, 1);
  EXPECT_GT(*budget, .99);
  EXPECT_TRUE(g.stoppingNeighborhoodClear({0, 0}, *budget, body));
  for (double edge : {0.0, .5, .6})
  {
    m.markers.back() = MowingBox(edge - .001, edge + .001, -.001, .001).markers.front();
    m.markers.back().ns = "dock_body";
    if (edge == .6)
    {
      // A positive real room can round back to equality during the final
      // proof. The helper must decline rather than iterate toward permission.
      m.markers.back() = MowingBox(.6, .601, -.001, .001).markers.front();
      m.markers.back().ns = "obstacle";
    }
    EXPECT_FALSE(Geometry::parse(m).stoppingBudget({0, 0}, 1.2, body));
  }
  g = Geometry::parse(MowingBox());
  for (double r : std::vector<double>{0.0, -1.0, NAN, INFINITY})
    EXPECT_FALSE(g.stoppingBudget({0, 0}, r, body));
  EXPECT_FALSE(g.stoppingBudget({NAN, 0}, .1, body));
  EXPECT_FALSE(g.stoppingBudget({0, 0}, .1, {{0, 0}, {1, 1}, {0, 1}, {1, 0}}));
  EXPECT_FALSE(g.stoppingBudget({0, 0}, .1, {{0, 0}, {INFINITY, 0}, {0, 1}}));
  EXPECT_FALSE(Geometry::parse({}).stoppingBudget({0, 0}, .1, body));
  m = MowingBox();
  m.markers.front().text = "navigation";
  EXPECT_FALSE(Geometry::parse(m).stoppingBudget({0, 0}, .1, body));
}

// This fixture exercises the real controller with an empty configured costmap
// and explicit TF. It advances only the ROS clock: the fixed robot pose and
// moving odometry isolate command bounds from drivetrain response and DDS.
class FtcSpeedLimit : public ::testing::Test
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
    node_ = std::make_shared<nav2::LifecycleNode>("controller_server", "");
    for (const auto& parameter :
         std::vector<rclcpp::Parameter>{{"speed_fast", 0.20},
                                        {"speed_slow", 0.16},
                                        {"speed_fast_threshold", 0.5},
                                        {"min_speed_mps", 0.15},
                                        {"max_cmd_vel_speed", 0.30},
                                        {"max_cmd_vel_ang", 0.8},
                                        {"kp_lon", 1.0},
                                        {"acceleration", 1.0},
                                        {"stall_grace_s", 30.0},
                                        {"max_follow_distance", 2.0},
                                        {"goal_timeout", 30.0},
                                        {"check_obstacles", false},
                                        {"enable_obstacle_deviation", false},
                                        {"confine_deviation_to_zone", false},
                                        {"use_offset_lattice", true},
                                        {"use_footprint_clearance", true},
                                        {"obstacle_reverse_enabled", true},
                                        {"obstacle_reverse_speed_mps", 0.15},
                                        {"obstacle_reverse_max_dist_m", 0.30},
                                        {"turn_fallback_enabled", false}})
    {
      node_->declare_parameter(std::string(kPlugin) + "." + parameter.get_name(),
                               parameter.get_parameter_value());
    }
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", "/", false);
    for (const auto& parameter : std::vector<rclcpp::Parameter>{
             {"plugins", std::vector<std::string>{}},
             {"global_frame", "odom"},
             {"robot_base_frame", "base_footprint"},
             {"rolling_window", false},
             {"width", 20},
             {"height", 20},
             {"resolution", 0.05},
             {"origin_x", -5.0},
             {"origin_y", -5.0},
             {"footprint", "[[0.53,0.275],[0.53,-0.275],[-0.17,-0.275],[-0.17,0.275]]"},
             {"footprint_padding", 0.01}})
    {
      if (costmap_->has_parameter(parameter.get_name()))
      {
        ASSERT_TRUE(costmap_->set_parameter(parameter).successful);
      }
      else
      {
        costmap_->declare_parameter(parameter.get_name(), parameter.get_parameter_value());
      }
    }
    costmap_->configure();
    tf_ = costmap_->getTfBuffer();
    clock_ = node_->get_clock();
    ASSERT_EQ(rcl_enable_ros_time_override(clock_->get_clock_handle()), RCL_RET_OK);
    SetTime();
    SetTransform("map", "odom", 0.0, true);
    SetTransform("base_footprint", "base_link", 0.0, true);
    SetTransform("odom", "base_footprint", 0.0, false);
    controller_.configure(node_, kPlugin, tf_, costmap_);
    controller_.activate();
    mowgli_nav2_plugins::FTCSpeedLimitTestAccess::Geometry(controller_, MowingBox());
    StartPlan(StraightPlan());
  }

  void TearDown() override
  {
    controller_.deactivate();
    controller_.cleanup();
    mowgli_nav2_plugins_test::WaitForTfListenerSpinning(*costmap_);
    costmap_->cleanup();
  }

  void SetParameter(const std::string& key, double value)
  {
    ASSERT_TRUE(node_->set_parameter({std::string(kPlugin) + "." + key, value}).successful);
  }

  void SetParameter(const std::string& key, bool value)
  {
    ASSERT_TRUE(node_->set_parameter({std::string(kPlugin) + "." + key, value}).successful);
  }

  void StartPlan(const nav_msgs::msg::Path& plan)
  {
    controller_.newPathReceived(plan);
    // Begin these command-bound tests at rest before injecting their isolated
    // moving-odom feedback. PRE_ROTATE now correctly waits for translation.
    controller_.computeVelocityCommands(
        Pose{}, geometry_msgs::msg::Twist{}, nullptr, nav_msgs::msg::Path{}, Pose{});
  }

  geometry_msgs::msg::Twist TickCommand()
  {
    time_ns_ += 100000000;
    SetTime();
    SetTransform("odom", "base_footprint", robot_x_, false);
    geometry_msgs::msg::Twist measured;
    measured.linear.x = measured_speed_;
    return controller_
        .computeVelocityCommands(Pose{}, measured, nullptr, nav_msgs::msg::Path{}, Pose{})
        .twist;
  }

  double Tick()
  {
    return TickCommand().linear.x;
  }

  void ExpectTerminalFailure()
  {
    try
    {
      const auto command = TickCommand();
      ADD_FAILURE() << "Terminal failure returned a recovery command: linear=" << command.linear.x
                    << " angular=" << command.angular.z;
    }
    catch (const nav2_core::ControllerException& error)
    {
      EXPECT_STREQ(error.what(), "FTCController: terminal failure in planner state machine.");
    }
    EXPECT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::Crashed(controller_));
  }

  double PeakForward(int ticks = 30)
  {
    double peak = 0.0;
    for (int i = 0; i < ticks; ++i)
    {
      const double command = Tick();
      EXPECT_GE(command, 0.0);
      EXPECT_TRUE(std::isfinite(command));
      peak = std::max(peak, command);
    }
    return peak;
  }

  void Wall(double x)
  {
    auto* map = costmap_->getCostmap();
    for (double y = -1.5; y <= 2.0; y += 0.05)
    {
      unsigned int mx = 0;
      unsigned int my = 0;
      ASSERT_TRUE(map->worldToMap(x, y, mx, my));
      map->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
  }

  void SetTime()
  {
    ASSERT_EQ(rcl_set_ros_time_override(clock_->get_clock_handle(), time_ns_), RCL_RET_OK);
  }

  void SetTransform(const std::string& parent, const std::string& child, double x, bool fixed)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(time_ns_, RCL_ROS_TIME);
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.transform.translation.x = x;
    transform.transform.translation.y = child == "base_footprint" ? robot_y_ : 0.0;
    const double yaw = child == "base_footprint" ? robot_yaw_ : 0.0;
    transform.transform.rotation.z = std::sin(yaw / 2.0);
    transform.transform.rotation.w = std::cos(yaw / 2.0);
    ASSERT_TRUE(tf_->setTransform(transform, "test", fixed));
  }

  std::shared_ptr<nav2::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  mowgli_nav2_plugins::FTCController controller_;
  std::int64_t time_ns_{1000000000000};
  double robot_x_{0.0};
  double robot_yaw_{0.0};
  double robot_y_{0.0};
  double measured_speed_{0.20};
};

class FtcPercentageLimit : public FtcSpeedLimit, public ::testing::WithParamInterface<double>
{
};

TEST_P(FtcPercentageLimit, UsesPercentOfConfiguredMaximum)
{
  SetParameter("speed_fast", 0.80);
  controller_.setSpeedLimit(GetParam(), true);
  const double peak = PeakForward();
  EXPECT_NEAR(peak, 0.30 * GetParam() / 100.0, 1e-9);
}

INSTANTIATE_TEST_SUITE_P(Nav2Percent, FtcPercentageLimit, ::testing::Values(5.0, 50.0, 100.0));

// The state-transition log lies after departure readiness and before the
// state commit. It gives a deterministic executor/control-loop interleaving
// without adding a production test hook or relying on thread scheduling.
class DepartureInterleave
{
public:
  explicit DepartureInterleave(std::function<void()> attempt, int from = 0)
      : previous_(rcutils_logging_get_output_handler()), attempt_(std::move(attempt)), from_(from)
  {
    active_ = this;
    rcutils_logging_set_output_handler(Capture);
  }
  ~DepartureInterleave()
  {
    rcutils_logging_set_output_handler(previous_);
    active_ = nullptr;
  }

private:
  static void Capture(const rcutils_log_location_t* location,
                      int severity,
                      const char* name,
                      rcutils_time_point_value_t stamp,
                      const char* format,
                      va_list* args)
  {
    if (!active_)
      return;
    char message[512];
    va_list copy;
    va_copy(copy, *args);
    std::vsnprintf(message, sizeof(message), format, copy);
    va_end(copy);
    int before = -1, after = -1;
    if (std::sscanf(message, "FTCController: state transition %d -> %d", &before, &after) == 2 &&
        before == active_->from_ && after == 1)
      active_->attempt_();
    if (active_->from_ == -1 && std::string(message).rfind("FTCController PID |", 0) == 0)
      active_->attempt_();
    if (active_->previous_)
      active_->previous_(location, severity, name, stamp, format, args);
  }
  inline static thread_local DepartureInterleave* active_{};
  rcutils_logging_output_handler_t previous_;
  std::function<void()> attempt_;
  int from_;
};

TEST_F(FtcSpeedLimit, GeometryReplacementCannotInterleaveCommittedDeparture)
{
  using Access = mowgli_nav2_plugins::FTCSpeedLimitTestAccess;
  auto revoked = MowingBox();
  revoked.markers.front().text = "navigation";
  controller_.newPathReceived(StraightPlan());
  measured_speed_ = 0;
  bool attempted = false, replaced = false;
  double command = 0;
  {
    DepartureInterleave interleave(
        [&]()
        {
          // A real executor thread can publish the replacement here on the old
          // code. A committed handoff must serialize it after command selection.
          std::thread executor(
              [&]()
              {
                attempted = true;
                replaced = Access::TryGeometry(controller_, revoked);
              });
          executor.join();
        });
    command = Tick();
  }
  ASSERT_TRUE(attempted);
  EXPECT_FALSE(replaced) << "revocation was committed before a forward departure command="
                         << command;
  EXPECT_EQ(Access::State(controller_), 1);
  EXPECT_GE(command, 0);
  EXPECT_GT(Tick(), 0);
  // A replacement ordered before the next handoff must instead retain PRE
  // and its deadline, rather than silently consuming that transition.
  Access::Geometry(controller_, revoked);
  controller_.newPathReceived(StraightPlan());
  EXPECT_DOUBLE_EQ(Tick(), 0);
  EXPECT_EQ(Access::State(controller_), 0);
}

TEST_F(FtcSpeedLimit, SmallAbsoluteLimitWinsOverMinimumMovementSpeed)
{
  controller_.setSpeedLimit(0.05, false);
  const double peak = PeakForward();
  EXPECT_GT(peak, 0.0);
  EXPECT_LE(peak, 0.05 + 1e-9);
}

class FtcClearLimit : public FtcSpeedLimit, public ::testing::WithParamInterface<bool>
{
};

TEST_P(FtcClearLimit, ZeroClearsAnExistingLimit)
{
  controller_.setSpeedLimit(0.05, false);
  PeakForward();
  controller_.setSpeedLimit(0.0, GetParam());
  EXPECT_GT(PeakForward(), 0.15);
}

INSTANTIATE_TEST_SUITE_P(Nav2Clear, FtcClearLimit, ::testing::Bool());

TEST_F(FtcSpeedLimit, AbsoluteLimitCannotRaiseConfiguredMaximum)
{
  SetParameter("speed_fast", 0.80);
  controller_.setSpeedLimit(1.0, false);
  EXPECT_LE(PeakForward(), 0.30 + 1e-9);
}

TEST_F(FtcSpeedLimit, DynamicMaximumCannotRemoveAbsoluteLimit)
{
  controller_.setSpeedLimit(0.05, false);
  SetParameter("max_cmd_vel_speed", 0.40);
  SetParameter("speed_fast", 0.80);
  EXPECT_LE(PeakForward(), 0.05 + 1e-9);
}

TEST_F(FtcSpeedLimit, ActivePercentageTracksDynamicMaximum)
{
  controller_.setSpeedLimit(50.0, true);
  SetParameter("max_cmd_vel_speed", 0.10);
  EXPECT_LE(PeakForward(), 0.05 + 1e-9);
}

TEST_F(FtcSpeedLimit, DynamicFastSpeedCannotLiftAnActivePercentageLimit)
{
  controller_.setSpeedLimit(5.0, true);
  SetParameter("speed_fast", 0.80);
  EXPECT_LE(PeakForward(), 0.015 + 1e-9);
}

TEST_F(FtcSpeedLimit, ClearRestoresTheLatestConfiguredMaximum)
{
  controller_.setSpeedLimit(0.05, false);
  SetParameter("max_cmd_vel_speed", 0.10);
  controller_.setSpeedLimit(0.0, false);
  const double peak = PeakForward();
  EXPECT_GT(peak, 0.0);
  EXPECT_LE(peak, 0.10 + 1e-9);
}

TEST_F(FtcSpeedLimit, LegacyNegativeClearStillRestoresNormalFollowing)
{
  controller_.setSpeedLimit(0.05, false);
  controller_.setSpeedLimit(-1.0, false);
  EXPECT_GT(PeakForward(), 0.15);
}

class FtcInvalidLimit : public FtcSpeedLimit, public ::testing::WithParamInterface<double>
{
};

TEST_P(FtcInvalidLimit, NonfiniteInputPreservesAnExistingExternalCap)
{
  controller_.setSpeedLimit(0.05, false);
  controller_.setSpeedLimit(GetParam(), false);
  const double peak = PeakForward();
  EXPECT_GT(peak, 0.0);
  EXPECT_LE(peak, 0.05 + 1e-9);
}

INSTANTIATE_TEST_SUITE_P(InvalidNav2Input,
                         FtcInvalidLimit,
                         ::testing::Values(std::numeric_limits<double>::quiet_NaN(),
                                           std::numeric_limits<double>::infinity(),
                                           -std::numeric_limits<double>::infinity()));

TEST_F(FtcSpeedLimit, HealthyMotionAtALowCapIsNotMistakenForAStall)
{
  SetParameter("stall_grace_s", 0.6);
  measured_speed_ = 0.05;
  controller_.setSpeedLimit(0.05, false);
  for (int i = 0; i < 60; ++i)
  {
    const double command = Tick();
    ASSERT_LE(command, 0.05 + 1e-9);
    ASSERT_FALSE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::Stalled(controller_));
    robot_x_ += command * 0.10;
  }
  EXPECT_GT(robot_x_, 0.20);
}

TEST_F(FtcSpeedLimit, FollowingResyncFailurePreemptsActiveReverseEscape)
{
  using Access = mowgli_nav2_plugins::FTCSpeedLimitTestAccess;
  SetParameter("check_obstacles", true);
  SetParameter("enable_obstacle_deviation", true);
  robot_x_ = 2.0;
  SetTransform("odom", "base_footprint", robot_x_, false);
  StartPlan(StraightPlan(robot_x_));
  measured_speed_ = -0.05;
  Wall(3.0);
  ASSERT_LT(Tick(), 0.0);
  ASSERT_EQ(Access::State(controller_), 1);
  ASSERT_TRUE(Access::ReverseEscaping(controller_));

  // A newly supported pose too far from this plan is terminal even while a
  // recovery owns output. No injected FSM state or shortened timeout is used.
  robot_y_ = 3.0;
  ExpectTerminalFailure();

  costmap_->getCostmap()->resetMapToValue(0, 0, 400, 400, nav2_costmap_2d::FREE_SPACE);
  robot_y_ = 0.0;
  measured_speed_ = 0.20;
  StartPlan(StraightPlan(robot_x_));
  EXPECT_FALSE(Access::Crashed(controller_));
  EXPECT_FALSE(Access::ReverseEscaping(controller_));
  EXPECT_GT(PeakForward(5), 0.0);
}

TEST_F(FtcSpeedLimit, PreRotateTimeoutPreemptsBodyContactHold)
{
  using Access = mowgli_nav2_plugins::FTCSpeedLimitTestAccess;
  SetParameter("check_obstacles", true);
  SetParameter("enable_obstacle_deviation", true);
  robot_yaw_ = M_PI / 2.0;
  measured_speed_ = 0.0;
  SetTransform("odom", "base_footprint", robot_x_, false);
  controller_.newPathReceived(StraightPlan());
  const auto started = time_ns_;
  time_ns_ = started + 29800000000;
  const auto rotating = TickCommand();  // 29.9 s: legal pre-timeout rotation
  ASSERT_EQ(rotating.linear.x, 0.0);
  ASSERT_GT(std::abs(rotating.angular.z), 0.05);
  ASSERT_TRUE(Access::PreRotating(controller_));

  Wall(0.1);  // current footprint; start the obstacle wait only at 30.0 s
  const auto hold = TickCommand();
  ASSERT_EQ(hold.linear.x, 0.0);
  ASSERT_EQ(hold.angular.z, 0.0);
  ASSERT_TRUE(Access::ObstacleWaiting(controller_));
  ASSERT_FALSE(Access::Crashed(controller_));
  ExpectTerminalFailure();  // obstacle wait has run only 0.1 s, not its 2.5 s bound

  costmap_->getCostmap()->resetMapToValue(0, 0, 400, 400, nav2_costmap_2d::FREE_SPACE);
  controller_.newPathReceived(StraightPlan());
  EXPECT_FALSE(Access::Crashed(controller_));
  EXPECT_FALSE(Access::ObstacleWaiting(controller_));
  const auto reset_rotation = TickCommand();
  EXPECT_EQ(reset_rotation.linear.x, 0.0);
  EXPECT_GT(std::abs(reset_rotation.angular.z), 0.05);
}

TEST_F(FtcSpeedLimit, TransientBodyContactHoldReleasesBeforeTimeout)
{
  using Access = mowgli_nav2_plugins::FTCSpeedLimitTestAccess;
  SetParameter("check_obstacles", true);
  SetParameter("enable_obstacle_deviation", true);
  ASSERT_GT(PeakForward(5), 0.0);
  Wall(0.1);
  ASSERT_EQ(Tick(), 0.0);
  ASSERT_TRUE(Access::ObstacleWaiting(controller_));
  costmap_->getCostmap()->resetMapToValue(0, 0, 400, 400, nav2_costmap_2d::FREE_SPACE);
  EXPECT_GT(PeakForward(20), 0.0);
  EXPECT_FALSE(Access::ObstacleWaiting(controller_));
  EXPECT_FALSE(Access::Crashed(controller_));
}

TEST_F(FtcSpeedLimit, NonexpiredReverseEscapeReleasesOnClearProfile)
{
  using Access = mowgli_nav2_plugins::FTCSpeedLimitTestAccess;
  SetParameter("check_obstacles", true);
  SetParameter("enable_obstacle_deviation", true);
  robot_x_ = 2.0;
  measured_speed_ = -0.05;
  SetTransform("odom", "base_footprint", robot_x_, false);
  StartPlan(StraightPlan(robot_x_));
  Wall(3.0);
  ASSERT_LT(Tick(), 0.0);
  ASSERT_TRUE(Access::ReverseEscaping(controller_));
  costmap_->getCostmap()->resetMapToValue(0, 0, 400, 400, nav2_costmap_2d::FREE_SPACE);
  double forward = 0.0;
  for (int i = 0; i < 20; ++i)
  {
    forward = std::max(forward, Tick());
  }
  EXPECT_GT(forward, 0.0);
  EXPECT_FALSE(Access::ReverseEscaping(controller_));
  EXPECT_FALSE(Access::Crashed(controller_));
}

TEST_F(FtcSpeedLimit, ReverseEscapeObeysTheSignedExternalCap)
{
  SetParameter("check_obstacles", true);
  SetParameter("enable_obstacle_deviation", true);
  robot_x_ = 2.0;
  SetTransform("odom", "base_footprint", robot_x_, false);
  StartPlan(StraightPlan(robot_x_));
  Wall(3.0);
  controller_.setSpeedLimit(0.05, false);
  double reverse = 0.0;
  for (int i = 0; i < 15; ++i)
  {
    const double command = Tick();
    EXPECT_LE(std::abs(command), 0.05 + 1e-9);
    reverse = std::min(reverse, command);
  }
  EXPECT_LT(reverse, 0.0);
}

TEST_F(FtcSpeedLimit, TurnFallbackReverseObeysTheSignedExternalCap)
{
  SetParameter("check_obstacles", true);
  SetParameter("enable_obstacle_deviation", true);
  SetParameter("turn_fallback_enabled", true);
  robot_x_ = 2.85;
  SetTransform("odom", "base_footprint", robot_x_, false);
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  plan.poses.push_back(MakePose(2.85));
  plan.poses.push_back(MakePose(2.90));
  plan.poses.push_back(MakePose(2.95));
  const int steps = static_cast<int>(std::ceil(M_PI * 0.30 / 0.05));
  for (int i = 0; i <= steps; ++i)
  {
    const double angle = -M_PI / 2.0 + M_PI * i / steps;
    plan.poses.push_back(
        MakePose(3.0 + 0.30 * std::cos(angle), 0.30 + 0.30 * std::sin(angle), angle + M_PI / 2.0));
  }
  for (double x = 2.95; x >= 0.0; x -= 0.05)
  {
    plan.poses.push_back(MakePose(x, 0.60, M_PI));
  }
  StartPlan(plan);
  Wall(3.45);
  controller_.setSpeedLimit(0.05, false);
  bool saw_fallback_reverse = false;
  for (int i = 0; i < 15; ++i)
  {
    const double command = Tick();
    if (command < 0.0)
    {
      ASSERT_TRUE(mowgli_nav2_plugins::FTCSpeedLimitTestAccess::TurnFallbackReversing(controller_));
      saw_fallback_reverse = true;
      EXPECT_LE(std::abs(command), 0.05 + 1e-9);
      break;
    }
  }
  EXPECT_TRUE(saw_fallback_reverse);
}

// Public compute loop with true plant feedback. Controller: 10 Hz; numerical
// integration: 50 Hz. Ideal and tau=0.07 s responses are declared test plants,
// not robot calibration. Private access observes state only.
class FtcPivotArrival : public FtcSpeedLimit, public ::testing::WithParamInterface<double>
{
protected:
  using Access = mowgli_nav2_plugins::FTCSpeedLimitTestAccess;

  void RobotTransform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(time_ns_, RCL_ROS_TIME);
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.rotation.z = std::sin(yaw_ / 2);
    transform.transform.rotation.w = std::cos(yaw_ / 2);
    ASSERT_TRUE(tf_->setTransform(transform, "arrival-plant", false));
  }

  nav_msgs::msg::Path CornerPlan(double sign = 1.0, double spacing = 0.6)
  {
    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    for (int i = 0; i <= 20; ++i)
      plan.poses.push_back(MakePose(0.05 * i));
    plan.poses.push_back(MakePose(1, 0, sign * M_PI / 2));
    plan.poses.push_back(MakePose(1, sign * spacing, sign * M_PI / 2));
    plan.poses.push_back(MakePose(1, sign * spacing, sign * M_PI));
    for (int i = 1; i <= 20; ++i)
      plan.poses.push_back(MakePose(1 - 0.05 * i, sign * spacing, sign * M_PI));
    return plan;
  }

  void HolePivot(double reflection = 1,
                 bool required = true,
                 bool raw_hole = true,
                 bool long_obstacle = false)
  {
    auto geometry = MowingBox(-3, -1, -1, 1);
    if (raw_hole)
    {
      auto hole = MowingBox(-2.125, -1.875, .375, .625).markers.front();
      hole.ns = "obstacle";
      geometry.markers.push_back(hole);
    }
    if (long_obstacle)
    {
      auto hole = MowingBox(-2.70, -2.68, 1.53, 1.55).markers.front();
      hole.ns = "obstacle";
      geometry.markers.push_back(hole);
    }
    for (auto& marker : geometry.markers)
      for (auto& p : marker.points)
        p.x *= reflection;
    Access::Geometry(controller_, geometry);
    x_ = reflection * -2.4065665196399597;
    y_ = .9982199846917013;
    yaw_ = reflection > 0 ? -.002998904331640382 : M_PI + .002998904331640382;
    v_ = w_ = 0;
    const double target = reflection > 0 ? -2.4543692606170104 : M_PI + 2.4543692606170104;
    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    plan.poses = {MakePose(x_, y_, yaw_),
                  MakePose(x_, y_, target),
                  MakePose(x_ + .3 * std::cos(target), y_ + .3 * std::sin(target), target)};
    RobotTransform();
    controller_.newPathReceived(plan);
    if (required)
      Access::RequiredPivot(controller_);
  }

  geometry_msgs::msg::Twist Command()
  {
    time_ns_ += 100000000;
    SetTime();
    RobotTransform();
    geometry_msgs::msg::Twist measured;
    measured.linear.x = v_;
    measured.angular.z = w_;
    return controller_
        .computeVelocityCommands(Pose{}, measured, nullptr, nav_msgs::msg::Path{}, Pose{})
        .twist;
  }

  void Advance(const geometry_msgs::msg::Twist& command, double tau)
  {
    for (int i = 0; i < 5; ++i)
    {
      const double a = tau == 0 ? 0 : std::exp(-0.02 / tau);
      const double ah = tau == 0 ? 0 : std::exp(-0.01 / tau);
      const double v = command.linear.x + (v_ - command.linear.x) * ah;
      const double w = command.angular.z + (w_ - command.angular.z) * ah;
      const double delta = w * 0.02;
      const double sinc = std::abs(delta) < 1e-12 ? 1 : std::sin(delta / 2) / (delta / 2);
      x_ += v * 0.02 * sinc * std::cos(yaw_ + delta / 2);
      y_ += v * 0.02 * sinc * std::sin(yaw_ + delta / 2);
      yaw_ += delta;
      v_ = command.linear.x + (v_ - command.linear.x) * a;
      w_ = command.angular.z + (w_ - command.angular.z) * a;
    }
  }

  void RunArrival(
      double sign, double angle = 0, double spacing = 0.6, std::size_t trim = 0, double cap = .3)
  {
    auto plan = CornerPlan(sign, spacing);
    // The captured first tip/start from the retained ring replay are tested
    // separately outside the repository. This reachable approach also starts
    // 125 mm off line and exercises the real point-bearing correction.
    x_ = 0;
    y_ = -sign * 0.125;
    yaw_ = 0;
    if (trim)
    {
      x_ = plan.poses[trim].pose.position.x;
      y_ = plan.poses[trim].pose.position.y;
      if (trim < 20)
        y_ -= sign * .125;  // interrupted incoming leg retains a real lateral error
      yaw_ =
          2 * std::atan2(plan.poses[trim].pose.orientation.z, plan.poses[trim].pose.orientation.w);
      plan.poses.erase(plan.poses.begin(), plan.poses.begin() + trim);
    }
    const auto transform = [angle](double& x, double& y)
    {
      const double ox = x;
      x = std::cos(angle) * x - std::sin(angle) * y + 0.3;
      y = std::sin(angle) * ox + std::cos(angle) * y - 0.4;
    };
    transform(x_, y_);
    yaw_ += angle;
    for (auto& p : plan.poses)
    {
      transform(p.pose.position.x, p.pose.position.y);
      const double yaw = 2 * std::atan2(p.pose.orientation.z, p.pose.orientation.w) + angle;
      p.pose.orientation.z = std::sin(yaw / 2);
      p.pose.orientation.w = std::cos(yaw / 2);
    }
    double gx = 1, gy = 0;
    transform(gx, gy);
    RobotTransform();
    controller_.newPathReceived(plan);
    std::ofstream trace;
    if (const char* dir = std::getenv("FTC_ARRIVAL_TRACE_DIR"))
    {
      const auto* info = ::testing::UnitTest::GetInstance()->current_test_info();
      std::string name = info->name();
      std::replace(name.begin(), name.end(), '/', '-');
      trace.open(std::string(dir) + "/" + name + ".csv");
      trace << "t,x,y,yaw,v,w,cmd_v,cmd_w,state,index,distance\n" << std::setprecision(17);
    }
    bool braked = false, pivoted = trim >= 20, left = false;
    int completed = 0;
    for (int tick = 0; tick < 700; ++tick)
    {
      const int before = Access::State(controller_);
      const auto command = Command();
      const int state = Access::State(controller_);
      EXPECT_LE(command.linear.x, cap);
      const double distance = std::hypot(x_ - gx, y_ - gy);
      if (trace)
        trace << tick * .1 << ',' << x_ << ',' << y_ << ',' << yaw_ << ',' << v_ << ',' << w_ << ','
              << command.linear.x << ',' << command.angular.z << ',' << state << ','
              << Access::Index(controller_) << ',' << distance << '\n';
      if (!pivoted && state == 1 && command.linear.x > 0 && command.linear.x < .15)
        braked = true;
      if (!pivoted && state == 5)
      {
        EXPECT_LE(distance, 0.02);
        pivoted = true;
      }
      if ((state == 0 || state == 5) && std::abs(v_) > .01)
        EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
      if (pivoted && before == 5 && state == 1)
      {
        ++completed;
        if (completed == (trim >= 20 ? 1 : 2))
        {
          left = true;
          break;
        }
        pivoted = false;
        gx = 1;
        gy = sign * spacing;
        transform(gx, gy);
      }
      Advance(command, GetParam());
    }
    if (trim < 20)
      EXPECT_TRUE(braked);
    EXPECT_TRUE(pivoted);
    EXPECT_TRUE(left);
  }

  double x_{0}, y_{0}, yaw_{0}, v_{0}, w_{0};
};

// A resumed FOLLOWING state can leave the stop point beside the body. The
// ordinary driving floor must not sustain an orbit outside the arrival radius.
TEST_P(FtcPivotArrival, AsideCornerConvergesWithoutDrivingFloorOrbit)
{
  SetParameter("goal_timeout", 12.0);
  SetParameter("acceleration", .2);
  SetParameter("kp_ang_following", 1.0);
  SetParameter("kp_lat", .8);
  SetParameter("kd_lat", .5);
  SetParameter("derivative_filter_tau", .2);
  x_ = 1;
  y_ = -.06;
  yaw_ = M_PI;
  RobotTransform();
  controller_.newPathReceived(CornerPlan());
  Access::ApproachFixture(controller_, .06);
  bool pivoted = false, departed = false;
  for (int tick = 0; tick < 121; ++tick)
  {
    auto command = Command();
    EXPECT_GE(command.linear.x, 0);
    EXPECT_LE(command.linear.x, .3);
    if (Access::State(controller_) == 5)
    {
      EXPECT_LE(std::hypot(x_ - 1, y_), .02);
      pivoted = true;
    }
    if (pivoted && Access::State(controller_) == 1 && Access::Index(controller_) > 20)
    {
      departed = true;
      break;
    }
    command.angular.z = std::clamp(command.angular.z, -.7, .7);
    Advance(command, GetParam());
  }
  EXPECT_TRUE(pivoted);
  EXPECT_TRUE(departed);
}

TEST_P(FtcPivotArrival, RequiredHolePivotAndOpeningTwinRestartCheckTheOppositeArc)
{
  for (double reflection : {-1.0, 1.0})
  {
    geometry_msgs::msg::Twist command;
    for (bool required : {false, true})
    {
      // false uses newPathReceived([incoming, outgoing, next]) directly:
      // the real opening-twin PRE_ROTATE path after a precision restart.
      HolePivot(reflection, required);
      command = Command();
      EXPECT_DOUBLE_EQ(command.linear.x, 0);
      EXPECT_GT(reflection * command.angular.z, 0);
      ASSERT_TRUE(Access::CheckedTurn(controller_));
      EXPECT_GT(reflection * *Access::CheckedTurn(controller_), M_PI);
      EXPECT_LT(reflection * Access::GeometricError(controller_), 0)
          << "directed P target must not replace the original geometric error";
    }
    // With no short-arc blockage retain the old short direction, even if a
    // hypothetical full revolution would meet obstacles elsewhere.
    HolePivot(reflection, true, false, true);
    command = Command();
    EXPECT_LT(reflection * command.angular.z, 0);
    EXPECT_LT(std::abs(*Access::CheckedTurn(controller_)), M_PI);
  }
}

TEST_P(FtcPivotArrival, LocalObstacleOnlyOnLongArcBlocksAndKeepsFiniteFailure)
{
  HolePivot();
  auto* map = costmap_->getCostmap();
  map->resizeMap(2000, 2000, .005, -5, -5);
  std::fill_n(map->getCharMap(),
              map->getSizeInCellsX() * map->getSizeInCellsY(),
              nav2_costmap_2d::FREE_SPACE);
  unsigned int mx, my;
  ASSERT_TRUE(map->worldToMap(-2.65, 1.50, mx, my));
  map->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  SetParameter("obstacle_wait_timeout_s", .3);
  bool aborted = false;
  for (int i = 0; i < 8; ++i)
    try
    {
      const auto command = Command();
      EXPECT_DOUBLE_EQ(command.linear.x, 0);
      EXPECT_DOUBLE_EQ(command.angular.z, 0);
    }
    catch (const nav2_core::ControllerException&)
    {
      aborted = true;
      break;
    }
  EXPECT_TRUE(aborted);
}

TEST_P(FtcPivotArrival, LongPivotSettlingAndMissingGeometryKeepTheOriginalDeadline)
{
  SetParameter("goal_timeout", .3);
  for (bool missing_geometry : {false, true})
  {
    HolePivot();
    if (missing_geometry)
      Access::Geometry(controller_, {});
    else
      v_ = .15;
    bool aborted = false;
    for (int i = 0; i < 6; ++i)
      try
      {
        const auto command = Command();
        EXPECT_DOUBLE_EQ(command.linear.x, 0);
        EXPECT_DOUBLE_EQ(command.angular.z, 0);
      }
      catch (const nav2_core::ControllerException&)
      {
        aborted = true;
        break;
      }
    EXPECT_TRUE(aborted);
  }
}

TEST_P(FtcPivotArrival, OpeningTwinRestartKeepsBlockedAndSettlingDeadlines)
{
  SetParameter("goal_timeout", .3);
  for (int condition : {0, 1, 2})
  {
    HolePivot(1, false, true, condition == 2);  // real newPathReceived opening twin
    if (condition == 0)
      Access::Geometry(controller_, {});
    else if (condition == 1)
      v_ = .15;
    bool aborted = false;
    for (int i = 0; i < 6; ++i)
      try
      {
        const auto command = Command();
        EXPECT_DOUBLE_EQ(command.linear.x, 0);
        EXPECT_DOUBLE_EQ(command.angular.z, 0);
      }
      catch (const nav2_core::ControllerException&)
      {
        aborted = true;
        break;
      }
    EXPECT_TRUE(aborted);
  }
}

TEST_P(FtcPivotArrival, MowingAuthorityFlickerDoesNotStartObstacleDebounce)
{
  SetParameter("goal_timeout", 10.0);
  SetParameter("obstacle_wait_timeout_s", 2.5);
  SetParameter("obstacle_clear_hold_s", 1.5);
  for (bool required : {false, true})
  {
    HolePivot(1, required);
    x_ = -2.4284665686007214;
    y_ = 1.002289210706254;  // retained V7 supported TF, outside sole MOW
    yaw_ = .35889295088179485;
    const auto denied = Command();
    EXPECT_DOUBLE_EQ(denied.linear.x, 0);
    EXPECT_DOUBLE_EQ(denied.angular.z, 0);
    y_ = .998072653;  // next retained inside-authority observation
    const auto resumed = Command();
    EXPECT_DOUBLE_EQ(resumed.linear.x, 0);
    EXPECT_GT(resumed.angular.z, 0)
        << "positional-authority loss must not arm the physical-obstacle debounce";
  }
}

TEST_P(FtcPivotArrival, MissingMowingAuthorityRetainsTheOriginalRotationDeadline)
{
  SetParameter("goal_timeout", 10.0);
  SetParameter("obstacle_wait_timeout_s", 2.5);
  SetParameter("obstacle_clear_hold_s", 1.5);
  for (bool required : {false, true})
  {
    HolePivot(1, required);
    y_ = 1.002289210706254;
    const auto start = time_ns_;
    bool aborted = false;
    for (int i = 0; i < 105; ++i)
      try
      {
        const auto command = Command();
        EXPECT_DOUBLE_EQ(command.linear.x, 0);
        EXPECT_DOUBLE_EQ(command.angular.z, 0);
      }
      catch (const nav2_core::ControllerException& error)
      {
        aborted = true;
        EXPECT_EQ(std::string(error.what()).find("FTC_OBSTACLE_ABORT"), std::string::npos);
        EXPECT_GT(time_ns_ - start, 10000000000LL);
        EXPECT_LE(time_ns_ - start, 10200000000LL);
        break;
      }
    EXPECT_TRUE(aborted);
  }
}

TEST_P(FtcPivotArrival, ActiveObstacleDeadlineSurvivesMowingAuthorityLoss)
{
  SetParameter("goal_timeout", 10.0);
  SetParameter("obstacle_wait_timeout_s", 2.5);
  SetParameter("obstacle_clear_hold_s", 1.5);
  for (bool required : {false, true})
  {
    HolePivot(1, required, true, true);  // both actual body arcs blocked
    const auto first = Command();
    ASSERT_DOUBLE_EQ(first.linear.x, 0);
    ASSERT_DOUBLE_EQ(first.angular.z, 0);
    const auto armed = time_ns_;
    y_ = 1.002289210706254;
    bool aborted = false;
    for (int i = 0; i < 30; ++i)
      try
      {
        const auto command = Command();
        EXPECT_DOUBLE_EQ(command.linear.x, 0);
        EXPECT_DOUBLE_EQ(command.angular.z, 0);
      }
      catch (const nav2_core::ControllerException& error)
      {
        aborted = true;
        EXPECT_NE(std::string(error.what()).find("FTC_OBSTACLE_ABORT"), std::string::npos);
        EXPECT_LE(time_ns_ - armed, 2800000000LL);
        break;
      }
    EXPECT_TRUE(aborted);
  }
}

TEST_P(FtcPivotArrival, LongArcOutputRejectsAdversePidAndBoundsOscillation)
{
  for (bool integral : {false, true})
  {
    HolePivot();
    SetParameter("ki_ang", integral ? 100.0 : 0.0);
    SetParameter("kd_ang", integral ? 0.0 : 1.0);
    Access::AngularHistory(controller_, -10, 10);
    const auto command = Command();
    EXPECT_DOUBLE_EQ(command.linear.x, 0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0);
  }
  HolePivot();
  SetParameter("ki_ang", 0.0);
  SetParameter("kd_ang", 0.0);
  SetParameter("max_cmd_vel_ang", 10.0);
  SetParameter("oscillation_recovery", true);
  SetParameter("oscillation_recovery_min_duration", 0.0);
  Access::OscillatingHistory(controller_);
  const auto command = Command();
  EXPECT_GT(command.angular.z, 0);
  EXPECT_DOUBLE_EQ(command.linear.x, 0);
  ASSERT_TRUE(Access::CheckedTurn(controller_));
  EXPECT_LE(command.angular.z * .1, *Access::CheckedTurn(controller_));
}

TEST_P(FtcPivotArrival, GeometryReplacementBetweenLongArcCheckAndOutputSuppressesMotion)
{
  HolePivot();
  SetParameter("debug_pid", true);
  ASSERT_EQ(rcutils_logging_set_logger_level(node_->get_logger().get_name(),
                                             RCUTILS_LOG_SEVERITY_DEBUG),
            RCUTILS_RET_OK);
  bool replaced = false;
  geometry_msgs::msg::Twist command;
  {
    DepartureInterleave interleave(
        [&]()
        {
          Access::Geometry(controller_, {});
          replaced = true;
        },
        -1);
    command = Command();
  }
  EXPECT_EQ(rcutils_logging_set_logger_level(node_->get_logger().get_name(),
                                             RCUTILS_LOG_SEVERITY_INFO),
            RCUTILS_RET_OK);
  ASSERT_TRUE(replaced);
  EXPECT_DOUBLE_EQ(command.linear.x, 0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0);
}

TEST_P(FtcPivotArrival, CapturedInteriorTangentMicrometresUseRadialReserve)
{
  const double gx = -1.3250370444641841, gy = -0.67902344338509446;
  const double incoming = -1.5737952311265353, outgoing = 3.138593749258152;
  x_ = -1.3249271811646266;
  y_ = -0.64721615459390824;
  yaw_ = -1.5857167321048689;
  RobotTransform();
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  plan.poses = {MakePose(gx - .3 * std::cos(incoming), gy - .3 * std::sin(incoming), incoming),
                MakePose(gx, gy, incoming),
                MakePose(gx, gy, outgoing),
                MakePose(gx + .3 * std::cos(outgoing), gy + .3 * std::sin(outgoing), outgoing)};
  Access::Geometry(controller_, MowingBox(-3, -1, -1, 1));
  controller_.newPathReceived(plan);
  Access::ApproachFixture(controller_, std::hypot(gx - x_, gy - y_));
  // kp_lon=1 and I/D=0: preserve the unfloored body-longitudinal request.
  const double request = std::max(0.0, (gx - x_) * std::cos(yaw_) + (gy - y_) * std::sin(yaw_));
  const double allowed = std::min(request, std::hypot(gx - x_, gy - y_) / .8);
  const auto command = Command();
  EXPECT_NEAR(command.linear.x, allowed, 1e-12);
  EXPECT_LE(command.linear.x, std::hypot(gx - x_, gy - y_) / .8);
  EXPECT_EQ(Access::State(controller_), 1);
  const auto cached = Access::ApproachLimits(controller_);
  EXPECT_NEAR(cached.first, .0015178746472512962, 1e-12);
  EXPECT_NEAR(cached.second, .0017347138825729101, 1e-12);
  // Select the matching radial response reserve before testing feedback.
  v_ = .01;
  EXPECT_NEAR(Command().linear.x, allowed, 1e-12);
  v_ = .05;
  EXPECT_DOUBLE_EQ(Command().linear.x, 0);
  v_ = 0;
  Access::Geometry(controller_, MowingBox(x_ - .3, x_ + .0005, -1, 1));
  EXPECT_NEAR(Command().linear.x, cached.first, 1e-12);
  auto hole = MowingBox(-3, -1, -1, 1);
  auto blocked = MowingBox(x_ + .1, x_ + .101, y_ + .1, y_ + .101).markers.front();
  blocked.ns = "obstacle";
  hole.markers.push_back(blocked);
  Access::Geometry(controller_, hole);
  EXPECT_NEAR(Command().linear.x, cached.first, 1e-12);
  Access::Geometry(controller_, {});
  EXPECT_NEAR(Command().linear.x, cached.first, 1e-12);
  // Geometry qualification may alternate, but cannot renew this corner's clock.
  SetParameter("goal_timeout", 1.0);
  bool aborted = false;
  for (int i = 0; i < 15; ++i)
  {
    Access::Geometry(controller_, i % 2 ? MowingBox(-3, -1, -1, 1) : hole);
    try
    {
      (void)Command();
    }
    catch (const nav2_core::ControllerException&)
    {
      aborted = true;
      break;
    }
  }
  EXPECT_TRUE(aborted);
}

TEST_P(FtcPivotArrival, CapturedInteriorConvergesWithinExistingApproachDeadline)
{
  const double gx = -1.3250370444641841, gy = -0.67902344338509446;
  const double incoming = -1.5737952311265353, outgoing = 3.138593749258152;
  x_ = -1.3249271811646266;
  y_ = -0.64721615459390824;
  yaw_ = -1.5857167321048689;
  RobotTransform();
  SetParameter("goal_timeout", 5.0);
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  plan.poses = {MakePose(gx - .3 * std::cos(incoming), gy - .3 * std::sin(incoming), incoming),
                MakePose(gx, gy, incoming),
                MakePose(gx, gy, outgoing),
                MakePose(gx + .3 * std::cos(outgoing), gy + .3 * std::sin(outgoing), outgoing)};
  Access::Geometry(controller_, MowingBox(-3, -1, -1, 1));
  controller_.newPathReceived(plan);
  Access::ApproachFixture(controller_, std::hypot(gx - x_, gy - y_));
  bool arrived = false;
  for (int tick = 0; tick < 50; ++tick)
  {
    const auto command = Command();
    if (Access::State(controller_) == 5)
    {
      EXPECT_LE(std::hypot(gx - x_, gy - y_), .02);
      arrived = true;
      break;
    }
    Advance(command, GetParam());
    EXPECT_GE(x_, -3 - 1e-8);
    EXPECT_LE(x_, -1 + 1e-8);
    EXPECT_GE(y_, -1 - 1e-8);
    EXPECT_LE(y_, 1 + 1e-8);
  }
  EXPECT_TRUE(arrived);
}

TEST_P(FtcPivotArrival, CapturedOuterChamferUsesCertifiedClearanceReserve)
{
  const double gx = -2.9940096759401831, gy = 0.9973782762600798;
  const double incoming = 1.5677974224632547, outgoing = 0.78539816339736279;
  x_ = -2.9793589009967589;
  y_ = 0.98231435823658197;
  yaw_ = 2.3052855759549207;
  RobotTransform();
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  plan.poses = {MakePose(gx - .3 * std::cos(incoming), gy - .3 * std::sin(incoming), incoming),
                MakePose(gx, gy, incoming),
                MakePose(gx, gy, outgoing),
                MakePose(gx + .00367076 * std::cos(outgoing),
                         gy + .00367076 * std::sin(outgoing),
                         outgoing),
                MakePose(gx + .1, gy, 0)};
  Access::Geometry(controller_, MowingBox(-3, -1, -1, 1));
  controller_.newPathReceived(plan);
  Access::ApproachFixture(controller_, std::hypot(gx - x_, gy - y_));
  const double request = std::max(0.0, (gx - x_) * std::cos(yaw_) + (gy - y_) * std::sin(yaw_));
  const auto command = Command();
  const double budget = std::nextafter(1 - y_ - mowgli_nav2_plugins::transit::kEpsilon, 0);
  EXPECT_NEAR(command.linear.x, std::min(request, budget / .8), 1e-12);
  EXPECT_LE(command.linear.x, budget / .8);
  EXPECT_EQ(Access::State(controller_), 1);
  const auto cached = Access::ApproachLimits(controller_);
  EXPECT_NEAR(cached.first, .0071761130929037797, 1e-12);
  v_ = .01;  // above B's reserve, below the new matching reserve
  EXPECT_NEAR(Command().linear.x, std::min(request, budget / .8), 1e-12);
  v_ = .03;
  EXPECT_DOUBLE_EQ(Command().linear.x, 0);
  v_ = 0;
  controller_.setSpeedLimit(.01, false);
  EXPECT_DOUBLE_EQ(Command().linear.x, .01);
  controller_.setSpeedLimit(0, false);
  Access::Geometry(controller_, MowingBox(-3, -1, -1, y_ + .002));
  EXPECT_NEAR(Command().linear.x, cached.first, 1e-12);
  // The earlier 49s sample also changes, before the deadline is nearly spent.
  x_ = -2.966923554580374;
  y_ = 0.9672229516468057;
  yaw_ = 2.232941996005764;
  RobotTransform();
  Access::Geometry(controller_, MowingBox(-3, -1, -1, 1));
  controller_.newPathReceived(plan);
  Access::ApproachFixture(controller_, std::hypot(gx - x_, gy - y_));
  const double early_budget = std::nextafter(1 - y_ - mowgli_nav2_plugins::transit::kEpsilon, 0);
  const double early_request =
      std::max(0.0, (gx - x_) * std::cos(yaw_) + (gy - y_) * std::sin(yaw_));
  EXPECT_NEAR(Command().linear.x, std::min(early_request, early_budget / .8), 1e-12);
}

TEST_P(FtcPivotArrival, ApproachTimerUsesBodyAndCannotBeResetByRetreat)
{
  SetParameter("goal_timeout", 10.0);
  controller_.newPathReceived(CornerPlan());
  Access::ApproachFixture(controller_, .41);  // carrot is at the corner
  Access::ApproachStateTick(controller_);
  EXPECT_FALSE(Access::ApproachTimerArmed(controller_));
  time_ns_ += 11000000000LL;
  SetTime();
  EXPECT_NO_THROW(Access::ApproachStateTick(controller_));
  Access::ApproachFixture(controller_, .23);
  Access::ApproachStateTick(controller_);
  ASSERT_TRUE(Access::ApproachTimerArmed(controller_));
  time_ns_ += 6000000000LL;
  SetTime();
  Access::ApproachFixture(controller_, .41);
  EXPECT_NO_THROW(Access::ApproachStateTick(controller_));
  time_ns_ += 4100000000LL;
  SetTime();
  EXPECT_THROW(Access::ApproachStateTick(controller_), nav2_core::ControllerException);
  Access::ApproachFixture(controller_, .41, true);
  EXPECT_NO_THROW(Access::ApproachStateTick(controller_));
  EXPECT_FALSE(Access::ApproachTimerArmed(controller_));
  Access::ApproachFixture(controller_, .23, true);
  Access::ApproachStateTick(controller_);
  ASSERT_TRUE(Access::ApproachTimerArmed(controller_));
  controller_.newPathReceived(CornerPlan());
  EXPECT_FALSE(Access::ApproachTimerArmed(controller_));
}

TEST_P(FtcPivotArrival, FallbackReplacementCannotInheritSameIndexApproachDeadline)
{
  SetParameter("goal_timeout", 10.0);
  auto plan = CornerPlan();
  for (auto& pose : plan.poses)
    pose.pose.position.x *= .925;
  yaw_ = -M_PI / 2;
  RobotTransform();
  controller_.newPathReceived(plan);
  Access::ApproachFixture(controller_, .23);
  Access::ApproachStateTick(controller_);
  ASSERT_TRUE(Access::ApproachTimerArmed(controller_));
  time_ns_ += 9000000000LL;
  SetTime();
  ASSERT_TRUE(Access::SpliceApproachFallback(controller_));
  EXPECT_FALSE(Access::ApproachTimerArmed(controller_));
  time_ns_ += 2000000000LL;
  SetTime();
  Access::LeaveFallbackInitialPivot(controller_);
  EXPECT_NO_THROW(Access::ApproachStateTick(controller_));
  EXPECT_FALSE(Access::ApproachTimerArmed(controller_));
}

TEST_P(FtcPivotArrival, UnsupportedFrameAndExternalCapKeepFiniteBudgets)
{
  SetParameter("goal_timeout", 10.0);
  auto plan = CornerPlan();
  plan.header.frame_id = "odom";
  controller_.newPathReceived(plan);
  Access::ApproachFixture(controller_, .41);
  Access::ApproachStateTick(controller_);
  ASSERT_TRUE(Access::ApproachTimerArmed(controller_));
  time_ns_ += 10100000000LL;
  SetTime();
  EXPECT_THROW(Access::ApproachStateTick(controller_), nav2_core::ControllerException);

  controller_.newPathReceived(CornerPlan());
  controller_.setSpeedLimit(.1, false);  // window is .08 m at this cap
  Access::ApproachFixture(controller_, .1);
  Access::ApproachStateTick(controller_);
  EXPECT_FALSE(Access::ApproachTimerArmed(controller_));
  Access::ApproachFixture(controller_, .07);
  Access::ApproachStateTick(controller_);
  EXPECT_TRUE(Access::ApproachTimerArmed(controller_));
}

TEST_P(FtcPivotArrival, FiniteBoundaryDepartureAndStoppedCornerResume)
{
  for (bool pivot : {false, true})
    for (double sign : {-1.0, 1.0})
    {
      Access::Geometry(controller_, MowingBox(-3, -1, sign < 0 ? -1 : -3, sign < 0 ? 3 : 1));
      x_ = -3;
      y_ = sign;
      yaw_ = sign * (pivot ? M_PI / 2 : .1);
      v_ = 0;
      w_ = 0;
      nav_msgs::msg::Path plan;
      plan.header.frame_id = "map";
      if (pivot)
        for (int i = 0; i <= 20; ++i)
          plan.poses.push_back(MakePose(-3, sign * (.8 + .01 * i), yaw_));
      for (int i = 0; i < 120; ++i)
        plan.poses.push_back(MakePose(-3 + .005 * i, sign, 0));
      RobotTransform();
      controller_.newPathReceived(plan);
      bool saw_pivot = false, progress = false;
      for (int tick = 0; tick < 150; ++tick)
      {
        const auto command = Command();
        saw_pivot |= Access::State(controller_) == 5;
        EXPECT_GE(1 - sign * y_, -1e-8);
        EXPECT_GE(x_ + 3, -1e-8);
        Advance(command, GetParam());
        if (x_ > -2.8)
        {
          progress = true;
          break;
        }
      }
      EXPECT_TRUE(progress);
      if (pivot)
        EXPECT_TRUE(saw_pivot);
    }
}

TEST_P(FtcPivotArrival, MissingRoleAndPermanentDenialKeepOriginalDeadline)
{
  SetParameter("goal_timeout", 1.0);
  for (int invalid = 0; invalid < 4; ++invalid)
  {
    auto geometry = MowingBox(-3, -1, -1, 1);
    if (invalid == 0)
      geometry.markers.clear();
    if (invalid == 1)
      geometry.markers[0].text = "navigation";
    if (invalid == 2)
      geometry.markers[0].text = "";
    // A complete valid region that denies the actual axle is a permanent denial.
    if (invalid == 3)
      geometry = MowingBox(2, 4, 2, 4);
    Access::Geometry(controller_, geometry);
    x_ = -3;
    y_ = 1;
    yaw_ = .1;
    v_ = 0;
    w_ = 0;
    RobotTransform();
    controller_.newPathReceived(StraightPlan(-3));
    bool aborted = false;
    for (int i = 0; i < 15; ++i)
    {
      try
      {
        const auto c = Command();
        EXPECT_DOUBLE_EQ(c.linear.x, 0);
        EXPECT_DOUBLE_EQ(c.angular.z, 0);
      }
      catch (const nav2_core::ControllerException&)
      {
        aborted = true;
        break;
      }
    }
    EXPECT_TRUE(aborted);
  }
}

TEST_P(FtcPivotArrival, GeometryReplacementCannotInterleavePivotDeparture)
{
  auto revoked = MowingBox();
  revoked.markers.front().text = "navigation";
  controller_.newPathReceived(CornerPlan());
  bool attempted = false, replaced = false;
  geometry_msgs::msg::Twist command;
  {
    DepartureInterleave interleave(
        [&]()
        {
          std::thread executor(
              [&]()
              {
                attempted = true;
                replaced = Access::TryGeometry(controller_, revoked);
              });
          executor.join();
        },
        5);
    for (int i = 0; i < 1000 && !attempted; ++i)
    {
      command = Command();
      Advance(command, GetParam());
    }
  }
  ASSERT_TRUE(attempted);
  EXPECT_FALSE(replaced) << "revocation preceded a pivot departure command=" << command.linear.x;
  EXPECT_EQ(Access::State(controller_), 1);
  EXPECT_GT(command.linear.x, 0);
}

TEST_P(FtcPivotArrival, GeometryReplacementAndNewPlanRecomputeDeparture)
{
  x_ = -3;
  y_ = 1;
  yaw_ = .1;
  v_ = 0;
  w_ = 0;
  Access::Geometry(controller_, MowingBox(-3, -1, -1, 1));
  RobotTransform();
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i < 120; ++i)
    plan.poses.push_back(MakePose(-3 + .005 * i, 1, 0));
  controller_.newPathReceived(plan);
  auto c = Command();
  EXPECT_EQ(Access::State(controller_), 0);
  EXPECT_LT(c.angular.z, 0);
  EXPECT_DOUBLE_EQ(c.linear.x, 0);
  Access::Geometry(controller_, {});
  c = Command();
  EXPECT_DOUBLE_EQ(c.angular.z, 0);
  EXPECT_DOUBLE_EQ(c.linear.x, 0);
  // New plan and a different pose revoke the previous effective target.
  Access::Geometry(controller_, MowingBox());
  x_ = 0;
  y_ = 0;
  yaw_ = 0;
  RobotTransform();
  controller_.newPathReceived(StraightPlan());
  c = Command();
  EXPECT_EQ(Access::State(controller_), 1);
}

TEST_P(FtcPivotArrival, WholeSelectedArcAndBothBlockedRecovery)
{
  SetParameter("max_goal_angle_error", 30.0);
  for (bool local : {false, true})
  {
    auto geometry = MowingBox(-3, -1, -1, 1);
    auto triangle = geometry.markers.front();
    triangle.points.clear();
    for (auto xy : std::vector<std::pair<double, double>>{{-3, 1}, {-1, 1.4}, {-1, 1.8}})
    {
      geometry_msgs::msg::Point p;
      p.x = xy.first;
      p.y = xy.second;
      triangle.points.push_back(p);
    }
    geometry.markers.push_back(triangle);
    if (local)
    {
      auto* map = costmap_->getCostmap();
      map->resizeMap(2000, 2000, .005, -5, -5);
      unsigned int mx, my;
      ASSERT_TRUE(map->worldToMap(-2.57, .630, mx, my));
      map->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
    else
    {
      auto hole = MowingBox(-2.58, -2.56, .62, .64).markers.front();
      hole.ns = "obstacle";
      geometry.markers.push_back(hole);
    }
    Access::Geometry(controller_, geometry);
    x_ = -3;
    y_ = 1;
    yaw_ = .1;
    v_ = 0;
    w_ = 0;
    nav_msgs::msg::Path plan;
    plan.header.frame_id = "map";
    for (int i = 0; i < 120; ++i)
      plan.poses.push_back(MakePose(-3 + .005 * i, 1, 0));
    RobotTransform();
    controller_.newPathReceived(plan);
    const auto c = Command();
    EXPECT_DOUBLE_EQ(c.linear.x, 0);
    EXPECT_GT(c.angular.z, 0);
    if (local)
      costmap_->getCostmap()->resetMap(0, 0, 2000, 2000);
    else
    {
      auto hole = MowingBox(-2.56, -2.52, 1.37, 1.40).markers.front();
      hole.ns = "obstacle";
      geometry.markers.push_back(hole);
      Access::Geometry(controller_, geometry);
      controller_.newPathReceived(plan);
      SetParameter("obstacle_wait_timeout_s", .5);
      bool aborted = false;
      for (int tick = 0; tick < 10; ++tick)
      {
        try
        {
          const auto c = Command();
          EXPECT_DOUBLE_EQ(c.linear.x, 0);
          EXPECT_DOUBLE_EQ(c.angular.z, 0);
        }
        catch (const nav2_core::ControllerException&)
        {
          aborted = true;
          break;
        }
      }
      EXPECT_TRUE(aborted);
    }
  }
}

TEST_P(FtcPivotArrival, AdversePidAndOscillationStayInsideSelectedDirectionAndArc)
{
  SetParameter("max_goal_angle_error", 30.0);
  Access::Geometry(controller_, MowingBox(-3, -1, -1, 1));
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i < 120; ++i)
    plan.poses.push_back(MakePose(-3 + .005 * i, 1, 0));
  x_ = -3;
  y_ = 1;
  yaw_ = .1;
  RobotTransform();
  for (bool integral : {false, true})
  {
    controller_.newPathReceived(plan);
    SetParameter("ki_ang", integral ? 1.0 : 0.0);
    SetParameter("kd_ang", integral ? 0.0 : 1.0);
    Access::AngularHistory(controller_, 10, -10);
    const auto c = Command();
    EXPECT_DOUBLE_EQ(c.angular.z, 0);
    EXPECT_DOUBLE_EQ(c.linear.x, 0);
  }
  controller_.newPathReceived(plan);
  SetParameter("ki_ang", 0.0);
  SetParameter("kd_ang", 0.0);
  SetParameter("oscillation_recovery", true);
  SetParameter("max_cmd_vel_ang", 4.0);
  SetParameter("oscillation_recovery_min_duration", 0.0);
  Access::OscillatingHistory(controller_);
  const auto c = Command();
  EXPECT_LT(c.angular.z, 0);
  EXPECT_DOUBLE_EQ(c.linear.x, 0);
  EXPECT_LE(std::abs(c.angular.z) * .1, M_PI / 12 + .1);
}

TEST_P(FtcPivotArrival, OriginalBandAndSupportedPoseEpochRemainAuthoritative)
{
  Access::Geometry(controller_, MowingBox(-3, -1, -1, 1));
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "map";
  for (int i = 0; i < 120; ++i)
    plan.poses.push_back(MakePose(-3 + .005 * i, 1, 0));
  SetParameter("max_goal_angle_error", 30.0);
  x_ = -3;
  y_ = 1;
  yaw_ = .1;
  RobotTransform();
  controller_.newPathReceived(plan);
  auto c = Command();
  EXPECT_EQ(Access::State(controller_), 0);
  EXPECT_LT(c.angular.z, 0);
  // A future inward forecast cannot hide the supported outward actual pose.
  time_ns_ += 100000000;
  x_ = -2.9;
  yaw_ = -.1;
  RobotTransform();
  time_ns_ -= 100000000;
  x_ = -3;
  yaw_ = .1;
  c = controller_.computeVelocityCommands(Pose{}, {}, nullptr, {}, Pose{}).twist;
  EXPECT_EQ(Access::State(controller_), 0);
  EXPECT_DOUBLE_EQ(c.linear.x, 0);
  EXPECT_LT(c.angular.z, 0);
  // The previous finite target does not change the original completion band.
  yaw_ = -.7;
  c = Command();
  EXPECT_EQ(Access::State(controller_), 0);
  EXPECT_GT(c.angular.z, 0);
  yaw_ = -.1;
  c = Command();
  EXPECT_EQ(Access::State(controller_), 1);
}

TEST_P(FtcPivotArrival, BrakesAndReachesRadialTipLeft)
{
  RunArrival(1);
}
TEST_P(FtcPivotArrival, BrakesAndReachesRadialTipRight)
{
  RunArrival(-1);
}
TEST_P(FtcPivotArrival, RigidTransformPreservesArrival)
{
  RunArrival(1, .7);
}
TEST_P(FtcPivotArrival, CloseConsecutiveCorners)
{
  RunArrival(1, 0, .00367);
}
TEST_P(FtcPivotArrival, ResumedIncomingTwin)
{
  RunArrival(1, 0, .6, 20);
}
TEST_P(FtcPivotArrival, ResumedOutgoingTwin)
{
  RunArrival(1, 0, .6, 21);
}

TEST_P(FtcPivotArrival, TranslationHoldsPreRotateUntilStopped)
{
  auto plan = CornerPlan();
  controller_.newPathReceived(plan);
  v_ = .15;
  yaw_ = -.5;
  const auto command = Command();
  EXPECT_DOUBLE_EQ(command.linear.x, 0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0);
  EXPECT_EQ(Access::State(controller_), 0);
}

TEST_P(FtcPivotArrival, InterruptedIncomingApproach)
{
  // New path begins halfway along the incoming leg with an actual lateral
  // error; PRE_ROTATE must align with the tip before handing back to FOLLOWING.
  RunArrival(1, 0, .6, 18);
}

TEST_P(FtcPivotArrival, ExternalCapWinsDuringApproach)
{
  controller_.setSpeedLimit(.03, false);
  RunArrival(1, 0, .6, 18, .03);
}

TEST_P(FtcPivotArrival, SettlingCannotResetPreRotateDeadline)
{
  SetParameter("goal_timeout", .3);
  controller_.newPathReceived(CornerPlan());
  v_ = .15;
  for (int i = 0; i < 3; ++i)
  {
    const auto command = Command();
    EXPECT_DOUBLE_EQ(command.linear.x, 0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0);
  }
  EXPECT_THROW(Command(), nav2_core::ControllerException);
}

TEST_P(FtcPivotArrival, StationaryLateralMissNeverCountsAsArrivalAndTimesOut)
{
  auto plan = CornerPlan();
  plan.poses.erase(plan.poses.begin(), plan.poses.begin() + 18);
  x_ = 1;
  y_ = -.125;
  yaw_ = M_PI / 2;
  RobotTransform();
  controller_.newPathReceived(plan);
  bool threw = false;
  for (int i = 0; i < 350; ++i)
  {
    try
    {
      Command();  // blocked plant: measured zero, pose unchanged
      EXPECT_NE(Access::State(controller_), 5);
    }
    catch (const nav2_core::ControllerException&)
    {
      threw = true;
      break;
    }
  }
  EXPECT_TRUE(threw);
}

TEST_P(FtcPivotArrival, MissingAndStaleTfRefuseApproach)
{
  auto plan = CornerPlan();
  plan.poses.erase(plan.poses.begin(), plan.poses.begin() + 16);
  x_ = .8;
  RobotTransform();
  controller_.newPathReceived(plan);
  time_ns_ += 1000000000;
  SetTime();
  EXPECT_THROW(controller_.computeVelocityCommands(
                   Pose{}, geometry_msgs::msg::Twist{}, nullptr, nav_msgs::msg::Path{}, Pose{}),
               nav2_core::ControllerException);
  tf_->clear();
  EXPECT_THROW(controller_.computeVelocityCommands(
                   Pose{}, geometry_msgs::msg::Twist{}, nullptr, nav_msgs::msg::Path{}, Pose{}),
               nav2_core::ControllerException);
}

TEST_P(FtcPivotArrival, WrongPathFrameRefusesPivot)
{
  auto plan = CornerPlan();
  plan.poses.erase(plan.poses.begin(), plan.poses.begin() + 20);
  plan.header.frame_id = "odom";
  for (auto& p : plan.poses)
    p.header.frame_id = "odom";
  controller_.newPathReceived(plan);
  EXPECT_THROW(Command(), nav2_core::ControllerException);
}

TEST_P(FtcPivotArrival, FutureTfDoesNotForecastTheApproachCap)
{
  auto plan = CornerPlan();
  plan.poses.erase(plan.poses.begin(), plan.poses.begin() + 18);
  x_ = .96;
  RobotTransform();
  controller_.newPathReceived(plan);
  controller_.computeVelocityCommands(
      Pose{}, geometry_msgs::msg::Twist{}, nullptr, nav_msgs::msg::Path{}, Pose{});
  time_ns_ += 100000000;
  SetTime();
  RobotTransform();
  const auto now = time_ns_;
  time_ns_ += 100000000;
  x_ = .999;
  RobotTransform();
  time_ns_ = now;
  SetTime();
  const auto command = controller_
                           .computeVelocityCommands(Pose{},
                                                    geometry_msgs::msg::Twist{},
                                                    nullptr,
                                                    nav_msgs::msg::Path{},
                                                    Pose{})
                           .twist;
  // Current longitudinal request .04, below the .05 response cap; not future .001.
  EXPECT_NEAR(command.linear.x, .04, 1e-12);
}

TEST_P(FtcPivotArrival, TranslationSettlingRetainsPivotDeadline)
{
  controller_.newPathReceived(CornerPlan());
  bool pivoted = false;
  for (int i = 0; i < 300; ++i)
  {
    const auto command = Command();
    if (Access::State(controller_) == 5)
    {
      pivoted = true;
      break;
    }
    Advance(command, GetParam());
  }
  ASSERT_TRUE(pivoted);
  SetParameter("goal_timeout", .3);
  v_ = .15;  // persistent measured translation; rotation must remain inhibited
  for (int i = 0; i < 2; ++i)
  {
    const auto command = Command();
    EXPECT_DOUBLE_EQ(command.linear.x, 0);
    EXPECT_DOUBLE_EQ(command.angular.z, 0);
    EXPECT_EQ(Access::State(controller_), 5);
  }
  Command();
  EXPECT_THROW(Command(), nav2_core::ControllerException);
}

TEST_P(FtcPivotArrival, OneTickOdometryNoiseDoesNotStutterASettledPivot)
{
  // Field 2026-11-09: an in-place pivot reads +-0.027 m/s on the wheel odometry
  // (one tick of difference between the wheels). Every such sample used to zero
  // the rotation command; 6 of 13 pivots then hit the 10 s timeout.
  controller_.newPathReceived(CornerPlan());
  bool pivoted = false;
  for (int i = 0; i < 300; ++i)
  {
    const auto command = Command();
    if (Access::State(controller_) == 5)
    {
      pivoted = true;
      break;
    }
    Advance(command, GetParam());
  }
  ASSERT_TRUE(pivoted);
  v_ = 0;
  Command();  // translation observed settled in this PIVOT
  for (int i = 0; i < 4; ++i)
  {
    v_ = (i % 2 == 0) ? .027 : -.027;
    const auto command = Command();
    if (Access::State(controller_) != 5)
      break;  // the turn may already be complete
    EXPECT_NE(command.angular.z, 0) << "tick " << i;
  }
  // Real translation after the settle still inhibits the rotation.
  if (Access::State(controller_) == 5)
  {
    v_ = .15;
    const auto command = Command();
    EXPECT_DOUBLE_EQ(command.angular.z, 0);
  }
}

TEST_P(FtcPivotArrival, ObstacleOverlayRetainsArrivalAndSettling)
{
  SetParameter("enable_obstacle_deviation", true);
  RunArrival(1);
}

TEST_F(FtcSpeedLimit, InvalidStartupPivotCoastAssumptionIsRejected)
{
  auto invalid_node = std::make_shared<nav2::LifecycleNode>("invalid_controller", "");
  invalid_node->declare_parameter(std::string(kPlugin) + ".pivot_coast_time_bound_s", 0.0);
  mowgli_nav2_plugins::FTCController invalid;
  EXPECT_THROW(invalid.configure(invalid_node, kPlugin, tf_, costmap_),
               nav2_core::ControllerException);
}

TEST_F(FtcSpeedLimit, PivotCoastAssumptionHasValidatedDynamicRange)
{
  const std::string name = std::string(kPlugin) + ".pivot_coast_time_bound_s";
  EXPECT_DOUBLE_EQ(node_->get_parameter(name).as_double(), .5);
  for (double invalid : {0.0,
                         -.1,
                         2.01,
                         std::numeric_limits<double>::quiet_NaN(),
                         std::numeric_limits<double>::infinity()})
    EXPECT_FALSE(node_->set_parameter({name, invalid}).successful);
  for (double valid : {.01, .5, 2.0})
    EXPECT_TRUE(node_->set_parameter({name, valid}).successful);
}

INSTANTIATE_TEST_SUITE_P(DeclaredPlants, FtcPivotArrival, ::testing::Values(0.0, 0.07));
}  // namespace
