// SPDX-License-Identifier: GPL-3.0-or-later
// Exercise the installed Nav2 shim and its actual RPP fallback. The vendor
// patch is deliberately outside this package; an unpatched apt shim fails.
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav2_rotation_shim_controller/nav2_rotation_shim_controller.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

namespace
{
class Shim : public nav2_rotation_shim_controller::RotationShimController
{
public:
  void previousCommand(double omega)
  {
    last_angular_vel_ = omega;
  }
};

class GoalChecker : public nav2_core::GoalChecker
{
public:
  bool xy_reached = true;
  void initialize(const nav2::LifecycleNode::WeakPtr&,
                  const std::string&,
                  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
  }
  void reset() override
  {
  }
  bool isGoalReached(const geometry_msgs::msg::Pose&,
                     const geometry_msgs::msg::Pose&,
                     const geometry_msgs::msg::Twist&,
                     const nav_msgs::msg::Path&) override
  {
    return false;
  }
  bool isGoalXYReached(const geometry_msgs::msg::Pose&,
                       const geometry_msgs::msg::Pose&,
                       const geometry_msgs::msg::Twist&,
                       const nav_msgs::msg::Path&) override
  {
    return xy_reached;
  }
  bool getTolerances(geometry_msgs::msg::Pose& pose, geometry_msgs::msg::Twist&, double&) override
  {
    pose.position.x = pose.position.y = 0.03;
    return true;
  }
};

class RotationShimCollision : public testing::Test
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
    node_ = std::make_shared<nav2::LifecycleNode>("rotation_shim_collision");
    node_->declare_parameter("controller_frequency", 10.0);
    node_->declare_parameter("FollowPath.primary_controller.plugin",
                             std::string("nav2_regulated_pure_pursuit_controller::"
                                         "RegulatedPurePursuitController"));
    node_->declare_parameter("FollowPath.angular_dist_threshold", 0.785);
    node_->declare_parameter("FollowPath.angular_disengage_threshold", 0.3925);
    node_->declare_parameter("FollowPath.rotate_to_heading_angular_vel", 0.75);
    node_->declare_parameter("FollowPath.max_angular_accel", 2.5);
    node_->declare_parameter("FollowPath.simulate_ahead_time", 1.0);
    node_->declare_parameter("FollowPath.rotate_to_goal_heading", true);
    node_->declare_parameter("FollowPath.use_path_orientations", true);
    node_->declare_parameter("FollowPath.closed_loop", false);
    node_->declare_parameter("FollowPath.primary_controller.use_rotate_to_heading", false);
    auto options = rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("plugins", std::vector<std::string>{}),
         rclcpp::Parameter("filters", std::vector<std::string>{}),
         rclcpp::Parameter("global_frame", "odom"),
         rclcpp::Parameter("robot_base_frame", "base_link"),
         rclcpp::Parameter("footprint",
                           "[[0.53,0.275],[0.53,-0.275],[-0.17,-0.275],[-0.17,0.275]]"),
         rclcpp::Parameter("footprint_padding", 0.0)});
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(options);
    costmap_->on_configure(rclcpp_lifecycle::State());
    costmap_->getCostmap()->resizeMap(2000, 2000, 0.002, -2.0, -2.0);
    shim_.configure(node_, "FollowPath", costmap_->getTfBuffer(), costmap_);
    shim_.activate();
    pose_.header.frame_id = "odom";
    pose_.pose.orientation.w = 1.0;
  }

  void TearDown() override
  {
    shim_.deactivate();
    shim_.cleanup();
    costmap_->on_cleanup(rclcpp_lifecycle::State());
  }

  void clear()
  {
    costmap_->getCostmap()->resetMap(0, 0, 2000, 2000);
    shim_.reset();
  }

  void mark(double x, double y)
  {
    // A small patch remains visible to the footprint edge rasterizer at the
    // selected sample instead of depending on one cell's rounding.
    for (int dx = -3; dx <= 3; ++dx)
      for (int dy = -3; dy <= 3; ++dy)
      {
        unsigned int mx, my;
        ASSERT_TRUE(costmap_->getCostmap()->worldToMap(x + dx * 0.001, y + dy * 0.001, mx, my));
        costmap_->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
  }

  double footprintCost(double yaw)
  {
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> checker(
        costmap_->getCostmap());
    return checker.footprintCostAtPose(0.0, 0.0, yaw, costmap_->getRobotFootprint());
  }

  geometry_msgs::msg::Twist command(double heading, size_t path_size = 2)
  {
    auto goal = pose_;
    goal.pose.orientation.z = std::sin(heading / 2.0);
    goal.pose.orientation.w = std::cos(heading / 2.0);
    nav_msgs::msg::Path path;
    path.header.frame_id = "base_link";
    path.poses.resize(path_size, goal);
    for (auto& point : path.poses)
      point.header.frame_id = "base_link";
    return shim_.computeVelocityCommands(pose_, geometry_msgs::msg::Twist(), &checker_, path, goal)
        .twist;
  }

  // Test the public output: the unpatched shim catches collision refusal and
  // may enter RPP, which can still command forward motion on this path.
  void expectHoldOrRefusal(double heading)
  {
    try
    {
      auto result = command(heading);
      EXPECT_DOUBLE_EQ(result.linear.x, 0.0);
      EXPECT_DOUBLE_EQ(result.linear.y, 0.0);
      EXPECT_DOUBLE_EQ(result.angular.z, 0.0);
    }
    catch (const nav2_core::ControllerException&)
    {
    }
  }

  std::shared_ptr<nav2::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  Shim shim_;
  GoalChecker checker_;
  geometry_msgs::msg::PoseStamped pose_;
};

TEST_F(RotationShimCollision, TerminalAlignmentChecksBothDirectionsAndClearControls)
{
  for (int sign : {-1, 1})
  {
    SCOPED_TRACE(sign);
    clear();
    EXPECT_GT(sign * command(sign * 0.4).angular.z, 0.0);
    clear();
    mark(0.55, -sign * 0.20);
    ASSERT_EQ(footprintCost(0.0), 0.0);
    ASSERT_EQ(footprintCost(sign * 0.1), nav2_costmap_2d::LETHAL_OBSTACLE);
    expectHoldOrRefusal(sign * 0.4);
  }
}

TEST_F(RotationShimCollision, FirstIntervalIsCheckedEvenWhenItPassesTheGoalHeading)
{
  for (int sign : {-1, 1})
  {
    SCOPED_TRACE(sign);
    clear();
    const double omega = command(sign * 0.01).angular.z;
    EXPECT_GT(sign * omega, 0.0);
    clear();
    mark(0.536, -sign * 0.20);
    ASSERT_EQ(footprintCost(0.0), 0.0);
    ASSERT_EQ(footprintCost(omega * 0.1), nav2_costmap_2d::LETHAL_OBSTACLE);
    expectHoldOrRefusal(sign * 0.01);
  }
}

TEST_F(RotationShimCollision, InitialAlignmentRetainsHysteresisAndChecksReleaseSample)
{
  checker_.xy_reached = false;
  for (int sign : {-1, 1})
  {
    SCOPED_TRACE(sign);
    // A fresh shim does not engage below the 0.785 entry threshold.
    clear();
    EXPECT_DOUBLE_EQ(command(sign * 0.45).angular.z, 0.0);
    clear();
    EXPECT_GT(sign * command(sign * 0.9).angular.z, 0.0);
    // Once engaged, 0.45 remains above the 0.3925 release threshold.
    EXPECT_GT(sign * command(sign * 0.45).angular.z, 0.0);
    clear();
    EXPECT_GT(sign * command(sign * 0.9).angular.z, 0.0);
    mark(0.55, -sign * 0.20);
    ASSERT_EQ(footprintCost(0.0), 0.0);
    ASSERT_EQ(footprintCost(sign * 0.05), 0.0);
    ASSERT_EQ(footprintCost(sign * 0.1), nav2_costmap_2d::LETHAL_OBSTACLE);
    // The next request is 0.5 rad/s; sample 0.1 is the first one beyond
    // the remaining 0.0575 rad to release, and must still be checked.
    expectHoldOrRefusal(sign * 0.45);
    // Finish the active rotation before the next independent control.
    command(0.0);
  }
}

TEST_F(RotationShimCollision, PathSamplingFailureStillUsesThePrimaryController)
{
  checker_.xy_reached = false;
  clear();
  // One pose is insufficient for the shim's path sampling, but RPP can use
  // it. The collision-specific refusal must not remove this recovery path.
  geometry_msgs::msg::Twist result;
  ASSERT_NO_THROW(result = command(1.2, 1));
  EXPECT_GT(result.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular.z, 0.0);
}

TEST_F(RotationShimCollision, OppositeSignCommandHistoryChecksTheActualArc)
{
  for (int sign : {-1, 1})
  {
    SCOPED_TRACE(sign);
    clear();
    // A previous accepted command at the opposite configured maximum leaves
    // a 0.5 rad/s opposite turn after this tick's acceleration bound.
    shim_.previousCommand(-sign * 0.75);
    EXPECT_LT(sign * command(sign * 0.4).angular.z, 0.0);
    clear();
    shim_.previousCommand(-sign * 0.75);
    mark(0.55, sign * 0.20);
    ASSERT_EQ(footprintCost(0.0), 0.0);
    ASSERT_EQ(footprintCost(-sign * 0.1), nav2_costmap_2d::LETHAL_OBSTACLE);
    expectHoldOrRefusal(sign * 0.4);
  }
}
}  // namespace
