// Copyright (C) 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// LazyPrunePathHandler must be observably identical to the upstream
// nav2_controller::FeasiblePathHandler it extends: same segment, same
// transformed local plan, same exceptions, same goal, cycle after cycle. Both
// run side by side on the same parent node, costmap and TF buffer, fed the same
// plans and robot poses (coverage-shaped plans: serpentine, closed ring, pivot
// twins, cusps; robot on, off, behind, ahead of and far from the plan).

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <typeinfo>
#include <vector>

#include "mowgli_nav2_plugins/lazy_prune_path_handler.hpp"
#include "nav2_controller/plugins/feasible_path_handler.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <gtest/gtest.h>

namespace
{

using Pose = geometry_msgs::msg::PoseStamped;

Pose MakePose(double x, double y, double yaw)
{
  Pose p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

nav_msgs::msg::Path PathOf(std::vector<Pose> poses)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = std::move(poses);
  return path;
}

// Serpentine of 0.03 m steps, forward semicircle joins, with a pivot twin
// (same position, outgoing heading) at every other row end.
nav_msgs::msg::Path Serpentine(std::size_t n, double len)
{
  std::vector<Pose> poses;
  const double step = 0.03, r = 0.08;
  double y = 0.0, dir = 1.0;
  bool twin = false;
  while (poses.size() < n)
  {
    const double x0 = dir > 0 ? 0.0 : len;
    for (double s = 0.0; s < len - 1e-9 && poses.size() < n; s += step)
      poses.push_back(MakePose(x0 + dir * s, y, dir > 0 ? 0.0 : M_PI));
    if (twin && poses.size() < n)
      poses.push_back(MakePose(poses.back().pose.position.x, y, M_PI / 2.0));
    twin = !twin;
    const double cx = dir > 0 ? len : 0.0;
    for (double a = 0.0; a < M_PI - 1e-9 && poses.size() < n; a += step / r)
      poses.push_back(MakePose(cx + dir * r * std::sin(a),
                               y + r - r * std::cos(a),
                               (dir > 0 ? 0.0 : M_PI) + a));
    y += 2.0 * r;
    dir = -dir;
  }
  return PathOf(std::move(poses));
}

// Straight, an in-place rotation (a pivot corner: several poses at one position), straight on.
// enforce_path_rotation cuts the plan at the rotation.
nav_msgs::msg::Path LineRotateLine(std::size_t leg, double turn)
{
  std::vector<Pose> poses;
  for (std::size_t i = 0; i < leg; ++i)
    poses.push_back(MakePose(0.03 * static_cast<double>(i), 0.0, 0.0));
  const double x = poses.back().pose.position.x;
  for (int k = 1; k <= 12; ++k)
    poses.push_back(MakePose(x, 0.0, turn * static_cast<double>(k) / 12.0));
  for (std::size_t i = 1; i <= leg; ++i)
    poses.push_back(MakePose(x, 0.03 * static_cast<double>(i), turn));
  return PathOf(std::move(poses));
}

// Closed headland ring: start == end to the bit (the 2026-08-24 snap case).
nav_msgs::msg::Path Ring(double radius, std::size_t n)
{
  std::vector<Pose> poses;
  for (std::size_t i = 0; i <= n; ++i)
  {
    const double a = 2.0 * M_PI * static_cast<double>(i % n) / static_cast<double>(n);
    poses.push_back(MakePose(radius * std::cos(a), radius * std::sin(a), a + M_PI / 2.0));
  }
  return PathOf(std::move(poses));
}

// Forward, then reversing straight back over the same line: a cusp, for
// enforce_path_inversion.
nav_msgs::msg::Path Cusp(std::size_t half)
{
  std::vector<Pose> poses;
  for (std::size_t i = 0; i < half; ++i)
    poses.push_back(MakePose(0.05 * static_cast<double>(i), 0.0, 0.0));
  for (std::size_t i = half; i-- > 0;)
    poses.push_back(MakePose(0.05 * static_cast<double>(i), 0.0, 0.0));
  return PathOf(std::move(poses));
}

struct Outcome
{
  std::optional<std::string> error;
  Pose closest;
  std::ptrdiff_t segment{0};  // closest .. pruned_plan_end
  nav_msgs::msg::Path local;
};

Outcome Cycle(nav2_core::PathHandler& handler, const Pose& robot)
{
  Outcome out;
  try
  {
    auto [closest, end] = handler.findPlanSegment(robot);
    out.closest = *closest;
    out.segment = std::distance(closest, end);
    out.local = handler.transformLocalPlan(closest, end);
  }
  catch (const std::exception& e)
  {
    out.error = std::string(typeid(e).name()) + ": " + e.what();
  }
  return out;
}

// Read access to the pruned plan both handlers keep.
template <typename Handler>
struct Probe : Handler
{
  const std::vector<Pose>& live() const
  {
    return this->global_plan_up_to_constraint_.poses;
  }
};

class Fixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<nav2::LifecycleNode>("controller_server", "");
    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", "/", false);
    for (const auto& p :
         std::vector<rclcpp::Parameter>{rclcpp::Parameter("plugins", std::vector<std::string>{}),
                                        rclcpp::Parameter("global_frame", "odom"),
                                        rclcpp::Parameter("robot_base_frame", "base_footprint"),
                                        rclcpp::Parameter("rolling_window", false),
                                        rclcpp::Parameter("width", 12),
                                        rclcpp::Parameter("height", 12),
                                        rclcpp::Parameter("resolution", 0.05),
                                        rclcpp::Parameter("origin_x", -3.0),
                                        rclcpp::Parameter("origin_y", -3.0),
                                        rclcpp::Parameter("transform_tolerance", 0.5)})
    {
      if (costmap_->has_parameter(p.get_name()))
        ASSERT_TRUE(costmap_->set_parameter(p).successful) << p.get_name();
      else
        costmap_->declare_parameter(p.get_name(), p.get_parameter_value());
    }
    costmap_->configure();
    // configure() starts the costmap's executor thread; cleanup() cancels it and joins. If that
    // happens before the thread has begun to spin (likely on a single core, for a test that
    // finishes in a few ms) the cancel is lost and the join never returns. Let it start first.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    tf_ = costmap_->getTfBuffer();
    // A non-identity map -> odom, so every pose really goes through TF.
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    t.transform.translation.x = 0.7;
    t.transform.translation.y = -0.4;
    t.transform.rotation.z = std::sin(0.15);
    t.transform.rotation.w = std::cos(0.15);
    tf_->setTransform(t, "test", true);
    node_->declare_parameter("PathHandler.max_robot_pose_search_dist", 10.0);
    upstream_.initialize(node_, node_->get_logger(), "PathHandler", costmap_, tf_);
    lazy_.initialize(node_, node_->get_logger(), "PathHandler", costmap_, tf_);
  }

  void TearDown() override
  {
    costmap_->cleanup();
  }

  void SetPlan(const nav_msgs::msg::Path& path)
  {
    upstream_.setPlan(path);
    lazy_.setPlan(path);
    last_live_size_ = upstream_.live().size();
  }

  // One controller_server cycle on both; every observable must match.
  // Returns the (shared) exception text, if the cycle threw.
  std::optional<std::string> ExpectSameCycle(const Pose& robot,
                                             const std::string& where,
                                             bool compare_goal = true)
  {
    const auto a = Cycle(upstream_, robot);
    const auto b = Cycle(lazy_, robot);
    EXPECT_EQ(a.error, b.error) << where;
    if (a.error != b.error)
      return a.error;
    if (!a.error)
    {
      EXPECT_EQ(a.closest, b.closest) << where;
      EXPECT_EQ(a.segment, b.segment) << where;
      EXPECT_EQ(a.local, b.local) << where;
    }
    if (!compare_goal)
      return a.error;
    const auto stamp = robot.header.stamp;
    std::optional<std::string> ga, gb;
    Pose goal_a, goal_b;
    try
    {
      goal_a = upstream_.getTransformedGoal(stamp);
    }
    catch (const std::exception& e)
    {
      ga = e.what();
    }
    try
    {
      goal_b = lazy_.getTransformedGoal(stamp);
    }
    catch (const std::exception& e)
    {
      gb = e.what();
    }
    EXPECT_EQ(ga, gb) << where;
    EXPECT_EQ(goal_a, goal_b) << where;
    // The state behind the next cycle: upstream's pruned plan is exactly the
    // live tail of the lazy handler's vector (the rest is passed poses).
    const auto& up = upstream_.live();
    const auto& lz = lazy_.live();
    EXPECT_LE(up.size(), lz.size()) << where;
    if (up.size() <= lz.size())
    {
      EXPECT_TRUE(
          std::equal(up.begin(), up.end(), lz.end() - static_cast<std::ptrdiff_t>(up.size())))
          << where;
    }
    // The lazy path proper: passed poses kept in place instead of erased.
    lazy_behind_ += lz.size() > up.size() ? 1 : 0;
    // Only the inversion branch can make the pruned plan longer (it refills it
    // past the cusp from global_plan_).
    inversions_ += up.size() > last_live_size_ ? 1 : 0;
    last_live_size_ = up.size();
    ++cycles_;
    return a.error;
  }

  // Drive a robot along the plan with noise, stalls, small reversals and the
  // occasional jump (a localisation correction), as controller_server would see.
  void Drive(const nav_msgs::msg::Path& path,
             unsigned seed,
             int cycles,
             double jump_m,
             bool follow_yaw = false)
  {
    std::mt19937 rng(seed);
    std::normal_distribution<double> noise(0.0, 0.02);
    std::uniform_real_distribution<double> u(0.0, 1.0);
    double s = 0.0;  // index along the plan, fractional
    const double n = static_cast<double>(path.poses.size());
    for (int c = 0; c < cycles; ++c)
    {
      const double r = u(rng);
      if (r < 0.70)
        s += 1.7;  // 0.5 m/s at 10 Hz, 0.03 m steps
      else if (r < 0.80)
        s += 0.0;  // stalled / pivoting
      else if (r < 0.90)
        s -= 1.0;  // drifted back
      else if (r < 0.97)
        s += 6.0;  // fast
      else
        s += (u(rng) - 0.3) * jump_m / 0.03;  // jump
      s = std::clamp(s, 0.0, n - 1.0);
      const auto& p = path.poses[static_cast<std::size_t>(s)].pose;
      const double yaw =
          follow_yaw ? 2.0 * std::atan2(p.orientation.z, p.orientation.w) + noise(rng) : 0.0;
      Pose robot = MakePose(p.position.x + noise(rng), p.position.y + noise(rng), yaw);
      robot.header.frame_id = "map";
      robot.header.stamp = rclcpp::Time(static_cast<int64_t>(c) * 100000000LL, RCL_ROS_TIME);
      ExpectSameCycle(robot, "seed " + std::to_string(seed) + " cycle " + std::to_string(c));
      if (HasFailure())
        return;
    }
  }

  std::shared_ptr<nav2::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  Probe<nav2_controller::FeasiblePathHandler> upstream_;
  Probe<mowgli_nav2_plugins::LazyPrunePathHandler> lazy_;
  int cycles_{0};
  int lazy_behind_{0};  // cycles with passed poses not yet erased
  int inversions_{0};  // cycles that passed a cusp (enforce_path_inversion)
  std::size_t last_live_size_{0};
};

TEST_F(Fixture, SerpentineMatchesUpstreamEveryCycle)
{
  for (unsigned seed = 1; seed <= 6; ++seed)
  {
    SetPlan(Serpentine(3000, 4.0));
    Drive(Serpentine(3000, 4.0), seed, 2500, 3.0);
    ASSERT_FALSE(HasFailure());
  }
  EXPECT_EQ(cycles_, 6 * 2500);
  EXPECT_GT(lazy_behind_, cycles_ / 2);
}

TEST_F(Fixture, ClosedRingMatchesUpstream)
{
  const auto ring = Ring(1.2, 250);
  for (unsigned seed = 11; seed <= 14; ++seed)
  {
    SetPlan(ring);
    Drive(ring, seed, 600, 2.0);
    ASSERT_FALSE(HasFailure());
  }
}

TEST_F(Fixture, PlanLeavingTheCostmapAndRobotFarAwayMatch)
{
  // 40 m swaths: most of the plan is outside the 12 m costmap (local plan
  // truncated there); the robot also starts 30 m off the plan.
  const auto path = Serpentine(4000, 40.0);
  SetPlan(path);
  Pose far = MakePose(30.0, 30.0, 0.0);
  ExpectSameCycle(far, "far away");
  ASSERT_FALSE(HasFailure());
  Drive(path, 21, 1500, 5.0);
}

TEST_F(Fixture, ReplanMidPathResetsBoth)
{
  const auto a = Serpentine(2000, 3.0);
  const auto b = Ring(0.9, 120);
  SetPlan(a);
  Drive(a, 31, 700, 1.0);
  ASSERT_FALSE(HasFailure());
  SetPlan(b);
  Drive(b, 32, 300, 1.0);
  ASSERT_FALSE(HasFailure());
  SetPlan(a);
  Drive(a, 33, 900, 1.0);
}

TEST_F(Fixture, TinyPlansMatchIncludingExceptions)
{
  const Pose robot = MakePose(0.1, 0.0, 0.0);
  SetPlan(PathOf({MakePose(0.0, 0.0, 0.0)}));
  ExpectSameCycle(robot, "one pose");
  SetPlan(PathOf({MakePose(0.0, 0.0, 0.0), MakePose(0.03, 0.0, 0.0)}));
  for (int i = 0; i < 5; ++i)
    ExpectSameCycle(robot, "two poses");
  // Pivot twin as the last two poses.
  SetPlan(PathOf({MakePose(0.0, 0.0, 0.0), MakePose(0.03, 0.0, 0.0), MakePose(0.03, 0.0, 1.5)}));
  for (int i = 0; i < 5; ++i)
    ExpectSameCycle(MakePose(0.03, 0.0, 0.0), "twin end");
  // Off the costmap: upstream throws "Resulting plan has 0 poses in it."
  SetPlan(PathOf({MakePose(50.0, 50.0, 0.0), MakePose(50.03, 50.0, 0.0)}));
  EXPECT_EQ(ExpectSameCycle(robot, "outside the costmap"),
            std::optional<std::string>(std::string(typeid(nav2_core::InvalidPath).name()) +
                                       ": Resulting plan has 0 poses in it."));
}

TEST_F(Fixture, ConstraintBranchMatchesWithInversionEnforced)
{
  // The inherited enforce_path_inversion branch replaces the pruned plan; the
  // lazy handler must hand it upstream's vector.
  ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter("PathHandler.enforce_path_inversion", true))
                  .successful);
  const auto cusp = Cusp(120);
  for (unsigned seed = 41; seed <= 44; ++seed)
  {
    SetPlan(cusp);
    Drive(cusp, seed, 400, 0.5);
    ASSERT_FALSE(HasFailure());
  }
  EXPECT_GE(inversions_, 4);
}

TEST_F(Fixture, ConstraintBranchMatchesWithRotationEnforced)
{
  // enforce_path_rotation cuts the plan at the pivot and refills it past it once the robot is
  // within the inversion tolerances, with the robot's yaw following the plan (not a fixed 0).
  ASSERT_TRUE(
      node_
          ->set_parameters({rclcpp::Parameter("PathHandler.enforce_path_rotation", true),
                            rclcpp::Parameter("PathHandler.minimum_rotation_angle", 0.785)})[0]
          .successful);
  const auto path = LineRotateLine(120, 1.5);
  for (unsigned seed = 51; seed <= 56; ++seed)
  {
    SetPlan(path);
    Drive(path, seed, 500, 0.5, /*follow_yaw=*/true);
    ASSERT_FALSE(HasFailure());
  }
  EXPECT_GE(inversions_, 6);
}

TEST_F(Fixture, EmptyPlanAndUnreachableRobotFrameMatchIncludingExceptions)
{
  // No plan at all.
  // (getTransformedGoal is not compared: upstream takes .back() of the empty plan, and
  // controller_server never calls it after findPlanSegment threw.)
  SetPlan(PathOf({}));
  EXPECT_TRUE(ExpectSameCycle(MakePose(0.0, 0.0, 0.0), "empty plan", false).has_value());
  // A robot pose in a frame TF does not know, then a normal cycle: both must have thrown the same
  // thing and still agree afterwards.
  const auto path = Serpentine(600, 3.0);
  SetPlan(path);
  Pose lost = MakePose(0.1, 0.0, 0.0);
  lost.header.frame_id = "nowhere";
  EXPECT_TRUE(ExpectSameCycle(lost, "unknown frame").has_value());
  Pose ok = MakePose(0.2, 0.0, 0.0);
  ok.header.frame_id = "map";
  ExpectSameCycle(ok, "after the TF failure");
  Drive(path, 61, 200, 1.0);
}

TEST_F(Fixture, RejectUnitPathThrowsLikeUpstream)
{
  // reject_unit_path is read at initialize(), so a second pair is initialised under another
  // plugin name on the same node, costmap and TF buffer.
  node_->declare_parameter("PathHandlerUnit.reject_unit_path", true);
  Probe<nav2_controller::FeasiblePathHandler> upstream;
  Probe<mowgli_nav2_plugins::LazyPrunePathHandler> lazy;
  upstream.initialize(node_, node_->get_logger(), "PathHandlerUnit", costmap_, tf_);
  lazy.initialize(node_, node_->get_logger(), "PathHandlerUnit", costmap_, tf_);
  const auto one = PathOf({MakePose(0.0, 0.0, 0.0)});
  upstream.setPlan(one);
  lazy.setPlan(one);
  const Pose robot = MakePose(0.0, 0.0, 0.0);
  const auto a = Cycle(upstream, robot);
  const auto b = Cycle(lazy, robot);
  ASSERT_TRUE(a.error.has_value());
  EXPECT_EQ(a.error, b.error);
  EXPECT_NE(a.error->find("length of one"), std::string::npos);
}

}  // namespace

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
