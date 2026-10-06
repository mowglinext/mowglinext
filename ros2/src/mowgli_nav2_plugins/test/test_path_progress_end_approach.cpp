// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
//
// PathProgressGoalChecker — the END-APPROACH rule.
//
// Field, 2026-09-21: short coverage sub-paths (20-24 poses at ~0.03 m) stalled
// 28-30 s at their end and aborted with "Failed to make progress". FTC parks up
// to max_goal_distance_error (0.50 m) short of the last pose and then emits zero
// velocity; the checker only accepted >= 95 % of the POSES reached, which on a
// 0.6 m path is never met from 0.3-0.45 m short. The checker now also accepts a
// robot whose furthest monotonically-reached point is within xy_goal_tolerance
// of the end, measured ALONG the path — without letting a robot that is merely
// NEAR the end (the start of a looped path) complete.

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "mowgli_nav2_plugins/path_progress_goal_checker.hpp"
#include <gtest/gtest.h>

namespace mowgli_nav2_plugins
{

namespace
{

// Shipped coverage_goal_checker xy tolerance (nav2_params_base.yaml, floored at
// FTC's max_goal_distance_error by navigation.launch.py).
constexpr double kXyTolM = 0.50;
// Ships as 3.14 (heading ignored). Exactly pi here, so no case below can pass
// on a yaw rejection of a robot facing away from the goal (e.g. the start of a
// U-turn) instead of on the progress gate under test.
constexpr double kYawTolRad = M_PI;
// The robot queries in odom; FTC publishes the plan in map. map = odom + offset.
constexpr double kMapFromOdomX = 100.0;
// A 0.2 m/s robot polled by a 10 Hz controller.
constexpr double kDriveStepM = 0.02;

struct Pose2
{
  double x;
  double y;
  double yaw;
};

geometry_msgs::msg::Quaternion yawToQuat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

std::vector<Pose2> straightPath(int poses, double spacing_m)
{
  std::vector<Pose2> out;
  for (int i = 0; i < poses; ++i)
  {
    out.push_back({spacing_m * i, 0.0, 0.0});
  }
  return out;
}

// FTC republishes its plan with the last pose duplicated (newPathReceived).
std::vector<Pose2> withFtcTail(std::vector<Pose2> path)
{
  path.push_back(path.back());
  return path;
}

void appendLine(std::vector<Pose2>& path, double x0, double y0, double x1, double y1, double step)
{
  const double len = std::hypot(x1 - x0, y1 - y0);
  const double yaw = std::atan2(y1 - y0, x1 - x0);
  const int n = static_cast<int>(std::round(len / step));
  for (int i = path.empty() ? 0 : 1; i <= n; ++i)
  {
    const double t = static_cast<double>(i) / n;
    path.push_back({x0 + t * (x1 - x0), y0 + t * (y1 - y0), yaw});
  }
}

// Counter-clockwise half turn of radius r around (cx, cy), starting at angle a0.
void appendHalfTurn(
    std::vector<Pose2>& path, double cx, double cy, double r, double a0, double step)
{
  const int n = static_cast<int>(std::round(M_PI * r / step));
  for (int i = 1; i <= n; ++i)
  {
    const double a = a0 + M_PI * static_cast<double>(i) / n;
    path.push_back({cx + r * std::cos(a), cy + r * std::sin(a), a + M_PI / 2.0});
  }
}

// Leg out along +x, a 0.20 m-radius U-turn, leg back along -x. The end sits
// 0.40 m from the start: inside the 0.50 m goal tolerance while 1.83 m of path
// separates them — a boustrophedon swath pair in miniature.
std::vector<Pose2> uTurnPath()
{
  std::vector<Pose2> path;
  appendLine(path, 0.0, 0.0, 0.6, 0.0, 0.03);
  appendHalfTurn(path, 0.6, 0.2, 0.2, -M_PI / 2.0, 0.03);
  appendLine(path, 0.6, 0.4, 0.0, 0.4, 0.03);
  return path;
}

double pathLength(const std::vector<Pose2>& path)
{
  double len = 0.0;
  for (size_t i = 1; i < path.size(); ++i)
  {
    len += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  }
  return len;
}

// The point `s` metres along `path` (clamped to its ends).
Pose2 pointAt(const std::vector<Pose2>& path, double s)
{
  for (size_t i = 1; i < path.size(); ++i)
  {
    const double seg = std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    if (seg > 0.0 && s <= seg)
    {
      const double t = std::max(0.0, s) / seg;
      return {path[i - 1].x + t * (path[i].x - path[i - 1].x),
              path[i - 1].y + t * (path[i].y - path[i - 1].y),
              path[i].yaw};
    }
    s -= seg;
  }
  return path.back();
}

}  // namespace

class PathProgressEndApproachTest : public testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    node_ = std::make_shared<nav2::LifecycleNode>("path_progress_end_approach_test");
    node_->declare_parameter("coverage.xy_goal_tolerance", kXyTolM);
    node_->declare_parameter("coverage.yaw_goal_tolerance", kYawTolRad);
    checker_.initialize(node_, "coverage", nullptr);
    checker_.query_frame_ = "odom";
    checker_.tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";
    transform.transform.translation.x = kMapFromOdomX;
    transform.transform.rotation.w = 1.0;
    checker_.tf_buffer_->setTransform(transform, "test", true);
  }

  // Latch `path` (map frame) as FTC does, and aim the goal at its last pose.
  void loadPath(const std::vector<Pose2>& path)
  {
    path_ = path;
    auto msg = std::make_shared<nav_msgs::msg::Path>();
    msg->header.frame_id = "map";
    for (const auto& p : path)
    {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "map";
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.orientation = yawToQuat(p.yaw);
      msg->poses.push_back(ps);
    }
    checker_.onPath(msg);
    checker_.reset();
    goal_ = toOdom(path.back());
  }

  // controller_server hands both poses over in the costmap (odom) frame.
  static geometry_msgs::msg::Pose toOdom(const Pose2& map_pose)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = map_pose.x - kMapFromOdomX;
    pose.position.y = map_pose.y;
    pose.orientation = yawToQuat(map_pose.yaw);
    return pose;
  }

  bool reachedAt(const Pose2& robot_map)
  {
    return checker_.isGoalReached(toOdom(robot_map),
                                  goal_,
                                  geometry_msgs::msg::Twist{},
                                  nav_msgs::msg::Path{});
  }

  // Drive the robot along the path from s_from to s_to, one controller tick per
  // kDriveStepM. Returns the arc position of the first tick the goal was
  // reached at, or a negative value if it never was.
  double driveAlong(double s_from, double s_to)
  {
    for (double s = s_from; s <= s_to + 1e-9; s += kDriveStepM)
    {
      if (reachedAt(pointAt(path_, s)))
      {
        return s;
      }
    }
    return -1.0;
  }

  // FTC has parked: the stationary robot is polled for `ticks` cycles.
  bool reachedWhileParkedAt(const Pose2& robot_map, int ticks = 50)
  {
    for (int i = 0; i < ticks; ++i)
    {
      if (reachedAt(robot_map))
      {
        return true;
      }
    }
    return false;
  }

  std::size_t maxReachedIndex() const
  {
    return checker_.max_reached_index_;
  }

  void setXyGoalTolerance(double tolerance)
  {
    checker_.xy_goal_tolerance_ = tolerance;
  }

  std::shared_ptr<nav2::LifecycleNode> node_;
  PathProgressGoalChecker checker_;
  std::vector<Pose2> path_;
  geometry_msgs::msg::Pose goal_;
};

// (1) The field case: FTC parks 0.30 m / 0.45 m short of the end of a 24-pose,
// 0.03 m-spaced sub-path. 0.45 m short is pose 8 of 23 — 35 % of the poses —
// so the 95 % rule alone never passed and the goal aborted 30 s later.
TEST_F(PathProgressEndApproachTest, FieldSubPathParkedShortOfTheEndIsReached)
{
  for (const double short_m : {0.30, 0.45})
  {
    SCOPED_TRACE(short_m);
    loadPath(withFtcTail(straightPath(24, 0.03)));
    const double length = pathLength(path_);
    const Pose2 parked = pointAt(path_, length - short_m);

    // Nothing fires while more than the tolerance is still ahead.
    EXPECT_LT(driveAlong(0.0, length - kXyTolM - 0.02), 0.0);
    // FTC's park point: reached (the drive may already have fired just inside
    // the tolerance, exactly like a long path does 0.50 m before its end).
    const double fired_at = driveAlong(length - kXyTolM - 0.02, length - short_m);
    EXPECT_GE(fired_at, length - kXyTolM - 1e-6);
    EXPECT_TRUE(reachedWhileParkedAt(parked));
  }
}

TEST_F(PathProgressEndApproachTest, FirstQueryAtSearchBoundaryRecoversAfterRobotPassesIt)
{
  for (const double first_query : {0.286, 0.296})
  {
    SCOPED_TRACE(first_query);
    loadPath(withFtcTail(straightPath(201, 0.03)));
    const double length = pathLength(path_);

    // Index 10 (0.30 m) is the nearest pose, but the robot has not passed the
    // artificial boundary yet. Keep the anti-jump guard in place.
    EXPECT_FALSE(reachedAt(pointAt(path_, first_query)));
    EXPECT_EQ(maxReachedIndex(), 0u);

    // Sampling phase can put the next query more than half a pose beyond the
    // boundary. It still crosses locally and must release the stuck window.
    const double after_boundary = first_query + 0.02;
    EXPECT_FALSE(reachedAt(pointAt(path_, after_boundary)));
    EXPECT_EQ(maxReachedIndex(), 10u);

    // Normal forward motion can then carry the bounded cursor to the goal.
    EXPECT_GE(driveAlong(after_boundary + 0.02, length), 0.0);
    EXPECT_GT(maxReachedIndex(), 10u);
  }
}

TEST_F(PathProgressEndApproachTest, SearchBoundaryRecoversAcrossAPivotCorner)
{
  std::vector<Pose2> path;
  appendLine(path, 0.0, 0.0, 1.25, 0.0, 0.125);
  path.push_back({1.25, 0.0, M_PI / 2.0});  // outgoing twin at the pivot corner
  appendLine(path, 1.25, 0.0, 1.25, 1.25, 0.125);
  loadPath(path);

  // The corner is the artificial search boundary at index 10. Repeated
  // positions at the pivot cannot prove the outgoing leg has been traversed.
  EXPECT_FALSE(reachedAt({1.24, 0.0, 0.0}));
  EXPECT_FALSE(reachedAt({1.25, 0.0, M_PI / 2.0}));
  EXPECT_EQ(maxReachedIndex(), 0u);

  // Projection on the incoming leg remains exactly 1.0, but motion down the
  // outgoing leg is a valid local crossing of that same search frontier.
  EXPECT_FALSE(reachedAt({1.25, 0.02, M_PI / 2.0}));
  EXPECT_EQ(maxReachedIndex(), 10u);
}

TEST_F(PathProgressEndApproachTest, FirstQuerySlightlyPastBoundaryRecoversOnLongPath)
{
  loadPath(withFtcTail(straightPath(201, 0.03)));
  const double length = pathLength(path_);

  // A first localized query may arrive just beyond the window. Do not advance
  // on that observation alone; a later local forward query proves motion.
  EXPECT_FALSE(reachedAt(pointAt(path_, 0.306)));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt(pointAt(path_, 0.326)));
  EXPECT_EQ(maxReachedIndex(), 10u);
  EXPECT_GE(driveAlong(0.346, length), 0.0);
  EXPECT_GT(maxReachedIndex(), 10u);
}

TEST_F(PathProgressEndApproachTest, SmallForwardSamplesAccumulateAtSearchBoundary)
{
  loadPath(withFtcTail(straightPath(201, 0.03)));

  EXPECT_FALSE(reachedAt(pointAt(path_, 0.306)));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt(pointAt(path_, 0.316)));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt(pointAt(path_, 0.326)));
  EXPECT_EQ(maxReachedIndex(), 10u);
}

TEST_F(PathProgressEndApproachTest, LocalOffsetAtBoundaryRecovers)
{
  loadPath(withFtcTail(straightPath(201, 0.03)));

  EXPECT_FALSE(reachedAt(pointAt(path_, 0.360)));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt(pointAt(path_, 0.370)));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt(pointAt(path_, 0.380)));
  EXPECT_EQ(maxReachedIndex(), 10u);
}

TEST_F(PathProgressEndApproachTest, SmallSamplesPastBoundaryAccumulateBeyondLocalProjection)
{
  loadPath(withFtcTail(straightPath(201, 0.03)));

  EXPECT_FALSE(reachedAt(pointAt(path_, 0.379)));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt(pointAt(path_, 0.389)));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt(pointAt(path_, 0.399)));
  EXPECT_EQ(maxReachedIndex(), 10u);
}

TEST_F(PathProgressEndApproachTest, ObservationsOutsideLocalBoundaryRadiusDoNotAdvance)
{
  loadPath(withFtcTail(straightPath(201, 0.03)));

  for (const double x : {0.430, 0.450})
  {
    EXPECT_FALSE(reachedAt({x, 0.0, 0.0}));
    EXPECT_EQ(maxReachedIndex(), 0u);
  }
}

TEST_F(PathProgressEndApproachTest, OffsetStartCanRecoverAndReachTheEnd)
{
  loadPath(withFtcTail(straightPath(201, 0.03)));
  const double length = pathLength(path_);

  // Start 10 cm beyond the artificial boundary, just outside the old 9 cm
  // neighborhood, then keep moving forward in 2 cm samples.
  EXPECT_FALSE(reachedAt({0.400, 0.0, 0.0}));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt({0.420, 0.0, 0.0}));
  EXPECT_EQ(maxReachedIndex(), 10u);
  EXPECT_GE(driveAlong(0.440, length), 0.0);
  EXPECT_GT(maxReachedIndex(), 10u);
}

TEST_F(PathProgressEndApproachTest, ShortPathBoundaryRecoversInsideGoalTolerance)
{
  loadPath(withFtcTail(straightPath(24, 0.03)));

  // The artificial boundary itself has less than 0.50 m left, but a local
  // start and forward crossing still need to release the bounded cursor.
  EXPECT_FALSE(reachedAt({0.286, 0.0, 0.0}));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_TRUE(reachedAt({0.306, 0.0, 0.0}));
  EXPECT_EQ(maxReachedIndex(), 10u);
}

TEST_F(PathProgressEndApproachTest, DistantBoundaryObservationDoesNotPoisonLocalRecovery)
{
  loadPath(withFtcTail(straightPath(201, 0.03)));

  EXPECT_FALSE(reachedAt(pointAt(path_, 1.100)));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt(pointAt(path_, 0.306)));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt(pointAt(path_, 0.316)));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt(pointAt(path_, 0.326)));
  EXPECT_EQ(maxReachedIndex(), 10u);
}

TEST_F(PathProgressEndApproachTest, BoundaryProjectionCannotCompleteNearEndReplay)
{
  loadPath(withFtcTail(straightPath(20, 0.06)));
  const double length = pathLength(path_);
  const Pose2 endpoint = path_.back();

  // The artificial boundary leaves 0.54 m, just outside the shipped 0.50 m
  // tolerance. remainingPathLength() could subtract the next 0.06 m segment,
  // so boundary recovery must not make endpoint correction sufficient.
  EXPECT_FALSE(reachedAt(endpoint));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt({length + 0.025, 0.0, 0.0}));
  EXPECT_EQ(maxReachedIndex(), 0u);
}

TEST_F(PathProgressEndApproachTest, DistantJumpCannotReuseAValidBoundaryOrigin)
{
  loadPath(withFtcTail(straightPath(20, 0.06)));

  for (const double x : {0.600, 1.140, 1.165})
  {
    EXPECT_FALSE(reachedAt({x, 0.0, 0.0}));
    EXPECT_EQ(maxReachedIndex(), 0u);
  }
}

TEST_F(PathProgressEndApproachTest, NearEndRecoveryAccumulatesAcrossAdjacentSegmentEnd)
{
  loadPath(withFtcTail(straightPath(20, 0.06)));

  for (const double x : {0.645, 0.655})
  {
    EXPECT_FALSE(reachedAt({x, 0.0, 0.0}));
    EXPECT_EQ(maxReachedIndex(), 0u);
  }
  EXPECT_TRUE(reachedAt({0.665, 0.0, 0.0}));
  EXPECT_EQ(maxReachedIndex(), 10u);
}

TEST_F(PathProgressEndApproachTest, NearEndRecoveryCanStartBeyondAdjacentSegment)
{
  loadPath(withFtcTail(straightPath(20, 0.06)));

  EXPECT_FALSE(reachedAt({0.675, 0.0, 0.0}));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt({0.685, 0.0, 0.0}));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_TRUE(reachedAt({0.695, 0.0, 0.0}));
  EXPECT_EQ(maxReachedIndex(), 10u);
}

TEST_F(PathProgressEndApproachTest, NearEndRecoveryCanStartWithinGoalTolerance)
{
  setXyGoalTolerance(0.20);
  loadPath(withFtcTail(straightPath(12, 0.10)));

  EXPECT_FALSE(reachedAt({0.979, 0.0, 0.0}));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_TRUE(reachedAt({1.100, 0.0, 0.0}));
  EXPECT_EQ(maxReachedIndex(), 10u);
}

TEST_F(PathProgressEndApproachTest, MotionBelowRecoveryThresholdDoesNotReleaseBoundary)
{
  loadPath(withFtcTail(straightPath(201, 0.03)));

  for (const double x : {0.379, 0.389, 0.392})
  {
    EXPECT_FALSE(reachedAt({x, 0.0, 0.0}));
    EXPECT_EQ(maxReachedIndex(), 0u);
  }
}

TEST_F(PathProgressEndApproachTest, EndpointCorrectionDoesNotReleaseBoundaryInsideTolerance)
{
  setXyGoalTolerance(0.20);
  loadPath(withFtcTail(straightPath(12, 0.10)));

  for (const double x : {1.079, 1.100})
  {
    EXPECT_FALSE(reachedAt({x, 0.0, 0.0}));
    EXPECT_EQ(maxReachedIndex(), 0u);
  }
}

TEST_F(PathProgressEndApproachTest, EndpointCorrectionCannotReleaseAWindowJustBeforeGoal)
{
  setXyGoalTolerance(0.20);
  loadPath(withFtcTail(straightPath(14, 0.10)));

  for (const double x : {1.279, 1.300})
  {
    EXPECT_FALSE(reachedAt({x, 0.0, 0.0}));
    EXPECT_EQ(maxReachedIndex(), 0u);
  }
}

TEST_F(PathProgressEndApproachTest, EndpointJitterFarBeyondSearchBoundaryDoesNotReleaseIt)
{
  loadPath(withFtcTail(straightPath(21, 0.06)));

  for (const double x : {1.170, 1.195, 1.160})
  {
    EXPECT_FALSE(reachedAt({x, 0.0, 0.0}));
    EXPECT_EQ(maxReachedIndex(), 0u);
  }
}

TEST_F(PathProgressEndApproachTest, EndpointJitterCannotSeedRecoveryOnAClosedPath)
{
  std::vector<Pose2> path;
  appendLine(path, 0.0, 0.0, 0.30, 0.0, 0.03);
  appendLine(path, 0.30, 0.0, 0.42, 0.0, 0.03);
  appendLine(path, 0.42, 0.0, 0.42, 0.075, 0.025);
  appendLine(path, 0.42, 0.075, 0.30, 0.075, 0.03);
  appendLine(path, 0.30, 0.075, 0.30, 0.0, 0.025);
  loadPath(withFtcTail(path));

  // The terminal pose overlaps the artificial boundary. Its local path-arc
  // projection is the earlier segment, so endpoint geometry must prevent that
  // query and nearby jitter from creating recovery evidence.
  for (const Pose2 query : {
           Pose2{0.30, 0.0, 0.0},
           Pose2{0.32, 0.0, 0.0},
           Pose2{0.34, 0.0, 0.0},
       })
  {
    EXPECT_FALSE(reachedAt(query));
    EXPECT_EQ(maxReachedIndex(), 0u);
  }
}

TEST_F(PathProgressEndApproachTest, LateralJitterCannotSwitchRecoveryOntoReturnLeg)
{
  std::vector<Pose2> path;
  appendLine(path, 0.0, 0.0, 0.30, 0.0, 0.03);
  appendLine(path, 0.30, 0.0, 0.42, 0.0, 0.03);
  appendLine(path, 0.42, 0.0, 0.42, 0.075, 0.025);
  appendLine(path, 0.42, 0.075, 0.30, 0.075, 0.03);
  appendLine(path, 0.30, 0.075, 0.30, 0.0, 0.025);
  loadPath(withFtcTail(path));

  // The forward and return legs are close in XY. Lateral-only motion toward
  // the return leg must not project to its much-later path arc and release
  // the artificial search boundary.
  for (const Pose2 query : {
           Pose2{0.38, 0.034, 0.0},
           Pose2{0.379, 0.040, 0.0},
           Pose2{0.373, 0.046, 0.0},
           Pose2{0.367, 0.052, 0.0},
       })
  {
    EXPECT_FALSE(reachedAt(query));
    EXPECT_EQ(maxReachedIndex(), 0u);
  }
}

TEST_F(PathProgressEndApproachTest, DistantPointOnOutgoingPivotLegDoesNotLatchBoundary)
{
  std::vector<Pose2> path;
  appendLine(path, 0.0, 0.0, 0.60, 0.0, 0.06);
  path.push_back({0.60, 0.0, M_PI / 2.0});
  appendLine(path, 0.60, 0.0, 0.60, 0.60, 0.06);
  loadPath(withFtcTail(path));

  for (const Pose2 query : {
           Pose2{0.590, 0.570, 0.0},
           Pose2{0.615, 0.570, 0.0},
           Pose2{0.590, 0.570, 0.0},
           Pose2{0.590, 0.595, 0.0},
       })
  {
    EXPECT_FALSE(reachedAt(query));
    EXPECT_EQ(maxReachedIndex(), 0u);
  }
}

TEST_F(PathProgressEndApproachTest, LocalMotionRecoversAfterLocalizationCorrectionPastBoundary)
{
  loadPath(withFtcTail(straightPath(201, 0.03)));
  const double length = pathLength(path_);

  EXPECT_FALSE(reachedAt(pointAt(path_, 0.0)));
  EXPECT_FALSE(reachedAt(pointAt(path_, 0.306)));
  EXPECT_EQ(maxReachedIndex(), 0u);
  EXPECT_FALSE(reachedAt(pointAt(path_, 0.326)));
  EXPECT_EQ(maxReachedIndex(), 10u);
  EXPECT_GE(driveAlong(0.346, length), 0.0);
}

// The stationary robot the field bag shows: FTC already FINISHED, the checker is
// polled at a fixed pose. It must fire on the first tick, not 30 s later.
TEST_F(PathProgressEndApproachTest, ParkedRobotIsReachedOnTheFirstTick)
{
  loadPath(withFtcTail(straightPath(24, 0.03)));
  const double length = pathLength(path_);
  EXPECT_TRUE(reachedAt(pointAt(path_, length - 0.45)));
}

// FTC's park radius is measured to the control point: it can stop anywhere up
// to 0.50 m short, between two poses and off the line. 0.499 m short puts the
// NEAREST pose 0.51 m from the end — a pose-resolution rule would reject a robot
// FTC has already declared done.
TEST_F(PathProgressEndApproachTest, ParkedJustInsideFtcRadiusBetweenPosesIsReached)
{
  loadPath(withFtcTail(straightPath(24, 0.03)));
  const double length = pathLength(path_);
  EXPECT_LT(driveAlong(0.0, 0.16), 0.0);
  EXPECT_TRUE(reachedWhileParkedAt(pointAt(path_, length - 0.499)));
}

TEST_F(PathProgressEndApproachTest, ParkedOffTheLineInsideFtcRadiusIsReached)
{
  loadPath(withFtcTail(straightPath(24, 0.03)));
  const double length = pathLength(path_);
  EXPECT_LT(driveAlong(0.0, 0.16), 0.0);
  Pose2 parked = pointAt(path_, length - 0.43);
  parked.y += 0.10;  // 0.44 m from the end in a straight line
  EXPECT_TRUE(reachedWhileParkedAt(parked));
}

// (2) Same path, robot at the start or anywhere more than the tolerance short.
TEST_F(PathProgressEndApproachTest, SubPathNotReachedAtItsStart)
{
  loadPath(withFtcTail(straightPath(24, 0.03)));
  EXPECT_FALSE(reachedWhileParkedAt(path_.front()));
}

TEST_F(PathProgressEndApproachTest, SubPathNotReachedMoreThanToleranceShort)
{
  loadPath(withFtcTail(straightPath(24, 0.03)));
  const double length = pathLength(path_);
  EXPECT_LT(driveAlong(0.0, length - 0.55), 0.0);
  EXPECT_FALSE(reachedWhileParkedAt(pointAt(path_, length - 0.55)));
}

// (3) A looped path whose end is within the tolerance of its start: at the start
// (and for the first 0.30 m of the out leg) the robot is within 0.50 m of the
// goal, but the whole path is still ahead of it.
TEST_F(PathProgressEndApproachTest, LoopedPathNotReachedAtItsStart)
{
  loadPath(withFtcTail(uTurnPath()));
  ASSERT_LT(std::hypot(path_.back().x - path_.front().x, path_.back().y - path_.front().y),
            kXyTolM);
  EXPECT_FALSE(reachedWhileParkedAt(path_.front()));
}

TEST_F(PathProgressEndApproachTest, LoopedPathNotReachedWhilePassingNearItsEnd)
{
  loadPath(withFtcTail(uTurnPath()));
  // Out leg, x = 0 .. 0.30: sqrt(x^2 + 0.40^2) <= 0.50 m from the goal throughout.
  EXPECT_LT(driveAlong(0.0, 0.30), 0.0);
}

TEST_F(PathProgressEndApproachTest, LoopedPathReachedWhenParkedShortOfItsEnd)
{
  loadPath(withFtcTail(uTurnPath()));
  const double length = pathLength(path_);
  EXPECT_LT(driveAlong(0.0, length - kXyTolM - 0.02), 0.0);
  // 0.30 m short on the return leg is 84 % of the poses: the 95 % rule alone
  // would stall here.
  EXPECT_GE(driveAlong(length - kXyTolM - 0.02, length - 0.30), length - kXyTolM - 1e-6);
  EXPECT_TRUE(reachedWhileParkedAt(pointAt(path_, length - 0.30)));
}

// A closed ring (start == end, as the headland rings are): a robot sitting on the
// shared start/end point, exactly at the goal, has driven none of it.
TEST_F(PathProgressEndApproachTest, ClosedRingNotReachedAtItsStart)
{
  std::vector<Pose2> ring;
  appendLine(ring, 0.0, 0.0, 1.0, 0.0, 0.05);
  appendLine(ring, 1.0, 0.0, 1.0, 1.0, 0.05);
  appendLine(ring, 1.0, 1.0, 0.0, 1.0, 0.05);
  appendLine(ring, 0.0, 1.0, 0.0, 0.0, 0.05);
  loadPath(withFtcTail(ring));
  EXPECT_FALSE(reachedWhileParkedAt(path_.front()));
  EXPECT_LT(driveAlong(0.0, 3.4), 0.0);
  EXPECT_TRUE(reachedWhileParkedAt(pointAt(path_, pathLength(path_) - 0.30)));
}

// (4) Long paths: unchanged.
TEST_F(PathProgressEndApproachTest, LongPathReachedNearItsEnd)
{
  loadPath(withFtcTail(straightPath(201, 0.05)));  // 10 m
  EXPECT_LT(driveAlong(0.0, 10.0 - kXyTolM - 0.02), 0.0);
  EXPECT_TRUE(reachedWhileParkedAt(pointAt(path_, 10.0 - 0.45)));
}

// The historical 95 % rule is kept: a 20 m path that hooks back at its end is
// reached at 95 % of its poses, ~0.95 m of path before the end — the robot is
// already within 0.50 m of the goal there, but more than the tolerance of path
// is still ahead, so only the 95 % rule can fire.
TEST_F(PathProgressEndApproachTest, LongPathKeepsThe95PercentRule)
{
  std::vector<Pose2> hook;
  appendLine(hook, 0.0, 0.0, 19.2, 0.0, 0.03);
  appendHalfTurn(hook, 19.2, 0.2, 0.2, -M_PI / 2.0, 0.03);
  appendLine(hook, 19.2, 0.4, 19.03, 0.4, 0.03);
  loadPath(withFtcTail(hook));
  const double length = pathLength(path_);
  EXPECT_LT(driveAlong(0.0, 19.0), 0.0);
  const Pose2 turn_entry = pointAt(path_, 19.2);
  ASSERT_LT(std::hypot(turn_entry.x - path_.back().x, turn_entry.y - path_.back().y), kXyTolM);
  ASSERT_GT(length - 19.2, kXyTolM);
  EXPECT_GE(driveAlong(19.0, 19.2), 0.0);
}

// Field 2026-09-22: a 2391-pose sub-path completed at 97 % of its poses with
// 4.9 m of path still ahead — its end loops back within 0.49 m of that point, so
// the 95 % rule alone ended the goal and those metres were never mowed. The pose
// ratio may only forgive a SHORT remainder (kRatioRuleMaxRemainingM).
TEST_F(PathProgressEndApproachTest, The95PercentRuleNeverSkipsMetresOfPath)
{
  std::vector<Pose2> loop;
  appendLine(loop, 0.0, 0.0, 102.3, 0.0, 0.05);
  appendHalfTurn(loop, 102.3, 0.2, 0.2, -M_PI / 2.0, 0.05);
  appendLine(loop, 102.3, 0.4, 100.1, 0.4, 0.05);
  loadPath(withFtcTail(loop));
  const double length = pathLength(path_);
  const Pose2 loop_entry = pointAt(path_, 100.0);
  ASSERT_LT(std::hypot(loop_entry.x - path_.back().x, loop_entry.y - path_.back().y), kXyTolM);
  ASSERT_GE(100.0 / length, 0.95) << "the pose ratio alone must pass at the loop entry";
  ASSERT_GT(length - 100.0, 5.0);

  EXPECT_LT(driveAlong(0.0, 100.4), 0.0) << "completed with metres of path still ahead";
  EXPECT_GE(driveAlong(100.4, length - 0.30), 0.0);
}

// A path whose whole length fits inside the goal tolerance cannot tell its start
// from its end: it completes on proximity, as short_path_poses paths already do.
TEST_F(PathProgressEndApproachTest, PathShorterThanToleranceCompletesOnProximity)
{
  loadPath(withFtcTail(straightPath(14, 0.03)));  // 0.39 m, more than 10 poses
  EXPECT_TRUE(reachedAt(path_.front()));
}

}  // namespace mowgli_nav2_plugins
