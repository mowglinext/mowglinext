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
// Curvature regulation, closed loop: the real FTCController drives a
// kinematic diff-drive (first-order wheel/yaw response, controlled clock)
// along coverage-shaped plans until FTC itself reports FINISHED.
//
// Parameters: nav2_params_base.yaml as navigation.launch.py derives it. With
// FTC_CURVATURE_PARAMS_DIR set, ftc_<speed>.yaml there (written from the real
// yaml + robot_config_util.derive_turn_speed) is loaded as the node's params
// file; otherwise the same derivation is applied here (speed_slow =
// max(min_speed_mps, 0.8 x mowing_speed), max_cmd_vel_speed >= mowing_speed).
//
// Metrics, all against an ORACLE: the plan driven perfectly (rear axle on every
// pose, pivots in place), with the blade disc at its true offset (0.19 m ahead,
// radius 0.09 m) and the real footprint. Required band = op_width / 2 either
// side of every straight >= 1 m and arc R >= 1 m.
//   plan_gap   band cells the ideal sweep itself never reaches (not execution)
//   terminal   band cells within 0.35 m of the plan start/end (native endpoint)
//   exec_miss  band cells the ideal sweep cuts and the robot did not
//   lat_max    uncensored |cross-track| of the rear axle, every sample, against
//              the plan point it is associated with (bounded monotonic search)
//   exit_*     |cross-track| 0.3 / 1 / 2 m past every turn-around arc end
//   fp_excess  farthest the real footprint left the ideal footprint sweep
//   t_unavoid  extra time a yaw-feasible speed profile needs over the native
//              targets (minimum-time profile, same acceleration)

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "costmap_test_support.hpp"
#include "mowgli_nav2_plugins/ftc_controller.hpp"
#include "mowgli_nav2_plugins/ftc_curvature_feedforward.hpp"
#include <gtest/gtest.h>
#include <rcl/time.h>

namespace mowgli_nav2_plugins
{
struct FTCCurvatureTestAccess
{
  /// The rotation gates need a corner geometry covering the test plans (the
  /// stack publishes it from map_server); the same free square as the turn
  /// fallback harness.
  static void FreeMowingGeometry(FTCController& c)
  {
    auto g = std::make_shared<CornerGeometry>();
    g->valid = true;
    g->mowing.allowed = {{{-6, -6}, {18, -6}, {18, 18}, {-6, 18}}};
    c.corner_geometry_ = std::move(g);
  }
  static bool FinishedOk(const FTCController& c)
  {
    return c.current_state_ == FTCController::PlannerState::FINISHED && !c.is_crashed_;
  }
  static bool Finished(const FTCController& c)
  {
    return c.current_state_ == FTCController::PlannerState::FINISHED;
  }
  static bool Following(const FTCController& c)
  {
    return c.current_state_ == FTCController::PlannerState::FOLLOWING;
  }
  static const CurvatureAtRobot& Curvature(const FTCController& c)
  {
    return c.curvature_;
  }
  static void ForceFollowing(FTCController& c)
  {
    c.current_state_ = FTCController::PlannerState::FOLLOWING;
  }
  /// The live curvature configuration, in the order (speed limit, gain,
  /// yaw reserve, response, min turn). Adopts a staged parameter change first,
  /// as the control thread does at its next cycle.
  static std::tuple<bool, double, double, double, double> Settings(FTCController& c)
  {
    c.applyPendingConfig();
    return {c.config_.curvature_speed_limit_enabled,
            c.config_.curvature_feedforward_gain,
            c.config_.curvature_speed_yaw_reserve,
            c.config_.curvature_speed_response_s,
            c.config_.curvature_speed_min_turn_deg};
  }
  static bool Allowed(const FTCController& c)
  {
    return c.curvatureRegulationAllowed();
  }
  static void CacheBeforeUnsupportedConnector(FTCController& c, bool advance)
  {
    c.global_plan_.assign(3, geometry_msgs::msg::PoseStamped{});
    for (auto& p : c.global_plan_)
      p.pose.orientation.w = 1.0;
    c.global_plan_[1].pose.position.x = 0.1;
    c.global_plan_[2].pose.position.x = 0.1;
    c.global_plan_[2].pose.position.y = 0.1;
    c.global_plan_[2].pose.orientation.z = std::sin(M_PI / 4.0);
    c.global_plan_[2].pose.orientation.w = std::cos(M_PI / 4.0);
    c.current_state_ = FTCController::PlannerState::FOLLOWING;
    c.current_index_ = advance ? 1 : 0;
    c.current_progress_ = advance ? 0.05 : 0.99;
    c.curvature_.supported = true;  // evidence acquired before this tick's carrot advancement
  }
  static void Update(FTCController& c)
  {
    c.updateCurvatureAtRobot();
  }
  // Each non-nominal mode, one at a time.
  static void SetMode(FTCController& c, int mode, bool on)
  {
    switch (mode)
    {
      case 0:
        c.is_avoiding_ = on;
        break;
      case 1:
        c.lateral_deviation_ = on ? 0.1 : 0.0;
        break;
      case 2:
        c.target_lateral_deviation_ = on ? 0.1 : 0.0;
        break;
      case 3:
        c.reverse_escape_active_ = on;
        break;
      case 4:
        c.obstacle_waiting_ = on;
        break;
      case 5:
        c.obstacle_recovery_active_ = on;
        break;
      case 6:
        c.turn_fallback_.phase = on ? FTCController::TurnFallbackPhase::kReverse
                                    : FTCController::TurnFallbackPhase::kIdle;
        break;
      case 7:
        c.is_stalled_ = on;
        break;
      default:
        break;
    }
  }
};
}  // namespace mowgli_nav2_plugins

namespace
{

namespace mn = mowgli_nav2_plugins;
using Pose = geometry_msgs::msg::PoseStamped;

constexpr double kDt = 0.1;  // controller_frequency 10 Hz
constexpr double kBladeX = 0.19;
constexpr double kBladeR = 0.09;
constexpr double kHalfBand = 0.08;  // (tool_width 0.18 - swath_overlap 0.02) / 2
constexpr double kCell = 0.01;
constexpr double kFpCell = 0.02;
constexpr double kFpX0 = -0.17, kFpX1 = 0.53, kFpHalfW = 0.275;
constexpr double kTerminalM = 0.35;
constexpr double kMaxAng = 0.8;
constexpr double kAccel = 0.2;
const char* const kPlugin = "FollowCoveragePath";

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

/// A plan built from straights and arcs like the coverage server's output
/// (straights at 0.10 m, arcs at ~0.09 m chords).
struct Plan
{
  struct Band
  {
    double x, y, yaw, length, radius;  // radius 0 = straight; signed for arcs
  };
  nav_msgs::msg::Path path;
  std::vector<Band> bands;
  std::vector<double> turn_end_s;  // plan arc length at each tight arc's end
  std::vector<double> turn_next_s;  // end of the straight after it
  double x{0}, y{0}, yaw{0}, s{0};

  Plan()
  {
    path.header.frame_id = "map";
    path.poses.push_back(MakePose(0, 0, 0));
  }
  void Straight(double length)
  {
    if (length >= 1.0)
      bands.push_back({x, y, yaw, length, 0.0});
    const int n = std::max(1, static_cast<int>(std::ceil(length / 0.10)));
    const double x0 = x, y0 = y;
    for (int k = 1; k <= n; ++k)
    {
      x = x0 + length * k / n * std::cos(yaw);
      y = y0 + length * k / n * std::sin(yaw);
      path.poses.push_back(MakePose(x, y, yaw));
    }
    s += length;
    if (turn_next_s.size() < turn_end_s.size())
      turn_next_s.push_back(s);
  }
  void Arc(double radius, double angle)  // angle > 0 turns left
  {
    if (radius >= 1.0)
      bands.push_back({x, y, yaw, radius * std::abs(angle), std::copysign(radius, angle)});
    const int n = std::max(2, static_cast<int>(std::ceil(radius * std::abs(angle) / 0.09)));
    const double sg = angle > 0 ? 1.0 : -1.0;
    const double cx = x - sg * radius * std::sin(yaw), cy = y + sg * radius * std::cos(yaw);
    const double yaw0 = yaw;
    for (int k = 1; k <= n; ++k)
    {
      yaw = yaw0 + angle * k / n;
      x = cx + sg * radius * std::sin(yaw);
      y = cy - sg * radius * std::cos(yaw);
      path.poses.push_back(MakePose(x, y, yaw));
    }
    s += radius * std::abs(angle);
    if (radius < 1.0)
      turn_end_s.push_back(s);
  }
  void Pivot(double angle)  // coverage_geometry.hpp corner: same position, new heading
  {
    yaw += angle;
    path.poses.push_back(MakePose(x, y, yaw));
  }
};

/// Serpentine of `swaths` swaths joined by forward half-turns of radius `r`;
/// `first_left` chooses the side of the first turn.
Plan Serpentine(double length, double r, int swaths, bool first_left)
{
  Plan p;
  for (int i = 0; i < swaths; ++i)
  {
    p.Straight(length);
    if (i + 1 < swaths)
      p.Arc(r, ((i % 2 == 0) == first_left) ? M_PI : -M_PI);
  }
  return p;
}

Plan PivotRing()  // rectangular headland ring with planner pivot corners
{
  Plan p;
  for (int i = 0; i < 4; ++i)
  {
    p.Straight(i % 2 ? 3.0 : 4.0);
    if (i < 3)
      p.Pivot(M_PI / 2.0);
  }
  return p;
}

/// The plan driven perfectly, densely sampled (5 mm, pivots every 0.02 rad).
struct Dense
{
  std::vector<double> x, y, yaw, s, k;  // k: |curvature| of the source interval
};

Dense Densify(const Plan& plan)
{
  Dense d;
  const auto& ps = plan.path.poses;
  double s = 0.0;
  for (std::size_t i = 0; i + 1 < ps.size(); ++i)
  {
    const auto& a = ps[i].pose;
    const auto& b = ps[i + 1].pose;
    const double ya = tf2::getYaw(a.orientation), yb = tf2::getYaw(b.orientation);
    const double dyaw = std::atan2(std::sin(yb - ya), std::cos(yb - ya));
    const double len = std::hypot(b.position.x - a.position.x, b.position.y - a.position.y);
    const int n = std::max(1,
                           std::max(static_cast<int>(std::ceil(len / 0.005)),
                                    static_cast<int>(std::ceil(std::abs(dyaw) / 0.02))));
    const double k = len > 1e-6 ? std::abs(dyaw) / len : 0.0;
    for (int j = 0; j < n; ++j)
    {
      const double t = static_cast<double>(j) / n;
      d.x.push_back(a.position.x + t * (b.position.x - a.position.x));
      d.y.push_back(a.position.y + t * (b.position.y - a.position.y));
      d.yaw.push_back(ya + t * dyaw);
      d.s.push_back(s + t * len);
      d.k.push_back(k);
    }
    s += len;
  }
  const auto& e = ps.back().pose;
  d.x.push_back(e.position.x);
  d.y.push_back(e.position.y);
  d.yaw.push_back(tf2::getYaw(e.orientation));
  d.s.push_back(s);
  d.k.push_back(0.0);
  return d;
}

/// Minimum time along the dense plan under v <= vmax(k) and |dv/dt| <= a.
template <class VMax>
double ProfileTime(const Dense& d, VMax vmax)
{
  const std::size_t n = d.s.size();
  std::vector<double> v(n);
  for (std::size_t i = 0; i < n; ++i)
    v[i] = vmax(d.k[i]);
  v.front() = 0.0;
  v.back() = 0.0;
  for (std::size_t i = 1; i < n; ++i)
    v[i] = std::min(v[i], std::sqrt(v[i - 1] * v[i - 1] + 2 * kAccel * (d.s[i] - d.s[i - 1])));
  for (std::size_t i = n - 1; i-- > 0;)
    v[i] = std::min(v[i], std::sqrt(v[i + 1] * v[i + 1] + 2 * kAccel * (d.s[i + 1] - d.s[i])));
  double t = 0.0;
  for (std::size_t i = 1; i < n; ++i)
  {
    const double ds = d.s[i] - d.s[i - 1];
    const double vm = 0.5 * (v[i] + v[i - 1]);
    if (ds > 0.0)
      t += ds / std::max(vm, 0.02);
  }
  return t;
}

/// A binary raster over a bounding box.
struct Grid
{
  double x0{0}, y0{0}, cell{0.01};
  int nx{0}, ny{0};
  std::vector<int8_t> v;
  Grid(double ax0, double ay0, double ax1, double ay1, double c) : x0(ax0), y0(ay0), cell(c)
  {
    nx = static_cast<int>((ax1 - ax0) / c);
    ny = static_cast<int>((ay1 - ay0) / c);
    v.assign(static_cast<std::size_t>(nx) * ny, 0);
  }
  int8_t& at(int c, int r)
  {
    return v[static_cast<std::size_t>(r) * nx + c];
  }
  double cx(int c) const
  {
    return x0 + (c + .5) * cell;
  }
  double cy(int r) const
  {
    return y0 + (r + .5) * cell;
  }
  void Disc(double bx, double by, double rad)
  {
    for (int r = std::max(0, static_cast<int>((by - rad - y0) / cell));
         r <= std::min(ny - 1, static_cast<int>((by + rad - y0) / cell));
         ++r)
      for (int c = std::max(0, static_cast<int>((bx - rad - x0) / cell));
           c <= std::min(nx - 1, static_cast<int>((bx + rad - x0) / cell));
           ++c)
      {
        const double dx = cx(c) - bx, dy = cy(r) - by;
        if (dx * dx + dy * dy <= rad * rad)
          at(c, r) = 1;
      }
  }
  void Footprint(double x, double y, double yaw)
  {
    const double co = std::cos(yaw), si = std::sin(yaw), rad = std::hypot(kFpX1, kFpHalfW);
    for (int r = std::max(0, static_cast<int>((y - rad - y0) / cell));
         r <= std::min(ny - 1, static_cast<int>((y + rad - y0) / cell));
         ++r)
      for (int c = std::max(0, static_cast<int>((x - rad - x0) / cell));
           c <= std::min(nx - 1, static_cast<int>((x + rad - x0) / cell));
           ++c)
      {
        const double dx = cx(c) - x, dy = cy(r) - y;
        const double bx = dx * co + dy * si, by = -dx * si + dy * co;
        if (bx >= kFpX0 && bx <= kFpX1 && std::abs(by) <= kFpHalfW)
          at(c, r) = 1;
      }
  }
};

struct Arm
{
  std::string name;
  bool speed_limit{false};
  double ff{0.0};
  double speed_slow_override{0.0};  // parameter-only control; 0 = launch-derived
  /// Leave the curvature parameters undeclared: the controller's own
  /// declared defaults apply.
  bool declared_defaults{false};
  /// Further plugin parameters set before configure() (startup validation).
  std::vector<std::pair<std::string, rclcpp::ParameterValue>> startup{};
};

struct Result
{
  bool finished_ok{false};
  bool aborted{false};
  double band_cm2{0}, plan_gap_cm2{0}, terminal_cm2{0}, exec_miss_cm2{0};
  double lat_max_mm{0};
  double exit_max_mm[3]{0, 0, 0};
  double exit_mean_mm[3]{0, 0, 0};
  double fp_excess_mm{0};
  double seconds{0}, t_native_profile{0}, t_feasible_profile{0};
  double cpu_us{0};
  double yaw_sat_pct{0}, supported_pct{0}, min_cmd_v_turn{0}, max_v_turn{0};
  double speed_slow_get{0}, speed_fast_get{0}, max_v_get{0};
};

std::string ParamsFile(double speed)
{
  const char* dir = std::getenv("FTC_CURVATURE_PARAMS_DIR");
  if (!dir)
    return "";
  std::ostringstream f;
  f << dir << "/ftc_" << std::fixed << std::setprecision(2) << speed << ".yaml";
  return f.str();
}

class Harness
{
public:
  Harness(double speed, const Arm& arm, double tau) : tau_(tau)
  {
    rclcpp::NodeOptions options;
    const std::string file = ParamsFile(speed);
    if (!file.empty())
      options.arguments({"--ros-args", "--params-file", file});
    node_ = std::make_shared<nav2::LifecycleNode>("controller_server", "", options);
    const auto set = [this](const std::string& key, const rclcpp::ParameterValue& value)
    {
      const std::string name = std::string(kPlugin) + "." + key;
      if (!node_->has_parameter(name))
        node_->declare_parameter(name, value);
    };
    const auto force = [this](const std::string& key, const rclcpp::ParameterValue& value)
    {
      const std::string name = std::string(kPlugin) + "." + key;
      if (!node_->has_parameter(name))
        node_->declare_parameter(name, value);
      ASSERT_TRUE(node_->set_parameter(rclcpp::Parameter(name, value)).successful) << name;
    };
    // Fallback when no params file: nav2_params_base.yaml + navigation.launch.py.
    set("speed_fast", rclcpp::ParameterValue(speed));
    set("speed_slow", rclcpp::ParameterValue(std::max(0.15, 0.8 * speed)));
    set("speed_fast_threshold", rclcpp::ParameterValue(0.5));
    set("speed_fast_threshold_angle", rclcpp::ParameterValue(10.0));
    set("speed_angular", rclcpp::ParameterValue(45.0));
    set("acceleration", rclcpp::ParameterValue(kAccel));
    set("min_speed_mps", rclcpp::ParameterValue(0.15));
    set("kp_lon", rclcpp::ParameterValue(1.0));
    set("kp_lat", rclcpp::ParameterValue(0.8));
    set("kd_lat", rclcpp::ParameterValue(0.5));
    set("kp_ang", rclcpp::ParameterValue(1.5));
    set("kp_ang_following", rclcpp::ParameterValue(1.0));
    set("derivative_filter_tau", rclcpp::ParameterValue(0.2));
    set("max_cmd_vel_speed", rclcpp::ParameterValue(std::max(0.30, speed)));
    set("max_cmd_vel_ang", rclcpp::ParameterValue(kMaxAng));
    set("max_goal_distance_error", rclcpp::ParameterValue(0.50));
    set("max_goal_angle_error", rclcpp::ParameterValue(30.0));
    set("goal_timeout", rclcpp::ParameterValue(10.0));
    set("max_follow_distance", rclcpp::ParameterValue(2.0));
    set("forward_only", rclcpp::ParameterValue(true));
    // The test map has no zone costmap: confinement has nothing to read.
    force("confine_deviation_to_zone", rclcpp::ParameterValue(false));
    if (!arm.declared_defaults)
    {
      force("curvature_speed_limit_enabled", rclcpp::ParameterValue(arm.speed_limit));
      force("curvature_feedforward_gain", rclcpp::ParameterValue(arm.ff));
    }
    for (const auto& [key, value] : arm.startup)
      force(key, value);
    if (arm.speed_slow_override > 0.0)
      force("speed_slow", rclcpp::ParameterValue(arm.speed_slow_override));

    costmap_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", "/", false);
    for (const auto& p : std::vector<rclcpp::Parameter>{
             rclcpp::Parameter("plugins", std::vector<std::string>{}),
             rclcpp::Parameter("global_frame", "odom"),
             rclcpp::Parameter("robot_base_frame", "base_footprint"),
             rclcpp::Parameter("rolling_window", false),
             rclcpp::Parameter("width", 24),
             rclcpp::Parameter("height", 24),
             rclcpp::Parameter("resolution", 0.05),
             rclcpp::Parameter("origin_x", -6.0),
             rclcpp::Parameter("origin_y", -6.0),
             rclcpp::Parameter("footprint",
                               "[[0.53, 0.275], [0.53, -0.275], [-0.17, -0.275], [-0.17, 0.275]]"),
             rclcpp::Parameter("transform_tolerance", 0.5)})
    {
      if (costmap_->has_parameter(p.get_name()))
        EXPECT_TRUE(costmap_->set_parameter(p).successful) << p.get_name();
      else
        costmap_->declare_parameter(p.get_name(), p.get_parameter_value());
    }
    costmap_->configure();
    tf_ = costmap_->getTfBuffer();
    clock_ = node_->get_clock();
    EXPECT_EQ(rcl_enable_ros_time_override(clock_->get_clock_handle()), RCL_RET_OK);
    SetTime(1000.0);
    tf_->setTransform(Tf("map", "odom", 0, 0, 0), "test", true);
    tf_->setTransform(Tf("base_footprint", "base_link", 0, 0, 0), "test", true);
    PublishRobotTf();
    try
    {
      ftc_.configure(node_, kPlugin, tf_, costmap_);
    }
    catch (...)
    {
      // The destructor does not run for a throwing constructor: clean the
      // configured costmap up here (costmap_test_support.hpp).
      mowgli_nav2_plugins_test::WaitForTfListenerSpinning(*costmap_);
      costmap_->cleanup();
      throw;
    }
    ftc_.activate();
    mn::FTCCurvatureTestAccess::FreeMowingGeometry(ftc_);
  }

  ~Harness()
  {
    ftc_.deactivate();
    ftc_.cleanup();
    mowgli_nav2_plugins_test::WaitForTfListenerSpinning(*costmap_);
    costmap_->cleanup();
  }

  double Param(const std::string& key) const
  {
    return node_->get_parameter(std::string(kPlugin) + "." + key).as_double();
  }
  bool BoolParam(const std::string& key) const
  {
    return node_->get_parameter(std::string(kPlugin) + "." + key).as_bool();
  }
  /// One atomic parameter request, as `ros2 param set` / the GUI would send it.
  rcl_interfaces::msg::SetParametersResult SetAtomically(
      const std::vector<std::pair<std::string, rclcpp::ParameterValue>>& values)
  {
    std::vector<rclcpp::Parameter> params;
    for (const auto& [key, value] : values)
      params.emplace_back(std::string(kPlugin) + "." + key, value);
    return node_->set_parameters_atomically(params);
  }

  mn::FTCController& ftc()
  {
    return ftc_;
  }
  void Place(double x, double y, double yaw)
  {
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    PublishRobotTf();
  }
  void Advance(double dt)
  {
    SetTime(t_ + dt);
    PublishRobotTf();
  }

  Result Drive(const Plan& plan,
               double offset_y,
               double offset_yaw,
               double max_s,
               const std::string& trace = "",
               double external_limit = 0.0,
               std::vector<std::pair<double, double>>* commands = nullptr)
  {
    Result res;
    const auto get = [this](const char* key)
    {
      return node_->get_parameter(std::string(kPlugin) + "." + key).as_double();
    };
    res.speed_slow_get = get("speed_slow");
    res.speed_fast_get = get("speed_fast");
    res.max_v_get = get("max_cmd_vel_speed");

    const Dense dense = Densify(plan);
    double bx0 = 1e9, by0 = 1e9, bx1 = -1e9, by1 = -1e9;
    for (std::size_t i = 0; i < dense.x.size(); ++i)
    {
      bx0 = std::min(bx0, dense.x[i] - 1.0);
      by0 = std::min(by0, dense.y[i] - 1.0);
      bx1 = std::max(bx1, dense.x[i] + 1.0);
      by1 = std::max(by1, dense.y[i] + 1.0);
    }
    Grid band(bx0, by0, bx1, by1, kCell), ideal = band, actual = band;
    Grid fp_ideal(bx0, by0, bx1, by1, kFpCell), fp_actual = fp_ideal;
    for (int r = 0; r < band.ny; ++r)
      for (int c = 0; c < band.nx; ++c)
        for (const auto& b : plan.bands)
          if (std::abs(Lateral(b, band.cx(c), band.cy(r))) <= kHalfBand)
            band.at(c, r) = 1;
    for (std::size_t i = 0; i < dense.x.size(); ++i)
    {
      ideal.Disc(dense.x[i] + kBladeX * std::cos(dense.yaw[i]),
                 dense.y[i] + kBladeX * std::sin(dense.yaw[i]),
                 kBladeR);
      if (i % 2 == 0 || i + 1 == dense.x.size())
        fp_ideal.Footprint(dense.x[i], dense.y[i], dense.yaw[i]);
    }

    const auto& start = plan.path.poses.front().pose;
    Place(start.position.x, start.position.y + offset_y, offset_yaw);
    goal_ = plan.path.poses.back();
    ftc_.newPathReceived(plan.path);
    if (external_limit > 0.0)
      ftc_.setSpeedLimit(external_limit, false);

    std::ofstream out;
    if (!trace.empty())
    {
      out.open(trace);
      out << "t,state_following,x,y,yaw,v,w,cmd_v,cmd_w,supported,curvature,cap,s,lat\n";
    }
    std::size_t k_assoc = 0;
    std::vector<std::array<double, 3>> exits(plan.turn_end_s.size(), {NAN, NAN, NAN});
    const double exit_d[3] = {0.3, 1.0, 2.0};
    double cpu = 0;
    int calls = 0, following = 0, saturated = 0, supported = 0;
    res.min_cmd_v_turn = 1e9;
    for (int i = 0; i < static_cast<int>(max_s / kDt); ++i)
    {
      geometry_msgs::msg::Twist odom;
      odom.linear.x = v_;
      odom.angular.z = w_;
      geometry_msgs::msg::TwistStamped cmd;
      const auto t0 = std::chrono::steady_clock::now();
      try
      {
        cmd = ftc_.computeVelocityCommands(Pose{}, odom, nullptr, nav_msgs::msg::Path{}, goal_);
      }
      catch (const nav2_core::ControllerException& e)
      {
        res.aborted = true;
        std::cout << "[abort] " << e.what() << "\n";
        break;
      }
      cpu +=
          std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
      ++calls;
      if (commands)
        commands->emplace_back(cmd.twist.linear.x, cmd.twist.angular.z);
      const bool fol = mn::FTCCurvatureTestAccess::Following(ftc_);
      const auto& cv = mn::FTCCurvatureTestAccess::Curvature(ftc_);
      if (fol)
      {
        ++following;
        saturated += std::abs(cmd.twist.angular.z) >= kMaxAng - 1e-6;
        supported += cv.supported;
      }
      if (mn::FTCCurvatureTestAccess::Finished(ftc_))
      {
        res.finished_ok = mn::FTCCurvatureTestAccess::FinishedOk(ftc_);
        res.seconds = i * kDt;
        break;
      }
      const double a = std::min(1.0, kDt / tau_);
      v_ += (cmd.twist.linear.x - v_) * a;
      w_ += (cmd.twist.angular.z - w_) * a;
      constexpr int kSub = 5;
      double lat = 0.0;
      for (int sub = 0; sub < kSub; ++sub)
      {
        x_ += v_ * std::cos(yaw_) * kDt / kSub;
        y_ += v_ * std::sin(yaw_) * kDt / kSub;
        yaw_ += w_ * kDt / kSub;
        actual.Disc(x_ + kBladeX * std::cos(yaw_), y_ + kBladeX * std::sin(yaw_), kBladeR);
        fp_actual.Footprint(x_, y_, yaw_);
        // Associate with the plan: bounded window, monotonic in progress.
        const std::size_t lo = k_assoc > 60 ? k_assoc - 60 : 0;
        const std::size_t hi = std::min(dense.x.size(), k_assoc + 300);
        double best = 1e18;
        for (std::size_t k = lo; k < hi; ++k)
        {
          const double d2 =
              (dense.x[k] - x_) * (dense.x[k] - x_) + (dense.y[k] - y_) * (dense.y[k] - y_);
          if (d2 < best)
          {
            best = d2;
            k_assoc = k;
          }
        }
        lat = -(x_ - dense.x[k_assoc]) * std::sin(dense.yaw[k_assoc]) +
              (y_ - dense.y[k_assoc]) * std::cos(dense.yaw[k_assoc]);
        res.lat_max_mm = std::max(res.lat_max_mm, 1000.0 * std::abs(lat));
        const double sk = dense.s[k_assoc];
        for (std::size_t tn = 0; tn < plan.turn_end_s.size(); ++tn)
          for (int j = 0; j < 3; ++j)
            if (std::isnan(exits[tn][j]) && sk >= plan.turn_end_s[tn] + exit_d[j] &&
                plan.turn_end_s[tn] + exit_d[j] <= plan.turn_next_s[tn])
              exits[tn][j] = 1000.0 * std::abs(lat);
        if (dense.k[k_assoc] > 1.0)
        {
          res.min_cmd_v_turn = std::min(res.min_cmd_v_turn, cmd.twist.linear.x);
          res.max_v_turn = std::max(res.max_v_turn, v_);
        }
      }
      if (out)
        out << std::fixed << std::setprecision(4) << t_ << "," << fol << "," << x_ << "," << y_
            << "," << yaw_ << "," << v_ << "," << w_ << "," << cmd.twist.linear.x << ","
            << cmd.twist.angular.z << "," << cv.supported << "," << cv.curvature << ","
            << (std::isfinite(cv.speed_cap) ? cv.speed_cap : -1.0) << "," << dense.s[k_assoc] << ","
            << lat << "\n";
      Advance(kDt);
      res.seconds = (i + 1) * kDt;
    }
    if (res.min_cmd_v_turn > 1e8)
      res.min_cmd_v_turn = 0.0;
    // Oracle classification of the required band.
    const auto& e = plan.path.poses.back().pose;
    for (int r = 0; r < band.ny; ++r)
      for (int c = 0; c < band.nx; ++c)
      {
        if (!band.at(c, r))
          continue;
        res.band_cm2 += 1.0;
        const double px = band.cx(c), py = band.cy(r);
        const bool terminal =
            std::hypot(px - start.position.x, py - start.position.y) < kTerminalM ||
            std::hypot(px - e.position.x, py - e.position.y) < kTerminalM;
        if (!ideal.at(c, r))
          res.plan_gap_cm2 += 1.0;
        else if (actual.at(c, r))
          continue;
        else if (terminal)
          res.terminal_cm2 += 1.0;
        else
          res.exec_miss_cm2 += 1.0;
      }
    // Footprint excursion beyond the ideal footprint sweep (8-neighbour chamfer).
    std::vector<double> dist(fp_ideal.v.size(), 1e9);
    for (std::size_t i = 0; i < dist.size(); ++i)
      if (fp_ideal.v[i])
        dist[i] = 0.0;
    const int nx = fp_ideal.nx, ny = fp_ideal.ny;
    const auto relax = [&](int c, int r, int dc, int dr, double w)
    {
      const int c2 = c + dc, r2 = r + dr;
      if (c2 < 0 || r2 < 0 || c2 >= nx || r2 >= ny)
        return;
      double& d = dist[static_cast<std::size_t>(r) * nx + c];
      d = std::min(d, dist[static_cast<std::size_t>(r2) * nx + c2] + w);
    };
    for (int r = 0; r < ny; ++r)
      for (int c = 0; c < nx; ++c)
      {
        relax(c, r, -1, 0, 1.0);
        relax(c, r, 0, -1, 1.0);
        relax(c, r, -1, -1, M_SQRT2);
        relax(c, r, 1, -1, M_SQRT2);
      }
    for (int r = ny - 1; r >= 0; --r)
      for (int c = nx - 1; c >= 0; --c)
      {
        relax(c, r, 1, 0, 1.0);
        relax(c, r, 0, 1, 1.0);
        relax(c, r, 1, 1, M_SQRT2);
        relax(c, r, -1, 1, M_SQRT2);
      }
    for (std::size_t i = 0; i < dist.size(); ++i)
      if (fp_actual.v[i])
        res.fp_excess_mm = std::max(res.fp_excess_mm, 1000.0 * kFpCell * dist[i]);
    for (int j = 0; j < 3; ++j)
    {
      int n = 0;
      for (const auto& ex : exits)
        if (!std::isnan(ex[j]))
        {
          res.exit_max_mm[j] = std::max(res.exit_max_mm[j], ex[j]);
          res.exit_mean_mm[j] += ex[j];
          ++n;
        }
      if (n)
        res.exit_mean_mm[j] /= n;
    }
    res.cpu_us = calls ? cpu / calls : 0;
    res.yaw_sat_pct = following ? 100.0 * saturated / following : 0;
    res.supported_pct = following ? 100.0 * supported / following : 0;
    const double vf = res.speed_fast_get, vs = res.speed_slow_get;
    res.t_native_profile = ProfileTime(dense,
                                       [&](double k)
                                       {
                                         return k * 0.5 > 0.05 ? vs : vf;
                                       });
    res.t_feasible_profile =
        ProfileTime(dense,
                    [&](double k)
                    {
                      const double base = k * 0.5 > 0.05 ? vs : vf;
                      return k > 0.0 ? std::min(base, std::max(0.08, kMaxAng * 0.85 / k)) : base;
                    });
    return res;
  }

private:
  /// Signed lateral offset of (px, py) from band b's centreline, or NaN when
  /// the point is outside the band's longitudinal span.
  static double Lateral(const Plan::Band& b, double px, double py)
  {
    if (b.radius == 0.0)
    {
      const double dx = px - b.x, dy = py - b.y;
      const double s = dx * std::cos(b.yaw) + dy * std::sin(b.yaw);
      if (s < 0.0 || s > b.length)
        return NAN;
      return -dx * std::sin(b.yaw) + dy * std::cos(b.yaw);
    }
    const double sgn = b.radius > 0 ? 1.0 : -1.0, r = std::abs(b.radius);
    const double cx = b.x - sgn * r * std::sin(b.yaw), cy = b.y + sgn * r * std::cos(b.yaw);
    const double a0 = std::atan2(b.y - cy, b.x - cx);
    double a = std::atan2(py - cy, px - cx) - a0;
    a = std::atan2(std::sin(a), std::cos(a)) * sgn;
    if (a < 0.0)
      a += 2.0 * M_PI;
    if (a * r > b.length)
      return NAN;
    return sgn * (r - std::hypot(px - cx, py - cy));
  }
  static std::int64_t Ns(double s)
  {
    return static_cast<std::int64_t>(std::llround(s * 1e9));
  }
  void SetTime(double t)
  {
    t_ = t;
    EXPECT_EQ(rcl_set_ros_time_override(clock_->get_clock_handle(), Ns(t)), RCL_RET_OK);
  }
  geometry_msgs::msg::TransformStamped Tf(
      const std::string& parent, const std::string& child, double x, double y, double yaw) const
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = rclcpp::Time(Ns(t_), RCL_ROS_TIME);
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.rotation.z = std::sin(yaw / 2.0);
    t.transform.rotation.w = std::cos(yaw / 2.0);
    return t;
  }
  void PublishRobotTf()
  {
    tf_->setTransform(Tf("odom", "base_footprint", x_, y_, yaw_), "test", false);
  }

  std::shared_ptr<nav2::LifecycleNode> node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  mn::FTCController ftc_;
  Pose goal_;
  double tau_{0.15};
  double t_{0}, x_{0}, y_{0}, yaw_{0}, v_{0}, w_{0};
};

const Arm kBase{"base", false, 0.0, 0.0};
const Arm kCap{"cap", true, 0.0, 0.0};
const Arm kCapFf{"cap_ff", true, 1.0, 0.0};
const Arm kFf{"ff", false, 1.0, 0.0};
const Arm kSlow{"slow016", false, 0.0, 0.16};
const Arm kDefaults{"defaults", false, 0.0, 0.0, true};

class FtcCurvature : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
  static Result Run(const std::string& name,
                    const Plan& plan,
                    double speed,
                    const Arm& arm,
                    double dy,
                    double dyaw,
                    double tau)
  {
    std::string trace;
    if (const char* dir = std::getenv("FTC_CURVATURE_TRACE_DIR"))
    {
      std::ostringstream f;
      f << dir << "/" << name << "_v" << speed << "_" << arm.name << "_dy" << dy << "_tau" << tau
        << ".csv";
      trace = f.str();
    }
    Harness h(speed, arm, tau);
    const Result r = h.Drive(plan, dy, dyaw, 400.0, trace);
    std::cout << std::fixed << std::setprecision(1) << "[matrix] " << name << " v=" << speed
              << " arm=" << arm.name << std::setprecision(2) << " dy=" << dy << " tau=" << tau
              << " GET(speed_slow=" << std::setprecision(4) << r.speed_slow_get
              << " speed_fast=" << r.speed_fast_get << " max_v=" << r.max_v_get << ")"
              << std::setprecision(1) << " finished_ok=" << r.finished_ok
              << " aborted=" << r.aborted << " band_cm2=" << r.band_cm2
              << " plan_gap_cm2=" << r.plan_gap_cm2 << " terminal_cm2=" << r.terminal_cm2
              << " exec_miss_cm2=" << r.exec_miss_cm2 << " lat_max_mm=" << r.lat_max_mm
              << " exit03_max=" << r.exit_max_mm[0] << " exit1_max=" << r.exit_max_mm[1]
              << " exit2_max=" << r.exit_max_mm[2] << " exit03_mean=" << r.exit_mean_mm[0]
              << " exit1_mean=" << r.exit_mean_mm[1] << " fp_excess_mm=" << r.fp_excess_mm
              << " seconds=" << r.seconds << " t_native_profile=" << r.t_native_profile
              << " t_feasible_profile=" << r.t_feasible_profile << " yaw_sat_pct=" << r.yaw_sat_pct
              << " supported_pct=" << r.supported_pct << std::setprecision(3)
              << " turn_min_cmd_v=" << r.min_cmd_v_turn << " turn_max_v=" << r.max_v_turn
              << std::setprecision(1) << " cpu_us_per_cycle=" << r.cpu_us << "\n";
    return r;
  }
};

TEST(CurvatureMath, PureHelpers)
{
  EXPECT_NEAR(*mn::PlanIntervalCurvature(0.09, 0.375), 4.1667, 1e-3);
  EXPECT_FALSE(mn::PlanIntervalCurvature(0.0, 1.0));  // pivot twin: unsupported
  EXPECT_DOUBLE_EQ(*mn::PlanIntervalCurvature(0.0, 0.0), 0.0);  // duplicate pose
  EXPECT_FALSE(mn::PlanIntervalCurvature(0.05, M_PI / 2));  // corner, not an arc
  EXPECT_NEAR(*mn::PlanIntervalCurvature(1.0, 2 * M_PI + 0.1), 0.1, 1e-9);  // wrapped
  // R 0.20 with 15 % reserve of 0.8 rad/s.
  EXPECT_NEAR(mn::CurvatureFeasibleSpeed(5.0, 0.8, 0.15, 0.08), 0.136, 1e-12);
  EXPECT_TRUE(std::isnan(mn::CurvatureFeasibleSpeed(50.0, 0.8, 0.15, 0.08)));  // infeasible
  EXPECT_TRUE(std::isinf(mn::CurvatureFeasibleSpeed(0.0, 0.8, 0.15, 0.08)));
  // Braking: at the arc the limit; 1 m away v^2 = 0.136^2 + 2 * 0.2 * 1.
  EXPECT_NEAR(mn::BrakingAllowedSpeed(0.136, 0.0, 0.2, 0.0), 0.136, 1e-12);
  EXPECT_NEAR(mn::BrakingAllowedSpeed(0.136, 1.0, 0.2, 0.0), std::sqrt(0.136 * 0.136 + 0.4), 1e-12);
  // Response delay shortens the usable distance.
  EXPECT_LT(mn::BrakingAllowedSpeed(0.136, 1.0, 0.2, 0.3),
            mn::BrakingAllowedSpeed(0.136, 1.0, 0.2, 0.0));
  EXPECT_DOUBLE_EQ(mn::CurvatureFeedforward(0.0, 0.2, 4.0, 1.0, 0.0, 0.8), 0.0);  // off
  EXPECT_DOUBLE_EQ(mn::CurvatureFeedforward(1.0, 0.0, 4.0, 1.0, 0.0, 0.8), 0.0);  // stopped
  EXPECT_DOUBLE_EQ(mn::CurvatureFeedforward(1.0, -0.1, 4.0, 1.0, 0.0, 0.8), 0.0);  // reversing
  EXPECT_NEAR(mn::CurvatureFeedforward(1.0, 0.3, 0.5, 1.0, 0.3 * 0.5, 0.8), 0.0, 1e-12);
}

TEST(CurvatureMath, ProjectionStaysOnTheLegBehindTheCarrot)
{
  // Straight 2 m along x, then a pivot corner, then 2 m along y.
  std::vector<mn::CurvaturePlanPose> plan;
  for (int i = 0; i <= 20; ++i)
    plan.push_back({0.1 * i, 0.0, 0.0});
  plan.push_back({2.0, 0.0, M_PI / 2});
  for (int i = 1; i <= 20; ++i)
    plan.push_back({2.0, 0.1 * i, M_PI / 2});
  const auto at = [&](std::size_t i)
  {
    return plan[i];
  };
  // Robot at x 1.23, 0.05 left; carrot at index 15 (x 1.5).
  auto p = mn::ProjectRobotBehindCarrot(at, plan.size(), 15, 0.0, 1.23, 0.05, 1.0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->index, 12u);
  EXPECT_NEAR(p->t, 0.3, 1e-9);
  EXPECT_NEAR(p->distance, 0.05, 1e-9);
  // Never forward of the carrot: a robot ahead projects onto the carrot.
  p = mn::ProjectRobotBehindCarrot(at, plan.size(), 15, 0.5, 1.9, 0.0, 1.0);
  ASSERT_TRUE(p);
  EXPECT_EQ(p->index, 15u);
  EXPECT_NEAR(p->t, 0.5, 1e-9);
  // Carrot on the second leg: the search stops at the pivot corner and never
  // projects onto the first leg, however close the robot is to it.
  p = mn::ProjectRobotBehindCarrot(at, plan.size(), 25, 0.0, 1.5, 0.0, 3.0);
  EXPECT_FALSE(p);  // 0.5 m off the second leg's window
  // Robot farther back than the search may look: no projection.
  EXPECT_FALSE(mn::ProjectRobotBehindCarrot(at, plan.size(), 15, 0.0, 0.2, 0.0, 0.5));
}

TEST(CurvatureMath, CapHoldsFromTheRobotNotTheCarrot)
{
  // R 0.20 half-turn then a straight.
  std::vector<mn::CurvaturePlanPose> plan;
  for (int i = 0; i <= 7; ++i)
  {
    const double a = M_PI * i / 7;
    plan.push_back({0.2 * std::sin(a), 0.2 - 0.2 * std::cos(a), a});
  }
  for (int i = 1; i <= 20; ++i)
    plan.push_back({-0.1 * i, 0.4, M_PI});
  const auto at = [&](std::size_t i)
  {
    return plan[i];
  };
  // Robot on the arc's last interval, carrot already on the straight.
  const mn::CurvatureRobotOnPlan robot{6, 0.5, 0.0};
  const auto c = mn::CurvatureAhead(at, plan.size(), robot, 0.36, 0.8, 0.15, 0.08, 0.2, 0.3);
  ASSERT_TRUE(c.supported);
  EXPECT_NEAR(c.speed_cap, 0.136, 2e-3);
  EXPECT_NEAR(c.curvature, 5.04, 0.01);
  // 0.36 m/s x 0.3 s = 0.108 m ahead is already the straight.
  EXPECT_DOUBLE_EQ(c.anticipated_curvature, 0.0);
  // Lead accumulated along the plan: a carrot a whole half-turn ahead is +pi
  // (a wrapped difference could read -pi and flip the correction's sign).
  EXPECT_NEAR(*mn::PlanHeadingChange(at, 0, 0.0, 7, 0.0), M_PI, 1e-9);
  EXPECT_NEAR(*mn::PlanHeadingChange(at, 2, 0.5, 2, 0.75), 0.25 * M_PI / 7, 1e-9);
  EXPECT_FALSE(mn::PlanHeadingChange(at, 5, 0.0, 4, 0.0));  // behind
  // Robot on the straight: released.
  const mn::CurvatureRobotOnPlan out{9, 0.0, 0.0};
  const auto d = mn::CurvatureAhead(at, plan.size(), out, 0.36, 0.8, 0.15, 0.08, 0.2, 0.3);
  ASSERT_TRUE(d.supported);
  EXPECT_TRUE(std::isinf(d.speed_cap));
}

TEST(CurvatureMath, MinTurnExemptsShortBendsButNotTheTurnAround)
{
  // Straight, a 12 deg boundary kink over 0.06 m (k 3.5), straight, then an
  // R 0.20 half-turn: the two bend shapes of a real headland-ring plan.
  std::vector<mn::CurvaturePlanPose> plan;
  for (int i = 0; i <= 10; ++i)
    plan.push_back({0.1 * i, 0.0, 0.0});
  const double kink = 12.0 * M_PI / 180.0;
  plan.push_back({1.03, 0.0, kink / 2});
  plan.push_back({1.06, 0.0, kink});
  for (int i = 1; i <= 10; ++i)
    plan.push_back({1.06 + 0.1 * i, 0.0, kink});
  const std::size_t arc_start = plan.size() - 1;
  for (int i = 1; i <= 7; ++i)
  {
    const double a = kink + M_PI * i / 7;
    plan.push_back({plan.back().x + 0.2 * M_PI / 7, plan.back().y, a});
  }
  const auto at = [&](std::size_t i)
  {
    return plan.at(i);
  };
  // Runs: the kink is its own two intervals, the arc one run of pi.
  EXPECT_NEAR(mn::CurvatureRunHeadingChange(at, plan.size(), 10, 1.0), kink, 1e-9);
  EXPECT_NEAR(mn::CurvatureRunHeadingChange(at, plan.size(), 11, 1.0), kink, 1e-9);
  EXPECT_NEAR(mn::CurvatureRunHeadingChange(at, plan.size(), arc_start + 3, 1.0), M_PI, 1e-9);
  EXPECT_DOUBLE_EQ(mn::CurvatureRunHeadingChange(at, plan.size(), 3, 1.0), 0.0);  // straight
  // Too gentle to count as part of a run.
  EXPECT_DOUBLE_EQ(mn::CurvatureRunHeadingChange(at, plan.size(), 10, 5.0), 0.0);
  const double min_turn = 40.0 * M_PI / 180.0;
  // Robot just before the kink: native cap there, none with the exemption.
  const mn::CurvatureRobotOnPlan before_kink{9, 0.5, 0.0};
  const auto capped =
      mn::CurvatureAhead(at, plan.size(), before_kink, 0.45, 0.8, 0.05, 0.15, 0.2, 0.3);
  ASSERT_TRUE(capped.supported);
  EXPECT_LT(capped.speed_cap, 0.45);
  const auto exempt = mn::CurvatureAhead(
      at, plan.size(), before_kink, 0.45, 0.8, 0.05, 0.15, 0.2, 0.3, min_turn, 1.69);
  ASSERT_TRUE(exempt.supported);
  EXPECT_TRUE(std::isinf(exempt.speed_cap));
  EXPECT_DOUBLE_EQ(exempt.max_abs_curvature, capped.max_abs_curvature);
  // On the half-turn the exemption changes nothing.
  const mn::CurvatureRobotOnPlan on_arc{arc_start + 2, 0.5, 0.0};
  const auto arc = mn::CurvatureAhead(at, plan.size(), on_arc, 0.2, 0.8, 0.05, 0.15, 0.2, 0.3);
  const auto arc_exempt =
      mn::CurvatureAhead(at, plan.size(), on_arc, 0.2, 0.8, 0.05, 0.15, 0.2, 0.3, min_turn, 1.69);
  ASSERT_TRUE(arc_exempt.supported);
  EXPECT_DOUBLE_EQ(arc_exempt.speed_cap, arc.speed_cap);
  EXPECT_NEAR(arc_exempt.speed_cap, 0.8 * 0.95 / (M_PI / 7 / (0.2 * M_PI / 7)), 1e-9);
}

TEST(CurvatureMath, DenseGeometryAndIntervalBudgetNeverInventSupport)
{
  ASSERT_TRUE(mn::PlanIntervalCurvature(0.005, 0.025));
  EXPECT_NEAR(*mn::PlanIntervalCurvature(0.005, 0.025), 5.0, 1e-12);
  std::vector<mn::CurvaturePlanPose> plan;
  for (int i = 0; i <= 200; ++i)
    plan.push_back({i * 0.005, 0.0, 0.0});
  const auto at = [&](std::size_t i)
  {
    return plan.at(i);
  };
  // The robot is behind the 96-interval budget, but still inside the distance
  // acceptance band of its last point: that boundary is not a real projection.
  EXPECT_FALSE(mn::ProjectRobotBehindCarrot(at, plan.size(), 160, 0.0, 0.0, 0.0, 1.0));
  // A genuine interior projection remains supported on the same dense plan.
  ASSERT_TRUE(mn::ProjectRobotBehindCarrot(at, plan.size(), 160, 0.0, 0.6, 0.0, 1.0));
}

TEST(CurvatureMath, UnsupportedGeometryAndInfeasibleFloorSuppressWholeCorrection)
{
  std::vector<mn::CurvaturePlanPose> plan{{0, 0, 0}, {0.1, 0, 0}, {0.1, 0.1, M_PI / 2}};
  const auto at = [&](std::size_t i)
  {
    return plan.at(i);
  };
  EXPECT_FALSE(mn::PlanHeadingChange(at, 0, 0.5, 1, 0.05));
  // A configured crawl of .08 would require .8 rad/s on this k=10 arc,
  // beyond the configured .3 rad/s yaw limit. Reject evidence, never floor up.
  plan = {{0, 0, 0}, {0.1, 0, 1.0}, {0.2, 0, 2.0}};
  EXPECT_TRUE(std::isnan(mn::CurvatureFeasibleSpeed(10.0, 0.3, 0.15, 0.08)));
  EXPECT_FALSE(
      mn::CurvatureAhead(at, plan.size(), {0, 0.5, 0}, 0.2, 0.3, 0.15, 0.08, 0.2, 0.3).supported);
  EXPECT_DOUBLE_EQ(
      mn::CurvatureFeedforward(std::numeric_limits<double>::infinity(), 0.2, 0.0, 1.0, 0.0, 0.8),
      0.0);
}

TEST_F(FtcCurvature, CachedEvidenceCannotSurviveUnsupportedCarrotAdvancement)
{
  Harness h(0.30, kCapFf, 0.15);
  mn::FTCCurvatureTestAccess::CacheBeforeUnsupportedConnector(h.ftc(), false);
  ASSERT_TRUE(mn::FTCCurvatureTestAccess::Allowed(h.ftc()));
  mn::FTCCurvatureTestAccess::CacheBeforeUnsupportedConnector(h.ftc(), true);
  ASSERT_TRUE(mn::FTCCurvatureTestAccess::Curvature(h.ftc()).supported);
  EXPECT_FALSE(mn::FTCCurvatureTestAccess::Allowed(h.ftc()));
}

TEST_F(FtcCurvature, NonFiniteStartupGainIsRejectedBeforeActivation)
{
  Arm invalid = kCapFf;
  invalid.ff = std::numeric_limits<double>::infinity();
  EXPECT_THROW(Harness(0.30, invalid, 0.15), nav2_core::ControllerException);
}

/// Each numeric curvature knob and its accepted maximum (the minimum is 0).
const std::vector<std::pair<std::string, double>> kCurvatureKnobs{
    {"curvature_feedforward_gain", mn::kCurvatureFeedforwardGainMax},
    {"curvature_speed_yaw_reserve", mn::kCurvatureYawReserveMax},
    {"curvature_speed_response_s", mn::kCurvatureResponseMaxS},
    {"curvature_speed_min_turn_deg", mn::kCurvatureMinTurnMaxDeg},
};

std::vector<double> OutOfRange(double max)
{
  return {std::numeric_limits<double>::quiet_NaN(),
          std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity(),
          -1e-9,
          max + 1e-9};
}

// Startup: every numeric knob is range-checked at its boundaries, whether or
// not its feature is switched on; both ends of the range are accepted.
TEST_F(FtcCurvature, StartupRangesAreCheckedAtTheBoundaries)
{
  for (const auto& [key, max] : kCurvatureKnobs)
  {
    for (const double bad : OutOfRange(max))
    {
      Arm arm = kBase;
      arm.startup = {{key, rclcpp::ParameterValue(bad)}};
      EXPECT_THROW(Harness(0.30, arm, 0.15), nav2_core::ControllerException) << key << "=" << bad;
    }
    for (const double good : {0.0, max})
    {
      Arm arm = kBase;
      arm.startup = {{key, rclcpp::ParameterValue(good)}};
      EXPECT_NO_THROW({ Harness h(0.30, arm, 0.15); }) << key << "=" << good;
    }
  }
}

// Live: the same ranges; a rejected value leaves the last accepted one, both
// inside the controller and in the node's parameter store.
TEST_F(FtcCurvature, LiveRangesRejectOutsideAndKeepTheLastAcceptedValue)
{
  Harness h(0.30, kBase, 0.15);
  const auto value = [&h](const std::string& key)
  {
    const auto s = mn::FTCCurvatureTestAccess::Settings(h.ftc());
    if (key == "curvature_feedforward_gain")
      return std::get<1>(s);
    if (key == "curvature_speed_yaw_reserve")
      return std::get<2>(s);
    if (key == "curvature_speed_response_s")
      return std::get<3>(s);
    return std::get<4>(s);
  };
  for (const auto& [key, max] : kCurvatureKnobs)
  {
    for (const double good : {max, 0.0})
    {
      EXPECT_TRUE(h.SetAtomically({{key, rclcpp::ParameterValue(good)}}).successful) << key;
      EXPECT_DOUBLE_EQ(value(key), good) << key;
    }
    for (const double bad : OutOfRange(max))
    {
      EXPECT_FALSE(h.SetAtomically({{key, rclcpp::ParameterValue(bad)}}).successful)
          << key << "=" << bad;
      EXPECT_DOUBLE_EQ(value(key), 0.0) << key << "=" << bad;
      EXPECT_DOUBLE_EQ(h.Param(key), 0.0) << key << "=" << bad;
    }
  }
  EXPECT_TRUE(h.SetAtomically({{"curvature_speed_limit_enabled", rclcpp::ParameterValue(true)}})
                  .successful);
  EXPECT_TRUE(std::get<0>(mn::FTCCurvatureTestAccess::Settings(h.ftc())));
}

// A rejected multi-key request applies NO curvature value, whichever key is
// the invalid one and wherever it sits in the request: the controller keeps
// its settings and drives exactly like an untouched one.
TEST_F(FtcCurvature, RejectedMultiKeyRequestChangesNothing)
{
  using V = rclcpp::ParameterValue;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const std::vector<std::vector<std::pair<std::string, V>>> rejected{
      {{"curvature_feedforward_gain", V(1.0)}, {"curvature_speed_response_s", V(5.0)}},
      {{"curvature_speed_response_s", V(5.0)}, {"curvature_feedforward_gain", V(1.0)}},
      {{"curvature_speed_limit_enabled", V(true)}, {"curvature_speed_yaw_reserve", V(nan)}},
      // Invalid NON-curvature key after / before the curvature ones.
      {{"curvature_speed_limit_enabled", V(true)},
       {"curvature_feedforward_gain", V(1.0)},
       {"speed_fast", V(5.0)}},
      {{"speed_fast", V(5.0)}, {"curvature_speed_limit_enabled", V(true)}},
  };
  const Plan plan = Serpentine(3.0, 0.20, 2, true);
  std::vector<std::pair<double, double>> untouched, after_rejections;
  {
    Harness h(0.45, kBase, 0.15);
    h.Drive(plan, 0.03, 0.0, 120.0, "", 0.0, &untouched);
  }
  Result r;
  {
    Harness h(0.45, kBase, 0.15);
    const auto before = mn::FTCCurvatureTestAccess::Settings(h.ftc());
    for (std::size_t i = 0; i < rejected.size(); ++i)
    {
      EXPECT_FALSE(h.SetAtomically(rejected[i]).successful) << i;
      EXPECT_EQ(mn::FTCCurvatureTestAccess::Settings(h.ftc()), before) << i;
      EXPECT_FALSE(h.BoolParam("curvature_speed_limit_enabled")) << i;
      EXPECT_DOUBLE_EQ(h.Param("curvature_feedforward_gain"), 0.0) << i;
      EXPECT_DOUBLE_EQ(h.Param("speed_fast"), 0.45) << i;
    }
    r = h.Drive(plan, 0.03, 0.0, 120.0, "", 0.0, &after_rejections);
    // Positive control: the same keys in an all-valid request are applied.
    ASSERT_TRUE(h.SetAtomically({{"curvature_speed_limit_enabled", V(true)},
                                 {"curvature_feedforward_gain", V(1.0)}})
                    .successful);
    EXPECT_TRUE(std::get<0>(mn::FTCCurvatureTestAccess::Settings(h.ftc())));
    EXPECT_DOUBLE_EQ(std::get<1>(mn::FTCCurvatureTestAccess::Settings(h.ftc())), 1.0);
  }
  EXPECT_TRUE(r.finished_ok);
  EXPECT_DOUBLE_EQ(r.supported_pct, 0.0);
  ASSERT_EQ(untouched.size(), after_rejections.size());
  for (std::size_t i = 0; i < untouched.size(); ++i)
  {
    ASSERT_EQ(untouched[i].first, after_rejections[i].first) << i;
    ASSERT_EQ(untouched[i].second, after_rejections[i].second) << i;
  }
}

// Every non-nominal mode suppresses the whole correction: avoidance offsets,
// reverse escape, obstacle hold, restart recovery, turn fallback and stall.
TEST_F(FtcCurvature, NonNominalModesApplyNoCorrection)
{
  const Plan plan = Serpentine(3.0, 0.20, 2, true);
  for (int mode = 0; mode < 8; ++mode)
  {
    Harness h(0.45, kCapFf, 0.15);
    h.ftc().newPathReceived(plan.path);
    h.Place(0.0, 0.02, 0.0);
    mn::FTCCurvatureTestAccess::ForceFollowing(h.ftc());
    h.Advance(0.1);
    // Positive control: nominal following projects the robot.
    ASSERT_TRUE(mn::FTCCurvatureTestAccess::Allowed(h.ftc())) << mode;
    mn::FTCCurvatureTestAccess::Update(h.ftc());
    ASSERT_TRUE(mn::FTCCurvatureTestAccess::Curvature(h.ftc()).supported) << mode;
    mn::FTCCurvatureTestAccess::SetMode(h.ftc(), mode, true);
    EXPECT_FALSE(mn::FTCCurvatureTestAccess::Allowed(h.ftc())) << mode;
    mn::FTCCurvatureTestAccess::Update(h.ftc());
    EXPECT_FALSE(mn::FTCCurvatureTestAccess::Curvature(h.ftc()).supported) << mode;
    mn::FTCCurvatureTestAccess::SetMode(h.ftc(), mode, false);
  }
}

// A low external speed limit (collision_monitor slowdown, setSpeedLimit) below
// every curvature cap: the speed cap changes nothing, command for command.
TEST_F(FtcCurvature, LowExternalLimitIsBitIdentical)
{
  const Plan plan = Serpentine(3.0, 0.24, 2, true);
  std::vector<std::pair<double, double>> off, on;
  {
    Harness h(0.45, kBase, 0.15);
    h.Drive(plan, 0.03, 0.0, 120.0, "", 0.12, &off);
  }
  {
    Harness h(0.45, kCap, 0.15);
    h.Drive(plan, 0.03, 0.0, 120.0, "", 0.12, &on);
  }
  ASSERT_EQ(off.size(), on.size());
  for (std::size_t i = 0; i < off.size(); ++i)
  {
    ASSERT_EQ(off[i].first, on[i].first) << i;
    ASSERT_EQ(off[i].second, on[i].second) << i;
  }
}

// A plan of straights and pivot corners has no arc: bit-identical.
TEST_F(FtcCurvature, PivotRingIsBitIdentical)
{
  std::vector<std::pair<double, double>> off, on;
  {
    Harness h(0.45, kBase, 0.15);
    h.Drive(PivotRing(), 0.05, 0.08, 200.0, "", 0.0, &off);
  }
  {
    Harness h(0.45, kCapFf, 0.15);
    h.Drive(PivotRing(), 0.05, 0.08, 200.0, "", 0.0, &on);
  }
  ASSERT_EQ(off.size(), on.size());
  for (std::size_t i = 0; i < off.size(); ++i)
  {
    ASSERT_EQ(off[i].first, on[i].first) << i;
    ASSERT_EQ(off[i].second, on[i].second) << i;
  }
}

// The controller's own declared defaults are the native law: every curvature
// knob is off, no curvature evidence is ever computed, and the
// commands equal those of an explicitly-off controller, command for command.
TEST_F(FtcCurvature, DeclaredDefaultsAreOffAndNeverEngage)
{
  const Plan plan = Serpentine(3.0, 0.20, 2, true);
  std::vector<std::pair<double, double>> defaults, off;
  Result r_defaults, r_off;
  {
    Harness h(0.45, kDefaults, 0.15);
    EXPECT_FALSE(h.BoolParam("curvature_speed_limit_enabled"));
    EXPECT_DOUBLE_EQ(h.Param("curvature_feedforward_gain"), 0.0);
    EXPECT_DOUBLE_EQ(h.Param("curvature_speed_min_turn_deg"), 0.0);
    r_defaults = h.Drive(plan, 0.03, 0.0, 120.0, "", 0.0, &defaults);
  }
  {
    Harness h(0.45, kBase, 0.15);
    r_off = h.Drive(plan, 0.03, 0.0, 120.0, "", 0.0, &off);
  }
  EXPECT_TRUE(r_defaults.finished_ok);
  EXPECT_DOUBLE_EQ(r_defaults.supported_pct, 0.0);
  EXPECT_DOUBLE_EQ(r_off.supported_pct, 0.0);
  ASSERT_EQ(defaults.size(), off.size());
  for (std::size_t i = 0; i < defaults.size(); ++i)
  {
    ASSERT_EQ(defaults[i].first, off[i].first) << i;
    ASSERT_EQ(defaults[i].second, off[i].second) << i;
  }
}

}  // namespace
