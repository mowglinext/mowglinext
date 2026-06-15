// Copyright (C) 2026 Cedric <cedric@mowgli.dev>

#include "mowgli_coverage/coverage_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

// Fields2Cover v3 umbrella header (f2c::types::*, f2c::hg::ConstHL, ...).
#include "fields2cover.h"
#include "mowgli_coverage/coverage_planning.hpp"

namespace mowgli_coverage
{

CoverageServer::CoverageServer(const rclcpp::NodeOptions& options)
    : nav2_util::LifecycleNode("coverage_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());
}

nav2_util::CallbackReturn CoverageServer::on_configure(
    const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());

  robot_width_ = declare_parameter<double>("robot_width", 0.40);
  operation_width_ = declare_parameter<double>("operation_width", 0.18);
  default_headland_width_ =
      declare_parameter<double>("default_headland_width", 0.20);
  num_headland_passes_ = declare_parameter<int>("num_headland_passes", 0);
  // Declared here, READ LIVE in planCoverage — both are field-tuned between
  // plans with `ros2 param set` (no node restart).
  declare_parameter<double>("chassis_safety_inset", 0.0);
  declare_parameter<double>("min_swath_length", 0.15);

  // Action server result timeout. Keep this >= the BT client's per-plan wait
  // (PlanCoverageArea, 12 s): if the server expires the result first the
  // client's goal handle is invalidated underneath it. 15 s clears the client.
  double action_server_result_timeout =
      declare_parameter<double>("action_server_result_timeout", 15.0);
  rcl_action_server_options_t server_options =
      rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds =
      RCL_S_TO_NS(action_server_result_timeout);

  action_server_ = std::make_unique<ActionServer>(shared_from_this(),
                                                  "plan_coverage",
                                                  std::bind(&CoverageServer::planCoverage, this),
                                                  nullptr,
                                                  std::chrono::milliseconds(500),
                                                  true,
                                                  server_options);

  RCLCPP_INFO(get_logger(),
              "F2C v3 boustrophedon backend ready. robot_width=%.2fm "
              "op_width=%.2fm headland=%.2fm passes=%d",
              robot_width_,
              operation_width_,
              default_headland_width_,
              num_headland_passes_);
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CoverageServer::on_activate(
    const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating %s", get_name());
  action_server_->activate();
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CoverageServer::on_deactivate(
    const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating %s", get_name());
  action_server_->deactivate();
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CoverageServer::on_cleanup(
    const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up %s", get_name());
  action_server_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn CoverageServer::on_shutdown(
    const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down %s", get_name());
  return nav2_util::CallbackReturn::SUCCESS;
}

namespace
{

constexpr double kSwathStep = 0.10;  // m between poses on a straight swath

// Build the F2C cell from the goal's outer boundary + obstacle holes.
// F2C wants closed rings (first == last); the BT passes open rings.
f2c::types::Cell buildCellFromGoal(const mowgli_interfaces::action::PlanCoverage::Goal& goal)
{
  if (goal.outer_boundary.points.size() < 3)
  {
    throw std::invalid_argument("outer_boundary needs >= 3 points");
  }
  auto make_ring = [](const geometry_msgs::msg::Polygon& poly)
  {
    f2c::types::LinearRing ring;
    for (const auto& p : poly.points)
    {
      ring.addPoint(f2c::types::Point(p.x, p.y));
    }
    // dedupClosedRing drops consecutive-duplicate vertices (a doubled leading
    // vertex is the common case from OpenMower exports / GUI polygons) and
    // closes the ring. A zero-length edge makes the ring non-simple and
    // boost::geometry (under F2C) rejects it, silently dropping the area.
    return dedupClosedRing(ring);
  };

  f2c::types::Cell cell(make_ring(goal.outer_boundary));
  for (const auto& hole : goal.obstacles)
  {
    if (hole.points.size() >= 3)
    {
      cell.addRing(make_ring(hole));
    }
  }
  return cell;
}

geometry_msgs::msg::PoseStamped makePose(const std_msgs::msg::Header& header,
                                         double x,
                                         double y,
                                         double yaw)
{
  geometry_msgs::msg::PoseStamped ps;
  ps.header = header;
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  const double half = yaw * 0.5;
  ps.pose.orientation.z = std::sin(half);
  ps.pose.orientation.w = std::cos(half);
  return ps;
}

// Douglas-Peucker simplification of an open polyline (indices [lo, hi] of pts),
// marking which vertices to KEEP. Removes points that lie within `tol` of the
// chord — i.e. densification points (deviation 0) and digitisation jitter — so
// only genuine corners survive.
void douglasPeucker(const std::vector<std::pair<double, double>>& pts,
                    std::size_t lo,
                    std::size_t hi,
                    double tol,
                    std::vector<bool>& keep)
{
  if (hi <= lo + 1)
  {
    return;
  }
  const double ax = pts[lo].first, ay = pts[lo].second;
  const double bx = pts[hi].first, by = pts[hi].second;
  const double dx = bx - ax, dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  double worst = -1.0;
  std::size_t worst_i = lo;
  for (std::size_t i = lo + 1; i < hi; ++i)
  {
    const double px = pts[i].first, py = pts[i].second;
    double dist;
    if (len2 < 1e-12)
    {
      dist = std::hypot(px - ax, py - ay);
    }
    else
    {
      double t = ((px - ax) * dx + (py - ay) * dy) / len2;
      t = std::max(0.0, std::min(1.0, t));
      dist = std::hypot(px - (ax + t * dx), py - (ay + t * dy));
    }
    if (dist > worst)
    {
      worst = dist;
      worst_i = i;
    }
  }
  if (worst > tol)
  {
    keep[worst_i] = true;
    douglasPeucker(pts, lo, worst_i, tol, keep);
    douglasPeucker(pts, worst_i, hi, tol, keep);
  }
}

// One headland ring loop → a SEQUENCE of open drivable arcs, ONE per straight
// edge of the simplified perimeter, with a RotationShim pivot at every corner.
//
// Why not feed the ring as one path: a closed loop's goal pose == its start
// pose, so MPPI's GoalCritic thinks it's already arrived and barely drives it
// (field 2026-06-12: cmd_vel≈0, "Failed to make progress"). And this chassis
// CANNOT steer through a bend at low speed — the deadband kills fine angular
// corrections (that's why we pivot in place) — so any arc carrying a real bend
// gets driven STRAIGHT and overshoots the turn ("goes too far without turning",
// field 2026-06-12). So we Douglas-Peucker the densified ring down to its true
// corners (tol kDpTol), then emit one STRAIGHT arc per corner-to-corner edge
// (re-densified to kSwathStep). MPPI only ever tracks a straight line; every
// turn is a clean pivot. The loop is rotated to start at the point nearest the
// previous segment's end so the hand-over hop is minimal.
std::vector<nav_msgs::msg::Path> ringToArcs(const std::vector<std::pair<double, double>>& loop,
                                            const std_msgs::msg::Header& header,
                                            bool have_near,
                                            double near_x,
                                            double near_y)
{
  std::vector<nav_msgs::msg::Path> arcs;
  if (loop.size() < 3)
  {
    return arcs;
  }
  // Open ring (drop the duplicated closing vertex), rotated to start nearest
  // the previous segment's end.
  const std::size_t n = loop.size() - 1;
  std::size_t start = 0;
  if (have_near)
  {
    double best = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < n; ++i)
    {
      const double d = std::hypot(loop[i].first - near_x, loop[i].second - near_y);
      if (d < best)
      {
        best = d;
        start = i;
      }
    }
  }
  // Ordered closed point list: start, start+1, …, back to start.
  std::vector<std::pair<double, double>> pts;
  pts.reserve(n + 1);
  for (std::size_t k = 0; k <= n; ++k)
  {
    pts.push_back(loop[(start + k) % n]);
  }

  // Simplify to corners. Endpoints (the shared start/end vertex) always kept.
  constexpr double kDpTol = 0.08;  // m — collapses densification + jitter
  constexpr double kSwathStep = 0.10;
  std::vector<bool> keep(pts.size(), false);
  keep.front() = keep.back() = true;
  douglasPeucker(pts, 0, pts.size() - 1, kDpTol, keep);
  std::vector<std::pair<double, double>> corners;
  for (std::size_t i = 0; i < pts.size(); ++i)
  {
    if (keep[i])
    {
      corners.push_back(pts[i]);
    }
  }

  // One straight arc per corner-to-corner edge, densified to kSwathStep with a
  // constant tangent heading (a pure straight line MPPI tracks cleanly).
  for (std::size_t c = 0; c + 1 < corners.size(); ++c)
  {
    const double x0 = corners[c].first, y0 = corners[c].second;
    const double x1 = corners[c + 1].first, y1 = corners[c + 1].second;
    const double dx = x1 - x0, dy = y1 - y0;
    const double len = std::hypot(dx, dy);
    if (len < 1e-3)
    {
      continue;
    }
    const double yaw = std::atan2(dy, dx);
    const int steps = std::max(1, static_cast<int>(std::ceil(len / kSwathStep)));
    nav_msgs::msg::Path p;
    p.header = header;
    for (int k = 0; k <= steps; ++k)
    {
      const double t = static_cast<double>(k) / static_cast<double>(steps);
      p.poses.push_back(makePose(header, x0 + t * dx, y0 + t * dy, yaw));
    }
    arcs.push_back(std::move(p));
  }
  return arcs;
}

// One straight swath → drivable Path densified at kSwathStep, constant yaw.
nav_msgs::msg::Path swathToPath(
    double x0, double y0, double x1, double y1, const std_msgs::msg::Header& header)
{
  nav_msgs::msg::Path path;
  path.header = header;
  const double dx = x1 - x0, dy = y1 - y0;
  const double len = std::hypot(dx, dy);
  const double yaw = std::atan2(dy, dx);
  const int n = std::max(1, static_cast<int>(std::ceil(len / kSwathStep)));
  path.poses.reserve(n + 1);
  for (int k = 0; k <= n; ++k)
  {
    const double t = static_cast<double>(k) / static_cast<double>(n);
    path.poses.push_back(makePose(header, x0 + t * dx, y0 + t * dy, yaw));
  }
  return path;
}

double pathLength(const nav_msgs::msg::Path& path)
{
  double len = 0.0;
  for (std::size_t i = 1; i < path.poses.size(); ++i)
  {
    len += std::hypot(path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
                      path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
  }
  return len;
}

}  // namespace

void CoverageServer::planCoverage()
{
  const auto start_time = now();
  auto goal = action_server_->get_current_goal();
  auto result = std::make_shared<PlanCoverage::Result>();

  if (!action_server_ || !action_server_->is_server_active())
  {
    RCLCPP_DEBUG(get_logger(), "Action server inactive");
    return;
  }
  if (action_server_->is_cancel_requested())
  {
    RCLCPP_INFO(get_logger(), "Goal canceled");
    action_server_->terminate_all();
    return;
  }

  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = "map";

  try
  {
    // Geometry knobs read LIVE so they're `ros2 param set`-tunable between
    // plans (field iteration without a node restart).
    const double chassis_safety_inset = get_parameter("chassis_safety_inset").as_double();
    const double min_swath_length = get_parameter("min_swath_length").as_double();
    const double mow_angle_rad =
        (goal->mow_angle_deg < 0.0) ? -1.0 : goal->mow_angle_deg * M_PI / 180.0;

    f2c::types::Cell cell = buildCellFromGoal(*goal);

    BoustrophedonPlan plan = planBoustrophedon(cell,
                                               operation_width_,
                                               default_headland_width_,
                                               num_headland_passes_,
                                               chassis_safety_inset,
                                               mow_angle_rad,
                                               min_swath_length);

    if (plan.rings.empty() && plan.swaths.empty())
    {
      result->success = false;
      result->message = "field too small after insets (chassis_safety_inset=" +
                        std::to_string(chassis_safety_inset) +
                        "m, headland=" + std::to_string(default_headland_width_) + "m)";
      RCLCPP_WARN(get_logger(),
                  "PlanCoverage: %s (field area=%.2fm²)",
                  result->message.c_str(),
                  cell.area());
      action_server_->terminate_current(result);
      return;
    }

    // Assemble segments: rings outermost-first (each split into corner arcs),
    // then serpentine swaths. Each ring is rotated to start near the previous
    // segment's end so the inter-segment hop stays minimal.
    bool have_prev_end = false;
    double prev_x = 0.0, prev_y = 0.0;
    for (const auto& loop : plan.rings)
    {
      for (auto& arc : ringToArcs(loop, header, have_prev_end, prev_x, prev_y))
      {
        if (arc.poses.size() < 2)
        {
          continue;
        }
        prev_x = arc.poses.back().pose.position.x;
        prev_y = arc.poses.back().pose.position.y;
        have_prev_end = true;
        result->segments.push_back(std::move(arc));
        result->segment_types.push_back(PlanCoverage::Result::SEGMENT_RING);
      }
    }
    for (const auto& sw : plan.swaths)
    {
      nav_msgs::msg::Path p =
          swathToPath(sw.first.first, sw.first.second, sw.second.first, sw.second.second, header);
      if (p.poses.size() < 2)
      {
        continue;
      }
      result->segments.push_back(std::move(p));
      result->segment_types.push_back(PlanCoverage::Result::SEGMENT_SWATH);
    }

    // full_path = ONE CONTINUOUS route. buildContinuousPath connects the rings +
    // swaths with forward turn-around arcs (looping into the already-mowed
    // headland side), producing a single CUSP-FREE, in-bounds polyline. MPPI
    // follows this without the bimodal "dither/spin" it does at sharp ~180°
    // reversals, and keeps its dynamic obstacle avoidance; the backported
    // arc-length MPPI fix tracks the arcs cleanly. Re-mowing on the turn loops
    // is accepted. Cusp-free + in-bounds are GUARANTEED by test_coverage_planning
    // (CoverageContinuousPath) on the real recorded area. The discrete
    // result->segments above are kept for the GUI / resume bookkeeping; the BT
    // drives full_path.
    std::vector<std::pair<double, double>> outer;
    outer.reserve(goal->outer_boundary.points.size());
    for (const auto& p : goal->outer_boundary.points)
    {
      outer.emplace_back(p.x, p.y);
    }
    constexpr double kConnectorTurnRadius = 0.30;  // forward turn-around arc radius (m)
    constexpr double kConnectorStep = 0.03;  // connector densify step (m)
    const auto continuous = buildContinuousPath(plan, outer, kConnectorTurnRadius, kConnectorStep);

    result->full_path.header = header;
    double total = 0.0;
    std::size_t out_of_bounds = 0;
    for (std::size_t i = 0; i < continuous.size(); ++i)
    {
      const double x = continuous[i].first;
      const double y = continuous[i].second;
      double yaw = 0.0;
      if (i + 1 < continuous.size())
      {
        yaw = std::atan2(continuous[i + 1].second - y, continuous[i + 1].first - x);
      }
      else if (i > 0)
      {
        yaw = std::atan2(y - continuous[i - 1].second, x - continuous[i - 1].first);
      }
      if (i > 0)
      {
        total += std::hypot(x - continuous[i - 1].first, y - continuous[i - 1].second);
      }
      result->full_path.poses.push_back(makePose(header, x, y, yaw));
      if (outer.size() >= 3 && !pointInRing(x, y, outer))
      {
        ++out_of_bounds;
      }
    }
    if (out_of_bounds > 0)
    {
      // The continuous path is built from in-bounds insets + clipped connectors,
      // so any out-of-bounds pose is a connector-geometry bug (the test guards it).
      RCLCPP_ERROR(get_logger(),
                   "PlanCoverage: %zu/%zu continuous-path poses outside the boundary — "
                   "connector geometry bug",
                   out_of_bounds,
                   result->full_path.poses.size());
    }

    result->success = true;
    result->ring_count = static_cast<uint32_t>(std::count(result->segment_types.begin(),
                                                          result->segment_types.end(),
                                                          PlanCoverage::Result::SEGMENT_RING));
    result->swath_count = static_cast<uint32_t>(std::count(result->segment_types.begin(),
                                                           result->segment_types.end(),
                                                           PlanCoverage::Result::SEGMENT_SWATH));
    result->total_distance = total;
    result->planning_time_s = (now() - start_time).seconds();

    RCLCPP_INFO(get_logger(),
                "Coverage planned: %u ring(s) + %u swath(s) = %zu segments, "
                "%.1fm total, angle=%.1f°, field=%.2fm² (%.0fms)",
                result->ring_count,
                result->swath_count,
                result->segments.size(),
                total,
                plan.swath_angle_rad * 180.0 / M_PI,
                cell.area(),
                1e3 * result->planning_time_s);

    action_server_->succeeded_current(result);
  }
  catch (const std::invalid_argument& e)
  {
    RCLCPP_ERROR(get_logger(), "Invalid coverage goal: %s", e.what());
    result->success = false;
    result->message = e.what();
    action_server_->terminate_current(result);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Internal Fields2Cover error: %s", e.what());
    result->success = false;
    result->message = e.what();
    action_server_->terminate_current(result);
  }
}

}  // namespace mowgli_coverage

RCLCPP_COMPONENTS_REGISTER_NODE(mowgli_coverage::CoverageServer)
