// Copyright (C) 2026 Cedric <cedric@mowgli.dev>

#include "mowgli_coverage/coverage_server.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

// Fields2Cover 2.0 — pull in the umbrella header so we get
// f2c::types::*, f2c::hg::ConstHL, f2c::sg::BruteForce,
// f2c::rp::BoustrophedonOrder, f2c::pp::PathPlanning,
// f2c::pp::DubinsCurvesCC.
#include "fields2cover.h"

namespace mowgli_coverage
{

using std::placeholders::_1;

CoverageServer::CoverageServer(const rclcpp::NodeOptions& options)
    : nav2_util::LifecycleNode("coverage_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating %s", get_name());
}

nav2_util::CallbackReturn CoverageServer::on_configure(
    const rclcpp_lifecycle::State& /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring %s", get_name());

  // Snapshot static parameters. Keys mirror the upstream
  // opennav_coverage server so nav2_params.yaml can stay unchanged
  // during the migration.
  robot_width_ = declare_parameter<double>("robot_width", 0.40);
  operation_width_ = declare_parameter<double>("operation_width", 0.20);
  default_headland_width_ =
      declare_parameter<double>("default_headland_width", 0.20);
  min_turning_radius_ = declare_parameter<double>("min_turning_radius", 0.05);
  max_diff_curvature_ = declare_parameter<double>("linear_curv_change", 200.0);
  chassis_safety_inset_ =
      declare_parameter<double>("chassis_safety_inset", 0.0);
  num_headland_passes_ =
      declare_parameter<int>("num_headland_passes", 0);
  min_subcell_area_m2_ = declare_parameter<double>("min_subcell_area_m2", 0.25);

  // The legacy server exposed a handful of mode strings (DUBIN /
  // DUBIN_CC / REEDS_SHEPP, BOUSTROPHEDON / SNAKE, BRUTE_FORCE,
  // CONSTANT, etc.). For Mowgli we always use the same combination,
  // so we read but don't act on these — log them once so an operator
  // knows why their YAML override is being ignored.
  declare_parameter<std::string>("default_swath_angle_type", "BRUTE_FORCE");
  declare_parameter<std::string>("default_swath_type", "LENGTH");
  declare_parameter<std::string>("default_route_type", "BOUSTROPHEDON");
  declare_parameter<std::string>("default_path_type", "DUBIN");
  declare_parameter<std::string>("default_path_continuity_type",
                                 "DISCONTINUOUS");
  declare_parameter<bool>("coordinates_in_cartesian_frame", true);

  // Action server result timeout. Keep this >= the BT client's per-piece wait
  // (PlanCoverageArea, 12 s): if the server expires the result first the
  // client's goal handle is invalidated underneath it. 15 s clears the client.
  double action_server_result_timeout = 15.0;
  declare_parameter<double>("action_server_result_timeout", 15.0);
  get_parameter("action_server_result_timeout", action_server_result_timeout);
  rcl_action_server_options_t server_options =
      rcl_action_server_get_default_options();
  server_options.result_timeout.nanoseconds =
      RCL_S_TO_NS(action_server_result_timeout);

  action_server_ = std::make_unique<ActionServer>(
      shared_from_this(),
      "compute_coverage_path",
      std::bind(&CoverageServer::computeCoveragePath, this),
      nullptr,
      std::chrono::milliseconds(500),
      true,
      server_options);

  RCLCPP_INFO(get_logger(),
              "F2C v2.0 backend ready. robot_width=%.2fm op_width=%.2fm "
              "headland=%.2fm min_R=%.2fm",
              robot_width_, operation_width_, default_headland_width_,
              min_turning_radius_);
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

// Build an F2CCell from the goal's polygon list. The first
// Coordinates entry is the outer ring; any subsequent entries are
// interior holes (obstacles). All coordinates are interpreted in the
// cartesian map frame — Mowgli does not use the GPS-coordinate path.
//
// NOTE: F2C wants linear rings closed (first == last). The BT side
// generally passes open rings, so we close them here.
f2c::types::Cell buildCellFromGoal(
    const opennav_coverage_msgs::action::ComputeCoveragePath::Goal& goal)
{
  if (goal.polygons.empty())
  {
    throw std::invalid_argument("Goal has no polygons");
  }

  auto make_ring = [](const auto& coords) {
    f2c::types::LinearRing ring;
    if (coords.coordinates.empty())
    {
      throw std::invalid_argument("Polygon has no coordinates");
    }
    for (const auto& c : coords.coordinates)
    {
      ring.addPoint(f2c::types::Point(c.axis1, c.axis2));
    }
    // Close the ring if the caller didn't.
    const auto& first = coords.coordinates.front();
    const auto& last = coords.coordinates.back();
    if (first.axis1 != last.axis1 || first.axis2 != last.axis2)
    {
      ring.addPoint(f2c::types::Point(first.axis1, first.axis2));
    }
    return ring;
  };

  f2c::types::Cell cell(make_ring(goal.polygons[0]));
  for (std::size_t i = 1; i < goal.polygons.size(); ++i)
  {
    cell.addRing(make_ring(goal.polygons[i]));
  }
  return cell;
}

nav_msgs::msg::Path toNavPath(const f2c::types::Path& path,
                              const std_msgs::msg::Header& header)
{
  nav_msgs::msg::Path msg;
  msg.header = header;
  for (const auto& state : path.getStates())
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = header;
    ps.pose.position.x = state.point.getX();
    ps.pose.position.y = state.point.getY();
    ps.pose.position.z = 0.0;
    const double half = state.angle * 0.5;
    ps.pose.orientation.z = std::sin(half);
    ps.pose.orientation.w = std::cos(half);
    msg.poses.push_back(ps);
  }
  return msg;
}

}  // namespace

void CoverageServer::computeCoveragePath()
{
  const auto start_time = now();
  auto goal = action_server_->get_current_goal();
  auto result = std::make_shared<ComputeCoveragePath::Result>();

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

  // F2C planning is single-threaded and can take 1-2 s on a non-trivial
  // field — between phases, poll for a cancel from the BT (raised by
  // PlanCoverageArea::onHalted via async_cancel_goal). Without these
  // checks the server would finish planning, succeeded_current() against
  // a goal the BT already cancelled, and the BT would sit in
  // WaitingForResult until its 30 s timeout — a 30 s freeze on every
  // emergency / area switch on production hardware.
  auto check_cancel = [this, &result]() -> bool {
    if (!action_server_->is_cancel_requested())
    {
      return false;
    }
    RCLCPP_INFO(get_logger(),
                "Goal canceled mid-planning — terminating");
    result->error_code = ComputeCoveragePath::Result::NONE;
    action_server_->terminate_all(result);
    return true;
  };

  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = goal->frame_id.empty() ? std::string{"map"} : goal->frame_id;

  try
  {
    // (0) Robot setup. F2C v2 uses setters (private fields).
    f2c::types::Robot robot(robot_width_, operation_width_);
    robot.setMinTurningRadius(min_turning_radius_);
    robot.setMaxDiffCurv(max_diff_curvature_);

    // (1) Cell with outer + obstacle rings from the goal.
    f2c::types::Cell cell = buildCellFromGoal(*goal);

    // F2C wraps cells in F2CCells (a multi-cell collection). For
    // single-cell coverage we wrap our one cell into a one-element
    // collection.
    f2c::types::Cells cells;
    cells.addGeometry(cell);

    // (2) Headland. Override the default if the caller specified one
    // via headland_mode (string-coded as the width on the legacy
    // server). The BT passes the inset directly through this field.
    double headland_width = default_headland_width_;
    if (!goal->headland_mode.width || goal->headland_mode.width <= 0.0)
    {
      // Field unset / nonsense — keep our default.
    }
    else
    {
      headland_width = goal->headland_mode.width;
    }
    f2c::hg::ConstHL hl;

    // (1.5) Safety pre-inset. The polygon F2C plans into is the
    // operator's mowing area shrunk by chassis_safety_inset_ (default
    // 0.20 m ≈ chassis_width/2). Without this, F2C's outermost
    // headland ring sits at op_width/2 = 9 cm from the polygon edge,
    // and FTC tracking error during corner traversals (~ 15 cm) pushes
    // the chassis 6 cm OUTSIDE the polygon — visible as the boundary
    // excursion at session 2026-05-12-post-bug-sweep, t=1778592275:
    // robot at (1.05, -3.00) → (1.38, -3.07) → (1.47, -3.11),
    // BoundaryGuard fires, mowing aborts at 22 % coverage.
    //
    // Pre-insetting trades 11 % of usable area (on a 9 × 6 m field;
    // <2 % on real-world 100 m² lawns) for a hard guarantee that the
    // chassis cannot leave the authorised polygon even with worst-
    // case tracking error. The headland traversal AND the inner-field
    // swaths both ride inside this safety buffer.
    f2c::types::Cells safe_cells = cells;
    if (chassis_safety_inset_ > 0.001)
    {
      safe_cells = hl.generateHeadlands(cells, chassis_safety_inset_);
      if (safe_cells.size() == 0 ||
          safe_cells.getGeometry(0).area() < 1e-6)
      {
        RCLCPP_WARN(get_logger(),
                    "Chassis safety inset (%.2f m) consumed the entire "
                    "field — refusing to plan.",
                    chassis_safety_inset_);
        result->error_code =
            ComputeCoveragePath::Result::INTERNAL_F2C_ERROR;
        action_server_->terminate_current(result);
        return;
      }
    }

    f2c::types::Cells inner = hl.generateHeadlands(safe_cells, headland_width);

    if (inner.size() == 0 || inner.getGeometry(0).area() < 1e-6)
    {
      RCLCPP_WARN(get_logger(),
                  "Headland inset (%.2f m on top of %.2f m chassis safety) "
                  "consumed the entire field — nothing left to swath.",
                  headland_width, chassis_safety_inset_);
      result->error_code =
          ComputeCoveragePath::Result::INTERNAL_F2C_ERROR;
      action_server_->terminate_current(result);
      return;
    }

    // (2b) Headland coverage — F2C v2 ships generateHeadlandSwaths()
    // which returns N concentric rings at op_width spacing; we emit
    // them as a perimeter traversal that prefixes the inner-field
    // swaths. Without this the polygon's outer `headland_width` strip
    // never gets mowed (ConstHL.generateHeadlands only INSETS the
    // planning field, doesn't generate rings to traverse it).
    //
    // n_passes: operator override (num_headland_passes_, set via
    // mowgli_robot.yaml → coverage_server param) when > 0, otherwise
    // auto-derive as ceil(headland_width / op_width). One pass minimum.
    // dir_out2in=true: the outer ring is the first one in the vector,
    // so the robot follows the polygon perimeter first then spirals
    // inward toward the F2C swath start.
    f2c::types::Path headland_path;
    const int n_headland_passes =
        (num_headland_passes_ > 0)
            ? num_headland_passes_
            : std::max(1,
                       static_cast<int>(std::ceil(
                           headland_width / robot.getCovWidth())));
    // Use safe_cells (pre-inset polygon) so the outermost headland
    // ring sits at chassis_safety_inset_ + op_width/2 from the real
    // polygon edge — keeps the chassis inside under FTC tracking
    // error.
    auto headland_passes = hl.generateHeadlandSwaths(
        safe_cells, robot.getCovWidth(), n_headland_passes,
        /*dir_out2in=*/true);
    // F2C returns the ring as a sparse vertex list (typically just the
    // polygon corners). Densify between consecutive vertices so FTC's
    // carrot has poses to chase along each edge — without this, an
    // 8-pose ring leaves the controller jumping between corners and
    // PolygonStop fires on the discontinuity.
    constexpr double kHeadlandStep = 0.10;  // m between path poses
    std::size_t headland_pose_count = 0;
    for (const auto& pass_cells : headland_passes)
    {
      for (std::size_t c = 0; c < pass_cells.size(); ++c)
      {
        const auto pass_cell = pass_cells.getGeometry(c);
        const auto ring = pass_cell.getExteriorRing();
        const std::size_t n = ring.size();
        if (n < 3)
        {
          continue;
        }
        // Skip the trailing duplicate vertex (F2C closes the ring by
        // repeating index 0 at index n-1).
        const std::size_t last = (ring.getGeometry(0) ==
                                   ring.getGeometry(n - 1)) ? n - 1 : n;
        for (std::size_t i = 0; i < last; ++i)
        {
          const auto a = ring.getGeometry(i);
          const auto b = ring.getGeometry((i + 1) % n);
          const double dx = b.getX() - a.getX();
          const double dy = b.getY() - a.getY();
          const double seg_len = std::hypot(dx, dy);
          const double yaw = std::atan2(dy, dx);
          const int n_steps = std::max(
              1, static_cast<int>(std::ceil(seg_len / kHeadlandStep)));
          // Emit n_steps poses for t = k/n_steps, k in [0, n_steps).
          // Endpoint b is NOT emitted here — the next segment's start
          // == this segment's end, so b gets emitted as the next
          // segment's k=0 pose. Avoids double-emit at vertices.
          for (int k = 0; k < n_steps; ++k)
          {
            const double t = static_cast<double>(k) /
                             static_cast<double>(n_steps);
            f2c::types::PathState st;
            st.point = f2c::types::Point(a.getX() + t * dx,
                                          a.getY() + t * dy);
            st.angle = yaw;
            headland_path.addState(st);
            ++headland_pose_count;
          }
        }
        // Close the ring — emit the wrap-back vertex (== ring[0])
        // explicitly so the densified polyline reaches the starting
        // point. Without this the very last segment of a closed
        // ring drops its endpoint, leaving up to kHeadlandStep of
        // unmowed perimeter at the close.
        const auto v0 = ring.getGeometry(0);
        const auto v1 = ring.getGeometry(1 % n);
        f2c::types::PathState close_st;
        close_st.point = v0;
        close_st.angle = std::atan2(v1.getY() - v0.getY(),
                                     v1.getX() - v0.getX());
        headland_path.addState(close_st);
        ++headland_pose_count;
      }
    }
    RCLCPP_INFO(get_logger(),
                "Headland: %d pass(es) at %.2f m spacing → %zu poses",
                n_headland_passes, robot.getCovWidth(),
                headland_pose_count);
    if (check_cancel()) return;

    // (3) Decompose the inner field around interior holes. Without
    // this, BoustrophedonOrder serialises ALL swaths north-to-south
    // and PathPlanning connects non-adjacent endpoints with straight
    // lines (min_turning_radius=0.05 m, so effectively no arc) —
    // visible in Foxglove as long diagonal connectors cutting across
    // the field around the obstacle. TrapezoidalDecomp splits the
    // (now headland-trimmed) field into sub-cells so the routing
    // operates on each cleanly. Skip when there are no holes (single
    // outer ring → decompose returns the same one cell unchanged but
    // we save the cycle).
    f2c::types::Cells planning_cells;
    if (cell.size() > 1)
    {
      f2c::decomp::TrapezoidalDecomp decomp;
      // Split perpendicular to swath direction (which we don't know
      // until BruteForce runs). 0 rad as a starting point is fine —
      // BoustrophedonDecomp would pick this for us; for trapezoidal
      // the angle just controls split orientation, not coverage.
      decomp.setSplitAngle(0.0);
      planning_cells = decomp.decompose(inner);
      RCLCPP_INFO(get_logger(),
                  "TrapezoidalDecomp split the inner field into %zu sub-cell(s)",
                  planning_cells.size());
    }
    else
    {
      planning_cells = inner;
    }

    // (4) Per sub-cell pipeline: BruteForce swaths → BoustrophedonOrder
    // → PathPlanning(DubinsCurvesCC). Concatenate paths in cell order
    // with explicit Dubins connectors between sub-cells so the path is
    // ONE continuous F2CPath instead of N disjoint pieces. Without the
    // connectors, the path-end pose of cell N and the path-start pose
    // of cell N+1 are arbitrary (no shared edge), so FTC's
    // WAITING_FOR_GOAL_APPROACH state times out trying to drive the
    // last 25 cm to a goal pose that isn't reachable in a straight
    // line. planPathForConnection inserts a Dubins arc using the
    // robot's min_turning_radius; FTC follows it like any other path
    // segment.
    f2c::obj::NSwath n_swath_obj;
    f2c::sg::BruteForce bf;
    f2c::rp::BoustrophedonOrder order;
    f2c::pp::PathPlanning pp;
    f2c::pp::DubinsCurvesCC turn;
    // Path starts with the headland traversal so the perimeter strip
    // gets mowed first; the inner-field swaths are appended after.
    f2c::types::Path path = headland_path;
    std::size_t total_swaths = 0;
    // Carry the headland's last pose as the connector start so the
    // first inner-field path links smoothly via Dubins.
    bool have_prev_end = !headland_path.getStates().empty();
    f2c::types::Point prev_end_pt;
    double prev_end_ang = 0.0;
    if (have_prev_end)
    {
      const auto& last = headland_path.getStates().back();
      prev_end_pt = last.point;
      prev_end_ang = last.angle;
    }
    std::size_t skipped_tiny = 0;
    for (std::size_t i = 0; i < planning_cells.size(); ++i)
    {
      // Cancel poll between sub-cells (each iteration is the most
      // expensive single phase here — BruteForce + PathPlanning can
      // each be ~100 ms on a non-trivial sub-cell).
      if (check_cancel()) return;

      const auto sub_cell = planning_cells.getGeometry(i);
      // Drop sub-cells that can't fit a single useful swath BEFORE the
      // costly BruteForce below (generateBestSwaths is ~0.3-0.6 s on each
      // sub-cell even when it ultimately yields zero swaths). F2C v2's
      // TrapezoidalDecomp splits a hole-bearing polygon into dozens of
      // slivers along the densified ConstHL output — a resume plan on a
      // mowed-out field produced 224 sub-cells, ~170 of them 0.07-0.28 m²,
      // each burning a full BruteForce for no coverage → 94 s per piece
      // (field 2026-06-01). These slivers never produce a swath, so screen
      // them on area here. The simplify-the-mowed-hole fix in
      // remaining_polygon keeps the sub-cell COUNT sane; this floor keeps
      // the per-sub-cell COST off the staircase remnants. 0.25 m² ≈ a
      // 0.18 m swath × 1.4 m — anything smaller can't tile. Tunable via the
      // min_subcell_area_m2 param.
      if (sub_cell.area() < min_subcell_area_m2_)
      {
        ++skipped_tiny;
        continue;
      }
      f2c::types::Swaths swaths =
          bf.generateBestSwaths(n_swath_obj, robot.getCovWidth(), sub_cell);
      if (swaths.size() == 0)
      {
        RCLCPP_WARN(get_logger(),
                    "Sub-cell %zu (area %.2f m²) produced no swaths — skipping",
                    i, sub_cell.area());
        continue;
      }
      f2c::types::Swaths ordered = order.genSortedSwaths(swaths);
      f2c::types::Path sub_path = pp.planPath(robot, ordered, turn);
      if (sub_path.size() == 0)
      {
        continue;
      }
      const auto& first_state = sub_path.getStates().front();
      const auto& last_state = sub_path.getStates().back();

      if (have_prev_end)
      {
        // Bridge from previous cell's end-pose to this cell's start-pose.
        // Empty middle MultiPoint — no waypoints in between, just the
        // two endpoints. Dubins finds the shortest forward-only arc
        // respecting the robot's turning radius.
        f2c::types::MultiPoint no_waypoints;
        f2c::types::Path connector = pp.planPathForConnection(
            robot, prev_end_pt, prev_end_ang, no_waypoints,
            first_state.point, first_state.angle, turn);
        if (connector.size() > 0)
        {
          path += connector;
        }
      }
      path += sub_path;
      prev_end_pt = last_state.point;
      prev_end_ang = last_state.angle;
      have_prev_end = true;
      total_swaths += ordered.size();
    }

    if (path.size() == 0)
    {
      RCLCPP_WARN(get_logger(),
                  "F2C produced an empty coverage path (cov_width=%.2f m, "
                  "headland=%.2f m, field area=%.2f m², %zu sub-cells)",
                  robot.getCovWidth(), headland_width, cell.area(),
                  planning_cells.size());
      result->error_code =
          ComputeCoveragePath::Result::INTERNAL_F2C_ERROR;
      action_server_->terminate_current(result);
      return;
    }

    // (6) Convert to nav_msgs/Path. We don't populate the older
    // PathComponents structure — Mowgli's BT only consumes nav_path.
    result->nav_path = toNavPath(path, header);
    result->planning_time = now() - start_time;
    result->error_code = ComputeCoveragePath::Result::NONE;

    RCLCPP_INFO(
        get_logger(),
        "Coverage path planned: %zu poses, %zu swaths across %zu sub-cell(s) "
        "(skipped %zu < %.3f m² as too small), headland=%.2fm, "
        "chassis_safety=%.2fm, field=%.2fm² (planning %.0fms)",
        result->nav_path.poses.size(), total_swaths, planning_cells.size(),
        skipped_tiny, min_subcell_area_m2_, headland_width,
        chassis_safety_inset_, cell.area(),
        1e3 * (now() - start_time).seconds());

    action_server_->succeeded_current(result);
  }
  catch (const std::invalid_argument& e)
  {
    RCLCPP_ERROR(get_logger(), "Invalid coverage goal: %s", e.what());
    result->error_code = ComputeCoveragePath::Result::INVALID_COORDS;
    action_server_->terminate_current(result);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Internal Fields2Cover error: %s", e.what());
    result->error_code = ComputeCoveragePath::Result::INTERNAL_F2C_ERROR;
    action_server_->terminate_current(result);
  }
}

}  // namespace mowgli_coverage

RCLCPP_COMPONENTS_REGISTER_NODE(mowgli_coverage::CoverageServer)
