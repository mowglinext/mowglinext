// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// CoverageServer — Mowgli's direct-to-Fields2Cover-v2 coverage planner.
//
// Listens on the same `compute_coverage_path` action as the upstream
// opennav_coverage server (we share the same opennav_coverage_msgs
// types) so the BT's PlanCoverageArea client doesn't change. Internally
// builds an F2C v2 pipeline:
//
//   F2CCell  (outer + interior holes from the goal polygons)
//      ↓ ConstHL::generateHeadlands(cell, headland_width)
//   F2CCells (inner field, headland already removed)
//      ↓ BruteForce::generateSwaths(angle, cov_width, inner)
//   F2CSwaths
//      ↓ BoustrophedonOrder::genSortedSwaths()
//   F2CSwaths (ordered)
//      ↓ PathPlanning::planPath(robot, swaths, DubinsCurvesCC)
//   F2CPath
//      → nav_msgs/Path  (action result.nav_path)
//
// Lifecycle is a 1:1 mirror of the upstream node (configure / activate
// / deactivate / cleanup / shutdown) so lifecycle_manager_navigation
// can manage us with no config change. Bond is created on activate.

#ifndef MOWGLI_COVERAGE__COVERAGE_SERVER_HPP_
#define MOWGLI_COVERAGE__COVERAGE_SERVER_HPP_

#include <memory>
#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "opennav_coverage_msgs/action/compute_coverage_path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_coverage
{

class CoverageServer : public nav2_util::LifecycleNode
{
public:
  using ComputeCoveragePath = opennav_coverage_msgs::action::ComputeCoveragePath;
  using ActionServer = nav2_util::SimpleActionServer<ComputeCoveragePath>;

  explicit CoverageServer(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  ~CoverageServer() override = default;

protected:
  nav2_util::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& state) override;
  nav2_util::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& state) override;

private:
  // Action callback. Pulls the active goal, runs the F2C pipeline, and
  // either succeeded_current() or terminate_current() on the result.
  void computeCoveragePath();

  std::unique_ptr<ActionServer> action_server_;

  // Static parameters (snapshot at on_configure). Match the keys the
  // upstream opennav_coverage server reads so nav2_params.yaml stays
  // unchanged.
  double robot_width_{0.40};            // chassis width (m)
  double operation_width_{0.20};        // mower cut width (m)
  double default_headland_width_{0.20}; // headland inset (m)
  double min_turning_radius_{0.05};     // F2C turning radius (m)
  double max_diff_curvature_{200.0};    // 1/m^2, clamps DubinsCC integrator
  // Safety inset applied to the polygon BEFORE F2C planning. By
  // default 0 — F2C plans all the way to the operator polygon
  // edge so the robot mows the entire authorised area. FTC
  // tracking error during corner traversals can briefly push the
  // chassis past the polygon edge; that's absorbed by map_server's
  // boundary tolerance (soft_boundary_margin_m_, set to
  // chassis_width/2 in production). Only bump this on sites where
  // the operator polygon hugs a hard obstacle (fence, drop-off)
  // and the briefly-outside chassis is unsafe — then the cost is
  // a strip of unmowed grass at the edge.
  double chassis_safety_inset_{0.0};
  // Operator override for the number of concentric headland rings.
  // 0 = auto, derive from ceil(headland_width / op_width). Anything
  // > 0 forces exactly that many rings regardless of headland_width —
  // useful when an operator wants a single perimeter pass for time
  // savings on a polygon whose `headland_width` would otherwise imply
  // 2-3 rings, or conversely to add an extra ring on a sloppy edge.
  int num_headland_passes_{0};
  // Drop sub-cells from TrapezoidalDecomp whose area is smaller
  // than this threshold (m²). A sub-cell under ~0.25 m² (≈ a 0.18 m
  // swath × 1.4 m) can't tile a useful swath, and screening it here
  // skips the costly per-sub-cell BruteForce on the staircase slivers
  // a resume plan leaves behind — see the rationale in coverage_server's
  // sub-cell loop (94 s/piece regression fixed 2026-06-01).
  double min_subcell_area_m2_{0.25};
};

}  // namespace mowgli_coverage

#endif  // MOWGLI_COVERAGE__COVERAGE_SERVER_HPP_
