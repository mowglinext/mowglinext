// Copyright (C) 2026 Cedric <cedric@mowgli.dev>
//
// CoverageServer — Mowgli's direct-to-Fields2Cover-v3 coverage planner.
//
// Serves mowgli_interfaces/action/PlanCoverage on `plan_coverage`. The result
// is a list of explicit, ordered, individually-drivable segments — concentric
// headland rings (outermost first) then straight serpentine swaths — built by
// planBoustrophedon() (coverage_planning.hpp). There is NO turn planning and
// NO downstream path re-segmentation: the BT dispatches one segment per
// FollowCoveragePath goal and the controller pivots in place between them.
//
// Lifecycle mirrors the other Nav2 servers (configure / activate / deactivate
// / cleanup / shutdown) so lifecycle_manager_navigation manages it with no
// config change. Bond is created on activate.

#ifndef MOWGLI_COVERAGE__COVERAGE_SERVER_HPP_
#define MOWGLI_COVERAGE__COVERAGE_SERVER_HPP_

#include <memory>
#include <string>

#include "mowgli_interfaces/action/plan_coverage.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_coverage
{

class CoverageServer : public nav2_util::LifecycleNode
{
public:
  using PlanCoverage = mowgli_interfaces::action::PlanCoverage;
  using ActionServer = nav2_util::SimpleActionServer<PlanCoverage>;

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
  // Action callback. Pulls the active goal, runs planBoustrophedon, converts
  // the plan to per-segment nav_msgs/Paths, and succeeds/terminates the goal.
  void planCoverage();

  std::unique_ptr<ActionServer> action_server_;

  // Static parameters (snapshot at on_configure). Geometry knobs that are
  // field-tuned (chassis_safety_inset) are read LIVE in planCoverage instead.
  double robot_width_{0.40};  // physical chassis width (m) — semantic
  double operation_width_{0.18};  // swath spacing = F2C cov_width (m)
  double default_headland_width_{0.20};  // headland band width (m)
  // Operator override for the number of concentric headland rings.
  // 0 = auto: ceil(headland_width / operation_width), min 1.
  int num_headland_passes_{0};
  // Drop straight swaths shorter than this (m) — a sliver clip from a concave
  // notch isn't worth an in-place pivot. Read live (field-tunable).
  double min_swath_length_{0.15};
};

}  // namespace mowgli_coverage

#endif  // MOWGLI_COVERAGE__COVERAGE_SERVER_HPP_
