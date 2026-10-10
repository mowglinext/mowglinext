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
#include <mutex>
#include <string>

#include "mowgli_interfaces/action/plan_coverage.hpp"
#include "mowgli_interfaces/srv/correct_recorded_obstacle.hpp"
#include "mowgli_interfaces/srv/preview_coverage.hpp"
#include "mowgli_interfaces/srv/preview_obstacle_clearance.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/simple_action_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_coverage
{

class CoverageServer : public nav2::LifecycleNode
{
public:
  using PlanCoverage = mowgli_interfaces::action::PlanCoverage;
  using ActionServer = nav2::SimpleActionServer<PlanCoverage>;

  explicit CoverageServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
  ~CoverageServer() override = default;

protected:
  nav2::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  nav2::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  nav2::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  nav2::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  nav2::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
  // The geometry knobs a plan reads LIVE (so they are `ros2 param set`-tunable
  // between plans). One reader shared by planCoverage and previewCoverage: the
  // preview must plan with exactly what a real plan would use, and two copies of
  // these reads would drift.
  struct LivePlanParams
  {
    double effective_inset;  // chassis_safety_inset, clamped at 0
    double min_swath_length;
    double obstacle_margin;  // clamped to [0, 1]
    double min_turning_radius;
    int connector_max_headland_passes;
    int ring_direction;  // 0 planner default, 1 CW, 2 CCW
    double footprint_width;  // 2 x footprint_half_width, 0 = keep narrow tongues
  };
  LivePlanParams readLivePlanParams();

  // Action callback. Pulls the active goal, runs planBoustrophedon, converts
  // the plan to per-segment nav_msgs/Paths, and succeeds/terminates the goal.
  void planCoverage();

  // Dry run of the planner for the GUI map editor's line preview: the same
  // planBoustrophedon call as planCoverage, with the swath angle and perimeter
  // winding taken from the request, answered as rings + swaths in drive order.
  // Read-only and stateless — nothing is queued and nothing moves. See
  // previewObstacleClearance below for why request_header exists.
  void previewCoverage(
      const std::shared_ptr<rmw_request_id_s> request_header,
      const std::shared_ptr<mowgli_interfaces::srv::PreviewCoverage::Request> request,
      std::shared_ptr<mowgli_interfaces::srv::PreviewCoverage::Response> response);

  // Read-only utility service: buffers each input obstacle polygon outward by
  // the live obstacle_margin, exactly as buildCellFromGoal does for a real
  // plan (bufferRingOutward). Backs the GUI's "obstacle clearance preview"
  // map overlay — never consumed by planning itself.
  //
  // nav2::LifecycleNode::create_service demands the 3-argument service
  // callback signature (request_header, request, response) — the 2-argument
  // form a plain rclcpp::Node::create_service accepts does not convert to
  // its CallbackType (CI-caught, mowglinext#802 PR build failure). The
  // header is unused here; every request is served the same way regardless
  // of who's asking.
  void previewObstacleClearance(
      const std::shared_ptr<rmw_request_id_s> request_header,
      const std::shared_ptr<mowgli_interfaces::srv::PreviewObstacleClearance::Request> request,
      std::shared_ptr<mowgli_interfaces::srv::PreviewObstacleClearance::Response> response);

  // Shrinks a just-recorded obstacle polygon inward by the raw chassis
  // half-width (erodeRingInward) so RecordArea (mowgli_behavior) can save
  // what the operator actually traced, not base_footprint's own trajectory
  // offset by however close they drove to the object. Physical correction
  // only — no obstacle_margin or footprint safety margin involved. See
  // previewObstacleClearance's doc comment for why request_header exists.
  void correctRecordedObstacle(
      const std::shared_ptr<rmw_request_id_s> request_header,
      const std::shared_ptr<mowgli_interfaces::srv::CorrectRecordedObstacle::Request> request,
      std::shared_ptr<mowgli_interfaces::srv::CorrectRecordedObstacle::Response> response);

  std::unique_ptr<ActionServer> action_server_;
  // nav2::LifecycleNode::create_service returns a nav2::ServiceServer<T>
  // handle, NOT a plain rclcpp::Service<T> (CI-caught, mowglinext#802 PR
  // build failure, second error uncovered once the callback signature
  // itself was fixed).
  nav2::ServiceServer<mowgli_interfaces::srv::PreviewObstacleClearance>::SharedPtr
      preview_obstacle_clearance_service_;
  nav2::ServiceServer<mowgli_interfaces::srv::CorrectRecordedObstacle>::SharedPtr
      correct_recorded_obstacle_service_;
  nav2::ServiceServer<mowgli_interfaces::srv::PreviewCoverage>::SharedPtr preview_coverage_service_;

  // Serialises planner runs (planCoverage vs previewCoverage) — see planCoverage.
  std::mutex plan_mutex_;

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
