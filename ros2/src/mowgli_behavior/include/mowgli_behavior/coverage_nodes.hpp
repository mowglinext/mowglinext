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

#pragma once

#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/action/plan_coverage.hpp"
#include "mowgli_interfaces/srv/get_mowing_area.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// FollowStrip — execute the planned coverage segments, blade ON.
//
// Consumes ctx->current_strip_segments (EXPLICIT ordered segments from the
// coverage server: headland rings first, then straight serpentine swaths) and
// dispatches ONE segment per FollowCoveragePath goal. RotationShim pivots in
// place to the segment-start heading; MPPI tracks the straight swath / smooth
// ring; the PathProgressGoalChecker fires at the segment end.
//
// When the next segment's start is far from the robot (resume mid-list, a
// skipped segment, or a concave field whose serpentine hops across a notch),
// the node first runs a NavigateToPose transit to the segment start —
// boundary-aware (global costmap keepout) instead of letting MPPI cut
// cross-country.
// ---------------------------------------------------------------------------

class FollowStrip : public BT::StatefulActionNode
{
public:
  using Nav2FollowPath = nav2_msgs::action::FollowPath;
  using FollowGoalHandle = rclcpp_action::ClientGoalHandle<Nav2FollowPath>;
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;

  FollowStrip(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void setBladeEnabled(bool enabled);
  // Dispatch swaths_[swath_idx_]: if the robot is farther than
  // kSegmentTransitGap from the segment start, first run a NavigateToPose
  // transit (sets transit_active_); otherwise send the FollowPath goal
  // directly. Returns false only if a client is missing.
  bool sendCurrentSwath(const std::shared_ptr<BTContext>& ctx);
  // Send the FollowPath goal for the current segment (no gap check).
  bool sendFollowGoal(const std::shared_ptr<BTContext>& ctx);
  // Robot distance to the current segment's first pose (TF map→base_footprint);
  // returns a large value if TF is unavailable (forces the safe transit path).
  double distanceToSegmentStart(const std::shared_ptr<BTContext>& ctx) const;

  rclcpp_action::Client<Nav2FollowPath>::SharedPtr follow_client_;
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr blade_client_;
  // Mirrors the active segment onto the coverage controller's global_plan
  // topic so the PathProgressGoalChecker (coverage_goal_checker) can track
  // per-pose progress (MPPI/RotationShim does not republish the plan).
  // Latched (transient_local) so a late-subscribing goal checker still
  // receives the current segment.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr coverage_plan_pub_;
  std::shared_future<FollowGoalHandle::SharedPtr> follow_future_;
  FollowGoalHandle::SharedPtr follow_handle_;
  // Inter-segment transit (NavigateToPose) state.
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;
  bool transit_active_ = false;

  // The explicit segments being executed (copied from
  // ctx->current_strip_segments in onStart).
  std::vector<nav_msgs::msg::Path> swaths_;
  std::size_t swath_idx_ = 0;
  std::size_t swaths_skipped_ = 0;
  bool swath_goal_sent_ = false;
  // Area being mowed (from ctx->current_area) — keys the swath-completion
  // tracking in BTContext so a resume/re-plan skips already-mowed segments.
  uint32_t area_idx_ = 0;

  // Start an explicit transit when the segment start is farther than this.
  // Below it, RotationShim+MPPI close the gap themselves (adjacent swaths are
  // one op_width ≈ 0.16 m apart).
  static constexpr double kSegmentTransitGap = 0.6;

  // Blade spinup delay — wait before sending the FIRST segment goal
  static constexpr double kBladeSpinupDelaySec = 1.5;
  std::chrono::steady_clock::time_point blade_start_time_;
  bool goal_sent_ = false;
};

// ---------------------------------------------------------------------------
// TransitToStrip — navigate to strip start using Nav2 navigate_to_pose
// ---------------------------------------------------------------------------

class TransitToStrip : public BT::StatefulActionNode
{
public:
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;

  TransitToStrip(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;
};

// ---------------------------------------------------------------------------
// DetourAroundObstacle — when FollowStrip aborts on a lookahead-collision,
// drive a short side-step path through the global planner so the robot
// physically gets out from in front of the obstacle (a person standing in
// the strip). The next strip iteration replans from the new pose; the
// `mow_progress` layer prevents re-cutting already-mowed cells.
//
// The detour is a NavigateToPose at (current_pose ⊕ forward·x̂_body
// + lateral·ŷ_body), routed via SmacPlanner over the local costmap which
// has the obstacle layer enabled — so the planner naturally curves around
// the obstruction rather than charging through it.
// ---------------------------------------------------------------------------

class DetourAroundObstacle : public BT::StatefulActionNode
{
public:
  using Nav2Navigate = nav2_msgs::action::NavigateToPose;
  using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav2Navigate>;

  DetourAroundObstacle(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("forward_m", 0.8, "Forward offset from current pose, body frame"),
        BT::InputPort<double>("lateral_m", 0.6, "Lateral offset (positive = left), body frame"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_action::Client<Nav2Navigate>::SharedPtr nav_client_;
  std::shared_future<NavGoalHandle::SharedPtr> nav_future_;
  NavGoalHandle::SharedPtr nav_handle_;
};

// ---------------------------------------------------------------------------
// GetNextUnmowedArea — find next area with remaining strips
// ---------------------------------------------------------------------------

class GetNextUnmowedArea : public BT::StatefulActionNode
{
public:
  GetNextUnmowedArea(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<uint32_t>("max_areas", 20u, "Maximum number of areas to check"),
        BT::OutputPort<uint32_t>("area_index", "Index of the next unmowed area"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  /// Process a completed service response. Returns SUCCESS if an unmowed area
  /// was found, FAILURE if all areas are done / no areas defined, or RUNNING
  /// if more areas need to be checked (launches next async call internally).
  BT::NodeStatus processResponse();

  /// Advance current_area_idx_ past completed/attempted areas and fire the
  /// next existence probe. Returns RUNNING (probe in flight) or FAILURE.
  BT::NodeStatus advanceAndProbe();

  // Existence probe: GetMowingArea(index).success is false once index passes
  // the last defined area. Area completion is tracked in-memory via the
  // swath-completion model (ctx->completed_areas), not the removed cell grid.
  rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedPtr client_;
  std::optional<rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::FutureAndRequestId>
      pending_future_;
  std::chrono::steady_clock::time_point call_start_;
  uint32_t current_area_idx_{0};
  uint32_t max_areas_{20};
  uint32_t areas_queried_{0};
  uint32_t areas_complete_{0};
};

// ---------------------------------------------------------------------------
// PlanCoverageArea — calls map_server's ~/get_mowing_area for the area's
// polygon (outer + obstacle holes), then asks mowgli_coverage's
// /plan_coverage action for the EXPLICIT segment list (headland rings +
// straight serpentine swaths — no turn geometry). Stores the segments in
// ctx->current_strip_segments (and the concatenated path in
// ctx->current_strip_path for the GUI); FollowStrip executes them one
// FollowCoveragePath goal at a time.
//
// Plans the whole area in one shot at each (re)start; resume is swath-based
// (FollowStrip skips segment indices already in ctx->area_completed_swaths —
// the plan is deterministic for a fixed polygon + params).
// ---------------------------------------------------------------------------

class PlanCoverageArea : public BT::StatefulActionNode
{
public:
  using PlanCoverage = mowgli_interfaces::action::PlanCoverage;
  using PlanGoalHandle = rclcpp_action::ClientGoalHandle<PlanCoverage>;

  PlanCoverageArea(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<uint32_t>("area_index", 0u, "Mowing area index"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase
  {
    QueryRemaining,
    Dispatch,
    WaitingForGoal,
    WaitingForResult,
  };

  /// Build a PlanCoverage::Goal from the area polygon. The coverage
  /// geometry (operation_width, headland, insets) lives in the coverage
  /// server's parameters (injected at launch from mowgli_robot.yaml).
  PlanCoverage::Goal buildGoal(const mowgli_interfaces::msg::MapArea& area) const;

  rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedPtr srv_client_;
  std::optional<rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::FutureAndRequestId>
      srv_future_;

  rclcpp_action::Client<PlanCoverage>::SharedPtr action_client_;
  std::shared_future<PlanGoalHandle::SharedPtr> goal_future_;
  PlanGoalHandle::SharedPtr goal_handle_;
  std::shared_future<PlanGoalHandle::WrappedResult> result_future_;

  mowgli_interfaces::msg::MapArea area_;
  // Publishes the FULL plan (concatenation of all segments) for visualisation.
  // FollowStrip feeds the coverage controller one segment at a time (and
  // republishes only that segment on FollowCoveragePath/global_plan for the
  // goal checker), so without this the operator/GUI could only ever see a
  // single segment. Latched (transient_local) so a late GUI subscriber still
  // gets the whole plan for the current area.
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr full_plan_pub_;
  Phase phase_{Phase::QueryRemaining};
  std::chrono::steady_clock::time_point phase_start_;
};

}  // namespace mowgli_behavior
