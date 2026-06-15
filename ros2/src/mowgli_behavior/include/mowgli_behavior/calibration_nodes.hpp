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

#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "mowgli_behavior/bt_context.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// RecordUndockStart — snapshot GPS position before the undock BackUp
// ---------------------------------------------------------------------------

class RecordUndockStart : public BT::SyncActionNode
{
public:
  RecordUndockStart(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }
  BT::NodeStatus tick() override;
};

// ---------------------------------------------------------------------------
// CalibrateHeadingFromUndock — derive yaw from the BackUp displacement
// ---------------------------------------------------------------------------

/// Sync node that runs immediately after the UndockSequence BackUp. Uses
/// the GPS position captured by RecordUndockStart and the current GPS
/// position to compute the robot's heading: since the BackUp reversed
/// ~1 m in a straight line, the motion vector points OPPOSITE to the
/// robot's heading, so `heading = atan2(-dy, -dx)` (equivalent to adding
/// π to the motion angle).
///
/// Publishes a PoseWithCovarianceStamped seed to /ekf_map_node/set_pose
/// so the global EKF aligns with reality before any Nav2 goal runs. Sets
/// ctx->yaw_seeded_this_session so the off-dock SeedYawFromMotion node
/// will skip if triggered later in the same session.
///
/// Returns FAILURE if the GPS displacement is below min_displacement_m
/// (typically because the BackUp reported complete but the robot never
/// physically left the dock) — this propagates back up to UndockSequence
/// failure and the BT retries the whole undock, giving the mechanical
/// setup another chance to break free.
class CalibrateHeadingFromUndock : public BT::SyncActionNode
{
public:
  CalibrateHeadingFromUndock(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("min_displacement_m",
                              0.5,
                              "Minimum GPS displacement for a valid yaw"),
    };
  }
  BT::NodeStatus tick() override;

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_pub_;
};

// ---------------------------------------------------------------------------
// SeedYawFromMotion — forward-drive yaw seeder for off-dock starts
// ---------------------------------------------------------------------------

/// Used only in the NotDockedBranch of the undock fallback, where no
/// BackUp happens and therefore CalibrateHeadingFromUndock has no prior
/// motion to work with. Drives the robot forward a short distance under
/// collision_monitor supervision, then derives yaw from the GPS track
/// direction and publishes a set_pose seed.
///
/// Guarded by ctx->yaw_seeded_this_session so that a ReactiveSequence
/// halt/resume cycle does not re-trigger the forward drive mid-session.
class SeedYawFromMotion : public BT::StatefulActionNode
{
public:
  SeedYawFromMotion(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("distance_m", 1.0, "Forward distance [m] to drive for yaw seeding"),
        BT::InputPort<double>("speed_ms", 0.2, "Forward speed [m/s] during the seed drive"),
        BT::InputPort<double>("timeout_sec", 30.0, "Abort if distance not reached within timeout"),
        BT::InputPort<double>("min_displacement_m",
                              0.5,
                              "Minimum GPS displacement for a valid yaw"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  void publish_forward(const rclcpp::Node::SharedPtr& node, double speed);
  void publish_zero(const rclcpp::Node::SharedPtr& node);

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr set_pose_pub_;

  double x0_{0.0};
  double y0_{0.0};
  double distance_m_{1.0};
  double speed_ms_{0.2};
  double timeout_sec_{30.0};
  double min_displacement_m_{0.5};
  rclcpp::Time start_time_;
};

}  // namespace mowgli_behavior
