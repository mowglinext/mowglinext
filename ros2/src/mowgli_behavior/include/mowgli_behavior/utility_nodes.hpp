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
#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/srv/emergency_stop.hpp"
#include "mowgli_interfaces/srv/mower_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// SetMowerEnabled
// ---------------------------------------------------------------------------

/// Calls the /hardware_bridge/mower_control service to enable or disable the
/// cutting blade motor.
///
/// Input ports:
///   enabled (bool) – true to start the blade, false to stop it.
class SetMowerEnabled : public BT::SyncActionNode
{
public:
  SetMowerEnabled(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<bool>("enabled", "Enable (true) or disable (false) the mow motor")};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<mowgli_interfaces::srv::MowerControl>::SharedPtr client_;
};

// ---------------------------------------------------------------------------
// WaitForDuration
// ---------------------------------------------------------------------------

/// Stateful action that returns RUNNING until the requested duration has
/// elapsed, then returns SUCCESS.
///
/// Input ports:
///   duration_sec (double, default "1.0") – wait duration in seconds.
class WaitForDuration : public BT::StatefulActionNode
{
public:
  WaitForDuration(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("duration_sec", 1.0, "Duration to wait in seconds")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::duration<double> duration_;
};

// ---------------------------------------------------------------------------
// WaitForGpsFix
// ---------------------------------------------------------------------------

/// Stateful action that returns RUNNING until the GPS fix quality reaches at
/// least `min_fix_type`, then returns SUCCESS. On `timeout_sec` elapsed
/// without meeting the threshold it **still returns SUCCESS** and logs a
/// warning — the intent is to let the robot start/resume mowing at degraded
/// precision rather than freeze indefinitely at a partly-obstructed site.
///
/// Typical use: insert after BackUp (undock) and before the first
/// TransitToStrip, to give the F9P a few seconds out from under the dock's
/// metal canopy to re-acquire RTK before navigation starts.
///
/// Input ports:
///   timeout_sec    (double, default 20.0) – max wait before proceeding.
///   min_fix_type   (int,    default 2)    – BTContext gps_fix_type threshold
///                                           (quality-monotonic, higher=better):
///                                           0=no fix, 2=DGPS/SBAS,
///                                           3=RTK Float, 4=RTK Fixed.
class WaitForGpsFix : public BT::StatefulActionNode
{
public:
  WaitForGpsFix(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<double>("timeout_sec", 20.0, "Max seconds to wait before proceeding anyway"),
        BT::InputPort<int>("min_fix_type", 2, "Min BtContext gps_fix_type (2=DGPS, 4=RTK Fixed)"),
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::duration<double> timeout_{};
  int min_fix_type_{2};
};

// ---------------------------------------------------------------------------
// SaveObstacles
// ---------------------------------------------------------------------------

/// Calls /obstacle_tracker/save_obstacles to persist the obstacle map to disk.
class SaveObstacles : public BT::SyncActionNode
{
public:
  SaveObstacles(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

// ---------------------------------------------------------------------------
// ResetEmergency
// ---------------------------------------------------------------------------

/// Calls the /hardware_bridge/emergency_stop service with emergency=0 to
/// release a latched emergency state in the firmware.  Used to auto-clear
/// emergencies when the robot is placed back on the dock.
///
/// Returns SUCCESS when the release request is sent (fire-and-forget to the
/// firmware, which is the safety authority and may refuse the release if a
/// physical trigger is still asserted).
class ResetEmergency : public BT::SyncActionNode
{
public:
  ResetEmergency(const std::string& name, const BT::NodeConfig& config)
      : BT::SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;

private:
  rclcpp::Client<mowgli_interfaces::srv::EmergencyStop>::SharedPtr client_;
};

}  // namespace mowgli_behavior
