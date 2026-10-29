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

// SPDX-License-Identifier: GPL-3.0

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_behavior/navigation_nodes.hpp"
#include "mowgli_interfaces/srv/get_recovery_point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <gtest/gtest.h>

namespace mowgli_behavior
{
struct NavigateToPoseOutcomeTestAccess
{
  static std::shared_ptr<ActionOutcomeSlot> outcome(NavigateToPose& node)
  {
    return node.outcome_;
  }
  static bool hasHandle(NavigateToPose& node)
  {
    return static_cast<bool>(node.goal_handle_);
  }
};
}  // namespace mowgli_behavior

namespace
{

using mowgli_behavior::BTContext;
using mowgli_behavior::NavigateInsideBoundary;
using NavigateAction = nav2_msgs::action::NavigateToPose;
using NavigateGoalHandle = rclcpp_action::ServerGoalHandle<NavigateAction>;
using RecoverySrv = mowgli_interfaces::srv::GetRecoveryPoint;
using ClearSrv = nav2_msgs::srv::ClearEntireCostmap;
using ToggleSrv = std_srvs::srv::SetBool;

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

::testing::Environment* const rclcpp_env =
    ::testing::AddGlobalTestEnvironment(new RclcppEnvironment());

class BoundaryRecoveryTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    client_node_ = rclcpp::Node::make_shared("boundary_recovery_client");
    server_node_ = rclcpp::Node::make_shared("boundary_recovery_server");

    ctx_ = std::make_shared<BTContext>();
    ctx_->node = client_node_;
    ctx_->boundary_recovery_tree_xml = "explicit_boundary_recovery.xml";
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("context", ctx_);
    factory_.registerNodeType<NavigateInsideBoundary>("NavigateInsideBoundary");
    factory_.registerNodeType<mowgli_behavior::NavigateToPose>("NavigateToPose");

    recovery_service_ =
        server_node_->create_service<RecoverySrv>("/map_server_node/get_recovery_point",
                                                  [this](const RecoverySrv::Request::SharedPtr,
                                                         RecoverySrv::Response::SharedPtr response)
                                                  {
                                                    events_.push_back("recovery_pose");
                                                    response->success = true;
                                                    response->recovery_pose.position.x = 1.0;
                                                    response->recovery_pose.orientation.w = 1.0;
                                                  });

    toggle_service_ = server_node_->create_service<ToggleSrv>(
        "/global_costmap/keepout_filter/toggle_filter",
        [this](const ToggleSrv::Request::SharedPtr request, ToggleSrv::Response::SharedPtr response)
        {
          events_.push_back(request->data ? "keepout_on" : "keepout_off");
          response->success = toggle_succeeds_;
          response->message = toggle_succeeds_ ? "ok" : "refused";
        });

    clear_service_ =
        server_node_->create_service<ClearSrv>("/global_costmap/clear_entirely_global_costmap",
                                               [this](const ClearSrv::Request::SharedPtr,
                                                      ClearSrv::Response::SharedPtr)
                                               {
                                                 events_.push_back("clear_costmap");
                                               });

    navigate_server_ = rclcpp_action::create_server<NavigateAction>(
        server_node_,
        "/navigate_to_pose",
        [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const NavigateAction::Goal> goal)
        {
          if (!manual_navigation_)
            EXPECT_EQ(goal->behavior_tree, ctx_->boundary_recovery_tree_xml);
          events_.push_back("navigate_goal");
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<NavigateGoalHandle>)
        {
          ++cancel_requests_;
          return manual_navigation_ ? rclcpp_action::CancelResponse::REJECT
                                    : rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<NavigateGoalHandle> goal_handle)
        {
          if (manual_navigation_)
          {
            navigation_handles_.push_back(goal_handle);
            return;
          }
          auto result = std::make_shared<NavigateAction::Result>();
          if (navigation_succeeds_)
          {
            goal_handle->succeed(result);
          }
          else
          {
            goal_handle->abort(result);
          }
        });

    executor_.add_node(client_node_);
    executor_.add_node(server_node_);
  }

  void TearDown() override
  {
    executor_.remove_node(client_node_);
    executor_.remove_node(server_node_);
  }

  BT::NodeStatus runTree()
  {
    const std::string xml =
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
        "<NavigateInsideBoundary/>"
        "</BehaviorTree></root>";
    auto tree = factory_.createTreeFromText(xml, blackboard_);

    BT::NodeStatus status = tree.tickOnce();
    for (int i = 0; i < 300 && status == BT::NodeStatus::RUNNING; ++i)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      status = tree.tickOnce();
    }
    executor_.spin_some();
    return status;
  }

  std::shared_ptr<BTContext> ctx_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Node::SharedPtr server_node_;
  BT::Blackboard::Ptr blackboard_;
  BT::BehaviorTreeFactory factory_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::vector<std::string> events_;
  bool manual_navigation_{false};
  std::size_t cancel_requests_{0};
  std::vector<std::shared_ptr<NavigateGoalHandle>> navigation_handles_;
  bool toggle_succeeds_{true};
  bool navigation_succeeds_{true};
  rclcpp::Service<RecoverySrv>::SharedPtr recovery_service_;
  rclcpp::Service<ToggleSrv>::SharedPtr toggle_service_;
  rclcpp::Service<ClearSrv>::SharedPtr clear_service_;
  rclcpp_action::Server<NavigateAction>::SharedPtr navigate_server_;
};

TEST_F(BoundaryRecoveryTest, NavigateToPoseIgnoresLateOldSuccess)
{
  manual_navigation_ = true;
  auto tree = factory_.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"MainTree\">"
      "<NavigateToPose goal=\"2;0;0\"/></BehaviorTree></root>",
      blackboard_);
  auto* node = dynamic_cast<mowgli_behavior::NavigateToPose*>(tree.rootNode());
  ASSERT_NE(node, nullptr);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  const auto pump_until = [&](const std::function<bool()>& done)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!done() && std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
  };
  ASSERT_TRUE(pump_until(
      [&]()
      {
        return navigation_handles_.size() == 1;
      }));
  ASSERT_TRUE(pump_until(
      [&]()
      {
        tree.tickOnce();
        return mowgli_behavior::NavigateToPoseOutcomeTestAccess::hasHandle(*node);
      }));
  auto old_outcome = mowgli_behavior::NavigateToPoseOutcomeTestAccess::outcome(*node);
  tree.haltTree();
  ASSERT_TRUE(pump_until(
      [&]()
      {
        return cancel_requests_ > 0;
      }));
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  ASSERT_TRUE(pump_until(
      [&]()
      {
        return navigation_handles_.size() == 2;
      }));
  ASSERT_TRUE(pump_until(
      [&]()
      {
        tree.tickOnce();
        return mowgli_behavior::NavigateToPoseOutcomeTestAccess::hasHandle(*node);
      }));
  auto new_outcome = mowgli_behavior::NavigateToPoseOutcomeTestAccess::outcome(*node);
  navigation_handles_[0]->succeed(std::make_shared<NavigateAction::Result>());
  ASSERT_TRUE(pump_until(
      [&]()
      {
        return old_outcome->Get().has_value();
      }));
  ASSERT_EQ(old_outcome->Get(), mowgli_behavior::GoalOutcome::kSucceeded);
  ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING)
      << "the old result must not complete the current navigation";
  ASSERT_FALSE(new_outcome->Get().has_value());
  navigation_handles_[1]->succeed(std::make_shared<NavigateAction::Result>());
  ASSERT_TRUE(pump_until(
      [&]()
      {
        return new_outcome->Get().has_value();
      }));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(BoundaryRecoveryTest, TogglesFilterAroundSuccessfulNavigation)
{
  EXPECT_EQ(runTree(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(events_,
            (std::vector<std::string>{
                "recovery_pose", "keepout_off", "clear_costmap", "navigate_goal", "keepout_on"}));
}

TEST_F(BoundaryRecoveryTest, NavigationAbortReEnablesWithoutBlindBackup)
{
  navigation_succeeds_ = false;
  EXPECT_EQ(runTree(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(events_,
            (std::vector<std::string>{
                "recovery_pose", "keepout_off", "clear_costmap", "navigate_goal", "keepout_on"}));
}

TEST_F(BoundaryRecoveryTest, ToggleFailureRefusesMotion)
{
  toggle_succeeds_ = false;
  EXPECT_EQ(runTree(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(events_, (std::vector<std::string>{"recovery_pose", "keepout_off"}));
}

}  // namespace
