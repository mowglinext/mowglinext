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

#include <filesystem>
#include <fstream>
#include <future>
#include <thread>

#include <gtest/gtest.h>
#define main fleet_behavior_main_for_test
#include "../src/behavior_tree_node.cpp"  // NOLINT(build/include)
#undef main
#include "mowgli_behavior/status_nodes.hpp"

namespace mowgli_behavior
{
struct FleetYieldServiceTestPeer
{
  static void setup(const std::shared_ptr<BehaviorTreeNode>& node,
                    const std::function<BT::NodeStatus(BT::TreeNode&)>& tick)
  {
    node->context_->node = node;
    node->context_->helper_node = rclcpp::Node::make_shared("fleet_test_helper");
    node->setupSubscribers();
    node->setupServiceServer();
    node->factory_.registerSimpleAction("TestTick", tick);
    node->tree_ = node->factory_.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\"><TestTick/></BehaviorTree></root>");
    node->declare_parameter("tick_rate", 100.0);
    node->setupTimer();
    node->tick_timer_->cancel();
  }
  static void startTimer(BehaviorTreeNode& node)
  {
    node.tick_timer_->reset();
  }
  static void tick(BehaviorTreeNode& node)
  {
    node.tickTree();
  }
  static void stopTimer(BehaviorTreeNode& node)
  {
    node.tick_timer_->cancel();
  }
  static bool sharedGroup(BehaviorTreeNode& node)
  {
    auto group = node.get_node_base_interface()->get_default_callback_group();
    return group->type() == rclcpp::CallbackGroupType::MutuallyExclusive &&
           group->find_timer_ptrs_if(
               [&](const auto& timer)
               {
                 return timer == node.tick_timer_;
               }) &&
           group->find_service_ptrs_if(
               [&](const auto& service)
               {
                 return service == node.high_level_control_srv_;
               }) &&
           group->find_service_ptrs_if(
               [&](const auto& service)
               {
                 return service == node.start_in_area_srv_;
               }) &&
           group->find_service_ptrs_if(
               [&](const auto& service)
               {
                 return service == node.clear_coverage_resume_srv_;
               }) &&
           group->find_service_ptrs_if(
               [](const auto& service)
               {
                 return std::string(service->get_service_name()) ==
                        "/mowgli_behavior_node/fleet_yield";
               });
  }
};

class FleetYieldServices : public ::testing::Test
{
protected:
  using Fleet = mowgli_interfaces::srv::FleetYield;
  using Request = Fleet::Request;
  using Response = Fleet::Response;
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
  void SetUp() override
  {
    node = std::make_shared<BehaviorTreeNode>();
    FleetYieldServiceTestPeer::setup(node,
                                     [](auto&)
                                     {
                                       return BT::NodeStatus::SUCCESS;
                                     });
    ctx = node->context();
    client_node = rclcpp::Node::make_shared("fleet_test_client");
    executor.add_node(node);
    executor.add_node(client_node);
    client = client_node->create_client<Fleet>("/mowgli_behavior_node/fleet_yield");
    ASSERT_TRUE(client->wait_for_service(5s));
    ASSERT_TRUE(FleetYieldServiceTestPeer::sharedGroup(*node));
    factory.registerNodeType<IsEmergency>("IsEmergency");
    factory.registerNodeType<PublishHighLevelStatus>("PublishHighLevelStatus");
    factory.registerNodeType<ClearCommand>("ClearCommand");
    factory.registerNodeType<MarkGuardHalt>("MarkGuardHalt");
    factory.registerNodeType<EndSession>("EndSession");
    blackboard = BT::Blackboard::create();
    blackboard->set("context", ctx);
    status_tree = factory.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Status\">"
        "<PublishHighLevelStatus state=\"{state}\" state_name=\"{name}\"/>"
        "</BehaviorTree></root>",
        blackboard);
    ctx->last_emergency_time = std::chrono::steady_clock::now();
    ctx->current_command = 1;
    status(2, "MOWING");
  }
  void TearDown() override
  {
    unsetenv("MOWGLI_UPDATE_MAINTENANCE");
    ctx->node.reset();
    if (!temp_dir.empty())
      std::filesystem::remove_all(temp_dir);
  }
  void status(uint8_t state, const std::string& name)
  {
    blackboard->set("state", state);
    blackboard->set("name", name);
    ASSERT_EQ(status_tree.tickOnce(), BT::NodeStatus::SUCCESS);
  }
  void idle()
  {
    for (int i = 0; i < 3; ++i)
      status(1, "IDLE");
  }
  Response call(uint8_t operation, const std::string& epoch = "", const std::string& id = "")
  {
    auto req = std::make_shared<Request>();
    req->operation = operation;
    req->epoch = epoch;
    req->request_id = id;
    auto future = client->async_send_request(req);
    EXPECT_EQ(executor.spin_until_future_complete(future, 5s), rclcpp::FutureReturnCode::SUCCESS);
    return *future.get();
  }
  template <class Service>
  void operatorCall(const std::string& name, std::shared_ptr<typename Service::Request> req)
  {
    auto other = client_node->create_client<Service>("/mowgli_behavior_node/" + name);
    ASSERT_TRUE(other->wait_for_service(5s));
    auto future = other->async_send_request(req);
    ASSERT_EQ(executor.spin_until_future_complete(future, 5s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);
  }
  std::string pause()
  {
    // Ordinary lifecycles start with healthy emergency telemetry. Stale-stream
    // regressions explicitly age the receipt AFTER acquisition; call() never
    // refreshes it, so those tests exercise the actual service-time predicate.
    ctx->last_emergency_time = std::chrono::steady_clock::now();
    const auto epoch = call(Request::QUERY).epoch;
    EXPECT_EQ(call(Request::PAUSE, epoch, "client-a").result, Response::HELD);
    return epoch;
  }
  void scratch()
  {
    char path[] = "/tmp/fleet-yield-test-XXXXXX";
    auto created = mkdtemp(path);
    ASSERT_NE(created, nullptr);
    temp_dir = created;
  }
  template <class Message, class Predicate>
  void deliver(const std::string& topic, const Message& message, Predicate done)
  {
    auto pub = client_node->create_publisher<Message>(topic, 10);
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!done() && std::chrono::steady_clock::now() < deadline)
    {
      pub->publish(message);
      executor.spin_some();
      std::this_thread::sleep_for(5ms);
    }
    ASSERT_TRUE(done());
  }
  std::shared_ptr<BehaviorTreeNode> node;
  std::shared_ptr<BTContext> ctx;
  rclcpp::Node::SharedPtr client_node;
  rclcpp::Client<Fleet>::SharedPtr client;
  rclcpp::executors::SingleThreadedExecutor executor;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr blackboard;
  BT::Tree status_tree;
  std::string temp_dir;
};

TEST_F(FleetYieldServices, LostPauseAndResumeRepliesAreIdempotentAndPreserveTheRun)
{
  ctx->single_area_target = 2;
  ctx->single_area_target_id = 42;
  ctx->target_area_index = 3;
  ctx->area_resume_pose_index[2] = 123;
  ctx->blade_direction.forOperatorCommand(false, 1);
  const auto epoch = pause();  // Caller can lose this reply.
  EXPECT_EQ(ctx->current_command, 8);
  EXPECT_EQ(call(Request::QUERY).result, Response::AVAILABLE);
  EXPECT_TRUE(call(Request::QUERY).held);  // Does not grant a second client ownership.
  EXPECT_EQ(call(Request::PAUSE, epoch, "client-a").result, Response::HELD);
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::NOT_READY);
  // The first two IDLE publishes may still carry numeric AUTONOMOUS.
  status(1, "IDLE");
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::NOT_READY);
  status(1, "IDLE");
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::NOT_READY);
  status(1, "IDLE");
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::RESUMED);
  status(2, "MOWING");
  const auto current_epoch = call(Request::QUERY).epoch;
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::ALREADY_RESUMED);
  EXPECT_EQ(call(Request::QUERY).epoch, current_epoch);
  EXPECT_EQ(call(Request::PAUSE, epoch, "client-a").result, Response::REVOKED);
  EXPECT_EQ(ctx->current_command, 1);
  EXPECT_EQ(ctx->single_area_target, 2u);
  EXPECT_EQ(ctx->single_area_target_id, 42u);
  EXPECT_EQ(ctx->target_area_index, 3);
  EXPECT_EQ(ctx->area_resume_pose_index[2], 123u);
  EXPECT_EQ(ctx->blade_direction.forMowerCommand(true, true).enabled, 0u);
}

TEST_F(FleetYieldServices, CancelBeforeDelayedPauseAndDifferentClientsCannotSteal)
{
  auto epoch = call(Request::QUERY).epoch;
  EXPECT_EQ(call(Request::CANCEL, epoch, "client-a").result, Response::CANCELLED);
  EXPECT_EQ(call(Request::PAUSE, epoch, "client-a").result, Response::REVOKED);
  EXPECT_EQ(ctx->current_command,
            1);  // Cancelling an unacquired fence does not stop an operator run.
  epoch = pause();
  for (auto operation : {Request::PAUSE, Request::RESUME, Request::CANCEL})
    EXPECT_EQ(call(operation, epoch, "client-b").result, Response::BUSY);
  EXPECT_EQ(call(Request::CANCEL, epoch, "client-a").result, Response::CANCELLED);
  EXPECT_EQ(ctx->current_command, 8);
  EXPECT_FALSE(call(Request::QUERY).held);
  idle();
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::REVOKED);
}

TEST_F(FleetYieldServices, AllAcceptedOperatorCommandsRevokeIncludingRepeatedStop)
{
  using Control = mowgli_interfaces::srv::HighLevelControl;
  for (uint8_t command : {1, 2, 3, 4, 5, 6, 7, 8, 254, 255})
  {
    ctx->current_command = 1;
    status(2, "MOWING");
    const auto epoch = pause();
    auto req = std::make_shared<Control::Request>();
    req->command = command;
    operatorCall<Control>("high_level_control", req);
    EXPECT_FALSE(call(Request::QUERY).held);
    EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::REVOKED);
    EXPECT_EQ(ctx->current_command, command == 4 ? 1 : command);
    EXPECT_EQ(call(Request::PAUSE, epoch, "client-a").result, Response::REVOKED);
  }
}

TEST_F(FleetYieldServices, StartInAreaAndImmediateResetRevokeBeforeAnyTick)
{
  auto epoch = pause();
  auto req = std::make_shared<mowgli_interfaces::srv::StartInArea::Request>();
  req->area = 4;
  operatorCall<mowgli_interfaces::srv::StartInArea>("start_in_area", req);
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::REVOKED);
  EXPECT_EQ(ctx->target_area_index, 4);
  status(2, "MOWING");
  epoch = pause();
  idle();
  operatorCall<std_srvs::srv::Trigger>("clear_coverage_resume",
                                       std::make_shared<std_srvs::srv::Trigger::Request>());
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::REVOKED);
  EXPECT_EQ(ctx->current_command, 8);
}

TEST_F(FleetYieldServices, NamedHoldsFenceActiveAndUnacquiredEpochsEvenAfterClearing)
{
  for (const auto& [state, name] :
       std::vector<std::pair<uint8_t, std::string>>{{1, "DIG_OBSTRUCTION"},
                                                    {1, "CHARGING"},
                                                    {1, "MANUAL_CHARGING"},
                                                    {1, "RAIN_WAITING"},
                                                    {2, "WAITING_FOR_RTK"},
                                                    {0, "EMERGENCY"},
                                                    {2, "RETURNING_HOME"},
                                                    {2, "MOWING_COMPLETE"},
                                                    {1, "IDLE_DOCKED"}})
  {
    ctx->current_command = 1;
    status(2, "MOWING");
    const auto pending_epoch = call(Request::QUERY).epoch;
    status(state, name);
    const auto blocked_epoch = call(Request::QUERY).epoch;
    status(2, "MOWING");
    EXPECT_EQ(call(Request::PAUSE, pending_epoch, "pending").result, Response::REVOKED) << name;
    EXPECT_EQ(call(Request::PAUSE, blocked_epoch, "pending").result, Response::REVOKED) << name;
    const auto epoch = pause();
    status(state, name);
    idle();
    EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::REVOKED) << name;
    EXPECT_EQ(ctx->current_command, 8);
  }
}

TEST_F(FleetYieldServices, UnpublishedGuardHoldFencesUntilANewTreeStatus)
{
  auto guard = factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Guard\">"
      "<MarkGuardHalt reason=\"scan_stale\"/></BehaviorTree></root>",
      blackboard);
  const auto pending = call(Request::QUERY).epoch;
  ASSERT_EQ(guard.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto during = call(Request::QUERY).epoch;
  EXPECT_NE(pending, during);
  EXPECT_EQ(ctx->last_high_level_status.state_name, "MOWING");  // Still the old cache.
  EXPECT_EQ(call(Request::PAUSE, during, "pending").result, Response::INELIGIBLE);
  ASSERT_EQ(guard.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(call(Request::QUERY).epoch, during);
  status(2, "MOWING");  // Production tree can publish again after the guard releases.
  EXPECT_EQ(call(Request::PAUSE, pending, "pending").result, Response::REVOKED);
  EXPECT_EQ(call(Request::PAUSE, during, "pending").result, Response::REVOKED);
  const auto active = pause();
  ASSERT_EQ(guard.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_FALSE(call(Request::QUERY).held);
  idle();
  EXPECT_EQ(call(Request::RESUME, active, "client-a").result, Response::REVOKED);
}

TEST_F(FleetYieldServices, EmergencyAssertionRevokesThroughProductionSubscriber)
{
  auto epoch = pause();
  mowgli_interfaces::msg::Emergency emergency;
  emergency.active_emergency = true;
  deliver("/hardware_bridge/emergency",
          emergency,
          [&]
          {
            return ctx->latest_emergency.active_emergency;
          });
  EXPECT_FALSE(ctx->fleet_yield.held());
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::INELIGIBLE);
  emergency.active_emergency = false;
  deliver("/hardware_bridge/emergency",
          emergency,
          [&]
          {
            return !ctx->latest_emergency.active_emergency;
          });
  idle();
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::REVOKED);
}

TEST_F(FleetYieldServices, ChargerTelemetryRevokesBeforeStatusCanReturnIdle)
{
  const auto epoch = pause();
  mowgli_interfaces::msg::Status hardware;
  hardware.is_charging = true;
  deliver("/hardware_bridge/status",
          hardware,
          [&]
          {
            return ctx->latest_status.is_charging;
          });
  EXPECT_FALSE(ctx->fleet_yield.held());
  hardware.is_charging = false;
  deliver("/hardware_bridge/status",
          hardware,
          [&]
          {
            return !ctx->latest_status.is_charging;
          });
  idle();
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::REVOKED);
}

TEST_F(FleetYieldServices, PowerChargerRevokesReadyHoldEvenWithInvalidVoltageAndNoStatusUpdate)
{
  for (float voltage : {0.0f, 1.0f})
  {
    ctx->current_command = 1;
    status(2, "MOWING");
    const auto epoch = pause();
    idle();
    ASSERT_TRUE(ctx->fleet_yield.held());
    ASSERT_FALSE(ctx->latest_status.is_charging);  // Status may be absent or delayed.
    mowgli_interfaces::msg::Power power;
    power.charger_enabled = true;
    power.v_battery = voltage;  // The battery filter rejects both readings.
    deliver("/hardware_bridge/power",
            power,
            [&]
            {
              return ctx->latest_power.charger_enabled;
            });
    EXPECT_FALSE(ctx->fleet_yield.held());  // Revoke in the subscriber, before any request.
    EXPECT_EQ(ctx->current_command, 8);
    EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::INELIGIBLE);
    EXPECT_EQ(ctx->current_command, 8);
    EXPECT_FALSE(ctx->latest_status.is_charging);
    power.charger_enabled = false;
    deliver("/hardware_bridge/power",
            power,
            [&]
            {
              return !ctx->latest_power.charger_enabled;
            });
    EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::REVOKED);
    EXPECT_EQ(call(Request::PAUSE, epoch, "client-a").result, Response::REVOKED);
    EXPECT_EQ(ctx->current_command, 8);
  }
}

TEST_F(FleetYieldServices, StaleClearEmergencyRevokesReadyHoldBeforeAnyEmergencyStatus)
{
  const auto epoch = pause();
  idle();
  auto emergency_tree = factory.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Emergency\">"
      "<IsEmergency/></BehaviorTree></root>",
      blackboard);
  ASSERT_FALSE(ctx->latest_emergency.active_emergency);
  ASSERT_FALSE(ctx->latest_emergency.latched_emergency);
  const auto stale_receipt = std::chrono::steady_clock::now() - 3s;
  ctx->last_emergency_time = stale_receipt;
  // Preserve the existing strict >2 s boundary without sleeping on a timer.
  EXPECT_FALSE(ctx->emergencyActiveOrStale(stale_receipt + 2s));
  EXPECT_TRUE(ctx->emergencyActiveOrStale(stale_receipt + 2s + 1ns));
  ASSERT_EQ(emergency_tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::INELIGIBLE);
  EXPECT_FALSE(ctx->fleet_yield.held());
  EXPECT_EQ(ctx->current_command, 8);
  EXPECT_EQ(ctx->last_high_level_status.state_name, "IDLE");  // No EMERGENCY publication yet.
  mowgli_interfaces::msg::Emergency fresh_clear;
  deliver("/hardware_bridge/emergency",
          fresh_clear,
          [&]
          {
            return ctx->last_emergency_time > stale_receipt;
          });
  ASSERT_EQ(emergency_tree.tickOnce(), BT::NodeStatus::FAILURE);
  const auto fresh_epoch = ctx->fleet_yield.epoch();  // Subscriber already observed the recovery.
  EXPECT_EQ(call(Request::QUERY).epoch, fresh_epoch);
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::REVOKED);
  EXPECT_EQ(call(Request::PAUSE, epoch, "client-a").result, Response::REVOKED);
  EXPECT_EQ(ctx->current_command, 8);
}

TEST_F(FleetYieldServices, BoundarySubscribersFencePendingEpochAndOwnedHoldBeforeUnsafeStatus)
{
  for (bool lethal : {false, true})
  {
    const std::string topic = lethal ? "/map_server_node/lethal_boundary_violation"
                                     : "/map_server_node/boundary_violation";
    const auto asserted = [&]
    {
      return lethal ? ctx->lethal_boundary_violation : ctx->boundary_violation;
    };
    ctx->current_command = 1;
    status(2, "MOWING");
    const auto pending = call(Request::QUERY).epoch;
    std_msgs::msg::Bool signal;
    signal.data = true;
    deliver(topic, signal, asserted);
    const auto during = ctx->fleet_yield.epoch();
    EXPECT_NE(pending, during);  // Immediate revocation, before any fleet request/status tick.
    EXPECT_EQ(ctx->last_high_level_status.state_name, "MOWING");
    EXPECT_EQ(call(Request::PAUSE, pending, "pending").result, Response::INELIGIBLE);
    EXPECT_EQ(ctx->current_command, 1);  // Fleet interlock does not rewrite raw command policy.
    signal.data = false;
    deliver(topic,
            signal,
            [&]
            {
              return !asserted();
            });
    EXPECT_EQ(call(Request::PAUSE, pending, "pending").result, Response::REVOKED);
    EXPECT_EQ(call(Request::PAUSE, during, "pending").result, Response::REVOKED);
    const auto owned = pause();
    idle();
    signal.data = true;
    deliver(topic, signal, asserted);
    EXPECT_FALSE(ctx->fleet_yield.held());
    EXPECT_EQ(ctx->last_high_level_status.state_name, "IDLE");
    EXPECT_EQ(call(Request::RESUME, owned, "client-a").result, Response::INELIGIBLE);
    signal.data = false;
    deliver(topic,
            signal,
            [&]
            {
              return !asserted();
            });
    EXPECT_EQ(call(Request::RESUME, owned, "client-a").result, Response::REVOKED);
    EXPECT_EQ(ctx->current_command, 8);
  }
}

TEST_F(FleetYieldServices, MaintenanceIsTerminalAndDoesNotRotateEveryRequest)
{
  scratch();
  const auto path = temp_dir + "/maintenance";
  ASSERT_EQ(setenv("MOWGLI_UPDATE_MAINTENANCE", path.c_str(), 1), 0);
  const auto epoch = pause();
  std::ofstream(path) << "maintenance";
  FleetYieldServiceTestPeer::tick(*node);  // Entry revokes even without a fleet request.
  EXPECT_FALSE(ctx->fleet_yield.held());
  const auto entry_epoch = ctx->fleet_yield.epoch();
  FleetYieldServiceTestPeer::tick(*node);
  EXPECT_EQ(ctx->fleet_yield.epoch(), entry_epoch);
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::INELIGIBLE);
  const auto during = call(Request::QUERY).epoch;
  EXPECT_EQ(call(Request::QUERY).epoch, during);
  EXPECT_EQ(call(Request::PAUSE, during, "new").result, Response::INELIGIBLE);
  std::filesystem::remove(path);
  idle();
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::REVOKED);
  EXPECT_EQ(call(Request::PAUSE, during, "new").result, Response::REVOKED);
  EXPECT_EQ(ctx->current_command, 8);
}

TEST_F(FleetYieldServices, AcceptedPauseReplacesPersistedStartBeforeReplyAndRestartRejectsOldTokens)
{
  scratch();
  ctx->coverage_resume_path = temp_dir + "/coverage_resume.txt";
  ctx->area_resume_pose_index[2] = 12;
  ctx->single_area_target = 2;
  ASSERT_TRUE(saveCoverageResumeState(*ctx));
  BTContext previous;
  previous.coverage_resume_path = ctx->coverage_resume_path;
  ASSERT_TRUE(loadCoverageResumeState(previous));
  ASSERT_EQ(previous.current_command, 1);
  const auto epoch = pause();
  // Boot the actual production init() against the checkpoint saved by PAUSE.
  // Remove the old service before creating the replacement with the same name.
  executor.remove_node(node);
  ctx->node.reset();
  node.reset();
  const auto tree_path = temp_dir + "/boot.xml";
  std::ofstream(tree_path) << "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Boot\">"
                              "<AlwaysSuccess/></BehaviorTree></root>";
  auto options = rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("coverage_resume_path", previous.coverage_resume_path),
       rclcpp::Parameter("tree_file", tree_path)});
  node = std::make_shared<BehaviorTreeNode>(options);
  node->init();
  FleetYieldServiceTestPeer::stopTimer(*node);
  ctx = node->context();
  executor.add_node(node);
  EXPECT_EQ(ctx->current_command, 0);  // init() never auto-continues STOP.
  EXPECT_EQ(ctx->single_area_target, 2u);
  EXPECT_FALSE(call(Request::QUERY).held);
  for (auto operation : {Request::PAUSE, Request::RESUME, Request::CANCEL})
    EXPECT_EQ(call(operation, epoch, "client-a").result, Response::REVOKED);
  EXPECT_EQ(ctx->current_command, 0);
  node->releaseResources();
}

TEST_F(FleetYieldServices, PersistenceFailureRetainsStopAndNeverAcknowledgesOwnership)
{
  scratch();
  ctx->coverage_resume_path = temp_dir + "/coverage_resume.txt";
  ctx->area_resume_pose_index[2] = 12;
  ASSERT_TRUE(saveCoverageResumeState(*ctx));
  // Block the sibling temp file without permissions/root-specific behavior.
  std::filesystem::create_directory(ctx->coverage_resume_path + ".tmp");
  const auto epoch = call(Request::QUERY).epoch;
  const auto failed = call(Request::PAUSE, epoch, "client-a");
  EXPECT_EQ(failed.result, Response::INELIGIBLE);
  EXPECT_FALSE(failed.held);
  EXPECT_FALSE(failed.message.empty());
  EXPECT_EQ(ctx->current_command, 8);
  EXPECT_EQ(call(Request::PAUSE, epoch, "client-a").result, Response::REVOKED);
  EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::REVOKED);
}

TEST_F(FleetYieldServices, ClearCommandAndEndSessionRevokeUsingProductionTreeNodes)
{
  for (const auto* action : {"ClearCommand", "EndSession"})
  {
    ctx->current_command = 1;
    status(2, "MOWING");
    const auto epoch = pause();
    auto tree =
        factory.createTreeFromText(std::string(
                                       "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Action\"><") +
                                       action + "/></BehaviorTree></root>",
                                   blackboard);
    ASSERT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    idle();
    EXPECT_EQ(call(Request::RESUME, epoch, "client-a").result, Response::REVOKED);
  }
}

TEST_F(FleetYieldServices, InvalidAndForgottenIdentitiesCannotMove)
{
  EXPECT_EQ(call(255).result, Response::INVALID);
  const auto epoch = call(Request::QUERY).epoch;
  EXPECT_EQ(call(Request::PAUSE, epoch, "").result, Response::INVALID);
  EXPECT_EQ(call(Request::PAUSE, epoch, std::string(129, 'x')).result, Response::INVALID);
  EXPECT_EQ(call(Request::RESUME, epoch, "unknown").result, Response::REVOKED);
  EXPECT_EQ(ctx->current_command, 1);
  const auto first = pause();
  idle();
  ASSERT_EQ(call(Request::RESUME, first, "client-a").result, Response::RESUMED);
  status(2, "MOWING");
  const auto second = pause();
  const auto active_epoch = call(Request::QUERY).epoch;
  EXPECT_EQ(call(Request::RESUME, first, "client-a").result, Response::ALREADY_RESUMED);
  EXPECT_EQ(ctx->current_command, 8);
  EXPECT_TRUE(call(Request::QUERY).held);
  EXPECT_EQ(call(Request::QUERY).epoch, active_epoch);
  idle();
  ASSERT_EQ(call(Request::RESUME, second, "client-a").result, Response::RESUMED);
  EXPECT_EQ(call(Request::RESUME, first, "client-a").result, Response::REVOKED);
}

TEST_F(FleetYieldServices, FleetCallbacksCannotOverlapTheProductionTick)
{
  executor.remove_node(node);
  executor.remove_node(client_node);
  ctx->node.reset();
  node.reset();
  node = std::make_shared<BehaviorTreeNode>();
  std::promise<void> entered, release;
  auto entered_future = entered.get_future();
  auto released = release.get_future().share();
  std::atomic<bool> first{true};
  FleetYieldServiceTestPeer::setup(node,
                                   [&](auto&)
                                   {
                                     if (first.exchange(false))
                                     {
                                       entered.set_value();
                                       released.wait_for(5s);
                                     }
                                     return BT::NodeStatus::SUCCESS;
                                   });
  ASSERT_TRUE(FleetYieldServiceTestPeer::sharedGroup(*node));
  ctx = node->context();
  rclcpp::executors::MultiThreadedExecutor multi(rclcpp::ExecutorOptions(), 3);
  multi.add_node(node);
  multi.add_node(client_node);
  FleetYieldServiceTestPeer::startTimer(*node);
  std::thread worker(
      [&]
      {
        multi.spin();
      });
  EXPECT_EQ(entered_future.wait_for(3s), std::future_status::ready);
  auto response = client->async_send_request(std::make_shared<Request>());
  EXPECT_EQ(response.wait_for(150ms), std::future_status::timeout);
  release.set_value();
  EXPECT_EQ(response.wait_for(3s), std::future_status::ready);
  multi.cancel();
  worker.join();
  FleetYieldServiceTestPeer::stopTimer(*node);
}
}  // namespace mowgli_behavior
