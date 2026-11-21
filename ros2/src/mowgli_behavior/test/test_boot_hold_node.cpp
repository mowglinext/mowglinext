// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// A restarted behaviour tree must not inherit what its dead predecessor left running: blade ON and
// the Nav2 goals it sent (the action server owns them, so they keep driving). The real node runs
// its start-up hold against a fake bridge service and a fake /follow_path server that still holds
// the dead instance's goal.

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#define main boot_hold_behavior_main_for_test
// Include this package's implementation to exercise the real start-up wiring.
#include "../src/behavior_tree_node.cpp"  // NOLINT(build/include)
#undef main

namespace mowgli_behavior
{
struct BootHoldTestPeer
{
  /// Same wiring init() has around the hold; the rest of init() is not needed here.
  static void setup(const std::shared_ptr<BehaviorTreeNode>& node,
                    std::atomic<int>& ticks,
                    std::chrono::steady_clock::time_point* first_tick,
                    std::mutex* mutex)
  {
    node->context_->node = node;
    node->context_->helper_node = rclcpp::Node::make_shared("boot_hold_test_helper");
    node->setupServiceServer();  // tickTree() uses the coverage-orientation service
    node->factory_.registerSimpleAction("TestTick",
                                        [&ticks, first_tick, mutex](BT::TreeNode&)
                                        {
                                          std::lock_guard<std::mutex> lock(*mutex);
                                          if (ticks.load() == 0)
                                          {
                                            *first_tick = std::chrono::steady_clock::now();
                                          }
                                          ++ticks;
                                          return BT::NodeStatus::SUCCESS;
                                        });
    node->tree_ = node->factory_.createTreeFromText(
        "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\"><TestTick/></BehaviorTree></root>");
    node->declare_parameter("tick_rate", 20.0);
    node->startBootHold();
    node->setupTimer();
  }
};
}  // namespace mowgli_behavior

namespace
{
using Mower = mowgli_interfaces::srv::MowerControl;
using Clock = std::chrono::steady_clock;

/// The fixture's node runs on its own thread, so a slow server really is slow while the node under
/// test spins on the test thread. One server per action endpoint the hold sweeps; each holds one
/// goal from a "dead" client and logs when its cancel is processed.
class Endpoints
{
public:
  struct CancelLog
  {
    std::string name;
    Clock::time_point at;
  };

  /// `late_follow_goal`: once /follow_path has processed its first cancel, the dead parent still
  /// dispatches one more controller goal 100 ms later (a parent acknowledges a cancel before it has
  /// stopped). The first sweep is over by then, so only the second sweep can catch it.
  /// `slow_endpoint`: one server whose cancel answer takes `slow_delay` instead of `cancel_delay`
  /// (a server that is discovered but does not answer in time).
  /// `refused_follow_cancels`: /follow_path refuses its first cancel requests (a server that
  /// cannot cancel yet).
  /// `follow_finish_delay`: a cancelled /follow_path goal stays canceling this long before it ends
  /// (a controller that has not noticed the cancel yet).
  Endpoints(const std::vector<std::string>& served,
            bool with_blade,
            std::chrono::milliseconds cancel_delay,
            bool late_follow_goal = false,
            std::string slow_endpoint = "",
            std::chrono::milliseconds slow_delay = 0ms,
            int refused_follow_cancels = 0,
            std::chrono::milliseconds follow_finish_delay = 0ms)
      : peer_(rclcpp::Node::make_shared("boot_hold_fixture")),
        delay_(cancel_delay),
        slow_endpoint_(std::move(slow_endpoint)),
        slow_delay_(slow_delay),
        refused_follow_cancels_(refused_follow_cancels),
        follow_finish_delay_(follow_finish_delay),
        late_follow_goal_(late_follow_goal)
  {
    // Reentrant: a server that sits on its answer must not stall the others.
    group_ = peer_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    if (with_blade)
    {
      blade_ = peer_->create_service<Mower>("/hardware_bridge/mower_control",
                                            [this](const std::shared_ptr<Mower::Request> req,
                                                   std::shared_ptr<Mower::Response> res)
                                            {
                                              std::lock_guard<std::mutex> lock(mutex_);
                                              blade_requests_.push_back(
                                                  {req->mow_enabled, Clock::now()});
                                              res->success = true;
                                            });
    }
    for (const auto& name : served)
    {
      if (name == "/navigate_to_pose")
        add<nav2_msgs::action::NavigateToPose>(name);
      else if (name == "/undock_robot")
        add<nav2_msgs::action::UndockRobot>(name);
      else if (name == "/dock_robot")
        add<nav2_msgs::action::DockRobot>(name);
      else if (name == "/backup" || name == "/retrace")
        add<nav2_msgs::action::BackUp>(name);
      else if (name == "/follow_path")
        add<nav2_msgs::action::FollowPath>(name);
    }
    // The fake servers finish a goal whose cancel was accepted, as Nav2 does shortly after.
    timer_ = peer_->create_wall_timer(20ms,
                                      [this]
                                      {
                                        std::lock_guard<std::mutex> lock(mutex_);
                                        for (auto& finish : finishers_)
                                        {
                                          finish();
                                        }
                                        if (late_due_ && Clock::now() >= *late_due_)
                                        {
                                          late_due_.reset();
                                          if (send_late_)
                                          {
                                            send_late_();
                                          }
                                        }
                                      });
    executor_.add_node(peer_);
    thread_ = std::thread(
        [this]
        {
          executor_.spin();
        });
  }

  ~Endpoints()
  {
    executor_.cancel();
    thread_.join();
  }

  /// Waits until every served endpoint holds its dead client's goal.
  bool allGoalsHeld(std::chrono::milliseconds limit = 8s)
  {
    const auto end = Clock::now() + limit;
    while (Clock::now() < end)
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (held_ == expected_)
        {
          return true;
        }
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  std::vector<CancelLog> cancels() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return cancels_;
  }
  /// When each cancelled goal reached its terminal state.
  std::vector<CancelLog> terminals() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return terminals_;
  }
  std::vector<std::pair<bool, Clock::time_point>> bladeRequests() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return blade_requests_;
  }

private:
  template <typename ActionT>
  void add(const std::string& name)
  {
    using Handle = rclcpp_action::ServerGoalHandle<ActionT>;
    ++expected_;
    auto server = rclcpp_action::create_server<ActionT>(
        peer_,
        name,
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const typename ActionT::Goal>)
        {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this, name](const std::shared_ptr<Handle>)
        {
          // a busy server: the answer comes late
          std::this_thread::sleep_for(name == slow_endpoint_ ? slow_delay_ : delay_);
          std::lock_guard<std::mutex> lock(mutex_);
          cancels_.push_back({name, Clock::now()});
          if (name == "/follow_path" && refused_follow_cancels_ > 0)
          {
            --refused_follow_cancels_;
            return rclcpp_action::CancelResponse::REJECT;
          }
          if (late_follow_goal_ && name == "/follow_path" && !late_sent_)
          {
            late_sent_ = true;
            late_due_ = Clock::now() + 100ms;
          }
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this, name](const std::shared_ptr<Handle> goal)
        {
          std::lock_guard<std::mutex> lock(mutex_);
          goals_.push_back(goal);  // held running: the dead instance's goal
          ++held_;
          finishers_.push_back(
              [this, goal, name, seen = std::optional<Clock::time_point>{}]() mutable
              {
                if (!goal->is_canceling())
                {
                  return;
                }
                if (!seen)
                {
                  seen = Clock::now();
                }
                if (name == "/follow_path" && Clock::now() - *seen < follow_finish_delay_)
                {
                  return;
                }
                goal->canceled(std::make_shared<typename ActionT::Result>());
                terminals_.push_back({name, Clock::now()});
              });
        },
        rcl_action_server_get_default_options(),
        group_);
    auto client = rclcpp_action::create_client<ActionT>(peer_, name);
    servers_.push_back(server);
    clients_.push_back(client);
    sender_.push_back(
        [client]
        {
          client->wait_for_action_server(5s);
          client->async_send_goal(typename ActionT::Goal{});
        });
    if constexpr (std::is_same_v<ActionT, nav2_msgs::action::FollowPath>)
    {
      send_late_ = [client]
      {
        client->async_send_goal(typename ActionT::Goal{});
      };
    }
  }

public:
  /// Sends the dead instance's goals (call before the node under test starts).
  void sendDeadGoals()
  {
    for (auto& send : sender_)
    {
      send();
    }
  }

private:
  rclcpp::Node::SharedPtr peer_;
  std::chrono::milliseconds delay_;
  std::string slow_endpoint_;
  std::chrono::milliseconds slow_delay_;
  int refused_follow_cancels_{0};
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions(), 4};
  std::thread thread_;
  mutable std::mutex mutex_;
  std::vector<CancelLog> cancels_;
  std::vector<CancelLog> terminals_;
  std::chrono::milliseconds follow_finish_delay_{0};
  std::vector<std::pair<bool, Clock::time_point>> blade_requests_;
  std::vector<std::shared_ptr<void>> goals_;
  std::vector<std::shared_ptr<void>> servers_;
  std::vector<std::shared_ptr<void>> clients_;
  std::vector<std::function<void()>> sender_;
  std::vector<std::function<void()>> finishers_;
  std::function<void()> send_late_;
  std::optional<Clock::time_point> late_due_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool late_follow_goal_{false};
  bool late_sent_{false};
  rclcpp::Service<Mower>::SharedPtr blade_;
  std::size_t held_{0};
  std::size_t expected_{0};
};

const std::vector<std::string> kAllEndpoints{
    "/navigate_to_pose", "/undock_robot", "/dock_robot", "/backup", "/retrace", "/follow_path"};

class BootHoldNode : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  /// Spins the node under test until `done` or the limit.
  static bool spinUntil(rclcpp::executors::SingleThreadedExecutor& executor,
                        const std::function<bool()>& done,
                        std::chrono::milliseconds limit = 10s)
  {
    const auto end = Clock::now() + limit;
    while (Clock::now() < end)
    {
      executor.spin_some(20ms);
      if (done())
      {
        return true;
      }
    }
    return done();
  }

  struct Probe
  {
    std::atomic<int> ticks{0};
    std::mutex mutex;
    Clock::time_point first_tick{};
  };
  static std::shared_ptr<mowgli_behavior::BehaviorTreeNode> makeNode(
      Probe& probe, std::vector<rclcpp::Parameter> overrides = {})
  {
    auto node = std::make_shared<mowgli_behavior::BehaviorTreeNode>(
        rclcpp::NodeOptions{}.parameter_overrides(overrides));
    mowgli_behavior::BootHoldTestPeer::setup(node, probe.ticks, &probe.first_tick, &probe.mutex);
    return node;
  }
};

TEST_F(BootHoldNode, WaitsForEveryServerToAnswerItsCancelBeforeTheFirstTick)
{
  Endpoints fx(kAllEndpoints,
               true,
               150ms,
               /*late_follow_goal=*/true,
               "",
               0ms,
               0,
               /*follow_finish_delay=*/800ms);
  fx.sendDeadGoals();
  ASSERT_TRUE(fx.allGoalsHeld());

  Probe probe;
  auto node = makeNode(probe);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(spinUntil(executor,
                        [&]
                        {
                          return probe.ticks.load() > 0;
                        }));
  node->context()->node.reset();

  const auto cancels = fx.cancels();
  const auto blades = fx.bladeRequests();
  ASSERT_EQ(blades.size(), 1u);
  EXPECT_FALSE(blades[0].first) << "blade OFF";
  EXPECT_LT(blades[0].second, probe.first_tick);

  // First sweep: the five parents, then the controller goal, each answered (the servers take
  // 150 ms) before the tree ticks: a goal the new tree sends first must never meet a cancel-all
  // that is still queued.
  ASSERT_EQ(cancels.size(), 7u);
  for (const auto& c : cancels)
  {
    EXPECT_LT(c.at, probe.first_tick) << c.name << " cancel processed after the first tick";
  }
  for (std::size_t i = 0; i < 5; ++i)
  {
    EXPECT_NE(cancels[i].name, "/follow_path");
    EXPECT_LE(cancels[i].at, cancels[5].at);
  }
  EXPECT_EQ(cancels[5].name, "/follow_path");
  // The dead parent dispatched one more controller goal after the first controller sweep was over:
  // only the second sweep catches it.
  EXPECT_EQ(cancels[6].name, "/follow_path");
  EXPECT_GT(cancels[6].at, cancels[5].at);

  // An answered cancel leaves a goal canceling; the controller takes another 800 ms to end it. The
  // tree must not tick while any goal is still alive.
  const auto terminals = fx.terminals();
  ASSERT_EQ(terminals.size(), 7u);
  for (const auto& t : terminals)
  {
    EXPECT_LT(t.at, probe.first_tick) << t.name << " goal still active at the first tick";
  }
}

TEST_F(BootHoldNode, ARefusedCancelIsAskedAgainBeforeTheFirstTick)
{
  Endpoints fx(kAllEndpoints,
               true,
               0ms,
               /*late_follow_goal=*/false,
               "",
               0ms,
               /*refused_follow_cancels=*/2);
  fx.sendDeadGoals();
  ASSERT_TRUE(fx.allGoalsHeld());

  Probe probe;
  auto node = makeNode(probe);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(spinUntil(executor,
                        [&]
                        {
                          return probe.ticks.load() > 0;
                        }));
  node->context()->node.reset();

  std::vector<Endpoints::CancelLog> follow;
  for (const auto& c : fx.cancels())
  {
    if (c.name == "/follow_path")
    {
      follow.push_back(c);
    }
  }
  // Refused twice, accepted on the third ask. Counting a refusal as an answer would leave the
  // goal running when the tree ticks (the second sweep is refused as well).
  ASSERT_EQ(follow.size(), 3u);
  EXPECT_LT(follow[2].at, probe.first_tick);
}

TEST_F(BootHoldNode, GivingUpStillAsksTheControllerToCancel)
{
  // The parent is discovered but sits on its answer past the hold's deadline, so the controller
  // sweep (gated behind it) never runs inside the hold. At the deadline the node cancels anyway.
  Endpoints fx(kAllEndpoints,
               true,
               0ms,
               /*late_follow_goal=*/false,
               "/navigate_to_pose",
               3s);
  fx.sendDeadGoals();
  ASSERT_TRUE(fx.allGoalsHeld());

  Probe probe;
  auto node = makeNode(probe, {rclcpp::Parameter("boot_hold_timeout_s", 1.5)});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto t0 = Clock::now();
  ASSERT_TRUE(spinUntil(executor,
                        [&]
                        {
                          return probe.ticks.load() > 0;
                        }));
  node->context()->node.reset();
  std::this_thread::sleep_for(300ms);  // the give-up cancel is fire and forget

  EXPECT_LT(probe.first_tick - t0, 2500ms) << "the hold gave up at its deadline";
  bool follow_cancelled_before_parent_answered = false;
  Clock::time_point parent_answered = Clock::time_point::max();
  for (const auto& c : fx.cancels())
  {
    if (c.name == "/navigate_to_pose")
    {
      parent_answered = c.at;
    }
  }
  for (const auto& c : fx.cancels())
  {
    if (c.name == "/follow_path" && c.at < parent_answered)
    {
      follow_cancelled_before_parent_answered = true;
    }
  }
  EXPECT_TRUE(follow_cancelled_before_parent_answered)
      << "/follow_path was cancelled by the give-up path, not by the gated sweep";
}

TEST_F(BootHoldNode, ServersThatAreNotRunningHoldNoGoalsAndAreSkippedAfterTheGrace)
{
  Endpoints fx({"/follow_path"}, true, 0ms);  // no Nav2 parents: an orphan controller goal
  fx.sendDeadGoals();
  ASSERT_TRUE(fx.allGoalsHeld());

  Probe probe;
  auto node = makeNode(probe);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto t0 = Clock::now();
  ASSERT_TRUE(spinUntil(executor,
                        [&]
                        {
                          return probe.ticks.load() > 0;
                        }));
  node->context()->node.reset();

  EXPECT_GE(probe.first_tick - t0, 900ms) << "absent parents count as running until the grace";
  const auto cancels = fx.cancels();
  ASSERT_EQ(cancels.size(), 1u) << "the orphan controller goal is still cancelled";
  EXPECT_EQ(cancels[0].name, "/follow_path");
  EXPECT_LT(cancels[0].at, probe.first_tick);
}

TEST_F(BootHoldNode, ColdBootWithNothingRunningStartsTickingAfterTheGrace)
{
  Endpoints fx({}, false, 0ms);
  Probe probe;
  auto node = makeNode(probe);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto t0 = Clock::now();
  ASSERT_TRUE(spinUntil(executor,
                        [&]
                        {
                          return probe.ticks.load() > 0;
                        }));
  node->context()->node.reset();
  EXPECT_GE(probe.first_tick - t0, 900ms) << "nothing to clean up, but the graph needs a moment";
  EXPECT_LE(probe.first_tick - t0, 3500ms);
}

TEST_F(BootHoldNode, ZeroTimeoutDisablesTheHoldAndTicksAtOnce)
{
  Endpoints fx(kAllEndpoints, true, 0ms);
  fx.sendDeadGoals();
  ASSERT_TRUE(fx.allGoalsHeld());
  Probe probe;
  auto node = makeNode(probe, {rclcpp::Parameter("boot_hold_timeout_s", 0.0)});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(spinUntil(
      executor,
      [&]
      {
        return probe.ticks.load() > 0;
      },
      3s));
  node->context()->node.reset();
  EXPECT_TRUE(fx.cancels().empty());
  EXPECT_TRUE(fx.bladeRequests().empty());
}
}  // namespace
