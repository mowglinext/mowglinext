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
#include <utility>
#include <vector>

#include <nav2_msgs/action/back_up.hpp>

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
    node->precision_support_ = std::make_shared<PrecisionCoverageSupport>();
    node->setupSubscribers();
    node->boot_cleanup_required_ = true;
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
using HardwareStatus = mowgli_interfaces::msg::Status;
using HighLevelStatus = mowgli_interfaces::msg::HighLevelStatus;

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
  /// `follow_finish_delay`: the time a /follow_path goal remains CANCELING before it ends.
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
                                              mow_enabled_ = req->mow_enabled;
                                              res->success = true;
                                            });
    }
    if (with_blade)
    {
      hardware_status_ = peer_->create_publisher<HardwareStatus>("/hardware_bridge/status", 10);
    }
    boot_status_ = peer_->create_subscription<HighLevelStatus>(
        "/mowgli_behavior_node/high_level_status",
        10,
        [this](HighLevelStatus::ConstSharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (msg->state == HighLevelStatus::HIGH_LEVEL_STATE_IDLE &&
              msg->state_name == "BOOT_HOLD" && !boot_idle_at_)
          {
            boot_idle_at_ = Clock::now();
          }
        });
    for (const auto& name : served)
    {
      // Cancellation and status use the same action protocol for every server.
      add<nav2_msgs::action::BackUp>(name);
    }
    // The fake servers finish a goal whose cancel was accepted, as Nav2 does shortly after.
    timer_ = peer_->create_wall_timer(20ms,
                                      [this]
                                      {
                                        std::lock_guard<std::mutex> lock(mutex_);
                                        if (hardware_status_)
                                        {
                                          HardwareStatus report;
                                          report.mow_enabled = mow_enabled_;
                                          hardware_status_->publish(report);
                                        }
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
  std::vector<CancelLog> finished() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return finished_;
  }

  std::optional<Clock::time_point> bootIdleAt() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return boot_idle_at_;
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
              [this, goal, name, canceling_since = std::optional<Clock::time_point>{}]() mutable
              {
                if (!goal->is_canceling())
                {
                  return;
                }
                const auto now = Clock::now();
                if (!canceling_since)
                {
                  canceling_since = now;
                }
                if (name != "/follow_path" || now - *canceling_since >= follow_finish_delay_)
                {
                  goal->canceled(std::make_shared<typename ActionT::Result>());
                  finished_.push_back({name, Clock::now()});
                }
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
    if (name == "/follow_path")
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
  std::chrono::milliseconds follow_finish_delay_{};
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions(), 4};
  std::thread thread_;
  mutable std::mutex mutex_;
  std::vector<CancelLog> cancels_;
  std::vector<CancelLog> finished_;
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
  rclcpp::Publisher<HardwareStatus>::SharedPtr hardware_status_;
  rclcpp::Subscription<HighLevelStatus>::SharedPtr boot_status_;
  std::optional<Clock::time_point> boot_idle_at_;
  bool mow_enabled_{true};
  std::size_t held_{0};
  std::size_t expected_{0};
};

const std::vector<std::string> kAllEndpoints{"/follow_waypoints",
                                             "/follow_gps_waypoints",
                                             "/dock_robot",
                                             "/undock_robot",
                                             "/navigate_to_pose",
                                             "/navigate_through_poses",
                                             "/follow_path",
                                             "/backup",
                                             "/retrace",
                                             "/spin",
                                             "/drive_on_heading",
                                             "/assisted_teleop"};

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

TEST_F(BootHoldNode, PublishesIdleAndWaitsForTerminalGoalsBeforeTheFirstTick)
{
  Endpoints fx(kAllEndpoints, true, 150ms, true, "", 0ms, 0, 1s);
  fx.sendDeadGoals();
  ASSERT_TRUE(fx.allGoalsHeld());

  Probe probe;
  auto node = makeNode(probe);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  ASSERT_TRUE(spinUntil(executor,
                        [&]()
                        {
                          return probe.ticks.load() > 0;
                        }));
  node->context()->node.reset();

  const auto cancels = fx.cancels();
  const auto completed = fx.finished();
  const auto blades = fx.bladeRequests();
  ASSERT_EQ(blades.size(), 1u);
  EXPECT_FALSE(blades[0].first);
  EXPECT_LT(blades[0].second, probe.first_tick);
  ASSERT_TRUE(fx.bootIdleAt());
  EXPECT_LT(*fx.bootIdleAt(), probe.first_tick);

  ASSERT_EQ(completed.size(), kAllEndpoints.size() + 1u);  // Includes the late child goal.
  for (const auto& goal : completed)
  {
    EXPECT_LT(goal.at, probe.first_tick) << goal.name;
  }
  const auto nav = std::find_if(completed.begin(),
                                completed.end(),
                                [](const auto& goal)
                                {
                                  return goal.name == "/navigate_to_pose";
                                });
  ASSERT_NE(nav, completed.end());
  const auto retrace = std::find_if(cancels.begin(),
                                    cancels.end(),
                                    [](const auto& cancel)
                                    {
                                      return cancel.name == "/retrace";
                                    });
  ASSERT_NE(retrace, cancels.end());
  EXPECT_LT(nav->at, retrace->at);
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

TEST_F(BootHoldNode, TimeoutKeepsIdleUntilTheUnresponsiveServerFinishes)
{
  Endpoints fx(kAllEndpoints, true, 0ms, false, "/navigate_to_pose", 1500ms);
  fx.sendDeadGoals();
  ASSERT_TRUE(fx.allGoalsHeld());

  Probe probe;
  auto node = makeNode(probe, {rclcpp::Parameter("boot_hold_timeout_s", 0.3)});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  EXPECT_FALSE(spinUntil(
      executor,
      [&]()
      {
        return probe.ticks.load() > 0;
      },
      800ms));
  ASSERT_TRUE(fx.bootIdleAt());
  ASSERT_TRUE(spinUntil(executor,
                        [&]()
                        {
                          return probe.ticks.load() > 0;
                        }));
  node->context()->node.reset();
  ASSERT_EQ(fx.finished().size(), kAllEndpoints.size());
}

TEST_F(BootHoldNode, AServerThatHasNotBeenDiscoveredCannotReleaseTheHold)
{
  Probe probe;
  auto node = makeNode(probe, {rclcpp::Parameter("boot_hold_timeout_s", 0.3)});
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  EXPECT_FALSE(spinUntil(
      executor,
      [&]()
      {
        return probe.ticks.load() > 0;
      },
      1300ms));

  // Once the bridge and all action servers exist, an empty cancel response
  // and settled status subscription let a server with no goals finish.
  Endpoints fx(kAllEndpoints, true, 0ms);
  ASSERT_TRUE(spinUntil(executor,
                        [&]()
                        {
                          return probe.ticks.load() > 0;
                        }));
  node->context()->node.reset();
  EXPECT_TRUE(fx.cancels().empty());
}
}  // namespace
