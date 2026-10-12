// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "mowgli_monitoring/mqtt_bridge_node.hpp"
#include <gtest/gtest.h>

namespace mowgli_monitoring
{
namespace
{
using AreaService = mowgli_interfaces::srv::GetMowingArea;

class MapMqttClient : public IMqttClient
{
public:
  bool connect() noexcept override
  {
    connected = true;
    return true;
  }
  void disconnect() noexcept override
  {
    connected = false;
  }
  bool is_connected() const noexcept override
  {
    return connected;
  }
  bool subscribe(const std::string&, MessageCallback) noexcept override
  {
    return true;
  }
  void spin_once() noexcept override
  {
  }
  bool publish(const std::string& topic,
               const std::string& payload,
               bool retained) noexcept override
  {
    if (topic == observed_topic)
    {
      EXPECT_TRUE(retained);
      accepted.push_back(payload);
    }
    return connected;
  }
  bool connected{true};
  std::string observed_topic;
  std::vector<std::string> accepted;
};
}  // namespace

class MqttMapPollTest : public ::testing::TestWithParam<std::string>
{
protected:
  struct Request
  {
    std::shared_ptr<rmw_request_id_t> header;
    uint32_t index;
  };
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
    auto client = std::make_unique<MapMqttClient>();
    mqtt = client.get();
    mqtt->observed_topic = "mowgli/" + GetParam();
    rclcpp::NodeOptions options;
    // Timers are driven explicitly; no guessed wall-clock sleeps or ROS clock
    // advance is needed to exercise the real reconnect/poll timer path.
    options.parameter_overrides({rclcpp::Parameter("publish_rate", 0.01)});
    node = std::make_shared<MqttBridgeNode>(std::move(client), options);
    node->map_poll_now_ = [this]
    {
      return time;
    };
    server_node = std::make_shared<rclcpp::Node>("controlled_map_service");
    server = server_node->create_service<AreaService>(
        "/map_server_node/get_mowing_area",
        [this](std::shared_ptr<rmw_request_id_t> header,
               std::shared_ptr<AreaService::Request> request)
        {
          requests.push_back({header, request->index});
        });
    executor.add_node(node);
    executor.add_node(server_node);
    ASSERT_TRUE(pump(
        [this]
        {
          return node->srv_get_area_->service_is_ready() &&
                 node->srv_get_mowing_area_->service_is_ready();
        }));
    node->last_area_poll_ = node->now();
    node->last_areas_poll_ = node->now();
  }
  void TearDown() override
  {
    executor.remove_node(node);
    executor.remove_node(server_node);
  }
  bool pump(const std::function<bool()>& ready)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!ready() && std::chrono::steady_clock::now() < deadline)
      executor.spin_some(std::chrono::milliseconds(1));
    return ready();
  }
  bool active() const
  {
    return GetParam() == "areas" ? node->areas_poll_in_flight_ : node->area_poll_in_progress_;
  }
  MqttBridgeNode::MapPollState& state()
  {
    return GetParam() == "areas" ? node->areas_poll_ : node->boundary_poll_;
  }
  MqttBridgeNode::AreaClient::SharedPtr client()
  {
    return GetParam() == "areas" ? node->srv_get_area_ : node->srv_get_mowing_area_;
  }
  void tick(bool due = false)
  {
    if (due)
    {
      auto& last = GetParam() == "areas" ? node->last_areas_poll_ : node->last_area_poll_;
      last = node->now() - rclcpp::Duration::from_seconds(11.0);
    }
    node->on_timer();
  }
  void reply(const Request& request, bool success, const std::string& name = "")
  {
    AreaService::Response response;
    response.success = success;
    response.area.name = name;
    geometry_msgs::msg::Point32 point;
    point.x = name == "map B" ? 2.0f : 1.0f;
    response.area.area.points.push_back(point);
    server->send_response(*request.header, response);
  }
  void completed_map(const std::string& name)
  {
    const auto start = requests.size();
    tick(true);
    ASSERT_TRUE(pump(
        [&]
        {
          return requests.size() == start + 1;
        }));
    ASSERT_EQ(requests.back().index, 0U);
    reply(requests.back(), true, name);
    ASSERT_TRUE(pump(
        [&]
        {
          return requests.size() == start + 2;
        }));
    ASSERT_EQ(requests.back().index, 1U);
    reply(requests.back(), false);
    ASSERT_TRUE(pump(
        [&]
        {
          return !active();
        }));
  }
  void obsolete_callback(uint64_t generation)
  {
    std::promise<std::shared_ptr<AreaService::Response>> promise;
    promise.set_exception(std::make_exception_ptr(std::runtime_error("obsolete future consumed")));
    auto future = promise.get_future().share();
    if (GetParam() == "areas")
      node->on_areas_response(1,
                              generation,
                              std::make_shared<std::vector<MqttBridgeNode::AreaSummary>>(),
                              future);
    else
      node->on_area_boundary_response(
          1,
          generation,
          std::make_shared<std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>>>(),
          future);
  }
  void advance()
  {
    time += MqttBridgeNode::kMapPollTimeout + std::chrono::milliseconds(1);
  }
  void start_other()
  {
    if (GetParam() == "areas")
    {
      node->last_area_poll_ = node->now() - rclcpp::Duration::from_seconds(11.0);
      node->maybe_poll_area_boundaries();
    }
    else
      node->poll_areas();
  }
  bool other_active() const
  {
    return GetParam() == "areas" ? node->area_poll_in_progress_ : node->areas_poll_in_flight_;
  }

  std::chrono::steady_clock::time_point time{};
  MapMqttClient* mqtt;
  std::shared_ptr<MqttBridgeNode> node;
  rclcpp::Node::SharedPtr server_node;
  rclcpp::Service<AreaService>::SharedPtr server;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::vector<Request> requests;
};

TEST_P(MqttMapPollTest, WithheldResponseExpiresAndLateResponseCannotFinishReplacement)
{
  completed_map("map A");
  ASSERT_EQ(mqtt->accepted.size(), 1U);
  const auto old_snapshot = mqtt->accepted.back();
  tick(true);
  ASSERT_TRUE(pump(
      [&]
      {
        return requests.size() == 3;
      }));
  reply(requests.back(), true, "incomplete map");
  ASSERT_TRUE(pump(
      [&]
      {
        return requests.size() == 4;
      }));
  const auto old_request = requests.back();
  const auto generation = state().generation;
  const auto request_id = *state().request_id;
  advance();
  tick();  // ROS time has not moved, but the monotonic deadline expires.
  ASSERT_FALSE(active());
  EXPECT_FALSE(client()->remove_pending_request(request_id));  // already removed
  ASSERT_EQ(mqtt->accepted.size(), 1U);
  EXPECT_EQ(mqtt->accepted.back(), old_snapshot);

  tick(true);
  ASSERT_TRUE(pump(
      [&]
      {
        return requests.size() == 5;
      }));
  ASSERT_TRUE(active());
  const auto replacement_id = state().request_id;
  const auto replacement_generation = state().generation;
  EXPECT_NO_THROW(obsolete_callback(generation));
  EXPECT_TRUE(active());
  EXPECT_EQ(state().request_id, replacement_id);
  EXPECT_EQ(state().generation, replacement_generation);
  reply(old_request, false);  // real obsolete server response, after replacement starts
  reply(requests.back(), true, "map B");
  ASSERT_TRUE(pump(
      [&]
      {
        return requests.size() == 6;
      }));
  reply(requests.back(), false);
  ASSERT_TRUE(pump(
      [&]
      {
        return !active();
      }));
  ASSERT_EQ(mqtt->accepted.size(), 2U);
  EXPECT_NE(mqtt->accepted.back(), old_snapshot);
  EXPECT_NE(mqtt->accepted.back().find("map B"), std::string::npos);
}

TEST_P(MqttMapPollTest, DisconnectedMqttStillReleasesTimedOutRequest)
{
  tick(true);
  ASSERT_TRUE(pump(
      [&]
      {
        return requests.size() == 1;
      }));
  const auto request_id = *state().request_id;
  mqtt->disconnect();
  advance();
  tick();
  EXPECT_FALSE(active());
  EXPECT_FALSE(client()->remove_pending_request(request_id));
}

TEST_P(MqttMapPollTest, CallbackAfterDeadlineCannotPublishPartialMap)
{
  completed_map("map A");
  ASSERT_EQ(mqtt->accepted.size(), 1U);
  tick(true);
  ASSERT_TRUE(pump(
      [&]
      {
        return requests.size() == 3;
      }));
  advance();
  reply(requests.back(), false);  // callback races the next timer tick
  ASSERT_TRUE(pump(
      [&]
      {
        return !active();
      }));
  EXPECT_EQ(mqtt->accepted.size(), 1U);
}

TEST_P(MqttMapPollTest, ExpiringOneChainDoesNotAbandonTheOther)
{
  tick(true);
  ASSERT_TRUE(pump(
      [&]
      {
        return requests.size() == 1;
      }));
  time += std::chrono::seconds(4);
  start_other();
  ASSERT_TRUE(pump(
      [&]
      {
        return requests.size() == 2;
      }));
  time += std::chrono::seconds(2);
  tick();
  EXPECT_FALSE(active());
  ASSERT_TRUE(other_active());
  reply(requests.back(), false);
  ASSERT_TRUE(pump(
      [&]
      {
        return !other_active();
      }));
  EXPECT_TRUE(mqtt->accepted.empty());
}

INSTANTIATE_TEST_SUITE_P(MapChains, MqttMapPollTest, ::testing::Values("areas", "area_boundary"));
}  // namespace mowgli_monitoring
