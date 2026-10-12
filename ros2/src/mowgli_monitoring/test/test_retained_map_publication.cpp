// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#include <memory>
#include <string>
#include <vector>

#include "mowgli_monitoring/mqtt_bridge_node.hpp"
#include <gtest/gtest.h>

namespace mowgli_monitoring
{
namespace
{
class FallibleMqttClient : public IMqttClient
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
  bool publish(const std::string& topic, const std::string& payload, bool retain) noexcept override
  {
    if (!connected || fail_publication || topic == fail_topic)
      return false;
    if (topic == observed_topic)
    {
      EXPECT_TRUE(retain);
      accepted.push_back(payload);
    }
    if (topic == "homeassistant/device/mowglinext_mowgli/config")
    {
      latest_discovery = payload;
    }
    return true;
  }
  bool connected{true};
  bool fail_publication{false};
  std::string observed_topic;
  std::string fail_topic;
  std::string latest_discovery;
  std::vector<std::string> accepted;
};
}  // namespace

// Calls the real subscription/poll completion callbacks and reconnect timer.
// The MQTT transport alone is injected; no broker, robot or elapsed sleeps.
class RetainedMapPublicationTest : public ::testing::TestWithParam<std::string>
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
  void SetUp() override
  {
    make_node(false);
  }
  void make_node(bool discovery)
  {
    auto client = std::make_unique<FallibleMqttClient>();
    mqtt = client.get();
    mqtt->observed_topic = "mowgli/" + GetParam();
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("home_assistant_discovery_enabled", discovery)});
    node = std::make_shared<MqttBridgeNode>(std::move(client), options);
  }
  void timer()
  {
    node->on_timer();
  }
  void observation(float value)
  {
    if (GetParam() == "coverage_path")
    {
      auto path = std::make_shared<nav_msgs::msg::Path>();
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = value;
      path->poses.push_back(pose);
      node->on_coverage_path(path);
    }
    else if (GetParam() == "areas")
    {
      node->publish_areas_if_changed({{0, "area " + std::to_string(value)}});
    }
    else
    {
      using Boundaries = std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>>;
      auto boundaries = std::make_shared<Boundaries>();
      mowgli_interfaces::msg::MapArea area;
      area.name = "front";
      geometry_msgs::msg::Point32 point;
      point.x = value;
      area.area.points.push_back(point);
      boundaries->emplace_back(0, area);
      node->finish_area_boundary_poll(boundaries);
    }
  }
  FallibleMqttClient* mqtt;
  std::shared_ptr<MqttBridgeNode> node;
};

TEST_P(RetainedMapPublicationTest, RetriesOnReconnectWithoutAnotherObservation)
{
  mqtt->disconnect();
  observation(1.0f);
  ASSERT_TRUE(mqtt->accepted.empty());
  timer();  // normal reconnect attempt
  ASSERT_TRUE(mqtt->connected);
  timer();  // connection edge, no new observation
  ASSERT_EQ(mqtt->accepted.size(), 1U);
  const auto payload = mqtt->accepted.front();
  observation(1.0f);
  timer();
  EXPECT_EQ(mqtt->accepted.size(), 1U);
  EXPECT_EQ(mqtt->accepted.front(), payload);
}

TEST_P(RetainedMapPublicationTest, RetriesFailedPublishWhileStillConnected)
{
  mqtt->fail_publication = true;
  observation(2.0f);
  timer();
  ASSERT_TRUE(mqtt->accepted.empty());
  mqtt->fail_publication = false;
  timer();
  ASSERT_EQ(mqtt->accepted.size(), 1U);
  observation(2.0f);
  EXPECT_EQ(mqtt->accepted.size(), 1U);
}

TEST_P(RetainedMapPublicationTest, LatestObservationReplacesFailedPayload)
{
  observation(1.0f);
  ASSERT_EQ(mqtt->accepted.size(), 1U);
  const auto first = mqtt->accepted.front();
  mqtt->fail_publication = true;
  observation(2.0f);
  observation(3.0f);
  ASSERT_EQ(mqtt->accepted.size(), 1U);
  mqtt->fail_publication = false;
  timer();
  ASSERT_EQ(mqtt->accepted.size(), 2U);
  EXPECT_NE(mqtt->accepted.back(), first);
  const auto latest = mqtt->accepted.back();
  observation(3.0f);
  timer();
  ASSERT_EQ(mqtt->accepted.size(), 2U);
  EXPECT_EQ(mqtt->accepted.back(), latest);
}

TEST_P(RetainedMapPublicationTest, ReturningToAcceptedPayloadDropsObsoleteFailure)
{
  observation(1.0f);
  ASSERT_EQ(mqtt->accepted.size(), 1U);
  mqtt->fail_publication = true;
  observation(2.0f);
  observation(1.0f);
  mqtt->fail_publication = false;
  timer();
  ASSERT_EQ(mqtt->accepted.size(), 1U);
}

class DesiredAreaDiscoveryTest : public RetainedMapPublicationTest
{
};

TEST_P(DesiredAreaDiscoveryTest, DiscoveryFollowsDesiredAreasWhenTheirPublishFails)
{
  make_node(true);
  observation(1.0f);
  ASSERT_EQ(mqtt->accepted.size(), 1U);
  const auto discovery_a = mqtt->latest_discovery;
  ASSERT_NE(discovery_a.find("Mow area 1.000000"), std::string::npos);
  mqtt->fail_topic = "mowgli/areas";
  observation(2.0f);
  ASSERT_EQ(mqtt->accepted.size(), 1U);
  ASSERT_NE(mqtt->latest_discovery, discovery_a);
  observation(1.0f);
  EXPECT_EQ(mqtt->latest_discovery, discovery_a);
  mqtt->fail_topic.clear();
  timer();
  EXPECT_EQ(mqtt->accepted.size(), 1U);
  EXPECT_EQ(mqtt->latest_discovery, discovery_a);
}

INSTANTIATE_TEST_SUITE_P(MapTopics,
                         RetainedMapPublicationTest,
                         ::testing::Values("coverage_path", "areas", "area_boundary"));
INSTANTIATE_TEST_SUITE_P(AreaTopic, DesiredAreaDiscoveryTest, ::testing::Values("areas"));
}  // namespace mowgli_monitoring
