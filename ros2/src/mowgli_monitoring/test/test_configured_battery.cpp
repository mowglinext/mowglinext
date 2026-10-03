// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "mowgli_monitoring/battery_percentage.hpp"
#include "mowgli_monitoring/diagnostics_node.hpp"
#include "mowgli_monitoring/mqtt_bridge_node.hpp"
#include <gtest/gtest.h>

namespace
{

TEST(BatteryPercentage, InvalidRangesAndUnknownVoltageDoNotProduceFullBattery)
{
  EXPECT_EQ(mowgli_monitoring::battery_percentage(26.0, 26.0, 26.0), 0.0);
  EXPECT_EQ(mowgli_monitoring::battery_percentage(26.0, 28.0, 24.0), 0.0);
  EXPECT_EQ(mowgli_monitoring::battery_percentage(std::numeric_limits<double>::quiet_NaN(),
                                                  24.0,
                                                  28.0),
            0.0);
}

class PowerMqttClient : public mowgli_monitoring::IMqttClient
{
public:
  bool connect() noexcept override
  {
    return true;
  }
  void disconnect() noexcept override
  {
  }
  bool publish(const std::string& topic,
               const std::string& payload,
               bool retained) noexcept override
  {
    if (topic == "battery_test/power")
    {
      power = payload;
      power_retained = retained;
    }
    return true;
  }
  bool subscribe(const std::string&, MessageCallback) noexcept override
  {
    return true;
  }
  void spin_once() noexcept override
  {
  }
  bool is_connected() const noexcept override
  {
    return true;
  }
  std::string power;
  bool power_retained{false};
};

struct Pack
{
  double empty;
  double full;
};

class ConfiguredBatteryTest : public ::testing::TestWithParam<Pack>
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
};

TEST_P(ConfiguredBatteryTest, RealConsumersUseConfiguredEndpoints)
{
  const auto pack = GetParam();
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  options.parameter_overrides({rclcpp::Parameter("battery_empty_voltage", pack.empty),
                               rclcpp::Parameter("battery_full_voltage", pack.full),
                               rclcpp::Parameter("mqtt_topic_prefix", "battery_test"),
                               rclcpp::Parameter("publish_rate", 100.0)});
  auto diagnostics = std::make_shared<mowgli_monitoring::DiagnosticsNode>(options);
  auto client = std::make_unique<PowerMqttClient>();
  auto* recording = client.get();
  auto mqtt = std::make_shared<mowgli_monitoring::MqttBridgeNode>(std::move(client), options);
  auto publisher = std::make_shared<rclcpp::Node>("battery_test_source", options);
  auto power =
      publisher->create_publisher<mowgli_interfaces::msg::Power>("/hardware_bridge/power", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(diagnostics);
  executor.add_node(mqtt);
  executor.add_node(publisher);

  for (const double fraction : {-0.25, 0.0, 0.05, 0.15, 0.5, 1.0, 1.25})
  {
    recording->power.clear();
    mowgli_interfaces::msg::Power msg;
    msg.v_battery = static_cast<float>(pack.empty + fraction * (pack.full - pack.empty));
    power->publish(msg);
    // Await an observed publication rather than sleeping for a guessed timer
    // interval. Intra-process subscriptions make delivery independent of DDS
    // discovery; this deadline only bounds a broken test.
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (recording->power.empty() && std::chrono::steady_clock::now() < deadline)
    {
      executor.spin_some(std::chrono::milliseconds(1));
    }
    ASSERT_FALSE(recording->power.empty()) << "No power publication for fraction " << fraction;
    ASSERT_TRUE(diagnostics->state().last_power.has_value());
    const double expected = 100.0 * std::max(0.0, std::min(1.0, fraction));
    const auto status = diagnostics->check_battery();
    bool found_percentage = false;
    for (const auto& value : status.values)
    {
      if (value.key == "percentage")
      {
        found_percentage = true;
        EXPECT_NEAR(std::stod(value.value), expected, 0.1);
      }
    }
    EXPECT_TRUE(found_percentage);
    EXPECT_EQ(status.level, expected < 10.0 ? 2 : expected < 20.0 ? 1 : 0);
    char expected_json[48];
    std::snprintf(expected_json, sizeof(expected_json), "\"battery_pct\":%.1f", expected);
    EXPECT_NE(recording->power.find(expected_json), std::string::npos);
    EXPECT_TRUE(recording->power_retained);
  }
}

INSTANTIATE_TEST_SUITE_P(SupportedPacks,
                         ConfiguredBatteryTest,
                         ::testing::Values(Pack{24.0, 28.0}, Pack{24.0, 28.5}, Pack{21.0, 28.5}));

}  // namespace
