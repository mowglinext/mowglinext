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
/**
 * @file mqtt_bridge_node.cpp
 * @brief Implementation of MqttBridgeNode and MQTT client classes.
 *
 * JSON serialisation uses simple snprintf-based string construction so that
 * the package has no external JSON library dependency.  All value types
 * produced are numeric or boolean literals — no user-visible string is embedded
 * inside a JSON string field without going through json_escape().
 */

#include "mowgli_monitoring/mqtt_bridge_node.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#ifdef MOWGLI_HAS_MOSQUITTO
#include <functional>
#include <mutex>
#include <unordered_map>

#include <mosquitto.h>
#endif

namespace mowgli_monitoring
{

using namespace std::chrono_literals;

// ===========================================================================
// StubMqttClient
// ===========================================================================

StubMqttClient::StubMqttClient(rclcpp::Logger logger) : logger_(logger)
{
}

bool StubMqttClient::connect() noexcept
{
  RCLCPP_DEBUG(logger_, "[StubMqttClient] connect()");
  connected_ = true;
  return true;
}

void StubMqttClient::disconnect() noexcept
{
  RCLCPP_DEBUG(logger_, "[StubMqttClient] disconnect()");
  connected_ = false;
}

bool StubMqttClient::publish(const std::string& topic,
                             const std::string& payload,
                             bool /*retain*/) noexcept
{
  RCLCPP_DEBUG(logger_,
               "[StubMqttClient] publish  topic='%s'  payload='%s'",
               topic.c_str(),
               payload.c_str());
  return connected_;
}

bool StubMqttClient::subscribe(const std::string& topic, MessageCallback /*callback*/) noexcept
{
  RCLCPP_DEBUG(logger_, "[StubMqttClient] subscribe  topic='%s'", topic.c_str());
  return connected_;
}

void StubMqttClient::spin_once() noexcept
{
  // Nothing to do for the stub.
}

bool StubMqttClient::is_connected() const noexcept
{
  return connected_;
}

// ===========================================================================
// MosquittoMqttClient
// ===========================================================================

#ifdef MOWGLI_HAS_MOSQUITTO

struct MosquittoMqttClient::Impl
{
  Config config;
  // rclcpp::Logger has no public default constructor; without an initializer
  // Impl's default constructor is deleted and make_unique<Impl>() fails to
  // compile. The constructor overwrites this with the node's logger.
  rclcpp::Logger logger{rclcpp::get_logger("mqtt_bridge_node")};
  // Throttle clock for the spin loop warning (a temporary cannot be bound by
  // the RCLCPP_*_THROTTLE macros).
  rclcpp::Clock throttle_clock{RCL_STEADY_TIME};
  mosquitto* mosq{nullptr};
  bool connected{false};

  std::mutex callbacks_mutex;
  std::unordered_map<std::string, MessageCallback> callbacks;

  static void on_connect_cb(mosquitto* /*mosq*/, void* userdata, int rc)
  {
    auto* self = static_cast<Impl*>(userdata);
    if (rc == 0)
    {
      self->connected = true;
      RCLCPP_INFO(self->logger,
                  "MQTT connected to %s:%d",
                  self->config.host.c_str(),
                  self->config.port);
      // (Re)issue all subscriptions on every successful connect. subscribe()
      // is called once at node construction — before the async mosquitto
      // connect completes — and the client uses clean_session=true, so the
      // broker drops subscription state on every (re)connect. Without
      // re-subscribing here the MQTT->ROS command path never received a
      // message. Safe to call from the connect callback (mosquitto loop ctx).
      {
        std::lock_guard<std::mutex> lock(self->callbacks_mutex);
        for (const auto& [topic, _cb] : self->callbacks)
        {
          const int sub_rc = mosquitto_subscribe(self->mosq, nullptr, topic.c_str(), 1 /* QoS */);
          if (sub_rc != MOSQ_ERR_SUCCESS)
          {
            RCLCPP_WARN(self->logger,
                        "MQTT re-subscribe on '%s' failed: %s",
                        topic.c_str(),
                        mosquitto_strerror(sub_rc));
          }
        }
      }

      // Announce availability now that we're actually connected. The LWT
      // (set once, pre-connect, in the constructor below) covers an
      // ungraceful disconnect; this covers the happy path AND every
      // reconnect, so a broker that watched us go offline sees us come
      // back — a retained "online" that only fired once at startup would
      // stay stale in that case.
      if (!self->config.availability_topic.empty())
      {
        mosquitto_publish(self->mosq,
                          nullptr,
                          self->config.availability_topic.c_str(),
                          6,
                          "online",
                          1 /* QoS */,
                          true /* retain */);
      }
    }
    else
    {
      RCLCPP_ERROR(self->logger, "MQTT connect failed rc=%d", rc);
    }
  }

  static void on_disconnect_cb(mosquitto* /*mosq*/, void* userdata, int rc)
  {
    auto* self = static_cast<Impl*>(userdata);
    self->connected = false;
    if (rc != 0)
    {
      RCLCPP_WARN(self->logger, "MQTT unexpected disconnect rc=%d", rc);
    }
  }

  static void on_message_cb(mosquitto* /*mosq*/,
                            void* userdata,
                            const struct mosquitto_message* msg)
  {
    auto* self = static_cast<Impl*>(userdata);
    if (!msg || !msg->payload)
    {
      return;
    }
    const std::string topic{msg->topic};
    const std::string payload{static_cast<const char*>(msg->payload),
                              static_cast<std::size_t>(msg->payloadlen)};

    std::lock_guard<std::mutex> lock(self->callbacks_mutex);
    auto it = self->callbacks.find(topic);
    if (it != self->callbacks.end())
    {
      it->second(topic, payload);
    }
  }
};

MosquittoMqttClient::MosquittoMqttClient(Config config, rclcpp::Logger logger)
    : impl_(std::make_unique<Impl>())
{
  impl_->config = std::move(config);
  impl_->logger = logger;

  mosquitto_lib_init();

  impl_->mosq = mosquitto_new(impl_->config.client_id.c_str(),
                              true,  // clean session
                              impl_.get());

  if (!impl_->mosq)
  {
    RCLCPP_ERROR(logger, "Failed to create mosquitto instance.");
    return;
  }

  if (!impl_->config.availability_topic.empty())
  {
    // Last Will and Testament: the broker publishes this retained "offline"
    // on our behalf if we vanish without a clean disconnect (crash, network
    // loss). Must be set here, before connect() — mosquitto_will_set() is a
    // pre-connect-only call. The clean disconnect path (offline published
    // explicitly, then disconnect()) and the reconnect "online" publish live
    // in disconnect() and on_connect_cb() respectively.
    const int will_rc = mosquitto_will_set(impl_->mosq,
                                           impl_->config.availability_topic.c_str(),
                                           7,
                                           "offline",
                                           1 /* QoS */,
                                           true /* retain */);
    if (will_rc != MOSQ_ERR_SUCCESS)
    {
      RCLCPP_WARN(logger,
                  "mosquitto_will_set failed: %s — availability will not be reported.",
                  mosquitto_strerror(will_rc));
    }
  }

  if (impl_->config.use_ssl)
  {
    // Apply TLS against the system CA store. use_ssl was parsed into the
    // config but never applied, so connections were always plaintext despite
    // the setting. (For a self-signed broker, add a ca_cert path param and
    // pass it as the cafile argument here.)
    const int tls_rc =
        mosquitto_tls_set(impl_->mosq, nullptr, "/etc/ssl/certs", nullptr, nullptr, nullptr);
    if (tls_rc != MOSQ_ERR_SUCCESS)
    {
      // TLS was requested but could not be configured. Tear down the handle so
      // connect() refuses rather than silently establishing a plaintext session
      // that would leak the broker credentials and the HighLevelControl channel.
      RCLCPP_ERROR(logger,
                   "mosquitto_tls_set failed (%d) — refusing to connect in plaintext.",
                   tls_rc);
      mosquitto_destroy(impl_->mosq);
      impl_->mosq = nullptr;
      return;
    }
    else
    {
      RCLCPP_INFO(logger, "MQTT TLS enabled (system CA store).");
    }
  }

  if (!impl_->config.username.empty())
  {
    mosquitto_username_pw_set(impl_->mosq,
                              impl_->config.username.c_str(),
                              impl_->config.password.empty() ? nullptr
                                                             : impl_->config.password.c_str());
  }

  mosquitto_connect_callback_set(impl_->mosq, Impl::on_connect_cb);
  mosquitto_disconnect_callback_set(impl_->mosq, Impl::on_disconnect_cb);
  mosquitto_message_callback_set(impl_->mosq, Impl::on_message_cb);
}

MosquittoMqttClient::~MosquittoMqttClient()
{
  disconnect();
  if (impl_->mosq)
  {
    mosquitto_destroy(impl_->mosq);
    impl_->mosq = nullptr;
  }
  mosquitto_lib_cleanup();
}

bool MosquittoMqttClient::connect() noexcept
{
  if (!impl_->mosq)
  {
    return false;
  }
  const int rc = mosquitto_connect(impl_->mosq,
                                   impl_->config.host.c_str(),
                                   impl_->config.port,
                                   60 /* keepalive seconds */);

  if (rc != MOSQ_ERR_SUCCESS)
  {
    RCLCPP_ERROR(impl_->logger, "mosquitto_connect failed: %s", mosquitto_strerror(rc));
    return false;
  }
  return true;
}

void MosquittoMqttClient::disconnect() noexcept
{
  if (impl_->mosq && impl_->connected)
  {
    // A clean disconnect does NOT trigger our own LWT (that only fires on an
    // ungraceful drop) — publish "offline" explicitly first so a normal
    // shutdown is reported too, not just a crash.
    if (!impl_->config.availability_topic.empty())
    {
      mosquitto_publish(impl_->mosq,
                        nullptr,
                        impl_->config.availability_topic.c_str(),
                        7,
                        "offline",
                        1 /* QoS */,
                        true /* retain */);
      // Let it actually leave the socket before we tear the connection down.
      mosquitto_loop(impl_->mosq, 100, 1);
    }
    mosquitto_disconnect(impl_->mosq);
  }
}

bool MosquittoMqttClient::publish(const std::string& topic,
                                  const std::string& payload,
                                  bool retain) noexcept
{
  if (!impl_->mosq || !impl_->connected)
  {
    return false;
  }
  const int rc = mosquitto_publish(impl_->mosq,
                                   nullptr,  // message id (out)
                                   topic.c_str(),
                                   static_cast<int>(payload.size()),
                                   payload.data(),
                                   1,  // QoS 1
                                   retain);

  if (rc != MOSQ_ERR_SUCCESS)
  {
    RCLCPP_WARN(impl_->logger,
                "mosquitto_publish failed on '%s': %s",
                topic.c_str(),
                mosquitto_strerror(rc));
    return false;
  }
  return true;
}

bool MosquittoMqttClient::subscribe(const std::string& topic, MessageCallback callback) noexcept
{
  if (!impl_->mosq)
  {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(impl_->callbacks_mutex);
    impl_->callbacks[topic] = std::move(callback);
  }

  const int rc = mosquitto_subscribe(impl_->mosq,
                                     nullptr,  // message id
                                     topic.c_str(),
                                     1 /* QoS */);

  if (rc != MOSQ_ERR_SUCCESS)
  {
    RCLCPP_WARN(impl_->logger,
                "mosquitto_subscribe failed on '%s': %s",
                topic.c_str(),
                mosquitto_strerror(rc));
    return false;
  }
  return true;
}

void MosquittoMqttClient::spin_once() noexcept
{
  if (!impl_->mosq)
  {
    return;
  }
  // Non-blocking loop iteration; timeout=0 means return immediately.
  const int rc = mosquitto_loop(impl_->mosq, 0, 1);
  if (rc != MOSQ_ERR_SUCCESS && rc != MOSQ_ERR_NO_CONN)
  {
    RCLCPP_WARN_THROTTLE(impl_->logger,
                         impl_->throttle_clock,
                         10000,
                         "mosquitto_loop error: %s — attempting reconnect",
                         mosquitto_strerror(rc));
    mosquitto_reconnect(impl_->mosq);
  }
}

bool MosquittoMqttClient::is_connected() const noexcept
{
  return impl_->connected;
}

#endif  // MOWGLI_HAS_MOSQUITTO

// ===========================================================================
// MqttBridgeNode
// ===========================================================================

MqttBridgeNode::MqttBridgeNode(const rclcpp::NodeOptions& options)
    : Node("mqtt_bridge_node", options)
{
  declare_parameters();
  create_mqtt_client();
  create_subscriptions();
  create_service_client();
  create_timer();
}

MqttBridgeNode::MqttBridgeNode(std::unique_ptr<IMqttClient> client,
                               const rclcpp::NodeOptions& options)
    : Node("mqtt_bridge_node", options), mqtt_client_(std::move(client))
{
  declare_parameters();
  // Client is already provided — skip create_mqtt_client().
  create_subscriptions();
  create_service_client();
  create_timer();
}

// ---------------------------------------------------------------------------
// Initialisation helpers
// ---------------------------------------------------------------------------

void MqttBridgeNode::declare_parameters()
{
  mqtt_host_ = declare_parameter<std::string>("mqtt_host", "localhost");
  mqtt_port_ = declare_parameter<int>("mqtt_port", 1883);
  mqtt_username_ = declare_parameter<std::string>("mqtt_username", "");
  mqtt_password_ = declare_parameter<std::string>("mqtt_password", "");
  mqtt_client_id_ = declare_parameter<std::string>("mqtt_client_id", "mowgli_ros2");
  topic_prefix_ = declare_parameter<std::string>("mqtt_topic_prefix", "mowgli");
  publish_rate_ = declare_parameter<double>("publish_rate", 1.0);
  use_ssl_ = declare_parameter<bool>("use_ssl", false);

  if (publish_rate_ < 0.01 || publish_rate_ > 100.0)
  {
    RCLCPP_WARN(get_logger(), "publish_rate out of range; clamping to 1.0 Hz.");
    publish_rate_ = 1.0;
  }
}

void MqttBridgeNode::create_mqtt_client()
{
#ifdef MOWGLI_HAS_MOSQUITTO
  MosquittoMqttClient::Config cfg;
  cfg.host = mqtt_host_;
  cfg.port = mqtt_port_;
  cfg.username = mqtt_username_;
  cfg.password = mqtt_password_;
  cfg.client_id = mqtt_client_id_;
  cfg.use_ssl = use_ssl_;
  cfg.availability_topic = full_topic("available");

  mqtt_client_ = std::make_unique<MosquittoMqttClient>(std::move(cfg), get_logger());
  RCLCPP_INFO(get_logger(), "Using MosquittoMqttClient → %s:%d", mqtt_host_.c_str(), mqtt_port_);
#else
  mqtt_client_ = std::make_unique<StubMqttClient>(get_logger());
  RCLCPP_WARN(get_logger(),
              "libmosquitto not available — using StubMqttClient. "
              "MQTT messages will be logged at DEBUG level only.");
#endif

  if (!mqtt_client_->connect())
  {
    RCLCPP_ERROR(get_logger(),
                 "Failed to connect to MQTT broker at %s:%d. Will retry via spin_once.",
                 mqtt_host_.c_str(),
                 mqtt_port_);
  }
}

void MqttBridgeNode::create_subscriptions()
{
  const auto sensor_qos = rclcpp::SensorDataQoS();

  sub_status_ = create_subscription<mowgli_interfaces::msg::Status>(
      "/hardware_bridge/status",
      10,
      [this](mowgli_interfaces::msg::Status::ConstSharedPtr msg)
      {
        on_status(msg);
      });

  sub_power_ = create_subscription<mowgli_interfaces::msg::Power>(
      "/hardware_bridge/power",
      10,
      [this](mowgli_interfaces::msg::Power::ConstSharedPtr msg)
      {
        on_power(msg);
      });

  sub_emergency_ = create_subscription<mowgli_interfaces::msg::Emergency>(
      "/hardware_bridge/emergency",
      10,
      [this](mowgli_interfaces::msg::Emergency::ConstSharedPtr msg)
      {
        on_emergency(msg);
      });

  sub_odom_ =
      create_subscription<nav_msgs::msg::Odometry>("/wheel_odom",
                                                   sensor_qos,
                                                   [this](
                                                       nav_msgs::msg::Odometry::ConstSharedPtr msg)
                                                   {
                                                     on_odom(msg);
                                                   });

  sub_diagnostics_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics",
      10,
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
      {
        on_diagnostics(msg);
      });

  // Control-plane state (BT high-level status), not raw sensor data — reliable
  // QoS(10) like status/power/emergency above, not SensorDataQoS.
  sub_high_level_status_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
      "/behavior_tree_node/high_level_status",
      10,
      [this](mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg)
      {
        on_high_level_status(msg);
      });

  // Raw GPS fix (lat/lon/alt) for external map/device_tracker consumers.
  // SensorDataQoS per .claude/rules/ros2.md: GPS drivers publish BEST_EFFORT.
  sub_gps_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix",
      sensor_qos,
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
      {
        on_gps_fix(msg);
      });

  // Subscribe to MQTT command topic.
  mqtt_client_->subscribe(full_topic("command"),
                          [this](const std::string& topic, const std::string& payload)
                          {
                            on_mqtt_command(topic, payload);
                          });
}

void MqttBridgeNode::create_service_client()
{
  srv_high_level_ = create_client<mowgli_interfaces::srv::HighLevelControl>(
      "/behavior_tree_node/high_level_control");
}

void MqttBridgeNode::create_timer()
{
  const auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
  timer_ = create_wall_timer(period_ms,
                             [this]()
                             {
                               on_timer();
                             });
}

// ---------------------------------------------------------------------------
// ROS2 subscription callbacks → MQTT publish
// ---------------------------------------------------------------------------

void MqttBridgeNode::on_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg)
{
  mqtt_client_->publish(full_topic("status"), serialise_status(*msg), /*retain=*/true);
}

void MqttBridgeNode::on_power(mowgli_interfaces::msg::Power::ConstSharedPtr msg)
{
  mqtt_client_->publish(full_topic("power"), serialise_power(*msg), /*retain=*/true);
}

void MqttBridgeNode::on_emergency(mowgli_interfaces::msg::Emergency::ConstSharedPtr msg)
{
  mqtt_client_->publish(full_topic("emergency"), serialise_emergency(*msg), /*retain=*/true);
}

void MqttBridgeNode::on_odom(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // Store the latest odometry; the timer will rate-limit publication.
  pending_odom_ = *msg;
}

void MqttBridgeNode::on_diagnostics(diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg)
{
  mqtt_client_->publish(full_topic("diagnostics"), serialise_diagnostics(*msg), /*retain=*/false);
}

void MqttBridgeNode::on_high_level_status(
    mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg)
{
  mqtt_client_->publish(full_topic("high_level_status"),
                        serialise_high_level_status(*msg),
                        /*retain=*/true);
}

void MqttBridgeNode::on_gps_fix(sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
{
  // Store the latest fix; the timer will rate-limit publication (same
  // pattern as on_odom/pending_odom_ above).
  pending_gps_ = *msg;
}

// ---------------------------------------------------------------------------
// MQTT command callback → ROS2 service call
// ---------------------------------------------------------------------------

bool MqttBridgeNode::parse_command_payload(const std::string& payload, uint8_t& out_command)
{
  int command_int = -1;
  if (std::sscanf(payload.c_str(), "%d", &command_int) != 1 || command_int < 0 || command_int > 255)
  {
    return false;
  }
  out_command = static_cast<uint8_t>(command_int);
  return true;
}

void MqttBridgeNode::on_mqtt_command(const std::string& /*topic*/, const std::string& payload)
{
  // Expected payload: a single ASCII decimal integer matching HighLevelControl
  // command codes — NOT a raw byte. E.g.: "1" → COMMAND_START,
  // "2" → COMMAND_HOME, "254" → COMMAND_RESET_EMERGENCY.
  uint8_t command_uint = 0;
  if (!parse_command_payload(payload, command_uint))
  {
    RCLCPP_WARN(get_logger(),
                "MQTT command payload '%s' is not a valid uint8 command code. Ignored.",
                payload.c_str());
    return;
  }
  const int command_int = static_cast<int>(command_uint);

  if (!srv_high_level_->service_is_ready())
  {
    RCLCPP_WARN(get_logger(), "HighLevelControl service not available; command dropped.");
    return;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::HighLevelControl::Request>();
  request->command = command_uint;

  // Fire-and-forget async call — we do not block the ROS2 executor.
  srv_high_level_->async_send_request(
      request,
      [this,
       command_int](rclcpp::Client<mowgli_interfaces::srv::HighLevelControl>::SharedFuture future)
      {
        const auto response = future.get();
        if (response->success)
        {
          RCLCPP_INFO(get_logger(), "HighLevelControl command=%d succeeded.", command_int);
        }
        else
        {
          RCLCPP_WARN(get_logger(), "HighLevelControl command=%d reported failure.", command_int);
        }
      });
}

// ---------------------------------------------------------------------------
// Timer: network loop + rate-limited position publish
// ---------------------------------------------------------------------------

void MqttBridgeNode::on_timer()
{
  // Drive the MQTT network loop.
  mqtt_client_->spin_once();

  // Attempt reconnect if disconnected.
  if (!mqtt_client_->is_connected())
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         10000,
                         "MQTT disconnected — attempting reconnect.");
    mqtt_client_->connect();
    return;
  }

  // Rate-limited position publish.
  if (pending_odom_.has_value())
  {
    const rclcpp::Time t = now();
    const double elapsed = (t - last_odom_publish_).seconds();
    const double min_interval = 1.0 / publish_rate_;

    if (elapsed >= min_interval)
    {
      mqtt_client_->publish(full_topic("position"),
                            serialise_position(*pending_odom_),
                            /*retain=*/false);
      last_odom_publish_ = t;
      pending_odom_.reset();
    }
  }

  // Rate-limited GPS publish (same window as position above).
  if (pending_gps_.has_value())
  {
    const rclcpp::Time t = now();
    const double elapsed = (t - last_gps_publish_).seconds();
    const double min_interval = 1.0 / publish_rate_;

    if (elapsed >= min_interval)
    {
      mqtt_client_->publish(full_topic("gps"), serialise_gps(*pending_gps_), /*retain=*/false);
      last_gps_publish_ = t;
      pending_gps_.reset();
    }
  }
}

// ---------------------------------------------------------------------------
// JSON serialisers
// ---------------------------------------------------------------------------

std::string MqttBridgeNode::serialise_status(const mowgli_interfaces::msg::Status& msg)
{
  char buf[512];
  std::snprintf(buf,
                sizeof(buf),
                "{"
                "\"mower_status\":%u,"
                "\"raspberry_pi_power\":%s,"
                "\"is_charging\":%s,"
                "\"esc_power\":%s,"
                "\"rain_detected\":%s,"
                "\"sound_module_available\":%s,"
                "\"sound_module_busy\":%s,"
                "\"ui_board_available\":%s,"
                "\"mow_enabled\":%s,"
                "\"mower_esc_status\":%u,"
                "\"mower_esc_temperature\":%.2f,"
                "\"mower_esc_current\":%.3f,"
                "\"mower_motor_temperature\":%.2f,"
                "\"mower_motor_rpm\":%.1f"
                "}",
                static_cast<unsigned>(msg.mower_status),
                msg.raspberry_pi_power ? "true" : "false",
                msg.is_charging ? "true" : "false",
                msg.esc_power ? "true" : "false",
                msg.rain_detected ? "true" : "false",
                msg.sound_module_available ? "true" : "false",
                msg.sound_module_busy ? "true" : "false",
                msg.ui_board_available ? "true" : "false",
                msg.mow_enabled ? "true" : "false",
                static_cast<unsigned>(msg.mower_esc_status),
                static_cast<double>(msg.mower_esc_temperature),
                static_cast<double>(msg.mower_esc_current),
                static_cast<double>(msg.mower_motor_temperature),
                static_cast<double>(msg.mower_motor_rpm));
  return std::string{buf};
}

std::string MqttBridgeNode::serialise_power(const mowgli_interfaces::msg::Power& msg)
{
  // Derive battery percentage same as diagnostics (4S LiPo 12.0–16.8V range).
  constexpr double kVFull = 16.8;
  constexpr double kVEmpty = 12.0;
  const double voltage = static_cast<double>(msg.v_battery);
  double pct = 100.0 * (voltage - kVEmpty) / (kVFull - kVEmpty);
  pct = std::max(0.0, std::min(100.0, pct));

  char buf[256];
  std::snprintf(buf,
                sizeof(buf),
                "{"
                "\"v_charge\":%.3f,"
                "\"v_battery\":%.3f,"
                "\"charge_current\":%.3f,"
                "\"charger_enabled\":%s,"
                "\"charger_status\":\"%s\","
                "\"battery_pct\":%.1f"
                "}",
                static_cast<double>(msg.v_charge),
                voltage,
                static_cast<double>(msg.charge_current),
                msg.charger_enabled ? "true" : "false",
                json_escape(msg.charger_status).c_str(),
                pct);
  return std::string{buf};
}

std::string MqttBridgeNode::serialise_emergency(const mowgli_interfaces::msg::Emergency& msg)
{
  char buf[256];
  std::snprintf(buf,
                sizeof(buf),
                "{"
                "\"active_emergency\":%s,"
                "\"latched_emergency\":%s,"
                "\"reason\":\"%s\""
                "}",
                msg.active_emergency ? "true" : "false",
                msg.latched_emergency ? "true" : "false",
                json_escape(msg.reason).c_str());
  return std::string{buf};
}

std::string MqttBridgeNode::serialise_position(const nav_msgs::msg::Odometry& msg)
{
  // Extract 2D pose + heading from the odometry message.
  const double x = msg.pose.pose.position.x;
  const double y = msg.pose.pose.position.y;
  const double qz = msg.pose.pose.orientation.z;
  const double qw = msg.pose.pose.orientation.w;
  const double theta = 2.0 * std::atan2(qz, qw);

  char buf[128];
  std::snprintf(buf, sizeof(buf), "{\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f}", x, y, theta);
  return std::string{buf};
}

std::string MqttBridgeNode::serialise_diagnostics(const diagnostic_msgs::msg::DiagnosticArray& msg)
{
  // Produce a compact summary: array of {name, level, message} objects.
  std::string json = "[";
  bool first = true;
  for (const auto& status : msg.status)
  {
    if (!first)
    {
      json += ',';
    }
    first = false;

    char entry[512];
    std::snprintf(entry,
                  sizeof(entry),
                  "{\"name\":\"%s\",\"level\":%u,\"message\":\"%s\"}",
                  json_escape(status.name).c_str(),
                  static_cast<unsigned>(status.level),
                  json_escape(status.message).c_str());
    json += entry;
  }
  json += ']';
  return json;
}

std::string MqttBridgeNode::serialise_high_level_status(
    const mowgli_interfaces::msg::HighLevelStatus& msg)
{
  char buf[768];
  std::snprintf(buf,
                sizeof(buf),
                "{"
                "\"state\":%u,"
                "\"state_name\":\"%s\","
                "\"sub_state_name\":\"%s\","
                "\"current_area\":%d,"
                "\"current_path\":%d,"
                "\"current_path_index\":%d,"
                "\"total_swaths\":%d,"
                "\"completed_swaths\":%d,"
                "\"skipped_swaths\":%d,"
                "\"coverage_percent\":%.1f,"
                "\"gps_quality_percent\":%.1f,"
                "\"battery_percent\":%.1f,"
                "\"is_charging\":%s,"
                "\"emergency\":%s"
                "}",
                static_cast<unsigned>(msg.state),
                json_escape(msg.state_name).c_str(),
                json_escape(msg.sub_state_name).c_str(),
                static_cast<int>(msg.current_area),
                static_cast<int>(msg.current_path),
                static_cast<int>(msg.current_path_index),
                static_cast<int>(msg.total_swaths),
                static_cast<int>(msg.completed_swaths),
                static_cast<int>(msg.skipped_swaths),
                static_cast<double>(msg.coverage_percent),
                static_cast<double>(msg.gps_quality_percent),
                static_cast<double>(msg.battery_percent),
                msg.is_charging ? "true" : "false",
                msg.emergency ? "true" : "false");
  return std::string{buf};
}

std::string MqttBridgeNode::serialise_gps(const sensor_msgs::msg::NavSatFix& msg)
{
  // status.status is STATUS_NO_FIX(-1)/STATUS_FIX(0)/STATUS_SBAS_FIX(1)/
  // STATUS_GBAS_FIX(2); this is a raw NavSatFix relay, not the richer
  // universal_gnss RTK-quality summary the GUI/BT use elsewhere — a
  // consumer wanting Fixed-vs-Float should not read this field as that.
  char buf[192];
  std::snprintf(buf,
                sizeof(buf),
                "{\"latitude\":%.8f,\"longitude\":%.8f,\"altitude\":%.3f,"
                "\"status\":%d,\"service\":%u}",
                msg.latitude,
                msg.longitude,
                msg.altitude,
                static_cast<int>(msg.status.status),
                static_cast<unsigned>(msg.status.service));
  return std::string{buf};
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

std::string MqttBridgeNode::full_topic(const std::string& suffix) const
{
  return topic_prefix_ + "/" + suffix;
}

std::string MqttBridgeNode::json_escape(const std::string& raw)
{
  std::string out;
  out.reserve(raw.size() + 4);
  for (const char c : raw)
  {
    switch (c)
    {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out += c;
        break;
    }
  }
  return out;
}

}  // namespace mowgli_monitoring
