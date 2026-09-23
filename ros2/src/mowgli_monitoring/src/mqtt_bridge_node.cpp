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

#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#ifdef MOWGLI_HAS_MOSQUITTO
#include <chrono>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <vector>

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
  struct PendingMessage
  {
    std::string topic;
    std::string payload;
    bool retained{false};
  };

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
  std::chrono::steady_clock::time_point last_reconnect_attempt{};

  static constexpr int kMaxLoopIterationsPerSpin = 64;
  static constexpr std::chrono::seconds kReconnectMinInterval{1};

  std::mutex callbacks_mutex;
  std::unordered_map<std::string, MessageCallback> callbacks;
  std::mutex pending_messages_mutex;
  std::vector<PendingMessage> pending_messages;

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
    const bool retained = msg->retain;

    {
      std::lock_guard<std::mutex> lock(self->pending_messages_mutex);
      self->pending_messages.push_back({topic, payload, retained});
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
  // One non-blocking mosquitto_loop() call moves too little per tick: a field capture
  // (2026-09-21) showed <prefix>/high_level_status reaching the broker at ~0.57 msg/s
  // against ~1 msg/s produced, so the lag grew without bound (>9 min after 16 min).
  // Keep looping while packets are still waiting to be written.
  int rc = MOSQ_ERR_SUCCESS;
  for (int i = 0; i < Impl::kMaxLoopIterationsPerSpin; ++i)
  {
    rc = mosquitto_loop(impl_->mosq, 0, 1);
    if (rc != MOSQ_ERR_SUCCESS || !mosquitto_want_write(impl_->mosq))
    {
      break;
    }
  }

  if (rc != MOSQ_ERR_SUCCESS && rc != MOSQ_ERR_NO_CONN)
  {
    RCLCPP_WARN_THROTTLE(impl_->logger,
                         impl_->throttle_clock,
                         10000,
                         "mosquitto_loop error: %s — attempting reconnect",
                         mosquitto_strerror(rc));
    // spin_once() runs at 20 Hz; mosquitto_reconnect() blocks on the TCP connect.
    const auto now = std::chrono::steady_clock::now();
    if (now - impl_->last_reconnect_attempt >= Impl::kReconnectMinInterval)
    {
      impl_->last_reconnect_attempt = now;
      mosquitto_reconnect(impl_->mosq);
    }
  }

  // libmosquitto invokes on_message_cb from inside mosquitto_loop(). Queue
  // deliveries there, then invoke application callbacks only after the loop
  // has returned. A callback may publish a response (Home Assistant's birth
  // message does exactly that), and re-entering the same client from inside
  // the library callback can leave that publish queued indefinitely on some
  // libmosquitto versions.
  std::vector<Impl::PendingMessage> pending;
  {
    std::lock_guard<std::mutex> lock(impl_->pending_messages_mutex);
    pending.swap(impl_->pending_messages);
  }
  for (const auto& message : pending)
  {
    MessageCallback callback;
    {
      std::lock_guard<std::mutex> lock(impl_->callbacks_mutex);
      const auto it = impl_->callbacks.find(message.topic);
      if (it != impl_->callbacks.end())
      {
        callback = it->second;
      }
    }
    if (callback)
    {
      callback(message.topic, message.payload, message.retained);
    }
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
  home_assistant_discovery_enabled_ =
      declare_parameter<bool>("home_assistant_discovery_enabled", false);
  // Injected by full_system.launch.py from mowgli_robot.yaml, same as
  // map_server_node/navsat_to_absolute_pose_node — labels
  // <prefix>/area_boundary's map-frame geometry with its WGS84 origin.
  datum_lat_ = declare_parameter<double>("datum_lat", 0.0);
  datum_lon_ = declare_parameter<double>("datum_lon", 0.0);
  // Charging dock pose (map frame), injected from mowgli_robot.yaml by
  // full_system.launch.py like the datum above; shown on <prefix>/area_boundary.
  dock_pose_x_ = declare_parameter<double>("dock_pose_x", 0.0);
  dock_pose_y_ = declare_parameter<double>("dock_pose_y", 0.0);
  dock_pose_yaw_ = declare_parameter<double>("dock_pose_yaw", 0.0);

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
  create_high_level_status_subscription();

  // Raw GPS fix (lat/lon/alt) for external map/device_tracker consumers.
  // SensorDataQoS per .claude/rules/ros2.md: GPS drivers publish BEST_EFFORT.
  sub_gps_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps/fix",
      sensor_qos,
      [this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)
      {
        on_gps_fix(msg);
      });

  // RTK/GNSS fix quality — the same typed status (and gnss_status_utils
  // helpers) the LED ring and behavior tree read, so this topic can never
  // disagree with what the robot itself shows. QoS(10) reliable, matching
  // led_ring_node's and behavior_tree_node's own subscriptions to it (a
  // derived status topic, not raw sensor data).
  sub_gnss_status_ = create_subscription<mowgli_interfaces::msg::GnssStatus>(
      "/gps/status",
      10,
      [this](mowgli_interfaces::msg::GnssStatus::ConstSharedPtr msg)
      {
        on_gnss_status(msg);
      });

  // Fused map-frame pose from the localizer: position and heading that do not jitter
  // like the raw GPS fix. SensorDataQoS is compatible with the localizer's reliable
  // publisher and with a best-effort one, should it ever become one.
  sub_pose_ =
      create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered_map",
                                                   sensor_qos,
                                                   [this](
                                                       nav_msgs::msg::Odometry::ConstSharedPtr msg)
                                                   {
                                                     on_pose(msg);
                                                   });

  // Subscribe to MQTT command topics.
  mqtt_client_->subscribe(full_topic("command"),
                          [this](const std::string& topic,
                                 const std::string& payload,
                                 bool retained)
                          {
                            on_mqtt_command(topic, payload, retained);
                          });

  mqtt_client_->subscribe(full_topic("start_area"),
                          [this](const std::string& topic,
                                 const std::string& payload,
                                 bool retained)
                          {
                            on_mqtt_start_area(topic, payload, retained);
                          });

  if (home_assistant_discovery_enabled_)
  {
    mqtt_client_->subscribe("homeassistant/status",
                            [this](const std::string& topic,
                                   const std::string& payload,
                                   bool retained)
                            {
                              on_home_assistant_status(topic, payload, retained);
                            });
  }
}

void MqttBridgeNode::create_high_level_status_subscription()
{
  sub_high_level_status_ = create_subscription<mowgli_interfaces::msg::HighLevelStatus>(
      "/behavior_tree_node/high_level_status",
      10,
      [this](mowgli_interfaces::msg::HighLevelStatus::ConstSharedPtr msg)
      {
        on_high_level_status(msg);
      });
}

void MqttBridgeNode::create_service_client()
{
  srv_high_level_ = create_client<mowgli_interfaces::srv::HighLevelControl>(
      "/behavior_tree_node/high_level_control");
  srv_get_area_ =
      create_client<mowgli_interfaces::srv::GetMowingArea>("/map_server_node/get_mowing_area");
  srv_start_area_ =
      create_client<mowgli_interfaces::srv::StartInArea>("/behavior_tree_node/start_in_area");
  srv_get_mowing_area_ =
      create_client<mowgli_interfaces::srv::GetMowingArea>("/map_server_node/get_mowing_area");
}

void MqttBridgeNode::create_timer()
{
  // The MQTT socket must be serviced independently of the configured
  // telemetry rate. At a normal 1 Hz publish rate, using that same one-second
  // period for mosquitto_loop() can leave retained discovery and inbound
  // commands queued behind sensor traffic. Position and GPS remain throttled
  // below with publish_rate_; this timer only bounds network latency.
  timer_ = create_wall_timer(std::chrono::milliseconds(100),
                             [this]()
                             {
                               on_timer();
                             });

  // The network loop must not share publish_rate's cadence: at 1 Hz it could not
  // drain the outgoing queue and <prefix>/high_level_status fell minutes behind.
  net_timer_ = create_wall_timer(std::chrono::milliseconds(kNetworkLoopPeriodMs),
                                 [this]()
                                 {
                                   mqtt_client_->spin_once();
                                 });
}

// ---------------------------------------------------------------------------
// ROS2 subscription callbacks → MQTT publish
// ---------------------------------------------------------------------------

void MqttBridgeNode::on_status(mowgli_interfaces::msg::Status::ConstSharedPtr msg)
{
  // Latest value only; on_timer() publishes it at most publish_rate_ times a second.
  pending_status_ = *msg;
}

void MqttBridgeNode::on_power(mowgli_interfaces::msg::Power::ConstSharedPtr msg)
{
  pending_power_ = *msg;
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
  received_high_level_status_ = true;
  last_high_level_status_received_ = now();

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

void MqttBridgeNode::on_gnss_status(mowgli_interfaces::msg::GnssStatus::ConstSharedPtr msg)
{
  pending_gnss_status_ = *msg;
}

void MqttBridgeNode::on_pose(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const auto& p = msg->pose.pose;
  if (!std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
      !std::isfinite(p.orientation.z) || !std::isfinite(p.orientation.w))
  {
    return;  // a localizer that has not converged yet must not put NaN on the wire
  }
  pending_pose_ = *msg;
}

// ---------------------------------------------------------------------------
// MQTT command callback → ROS2 service call
// ---------------------------------------------------------------------------

bool MqttBridgeNode::parse_command_payload(const std::string& payload, uint8_t& out_command)
{
  // from_chars neither skips whitespace nor accepts a leading '+'. Checking
  // that it consumed all bytes makes the accepted grammar exactly [0-9]+.
  unsigned int command_int = 0;
  const auto [end, error] =
      std::from_chars(payload.data(), payload.data() + payload.size(), command_int, 10);
  if (payload.empty() || error != std::errc{} || end != payload.data() + payload.size() ||
      command_int > 255)
  {
    return false;
  }
  out_command = static_cast<uint8_t>(command_int);
  return true;
}

bool MqttBridgeNode::is_publish_due(const rclcpp::Time& now,
                                    const rclcpp::Time& last_publish,
                                    double min_interval_s)
{
  return (now - last_publish).seconds() >= min_interval_s;
}

bool MqttBridgeNode::is_high_level_status_stale(bool received_before,
                                                const rclcpp::Time& now,
                                                const rclcpp::Time& last_received,
                                                double threshold_s)
{
  if (!received_before)
  {
    // Never received one yet — normal during startup (behavior_tree_node may
    // not be up), not evidence of a stuck subscription.
    return false;
  }
  return (now - last_received).seconds() > threshold_s;
}

void MqttBridgeNode::on_mqtt_command(const std::string& /*topic*/,
                                     const std::string& payload,
                                     bool retained)
{
  if (!is_fresh_control_message(retained))
  {
    RCLCPP_WARN(get_logger(), "Retained MQTT command ignored as stale operator intent.");
    return;
  }

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
// MQTT start_area callback → StartInArea service call
// ---------------------------------------------------------------------------

void MqttBridgeNode::on_mqtt_start_area(const std::string& /*topic*/,
                                        const std::string& payload,
                                        bool retained)
{
  if (!is_fresh_control_message(retained))
  {
    RCLCPP_WARN(get_logger(), "Retained MQTT start_area ignored as stale operator intent.");
    return;
  }

  // Same payload convention as <prefix>/command: ASCII decimal, not a raw
  // byte. Reuses parse_command_payload — StartInArea.area is a uint8, same
  // range and format as the HighLevelControl command codes.
  uint8_t area_index = 0;
  if (!parse_command_payload(payload, area_index))
  {
    RCLCPP_WARN(get_logger(),
                "MQTT start_area payload '%s' is not a valid uint8 area index. Ignored.",
                payload.c_str());
    return;
  }

  if (!srv_start_area_->service_is_ready())
  {
    RCLCPP_WARN(get_logger(), "StartInArea service not available; command dropped.");
    return;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::StartInArea::Request>();
  request->area = area_index;

  // Fire-and-forget async call, same pattern as on_mqtt_command above — this
  // is exactly as consequential as the existing bare COMMAND_START relay
  // (StartInArea raises COMMAND_START internally too, ahead of the normal
  // area-iteration order), so it inherits the same "no ack topic, poll
  // high_level_status" contract rather than a bespoke response channel.
  srv_start_area_->async_send_request(
      request,
      [this, area_index](rclcpp::Client<mowgli_interfaces::srv::StartInArea>::SharedFuture future)
      {
        const auto response = future.get();
        if (response->success)
        {
          RCLCPP_INFO(get_logger(),
                      "StartInArea area=%u accepted.",
                      static_cast<unsigned>(area_index));
        }
        else
        {
          RCLCPP_WARN(get_logger(),
                      "StartInArea area=%u reported failure.",
                      static_cast<unsigned>(area_index));
        }
      });
}

void MqttBridgeNode::on_home_assistant_status(const std::string& /*topic*/,
                                              const std::string& payload,
                                              bool /*retained*/)
{
  if (home_assistant_discovery_enabled_ && payload == "online")
  {
    RCLCPP_INFO(get_logger(), "Home Assistant is online; republishing MQTT discovery.");
    home_assistant_discovery_publish_pending_ = true;
  }
}

bool MqttBridgeNode::publish_home_assistant_discovery()
{
  return mqtt_client_->publish(home_assistant_discovery_topic(topic_prefix_),
                               serialise_home_assistant_discovery(topic_prefix_, last_areas_),
                               /*retain=*/true);
}

// ---------------------------------------------------------------------------
// Area list: periodic poll of GetMowingArea + publish
// ---------------------------------------------------------------------------

void MqttBridgeNode::poll_areas()
{
  if (!srv_get_area_->service_is_ready())
  {
    // map_server_node not up yet (or currently down) — try again next
    // interval; areas_poll_in_flight_ stays false so on_timer() will retry.
    return;
  }
  areas_poll_in_flight_ = true;
  poll_areas_step(0, std::make_shared<std::vector<AreaSummary>>());
}

void MqttBridgeNode::poll_areas_step(uint32_t index,
                                     std::shared_ptr<std::vector<AreaSummary>> collected)
{
  if (index >= kMaxAreasPoll)
  {
    areas_poll_in_flight_ = false;
    publish_areas_if_changed(*collected);
    return;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  request->index = index;

  srv_get_area_->async_send_request(
      request,
      [this, index, collected](
          rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedFuture future)
      {
        const auto response = future.get();
        if (!response->success)
        {
          // index >= areas_.size() on the server side — end of the list.
          areas_poll_in_flight_ = false;
          publish_areas_if_changed(*collected);
          return;
        }

        // Navigation-only areas (keepout/boundary zones, never mowed) are
        // deliberately excluded — same split the GUI applies
        // (splitMapAreas) before offering "mow this area" to an operator. A
        // start_area command targeting one would just be skipped by the
        // BT's own mow-selection loop regardless, but excluding it here
        // keeps the published list meaning "things you can actually ask
        // this topic to mow".
        if (!response->area.is_navigation_area)
        {
          collected->push_back(AreaSummary{index, response->area.name});
        }

        poll_areas_step(index + 1, collected);
      });
}

void MqttBridgeNode::publish_areas_if_changed(const std::vector<AreaSummary>& areas)
{
  const std::string json = serialise_areas(areas);
  if (json == last_areas_json_)
  {
    return;
  }
  last_areas_json_ = json;
  last_areas_ = areas;
  mqtt_client_->publish(full_topic("areas"), json, /*retain=*/true);
  if (home_assistant_discovery_enabled_)
  {
    // Area buttons are part of the same device-discovery document. Refresh it
    // when the map's mowable area list changes so Home Assistant adds, renames
    // or removes the corresponding action buttons.
    home_assistant_discovery_publish_pending_ = !publish_home_assistant_discovery();
  }
}

// ---------------------------------------------------------------------------
// Timers: rate-limited publishes (on_timer) + network loop (net_timer_)
// ---------------------------------------------------------------------------

void MqttBridgeNode::on_timer()
{
  // The MQTT network loop runs on net_timer_, not here.

  // Attempt reconnect if disconnected.
  if (!mqtt_client_->is_connected())
  {
    mqtt_was_connected_ = false;
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         10000,
                         "MQTT disconnected — attempting reconnect.");
    mqtt_client_->connect();
    return;
  }

  if (!mqtt_was_connected_)
  {
    mqtt_was_connected_ = true;
    if (home_assistant_discovery_enabled_)
    {
      home_assistant_discovery_publish_pending_ = true;
    }
    else
    {
      // An empty retained discovery payload removes a configuration left by
      // an earlier enabled run with this topic prefix.
      mqtt_client_->publish(home_assistant_discovery_topic(topic_prefix_), "", /*retain=*/true);
    }
  }

  // A Home Assistant birth message is received while spin_once() is driving
  // the MQTT client. Publish only after spin_once() has returned completely,
  // then let the next timer tick flush the queued QoS message. The same path
  // handles the initial connection edge and retries a synchronous failure.
  if (home_assistant_discovery_publish_pending_)
  {
    const bool queued = publish_home_assistant_discovery();
    home_assistant_discovery_publish_pending_ = !queued;
    if (queued)
    {
      // Discovery is a relatively large retained QoS message. Give the MQTT
      // client one immediate network-loop pass so a busy ROS executor cannot
      // delay delivery until a later timer callback.
      mqtt_client_->spin_once();
    }
  }

  // Rate-limited publishes: each topic sends only its latest pending message, at most
  // once per 1/publish_rate_ seconds. emergency and high_level_status are not limited
  // (see their callbacks) — they are low-rate and must not be delayed.
  const rclcpp::Time flush_time = now();
  const double min_interval = 1.0 / publish_rate_;
  const auto flush = [&](auto& pending,
                         rclcpp::Time& last_publish,
                         const char* suffix,
                         auto&& serialise,
                         bool retain)
  {
    if (pending.has_value() && is_publish_due(flush_time, last_publish, min_interval))
    {
      mqtt_client_->publish(full_topic(suffix), serialise(*pending), retain);
      last_publish = flush_time;
      pending.reset();
    }
  };

  flush(pending_odom_, last_odom_publish_, "position", serialise_position, /*retain=*/false);
  flush(pending_gps_, last_gps_publish_, "gps", serialise_gps, /*retain=*/false);
  flush(pending_status_, last_status_publish_, "status", serialise_status, /*retain=*/true);
  flush(pending_power_, last_power_publish_, "power", serialise_power, /*retain=*/true);
  flush(pending_pose_, last_pose_publish_, "pose", serialise_pose, /*retain=*/false);
  flush(pending_gnss_status_,
        last_gnss_status_publish_,
        "rtk_status",
        serialise_rtk_status,
        /*retain=*/true);

  maybe_poll_area_boundaries();

  // Slow periodic area-list poll — independent of publish_rate_, see
  // kAreasPollIntervalS's doc comment (mqtt_bridge_node.hpp).
  if (!areas_poll_in_flight_)
  {
    const rclcpp::Time t = now();
    const double elapsed = (t - last_areas_poll_).seconds();
    if (elapsed >= kAreasPollIntervalS)
    {
      last_areas_poll_ = t;
      poll_areas();
    }
  }

  // <prefix>/high_level_status subscription watchdog (mowglinext#644) — see
  // the file-level doc comment (mqtt_bridge_node.hpp) for the full field
  // observation this recovers from.
  if (is_high_level_status_stale(received_high_level_status_,
                                 now(),
                                 last_high_level_status_received_,
                                 kHighLevelStatusStaleAfterS))
  {
    RCLCPP_WARN(get_logger(),
                "No /behavior_tree_node/high_level_status message in over %.0fs — "
                "recreating the subscription.",
                kHighLevelStatusStaleAfterS);
    create_high_level_status_subscription();
    // Give the fresh subscription a full window before re-checking, rather
    // than re-triggering next tick if behavior_tree_node itself is what's
    // actually down.
    last_high_level_status_received_ = now();
  }
}

// ---------------------------------------------------------------------------
// Area boundary polling
// ---------------------------------------------------------------------------

void MqttBridgeNode::maybe_poll_area_boundaries()
{
  if (area_poll_in_progress_)
  {
    return;
  }
  const rclcpp::Time t = now();
  if ((t - last_area_poll_).seconds() < kAreaPollIntervalS)
  {
    return;
  }
  if (!srv_get_mowing_area_->service_is_ready())
  {
    // map_server_node not up (yet, or at all) — retry after the same
    // interval rather than hammering service_is_ready() every tick.
    last_area_poll_ = t;
    return;
  }

  area_poll_in_progress_ = true;
  last_area_poll_ = t;
  auto accumulated =
      std::make_shared<std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>>>();
  poll_area_boundary_step(0, accumulated);
}

void MqttBridgeNode::poll_area_boundary_step(
    uint32_t index,
    std::shared_ptr<std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>>> accumulated)
{
  if (index >= kMaxAreaPollCount)
  {
    finish_area_boundary_poll(accumulated);
    return;
  }

  auto request = std::make_shared<mowgli_interfaces::srv::GetMowingArea::Request>();
  request->index = index;

  // Not native recursion: async_send_request's callback runs later, off the
  // executor's queue, not synchronously inline — each step's lambda returns
  // immediately after scheduling the next request, so there is no growing
  // call stack even for many areas.
  srv_get_mowing_area_->async_send_request(
      request,
      [this, index, accumulated](
          rclcpp::Client<mowgli_interfaces::srv::GetMowingArea>::SharedFuture future)
      {
        const auto response = future.get();
        if (!response->success)
        {
          // Index out of range = end of the (possibly non-contiguous, see
          // docs/MQTT_CONTROL.md) area list.
          finish_area_boundary_poll(accumulated);
          return;
        }
        if (!response->area.is_navigation_area)
        {
          // Navigation-only areas (keepout/boundary zones, never mowed) are
          // excluded — matches <prefix>/areas' own filter (PR #638).
          accumulated->emplace_back(index, response->area);
        }
        poll_area_boundary_step(index + 1, accumulated);
      });
}

void MqttBridgeNode::finish_area_boundary_poll(
    std::shared_ptr<std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>>> accumulated)
{
  area_poll_in_progress_ = false;
  const std::string json =
      serialise_area_boundaries(*accumulated,
                                datum_lat_,
                                datum_lon_,
                                make_dock_pose(dock_pose_x_, dock_pose_y_, dock_pose_yaw_));
  if (json == last_area_boundary_json_)
  {
    // Retained topic: republish only when the geometry actually changed,
    // not every ~10s poll tick.
    return;
  }
  last_area_boundary_json_ = json;
  mqtt_client_->publish(full_topic("area_boundary"), json, /*retain=*/true);
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

std::string MqttBridgeNode::serialise_pose(const nav_msgs::msg::Odometry& msg)
{
  const auto& q = msg.pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  char buf[128];
  std::snprintf(buf,
                sizeof(buf),
                "{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.4f}",
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw);
  return std::string{buf};
}

std::optional<MqttBridgeNode::DockPose> MqttBridgeNode::make_dock_pose(double x,
                                                                       double y,
                                                                       double yaw)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw))
  {
    return std::nullopt;
  }
  if (x == 0.0 && y == 0.0 && yaw == 0.0)
  {
    return std::nullopt;  // the template default: no dock calibrated yet
  }
  return DockPose{x, y, yaw};
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
  // HighLevelStatus.gps_quality_percent is misnamed at the source: BT
  // context (behavior_tree_node.cpp's context_->gps_quality) is a 0.0-1.0
  // normalized fraction (std::clamp(..., 0.0f, 1.0f) / gnss_status_utils::
  // NormalizedQuality), and status_snapshot.cpp assigns it straight into
  // this field with no *100 — so a "fully good" fix reads as 1.0, not
  // 100.0. Not something to fix at the source (mowgli_behavior has other
  // consumers of that same field/context member; changing its scale there
  // is a separate, larger change). Scale it here so the MQTT contract's
  // field genuinely means "percent", matching docs/MQTT_CONTROL.md and
  // what an external consumer (e.g. a Home Assistant sensor with
  // PERCENTAGE as its unit) will reasonably assume from the field name.
  const double gps_quality_pct =
      std::max(0.0, std::min(100.0, static_cast<double>(msg.gps_quality_percent) * 100.0));

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
                gps_quality_pct,
                static_cast<double>(msg.battery_percent),
                msg.is_charging ? "true" : "false",
                msg.emergency ? "true" : "false");
  return std::string{buf};
}

std::string MqttBridgeNode::serialise_gps(const sensor_msgs::msg::NavSatFix& msg)
{
  // status.status is STATUS_NO_FIX(-1)/STATUS_FIX(0)/STATUS_SBAS_FIX(1)/
  // STATUS_GBAS_FIX(2); this is a raw NavSatFix relay, not RTK-quality — a
  // consumer wanting Fixed-vs-Float should read <prefix>/rtk_status
  // (serialise_rtk_status() below) instead.
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

namespace
{
const char* FixTypeName(uint8_t fix_type)
{
  using mowgli_interfaces::msg::GnssStatus;
  switch (fix_type)
  {
    case GnssStatus::FIX_TYPE_NO_FIX:
      return "NO_FIX";
    case GnssStatus::FIX_TYPE_GPS_FIX:
      return "GPS_FIX";
    case GnssStatus::FIX_TYPE_RTK_FLOAT:
      return "RTK_FLOAT";
    case GnssStatus::FIX_TYPE_RTK_FIXED:
      return "RTK_FIXED";
    case GnssStatus::FIX_TYPE_DEAD_RECKONING:
      return "DEAD_RECKONING";
    default:
      return "UNKNOWN";
  }
}

const char* RtkModeName(uint8_t rtk_mode)
{
  using mowgli_interfaces::msg::GnssStatus;
  switch (rtk_mode)
  {
    case GnssStatus::RTK_MODE_UNKNOWN:
      return "UNKNOWN";
    case GnssStatus::RTK_MODE_NONE:
      return "NONE";
    case GnssStatus::RTK_MODE_FLOAT:
      return "FLOAT";
    case GnssStatus::RTK_MODE_FIXED:
      return "FIXED";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

std::string MqttBridgeNode::serialise_rtk_status(const mowgli_interfaces::msg::GnssStatus& msg)
{
  // quality_percent here is mowgli_interfaces::gnss_status_utils::
  // HardwareQualityPercent(msg) — deliberately NOT msg.quality_percent
  // directly. That field's own population is backend-dependent per
  // GnssStatus.msg's header comment ("richer value" vs "derived from
  // fix_type") and isn't guaranteed 0-100 the way this already-relied-upon
  // helper is: it's the exact computation hardware_bridge_node.cpp uses for
  // its own gps_quality_ (the GUI's "GPS %" health-check card), so this
  // topic can never disagree with what the robot itself already shows.
  const unsigned quality_percent =
      static_cast<unsigned>(mowgli_interfaces::gnss_status_utils::HardwareQualityPercent(msg));

  char buf[320];
  std::snprintf(buf,
                sizeof(buf),
                "{"
                "\"fix_type\":%u,"
                "\"fix_type_name\":\"%s\","
                "\"rtk_mode\":%u,"
                "\"rtk_mode_name\":\"%s\","
                "\"fix_valid\":%s,"
                "\"quality_percent\":%u"
                "}",
                static_cast<unsigned>(msg.fix_type),
                FixTypeName(msg.fix_type),
                static_cast<unsigned>(msg.rtk_mode),
                RtkModeName(msg.rtk_mode),
                msg.fix_valid ? "true" : "false",
                quality_percent);
  return std::string{buf};
}

std::string MqttBridgeNode::serialise_areas(const std::vector<AreaSummary>& areas)
{
  // Dynamic string building, not the fixed-size snprintf buffers the other
  // serialise_* functions use above — area names are operator-supplied free
  // text of unbounded length, so a fixed buffer could silently truncate the
  // JSON (see the file-level doc comment's note on this).
  std::string json = "[";
  bool first = true;
  for (const auto& area : areas)
  {
    if (!first)
    {
      json += ',';
    }
    first = false;
    json += "{\"index\":";
    json += std::to_string(area.index);
    json += ",\"name\":\"";
    json += json_escape(area.name);
    json += "\"}";
  }
  json += ']';
  return json;
}

std::string MqttBridgeNode::serialise_area_boundaries(
    const std::vector<std::pair<uint32_t, mowgli_interfaces::msg::MapArea>>& areas,
    double datum_lat,
    double datum_lon,
    const std::optional<DockPose>& dock)
{
  // Unbounded-length payload (polygon point counts vary), so this is built
  // with std::string concatenation rather than a fixed snprintf buffer —
  // same precedent as <prefix>/areas (PR #638).
  auto append_point = [](std::string& json, double x, double y)
  {
    char buf[48];
    std::snprintf(buf, sizeof(buf), "[%.3f,%.3f]", x, y);
    json += buf;
  };

  auto append_polygon = [&](std::string& json, const geometry_msgs::msg::Polygon& polygon)
  {
    json += '[';
    bool first_point = true;
    for (const auto& point : polygon.points)
    {
      if (!first_point)
      {
        json += ',';
      }
      first_point = false;
      append_point(json, static_cast<double>(point.x), static_cast<double>(point.y));
    }
    json += ']';
  };

  char header[96];
  std::snprintf(header,
                sizeof(header),
                "{\"datum_lat\":%.8f,\"datum_lon\":%.8f,\"areas\":[",
                datum_lat,
                datum_lon);
  std::string json{header};

  bool first_area = true;
  for (const auto& [index, area] : areas)
  {
    if (!first_area)
    {
      json += ',';
    }
    first_area = false;

    json += "{\"index\":";
    json += std::to_string(index);
    json += ",\"name\":\"";
    json += json_escape(area.name);
    json += "\",\"boundary\":";
    append_polygon(json, area.area);
    json += ",\"obstacles\":[";
    bool first_obstacle = true;
    for (const auto& obstacle : area.obstacles)
    {
      if (!first_obstacle)
      {
        json += ',';
      }
      first_obstacle = false;
      append_polygon(json, obstacle);
    }
    json += "]}";
  }
  json += ']';
  if (dock.has_value())
  {
    char dock_json[96];
    std::snprintf(dock_json,
                  sizeof(dock_json),
                  ",\"dock\":{\"x\":%.3f,\"y\":%.3f,\"yaw\":%.4f}",
                  dock->x,
                  dock->y,
                  dock->yaw);
    json += dock_json;
  }
  json += '}';
  return json;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

std::string MqttBridgeNode::full_topic(const std::string& suffix) const
{
  return topic_prefix_ + "/" + suffix;
}

std::string MqttBridgeNode::home_assistant_device_id(const std::string& topic_prefix)
{
  std::string id{"mowglinext_"};
  for (const unsigned char c : topic_prefix)
  {
    if (std::isalnum(c) || c == '_' || c == '-')
    {
      id += static_cast<char>(c);
    }
    else
    {
      id += '_';
    }
  }
  if (topic_prefix.empty())
  {
    id += "mowgli";
  }
  return id;
}

std::string MqttBridgeNode::home_assistant_discovery_topic(const std::string& topic_prefix)
{
  return "homeassistant/device/" + home_assistant_device_id(topic_prefix) + "/config";
}

std::string MqttBridgeNode::serialise_home_assistant_discovery(const std::string& topic_prefix)
{
  return serialise_home_assistant_discovery(topic_prefix, {});
}

std::string MqttBridgeNode::serialise_home_assistant_discovery(
    const std::string& topic_prefix, const std::vector<AreaSummary>& areas)
{
  const std::string prefix = topic_prefix.empty() ? "mowgli" : topic_prefix;
  const std::string id = home_assistant_device_id(topic_prefix);
  // The GUI normally supplies a simple prefix, but MQTT permits characters
  // that need escaping when the resulting topic is embedded in JSON.
  const std::string available = json_escape(prefix + "/available");
  const std::string high_level = json_escape(prefix + "/high_level_status");
  const std::string status = json_escape(prefix + "/status");
  const std::string power = json_escape(prefix + "/power");
  const std::string emergency = json_escape(prefix + "/emergency");
  const std::string rtk = json_escape(prefix + "/rtk_status");
  const std::string gps = json_escape(prefix + "/gps");
  const std::string command = json_escape(prefix + "/command");
  const std::string start_area = json_escape(prefix + "/start_area");

  const std::string activity_template =
      "{% if value_json.emergency or value_json.state == 0 %}error"
      "{% elif value_json.is_charging or value_json.state_name in "
      "['IDLE_DOCKED','CHARGING','CRITICAL_BATTERY_CHARGING'] %}docked"
      "{% elif value_json.state_name in "
      "['RETURNING_HOME','MOWING_COMPLETE','CRITICAL_BATTERY_DOCKING',"
      "'LOW_BATTERY_DOCKING','RAIN_DETECTED_DOCKING','COVERAGE_FAILED_DOCKING'] %}paused"
      "{% elif value_json.state_name in ['MOWING','MANUAL_MOWING'] %}mowing"
      "{% else %}paused{% endif %}";

  std::string json;
  json.reserve(5000);
  json += "{\"device\":{\"identifiers\":[\"" + id +
          "\"],\"name\":\"MowgliNext\",\"manufacturer\":\"MowgliNext\","
          "\"model\":\"Robot mower\"},"
          "\"origin\":{\"name\":\"MowgliNext MQTT bridge\","
          "\"url\":\"https://github.com/mowglinext/mowglinext\"},"
          "\"availability_topic\":\"" +
          available + "\",\"components\":{";

  auto append_component = [&json](const std::string& key, const std::string& component)
  {
    if (json.back() != '{')
    {
      json += ',';
    }
    json += "\"" + key + "\":" + component;
  };

  append_component("mower",
                   "{\"platform\":\"lawn_mower\",\"name\":null,\"unique_id\":\"" + id +
                       "_mower\",\"activity_state_topic\":\"" + high_level +
                       "\",\"activity_value_template\":\"" + json_escape(activity_template) +
                       "\",\"json_attributes_topic\":\"" + high_level +
                       "\",\"start_mowing_command_topic\":\"" + command +
                       "\",\"start_mowing_command_template\":\"1\",\"pause_command_topic\":\"" +
                       command + "\",\"pause_command_template\":\"8\",\"dock_command_topic\":\"" +
                       command + "\",\"dock_command_template\":\"2\"}");

  auto sensor = [&](const std::string& key,
                    const std::string& name,
                    const std::string& state_topic,
                    const std::string& value_template,
                    const std::string& extra = "")
  {
    append_component(key,
                     "{\"platform\":\"sensor\",\"name\":\"" + name + "\",\"unique_id\":\"" + id +
                         "_" + key + "\",\"state_topic\":\"" + state_topic +
                         "\",\"value_template\":\"" + json_escape(value_template) + "\"" + extra +
                         "}");
  };
  auto binary_sensor = [&](const std::string& key,
                           const std::string& name,
                           const std::string& state_topic,
                           const std::string& value_template,
                           const std::string& device_class = "")
  {
    std::string extra = "\",\"payload_on\":\"ON\",\"payload_off\":\"OFF";
    if (!device_class.empty())
    {
      extra += "\",\"device_class\":\"" + device_class;
    }
    append_component(key,
                     "{\"platform\":\"binary_sensor\",\"name\":\"" + name + "\",\"unique_id\":\"" +
                         id + "_" + key + "\",\"state_topic\":\"" + state_topic +
                         "\",\"value_template\":\"" + json_escape(value_template) + extra + "\"}");
  };

  sensor("battery",
         "Battery",
         high_level,
         "{{ value_json.battery_percent }}",
         ",\"device_class\":\"battery\",\"unit_of_measurement\":\"%\","
         "\"state_class\":\"measurement\"");
  sensor("coverage",
         "Coverage",
         high_level,
         "{{ value_json.coverage_percent }}",
         ",\"unit_of_measurement\":\"%\",\"state_class\":\"measurement\"");
  sensor("gps_quality",
         "GPS quality",
         high_level,
         "{{ value_json.gps_quality_percent }}",
         ",\"unit_of_measurement\":\"%\",\"state_class\":\"measurement\"");
  sensor("rtk_state", "RTK state", rtk, "{{ value_json.rtk_mode_name }}");
  sensor("blade_rpm",
         "Blade speed",
         status,
         "{{ value_json.mower_motor_rpm }}",
         ",\"unit_of_measurement\":\"rpm\",\"state_class\":\"measurement\"");
  sensor("blade_current",
         "Blade current",
         status,
         "{{ value_json.mower_esc_current }}",
         ",\"device_class\":\"current\",\"unit_of_measurement\":\"A\","
         "\"state_class\":\"measurement\"");
  sensor("battery_voltage",
         "Battery voltage",
         power,
         "{{ value_json.v_battery }}",
         ",\"device_class\":\"voltage\",\"unit_of_measurement\":\"V\","
         "\"state_class\":\"measurement\"");
  sensor("charge_current",
         "Charge current",
         power,
         "{{ value_json.charge_current }}",
         ",\"device_class\":\"current\",\"unit_of_measurement\":\"A\","
         "\"state_class\":\"measurement\"");
  binary_sensor("charging",
                "Charging",
                high_level,
                "{{ 'ON' if value_json.is_charging else 'OFF' }}",
                "battery_charging");
  binary_sensor("emergency",
                "Emergency",
                emergency,
                "{{ 'ON' if value_json.active_emergency else 'OFF' }}",
                "problem");
  binary_sensor(
      "rain", "Rain", status, "{{ 'ON' if value_json.rain_detected else 'OFF' }}", "moisture");
  append_component("location",
                   "{\"platform\":\"device_tracker\",\"name\":\"Location\","
                   "\"unique_id\":\"" +
                       id + "_location\",\"json_attributes_topic\":\"" + gps +
                       "\",\"source_type\":\"gps\"}");

  // A select entity would start mowing as soon as its value changed, which is
  // surprising and unsafe for a physical mower. Expose one explicit action
  // button per current mowable area instead. The index and sanitized name are
  // both part of the identity: if an edit reorders positional area indices,
  // Home Assistant replaces the affected button rather than silently keeping
  // an automation bound to a different physical area.
  for (const auto& area : areas)
  {
    const std::string area_name =
        area.name.empty() ? "Area " + std::to_string(area.index) : area.name;
    const std::string component_key =
        "mow_area_" + std::to_string(area.index) + "_" + home_assistant_device_id(area_name);
    append_component(component_key,
                     "{\"platform\":\"button\",\"name\":\"Mow " + json_escape(area_name) +
                         "\",\"unique_id\":\"" + id + "_" + component_key +
                         "\",\"icon\":\"mdi:robot-mower\","
                         "\"command_topic\":\"" +
                         start_area + "\",\"payload_press\":\"" + std::to_string(area.index) +
                         "\"}");
  }

  json += "}}";
  return json;
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
