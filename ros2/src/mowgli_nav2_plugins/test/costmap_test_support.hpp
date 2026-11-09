// Copyright 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0

#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include <gtest/gtest.h>

namespace mowgli_nav2_plugins_test
{

/// Make a configured Costmap2DROS safe to clean up.
///
/// Costmap2DROS::on_cleanup() resets its tf2_ros::TransformListener, whose
/// destructor cancel()s and join()s a dedicated SingleThreadedExecutor thread.
/// On Lyrical's rclcpp a cancel() that lands before that thread has entered
/// spin() is lost (spin() resets the flag cancel() cleared), so the join waits
/// forever. A fixture that configures a costmap and cleans it up a few tens of
/// ms later hit that on loaded CI runners as a 60 s ctest timeout right after
/// "Cleaning up" (test_ftc_speed_limit, test_authorized_transit).
///
/// The listener can only deliver a transform from inside spin(), so a probe
/// transform reaching the costmap's buffer proves the thread is spinning and
/// the later cancel() cannot be lost. The probe frame is disconnected from
/// every frame the tests use.
inline void WaitForTfListenerSpinning(nav2_costmap_2d::Costmap2DROS& costmap)
{
  if (!rclcpp::ok())
  {
    return;  // A shut-down context already ends spin(), whatever its order.
  }
  constexpr char kProbeFrame[] = "costmap_test_tf_listener_probe";
  constexpr auto kTimeout = std::chrono::seconds(5);
  const auto buffer = costmap.getTfBuffer();
  if (!buffer)
  {
    return;  // Not configured: no listener to protect.
  }
  auto node = std::make_shared<rclcpp::Node>("costmap_test_tf_listener_probe");
  tf2_ros::StaticTransformBroadcaster broadcaster(node);
  geometry_msgs::msg::TransformStamped probe;
  probe.header.frame_id = "costmap_test_tf_listener_probe_parent";
  probe.child_frame_id = kProbeFrame;
  probe.transform.rotation.w = 1.0;
  broadcaster.sendTransform(probe);  // TRANSIENT_LOCAL: delivered once matched.
  const auto deadline = std::chrono::steady_clock::now() + kTimeout;
  while (!buffer->_frameExists(kProbeFrame) && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  ASSERT_TRUE(buffer->_frameExists(kProbeFrame))
      << "the costmap's TF listener thread never delivered a transform; cleaning the "
         "costmap up now could hang in TransformListener::~TransformListener";
}

}  // namespace mowgli_nav2_plugins_test
