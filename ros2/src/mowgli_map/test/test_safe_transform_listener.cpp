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
/**
 * @file test_safe_transform_listener.cpp
 * @brief SafeTransformListener never hangs on destruction and still hears /tf.
 *
 * CI 2026-09-22: test_map_server hit the 60 s ctest timeout inside
 * ~MapServerNode → tf2_ros::~TransformListener, whose single executor cancel()
 * is lost when it lands before the listener thread has entered spin().
 */

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include "mowgli_map/safe_transform_listener.hpp"
#include <gtest/gtest.h>

using mowgli_map::SafeTransformListener;

class SafeTransformListenerTest : public ::testing::Test
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

TEST_F(SafeTransformListenerTest, BuildingAndDestroyingItBackToBackNeverHangs)
{
  // Arrange: the worst case for the old listener is destruction immediately
  // after construction, before its thread has had a chance to spin.
  auto node = std::make_shared<rclcpp::Node>("safe_tf_listener_churn");
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Act: build and destroy it repeatedly on a worker, under a watchdog.
  auto done = std::async(std::launch::async,
                         [&]()
                         {
                           for (int i = 0; i < 300; ++i)
                           {
                             SafeTransformListener listener(*buffer, *node);
                           }
                         });

  // Assert
  ASSERT_EQ(done.wait_for(std::chrono::seconds(45)), std::future_status::ready)
      << "a SafeTransformListener destructor hung";
}

TEST_F(SafeTransformListenerTest, StillReceivesPublishedTransforms)
{
  // Arrange
  auto node = std::make_shared<rclcpp::Node>("safe_tf_listener_rx");
  auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  SafeTransformListener listener(*buffer, *node);
  auto pub_node = std::make_shared<rclcpp::Node>("safe_tf_listener_tx");
  auto pub = pub_node->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static",
                                                                  tf2_ros::StaticListenerQoS());
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = pub_node->now();
  tf.header.frame_id = "map";
  tf.child_frame_id = "probe";
  tf.transform.translation.x = 1.5;
  tf.transform.rotation.w = 1.0;
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back(tf);

  // Act: publish until the listener has it (discovery takes a moment).
  bool seen = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (!seen && std::chrono::steady_clock::now() < deadline)
  {
    pub->publish(msg);
    seen = buffer->canTransform("map", "probe", tf2::TimePointZero, tf2::durationFromSec(0.2));
  }

  // Assert
  ASSERT_TRUE(seen) << "the listener never received the published transform";
  EXPECT_DOUBLE_EQ(
      buffer->lookupTransform("map", "probe", tf2::TimePointZero).transform.translation.x, 1.5);
}
