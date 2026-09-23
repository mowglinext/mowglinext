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
#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace mowgli_map
{

/// A tf2_ros::TransformListener whose destruction cannot hang.
///
/// tf2_ros's own dedicated-thread listener stops with `executor_->cancel();
/// thread->join();`. rclcpp's Executor::cancel() is LOST when it lands before
/// the thread has entered spin(): spin() then loops forever and join() never
/// returns. A node destroyed right after it was built — every map_server test —
/// can therefore hang until ctest kills it (CI 2026-09-22, test_map_server
/// ***Timeout inside ~MapServerNode → ~TransformListener).
///
/// Same behaviour otherwise: /tf and /tf_static are received on a dedicated
/// thread (so Buffer::lookupTransform timeouts work, setUsingDedicatedThread),
/// on a callback group the node's own executor never spins. The destructor
/// repeats cancel() until the thread has actually left spin().
class SafeTransformListener
{
public:
  SafeTransformListener(tf2_ros::Buffer& buffer, rclcpp::Node& node)
  {
    group_ = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = group_;
    listener_ = std::make_unique<tf2_ros::TransformListener>(buffer,
                                                             node,
                                                             /*spin_thread=*/false,
                                                             tf2_ros::DynamicListenerQoS(),
                                                             tf2_ros::StaticListenerQoS(),
                                                             options,
                                                             options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(group_, node.get_node_base_interface());
    auto exited = std::make_shared<std::promise<void>>();
    exited_ = exited->get_future();
    thread_ = std::thread(
        [executor = executor_, exited]()
        {
          executor->spin();
          exited->set_value();
        });
    buffer.setUsingDedicatedThread(true);
  }

  ~SafeTransformListener()
  {
    // A single cancel() can land before spin() has started and be lost; keep
    // cancelling until the thread has returned from spin().
    do
    {
      executor_->cancel();
    } while (exited_.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready);
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  SafeTransformListener(const SafeTransformListener&) = delete;
  SafeTransformListener& operator=(const SafeTransformListener&) = delete;

private:
  rclcpp::CallbackGroup::SharedPtr group_;
  std::unique_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::future<void> exited_;
  std::thread thread_;
};

}  // namespace mowgli_map
