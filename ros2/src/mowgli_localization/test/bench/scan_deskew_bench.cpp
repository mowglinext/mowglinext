// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
// Calls the actual node callback, with a private, isolated ROS domain. The
// access macros affect only this benchmark translation unit, never production.
#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <thread>

#include "mowgli_localization/deskew_history_cursor.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <time.h>

#ifndef DESKEW_NODE_SOURCE
#define DESKEW_NODE_SOURCE "../../src/scan_deskew_node.cpp"
#endif
#define private public
#define main DeskewNodeEntrypoint
#include DESKEW_NODE_SOURCE
#undef main
#undef private

std::uint64_t CpuNs()
{
  timespec t{};
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &t);
  return static_cast<std::uint64_t>(t.tv_sec) * 1000000000ULL + t.tv_nsec;
}

int main(int argc, char** argv)
{
  const std::string scene = argc > 1 ? argv[1] : "rotation";
  const unsigned seed = argc > 2 ? std::stoul(argv[2]) : 42;
  const int repeats = argc > 3 ? std::stoi(argv[3]) : 1000;
  const std::size_t rays = argc > 4 ? std::stoul(argv[4]) : 720;
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<mowgli_localization::ScanDeskewNode>();
  node->imu_max_age_s_ = 1e12;  // Fixed deterministic observation time, no ageing during timing.
  node->wheel_max_age_s_ = 1e12;
  node->linear_comp_enabled_ = scene.starts_with("linear");
  node->reference_ = scene == "start" ? "start" : "end";
  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int k = 0; k <= 50; ++k)
  {
    sensor_msgs::msg::Imu imu;
    double t = 1000.0 + k * 0.01;
    if (scene == "duplicates" && k % 9 == 0 && k > 0)
      t -= 0.01;
    if (scene == "unordered" && k % 11 == 0 && k > 0)
      t -= 0.03;
    imu.header.stamp = rclcpp::Time(static_cast<std::int64_t>(t * 1e9));
    imu.angular_velocity.z = 0.4 + 0.3 * std::sin(k * 0.3) + u(rng) * 0.02;
    node->on_imu(imu);
    nav_msgs::msg::Odometry wheel;
    wheel.header = imu.header;
    wheel.twist.twist.linear.x = 0.2 + u(rng) * 0.05;
    node->on_wheel(wheel);
  }
  if (scene == "missing_imu")
    node->have_imu_ = false;
  if (scene == "stale_imu")
    node->imu_max_age_s_ = 0.0;
  if (scene == "linear_missing_wheel")
    node->have_wheel_ = false;
  if (scene == "linear_stale_wheel")
    node->wheel_max_age_s_ = 0.0;
  if (scene == "empty_history")
    node->imu_buffer_.clear();
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.header.stamp =
      rclcpp::Time(1000, scene == "start" || scene == "reverse_time" ? 400000000 : 500000000);
  scan.angle_min = -3.14159265f;
  scan.angle_increment = 6.2831853f / std::max<std::size_t>(1, rays);
  scan.time_increment = 0.1f / std::max<std::size_t>(1, rays - 1);
  if (scene == "clockwise")
    scan.angle_increment *= -1;
  if (scene == "reverse_time")
    scan.time_increment *= -1;
  if (scene == "zero_time")
    scan.time_increment = 0;
  if (scene == "zero_angle")
    scan.angle_increment = 0;
  scan.scan_time = 0.1f;
  scan.range_min = 0.05f;
  scan.range_max = 20.0f;
  scan.ranges.resize(rays);
  scan.intensities.resize(rays);
  for (std::size_t i = 0; i < rays; ++i)
  {
    scan.ranges[i] = static_cast<float>(2.0 + u(rng));
    scan.intensities[i] = static_cast<float>(100 + i % 31);
    if (scene == "sparse" && i % 3 != 0)
      scan.ranges[i] = INFINITY;
    if (i % 73 == 0)
      scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
    if (i % 79 == 0)
      scan.ranges[i] = 0.01f;
    if (i % 83 == 0)
      scan.ranges[i] = 21.0f;
  }
  if (scene == "short_intensities")
    scan.intensities.resize(rays / 2);
  if (scene == "no_intensities")
    scan.intensities.clear();
  auto listener = std::make_shared<rclcpp::Node>("deskew_benchmark_listener");
  sensor_msgs::msg::LaserScan captured;
  bool received = false;
  auto sub = listener->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan_deskewed",
      rclcpp::SensorDataQoS(),
      [&](sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
      {
        captured = *msg;
        received = true;
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(listener);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (node->pub_->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  if (node->pub_->get_subscription_count() == 0)
    return 2;
  // Warm-up also proves the output subscription can receive; all callback
  // work, including copying and publish, remains inside the timed loop.
  node->on_scan(scan);
  for (int k = 0; k < 100 && !received; ++k)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!received)
    return 3;
  node->interp_misses_ = 0;
  const auto wall_start = std::chrono::steady_clock::now();
  const auto cpu_start = CpuNs();
  for (int k = 0; k < repeats; ++k)
    node->on_scan(scan);
  const auto cpu_ns = CpuNs() - cpu_start;
  const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                           std::chrono::steady_clock::now() - wall_start)
                           .count();
  // Receive a final single output outside the timing window.
  received = false;
  executor.spin_some();
  node->on_scan(scan);
  for (int k = 0; k < 100 && !received; ++k)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!received)
    return 4;
  std::cout << "{\"scene\":\"" << scene << "\",\"seed\":" << seed << ",\"rays\":" << rays
            << ",\"repeats\":" << repeats << ",\"cpu_ns\":" << cpu_ns << ",\"wall_ns\":" << wall_ns
            << ",\"misses\":" << node->interp_misses_ << ",\"published\":" << node->pub_count_
            << ",\"skipped\":" << node->skipped_count_ << ",\"range_bits\":[";
  for (std::size_t i = 0; i < captured.ranges.size(); ++i)
  {
    if (i)
      std::cout << ',';
    std::cout << std::bit_cast<std::uint32_t>(captured.ranges[i]);
  }
  std::cout << "],\"intensity_bits\":[";
  for (std::size_t i = 0; i < captured.intensities.size(); ++i)
  {
    if (i)
      std::cout << ',';
    std::cout << std::bit_cast<std::uint32_t>(captured.intensities[i]);
  }
  std::cout << "]}\n";
  executor.remove_node(listener);
  sub.reset();
  listener.reset();
  node.reset();
  rclcpp::shutdown();
}
