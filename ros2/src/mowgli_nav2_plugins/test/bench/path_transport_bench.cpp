// Copyright (C) 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// What moving one dense coverage path costs, by pose count:
//   path_transport_bench serialize    deep copy / CDR serialize / deserialize, in-process
//   path_transport_bench sub          receiver for `pub` (acks each path)
//   path_transport_bench pub [reps]   publisher in a second process: how long publish()
//                                     blocks, and until the receiver has it — the stack's
//                                     plan QoS (reliable, transient_local, depth 1)
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/u_int64.hpp>

namespace
{
using Clock = std::chrono::steady_clock;
const std::vector<std::size_t> kSizes{10000, 50000, 100000, 400000};

nav_msgs::msg::Path MakePath(std::size_t n)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses.resize(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    path.poses[i].header.frame_id = "map";
    path.poses[i].pose.position.x = 0.03 * static_cast<double>(i);
    path.poses[i].pose.orientation.w = 1.0;
  }
  return path;
}

double Ms(Clock::time_point a, Clock::time_point b)
{
  return std::chrono::duration<double, std::milli>(b - a).count();
}

double Median(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  return v[v.size() / 2];
}

int Serialize()
{
  rclcpp::Serialization<nav_msgs::msg::Path> ser;
  std::printf("%8s %10s %10s %10s %10s %10s\n",
              "poses",
              "cdr_bytes",
              "mem_MB",
              "copy_ms",
              "ser_ms",
              "deser_ms");
  for (const std::size_t n : kSizes)
  {
    const auto path = MakePath(n);
    std::vector<double> c, s, d;
    std::size_t bytes = 0;
    for (int r = 0; r < 7; ++r)
    {
      const auto t0 = Clock::now();
      const nav_msgs::msg::Path copy = path;
      const auto t1 = Clock::now();
      rclcpp::SerializedMessage m;
      ser.serialize_message(&path, &m);
      const auto t2 = Clock::now();
      nav_msgs::msg::Path out;
      ser.deserialize_message(&m, &out);
      const auto t3 = Clock::now();
      if (out.poses.size() != n || copy.poses.size() != n)
        return 1;
      bytes = m.size();
      c.push_back(Ms(t0, t1));
      s.push_back(Ms(t1, t2));
      d.push_back(Ms(t2, t3));
    }
    std::printf("%8zu %10zu %10.1f %10.2f %10.2f %10.2f\n",
                n,
                bytes,
                static_cast<double>(n * sizeof(geometry_msgs::msg::PoseStamped)) / 1e6,
                Median(c),
                Median(s),
                Median(d));
  }
  return 0;
}

int Sub()
{
  auto node = std::make_shared<rclcpp::Node>("path_transport_sub");
  auto ack = node->create_publisher<std_msgs::msg::UInt64>("/path_transport_bench/ack", 10);
  auto sub =
      node->create_subscription<nav_msgs::msg::Path>("/path_transport_bench/path",
                                                     rclcpp::QoS(1).transient_local(),
                                                     [&](nav_msgs::msg::Path::ConstSharedPtr m)
                                                     {
                                                       std_msgs::msg::UInt64 a;
                                                       a.data = m->poses.size();
                                                       ack->publish(a);
                                                     });
  rclcpp::spin(node);
  return 0;
}

int Pub(int reps)
{
  auto node = std::make_shared<rclcpp::Node>("path_transport_pub");
  auto pub = node->create_publisher<nav_msgs::msg::Path>("/path_transport_bench/path",
                                                         rclcpp::QoS(1).transient_local());
  std::atomic<uint64_t> acked{0};
  auto sub =
      node->create_subscription<std_msgs::msg::UInt64>("/path_transport_bench/ack",
                                                       10,
                                                       [&](std_msgs::msg::UInt64::ConstSharedPtr m)
                                                       {
                                                         acked = m->data;
                                                       });
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(node);
  std::thread spinner(
      [&]
      {
        ex.spin();
      });
  while (pub->get_subscription_count() == 0 || sub->get_publisher_count() == 0)
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  std::printf("%8s %12s %12s\n", "poses", "publish_ms", "delivered_ms");
  for (const std::size_t n : kSizes)
  {
    std::vector<double> p, d;
    for (int r = 0; r < reps; ++r)
    {
      const auto path = MakePath(n + static_cast<std::size_t>(r));  // the ack names the rep
      const auto t0 = Clock::now();
      pub->publish(path);
      const auto t1 = Clock::now();
      while (acked.load() != path.poses.size())
        std::this_thread::sleep_for(std::chrono::microseconds(200));
      p.push_back(Ms(t0, t1));
      d.push_back(Ms(t0, Clock::now()));
    }
    std::printf("%8zu %12.1f %12.1f\n", n, Median(p), Median(d));
    std::fflush(stdout);
  }
  ex.cancel();
  spinner.join();
  return 0;
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  const std::string mode = args.size() > 1 ? args[1] : "serialize";
  int rc = 2;
  if (mode == "serialize")
    rc = Serialize();
  else if (mode == "sub")
    rc = Sub();
  else if (mode == "pub")
    rc = Pub(args.size() > 2 ? std::atoi(args[2].c_str()) : 5);
  else
    std::fprintf(stderr, "usage: path_transport_bench [serialize|sub|pub [reps]]\n");
  rclcpp::shutdown();
  return rc;
}
