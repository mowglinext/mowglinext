// Copyright (C) 2026 Mowgli Project
//
// SPDX-License-Identifier: GPL-3.0
//
// End-to-end plan_coverage A/B through two REAL coverage_server processes (run_e2e.sh
// starts them as /base/coverage_server and /cand/coverage_server). Configures and
// activates both, latches a station-free /map_server_node/transit_geometry, then for
// every fixture sends the same goal to both, interleaved. The two serialized results
// must be byte-identical (planning_time_s and header stamps zeroed: the clocks);
// that covers full_path, drivable_subpaths, segments and every count. Prints the
// server's planning_time_s (the whole planCoverage callback) and the client round trip.
//
//   e2e_plan_coverage REPS FIXTURE.json...   (replay schema 1, first polygon)

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <mowgli_interfaces/action/plan_coverage.hpp>
#include <nlohmann/json.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace
{
using PlanCoverage = mowgli_interfaces::action::PlanCoverage;
using Clock = std::chrono::steady_clock;

geometry_msgs::msg::Polygon polygon(const nlohmann::json& ring)
{
  geometry_msgs::msg::Polygon out;
  for (const auto& p : ring)
  {
    geometry_msgs::msg::Point32 q;
    q.x = p.at(0).get<float>();
    q.y = p.at(1).get<float>();
    out.points.push_back(q);
  }
  return out;
}

struct Server
{
  std::string ns;
  rclcpp_action::Client<PlanCoverage>::SharedPtr action;
};

Server activate(const rclcpp::Node::SharedPtr& node, const std::string& ns)
{
  auto lifecycle = node->create_client<lifecycle_msgs::srv::ChangeState>(
      "/" + ns + "/coverage_server/change_state");
  if (!lifecycle->wait_for_service(std::chrono::seconds(60)))
    throw std::runtime_error(ns + ": no lifecycle service");
  for (const auto id : {lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE})
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = id;
    auto future = lifecycle->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(60)) !=
            rclcpp::FutureReturnCode::SUCCESS ||
        !future.get()->success)
      throw std::runtime_error(ns + ": lifecycle transition failed");
  }
  auto action = rclcpp_action::create_client<PlanCoverage>(node, "/" + ns + "/plan_coverage");
  if (!action->wait_for_action_server(std::chrono::seconds(60)))
    throw std::runtime_error(ns + ": no action server");
  return {ns, action};
}

PlanCoverage::Result plan(const rclcpp::Node::SharedPtr& node,
                          const Server& server,
                          const PlanCoverage::Goal& goal,
                          double& wall_ms)
{
  const auto t0 = Clock::now();
  auto sent = server.action->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(node, sent, std::chrono::seconds(60)) !=
          rclcpp::FutureReturnCode::SUCCESS ||
      !sent.get())
    throw std::runtime_error(server.ns + ": goal rejected");
  auto result = server.action->async_get_result(sent.get());
  if (rclcpp::spin_until_future_complete(node, result, std::chrono::hours(1)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    throw std::runtime_error(server.ns + ": no result");
  wall_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
  return *result.get().result;
}

std::vector<uint8_t> bytes(PlanCoverage::Result result)
{
  // The clocks: planning_time_s and every header stamp (now() at planning).
  result.planning_time_s = 0.0;
  const auto unstamp = [](nav_msgs::msg::Path& path)
  {
    path.header.stamp = builtin_interfaces::msg::Time();
    for (auto& pose : path.poses)
      pose.header.stamp = builtin_interfaces::msg::Time();
  };
  unstamp(result.full_path);
  std::for_each(result.drivable_subpaths.begin(), result.drivable_subpaths.end(), unstamp);
  std::for_each(result.segments.begin(), result.segments.end(), unstamp);
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<PlanCoverage::Result>().serialize_message(&result, &serialized);
  const auto& raw = serialized.get_rcl_serialized_message();
  return {raw.buffer, raw.buffer + raw.buffer_length};
}

double median(std::vector<double> v)
{
  std::sort(v.begin(), v.end());
  return v.size() % 2 ? v[v.size() / 2] : 0.5 * (v[v.size() / 2 - 1] + v[v.size() / 2]);
}
}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  if (args.size() < 3)
  {
    std::fprintf(stderr, "usage: e2e_plan_coverage REPS FIXTURE.json...\n");
    return 2;
  }
  int status = 0;
  try
  {
    const int reps = std::stoi(args[1]);
    auto node = std::make_shared<rclcpp::Node>("coverage_e2e_driver");
    auto geometry = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/map_server_node/transit_geometry", rclcpp::QoS(1).transient_local());
    visualization_msgs::msg::MarkerArray snapshot;
    snapshot.markers.emplace_back().ns = "e2e_no_station";  // non-empty, no dock body
    geometry->publish(snapshot);
    const std::vector<Server> servers{activate(node, "base"), activate(node, "cand")};
    std::printf("%-20s %9s %5s %12s %12s %8s %12s %12s\n",
                "fixture",
                "poses",
                "sub",
                "base_ms",
                "cand_ms",
                "speedup",
                "base_rt_ms",
                "cand_rt_ms");
    for (std::size_t f = 2; f < args.size(); ++f)
    {
      std::ifstream file(args[f]);
      const auto fixture = nlohmann::json::parse(file);
      const auto& rings = fixture.at("polygons").at(0);
      PlanCoverage::Goal goal;
      goal.outer_boundary = polygon(rings.at(0));
      for (std::size_t r = 1; r < rings.size(); ++r)
        goal.obstacles.push_back(polygon(rings.at(r)));
      goal.mow_angle_deg = -1.0;
      std::vector<double> server_ms[2], wall_ms[2];
      std::vector<uint8_t> reference[2];
      std::size_t poses = 0, subpaths = 0;
      for (int rep = 0; rep < reps; ++rep)
      {
        for (int s = 0; s < 2; ++s)
        {
          double wall = 0.0;
          const auto result = plan(node, servers[s], goal, wall);
          if (!result.success)
            throw std::runtime_error(args[f] + "/" + servers[s].ns + ": " + result.message);
          server_ms[s].push_back(1e3 * result.planning_time_s);
          wall_ms[s].push_back(wall);
          auto b = bytes(result);
          if (reference[s].empty())
            reference[s] = std::move(b);
          else if (b != reference[s])
            throw std::runtime_error(args[f] + "/" + servers[s].ns + ": not deterministic");
          poses = result.full_path.poses.size();
          subpaths = result.drivable_subpaths.size();
        }
      }
      if (reference[0] != reference[1])
      {
        std::printf("%s: base and cand results DIFFER\n", args[f].c_str());
        status = 1;
        break;
      }
      const std::string name = args[f].substr(args[f].find_last_of('/') + 1);
      std::printf("%-20s %9zu %5zu %12.1f %12.1f %7.1fx %12.1f %12.1f  identical(%zu B)\n",
                  name.substr(0, name.size() - 5).c_str(),
                  poses,
                  subpaths,
                  median(server_ms[0]),
                  median(server_ms[1]),
                  median(server_ms[0]) / median(server_ms[1]),
                  median(wall_ms[0]),
                  median(wall_ms[1]),
                  reference[0].size());
      std::fflush(stdout);
    }
  }
  catch (const std::exception& e)
  {
    std::fprintf(stderr, "%s\n", e.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
