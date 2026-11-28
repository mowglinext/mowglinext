# Standalone build of the end-to-end plan_coverage A/B client (not part of the
# package build). Needs a sourced ROS environment with mowgli_interfaces.
cmake_minimum_required(VERSION 3.22)
project(coverage_e2e LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
add_executable(e2e_plan_coverage e2e_plan_coverage.cpp)
target_link_libraries(e2e_plan_coverage
  rclcpp::rclcpp rclcpp_action::rclcpp_action
  ${lifecycle_msgs_TARGETS} ${visualization_msgs_TARGETS} ${mowgli_interfaces_TARGETS}
  nlohmann_json::nlohmann_json)
