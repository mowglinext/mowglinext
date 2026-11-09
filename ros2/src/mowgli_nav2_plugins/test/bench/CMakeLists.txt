# Standalone build of the per-control-cycle benchmark (not part of the package
# build). Needs a sourced ROS environment with Nav2 and an installed
# mowgli_nav2_plugins (see README.md).
cmake_minimum_required(VERSION 3.22)
project(mowgli_nav2_plugins_bench LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mowgli_nav2_plugins REQUIRED)
add_executable(cycle_bench cycle_bench.cpp)
target_link_libraries(cycle_bench
  rclcpp::rclcpp pluginlib::pluginlib ${nav2_core_TARGETS} nav2_costmap_2d::nav2_costmap_2d_core
  nav2_util::nav2_util_core nav2_ros_common::nav2_ros_common ${nav_msgs_TARGETS}
  ${mowgli_nav2_plugins_LIBRARIES})
# mowgli_nav2_plugins uses the classic ament_export_* variables, not targets.
target_include_directories(cycle_bench PRIVATE ${mowgli_nav2_plugins_INCLUDE_DIRS})

find_package(std_msgs REQUIRED)
add_executable(path_transport_bench path_transport_bench.cpp)
target_link_libraries(path_transport_bench rclcpp::rclcpp ${nav_msgs_TARGETS} ${std_msgs_TARGETS})
