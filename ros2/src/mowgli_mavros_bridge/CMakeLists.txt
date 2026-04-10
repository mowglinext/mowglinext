cmake_minimum_required(VERSION 3.08)
project(mowgli_mavros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(mavros_msgs REQUIRED)


add_executable(mavros_hardware_bridge_node
  src/mavros_hardware_bridge_node.cpp
)

target_include_directories(mavros_hardware_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mavros_hardware_bridge_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  mowgli_interfaces
  mavros_msgs
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS mavros_hardware_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()