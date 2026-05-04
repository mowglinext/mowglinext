cmake_minimum_required(VERSION 3.22)
project(mowgli_simulation)

# Enforce C++20 globally even though this package contains no compiled sources,
# so that any future C++ additions start from the correct standard.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# GPS degradation simulator node
# ---------------------------------------------------------------------------
add_executable(gps_degradation_sim_node
  src/gps_degradation_sim_node.cpp
)

target_include_directories(gps_degradation_sim_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gps_degradation_sim_node
  rclcpp
  geometry_msgs
  std_msgs
)

# ---------------------------------------------------------------------------
# NavSat → Pose converter for simulation GPS pipeline
# ---------------------------------------------------------------------------
add_executable(navsat_to_pose_node
  src/navsat_to_pose_node.cpp
)

target_include_directories(navsat_to_pose_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(navsat_to_pose_node
  rclcpp
  geometry_msgs
  sensor_msgs
)

# ---------------------------------------------------------------------------
# Fake hardware bridge for simulation
# ---------------------------------------------------------------------------
add_executable(fake_hardware_bridge_node
  src/fake_hardware_bridge_node.cpp
)

target_include_directories(fake_hardware_bridge_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fake_hardware_bridge_node
  rclcpp
  mowgli_interfaces
  nav_msgs
  sensor_msgs
)

install(TARGETS
  gps_degradation_sim_node
  navsat_to_pose_node
  fake_hardware_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Python helpers (sim-only)
# ---------------------------------------------------------------------------
install(
  PROGRAMS
    scripts/sim_imu_noise.py
    scripts/sim_navsat_rtk_fix.py
    scripts/sim_wheel_slip.py
    scripts/sim_cmd_vel_unstamp.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Install data directories
# ---------------------------------------------------------------------------
install(
  DIRECTORY
    worlds
    models
    launch
    config
    rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # Exclude uncrustify — we use clang-format (see .clang-format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
