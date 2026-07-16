cmake_minimum_required(VERSION 3.22)
project(mowgli_simulation)

# Enforce C++20 globally even though this package contains no compiled sources,
# so that any future C++ additions start from the correct standard.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
# Phase 2.2: ament_cmake_python is needed to install the
# `mowgli_simulation` Python package, which holds the kinematic
# drivetrain plugin loaded by webots_ros2_driver.
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(mowgli_hardware REQUIRED)  # for angular_rate_controller.hpp (sim_actuation)

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
  fake_hardware_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Sim actuation model — firmware static-friction deadband + host angular-rate
# PI (reuses the REAL mowgli_hardware/angular_rate_controller.hpp) so the sim
# reproduces the near-deadband angular limit cycle (mowing weave / dock hunt).
# ---------------------------------------------------------------------------
add_executable(sim_actuation_node
  src/sim_actuation_node.cpp
)

ament_target_dependencies(sim_actuation_node
  rclcpp
  geometry_msgs
  sensor_msgs
  mowgli_hardware
)

install(TARGETS
  sim_actuation_node
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
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Webots driver plugins (Python — loaded in-process by webots_ros2_driver)
# ---------------------------------------------------------------------------
# Installs the `mowgli_simulation/` Python package into PYTHON_INSTALL_DIR
# so that webots_ros2_driver can resolve `mowgli_simulation.<module>.<Class>`
# plugin paths declared in urdf_webots/mowgli_webots.urdf.
ament_python_install_package(${PROJECT_NAME})

# ---------------------------------------------------------------------------
# Install data directories
# ---------------------------------------------------------------------------
install(
  DIRECTORY
    worlds_webots
    urdf_webots
    launch
    config_webots
    protos
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
