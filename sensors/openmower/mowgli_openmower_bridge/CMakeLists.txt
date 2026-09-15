cmake_minimum_required(VERSION 3.8)
project(mowgli_openmower_bridge)

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)
# Exports mowgli_hardware::mowgli_hardware_core (COBS, CRC-16, serial port,
# packet framing). The OpenMower LowLevel board and the xESC 2040 use the very
# same 0x00-delimited COBS + CRC-16/CCITT-FALSE framing as the Mowgli STM32.
find_package(mowgli_hardware REQUIRED)

# ---------------------------------------------------------------------------
# Static library: pure protocol / control logic (no ROS node dependencies)
# ---------------------------------------------------------------------------

add_library(mowgli_openmower_bridge_core STATIC
  src/vesc_protocol.cpp
  src/xesc_2040_link.cpp
  src/xesc_mini_link.cpp
)

target_include_directories(mowgli_openmower_bridge_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mowgli_openmower_bridge_core
  mowgli_hardware::mowgli_hardware_core
)

# ---------------------------------------------------------------------------
# Executable: openmower_bridge_node
# ---------------------------------------------------------------------------

add_executable(openmower_bridge_node
  src/openmower_bridge_node.cpp
  src/openmower_bridge_lowlevel.cpp
  src/openmower_bridge_actuation.cpp
  src/main.cpp
)

target_link_libraries(openmower_bridge_node
  mowgli_openmower_bridge_core
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${mowgli_interfaces_TARGETS}
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(TARGETS openmower_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # SPDX identifiers instead of full license text (as in every ros2/src
  # package); clang-format owns the style, not uncrustify/cpplint.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # The launch file follows the ros2/src launch-file style (double quotes,
  # black-like layout), which ament's flake8/pep257 profiles reject.
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  foreach(test_name
      test_vesc_protocol
      test_xesc_2040_protocol
      test_wheel_velocity_controller
      test_xesc_odometry
      test_emergency_tracker
      test_lowlevel_config
      test_imu_bias_estimator
      test_blade_policy)
    ament_add_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} mowgli_openmower_bridge_core)
  endforeach()
endif()

ament_package()
