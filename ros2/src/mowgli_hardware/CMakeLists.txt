cmake_minimum_required(VERSION 3.8)
project(mowgli_hardware)

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
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

# ---------------------------------------------------------------------------
# Static library: mowgli_hardware_core
#   Contains the protocol logic (COBS, CRC-16, packet framing, serial port).
#   Kept as a separate target so the unit tests can link against it without
#   pulling in ROS2 node dependencies.
# ---------------------------------------------------------------------------

add_library(mowgli_hardware_core STATIC
  src/cobs.cpp
  src/crc16.cpp
  src/serial_port.cpp
  src/packet_handler.cpp
  src/clock_fit.cpp
)

target_include_directories(mowgli_hardware_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# serial_port.cpp uses POSIX pthreads indirectly via the OS serial layer.
target_link_libraries(mowgli_hardware_core pthread)

# clock_fit.cpp needs rclcpp::Time. Promote rclcpp to the static lib's
# public deps so test executables also pick it up transitively.
ament_target_dependencies(mowgli_hardware_core
  rclcpp
)

# ---------------------------------------------------------------------------
# Executable: hardware_bridge_node
# ---------------------------------------------------------------------------

add_executable(hardware_bridge_node
  src/hardware_bridge_node.cpp
  src/odometry_publisher.cpp
)

target_link_libraries(hardware_bridge_node
  mowgli_hardware_core
)

ament_target_dependencies(hardware_bridge_node
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  mowgli_interfaces
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS mowgli_hardware_core hardware_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # Exclude uncrustify — we use clang-format (see .clang-format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # ---- test_cobs ----
  ament_add_gtest(test_cobs
    test/test_cobs.cpp
  )
  target_link_libraries(test_cobs
    mowgli_hardware_core
  )

  # ---- test_packet_handler ----
  ament_add_gtest(test_packet_handler
    test/test_packet_handler.cpp
  )
  target_link_libraries(test_packet_handler
    mowgli_hardware_core
  )

  # ---- test_serial_port ----
  ament_add_gtest(test_serial_port
    test/test_serial_port.cpp
  )
  target_link_libraries(test_serial_port
    mowgli_hardware_core
    util
  )

  # ---- test_protocol ----
  ament_add_gtest(test_protocol
    test/test_protocol.cpp
  )
  target_link_libraries(test_protocol
    mowgli_hardware_core
  )

  # ---- test_clock_fit ----
  ament_add_gtest(test_clock_fit
    test/test_clock_fit.cpp
  )
  target_link_libraries(test_clock_fit
    mowgli_hardware_core
  )

  # ---- test_dig_detector ----
  # Header-only pure logic (dig_detector.hpp): no link against
  # mowgli_hardware_core needed, just the include path.
  ament_add_gtest(test_dig_detector
    test/test_dig_detector.cpp
  )
  target_include_directories(test_dig_detector
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_dig_detector mowgli_interfaces)

  # ---- test_dig_escalation ----
  # Header-only pure logic (dig_escalation.hpp): no link against
  # mowgli_hardware_core needed, just the include path.
  ament_add_gtest(test_dig_escalation
    test/test_dig_escalation.cpp
  )
  target_include_directories(test_dig_escalation
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # ---- test_imu_liveness ----
  # Header-only pure logic (imu_liveness.hpp): no link against
  # mowgli_hardware_core needed, just the include path.
  ament_add_gtest(test_imu_liveness
    test/test_imu_liveness.cpp
  )
  target_include_directories(test_imu_liveness
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # ---- test_blade_gate ----
  # Header-only pure logic (blade_gate.hpp): no link against
  # mowgli_hardware_core needed, just the include path.
  ament_add_gtest(test_blade_gate
    test/test_blade_gate.cpp
  )
  target_include_directories(test_blade_gate
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # ---- test_heartbeat_emergency_policy ----
  # The firmware's pure heartbeat safety policy is compiled here directly so
  # all stop/release combinations are verified without physical hardware.
  ament_add_gtest(test_heartbeat_emergency_policy
    test/test_heartbeat_emergency_policy.cpp
  )
  target_include_directories(test_heartbeat_emergency_policy
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../firmware/stm32/ros_usbnode/include
  )

  # ---- test_cmd_vel_validation ----
  ament_add_gtest(test_cmd_vel_validation
    test/test_cmd_vel_validation.cpp
  )
  target_include_directories(test_cmd_vel_validation
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # ---- test_battery_state_semantics ----
  ament_add_gtest(test_battery_state_semantics
    test/test_battery_state_semantics.cpp
  )
  target_include_directories(test_battery_state_semantics
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # ---- test_timer_period ----
  ament_add_gtest(test_timer_period
    test/test_timer_period.cpp
  )
  target_include_directories(test_timer_period
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # ---- test_hardware_bridge_parameters ----
  # Includes the node implementation to exercise ROS's actual read-only
  # parameter contract while a missing serial device keeps the test hardware-free.
  ament_add_gtest(test_hardware_bridge_parameters
    test/test_hardware_bridge_parameters.cpp
    src/odometry_publisher.cpp
  )
  target_link_libraries(test_hardware_bridge_parameters
    mowgli_hardware_core
  )
  target_include_directories(test_hardware_bridge_parameters
    PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_hardware_bridge_parameters
    rclcpp
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    nav_msgs
    mowgli_interfaces
  )
endif()

# ---------------------------------------------------------------------------
# Package export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(mowgli_hardware_core)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  mowgli_interfaces
)

ament_package()
