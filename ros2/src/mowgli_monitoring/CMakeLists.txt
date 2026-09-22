cmake_minimum_required(VERSION 3.8)
project(mowgli_monitoring)

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
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# Optional: libmosquitto
# Searched via pkg-config first (preferred on Ubuntu), then by raw library
# name.  If found, the MOWGLI_HAS_MOSQUITTO compile definition enables the
# MosquittoMqttClient implementation in mqtt_bridge_node.cpp.
# ---------------------------------------------------------------------------

option(MOWGLI_REQUIRE_MOSQUITTO
  "Fail the configure step when libmosquitto is missing instead of falling back to StubMqttClient (CI and the release image set this so the shipped client is what gets compiled)"
  OFF)

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(MOSQUITTO QUIET libmosquitto)
endif()

if(NOT MOSQUITTO_FOUND)
  find_library(MOSQUITTO_LIB mosquitto)
  if(MOSQUITTO_LIB)
    set(MOSQUITTO_FOUND TRUE)
    set(MOSQUITTO_LIBRARIES ${MOSQUITTO_LIB})
    set(MOSQUITTO_INCLUDE_DIRS "")
  endif()
endif()

if(MOSQUITTO_FOUND)
  message(STATUS "libmosquitto found — MosquittoMqttClient will be compiled.")
elseif(MOWGLI_REQUIRE_MOSQUITTO)
  message(FATAL_ERROR
    "libmosquitto NOT found and MOWGLI_REQUIRE_MOSQUITTO=ON — the real "
    "MosquittoMqttClient would be silently replaced by StubMqttClient and "
    "never compiled. Install with: sudo apt-get install libmosquitto-dev")
else()
  message(STATUS
    "libmosquitto NOT found — mqtt_bridge_node will use StubMqttClient. "
    "Install with: sudo apt-get install libmosquitto-dev")
endif()

# ---------------------------------------------------------------------------
# Static library: mowgli_monitoring_core
# Contains all node logic; linked by both executables and the test binary.
# ---------------------------------------------------------------------------

add_library(mowgli_monitoring_core STATIC
  src/diagnostics_node.cpp
  src/mqtt_bridge_node.cpp
)

target_include_directories(mowgli_monitoring_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mowgli_monitoring_core
  ${rclcpp_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${mowgli_interfaces_TARGETS}
)

if(MOSQUITTO_FOUND)
  target_compile_definitions(mowgli_monitoring_core PUBLIC MOWGLI_HAS_MOSQUITTO)
  target_include_directories(mowgli_monitoring_core
    PUBLIC ${MOSQUITTO_INCLUDE_DIRS})
  target_link_libraries(mowgli_monitoring_core ${MOSQUITTO_LIBRARIES})
endif()

# ---------------------------------------------------------------------------
# Executable: diagnostics_node
# ---------------------------------------------------------------------------

add_executable(diagnostics_node
  src/diagnostics_main.cpp
)

target_link_libraries(diagnostics_node
  mowgli_monitoring_core
)

target_link_libraries(diagnostics_node
  ${rclcpp_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${mowgli_interfaces_TARGETS}
)

# ---------------------------------------------------------------------------
# Executable: mqtt_bridge_node
# ---------------------------------------------------------------------------

add_executable(mqtt_bridge_node
  src/mqtt_bridge_main.cpp
)

target_link_libraries(mqtt_bridge_node
  mowgli_monitoring_core
)

target_link_libraries(mqtt_bridge_node
  ${rclcpp_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${mowgli_interfaces_TARGETS}
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS mowgli_monitoring_core diagnostics_node mqtt_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
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
  # Every test that brings up rclcpp runs through ament_add_ros_isolated_gtest:
  # colcon tests packages in parallel, and on one shared DDS domain the test
  # processes discover each other's nodes (docs/claude/testing-ci.md).
  find_package(ament_cmake_ros REQUIRED)

  ament_add_ros_isolated_gtest(test_diagnostics
    test/test_diagnostics.cpp
  )

  target_link_libraries(test_diagnostics
    mowgli_monitoring_core
  )

  target_link_libraries(test_diagnostics
    ${rclcpp_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${nav2_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${mowgli_interfaces_TARGETS}
  )

  # ROS-free: exercises the shared reduction header on its own.
  ament_add_gtest(test_path_tracking_stats
    test/test_path_tracking_stats.cpp
  )

  target_link_libraries(test_path_tracking_stats
    ${mowgli_interfaces_TARGETS}
  )

  ament_add_ros_isolated_gtest(test_mqtt_bridge
    test/test_mqtt_bridge.cpp
  )

  target_link_libraries(test_mqtt_bridge
    mowgli_monitoring_core
  )

  target_link_libraries(test_mqtt_bridge
    ${rclcpp_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${nav2_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${mowgli_interfaces_TARGETS}
  )
endif()

# ---------------------------------------------------------------------------
# Package export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(mowgli_monitoring_core)
ament_export_dependencies(
  rclcpp
  diagnostic_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  mowgli_interfaces
)

ament_package()
