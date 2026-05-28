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

  # ---- test_angular_rate_controller ----
  # angular_rate_controller.hpp is header-only; link mowgli_hardware_core only
  # for its public include directories (no ROS deps needed — pure function).
  ament_add_gtest(test_angular_rate_controller
    test/test_angular_rate_controller.cpp
  )
  target_link_libraries(test_angular_rate_controller
    mowgli_hardware_core
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
