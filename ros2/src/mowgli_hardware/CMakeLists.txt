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
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# Static library: mowgli_hardware_core
#   Protocol logic (COBS, CRC-16, packet framing, serial port, clock fit).
#   Separate target so the unit tests can link it without ROS2 node deps.
# ---------------------------------------------------------------------------

add_library(mowgli_hardware_core STATIC
  src/cobs.cpp
  src/crc16.cpp
  src/serial_port.cpp
  src/packet_handler.cpp
  src/clock_fit.cpp
)

# -fPIC so the shared plugin library can link this static archive.
set_target_properties(mowgli_hardware_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

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
# Plugin library: mowgli_system_interface
#   ros2_control SystemInterface for the STM32 drive base. Contains BOTH the
#   hardware plugin AND the embedded HardwareBridgeNode serial-comms core (it
#   owns the serial link; the controller_manager loads this one .so). The old
#   standalone hardware_bridge_node executable is gone — the motor path moved
#   to ros2_control (diff_drive_controller + chained pid_controller).
# ---------------------------------------------------------------------------

add_library(mowgli_system_interface SHARED
  src/mowgli_system_interface.cpp
  src/hardware_bridge_node.cpp
)

target_include_directories(mowgli_system_interface
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(mowgli_system_interface
  mowgli_hardware_core
)

ament_target_dependencies(mowgli_system_interface
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  nav_msgs
  mowgli_interfaces
)

# Register the SystemInterface with pluginlib so controller_manager can load it.
pluginlib_export_plugin_description_file(hardware_interface mowgli_hardware_plugin.xml)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS mowgli_hardware_core mowgli_system_interface
  EXPORT export_mowgli_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES mowgli_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # ---- test_cobs ----
  ament_add_gtest(test_cobs test/test_cobs.cpp)
  target_link_libraries(test_cobs mowgli_hardware_core)

  # ---- test_packet_handler ----
  ament_add_gtest(test_packet_handler test/test_packet_handler.cpp)
  target_link_libraries(test_packet_handler mowgli_hardware_core)

  # ---- test_protocol ----
  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol mowgli_hardware_core)

  # ---- test_clock_fit ----
  ament_add_gtest(test_clock_fit test/test_clock_fit.cpp)
  target_link_libraries(test_clock_fit mowgli_hardware_core)

  # ---- test_velocity_pwm ----
  # velocity_pwm.hpp is header-only (pure function); link mowgli_hardware_core
  # only for its public include directories.
  ament_add_gtest(test_velocity_pwm test/test_velocity_pwm.cpp)
  target_link_libraries(test_velocity_pwm mowgli_hardware_core)

  # ---- test_motor_speed_velocity ----
  # motor_speed_velocity.hpp is header-only (pure auto-scale function).
  ament_add_gtest(test_motor_speed_velocity test/test_motor_speed_velocity.cpp)
  target_link_libraries(test_motor_speed_velocity mowgli_hardware_core)
endif()

# ---------------------------------------------------------------------------
# Package export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(mowgli_hardware_core)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  mowgli_interfaces
)

ament_package()
