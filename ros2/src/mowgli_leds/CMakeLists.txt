cmake_minimum_required(VERSION 3.8)
project(mowgli_leds)

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
find_package(mowgli_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# Static library: mowgli_leds_core
#
# The ROS node plus the spidev wrapper. The display logic itself
# (led_pattern.hpp, ws2812_encoder.hpp) is header-only and ROS-free so the
# unit tests can link it WITHOUT rclcpp -- keep it that way.
# ---------------------------------------------------------------------------

add_library(mowgli_leds_core STATIC
  src/led_ring_node.cpp
  src/spi_device.cpp
)

target_include_directories(mowgli_leds_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mowgli_leds_core
  rclcpp
  mowgli_interfaces
)

# ---------------------------------------------------------------------------
# Executable: led_ring_node
# ---------------------------------------------------------------------------

add_executable(led_ring_node
  src/led_ring_main.cpp
)

target_link_libraries(led_ring_node
  mowgli_leds_core
)

ament_target_dependencies(led_ring_node
  rclcpp
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
  TARGETS mowgli_leds_core led_ring_node
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
  # Exclude uncrustify -- we use clang-format (see ros2/.clang-format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Both suites cover pure, header-only logic: no rclcpp, no spidev, no clock.
  ament_add_gtest(test_ws2812_encoder
    test/test_ws2812_encoder.cpp
  )
  target_include_directories(test_ws2812_encoder
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_gtest(test_led_pattern
    test/test_led_pattern.cpp
  )
  target_include_directories(test_led_pattern
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

# ---------------------------------------------------------------------------
# Package export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(mowgli_leds_core)
ament_export_dependencies(
  rclcpp
  mowgli_interfaces
)

ament_package()
