cmake_minimum_required(VERSION 3.8)
project(mowgli_lidar_pwm)

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
find_package(sensor_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# Executable: lidar_pwm_node
# ---------------------------------------------------------------------------

add_executable(lidar_pwm_node
  src/lidar_pwm_node.cpp
)

target_include_directories(lidar_pwm_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(lidar_pwm_node
  ${rclcpp_TARGETS}
  ${sensor_msgs_TARGETS}
  ${mowgli_interfaces_TARGETS}
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(TARGETS
    lidar_pwm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Same reasoning as sensors/gps/mowgli_gnss_bridge: this package lives
  # under sensors/, never visited by ros2/scripts/format.sh (which globs
  # ros2/src/ only), so it follows ament's uncrustify profile instead of
  # clang-format, and ament_copyright is muted because licensing here is
  # declared with an SPDX identifier, which ament does not recognise.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pwm_controller
    test/test_pwm_controller.cpp
    test/test_sysfs_pwm.cpp
  )
  target_include_directories(test_pwm_controller PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

ament_package()
