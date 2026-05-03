cmake_minimum_required(VERSION 3.8)
project(mowgli_localization)

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

find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(nav_msgs           REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(std_srvs           REQUIRED)
find_package(tf2                REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(mowgli_interfaces  REQUIRED)
find_package(yaml-cpp           REQUIRED)

# ---------------------------------------------------------------------------
# Common include directory macro
# ---------------------------------------------------------------------------

set(INCLUDE_DIR
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ---------------------------------------------------------------------------
# Executable: wheel_odometry_node
# ---------------------------------------------------------------------------

add_executable(wheel_odometry_node
  src/wheel_odometry_node.cpp
)

target_include_directories(wheel_odometry_node PUBLIC ${INCLUDE_DIR})

ament_target_dependencies(wheel_odometry_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mowgli_interfaces
)

# ---------------------------------------------------------------------------
# Executable: navsat_to_absolute_pose_node
# ---------------------------------------------------------------------------

add_executable(navsat_to_absolute_pose_node
  src/navsat_to_absolute_pose_node.cpp
)

target_include_directories(navsat_to_absolute_pose_node PUBLIC ${INCLUDE_DIR})

ament_target_dependencies(navsat_to_absolute_pose_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  mowgli_interfaces
)

# ---------------------------------------------------------------------------
# Executable: localization_monitor_node
# ---------------------------------------------------------------------------

add_executable(localization_monitor_node
  src/localization_monitor_node.cpp
)

target_include_directories(localization_monitor_node PUBLIC ${INCLUDE_DIR})

ament_target_dependencies(localization_monitor_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  mowgli_interfaces
)

# ---------------------------------------------------------------------------
# Executable: cog_to_imu (ported from cog_to_imu.py)
# ---------------------------------------------------------------------------

add_executable(cog_to_imu
  src/cog_to_imu_node.cpp
)

ament_target_dependencies(cog_to_imu
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
)

target_link_libraries(cog_to_imu yaml-cpp)

# ---------------------------------------------------------------------------
# Executable: mag_yaw_publisher (ported from mag_yaw_publisher.py)
# ---------------------------------------------------------------------------

add_executable(mag_yaw_publisher
  src/mag_yaw_publisher_node.cpp
)

ament_target_dependencies(mag_yaw_publisher
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

target_link_libraries(mag_yaw_publisher yaml-cpp)

# ---------------------------------------------------------------------------
# Executable: dock_yaw_to_set_pose (ported from dock_yaw_to_set_pose.py)
# ---------------------------------------------------------------------------

add_executable(dock_yaw_to_set_pose
  src/dock_yaw_to_set_pose_node.cpp
)

ament_target_dependencies(dock_yaw_to_set_pose
  rclcpp
  geometry_msgs
  sensor_msgs
  mowgli_interfaces
)

target_link_libraries(dock_yaw_to_set_pose yaml-cpp)

# ---------------------------------------------------------------------------
# Executable: calibrate_imu_yaw_node (ported from calibrate_imu_yaw_node.py)
# ---------------------------------------------------------------------------

add_executable(calibrate_imu_yaw_node
  src/calibrate_imu_yaw_node.cpp
)

ament_target_dependencies(calibrate_imu_yaw_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  mowgli_interfaces
)

target_link_libraries(calibrate_imu_yaw_node yaml-cpp)

# ---------------------------------------------------------------------------
# Executable: costmap_scan_filter_node (conditional radial blank for the
# local_costmap obstacle_layer — masks the dock while charging without
# opening a 0.10–0.65 m blind ring during normal mowing)
# ---------------------------------------------------------------------------

add_executable(costmap_scan_filter_node
  src/costmap_scan_filter_node.cpp
)

ament_target_dependencies(costmap_scan_filter_node
  rclcpp
  sensor_msgs
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
  TARGETS
    wheel_odometry_node
    navsat_to_absolute_pose_node
    localization_monitor_node
    cog_to_imu
    mag_yaw_publisher
    dock_yaw_to_set_pose
    calibrate_imu_yaw_node
    costmap_scan_filter_node
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
  find_package(ament_lint_auto      REQUIRED)
  # Exclude uncrustify — we use clang-format (see .clang-format).
  set(ament_cmake_uncrustify_FOUND TRUE)
  # ament_copyright requires the full GPL license text; the project uses
  # SPDX identifiers (`SPDX-License-Identifier: GPL-3.0-or-later`) which
  # ament doesn't recognise as a known license — skip the lint.
  set(ament_cmake_copyright_FOUND TRUE)
  # cpplint enforces Google C++ style; clang-format with the project's
  # .clang-format covers the same ground with our own rules.
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # ---- test_wheel_odometry ----
  # The kinematics test is pure C++ with no ROS2 dependencies; link only gtest.
  ament_add_gtest(test_wheel_odometry
    test/test_wheel_odometry.cpp
  )

  target_include_directories(test_wheel_odometry PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  # ---- test_costmap_scan_filter ----
  # Re-implements the static helper so we don't drag rclcpp/main symbols
  # into the test binary; the helper is small enough to keep in sync by
  # eye and a regression there will trigger a test failure here.
  ament_add_gtest(test_costmap_scan_filter
    test/test_costmap_scan_filter.cpp
  )
  ament_target_dependencies(test_costmap_scan_filter
    sensor_msgs
  )
endif()

# ---------------------------------------------------------------------------
# Package export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mowgli_interfaces
)

ament_package()
