cmake_minimum_required(VERSION 3.8)
project(mowgli_nav2_plugins)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# Shared library: mowgli_nav2_plugins
# ---------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/oscillation_detector.cpp
  src/obstacle_deviation.cpp
  src/ftc_lattice_solver.cpp
  src/ftc_turn_fallback.cpp
  src/ftc_controller.cpp
  src/ftc_controller_turn_fallback.cpp
  src/ftc_controller_plugin.cpp
  src/path_progress_goal_checker.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_include_directories(${PROJECT_NAME}
  SYSTEM PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${rclcpp_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${nav2_core_TARGETS}
  ${nav2_costmap_2d_TARGETS}
  ${nav2_util_TARGETS}
  ${nav2_ros_common_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${pluginlib_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  ${tf2_eigen_TARGETS}
  ${visualization_msgs_TARGETS}
  ${mowgli_interfaces_TARGETS}
)

# pluginlib needs the export macro defined.
pluginlib_export_plugin_description_file(nav2_core ftc_controller_plugin.xml)
pluginlib_export_plugin_description_file(nav2_core goal_checker_plugin.xml)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES ftc_controller_plugin.xml goal_checker_plugin.xml
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
  # Exclude uncrustify — we use clang-format (see .clang-format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  # Every test that brings up rclcpp runs through ament_add_ros_isolated_gtest:
  # colcon tests packages in parallel, and on one shared DDS domain the test
  # processes discover each other's nodes (docs/claude/testing-ci.md).
  find_package(ament_cmake_ros REQUIRED)

  ament_add_ros_isolated_gtest(test_lyrical_goal_checker test/test_lyrical_goal_checker.cpp)
  target_link_libraries(test_lyrical_goal_checker ${PROJECT_NAME})

  ament_add_ros_isolated_gtest(test_path_progress_end_approach
    test/test_path_progress_end_approach.cpp
  )
  target_link_libraries(test_path_progress_end_approach ${PROJECT_NAME})

  ament_add_gtest(test_oscillation_detector
    test/test_oscillation_detector.cpp
    src/oscillation_detector.cpp
  )

  target_include_directories(test_oscillation_detector
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_obstacle_deviation
    test/test_obstacle_deviation.cpp
    src/obstacle_deviation.cpp
  )

  target_include_directories(test_obstacle_deviation
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_obstacle_deviation
    ${geometry_msgs_TARGETS}
    ${nav2_costmap_2d_TARGETS}
    ${tf2_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
  )

  ament_add_gtest(test_ftc_start_index
    test/test_ftc_start_index.cpp
  )

  target_include_directories(test_ftc_start_index
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_ftc_stall
    test/test_ftc_stall.cpp
  )

  target_include_directories(test_ftc_stall
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_ftc_blade_load
    test/test_ftc_blade_load.cpp
  )

  target_include_directories(test_ftc_blade_load
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_ftc_reverse_escape
    test/test_ftc_reverse_escape.cpp
  )

  target_include_directories(test_ftc_reverse_escape
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_ftc_obstacle_wait
    test/test_ftc_obstacle_wait.cpp
  )

  target_include_directories(test_ftc_obstacle_wait
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_ftc_carrot_lead
    test/test_ftc_carrot_lead.cpp
  )

  target_include_directories(test_ftc_carrot_lead
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_ftc_resync
    test/test_ftc_resync.cpp
  )

  target_include_directories(test_ftc_resync
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  ament_add_gtest(test_ftc_offset_lattice
    test/test_ftc_offset_lattice.cpp
  )

  target_include_directories(test_ftc_offset_lattice
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  # In-place pivots at the coverage planner's explicit corners (ftc_pivot.hpp):
  # corner detection, leg bounds, arrival/alignment, sweep probes. ROS-free;
  # needs only the header-only corner contract from mowgli_interfaces.
  ament_add_gtest(test_ftc_pivot
    test/test_ftc_pivot.cpp
  )

  target_include_directories(test_ftc_pivot
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_ftc_pivot mowgli_interfaces::mowgli_interfaces_headers)

  # Costmap side of the offset lattice (ftc_lattice_solver.hpp): node tests,
  # degrading solve, and a case recorded on the robot (2026-09-22). ROS-free.
  ament_add_gtest(test_ftc_lattice_solver
    test/test_ftc_lattice_solver.cpp
    src/ftc_lattice_solver.cpp
    src/obstacle_deviation.cpp
  )

  target_include_directories(test_ftc_lattice_solver
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_ftc_lattice_solver
    ${geometry_msgs_TARGETS}
    ${nav2_costmap_2d_TARGETS}
    ${tf2_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
  )

  # Turn fallback decision (ftc_turn_fallback.hpp): turn detection, rejoin
  # search, bounded reverse, pivot/straight sweeps, zone rule, and the recorded
  # 2026-09-22 08:08:25 WEDGED. ROS-free.
  ament_add_gtest(test_ftc_turn_fallback
    test/test_ftc_turn_fallback.cpp
    src/ftc_turn_fallback.cpp
    src/ftc_lattice_solver.cpp
    src/obstacle_deviation.cpp
  )

  target_include_directories(test_ftc_turn_fallback
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )

  target_link_libraries(test_ftc_turn_fallback
    ${geometry_msgs_TARGETS}
    ${nav2_costmap_2d_TARGETS}
    ${tf2_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
    mowgli_interfaces::mowgli_interfaces_headers
  )

  # FTCController through a turn fallback, closed loop (real controller + a
  # configured Costmap2DROS + kinematic robot + controlled clock). Brings up
  # rclcpp, hence the isolated DDS domain.
  ament_add_ros_isolated_gtest(test_ftc_turn_fallback_controller
    test/test_ftc_turn_fallback_controller.cpp
    TIMEOUT 300
  )
  target_link_libraries(test_ftc_turn_fallback_controller ${PROJECT_NAME})

  add_executable(lattice_replay test/lattice_replay.cpp)
  target_link_libraries(lattice_replay ${PROJECT_NAME})
endif()

# ---------------------------------------------------------------------------
# Package export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_ros_common
  nav_msgs
  geometry_msgs
  pluginlib
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  visualization_msgs
  mowgli_interfaces
)

ament_package()
