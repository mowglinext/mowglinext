cmake_minimum_required(VERSION 3.8)
project(mowgli_map)

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
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED)

# ---------------------------------------------------------------------------
# Library: mowgli_map_lib
# Separating the node implementation from main.cpp allows GTest to link
# against the library without spinning up a full executable.
# ---------------------------------------------------------------------------

add_library(mowgli_map_lib STATIC
  src/map_server_node.cpp
  src/area_manager.cpp
  src/costmap_filters.cpp
  src/progress_tracker.cpp
)

target_include_directories(mowgli_map_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(mowgli_map_lib
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${grid_map_core_TARGETS}
  ${grid_map_ros_LIBRARIES}
  ${grid_map_msgs_TARGETS}
  ${mowgli_interfaces_TARGETS}
  ${nav2_msgs_TARGETS}
  ${tf2_ros_TARGETS}
)
target_include_directories(mowgli_map_lib PUBLIC ${grid_map_ros_INCLUDE_DIRS})

# ---------------------------------------------------------------------------
# Library: mowgli_obstacle_tracker_lib
# Separating the node implementation from main allows GTest to link
# against the library without spinning up a full executable.
# ---------------------------------------------------------------------------

add_library(mowgli_obstacle_tracker_lib STATIC
  src/obstacle_tracker_node.cpp
)

target_include_directories(mowgli_obstacle_tracker_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(mowgli_obstacle_tracker_lib
  ${rclcpp_TARGETS}
  ${std_srvs_TARGETS}
  ${nav_msgs_TARGETS}
  ${map_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${mowgli_interfaces_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)

# ---------------------------------------------------------------------------
# Executable: map_server_node
# ---------------------------------------------------------------------------

add_executable(map_server_node
  src/main.cpp
)

target_link_libraries(map_server_node
  mowgli_map_lib
)

target_link_libraries(map_server_node
  ${rclcpp_TARGETS}
)

# ---------------------------------------------------------------------------
# Executable: obstacle_tracker_node
# ---------------------------------------------------------------------------

add_executable(obstacle_tracker_node
  src/obstacle_tracker_main.cpp
)

target_link_libraries(obstacle_tracker_node
  mowgli_obstacle_tracker_lib
)

target_link_libraries(obstacle_tracker_node
  ${rclcpp_TARGETS}
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(TARGETS
    map_server_node
    obstacle_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    mowgli_map_lib
    mowgli_obstacle_tracker_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Every test that brings up rclcpp runs through ament_add_ros_isolated_gtest:
  # colcon tests packages in parallel, and on one shared DDS domain the test
  # processes discover each other's nodes (docs/claude/testing-ci.md).
  find_package(ament_cmake_ros REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Exclude uncrustify — we use clang-format (see .clang-format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_map_server
    test/test_map_server.cpp
  )

  target_link_libraries(test_map_server
    mowgli_map_lib
  )

  target_link_libraries(test_map_server
    ${rclcpp_TARGETS}
    ${grid_map_core_TARGETS}
    ${grid_map_ros_LIBRARIES}
    ${grid_map_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${mowgli_interfaces_TARGETS}
  )
  target_include_directories(test_map_server PUBLIC ${grid_map_ros_INCLUDE_DIRS})

  # SafeTransformListener: back-to-back build/destroy never hangs (tf2_ros's own
  # dedicated-thread listener can lose its cancel and hang the destructor).
  ament_add_ros_isolated_gtest(test_safe_transform_listener
    test/test_safe_transform_listener.cpp
    TIMEOUT 120
  )
  target_include_directories(test_safe_transform_listener PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_safe_transform_listener
    ${rclcpp_TARGETS}
    ${tf2_ros_TARGETS}
    ${tf2_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
  )

  # Map replace (GUI "Save map"): fast keepout mask == per-cell definition,
  # and add_area does no grid work inside the service callback.
  ament_add_ros_isolated_gtest(test_map_replace
    test/test_map_replace.cpp
    TIMEOUT 300
  )

  target_link_libraries(test_map_replace
    mowgli_map_lib
  )

  target_link_libraries(test_map_replace
    ${rclcpp_TARGETS}
    ${grid_map_core_TARGETS}
    ${grid_map_ros_LIBRARIES}
    ${grid_map_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${mowgli_interfaces_TARGETS}
  )
  target_include_directories(test_map_replace PUBLIC ${grid_map_ros_INCLUDE_DIRS})

  ament_add_ros_isolated_gtest(test_mow_progress
    test/test_mow_progress.cpp
  )

  target_link_libraries(test_mow_progress
    mowgli_map_lib
  )

  target_include_directories(test_mow_progress PRIVATE include)

  target_link_libraries(test_mow_progress
    ${rclcpp_TARGETS}
    ${grid_map_core_TARGETS}
    ${grid_map_ros_LIBRARIES}
    ${grid_map_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${mowgli_interfaces_TARGETS}
  )
  target_include_directories(test_mow_progress PUBLIC ${grid_map_ros_INCLUDE_DIRS})

  ament_add_ros_isolated_gtest(test_obstacle_tracker
    test/test_obstacle_tracker.cpp
  )

  target_link_libraries(test_obstacle_tracker
    mowgli_obstacle_tracker_lib
  )

  target_link_libraries(test_obstacle_tracker
    ${rclcpp_TARGETS}
    ${std_srvs_TARGETS}
    ${nav_msgs_TARGETS}
    ${map_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${mowgli_interfaces_TARGETS}
    ${tf2_ros_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
  )

  # Pure logic, no ROS deps — mirrors fusion_graph's header-only gate tests.
  ament_add_gtest(test_boundary_classifier
    test/test_boundary_classifier.cpp
  )
  target_include_directories(test_boundary_classifier PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
  )

  # Pure logic, no ROS deps — which gates a SetDockingPoint request must pass.
  ament_add_gtest(test_dock_set_gates
    test/test_dock_set_gates.cpp
  )
  target_include_directories(test_dock_set_gates PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
endif()

# ---------------------------------------------------------------------------
# Package export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(mowgli_map_lib mowgli_obstacle_tracker_lib)
ament_export_dependencies(
  rclcpp
  std_srvs
  nav_msgs
  map_msgs
  geometry_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  mowgli_interfaces
  nav2_msgs
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  visualization_msgs
)

ament_package()
