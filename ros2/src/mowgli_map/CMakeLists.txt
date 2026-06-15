cmake_minimum_required(VERSION 3.8)
project(mowgli_map)

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
find_package(nav_msgs REQUIRED)
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

ament_target_dependencies(mowgli_map_lib
  rclcpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  mowgli_interfaces
  nav2_msgs
  tf2_ros
)

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

ament_target_dependencies(mowgli_obstacle_tracker_lib
  rclcpp
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  mowgli_interfaces
  tf2_ros
  tf2_geometry_msgs
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

ament_target_dependencies(map_server_node
  rclcpp
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

ament_target_dependencies(obstacle_tracker_node
  rclcpp
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
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # Exclude uncrustify — we use clang-format (see .clang-format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_map_server
    test/test_map_server.cpp
  )

  target_link_libraries(test_map_server
    mowgli_map_lib
  )

  ament_target_dependencies(test_map_server
    rclcpp
    grid_map_core
    grid_map_ros
    grid_map_msgs
    nav_msgs
    geometry_msgs
    mowgli_interfaces
  )

  ament_add_gtest(test_obstacle_tracker
    test/test_obstacle_tracker.cpp
  )

  target_link_libraries(test_obstacle_tracker
    mowgli_obstacle_tracker_lib
  )

  ament_target_dependencies(test_obstacle_tracker
    rclcpp
    std_srvs
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    mowgli_interfaces
    tf2_ros
    tf2_geometry_msgs
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
