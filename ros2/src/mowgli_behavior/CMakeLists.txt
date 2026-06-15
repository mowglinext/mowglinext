cmake_minimum_required(VERSION 3.8)
project(mowgli_behavior)

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
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)


# ---------------------------------------------------------------------------
# Executable: behavior_tree_node
# ---------------------------------------------------------------------------

add_executable(behavior_tree_node
  src/behavior_tree_node.cpp
  src/condition_nodes.cpp
  src/navigation_nodes.cpp
  src/docking_nodes.cpp
  src/calibration_nodes.cpp
  src/status_nodes.cpp
  src/utility_nodes.cpp
  src/coverage_nodes.cpp
  src/recording_nodes.cpp
  src/register_nodes.cpp
)

target_include_directories(behavior_tree_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(behavior_tree_node
  ament_index_cpp
  rclcpp
  rclcpp_action
  behaviortree_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  nav_msgs
  geometry_msgs
  action_msgs
  std_msgs
  std_srvs
  mowgli_interfaces
  rcl_interfaces
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(TARGETS behavior_tree_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

install(DIRECTORY include/
  DESTINATION include
)

# ---------------------------------------------------------------------------
# Linting (optional, enabled when testing is ON)
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # Exclude uncrustify — we use clang-format (see .clang-format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # ---- test_recording_nodes ----
  ament_add_gtest(test_recording_nodes
    test/test_recording_nodes.cpp
    src/recording_nodes.cpp
  )

  target_include_directories(test_recording_nodes PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_recording_nodes
    rclcpp
    geometry_msgs
    behaviortree_cpp
    mowgli_interfaces
    nav_msgs
    tf2
    tf2_ros
  )

  # ---- test_obstacle_recovery ----
  # Exercises IsObstacleStuck by ticking it through a minimal BT, with
  # the BTContext (and its collision_monitor fields) mutated directly to
  # simulate the subscriber callback in behavior_tree_node.cpp.
  ament_add_gtest(test_obstacle_recovery
    test/test_obstacle_recovery.cpp
    src/condition_nodes.cpp
  )

  target_include_directories(test_obstacle_recovery PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_obstacle_recovery
    rclcpp
    behaviortree_cpp
    mowgli_interfaces
    nav2_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

endif()

ament_package()
