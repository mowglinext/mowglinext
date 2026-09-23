cmake_minimum_required(VERSION 3.8)
project(mowgli_coverage)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(mowgli_interfaces REQUIRED)

# Fields2Cover v3 links ortools::ortools as a PUBLIC dependency, so it lands in
# the exported target's INTERFACE_LINK_LIBRARIES — but v3's Fields2CoverConfig
# .cmake does NOT find_dependency(ortools), so the consumer must define that
# imported target itself or the CMake generate step aborts with "link interface
# of Fields2Cover::Fields2Cover contains ortools::ortools but the target was not
# found" (v2 linked OR-Tools by path and never needed this). Mirror F2C's own
# discovery: ortools_vendor registers the OR-Tools CMake dir on CMAKE_PREFIX_PATH,
# then find_package(ortools) defines ortools::ortools. Both ship in
# ros-lyrical-ortools-vendor (installed in the Dockerfile base stage).
find_package(ortools_vendor QUIET)
find_package(ortools CONFIG REQUIRED)

# Fields2Cover v3 (3.0.0) is built and installed at /opt/fields2cover-300 by
# the project Dockerfile. Keep that prefix first and probe the common config
# subdirs explicitly because the upstream install layout has varied between
# lib/cmake and share/*/cmake across revisions. We intentionally do NOT force
# NO_DEFAULT_PATH here: the Docker build exports /opt/fields2cover-300 onto
# CMAKE_PREFIX_PATH ahead of everything else, and the version pin already
# rejects incompatible system versions.
find_package(Fields2Cover 3.0.0 CONFIG REQUIRED
  PATHS /opt/fields2cover-300
  PATH_SUFFIXES
    lib/cmake/Fields2Cover
    lib64/cmake/Fields2Cover
    share/Fields2Cover/cmake
    share/fields2cover/cmake)

add_library(${PROJECT_NAME}_core SHARED
  src/coverage_server.cpp
  src/coverage_planning.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_core
  ${rclcpp_TARGETS}
  ${rclcpp_action_TARGETS}
  ${rclcpp_lifecycle_TARGETS}
  ${rclcpp_components_TARGETS}
  ${nav2_util_TARGETS}
  ${nav2_ros_common_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${builtin_interfaces_TARGETS}
  ${mowgli_interfaces_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_core Fields2Cover::Fields2Cover)

# Keep the pinned v3 library directory in the installed library's RUNPATH.
# The Docker runtime also registers it with ldconfig for transitive dependencies.
set_target_properties(${PROJECT_NAME}_core PROPERTIES
  INSTALL_RPATH "/opt/fields2cover-300/lib"
)

add_executable(${PROJECT_NAME}
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core)
target_link_libraries(${PROJECT_NAME}
  ${rclcpp_TARGETS}
)

# Make sure the executable can find libmowgli_coverage_core.so at the
# install lib/ dir + libFields2Cover.so at /opt/fields2cover-300/lib.
# CMake doesn't propagate RPATH from imported targets reliably, so spell
# both out here. The first $ORIGIN/.. hop is from
# install/mowgli_coverage/lib/mowgli_coverage/ → install/mowgli_coverage/lib/.
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "$ORIGIN/..:/opt/fields2cover-300/lib"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Geometry tests for the v3 native + legacy coverage planners. These run in
  # native CI (colcon test) against the real Fields2Cover v3 library, so they
  # validate genRoute / planPath output (the headland-rings-as-swaths trick is
  # untested upstream) WITHOUT needing the robot.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coverage_planning test/test_coverage_planning.cpp)
  target_link_libraries(test_coverage_planning
    ${PROJECT_NAME}_core
    Fields2Cover::Fields2Cover
  )
  set_target_properties(test_coverage_planning PROPERTIES
    INSTALL_RPATH "/opt/fields2cover-300/lib"
  )

  # Pivot joins: row-end joins that fit no arc kept blade-on with explicit
  # in-place pivot corners (field 2026-09-21), plus the rings-off invalid-cell
  # repair. Real F2C v3, no ROS.
  ament_add_gtest(test_pivot_joins test/test_pivot_joins.cpp)
  target_link_libraries(test_pivot_joins
    ${PROJECT_NAME}_core
    Fields2Cover::Fields2Cover
  )
  set_target_properties(test_pivot_joins PROPERTIES
    INSTALL_RPATH "/opt/fields2cover-300/lib"
  )
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  nav2_util
  nav2_ros_common
  nav_msgs
  geometry_msgs
  builtin_interfaces
  mowgli_interfaces
)

ament_package()
