cmake_minimum_required(VERSION 3.8)
project(fusion_graph)

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
find_package(diagnostic_msgs    REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2                REQUIRED)
find_package(tf2_ros            REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2_eigen          REQUIRED)
find_package(mowgli_interfaces  REQUIRED)
find_package(Eigen3             REQUIRED)

# GTSAM ships its own CMake config; require >= 4.2 (Pose2 + ISAM2 stable
# API). On Kilted the Docker image builds GTSAM from source.
find_package(GTSAM CONFIG REQUIRED)

# ---------------------------------------------------------------------------
# Library: fusion_graph_core (factors + graph manager, header-light)
# ---------------------------------------------------------------------------

add_library(fusion_graph_core
  src/factors.cpp
  src/graph_manager.cpp
  src/scan_matcher.cpp
)

target_include_directories(fusion_graph_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# GTSAM is exported via its imported target `gtsam`. Linking publicly so
# downstream targets (the executable + tests) inherit the include dirs.
target_link_libraries(fusion_graph_core PUBLIC
  gtsam
  Eigen3::Eigen
)

ament_target_dependencies(fusion_graph_core PUBLIC
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  mowgli_interfaces
)

# ---------------------------------------------------------------------------
# Executable: fusion_graph_node
# ---------------------------------------------------------------------------

add_executable(fusion_graph_node
  src/fusion_graph_node.cpp
)

target_link_libraries(fusion_graph_node
  fusion_graph_core
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
    fusion_graph_core
    fusion_graph_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto      REQUIRED)
  # uncrustify is replaced by clang-format (see .clang-format) — both are
  # style linters and disagree on subtle rules.
  set(ament_cmake_uncrustify_FOUND TRUE)
  # ament_copyright requires the full GPL license text; the project uses
  # SPDX identifiers (`SPDX-License-Identifier: GPL-3.0-or-later`) which
  # ament doesn't recognise as a known license — skip the lint.
  set(ament_cmake_copyright_FOUND TRUE)
  # cpplint enforces Google C++ style; clang-format with the project's
  # .clang-format covers the same ground with our own rules.
  set(ament_cmake_cpplint_FOUND TRUE)
  # flake8 + pep257 are PEP 8 / docstring linters for Python; the launch
  # files in this package follow the project's relaxed Python style.
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_factors
    test/test_factors.cpp
  )
  target_link_libraries(test_factors fusion_graph_core)

  ament_add_gtest(test_persistence
    test/test_persistence.cpp
  )
  target_link_libraries(test_persistence fusion_graph_core)

  ament_add_gtest(test_stationary_yaw
    test/test_stationary_yaw.cpp
  )
  target_link_libraries(test_stationary_yaw fusion_graph_core)

  # Performance benchmarks. Long-running (~30 s on ARM); only meaningful
  # in Release builds. Output is grep-friendly for tracking regressions.
  ament_add_gtest(test_perf
    test/test_perf.cpp
    TIMEOUT 300
  )
  target_link_libraries(test_perf fusion_graph_core)
endif()

# ---------------------------------------------------------------------------
# Package export
# ---------------------------------------------------------------------------

ament_export_include_directories(include)
ament_export_libraries(fusion_graph_core)
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  diagnostic_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  mowgli_interfaces
  Eigen3
)

ament_package()
