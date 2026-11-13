cmake_minimum_required(VERSION 3.22.1)
project(mowgli_ftc_solver_benchmark LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

get_filename_component(DEFAULT_PACKAGE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(MOWGLI_PACKAGE_ROOT "${DEFAULT_PACKAGE_ROOT}" CACHE PATH
  "mowgli_nav2_plugins package whose production sources are benchmarked")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(ftc_solver_bench
  ftc_solver_bench.cpp
  "${MOWGLI_PACKAGE_ROOT}/src/ftc_lattice_solver.cpp"
  "${MOWGLI_PACKAGE_ROOT}/src/obstacle_deviation.cpp"
)
target_compile_features(ftc_solver_bench PRIVATE cxx_std_20)
target_include_directories(ftc_solver_bench PRIVATE "${MOWGLI_PACKAGE_ROOT}/include")
target_link_libraries(ftc_solver_bench
  ${geometry_msgs_TARGETS}
  ${nav2_costmap_2d_TARGETS}
  ${tf2_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
)
