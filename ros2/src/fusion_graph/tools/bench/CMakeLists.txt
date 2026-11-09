# Standalone build of the fusion_graph GraphManager session benchmark.
# Needs only GTSAM + Eigen (no ROS): compiles the graph-side sources directly.
#
#   cmake -S ros2/src/fusion_graph/tools/bench -B build-bench -DCMAKE_BUILD_TYPE=Release \
#         -DCMAKE_PREFIX_PATH=/opt/gtsam
#   cmake --build build-bench -j2
#   build-bench/bench_graph_session --minutes 120 --dump /tmp/a.bin
cmake_minimum_required(VERSION 3.22.1)
project(fusion_graph_bench CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(GTSAM CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
set(FG ${CMAKE_CURRENT_SOURCE_DIR}/../..)
add_executable(bench_graph_session
  bench_graph_session.cpp
  ${FG}/src/factors.cpp
  ${FG}/src/graph_manager.cpp
  ${FG}/src/graph_manager_node.cpp
  ${FG}/src/graph_manager_measurements.cpp
  ${FG}/src/graph_manager_rebase.cpp
  ${FG}/src/graph_manager_persistence.cpp
  ${FG}/src/checkpoint_file.cpp
)
target_include_directories(bench_graph_session PRIVATE ${FG}/include)
target_link_libraries(bench_graph_session gtsam Eigen3::Eigen)
