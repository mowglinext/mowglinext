cmake_minimum_required(VERSION 3.22.1)
project(mowgli_scan_deskew_benchmark LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

get_filename_component(DEFAULT_PACKAGE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(MOWGLI_PACKAGE_ROOT "${DEFAULT_PACKAGE_ROOT}" CACHE PATH
  "mowgli_localization package whose production node source is benchmarked")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(scan_deskew_bench scan_deskew_bench.cpp)
target_compile_features(scan_deskew_bench PRIVATE cxx_std_20)
# The benchmark includes the current helper before its access macros. A baseline
# node never calls that helper; DESKEW_NODE_SOURCE selects the production node.
target_include_directories(scan_deskew_bench PRIVATE "${DEFAULT_PACKAGE_ROOT}/include")
target_compile_definitions(scan_deskew_bench PRIVATE
  DESKEW_NODE_SOURCE="${MOWGLI_PACKAGE_ROOT}/src/scan_deskew_node.cpp"
)
target_link_libraries(scan_deskew_bench
  ${rclcpp_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
)
