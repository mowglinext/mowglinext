cmake_minimum_required(VERSION 3.22)
project(coverage_replay LANGUAGES CXX)

# Standalone build of the REAL planner, without ROS/server stubs. Run as a normal user.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ortools CONFIG REQUIRED)
find_package(Fields2Cover 3.0.0 EXACT CONFIG REQUIRED PATHS /opt/fields2cover-300)
find_package(GTest REQUIRED)
find_package(Python3 COMPONENTS Interpreter REQUIRED)
enable_testing()

get_filename_component(PACKAGE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
get_filename_component(ROS_SRC "${PACKAGE_ROOT}/.." ABSOLUTE)
set(PLANNER_SOURCE "${PACKAGE_ROOT}/src/coverage_planning.cpp" CACHE FILEPATH
    "Planner implementation (override for a baseline comparison)")
set(PLANNER_INCLUDE "${PACKAGE_ROOT}/include" CACHE PATH "Matching planner headers")
option(REPLAY_BASELINE "Build against planner headers without optional trace support" OFF)
add_library(replay_planner STATIC "${PLANNER_SOURCE}")
target_include_directories(replay_planner PUBLIC "${PLANNER_INCLUDE}"
    "${ROS_SRC}/mowgli_interfaces/include")
target_link_libraries(replay_planner PUBLIC Fields2Cover::Fields2Cover)
target_compile_options(replay_planner PRIVATE -Wall -Wextra -Wpedantic)

set(REPLAY_REVISION "" CACHE STRING "Repository revision for a source archive")
include(configure_replay.cmake)
add_coverage_replay(replay_planner)

foreach(test_name coverage_planning pivot_joins start_point coverage_preview coverage_route)
  add_executable(test_${test_name} "${PACKAGE_ROOT}/test/test_${test_name}.cpp")
  target_link_libraries(test_${test_name} PRIVATE replay_planner GTest::gtest_main)
  add_test(NAME test_${test_name} COMMAND test_${test_name})
endforeach()
if(NOT REPLAY_BASELINE)
  add_executable(test_replay_trace test_trace.cpp)
  target_link_libraries(test_replay_trace PRIVATE replay_planner GTest::gtest_main)
  add_test(NAME test_replay_trace COMMAND test_replay_trace)
endif()
add_test(NAME replay_runner_unit COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_runner.py")
add_test(NAME replay_smoke COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/run_replay.py" --binary "$<TARGET_FILE:coverage_replay>"
    --same-process 2 --processes 2 --case square_auto --case split_mainland
    --output "${CMAKE_CURRENT_BINARY_DIR}/smoke")
