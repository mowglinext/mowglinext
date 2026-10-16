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
set(PLANNER_TEST_ROOT "${PACKAGE_ROOT}/test" CACHE PATH "Matching planner test sources")
option(REPLAY_BASELINE "Build against planner headers without optional trace support" OFF)
option(REPLAY_CERTIFIED_FILLET_REFERENCE "Build the narrow pristine-baseline fillet oracle" OFF)
if(REPLAY_CERTIFIED_FILLET_REFERENCE AND NOT REPLAY_BASELINE)
  message(FATAL_ERROR "Certified fillet reference requires REPLAY_BASELINE=ON")
endif()
add_library(replay_planner STATIC "${PLANNER_SOURCE}")
target_include_directories(replay_planner PUBLIC "${PLANNER_INCLUDE}"
    "${ROS_SRC}/mowgli_interfaces/include")
target_link_libraries(replay_planner PUBLIC Fields2Cover::Fields2Cover)
target_compile_options(replay_planner PRIVATE -Wall -Wextra -Wpedantic)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  # Retain cc1's effective target switches, including implicit/spec defaults.
  target_compile_options(replay_planner PRIVATE -frecord-gcc-switches)
endif()

set(REPLAY_REVISION "" CACHE STRING "Repository revision for a source archive")
include(configure_replay.cmake)
add_coverage_replay(replay_planner)
if(REPLAY_CERTIFIED_FILLET_REFERENCE)
  file(READ "${PLANNER_INCLUDE}/mowgli_coverage/coverage_planning.hpp" reference_header)
  if(NOT reference_header MATCHES "PlanningTrace")
    message(FATAL_ERROR
      "Certified fillets require the pristine baseline's original PlanningTrace API; no compatibility shim is allowed")
  endif()
  configure_file(ring_contract_adapter.cpp.in ring_contract_adapter.cpp @ONLY)
  add_library(replay_reference_planner STATIC "${CMAKE_CURRENT_BINARY_DIR}/ring_contract_adapter.cpp")
  target_include_directories(replay_reference_planner PUBLIC "${PLANNER_INCLUDE}"
    "${ROS_SRC}/mowgli_interfaces/include")
  target_link_libraries(replay_reference_planner PUBLIC Fields2Cover::Fields2Cover)
  target_compile_options(replay_reference_planner PRIVATE -Wall -Wextra -Wpedantic)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    target_compile_options(replay_reference_planner PRIVATE -frecord-gcc-switches)
  endif()
  add_coverage_replay(replay_reference_planner coverage_replay_reference)
  target_compile_definitions(coverage_replay_reference PRIVATE REPLAY_REFERENCE)
  configure_file(test_ring_contract_reference.cpp.in test_ring_contract_reference.cpp @ONLY)
  add_executable(test_ring_contract_reference "${CMAKE_CURRENT_BINARY_DIR}/test_ring_contract_reference.cpp")
  target_include_directories(test_ring_contract_reference PRIVATE "${PLANNER_INCLUDE}"
    "${ROS_SRC}/mowgli_interfaces/include")
  target_link_libraries(test_ring_contract_reference PRIVATE Fields2Cover::Fields2Cover GTest::gtest_main)
  add_test(NAME test_ring_contract_reference COMMAND test_ring_contract_reference)
endif()

foreach(test_name coverage_planning pivot_joins start_point coverage_preview coverage_route)
  add_executable(test_${test_name} "${PLANNER_TEST_ROOT}/test_${test_name}.cpp")
  target_link_libraries(test_${test_name} PRIVATE replay_planner GTest::gtest_main)
  add_test(NAME test_${test_name} COMMAND test_${test_name})
endforeach()
if(NOT REPLAY_BASELINE OR EXISTS "${PLANNER_TEST_ROOT}/test_ring_corner_smoothing.cpp")
  # The bevel repair uses a private planner seam; exercise its containment,
  # pivot-preservation and rollback controls in every replay configuration too.
  get_filename_component(PLANNER_SOURCE_DIR "${PLANNER_SOURCE}" DIRECTORY)
  add_executable(test_ring_corner_smoothing "${PLANNER_TEST_ROOT}/test_ring_corner_smoothing.cpp")
  target_include_directories(test_ring_corner_smoothing PRIVATE "${PLANNER_SOURCE_DIR}")
  target_link_libraries(test_ring_corner_smoothing PRIVATE replay_planner GTest::gtest_main)
  add_test(NAME test_ring_corner_smoothing COMMAND test_ring_corner_smoothing)
endif()

if(NOT REPLAY_BASELINE)
  add_executable(test_replay_trace test_trace.cpp)
  target_link_libraries(test_replay_trace PRIVATE replay_planner GTest::gtest_main)
  add_test(NAME test_replay_trace COMMAND test_replay_trace)
endif()
add_test(NAME replay_runner_unit COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_runner.py")
add_test(NAME replay_comparison_unit COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_comparison.py")
add_test(NAME replay_provenance_unit COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/test_provenance.py")
set_tests_properties(replay_provenance_unit PROPERTIES
    ENVIRONMENT "REPLAY_CMAKE=${CMAKE_COMMAND}")
add_test(NAME replay_smoke COMMAND "${Python3_EXECUTABLE}"
    "${CMAKE_CURRENT_SOURCE_DIR}/run_replay.py" --binary "$<TARGET_FILE:coverage_replay>"
    --same-process 2 --processes 2 --case square_auto --case split_mainland
    --output "${CMAKE_CURRENT_BINARY_DIR}/smoke")
