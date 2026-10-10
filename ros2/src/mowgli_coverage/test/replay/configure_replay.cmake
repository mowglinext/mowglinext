# Shared target definition for ament and the standalone planner build.
function(add_coverage_replay planner_target)
  find_package(nlohmann_json CONFIG REQUIRED)
  get_filename_component(replay_package_root "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../.." ABSOLUTE)
  set(replay_source "${PLANNER_SOURCE}")
  if(NOT replay_source)
    set(replay_source "${replay_package_root}/src/coverage_planning.cpp")
  endif()
  set(replay_include "${PLANNER_INCLUDE}")
  if(NOT replay_include)
    set(replay_include "${replay_package_root}/include")
  endif()
  # Git refs can move between loose and packed storage without touching HEAD.
  # Refresh at build time, preserving the generated header when values match.
  set(replay_provenance "${CMAKE_CURRENT_BINARY_DIR}/coverage_replay_provenance.hpp")
  add_custom_target(coverage_replay_provenance
    COMMAND "${CMAKE_COMMAND}"
      "-DPLANNER_SOURCE=${replay_source}"
      "-DPLANNER_HEADER=${replay_include}/mowgli_coverage/coverage_planning.hpp"
      "-DPACKAGE_ROOT=${replay_package_root}" "-DREPLAY_REVISION=${REPLAY_REVISION}"
      "-DOUTPUT=${replay_provenance}"
      -P "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/write_provenance.cmake"
    BYPRODUCTS "${replay_provenance}" VERBATIM)
  add_executable(coverage_replay "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/replay.cpp")
  add_dependencies(coverage_replay coverage_replay_provenance)
  target_include_directories(coverage_replay PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
  target_link_libraries(coverage_replay PRIVATE ${planner_target}
    Fields2Cover::Fields2Cover nlohmann_json::nlohmann_json)
  target_compile_definitions(coverage_replay PRIVATE
    REPLAY_COMPILER="${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}"
    REPLAY_BUILD_TYPE="${CMAKE_BUILD_TYPE}")
  if(REPLAY_BASELINE)
    target_compile_definitions(coverage_replay PRIVATE REPLAY_BASELINE)
  endif()
endfunction()
