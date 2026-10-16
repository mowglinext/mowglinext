# Shared target definition for ament and the standalone planner build.
function(add_coverage_replay planner_target)
  set(replay_executable coverage_replay)
  if(ARGC GREATER 1)
    set(replay_executable "${ARGV1}")
  endif()
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
  if(NOT TARGET coverage_replay_provenance)
    add_custom_target(coverage_replay_provenance
    COMMAND "${CMAKE_COMMAND}"
      "-DPLANNER_SOURCE=${replay_source}"
      "-DPLANNER_HEADER=${replay_include}/mowgli_coverage/coverage_planning.hpp"
      "-DPACKAGE_ROOT=${replay_package_root}" "-DREPLAY_REVISION=${REPLAY_REVISION}"
      "-DOUTPUT=${replay_provenance}"
      -P "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/write_provenance.cmake"
      BYPRODUCTS "${replay_provenance}" VERBATIM)
  endif()
  add_executable(${replay_executable} "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/replay.cpp")
  add_dependencies(${replay_executable} coverage_replay_provenance)
  target_include_directories(${replay_executable} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
  target_link_libraries(${replay_executable} PRIVATE ${planner_target}
    Fields2Cover::Fields2Cover nlohmann_json::nlohmann_json)
  string(TOUPPER "${CMAKE_BUILD_TYPE}" replay_build_type)
  string(SHA256 replay_flags_sha256
    "${CMAKE_CXX_FLAGS}\n${CMAKE_CXX_FLAGS_${replay_build_type}}")
  set(replay_reference_sources "")
  foreach(reference_file ring_contract_reference.hpp ring_contract.hpp ring_contract_adapter.cpp.in)
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
      "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/${reference_file}")
    file(READ "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/${reference_file}" reference_contents)
    string(APPEND replay_reference_sources "${reference_contents}")
  endforeach()
  string(SHA256 replay_reference_sha256 "${replay_reference_sources}")
  target_compile_definitions(${replay_executable} PRIVATE
    REPLAY_COMPILER="${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}"
    REPLAY_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
    REPLAY_REQUESTED_FLAGS_SHA256="${replay_flags_sha256}"
    REPLAY_REFERENCE_SHA256="${replay_reference_sha256}")
  if(REPLAY_BASELINE)
    target_compile_definitions(${replay_executable} PRIVATE REPLAY_BASELINE)
  endif()
endfunction()
