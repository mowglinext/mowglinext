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
  # A plain incremental build must refresh metadata when the planner changes.
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${replay_source}" "${replay_include}/mowgli_coverage/coverage_planning.hpp")
  set(replay_revision "${REPLAY_REVISION}")
  if(NOT replay_revision)
    execute_process(COMMAND git -C "${replay_package_root}" rev-parse HEAD
      OUTPUT_VARIABLE replay_revision OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    execute_process(COMMAND git -C "${replay_package_root}" symbolic-ref --quiet HEAD
      OUTPUT_VARIABLE replay_branch_ref OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    # HEAD covers checkout/detach; the branch ref covers commits; packed-refs
    # covers repositories without a loose branch ref. Resolve worktree paths.
    foreach(replay_git_ref HEAD packed-refs "${replay_branch_ref}")
      if(replay_git_ref)
        execute_process(COMMAND git -C "${replay_package_root}"
          rev-parse --path-format=absolute --git-path "${replay_git_ref}"
          OUTPUT_VARIABLE replay_git_path OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(EXISTS "${replay_git_path}")
          set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${replay_git_path}")
        endif()
      endif()
    endforeach()
  endif()
  file(SHA256 "${replay_source}" replay_source_sha256)
  file(SHA256 "${replay_include}/mowgli_coverage/coverage_planning.hpp" replay_header_sha256)
  add_executable(coverage_replay "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/replay.cpp")
  target_link_libraries(coverage_replay PRIVATE ${planner_target}
    Fields2Cover::Fields2Cover nlohmann_json::nlohmann_json)
  target_compile_definitions(coverage_replay PRIVATE
    REPLAY_REVISION="${replay_revision}" REPLAY_SOURCE_SHA256="${replay_source_sha256}"
    REPLAY_HEADER_SHA256="${replay_header_sha256}"
    REPLAY_COMPILER="${CMAKE_CXX_COMPILER_ID}-${CMAKE_CXX_COMPILER_VERSION}"
    REPLAY_BUILD_TYPE="${CMAKE_BUILD_TYPE}")
  if(REPLAY_BASELINE)
    target_compile_definitions(coverage_replay PRIVATE REPLAY_BASELINE)
  endif()
endfunction()
