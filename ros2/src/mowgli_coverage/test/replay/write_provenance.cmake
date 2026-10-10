# Executed before every replay build; unchanged metadata keeps its timestamp.
file(SHA256 "${PLANNER_SOURCE}" source_sha256)
file(SHA256 "${PLANNER_HEADER}" header_sha256)
if(NOT REPLAY_REVISION)
  execute_process(COMMAND git -C "${PACKAGE_ROOT}" rev-parse HEAD
    RESULT_VARIABLE git_result OUTPUT_VARIABLE REPLAY_REVISION
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
  if(NOT git_result EQUAL 0)
    message(FATAL_ERROR "Set REPLAY_REVISION when building an archive without Git metadata")
  endif()
endif()
if(NOT REPLAY_REVISION MATCHES "^[0-9a-fA-F]+$")
  message(FATAL_ERROR "REPLAY_REVISION must be a hexadecimal repository revision")
endif()
file(WRITE "${OUTPUT}.tmp" "#pragma once\n#define REPLAY_REVISION \"${REPLAY_REVISION}\"\n#define REPLAY_SOURCE_SHA256 \"${source_sha256}\"\n#define REPLAY_HEADER_SHA256 \"${header_sha256}\"\n")
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${OUTPUT}.tmp" "${OUTPUT}"
  RESULT_VARIABLE copy_result)
file(REMOVE "${OUTPUT}.tmp")
if(NOT copy_result EQUAL 0)
  message(FATAL_ERROR "Could not write replay provenance header")
endif()
