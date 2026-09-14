cmake_minimum_required(VERSION 3.8)
project(mowgli_gnss_bridge)

# ---------------------------------------------------------------------------
# Compiler flags
# ---------------------------------------------------------------------------

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(universal_gnss_ros2 REQUIRED)

# ---------------------------------------------------------------------------
# Library: mowgli_gnss_bridge_lib
# Split from main.cpp so GTest can link the node without an executable entry.
# ---------------------------------------------------------------------------

add_library(mowgli_gnss_bridge_lib STATIC
  src/correction_diagnostic_tracker.cpp
  src/universal_gnss_topic_bridge.cpp
)

target_include_directories(mowgli_gnss_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mowgli_gnss_bridge_lib
  rclcpp
  std_msgs
  builtin_interfaces
  diagnostic_msgs
  rtcm_msgs
  mowgli_interfaces
  universal_gnss_ros2
)

# ---------------------------------------------------------------------------
# Executable: universal_gnss_topic_bridge
# ---------------------------------------------------------------------------

add_executable(universal_gnss_topic_bridge
  src/main.cpp
)

target_link_libraries(universal_gnss_topic_bridge
  mowgli_gnss_bridge_lib
)

ament_target_dependencies(universal_gnss_topic_bridge
  rclcpp
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(TARGETS
    universal_gnss_topic_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # ament_copyright wants the full license text; this project declares
  # licensing with an SPDX identifier, which ament does not recognise. Every
  # package under ros2/src skips it for the same reason.
  set(ament_cmake_copyright_FOUND TRUE)
  # uncrustify and cpplint stay ENABLED, and that is deliberate. Unlike the
  # ros2/src packages, this one lives under sensors/ and is therefore NEVER
  # visited by ros2/scripts/format.sh, whose find(1) only walks ros2/src —
  # so clang-format has no claim here and running it would restyle ~180 lines.
  # ament's uncrustify profile is the only formatter this package has ever
  # followed, so enforcing it is what keeps the code consistent rather than
  # unformatted by anything. cpplint likewise passes and this package is the
  # GNSS trust boundary, so the extra checking earns its place.
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_universal_gnss_topic_bridge
    test/test_universal_gnss_topic_bridge.cpp
  )
  target_link_libraries(test_universal_gnss_topic_bridge
    mowgli_gnss_bridge_lib
  )
  ament_target_dependencies(test_universal_gnss_topic_bridge
    rclcpp
    mowgli_interfaces
    universal_gnss_ros2
    diagnostic_msgs
    rtcm_msgs
  )

  ament_add_gtest(test_correction_diagnostic_tracker
    test/test_correction_diagnostic_tracker.cpp
  )
  target_link_libraries(test_correction_diagnostic_tracker
    mowgli_gnss_bridge_lib
  )
  ament_target_dependencies(test_correction_diagnostic_tracker
    diagnostic_msgs
    mowgli_interfaces
  )

  # C++/Python bridge parity. sensors/CLAUDE.md requires the two bridges to
  # stay behaviour-identical, and nothing else checks that.
  #
  # The reference Python bridge lives one level UP in the repo
  # (sensors/gps/universal_gnss_topic_bridge.py) and is therefore outside this
  # package. A colcon workspace assembled by copying only the package -- which
  # is what sensors/gps/Dockerfile and the CI test job both do -- does not
  # contain it, and the test cannot run. Copy it next to the package in the
  # workspace to enable this gate.
  set(_reference_python_bridge
    "${CMAKE_CURRENT_SOURCE_DIR}/../universal_gnss_topic_bridge.py")
  if(EXISTS "${_reference_python_bridge}")
    get_filename_component(_reference_python_bridge_dir
      "${_reference_python_bridge}" DIRECTORY)
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(test_python_bridge
      test/test_python_bridge.py
      ENV "MOWGLI_GNSS_PYTHON_BRIDGE_DIR=${_reference_python_bridge_dir}"
      TIMEOUT 30
    )
  else()
    message(WARNING
      "universal_gnss_topic_bridge.py was not found next to this package, so "
      "the C++/Python bridge parity test is NOT registered. Copy "
      "sensors/gps/universal_gnss_topic_bridge.py into the workspace beside "
      "mowgli_gnss_bridge to enable it.")
  endif()
endif()

ament_package()
