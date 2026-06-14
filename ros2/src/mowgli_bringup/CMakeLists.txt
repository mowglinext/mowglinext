cmake_minimum_required(VERSION 3.8)
project(mowgli_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# Install resource directories
# ---------------------------------------------------------------------------
install(
  DIRECTORY
    launch
    config
    urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/wait_for_tf.py
    scripts/empty_static_map_pub.py
    scripts/cmd_vel_ws_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Testing
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # Exclude uncrustify — we use clang-format (see .clang-format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)

  # Integration test: verify all nodes start and advertise their public API
  add_launch_test(test/test_nodes_startup.launch.py
    TIMEOUT 60
  )
  
  add_launch_test(test/test_navsat_status_universal.launch.py
    TIMEOUT 45
  )

  # Static config-validation tests for nav2_params.yaml. Cheap (no nodes
  # spun up) and they guard against regressions like the 2026-05-08
  # coverage_goal_checker.xy_goal_tolerance=0.5 m field bug, where the
  # FTC controller silently SUCCEEDED on the first tick and the robot
  # never moved.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_nav2_params test/test_nav2_params.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_gnss_launch_config test/test_gnss_launch_config.py
    TIMEOUT 30
  )

endif()

ament_package()
