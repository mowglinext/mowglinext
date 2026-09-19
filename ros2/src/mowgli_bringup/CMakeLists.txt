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
  # The GNSS sidecar's embedded launcher (install/compose/docker-compose.gps.yml)
  # keeps its defaults equal to the template and needs no host-side config.
  ament_add_pytest_test(test_gnss_sidecar_launcher test/test_gnss_sidecar_launcher.py
    TIMEOUT 30
  )
  # Verifies the sparse-installed-config deep-merge (robot_config_util):
  # template-only defaults, sparse overrides + fall-through, reset-to-default,
  # and full-config backward compat.
  ament_add_pytest_test(test_robot_config_util test/test_robot_config_util.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_urdf_xacro test/test_urdf_xacro.py
    TIMEOUT 30
  )
  ament_add_pytest_test(test_cmd_vel_ws_relay test/test_cmd_vel_ws_relay.py
    TIMEOUT 30
  )

  # Static guard for Inv 2 (TF sole ownership) — greps sources/configs, no
  # nodes spun up.
  ament_add_pytest_test(test_tf_ownership test/test_tf_ownership.py
    TIMEOUT 30
  )

  # Static guards for the single-line mowgli_robot.yaml -> node-parameter
  # injections in the launch files (num_headland_passes, mowing_enabled,
  # dock_max_retries, dock_use_charger_detection). AST-based, no ROS2 runtime.
  ament_add_pytest_test(test_launch_injection test/test_launch_injection.py
    TIMEOUT 30
  )

  # Lockstep guard for the firmware wheel-PI defaults: template ==
  # mowgli.launch.py fallback == hardware_bridge_node.cpp declared default, and
  # the set clears the template's deadband_pwm (2026-09-15 stiction lock).
  ament_add_pytest_test(test_drive_pid_defaults test/test_drive_pid_defaults.py
    TIMEOUT 30
  )

  # Type guard for the LiDAR map geometry fusion_graph.launch.py forwards:
  # declare_parameter<double> aborts the localizer on an int, which is how a
  # GUI settings save bricked the robot on 2026-09-15. Executes the launch
  # file's own pure helper out of its AST, no ROS2 runtime.
  ament_add_pytest_test(test_fusion_graph_launch_types
    test/test_fusion_graph_launch_types.py
    TIMEOUT 30
  )

endif()

ament_package()
