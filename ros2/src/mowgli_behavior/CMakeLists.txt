cmake_minimum_required(VERSION 3.8)
project(mowgli_behavior)

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
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mowgli_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)


# ---------------------------------------------------------------------------
# Executable: behavior_tree_node
# ---------------------------------------------------------------------------

add_executable(behavior_tree_node
  src/behavior_tree_node.cpp
  src/battery_filter.cpp
  src/condition_nodes.cpp
  src/navigation_nodes.cpp
  src/docking_nodes.cpp
  src/escape_nodes.cpp
  src/calibration_nodes.cpp
  src/status_nodes.cpp
  src/status_snapshot.cpp
  src/utility_nodes.cpp
  src/coverage_nodes.cpp
  src/coverage_persistence.cpp
  src/recording_nodes.cpp
  src/register_nodes.cpp
)

target_include_directories(behavior_tree_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(behavior_tree_node
  ament_index_cpp
  rclcpp
  rclcpp_action
  behaviortree_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  action_msgs
  std_msgs
  std_srvs
  mowgli_interfaces
  rcl_interfaces
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------

install(TARGETS behavior_tree_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY trees/
  DESTINATION share/${PROJECT_NAME}/trees
)

install(DIRECTORY include/
  DESTINATION include
)

# ---------------------------------------------------------------------------
# Linting (optional, enabled when testing is ON)
# ---------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # Exclude uncrustify — we use clang-format (see .clang-format)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # ---- test_recording_nodes ----
  ament_add_gtest(test_recording_nodes
    test/test_recording_nodes.cpp
    src/recording_nodes.cpp
  )

  target_include_directories(test_recording_nodes PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_recording_nodes
    rclcpp
    geometry_msgs
    behaviortree_cpp
    mowgli_interfaces
    nav_msgs
    tf2
    tf2_ros
  )

  # ---- test_obstacle_recovery ----
  # Exercises IsObstacleStuck by ticking it through a minimal BT, with
  # the BTContext (and its collision_monitor fields) mutated directly to
  # simulate the subscriber callback in behavior_tree_node.cpp.
  ament_add_gtest(test_obstacle_recovery
    test/test_obstacle_recovery.cpp
    src/condition_nodes.cpp
  )

  target_include_directories(test_obstacle_recovery PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_obstacle_recovery
    rclcpp
    behaviortree_cpp
    mowgli_interfaces
    nav2_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  # ---- test_docking_boundary_exempt ----
  # Regression for "Mowing Complete → robot stops short of the dock": exercises
  # the IsDocking condition and the BoundaryGuard whitelist logic that exempts
  # the blade-off dock transit (command 1 + IsDocking) while keeping full
  # boundary protection during blade-on mowing.
  ament_add_gtest(test_docking_boundary_exempt
    test/test_docking_boundary_exempt.cpp
    src/condition_nodes.cpp
  )

  target_include_directories(test_docking_boundary_exempt PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_docking_boundary_exempt
    rclcpp
    behaviortree_cpp
    mowgli_interfaces
    nav2_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  # ---- test_set_nav2_lifecycle ----
  # Exercises the SetNav2Lifecycle action node's idle-suspend gating
  # (disabled = no-op, PAUSE refused off-dock, idempotent transitions) and,
  # with a fake manage_nodes service, the actual PAUSE/RESUME transition.
  ament_add_gtest(test_set_nav2_lifecycle
    test/test_set_nav2_lifecycle.cpp
    src/navigation_nodes.cpp
  )

  target_include_directories(test_set_nav2_lifecycle PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_set_nav2_lifecycle
    rclcpp
    rclcpp_action
    behaviortree_cpp
    mowgli_interfaces
    nav2_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
    rcl_interfaces
    action_msgs
  )

  # ---- test_get_next_unmowed_area ----
  # Regression: the mow-selection path SKIPS navigation-only areas. Stands up a
  # real in-process get_mowing_area service returning a nav-only area + a mowing
  # area and ticks GetNextUnmowedArea to confirm the nav-only zone is never
  # selected for coverage.
  # Also covers targeted runs (~/start_in_area): "mow only area N" is session
  # state, so the run must not roll over into another area when the BT re-enters
  # MowingSequence after the target completes. status_nodes/status_snapshot are
  # linked in for the real EndSession node (the session boundary that clears it).
  ament_add_gtest(test_get_next_unmowed_area
    test/test_get_next_unmowed_area.cpp
    src/coverage_nodes.cpp
    src/coverage_persistence.cpp
    src/status_nodes.cpp
    src/status_snapshot.cpp
  )

  target_include_directories(test_get_next_unmowed_area PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_get_next_unmowed_area
    rclcpp
    rclcpp_action
    behaviortree_cpp
    mowgli_interfaces
    nav2_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
    rcl_interfaces
    action_msgs
  )

  # ---- test_start_occupied_retry ----
  # Regression for issue #487: one occupied start pose forfeited a whole field.
  # Covers the pure transit-failure classifier (nav2 error_code + error_msg
  # fallback), the consume-once IsCoverageStartBlocked condition, and a
  # STRUCTURAL check that main_tree.xml still carries the non-motion recovery
  # branch (clear costmaps + retry) and still commands no escape motion.
  ament_add_gtest(test_start_occupied_retry
    test/test_start_occupied_retry.cpp
    src/condition_nodes.cpp
  )

  target_include_directories(test_start_occupied_retry PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # The structural half reads the real tree from the source tree.
  target_compile_definitions(test_start_occupied_retry PRIVATE
    MOWGLI_MAIN_TREE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/trees/main_tree.xml"
  )

  ament_target_dependencies(test_start_occupied_retry
    rclcpp
    rclcpp_action
    behaviortree_cpp
    mowgli_interfaces
    nav2_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
    rcl_interfaces
    action_msgs
  )

  # Round-trip + staleness coverage for the disk-backed resume state (issue #334:
  # coverage must resume after a full process/container restart, not just an
  # in-RAM BT halt).
  ament_add_gtest(test_coverage_persistence
    test/test_coverage_persistence.cpp
    src/coverage_persistence.cpp
  )

  target_include_directories(test_coverage_persistence PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_coverage_persistence
    rclcpp
    behaviortree_cpp
    mowgli_interfaces
    nav_msgs
    geometry_msgs
    tf2_ros
  )

  ament_add_gtest(test_gnss_status_authority
    test/test_gnss_status_authority.cpp
  )

  target_include_directories(test_gnss_status_authority PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_gnss_status_authority
    mowgli_interfaces
  )

  # Single-source regression guard (task #14): FollowStrip::kSegmentTransitGap
  # and mowgli_coverage's planning-side split threshold must agree — see
  # test_coverage_transit_gap.cpp. Header-only on both sides (constexpr class
  # member + constexpr free constant), so no coverage_nodes.cpp / .cpp source
  # needs linking here.
  ament_add_gtest(test_coverage_transit_gap
    test/test_coverage_transit_gap.cpp
  )

  target_include_directories(test_coverage_transit_gap PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_coverage_transit_gap
    rclcpp
    rclcpp_action
    behaviortree_cpp
    mowgli_interfaces
    nav2_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
    rcl_interfaces
    action_msgs
  )

  # ---- test_coverage_resume_location ----
  # Regression: resolveResumeLocation is the SHARED resume-cursor mapping used by
  # both FollowStrip (trim/mark driven units) and PlanCoverageArea (aim the
  # blade-off transit at the resume point). If they disagreed, the robot would
  # arrive at one place then re-transit elsewhere. Pins the mapping + guards.
  ament_add_gtest(test_coverage_resume_location
    test/test_coverage_resume_location.cpp
    src/coverage_nodes.cpp
    src/coverage_persistence.cpp
  )

  target_include_directories(test_coverage_resume_location PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_coverage_resume_location
    rclcpp
    rclcpp_action
    behaviortree_cpp
    mowgli_interfaces
    nav2_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
    rcl_interfaces
    action_msgs
  )

  # ---- test_swath_progress ----
  # Regression: the GUI live mowing-progress scalars (ctx.total_swaths /
  # ctx.completed_swaths, behind HighLevelStatus.current_path / current_path_index)
  # must be > 0 and climb during a pass, not stay 0 until the terminal branch.
  # Drives refreshSwathProgress through a simulated pass on a plain BTContext.
  ament_add_gtest(test_swath_progress
    test/test_swath_progress.cpp
    src/coverage_nodes.cpp
    src/coverage_persistence.cpp
  )

  target_include_directories(test_swath_progress PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_swath_progress
    rclcpp
    rclcpp_action
    behaviortree_cpp
    mowgli_interfaces
    nav2_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
    rcl_interfaces
    action_msgs
  )

  # ---- test_detour_resume ----
  # Unit tests for the pure detour-and-continue resume-pose search
  # (detour_resume.hpp): when FTC is blocked by an un-skirtable obstacle,
  # FollowStrip must find a clear pose past it and detour (blade-off) instead of
  # abandoning the whole swath. Header-only helper → no coverage_nodes.cpp link.
  ament_add_gtest(test_detour_resume
    test/test_detour_resume.cpp
  )

  target_include_directories(test_detour_resume PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_detour_resume
    geometry_msgs
    mowgli_interfaces
  )

  # ---- test_start_blocked_escape ----
  # SAFETY-CRITICAL (issue #487): the ONLY new physical motion in the
  # start-occupied workstream. Pure header, no ROS — pins the two field cases
  # (post-undock drives FORWARD, mid-mow drives REVERSE), every stand-down
  # (unknown / stale / ~zero direction, blade not verified off, missing
  # start-blocked signal), both bounds, and the compiled ceilings.
  ament_add_gtest(test_start_blocked_escape
    test/test_start_blocked_escape.cpp
  )

  target_include_directories(test_start_blocked_escape PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # ---- test_localization_health ----
  # Unit tests for the LocalizationGuard's input signal
  # (localization_health.hpp). Pins the 2026-08-20 livelock: pivot/slip
  # covariance inflation on a healthy RTK-Fixed receiver must NOT pause
  # mowing, while the 2026-08-02 plain-GPS fallback still must.
  # Header-only helper -> no source link.
  ament_add_gtest(test_localization_health
    test/test_localization_health.cpp
  )

  target_include_directories(test_localization_health PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_localization_health mowgli_interfaces)

  # ---- test_battery_critical_resume ----
  # Regression for "robot stuck after a critical-battery charge": the
  # CriticalBatteryDock tail must AUTO-CONTINUE on recovery (undock, keep
  # current_command + resume cursor) and only EndSession on a dead charger.
  ament_add_gtest(test_battery_critical_resume
    test/test_battery_critical_resume.cpp
    src/condition_nodes.cpp
    src/status_nodes.cpp
    src/status_snapshot.cpp
    src/coverage_persistence.cpp
  )

  target_include_directories(test_battery_critical_resume PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_battery_critical_resume
    rclcpp
    behaviortree_cpp
    mowgli_interfaces
    nav2_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  # ---- test_guard_fallthrough ----
  # Regression for the "blocking guard falls through into MainLogic" bug
  # class (issues #459, #445). LocalizationGuard's handler ended with a
  # WaitForDuration that returns SUCCESS, so the Root ReactiveSequence
  # advanced into MainLogic -> MowingSequence once per cycle and restarted
  # mowing from PREFLIGHT_CHECK/UNDOCKING every 2 s while sigma_xy flapped.
  # Covers the guard behaviour AND asserts structurally that every blocking
  # guard in main_tree.xml still terminates with <AlwaysFailure/>, so a
  # newly added guard cannot reintroduce it.
  ament_add_gtest(test_guard_fallthrough
    test/test_guard_fallthrough.cpp
    src/condition_nodes.cpp
  )

  target_include_directories(test_guard_fallthrough PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # The structural half reads the real tree from the source tree.
  target_compile_definitions(test_guard_fallthrough PRIVATE
    MOWGLI_MAIN_TREE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/trees/main_tree.xml"
  )

  ament_target_dependencies(test_guard_fallthrough
    rclcpp
    behaviortree_cpp
    mowgli_interfaces
    nav2_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  # ---- test_high_level_status_snapshot ----
  # Regression for the frozen GUI gauges: behavior_tree_node's 1 Hz republish
  # timer used to re-send the CACHED HighLevelStatus verbatim, so every live
  # field stalled while the tree sat in a transition-free branch (battery pinned
  # for the whole low-battery charge hold; mowing progress pinned for the whole
  # FollowStrip). Pins the tree-owned vs live-field split.
  ament_add_gtest(test_high_level_status_snapshot
    test/test_high_level_status_snapshot.cpp
    src/status_snapshot.cpp
  )

  target_include_directories(test_high_level_status_snapshot PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(test_high_level_status_snapshot
    rclcpp
    behaviortree_cpp
    mowgli_interfaces
    nav2_msgs
    nav_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )

  # ---- test_battery_filter ----
  # Regression for the premature LOW_BATTERY_DOCKING trip: battery_percent used
  # to be interpolated off the RAW rail voltage, so a sub-second motor sag
  # (PWM startup / stiction, 0.5-1 V) crossed battery_low_percent at a genuine
  # 30-40 % SoC. Pins that a brief sag does not trip while a real discharge
  # still does, that the smoothing is rate-independent (4 Hz robot vs 10 Hz
  # sim), and that an invalid reading never becomes a 0 % gauge.
  # Pure logic — no ROS, so no ament_target_dependencies needed.
  ament_add_gtest(test_battery_filter
    test/test_battery_filter.cpp
    src/battery_filter.cpp
  )

  target_include_directories(test_battery_filter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  # Pure dock-alignment math (header-only): the along/cross-track decomposition
  # logged at claimed contact, and the per-undock dock_pose_yaw drift check.
  ament_add_gtest(test_dock_alignment
    test/test_dock_alignment.cpp
  )

  target_include_directories(test_dock_alignment PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

endif()

ament_package()
