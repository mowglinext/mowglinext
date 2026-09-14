# ROS2 Project Rules

> These rules are specific to the MowgliNext ROS2 stack. They extend the common rules with ROS2/robotics conventions.

> Reference indexes: [`ros2/CLAUDE.md`](../../ros2/CLAUDE.md) and the per-package codemaps in
> [`docs/claude/codemaps/`](../../docs/claude/codemaps/); who publishes what in
> [`docs/claude/ros-interfaces.md`](../../docs/claude/ros-interfaces.md); every config key in
> [`docs/claude/parameters.md`](../../docs/claude/parameters.md); every test + its CI job in
> [`docs/claude/testing-ci.md`](../../docs/claude/testing-ci.md).

## Node Patterns

- Use rclcpp lifecycle nodes for anything managed by Nav2's lifecycle manager
- Use regular rclcpp::Node for standalone nodes (hardware_bridge, behavior_tree)
- Declare ALL parameters in the constructor — no undeclared parameter access
- Use `declare_parameter<T>()` with default values, never `get_parameter()` without prior declaration

## QoS Profiles

- Sensor data (IMU, LiDAR, GPS, COG/mag yaw): **subscribe** with `rclcpp::SensorDataQoS()` — a BEST_EFFORT publisher (`cog_to_imu_node`, `mag_yaw_publisher_node`, the LiDAR drivers) is silently dropped at the QoS handshake by a RELIABLE subscription
- Sensor **publishers** may still be RELIABLE where the stream is wanted lossless (`~/imu/data_raw`, `~/wheel_odom`, `/gps/fix` are all `rclcpp::QoS(10)`) — RELIABLE publisher → BEST_EFFORT subscriber is compatible; the reverse is not
- Commands (cmd_vel, `geometry_msgs/TwistStamped`): `rclcpp::SystemDefaultsQoS()` (reliable)
- Status/diagnostics: `rclcpp::QoS(10)` (reliable, depth 10)
- TF: use `tf2_ros` defaults — never override TF QoS

## Topic Naming

- Node-internal topics: use relative names (`~/status`, `~/cmd_vel`)
- Remap `~/topic` to its flat absolute name in the launch file, not in C++ source (`~/status` → `/hardware_bridge/status`, `~/wheel_odom` → `/wheel_odom`, `~/cmd_vel` → `/cmd_vel` in `mowgli.launch.py`). Nodes are NOT pushed into a `/mowgli/` namespace — no launch file sets `namespace=`
- Shared system topics: absolute (`/scan`, `/cmd_vel`, `/odometry/filtered_map`)

## Launch Files

- Python launch files only (`.launch.py`), not XML or YAML launch
- Load parameters from YAML files via `os.path.join(get_package_share_directory(...), 'config', '...')`
- Use `LaunchConfiguration` for runtime arguments, with sensible defaults
- Read robot config through `robot_config_util.load_robot_params(bringup_dir, "/ros2_ws/config/mowgli_robot.yaml")` — the SPARSE installed file deep-merged over the in-package template, so every key is always present (root CLAUDE.md Invariant 15). Never parse the installed yaml directly, and never fall back to a `LIDAR_ENABLED` env var (the `lidar_enabled` yaml key is the only source)

## Testing

- Use `ament_cmake_gtest` for C++ unit tests
- Use `launch_testing` (`add_launch_test`) for integration tests, and `ament_cmake_pytest` for the static launch/param guards (`test_nav2_params.py`, `test_tf_ownership.py`, `test_robot_config_util.py`, …)
- Test coverage target: 80% for new packages
- All BT nodes must have unit tests for tick() behavior

## Build System

- `ament_cmake` for every package under `ros2/src/` — there is no `ament_python` package, not even for the launch-only one (`mowgli_bringup`)
- Launch files/config install via `install(DIRECTORY …)`, Python entry points via `install(PROGRAMS … DESTINATION lib/${PROJECT_NAME})`, Python modules via `ament_cmake_python` (`tools/motor` → `mowgli_tools`)
- Always specify dependencies in both `CMakeLists.txt` AND `package.xml`
- Use `find_package()` for build deps, `<depend>` in package.xml for runtime

## Safety Rules

- Blade commands are fire-and-forget — firmware decides execution
- Emergency stop is firmware-level, not software-level
- Never disable collision_monitor on real hardware
- Undock is GPS-gated at both ends (`main_tree.xml`): `PreFlightCheck(min_gps_fix_type="2")` retried 120×1 s BEFORE the `BackUp`, then `WaitForGpsFix(timeout_sec="20.0" min_fix_type="4")` AFTER it. Do NOT demand RTK Fixed before undock — the dock canopy pins the receiver at Float, and the gate then deadlocks the undock it is waiting on
- Battery thresholds must have hysteresis (low != resume) — the gating pair is percent (`battery_critical_percent` 10 % in, `battery_full_percent` 95 % out), with matching voltage thresholds alongside in `mowgli_robot.yaml`

## Docker / ARM Considerations

- Test Docker builds for both `linux/amd64` and `linux/arm64`
- Avoid composition mode on ARM (causes crashes) — use separate processes; `navigation.launch.py` pins `use_composition:="False"` and that is also the `nav2_navigation_launch.py` default
- Gate slow ARM startup on an EVENT, not a fixed delay — `navigation.launch.py` runs `wait_for_tf.py --parent map --child odom --timeout 120` and launches the Nav2 group from its `OnProcessExit`. There is no `TimerAction` left in the stack; do not add one back. Bond load is trimmed for the Pi too (`bond_timeout` 10.0, `bond_heartbeat_period` 0.5 set once for all managed nodes)
- Cyclone DDS only — FastRTPS has stale shm issues on ARM
