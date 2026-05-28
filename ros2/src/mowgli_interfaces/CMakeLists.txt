cmake_minimum_required(VERSION 3.8)
project(mowgli_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare message files
set(msg_files
  "msg/Status.msg"
  "msg/Emergency.msg"
  "msg/Power.msg"
  "msg/ESCStatus.msg"
  "msg/HighLevelStatus.msg"
  "msg/GnssStatus.msg"
  "msg/AbsolutePose.msg"
  "msg/WheelTick.msg"
  "msg/ImuRaw.msg"
  "msg/MapArea.msg"
  "msg/ObstacleArray.msg"
  "msg/TrackedObstacle.msg"
)

# Declare service files
set(srv_files
  "srv/AddMowingArea.srv"
  "srv/ClearMap.srv"
  "srv/ClearObstacle.srv"
  "srv/EmergencyStop.srv"
  "srv/GetMowingArea.srv"
  "srv/HighLevelControl.srv"
  "srv/MowerControl.srv"
  "srv/SetDockingPoint.srv"
  "srv/TriggerReplan.srv"
  "srv/GetNextStrip.srv"
  "srv/GetNextSegment.srv"
  "srv/MarkSegmentBlocked.srv"
  "srv/GetCoverageStatus.srv"
  "srv/GetRemainingAreaPolygon.srv"
  "srv/AreaRecording.srv"
  "srv/CalibrateImuYaw.srv"
  "srv/GetRecoveryPoint.srv"
  "srv/StartInArea.srv"
  "srv/PromoteObstacle.srv"
)

# Declare action files
set(action_files
  "action/CoverageTask.action"
  "action/PlanCoverage.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
    nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
