#!/bin/bash
set -e

# Source ROS2
source /opt/ros/kilted/setup.bash

: "${ROS_DOMAIN_ID:=0}"
: "${RMW_IMPLEMENTATION:=rmw_cyclonedds_cpp}"

: "${HARDWARE_BACKEND:=mowgli}"
: "${MAVROS_PORT:=/dev/mavros}"
: "${MAVROS_BAUD:=921600}"
: "${MAVROS_GCS_URL:=}"
: "${MAVROS_TGT_SYSTEM:=1}"
: "${MAVROS_TGT_COMPONENT:=1}"
: "${MAVROS_AUTOPILOT:=ardupilot}"

if [ -n "${MAVROS_ENABLED:-}" ]; then
  expected_alias="false"
  if [ "${HARDWARE_BACKEND}" = "mavros" ]; then
    expected_alias="true"
  fi

  if [ "${MAVROS_ENABLED}" != "${expected_alias}" ]; then
    echo "Warning: MAVROS_ENABLED=${MAVROS_ENABLED} disagrees with HARDWARE_BACKEND=${HARDWARE_BACKEND}; using HARDWARE_BACKEND as the source of truth."
  fi
fi

if [ "${HARDWARE_BACKEND}" != "mavros" ]; then
  echo "HARDWARE_BACKEND=${HARDWARE_BACKEND}; MAVROS sidecar is not selected. Exiting."
  exit 0
fi

case "${MAVROS_AUTOPILOT}" in
  ardupilot|apm)
    MAVROS_LAUNCH_FILE="apm.launch"
    ;;
  px4)
    MAVROS_LAUNCH_FILE="px4.launch"
    ;;
  *)
    echo "Unsupported MAVROS_AUTOPILOT='${MAVROS_AUTOPILOT}'"
    echo "Supported values: ardupilot, apm, px4"
    exit 1
    ;;
esac

MAVROS_FCU_URL="serial://${MAVROS_PORT}:${MAVROS_BAUD}"

echo "Starting MAVROS with:"
echo "  MAVROS_AUTOPILOT=${MAVROS_AUTOPILOT}"
echo "  MAVROS_LAUNCH_FILE=${MAVROS_LAUNCH_FILE}"
echo "  MAVROS_FCU_URL=${MAVROS_FCU_URL}"
echo "  MAVROS_GCS_URL=${MAVROS_GCS_URL}"
echo "  MAVROS_TGT_SYSTEM=${MAVROS_TGT_SYSTEM}"
echo "  MAVROS_TGT_COMPONENT=${MAVROS_TGT_COMPONENT}"

launch_args=(
  "fcu_url:=${MAVROS_FCU_URL}"
  "tgt_system:=${MAVROS_TGT_SYSTEM}"
  "tgt_component:=${MAVROS_TGT_COMPONENT}"
)

if [ -n "${MAVROS_GCS_URL}" ]; then
  launch_args+=("gcs_url:=${MAVROS_GCS_URL}")
fi

exec ros2 launch mavros "${MAVROS_LAUNCH_FILE}" "${launch_args[@]}"