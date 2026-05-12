#!/usr/bin/env bash

upsert_env_key() {
  local file="$1"
  local key="$2"
  local value="$3"

  if grep -q "^${key}=" "$file" 2>/dev/null; then
    sed -i "s|^${key}=.*|${key}=${value}|" "$file"
  else
    echo "${key}=${value}" >> "$file"
  fi
}

setup_env() {
  step "Environment (.env)"

  local env_file="$REPO_DIR/docker/.env"
  mkdir -p "$REPO_DIR/docker"

  : "${ROS_DOMAIN_ID:=0}"
  : "${MOWER_IP:=10.0.0.161}"
  : "${DISABLE_BLUETOOTH:=true}"
  : "${ENABLE_FOXGLOVE:=true}"

  # GPS
  : "${GNSS_BACKEND:=gps}"
  : "${GPS_CONNECTION:=uart}"
  : "${GPS_PROTOCOL:=UBX}"
  : "${GPS_PORT:=/dev/gps}"
  : "${GPS_BY_ID:=}"
  : "${GPS_UART_DEVICE:=/dev/ttyAMA4}"
  : "${GPS_BAUD:=460800}"

  : "${GPS_DEBUG_ENABLED:=false}"
  : "${GPS_DEBUG_PORT:=/dev/gps_debug}"
  : "${GPS_DEBUG_UART_DEVICE:=/dev/ttyS0}"
  : "${GPS_DEBUG_BAUD:=115200}"

  # LiDAR
  : "${LIDAR_ENABLED:=true}"
  : "${LIDAR_TYPE:=ldlidar}"
  : "${LIDAR_MODEL:=LDLiDAR_LD19}"
  : "${LIDAR_CONNECTION:=uart}"
  : "${LIDAR_PORT:=/dev/lidar}"
  : "${LIDAR_UART_DEVICE:=/dev/ttyAMA5}"
  : "${LIDAR_BAUD:=230400}"

  # TF-Luna
  : "${TFLUNA_FRONT_ENABLED:=false}"
  : "${TFLUNA_FRONT_PORT:=/dev/tfluna_front}"
  : "${TFLUNA_FRONT_UART_DEVICE:=/dev/ttyAMA3}"
  : "${TFLUNA_FRONT_BAUD:=115200}"

  : "${TFLUNA_EDGE_ENABLED:=false}"
  : "${TFLUNA_EDGE_PORT:=/dev/tfluna_edge}"
  : "${TFLUNA_EDGE_UART_DEVICE:=/dev/ttyAMA2}"
  : "${TFLUNA_EDGE_BAUD:=115200}"

  # Images — select LiDAR image based on type
  : "${MOWGLI_ROS2_IMAGE:=${MOWGLI_ROS2_IMAGE_DEFAULT}}"
  : "${GPS_IMAGE:=${GPS_IMAGE_DEFAULT}}"
  : "${UNICORE_IMAGE:=${UNICORE_IMAGE_DEFAULT}}"
  : "${GUI_IMAGE:=${GUI_IMAGE_DEFAULT}}"
  : "${MAVROS_IMAGE:=${MAVROS_IMAGE_DEFAULT}}"
  : "${NMEA_IMAGE:=${NMEA_IMAGE_DEFAULT}}"

  if [[ -z "${LIDAR_IMAGE:-}" ]]; then
    case "${LIDAR_TYPE:-ldlidar}" in
      rplidar) LIDAR_IMAGE="${LIDAR_RPLIDAR_IMAGE_DEFAULT}" ;;
      stl27l)  LIDAR_IMAGE="${LIDAR_STL27L_IMAGE_DEFAULT}" ;;
      *)       LIDAR_IMAGE="${LIDAR_LDLIDAR_IMAGE_DEFAULT}" ;;
    esac
  fi

  # MAVROS / backend
  : "${HARDWARE_BACKEND:=mowgli}"
  : "${MAVROS_AUTOPILOT:=ardupilot}"
  : "${MAVROS_BY_ID:=}"
  : "${MAVROS_PORT:=/dev/mavros}"
  : "${MAVROS_BAUD:=921600}"
  : "${MAVROS_GCS_URL:=udp-b://@255.255.255.255:14550}" # udp-b = broadcast, udp = unicast, empty = disabled
  : "${MAVROS_TGT_SYSTEM:=1}"
  : "${MAVROS_TGT_COMPONENT:=1}"

  # Si MAVROS → GNSS backend désactivé
  if [[ "$HARDWARE_BACKEND" == "mavros" ]]; then
    GNSS_BACKEND="disabled"
  fi

  local enable_mavros="false"
  if [[ "$HARDWARE_BACKEND" == "mavros" ]]; then
    enable_mavros="true"
  fi
  MAVROS_ENABLED="$enable_mavros"

  touch "$env_file"

  upsert_env_key "$env_file" "ROS_DOMAIN_ID" "$ROS_DOMAIN_ID"
  upsert_env_key "$env_file" "MOWER_IP" "$MOWER_IP"
  upsert_env_key "$env_file" "DISABLE_BLUETOOTH" "$DISABLE_BLUETOOTH"
  upsert_env_key "$env_file" "ENABLE_FOXGLOVE" "$ENABLE_FOXGLOVE"

  upsert_env_key "$env_file" "GNSS_BACKEND" "$GNSS_BACKEND"
  upsert_env_key "$env_file" "GPS_CONNECTION" "$GPS_CONNECTION"
  upsert_env_key "$env_file" "GPS_PROTOCOL" "$GPS_PROTOCOL"
  upsert_env_key "$env_file" "GPS_PORT" "$GPS_PORT"
  upsert_env_key "$env_file" "GPS_BY_ID" "$GPS_BY_ID"
  upsert_env_key "$env_file" "GPS_UART_DEVICE" "$GPS_UART_DEVICE"
  upsert_env_key "$env_file" "GPS_BAUD" "$GPS_BAUD"
  upsert_env_key "$env_file" "GPS_DEBUG_ENABLED" "$GPS_DEBUG_ENABLED"
  upsert_env_key "$env_file" "GPS_DEBUG_PORT" "$GPS_DEBUG_PORT"
  upsert_env_key "$env_file" "GPS_DEBUG_UART_DEVICE" "$GPS_DEBUG_UART_DEVICE"
  upsert_env_key "$env_file" "GPS_DEBUG_BAUD" "$GPS_DEBUG_BAUD"

  upsert_env_key "$env_file" "LIDAR_ENABLED" "$LIDAR_ENABLED"
  upsert_env_key "$env_file" "LIDAR_TYPE" "$LIDAR_TYPE"
  upsert_env_key "$env_file" "LIDAR_MODEL" "$LIDAR_MODEL"
  upsert_env_key "$env_file" "LIDAR_CONNECTION" "$LIDAR_CONNECTION"
  upsert_env_key "$env_file" "LIDAR_PORT" "$LIDAR_PORT"
  upsert_env_key "$env_file" "LIDAR_UART_DEVICE" "$LIDAR_UART_DEVICE"
  upsert_env_key "$env_file" "LIDAR_BAUD" "$LIDAR_BAUD"

  upsert_env_key "$env_file" "TFLUNA_FRONT_ENABLED" "$TFLUNA_FRONT_ENABLED"
  upsert_env_key "$env_file" "TFLUNA_FRONT_PORT" "$TFLUNA_FRONT_PORT"
  upsert_env_key "$env_file" "TFLUNA_FRONT_UART_DEVICE" "$TFLUNA_FRONT_UART_DEVICE"
  upsert_env_key "$env_file" "TFLUNA_FRONT_BAUD" "$TFLUNA_FRONT_BAUD"

  upsert_env_key "$env_file" "TFLUNA_EDGE_ENABLED" "$TFLUNA_EDGE_ENABLED"
  upsert_env_key "$env_file" "TFLUNA_EDGE_PORT" "$TFLUNA_EDGE_PORT"
  upsert_env_key "$env_file" "TFLUNA_EDGE_UART_DEVICE" "$TFLUNA_EDGE_UART_DEVICE"
  upsert_env_key "$env_file" "TFLUNA_EDGE_BAUD" "$TFLUNA_EDGE_BAUD"

  upsert_env_key "$env_file" "MOWGLI_ROS2_IMAGE" "$MOWGLI_ROS2_IMAGE"
  upsert_env_key "$env_file" "GPS_IMAGE" "$GPS_IMAGE"
  upsert_env_key "$env_file" "UNICORE_IMAGE" "$UNICORE_IMAGE"
  upsert_env_key "$env_file" "LIDAR_IMAGE" "$LIDAR_IMAGE"
  upsert_env_key "$env_file" "MAVROS_IMAGE" "$MAVROS_IMAGE"
  upsert_env_key "$env_file" "NMEA_IMAGE" "$NMEA_IMAGE"
  upsert_env_key "$env_file" "GUI_IMAGE" "$GUI_IMAGE"

  upsert_env_key "$env_file" "HARDWARE_BACKEND" "$HARDWARE_BACKEND"
  upsert_env_key "$env_file" "MAVROS_ENABLED" "$MAVROS_ENABLED"
  upsert_env_key "$env_file" "MAVROS_BY_ID" "$MAVROS_BY_ID"
  upsert_env_key "$env_file" "MAVROS_PORT" "$MAVROS_PORT"
  upsert_env_key "$env_file" "MAVROS_BAUD" "$MAVROS_BAUD"
  upsert_env_key "$env_file" "MAVROS_GCS_URL" "$MAVROS_GCS_URL"
  upsert_env_key "$env_file" "MAVROS_TGT_SYSTEM" "$MAVROS_TGT_SYSTEM"
  upsert_env_key "$env_file" "MAVROS_TGT_COMPONENT" "$MAVROS_TGT_COMPONENT"
  upsert_env_key "$env_file" "MAVROS_AUTOPILOT" "$MAVROS_AUTOPILOT"
  
  info "Backend selection : HARDWARE_BACKEND=$HARDWARE_BACKEND GNSS_BACKEND=$GNSS_BACKEND"
  info "Updated $env_file"
}
