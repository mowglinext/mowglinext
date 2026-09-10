#!/usr/bin/env bash

configure_lidar() {
  step "LiDAR configuration"

  # Reset generated rule
  LIDAR_UART_RULE=""

  # If preset values exist (from web composer or CLI), skip interactive prompts
  if [[ "${PRESET_LOADED:-false}" == "true" && -n "${LIDAR_TYPE:-}" ]]; then
    : "${LIDAR_PORT:=/dev/lidar}"

    # Keep STL27L's driver contract available even when an older/minimal
    # preset only records the lidar type.  Explicit preset/user overrides
    # remain authoritative.
    if [[ "${LIDAR_TYPE}" == "stl27l" ]]; then
      : "${LIDAR_MODEL:=LDLiDAR_STL27L}"
      : "${LIDAR_BAUD:=921600}"
    fi

    info "LiDAR pre-configured (skipping prompts)"

    # For UART connections, always let user confirm/change the port
    if [[ "${LIDAR_CONNECTION:-}" == "uart" ]]; then
      pick_uart_port "${LIDAR_UART_DEVICE:-/dev/ttyAMA5}"
      LIDAR_UART_DEVICE="$REPLY"
    fi
  else
    # Defaults based on PCB / GUI-ready
    : "${LIDAR_ENABLED:=true}"
    : "${LIDAR_TYPE:=ldlidar}"
    : "${LIDAR_MODEL:=LDLiDAR_LD19}"
    : "${LIDAR_CONNECTION:=uart}"
    : "${LIDAR_PORT:=/dev/lidar}"
    : "${LIDAR_UART_DEVICE:=/dev/ttyAMA5}"
    : "${LIDAR_BAUD:=230400}"

    echo ""
    echo "$MSG_LIDAR_TYPE"
    echo "  1) $MSG_LIDAR_NONE"
    echo "  2) RPLidar Slamtec (A1/A2/A3)"
    echo "  3) LDLiDAR (LD06 / LD14 / LD19)"
    echo "  4) STL27L"
    prompt "$MSG_CHOICE" "3"
    local lidar_choice="$REPLY"

    case "$lidar_choice" in
      1)
        LIDAR_ENABLED="false"
        LIDAR_TYPE="none"
        LIDAR_MODEL=""
        LIDAR_CONNECTION=""
        LIDAR_UART_DEVICE=""
        LIDAR_UART_RULE=""
        ;;
      2)
        LIDAR_ENABLED="true"
        LIDAR_TYPE="rplidar"
        LIDAR_MODEL="RPLIDAR_A1"
        LIDAR_BAUD="115200"
        ;;
      3)
        LIDAR_ENABLED="true"
        LIDAR_TYPE="ldlidar"
        LIDAR_MODEL="LDLiDAR_LD19"
        LIDAR_BAUD="230400"
        ;;
      4)
        LIDAR_ENABLED="true"
        LIDAR_TYPE="stl27l"
        LIDAR_MODEL="LDLiDAR_STL27L"
        LIDAR_BAUD="921600"
        ;;
      *)
        error "$MSG_LIDAR_INVALID_TYPE"
        return 1
        ;;
    esac

    if [ "$LIDAR_ENABLED" = "true" ]; then
      echo ""
      echo "$MSG_LIDAR_CONNECTION"
      echo "  1) USB"
      echo "  2) UART"
      prompt "$MSG_CHOICE" "2"
      local conn_choice="$REPLY"

      case "$conn_choice" in
        1)
          LIDAR_CONNECTION="usb"
          LIDAR_UART_DEVICE=""
          LIDAR_UART_RULE=""
          ;;
        2)
          LIDAR_CONNECTION="uart"
          pick_uart_port "/dev/ttyAMA5"
          LIDAR_UART_DEVICE="$REPLY"
          ;;
        *)
          error "$MSG_LIDAR_INVALID_CONNECTION"
          return 1
          ;;
      esac
    fi
  fi

  # Generate udev rule if UART connection
  if [[ "${LIDAR_CONNECTION:-}" == "uart" && -n "${LIDAR_UART_DEVICE:-}" ]]; then
    local lidar_kernel
    lidar_kernel="$(basename "$LIDAR_UART_DEVICE")"
    LIDAR_UART_RULE="KERNEL==\"${lidar_kernel}\", SYMLINK+=\"lidar\", MODE=\"0666\""
  fi

  echo ""
  info "LiDAR : enabled=${LIDAR_ENABLED:-false} type=${LIDAR_TYPE:-none} model=${LIDAR_MODEL:-none} connection=${LIDAR_CONNECTION:-none} port=$LIDAR_PORT uart=${LIDAR_UART_DEVICE:-none} baud=${LIDAR_BAUD:-0}"
}

run_lidar_configuration_step() {
  configure_lidar
}
