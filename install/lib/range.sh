#!/usr/bin/env bash

configure_rangefinders() {
  step "Rangefinders configuration"

  # Reset generated rules
  TFLUNA_FRONT_UART_RULE=""
  TFLUNA_EDGE_UART_RULE=""

  : "${TFLUNA_FRONT_ENABLED:=false}"
  : "${TFLUNA_FRONT_PORT:=/dev/tfluna_front}"
  : "${TFLUNA_FRONT_UART_DEVICE:=/dev/ttyAMA3}"
  : "${TFLUNA_FRONT_BAUD:=115200}"

  : "${TFLUNA_EDGE_ENABLED:=false}"
  : "${TFLUNA_EDGE_PORT:=/dev/tfluna_edge}"
  : "${TFLUNA_EDGE_UART_DEVICE:=/dev/ttyAMA2}"
  : "${TFLUNA_EDGE_BAUD:=115200}"

  if ! feature_is_available tfluna; then
    if [[ "${TFLUNA_FRONT_ENABLED:-false}" == "true" || "${TFLUNA_EDGE_ENABLED:-false}" == "true" ]]; then
      warn_unavailable_feature_once \
        tfluna \
        "TF-Luna rangefinder services are not available on this branch yet; disabling TF-Luna options for this run."
    else
      info "TF-Luna rangefinder services are currently unavailable on this branch — skipping configuration"
    fi

    TFLUNA_FRONT_ENABLED="false"
    TFLUNA_EDGE_ENABLED="false"

    echo ""
    info "TF-Luna front : enabled=$TFLUNA_FRONT_ENABLED port=$TFLUNA_FRONT_PORT uart=${TFLUNA_FRONT_UART_DEVICE:-none} baud=$TFLUNA_FRONT_BAUD"
    info "TF-Luna edge  : enabled=$TFLUNA_EDGE_ENABLED port=$TFLUNA_EDGE_PORT uart=${TFLUNA_EDGE_UART_DEVICE:-none} baud=$TFLUNA_EDGE_BAUD"
    return 0
  fi

  # If preset values exist (from web composer or CLI), skip interactive prompts
  if [[ "${PRESET_LOADED:-false}" == "true" && -n "${TFLUNA_FRONT_ENABLED:-}" ]]; then
    info "Rangefinders pre-configured (skipping prompts)"

    # For enabled sensors, always let user confirm/change the UART port
    if [[ "${TFLUNA_FRONT_ENABLED}" == "true" ]]; then
      echo -e "\n${DIM}TF-Luna front UART:${NC}"
      pick_uart_port "${TFLUNA_FRONT_UART_DEVICE:-/dev/ttyAMA3}"
      TFLUNA_FRONT_UART_DEVICE="$REPLY"
    fi
    if [[ "${TFLUNA_EDGE_ENABLED}" == "true" ]]; then
      echo -e "\n${DIM}TF-Luna edge UART:${NC}"
      pick_uart_port "${TFLUNA_EDGE_UART_DEVICE:-/dev/ttyAMA2}"
      TFLUNA_EDGE_UART_DEVICE="$REPLY"
    fi
  else
    echo ""
    echo "$MSG_TFLUNA_CONFIG"
    echo "  1) $MSG_TFLUNA_NONE"
    echo "  2) $MSG_TFLUNA_FRONT_ONLY"
    echo "  3) $MSG_TFLUNA_EDGE_ONLY"
    echo "  4) $MSG_TFLUNA_FRONT_EDGE"
    prompt "$MSG_CHOICE" "1"
    local range_choice="$REPLY"

    case "$range_choice" in
      1)
        TFLUNA_FRONT_ENABLED="false"
        TFLUNA_EDGE_ENABLED="false"
        ;;
      2)
        TFLUNA_FRONT_ENABLED="true"
        TFLUNA_EDGE_ENABLED="false"
        echo -e "\n${DIM}TF-Luna front UART:${NC}"
        pick_uart_port "/dev/ttyAMA3"
        TFLUNA_FRONT_UART_DEVICE="$REPLY"
        ;;
      3)
        TFLUNA_FRONT_ENABLED="false"
        TFLUNA_EDGE_ENABLED="true"
        echo -e "\n${DIM}TF-Luna edge UART:${NC}"
        pick_uart_port "/dev/ttyAMA2"
        TFLUNA_EDGE_UART_DEVICE="$REPLY"
        ;;
      4)
        TFLUNA_FRONT_ENABLED="true"
        TFLUNA_EDGE_ENABLED="true"
        echo -e "\n${DIM}TF-Luna front UART:${NC}"
        pick_uart_port "/dev/ttyAMA3"
        TFLUNA_FRONT_UART_DEVICE="$REPLY"
        echo -e "\n${DIM}TF-Luna edge UART:${NC}"
        pick_uart_port "/dev/ttyAMA2"
        TFLUNA_EDGE_UART_DEVICE="$REPLY"
        ;;
      *)
        error "$MSG_TFLUNA_INVALID"
        return 1
        ;;
    esac
  fi

  # Generate udev rules based on enabled sensors
  if [[ "${TFLUNA_FRONT_ENABLED:-false}" == "true" && -n "${TFLUNA_FRONT_UART_DEVICE:-}" ]]; then
    local front_kernel
    front_kernel="$(basename "$TFLUNA_FRONT_UART_DEVICE")"
    TFLUNA_FRONT_UART_RULE="KERNEL==\"${front_kernel}\", SYMLINK+=\"tfluna_front\", MODE=\"0666\""
  fi
  if [[ "${TFLUNA_EDGE_ENABLED:-false}" == "true" && -n "${TFLUNA_EDGE_UART_DEVICE:-}" ]]; then
    local edge_kernel
    edge_kernel="$(basename "$TFLUNA_EDGE_UART_DEVICE")"
    TFLUNA_EDGE_UART_RULE="KERNEL==\"${edge_kernel}\", SYMLINK+=\"tfluna_edge\", MODE=\"0666\""
  fi

  echo ""
  info "TF-Luna front : enabled=$TFLUNA_FRONT_ENABLED port=$TFLUNA_FRONT_PORT uart=${TFLUNA_FRONT_UART_DEVICE:-none} baud=$TFLUNA_FRONT_BAUD"
  info "TF-Luna edge  : enabled=$TFLUNA_EDGE_ENABLED port=$TFLUNA_EDGE_PORT uart=${TFLUNA_EDGE_UART_DEVICE:-none} baud=$TFLUNA_EDGE_BAUD"
}

run_range_configuration_step() {
  configure_rangefinders
}
