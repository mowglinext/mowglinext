#!/usr/bin/env bash

show_banner() {
  echo ""
  echo -e "${GREEN}${BOLD}╔══════════════════════════════════════════════╗${NC}"
  echo -e "${GREEN}${BOLD}║       Mowgli Next — Setup & Diagnose     ║${NC}"
  echo -e "${GREEN}${BOLD}╚══════════════════════════════════════════════╝${NC}"
  echo ""
}

print_rerun_command() {
  echo -e "  ${BOLD}$(rerun_check_command)${NC}"
}

print_summary() {
  echo ""
  echo -e "${CYAN}${BOLD}══════════════════════════════════════════════${NC}"

  if [[ ${#ISSUES[@]} -eq 0 ]]; then
    echo -e "${GREEN}${BOLD}  All checks passed! Your mower is ready.${NC}"
    echo -e "${CYAN}${BOLD}══════════════════════════════════════════════${NC}"
    echo ""
    echo -e "  ${BOLD}Next steps:${NC}"
    echo -e "  1. Open the GUI in your browser"
    echo -e "  2. Record your garden boundary (Areas tab)"
    echo -e "  3. Set dock position (drive mower to dock, save pose)"
    echo -e "  4. Start mowing!"
  else
    echo -e "${YELLOW}${BOLD}  ${#ISSUES[@]} issue(s) to resolve:${NC}"
    echo -e "${CYAN}${BOLD}══════════════════════════════════════════════${NC}"
    echo ""
    local i=1
    for issue in "${ISSUES[@]}"; do
      echo -e "  ${YELLOW}${i}.${NC} $issue"
      ((i++))
    done
    echo ""
    echo -e "  ${DIM}Fix these issues, then re-run:${NC}"
    print_rerun_command
  fi

  echo ""
}

show_mowgli_loading() {
  [[ $- != *i* ]] && return

  local cyan='\e[0;36m'
  local nc='\e[0m'

  local frames=(
    "[■□□□□] Initializing Mowgli I I..."
    "[■■□□□] Loading ROS 2 stack..."
    "[■■■□□] Checking hardware links..."
    "[■■■■□] Preparing containers..."
    "[■■■■■] Ready."
  )

  for frame in "${frames[@]}"; do
    printf "\r\033[2K${cyan}%s${nc}" "$frame"
    sleep 0.22
  done
  printf "\n"
}
