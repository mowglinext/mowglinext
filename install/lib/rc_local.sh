#!/usr/bin/env bash

generate_rc_local() {
  step "UART serial init (rc.local)"

  if ! platform_supports_pi_uart_overlays; then
    info "Skipping rc.local UART init on this platform"
    return 0
  fi

  require_root_for "rc.local"

  local rclocal="/etc/rc.local"

  if [ -f "$rclocal" ] && ! grep -q "MOWGLI_UART_INIT" "$rclocal"; then
    $SUDO cp "$rclocal" "${rclocal}.bak"
    info "Saved existing rc.local to rc.local.bak"
  fi

  {
    echo '#!/bin/bash'
    echo '# MOWGLI_UART_INIT'
    echo 'exit 0'
  } | $SUDO tee "$rclocal" > /dev/null

  $SUDO chmod +x "$rclocal"

  if [ ! -f /etc/systemd/system/rc-local.service ]; then
    cat <<'EOF' | $SUDO tee /etc/systemd/system/rc-local.service > /dev/null
[Unit]
Description=/etc/rc.local Compatibility
ConditionPathExists=/etc/rc.local
After=network.target

[Service]
Type=forking
ExecStart=/etc/rc.local
TimeoutSec=0
StandardOutput=tty
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
  fi

  $SUDO systemctl daemon-reload
  $SUDO systemctl enable --now rc-local.service
  info "rc.local configured"
}
