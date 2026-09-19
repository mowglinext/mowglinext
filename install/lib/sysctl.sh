#!/usr/bin/env bash
# Kernel socket-buffer limits for Cyclone DDS discovery.
#
# ~40 DDS participants (every ROS node process across the containers) announce
# themselves in one burst at stack start. Cyclone asks the kernel for 1 MiB of
# receive buffer per socket but silently accepts less; Ubuntu's default
# net.core.rmem_max is 212992 B (208 KiB), so the burst overflows the SPDP
# sockets (UdpRcvbufErrors climbed to 275k on the test robot, 2026-09-15). With
# Cyclone 11 a dropped initial announcement is never repeated for a pruned peer
# locator, which left two participants mutually blind and Nav2 unable to start.
# 8 MiB is the value rmw_cyclonedds itself recommends (it warns below it since
# 4.1.5). Idempotent: rewrites the drop-in only when its content changed.

SYSCTL_DDS_FILE="/etc/sysctl.d/90-mowgli-dds.conf"

build_dds_sysctl_conf() {
  cat <<'EOF'
# MowgliNext: Cyclone DDS discovery bursts overflow the 208 KiB default socket
# buffers (UdpRcvbufErrors); Cyclone asks 1 MiB per socket, rmw_cyclonedds
# recommends 8 MiB. Installed by install/lib/sysctl.sh.
net.core.rmem_max = 8388608
net.core.rmem_default = 8388608
net.core.wmem_max = 8388608
EOF
}

install_dds_sysctl() {
  if [[ "$(uname -s)" != Linux ]] || ! command -v sysctl >/dev/null 2>&1; then
    warn "sysctl unavailable; skipping DDS socket-buffer tuning"
    return 0
  fi
  # Every other lib file that uses $SUDO sets it itself before first use; this
  # function used to rely on install_udev_rules (its only caller, always
  # immediately before it in both main()'s full flow and --only=udev) having
  # already called require_root_for as a side effect. Harmless while that
  # call order holds, but the same implicit-ordering assumption broke
  # install_host_updater under --only=updater (issue #632) — fix it here too
  # before this function can ever be reached any other way.
  require_root_for "DDS sysctl"
  local tmpfile
  tmpfile="$(mktemp)"
  build_dds_sysctl_conf > "$tmpfile"
  if [ -f "$SYSCTL_DDS_FILE" ] && cmp -s "$tmpfile" "$SYSCTL_DDS_FILE"; then
    info "DDS socket-buffer sysctl already up to date"
  else
    $SUDO install -m 0644 "$tmpfile" "$SYSCTL_DDS_FILE"
    info "DDS socket-buffer sysctl installed ($SYSCTL_DDS_FILE)"
  fi
  rm -f "$tmpfile"
  # Apply now (new sockets only; running containers pick it up on restart).
  $SUDO sysctl -q -p "$SYSCTL_DDS_FILE" || warn "sysctl -p $SYSCTL_DDS_FILE failed; values apply after reboot"
}
