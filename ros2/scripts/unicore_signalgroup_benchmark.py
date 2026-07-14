#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import os
import pathlib
import re
import shlex
import subprocess
import sys
import tempfile
import textwrap
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any


DEFAULT_CANDIDATES = ["3 6", "3 0", "2 0", "4 0", "7 0"]
DEFAULT_BAUD = 460800
DEFAULT_DEVICE = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
DEFAULT_MODEL = "UM982"
DEFAULT_PROFILE = "rover_high_precision"
DEFAULT_USER = "pepeuch"
DEFAULT_PASSWORD_ENV = "MOWGLI_SSH_PASSWORD"


def parse_args() -> argparse.Namespace:
  parser = argparse.ArgumentParser(
      description="Benchmark Unicore UM982 signal groups over SSH using runtime-only applies.")
  parser.add_argument("--host", required=True, help="Robot SSH host or IP")
  parser.add_argument("--port", type=int, default=22, help="Robot SSH port")
  parser.add_argument("--user", default=DEFAULT_USER, help="Robot SSH user")
  parser.add_argument(
      "--password-env",
      default=DEFAULT_PASSWORD_ENV,
      help="Environment variable that contains the SSH password; leave unset for key auth",
  )
  parser.add_argument(
      "--device",
      default=DEFAULT_DEVICE,
      help="Serial device path on the robot",
  )
  parser.add_argument(
      "--baud",
      type=int,
      default=DEFAULT_BAUD,
      help="Runtime/config baud to keep during the benchmark",
  )
  parser.add_argument(
      "--model",
      default=DEFAULT_MODEL,
      help="Resolved Unicore receiver model to pass to gnss_config_apply",
  )
  parser.add_argument(
      "--profile",
      default=DEFAULT_PROFILE,
      help="Receiver profile to apply for each candidate",
  )
  parser.add_argument(
      "--wait-seconds",
      type=int,
      default=90,
      help="Seconds to wait after restarting mowgli-gps before capturing diagnostics",
  )
  parser.add_argument(
      "--diagnostics-seconds",
      type=int,
      default=10,
      help="How long to capture /diagnostics after each candidate",
  )
  parser.add_argument(
      "--gps-status-timeout-seconds",
      type=int,
      default=12,
      help="Timeout for the /gps/status snapshot",
  )
  parser.add_argument(
      "--apply-timeout-ms",
      type=int,
      default=5000,
      help="Universal GNSS apply timeout in milliseconds",
  )
  parser.add_argument(
      "--restore-mode",
      choices=("previous", "best", "none"),
      default="previous",
      help="Signal group to restore after the benchmark",
  )
  parser.add_argument(
      "--previous-signal-group",
      default="3 6",
      help="Assumed pre-benchmark runtime signal group when --restore-mode previous is used",
  )
  parser.add_argument(
      "--output-dir",
      default="artifacts/unicore-signalgroup-benchmark",
      help="Local directory that receives the copied benchmark artifacts",
  )
  parser.add_argument(
      "--candidate",
      action="append",
      dest="candidates",
      help="Signal group candidate to test; may be supplied multiple times",
  )
  parser.add_argument(
      "--known-hosts-mode",
      choices=("accept-new", "off"),
      default="off",
      help="SSH host key handling. 'off' uses a disposable known_hosts file.",
  )
  return parser.parse_args()


def shell_quote_lines(items: list[str]) -> str:
  return "\n".join(f'  "{item}"' for item in items)


def parse_scalar(raw_value: str) -> Any:
  value = raw_value.strip()
  if value.startswith(("'", '"')) and value.endswith(("'", '"')) and len(value) >= 2:
    value = value[1:-1]
  lowered = value.lower()
  if lowered in {"true", "false"}:
    return lowered == "true"
  if lowered in {"nan", ".nan"}:
    return None
  if lowered in {"null", "~", "none"}:
    return None
  if re.fullmatch(r"-?\d+", value):
    try:
      return int(value)
    except ValueError:
      return value
  if re.fullmatch(r"-?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][+-]?\d+)?", value):
    try:
      return float(value)
    except ValueError:
      return value
  return value


def safe_float(value: Any) -> float | None:
  if value is None or isinstance(value, bool):
    return None
  if isinstance(value, (int, float)):
    return float(value)
  try:
    return float(value)
  except (TypeError, ValueError):
    return None


def safe_int(value: Any) -> int | None:
  if value is None or isinstance(value, bool):
    return None
  if isinstance(value, int):
    return value
  if isinstance(value, float):
    return int(value)
  try:
    return int(str(value))
  except (TypeError, ValueError):
    return None


def parse_gnss_status(path: pathlib.Path) -> dict[str, Any]:
  result: dict[str, Any] = {}
  if not path.exists():
    return result
  for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
    if raw_line.strip() == "---":
      break
    if raw_line.startswith("  "):
      continue
    if ":" not in raw_line:
      continue
    key, value = raw_line.split(":", 1)
    value = value.strip()
    if not value:
      continue
    result[key.strip()] = parse_scalar(value)
  return result


def parse_diagnostic_blocks(path: pathlib.Path) -> list[dict[str, Any]]:
  if not path.exists():
    return []
  blocks: list[dict[str, Any]] = []
  current: dict[str, Any] | None = None
  current_key: str | None = None
  for raw_line in path.read_text(encoding="utf-8", errors="replace").splitlines():
    line = raw_line.rstrip("\n")
    if line.startswith("- level:"):
      if current and str(current.get("name", "")).startswith("universal_gnss"):
        blocks.append(current)
      current = {"values": {}}
      current_key = None
      continue
    if current is None:
      continue
    if line.startswith("  name: "):
      current["name"] = line[len("  name: "):].strip()
      continue
    if line.startswith("  message: "):
      current["message"] = parse_scalar(line[len("  message: "):].strip())
      continue
    if line.startswith("  hardware_id: "):
      current["hardware_id"] = parse_scalar(line[len("  hardware_id: "):].strip())
      continue
    if line.startswith("  - key: "):
      current_key = line[len("  - key: "):].strip()
      continue
    if current_key and line.startswith("    value: "):
      current["values"][current_key] = parse_scalar(line[len("    value: "):].strip())
      continue
  if current and str(current.get("name", "")).startswith("universal_gnss"):
    blocks.append(current)
  return blocks


def latest_block(blocks: list[dict[str, Any]], name: str) -> dict[str, Any] | None:
  for block in reversed(blocks):
    if block.get("name") == name:
      return block
  return None


def latest_signal_masks(blocks: list[dict[str, Any]]) -> dict[str, Any]:
  masks: dict[str, Any] = {}
  for block in blocks:
    values = block.get("values", {})
    if "signal_mask" in values:
      masks[str(block.get("name", ""))] = values["signal_mask"]
  return masks


def latest_constellation_counters(blocks: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
  counters: dict[str, dict[str, Any]] = {}
  for block in blocks:
    name = str(block.get("name", ""))
    if not name.startswith("universal_gnss/rtcm_semantic/msm_"):
      continue
    if name.endswith("/msm_summary") or name == "universal_gnss/rtcm_semantic/msm_summary":
      continue
    values = dict(block.get("values", {}))
    constellation = str(values.get("constellation", name.rsplit("_", 2)[-2]))
    counters[constellation] = {
        "satellite_count": values.get("satellite_count"),
        "signal_count": values.get("signal_count"),
        "cell_count": values.get("cell_count"),
        "age_s": values.get("age_s"),
        "message_type": values.get("message_type"),
    }
  return counters


def flatten_constellation_counters(counters: dict[str, dict[str, Any]]) -> dict[str, Any]:
  flattened: dict[str, Any] = {}
  for constellation, values in counters.items():
    prefix = constellation.lower()
    for key, value in values.items():
      flattened[f"{prefix}_{key}"] = value
  return flattened


def read_text_if_exists(path: pathlib.Path) -> str:
  if not path.exists():
    return ""
  return path.read_text(encoding="utf-8", errors="replace")


def parse_signalgroup_write_log(path: pathlib.Path) -> dict[str, Any]:
  text = read_text_if_exists(path)
  result: dict[str, Any] = {
      "raw_attempted": bool(text.strip()),
      "raw_response_line": None,
      "runtime_accepted": None,
      "grammar_error": False,
  }
  for line in text.splitlines():
    upper = line.upper()
    if "CONFIG SIGNALGROUP" not in upper:
      continue
    result["raw_response_line"] = line.strip()
    if "PARSING FAILED" in upper or "GRAMMAR ERROR" in upper:
      result["runtime_accepted"] = False
      result["grammar_error"] = True
      return result
    if "RESPONSE: OK" in upper:
      result["runtime_accepted"] = True
      return result
  return result


def select_restore_signal_group(
    summaries: list[dict[str, Any]],
    restore_mode: str,
    previous_signal_group: str,
) -> str | None:
  if restore_mode == "none":
    return None
  if restore_mode == "previous":
    return previous_signal_group
  best = recommend_candidate(summaries)
  if best is None:
    return previous_signal_group
  return str(best.get("signal_group"))


def candidate_score(summary: dict[str, Any]) -> float:
  score = 0.0
  if summary.get("apply_ok"):
    score += 1_000_000.0
  if summary.get("fix_valid"):
    score += 100_000.0
  if summary.get("corrections_active"):
    score += 10_000.0
  score += float(safe_int(summary.get("rtk_mode")) or 0) * 5_000.0
  parser_rate = safe_float(summary.get("recent_parser_anomaly_rate_hz"))
  if parser_rate is not None:
    score -= parser_rate * 1_000.0
  score += float(safe_int(summary.get("satellites_used")) or 0) * 100.0
  score += float(safe_int(summary.get("satellites_visible")) or 0) * 10.0
  score += float(safe_float(summary.get("quality_percent")) or 0.0) * 5.0
  score += float(safe_float(summary.get("mean_cn0_db_hz")) or 0.0)
  return score


def candidate_viable(summary: dict[str, Any]) -> bool:
  if not summary.get("apply_ok"):
    return False
  if summary.get("signalgroup_runtime_accepted") is not True:
    return False
  satellites_used = safe_int(summary.get("satellites_used"))
  if satellites_used is not None and satellites_used <= 0:
    return False
  return True


def recommend_candidate(summaries: list[dict[str, Any]]) -> dict[str, Any] | None:
  viable = [summary for summary in summaries if candidate_viable(summary)]
  if not viable:
    return None
  return max(viable, key=candidate_score)


def format_recommendation(best: dict[str, Any] | None) -> str:
  if best is None:
    return (
        "No candidate met the runtime-only acceptance criteria "
        "(official or raw CONFIG acknowledged, with satellites_used > 0), "
        "so no SIGNALGROUP recommendation could be derived."
    )
  rate = best.get("recent_parser_anomaly_rate_hz")
  return (
      f"Recommended runtime-only SIGNALGROUP: {best['signal_group']} "
      f"(score={candidate_score(best):.1f}, fix_valid={best.get('fix_valid')}, "
      f"rtk_mode={best.get('rtk_mode')}, satellites_used={best.get('satellites_used')}, "
      f"mean_cn0_db_hz={best.get('mean_cn0_db_hz')}, "
      f"recent_parser_anomaly_rate_hz={rate})."
  )


@dataclass
class SSHClient:
  host: str
  user: str
  port: int
  password: str | None
  known_hosts_mode: str

  def _ssh_options(self) -> list[str]:
    options: list[str] = []
    if self.known_hosts_mode == "off":
      options.extend(
          [
              "-o",
              "StrictHostKeyChecking=no",
              "-o",
              "UserKnownHostsFile=/dev/null",
          ])
    else:
      options.extend(["-o", "StrictHostKeyChecking=accept-new"])
    options.extend(
        [
            "-o",
            "PreferredAuthentications=password,publickey,keyboard-interactive",
            "-o",
            "PubkeyAuthentication=yes",
        ])
    return options

  def _run(self, command: list[str], *, stdin_text: str | None = None) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    askpass_script: pathlib.Path | None = None
    full_command = command
    if self.password:
      helper = tempfile.NamedTemporaryFile("w", delete=False, encoding="utf-8")
      helper.write("#!/bin/sh\n")
      helper.write("printf '%s\\n' \"$MOWGLI_BENCHMARK_PASSWORD\"\n")
      helper.close()
      askpass_script = pathlib.Path(helper.name)
      askpass_script.chmod(0o700)
      env["DISPLAY"] = "dummy"
      env["SSH_ASKPASS"] = str(askpass_script)
      env["SSH_ASKPASS_REQUIRE"] = "force"
      env["MOWGLI_BENCHMARK_PASSWORD"] = self.password
      full_command = ["setsid", "-w"] + command
    try:
      return subprocess.run(
          full_command,
          input=stdin_text,
          text=True,
          capture_output=True,
          check=True,
          env=env,
      )
    finally:
      if askpass_script is not None:
        askpass_script.unlink(missing_ok=True)

  def ssh(self, remote_command: str, *, stdin_text: str | None = None) -> subprocess.CompletedProcess[str]:
    command = ["ssh"] + self._ssh_options() + ["-p", str(self.port), f"{self.user}@{self.host}", remote_command]
    return self._run(command, stdin_text=stdin_text)

  def scp_from(self, remote_path: str, local_path: pathlib.Path) -> subprocess.CompletedProcess[str]:
    command = ["scp", "-r"] + self._ssh_options() + ["-P", str(self.port)]
    source = f"{self.user}@{self.host}:{remote_path}"
    command.extend([source, str(local_path)])
    return self._run(command)


def build_remote_benchmark_script(args: argparse.Namespace, remote_results_dir: str) -> str:
  candidates = args.candidates or DEFAULT_CANDIDATES
  candidate_lines = shell_quote_lines(candidates)
  return textwrap.dedent(
      f"""\
      #!/usr/bin/env bash
      set -euo pipefail

      RESULTS_DIR={shlex.quote(remote_results_dir)}
      DEVICE={shlex.quote(args.device)}
      BAUD={shlex.quote(str(args.baud))}
      MODEL={shlex.quote(args.model)}
      PROFILE={shlex.quote(args.profile)}
      WAIT_SECONDS={shlex.quote(str(args.wait_seconds))}
      DIAGNOSTICS_SECONDS={shlex.quote(str(args.diagnostics_seconds))}
      GPS_STATUS_TIMEOUT_SECONDS={shlex.quote(str(args.gps_status_timeout_seconds))}
      APPLY_TIMEOUT_MS={shlex.quote(str(args.apply_timeout_ms))}
      BASE_SIGNAL_GROUP="3 6"
      GPS_IMAGE="$(docker inspect --format '{{{{.Config.Image}}}}' mowgli-gps)"
      CANDIDATES=(
      {candidate_lines}
      )

      mkdir -p "$RESULTS_DIR"

      capture_state() {{
        local tag="$1"
        docker exec mowgli-ros2 bash -lc 'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout '"$GPS_STATUS_TIMEOUT_SECONDS"' ros2 topic echo /gps/status --once' \
          >"$RESULTS_DIR/${{tag}}_gps_status.yaml" 2>"$RESULTS_DIR/${{tag}}_gps_status.stderr" || true
        docker exec mowgli-ros2 bash -lc 'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout '"$DIAGNOSTICS_SECONDS"' ros2 topic echo /diagnostics' \
          >"$RESULTS_DIR/${{tag}}_diagnostics_raw.yaml" 2>"$RESULTS_DIR/${{tag}}_diagnostics_raw.stderr" || true
        docker stats --no-stream --format '{{{{json .}}}}' mowgli-gps >"$RESULTS_DIR/${{tag}}_docker_stats.json" 2>"$RESULTS_DIR/${{tag}}_docker_stats.stderr" || true
        docker inspect --format '{{{{json .State}}}}' mowgli-gps >"$RESULTS_DIR/${{tag}}_docker_state.json" 2>"$RESULTS_DIR/${{tag}}_docker_state.stderr" || true
      }}

      plan_profile() {{
        local tag="$1"
        local signal_group="$2"
        set +e
        docker run --rm --network host \
          --entrypoint /opt/gnss_sidecar/bin/gnss_config_plan \
          "$GPS_IMAGE" \
          --json \
          --config-baud "$BAUD" \
          --rate-hz 10 \
          --model "$MODEL" \
          --signal-group "$signal_group" \
          unicore "$PROFILE" \
          >"$RESULTS_DIR/${{tag}}_plan.json" 2>"$RESULTS_DIR/${{tag}}_plan.stderr"
        local plan_exit_code=$?
        set -e
        printf '%s\\n' "$plan_exit_code" >"$RESULTS_DIR/${{tag}}_plan.exitcode"
        return "$plan_exit_code"
      }}

      apply_profile() {{
        local tag="$1"
        local signal_group="$2"
        set +e
        docker run --rm --privileged --network host -v /dev:/dev \
          --entrypoint /opt/gnss_sidecar/bin/gnss_config_apply \
          "$GPS_IMAGE" \
          --json \
          --family unicore \
          --device "$DEVICE" \
          --baud "$BAUD" \
          --profile "$PROFILE" \
          --apply-mode runtime-only \
          --config-baud "$BAUD" \
          --rate-hz 10 \
          --timeout-ms "$APPLY_TIMEOUT_MS" \
          --confirm \
          --model "$MODEL" \
          --signal-group "$signal_group" \
          >"$RESULTS_DIR/${{tag}}_apply_profile.json" 2>"$RESULTS_DIR/${{tag}}_apply_profile.stderr"
        local apply_exit_code=$?
        set -e
        printf '%s\\n' "$apply_exit_code" >"$RESULTS_DIR/${{tag}}_apply_profile.exitcode"
        return "$apply_exit_code"
      }}

      apply_profile_base() {{
        local tag="$1"
        set +e
        docker run --rm --privileged --network host -v /dev:/dev \
          --entrypoint /opt/gnss_sidecar/bin/gnss_config_apply \
          "$GPS_IMAGE" \
          --json \
          --family unicore \
          --device "$DEVICE" \
          --baud "$BAUD" \
          --profile "$PROFILE" \
          --apply-mode runtime-only \
          --config-baud "$BAUD" \
          --rate-hz 10 \
          --timeout-ms "$APPLY_TIMEOUT_MS" \
          --confirm \
          --model "$MODEL" \
          --signal-group "$BASE_SIGNAL_GROUP" \
          >"$RESULTS_DIR/${{tag}}_fallback_apply_profile.json" 2>"$RESULTS_DIR/${{tag}}_fallback_apply_profile.stderr"
        local fallback_apply_exit_code=$?
        set -e
        printf '%s\\n' "$fallback_apply_exit_code" >"$RESULTS_DIR/${{tag}}_fallback_apply_profile.exitcode"
        return "$fallback_apply_exit_code"
      }}

      write_signal_group_raw() {{
        local tag="$1"
        local signal_group="$2"
        python3 - "$DEVICE" "$BAUD" "$signal_group" >"$RESULTS_DIR/${{tag}}_signalgroup_write.log" 2>"$RESULTS_DIR/${{tag}}_signalgroup_write.stderr" <<'PY'
import os
import select
import sys
import termios
import time

device = sys.argv[1]
baud = int(sys.argv[2])
signal_group = sys.argv[3]

baud_map = dict([
    (9600, termios.B9600),
    (19200, termios.B19200),
    (38400, termios.B38400),
    (57600, termios.B57600),
    (115200, termios.B115200),
    (230400, getattr(termios, "B230400", None)),
    (460800, getattr(termios, "B460800", None)),
    (921600, getattr(termios, "B921600", None)),
])
speed = baud_map.get(baud)
if speed is None:
  print("unsupported baud for raw signal-group write:", baud, file=sys.stderr)
  sys.exit(2)

fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
try:
  attrs = termios.tcgetattr(fd)
  attrs[0] = 0
  attrs[1] = 0
  attrs[2] &= ~termios.CSIZE
  attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
  attrs[2] &= ~termios.PARENB
  attrs[2] &= ~termios.CSTOPB
  if hasattr(termios, "CRTSCTS"):
    attrs[2] &= ~termios.CRTSCTS
  attrs[3] = 0
  attrs[4] = speed
  attrs[5] = speed
  attrs[6][termios.VMIN] = 0
  attrs[6][termios.VTIME] = 0
  termios.tcsetattr(fd, termios.TCSANOW, attrs)
  try:
    termios.tcflush(fd, termios.TCIOFLUSH)
  except termios.error:
    pass

  payload = ("CONFIG SIGNALGROUP " + signal_group + "\\r\\nVERSIONA\\r\\n").encode("ascii")
  os.write(fd, payload)
  deadline = time.monotonic() + 3.0
  chunks: list[bytes] = []
  while time.monotonic() < deadline:
    readable, _, _ = select.select([fd], [], [], 0.25)
    if fd not in readable:
      continue
    try:
      data = os.read(fd, 4096)
    except BlockingIOError:
      continue
    if data:
      chunks.append(data)
  text = b"".join(chunks).decode("ascii", "replace")
  if text:
    print(text, end="")
  config_response = None
  for line in text.splitlines():
    upper = line.upper()
    if "CONFIG SIGNALGROUP" not in upper:
      continue
    config_response = upper
    break
  if config_response is not None:
    if "PARSING FAILED" in config_response or "GRAMMAR ERROR" in config_response:
      sys.exit(6)
    if "RESPONSE: OK" in config_response:
      sys.exit(0)
  sys.exit(4 if text else 5)
finally:
  os.close(fd)
PY
        local raw_exit_code=$?
        printf '%s\\n' "$raw_exit_code" >"$RESULTS_DIR/${{tag}}_signalgroup_write.exitcode"
        return "$raw_exit_code"
      }}

      apply_candidate() {{
        local signal_group="$1"
        local tag="${{signal_group// /_}}"
        local combined_exit_code=0
        docker stop mowgli-gps >"$RESULTS_DIR/${{tag}}_docker_stop.log" 2>&1 || true

        plan_profile "$tag" "$signal_group" || true

        if apply_profile "$tag" "$signal_group"; then
          printf '%s\\n' "official_apply" >"$RESULTS_DIR/${{tag}}_apply.mode"
          printf '%s\\n' "0" >"$RESULTS_DIR/${{tag}}_fallback_apply_profile.exitcode"
          printf '%s\\n' "0" >"$RESULTS_DIR/${{tag}}_signalgroup_write.exitcode"
          : >"$RESULTS_DIR/${{tag}}_fallback_apply_profile.json"
          : >"$RESULTS_DIR/${{tag}}_fallback_apply_profile.stderr"
          : >"$RESULTS_DIR/${{tag}}_signalgroup_write.log"
          : >"$RESULTS_DIR/${{tag}}_signalgroup_write.stderr"
        else
          printf '%s\\n' "official_apply_failed_then_raw_signalgroup" >"$RESULTS_DIR/${{tag}}_apply.mode"
          if apply_profile_base "$tag"; then
            if ! write_signal_group_raw "$tag" "$signal_group"; then
              combined_exit_code=10
            fi
          else
            combined_exit_code=11
            printf '%s\\n' "9" >"$RESULTS_DIR/${{tag}}_signalgroup_write.exitcode"
            : >"$RESULTS_DIR/${{tag}}_signalgroup_write.log"
            : >"$RESULTS_DIR/${{tag}}_signalgroup_write.stderr"
          fi
        fi

        printf '%s\\n' "$combined_exit_code" >"$RESULTS_DIR/${{tag}}_apply.exitcode"

        docker start mowgli-gps >"$RESULTS_DIR/${{tag}}_docker_start.log" 2>&1
        sleep "$WAIT_SECONDS"
        capture_state "$tag"
      }}

      printf '%s\\n' "$GPS_IMAGE" >"$RESULTS_DIR/gps_image.txt"
      printf '%s\\n' "$DEVICE" >"$RESULTS_DIR/device.txt"
      printf '%s\\n' "$BAUD" >"$RESULTS_DIR/baud.txt"
      printf '%s\\n' "$MODEL" >"$RESULTS_DIR/model.txt"

      for candidate in "${{CANDIDATES[@]}}"; do
        apply_candidate "$candidate"
      done

      printf '%s\\n' "$RESULTS_DIR"
      """)


def build_remote_restore_script(
    args: argparse.Namespace,
    remote_results_dir: str,
    restore_signal_group: str,
) -> str:
  return textwrap.dedent(
      f"""\
      #!/usr/bin/env bash
      set -euo pipefail

      RESULTS_DIR={shlex.quote(remote_results_dir)}
      RESTORE_SIGNAL_GROUP={shlex.quote(restore_signal_group)}
      DEVICE={shlex.quote(args.device)}
      BAUD={shlex.quote(str(args.baud))}
      MODEL={shlex.quote(args.model)}
      PROFILE={shlex.quote(args.profile)}
      APPLY_TIMEOUT_MS={shlex.quote(str(args.apply_timeout_ms))}
      BASE_SIGNAL_GROUP="3 6"
      GPS_IMAGE="$(docker inspect --format '{{{{.Config.Image}}}}' mowgli-gps)"
      tag="restore_${{RESTORE_SIGNAL_GROUP// /_}}"

      docker stop mowgli-gps >"$RESULTS_DIR/${{tag}}_docker_stop.log" 2>&1 || true
      set +e
      docker run --rm --privileged --network host -v /dev:/dev \
        --entrypoint /opt/gnss_sidecar/bin/gnss_config_apply \
        "$GPS_IMAGE" \
        --json \
        --family unicore \
        --device "$DEVICE" \
        --baud "$BAUD" \
        --profile "$PROFILE" \
        --apply-mode runtime-only \
        --config-baud "$BAUD" \
        --rate-hz 10 \
        --timeout-ms "$APPLY_TIMEOUT_MS" \
        --confirm \
        --model "$MODEL" \
        --signal-group "$BASE_SIGNAL_GROUP" \
        >"$RESULTS_DIR/${{tag}}_apply_profile.json" 2>"$RESULTS_DIR/${{tag}}_apply_profile.stderr"
      apply_exit_code=$?
      set -e
      printf '%s\\n' "$apply_exit_code" >"$RESULTS_DIR/${{tag}}_apply_profile.exitcode"
      if [[ "$apply_exit_code" -eq 0 && "$RESTORE_SIGNAL_GROUP" != "$BASE_SIGNAL_GROUP" ]]; then
        set +e
        python3 - "$DEVICE" "$BAUD" "$RESTORE_SIGNAL_GROUP" >"$RESULTS_DIR/${{tag}}_signalgroup_write.log" 2>"$RESULTS_DIR/${{tag}}_signalgroup_write.stderr" <<'PY'
import os
import select
import sys
import termios
import time

device = sys.argv[1]
baud = int(sys.argv[2])
signal_group = sys.argv[3]

baud_map = dict([
    (9600, termios.B9600),
    (19200, termios.B19200),
    (38400, termios.B38400),
    (57600, termios.B57600),
    (115200, termios.B115200),
    (230400, getattr(termios, "B230400", None)),
    (460800, getattr(termios, "B460800", None)),
    (921600, getattr(termios, "B921600", None)),
])
speed = baud_map.get(baud)
if speed is None:
  print("unsupported baud for raw signal-group write:", baud, file=sys.stderr)
  sys.exit(2)

fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
try:
  attrs = termios.tcgetattr(fd)
  attrs[0] = 0
  attrs[1] = 0
  attrs[2] &= ~termios.CSIZE
  attrs[2] |= termios.CS8 | termios.CLOCAL | termios.CREAD
  attrs[2] &= ~termios.PARENB
  attrs[2] &= ~termios.CSTOPB
  if hasattr(termios, "CRTSCTS"):
    attrs[2] &= ~termios.CRTSCTS
  attrs[3] = 0
  attrs[4] = speed
  attrs[5] = speed
  attrs[6][termios.VMIN] = 0
  attrs[6][termios.VTIME] = 0
  termios.tcsetattr(fd, termios.TCSANOW, attrs)
  try:
    termios.tcflush(fd, termios.TCIOFLUSH)
  except termios.error:
    pass

  payload = ("CONFIG SIGNALGROUP " + signal_group + "\\r\\nVERSIONA\\r\\n").encode("ascii")
  os.write(fd, payload)
  deadline = time.monotonic() + 3.0
  chunks: list[bytes] = []
  while time.monotonic() < deadline:
    readable, _, _ = select.select([fd], [], [], 0.25)
    if fd not in readable:
      continue
    try:
      data = os.read(fd, 4096)
    except BlockingIOError:
      continue
    if data:
      chunks.append(data)
  text = b"".join(chunks).decode("ascii", "replace")
  if text:
    print(text, end="")
  config_response = None
  for line in text.splitlines():
    upper = line.upper()
    if "CONFIG SIGNALGROUP" not in upper:
      continue
    config_response = upper
    break
  if config_response is not None:
    if "PARSING FAILED" in config_response or "GRAMMAR ERROR" in config_response:
      sys.exit(6)
    if "RESPONSE: OK" in config_response:
      sys.exit(0)
  sys.exit(4 if text else 5)
finally:
  os.close(fd)
PY
        signalgroup_exit_code=$?
        set -e
      else
        signalgroup_exit_code=0
        : >"$RESULTS_DIR/${{tag}}_signalgroup_write.log"
        : >"$RESULTS_DIR/${{tag}}_signalgroup_write.stderr"
      fi
      printf '%s\\n' "$signalgroup_exit_code" >"$RESULTS_DIR/${{tag}}_signalgroup_write.exitcode"
      if [[ "$apply_exit_code" -eq 0 && "$signalgroup_exit_code" -eq 0 ]]; then
        printf '%s\\n' "0" >"$RESULTS_DIR/${{tag}}_apply.exitcode"
      else
        printf '%s\\n' "1" >"$RESULTS_DIR/${{tag}}_apply.exitcode"
      fi
      docker start mowgli-gps >"$RESULTS_DIR/${{tag}}_docker_start.log" 2>&1
      sleep 30
      docker exec mowgli-ros2 bash -lc 'source /opt/ros/kilted/setup.bash && source /ros2_ws/install/setup.bash && timeout 12 ros2 topic echo /gps/status --once' \
        >"$RESULTS_DIR/${{tag}}_gps_status.yaml" 2>"$RESULTS_DIR/${{tag}}_gps_status.stderr" || true
      """)


def summarize_candidate(candidate: str, candidate_dir: pathlib.Path) -> dict[str, Any]:
  tag = candidate.replace(" ", "_")
  gps_status = parse_gnss_status(candidate_dir / f"{tag}_gps_status.yaml")
  blocks = parse_diagnostic_blocks(candidate_dir / f"{tag}_diagnostics_raw.yaml")
  summary_block = latest_block(blocks, "universal_gnss/summary")
  parser_block = latest_block(blocks, "universal_gnss/parser_counters")
  discovery_block = latest_block(blocks, "universal_gnss/discovery")
  rtcm_active_block = latest_block(blocks, "universal_gnss/receiver_rtcm_active")
  all_names = sorted({str(block.get("name", "")) for block in blocks})
  signal_masks = latest_signal_masks(blocks)
  constellation_counters = latest_constellation_counters(blocks)
  raw_signalgroup = parse_signalgroup_write_log(candidate_dir / f"{tag}_signalgroup_write.log")
  summary: dict[str, Any] = {
      "candidate": tag,
      "signal_group": candidate,
      "apply_exitcode": safe_int(read_text_if_exists(candidate_dir / f"{tag}_apply.exitcode").strip() or None),
      "apply_ok": (read_text_if_exists(candidate_dir / f"{tag}_apply.exitcode").strip() == "0"),
      "apply_mode": read_text_if_exists(candidate_dir / f"{tag}_apply.mode").strip() or None,
      "plan_exitcode": safe_int(read_text_if_exists(candidate_dir / f"{tag}_plan.exitcode").strip() or None),
      "profile_apply_exitcode": safe_int(
          read_text_if_exists(candidate_dir / f"{tag}_apply_profile.exitcode").strip() or None),
      "fallback_profile_apply_exitcode": safe_int(
          read_text_if_exists(candidate_dir / f"{tag}_fallback_apply_profile.exitcode").strip() or None),
      "signalgroup_write_exitcode": safe_int(
          read_text_if_exists(candidate_dir / f"{tag}_signalgroup_write.exitcode").strip() or None),
      "gps_status": gps_status,
      "diagnostic_block_names": all_names,
      "signal_masks": signal_masks,
      "constellation_counters": constellation_counters,
      "satsinfo_diagnostics_present": any("SATSINFO" in name for name in all_names),
      "bestsat_diagnostics_present": any("BESTSAT" in name or "BESTSATA" in name for name in all_names),
      "signalgroup_raw_attempted": raw_signalgroup["raw_attempted"],
      "signalgroup_raw_response_line": raw_signalgroup["raw_response_line"],
      "signalgroup_runtime_grammar_error": raw_signalgroup["grammar_error"],
      "apply_profile_stdout_file": f"{tag}_apply_profile.json",
      "apply_profile_stderr_file": f"{tag}_apply_profile.stderr",
      "fallback_apply_profile_stdout_file": f"{tag}_fallback_apply_profile.json",
      "fallback_apply_profile_stderr_file": f"{tag}_fallback_apply_profile.stderr",
      "plan_stdout_file": f"{tag}_plan.json",
      "plan_stderr_file": f"{tag}_plan.stderr",
      "signalgroup_write_log_file": f"{tag}_signalgroup_write.log",
      "signalgroup_write_stderr_file": f"{tag}_signalgroup_write.stderr",
      "gps_status_file": f"{tag}_gps_status.yaml",
      "diagnostics_raw_file": f"{tag}_diagnostics_raw.yaml",
  }
  if summary["apply_mode"] == "official_apply":
    summary["signalgroup_runtime_accepted"] = True
  else:
    summary["signalgroup_runtime_accepted"] = raw_signalgroup["runtime_accepted"] is True
  if summary_block:
    summary.update(summary_block.get("values", {}))
  if parser_block:
    summary.update(parser_block.get("values", {}))
  if discovery_block:
    summary.update({f"discovery_{key}": value for key, value in discovery_block.get("values", {}).items()})
  if rtcm_active_block:
    summary.update({f"rtcm_{key}": value for key, value in rtcm_active_block.get("values", {}).items()})
  summary.update(gps_status)
  summary.update(flatten_constellation_counters(constellation_counters))
  return summary


def copy_remote_results(
    ssh_client: SSHClient,
    remote_results_dir: str,
    local_output_dir: pathlib.Path,
) -> None:
  local_output_dir.parent.mkdir(parents=True, exist_ok=True)
  local_output_dir.mkdir(parents=True, exist_ok=True)
  ssh_client.scp_from(f"{remote_results_dir}/.", local_output_dir)


def write_json(path: pathlib.Path, payload: Any) -> None:
  path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def write_csv(path: pathlib.Path, summaries: list[dict[str, Any]]) -> None:
  columns = [
      "signal_group",
      "apply_ok",
      "apply_exitcode",
      "apply_mode",
      "profile_apply_exitcode",
      "signalgroup_write_exitcode",
      "signalgroup_runtime_accepted",
      "signalgroup_runtime_grammar_error",
      "fix_valid",
      "rtk_mode",
      "quality_percent",
      "satellites_used",
      "satellites_visible",
      "satellites_tracked",
      "mean_cn0_db_hz",
      "max_cn0_db_hz",
      "recent_parser_anomaly_rate_hz",
      "recent_malformed_records",
      "recent_rejected_records",
      "malformed_records_total",
      "rejected_records_total",
      "parser_anomalies_total",
      "unicore_records_parsed",
      "unicore_records_rejected",
      "unicore_malformed_lines",
      "parser_healthy",
      "receiver_healthy",
      "transport_healthy",
      "correction_available",
      "stale_data",
      "discovery_selected_baud",
      "discovery_receiver_model",
      "gps_satellite_count",
      "gps_signal_count",
      "gps_cell_count",
      "glonass_satellite_count",
      "glonass_signal_count",
      "glonass_cell_count",
      "galileo_satellite_count",
      "galileo_signal_count",
      "galileo_cell_count",
      "beidou_satellite_count",
      "beidou_signal_count",
      "beidou_cell_count",
      "satsinfo_diagnostics_present",
      "bestsat_diagnostics_present",
  ]
  with path.open("w", encoding="utf-8", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=columns)
    writer.writeheader()
    for summary in summaries:
      row = {column: summary.get(column) for column in columns}
      writer.writerow(row)


def main() -> int:
  args = parse_args()
  password = os.environ.get(args.password_env)
  timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
  remote_results_dir = f"/tmp/unicore_signalgroup_benchmark_{timestamp}"
  local_output_dir = pathlib.Path(args.output_dir) / timestamp
  local_output_dir.mkdir(parents=True, exist_ok=True)

  ssh_client = SSHClient(
      host=args.host,
      user=args.user,
      port=args.port,
      password=password,
      known_hosts_mode=args.known_hosts_mode,
  )

  remote_script = build_remote_benchmark_script(args, remote_results_dir)
  ssh_client.ssh("bash -s", stdin_text=remote_script)
  copy_remote_results(ssh_client, remote_results_dir, local_output_dir)

  summaries: list[dict[str, Any]] = []
  diagnostic_blocks_by_candidate: dict[str, list[dict[str, Any]]] = {}
  for candidate in (args.candidates or DEFAULT_CANDIDATES):
    tag = candidate.replace(" ", "_")
    summaries.append(summarize_candidate(candidate, local_output_dir))
    diagnostic_blocks_by_candidate[tag] = parse_diagnostic_blocks(
        local_output_dir / f"{tag}_diagnostics_raw.yaml")

  best = recommend_candidate(summaries)
  recommendation = format_recommendation(best)
  restore_signal_group = select_restore_signal_group(
      summaries,
      args.restore_mode,
      args.previous_signal_group,
  )
  restore_status: dict[str, Any] = {
      "restore_mode": args.restore_mode,
      "requested_restore_signal_group": restore_signal_group,
  }
  if restore_signal_group:
    restore_script = build_remote_restore_script(args, remote_results_dir, restore_signal_group)
    ssh_client.ssh("bash -s", stdin_text=restore_script)
    copy_remote_results(ssh_client, remote_results_dir, local_output_dir)
    restore_tag = f"restore_{restore_signal_group.replace(' ', '_')}"
    restore_status["apply_exitcode"] = safe_int(
        read_text_if_exists(local_output_dir / f"{restore_tag}_apply.exitcode").strip() or None)
    restore_status["apply_ok"] = (
        read_text_if_exists(local_output_dir / f"{restore_tag}_apply.exitcode").strip() == "0")
    restore_status["gps_status"] = parse_gnss_status(
        local_output_dir / f"{restore_tag}_gps_status.yaml")

  write_json(local_output_dir / "summaries.json", summaries)
  write_json(local_output_dir / "diagnostic_blocks.json", diagnostic_blocks_by_candidate)
  write_json(
      local_output_dir / "benchmark_metadata.json",
      {
          "host": args.host,
          "user": args.user,
          "port": args.port,
          "baud": args.baud,
          "device": args.device,
          "model": args.model,
          "profile": args.profile,
          "candidates": args.candidates or DEFAULT_CANDIDATES,
          "remote_results_dir": remote_results_dir,
          "restore": restore_status,
      },
  )
  write_csv(local_output_dir / "summary.csv", summaries)
  (local_output_dir / "recommendation.txt").write_text(recommendation + "\n", encoding="utf-8")

  print(recommendation)
  print(f"Artifacts: {local_output_dir}")
  if restore_signal_group:
    print(
        f"Restored runtime-only SIGNALGROUP {restore_signal_group}: "
        f"{'ok' if restore_status.get('apply_ok') else 'failed'}"
    )
  else:
    print("Restore skipped by request.")
  return 0


if __name__ == "__main__":
  sys.exit(main())
