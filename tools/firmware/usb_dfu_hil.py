#!/usr/bin/env python3
"""Repeatable bench runner for the STM32F401 USB-DFU HIL procedure.

The tool only drives the same explicit GUI API used by an operator. It never
controls mower motion, blade hardware, power, ST-Link, or USB fault injection.
Starting a destructive update requires an exact bench-safety acknowledgement.
Every response is appended to JSONL so timing and terminal evidence can be
attached to the Draft PR without turning a tool exit into a success claim.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
import uuid
from datetime import datetime, timezone
from pathlib import Path


TERMINAL_STATES = {
    "succeeded",
    "rejected",
    "failed_recoverable",
    "failed_requires_stlink",
    "cancelled_before_entry",
}
SAFETY_ACK = "blade-removed-wheels-constrained-stlink-ready"


def request_json(base_url: str, path: str, method: str = "GET", body=None):
    data = None if body is None else json.dumps(body).encode("utf-8")
    request = urllib.request.Request(
        f"{base_url.rstrip('/')}{path}",
        data=data,
        method=method,
        headers={"Content-Type": "application/json", "Accept": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=15) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as error:
        try:
            payload = json.load(error)
        except Exception:
            payload = {"error": error.reason}
        return error.code, payload


def record(log_path: Path, event: str, payload) -> None:
    entry = {
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "event": event,
        "payload": payload,
    }
    log_path.parent.mkdir(parents=True, exist_ok=True)
    with log_path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(entry, sort_keys=True) + "\n")
    print(json.dumps(entry, sort_keys=True))


def wait_for_terminal(base_url: str, operation_id: str, log_path: Path, timeout: float):
    deadline = time.monotonic() + timeout
    last_state = None
    while time.monotonic() < deadline:
        status, snapshot = request_json(
            base_url, f"/api/setup/firmware-update/{operation_id}"
        )
        if status != 200:
            record(log_path, "status_error", {"http_status": status, "body": snapshot})
            time.sleep(1)
            continue
        if snapshot.get("state") != last_state:
            record(log_path, "state", snapshot)
            last_state = snapshot.get("state")
        if last_state in TERMINAL_STATES:
            return snapshot
        time.sleep(0.75)
    raise TimeoutError(f"operation {operation_id} did not reach a terminal state")


def start_cycle(args, cycle: int):
    key = f"hil-{args.run_id}-cycle-{cycle}-{uuid.uuid4()}"
    payload = {
        "idempotencyKey": key,
        "board": "BOARD_YARDFORCE500B",
        "environment": "Yardforce500B",
        "panel": args.panel,
        "recovery": False,
        "recoveryConfirmed": False,
    }
    status, response = request_json(
        args.base_url, "/api/setup/firmware-update", "POST", payload
    )
    record(args.log, "start", {"cycle": cycle, "http_status": status, "body": response})
    if status != 202 or "operation" not in response:
        raise RuntimeError(f"cycle {cycle}: update start was rejected")
    operation = response["operation"]
    terminal = wait_for_terminal(
        args.base_url, operation["id"], args.log, args.timeout
    )
    record(args.log, "terminal", {"cycle": cycle, "operation": terminal})
    if terminal.get("state") != "succeeded":
        raise RuntimeError(
            f"cycle {cycle}: terminal state is {terminal.get('state')}; stop and diagnose"
        )


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base-url", default="http://localhost:8080")
    parser.add_argument("--log", type=Path, required=True, help="append-only JSONL evidence log")
    subparsers = parser.add_subparsers(dest="command", required=True)

    status = subparsers.add_parser("status", help="read one existing operation; no writes")
    status.add_argument("operation_id")

    run = subparsers.add_parser("run", help="run sequential normal USB update cycles")
    run.add_argument("--cycles", type=int, required=True)
    run.add_argument("--panel", default="PANEL_TYPE_YARDFORCE_500B_CLASSIC")
    run.add_argument("--timeout", type=float, default=180.0)
    run.add_argument("--run-id", default=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    run.add_argument(
        "--i-confirm-bench-safe",
        metavar="ACK",
        help=f"must equal: {SAFETY_ACK}",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.command == "status":
        status, body = request_json(
            args.base_url, f"/api/setup/firmware-update/{args.operation_id}"
        )
        record(args.log, "status", {"http_status": status, "body": body})
        return 0 if status == 200 else 1

    if args.i_confirm_bench_safe != SAFETY_ACK:
        print(
            "Refusing destructive HIL run. Remove the blade, constrain the wheels, "
            "attach ST-Link for recovery, then pass the exact acknowledgement shown in --help.",
            file=sys.stderr,
        )
        return 2
    if not 1 <= args.cycles <= 100:
        print("--cycles must be between 1 and 100", file=sys.stderr)
        return 2

    record(
        args.log,
        "run_begin",
        {"run_id": args.run_id, "cycles": args.cycles, "base_url": args.base_url},
    )
    try:
        for cycle in range(1, args.cycles + 1):
            start_cycle(args, cycle)
    except Exception as error:
        record(args.log, "run_failed", {"error": str(error)})
        return 1
    record(args.log, "run_complete", {"cycles": args.cycles})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
