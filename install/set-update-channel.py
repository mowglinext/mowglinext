#!/usr/bin/env python3
"""Switch the mowgli host updater to a release channel and apply its latest deployment.

Recovery / support tool for the case where the in-GUI updater is unreachable but the
robot must still be moved onto the `dev` (or back onto `main`) channel and updated to
the newest deployment. It talks to the same local updater daemon the GUI proxies to
(the unix socket at /var/lib/mowgli-updater/run/updater.sock), replaying the exact
policy -> check -> plan -> apply sequence.

There is NO docker-compose tag to edit anymore: the updater pins every image by digest
into an updater-owned docker-compose.yaml guarded by a sha256 baseline, so hand-editing
tags is rejected. "Move the tags to dev" == "switch the update channel to dev and apply
the latest dev deployment", which is what this does.

Usage (run on the robot host, as root):

    sudo python3 install/set-update-channel.py dev        # switch to dev + update
    sudo python3 install/set-update-channel.py main       # switch back to stable + update
    sudo python3 install/set-update-channel.py dev --yes  # no interactive confirmation

Stdlib only (no curl/jq needed). Exit code 0 on a successful/no-op update, non-zero otherwise.
"""

import argparse
import http.client
import json
import socket
import sys
import time

DEFAULT_SOCKET = "/var/lib/mowgli-updater/run/updater.sock"
TERMINAL_PHASES = {"succeeded", "failed", "rolled_back"}


class UnixHTTPConnection(http.client.HTTPConnection):
    """HTTPConnection over a unix-domain socket."""

    def __init__(self, path, timeout):
        super().__init__("localhost", timeout=timeout)
        self._path = path

    def connect(self):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(self.timeout)
        s.connect(self._path)
        self.sock = s


def call(sock_path, method, path, body=None, timeout=300):
    conn = UnixHTTPConnection(sock_path, timeout)
    headers = {"Content-Type": "application/json", "Accept": "application/json"}
    payload = json.dumps(body) if body is not None else ("{}" if method == "POST" else None)
    try:
        conn.request(method, path, body=payload, headers=headers)
        resp = conn.getresponse()
        raw = resp.read()
    except (ConnectionRefusedError, FileNotFoundError, PermissionError) as e:
        die(f"Cannot reach the updater socket at {sock_path}: {e}\n"
            f"       Is the mowgli-updater service running, and are you root (try: sudo)?")
    finally:
        conn.close()
    data = {}
    if raw:
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            data = {"raw": raw.decode("utf-8", "replace")}
    return resp.status, data


def die(msg):
    print(f"[x] {msg}", file=sys.stderr)
    sys.exit(1)


def check_ok(status, data, what):
    if status >= 400:
        die(f"{what} failed (HTTP {status}): {data.get('error', data)}")
    return data


def get_state(sock_path):
    status, data = call(sock_path, "GET", "/v1/state", timeout=30)
    check_ok(status, data, "Reading updater state")
    return data


def main():
    ap = argparse.ArgumentParser(description="Switch the mowgli updater channel and update.")
    ap.add_argument("channel", choices=["dev", "main"], help="release channel to switch to")
    ap.add_argument("--socket", default=DEFAULT_SOCKET, help=f"updater socket (default: {DEFAULT_SOCKET})")
    ap.add_argument("--yes", "-y", action="store_true", help="do not prompt for confirmation")
    ap.add_argument("--check-timeout", type=int, default=200, help="seconds to wait for the release check")
    ap.add_argument("--apply-timeout", type=int, default=1800, help="seconds to wait for the update job")
    args = ap.parse_args()

    track, branch = ("dev", "dev") if args.channel == "dev" else ("stable", "main")

    envelope = get_state(args.socket)
    state = envelope.get("state", {})
    trusted = envelope.get("trusted_repositories") or []
    policy = state.get("policy", {}) or {}
    src = policy.get("source", {}) or {}
    repo = src.get("repository") or (trusted[0] if trusted else None)
    if not repo:
        die("Could not determine the trusted update repository from updater state.")
    if trusted and repo not in trusted:
        die(f"Repository {repo} is not in the trusted list {trusted}.")

    job = state.get("job")
    if job and job.get("phase") not in TERMINAL_PHASES and job.get("phase"):
        die(f"An update/recovery job is already in progress (phase={job.get('phase')}). Try again later.")

    active = state.get("active") or {}
    print(f"Repository        : {repo}")
    print(f"Current channel   : track={src.get('track', '?')} branch={src.get('branch', '?')}")
    print(f"Active deployment : {active.get('id', '(none)')}")
    print(f"Target channel    : track={track} branch={branch}")

    interval = policy.get("interval_hours")
    if interval not in (0, 1, 4, 24):
        interval = 24

    # 1) Point the channel at the target track (clears the release cache server-side).
    print(f"\n[1/4] Switching channel to '{args.channel}' ...")
    status, data = call(args.socket, "POST", "/v1/policy",
                        {"source": {"repository": repo, "track": track, "branch": branch},
                         "interval_hours": interval, "pinned": False})
    check_ok(status, data, "Setting update policy")

    # 2) Force a release check and wait for the new channel's releases to land.
    print("[2/4] Checking for the latest deployment ...")
    status, data = call(args.socket, "POST", "/v1/check")
    if status >= 400:
        die(f"Triggering the release check failed (HTTP {status}): {data.get('error', data)}")

    deadline = time.time() + args.check_timeout
    target = None
    while time.time() < deadline:
        time.sleep(3)
        st = get_state(args.socket).get("state", {})
        if st.get("check_error"):
            die(f"Release check failed: {st['check_error']}")
        releases = st.get("releases") or []
        if releases and st.get("last_check"):
            target = releases[0]
            break
    if not target:
        die("Timed out waiting for the release list. Check network/GHCR access and try again.")

    print(f"      Latest {args.channel} deployment: {target.get('id')} "
          f"(published {target.get('published_at', '?')}, rev {str(target.get('revision', ''))[:12]})")

    if not args.yes:
        print("\n  This will recreate the mowgli containers (the ROS2 stack that drives the")
        print("  robot will restart). Make sure the mower is docked/idle and not mowing.")
        try:
            reply = input("  Proceed with the update? [y/N] ").strip().lower()
        except EOFError:
            die("No terminal for confirmation. Re-run with --yes to proceed non-interactively.")
        if reply not in ("y", "yes", "o", "oui"):
            die("Aborted by operator.")

    # 3) Build the plan for the newest deployment.
    print("\n[3/4] Building the update plan ...")
    status, data = call(args.socket, "POST", "/v1/plan", {"deployment": target["id"], "pinned": False})
    check_ok(status, data, "Planning the update")
    plan_id = data.get("id")
    if not plan_id:
        die(f"Planner returned no plan id: {data}")
    changes = (data.get("stack") or {}).get("changes") or []
    if changes:
        summary = ", ".join(f"{c['service']}:{c['action']}" for c in changes)
        print(f"      Plan {plan_id}: {summary}")
    else:
        print(f"      Plan {plan_id}")

    # 4) Apply and follow the job to completion.
    print("[4/4] Applying the update ...")
    status, data = call(args.socket, "POST", "/v1/apply", {"plan": plan_id, "custom_acknowledged": False})
    check_ok(status, data, "Applying the update")
    job_id = data.get("job")
    print(f"      Job {job_id} started; following progress ...")

    deadline = time.time() + args.apply_timeout
    last_phase = None
    while time.time() < deadline:
        time.sleep(4)
        st = get_state(args.socket).get("state", {})
        j = st.get("job") or {}
        phase = j.get("phase")
        if phase and phase != last_phase:
            print(f"      phase: {phase}")
            last_phase = phase
        if phase in TERMINAL_PHASES:
            if phase == "succeeded":
                new_active = (st.get("active") or {}).get("id", "?")
                print(f"\n[ok] Update succeeded. Active deployment is now {new_active} on '{args.channel}'.")
                return 0
            err = j.get("error", "(no error message)")
            die(f"Update ended in phase '{phase}': {err}\n"
                f"       Inspect with:  mowgli-updater status   (or the host updater logs)")
    die("Timed out following the update job. Inspect with: mowgli-updater status")


if __name__ == "__main__":
    sys.exit(main())
