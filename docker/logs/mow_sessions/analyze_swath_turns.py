#!/usr/bin/env python3
"""Analyze coverage swath-transition turning behavior from a mow_session JSONL.

For each swath transition (bt.strip increments while MOWING) we report:
  - peak |cmd_vel.nav.wz| during the turn   (did the controller command a hard turn?)
  - mean |fusion.vx| while |wz| is high       (low => clean in-place pivot; high => arcing/corner-cut)
  - turn duration (s) until heading settles    (|fusion.wz| < 0.15 and |cmd wz| < 0.2)
  - heading swept (deg)

Usage: analyze_swath_turns.py <session.jsonl>
"""
import json
import math
import sys


def load(path):
    samples = []
    with open(path) as f:
        for line in f:
            try:
                r = json.loads(line)
            except json.JSONDecodeError:
                continue
            if r.get("type") == "sample":
                samples.append(r)
    return samples


def g(d, *keys, default=None):
    for k in keys:
        if d is None:
            return default
        d = d.get(k) if isinstance(d, dict) else None
    return d if d is not None else default


def main():
    if len(sys.argv) < 2:
        print("usage: analyze_swath_turns.py <session.jsonl>")
        return
    s = load(sys.argv[1])
    mow = [r for r in s if g(r, "bt", "state_name") == "MOWING"]
    print(f"total samples={len(s)}  MOWING samples={len(mow)}")
    if not mow:
        print("no MOWING samples yet")
        return

    # Detect swath transitions: bt.strip changes to a new non-negative value.
    transitions = []
    prev = None
    for i, r in enumerate(mow):
        strip = g(r, "bt", "strip", default=-1)
        if strip is not None and strip >= 0 and strip != prev:
            transitions.append((i, strip, r["t"]))
            prev = strip

    print(f"swath transitions detected={len(transitions)}: "
          f"{[t[1] for t in transitions]}")

    for (idx, strip, t0) in transitions:
        win = [r for r in mow if t0 - 0.2 <= r["t"] <= t0 + 9.0]
        if len(win) < 3:
            continue
        peak_cmd_wz = max((abs(g(r, "cmd_vel", "nav", "wz", default=0.0)) for r in win), default=0.0)
        peak_act_wz = max((abs(g(r, "fusion", "wz", default=0.0)) for r in win), default=0.0)
        # vx while turning hard (|cmd wz|>0.3)
        turning = [r for r in win if abs(g(r, "cmd_vel", "nav", "wz", default=0.0)) > 0.3]
        mean_vx_turn = (sum(abs(g(r, "fusion", "vx", default=0.0)) for r in turning) / len(turning)
                        if turning else 0.0)
        # turn duration: from t0 until heading settles
        settle_t = None
        yaws = []
        for r in win:
            yaw = g(r, "fusion", "yaw_deg")
            if yaw is not None:
                yaws.append(yaw)
            cwz = abs(g(r, "cmd_vel", "nav", "wz", default=0.0))
            awz = abs(g(r, "fusion", "wz", default=0.0))
            if r["t"] > t0 + 0.5 and cwz < 0.2 and awz < 0.15 and settle_t is None:
                settle_t = r["t"] - t0
        swept = (max(yaws) - min(yaws)) if len(yaws) >= 2 else 0.0
        print(f"  swath {strip:>2}: peak_cmd_wz={peak_cmd_wz:4.2f} "
              f"peak_act_wz={peak_act_wz:4.2f} "
              f"vx_during_turn={mean_vx_turn:5.3f} "
              f"settle={settle_t if settle_t is not None else '>8'}s "
              f"yaw_swept={swept:5.1f}deg")


if __name__ == "__main__":
    main()
