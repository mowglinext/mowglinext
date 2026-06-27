#!/usr/bin/env python3
"""Cross-track error: robot fused trajectory vs the planned /coverage/full_plan.

Grabs the latched full coverage plan once, then (optionally loops) reading the
latest MOWING samples from the session JSONL and reports how far the robot is
from the path it should be following.
"""
import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

SESSION = sys.argv[1] if len(sys.argv) > 1 else \
    "/ros2_ws/maps/2026-06-04-coverage-continuous-rotshim.jsonl"
WINDOW = int(sys.argv[2]) if len(sys.argv) > 2 else 200       # samples (~20 s)
LOOP_S = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0     # 0 = one-shot


def get_plan():
    rclpy.init()
    node = rclpy.create_node("xtrack_probe")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)
    got = {}
    node.create_subscription(Path, "/coverage/full_plan",
                             lambda m: got.setdefault(
                                 "p", [(p.pose.position.x, p.pose.position.y)
                                       for p in m.poses]), qos)
    t0 = time.time()
    while "p" not in got and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    return got.get("p")


def read_traj(n):
    rows = []
    try:
        with open(SESSION) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    r = json.loads(line)
                except Exception:
                    continue
                if r.get("type") != "sample":
                    continue
                fu = r.get("fusion") or {}
                bt = r.get("bt") or {}
                x, y = fu.get("x"), fu.get("y")
                if x is not None and y is not None and bt.get("state_name") == "MOWING":
                    rows.append((x, y))
    except FileNotFoundError:
        return []
    return rows[-n:]


def xtrack(pt, plan):
    px, py = pt
    best = 1e9
    for i in range(len(plan) - 1):
        ax, ay = plan[i]
        bx, by = plan[i + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 < 1e-12:
            d = math.hypot(px - ax, py - ay)
        else:
            t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
            d = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
        if d < best:
            best = d
    return best


def report(plan):
    traj = read_traj(WINDOW)
    if not traj:
        print("no MOWING trajectory yet")
        return
    errs = sorted(xtrack(p, plan) for p in traj)
    n = len(errs)
    mean = sum(errs) / n
    p50 = errs[n // 2]
    p95 = errs[min(n - 1, int(n * 0.95))]
    cur = xtrack(traj[-1], plan)
    print("xtrack(m) cur=%.3f mean=%.3f p50=%.3f p95=%.3f max=%.3f  (plan=%d pts, win=%d)"
          % (cur, mean, p50, p95, errs[-1], len(plan), n))


plan = get_plan()
if not plan:
    print("no /coverage/full_plan latched")
    sys.exit(1)
if LOOP_S <= 0:
    report(plan)
else:
    while True:
        report(plan)
        time.sleep(LOOP_S)
