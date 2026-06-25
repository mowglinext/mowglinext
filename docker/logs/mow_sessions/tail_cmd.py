import json, sys
p = sys.argv[1] if len(sys.argv) > 1 else "/ros2_ws/maps/2026-06-04-coverage-mppi-rotationshim-tuning.jsonl"
n = int(sys.argv[2]) if len(sys.argv) > 2 else 70


def g(d, *ks, default=None):
    for k in ks:
        d = d.get(k) if isinstance(d, dict) else None
        if d is None:
            return default
    return d


rows = []
with open(p) as f:
    for line in f:
        try:
            r = json.loads(line)
        except Exception:
            continue
        if r.get("type") == "sample":
            rows.append(r)
last = rows[-n:]
print("showing last %d samples (~%.0fs); total=%d" % (len(last), len(last) * 0.1, len(rows)))
print("%6s %12s %5s %7s %7s %7s %7s %7s" % ("t_rel", "state", "strip", "cmd_vx", "cmd_wz", "act_vx", "act_wz", "yaw"))
t0 = last[0]["t"]
for r in last:
    print("%6.1f %12s %5s %7.3f %7.3f %7.3f %7.3f %7.1f" % (
        r["t"] - t0,
        str(g(r, "bt", "state_name")),
        str(g(r, "bt", "strip", default=-1)),
        g(r, "cmd_vel", "nav", "vx", default=0.0),
        g(r, "cmd_vel", "nav", "wz", default=0.0),
        g(r, "fusion", "vx", default=0.0),
        g(r, "fusion", "wz", default=0.0),
        g(r, "fusion", "yaw_deg", default=0.0),
    ))
