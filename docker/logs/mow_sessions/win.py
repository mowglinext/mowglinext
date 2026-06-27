import json, sys
p = sys.argv[1]
lo = float(sys.argv[2]); hi = float(sys.argv[3])


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
t0 = rows[0]["t"]
seg = [r for r in rows if lo <= (r["t"]-t0) <= hi]
print("window %.0f-%.0fs : %d samples" % (lo, hi, len(seg)))
print("%6s %14s %5s %7s %7s | %8s %8s | %7s %6s" % (
    "t", "state", "strip", "cmdvx", "cmdwz", "whl_vx", "whl_wz", "yaw", "dgoal"))
for r in seg:
    print("%6.1f %14s %5s %7.3f %7.3f | %8.3f %8.3f | %7.1f %6s" % (
        r["t"]-t0,
        str(g(r, "bt", "state_name")),
        str(g(r, "bt", "strip", default=-1)),
        g(r, "cmd_vel", "nav", "vx", default=0.0),
        g(r, "cmd_vel", "nav", "wz", default=0.0),
        g(r, "wheel", "vx", default=0.0),
        g(r, "wheel", "wz", default=0.0),
        g(r, "fusion", "yaw_deg", default=0.0),
        ("%.2f" % g(r, "plan", "distance_to_goal_m")) if g(r, "plan", "distance_to_goal_m") is not None else "-",
    ))
