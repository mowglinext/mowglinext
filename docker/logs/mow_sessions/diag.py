import json, sys
p = sys.argv[1] if len(sys.argv) > 1 else "/ros2_ws/maps/2026-06-04-coverage-mppi-rotationshim-tuning.jsonl"
n = int(sys.argv[2]) if len(sys.argv) > 2 else 150


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
# State histogram over window
from collections import Counter
hist = Counter(str(g(r, "bt", "state_name")) for r in last)
print("window=%d samples (~%.0fs) total=%d" % (len(last), len(last)*0.1, len(rows)))
print("state histogram:", dict(hist))
# cmd_wz sign-flip count (true oscillation indicator) over moving samples
flips = 0
prev_sign = 0
amps = []
for r in last:
    wz = g(r, "cmd_vel", "nav", "wz", default=0.0)
    amps.append(abs(wz))
    s = (wz > 0.05) - (wz < -0.05)
    if s != 0:
        if prev_sign != 0 and s != prev_sign:
            flips += 1
        prev_sign = s
print("cmd_wz sign-flips in window: %d   max|cmd_wz|=%.2f" % (flips, max(amps) if amps else 0))
# last 30 samples detail
print("--- last 30 ---")
print("%6s %16s %5s %7s %7s %7s %7s %7s %6s" % ("t_rel","state","strip","cmd_vx","cmd_wz","act_wz","yaw","gps%","emg"))
t0 = last[-30]["t"]
for r in last[-30:]:
    print("%6.1f %16s %5s %7.3f %7.3f %7.3f %7.1f %6s %6s" % (
        r["t"]-t0,
        str(g(r,"bt","state_name")),
        str(g(r,"bt","strip",default=-1)),
        g(r,"cmd_vel","nav","vx",default=0.0),
        g(r,"cmd_vel","nav","wz",default=0.0),
        g(r,"fusion","wz",default=0.0),
        g(r,"fusion","yaw_deg",default=0.0),
        str(g(r,"hardware","emergency_active")),
        "",
    ))
