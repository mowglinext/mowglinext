#!/usr/bin/env python3
"""Evaluate a replay log: divergence from dead reckoning during the GPS outage, error at RTK return, recovery time."""
import csv, math, sys
rows = list(csv.DictReader(open(sys.argv[1])))
f = lambda r, k: float(r[k]) if r[k] not in ("", "nan") else float("nan")
LEVER = 0.3
def base_from_gps(r):  # antenna sits LEVER m ahead of base along the fused yaw
    th = math.radians(f(r, "fyaw")); return (f(r, "gx") - LEVER * math.cos(th), f(r, "gy") - LEVER * math.sin(th))
# outage = rows where gps_age > 3 s
out = [i for i, r in enumerate(rows) if f(r, "gps_age") > 3.0]
if not out: print("no outage found"); sys.exit(0)
i0, i1 = out[0], out[-1]
pre = rows[i0 - 1]; mo = (f(pre, "mo_x"), f(pre, "mo_y"), math.radians(f(pre, "mo_yaw")))
def dr(r):
    c, s = math.cos(mo[2]), math.sin(mo[2]); ox, oy = f(r, "ob_x"), f(r, "ob_y")
    return (mo[0] + c * ox - s * oy, mo[1] + s * ox + c * oy)
print(f"outage: t={f(rows[i0],'t'):.1f}..{f(rows[i1],'t'):.1f} ({f(rows[i1],'t')-f(rows[i0],'t'):.0f} s), rows {i0}..{i1}")
div = []
for r in rows[i0:i1 + 1]:
    d = dr(r); div.append((math.hypot(f(r, "fx") - d[0], f(r, "fy") - d[1]), f(r, "t")))
mx = max(div); print(f"max |fused - DR| during outage: {mx[0]:.2f} m at t={mx[1]:.1f}; at end of outage: {div[-1][0]:.2f} m")
# first RTK Fixed after outage
after = [r for r in rows[i1 + 1:] if r["rtk_mode"] == "3"]
if after:
    r = after[0]; b = base_from_gps(r); d = dr(r)
    print(f"at RTK return t={f(r,'t'):.1f}: fused err={math.hypot(f(r,'fx')-b[0], f(r,'fy')-b[1]):.2f} m | DR-only err={math.hypot(d[0]-b[0], d[1]-b[1]):.2f} m | fused yaw {f(r,'fyaw'):.0f} vs DR yaw {f(pre,'mo_yaw')+f(r,'ob_yaw'):.0f}")
    # recovery: first time fused err < 0.15 m after return
    for rr in after:
        b = base_from_gps(rr)
        if math.hypot(f(rr, "fx") - b[0], f(rr, "fy") - b[1]) < 0.15: print(f"recovered (<0.15 m) at t={f(rr,'t'):.1f}, {f(rr,'t')-f(r,'t'):.0f} s after RTK return"); break
    else: print("never recovered below 0.15 m")
last = rows[-1]; print("anchor diag at end:", {k: last[k] for k in ("lidar_anchor_state", "lidar_anchor_seeds", "lidar_anchor_factors", "lidar_anchor_updates", "lidar_anchor_rej_score", "lidar_anchor_rej_spread", "lidar_anchor_rej_dr", "lidar_anchor_reseeds") if k in last})
# steady-state under RTK before outage
pre_rows = [r for r in rows[:i0] if r["rtk_mode"] == "3"]
if pre_rows:
    errs = sorted(math.hypot(f(r, "fx") - base_from_gps(r)[0], f(r, "fy") - base_from_gps(r)[1]) for r in pre_rows[-60:])
    print(f"pre-outage fused-vs-GPS (lever corrected) p50={errs[len(errs)//2]:.2f} max={errs[-1]:.2f} m over last {len(errs)} rows")
