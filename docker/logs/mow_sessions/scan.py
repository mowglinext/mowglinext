import json, sys
from collections import Counter
p = sys.argv[1]


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
print("total samples=%d  dur=%.0fs" % (len(rows), (rows[-1]["t"]-rows[0]["t"]) if rows else 0))
# max strip reached + states seen
strips = [g(r, "bt", "strip", default=-1) for r in rows]
print("max strip reached:", max(strips) if strips else None)
print("states seen:", dict(Counter(str(g(r, "bt", "state_name")) for r in rows)))
# sliding 3s (30-sample) window oscillation score = sign flips of cmd_wz
best = (0, 0, "")
win = 30
for i in range(0, len(rows)-win, 5):
    seg = rows[i:i+win]
    flips = 0
    prev = 0
    for r in seg:
        wz = g(r, "cmd_vel", "nav", "wz", default=0.0)
        s = (wz > 0.08) - (wz < -0.08)
        if s != 0:
            if prev != 0 and s != prev:
                flips += 1
            prev = s
    if flips > best[0]:
        st = str(g(seg[len(seg)//2], "bt", "state_name"))
        best = (flips, rows[i]["t"]-rows[0]["t"], st)
print("most oscillatory 3s window: sign_flips=%d at t=%.0fs state=%s" % best)
# report all windows with flips>=4
print("--- windows with >=4 cmd_wz sign-flips (true oscillation) ---")
cnt = 0
for i in range(0, len(rows)-win, win):
    seg = rows[i:i+win]
    flips = 0
    prev = 0
    mx = 0
    for r in seg:
        wz = g(r, "cmd_vel", "nav", "wz", default=0.0)
        mx = max(mx, abs(wz))
        s = (wz > 0.08) - (wz < -0.08)
        if s != 0:
            if prev != 0 and s != prev:
                flips += 1
            prev = s
    if flips >= 4:
        st = Counter(str(g(r, "bt", "state_name")) for r in seg).most_common(1)[0][0]
        print("  t=%5.0fs flips=%2d max|wz|=%.2f state=%s" % (rows[i]["t"]-rows[0]["t"], flips, mx, st))
        cnt += 1
if cnt == 0:
    print("  (none — no sustained command-level oscillation anywhere in session)")
