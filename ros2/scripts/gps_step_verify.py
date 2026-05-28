#!/usr/bin/env python3
"""Verify whether /gps/fix position steps are caused by stale-status pairing
in ublox_nav_sat_fix_hp_node, or by genuine receiver behaviour.

Subscribes to:
  /ubx_nav_hp_pos_llh   -> lat/lon (1e-7 + 1e-9 high-precision), h_acc, iTOW
  /ubx_nav_status       -> carr_soln.status, gps_fix.fix_type, diff_soln, iTOW
  /ubx_nav_cov          -> pos_cov_ee/nn, iTOW
  /gps/fix              -> the published NavSatFix (what consumers actually see)

Stores per-iTOW records. Detects horizontal position steps > step_threshold_m
(default 0.05 m / 5 cm) between consecutive iTOWs and prints the carr_soln /
h_acc at that epoch + the surrounding context.

Run inside the mowgli-gps (or ros2) container.

Usage:
  ros2 run ... OR  python3 gps_step_verify.py --duration 120 --step 0.05
"""
import argparse
import json
import math
import sys
import time
from collections import defaultdict, deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from ublox_ubx_msgs.msg import UBXNavHPPosLLH, UBXNavStatus, UBXNavCov


def llh_to_enu_meters(lat_deg, lon_deg, lat0_deg, lon0_deg):
    """Tiny local ENU approximation around (lat0, lon0). Accurate enough for
    step-detection (<= 1 km baseline)."""
    R = 6378137.0
    lat0 = math.radians(lat0_deg)
    dlat = math.radians(lat_deg - lat0_deg)
    dlon = math.radians(lon_deg - lon0_deg)
    east = R * math.cos(lat0) * dlon
    north = R * dlat
    return east, north


class StepVerifier(Node):
    def __init__(self, duration, step_threshold):
        super().__init__("gps_step_verify")
        self.duration = duration
        self.step_threshold = step_threshold

        # Per-iTOW state: {itow: {"hp": dict, "status": dict, "cov": dict}}
        self.epochs = {}
        # Ordered list of iTOWs we have seen with HPPOSLLH (used to detect steps)
        self.itow_order = deque()
        self.last_pos = None        # (itow, east_m, north_m, lat, lon)
        self.lat0 = None
        self.lon0 = None
        self.steps = []
        self.fix_msgs = 0
        self.start_t = time.monotonic()

        self.create_subscription(UBXNavHPPosLLH, "/ubx_nav_hp_pos_llh", self.hp_cb, 50)
        self.create_subscription(UBXNavStatus,   "/ubx_nav_status",     self.status_cb, 50)
        self.create_subscription(UBXNavCov,      "/ubx_nav_cov",        self.cov_cb, 50)
        self.create_subscription(NavSatFix,      "/gps/fix",            self.fix_cb, 50)

        self.timer = self.create_timer(0.5, self.tick)

    def _slot(self, itow):
        if itow not in self.epochs:
            self.epochs[itow] = {"hp": None, "status": None, "cov": None}
            self.itow_order.append(itow)
            # Cap memory: keep the last 200 epochs (~30 s @ 7 Hz)
            while len(self.itow_order) > 200:
                old = self.itow_order.popleft()
                self.epochs.pop(old, None)
        return self.epochs[itow]

    def hp_cb(self, m):
        lat = m.lat * 1e-7 + m.lat_hp * 1e-9
        lon = m.lon * 1e-7 + m.lon_hp * 1e-9
        height = m.height * 1e-3 + m.height_hp * 1e-4
        h_acc_mm = m.h_acc * 0.1   # h_acc unit is 0.1 mm
        slot = self._slot(m.itow)
        slot["hp"] = {
            "lat": lat, "lon": lon, "height": height,
            "h_acc_mm": h_acc_mm,
            "invalid_lat": m.invalid_lat, "invalid_lon": m.invalid_lon,
            "invalid_lat_hp": m.invalid_lat_hp, "invalid_lon_hp": m.invalid_lon_hp,
            "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
        }
        if self.lat0 is None:
            self.lat0 = lat
            self.lon0 = lon

        # Step detection — compare to last good HP fix
        e, n = llh_to_enu_meters(lat, lon, self.lat0, self.lon0)
        if self.last_pos is not None:
            de = e - self.last_pos[1]
            dn = n - self.last_pos[2]
            d = math.hypot(de, dn)
            if d >= self.step_threshold:
                # Capture full context (and the status/cov for this AND prior iTOW)
                prev_itow = self.last_pos[0]
                self.steps.append({
                    "from_itow": prev_itow,
                    "to_itow": m.itow,
                    "delta_e_m": de,
                    "delta_n_m": dn,
                    "delta_horiz_m": d,
                    "prev_epoch": dict(self.epochs.get(prev_itow, {})),
                    "this_epoch": dict(self.epochs.get(m.itow, {})),
                    "h_acc_mm": h_acc_mm,
                })
        self.last_pos = (m.itow, e, n, lat, lon)

    def status_cb(self, m):
        slot = self._slot(m.itow)
        slot["status"] = {
            "carr_soln": m.carr_soln.status,    # 0=none, 1=Float, 2=Fixed
            "fix_type": m.gps_fix.fix_type,
            "diff_soln": m.diff_soln,
            "gps_fix_ok": m.gps_fix_ok,
            "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
        }

    def cov_cb(self, m):
        slot = self._slot(m.itow)
        slot["cov"] = {
            "pos_cov_ee": m.pos_cov_ee,
            "pos_cov_nn": m.pos_cov_nn,
            "stamp": m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
        }

    def fix_cb(self, m):
        self.fix_msgs += 1
        # We compare the /gps/fix status (cached in HP node) against the truth
        # in /ubx_nav_status, but we can't do per-iTOW correlation here because
        # NavSatFix doesn't carry iTOW. We just count messages.

    def tick(self):
        dt = time.monotonic() - self.start_t
        if dt >= self.duration:
            self.print_summary()
            rclpy.shutdown()
            sys.exit(0)

    def print_summary(self):
        n_epochs = len(self.epochs)
        n_paired = sum(
            1 for v in self.epochs.values()
            if v["hp"] and v["status"] and v["cov"]
        )
        print("=" * 78)
        print(f"Duration:       {self.duration:.0f} s")
        print(f"HP epochs:      {n_epochs}")
        print(f"Fully paired:   {n_paired}  ({100*n_paired/max(n_epochs,1):.1f}%)")
        print(f"/gps/fix msgs:  {self.fix_msgs}")
        print(f"Steps >= {self.step_threshold*100:.0f} cm: {len(self.steps)}")
        print("=" * 78)
        for i, s in enumerate(self.steps):
            print()
            print(f"--- Step {i+1}: {s['delta_horiz_m']*100:.1f} cm "
                  f"(dE={s['delta_e_m']*100:+.1f} cm, dN={s['delta_n_m']*100:+.1f} cm) ---")
            print(f"  from iTOW {s['from_itow']}  ->  to iTOW {s['to_itow']}")
            print(f"  h_acc at step: {s['h_acc_mm']:.1f} mm")
            for label, ep in (("prev", s["prev_epoch"]), ("this", s["this_epoch"])):
                st = ep.get("status") or {}
                hp = ep.get("hp") or {}
                cov = ep.get("cov") or {}
                cs = st.get("carr_soln", "?")
                cs_label = {0: "None", 1: "Float", 2: "Fixed"}.get(cs, str(cs))
                ftype = st.get("fix_type", "?")
                diff = st.get("diff_soln", "?")
                hacc = hp.get("h_acc_mm", "?")
                ee = cov.get("pos_cov_ee", "?")
                nn = cov.get("pos_cov_nn", "?")
                inv = (
                    f"inv_lat={hp.get('invalid_lat')}, inv_lon={hp.get('invalid_lon')}, "
                    f"inv_lat_hp={hp.get('invalid_lat_hp')}, inv_lon_hp={hp.get('invalid_lon_hp')}"
                )
                print(f"  [{label}] carr_soln={cs_label}  fix_type={ftype}  "
                      f"diff_soln={diff}  h_acc={hacc} mm  "
                      f"cov_ee={ee}  cov_nn={nn}")
                print(f"         {inv}")

        # Distribution of carr_soln across all paired epochs
        soln_counts = defaultdict(int)
        for ep in self.epochs.values():
            if ep["status"]:
                soln_counts[ep["status"]["carr_soln"]] += 1
        print()
        print("carr_soln distribution across all epochs:")
        for v in (0, 1, 2):
            label = {0: "None", 1: "Float", 2: "Fixed"}[v]
            print(f"  {label}:  {soln_counts.get(v, 0)} "
                  f"({100*soln_counts.get(v,0)/max(n_epochs,1):.1f}%)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=120.0,
                    help="Capture duration in seconds")
    ap.add_argument("--step", type=float, default=0.05,
                    help="Horizontal step threshold in metres (default 0.05 = 5 cm)")
    args = ap.parse_args()

    rclpy.init()
    node = StepVerifier(args.duration, args.step)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.print_summary()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
