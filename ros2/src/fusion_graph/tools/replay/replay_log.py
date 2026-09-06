#!/usr/bin/env python3
"""Replay harness logger (sim time). 2 Hz rows: fused pose, map->odom, odom->base, GPS projected, RTK status, anchor diag, anchor candidate."""
import csv, math, sys
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from mowgli_interfaces.msg import GnssStatus
from diagnostic_msgs.msg import DiagnosticArray
OUT = sys.argv[1]; LAT0, LON0 = 48.879649550, 2.172814460; R = 6371000.0
DIAG = ["lidar_anchor_state", "lidar_map_occupied_cells", "lidar_anchor_updates", "lidar_anchor_seeds", "lidar_anchor_factors", "lidar_anchor_hit_ratio",
        "lidar_anchor_rej_score", "lidar_anchor_rej_spread", "lidar_anchor_rej_dr", "lidar_anchor_reseeds", "cov_xx", "gps_rejects_wrongfix"]
def yaw(q): return math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
class L(Node):
    def __init__(self):
        super().__init__("replay_log", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.mo = self.ob = self.pose = None; self.gps = None; self.gps_t = None; self.st = {}; self.d = {}; self.cand = None
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        self.create_subscription(Odometry, "/odometry/filtered_map", self.on_pose, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, "/gps/fix", self.on_gps, qos_profile_sensor_data)
        self.create_subscription(GnssStatus, "/gps/status", self.on_st, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, "/fusion_graph/diagnostics", self.on_diag, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/fusion_graph/lidar_anchor_candidate", self.on_cand, 10)
        self.f = open(OUT, "w", newline=""); self.w = csv.writer(self.f)
        self.w.writerow(["t", "fx", "fy", "fyaw", "mo_x", "mo_y", "mo_yaw", "ob_x", "ob_y", "ob_yaw", "gx", "gy", "gps_age", "rtk_mode", "hacc", "cand_x", "cand_y", "cand_ok"] + DIAG)
        self.create_timer(0.5, self.tick)
    def now(self): return self.get_clock().now().nanoseconds / 1e9
    def on_tf(self, m):
        for tr in m.transforms:
            t = tr.transform
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom": self.mo = (t.translation.x, t.translation.y, yaw(t.rotation))
            elif tr.header.frame_id == "odom" and tr.child_frame_id == "base_footprint": self.ob = (t.translation.x, t.translation.y, yaw(t.rotation))
    def on_pose(self, m): p = m.pose.pose; self.pose = (p.position.x, p.position.y, yaw(p.orientation))
    def on_gps(self, m):
        self.gps = ((m.longitude - LON0) * math.pi / 180 * R * math.cos(LAT0 * math.pi / 180), (m.latitude - LAT0) * math.pi / 180 * R); self.gps_t = self.now()
    def on_st(self, m): self.st = {"rtk_mode": m.rtk_mode, "hacc": m.horizontal_accuracy_m}
    def on_diag(self, m):
        for s in m.status:
            kv = {k.key: k.value for k in s.values}
            if "lidar_anchor_state" in kv: self.d = {k: kv.get(k, "") for k in DIAG}
    def on_cand(self, m):
        # covariance[35] carries the verdict flag (1 accepted / 0 rejected) — see node
        self.cand = (m.pose.pose.position.x, m.pose.pose.position.y, m.pose.covariance[35])
    def tick(self):
        if self.pose is None or self.mo is None or self.ob is None: return
        g = self.gps or (float("nan"), float("nan")); age = (self.now() - self.gps_t) if self.gps_t else float("nan")
        c = self.cand or (float("nan"), float("nan"), float("nan"))
        self.w.writerow([f"{self.now():.2f}", *[f"{v:.3f}" for v in self.pose], *[f"{v:.3f}" for v in self.mo], *[f"{v:.3f}" for v in self.ob],
                         f"{g[0]:.3f}", f"{g[1]:.3f}", f"{age:.1f}", self.st.get("rtk_mode", ""), self.st.get("hacc", ""), f"{c[0]:.3f}", f"{c[1]:.3f}", c[2]]
                        + [self.d.get(k, "") for k in DIAG]); self.f.flush()
rclpy.init(); n = L()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
n.f.close()
