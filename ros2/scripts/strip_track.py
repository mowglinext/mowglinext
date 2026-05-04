#!/usr/bin/env python3
"""Sample robot pose vs F2C coverage path and report cross-track error.

Dumps the coverage plan once, then every 1s computes:
  - nearest path point and its index (0..N-1)
  - cross-track error (perpendicular distance to nearest path segment)
  - along-track progress (idx / N as %)
  - heading error vs path tangent at nearest point
"""
import math
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped


class StripTracker(Node):
    def __init__(self):
        super().__init__("strip_tracker")
        # /coverage_server/coverage_plan is volatile reliable; use SystemDefaultsQoS-ish.
        plan_qos = QoSProfile(depth=1)
        plan_qos.reliability = ReliabilityPolicy.RELIABLE
        plan_qos.durability = DurabilityPolicy.VOLATILE
        self.path = None
        # /coverage_server/coverage_plan is volatile + one-shot at compute time.
        # Subscribe BEFORE START so we catch the message when ComputeCoveragePath
        # fires (~13s after START, post-undock).
        self.create_subscription(Path, "/coverage_server/coverage_plan",
                                 self._on_path, plan_qos)
        self.create_subscription(Odometry, "/odometry/filtered_map",
                                 self._on_odom, 10)
        self.last_pose = None
        self.last_yaw = None

    def _on_path(self, msg):
        if self.path is None or len(msg.poses) != len(self.path):
            self.get_logger().info(f"Path received: {len(msg.poses)} poses, "
                                    f"first=({msg.poses[0].pose.position.x:.2f},"
                                    f"{msg.poses[0].pose.position.y:.2f}) "
                                    f"last=({msg.poses[-1].pose.position.x:.2f},"
                                    f"{msg.poses[-1].pose.position.y:.2f})")
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _on_odom(self, msg):
        self.last_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.last_yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                                   1 - 2*(q.y*q.y + q.z*q.z))


def find_nearest_segment(path, p):
    """Return (idx, dist_perp, t) of the closest segment."""
    best = (None, float("inf"), 0.0)
    px, py = p
    for i in range(len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i+1]
        dx = bx - ax
        dy = by - ay
        L2 = dx*dx + dy*dy
        if L2 < 1e-9:
            continue
        t = ((px - ax) * dx + (py - ay) * dy) / L2
        t = max(0.0, min(1.0, t))
        cx = ax + t*dx
        cy = ay + t*dy
        d = math.hypot(px - cx, py - cy)
        if d < best[1]:
            best = (i, d, t)
    return best


def main():
    rclpy.init()
    node = StripTracker()
    # Wait indefinitely for the first coverage path. ComputeCoveragePath
    # fires ~13s after START — we need to be subscribed when it fires
    # (volatile QoS, no late-joiner replay).
    print("Waiting for coverage path on /coverage_server/coverage_plan...", flush=True)
    while node.path is None or node.last_pose is None:
        rclpy.spin_once(node, timeout_sec=1.0)

    print(f"Tracking {len(node.path)} path points\n")
    print(f"{'t':>4} {'x':>7} {'y':>7} {'yaw°':>5} | "
          f"{'nearest':>7} {'idx%':>4} | "
          f"{'cross-err':>9} {'head-err°':>9}")

    t0 = time.time()
    while True:
        t = time.time() - t0
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.last_pose is None or node.path is None:
            time.sleep(1.0)
            continue
        idx, dist, frac = find_nearest_segment(node.path, node.last_pose)
        if idx is None:
            time.sleep(1.0)
            continue
        # Path tangent at nearest segment
        ax, ay = node.path[idx]
        bx, by = node.path[idx+1]
        path_yaw = math.atan2(by - ay, bx - ax)
        head_err = node.last_yaw - path_yaw
        # wrap to [-pi, pi]
        while head_err > math.pi: head_err -= 2*math.pi
        while head_err < -math.pi: head_err += 2*math.pi
        prog = (idx + frac) / max(1, len(node.path) - 1) * 100.0
        x, y = node.last_pose
        print(f"{t:>4.0f} {x:>7.2f} {y:>7.2f} {math.degrees(node.last_yaw):>5.0f} | "
              f"{idx:>7d} {prog:>4.1f} | "
              f"{dist*1000:>8.0f}mm {math.degrees(head_err):>+8.0f}°", flush=True)
        time.sleep(1.0)


if __name__ == "__main__":
    main()
