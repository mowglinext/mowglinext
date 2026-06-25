#!/usr/bin/env python3
"""Send DWB a square path starting at the robot's current pose, to evaluate
path-following (cross-track) away from the coverage/boundary logic.

Publishes the square on /coverage/full_plan (Foxglove viz + xtrack.py) and on
the goal-checker topic, then dispatches a /follow_path goal to FollowCoveragePath
(currently DWB). Blades are untouched (this only moves the base)."""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

SIDE = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
STEP = 0.05

rclpy.init()
node = rclpy.create_node("square_test")

pose = {}
def odom_cb(m):
    p = m.pose.pose
    siny = 2 * (p.orientation.w * p.orientation.z)
    cosy = 1 - 2 * (p.orientation.z ** 2)
    pose.update(x=p.position.x, y=p.position.y, yaw=math.atan2(siny, cosy))
node.create_subscription(Odometry, "/odometry/filtered_map", odom_cb, 10)
t0 = time.time()
while "x" not in pose and time.time() - t0 < 5:
    rclpy.spin_once(node, timeout_sec=0.2)
if "x" not in pose:
    print("ERROR: no /odometry/filtered_map")
    sys.exit(1)
x0, y0, yaw0 = pose["x"], pose["y"], pose["yaw"]

def quat(yaw):
    ps = PoseStamped()
    ps.pose.orientation.z = math.sin(yaw / 2)
    ps.pose.orientation.w = math.cos(yaw / 2)
    return ps.pose.orientation

path = Path()
path.header.frame_id = "map"
path.header.stamp = node.get_clock().now().to_msg()
hx, hy = x0, y0
for k in range(4):                       # 4 sides, each +90 deg from the last
    ang = yaw0 + k * math.pi / 2
    nx, ny = hx + SIDE * math.cos(ang), hy + SIDE * math.sin(ang)
    n = int(SIDE / STEP)
    for i in range(n):
        t = i / n
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = hx + t * (nx - hx)
        ps.pose.position.y = hy + t * (ny - hy)
        ps.pose.orientation = quat(ang)
        path.poses.append(ps)
    hx, hy = nx, ny
# Do NOT end exactly on the start: a closed path makes start==goal, and
# PathProgressGoalChecker then sees the robot already on the LAST pose and fires
# "goal reached" at t=0 (zero motion). Continue EXTRA past the start along
# side-1 so the goal is a distinct pose; this also exercises the 4th corner.
EXTRA = 0.6
ang = yaw0
nx, ny = hx + EXTRA * math.cos(ang), hy + EXTRA * math.sin(ang)
n = int(EXTRA / STEP)
for i in range(n + 1):
    t = i / n
    ps = PoseStamped()
    ps.header = path.header
    ps.pose.position.x = hx + t * (nx - hx)
    ps.pose.position.y = hy + t * (ny - hy)
    ps.pose.orientation = quat(ang)
    path.poses.append(ps)

latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST)
pv = node.create_publisher(Path, "/coverage/full_plan", latched)
pg = node.create_publisher(Path, "/controller_server/FollowCoveragePath/global_plan", latched)
pv.publish(path)
pg.publish(path)
for _ in range(8):
    rclpy.spin_once(node, timeout_sec=0.1)

ac = ActionClient(node, FollowPath, "/follow_path")
if not ac.wait_for_server(timeout_sec=5):
    print("ERROR: /follow_path action server not available")
    sys.exit(1)
goal = FollowPath.Goal()
goal.path = path
goal.controller_id = "FollowCoveragePath"
goal.goal_checker_id = "coverage_goal_checker"
print("SQUARE: %d poses, side=%.1fm, start=(%.2f,%.2f) yaw0=%.0fdeg"
      % (len(path.poses), SIDE, x0, y0, math.degrees(yaw0)))
fut = ac.send_goal_async(goal)
rclpy.spin_until_future_complete(node, fut, timeout_sec=5)
gh = fut.result()
print("goal accepted:", bool(gh and gh.accepted))
node.destroy_node()
rclpy.shutdown()
