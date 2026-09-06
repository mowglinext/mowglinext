#!/usr/bin/env python3
"""Publish the bag's first fused pose as a latched /fusion_graph_node/set_pose so the replayed node bootstraps
where the robot actually was (the bag starts mid-drive; without this the node seeds at the dock with a wrong yaw)."""
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
BAG, HOLD = sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
r = SequentialReader(); r.open(StorageOptions(uri=BAG, storage_id="mcap"), ConverterOptions("", ""))
first = None
while r.has_next():
    topic, data, t = r.read_next()
    if topic == "/odometry/filtered_map": first = deserialize_message(data, Odometry); break
assert first is not None
rclpy.init(); n = Node("seed_pose", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
pub = n.create_publisher(PoseWithCovarianceStamped, "/fusion_graph_node/set_pose",
                         QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
m = PoseWithCovarianceStamped(); m.header.frame_id = "map"; m.pose.pose = first.pose.pose
m.pose.covariance[0] = 0.01; m.pose.covariance[7] = 0.01; m.pose.covariance[35] = 0.01
p = m.pose.pose.position; print(f"seed pose from bag: ({p.x:.2f}, {p.y:.2f}) q=({m.pose.pose.orientation.z:.3f},{m.pose.pose.orientation.w:.3f})", flush=True)
t0 = time.time()
while time.time() - t0 < HOLD:
    pub.publish(m); rclpy.spin_once(n, timeout_sec=1.0)
