#!/usr/bin/env python3
"""
fleet_peer_obstacles — turn the other fleet members' poses into costmap points.

Robots never see each other over DDS (Cyclone is pinned to loopback, issue
#418). The GUI fleet coordinator on THIS robot learns the peers' positions over
the GUI API and publishes them into the local ROS graph through foxglove's
clientPublish as a geometry_msgs/PoseArray on /fleet/peers (map frame). This
node turns every fresh peer pose into a ring of points and publishes them as
a sensor_msgs/PointCloud2 on /fleet/peer_obstacles at a steady rate, so the
LOCAL costmap's obstacle layer marks the other mowers as lethal and Nav2 keeps
its distance on transit. See docs/MULTI_ROBOT.md.

Two properties are load-bearing:
  * The cloud is published CONTINUOUSLY, empty when there are no peers, so a
    costmap observation source with a persistence window never goes stale and
    nothing else in Nav2 ever waits on it.
  * A peer pose older than peer_timeout_s is dropped: a robot that went
    offline must not leave a phantom obstacle on the lawn.
"""
import math
import struct
from dataclasses import dataclass
from time import monotonic
from typing import Dict, Iterable, List, Tuple

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

_DEFAULT_PEER_RADIUS_M = 0.6
_DEFAULT_RING_POINTS = 24
_DEFAULT_PUBLISH_RATE_HZ = 5.0
_DEFAULT_PEER_TIMEOUT_S = 5.0
_DEFAULT_POINT_HEIGHT_M = 0.30
_DEFAULT_FRAME_ID = "map"
_POINT_STRIDE = 12  # three float32


@dataclass(frozen=True)
class PeerPose:
    x: float
    y: float
    seen_at: float


def ring_points(cx: float, cy: float, radius: float, count: int, z: float) -> List[Tuple[float, float, float]]:
    """Evenly spaced points on a circle of `radius` around (cx, cy) at height z.

    The centre point is included as well so a very small radius (or a costmap
    coarser than the ring spacing) still marks at least the robot's centre.
    """
    if count < 1 or radius <= 0.0:
        return [(cx, cy, z)]
    step = 2.0 * math.pi / count
    pts = [(cx + radius * math.cos(i * step), cy + radius * math.sin(i * step), z) for i in range(count)]
    pts.append((cx, cy, z))
    return pts


def fresh_peers(peers: Dict[int, PeerPose], now: float, timeout_s: float) -> List[PeerPose]:
    """Peers whose last pose is younger than timeout_s (stale ones are dropped)."""
    return [p for p in peers.values() if now - p.seen_at <= timeout_s]


def build_cloud(header: Header, points: Iterable[Tuple[float, float, float]]) -> PointCloud2:
    """Pack xyz float32 points into an unorganised PointCloud2."""
    pts = list(points)
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(pts)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = _POINT_STRIDE
    cloud.row_step = _POINT_STRIDE * len(pts)
    cloud.is_dense = True
    cloud.data = b"".join(struct.pack("<fff", x, y, z) for x, y, z in pts)
    return cloud


class FleetPeerObstaclesNode(Node):
    def __init__(self) -> None:
        super().__init__("fleet_peer_obstacles")
        self.declare_parameter("peer_radius_m", _DEFAULT_PEER_RADIUS_M)
        self.declare_parameter("ring_points", _DEFAULT_RING_POINTS)
        self.declare_parameter("publish_rate_hz", _DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("peer_timeout_s", _DEFAULT_PEER_TIMEOUT_S)
        self.declare_parameter("point_height_m", _DEFAULT_POINT_HEIGHT_M)
        self.declare_parameter("frame_id", _DEFAULT_FRAME_ID)

        self._radius = float(self.get_parameter("peer_radius_m").value)
        self._ring_points = int(self.get_parameter("ring_points").value)
        self._timeout_s = float(self.get_parameter("peer_timeout_s").value)
        self._height = float(self.get_parameter("point_height_m").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        rate_hz = max(0.5, float(self.get_parameter("publish_rate_hz").value))

        # Keyed by the pose's index in the PoseArray: the coordinator sends all
        # peers in one message, so a full message replaces the whole set.
        self._peers: Dict[int, PeerPose] = {}

        # The GUI publishes through foxglove_bridge (reliable, volatile).
        self._sub = self.create_subscription(
            PoseArray,
            "/fleet/peers",
            self._on_peers,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE),
        )
        # A costmap observation source subscribes with its own (sensor-data)
        # QoS; RELIABLE publisher → BEST_EFFORT subscriber is compatible.
        self._pub = self.create_publisher(PointCloud2, "/fleet/peer_obstacles", QoSProfile(depth=5))
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(
            f"fleet_peer_obstacles: radius {self._radius:.2f} m, {self._ring_points} ring points, "
            f"{rate_hz:.1f} Hz, peers expire after {self._timeout_s:.1f} s"
        )

    def _on_peers(self, msg: PoseArray) -> None:
        now = monotonic()
        self._peers = {
            i: PeerPose(p.position.x, p.position.y, now)
            for i, p in enumerate(msg.poses)
            if math.isfinite(p.position.x) and math.isfinite(p.position.y)
        }

    def _publish(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        points: List[Tuple[float, float, float]] = []
        for peer in fresh_peers(self._peers, monotonic(), self._timeout_s):
            points.extend(ring_points(peer.x, peer.y, self._radius, self._ring_points, self._height))
        self._pub.publish(build_cloud(header, points))


def main() -> None:
    rclpy.init()
    node = FleetPeerObstaclesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
