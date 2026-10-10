#!/usr/bin/env python3
"""
fleet_peer_obstacles — the other fleet members' footprints as costmap points.

Robots never see each other over DDS (Cyclone is pinned to loopback, issue
#418). The GUI backend on THIS robot mirrors its peers over their GUI API and
publishes their live poses into the local ROS graph through foxglove's
clientPublish as a geometry_msgs/PoseArray on /fleet/peers (map frame, 5 Hz,
heading included). This node turns every fresh peer pose into a dense block
of points covering the peer's CHASSIS FOOTPRINT (plus a margin) and publishes
them as a sensor_msgs/PointCloud2 on /fleet/peer_obstacles at a steady rate.
The Nav2 costmaps mark them lethal, the controllers stop or skirt them, and
the collision monitor (no-LiDAR variant) halts before contact. See
docs/MULTI_ROBOT.md.

Load-bearing properties:
  * The cloud is published CONTINUOUSLY, empty when there are no peers, so a
    costmap observation source with a persistence window never goes stale and
    nothing in Nav2 ever waits on it.
  * A peer pose older than peer_timeout_s is dropped: a robot that went
    offline must not leave a phantom obstacle on the lawn.
  * The footprint is DENSE (fill_step_m, at the costmap resolution) and
    oriented by the peer's heading. A pose whose orientation is all zeros —
    the feed's "no heading" marker, when only the antenna fix was fresh —
    gets a disc covering every orientation instead.
"""
import math
import struct
from dataclasses import dataclass
from time import monotonic
from typing import Dict, Iterable, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseArray, Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

# Chassis extents in base_link (front_x, rear_x, half_width), the same triple
# robot_config_util.chassis_footprint() derives for Nav2's footprint; the
# launch file injects the real values from mowgli_robot.yaml. Peers run the
# same software, so our chassis stands in for theirs.
_DEFAULT_CHASSIS_FRONT_M = 0.48
_DEFAULT_CHASSIS_REAR_M = -0.12
_DEFAULT_CHASSIS_HALF_WIDTH_M = 0.225
# Grown around the footprint: pose latency (up to ~0.4 s over two GUI hops
# at 0.5 m/s is 0.2 m) plus RTK noise between two receivers.
_DEFAULT_PEER_MARGIN_M = 0.15
_DEFAULT_FILL_STEP_M = 0.05  # local costmap resolution
_DEFAULT_PUBLISH_RATE_HZ = 5.0
_DEFAULT_PEER_TIMEOUT_S = 3.0
_DEFAULT_POINT_HEIGHT_M = 0.30
_DEFAULT_FRAME_ID = "map"
_POINT_STRIDE = 12  # three float32
_MIN_QUATERNION_NORM = 0.5


@dataclass(frozen=True)
class PeerPose:
    x: float
    y: float
    yaw: Optional[float]  # None = heading unknown (disc)
    seen_at: float


@dataclass(frozen=True)
class Footprint:
    front: float
    rear: float
    half_width: float


def yaw_from_quaternion(q: Quaternion) -> Optional[float]:
    """Yaw of a unit quaternion, or None for the all-zero "no heading" marker."""
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not math.isfinite(norm) or norm < _MIN_QUATERNION_NORM:
        return None
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _grid(lo: float, hi: float, step: float) -> List[float]:
    """Evenly spaced samples covering [lo, hi] inclusive of both ends."""
    if step <= 0.0 or hi <= lo:
        return [lo]
    n = max(1, int(math.ceil((hi - lo) / step)))
    return [lo + (hi - lo) * i / n for i in range(n + 1)]


def footprint_points(
    cx: float, cy: float, yaw: float, fp: Footprint, margin: float, step: float, z: float
) -> List[Tuple[float, float, float]]:
    """Dense points filling the chassis rectangle (grown by `margin`), rotated by
    `yaw` and placed at (cx, cy). Dense on purpose: the costmap marks a cell per
    point, and an outline would let the inflation layer alone fill the body."""
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    for lx in _grid(fp.rear - margin, fp.front + margin, step):
        for ly in _grid(-fp.half_width - margin, fp.half_width + margin, step):
            pts.append((cx + c * lx - s * ly, cy + s * lx + c * ly, z))
    return pts


def disc_points(cx: float, cy: float, radius: float, step: float, z: float) -> List[Tuple[float, float, float]]:
    """Dense points filling a disc — the heading-agnostic footprint."""
    pts = []
    for lx in _grid(-radius, radius, step):
        for ly in _grid(-radius, radius, step):
            if math.hypot(lx, ly) <= radius:
                pts.append((cx + lx, cy + ly, z))
    if not pts:
        pts.append((cx, cy, z))
    return pts


def circumscribed_radius(fp: Footprint, margin: float) -> float:
    """Radius of the smallest disc around base_link that holds the grown footprint."""
    reach = max(abs(fp.front), abs(fp.rear)) + margin
    return math.hypot(reach, fp.half_width + margin)


def peer_points(peer: PeerPose, fp: Footprint, margin: float, step: float, z: float) -> List[Tuple[float, float, float]]:
    if peer.yaw is None:
        return disc_points(peer.x, peer.y, circumscribed_radius(fp, margin), step, z)
    return footprint_points(peer.x, peer.y, peer.yaw, fp, margin, step, z)


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
        self.declare_parameter("chassis_front_m", _DEFAULT_CHASSIS_FRONT_M)
        self.declare_parameter("chassis_rear_m", _DEFAULT_CHASSIS_REAR_M)
        self.declare_parameter("chassis_half_width_m", _DEFAULT_CHASSIS_HALF_WIDTH_M)
        self.declare_parameter("peer_margin_m", _DEFAULT_PEER_MARGIN_M)
        self.declare_parameter("fill_step_m", _DEFAULT_FILL_STEP_M)
        self.declare_parameter("publish_rate_hz", _DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("peer_timeout_s", _DEFAULT_PEER_TIMEOUT_S)
        self.declare_parameter("point_height_m", _DEFAULT_POINT_HEIGHT_M)
        self.declare_parameter("frame_id", _DEFAULT_FRAME_ID)

        self._footprint = Footprint(
            front=float(self.get_parameter("chassis_front_m").value),
            rear=float(self.get_parameter("chassis_rear_m").value),
            half_width=float(self.get_parameter("chassis_half_width_m").value),
        )
        self._margin = float(self.get_parameter("peer_margin_m").value)
        self._step = max(0.01, float(self.get_parameter("fill_step_m").value))
        self._timeout_s = float(self.get_parameter("peer_timeout_s").value)
        self._height = float(self.get_parameter("point_height_m").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        rate_hz = max(0.5, float(self.get_parameter("publish_rate_hz").value))

        # Keyed by the pose's index in the PoseArray: the feed sends all
        # peers in one message, so a full message replaces the whole set.
        self._peers: Dict[int, PeerPose] = {}
        self._messages = 0

        # The GUI publishes through foxglove_bridge (reliable, volatile).
        self._sub = self.create_subscription(
            PoseArray,
            "/fleet/peers",
            self._on_peers,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE),
        )
        # Costmap and collision-monitor sources subscribe with their own
        # (sensor-data) QoS; RELIABLE publisher → BEST_EFFORT subscriber is fine.
        self._pub = self.create_publisher(PointCloud2, "/fleet/peer_obstacles", QoSProfile(depth=5))
        self._timer = self.create_timer(1.0 / rate_hz, self._publish)
        self.get_logger().info(
            "fleet_peer_obstacles: footprint "
            f"[{self._footprint.rear:+.2f}, {self._footprint.front:+.2f}] x ±{self._footprint.half_width:.3f} m "
            f"+ {self._margin:.2f} m margin, step {self._step:.2f} m, {rate_hz:.1f} Hz, "
            f"peers expire after {self._timeout_s:.1f} s"
        )

    def _on_peers(self, msg: PoseArray) -> None:
        now = monotonic()
        self._messages += 1
        if self._messages == 1:
            self.get_logger().info(f"fleet_peer_obstacles: first /fleet/peers message ({len(msg.poses)} pose(s))")
        self._peers = {
            i: PeerPose(p.position.x, p.position.y, yaw_from_quaternion(p.orientation), now)
            for i, p in enumerate(msg.poses)
            if math.isfinite(p.position.x) and math.isfinite(p.position.y)
        }

    def _publish(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        points: List[Tuple[float, float, float]] = []
        for peer in fresh_peers(self._peers, monotonic(), self._timeout_s):
            points.extend(peer_points(peer, self._footprint, self._margin, self._step, self._height))
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
