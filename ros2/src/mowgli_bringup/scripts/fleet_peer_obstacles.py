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
  * A peer ON ITS CHARGER next to our dock is dock infrastructure while we
    undock or dock beside it (classify_docked_peer). Its block is grown by
    the footprint margin AND peer_margin_m, on top of OUR footprint's own
    margin: two mowers parked side by side (2026-10-28: 0.53 m apart, 7.6 cm
    between the bodies) overlap by 0.18 m in that model, so BackUp refused
    every undock at its first cycle and collision_monitor would have frozen
    it too. No margin value fixes it — even the bare chassis collides once
    rasterised at 0.05 m. The exception is narrow: only a peer the feed
    lists on /fleet/docked_peers, only while we are aligned on our own dock
    rail, only while its physical body stays clear of that rail and of our
    body, and only while the margins are what would collide. Anywhere else,
    or with any input missing, it is a full obstacle.
"""
import math
import struct
from dataclasses import dataclass
from enum import Enum
from time import monotonic
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
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
# Template undock_distance (mowgli_robot.yaml); the launch injects the real one.
_DEFAULT_UNDOCK_DISTANCE_M = 1.5

# Dock-neighbour exception (classify_docked_peer). Our dock rail runs from the
# dock pose back by undock_distance; staging_x_offset (-1.5 m) lies on it.
_DOCK_RAIL_SLACK_M = 0.5  # along-track, past the dock and past the undock end
_DOCK_RAIL_LATERAL_M = 0.30  # base_footprint off the rail axis
_DOCK_RAIL_MAX_YAW_RAD = math.radians(30.0)  # heading off the dock heading
# Real gap kept between the two PHYSICAL bodies while the exception holds.
_DOCKED_PEER_MIN_CLEARANCE_M = 0.02
# The exception ends once our footprint is this far from the peer's grown
# block. Above RPP's collision look-ahead (1.0 s at 0.30 m/s), so the block is
# gone before a transit to the staging pose reaches it and back once the
# undock has reversed clear of it (0.54 m at the 1.5 m template distance).
_DOCK_NEIGHBOUR_RELEASE_GAP_M = 0.5
# /fleet/docked_peers repeats the matching /fleet/peers pose from the same feed tick.
_DOCKED_MATCH_TOLERANCE_M = 0.05
_DOCKED_MATCH_YAW_RAD = 0.1
_ROBOT_POSE_MAX_AGE_S = 1.0


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


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class DockGeometry:
    """What classify_docked_peer measures with: the PHYSICAL chassis, the
    margin-grown footprint the peer block is built from, the extra peer
    margin, and how far our undock reverses."""

    body: Footprint
    grown: Footprint
    peer_margin: float
    undock_distance: float


class DockNeighbour(Enum):
    OFF_RAIL = "off_rail"  # we are not undocking/docking: full obstacle
    BLOCKS_RAIL = "blocks_rail"  # its body is on our way out: full obstacle
    TOO_CLOSE = "too_close"  # bodies about to touch: full obstacle
    CLEAR = "clear"  # its block is nowhere near us: full obstacle
    DOCK_INFRASTRUCTURE = "dock_infrastructure"  # dropped from the cloud


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


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def pose2d_from_msg(pose: Pose) -> Optional[Pose2D]:
    """A finite pose with a real heading, or None."""
    yaw = yaw_from_quaternion(pose.orientation)
    if yaw is None or not (math.isfinite(pose.position.x) and math.isfinite(pose.position.y)):
        return None
    return Pose2D(pose.position.x, pose.position.y, yaw)


def in_frame(pose: Pose2D, origin: Pose2D) -> Pose2D:
    """`pose` expressed in the frame of `origin` (x along origin's heading)."""
    dx, dy = pose.x - origin.x, pose.y - origin.y
    c, s = math.cos(origin.yaw), math.sin(origin.yaw)
    return Pose2D(c * dx + s * dy, -s * dx + c * dy, wrap_angle(pose.yaw - origin.yaw))


def footprint_corners(pose: Pose2D, fp: Footprint, grow: float = 0.0) -> List[Tuple[float, float]]:
    """The four corners of `fp` (grown by `grow` on every side) placed at `pose`."""
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    front, rear, half = fp.front + grow, fp.rear - grow, fp.half_width + grow
    local = [(front, half), (front, -half), (rear, -half), (rear, half)]
    return [(pose.x + c * lx - s * ly, pose.y + s * lx + c * ly) for lx, ly in local]


def _point_segment_distance(p: Tuple[float, float], a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length_sq = dx * dx + dy * dy
    t = 0.0 if length_sq == 0.0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length_sq))
    return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))


def _edges(poly: Sequence[Tuple[float, float]]) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    return [(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly))]


def _separated(a: Sequence[Tuple[float, float]], b: Sequence[Tuple[float, float]]) -> bool:
    """Separating-axis test for two convex polygons."""
    for p0, p1 in _edges(a) + _edges(b):
        nx, ny = p1[1] - p0[1], p0[0] - p1[0]
        pa = [nx * x + ny * y for x, y in a]
        pb = [nx * x + ny * y for x, y in b]
        if max(pa) < min(pb) or max(pb) < min(pa):
            return True
    return False


def polygon_clearance(a: Sequence[Tuple[float, float]], b: Sequence[Tuple[float, float]]) -> float:
    """Gap between two convex polygons; 0 when they touch or overlap."""
    if not _separated(a, b):
        return 0.0
    return min(
        min(_point_segment_distance(p, e0, e1) for p in a for e0, e1 in _edges(b)),
        min(_point_segment_distance(p, e0, e1) for p in b for e0, e1 in _edges(a)),
    )


def on_dock_rail(robot: Pose2D, dock: Pose2D, undock_distance: float) -> bool:
    """We are on the straight line an undock reverses along (and a dock
    approach comes in on), heading the dock's way."""
    r = in_frame(robot, dock)
    return (
        -(undock_distance + _DOCK_RAIL_SLACK_M) <= r.x <= _DOCK_RAIL_SLACK_M
        and abs(r.y) <= _DOCK_RAIL_LATERAL_M
        and abs(r.yaw) <= _DOCK_RAIL_MAX_YAW_RAD
    )


def classify_docked_peer(peer: Pose2D, robot: Pose2D, dock: Pose2D, geo: DockGeometry) -> DockNeighbour:
    """How a peer that is ON ITS CHARGER relates to our own dock manoeuvres.

    Only DOCK_INFRASTRUCTURE drops it from the cloud. The physical checks use
    the bare chassis because the question is "can the bodies touch", which no
    margin answers; the release check uses the same grown shapes the costmap
    and the collision monitor see, so the peer is back before they would."""
    if not on_dock_rail(robot, dock, geo.undock_distance):
        return DockNeighbour.OFF_RAIL
    peer_body = footprint_corners(peer, geo.body)
    rail = Footprint(front=geo.body.front, rear=geo.body.rear - geo.undock_distance, half_width=geo.body.half_width)
    if polygon_clearance(peer_body, footprint_corners(dock, rail)) < _DOCKED_PEER_MIN_CLEARANCE_M:
        return DockNeighbour.BLOCKS_RAIL
    if polygon_clearance(peer_body, footprint_corners(robot, geo.body)) < _DOCKED_PEER_MIN_CLEARANCE_M:
        return DockNeighbour.TOO_CLOSE
    block = footprint_corners(peer, geo.grown, geo.peer_margin)
    if polygon_clearance(block, footprint_corners(robot, geo.grown)) >= _DOCK_NEIGHBOUR_RELEASE_GAP_M:
        return DockNeighbour.CLEAR
    return DockNeighbour.DOCK_INFRASTRUCTURE


def is_listed_docked(peer: PeerPose, docked: Sequence[Pose2D]) -> bool:
    """The feed listed this peer on /fleet/docked_peers (same tick, same pose)."""
    if peer.yaw is None:
        return False
    return any(
        math.hypot(peer.x - d.x, peer.y - d.y) <= _DOCKED_MATCH_TOLERANCE_M
        and abs(wrap_angle(peer.yaw - d.yaw)) <= _DOCKED_MATCH_YAW_RAD
        for d in docked
    )


def dock_verdicts(
    peers: Sequence[PeerPose],
    docked: Sequence[Pose2D],
    robot: Optional[Pose2D],
    dock: Optional[Pose2D],
    geo: DockGeometry,
) -> List[Optional[DockNeighbour]]:
    """One verdict per peer: None when it is not a charging peer or when our
    own pose or dock pose is unknown — i.e. it stays a full obstacle."""
    if robot is None or dock is None:
        return [None] * len(peers)
    return [
        classify_docked_peer(Pose2D(p.x, p.y, p.yaw), robot, dock, geo)
        if p.yaw is not None and is_listed_docked(p, docked)
        else None
        for p in peers
    ]


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
        # The bare chassis (no footprint margin) and the undock reverse, for
        # the dock-neighbour exception; injected from mowgli_robot.yaml.
        self.declare_parameter("body_front_m", _DEFAULT_CHASSIS_FRONT_M)
        self.declare_parameter("body_rear_m", _DEFAULT_CHASSIS_REAR_M)
        self.declare_parameter("body_half_width_m", _DEFAULT_CHASSIS_HALF_WIDTH_M)
        self.declare_parameter("undock_distance_m", _DEFAULT_UNDOCK_DISTANCE_M)

        self._footprint = Footprint(
            front=float(self.get_parameter("chassis_front_m").value),
            rear=float(self.get_parameter("chassis_rear_m").value),
            half_width=float(self.get_parameter("chassis_half_width_m").value),
        )
        self._margin = float(self.get_parameter("peer_margin_m").value)
        self._dock_geometry = DockGeometry(
            body=Footprint(
                front=float(self.get_parameter("body_front_m").value),
                rear=float(self.get_parameter("body_rear_m").value),
                half_width=float(self.get_parameter("body_half_width_m").value),
            ),
            grown=self._footprint,
            peer_margin=self._margin,
            undock_distance=float(self.get_parameter("undock_distance_m").value),
        )
        self._step = max(0.01, float(self.get_parameter("fill_step_m").value))
        self._timeout_s = float(self.get_parameter("peer_timeout_s").value)
        self._height = float(self.get_parameter("point_height_m").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        rate_hz = max(0.5, float(self.get_parameter("publish_rate_hz").value))

        # Keyed by the pose's index in the PoseArray: the feed sends all
        # peers in one message, so a full message replaces the whole set.
        self._peers: Dict[int, PeerPose] = {}
        self._messages = 0
        # Dock-neighbour inputs. Each is replaced whole on arrival; a stale or
        # missing one means no exception (the peer stays a full obstacle).
        self._docked: Tuple[Pose2D, ...] = ()
        self._docked_seen_at = -math.inf
        self._robot: Optional[Pose2D] = None
        self._robot_seen_at = -math.inf
        self._dock: Optional[Pose2D] = None
        self._verdicts: Tuple[Optional[DockNeighbour], ...] = ()

        # The GUI publishes through foxglove_bridge (reliable, volatile).
        feed_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self._sub = self.create_subscription(PoseArray, "/fleet/peers", self._on_peers, feed_qos)
        self._docked_sub = self.create_subscription(PoseArray, "/fleet/docked_peers", self._on_docked, feed_qos)
        self._odom_sub = self.create_subscription(
            Odometry,
            "/odometry/filtered_map",
            self._on_odom,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE),
        )
        # map_server latches the dock pose (transient_local) and re-sends it on change.
        self._dock_sub = self.create_subscription(
            PoseStamped,
            "/map_server_node/docking_pose",
            self._on_dock,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
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

    def _on_docked(self, msg: PoseArray) -> None:
        poses = (pose2d_from_msg(p) for p in msg.poses)
        self._docked = tuple(p for p in poses if p is not None)
        self._docked_seen_at = monotonic()

    def _on_odom(self, msg: Odometry) -> None:
        self._robot = pose2d_from_msg(msg.pose.pose)
        self._robot_seen_at = monotonic()

    def _on_dock(self, msg: PoseStamped) -> None:
        self._dock = pose2d_from_msg(msg.pose)
        self.get_logger().info(f"fleet_peer_obstacles: dock pose {self._dock}")

    def _publish(self) -> None:
        now = monotonic()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        peers = fresh_peers(self._peers, now, self._timeout_s)
        docked = self._docked if now - self._docked_seen_at <= self._timeout_s else ()
        robot = self._robot if now - self._robot_seen_at <= _ROBOT_POSE_MAX_AGE_S else None
        verdicts = dock_verdicts(peers, docked, robot, self._dock, self._dock_geometry)
        self._log_verdict_changes(verdicts)
        points: List[Tuple[float, float, float]] = []
        for peer, verdict in zip(peers, verdicts):
            if verdict is not DockNeighbour.DOCK_INFRASTRUCTURE:
                points.extend(peer_points(peer, self._footprint, self._margin, self._step, self._height))
        self._pub.publish(build_cloud(header, points))

    def _log_verdict_changes(self, verdicts: List[Optional[DockNeighbour]]) -> None:
        """One line per change, never per tick: the operator needs to know
        when a docked neighbour is let through, and above all when it is not."""
        current = tuple(verdicts)
        if current == self._verdicts:
            return
        self._verdicts = current
        for i, verdict in enumerate(current):
            if verdict is DockNeighbour.DOCK_INFRASTRUCTURE:
                self.get_logger().info(f"fleet_peer_obstacles: peer {i} is on its charger beside our dock — "
                                       "treated as dock infrastructure while we undock/dock")
            elif verdict in (DockNeighbour.BLOCKS_RAIL, DockNeighbour.TOO_CLOSE):
                self.get_logger().warning(
                    f"fleet_peer_obstacles: peer {i} is on its charger but {verdict.value.replace('_', ' ')} "
                    f"(bodies closer than {_DOCKED_PEER_MIN_CLEARANCE_M:.2f} m to our dock rail or chassis) — "
                    "kept as an obstacle, so undocking/docking beside it is refused; move the docks apart"
                )


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
