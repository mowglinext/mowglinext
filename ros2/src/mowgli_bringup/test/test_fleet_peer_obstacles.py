"""Unit tests for the fleet peer → costmap point-cloud node's pure helpers."""
import importlib.util
import math
import struct
from pathlib import Path

from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header

NODE_PATH = Path(__file__).parents[1] / "scripts" / "fleet_peer_obstacles.py"
SPEC = importlib.util.spec_from_file_location("fleet_peer_obstacles", NODE_PATH)
fleet = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
SPEC.loader.exec_module(fleet)

FP = fleet.Footprint(front=0.48, rear=-0.12, half_width=0.225)


def _quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))


def test_yaw_from_quaternion_reads_the_heading_and_flags_the_no_heading_marker() -> None:
    assert math.isclose(fleet.yaw_from_quaternion(_quat(0.0)), 0.0, abs_tol=1e-9)
    assert math.isclose(fleet.yaw_from_quaternion(_quat(math.pi / 2)), math.pi / 2, abs_tol=1e-9)
    assert math.isclose(fleet.yaw_from_quaternion(_quat(-2.0)), -2.0, abs_tol=1e-9)
    # The feed sends an all-zero quaternion when only the antenna fix was fresh
    # (ROS's default Quaternion() is the IDENTITY, w=1 — that one is a heading).
    assert fleet.yaw_from_quaternion(Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)) is None
    assert fleet.yaw_from_quaternion(Quaternion()) == 0.0


def test_footprint_points_fill_the_rotated_chassis_rectangle() -> None:
    pts = fleet.footprint_points(2.0, -1.0, math.pi / 2, FP, margin=0.0, step=0.05, z=0.3)

    # Dense: a 0.60 x 0.45 m body at 0.05 m needs about 13 x 10 samples.
    assert len(pts) >= 13 * 10
    assert all(z == 0.3 for _, _, z in pts)
    # Heading +90°: the body's +x (front) axis points along +y in the map.
    xs = [x for x, _, _ in pts]
    ys = [y for _, y, _ in pts]
    assert math.isclose(min(xs), 2.0 - FP.half_width, abs_tol=1e-6)
    assert math.isclose(max(xs), 2.0 + FP.half_width, abs_tol=1e-6)
    assert math.isclose(min(ys), -1.0 + FP.rear, abs_tol=1e-6)
    assert math.isclose(max(ys), -1.0 + FP.front, abs_tol=1e-6)


def test_footprint_margin_grows_the_rectangle_on_every_side() -> None:
    plain = fleet.footprint_points(0.0, 0.0, 0.0, FP, margin=0.0, step=0.05, z=0.3)
    grown = fleet.footprint_points(0.0, 0.0, 0.0, FP, margin=0.15, step=0.05, z=0.3)

    assert max(x for x, _, _ in grown) > max(x for x, _, _ in plain)
    assert min(x for x, _, _ in grown) < min(x for x, _, _ in plain)
    assert max(y for _, y, _ in grown) > max(y for _, y, _ in plain)
    assert math.isclose(max(x for x, _, _ in grown), FP.front + 0.15, abs_tol=1e-6)
    assert math.isclose(min(x for x, _, _ in grown), FP.rear - 0.15, abs_tol=1e-6)


def test_disc_points_cover_every_heading_when_the_heading_is_unknown() -> None:
    radius = fleet.circumscribed_radius(FP, margin=0.15)
    assert math.isclose(radius, math.hypot(0.48 + 0.15, 0.225 + 0.15), abs_tol=1e-9)

    peer = fleet.PeerPose(x=1.0, y=1.0, yaw=None, seen_at=0.0)
    pts = fleet.peer_points(peer, FP, margin=0.15, step=0.05, z=0.3)

    assert len(pts) > 100
    assert all(math.hypot(x - 1.0, y - 1.0) <= radius + 1e-9 for x, y, _ in pts)
    # The nose of the longest possible orientation is inside the disc.
    assert any(math.hypot(x - 1.0, y - 1.0) > radius - 0.06 for x, y, _ in pts)


def test_peer_points_use_the_oriented_footprint_when_a_heading_is_known() -> None:
    peer = fleet.PeerPose(x=0.0, y=0.0, yaw=0.0, seen_at=0.0)
    pts = fleet.peer_points(peer, FP, margin=0.0, step=0.05, z=0.3)
    assert math.isclose(max(x for x, _, _ in pts), FP.front, abs_tol=1e-6)
    assert math.isclose(min(x for x, _, _ in pts), FP.rear, abs_tol=1e-6)


def test_fresh_peers_drops_stale_poses() -> None:
    peers = {
        0: fleet.PeerPose(0.0, 0.0, 0.0, seen_at=100.0),
        1: fleet.PeerPose(5.0, 5.0, None, seen_at=90.0),
    }

    fresh = fleet.fresh_peers(peers, now=102.0, timeout_s=3.0)

    assert [p.x for p in fresh] == [0.0]
    assert fleet.fresh_peers(peers, now=200.0, timeout_s=3.0) == []


def test_build_cloud_packs_xyz_float32_and_is_valid_when_empty() -> None:
    header = Header()
    header.frame_id = "map"

    cloud = fleet.build_cloud(header, [(1.0, 2.0, 0.3), (-1.5, 0.0, 0.3)])

    assert cloud.header.frame_id == "map"
    assert cloud.height == 1 and cloud.width == 2
    assert [f.name for f in cloud.fields] == ["x", "y", "z"]
    assert cloud.point_step == 12 and cloud.row_step == 24
    assert struct.unpack("<fff", bytes(cloud.data[:12])) == (1.0, 2.0, 0.30000001192092896)

    empty = fleet.build_cloud(header, [])
    assert empty.width == 0 and empty.row_step == 0 and len(empty.data) == 0


def test_defaults_match_the_costmap_and_the_feed() -> None:
    # nav2_params_*.yaml fleet source height band is [0.0 or 0.12, 1.5]: the
    # points must land inside it or they are silently ignored.
    assert 0.12 < fleet._DEFAULT_POINT_HEIGHT_M < 1.5
    # Local costmap resolution is 0.05 m: a coarser fill leaves holes in the body.
    assert fleet._DEFAULT_FILL_STEP_M <= 0.05
    # The GUI feed runs at 5 Hz; a peer must outlive one missed message but
    # vanish quickly once its robot goes quiet.
    assert 0.4 < fleet._DEFAULT_PEER_TIMEOUT_S <= 5.0


# --- Dock-neighbour exception (two mowers on side-by-side docks) -------------

BODY = fleet.Footprint(front=0.48, rear=-0.12, half_width=0.225)  # chassis_footprint(margin=0)
GROWN = fleet.Footprint(front=0.53, rear=-0.17, half_width=0.275)  # what launch injects as chassis_*
GEO = fleet.DockGeometry(body=BODY, grown=GROWN, peer_margin=0.15, undock_distance=1.5)
DOCK = fleet.Pose2D(0.0, 0.0, 0.0)
# The 2026-10-28 bag, in our dock frame: the neighbour sits 0.526 m to the
# left, 0.107 m behind, same heading — 7.6 cm between the two chassis.
NEIGHBOUR = fleet.Pose2D(-0.107, 0.526, 0.0)


def _square(cx: float, cy: float, half: float):
    return [(cx + half, cy + half), (cx + half, cy - half), (cx - half, cy - half), (cx - half, cy + half)]


def test_polygon_clearance_measures_the_gap_and_is_zero_on_contact() -> None:
    assert math.isclose(fleet.polygon_clearance(_square(0, 0, 0.5), _square(2, 0, 0.5)), 1.0, abs_tol=1e-9)
    assert fleet.polygon_clearance(_square(0, 0, 0.5), _square(0.9, 0.0, 0.5)) == 0.0
    assert fleet.polygon_clearance(_square(0, 0, 0.5), _square(0.2, 0.1, 0.1)) == 0.0  # contained
    # Corner to corner along the diagonal.
    gap = fleet.polygon_clearance(_square(0, 0, 0.5), _square(2, 2, 0.5))
    assert math.isclose(gap, math.hypot(1.0, 1.0), abs_tol=1e-9)


def test_in_frame_expresses_a_pose_relative_to_the_dock() -> None:
    dock = fleet.Pose2D(1.0, 2.0, math.pi / 2)
    behind = fleet.Pose2D(1.0, 1.0, math.pi / 2)  # 1 m "south" = 1 m behind a north-facing dock
    r = fleet.in_frame(behind, dock)
    assert math.isclose(r.x, -1.0, abs_tol=1e-9)
    assert math.isclose(r.y, 0.0, abs_tol=1e-9)
    assert math.isclose(r.yaw, 0.0, abs_tol=1e-9)


def test_the_neighbour_overlaps_our_footprint_in_todays_model() -> None:
    """Why BackUp refused the undock: the grown block reaches 0.184 m into us."""
    block = fleet.footprint_corners(NEIGHBOUR, GROWN, 0.15)
    assert fleet.polygon_clearance(block, fleet.footprint_corners(DOCK, GROWN)) == 0.0
    # ...while the bodies themselves are 7.6 cm apart.
    gap = fleet.polygon_clearance(fleet.footprint_corners(NEIGHBOUR, BODY), fleet.footprint_corners(DOCK, BODY))
    assert math.isclose(gap, 0.526 - 2 * 0.225, abs_tol=1e-9)


def test_a_charging_neighbour_is_dock_infrastructure_while_we_sit_on_our_dock() -> None:
    assert fleet.classify_docked_peer(NEIGHBOUR, DOCK, DOCK, GEO) is fleet.DockNeighbour.DOCK_INFRASTRUCTURE


def test_the_2026_10_28_bag_poses_are_dock_infrastructure() -> None:
    """Map-frame poses straight from rosbag_20261028T151820Z."""
    ours = fleet.Pose2D(-3.957, 1.277, math.radians(-77.0))
    peer = fleet.Pose2D(-3.469, 1.500, math.radians(-76.9))
    assert fleet.classify_docked_peer(peer, ours, ours, GEO) is fleet.DockNeighbour.DOCK_INFRASTRUCTURE


def test_the_exception_holds_through_the_undock_and_ends_once_clear_of_the_block() -> None:
    mid_undock = fleet.Pose2D(-0.8, 0.0, 0.0)
    assert fleet.classify_docked_peer(NEIGHBOUR, mid_undock, DOCK, GEO) is fleet.DockNeighbour.DOCK_INFRASTRUCTURE
    # End of the 1.5 m reverse = the staging pose: our grown front is 0.54 m
    # behind the grown block, so the neighbour is a full obstacle again for
    # the first transit plan (and for the approach to staging).
    staging = fleet.Pose2D(-1.5, 0.0, 0.0)
    assert fleet.classify_docked_peer(NEIGHBOUR, staging, DOCK, GEO) is fleet.DockNeighbour.CLEAR


def test_a_charging_peer_on_our_way_out_is_never_dock_infrastructure() -> None:
    behind_us = fleet.Pose2D(-1.0, 0.0, 0.0)
    assert fleet.classify_docked_peer(behind_us, DOCK, DOCK, GEO) is fleet.DockNeighbour.BLOCKS_RAIL


def test_bodies_about_to_touch_bring_the_neighbour_back() -> None:
    drifted = fleet.Pose2D(0.0, 0.07, 0.0)  # 6 mm left between the chassis
    assert fleet.classify_docked_peer(NEIGHBOUR, drifted, DOCK, GEO) is fleet.DockNeighbour.TOO_CLOSE


def test_off_our_dock_rail_the_neighbour_is_a_full_obstacle() -> None:
    turned = fleet.Pose2D(-1.0, 0.0, math.radians(45.0))
    elsewhere = fleet.Pose2D(-5.0, 3.0, 0.0)
    assert fleet.classify_docked_peer(NEIGHBOUR, turned, DOCK, GEO) is fleet.DockNeighbour.OFF_RAIL
    assert fleet.classify_docked_peer(NEIGHBOUR, elsewhere, DOCK, GEO) is fleet.DockNeighbour.OFF_RAIL


def test_dock_verdicts_need_the_feed_listing_our_pose_and_our_dock() -> None:
    charging = fleet.PeerPose(NEIGHBOUR.x, NEIGHBOUR.y, NEIGHBOUR.yaw, seen_at=0.0)
    disc = fleet.PeerPose(NEIGHBOUR.x, NEIGHBOUR.y, None, seen_at=0.0)
    listed = [fleet.Pose2D(NEIGHBOUR.x + 0.01, NEIGHBOUR.y, NEIGHBOUR.yaw)]

    infra = fleet.DockNeighbour.DOCK_INFRASTRUCTURE
    assert fleet.dock_verdicts([charging], listed, DOCK, DOCK, GEO) == [infra]
    # Not on /fleet/docked_peers (moving, or an older GUI): full obstacle.
    assert fleet.dock_verdicts([charging], [], DOCK, DOCK, GEO) == [None]
    # A heading-less disc is never matched.
    assert fleet.dock_verdicts([disc], listed, DOCK, DOCK, GEO) == [None]
    # Our own pose or dock pose unknown/stale: full obstacle.
    assert fleet.dock_verdicts([charging], listed, None, DOCK, GEO) == [None]
    assert fleet.dock_verdicts([charging], listed, DOCK, None, GEO) == [None]


def test_is_listed_docked_matches_the_same_pose_only() -> None:
    peer = fleet.PeerPose(1.0, 2.0, 0.5, seen_at=0.0)
    assert fleet.is_listed_docked(peer, [fleet.Pose2D(1.02, 2.0, 0.52)])
    assert not fleet.is_listed_docked(peer, [fleet.Pose2D(1.2, 2.0, 0.5)])  # another robot
    assert not fleet.is_listed_docked(peer, [fleet.Pose2D(1.0, 2.0, 0.9)])  # same spot, other heading
