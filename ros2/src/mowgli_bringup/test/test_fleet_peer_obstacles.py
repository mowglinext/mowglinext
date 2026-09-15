"""Unit tests for the fleet peer → costmap point-cloud node's pure helpers."""
import importlib.util
import math
import struct
from pathlib import Path

from std_msgs.msg import Header

NODE_PATH = Path(__file__).parents[1] / "scripts" / "fleet_peer_obstacles.py"
SPEC = importlib.util.spec_from_file_location("fleet_peer_obstacles", NODE_PATH)
fleet = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
SPEC.loader.exec_module(fleet)


def test_ring_points_surround_the_peer_and_keep_its_centre() -> None:
    pts = fleet.ring_points(2.0, -1.0, 0.6, 8, 0.3)

    assert len(pts) == 9
    centre = pts[-1]
    assert centre == (2.0, -1.0, 0.3)
    for x, y, z in pts[:-1]:
        assert math.isclose(math.hypot(x - 2.0, y + 1.0), 0.6, abs_tol=1e-9)
        assert z == 0.3


def test_ring_points_degenerate_radius_marks_only_the_centre() -> None:
    assert fleet.ring_points(1.0, 1.0, 0.0, 8, 0.3) == [(1.0, 1.0, 0.3)]
    assert fleet.ring_points(1.0, 1.0, 0.5, 0, 0.3) == [(1.0, 1.0, 0.3)]


def test_fresh_peers_drops_stale_poses() -> None:
    peers = {
        0: fleet.PeerPose(0.0, 0.0, seen_at=100.0),
        1: fleet.PeerPose(5.0, 5.0, seen_at=90.0),
    }

    fresh = fleet.fresh_peers(peers, now=104.0, timeout_s=5.0)

    assert [p.x for p in fresh] == [0.0]
    assert fleet.fresh_peers(peers, now=200.0, timeout_s=5.0) == []


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


def test_point_height_sits_inside_the_costmap_height_band() -> None:
    # nav2_params_lidar.yaml local obstacle_layer: min_obstacle_height 0.12,
    # max 1.5 — the peer ring must land inside it or it is silently ignored.
    assert 0.12 < fleet._DEFAULT_POINT_HEIGHT_M < 1.5
