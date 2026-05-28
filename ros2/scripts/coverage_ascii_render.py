#!/usr/bin/env python3
"""ASCII render of the current mowing state.

Subscribes briefly to /map_server_node/coverage_cells (OccupancyGrid) and
/odometry/filtered_map, then prints:
  ' ' outside the polygon              (cell value -1 == unknown)
  '.' unmowed inside the polygon       (cell value ~60, map_server's
                                        in-polygon-not-yet-mowed marker)
  '#' mowed                            (cell value 0)
  'R' current robot pose
plus a header with coverage % and grid size.

Run from inside the workspace with ROS2 sourced:
  python3 ros2/scripts/coverage_ascii_render.py [--rows 30]
"""

import argparse
import math
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry


class CoverageSnapshot(Node):
    def __init__(self):
        super().__init__('coverage_ascii_render')
        self.grid = None
        self.robot_x = None
        self.robot_y = None
        self._done = threading.Event()

        # map_server_node publishes coverage_cells VOLATILE; match that.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, '/map_server_node/coverage_cells', self.on_grid, qos)
        self.create_subscription(
            Odometry, '/odometry/filtered_map', self.on_odom, 10)

    def on_grid(self, msg):
        self.grid = msg
        if self.robot_x is not None:
            self._done.set()

    def on_odom(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        if self.grid is not None:
            self._done.set()

    def wait(self, timeout):
        return self._done.wait(timeout)


def render(grid: OccupancyGrid, robot_x, robot_y, target_rows):
    w, h = grid.info.width, grid.info.height
    res = grid.info.resolution
    origin_x = grid.info.origin.position.x
    origin_y = grid.info.origin.position.y

    # Downsample so the printout fits a terminal.
    # OccupancyGrid is row-major: data[y_row * w + x_col].
    step_y = max(1, h // target_rows)
    # Terminal char aspect ratio is ~2:1 (chars are taller than wide); for
    # visual squareness, double the X-step so we don't get a smushed map.
    step_x = max(1, step_y * 2)

    rows = []
    for r0 in range(0, h, step_y):
        line = []
        for c0 in range(0, w, step_x):
            # Take the most-mowed cell within the block (so coverage shows
            # up even when downsampling). Mark mowed if any block-cell is
            # mowed, in-polygon if any is unknown-but-not-outside, etc.
            # map_server_node encodes: 0 = mowed, ~60 = unmowed-in-polygon,
            # -1 = outside. Within a downsample block, prefer 'mowed' >
            # 'unmowed' > 'outside' so coverage stays visible.
            mowed = False
            in_poly = False
            for dy in range(step_y):
                rr = r0 + dy
                if rr >= h:
                    break
                for dx in range(step_x):
                    cc = c0 + dx
                    if cc >= w:
                        break
                    v = grid.data[rr * w + cc]
                    if v == 0:
                        mowed = True
                    elif v != -1:
                        in_poly = True
            if mowed:
                ch = '#'
            elif in_poly:
                ch = '.'
            else:
                ch = ' '
            line.append(ch)
        rows.append(''.join(line))

    # Robot marker — convert world XY to grid (col, row), then to
    # downsampled (block_col, block_row).
    if robot_x is not None and res > 0:
        col = int((robot_x - origin_x) / res)
        row = int((robot_y - origin_y) / res)
        if 0 <= col < w and 0 <= row < h:
            br = row // step_y
            bc = col // step_x
            if 0 <= br < len(rows) and 0 <= bc < len(rows[br]):
                row_chars = list(rows[br])
                row_chars[bc] = 'R'
                rows[br] = ''.join(row_chars)

    # Flip vertically so +Y points up (north) in the printout, like a map.
    rows.reverse()

    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rows', type=int, default=30,
                    help='target number of output rows (downsample target)')
    ap.add_argument('--timeout', type=float, default=5.0)
    args = ap.parse_args()

    rclpy.init()
    node = CoverageSnapshot()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    ok = node.wait(args.timeout)
    if not ok or node.grid is None:
        print(f'ERROR: no coverage grid received in {args.timeout}s', file=sys.stderr)
        rclpy.shutdown()
        sys.exit(1)

    grid = node.grid
    # 0 = mowed, anything > 0 = unmowed-in-polygon, -1 = outside.
    total = sum(1 for v in grid.data if v != -1)
    mowed = sum(1 for v in grid.data if v == 0)
    pct = (100.0 * mowed / total) if total else 0.0

    rows = render(grid, node.robot_x, node.robot_y, args.rows)

    print(f'Coverage: {pct:.1f}% ({mowed}/{total} cells, '
          f'grid {grid.info.width}x{grid.info.height}, '
          f'res={grid.info.resolution:.3f}m, '
          f'origin=({grid.info.origin.position.x:.2f}, '
          f'{grid.info.origin.position.y:.2f}))')
    if node.robot_x is not None:
        print(f'Robot pose (map): ({node.robot_x:.2f}, {node.robot_y:.2f})')
    print('Legend: # mowed, . unmowed-in-polygon, + boundary, R robot, space outside')
    print('North is up; +X = east, +Y = north.')
    print('-' * (len(rows[0]) if rows else 1))
    for r in rows:
        print(r)
    print('-' * (len(rows[0]) if rows else 1))

    rclpy.shutdown()


if __name__ == '__main__':
    main()
