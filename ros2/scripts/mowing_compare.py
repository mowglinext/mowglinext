#!/usr/bin/env python3
# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0-or-later

"""
mowing_compare.py — Visualise the F2C planned path vs the robot's actual
trajectory vs the mowed-cell mask, all overlaid in one PNG.

Subscribes to:
  /coverage_server/coverage_plan       (one-shot, volatile)  — the F2C plan
  /odometry/filtered_map               (continuous)          — robot pose
  /map_server_node/coverage_cells      (transient_local)     — mowed mask
  /map_server_node/keepout_mask        (transient_local)     — polygon

Every refresh_sec (default 5 s) it overwrites /tmp/mowing_compare.png with:
  * blue line       : planned F2C path
  * red trajectory  : robot's actual /odometry/filtered_map track
  * red dot         : current robot pose
  * green raster    : mowed cells (mow_progress ≥ 0.3)
  * gray background : keepout mask (outside polygon = lethal)

Run BEFORE sending COMMAND_START so the volatile coverage_plan message is
captured. The plot updates while the robot mows. Ctrl-C saves a final PNG
and exits cleanly.

Usage:
  source /opt/ros/kilted/setup.bash && source install/setup.bash
  python3 ros2/scripts/mowing_compare.py [--out /tmp/mowing_compare.png]
                                         [--refresh-sec 5.0]
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from nav_msgs.msg import OccupancyGrid, Odometry, Path

import matplotlib

matplotlib.use("Agg")  # no display; we only write PNGs
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap


@dataclass
class GridFrame:
    """Snapshot of an OccupancyGrid that's stable enough to plot from."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: np.ndarray  # shape (height, width), int8

    @classmethod
    def from_msg(cls, msg: OccupancyGrid) -> "GridFrame":
        arr = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        return cls(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            data=arr,
        )

    def extent(self) -> Tuple[float, float, float, float]:
        x0 = self.origin_x
        y0 = self.origin_y
        x1 = x0 + self.width * self.resolution
        y1 = y0 + self.height * self.resolution
        return (x0, x1, y0, y1)


class MowingCompare(Node):
    def __init__(self) -> None:
        super().__init__("mowing_compare")
        self._lock = threading.Lock()
        self._path: List[Tuple[float, float]] = []
        self._traj: List[Tuple[float, float]] = []
        self._latest_pose: Optional[Tuple[float, float, float]] = None  # x, y, yaw
        self._coverage_grid: Optional[GridFrame] = None
        self._keepout_grid: Optional[GridFrame] = None
        self._path_received = False

        # /coverage_server/coverage_plan is published once per ComputeCoverage
        # Path call with volatile QoS — we MUST be subscribed when it fires.
        plan_qos = QoSProfile(depth=1)
        plan_qos.reliability = ReliabilityPolicy.RELIABLE
        plan_qos.durability = DurabilityPolicy.VOLATILE
        plan_qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(Path, "/coverage_server/coverage_plan", self._on_path, plan_qos)

        # /keepout_mask is published transient_local (latched); /coverage_cells
        # is volatile and republished at map_server's publish_rate (~1 Hz).
        keepout_qos = QoSProfile(depth=1)
        keepout_qos.reliability = ReliabilityPolicy.RELIABLE
        keepout_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        keepout_qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(OccupancyGrid, "/keepout_mask", self._on_keepout, keepout_qos)

        coverage_qos = QoSProfile(depth=1)
        coverage_qos.reliability = ReliabilityPolicy.RELIABLE
        coverage_qos.durability = DurabilityPolicy.VOLATILE
        coverage_qos.history = HistoryPolicy.KEEP_LAST
        self.create_subscription(
            OccupancyGrid, "/map_server_node/coverage_cells", self._on_coverage, coverage_qos
        )

        self.create_subscription(Odometry, "/odometry/filtered_map", self._on_odom, 10)

    def _on_path(self, msg: Path) -> None:
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self._lock:
            self._path = pts
            self._path_received = True
        self.get_logger().info(
            f"Coverage path received: {len(pts)} poses"
            + (
                f", first=({pts[0][0]:.2f},{pts[0][1]:.2f}) last=({pts[-1][0]:.2f},{pts[-1][1]:.2f})"
                if pts
                else ""
            )
        )

    def _on_coverage(self, msg: OccupancyGrid) -> None:
        grid = GridFrame.from_msg(msg)
        with self._lock:
            self._coverage_grid = grid

    def _on_keepout(self, msg: OccupancyGrid) -> None:
        grid = GridFrame.from_msg(msg)
        with self._lock:
            self._keepout_grid = grid

    def _on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._latest_pose = (x, y, yaw)
            # Decimate: only append if moved > 5 cm from the last sample.
            if not self._traj or math.hypot(x - self._traj[-1][0], y - self._traj[-1][1]) > 0.05:
                self._traj.append((x, y))

    def render(self, out_path: str) -> None:
        with self._lock:
            path = list(self._path)
            traj = list(self._traj)
            pose = self._latest_pose
            cov = self._coverage_grid
            keep = self._keepout_grid

        fig, ax = plt.subplots(figsize=(10, 8), dpi=110)

        # Background: keepout mask (gray = lethal, white = free).
        if keep is not None:
            kmap = ListedColormap(["white", "#dddddd", "#888888"])
            # OccupancyGrid: -1 unknown, 0 free, 100 lethal — bin to 0/1/2.
            disp = np.full_like(keep.data, 0, dtype=np.int8)
            disp[keep.data < 0] = 1  # unknown
            disp[keep.data >= 50] = 2  # lethal
            ax.imshow(
                disp,
                origin="lower",
                extent=keep.extent(),
                cmap=kmap,
                alpha=0.6,
                interpolation="nearest",
            )

        # Coverage cells: green where mowed.
        if cov is not None:
            mowed = (cov.data >= 50).astype(float)
            cmap = ListedColormap(["#ffffff00", "#7fcc7f"])
            ax.imshow(
                mowed,
                origin="lower",
                extent=cov.extent(),
                cmap=cmap,
                alpha=0.7,
                interpolation="nearest",
                vmin=0,
                vmax=1,
            )

        # Planned F2C path.
        if path:
            xs, ys = zip(*path)
            ax.plot(xs, ys, "-", color="#3060d0", lw=1.2, label=f"F2C path ({len(path)} pts)")
            # Tag start/end.
            ax.plot(xs[0], ys[0], "o", color="#3060d0", ms=8)
            ax.plot(xs[-1], ys[-1], "s", color="#3060d0", ms=8)

        # Actual trajectory.
        if traj:
            xs, ys = zip(*traj)
            ax.plot(xs, ys, "-", color="#d04030", lw=1.2, label=f"robot track ({len(traj)} pts)")

        # Current robot pose: red triangle pointing in heading direction.
        if pose is not None:
            x, y, yaw = pose
            arrow_len = 0.3
            ax.arrow(
                x,
                y,
                arrow_len * math.cos(yaw),
                arrow_len * math.sin(yaw),
                width=0.05,
                head_width=0.18,
                head_length=0.18,
                length_includes_head=True,
                color="#d04030",
                zorder=10,
            )
            ax.plot(x, y, "o", color="#d04030", ms=6, zorder=11)

        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        title = f"mow compare  |  path={len(path)}  traj={len(traj)}"
        if pose is not None:
            title += f"  |  pose=({pose[0]:.2f},{pose[1]:.2f}, {math.degrees(pose[2]):+.0f}°)"
        ax.set_title(title)
        ax.grid(True, ls=":", lw=0.4, alpha=0.5)
        ax.legend(loc="upper right", fontsize=8)

        # Keep camera framing the polygon if we have one, else auto.
        if keep is not None:
            x0, x1, y0, y1 = keep.extent()
            ax.set_xlim(x0 - 0.5, x1 + 0.5)
            ax.set_ylim(y0 - 0.5, y1 + 0.5)

        # Atomic-replace via a sibling tmp file so a half-written PNG never
        # gets observed mid-render. Matplotlib infers the format from the
        # extension, so the tmp must end in .png too.
        tmp = out_path + ".part.png"
        fig.tight_layout()
        fig.savefig(tmp)
        plt.close(fig)
        os.replace(tmp, out_path)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--out", default="/tmp/mowing_compare.png", help="output PNG path")
    parser.add_argument(
        "--refresh-sec", type=float, default=5.0, help="render cadence (default 5 s)"
    )
    args = parser.parse_args()

    rclpy.init()
    node = MowingCompare()
    last_render = 0.0
    print(
        f"Listening on /coverage_server/coverage_plan, /odometry/filtered_map, "
        f"/map_server_node/coverage_cells, /keepout_mask",
        flush=True,
    )
    print(f"Writing PNG to {args.out} every {args.refresh_sec:.1f} s. Ctrl-C to stop.", flush=True)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_render >= args.refresh_sec:
                node.render(args.out)
                last_render = now
                with node._lock:
                    msg = (
                        f"[{time.strftime('%H:%M:%S')}] rendered "
                        f"path={len(node._path)} traj={len(node._traj)} "
                        f"cov={'y' if node._coverage_grid else 'n'} "
                        f"keep={'y' if node._keepout_grid else 'n'}"
                    )
                print(msg, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.render(args.out)
        print(f"Final render saved to {args.out}", flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
