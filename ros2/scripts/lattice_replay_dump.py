#!/usr/bin/env python3
"""Dump FTC offset-lattice inputs from a rosbag for offline replay.

Writes what FTCController::planOffsetLattice reads on the robot, one sample per
/local_costmap/costmap message inside each requested time window, for the
`lattice_replay` tool (ros2/src/mowgli_nav2_plugins/test/lattice_replay.cpp):

  <out>/plans/<id>.txt          FTC's plan as published (x y yaw, map frame)
  <out>/keepout.txt|.bin        /keepout_mask, if the bag carries it
  <out>/<window>/<k>/meta.txt   t, grid geometry, map->odom, odom->base_footprint,
                                FTC's carrot (global_point), plan id
  <out>/<window>/<k>/local.bin  /local_costmap/costmap data (int8)
  <out>/<window>/<k>/global.bin /global_costmap/costmap (int8), updates applied

The bag needs /controller_server/FollowCoveragePath/global_plan,
FollowCoveragePath/global_point, /tf, /local_costmap/costmap and
/global_costmap/costmap(+_updates). Split files are read in index order.

Usage (ROS 2 sourced; mowgli_interfaces too if the bag carries its messages):
  python3 ros2/scripts/lattice_replay_dump.py <bag_dir> <out_dir> T0:T1[:name] ...
then
  lattice_replay [--offset M --side S ...] <out_dir>/<name>/*
"""

import argparse
import glob
import math
import os
import re

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

PLAN_TOPICS = ('/controller_server/FollowCoveragePath/global_plan',)
CARROT_TOPICS = ('/FollowCoveragePath/global_point',
                 '/controller_server/FollowCoveragePath/global_point')
LOCAL = '/local_costmap/costmap'
GLOBAL = '/global_costmap/costmap'
GLOBAL_UPDATES = '/global_costmap/costmap_updates'
KEEPOUT = '/keepout_mask'
WANT = set(PLAN_TOPICS + CARROT_TOPICS + (LOCAL, GLOBAL, GLOBAL_UPDATES, KEEPOUT, '/tf'))


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def split_files(bag):
    def index(path):
        m = re.match(r'(\d+)_', os.path.basename(path))
        return (int(m.group(1)) if m else 0, path)
    return sorted(glob.glob(os.path.join(bag, '*.mcap')), key=index)


def parse_windows(specs):
    windows = []
    for i, spec in enumerate(specs):
        parts = spec.split(':')
        name = parts[2] if len(parts) > 2 else 'w%d' % (i + 1)
        windows.append((float(parts[0]), float(parts[1]), name))
    return windows


class Dumper:

    def __init__(self, out, windows):
        self.out = out
        self.windows = windows
        self.plan_id = -1
        self.glob = None
        self.map_odom = None
        self.odom_base = None
        self.carrot = None
        self.counters = {}
        os.makedirs(os.path.join(out, 'plans'), exist_ok=True)

    def window_of(self, t):
        for t0, t1, name in self.windows:
            if t0 <= t <= t1:
                return name
        return None

    def on_plan(self, m, t):
        self.plan_id += 1
        with open(os.path.join(self.out, 'plans', '%d.txt' % self.plan_id), 'w') as fp:
            fp.write('# t %.4f frame %s n %d\n' % (t, m.header.frame_id, len(m.poses)))
            for p in m.poses:
                fp.write('%.5f %.5f %.6f\n' % (p.pose.position.x, p.pose.position.y,
                                               yaw_of(p.pose.orientation)))

    def on_keepout(self, m):
        if os.path.exists(os.path.join(self.out, 'keepout.bin')):
            return
        np.frombuffer(m.data, dtype=np.int8).tofile(os.path.join(self.out, 'keepout.bin'))
        with open(os.path.join(self.out, 'keepout.txt'), 'w') as fp:
            fp.write('%d %d %.6f %.5f %.5f %s\n' % (
                m.info.width, m.info.height, m.info.resolution, m.info.origin.position.x,
                m.info.origin.position.y, m.header.frame_id))

    def on_global(self, m):
        self.glob = dict(info=m.info, frame=m.header.frame_id,
                         data=np.frombuffer(m.data, dtype=np.int8).reshape(
                             m.info.height, m.info.width).copy())

    def on_global_update(self, m):
        if self.glob is None:
            return
        g = self.glob['data']
        upd = np.frombuffer(m.data, dtype=np.int8).reshape(m.height, m.width)
        y1, x1 = min(g.shape[0], m.y + m.height), min(g.shape[1], m.x + m.width)
        g[m.y:y1, m.x:x1] = upd[:y1 - m.y, :x1 - m.x]

    def on_tf(self, m):
        for tr in m.transforms:
            p = (tr.transform.translation.x, tr.transform.translation.y,
                 yaw_of(tr.transform.rotation))
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                self.map_odom = p
            elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_footprint':
                self.odom_base = p

    def on_local(self, m, t):
        name = self.window_of(t)
        if name is None or self.glob is None or self.map_odom is None:
            return
        k = self.counters.get(name, 0)
        self.counters[name] = k + 1
        d = os.path.join(self.out, name, '%03d' % k)
        os.makedirs(d, exist_ok=True)
        gi = self.glob['info']
        with open(os.path.join(d, 'meta.txt'), 'w') as fp:
            fp.write('t %.4f\n' % t)
            fp.write('local %d %d %.6f %.5f %.5f %s\n' % (
                m.info.width, m.info.height, m.info.resolution, m.info.origin.position.x,
                m.info.origin.position.y, m.header.frame_id))
            fp.write('global %d %d %.6f %.5f %.5f %s\n' % (
                gi.width, gi.height, gi.resolution, gi.origin.position.x, gi.origin.position.y,
                self.glob['frame']))
            fp.write('map_odom %.5f %.5f %.6f\n' % self.map_odom)
            if self.odom_base is not None:
                fp.write('odom_base %.5f %.5f %.6f\n' % self.odom_base)
            if self.carrot is not None:
                fp.write('carrot %.4f %.5f %.5f %.6f\n' % self.carrot)
            fp.write('plan %d\n' % self.plan_id)
        np.frombuffer(m.data, dtype=np.int8).tofile(os.path.join(d, 'local.bin'))
        self.glob['data'].tofile(os.path.join(d, 'global.bin'))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('windows', nargs='+', help='T0:T1[:name], epoch seconds')
    args = ap.parse_args()
    windows = parse_windows(args.windows)
    t_min = min(w[0] for w in windows) - 5.0  # TF / carrot history before a window
    t_max = max(w[1] for w in windows)
    dumper = Dumper(args.out, windows)
    t = 0.0
    for f in split_files(args.bag):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=f, storage_id='mcap'),
                    rosbag2_py.ConverterOptions('', ''))
        types = {tt.name: tt.type for tt in reader.get_all_topics_and_types()}
        cls = {name: get_message(types[name]) for name in WANT if name in types}
        while reader.has_next():
            topic, data, stamp = reader.read_next()
            if topic not in cls:  # set_filter is not honoured by every storage plugin
                continue
            t = stamp * 1e-9
            if t > t_max:
                break
            # Plans, the keepout mask and the global costmap are state that
            # outlives the window; the rest is only needed inside it.
            is_state = topic in PLAN_TOPICS or topic in (KEEPOUT, GLOBAL, GLOBAL_UPDATES)
            if not is_state and (t < t_min or (topic == LOCAL and dumper.window_of(t) is None)):
                continue
            m = deserialize_message(data, cls[topic])
            if topic in PLAN_TOPICS:
                dumper.on_plan(m, t)
            elif topic in CARROT_TOPICS:
                dumper.carrot = (t, m.pose.position.x, m.pose.position.y,
                                 yaw_of(m.pose.orientation))
            elif topic == KEEPOUT:
                dumper.on_keepout(m)
            elif topic == GLOBAL:
                dumper.on_global(m)
            elif topic == GLOBAL_UPDATES:
                dumper.on_global_update(m)
            elif topic == '/tf':
                dumper.on_tf(m)
            elif topic == LOCAL:
                dumper.on_local(m, t)
        if t > t_max:
            break
    print('samples per window:', dumper.counters)


if __name__ == '__main__':
    main()
