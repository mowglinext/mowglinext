# fusion_graph replay harness

Replays a recorded bag through `fusion_graph_node` inside the ROS2 image, on a
laptop, and scores the result against dead reckoning and GPS. Built to
reproduce the 2026-09-06 LiDAR-anchor failure (10 m off during a GPS outage)
without a robot; it does, in ~10 minutes.

```
# once: pull the image, copy the bag + these scripts into a scratch dir
docker run --rm --network none \
  -e TAG=run1 -e ANCHOR=true -e RATE=1 \
  -e PLAY_EXTRA_TOPICS=/fusion_graph/lidar_map \
  -e EXTRA="-p lidar_map_import_topic:=/fusion_graph/lidar_map" \
  -v "$PWD:/data" ghcr.io/mowglinext/mowglinext/mowgli-ros2:dev bash /data/replay.sh
```

* `replay.sh` — orchestrates: `ros2 bag play --clock` first (so the node
  bootstraps at bag time), `seed_pose.py` (latched `set_pose` = the bag's first
  fused pose; without it the node seeds at the dock with a wrong yaw and dead
  reckoning drifts metres), the node under test, `replay_log.py`, then
  `replay_eval.py`. Env: `TAG` (output dir under `/data/replay/`), `ANCHOR`,
  `EXTRA` (extra `-p` node params), `OVERLAY` (a local colcon install to
  source, e.g. `/data/localbuild/install/setup.bash`), `PLAY_EXTRA_TOPICS`,
  `RATE`. Edit the datum / lever / dock params at the top for another site.
* `replay_log.py` — 2 Hz CSV on sim time: fused pose, map→odom, odom→base,
  GPS projected about the datum, RTK status, anchor diagnostics, anchor
  candidate + verdict.
* `replay_eval.py` — max ‖fused − dead reckoning‖ during the GPS outage
  (`gps_age > 3 s`), fused and DR-only error vs GPS at RTK return, time to
  recover below 0.15 m, pre-outage fused-vs-GPS.
* `seed_pose.py` — see above.

**Import the robot's own map.** The bag's latched `/fusion_graph/lidar_map` is
the map the robot had; replay it and set `lidar_map_import_topic` so the node
localises against the same clutter. The 2026-09-06 failure does NOT reproduce
without it.

**Local build without CI:** mount `ros2/src/fusion_graph` + `mowgli_interfaces`
into the image and `colcon build --packages-select fusion_graph` (~3 min on an
M-series Mac); pass the install as `OVERLAY`. Run replays with
`--network none` so concurrent replays cannot see each other's DDS traffic.
Never `set -u` around ROS `setup.bash`.

Bag topics to record next time: `/scan /wheel_odom /imu/data /imu/cog_heading
/gps/fix /gps/status /hardware_bridge/status /fusion_graph/lidar_map
/fusion_graph/diagnostics /odometry/filtered_map /tf /tf_static`.
