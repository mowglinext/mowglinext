# LiDAR ignore lines

Some gardens have a **hedge, a row of ornamental grasses or a similar soft plant edge exactly on the recorded boundary**. The operator drove the perimeter along it on purpose, so the outermost mowing pass rides on the recorded line. The LiDAR, however, sees that vegetation as a hard obstacle: the mower steers away from the boundary, cuts corners, reverses, or gets stuck in the obstacle-recovery loop next to the plant edge.

A **LiDAR ignore line** tells the mower: *along this line, do not treat what the LiDAR sees as an obstacle.* Everywhere else the LiDAR keeps working normally.

> **Safety trade-off — read this first.** Inside an ignore line the LiDAR returns are dropped for **both** the obstacle costmap (what FTC and Nav2 use to steer around things) **and** `collision_monitor` (the near-field safety stop). Nothing in software will stop the mower for an obstacle inside the line. The accuracy of the line and its width are the only thing standing between the mower and whatever is really there. Only draw a line where you know the only thing inside it is the plant edge.

## When to use it

Use it when the mower behaves badly next to a boundary-side plant edge:

- it steers sideways away from the boundary line by 0.3-0.6 m (`FTCController: entering AVOIDANCE (lattice, peak offset ...)` in the log),
- it cuts a corner (`TURN FALLBACK ... skipping N m of path`),
- it stalls with `detected collision ahead!` / `Controller patience exceeded`, or a DETOUR is started and the same unit is retried over and over.

Do **not** use it for a real obstacle you want to mow around (a tree, a pot, a fence post): draw that as an obstacle in the map instead.

## How to draw one

1. Open the **Map** page and switch to **edit mode**. The ignore-line panel is read-only outside edit mode.
2. In the **LiDAR ignore lines** panel, press **Draw ignore line** and click points on the map along the plant edge. Finish with **Finish line** (Escape cancels).
3. Set the **width** in the panel (in cm). This is the *total* width, half on each side of the line.
4. To reshape a line, select it on the map and double-click it. Then, like an area: drag a point to move it, drag the small midpoint of a segment to add a point (this gives you gentle bends), select a point and press Delete to remove it. Changes are saved as soon as you finish the edit.
5. **Make curved** rounds the selected line through its points, **Simplify** thins it out again.

Lines are stored on the robot next to the map (`areas.dat`), survive a restart and a map save, and follow the map if the datum is moved. They are removed with the trash button in the panel, not by clearing the map.

## What we learned in the field — how to make it actually work

The first tests looked as if the feature did nothing. Every time the cause was the line, not the filter:

### 1. Cover the whole stretch, and the corners

The line only helps **where it is**. The mower does not need the line at the spot where it finally gets stuck; it needs it along the entire stretch where it reacts, including the run-up to it. In our test the line ended right at a corner, and the mower kept getting stuck exactly there because the plant edge simply continues around the corner.

- Take the line **around every corner** of the plant edge and **1-2 m beyond** it along the next edge.
- At a corner, put an extra point on the corner itself so the line follows the bend instead of cutting across it.
- Corners are where it matters most: the mower body overhangs the recorded line most in a turn (roughly half a metre in front of the axle), so a corner reads as blocked long before the mower reaches it.

### 2. Make the width wider rather than narrower

The default of 20 cm total only removes LiDAR returns within 10 cm of the line. The foliage of a hedge or grass clump reaches much further than that: in our garden the avoidance offsets were 0.5-0.6 m. A 40 cm line helped along a straight stretch but not at a corner.

- Start at **80-100 cm total** (the maximum is 100 cm). If the mower still avoids the plant edge, the line is either too narrow or does not cover that spot.
- A wider band only affects what is *near the line*; it does not blind the LiDAR elsewhere.

### 3. The line does not clean up what was already seen

The filter stops **new** returns from becoming obstacles. It does not erase costmap cells that were marked earlier. After changing a line, restart the mowing run (or clear the costmaps) before judging the result.

### 4. Check that the line really sits on the plant edge

The map, the recorded boundary and the mower position are RTK-accurate. If the mower stands next to the plant edge in reality, it stands there on the map too. Draw the line where the plant edge is, not where you would like the mower to go. Use the satellite view and zoom in.

## How to verify it works (logs)

While the mower has a fresh position, `costmap_scan_filter` logs every 5 seconds:

```
corridor filter: robot (1.12, -13.26) is 0.42 m from the nearest line, 11 beam(s) suppressed this scan
```

- **Distance to the nearest line is large (metres)** at the spot where it misbehaves → the line does not cover that spot. Extend it.
- **Distance is small but `0 beam(s) suppressed`** → the line is on the wrong side or too narrow for where the plant edge actually is.
- **Beams are suppressed and the mower still avoids** → widen the line, and check the `FTCController: lattice profile ... peak offset` lines: if the offsets disappear along the line, it works.
- `corridor filter idle: last fused pose ... old` → the position was stale, so the filter deliberately passed everything through (fail-safe).

## Technical notes

- Implemented in `costmap_scan_filter_node` (`mowgli_localization`) as a third filter stage after the dock blank and before the collision-scan publish. It projects every beam into the map frame with the fused pose (`/odometry/filtered_map`) and the LiDAR mount offset, and sets ranges within `width/2` of any line to +inf.
- Lines are `mowgli_interfaces/LidarIgnoreCorridor` (polyline + `width_m`), managed by `map_server_node` (`~/add_lidar_ignore_corridor`, `~/get_lidar_ignore_corridors`, `~/clear_lidar_ignore_corridors`) and published transient_local on `/mowgli/lidar_ignore_corridors`. `width_m` is clamped server-side to 0.05-1.0 m.
- With no lines drawn the filter is a no-op.
