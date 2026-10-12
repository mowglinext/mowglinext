# LiDAR ignore lines

Some gardens have a **hedge, a row of ornamental grasses or a similar soft plant edge exactly on the recorded boundary**. The operator drove the perimeter along it on purpose, so the outermost mowing pass rides on the recorded line. The LiDAR, however, sees that vegetation as a hard obstacle: the mower steers away from the boundary, cuts corners, reverses, or gets stuck in the obstacle-recovery loop next to the plant edge.

A **LiDAR ignore line** tells the mower: *along this line, do not treat what the LiDAR sees as an obstacle.* Everywhere else the LiDAR keeps working normally.

> **Safety trade-off — read this first.** A drawn line splits into two very different rules, and BOTH drop the LiDAR returns for **both** the obstacle costmap (what FTC and Nav2 use to steer around things) **and** `collision_monitor` (the near-field safety stop):
> - **On the side that overlaps a mowing or navigation area** ("the lawn side"), returns are dropped up to the **set distance** from the line.
> - **On the far side** (outside every recorded area — e.g. into the hedge itself), returns are dropped **along the whole length of the line, at any distance** — no distance limit at all, because the mower can never physically be there anyway.
>
> Nothing in software will stop the mower for an obstacle within that reach. The accuracy of the line (and, on the lawn side, its distance) are the only thing standing between the mower and whatever is really there. Only draw a line where you know the only thing on either side is the plant edge — the far side has no distance cap to fall back on.

## When to use it

Use it when the mower behaves badly next to a boundary-side plant edge:

- it steers sideways away from the boundary line by 0.3-0.6 m (`FTCController: entering AVOIDANCE (lattice, peak offset ...)` in the log),
- it cuts a corner (`TURN FALLBACK ... skipping N m of path`),
- it stalls with `detected collision ahead!` / `Controller patience exceeded`, or a DETOUR is started and the same unit is retried over and over.

Do **not** use it for a real obstacle you want to mow around (a tree, a pot, a fence post): draw that as an obstacle in the map instead.

## How to draw one

1. Open the **Map** page and switch to **edit mode**. The ignore-line panel is read-only outside edit mode.
2. In the **LiDAR ignore lines** panel, press **Draw ignore line** and click points on the map along the plant edge. Finish with **Finish line** (Escape cancels).
3. Set the **distance** in the panel (in cm). This is how close the mower can get to the line, measured on the side that overlaps a recorded (mowing or navigation) area. The far side of the line (e.g. into the hedge itself) has no distance setting at all — it is always ignored along the whole line, since the mower is never physically there.
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

### 2. Make the distance wider rather than narrower

The default is 40 cm, and — since a September 2026 change — that is the *full* distance on the area side, not a band split in half. Even so, the foliage of a hedge or grass clump often reaches further than that: in our garden the avoidance offsets were 0.5-0.6 m. A 40 cm line (now the default) helped along a straight stretch but not at a corner.

- Start at **80-100 cm** (the maximum is 120 cm). If the mower still avoids the plant edge, the line is either too narrow or does not cover that spot.
- A wider distance only affects the area side; it does not blind the LiDAR elsewhere. The far side is already unconditional regardless of this setting — widening it never changes far-side behaviour.

### 3. The line does not clean up what was already seen

The filter stops **new** returns from becoming obstacles. It does not erase costmap cells that were marked earlier. After changing a line, restart the mowing run (or clear the costmaps) before judging the result.

### 4. Check that the line really sits on the plant edge

The map, the recorded boundary and the mower position are RTK-accurate. If the mower stands next to the plant edge in reality, it stands there on the map too. Draw the line where the plant edge is, not where you would like the mower to go. Use the satellite view and zoom in.

## How to verify it works (logs)

While the mower has a fresh position, `costmap_scan_filter` logs every 5 seconds:

```
corridor filter: robot (1.12, -13.26) is 0.42 m from the nearest line, 11 beam(s) suppressed this scan (8 area-side within reach, 3 beyond the recorded boundary)
```

- **Distance to the nearest line is large (metres)** at the spot where it misbehaves → the line does not cover that spot. Extend it.
- **Distance is small but `0 beam(s) suppressed`** → the line doesn't actually reach this point on either side: too narrow for the area-side gap, and this exact point doesn't fall alongside the drawn segment's own span on the far side either (e.g. it's past one of the line's endpoints). Extend or reposition the line.
- Split the suppressed count into its two parts to tell WHICH rule is (or isn't) doing the work:
  - **Area-side count stays low/zero while you're clearly on the lawn near the line** → the *distance* setting is too small, or the line doesn't run where the recorded area boundary actually is at that spot (check the two against each other on the map).
  - **Beyond-the-boundary count stays zero while you're clearly past the hedge** → the point doesn't project onto the drawn segment's span (you're past one of its endpoints) — extend the line further along the boundary/around the corner.
- **Beams are suppressed and the mower still avoids** → check the `FTCController: lattice profile ... peak offset` lines: if the offsets disappear along the line, it works. If they don't, the offending returns are landing on the AREA side, beyond the set distance — widen the distance, since the far side has no limit to widen.
- `corridor filter idle: last fused pose ... old` → the position was stale, so the filter deliberately passed everything through (fail-safe).

## Technical notes

- Implemented in `costmap_scan_filter_node` (`mowgli_localization`) as a third filter stage after the dock blank and before the collision-scan publish. It projects every beam into the map frame with the fused pose (`/odometry/filtered_map`) and the LiDAR mount offset, then applies ONE of two rules depending ONLY on the true recorded-area polygon (`/mowgli/recorded_area_polygons`, republished by `map_server_node` on every area-list change) — never on which side of the drawn LINE the point falls on, so a sloppily-drawn line can never blind a beam that's genuinely still inside a recorded area: **inside** a recorded (working or navigation) area, within `width_m` of the line (honoured in full, not halved); **outside** every recorded area, alongside the line's own span at ANY distance (no width limit — nothing out there is ever reachable by the mower, so a limit would protect nothing).
- Lines are `mowgli_interfaces/LidarIgnoreCorridor` (polyline + `width_m`), managed by `map_server_node` (`~/add_lidar_ignore_corridor`, `~/get_lidar_ignore_corridors`, `~/clear_lidar_ignore_corridors`) and published transient_local on `/mowgli/lidar_ignore_corridors`. `width_m` is clamped server-side to 0.05-1.2 m.
- With no lines drawn, or no recorded area yet, the filter is a no-op.
- The map's Map page also draws the actual ignored band as a translucent fill next to the thin line (in both view and edit mode), so it's visible at a glance how far a line's distance reaches — compare it against the satellite imagery to see whether it actually covers the plant edge.
