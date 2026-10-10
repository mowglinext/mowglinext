# Layered mower assembly

The shared renderer is used by Hardware, Sensors and the map's URDF appearance.
It is a scaled illustration, not CAD or collision geometry.

- Configure the shell once in Hardware. Sensors and Map inherit the selection.
- Four shells: Rounded, Sculpted, Utility, Yardforce-inspired.
- Style and transparency are display preferences saved in this browser.
- All top views face up; all side views face left. The separate red stop button
  is always at the rear, regardless of shell style or transparency.
- Transparent mode changes the opacity of the same shell art; it reveals the
  independent cutting-disc artwork/IMU. It never selects alternate artwork or different sizing.

## Geometry and anchors

Everything is expressed in metres in base_link: +X forward, +Y left, +Z up.
Top projection is (-Y,-X); side projection is (-X,-Z). The map anchors base_link
at the rear axle, not the shell centre. It projects the complete assembly plane
through the map camera, including bearing, heading, pitch and close-zoom perspective.

The parser reads visual sizes and origins from the processed robot description.
Settings previews overlay the edited API values (which already include model/
backend defaults). Chassis length, width, height and centre can therefore change
without stretching the wheels or sensors. Automatic caster/blade positions follow
mowgli.urdf.xacro. Hardware also exposes caster_x_offset (metres forward from
base_link; -1 preserves automatic placement). Explicit placement flows through
schema, settings, launch and xacro to both caster axles. It does not move other
sensors or change wheel control. Map URDF mode uses only the running description, never drafts.
A changed XML description is accepted; unsupported or malformed geometry clears
the assembled marker and allows the existing map fallback.

Supported layout is the standard Mowgli URDF: axis-aligned chassis box and named
wheel/caster/blade cylinders with direct base_link joints. Sensors use named
GPS/LiDAR/IMU box/cylinder visuals and direct joints. Other URDF layouts use the
existing map fallback rather than claiming guessed geometry is authoritative.

Source atlas bounds exclude transparent padding. A browser regression scans the
actual alpha pixels of all eight shell projections (outside the configured crop
as well) and requires their solid edges to match the crop exactly. The crop maps
to chassis width/length in top view and length/height in side view. Viewport
padding provides display space only; it cannot alter those physical extents. Caster artwork has a separate
tyre reference rectangle so its fork does not change the diameter or axle anchor.
Sensor art selects a visible local face and projects mounting roll/pitch/yaw.
The side of a vertical cylindrical LiDAR is yaw-invariant.

## Assets and maintenance

Shell atlases are in public/assets/robots/layered and registered in shellAssets.ts.
Component atlases (drive-wheel, caster, blade, gps, lidar, imu, dock) use partAssets.ts.
Each atlas contains top view on the left and side elevation on the right.
The stop button remains a small vector layer. The disc is a generated top/side
atlas; its cutting envelope, not the asymmetric image crop, sets its radius.

All images were generated with the built-in ImageGen tool. No source photos,
logos or brand text are shipped. The Yardforce shape was guided by the three
user-provided Classic 500/500B reference photos, then recoloured graphite/green.

Generation specification:
- Transparent alpha, complete isolated parts, no floor or external shadow.
- Matched orthographic top and side elevations, graphite grey and restrained
  mint-green highlights, no labels or branding.
- Shells contain no wheels, sensors or stop button; rear controls are closed panels.
- Drive wheel: tread from overhead, closed modern polymer hub in side elevation.
- Caster: modern low-profile polymer fork; true overhead pivot cap, tyre and fork.
- GPS: low rounded square graphite puck; LiDAR: short graphite cylinder;
  IMU: small green PCB with chip and mounting holes.
- Dock: curved existing generic-station silhouette; head at top/left, horizontal
  contact pins towards the parking tray. Nominal illustrative footprint 67 × 46 cm,
  height 15 cm; not a physical URDF/collision object. The saved dock pose anchors
  the parked rear axle, 10 cm ahead of the entry. Select Styled dock in the map
  appearance menu; existing photo/marker selections remain available.
- Sensors have a thin mint silhouette highlight in the placement editor only.
- For replacements, remeasure visible pixel bounds and tyre reference bounds;
  do not compensate padding by changing the physical URDF dimensions.

## Verification

Unit tests cover parser rejection, edited vs live geometry, sensor rotations,
scale independence, identical solid/transparent assets and map projection.
Mocked Playwright checks cover all four styles, Hardware-only selection, mobile
navigation, sensor editing/dragging and actual Mapbox projection at multiple
zooms/headings/bearings/pitches. Screenshots are in screenshots.local/layered-mower.

These checks use the processed URDF fixture matching current template geometry
on upstream dev d4924a4b; no mower was moved, flashed or deployed for this work.

Latest artwork corrections (built-in ImageGen edits/generations):
- Caster: narrow true overhead tread and small pivot cap; rolling direction +X.
  The top sprite is rotated 180 degrees to put the pivot ahead of the tyre axle,
  matching the side view. Its tyre reference maintains URDF diameter and width.
- Rear wheel: closed graphite aero hub, five recessed panels, subtle mint arc.
- Blade: round graphite disc, three silver pivot blades, separate side elevation.
- Dock: retained reference proportions after reviewing shorter/longer variants;
  head rotated to top in plan and a horizontal contact pin in side elevation.
  Atlas clips omit external padding. No source bitmap is edited by Python.

ROS validation: non-root Docker xacro tests exercise automatic, positive, zero
and negative caster positions. GUI schema/template/default-type parity tests
run in a non-root Go container. This feature has not been deployed to .118.

## Chassis vertical placement

`chassis_z_offset` places the bottom of the shell relative to the rear axle.
The YardForce 500/500B starting value is **-0.050 m**, estimated from the supplied
Classic 500/500B photos, not measured. With the configured 0.100 m wheel radius
and 0.190 m body height, this puts the bottom 0.050 m and top 0.240 m above ground.
The value is editable in Hardware → Chassis & Geometry, mirrored in the schema
and passed through launch to both URDF body visual/collision origins. It does
not move base_link, wheels, casters, blade or sensor frames and does not change
the 2D navigation footprint. Other populated mower presets explicitly retain
zero offset; custom/sparse configurations inherit the template unless overridden.

The live map uses the running URDF. Hardware/Sensors previews show edited values;
save and restart ROS before the running model reflects a changed offset.
Changing body height preserves the configured bottom position.

Physical validation: **HARDWARE_REQUIRED** for the photo-estimated value.
Baseline: codex/layered-mower with chassis_z_offset=-0.050, YardForce500/B preset,
wheel_radius=0.100, chassis_height=0.190. No firmware or robot was accessed.
Procedure: on the intended Yardforce unit, powered off on a level surface with
blades removed, measure ground to the lowest body skirt and highest shell point.
Compare against 50 mm and 240 mm; a discrepancy over 5 mm rejects this provisional
fit and calls for adjusting the offset and/or body height to the measurements.
These measurements are not a prerequisite to using this illustrative preview.
