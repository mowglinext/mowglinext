# Biltema RM1000: measured geometry and conservative GUI preset

Status: **HARDWARE_REQUIRED** for runtime acceptance. These are starting
geometry values for **one** Biltema RM1000, not verified dimensions for every
RM1000 revision. Physical measurements were reported by an owner on
**2026-10-25**, with a tape measure on a stationary mower on paving. The
unit's serial number, tyre variant, installed ROS/firmware revisions and
physical validation baseline were not recorded. No autonomous drive or
mowing test was performed.

This document explains the `BiltemaRM1000` preset in
`gui/web/src/constants/mowerModels.ts`. Units below are millimetres unless
otherwise stated. `base_link` is at the midpoint of the rear drive axle:
+X forwards, +Y left, +Z up.

## Reported measurements (preserve these independently of the preset)

| Dimension / reference | Owner report | Uncertainty |
| --- | ---: | --- |
| Rear-wheel outside diameter excluding tread lugs | 180 | Base tread diameter |
| Rear-wheel outside diameter including protruding lugs | 200 | Lug engagement and compression vary by terrain |
| Rear-wheel centre-to-centre track | 325 | Direct tape measurement |
| Rear tyre width along axle | 36 | Direct tape measurement |
| Front caster outside diameter | ~85 | Tapered/rounded tyre; hard to measure |
| Front caster centre-to-centre track | ~215 | Caster centres change as forks swivel |
| Rear drive axle → front caster wheel axle (+X) | ~295 | Casters aimed approximately forwards |
| Rear axle → foremost bumper (+X) | 403 | Direct tape measurement |
| Rear axle → rearmost bumper (-X) | 145 | Direct tape measurement |
| Body length from the two measured edges | 548 | 403 + 145 |
| Body centre ahead of rear axle | 129 | (403 - 145) / 2 |
| Max body width including bumper | ~400 | Direct tape measurement |
| Shell height (excluding cutting-height knob) | ~215 | Explicitly described as difficult / uncertain |
| Ground → body bottom on paving | ~37 | Changes with lug contact |
| Rear axle → blade motor rotation centre (+X) | ~190 | Direct tape measurement |
| Blade centre lateral offset | ~0 | Visually centred, not instrument-measured |

The 180 mm **cutting width** is from Biltema's published specification, not
a physical knife sweep measurement. No independent encoder or motor calibration
was performed.

## External reference data

- [Biltema RM1000 product specification](https://www.biltema.dk/fritid/haven/havemaskiner/planeklippere/robot-plaeneklippere/robotplaneklipper-rm1000-2000043815): nominal 180 mm cutting width; 8.5 kg **total mower** mass.
  The RM1000 product article is 17-119.
- [Forbrugerrådet Tænk, Biltema RM1000](https://taenk.dk/test/robotplaeneklipper/biltema-rm1000):
  listed mower envelope **570 × 395 × 265 mm** (L × W × total H);
  charging dock **630 × 460 × 230 mm**.
- [Yard Force SA650ECO manual](https://www.libble.eu/yardforce-sa650-eco/online-manual-929220/?page=0004):
  listed mower envelope **570 × 390 × 260 mm** and 180 mm cut.
  Similar dimensions do **not** prove interchangeable wheels or axle positions.
- `docs/WHEEL_PRESET_DIMENSIONS.md`: an SA650 owner reported 190 mm rear
  and 90 mm caster diameters on 2026-10-11. The other model presets using
  those diameters are assumptions, not RM1000 measurements.

## Rationale for proposed preset (metres in code)

| Parameter | Preset | Rationale |
| --- | ---: | --- |
| `wheel_radius` | 0.100 | Half 200 mm outer diameter including lugs; conservative wheel shape, **not** a drive-distance calibration. Retain 0.090 m tread-base radius as an alternative. |
| `wheel_track` | 0.325 | Owner's 325 mm direct measurement. |
| `wheel_width` | 0.036 | Owner's 36 mm direct measurement. |
| `wheel_x_offset` | 0 | By the chosen rear-drive-axle `base_link` definition. |
| `caster_radius` | 0.045 | Rounds the uncertain ~85 mm caster diameter to a 90 mm outside envelope (the separate SA650 owner also reported 90 mm). **Provisional.** |
| `caster_track` | 0.215 | Owner's ~215 mm, caster wheels facing approximately forwards. **Provisional.** |
| `caster_x_offset` | 0.300 | Owner's ~295 mm rounded to nearest centimetre. **Provisional.** |
| `chassis_length` | 0.570 | Conservative published 570 mm overall envelope rather than 548 mm tape span. |
| `chassis_width` | 0.400 | Owner's 400 mm; slightly more conservative than Tænk's 395 mm. |
| `chassis_center_x` | 0.130 | Rounds the owner's derived +129 mm; **paired** with the 570 mm length. |
| `chassis_height` | 0.220 | Approximate shell-only height from uncertain ~215 mm; not the 265 mm total height. **Provisional.** |
| `chassis_z_offset` | -0.060 | Approximate ground clearance 37 mm minus assumed axle height 100 mm → -63 mm, rounded to -60 mm. **Provisional.** |
| `blade_x_offset` | 0.060 | Blade rotation centre 190 mm ahead of axle minus shell centre 130 mm. |
| `blade_y_offset` | 0.000 | Centreline visually confirmed, low precision. |
| `blade_radius`, `tool_width` | 0.090, 0.180 | 180 mm specified cut width, half-width radius. |
| `dock_visual_length`, `dock_visual_width` | 0.630, 0.460 | Tænk-listed dock dimensions, **illustration only**; not docking calibration. |

This yields a model envelope with **front X = +415 mm** and **rear X =
-155 mm**, covering the tape-reported +403/-145 mm edges by ~12/10 mm.
The rear tyres span 325 + 36 = 361 mm, inside the 400 mm body width.
The fore-aft distance from the model front caster tyre to front bumper is
415 - 300 - 45 = 70 mm. The blade rotation centre remains exactly
130 + 60 = **190 mm ahead of the rear axle**.

At a nominal wheel radius of 100 mm, the modeled body bottom is
100 - 60 = **40 mm above ground** (owner ~37 mm), and body top is
40 + 220 = **260 mm above ground** (published total height 265 mm may
include the excluded cutting-height knob). These are geometry **estimates**,
not a measurement of the actual stationary rear-axle height. Lugs may
penetrate soft soil, so the effective rolling radius can differ.

## Configuration scope and upgrade behavior

The new dimensions are proposed when an operator **explicitly applies**
the Biltema RM1000 model preset. **Existing saved values are not
overwritten automatically**, and selecting another model requires the
existing GUI confirmation.

One exception matters: `chassis_z_offset` is a preset-backed **upgrade
geometry default**. Its generated copies in
`gui/asserts/model_geometry_defaults.json` and
`ros2/src/mowgli_bringup/config/model_geometry_defaults.json`
now supply -0.060 m for existing RM1000 configurations **only when that
key is missing**. An explicitly saved value retains precedence. This
changes the modeled shell vertical placement; it does not move the
wheels, blade, sensor frames, or 2D navigation footprint.

Deliberately **not** set: `ticks_per_meter`, motor PID, battery thresholds,
GPS/LiDAR/IMU positions, `chassis_mass_kg`, docking pose and firmware.
The published 8.5 kg is **total robot mass**, whereas the GUI's
`chassis_mass_kg` excludes separately modeled wheels/casters/blade assembly;
therefore 8.5 must not be inserted as that parameter by inference.

## Acceptance on real hardware — HARDWARE_REQUIRED

Before using this shared preset for autonomous operation:

1. Record robot model/serial, tyres/lug style, commit SHA, installed ROS
   image and firmware revisions; power off and immobilise mower, remove blades,
   and support it securely for underside inspection.
2. Confirm outer wheel diameters (tread base and lugs), wheel centre track,
   front-caster wheel-axle position with both casters facing forwards,
   and both body extents relative to the rear axle. If measured body extends
   beyond X +415 / -155 mm or +/-200 mm laterally, enlarge/correct the
   footprint before using navigation.
3. Check ground-to-axle, ground-to-body-bottom and ground-to-body-top
   on a flat hard surface. If the modeled 40/260 mm body bottom/top differ
   materially, adjust `chassis_z_offset` / `chassis_height` and verify
   `base_link` height rather than assuming 100 mm is the effective radius.
4. With blade removed and actuation inhibited, confirm generated URDF wheel,
   caster, blade, chassis collision envelope and Nav2 footprint against the
   robot. Re-verify sensor heights after editing wheel radius.
5. Calibrate **encoder ticks/metre** from a supervised, blade-disabled travel
   test separately. Do not infer it from the wheel radius or copy SA650 values.
   Any motorised field acceptance must have a clear area and accessible stop.

This report documents one unit's manual measurements; it **does not**
claim hardware field acceptance or universal factory dimensions.
