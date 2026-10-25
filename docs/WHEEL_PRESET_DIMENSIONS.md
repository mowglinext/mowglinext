# Mower dimensions and preset corrections

## Values and evidence

All configuration values below are **radii in metres**. On 2026-10-11 the
contributor reported SA650 rear-wheel diameter **190 mm** and caster diameter
**90 mm**. Dividing by two and converting millimetres to metres gives **0.095 m**
and **0.045 m**. The robot serial, tyre variant and measurement method were not
provided. These values must not be described as manufacturer specifications.

The contributor explicitly requested applying the same dimensions to the other
three presets as assumptions, replacing their implausible legacy rear-wheel size.
Those models have not been measured; shared values do not establish shared parts.

| GUI model | Old rear diameter | New rear diameter | Old caster diameter | New caster diameter | Basis |
| --- | ---: | ---: | ---: | ---: | --- |
| YardForce SA650 | 89.5 mm | 190 mm | 60 mm | 90 mm | Contributor report, 2026-10-11 |
| YardForce 900 ECO | 89.5 mm | 190 mm | 60 mm | 90 mm | Provisional SA650-based assumption |
| LUV1000RI | 89.5 mm | 190 mm | 60 mm | 90 mm | Provisional SA650-based assumption |
| Sabo MOWiT 500F | 89.5 mm | 190 mm | 70 mm | 90 mm | Provisional SA650-based assumption |

The measured YardForce 500/500B defaults remain unchanged. RM1000 and Custom
were unchanged by this 2026-10-11 wheel correction. The later RM1000
owner-measurement preset is documented separately in [RM1000_GEOMETRY.md](RM1000_GEOMETRY.md).
The previous rear value came from the same 0.04475 m radius
copied across unrelated machines, as documented by commit `c0aa94fa`.

## Application and geometry impact

This is a **safety-critical geometry change**, not just artwork sizing.
Hardware and onboarding share `gui/web/src/constants/mowerModels.ts`; choosing
one of these presets proposes the new radii through the existing settings flow.
Existing saved files are not automatically migrated. An existing SA650 owner can
edit Wheel Radius and Caster Radius directly, avoiding reapplying unrelated model
preset values. Saving and restarting ROS applies the URDF change.

At upstream baseline `fd244fecace467d0046b28012bc29d5a4099220f`,
`mowgli.urdf.xacro` uses the rear radius for base_link's height above ground, drive
wheel visuals/collisions/inertia, and derived caster/blade transforms. Increasing
rear radius from 44.75 mm to 95 mm raises base_link by **50.25 mm**, also shifting
sensors mounted relative to it. Caster radius changes caster geometry and its
axle-height relationship. These are model changes, not measurements of an installed
sensor height. No wheel track, caster spacing, chassis dimensions, sensor mount,
encoder ticks-per-metre or firmware calibration is changed by this patch.

## Physical acceptance: HARDWARE_REQUIRED

Software checks do not validate these assumed dimensions. Before accepting each
preset for a physical mower, record:

- Exact model, serial/unit, wheel and tyre variant, wear and any spike attachments.
- The PR commit, installed GUI/ROS image IDs, URDF/config revision, firmware image
  and submodule revisions. No physical test has been run on any such baseline here.
- With power off, blades removed, and the mower stable with wheels immobilised,
  measure the rear and caster tyre outside diameters through each axle centre.
  Measure both sides; do not measure a hub cap, rim or image padding.
- Pass the proposed dimensions only if rear diameter is 190 mm and caster diameter
  is 90 mm within 5 mm on that exact setup. Otherwise reject the assumption, use
  measured radius overrides and report that model/variant for a preset correction.
- With motors inhibited, check the generated base_link-to-ground transform and
  wheel/caster positions against those measurements before any drive test. Recheck
  actual sensor heights after applying the model change.

The radius corrections are independent; the WORX preset below depends on #966.


## WORX 500M Plus preset

Added on top of #966, which provides configurable blade offsets and the paired WORX shell/dock.
Source: contributor configuration screenshot and coordinate clarification, 2026-10-11.
These are reported configuration values for their mower, **not manufacturer specifications**.
The robot serial, controller conversion, firmware, tyre variant and measuring method were not supplied.

| Setting | Preset value | Source |
| --- | ---: | --- |
| Rear-wheel radius / width / centre track | 100 / 40 / 360 mm | Supplied screenshot |
| Rear-wheel X offset | −5 mm | Supplied screenshot |
| Caster radius / track | 30 / 0 mm | Supplied screenshot; central caster |
| Caster X position | 405 mm | Provisional: places the tyre 50 mm inside the front edge, consistent with the other built-in presets; not measured |
| Chassis length / width | 580 / 395 mm | Supplied screenshot |
| Chassis shell height / bottom Z | 140 / −70 mm | Provisional shallow envelope from supplied photos and the earlier approved WORX preview; replaces the screenshot's unverified 230 mm height |
| Chassis centre X | 195 mm | Supplied screenshot |
| Mass | 8.76 kg | Screenshot value; not independently weighed |
| Blade radius / tool width | 100 / 200 mm | Supplied screenshot |
| Encoder ticks per metre | 343 | Screenshot calibration; must be checked for the installed encoder/controller |
| Blade centre from base_link | X = 195 mm, Y = −57.5 mm | Contributor explicitly confirmed base_link, right of centre |
| Blade offsets from chassis centre | X = 0, Y = −57.5 mm | Conversion to #966 offset settings |

With `wheel_x_offset = -0.005`, the blade is 200 mm ahead of the rear-wheel axle;
195 mm is the distance from **base_link**, not the axle. Positive Y is left, so right
uses a negative value. Four decimal places preserve `-0.0575` exactly.

The preset deliberately does not replace battery voltages, sensor mounts,
or firmware selection. They were not established by the supplied data;
review those retained/defaulted values and the provisional caster X on the installed conversion. Selecting a body preset does not
establish stock WORX mainboard compatibility. The matching appearance is the registered WORX shell
and side-charging dock, without a separate artwork mapping.

Unlike the four radius-only corrections, **applying this new preset also proposes the screenshot's
encoder and chassis values**. No settings are automatically migrated or written without Save.

Physical acceptance remains **HARDWARE_REQUIRED**. Record the exact #970 commit, robot unit,
controller/encoder, firmware and ROS image. With power isolated and blades removed, measure wheel
radii and centre track, chassis dimensions and blade rotation centre; verify the blade URDF transform
is `(0.195, -0.0575)` relative to base_link and both caster links share Y = 0. Check sensor heights and
caster X on the actual robot. Then, blades removed and in a supervised clear test area with an
accessible cutoff, validate ticks-per-metre against a measured travel distance using the drive
calibration procedure before autonomous operation. The asymmetric cutting-tool clearance and
coverage require a supervised field test; these software tests do not establish that behaviour.

The shallow WORX default puts the shell bottom 30 mm above ground and its top 170 mm above ground with the 100 mm wheel radius. These are provisional model coordinates, not physical measurements. For acceptance on the exact unit/configuration/firmware baseline above, measure shell bottom/top relative to the rear axle with power isolated and blades removed; accept only if both agree within 5 mm, otherwise use measured overrides.
