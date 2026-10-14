# Layered mower assembly

The shared renderer is used by Hardware, Sensors and the map's URDF appearance.
It is a scaled illustration, not CAD or collision geometry.

- Configure the shell once in Hardware. Sensors and Map inherit the selection.
- Shell choices come from the registry: Rounded, Sculpted, Utility, Yardforce-inspired, RM1000-inspired, Worx-inspired and Tron-inspired.
- Shell style is saved in mower configuration; Hardware transparency stays browser-local.
- Sensors starts transparent each time it opens; its toggle affects only that
  editor session (both top and side). The map always remains opaque.
- Hardware and Sensors show a stationary docked illustration in top and side
  views. The shared nominal dock keeps the map anchor in top view; the side tray
  meets the tyre contact plane. Framing includes both without changing robot
  dimensions or sensor coordinates. The dock stays faint in both shell modes to keep
  components legible. This illustration does not report live dock status.
- All top views face up; all side views face left. There is no separate stop-button
  layer; future shell artwork can include a rear stop button if appropriate.
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
mowgli.urdf.xacro. Hardware also exposes a fixed caster_x_offset (metres forward from
base_link; legacy -1 values are resolved for display and materialized on editing). Explicit placement flows through
schema, settings, launch and xacro to both caster axles. It does not move other
sensors or change wheel control. Map URDF mode uses only the running description, never drafts.
A changed XML description is accepted; unsupported or malformed geometry clears
the assembled marker and allows the existing map fallback.

Supported layout is the standard Mowgli URDF: axis-aligned chassis box and named
wheel/caster/blade cylinders with direct base_link joints. Sensors use named
GPS/LiDAR/IMU box/cylinder visuals and direct joints. Other URDF layouts use the
existing map fallback rather than claiming guessed geometry is authoritative.

Source atlas bounds exclude transparent padding. A browser regression scans the
actual alpha pixels of all registered shell projections (outside the configured crop
as well) and requires their solid edges to match the crop exactly. The crop maps
to chassis width/length in top view and length/height in side view. Viewport
padding provides display space only; it cannot alter those physical extents. Caster artwork has a separate
tyre reference rectangle so its fork does not change the diameter or axle anchor.
Sensor art selects a visible local face and projects mounting roll/pitch/yaw.
The side of a vertical cylindrical LiDAR is yaw-invariant.

## Assets and maintenance

Shell atlases are in public/assets/robots/layered and registered in `appearanceRegistry.ts`.
Component atlases (drive-wheel, caster, blade, gps, lidar, imu, dock) use the same registry.
Each atlas contains top view on the left and side elevation on the right.
The disc is a generated top/side atlas; its cutting envelope, not the asymmetric image crop, sets its radius.

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
  contact pins towards the parking tray. Nominal illustrative footprint 67 Ãƒâ€” 46 cm,
  height 15 cm; not a physical URDF/collision object. The saved dock pose anchors
  the parked rear axle, 10 cm ahead of the entry. Hardware supplies the paired
  dock automatically; the map has no independent dock appearance selector.
- Sensors have a thin mint silhouette highlight in the placement editor only.
- For replacements, remeasure visible pixel bounds and tyre reference bounds;
  do not compensate padding by changing the physical URDF dimensions.

## Verification

Unit tests cover parser rejection, edited vs live geometry, sensor rotations,
scale independence, identical solid/transparent assets and map projection.
Mocked Playwright checks cover the registered shell styles, Hardware-only selection, mobile
navigation, sensor editing/dragging and actual Mapbox projection at multiple
zooms/headings/bearings/pitches. Screenshots are in screenshots.local/layered-mower.

These checks use the processed URDF fixture matching current template geometry
on upstream dev d4924a4b; those fixture checks do not validate a physical mower.

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
run in a non-root Go container. The GUI feature has also been exercised in a test deployment; that does not validate physical geometry.

## Chassis vertical placement

`chassis_z_offset` places the bottom of the shell relative to the rear axle.
The YardForce 500/500B starting value is **-0.050 m**, estimated from the supplied
Classic 500/500B photos, not measured. With the configured 0.100 m wheel radius
and 0.190 m body height, this puts the bottom 0.050 m and top 0.240 m above ground.
The value is editable in Hardware Ã¢â€ â€™ Chassis & Geometry, mirrored in the schema
and passed through launch to both URDF body visual/collision origins. It does
not move base_link, wheels, casters, blade or sensor frames and does not change
the 2D navigation footprint. Other populated mower presets explicitly retain
zero offset. A missing chassis offset inherits the selected preset on upgrade;
explicit saved values still win. CUSTOM/unknown models use the generic template.

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

## Preset review and mobile map budget (2026-10-10)

The Hardware preview follows the model picker; it no longer includes the dock.
Wheel layers precede the shell in both views, so the opaque body hides the upper
wheel while transparent mode reveals the same wheel at exactly the same axle.
Height is the physical visible shell height, excluding image margins. With body
height H and bottom offset O: axle-relative bottom = O, top = O + H, centre =
O + H/2. Ground clearance adds the drive wheel radius. The Yardforce preset uses
H=190 mm, O=-50 mm, radius=100 mm: bottom=50 mm, top=240 mm above ground.
Its rear track is 325 mm centre-to-centre (365 mm outer tyre width), inside the
450 mm body box. We do not alter drive geometry just to make an image look wider.
The automatic rear-to-front-caster axle distance is 450 mm for that preset.

The full application preset gallery actually selects YardForce500B, SA650,
LUV1000RI, Sabo, and RM1000 through Hardware controls, independently on desktop
and mobile. The legacy SA650/900ECO/LUV1000RI/Sabo presets still have 44.75 mm
wheel radius, an unverified inherited value documented in mowerModels.ts;
those very small rendered wheels faithfully expose that preset limitation.
RM1000 has no geometry defaults and retains existing values (Sabo in this
ordered gallery); it is NOT evidence for RM1000 physical dimensions.

### Compact map artwork

Only map markers use `public/assets/robots/layered/map/*-*.webp`; settings use
the original PNG atlases. Source crop/reference rectangles remain authoritative.
The derivative image is placed back into that exact pixel-coordinate rectangle,
so reduced resolution changes neither the physical chassis edges nor axle,
sensor, caster or blade anchors. Both opacity modes share the same derivatives.
Generate after any source image or crop-metadata change, from gui/web:

```sh
node --experimental-strip-types scripts/generate-map-artwork.mjs
```

This deterministic crop/resize/WebP pipeline uses the existing pinned Playwright
Chromium (install it with `npx playwright install chromium` if absent). Originals
are never overwritten. Body/dock longest edge: 384 px; blade: 256 px; wheels and
sensors: 128 px. WebP quality 0.85, preserved alpha. All 24 derivatives total
223,862 bytes. No runtime image processing or new runtime dependency is added.

Memoized assembly content is reused between poses, geometry bounds are memoized,
and identical map projection updates preserve state. No sensor glow is enabled
on the map. Opaque map mode omits concealed blade/IMU assets. Existing map display
budgets remain 20/10/5 Hz for Visual/Balanced/Efficient. URDF XML is only parsed
when the description changes, never for each pose.

Production Chromium check: 390x844 viewport, 4x CPU throttle, local plain Mapbox
basemap, mocked 20 Hz pose/status, Efficient mode, three-second sample. The
Yardforce plus dock initial artwork is **50,536 bytes**, down from 6,608,981 bytes
(99.2% less); 40 SVG descendants, zero artwork DOM mutations during movement.
Median and p95 animation frame interval were about 16.7 ms. These are desktop
browser emulation results, not a physical-phone or live satellite-map benchmark.
Screenshots/JSON in screenshots.local/layered-mower include the full app chrome;
all telemetry is fixture data and no mower was contacted. Full source imagery
is retained for the larger settings previews.

Verification: TypeScript/production build; 27 focused unit tests; 12 application
Playwright cases including the registered shell styles, five preset selections, mobile,
height/offset/track, sensors and map; map projection fixture checks physical
corners under zoom, heading, bearing and pitch. Map tests reject PNG requests,
images above 384 px, and initial artwork transfers above 150 kB. Regeneration
and hardware physical measurements remain distinct from this rendering evidence.

## Wheel enclosure audit (2026-10-10)

Rendering baseline: d94ea7f4, YardForce500/B preset, full and compact assets.
No robot, firmware, receiver or installed config was read for this audit.
Historical source comparisons:

- 5acb6f78 (initial ROS2, 2026-03-26): approximate 400 mm body width,
  350 mm centre track and 50 mm tyre width.
- cb134cb9 (2026-04-07): actual xacro arguments 400 mm body width,
  325 mm centre track, 40 mm tyre width. Its header says 400 mm track,
  but that is contradicted by the executable argument; not an old 400 mm track.
- 78a0a015 (2026-09-05): template/presets changed body width to 450 mm
  after a maintainer measurement on their Yardforce500. This does not establish
  dimensions on .118. Wheel track/width stayed 325/40 mm.
- Current template describes wheel_track as "from OpenMower"; the inspected
  history does not establish a physical Yardforce track/tyre-width measurement.

Source-alpha inspection at the drive axle (top atlas row 683, x=0 in robot
coordinates) gives a 551 px shell span within the 629 px body crop. At 450 mm
body width, that is about 394 mm of body at the axle versus 365 mm outer tyre
span. Thus the current opaque shell covers the wheel centres and outer edges
at that row, even with correct crop scaling and track/width arithmetic.

The supplied Yardforce overhead and rear-quarter photos visibly have narrower
rear bodywork and pronounced wheel recesses. Our generated top shell is too
full around that region, and its side panel lacks the real wheel opening.
This is a visual fidelity defect; correct crop bounds do not validate shell
contours. Correct the shell recesses while retaining the configured outer body
extent and axle anchors. Do not enlarge drive track merely to expose wheels.

HARDWARE_REQUIRED for selecting different physical track/tyre-width defaults.
Baseline to record at measurement: intended unit (.118 if applicable), installed
mower model/config and wheel type; renderer comparison remains d94ea7f4 with
325 mm track, 40 mm width, 450 mm maximum body width. On a level surface with
power off and blades removed, measure both tyre widths, their outer-to-outer
span and body width at the rear axle (also maximum body width). Calculate
centre track as outer span minus half the sum of the two tyre widths. Agreement
within 5 mm supports the current values; a larger discrepancy rejects them for
that unit and calls for a measured override. Perspective photos alone cannot
settle those absolute measurements. No parameter or artwork was changed by
this audit.


## Review regression fixes

Sensor drag and yaw handles keep separate 48 CSS-pixel hit targets when the
model is fitted to the editor, including narrow mobile layouts and large custom
chassis. A ResizeObserver converts those screen dimensions into SVG coordinates.
The regression test drags the LiDAR centre on a 1.2 m chassis without changing
yaw, then rotates it without changing position.

Sensor geometry retains its joint mounting point separately from its local
visual origin. Settings edit the mount/rotation; the preview recomputes the
rotated visual offset, matching the processed URDF used by the map. Unchanged
settings, combined mounting rotations and repeated previews have regression
coverage. Input fallbacks use joint coordinates rather than artwork centres.

Validation: 29 focused unit tests, 15 settings/map browser tests and three PR
gallery captures pass; TypeScript, focused lint (zero errors) and production
build pass. Sensor-editor screenshots refreshed after these changes.

## Hardware appearance pairs

Hardware presets may declare `appearance: {style: "yardforce", dockAppearance: "styled"}`
in `constants/mowerModels.ts`. Applying a confirmed hardware preset selects both
visual defaults; presets without a pair use the sculpted chassis and styled dock.
Appearance metadata is separate from numerical `defaults` and is never sent to
ROS. The 500/500B supply the Yardforce-inspired shell; RM1000 supplies its own
layered shell. They currently share the illustrative styled dock, whose shape
was already inspired by the RM1000 station. Neither choice measures a robot.

The `mower_visual_style` setting selects the shared shell/dock pair. Hardware
selection is a draft until Settings Save; Sensors use that settings draft and
Map reads saved configuration. A confirmed model change drafts the model's pair.
Empty or unknown styles follow the hardware model. Legacy localStorage shell and
dock fields and retired map appearance overrides cannot overwrite configuration.
Only transparency remains browser-local; Sensors starts transparent, Map is solid.
Saving appearance alone does not request a ROS restart or change physical geometry.

Future dock artwork belongs in `APPEARANCE_REGISTRY.parts` and `.docks`; each shell declares its paired dock ID. Photo docks use the existing calibrated MapImageMarker; a new
assembled dock uses declarative artwork metadata shared by the setup and map renderers. Old photo
assets remain as contributed source/reference material, not selectable mower
renderers. The obsolete static generic mower WebP is no longer shipped.

### RM1000 artwork provenance

`rm1000.png` was generated with the built-in imagegen tool from the contributed
RM1000 overhead photograph plus `sculpted.png` as a style reference. Its approved
top view is unchanged. The side now uses `rm1000-side.png`, generated from the
user-supplied RM1000 side photograph (clipboard ff774aba-79d3-44f9-95be-269d297f4027).
It follows the stepped front skirt, raised console and large rear wheel opening,
restyled in graphite/mint. Wheels and sensors remain separate layers; there is no separate stop-button layer.
This is a photo-informed illustration, not CAD or new physical measurements.

The exact generation and projection-correction prompts are in
`rm1000-artwork-prompt.md`. Both committed sources have a 1774 Ã— 887 canvas;
`SHELLS.files.side` selects the separate side source without modifying the top.
Alpha >220 bounds are [73,48,624,804] for top and [44,162,1702,622] for side.
Both solid and transparent modes use the same bounds. The compact map top WebP
is also unchanged; the side derivative is regenerated by the existing pipeline.
No RM1000 physical defaults have been invented; its geometry preset remains empty
and existing/custom dimensions drive the renderer.

### Rear console details

The separate SVG stop button and its per-shell anchors have been removed.
Future shell images can include the button directly so it follows the shell
silhouette and transparency without a separate floating overlay. This is artwork
only; emergency-stop controls and firmware safety are unchanged.
The RM1000 screenshot fixtures align the chassis wheel arch to the rear axle;
production wheel placement continues to come only from configured/running geometry.

## Worx-inspired shell and side-charging dock

The Worx-inspired shell uses a shallow front deck, rear battery housing and
transparent rear wheel cutouts. Top faces up; the left-side elevation faces left.
The charging station is on the robot's right, behind the body in the left-side
view. This is an appearance choice, not a new measured hardware preset: chassis
height/width/length and axle positions continue to follow the configured URDF.

`MOWER_STYLE_DOCKS` is derived from the registry, which pairs every shell with a dock appearance. Existing
shells share the original styled dock asset (no duplicated image files); Worx
selects the side-charging dock. Hardware, Sensors and map all use that pair.
The Worx station uses the editable dock footprint (including its right pedestal)
and a nominal 0.14 m illustration height; these are visual estimates, not collision
or charging calibration. In top projection its bounds extend further to the right
of the mower. In side view the same rear-axle anchor is retained and the tray
meets the tyre contact plane in setup previews.

Source atlases: `worx.png` and `dock-worx.png`. Their alpha bounds exclude image
margins. Compact top/side map WebPs are generated by the existing asset pipeline.
The shell was generated from the supplied top/side photographs, then corrected
for the two open rear wheel bays and graphite/mint palette. The dock was generated
from the supplied slatted station photographs. Source photos and branding are
not shipped. Earlier rejected shell variants are not included.

Dock setup previews use 70% opacity with a solid shell and 18% with a transparent shell; map docks remain opaque. Hardware exposes dock footprint length/width and Dock Center X/Y adjustments. They persist with GUI settings without a ROS restart and do not change calibrated dock poses, navigation, keepouts or URDF geometry. Artwork registration and additive adjustments are described below.

The Worx side projection uses `worx-side.png` with a round, open rear wheel arch; its approved top remains in `worx.png`. Wheels stay independently scaled and positioned by the configured robot geometry, so a shell choice never silently changes physical dimensions.

Worx side drive wheels are drawn outside the body panels, matching the supplied photo and exposing the full wheel face at any configured size. Other styles retain shell-over-wheel layering.

Dock Center X and Dock Center Y adjust the built-in artwork registration. Zero uses the paired dock alignment; positive X moves forward and positive Y moves left. Earlier test settings that stored a nominal centre need their adjustment reset to zero.

### Side projection registration

`Shell.frames` separates the physical chassis projection from the opaque artwork bounds. If a wheel opening removes the visible rear of a side shell, its frame still extends to the actual chassis rear derived from the top reference. Both full-resolution SVG and generated map WebP use this same frame; transparent mode never changes it. Per-view source sizes allow a separate side image without resizing or changing the approved top source.

### Worx left-side correction (2026-10-11)

Generated with the built-in image tool using the supplied real side photograph as the shape reference and the top photograph to identify the hidden rear. The earlier raised fender was rejected. The final editing instruction was: preserve the existing canvas and panel placement, remove the opposite-side charging contacts, remove the detached rear apron, and leave a complete circular wheel clearance open through both upper and lower body edges. Do not crop or recenter the remaining panel.

The approved top image is unchanged. The separate side PNG is 1254 × 1254 pixels. Its tight alpha >220 envelope is `(44,487,793,295)`. Its physical projection frame is `(44,487,1167,295)`: the real rear registration at x=1211 is retained from the full side reference before the hidden rear apron was removed. Thus the visible front panel occupies 68% of the configured chassis length; it is never enlarged to fill the whole chassis. Per-view dimensions and projection frames are also used by the compact WebP generator. The left elevation contains no right-side charging contacts.

This is a photo-proportioned illustration, not a measured CAD model. The configured chassis length/height, chassis centre and independent drive-wheel radius/axle position still determine the actual metre-scale assembly. Selecting Worx artwork must not rewrite those hardware settings to force a visual fit.


### Disabled sensor artwork

Hardware and Sensors filter the draft geometry with `lidar_enabled`; the map
filters its running URDF geometry with the saved setting. Omitted keys mean false,
matching sparse YAML and the Sensors switch. The source mounting geometry is not
modified; enabling LiDAR restores its image and editor handles at the saved mount.

### Worx preview proportions

The example fixture uses a 540 × 360 × 140 mm chassis, its bottom 70 mm below the
rear axle, with 220 mm drive wheels. This puts the shell top 70 mm above the axle
and the bottom 40 mm above ground. These are photo-proportioned illustration
values, not verified manufacturer dimensions or a hardware preset. Shell selection
must never apply them to a real mower configuration.


### Worx tray-centre alignment (2026-10-11)

Worx Dock Center X/Y are display offsets of the slatted platform from the chassis
centre/centreline. With both at zero the mower is centred on the tray. Top source
registration is x=355 within crop [47,53,721,772]; the charging pedestal extends
right of the tray and does not define its centre. The full configured dock width
still sizes the complete artwork. Hardware, Sensors and map use the same anchor.
The standard dock uses its rear-axle registration and built-in +0.220 m forward alignment.
Solid setup views use 70% dock opacity; transparent views use 18%.


Dock registration is declared in `appearanceRegistry.ts`: part source, nominal height,
reference origin, built-in forward/left offset, and top/side anchor within the
cropped footprint. `dockGeometry.ts` and `DockGraphic.tsx` have no model-specific
positioning branches. A new dock is added as metadata; mower `shellAssets.ts`
physical frames similarly preserve its true chassis extent despite image padding
or wheel openings. Both map and setup previews consume the same registration.


The standard dock has a built-in +0.220 m forward offset from the parked rear
axle in `appearanceRegistry.ts`. Operator X/Y adjustments default to zero in the frontend,
GUI schema and template; they are added to that registration, not a second nominal
alignment. Existing explicitly saved adjustments remain additive; early test
installations using a nominal dock-centre value should reset X to zero. This only
moves the illustration and does not change the calibrated dock pose.


## Focused hardware measurements

Hardware focus overlays the active dimension on the existing top/side preview.
On mobile (or a short touch viewport), the field opens a bottom-sheet editor.
Apply changes the settings draft only; Cancel/Escape discard the local edit and
restore focus. Persistence still belongs to the page Save action.

- measurements.ts registers the 18 supported keys and computes physical endpoints.
  Lengths use chassis/dock frames, tracks use tyre centres, radii start at axles,
  and signed geometry offsets use the robot origin. Dock adjustments start at the
  artwork's nominal registration, including the standard +0.220 m offset.
- MeasurementOverlay.tsx adds labels/extension lines and expands viewport bounds
  without changing any physical dimensions.
- MowerView.tsx shares the existing assembly rendering; MeasurementEditor.tsx
  reuses the Hardware preview's live description, with no extra subscription.
- useMeasurementEditor.ts connects fields to focus/dialog state. Empty, invalid or
  out-of-range input must never display the last valid value as the current measurement.
- The dialog follows visualViewport resize/scroll, uses Ant Design's focus trap,
  and keeps actions outside its scrolling body. English/French endpoint/help
  strings are shared with the existing Hardware labels/tooltips.
- Map artwork and runtime geometry are unaffected until settings are explicitly
  saved and the existing geometry-application workflow is followed.

Tests: measurements.test.ts checks physical endpoints, signed/zero offsets and
independent blade/swath widths. measurement-editor.spec.ts covers real-app desktop
and mobile editing, cancellation, focus, validation, viewport resizing and no
automatic settings writes. PR screenshots and dimensional fixture values are in
docs/pr/layered-mower/README.md.


## Adding shells and docks

`appearanceRegistry.ts` is the only authored appearance catalog. The asset wrappers,
TypeScript IDs, selectors, bilingual labels, defaults and shell/dock pairs derive
from it. Renderers contain no shell-name conditionals.

1. Add transparent PNG artwork under `public/assets/robots/layered/`.
2. Add a `shells` entry with English/French labels, top/side filenames and canvas
   sizes, source atlas cells, tight bounds, optional physical projection frames, paired dock ID,
   compact map size and per-view wheel layering.
3. For a new dock, add its artwork under `parts` and its registration under `docks`:
   appearance ID, labels, part reference, origin, built-in offset and top/side anchors.
   Reuse an existing dock ID when its artwork is suitable.
4. Run `npm run generate:appearances` from `gui/web`. This generates the backend
   `mower_visual_style` enum and compact map WebPs and registered paint-mask PNGs. Commit these derived files.
5. Run `npm run test:appearances` and the renderer/browser checks. Normal build
   validates artwork files, dimensions, bounds and pairings; normal tests also
   enforce backend enum parity. No selector, renderer or separate ID list needs editing.

Physical model presets remain independent: selecting new artwork does not alter
chassis dimensions, sensor positions or control configuration. An optional preset
association belongs to that model's metadata, not a renderer branch.


## Paintable body panels

All six current shells include hand-traced top/side paint masks. The Hardware
colour picker edits `mower_visual_colour` alongside the shell choice. Save writes
it to the shared mower YAML; Sensors, measurement previews and Map read that same
value. Empty means **Original**, preserving the source artwork. A colour-only
save requires no ROS restart and changes no physical geometry or control settings.

For every new paintable shell:

1. Keep the original transparent PNGs. Add `{id}-top-paint.svg` and
   `{id}-side-paint.svg` in the same asset directory. Use transparent SVG canvases
   with **exactly the source PNG dimensions**, including atlas padding, and
   `viewBox="0 0 WIDTH HEIGHT"`. Do not crop/reposition the mask independently.
2. Trace white filled paths over the paintable body panels. Transparent holes
   exclude tyres, black bumpers/trim, controls, display windows, metallic fittings
   and green accent lights. Use an even-odd fill for holes. Keep vector edges
   tight to panel seams; antialiasing provides the edge blend. Masks use **alpha**,
   so black opaque paths do not exclude paint: they must be transparent holes.
3. Add only this metadata to the registry shell entry:

   ```ts
   paint: {
       masks: {top: "example-top-paint.svg", side: "example-side-paint.svg"},
       suggestedColour: "#167bc1",
   },
   ```

   IDs and suggested colours are data, never renderer branches. A shell without
   `paint` remains supported with original artwork and no colour control.
4. Run `npm run generate:appearances`. It uses the same physical projection frame
   and map resolution for the PNG and mask, including shells whose true rear
   boundary extends beyond their visible side artwork. Commit the SVGs and the
   generated `map/{id}-{view}-paint.png` files; do not hand-edit map derivatives.
5. Run `npm run test:appearances`, the focused robot unit tests, and
   `playwright test tests/e2e/shell-colour.spec.ts`. Inspect the production-renderer
   gallery at `/tests/e2e/fixtures/chassis-gallery.html?paint=1`, checking both views
   at full size and compact map size. Check a light and a dark colour, seam edges,
   original trim, transparent-shell mode and side wheel openings. The fixture
   reads new entries directly from the registry. Masks must contain self-contained
   vector paths, no embedded images, scripts or external references.

`shellPaint.ts` applies the chosen pigment only inside the mask, preserving source
luminance for shadows and neutral highlights. A second coverage factor protects
existing mint accents. Output alpha is copied unchanged: recolouring must not
change the chassis footprint, padding registration or wheel openings. Invalid
colour values and unavailable masks/canvas fall back to the original shell.

`usePaintedShell.ts` rasterises once per style/view/resolution/colour, coalescing
picker input by 60 ms. Full previews are capped at 768 px and map artwork at the
registry's compact limit (currently 384 px). It retains up to eight prepared
pixel/coverage buffers and sixteen painted images per browser, not an unbounded
history of picker colours. Map pose updates only move the existing image; they
never rerun pixel processing. No separate set of full-colour PNGs is shipped.


Side-charging dock: the top atlas remains the original reference. Its side view
uses `dock-worx-side.png`, with a broad charging housing midway along the tray
and two contacts at one height. Part entries may declare per-view `sizes` when
separate source images differ; crops always use that view's native pixel coordinates.
Registration and dock footprint settings are unchanged by an artwork correction.


All layered artwork URLs go through `artworkUrl.ts`, which appends the web build
ID to the filename. The GUI serves `/assets` with a year-long immutable cache;
without this build identity an updated mask or sprite can remain stale after a
normal reload. Use the helper for full-size sources, SVG masks and compact map
assets alike. Pose changes do not alter the URL. Rebuild/deploy the web bundle
when replacing artwork; copying a changed image under an old build ID is insufficient.


### Airseekers Tron-inspired shell

The `tron` registry entry pairs with the standard dock (`styled` appearance ID).
Top and left-side projections share `tron.png`; the silver upper cover alone is
paintable through `tron-top-paint.svg` and `tron-side-paint.svg`. The graphite
exoskeleton, lower deck and bumper remain neutral. The native SVG width/height
must match the full atlas canvas, as well as its viewBox: cropped canvas drawing
uses native image coordinates. A viewBox without dimensions can load successfully
but paint nothing. Browser regression samples verify a visible tint in both
views at full and compact map resolution.

The rear wheel draws over the open side frame. Wheel/caster sizes and positions,
sensors and chassis bounds still come from the configured robot geometry. This
is a shell addition, not a Tron hardware preset or omni-wheel drivetrain model.
The colour gallery hides independent components to make the paint boundary clear;
Hardware/Sensors screenshots retain the normal independently rendered components.
Provenance, generation specification and screenshots: `docs/pr/tron-shell/README.md`.

## Cross-browser artwork coordinates

Keep shell and part rasters inside the shared `ArtworkViewport`. Its nested SVG
uses source-pixel dimensions; a parent group maps that viewport to physical
metres. Tiny metre-sized nested viewports caused Firefox to displace the raster
while the sensor editor position marker remained correct. Putting the transform
on the nested SVG itself instead of its parent caused WebKit clipping. Source
crops, physical bounds, sensor mounting transforms and robot dimensions stay
unchanged. New artwork should use this renderer rather than another SVG wrapper.

Sensor highlights use two translucent geometric strokes behind the image. Avoid
restoring the SVG morphology/blur filter: WebKit dropped the sensor raster through
that filter. The shared renderer also covers blade, wheel and dock artwork.

The opt-in `playwright.robot-artwork.config.ts` suite checks actual raster centres
against independently rendered sensor position markers, not just DOM transforms.
See the end-to-end test README for commands and the browser coverage limits.


### Preset geometry upgrade defaults

Preset numbers are authored only in `src/constants/mowerModels.ts`.
`npm run generate:model-geometry` emits matching GUI/ROS assets for the explicit
upgrade-key allowlist in `scripts/model-geometry-defaults.mjs` (currently only
`chassis_z_offset`). The Go asset parity test (`go test ./...` in GUI CI) rejects stale copies. The loader
applies these before installed values, and the GUI uses them for reads, reset and
sparse-save pruning; zero is a valid explicit override. Do not add model-name or
offset branches to those consumers. Other calibrated preset fields are not reapplied.
