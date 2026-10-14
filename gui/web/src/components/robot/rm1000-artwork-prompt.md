# RM1000 shell atlas generation

Tool: built-in imagegen, transparent background enabled.

Shape reference: `public/assets/robots/biltema-rm1000/mower.webp`.
Style/layout reference: `public/assets/robots/layered/sculpted.png`.
Project output: `public/assets/robots/layered/rm1000.png`.

Prompt:

Create one transparent sprite atlas with two precisely orthographic views of a robotic mower BODY SHELL ONLY. First image is shape reference: Biltema RM1000, recognise its long rounded rectangular shell, two broad shoulder strips, black recessed central panel, forward vents, rear console. Second image is style and atlas layout reference only: graphite gray materials and subtle mint green highlights. Restyle the RM1000 reference shell in graphite gray with restrained mint highlights (no blue). Layout: top view occupies left 45% of canvas, front faces UP, rear console at BOTTOM. Side elevation occupies right 52%, front faces LEFT, rear wheel arch on RIGHT. Side elevation is illustrative because only a top reference exists; keep consistent top features and a low smooth profile. Both views must depict same shell. Remove all wheels, casters, sensors, stickers, writing, branding and logos. No red STOP button baked into either view: app adds a separate red button at rear. Rear body intact with a clean console panel, no empty rear hole; side has a rear wheel recess. Truly transparent background and transparent wheel recesses. No floor, ground shadows, labels, measurement marks or perspective. Keep all shell edges inside frame with a little clear padding and a clear gap between views. Professional UI asset, matching second reference's lighting and quality. Landscape atlas.

## Side-only revision from supplied photograph

Output: `public/assets/robots/layered/rm1000-side.png`.
Input: user-supplied RM1000 side photo (clipboard ff774aba-79d3-44f9-95be-269d297f4027), with the existing atlas as style reference.
The top atlas is retained unchanged. Built-in imagegen, transparent background.

Prompt:

Use case: style-transfer. Create a single transparent orthographic SIDE ELEVATION asset of the RM1000 mower BODY SHELL ONLY, front facing LEFT, rear facing RIGHT. First input is the real side/three-quarter photograph: use it for the actual silhouette and body architecture. Second input is our approved gray/mint sprite atlas: use only for color/material/illustration style; do not reproduce its imaginary side shape and do not produce a top view. The real shell has a rounded low front bumper, stepped black lower front skirt, rising rounded shoulder toward rear, recessed central height-adjustment knob near middle/front, rear control console, very large rear wheel opening and a short rear apron descending behind that opening. Translate this photograph into a true horizontal orthographic side elevation without perspective. Gray graphite shell instead of blue, black lower skirt and wheel arch trim, restrained mint accents along shoulder matching approved top. Remove branding/text/logos, remove wheels and casters completely (wheel cutout must be genuinely transparent), remove ground/shadows. No external GPS/lidar, no red stop button (separate UI layer adds rear button); preserve closed rear console around it, not a rear hole. The body only, precise clean edges on fully transparent background, generous clear margin, landscape canvas. This is a layer for assembly with separate wheels. Main wheel arch must be large and open downwards, centred near 80% of shell length from the front, like photograph. Render a faithful stylized shell, not a sports car or generic futuristic body.

Projection correction:
Correct only the camera projection of this mower shell asset to a TRUE ORTHOGRAPHIC SIDE ELEVATION. Current image shows far too much of the top deck and front face. Camera must be perfectly horizontal at body mid-height, perpendicular to the left side; zero perspective and zero downward angle. Front LEFT, rear RIGHT. Keep the exact same RM1000 body architecture, gray/mint style, big transparent rear wheel opening, lower black front skirt, height adjustment knob and rear console. From exact side-on the knob and console are seen only as low profile silhouettes above the shoulder, NOT broad visible top surfaces. The far side must not be visible at all. Shell only, no wheels/casters/sensors/red stop button/text/branding/shadows. Fully transparent background; clean single side asset centered with padding. Keep skirt horizontal.
