# Side-charging dock side-view correction

The original top artwork is unchanged. The side view is independently registered
in `appearanceRegistry.ts`, preserving footprint dimensions and dock offsets.
The charging housing is centred along the tray, broadened to match the top-view
layout, and has two visible contacts at the same height.

Tool: built-in image generation, transparent-background edit. Final asset:
`gui/web/public/assets/robots/layered/dock-worx-side.png`.

Final prompt (references: original dock atlas and the first centred side draft):

> Precise-object-edit for production mower charging dock side sprite. Image 1 is original atlas, its TOP VIEW on left is authoritative geometry. Image 2 is side-view edit target: tray and centered tower are good, but tower is much TOO NARROW and two charging contacts are missing. Output ONLY corrected SIDE VIEW on genuine transparent background. Preserve Image 2 horizontal tray length, thin base height, level orthographic camera, graphite grey material and shading, centered tower location, subtle mint indicator and transparent padding. Correct upright housing to match top reference: housing body spans approximately 28% of total tray length horizontally (current body only about16%); its center exactly halfway along tray. Housing must be a BROAD low rectangular charging unit, not a skinny column, with subtly beveled edges; same height as Image2. Add TWO clearly visible SILVER METAL CHARGING PRONGS to the broad face facing the viewer, separated HORIZONTALLY, at the SAME HEIGHT, one near left end and one near right end of housing. These contacts project towards mower/viewer, foreshortened short cylindrical steel pins with rounded ends, visible short shafts and dark bases; NEVER stacked vertically. Their horizontal separation is approximately 14% of tray length, matching top view reference contact spacing. Broad housing about 460px wide relative to approximately1650px tray, two contacts around240px apart. Do NOT shift the housing towards either end of tray. No mower, no text, no brands, no cables, no extra panels, no other views, no ground/background/shadow. Keep full tray silhouette uncut.

Generation is illustrative, not a dimensional survey. Physical sizing still
comes from configured dock length/width and registry height, not source padding.
