# Map robot images

Store bundled, display-only robot images under `gui/web/public/assets/robots/`.
Use product directories for model-specific images and `generic/` for optional
visual approximations available across models. Hardware preset names are not
proof of shell identity; add an appearance mapping only when that identity is
explicit.

Prepare each image as a near-square transparent PNG or WebP at a resolution
appropriate for the visible map size. Use a true top-down view with the mower's
front or dock-local +X pointing to the top edge. Crop around the visible object,
leave a small even margin, and center it. Preserve transparent pixels. Keep the
asset crisp; lossless WebP or PNG is preferred for detailed artwork.

Appearance metadata records the fraction of the image occupied along each
calibrated axis, the corresponding real-world dimension, and a normalized pose
anchor (`0..1` from left/top to right/bottom). The mower's front points to the
source image's top edge. The RM1000 mower's 0.57 m length and charging station's
0.63 × 0.46 m dimensions are reported by [Forbrugerrådet Tænk's product test](https://taenk.dk/test/robotplaeneklipper/biltema-rm1000).
The mower's configured rear-axle anchor is an image-based estimate; confirm it
against a model-specific `base_link` measurement before treating sub-body
alignment as calibrated.

For dock images, the top edge points along dock-local +X (out toward the staging
area). The map server places the dock body from `-dock_body_length` to the dock
pose at `x=0`, so the top-center image anchor maps to that pose; it is calibrated
to the image's top edge, not to a generic keepout rectangle. The support-sticker
dock image shows the RM1000 station and is offered only for the RM1000
appearance. The clean image is the contributor's RM1000 station photo with its
model-specific branding removed. It is offered as a generic visual
approximation that may suit many conventional mower docks; its nominal 0.63 ×
0.46 m visual footprint comes from the RM1000 product dimensions above and is
not a claim about another station's actual dimensions. Use a dedicated
model-specific image and calibration when exact geometry is needed. The
existing dock marker remains the universal default and fallback.

The mower and dock images originate from photographs taken by the contributor,
edited with AI-assisted tools and intentionally contributed under this
repository's licensing terms. Keep the existing drawn footprint and dock
marker as runtime fallbacks whenever an image or valid pose is missing or an
image cannot be decoded.
