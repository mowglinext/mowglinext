# Webots appearance assets

Unmodified Cyberbotics Webots **R2025a** TireRubber appearance and its texture
maps, vendored so the garden scene resolves in the stripped, offline SDK.
Both material variants are retained to keep the upstream PROTO self-contained.

Source: https://github.com/cyberbotics/webots/tree/R2025a/projects/appearances/protos

- `TireRubber.proto`
- `textures/tire_rubber/{car,bike}_tire_rubber_{base_color,roughness,normal,occlusion}.jpg`

The upstream PROTO declares Apache-2.0; see `LICENSE`. Only the referencing
`MowgliMower.proto` import path changes. This appearance does not configure
contact material or wheel physics.
