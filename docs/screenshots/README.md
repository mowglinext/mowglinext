# GUI screenshots

Captured from a live robot (`http://<mower-ip>:4006`) with the interface in
English, at 1440×900 (desktop) and 390×844 (mobile). Personal data — the GPS
datum and live position, the NTRIP mount point, satellite tiles of the garden —
is blurred before the capture is committed; the Map page uses the GUI's
**Dark map** style, which shows the areas and the LiDAR tile map with no
imagery.

| File | State / View |
|------|-------------|
| `dashboard-idle.png` | Home, robot idle and charging on the dock |
| `dashboard-charging.png` | same capture, kept under its historical name |
| `dashboard-mobile.png` | Home on a phone |
| `map.png` | Map page, dark style |
| `schedule.png` | Schedule page |
| `stats.png` | Statistics page |

The per-page walkthrough set (settings tabs, diagnostics tabs, onboarding
steps, map editing) lives in `../gui-walkthrough/screenshots/`.

Captures of the MOWING and EMERGENCY states were dropped when the interface
was redesigned (September 2026) — they need a live mow to reproduce. Retake
them the same way and add them back here when one is available.

## How to retake

1. Open the GUI in a desktop browser at 1440×900, switch it to English.
2. Blur anything personal before saving (coordinates, mount point, map
   imagery); on the Map page pick More → Dark map instead of blurring.
3. Save as PNG under the names above, then update the references in the
   README, `docs/index.html` and the wiki.
