# Layered mower draft PR screenshots

Captured from the real application with mocked REST/WebSocket telemetry using
`gui/web/tests/e2e/layered-pr-gallery.spec.ts`, Chromium, 1440 x 1250.
No mower was contacted. The settings preview uses the example values below;
the map continues to use the running robot description, not unsaved settings.
These are illustrative installations, not measurements of .118 or new presets.

| Geometry (metres) | Yardforce 500 example | Custom angular (Utility) |
| --- | --- | --- |
| Chassis length / width / height | .600 / .450 / .190 | .660 / .460 / .180 |
| Chassis centre X / bottom Z | .180 / -.050 | .180 / -.060 |
| Drive wheel radius / width / centre track | .100 / .040 / .325 | .100 / .050 / .300 |
| Caster radius / centre track / X | .030 / .300 / .400 | .040 / .280 / .390 |
| GPS X / Y / Z | .150 / 0 / .148 | .160 / 0 / .130 |
| LiDAR X / Y / Z | .310 / 0 / .139 | .320 / 0 / .131 |
| IMU X / Y / Z | .040 / -.090 / .015 | .035 / -.090 / .005 |

All sensor yaw values are zero. Coordinates are relative to the rear axle;
X is forward, Y left and Z up. GPS and LiDAR bases meet the local visible shell roof; the IMU
is enclosed. Drive-wheel outer spans are 365 mm (500) and 350 mm (custom). Wheels and
casters fit within the chassis plan bounds. The 500 retains the shipped chassis
and drive-wheel dimensions, with example caster and sensor installation overrides.
The custom values are screenshot fixtures only and do not alter default settings.

Source shell contours are illustrative, not CAD. The Yardforce rear wheel recess
still needs refinement; see the wheel enclosure audit and HARDWARE_REQUIRED
measurement procedure in `gui/web/src/components/robot/README.md`.

Validation on 2026-10-10: all 942 GUI unit tests in 116 files passed; GUI lint
passed with 898 warnings and zero errors. Two screenshot tests, four Xacro tests,
configuration drift and Go schema/template/model/generated-type parity passed.
Earlier checks on this feature: GUI production build; 12 application Playwright
cases and map projection checks. Full ROS workspace build/test and physical
geometry acceptance have not been completed.


## Chassis collection gallery

`chassis-gallery.png` is a presentation fixture, not an additional application
screen. It uses the production `LayeredMower` renderer with the same
600 x 450 x 190 mm geometry for all four styles, 200 mm drive wheels,
325 mm centre track, 40 mm tyre width and casters at X=390 mm / track=280 mm.
Sensors are omitted to make the shell shapes easy to compare. Reproduce with:

```sh
npx playwright test tests/e2e/layered-pr-gallery.spec.ts -g "chassis style gallery" --workers=1
```

The gallery capture test passed and the final screenshot was visually inspected.
