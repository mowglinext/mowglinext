# Isabey fixture

`isabey.hpp` contains only map-frame geometry from the map supplied on 2026-10-05:
one recorded outer boundary and ten active obstacles. Dock position, navigation
areas, LiDAR-ignore lines, obstacle descriptions and pending proposals are excluded.
The coordinates are cast to float before planning to match ROS Polygon transport.

Original export SHA-256:
`fe0e60ed1053bc94dde1f9d43d0670b4278575f754dc40a4f457d425258f80e7`.

At `e1fd5b2734a559392195216bb52930301e22bd90`, real F2C 3.0.0 (`884d895`) with
repository defaults produced 9 sections, 1070.18 m of mowing path, 22.01 m of
straight-line transit links and a 7.43 m maximum link. Connector-aware swath ordering
and grouped obstacle loops produce 5 sections, 1015.67 m of mowing path, 9.23 m of
links and a 3.20 m maximum link. Both retain all 115 rows and 16 loops. These are
**offline geometry measurements**, not robot travel or actual Nav2 transit routes.

`test_coverage_route.cpp` pins conservative improvement bounds and verifies the cut,
segment containment, obstacle clearance, determinism and perimeter winding. Run:

```sh
colcon test --packages-select mowgli_coverage --return-code-on-test-failure
```

Physical acceptance remains **HARDWARE_REQUIRED**. Robot unit, installed firmware
image/hash and receiver/driver revisions are unknown. Record those alongside the
exact ROS image, repository commit/patch, submodule gitlinks and merged configuration
before a same-robot baseline/candidate comparison. Use this map, five passes, 0.16 m
row spacing, AUTO angle, 0.389 m obstacle margin, zero boundary inset, 0.20 m turn
radius and unrestricted headland turns. Start a new job rather than resuming old
section indices. Perform a supervised blade-disabled traversal first, logging
planned/executed paths, completed sections and recovery events. Pass requires every
row/loop completed, configured ring winding retained, no collision/boundary breach
and no untrackable turn or failed section; compare actual total and blade-off travel
against the baseline before claiming a field efficiency improvement. Operator and
emergency stop must be present, the field clear, localization/drive systems healthy,
and normal firmware safety and collision-monitor protections enabled.
