# Importing an OpenMower map into MowgliNext

> **Status: live for `map.json`.** Apply runs
> `/map_server_node/clear_map` → `add_area` ×N → `save_areas` →
> `set_docking_point`. Triggered from **More menu → Import from
> OpenMower** on the map page. `.bag` is still designed-only (§6).
> Implementation: `gui/pkg/api/openmower_import.go::applyImport`
> (shares `replaceMapInternal` + `setDockingPointInternal` from
> `mowglinext.go`) and the React modal at
> `gui/web/src/pages/map/components/ImportOpenMowerModal.tsx`.

This document specifies how a user can take a pre-existing OpenMower
1.x deployment (one of the many garden setups out there) and bring its
recorded mowing/navigation areas + docking station across to a fresh
MowgliNext install without re-driving the boundaries.

Three source formats are addressed:

1. **`map.json`** (modern) — OpenMower's current persistence format.
   Lower-case `areas` / `docking_stations` keys, polygons as
   `outline: [{x, y}]`, dock as a struct with `position` + `heading`.
   **Implemented** as parse + preview; write step stubbed.
2. **Legacy bag-JSON** — `mower_map_service`'s rosbag content serialised
   to JSON (e.g. via the rosbridge JSON encoder or any ROS1→JSON tool).
   PascalCase fields with `Package: 0` envelopes, top-level
   `NavigationAreas` / `WorkingArea` arrays of `{Name, Area: {Points:
   [{X, Y, Z}]}, Obstacles}`, dock as scalar `DockX` / `DockY` /
   `DockHeading`. **Implemented** as a conversion layer:
   `tryConvertLegacyOpenMowerMap` rewrites the body in-place to the
   modern shape before the existing parser runs. WorkingArea entries
   become mow areas; their nested Obstacles become top-level
   `type="obstacle"` entries (matched back to their parent by centroid
   containment); NavigationAreas become nav areas; the scalar dock
   fields become the single docking station.
3. **`map.bag`** (raw rosbag bytes) — the original binary container,
   still found on very old installs that haven't migrated to either
   form above. **Designed only**; deferred because pure-Go ROS1-bag
   readers are unmaintained and we'd prefer a sidecar. Users with
   raw `.bag` files should run OpenMower 1.x once to auto-convert to
   either the modern `map.json` or the legacy bag-JSON above.

---

## 1. OpenMower `map.json` schema

Source of truth: `ClemensElflein/open_mower_ros` →
`src/mower_map/src/mower_map_service.cpp`. The JSON is produced via
`nlohmann::ordered_json` from these C++ structs:

```cpp
struct Point        { double x; double y; };
typedef std::vector<Point> Polygon;

struct MapArea {
  std::string id;        // 32-char nano-id
  std::string name;      // user-facing label, may be ""
  std::string type;      // "mow" | "nav" | "obstacle" | "draft"
  bool        active;    // omitted from JSON when true
  Polygon     outline;   // local x/y in metres, OpenMower map frame
};

struct DockingStation {
  std::string id;
  std::string name;       // "Docking Station" by default
  bool        active;
  Point       position;   // metres in the OpenMower map frame
  double      heading;    // radians, 0 = +x (east)
};

struct MapData {
  std::vector<MapArea>        areas;
  std::vector<DockingStation> docking_stations;
};
```

### Wire format example

```json
{
  "areas": [
    {
      "id": "AbCdEfGhIjKlMnOpQrStUvWxYz012345",
      "properties": { "name": "Front lawn", "type": "mow" },
      "outline": [
        { "x":  0.00, "y":  0.00 },
        { "x": 12.30, "y":  0.00 },
        { "x": 12.30, "y":  8.10 },
        { "x":  0.00, "y":  8.10 }
      ]
    },
    {
      "id": "ZzZzZzZzZzZzZzZzZzZzZzZzZzZz0001",
      "properties": { "type": "obstacle" },
      "outline": [
        { "x": 4.0, "y": 2.0 },
        { "x": 5.0, "y": 2.0 },
        { "x": 5.0, "y": 3.0 },
        { "x": 4.0, "y": 3.0 }
      ]
    },
    {
      "id": "NnNnNnNnNnNnNnNnNnNnNnNnNnNn0002",
      "properties": { "name": "Driveway link", "type": "nav" },
      "outline": [
        { "x": -1.0, "y":  0.0 },
        { "x": -1.0, "y":  2.0 },
        { "x":  0.0, "y":  2.0 },
        { "x":  0.0, "y":  0.0 }
      ]
    }
  ],
  "docking_stations": [
    {
      "id": "DdDdDdDdDdDdDdDdDdDdDdDdDdDd0003",
      "properties": { "name": "Docking Station" },
      "position": { "x": -0.45, "y":  0.20 },
      "heading":  1.5707963267948966
    }
  ]
}
```

Notes on the structure that bit us in early prototypes:

- **Obstacles are flattened to top-level `MapArea`s with `type:"obstacle"`.**
  They are *not* nested inside the parent area as MowgliNext stores them.
  An obstacle has no parent reference in the JSON — it's an island
  polygon that must be matched back to a containing area by the
  importer (point-in-polygon).
- The `properties` envelope is optional. `name` is omitted when empty,
  `active` is omitted when `true`, so a freshly written area can collapse
  to `{ "id": "...", "properties": { "type": "mow" }, "outline": [...] }`.
- `type:"draft"` is the default for half-recorded areas — the importer
  treats `draft` as a soft warning and skips it (no MowgliNext analogue).
- `outline` is **not** closed. The first/last point are typically
  distinct; MowgliNext's `MapArea.area.points` doesn't require a closing
  point either.
- `docking_stations` is plural, but in practice OpenMower writes
  exactly one entry. The importer takes the first `active` station and
  warns if there are more.

### What is *not* in `map.json`

- **No datum.** OpenMower stores the geodetic datum (`OM_DATUM_LAT`,
  `OM_DATUM_LONG`) in `mower_config.sh` env vars, *not* in `map.json`.
  All polygon points are local metres in the OpenMower `map` frame
  anchored at that datum.
- **No frame metadata.** The frame is implied; it's flat ENU, X = east,
  Y = north (REP-103-aligned).
- **No timestamps**, no version field, no schema URL.

This means the importer **must** ask the user — or be told via an env
var — what OpenMower datum the points are anchored to. See
[§4 Coordinate frame handling](#4-coordinate-frame-handling).

---

## 2. Field-by-field mapping

| OpenMower (`map.json`)                              | MowgliNext target                                                        | Notes |
|-----------------------------------------------------|--------------------------------------------------------------------------|-------|
| `area.properties.type == "mow"`                     | `MapArea` with `is_navigation_area=false`, `Obstacles=[]`                | Saved via `/map_server_node/add_area`. |
| `area.properties.type == "nav"`                     | `MapArea` with `is_navigation_area=true`, `Obstacles=[]`                 | Same service, flag flipped. |
| `area.properties.type == "obstacle"`                | `geometry_msgs/Polygon` appended to the parent area's `Obstacles`        | Parent found by point-in-polygon against the `mow` and `nav` areas. Obstacles outside any area are dropped with a warning. |
| `area.properties.type == "draft"`                   | dropped                                                                  | Warned. |
| `area.properties.name`                              | `MapArea.name`                                                           | Empty string → MowgliNext auto-names it (`mowing_area_<idx>`). |
| `area.properties.active == false`                   | dropped                                                                  | Inactive areas are not imported. |
| `area.outline[i].{x,y}`                             | `MapArea.area.points[i].{x, y}`, `z=0`                                   | After datum re-anchoring (see §4). |
| `docking_stations[0].position.{x,y}`                | `dock_pose_x`, `dock_pose_y` in `mowgli_robot.yaml`                      | After datum re-anchoring. |
| `docking_stations[0].heading`                       | `dock_pose_yaw` in `mowgli_robot.yaml`                                   | Already radians. The MowgliNext convention is identical (yaw 0 = +x = east, CCW positive), so no rotation flip is needed at the ENU level — but see §5 if the user is also rotating the map. |
| (no equivalent)                                     | `is_navigation_area` flag on obstacles                                   | MowgliNext stores obstacles per-area, OpenMower stores them globally. Reconstructed on import. |

The MowgliNext write-side surface (already exists, just not yet wired
to the importer):

- `PUT /api/mowglinext/map` — clear + bulk-insert all areas, calls
  `/map_server_node/clear_map`, then loops `/map_server_node/add_area`,
  then `/map_server_node/save_areas`.
- `POST /api/mowglinext/map/docking` — calls
  `/map_server_node/set_docking_point`, which line-splices `dock_pose_*`
  into `mowgli_robot.yaml` while preserving comments + perms.

The importer plans to call **exactly these two endpoints** from the Go
handler — no new ROS service surface is needed.

---

## 3. Coordinate frame handling

MowgliNext's `map` frame is a **true-north** equirectangular ENU anchored
at `(datum_lat, datum_lon)` from `mowgli_robot.yaml`
(`navsat_to_absolute_pose_node.cpp`: `east=(lon−datum_lon)·cos(datum_lat)·M`,
`north=(lat−datum_lat)·M`).

OpenMower's `map` frame is **NOT** the same kind of ENU. Its
`xbot_driver_gps` stores every point as **UTM grid** eastings/northings
relative to its datum (`LLtoUTM(fix) − LLtoUTM(OM_DATUM)`; see
`xbot_driver_gps/src/interfaces/ublox_gps_interface.cpp`). UTM axes follow
**grid north**, which differs from true north by the meridian convergence
`γ = atan(tan(λ − λ₀)·sin φ)` (`λ₀` = the zone's central meridian). A couple
of degrees off the central meridian, `γ` reaches **1–2°** — this is the
"imported map is rotated 1–2°" report. On a large lawn the same rotation
also pushes the boundary far enough out of place to break coverage planning.

The importer therefore does a **full UTM round-trip**, not an additive
shift: it inverts OpenMower's exact projection and re-projects into
MowgliNext's frame (`gui/pkg/api/utm.go`, a port of the Snyder-series
`RobotLocalization::NavsatConversions` OpenMower links):

```
abs_UTM = LLtoUTM(om_datum) + (x_om, y_om)   # OpenMower's stored grid coords
lat,lon = UTMtoLL(abs_UTM, om_zone)          # invert OM's projection
x_mn    = (lon − mn_lon) · cos(mn_lat) · M   # MowgliNext true-north ENU
y_mn    = (lat − mn_lat)              · M     # M = 111319.49079327357
```

This corrects the datum offset, the east-scale AND the grid-north
convergence in one step, and the dock heading is rotated by `−γ` to match
(`datumReprojector.projectYaw`).

### 3a. Same datum

User reuses the same RTK base + configured datum. The offset is zero, but
the UTM grid→true-north convergence is **still removed**, so even a
same-datum import is de-rotated (this is intentional — OpenMower's stored
coordinates are grid-aligned regardless of the datum).

### 3b. Different datum / migrated install

The round-trip above absorbs an arbitrary datum displacement exactly (no
small-angle/flat-earth approximation), so a moved RTK base or a relocated
install imports correctly as long as the OpenMower datum is supplied.

### 3c. OpenMower datum required for georeferencing

The fix needs the source `OM_DATUM_LAT/LONG` (the UTM anchor to invert
against). The import modal has two optional inputs for it. When omitted,
the importer **cannot** undo the convergence and falls back to copying the
OpenMower coordinates through unchanged, with a prominent warning that the
map will be both offset and rotated — the user should re-import with the
datum filled in. (The GUI's display-only **Map offset / bearing** panel is
no longer the rotation fix — it only spins the Mapbox camera, never the
saved coordinates.)

### Lat/lon round-trip

The actual OpenMower datum is discoverable at runtime by sniffing the
robot — `mower_config.sh` sets it as an env var that's exposed on the
ROS parameter server (and in `/odom_to_world_node` config), or it can
be read from `mower_logic` debug topics. **We will not auto-discover
it from the JSON file**, because the file doesn't carry it. The UI asks.

---

## 4. Validation rules

The importer enforces, before showing the preview modal:

1. JSON parses as `MapData`. Unknown top-level keys are tolerated.
2. There is at least one `area` with `type == "mow"`.
3. Each `outline` has ≥3 points. Polygons with <3 points are dropped
   with a per-area warning.
4. Each `outline` is finite: no `NaN`, `Inf`, or absurd magnitudes
   (`|x| > 100000` or `|y| > 100000` ⇒ rejected — sanity bound).
5. Exactly one `active` docking station. >1 → first wins, warning.
   0 → import succeeds without touching `dock_pose_*`.
6. Obstacle ↔ parent matching: every `obstacle` polygon's centroid must
   fall inside exactly one mow/nav area. Ambiguous (in 2+) → assigned
   to the first match with a warning. Orphan (in 0) → dropped, warning.

The summary returned to the GUI lists all warnings + counts so the user
can decide whether to confirm.

---

## 5. Decisions taken when flipping the write path live

1. **Datum mismatch.** The preview modal now has `om_datum_lat`/
   `om_datum_lon` inputs and re-previews on change; with the datum
   supplied the importer does the full UTM round-trip (§3) so the offset,
   scale and grid-north convergence are all corrected before Apply. It
   still surfaces the computed datum shift loudly ("Datum shift:
   east=X m, north=Y m") and the per-area approximate-area column. Without
   the datum it warns that the map will be offset+rotated rather than
   silently shifting.

2. **Dock yaw convention.** Both stacks use `yaw = atan2(north, east)`,
   so the heading is portable. But OpenMower stores the heading as a
   single scalar; some early OpenMower versions stored it as a
   two-point line (`docking_pose_a`, `docking_pose_b` in the legacy
   `.bag` topic) — those need conversion via `atan2(by-ay, bx-ax)`.
   The .bag importer (§6) handles this; the JSON importer doesn't need
   to.

3. **Multiple docking stations.** OpenMower's schema supports a list,
   MowgliNext stores exactly one in `mowgli_robot.yaml`. We pick the
   first `active`, warn on the rest. If a real-world user has multiple
   docks, this needs a follow-up (likely a `dock_id` on the saved
   areas + multiple dock poses).

4. **Obstacle re-parenting on edit.** Once imported, MowgliNext stores
   obstacles per-area. If the user later moves an area boundary so an
   obstacle is no longer inside, that's pre-existing behaviour, not
   an import concern. Documented for the record.

5. **`type: "draft"` handling.** Currently dropped. Could promote to
   `mow` if the user opts in via a checkbox. Probably not worth it —
   drafts in OpenMower are almost always half-recorded sessions the
   user abandoned.

6. **Replace vs merge.** Shipped **replace-only**: Apply calls
   `clear_map` before adding the imported areas, matching the
   semantics of the regular *Save* button. Merge mode is a one-line
   change (skip `clear_map`) but is gated on a real use case — the
   risk of silent duplicate areas wasn't worth the toggle.

7. **Backup before import.** Not auto-triggered. The modal's confirm
   alert tells the user to use **Backup Map** (already in the same
   More menu) first if they want a rollback file; we did not wire an
   automatic browser-side download into the Apply path because
   producing a sidecar file as a side-effect of clicking Apply is
   surprising. If incident rate justifies it later, the change is
   ~5 lines (call `handleBackupMap` from the modal's `onApply`).

---

## 6. `.bag` (rosbag2 / ROS1 bag) — design only

OpenMower's *legacy* persistence is a ROS1 `.bag` file. The current
OpenMower code includes a one-shot in-process converter
(`convertLegacyMapToJson` in `mower_map_service.cpp`) that reads three
topics and re-emits `map.json`:

| Topic                | ROS msg type                      | Maps to                    |
|----------------------|-----------------------------------|----------------------------|
| `mowing_areas`       | `mower_map/MapArea`               | `area.type = "mow"`, with `obstacles[]` flattened to top-level `obstacle` areas |
| `navigation_areas`   | `mower_map/MapArea`               | `area.type = "nav"`        |
| `docking_point`      | `geometry_msgs/Pose`              | `docking_stations[0]`      |

`mower_map/MapArea` is:

```
string                 name
geometry_msgs/Polygon  area
geometry_msgs/Polygon[] obstacles
```

i.e. the *exact* shape of MowgliNext's own `mowgli_interfaces/msg/MapArea`
— so once the bag is parsed, mapping is trivial.

### MowgliNext `.bag` import — three options

**(a) Tell the user to run OpenMower's converter once, then upload the
resulting `map.json`.** Zero code, but requires the user to have a
working OpenMower install. Recommended fallback.

**(b) Pure-Go bag reader.** ROS1 bag format is documented and there
are Go implementations (`github.com/aler9/rosbag-go`,
`github.com/brychanrobot/goros`, `github.com/foxglove/go-rosbag`).
None of them are well-maintained, and ROS1 message-class metadata
(`mower_map/MapArea`) would need to be hand-fed in for deserialisation.
**Effort: ~1 day, fragile.**

**(c) Python sidecar invoked by the Go importer.** The MowgliNext
container already ships ROS2 + `rosbags` Python package
(`pip install rosbags`) which reads ROS1 `.bag` and ROS2 `.db3` /
`.mcap`. The Go handler shells out to a small `import_bag.py` script
that re-emits `map.json`-shaped output, then reuses the existing JSON
import pipeline. **Effort: ~½ day, robust, single dependency
(`rosbags`).** Recommended.

For ROS2 bags (`.db3` / `.mcap`) — note that OpenMowerNext (the
ROS2 fork by `jkaflik`) records `/xbot_positioning/map` periodically;
the importer would extract the *most recent* message of that topic
and treat it identically to a `.json` import.

**Decision deferred to follow-up PR.** This iteration ships the JSON
path; the UI shows a "coming soon" notification when the user picks a
`.bag` file.

---

## 7. Files

- `gui/pkg/api/openmower_import.go` — Gin handler, JSON parse,
  validation, structured preview log, live `applyImport()` calling the
  shared write helpers.
- `gui/pkg/api/mowglinext.go` — `replaceMapInternal` +
  `setDockingPointInternal` shared between the public PUT/POST
  endpoints and the importer.
- `gui/pkg/api/openmower_import_test.go` — fixture-driven parse +
  validation tests, plus an apply-path test that asserts the
  clear_map → add_area×N → save_areas → set_docking_point sequence.
- `gui/web/src/pages/MapPage.tsx` — stashes the uploaded file text,
  wires the modal's `onApply` to `handleApplyOpenMowerImport`.
- `gui/web/src/pages/map/hooks/useMapFiles.ts` — adds
  `handleImportOpenMower` (preview) + `handleApplyOpenMowerImport`
  (re-POST with `apply: true`).
- `gui/web/src/pages/map/components/ImportOpenMowerModal.tsx` —
  preview + apply UI with loading / error states.
- `docs/IMPORT_OPENMOWER_MAP.md` — this file.
