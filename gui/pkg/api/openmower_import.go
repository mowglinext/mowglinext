package api

import (
	"context"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"log"
	"math"
	"net/http"
	"os"
	"strings"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/msgs/geometry"
	"github.com/cedbossneo/mowglinext/pkg/msgs/mowgli"
	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"gopkg.in/yaml.v3"
)

// ---------------------------------------------------------------------------
// OpenMower map.json schema (read-only mirror of the format produced by
// ClemensElflein/open_mower_ros mower_map_service.cpp). Field-by-field
// mapping → MowgliNext is documented in docs/IMPORT_OPENMOWER_MAP.md.
// ---------------------------------------------------------------------------

// openMowerPoint is one (x, y) vertex in OpenMower's local map frame
// (metres, ENU, anchored at OpenMower's OM_DATUM_LAT / OM_DATUM_LONG).
type openMowerPoint struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// openMowerAreaProperties is the optional `properties` envelope around
// each area. OpenMower omits empty fields, so all of these are
// optional with sensible defaults.
type openMowerAreaProperties struct {
	Name   string `json:"name"`
	Type   string `json:"type"`   // "mow" | "nav" | "obstacle" | "draft"
	Active *bool  `json:"active"` // omitted ⇒ true (per OpenMower's to_json)
}

// openMowerArea is one polygon. Obstacles are top-level entries with
// type=="obstacle"; they are NOT nested inside their parent area.
type openMowerArea struct {
	ID         string                  `json:"id"`
	Properties openMowerAreaProperties `json:"properties"`
	Outline    []openMowerPoint        `json:"outline"`
}

// openMowerDockingStation is a single dock pose. The schema is plural
// but in practice OpenMower writes exactly one.
type openMowerDockingStation struct {
	ID         string `json:"id"`
	Properties struct {
		Name   string `json:"name"`
		Active *bool  `json:"active"`
	} `json:"properties"`
	Position openMowerPoint `json:"position"`
	Heading  float64        `json:"heading"` // radians
}

// openMowerMap is the root document.
type openMowerMap struct {
	Areas           []openMowerArea           `json:"areas"`
	DockingStations []openMowerDockingStation `json:"docking_stations"`
}

// ---------------------------------------------------------------------------
// Request / response types
// ---------------------------------------------------------------------------

// ImportOpenMowerRequest is the JSON body of POST /import/openmower.
//
// Map: the verbatim contents of the user's OpenMower map.json. Required.
//
// OmDatumLat / OmDatumLon: optional. When provided AND different from
// the configured MowgliNext datum (in mowgli_robot.yaml), the importer
// applies a translation so OpenMower's local x/y land at the right
// MowgliNext map-frame coordinates. When omitted, "same datum" is
// assumed (identity transform). See docs/IMPORT_OPENMOWER_MAP.md §3.
//
// Apply: when false (the default), the handler runs in **preview-only**
// mode — parse + validate + summary, no writes. When true, the handler
// will perform the write. The write path is currently stubbed (logs
// only) per the design doc; flipping `Apply` to true today still does
// not mutate the live map. See applyImport() for the TODO.
type ImportOpenMowerRequest struct {
	Map        json.RawMessage `json:"map"`
	OmDatumLat *float64        `json:"om_datum_lat,omitempty"`
	OmDatumLon *float64        `json:"om_datum_lon,omitempty"`
	Apply      bool            `json:"apply,omitempty"`
}

// ImportOpenMowerSummary is the user-facing preview that the GUI shows
// in its confirmation modal.
type ImportOpenMowerSummary struct {
	MowingAreas      int                 `json:"mowing_areas"`
	NavigationAreas  int                 `json:"navigation_areas"`
	Obstacles        int                 `json:"obstacles"`
	OrphanObstacles  int                 `json:"orphan_obstacles"`
	DockPose         *ImportDockPose     `json:"dock_pose,omitempty"`
	DatumShiftEastM  float64             `json:"datum_shift_east_m"`
	DatumShiftNorthM float64             `json:"datum_shift_north_m"`
	Warnings         []string            `json:"warnings"`
	Areas            []ImportedAreaBrief `json:"areas"`
	Applied          bool                `json:"applied"`
}

// ImportedAreaBrief is one row in the per-area preview table.
type ImportedAreaBrief struct {
	Name             string  `json:"name"`
	Type             string  `json:"type"` // "mow" | "nav"
	Vertices         int     `json:"vertices"`
	Obstacles        int     `json:"obstacles"`
	IsNavigationArea bool    `json:"is_navigation_area"`
	ApproxAreaSqm    float64 `json:"approx_area_sqm"`
}

// ImportDockPose is the dock pose preview.
type ImportDockPose struct {
	X      float64 `json:"x"`
	Y      float64 `json:"y"`
	YawRad float64 `json:"yaw_rad"`
}

// ---------------------------------------------------------------------------
// Constants (kept in sync with gui/web/src/utils/map.tsx)
// ---------------------------------------------------------------------------

// metersPerDegreeLat is the WGS84 1° approximation used throughout
// the GUI for local ENU around the datum. Matches METERS_PER_DEG in
// gui/web/src/utils/map.tsx.
const metersPerDegreeLat = 111319.49079327357

// outlinePolygonAbsBound is the sanity bound on |x| / |y| for any
// outline vertex. OpenMower lawns are ~1 hectare; anything beyond
// 100 km from the local origin is corrupt or in the wrong frame.
const outlinePolygonAbsBound = 100000.0

// ---------------------------------------------------------------------------
// Route registration
// ---------------------------------------------------------------------------

// ImportRoutes registers the /import/* endpoints on the API group.
// Hooked from api.go alongside the other route registrations.
func ImportRoutes(r *gin.RouterGroup, rosProvider types.IRosProvider, dbProvider types.IDBProvider) {
	group := r.Group("/import")
	group.POST("/openmower", postImportOpenMower(rosProvider, dbProvider))
}

// ---------------------------------------------------------------------------
// POST /import/openmower
// ---------------------------------------------------------------------------

// postImportOpenMower is the gin handler. It accepts the user's
// OpenMower map.json (either nested in {"map": {...}} for clients that
// also want to pass om_datum_*, or as a raw JSON body for clients that
// just want to drop the file in) and returns an ImportOpenMowerSummary.
//
// @Summary import an OpenMower map.json (preview-only by default)
// @Description Parse a user-supplied OpenMower map.json, translate it
// @Description into MowgliNext's coordinate frame, and return a summary
// @Description for confirmation. Setting `apply=true` is currently a
// @Description no-op stub — see docs/IMPORT_OPENMOWER_MAP.md.
// @Tags import
// @Accept  json
// @Produce json
// @Param   body body ImportOpenMowerRequest true "import request"
// @Success 200 {object} ImportOpenMowerSummary
// @Failure 400 {object} ErrorResponse
// @Failure 500 {object} ErrorResponse
// @Router /import/openmower [post]
func postImportOpenMower(rosProvider types.IRosProvider, dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		raw, err := io.ReadAll(c.Request.Body)
		if err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "cannot read body: " + err.Error()})
			return
		}

		req, err := parseImportRequest(raw)
		if err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}

		var omMap openMowerMap
		if err := json.Unmarshal(req.Map, &omMap); err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: "invalid OpenMower map.json: " + err.Error()})
			return
		}

		// Resolve MowgliNext datum from mowgli_robot.yaml. When the
		// user passed an OpenMower datum, we'll compute the shift; when
		// they didn't, we stay at zero shift and warn.
		mnLat, mnLon, datumErr := readMowgliNextDatum(dbProvider)
		shiftE, shiftN, datumWarn := computeDatumShift(req.OmDatumLat, req.OmDatumLon, mnLat, mnLon, datumErr)

		summary := buildImportSummary(omMap, shiftE, shiftN)
		if datumWarn != "" {
			summary.Warnings = append([]string{datumWarn}, summary.Warnings...)
		}

		// Build (in memory) the MowgliNext-shaped payload that *would*
		// be POSTed to /map_server_node/add_area, plus the dock pose.
		// Currently logged + counted but not actually written.
		replaceReq, dockReq := buildMowgliNextPayload(omMap, shiftE, shiftN)

		if req.Apply {
			if err := applyImport(c, rosProvider, replaceReq, dockReq); err != nil {
				c.JSON(http.StatusInternalServerError, ErrorResponse{Error: err.Error()})
				return
			}
			summary.Applied = true
		} else {
			logImportPreview(summary, replaceReq, dockReq)
		}

		c.JSON(http.StatusOK, summary)
	}
}

// parseImportRequest accepts three shapes:
//
//  1. **Wrapped modern**: `{"map": {...}, "om_datum_lat": ...}` — the
//     in-app GUI sends this, body already contains `om_datum_*` knobs.
//  2. **Bare modern**: `{"areas": [...], "docking_stations": [...]}` —
//     a user dropping their `map.json` straight into the upload field.
//  3. **Legacy bag-JSON**: `{"WorkingArea": [...], "NavigationAreas":
//     [...], "DockX": ..., "DockY": ..., "DockHeading": ...}` —
//     OpenMower 1.x's mower_map_service bag content serialised to JSON
//     (PascalCase fields with `Package: 0` envelopes). Converted in
//     place to the modern shape so the rest of the pipeline doesn't
//     need to know about it.
func parseImportRequest(raw []byte) (ImportOpenMowerRequest, error) {
	var req ImportOpenMowerRequest
	// 1. Wrapped form.
	if err := json.Unmarshal(raw, &req); err == nil && len(req.Map) > 0 {
		// The wrapped payload may itself contain a legacy body — try
		// converting it; harmless if it's already modern.
		if converted, ok := tryConvertLegacyOpenMowerMap(req.Map); ok {
			req.Map = converted
		}
		return req, nil
	}
	// 2 / 3. Bare form. Probe expected keys for both modern and legacy
	// shapes; reject random JSON that matches neither.
	var probe struct {
		Areas           json.RawMessage `json:"areas"`
		DockingStations json.RawMessage `json:"docking_stations"`
		NavigationAreas json.RawMessage `json:"NavigationAreas"`
		WorkingArea     json.RawMessage `json:"WorkingArea"`
		DockX           *float64        `json:"DockX"`
		DockY           *float64        `json:"DockY"`
	}
	if err := json.Unmarshal(raw, &probe); err != nil {
		return ImportOpenMowerRequest{}, fmt.Errorf("body is not valid JSON: %w", err)
	}
	modernPresent := len(probe.Areas) > 0 || len(probe.DockingStations) > 0
	legacyPresent := len(probe.NavigationAreas) > 0 || len(probe.WorkingArea) > 0 ||
		probe.DockX != nil || probe.DockY != nil
	if !modernPresent && !legacyPresent {
		return ImportOpenMowerRequest{}, errors.New("body has neither {map: ...} envelope nor OpenMower fields (areas, docking_stations) nor legacy bag fields (NavigationAreas, WorkingArea, DockX)")
	}
	if !modernPresent && legacyPresent {
		converted, ok := tryConvertLegacyOpenMowerMap(raw)
		if !ok {
			return ImportOpenMowerRequest{}, errors.New("body looks like a legacy OpenMower bag-JSON map but conversion failed")
		}
		return ImportOpenMowerRequest{Map: converted}, nil
	}
	return ImportOpenMowerRequest{Map: raw}, nil
}

// ---------------------------------------------------------------------------
// Legacy bag-JSON adapter
// ---------------------------------------------------------------------------

// legacyOpenMowerPoint mirrors geometry_msgs/Point32 the way Go's
// rosbridge JSON serialiser emits ROS1 messages (PascalCase fields, with
// each composite carrying a `Package: 0` envelope from the ROS1 msg
// package tagging). We only care about X/Y; Z is always 0 in OpenMower's
// map polygons.
type legacyOpenMowerPoint struct {
	X float64 `json:"X"`
	Y float64 `json:"Y"`
}

// legacyOpenMowerPolygon is a Polygon (geometry_msgs/Polygon).
type legacyOpenMowerPolygon struct {
	Points []legacyOpenMowerPoint `json:"Points"`
}

// legacyOpenMowerMapArea is mower_map/MapArea — one named polygon with
// optional obstacle polygons nested in the same struct. In the modern
// map.json schema obstacles are top-level entries; here they live
// inside their parent.
type legacyOpenMowerMapArea struct {
	Name      string                   `json:"Name"`
	Area      legacyOpenMowerPolygon   `json:"Area"`
	Obstacles []legacyOpenMowerPolygon `json:"Obstacles"`
}

// legacyOpenMowerMap is the top-level bag-JSON shape. Only the fields
// we promote downstream are listed; everything else (Package, MapWidth,
// MapHeight, MapCenter*) is ignored. WorkingArea is plural because the
// legacy schema allowed several disjoint mowing zones; we keep that.
type legacyOpenMowerMap struct {
	NavigationAreas []legacyOpenMowerMapArea `json:"NavigationAreas"`
	WorkingArea     []legacyOpenMowerMapArea `json:"WorkingArea"`
	DockX           *float64                 `json:"DockX"`
	DockY           *float64                 `json:"DockY"`
	DockHeading     *float64                 `json:"DockHeading"`
}

// tryConvertLegacyOpenMowerMap inspects `raw`, and if it's the legacy
// bag-JSON shape (PascalCase NavigationAreas / WorkingArea / Dock*),
// returns a re-serialised modern map.json body that the rest of the
// pipeline can parse. The boolean is true iff a conversion actually
// happened — callers should still treat the original raw as authoritative
// when it's false.
func tryConvertLegacyOpenMowerMap(raw []byte) ([]byte, bool) {
	var legacy legacyOpenMowerMap
	if err := json.Unmarshal(raw, &legacy); err != nil {
		return nil, false
	}
	if len(legacy.NavigationAreas) == 0 && len(legacy.WorkingArea) == 0 &&
		legacy.DockX == nil && legacy.DockY == nil && legacy.DockHeading == nil {
		// Not a legacy payload — let the modern parser handle it.
		return nil, false
	}

	type modernArea struct {
		ID         string                  `json:"id"`
		Properties openMowerAreaProperties `json:"properties"`
		Outline    []openMowerPoint        `json:"outline"`
	}
	type modernDock struct {
		ID         string `json:"id"`
		Properties struct {
			Name   string `json:"name"`
			Active *bool  `json:"active"`
		} `json:"properties"`
		Position openMowerPoint `json:"position"`
		Heading  float64        `json:"heading"`
	}
	type modernMap struct {
		Areas           []modernArea `json:"areas"`
		DockingStations []modernDock `json:"docking_stations"`
	}

	toModernOutline := func(poly legacyOpenMowerPolygon) []openMowerPoint {
		out := make([]openMowerPoint, 0, len(poly.Points))
		for _, p := range poly.Points {
			out = append(out, openMowerPoint{X: p.X, Y: p.Y})
		}
		return out
	}

	out := modernMap{}

	// Working areas are mow areas; their nested Obstacles become
	// top-level type="obstacle" entries (matchObstaclesToParents will
	// re-attach them to the right parent via centroid containment).
	for i, w := range legacy.WorkingArea {
		name := w.Name
		if name == "" {
			name = fmt.Sprintf("Mowing area %d", i+1)
		}
		out.Areas = append(out.Areas, modernArea{
			ID:         fmt.Sprintf("legacy-mow-%02d", i+1),
			Properties: openMowerAreaProperties{Name: name, Type: "mow"},
			Outline:    toModernOutline(w.Area),
		})
		for j, ob := range w.Obstacles {
			out.Areas = append(out.Areas, modernArea{
				ID: fmt.Sprintf("legacy-mow-%02d-obs-%02d", i+1, j+1),
				Properties: openMowerAreaProperties{
					Name: fmt.Sprintf("%s obstacle %d", name, j+1),
					Type: "obstacle",
				},
				Outline: toModernOutline(ob),
			})
		}
	}

	for i, n := range legacy.NavigationAreas {
		name := n.Name
		if name == "" {
			name = fmt.Sprintf("Navigation area %d", i+1)
		}
		out.Areas = append(out.Areas, modernArea{
			ID:         fmt.Sprintf("legacy-nav-%02d", i+1),
			Properties: openMowerAreaProperties{Name: name, Type: "nav"},
			Outline:    toModernOutline(n.Area),
		})
		// Navigation areas can also carry obstacles in the bag schema;
		// keep them for parity even though obstacles inside nav zones
		// are unusual.
		for j, ob := range n.Obstacles {
			out.Areas = append(out.Areas, modernArea{
				ID: fmt.Sprintf("legacy-nav-%02d-obs-%02d", i+1, j+1),
				Properties: openMowerAreaProperties{
					Name: fmt.Sprintf("%s obstacle %d", name, j+1),
					Type: "obstacle",
				},
				Outline: toModernOutline(ob),
			})
		}
	}

	if legacy.DockX != nil && legacy.DockY != nil {
		yaw := 0.0
		if legacy.DockHeading != nil {
			yaw = *legacy.DockHeading
		}
		d := modernDock{
			ID:       "legacy-dock-01",
			Position: openMowerPoint{X: *legacy.DockX, Y: *legacy.DockY},
			Heading:  yaw,
		}
		d.Properties.Name = "Docking Station"
		out.DockingStations = append(out.DockingStations, d)
	}

	converted, err := json.Marshal(out)
	if err != nil {
		return nil, false
	}
	return converted, true
}

// ---------------------------------------------------------------------------
// Datum handling
// ---------------------------------------------------------------------------

// readMowgliNextDatum pulls datum_lat / datum_lon from mowgli_robot.yaml.
// Mirrors the lookup in diagnostics.go so the importer doesn't depend
// on an /api round-trip.
func readMowgliNextDatum(dbProvider types.IDBProvider) (float64, float64, error) {
	if dbProvider == nil {
		return 0, 0, errors.New("no db provider")
	}
	configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
	if err != nil {
		return 0, 0, err
	}
	data, err := readFileOrEmpty(string(configFilePath))
	if err != nil {
		return 0, 0, err
	}
	if len(data) == 0 {
		return 0, 0, errors.New("mowgli_robot.yaml is empty or missing")
	}
	yamlData, err := parseYAMLToMap(data)
	if err != nil {
		return 0, 0, err
	}
	lat := extractYAMLFloat(yamlData, "datum_lat")
	lon := extractYAMLFloat(yamlData, "datum_lon")
	if lat == 0 && lon == 0 {
		return 0, 0, errors.New("datum_lat / datum_lon not set in mowgli_robot.yaml")
	}
	return lat, lon, nil
}

// computeDatumShift returns the (east, north) shift in metres that the
// importer must add to every OpenMower polygon vertex to land it in
// the MowgliNext map frame, plus a warning string when the inputs
// don't allow a confident answer.
func computeDatumShift(omLat, omLon *float64, mnLat, mnLon float64, mnDatumErr error) (float64, float64, string) {
	if omLat == nil || omLon == nil {
		return 0, 0, "OpenMower datum not provided — assuming same datum as MowgliNext (identity transform). If the source robot used a different OM_DATUM_LAT/OM_DATUM_LONG, supply om_datum_lat/om_datum_lon and re-import."
	}
	if mnDatumErr != nil {
		return 0, 0, fmt.Sprintf("OpenMower datum provided but MowgliNext datum unreadable (%v) — applying identity transform. Configure MowgliNext datum_lat/datum_lon first.", mnDatumErr)
	}
	cosLat := math.Cos(mnLat * math.Pi / 180.0)
	shiftE := (*omLon - mnLon) * cosLat * metersPerDegreeLat
	shiftN := (*omLat - mnLat) * metersPerDegreeLat
	return shiftE, shiftN, ""
}

// ---------------------------------------------------------------------------
// Summary + payload builders
// ---------------------------------------------------------------------------

// buildImportSummary walks the parsed OpenMower map, applies the datum
// shift, validates each area, and produces the user-facing preview.
func buildImportSummary(omMap openMowerMap, shiftE, shiftN float64) ImportOpenMowerSummary {
	summary := ImportOpenMowerSummary{
		Warnings:         []string{},
		Areas:            []ImportedAreaBrief{},
		DatumShiftEastM:  shiftE,
		DatumShiftNorthM: shiftN,
	}

	mowAndNav := []openMowerArea{}
	obstacles := []openMowerArea{}

	for _, a := range omMap.Areas {
		if a.Properties.Active != nil && !*a.Properties.Active {
			summary.Warnings = append(summary.Warnings, fmt.Sprintf("area %q (id=%s) is inactive — skipped", a.Properties.Name, shortID(a.ID)))
			continue
		}
		a.Outline = dedupOutline(a.Outline)
		switch strings.ToLower(a.Properties.Type) {
		case "mow", "nav":
			if !validateOutline(a, &summary) {
				continue
			}
			mowAndNav = append(mowAndNav, a)
		case "obstacle":
			if !validateOutline(a, &summary) {
				continue
			}
			obstacles = append(obstacles, a)
		case "draft", "":
			summary.Warnings = append(summary.Warnings, fmt.Sprintf("area %q (id=%s) has type=%q — skipped (drafts are not imported)", a.Properties.Name, shortID(a.ID), a.Properties.Type))
		default:
			summary.Warnings = append(summary.Warnings, fmt.Sprintf("area %q (id=%s) has unknown type=%q — skipped", a.Properties.Name, shortID(a.ID), a.Properties.Type))
		}
	}

	// Per-area summary + obstacle re-parenting.
	obstacleAssign := matchObstaclesToParents(obstacles, mowAndNav, &summary)
	for i, a := range mowAndNav {
		isNav := strings.EqualFold(a.Properties.Type, "nav")
		brief := ImportedAreaBrief{
			Name:             a.Properties.Name,
			Type:             a.Properties.Type,
			Vertices:         len(a.Outline),
			Obstacles:        len(obstacleAssign[i]),
			IsNavigationArea: isNav,
			ApproxAreaSqm:    polygonAreaSqm(a.Outline),
		}
		summary.Areas = append(summary.Areas, brief)
		if isNav {
			summary.NavigationAreas++
		} else {
			summary.MowingAreas++
		}
		summary.Obstacles += len(obstacleAssign[i])
	}

	if summary.MowingAreas == 0 {
		summary.Warnings = append(summary.Warnings, "no active mowing areas in this map — MowgliNext requires at least one to mow.")
	}

	// Dock pose: take the first active station, warn on extras.
	activeDocks := []openMowerDockingStation{}
	for _, d := range omMap.DockingStations {
		if d.Properties.Active != nil && !*d.Properties.Active {
			continue
		}
		activeDocks = append(activeDocks, d)
	}
	if len(activeDocks) > 1 {
		summary.Warnings = append(summary.Warnings, fmt.Sprintf("%d active docking stations in source map; MowgliNext supports one — first wins.", len(activeDocks)))
	}
	if len(activeDocks) >= 1 {
		d := activeDocks[0]
		summary.DockPose = &ImportDockPose{
			X:      d.Position.X + shiftE,
			Y:      d.Position.Y + shiftN,
			YawRad: d.Heading,
		}
	}

	return summary
}

// validateOutline performs the per-polygon sanity checks. Returns
// false (and appends a warning) when the area should be dropped.
func validateOutline(a openMowerArea, summary *ImportOpenMowerSummary) bool {
	if len(a.Outline) < 3 {
		summary.Warnings = append(summary.Warnings, fmt.Sprintf("area %q (id=%s) has only %d vertices — skipped (need ≥3).", a.Properties.Name, shortID(a.ID), len(a.Outline)))
		return false
	}
	for _, p := range a.Outline {
		if math.IsNaN(p.X) || math.IsNaN(p.Y) || math.IsInf(p.X, 0) || math.IsInf(p.Y, 0) {
			summary.Warnings = append(summary.Warnings, fmt.Sprintf("area %q (id=%s) has non-finite vertex — skipped.", a.Properties.Name, shortID(a.ID)))
			return false
		}
		if math.Abs(p.X) > outlinePolygonAbsBound || math.Abs(p.Y) > outlinePolygonAbsBound {
			summary.Warnings = append(summary.Warnings, fmt.Sprintf("area %q (id=%s) has out-of-range vertex (%.1f, %.1f) — skipped (frame mismatch?).", a.Properties.Name, shortID(a.ID), p.X, p.Y))
			return false
		}
	}
	return true
}

// matchObstaclesToParents assigns each obstacle to the first
// mow/nav area whose polygon contains the obstacle's centroid.
// Returns a map keyed by parent-area-index → list of obstacle indices
// (into the obstacles slice).
func matchObstaclesToParents(obstacles, parents []openMowerArea, summary *ImportOpenMowerSummary) map[int][]int {
	out := map[int][]int{}
	for oi, ob := range obstacles {
		cx, cy := centroid(ob.Outline)
		matched := -1
		ambiguous := false
		for pi, p := range parents {
			if pointInPolygon(cx, cy, p.Outline) {
				if matched != -1 {
					ambiguous = true
					break
				}
				matched = pi
			}
		}
		if matched == -1 {
			summary.OrphanObstacles++
			summary.Warnings = append(summary.Warnings, fmt.Sprintf("obstacle id=%s centroid (%.2f, %.2f) does not fall inside any mow/nav area — skipped.", shortID(ob.ID), cx, cy))
			continue
		}
		if ambiguous {
			summary.Warnings = append(summary.Warnings, fmt.Sprintf("obstacle id=%s centroid is inside multiple areas — assigned to first match (%q).", shortID(ob.ID), parents[matched].Properties.Name))
		}
		out[matched] = append(out[matched], oi)
	}
	return out
}

// buildMowgliNextPayload constructs the (unsent) ReplaceMapReq + dock
// pose request that the apply path will eventually use. Produced even
// in preview mode so logging shows exactly what would be written.
func buildMowgliNextPayload(omMap openMowerMap, shiftE, shiftN float64) (*mowgli.ReplaceMapReq, *mowgli.SetDockingPointReq) {
	// Re-walk the validated areas (cheap; smaller than the duplication
	// cost of returning more from buildImportSummary).
	mowAndNav := []openMowerArea{}
	obstacles := []openMowerArea{}
	for _, a := range omMap.Areas {
		if a.Properties.Active != nil && !*a.Properties.Active {
			continue
		}
		a.Outline = dedupOutline(a.Outline)
		switch strings.ToLower(a.Properties.Type) {
		case "mow", "nav":
			if len(a.Outline) >= 3 {
				mowAndNav = append(mowAndNav, a)
			}
		case "obstacle":
			if len(a.Outline) >= 3 {
				obstacles = append(obstacles, a)
			}
		}
	}

	// Quick re-match of obstacles to parents (no warnings here — those
	// already went into the summary).
	obstacleAssign := map[int][]int{}
	for oi, ob := range obstacles {
		cx, cy := centroid(ob.Outline)
		for pi, p := range mowAndNav {
			if pointInPolygon(cx, cy, p.Outline) {
				obstacleAssign[pi] = append(obstacleAssign[pi], oi)
				break
			}
		}
	}

	replaceReq := &mowgli.ReplaceMapReq{Areas: []mowgli.ReplaceMapArea{}}
	for pi, a := range mowAndNav {
		isNav := strings.EqualFold(a.Properties.Type, "nav")
		obs := []geometry.Polygon{}
		for _, oi := range obstacleAssign[pi] {
			obs = append(obs, geometry.Polygon{Points: outlineToPoint32s(obstacles[oi].Outline, shiftE, shiftN)})
		}
		mapArea := mowgli.MapArea{
			Name:      a.Properties.Name,
			Area:      geometry.Polygon{Points: outlineToPoint32s(a.Outline, shiftE, shiftN)},
			Obstacles: obs,
		}
		replaceReq.Areas = append(replaceReq.Areas, mowgli.ReplaceMapArea{
			Area:             mapArea,
			IsNavigationArea: isNav,
		})
	}

	var dockReq *mowgli.SetDockingPointReq
	for _, d := range omMap.DockingStations {
		if d.Properties.Active != nil && !*d.Properties.Active {
			continue
		}
		dockReq = &mowgli.SetDockingPointReq{
			DockingPose: geometry.Pose{
				Position:    geometry.Point{X: d.Position.X + shiftE, Y: d.Position.Y + shiftN, Z: 0},
				Orientation: yawToQuaternion(d.Heading),
			},
		}
		break
	}

	return replaceReq, dockReq
}

// applyImport runs the live write path: clear_map → add_area×N →
// save_areas, then (if a dock pose came through) set_docking_point.
//
// Replace-mode only. Mirrors the same service sequence used by the
// HTTP-driven ReplaceMapRoute + SetDockingPointRoute via the shared
// helpers in mowglinext.go, so the importer and the regular save path
// can't drift apart on what "save" means. A failure mid-sequence
// surfaces as a 500 to the caller; areas already written before the
// failure stay in `map_server_node`'s in-memory state (the same
// partial-write semantics the HTTP route has — there is no transaction).
func applyImport(c *gin.Context, rosProvider types.IRosProvider, replaceReq *mowgli.ReplaceMapReq, dockReq *mowgli.SetDockingPointReq) error {
	if rosProvider == nil {
		return errors.New("apply: no ROS provider")
	}
	if replaceReq == nil {
		return errors.New("apply: nil replace request")
	}
	// Reuse the gin request context for cancellation propagation, but
	// give the whole sequence its own 30 s budget — same envelope as
	// ReplaceMapRoute's timeout. save_areas does disk IO; add_area on a
	// large polygon walk can take ~1 s each.
	ctx, cancel := context.WithTimeout(c.Request.Context(), 30*time.Second)
	defer cancel()

	logImportf("import/openmower applying: %d areas, dock=%v", len(replaceReq.Areas), dockReq != nil)
	if err := replaceMapInternal(ctx, rosProvider, replaceReq); err != nil {
		return fmt.Errorf("replace map: %w", err)
	}
	if dockReq != nil {
		if err := setDockingPointInternal(ctx, rosProvider, dockReq); err != nil {
			return fmt.Errorf("set docking point: %w", err)
		}
	}
	return nil
}

// logImportPreview emits a structured summary of what the import would
// have written. Used both in preview mode (default) and as a dry-run
// trace inside applyImport.
func logImportPreview(summary ImportOpenMowerSummary, replaceReq *mowgli.ReplaceMapReq, dockReq *mowgli.SetDockingPointReq) {
	areaCount := 0
	if replaceReq != nil {
		areaCount = len(replaceReq.Areas)
	}
	hasDock := dockReq != nil
	logImportf("import/openmower preview: %d areas (%d mow, %d nav, %d obstacles, %d orphan), dock=%v, datum_shift=(%.3f, %.3f) m, warnings=%d",
		areaCount,
		summary.MowingAreas,
		summary.NavigationAreas,
		summary.Obstacles,
		summary.OrphanObstacles,
		hasDock,
		summary.DatumShiftEastM,
		summary.DatumShiftNorthM,
		len(summary.Warnings),
	)
	for _, w := range summary.Warnings {
		logImportf("import/openmower warning: %s", w)
	}
	logImportf("import/openmower would call: PUT /api/mowglinext/map (%d areas) + POST /api/mowglinext/map/docking (%v)",
		areaCount, hasDock)
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

// dedupOutline removes consecutive duplicate vertices and a wrap-around
// closing duplicate (rings are stored open). OpenMower exports routinely
// start a ring with a doubled vertex (points[0] == points[1]) and/or
// close it with a repeat of the first point. Both create zero-length
// edges that make the polygon non-simple. boost::geometry (under the F2C
// v3 coverage pipeline and map_server's grid_map polygon stamping)
// rejects a zero-length segment, so an affected area imports into the
// preview but is silently dropped by map_server — which is why areas
// whose *first* edge is degenerate fail to import. Normalising here keeps
// the rest of the pipeline (validation, centroid matching, the written
// payload) working on clean geometry.
func dedupOutline(in []openMowerPoint) []openMowerPoint {
	if len(in) == 0 {
		return in
	}
	out := make([]openMowerPoint, 0, len(in))
	for _, p := range in {
		if n := len(out); n > 0 && p == out[n-1] {
			continue
		}
		out = append(out, p)
	}
	// Drop wrap-around closing duplicate(s) so the final edge back to the
	// first vertex isn't zero-length.
	for len(out) > 1 && out[0] == out[len(out)-1] {
		out = out[:len(out)-1]
	}
	return out
}

// outlineToPoint32s materialises an OpenMower outline as the geometry
// _msgs/Polygon.Points list expected by mowgli.MapArea.
func outlineToPoint32s(outline []openMowerPoint, shiftE, shiftN float64) []geometry.Point32 {
	out := make([]geometry.Point32, 0, len(outline))
	for _, p := range outline {
		out = append(out, geometry.Point32{
			X: float32(p.X + shiftE),
			Y: float32(p.Y + shiftN),
			Z: 0,
		})
	}
	return out
}

// pointInPolygon does a winding-number test. Polygon may be open
// (first/last vertex distinct) or closed; both work.
func pointInPolygon(x, y float64, poly []openMowerPoint) bool {
	inside := false
	n := len(poly)
	if n < 3 {
		return false
	}
	for i, j := 0, n-1; i < n; j, i = i, i+1 {
		xi, yi := poly[i].X, poly[i].Y
		xj, yj := poly[j].X, poly[j].Y
		intersect := ((yi > y) != (yj > y)) && (x < (xj-xi)*(y-yi)/(yj-yi)+xi)
		if intersect {
			inside = !inside
		}
	}
	return inside
}

// centroid is the geometric centroid of an open polygon (good enough
// for "which area contains this obstacle?" — we don't need the exact
// area-weighted centroid for that).
func centroid(poly []openMowerPoint) (float64, float64) {
	if len(poly) == 0 {
		return 0, 0
	}
	var sx, sy float64
	for _, p := range poly {
		sx += p.X
		sy += p.Y
	}
	n := float64(len(poly))
	return sx / n, sy / n
}

// polygonAreaSqm uses the shoelace formula. Result is always positive
// (we don't care about winding order for the preview).
func polygonAreaSqm(poly []openMowerPoint) float64 {
	if len(poly) < 3 {
		return 0
	}
	var s float64
	n := len(poly)
	for i := 0; i < n; i++ {
		j := (i + 1) % n
		s += poly[i].X*poly[j].Y - poly[j].X*poly[i].Y
	}
	return math.Abs(s) * 0.5
}

// yawToQuaternion converts a 2D yaw (radians, +z) to a Quaternion
// matching the convention used by gui/web/src/utils/map.tsx
// (`getQuaternionFromHeading`). Pure rotation about Z.
func yawToQuaternion(yaw float64) geometry.Quaternion {
	half := yaw * 0.5
	return geometry.Quaternion{
		X: 0,
		Y: 0,
		Z: math.Sin(half),
		W: math.Cos(half),
	}
}

// shortID truncates an OpenMower nano-id for log output (full ID is
// noise; first 8 chars are plenty to disambiguate inside one map).
func shortID(id string) string {
	if len(id) <= 8 {
		return id
	}
	return id[:8]
}

// ---------------------------------------------------------------------------
// Tiny IO helpers (kept indirected via vars so the test file can stub
// them without dragging in a DB provider).
// ---------------------------------------------------------------------------

// readFileOrEmpty returns the file's bytes, or nil with no error when
// the path is empty (lets callers default-skip cleanly).
func readFileOrEmpty(path string) ([]byte, error) {
	if path == "" {
		return nil, nil
	}
	return osReadFile(path)
}

// parseYAMLToMap is the YAML decode used to dig datum_lat/lon out of
// mowgli_robot.yaml. Indirected for the same testing reason.
func parseYAMLToMap(data []byte) (map[string]interface{}, error) {
	var out map[string]interface{}
	if err := yaml.Unmarshal(data, &out); err != nil {
		return nil, err
	}
	return out, nil
}

// osReadFile is a var so tests can stub it out without cross-package
// dependencies. Default is the real os.ReadFile.
var osReadFile = os.ReadFile

// logImportf is the structured-log indirection. It's a `var` so the
// test file can capture log output without touching the global logger.
var logImportf = func(format string, args ...any) {
	log.Printf("[import/openmower] "+format, args...)
}
