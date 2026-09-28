package providers

import (
	"encoding/json"
	"math"
	"sort"
	"time"
)

// Pure decision logic of the fleet coordinator (docs/MULTI_ROBOT.md § 3b/3c).
// Everything here is a function of the fleet snapshot and a little remembered
// state, so it is unit-tested without ROS, HTTP or timers.

// metersPerDegreeLat is the WGS84 1° approximation the map importer and the
// browser use (gui/pkg/api/openmower_import.go, web/src/utils/map.tsx).
const metersPerDegreeLat = 111319.49079327357

// datumEpsilonDeg is map_server's own "same datum" tolerance
// (mowgli_map area_manager.cpp, 1e-8°): two robots must agree to this
// precision or one of them re-projects the shared map on load.
const datumEpsilonDeg = 1e-8

// CoordinatorSettings is the operator-facing configuration, DB key
// fleet.coordination (JSON). Defaults are what a two-mower lawn needs.
type CoordinatorSettings struct {
	Enabled           bool    `json:"enabled"`
	YieldDistanceM    float64 `json:"yield_distance_m"`
	ResumeDistanceM   float64 `json:"resume_distance_m"`
	CompletedTTLHours float64 `json:"completed_ttl_h"`
}

// DefaultCoordinatorSettings: coordination OFF until the operator turns it on.
func DefaultCoordinatorSettings() CoordinatorSettings {
	return CoordinatorSettings{
		Enabled:           false,
		YieldDistanceM:    3.0,
		ResumeDistanceM:   5.0,
		CompletedTTLHours: 12.0,
	}
}

// Normalize fills zero/negative numbers with the defaults and keeps the
// resume distance above the yield distance (hysteresis).
func (s CoordinatorSettings) Normalize() CoordinatorSettings {
	d := DefaultCoordinatorSettings()
	out := s
	if out.YieldDistanceM <= 0 {
		out.YieldDistanceM = d.YieldDistanceM
	}
	if out.ResumeDistanceM <= out.YieldDistanceM {
		out.ResumeDistanceM = out.YieldDistanceM + 2.0
	}
	if out.CompletedTTLHours <= 0 {
		out.CompletedTTLHours = d.CompletedTTLHours
	}
	return out
}

// LatLon is a WGS84 position.
type LatLon struct {
	Lat float64
	Lon float64
}

// fleetMember is the slice of a FleetRobot the coordinator reasons about.
type fleetMember struct {
	ID            string
	Self          bool
	Online        bool
	Autonomous    bool // HighLevelStatus.state == AUTONOMOUS (2)
	StateName     string
	CurrentArea   int // -1 when none
	Completed     []uint32
	SessionActive bool
	Pos           *LatLon
}

// FleetAssignment is what gets pushed to the local BT.
type FleetAssignment struct {
	Excluded       []uint32
	PreferredStart int32
}

const hlStateAutonomous = 2

// memberFromRobot derives a fleetMember from a snapshot row (self included).
func memberFromRobot(r FleetRobot) fleetMember {
	m := fleetMember{ID: r.Identity.ID, Self: r.Self, Online: r.Online, CurrentArea: -1}
	var hl struct {
		State       *int   `json:"state"`
		StateName   string `json:"state_name"`
		CurrentArea *int   `json:"current_area"`
	}
	if raw, ok := r.Topics["highLevelStatus"]; ok && json.Unmarshal(raw, &hl) == nil {
		m.Autonomous = hl.State != nil && *hl.State == hlStateAutonomous
		m.StateName = hl.StateName
		if hl.CurrentArea != nil {
			m.CurrentArea = *hl.CurrentArea
		}
	}
	var session struct {
		SessionActive  bool     `json:"session_active"`
		CurrentArea    *int     `json:"current_area"`
		CompletedAreas []uint32 `json:"completed_areas"`
	}
	if raw, ok := r.Topics["coverageSession"]; ok && json.Unmarshal(raw, &session) == nil {
		m.SessionActive = session.SessionActive
		m.Completed = session.CompletedAreas
		// The session topic is the BT's own bookkeeping; prefer it for the
		// area index when both are present.
		if session.CurrentArea != nil {
			m.CurrentArea = *session.CurrentArea
		}
	}
	// The "gps" key is the adapted AbsolutePose: latitude in pose.position.x,
	// longitude in pose.position.y (transform.go adaptGPS).
	var gps struct {
		Pose struct {
			Pose struct {
				Position struct {
					X float64 `json:"x"`
					Y float64 `json:"y"`
				} `json:"position"`
			} `json:"pose"`
		} `json:"pose"`
	}
	if raw, ok := r.Topics["gps"]; ok && json.Unmarshal(raw, &gps) == nil {
		lat, lon := gps.Pose.Pose.Position.X, gps.Pose.Pose.Position.Y
		if lat != 0 || lon != 0 {
			m.Pos = &LatLon{Lat: lat, Lon: lon}
		}
	}
	return m
}

// splitMembers returns (self, online peers) from the derived members.
func splitMembers(members []fleetMember) (fleetMember, []fleetMember) {
	var self fleetMember
	peers := make([]fleetMember, 0, len(members))
	for _, m := range members {
		if m.Self {
			self = m
			continue
		}
		if m.Online {
			peers = append(peers, m)
		}
	}
	return self, peers
}

// hasPriorityOver: the robot with the smaller id wins every tie-break
// (same area, close approach). Ids are UUIDs, so this is arbitrary but
// total and stable — which is all a tie-break needs.
func hasPriorityOver(a, b string) bool {
	return a < b
}

// computeAssignment decides which areas the LOCAL robot must leave alone:
// every area a peer is mowing right now, plus every area any member finished
// this fleet session (completedMemory). The local robot's own current area is
// only excluded when a peer with priority claims it (then WE yield); when we
// have priority the peer yields instead. preferred_start is our rank among
// the online members so idle robots fan out instead of all picking area 0.
func computeAssignment(self fleetMember, peers []fleetMember, completedMemory map[uint32]time.Time) FleetAssignment {
	excluded := map[uint32]bool{}
	for area := range completedMemory {
		excluded[area] = true
	}
	for _, p := range peers {
		if !p.Autonomous || p.CurrentArea < 0 {
			continue
		}
		area := uint32(p.CurrentArea)
		if self.Autonomous && self.CurrentArea == p.CurrentArea && hasPriorityOver(self.ID, p.ID) {
			continue // the peer yields, not us
		}
		excluded[area] = true
	}
	list := make([]uint32, 0, len(excluded))
	for area := range excluded {
		list = append(list, area)
	}
	sort.Slice(list, func(i, j int) bool { return list[i] < list[j] })

	ids := []string{self.ID}
	for _, p := range peers {
		ids = append(ids, p.ID)
	}
	sort.Strings(ids)
	rank := int32(sort.SearchStrings(ids, self.ID))
	return FleetAssignment{Excluded: list, PreferredStart: rank}
}

// updateCompletedMemory returns a NEW memory map: every area any member
// reports completed is stamped `now`, and entries older than ttl expire.
func updateCompletedMemory(memory map[uint32]time.Time, members []fleetMember, now time.Time, ttl time.Duration) map[uint32]time.Time {
	out := make(map[uint32]time.Time, len(memory))
	for area, seen := range memory {
		if now.Sub(seen) <= ttl {
			out[area] = seen
		}
	}
	for _, m := range members {
		if !m.Online {
			continue
		}
		for _, area := range m.Completed {
			out[area] = now
		}
	}
	return out
}

// distanceM is the equirectangular ground distance between two fixes.
func distanceM(a, b LatLon) float64 {
	cosLat := math.Cos((a.Lat + b.Lat) / 2 * math.Pi / 180)
	dx := (b.Lon - a.Lon) * metersPerDegreeLat * cosLat
	dy := (b.Lat - a.Lat) * metersPerDegreeLat
	return math.Hypot(dx, dy)
}

// enuFromDatum projects a fix into the map frame of a robot with the given
// datum (X east, Y north, same equirectangular math as the localizer).
func enuFromDatum(datum, p LatLon) (x, y float64) {
	x = (p.Lon - datum.Lon) * metersPerDegreeLat * math.Cos(datum.Lat*math.Pi/180)
	y = (p.Lat - datum.Lat) * metersPerDegreeLat
	return x, y
}

// sameDatum: within map_server's re-projection tolerance.
func sameDatum(a, b LatLon) bool {
	return math.Abs(a.Lat-b.Lat) <= datumEpsilonDeg && math.Abs(a.Lon-b.Lon) <= datumEpsilonDeg
}

// yieldState is the coordinator's memory of the proximity rule.
type yieldState struct {
	Yielded    bool
	ClearSince time.Time
}

// YieldAction is what the proximity rule wants sent to the local BT.
type YieldAction string

const (
	YieldNone  YieldAction = ""
	YieldStop  YieldAction = "stop"  // COMMAND_STOP (8): hold in place
	YieldStart YieldAction = "start" // COMMAND_START (1): resume at the cursor
)

const yieldResumeHold = 3 * time.Second

// decideYield implements the mutual-avoidance rule: when a peer WITH PRIORITY
// is autonomous within yield_distance of us while we are autonomous, we stop
// in place; once no such peer has been within resume_distance for a few
// seconds, we resume. A robot that stopped for another reason (operator
// Pause, Home, charging) is never resumed by this rule: we only resume what
// we ourselves stopped, and only while the robot still sits where we left it
// (IDLE with our stop reason still standing).
func decideYield(self fleetMember, peers []fleetMember, st yieldState, s CoordinatorSettings, now time.Time) (yieldState, YieldAction) {
	if self.Pos == nil {
		return st, YieldNone
	}
	nearest := math.Inf(1)
	for _, p := range peers {
		if !p.Autonomous || p.Pos == nil || !hasPriorityOver(p.ID, self.ID) {
			continue
		}
		if d := distanceM(*self.Pos, *p.Pos); d < nearest {
			nearest = d
		}
	}
	if !st.Yielded {
		if self.Autonomous && nearest < s.YieldDistanceM {
			return yieldState{Yielded: true}, YieldStop
		}
		return st, YieldNone
	}
	// We are holding. If the operator moved the robot on (it is autonomous
	// again, or docked / charging), our hold is over without a resume.
	if self.Autonomous || self.StateName == "CHARGING" || self.StateName == "MANUAL_CHARGING" || self.StateName == "RETURNING_HOME" {
		return yieldState{}, YieldNone
	}
	if nearest < s.ResumeDistanceM {
		return yieldState{Yielded: true}, YieldNone
	}
	if st.ClearSince.IsZero() {
		return yieldState{Yielded: true, ClearSince: now}, YieldNone
	}
	if now.Sub(st.ClearSince) < yieldResumeHold {
		return st, YieldNone
	}
	return yieldState{}, YieldStart
}

// assignmentEqual compares two assignments.
func assignmentEqual(a, b FleetAssignment) bool {
	if a.PreferredStart != b.PreferredStart || len(a.Excluded) != len(b.Excluded) {
		return false
	}
	for i := range a.Excluded {
		if a.Excluded[i] != b.Excluded[i] {
			return false
		}
	}
	return true
}
