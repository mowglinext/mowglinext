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
	// Longest a yielded robot holds before resuming regardless of the peer:
	// the deadlock breaker when the priority robot stopped in front of it
	// (its own collision checks) and neither will move first. Both robots
	// carry each other in their costmaps, so a resumed robot skirts the
	// stopped one instead of hitting it.
	YieldMaxHoldS float64 `json:"yield_max_hold_s"`
	// After any resume, no new yield for this long: lets the resumed robot
	// actually clear the peer instead of stop/start flapping at 3 m.
	YieldCooldownS float64 `json:"yield_cooldown_s"`
}

// DefaultCoordinatorSettings: coordination OFF until the operator turns it on.
func DefaultCoordinatorSettings() CoordinatorSettings {
	return CoordinatorSettings{
		Enabled:           false,
		YieldDistanceM:    3.0,
		ResumeDistanceM:   5.0,
		CompletedTTLHours: 12.0,
		YieldMaxHoldS:     45.0,
		YieldCooldownS:    20.0,
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
	if out.YieldMaxHoldS <= 0 {
		out.YieldMaxHoldS = d.YieldMaxHoldS
	}
	if out.YieldCooldownS <= 0 {
		out.YieldCooldownS = d.YieldCooldownS
	}
	return out
}

// LatLon is a WGS84 position.
type LatLon struct {
	Lat float64
	Lon float64
}

// XY is a position in THIS robot's map frame (metres, x east, y north).
type XY struct {
	X float64
	Y float64
}

// fleetMember is the slice of a FleetRobot the coordinator reasons about.
type fleetMember struct {
	ID                 string
	Self               bool
	Online             bool
	Autonomous         bool // HighLevelStatus.state == AUTONOMOUS (2)
	StopHold           bool // state == IDLE (1) and state_name == IDLE, as published after STOP
	StateName          string
	CurrentArea        int // -1 when none
	Completed          []uint32
	SessionActive      bool
	SessionState       coverageState
	SessionObservation *coverageObservation
	// Fresh position in our map frame (fused pose first, antenna fix as the
	// fallback — see peerMapPoses); nil when unknown or stale.
	Pos *XY
}

// FleetAssignment is what gets pushed to the local BT.
type FleetAssignment struct {
	Excluded       []uint32
	PreferredStart int32
}

const hlStateAutonomous = 2

// memberFromRobotAt derives a fleetMember from a snapshot row (self
// included), placing it in the map frame anchored at selfDatum. A zero
// selfDatum means "no map frame yet": every position stays nil.
func memberFromRobotAt(r FleetRobot, selfDatum LatLon, maxAge time.Duration) fleetMember {
	m := fleetMember{ID: r.Identity.ID, Self: r.Self, Online: r.Online, CurrentArea: -1}
	var hl struct {
		State       *int   `json:"state"`
		StateName   string `json:"state_name"`
		CurrentArea *int   `json:"current_area"`
	}
	hlValid := json.Unmarshal(r.Topics["highLevelStatus"], &hl) == nil
	if hlValid {
		m.Autonomous = hl.State != nil && *hl.State == hlStateAutonomous
		m.StateName = hl.StateName
		m.StopHold = hl.State != nil && *hl.State == 1 && hl.StateName == "IDLE"
		if hl.CurrentArea != nil {
			m.CurrentArea = *hl.CurrentArea
		}
	}
	m.SessionState = coverageAbsent
	if raw, ok := r.Topics["coverageSession"]; ok {
		m.SessionState = coverageUnknown
		var session struct {
			SessionActive  *bool     `json:"session_active"`
			CurrentArea    *int      `json:"current_area"`
			CompletedAreas *[]uint32 `json:"completed_areas"`
		}
		age, ageKnown := r.TopicAgeS["coverageSession"]
		switch {
		case !r.Online || r.Identity.ID == "" || !ageKnown || math.IsNaN(age) || math.IsInf(age, 0) || age < 0:
		case age > peerOnlineMaxAge.Seconds():
			m.SessionState = coverageStale
		case r.coverageObservation == nil || r.coverageObservation.receivedAt.IsZero():
		case json.Unmarshal(raw, &session) != nil || session.SessionActive == nil || session.CurrentArea == nil || session.CompletedAreas == nil || *session.CurrentArea < -1:
		default:
			m.SessionState = coverageFresh
			hlAge, hlAgeKnown := r.TopicAgeS["highLevelStatus"]
			hlFresh := hlAgeKnown && hlAge >= 0 && hlAge <= peerOnlineMaxAge.Seconds()
			if hlValid && hlFresh && hl.CurrentArea != nil && *hl.CurrentArea != *session.CurrentArea {
				// No producer sequence orders these two 1 Hz topics. Retain
				// HLS and do not import contradictory completion bookkeeping.
				m.SessionState = coverageContradictory
				break
			}
			m.SessionActive = *session.SessionActive
			m.Completed = *session.CompletedAreas
			m.SessionObservation = r.coverageObservation
			// A known HLS area wins unless it is explicitly stale. Missing
			// HLS freshness is not evidence of a newer session observation.
			if !hlValid || hl.CurrentArea == nil || (hlAgeKnown && hlAge > peerOnlineMaxAge.Seconds()) {
				m.CurrentArea = *session.CurrentArea
			}
		}
	}
	if selfDatum.Lat == 0 && selfDatum.Lon == 0 {
		return m
	}
	// Same placement as the costmap feed, so the yield rule and the
	// obstacle the other robot sees agree.
	if p, ok := fusedMapPose(r, selfDatum, maxAge); ok {
		m.Pos = &XY{X: p.X, Y: p.Y}
	} else if p, ok := fixMapPose(r, selfDatum, maxAge); ok {
		m.Pos = &XY{X: p.X, Y: p.Y}
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

// coverageState distinguishes absence, unknown provenance, expiry and a
// contradiction with fresh HLS. None of them is a new completed-area report.
type coverageState string

const (
	coverageAbsent        coverageState = "absent"
	coverageUnknown       coverageState = "unknown"
	coverageStale         coverageState = "stale"
	coverageFresh         coverageState = "fresh"
	coverageContradictory coverageState = "contradictory"
)

// The last accepted membership set per qualified robot. Retain it across
// disconnection so replaying the same session cannot restart completion TTLs.
// A new completion requires an observed absent -> present transition; a new
// transport delivery of an unchanged set is not evidence of another mowing.
type completionObservation struct {
	receipt *coverageObservation
	areas   map[uint32]time.Time
}

// updateCompletedMemory returns a NEW memory map and consumes qualified
// observations under the coordinator lock. Completion times are the first
// accepted receipt of each membership, never the time we reread a snapshot.
func updateCompletedMemory(memory map[uint32]time.Time, members []fleetMember, now time.Time, ttl time.Duration, observations map[string]completionObservation) map[uint32]time.Time {
	out := make(map[uint32]time.Time, len(memory))
	for area, seen := range memory {
		if !seen.After(now) && now.Sub(seen) <= ttl {
			out[area] = seen
		}
	}
	for _, m := range members {
		receipt := m.SessionObservation
		if !m.Online || m.ID == "" || m.SessionState != coverageFresh || receipt == nil || receipt.receivedAt.IsZero() || receipt.receivedAt.After(now) || now.Sub(receipt.receivedAt) > peerOnlineMaxAge {
			continue
		}
		previous := observations[m.ID]
		if previous.receipt != nil && receipt != previous.receipt &&
			(receipt.receivedAt.Before(previous.receipt.receivedAt) ||
				(receipt.receivedAt.Equal(previous.receipt.receivedAt) && receipt.sequence <= previous.receipt.sequence)) {
			continue // out-of-order snapshot, not a new publication
		}
		current := previous
		if receipt != previous.receipt {
			current = completionObservation{receipt: receipt, areas: make(map[uint32]time.Time, len(m.Completed))}
			for _, area := range m.Completed {
				seen, known := previous.areas[area]
				if !known {
					seen = receipt.receivedAt
					// On startup, preserve a still-persisted earlier completion
					// rather than refreshing it with a subscription replay.
					if saved, ok := memory[area]; previous.receipt == nil && ok && saved.Before(seen) {
						seen = saved
					}
				}
				current.areas[area] = seen
			}
			observations[m.ID] = current
		}
		for area, seen := range current.areas {
			// A zero time retains membership suppressed by Start fresh; it
			// cannot become a completion until absent then present again.
			if !seen.IsZero() && !seen.After(now) && now.Sub(seen) <= ttl && seen.After(out[area]) {
				out[area] = seen
			}
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
