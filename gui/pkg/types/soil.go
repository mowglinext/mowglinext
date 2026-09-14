package types

import "time"

// SoilZoneStatus is one irrigation zone as the wetness rule saw it on the last
// successful IrriSense fetch. Selected is false for zones the operator filtered
// out; Wet is only ever true for a selected, enabled zone.
type SoilZoneStatus struct {
	ID            string     `json:"id"`
	Label         string     `json:"label"`
	Enabled       bool       `json:"enabled"`
	Selected      bool       `json:"selected"`
	DeficitMm     float64    `json:"deficitMm"`
	LastWateredAt *time.Time `json:"lastWateredAt,omitempty"`
	Wet           bool       `json:"wet"`
	Reason        string     `json:"reason,omitempty"`
}

// SoilStatus is the cached verdict of the IrriSense soil-moisture provider.
//
// The scheduler blocks a run ONLY when Enabled && GateScheduler && Fresh && Wet.
// Every other combination is fail-open: Unknown covers "not configured", "the
// service is unreachable" and "the last good fetch is older than the staleness
// window" — a cloud outage must never keep the robot in the dock.
type SoilStatus struct {
	Enabled       bool             `json:"enabled"`
	Configured    bool             `json:"configured"`
	GateScheduler bool             `json:"gateScheduler"`
	Fresh         bool             `json:"fresh"`
	Wet           bool             `json:"wet"`
	Unknown       bool             `json:"unknown"`
	Reason        string           `json:"reason"`
	GardenName    string           `json:"gardenName,omitempty"`
	FetchedAt     *time.Time       `json:"fetchedAt,omitempty"`
	Zones         []SoilZoneStatus `json:"zones"`
	Error         string           `json:"error,omitempty"`
}

// BlocksScheduledMowing is the single place the "is the grass too wet to mow"
// decision is spelled out, so the scheduler and its tests cannot disagree.
func (s SoilStatus) BlocksScheduledMowing() bool {
	return s.Enabled && s.GateScheduler && s.Fresh && s.Wet && !s.Unknown
}

// ISoilProvider is what the scheduler consumes; the IrriSense provider is the
// live implementation and tests substitute a fake.
type ISoilProvider interface {
	SoilStatus() SoilStatus
}
