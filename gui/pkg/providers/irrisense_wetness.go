package providers

import (
	"fmt"
	"math"
	"strings"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
)

// WetnessConfig is the operator-tunable part of the wetness rule.
type WetnessConfig struct {
	// WetDeficitMm: a zone whose deficit (mm evaporated since it was last
	// wetted by irrigation or rain) is at or below this is wet.
	WetDeficitMm float64
	// DryAfterWateringHours: a zone watered more recently than this is wet
	// regardless of its deficit (the surface has not had time to dry).
	DryAfterWateringHours float64
	// ZoneIDs restricts the rule to these zones; empty means every enabled
	// zone of the garden.
	ZoneIDs []string
}

// EvaluateWetness applies the wetness rule to one garden.
//
// A zone is WET if it is enabled AND (deficit <= WetDeficitMm OR it was watered
// within DryAfterWateringHours of now). The garden is WET if ANY selected zone
// is wet. The reason is written for a human, e.g.
//
//	zone "Pelouse nord": deficit 0.8 mm ≤ 2.0 mm, watered 40 min ago
//
// Pure: no I/O, no clock — `now` is injected so the rule is table-testable.
func EvaluateWetness(garden IrriSenseGarden, cfg WetnessConfig, now time.Time) (wet bool, reason string) {
	zones := EvaluateZones(garden, cfg, now)

	selected := 0
	minDeficit := math.Inf(1)
	for _, z := range zones {
		if z.Wet {
			return true, fmt.Sprintf("zone %q: %s", z.Label, z.Reason)
		}
		if !z.Selected || !z.Enabled {
			continue
		}
		selected++
		minDeficit = math.Min(minDeficit, z.DeficitMm)
	}

	if selected == 0 {
		return false, "no enabled zone selected"
	}
	return false, fmt.Sprintf("%s dry (lowest deficit %.1f mm > %.1f mm, none watered in the last %s)",
		pluralZones(selected), minDeficit, cfg.WetDeficitMm, formatHours(cfg.DryAfterWateringHours))
}

// EvaluateZones is the per-zone view of the same rule, kept so the GUI can
// show why each zone counted (or did not).
func EvaluateZones(garden IrriSenseGarden, cfg WetnessConfig, now time.Time) []types.SoilZoneStatus {
	filter := zoneFilter(cfg.ZoneIDs)
	dryAfter := time.Duration(cfg.DryAfterWateringHours * float64(time.Hour))

	out := make([]types.SoilZoneStatus, 0, len(garden.Zones))
	for _, z := range garden.Zones {
		status := types.SoilZoneStatus{
			ID:            z.ID,
			Label:         z.Label,
			Enabled:       z.Enabled,
			Selected:      filter(z.ID),
			DeficitMm:     z.DeficitMm,
			LastWateredAt: z.LastWateredAt,
		}
		if status.Selected && status.Enabled {
			status.Wet, status.Reason = evaluateZone(z, cfg.WetDeficitMm, dryAfter, now)
		}
		out = append(out, status)
	}
	return out
}

func evaluateZone(z IrriSenseZone, wetDeficit float64, dryAfter time.Duration, now time.Time) (bool, string) {
	byDeficit := z.DeficitMm <= wetDeficit
	byWatering := false
	var sinceWatered time.Duration
	if z.LastWateredAt != nil && dryAfter > 0 {
		sinceWatered = now.Sub(*z.LastWateredAt)
		byWatering = sinceWatered >= 0 && sinceWatered <= dryAfter
	}

	var parts []string
	if byDeficit {
		parts = append(parts, fmt.Sprintf("deficit %.1f mm ≤ %.1f mm", z.DeficitMm, wetDeficit))
	}
	if byWatering {
		parts = append(parts, fmt.Sprintf("watered %s ago", formatAgo(sinceWatered)))
	}
	if len(parts) > 0 {
		return true, strings.Join(parts, ", ")
	}
	return false, fmt.Sprintf("deficit %.1f mm > %.1f mm", z.DeficitMm, wetDeficit)
}

func zoneFilter(ids []string) func(string) bool {
	set := make(map[string]struct{}, len(ids))
	for _, id := range ids {
		if trimmed := strings.TrimSpace(id); trimmed != "" {
			set[trimmed] = struct{}{}
		}
	}
	if len(set) == 0 {
		return func(string) bool { return true }
	}
	return func(id string) bool {
		_, ok := set[id]
		return ok
	}
}

func pluralZones(n int) string {
	if n == 1 {
		return "1 zone"
	}
	return fmt.Sprintf("%d zones", n)
}

func formatAgo(d time.Duration) string {
	if d < time.Minute {
		return "under a minute"
	}
	if d < time.Hour {
		return fmt.Sprintf("%d min", int(d.Minutes()))
	}
	hours := d.Hours()
	if hours < 48 {
		return fmt.Sprintf("%.1f h", hours)
	}
	return fmt.Sprintf("%.0f days", hours/24)
}

func formatHours(h float64) string {
	if h == math.Trunc(h) {
		return fmt.Sprintf("%.0f h", h)
	}
	return fmt.Sprintf("%.1f h", h)
}
