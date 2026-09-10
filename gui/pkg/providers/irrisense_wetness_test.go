package providers

import (
	"testing"
	"time"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func wetnessNow() time.Time {
	return time.Date(2026, 9, 3, 8, 0, 0, 0, time.UTC)
}

func wetnessDefaults() WetnessConfig {
	return WetnessConfig{WetDeficitMm: 2.0, DryAfterWateringHours: 3.0}
}

func zone(id, label string, enabled bool, deficit float64, watered *time.Time) IrriSenseZone {
	return IrriSenseZone{ID: id, Label: label, Enabled: enabled, DeficitMm: deficit, LastWateredAt: watered}
}

func agoPtr(d time.Duration) *time.Time {
	t := wetnessNow().Add(-d)
	return &t
}

func TestEvaluateWetness(t *testing.T) {
	cases := []struct {
		name       string
		zones      []IrriSenseZone
		cfg        WetnessConfig
		wantWet    bool
		wantReason string
	}{
		{
			name:       "wet by deficit",
			zones:      []IrriSenseZone{zone("z1", "Pelouse nord", true, 0.8, nil)},
			cfg:        wetnessDefaults(),
			wantWet:    true,
			wantReason: `zone "Pelouse nord": deficit 0.8 mm ≤ 2.0 mm`,
		},
		{
			name:       "wet by recent watering despite a high deficit",
			zones:      []IrriSenseZone{zone("z1", "Pelouse nord", true, 6.5, agoPtr(40*time.Minute))},
			cfg:        wetnessDefaults(),
			wantWet:    true,
			wantReason: `zone "Pelouse nord": watered 40 min ago`,
		},
		{
			name:       "wet by both reasons lists both",
			zones:      []IrriSenseZone{zone("z1", "Pelouse nord", true, 0.8, agoPtr(40*time.Minute))},
			cfg:        wetnessDefaults(),
			wantWet:    true,
			wantReason: `zone "Pelouse nord": deficit 0.8 mm ≤ 2.0 mm, watered 40 min ago`,
		},
		{
			name:       "dry: deficit above threshold and watered long ago",
			zones:      []IrriSenseZone{zone("z1", "Pelouse nord", true, 4.1, agoPtr(26*time.Hour))},
			cfg:        wetnessDefaults(),
			wantWet:    false,
			wantReason: "1 zone dry (lowest deficit 4.1 mm > 2.0 mm, none watered in the last 3 h)",
		},
		{
			name: "disabled zone is ignored even when soaking",
			zones: []IrriSenseZone{
				zone("z1", "Potager", false, 0.0, agoPtr(5*time.Minute)),
				zone("z2", "Pelouse", true, 5.0, nil),
			},
			cfg:        wetnessDefaults(),
			wantWet:    false,
			wantReason: "1 zone dry (lowest deficit 5.0 mm > 2.0 mm, none watered in the last 3 h)",
		},
		{
			name: "zone filter excludes a wet zone the operator did not select",
			zones: []IrriSenseZone{
				zone("z1", "Potager", true, 0.0, nil),
				zone("z2", "Pelouse", true, 5.0, nil),
			},
			cfg:        WetnessConfig{WetDeficitMm: 2.0, DryAfterWateringHours: 3.0, ZoneIDs: []string{"z2"}},
			wantWet:    false,
			wantReason: "1 zone dry (lowest deficit 5.0 mm > 2.0 mm, none watered in the last 3 h)",
		},
		{
			name: "zone filter selects the wet zone",
			zones: []IrriSenseZone{
				zone("z1", "Potager", true, 0.0, nil),
				zone("z2", "Pelouse", true, 5.0, nil),
			},
			cfg:        WetnessConfig{WetDeficitMm: 2.0, DryAfterWateringHours: 3.0, ZoneIDs: []string{"z1"}},
			wantWet:    true,
			wantReason: `zone "Potager": deficit 0.0 mm ≤ 2.0 mm`,
		},
		{
			name:       "empty garden is dry",
			zones:      nil,
			cfg:        wetnessDefaults(),
			wantWet:    false,
			wantReason: "no enabled zone selected",
		},
		{
			name:       "watering exactly at the boundary still counts as wet",
			zones:      []IrriSenseZone{zone("z1", "Pelouse", true, 9.0, agoPtr(3*time.Hour))},
			cfg:        wetnessDefaults(),
			wantWet:    true,
			wantReason: `zone "Pelouse": watered 3.0 h ago`,
		},
		{
			name:       "a watering timestamp in the future is not trusted",
			zones:      []IrriSenseZone{zone("z1", "Pelouse", true, 9.0, agoPtr(-10*time.Minute))},
			cfg:        wetnessDefaults(),
			wantWet:    false,
			wantReason: "1 zone dry (lowest deficit 9.0 mm > 2.0 mm, none watered in the last 3 h)",
		},
		{
			name:       "zero dry-after window disables the watering clause",
			zones:      []IrriSenseZone{zone("z1", "Pelouse", true, 9.0, agoPtr(1*time.Minute))},
			cfg:        WetnessConfig{WetDeficitMm: 2.0, DryAfterWateringHours: 0},
			wantWet:    false,
			wantReason: "1 zone dry (lowest deficit 9.0 mm > 2.0 mm, none watered in the last 0 h)",
		},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			wet, reason := EvaluateWetness(IrriSenseGarden{Name: "Jardin", Zones: tc.zones}, tc.cfg, wetnessNow())
			assert.Equal(t, tc.wantWet, wet)
			assert.Equal(t, tc.wantReason, reason)
		})
	}
}

func TestEvaluateZones_MarksSelectionAndWetnessPerZone(t *testing.T) {
	garden := IrriSenseGarden{Zones: []IrriSenseZone{
		zone("z1", "Potager", true, 0.5, nil),
		zone("z2", "Pelouse", true, 5.0, nil),
		zone("z3", "Haie", false, 0.1, nil),
	}}
	cfg := WetnessConfig{WetDeficitMm: 2.0, DryAfterWateringHours: 3.0, ZoneIDs: []string{"z1", "z3"}}

	zones := EvaluateZones(garden, cfg, wetnessNow())
	require.Len(t, zones, 3)

	assert.True(t, zones[0].Selected)
	assert.True(t, zones[0].Wet)
	assert.Equal(t, "deficit 0.5 mm ≤ 2.0 mm", zones[0].Reason)

	assert.False(t, zones[1].Selected, "z2 was not in the operator's list")
	assert.False(t, zones[1].Wet)
	assert.Empty(t, zones[1].Reason)

	assert.True(t, zones[2].Selected)
	assert.False(t, zones[2].Enabled)
	assert.False(t, zones[2].Wet, "a disabled zone never counts")
}

func TestFormatAgo(t *testing.T) {
	assert.Equal(t, "under a minute", formatAgo(20*time.Second))
	assert.Equal(t, "40 min", formatAgo(40*time.Minute))
	assert.Equal(t, "1.5 h", formatAgo(90*time.Minute))
	assert.Equal(t, "3 days", formatAgo(72*time.Hour))
}
