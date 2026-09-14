package providers

import "time"

// IrriSenseZone mirrors IrriSense Cloud's ReadOnlyZone
// (backend/app/api/schemas.py). Only the fields the wetness rule and the GUI
// need are decoded; unknown fields are ignored.
type IrriSenseZone struct {
	ID            string     `json:"id"`
	Label         string     `json:"label"`
	Mode          string     `json:"mode"`
	Enabled       bool       `json:"enabled"`
	DeficitMm     float64    `json:"deficit_mm"`
	TriggerMm     float64    `json:"trigger_mm"`
	DoseMm        float64    `json:"dose_mm"`
	Readiness     float64    `json:"readiness"`
	LastWateredAt *time.Time `json:"last_watered_at"`
	DeviceSerial  string     `json:"device_serial"`
}

// IrriSenseGarden mirrors IrriSense Cloud's ReadOnlyGarden.
type IrriSenseGarden struct {
	ID             string          `json:"id"`
	Name           string          `json:"name"`
	Mode           string          `json:"mode"`
	Timezone       string          `json:"timezone"`
	Latitude       float64         `json:"latitude"`
	Longitude      float64         `json:"longitude"`
	WorstDeficitMm float64         `json:"worst_deficit_mm"`
	Zones          []IrriSenseZone `json:"zones"`
}

// IrriSenseGardenSummary is what the GUI's garden dropdown needs.
type IrriSenseGardenSummary struct {
	ID    string                 `json:"id"`
	Name  string                 `json:"name"`
	Zones []IrriSenseZoneSummary `json:"zones"`
}

// IrriSenseZoneSummary is one entry of the GUI's zone multi-select.
type IrriSenseZoneSummary struct {
	ID      string `json:"id"`
	Label   string `json:"label"`
	Enabled bool   `json:"enabled"`
}

func summarizeGarden(g IrriSenseGarden) IrriSenseGardenSummary {
	zones := make([]IrriSenseZoneSummary, 0, len(g.Zones))
	for _, z := range g.Zones {
		zones = append(zones, IrriSenseZoneSummary{ID: z.ID, Label: z.Label, Enabled: z.Enabled})
	}
	return IrriSenseGardenSummary{ID: g.ID, Name: g.Name, Zones: zones}
}
