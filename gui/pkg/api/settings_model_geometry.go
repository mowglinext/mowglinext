package api

import (
	"encoding/json"
	"log"
	"os"
)

// Only opted-in, newly introduced geometry defaults are taken from the selected
// preset. This asset is generated from mowerModels.ts, also shipped to ROS, and
// never contains wheel/control/calibration values to replay over an upgrade.
func applyModelGeometryDefaults(defaults map[string]any, selection map[string]any) {
	raw, err := os.ReadFile("asserts/model_geometry_defaults.json")
	if err != nil {
		log.Printf("settings: model geometry defaults unavailable: %v", err)
		return
	}
	var asset struct {
		Models map[string]map[string]any `json:"models"`
	}
	if err := json.Unmarshal(raw, &asset); err != nil {
		log.Printf("settings: invalid model geometry defaults: %v", err)
		return
	}
	model := defaults["mower_model"]
	if chosen, ok := selection["mower_model"]; ok && chosen != nil {
		model = chosen
	}
	name, _ := model.(string)
	for key, value := range asset.Models[name] {
		if _, known := defaults[key]; known {
			defaults[key] = value
		}
	}
}
