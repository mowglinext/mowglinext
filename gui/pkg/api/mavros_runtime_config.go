package api

import (
	"fmt"
	"math"
	"os"
	"path/filepath"

	"github.com/mowglinext/mowglinext/pkg/types"
	"gopkg.in/yaml.v3"
)

// Keep the dedicated sidecar files synchronized after GUI saves as well as
// installer runs. Defaults come from the template-derived GUI schema.
func writeMavrosRuntimeConfig(db types.IDBProvider) error {
	if activeHardwareBackendForDB(db) != "mavros" {
		return nil
	}
	configPath, err := db.Get("system.mower.yamlConfigFile")
	if err != nil {
		return err
	}
	envPath, err := db.Get("system.mower.runtimeEnvFile")
	if err != nil || len(envPath) == 0 {
		return fmt.Errorf("MAVROS runtime config directory is unavailable")
	}
	schema, err := getSchema(db)
	if err != nil {
		return err
	}
	flat := map[string]any{}
	extractDefaults(schema, flat)
	raw, err := os.ReadFile(string(configPath))
	if err != nil {
		return err
	}
	var doc map[string]any
	if err = yaml.Unmarshal(raw, &doc); err != nil {
		return err
	}
	for k, v := range flattenROS2YAML(doc) {
		flat[k] = v
	}
	odometry := map[string]any{}
	for canonical, target := range map[string]string{"ticks_per_meter": "ticks_per_meter", "wheel_track": "track_width_m"} {
		v, ok := asFloat64(flat[canonical])
		if !ok || math.IsNaN(v) || math.IsInf(v, 0) || v <= 0 {
			return fmt.Errorf("invalid %s", canonical)
		}
		odometry[target] = yamlFloatScalar(v)
	}
	bridge := map[string]any{"blade_control_enabled": false}
	for canonical, target := range map[string]string{"mavros_manual_control_enabled": "manual_control_enabled", "mavros_wheel_lift_safety_enabled": "wheel_lift_safety_enabled"} {
		v, ok := flat[canonical].(bool)
		if !ok {
			return fmt.Errorf("invalid %s", canonical)
		}
		bridge[target] = v
	}
	dir := filepath.Join(filepath.Dir(string(envPath)), "config", "mavros")
	if err = os.MkdirAll(dir, 0755); err != nil {
		return err
	}
	for name, payload := range map[string]any{
		"esc_wheel_odometry.yaml": map[string]any{"/**/esc_wheel_odometry": map[string]any{"ros__parameters": odometry}},
		"hardware_bridge.yaml":    map[string]any{"hardware_bridge": map[string]any{"ros__parameters": bridge}},
	} {
		out, err := yaml.Marshal(payload)
		if err != nil {
			return err
		}
		if err = writePreservingPerms(filepath.Join(dir, name), out); err != nil {
			return err
		}
	}
	return nil
}
