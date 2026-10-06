package api

import (
	"encoding/json"
	"fmt"
	"log"
	"os"
	"path/filepath"
	"sort"
	"strings"

	"github.com/gin-gonic/gin"
	"github.com/mowglinext/mowglinext/pkg/types"
	"gopkg.in/yaml.v3"
)

// Hardware backends (HARDWARE_BACKEND: which hardware bridge drives the robot).
//
// A backend changes the runtime parameter routes, a few configuration defaults,
// and the hardware-specific settings shown in the Settings page.
//
// ROS2 backend overlays are layered between the template and the installed
// config. The GUI image cannot read ros2/, so the overlays are baked into
// asserts/backend_defaults.json (guarded by TestBackendDefaultsAssetMatchesOverlays).
// The backend itself is chosen by the installer; this API only reports it.
const (
	defaultHardwareBackend    = "mowgli"
	backendDefaultsAssetPath  = "asserts/backend_defaults.json"
	ros2BackendOverlaysDir    = "../ros2/src/mowgli_bringup/config/backends"
	backendDefaultsAssetNotes = "GENERATED from ros2/src/mowgli_bringup/config/backends/*.yaml — do not edit by hand. " +
		"Regenerate with: cd gui && go run ./cmd/gen-backend-defaults"
)

// supportedHardwareBackends mirrors mowgli.launch.py SUPPORTED_HARDWARE_BACKENDS.
var supportedHardwareBackends = []string{"mowgli", "mavros", "openmower"}

// openMowerRuntimeEnvKeys maps OpenMower wiring settings to the installer env.
// If a key is absent from the installed YAML, the bridge uses the .env value;
// the GUI must report that same effective value.
var openMowerRuntimeEnvKeys = map[string]string{
	"openmower_ll_port":         "OPENMOWER_LL_PORT",
	"openmower_xesc_type":       "OPENMOWER_XESC_TYPE",
	"openmower_xesc_left_port":  "OPENMOWER_XESC_LEFT_PORT",
	"openmower_xesc_right_port": "OPENMOWER_XESC_RIGHT_PORT",
	"openmower_xesc_mow_port":   "OPENMOWER_XESC_MOW_PORT",
}

type backendDefaultsFile struct {
	Comment  string                    `json:"_comment"`
	Backends map[string]map[string]any `json:"backends"`
}

// HardwareParameterRoute maps a canonical mowgli_robot.yaml setting to a live
// ROS parameter on the selected backend. YAML persistence remains canonical;
// Runtime controls whether the live update is currently possible.
type HardwareParameterRoute struct {
	Parameter string `json:"parameter"`
	Runtime   string `json:"runtime"`
}

var hardwareParameterRoutes = map[string]map[string]HardwareParameterRoute{
	"mowgli": {
		"ticks_per_meter":          {Parameter: "hardware_bridge.ticks_per_meter", Runtime: "available"},
		"wheel_pid_kp":             {Parameter: "hardware_bridge.wheel_pid_kp", Runtime: "available"},
		"wheel_pid_ki":             {Parameter: "hardware_bridge.wheel_pid_ki", Runtime: "available"},
		"wheel_pid_kd":             {Parameter: "hardware_bridge.wheel_pid_kd", Runtime: "available"},
		"wheel_pid_integral_limit": {Parameter: "hardware_bridge.wheel_pid_integral_limit", Runtime: "available"},
		"wheel_pid_pwm_per_mps":    {Parameter: "hardware_bridge.wheel_pid_pwm_per_mps", Runtime: "available"},
	},
	"mavros": {
		"ticks_per_meter":                  {Parameter: "mavros/esc_wheel_odometry.ticks_per_meter", Runtime: "available"},
		"wheel_track":                      {Parameter: "mavros/esc_wheel_odometry.track_width_m", Runtime: "available"},
		"mavros_manual_control_enabled":    {Parameter: "hardware_bridge.manual_control_enabled", Runtime: "available"},
		"mavros_wheel_lift_safety_enabled": {Parameter: "hardware_bridge.wheel_lift_safety_enabled", Runtime: "available"},
	},
	// OpenMower drive settings are not live-routed through this parameter API.
	"openmower": {},
}

// normalizeHardwareBackend returns a supported backend or the default.
func normalizeHardwareBackend(value string) string {
	name := strings.ToLower(strings.TrimSpace(value))
	for _, known := range supportedHardwareBackends {
		if name == known {
			return name
		}
	}
	return defaultHardwareBackend
}

// loadRuntimeEnv reads docker/.env via system.mower.runtimeEnvFile.
func loadRuntimeEnv(dbProvider types.IDBProvider) map[string]string {
	path, err := dbProvider.Get("system.mower.runtimeEnvFile")
	if err != nil || len(path) == 0 {
		return map[string]string{}
	}
	env, err := loadGNSSRuntimeEnv(string(path))
	if err != nil {
		log.Printf("settings: %v; hardware backend falls back to %q", err, defaultHardwareBackend)
		return map[string]string{}
	}
	return env
}

// activeHardwareBackend prefers docker/.env over the process environment.
func activeHardwareBackend(runtimeEnv map[string]string) string {
	if value := strings.TrimSpace(runtimeEnv["HARDWARE_BACKEND"]); value != "" {
		return normalizeHardwareBackend(value)
	}
	return normalizeHardwareBackend(os.Getenv("HARDWARE_BACKEND"))
}

func activeHardwareBackendForDB(dbProvider types.IDBProvider) string {
	return activeHardwareBackend(loadRuntimeEnv(dbProvider))
}

// buildBackendDefaultsFile parses all ROS2 hardware backend YAML overlays.
func buildBackendDefaultsFile(dir string) (backendDefaultsFile, error) {
	paths, err := filepath.Glob(filepath.Join(dir, "*.yaml"))
	if err != nil {
		return backendDefaultsFile{}, err
	}
	sort.Strings(paths)
	file := backendDefaultsFile{Comment: backendDefaultsAssetNotes, Backends: map[string]map[string]any{}}
	for _, path := range paths {
		raw, err := os.ReadFile(path)
		if err != nil {
			return backendDefaultsFile{}, fmt.Errorf("read %s: %w", path, err)
		}
		doc := map[string]any{}
		if err := yaml.Unmarshal(raw, &doc); err != nil {
			return backendDefaultsFile{}, fmt.Errorf("parse %s: %w", path, err)
		}
		name := strings.TrimSuffix(filepath.Base(path), ".yaml")
		file.Backends[name] = flattenROS2YAML(doc)
	}
	return file, nil
}

func marshalBackendDefaultsFile(file backendDefaultsFile) ([]byte, error) {
	out, err := json.MarshalIndent(file, "", "  ")
	if err != nil {
		return nil, err
	}
	return append(out, '\n'), nil
}

// GenerateBackendDefaultsAsset regenerates the committed GUI overlay asset.
func GenerateBackendDefaultsAsset(overlaysDir, output string) error {
	file, err := buildBackendDefaultsFile(overlaysDir)
	if err != nil {
		return err
	}
	out, err := marshalBackendDefaultsFile(file)
	if err != nil {
		return err
	}
	return os.WriteFile(output, out, 0o644)
}

// backendDefaults returns one backend's overrides, or an empty map if missing.
func backendDefaults(backend string) map[string]any {
	raw, err := os.ReadFile(backendDefaultsAssetPath)
	if err != nil {
		log.Printf("settings: %s unavailable (%v); backend defaults ignored", backendDefaultsAssetPath, err)
		return map[string]any{}
	}
	var file backendDefaultsFile
	if err := json.Unmarshal(raw, &file); err != nil {
		log.Printf("settings: %s is not valid JSON (%v); backend defaults ignored", backendDefaultsAssetPath, err)
		return map[string]any{}
	}
	out := map[string]any{}
	for key, value := range file.Backends[backend] {
		out[key] = value
	}
	return out
}

// applyBackendDefaults layers backend defaults over template defaults, but only
// for schema-recognized keys.
func applyBackendDefaults(defaults map[string]any, backend string) {
	for key, value := range backendDefaults(backend) {
		if _, known := defaults[key]; known {
			defaults[key] = value
		}
	}
}

// applyOpenMowerRuntimeFallbacks uses installer wiring values only when the
// installed config does not explicitly set the corresponding key.
func applyOpenMowerRuntimeFallbacks(flat map[string]any, runtimeEnv map[string]string) {
	for key, envKey := range openMowerRuntimeEnvKeys {
		if hasExplicitFlatValue(flat[key]) {
			continue
		}
		if value := strings.TrimSpace(runtimeEnv[envKey]); value != "" {
			flat[key] = value
		}
	}
}

// HardwareBackendResponse is the GET /settings/hardware-backend response.
type HardwareBackendResponse struct {
	Backend   string   `json:"backend"`
	Supported []string `json:"supported"`
	// DefaultOverrides are settings for which this hardware backend replaces
	// the template default. Mower-model presets do not own these values.
	DefaultOverrides map[string]any                    `json:"default_overrides"`
	ParameterRoutes  map[string]HardwareParameterRoute `json:"parameter_routes"`
	RuntimeRouting   string                            `json:"runtime_routing"`
}

// GetSettingsHardwareBackend reports the installer's selected backend.
//
// @Summary returns the active hardware backend
// @Description HARDWARE_BACKEND from the runtime env (default mowgli)
// @Tags settings
// @Produce json
// @Success 200 {object} HardwareBackendResponse
// @Router /settings/hardware-backend [get]
func GetSettingsHardwareBackend(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/hardware-backend", func(c *gin.Context) {
		backend := activeHardwareBackendForDB(dbProvider)

		c.JSON(200, HardwareBackendResponse{
			Backend:          backend,
			Supported:        append([]string(nil), supportedHardwareBackends...),
			DefaultOverrides: backendDefaults(backend),
			ParameterRoutes:  hardwareParameterRoutes[backend],
			RuntimeRouting:   "available",
		})
	})
}

// requireMowgliHardwareBackend rejects STM32-only drive/PID endpoints on
// MAVROS and OpenMower backends.
func requireMowgliHardwareBackend(dbProvider types.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		if backend := activeHardwareBackendForDB(dbProvider); backend != "mowgli" {
			c.AbortWithStatusJSON(409, ErrorResponse{
				Error: "Mowgli drive PID/feed-forward tools are unavailable for HARDWARE_BACKEND=" + backend,
			})
			return
		}
		c.Next()
	}
}
