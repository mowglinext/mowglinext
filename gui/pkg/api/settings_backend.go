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
	"github.com/mowglinext/mowglinext/pkg/providers"
	"github.com/mowglinext/mowglinext/pkg/types"
	"gopkg.in/yaml.v3"
)

// Hardware backends (HARDWARE_BACKEND: which hardware bridge drives the robot).
//
// A backend changes two things the Settings page must follow:
//
//  1. a few DEFAULTS. ros2/src/mowgli_bringup/config/backends/<backend>.yaml is
//     layered between the template and the installed config by every consumer
//     (robot_config_util.load_robot_config, the OpenMower bridge's launch file),
//     so on an OpenMower robot "default" — the overridden dot, reset-to-default
//     and the sparse prune — must mean OpenMower's value. The GUI image cannot
//     read ros2/, so the overlays are baked into asserts/backend_defaults.json
//     (TestBackendDefaultsAssetMatchesOverlays keeps it in sync);
//  2. which hardware settings exist at all (serial ports, controller type):
//     the frontend shows the active backend's section only.
//
// The backend itself is still chosen by the installer (docker/.env decides
// which containers run); this file only reads it.

const (
	defaultHardwareBackend    = "mowgli"
	backendDefaultsAssetPath  = "asserts/backend_defaults.json"
	ros2BackendOverlaysDir    = "../ros2/src/mowgli_bringup/config/backends"
	backendDefaultsAssetNotes = "GENERATED from ros2/src/mowgli_bringup/config/backends/*.yaml — do not edit by hand. " +
		"Regenerate with: cd gui && go run ./cmd/gen-backend-defaults"
)

// supportedHardwareBackends mirrors mowgli.launch.py SUPPORTED_HARDWARE_BACKENDS.
var supportedHardwareBackends = []string{"mowgli", "mavros", "openmower"}

// openMowerRuntimeEnvKeys maps the OpenMower wiring keys to the docker/.env
// entries the installer writes. The bridge layers .env between the defaults
// and the installed config, so an ABSENT key runs with the .env value — the
// GUI must show that one, not the template's.
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

// normalizeHardwareBackend returns a known backend name, or the default.
func normalizeHardwareBackend(value string) string {
	name := strings.ToLower(strings.TrimSpace(value))
	for _, known := range supportedHardwareBackends {
		if name == known {
			return name
		}
	}
	return defaultHardwareBackend
}

// loadRuntimeEnv reads docker/.env (system.mower.runtimeEnvFile); empty on any
// failure, which every caller treats as "installer chose nothing".
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

// activeHardwareBackend is HARDWARE_BACKEND from docker/.env, then from this
// process's environment, then "mowgli".
func activeHardwareBackend(runtimeEnv map[string]string) string {
	if value := strings.TrimSpace(runtimeEnv["HARDWARE_BACKEND"]); value != "" {
		return normalizeHardwareBackend(value)
	}
	return normalizeHardwareBackend(os.Getenv("HARDWARE_BACKEND"))
}

// buildBackendDefaultsFile parses every overlay in dir.
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

// GenerateBackendDefaultsAsset rewrites the asset from the ROS2 overlays.
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

// backendDefaults returns the default overrides of one backend (empty when the
// backend has none or the asset is unavailable — then the schema defaults,
// i.e. the template, apply unchanged).
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

// applyBackendDefaults layers a backend's defaults over the schema defaults —
// the same template <- backend order the robot uses. Only keys the schema
// already knows are replaced, so an overlay can never invent a setting.
func applyBackendDefaults(defaults map[string]any, backend string) {
	for key, value := range backendDefaults(backend) {
		if _, known := defaults[key]; known {
			defaults[key] = value
		}
	}
}

// applyOpenMowerRuntimeFallbacks shows the installer's wiring (docker/.env)
// for every OpenMower wiring key the installed config does not set — that is
// the value the bridge runs with.
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

// HardwareBackendResponse is the body of GET /settings/hardware-backend.
type HardwareBackendResponse struct {
	Backend   string   `json:"backend"`
	Supported []string `json:"supported"`
	// DefaultOverrides are the settings whose default this backend replaces
	// (config/backends/<backend>.yaml). A mower-model preset must not write
	// these: the preset describes the machine, not its electronics.
	DefaultOverrides map[string]any `json:"default_overrides"`
	// RobotName is robot_name from the installed config (or its default):
	// the header badge shows it beside the backend.
	RobotName string `json:"robot_name"`
}

// GetSettingsHardwareBackend reports the active hardware backend.
//
// @Summary returns the active hardware backend
// @Description HARDWARE_BACKEND from the runtime env (default mowgli)
// @Tags settings
// @Produce json
// @Success 200 {object} HardwareBackendResponse
// @Router /settings/hardware-backend [get]
func GetSettingsHardwareBackend(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/hardware-backend", func(c *gin.Context) {
		backend := activeHardwareBackend(loadRuntimeEnv(dbProvider))
		c.JSON(200, HardwareBackendResponse{
			Backend:          backend,
			Supported:        append([]string(nil), supportedHardwareBackends...),
			DefaultOverrides: backendDefaults(backend),
			RobotName:        providers.ReadRobotName(dbProvider),
		})
	})
}
