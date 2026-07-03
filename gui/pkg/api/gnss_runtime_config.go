package api

import (
	"fmt"
	"net/http"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"

	pkgtypes "github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"github.com/joho/godotenv"
	"gopkg.in/yaml.v3"
)

var (
	gnssGlobPatterns = []string{
		"/dev/serial/by-id/*",
		"/dev/ttyUSB*",
		"/dev/ttyACM*",
	}
	gnssPathGlob     = filepath.Glob
	gnssPathStat     = os.Stat
	gnssPathEvalLink = filepath.EvalSymlinks
)

type gnssSettingsDocument struct {
	ConfigPath     string
	RuntimeEnvPath string
	ExistingYAML   map[string]any
	NodeMappings   map[string]string
	ExplicitFlat   map[string]any
	Flat           map[string]any
	RuntimeEnv     map[string]string
}

type gnssRuntimeValue struct {
	ConfiguredValue  string `json:"configured_value,omitempty"`
	FallbackEnvValue string `json:"fallback_env_value,omitempty"`
	ActiveValue      string `json:"active_value"`
	Source           string `json:"source"`
}

type gnssSerialDeviceOption struct {
	Path         string `json:"path"`
	ResolvedPath string `json:"resolved_path,omitempty"`
	DisplayLabel string `json:"display_label"`
	Exists       bool   `json:"exists"`
}

type gnssRuntimeConfigResponse struct {
	ReceiverFamily               gnssRuntimeValue         `json:"receiver_family"`
	Transport                    gnssRuntimeValue         `json:"transport"`
	SerialDevice                 gnssRuntimeValue         `json:"serial_device"`
	SerialBaud                   gnssRuntimeValue         `json:"serial_baud"`
	FrameID                      gnssRuntimeValue         `json:"frame_id"`
	NTRIPEnabled                 gnssRuntimeValue         `json:"ntrip_enabled"`
	NTRIPHost                    gnssRuntimeValue         `json:"ntrip_host"`
	NTRIPPort                    gnssRuntimeValue         `json:"ntrip_port"`
	NTRIPMountpoint              gnssRuntimeValue         `json:"ntrip_mountpoint"`
	NTRIPUsername                gnssRuntimeValue         `json:"ntrip_username"`
	NTRIPPassword                gnssRuntimeValue         `json:"ntrip_password"`
	NTRIPGGAEnabled              gnssRuntimeValue         `json:"ntrip_gga_enabled"`
	NTRIPGGAIntervalS            gnssRuntimeValue         `json:"ntrip_gga_interval_s"`
	SerialDevices                []gnssSerialDeviceOption `json:"serial_devices"`
	ConfiguredSerialDeviceExists bool                     `json:"configured_serial_device_exists"`
	ActiveSerialDeviceExists     bool                     `json:"active_serial_device_exists"`
	ConfiguredSerialResolvedPath string                   `json:"configured_serial_resolved_path,omitempty"`
	ActiveSerialResolvedPath     string                   `json:"active_serial_resolved_path,omitempty"`
}

func getGNSSRuntimeConfig(dbProvider pkgtypes.IDBProvider) gin.HandlerFunc {
	return func(c *gin.Context) {
		doc, err := loadGNSSSettingsDocument(dbProvider)
		if err != nil {
			c.JSON(http.StatusBadRequest, ErrorResponse{Error: err.Error()})
			return
		}

		c.JSON(http.StatusOK, buildGNSSRuntimeConfigResponse(doc))
	}
}

func loadGNSSSettingsDocument(dbProvider pkgtypes.IDBProvider) (gnssSettingsDocument, error) {
	configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
	if err != nil {
		return gnssSettingsDocument{}, fmt.Errorf("failed to read GNSS YAML config path: %w", err)
	}

	existingYAML := map[string]any{}
	if file, readErr := os.ReadFile(string(configFilePath)); readErr == nil {
		if err := yaml.Unmarshal(file, &existingYAML); err != nil {
			return gnssSettingsDocument{}, fmt.Errorf("invalid GNSS YAML config: %w", err)
		}
	} else if !os.IsNotExist(readErr) {
		return gnssSettingsDocument{}, fmt.Errorf("failed to read saved GNSS config: %w", readErr)
	}

	explicitFlat := flattenROS2YAML(existingYAML)
	flat := cloneFlatMap(explicitFlat)
	nodeMappings := map[string]string{}

	schema, err := getSchema(dbProvider)
	if err == nil {
		defaults := map[string]any{}
		extractDefaults(schema, defaults)
		for key, value := range defaults {
			if _, exists := flat[key]; !exists {
				flat[key] = value
			}
		}
		nodeMappings = extractNodeMappings(schema)
	}

	runtimeEnvPath, err := dbProvider.Get("system.mower.runtimeEnvFile")
	runtimeEnvFile := ""
	if err == nil {
		runtimeEnvFile = string(runtimeEnvPath)
	}

	runtimeEnv, err := loadGNSSRuntimeEnv(runtimeEnvFile)
	if err != nil {
		return gnssSettingsDocument{}, err
	}

	applyGNSSRuntimeFallbacks(flat, explicitFlat, runtimeEnv)

	return gnssSettingsDocument{
		ConfigPath:     string(configFilePath),
		RuntimeEnvPath: runtimeEnvFile,
		ExistingYAML:   existingYAML,
		NodeMappings:   nodeMappings,
		ExplicitFlat:   explicitFlat,
		Flat:           flat,
		RuntimeEnv:     runtimeEnv,
	}, nil
}

func loadGNSSRuntimeEnv(path string) (map[string]string, error) {
	if strings.TrimSpace(path) == "" {
		return map[string]string{}, nil
	}

	file, err := os.ReadFile(path)
	if err != nil {
		if os.IsNotExist(err) {
			return map[string]string{}, nil
		}
		return nil, fmt.Errorf("failed to read GNSS runtime env: %w", err)
	}

	values, err := godotenv.Parse(strings.NewReader(string(file)))
	if err != nil {
		return nil, fmt.Errorf("failed to parse GNSS runtime env: %w", err)
	}
	return values, nil
}

func cloneFlatMap(src map[string]any) map[string]any {
	dst := make(map[string]any, len(src))
	for key, value := range src {
		dst[key] = value
	}
	return dst
}

func hasExplicitFlatValue(value any) bool {
	if value == nil {
		return false
	}
	if text, ok := value.(string); ok {
		return strings.TrimSpace(text) != ""
	}
	return true
}

func applyGNSSRuntimeFallbacks(flat map[string]any, explicitFlat map[string]any, runtimeEnv map[string]string) {
	applyGNSSRuntimeFallback(flat, explicitFlat, runtimeEnv, []string{"gnss_receiver_family"}, "GNSS_RECEIVER_FAMILY", false)
	applyGNSSRuntimeFallback(flat, explicitFlat, runtimeEnv, []string{"gnss_transport"}, "GNSS_TRANSPORT", false)
	applyGNSSRuntimeFallback(flat, explicitFlat, runtimeEnv, []string{"gnss_serial_device"}, "GNSS_SERIAL_DEVICE", false)
	applyGNSSRuntimeFallback(flat, explicitFlat, runtimeEnv, []string{"gnss_serial_baud"}, "GNSS_SERIAL_BAUD", true)
	applyGNSSRuntimeFallback(flat, explicitFlat, runtimeEnv, []string{"gnss_frame_id"}, "GNSS_FRAME_ID", false)
	applyGNSSRuntimeFallback(flat, explicitFlat, runtimeEnv, []string{"gnss_ntrip_enabled", "ntrip_enabled"}, "GNSS_NTRIP_ENABLED", false)
	applyGNSSRuntimeFallback(flat, explicitFlat, runtimeEnv, []string{"gnss_ntrip_host", "ntrip_host"}, "GNSS_NTRIP_HOST", false)
	applyGNSSRuntimeFallback(flat, explicitFlat, runtimeEnv, []string{"gnss_ntrip_port", "ntrip_port"}, "GNSS_NTRIP_PORT", true)
	applyGNSSRuntimeFallback(flat, explicitFlat, runtimeEnv, []string{"gnss_ntrip_mountpoint", "ntrip_mountpoint"}, "GNSS_NTRIP_MOUNTPOINT", false)
	applyGNSSRuntimeFallback(flat, explicitFlat, runtimeEnv, []string{"gnss_ntrip_username", "ntrip_user"}, "GNSS_NTRIP_USERNAME", false)
	applyGNSSRuntimeFallback(flat, explicitFlat, runtimeEnv, []string{"gnss_ntrip_password", "ntrip_password"}, "GNSS_NTRIP_PASSWORD", false)
	applyGNSSRuntimeFallback(flat, explicitFlat, runtimeEnv, []string{"gnss_ntrip_gga_enabled"}, "GNSS_NTRIP_GGA_ENABLED", false)
	applyGNSSRuntimeFallback(flat, explicitFlat, runtimeEnv, []string{"gnss_ntrip_gga_interval_s"}, "GNSS_NTRIP_GGA_INTERVAL_S", true)
}

func applyGNSSRuntimeFallback(flat map[string]any, explicitFlat map[string]any, runtimeEnv map[string]string, yamlKeys []string, envKey string, numeric bool) {
	for _, yamlKey := range yamlKeys {
		if hasExplicitFlatValue(explicitFlat[yamlKey]) {
			return
		}
	}

	envValue := strings.TrimSpace(runtimeEnv[envKey])
	if envValue == "" {
		return
	}

	targetKey := yamlKeys[0]
	for _, yamlKey := range yamlKeys {
		if _, exists := flat[yamlKey]; exists {
			targetKey = yamlKey
			break
		}
	}

	if numeric {
		if parsed, err := strconv.Atoi(envValue); err == nil {
			flat[targetKey] = parsed
			return
		}
	}
	flat[targetKey] = envValue
}

func buildGNSSRuntimeConfigResponse(doc gnssSettingsDocument) gnssRuntimeConfigResponse {
	receiverFamily := resolvedGNSSRuntimeValue(doc, []string{"gnss_receiver_family"}, "GNSS_RECEIVER_FAMILY", "auto")
	transport := resolvedGNSSRuntimeValue(doc, []string{"gnss_transport"}, "GNSS_TRANSPORT", "serial")
	serialDevice := resolvedGNSSRuntimeValue(doc, []string{"gnss_serial_device"}, "GNSS_SERIAL_DEVICE", "/dev/ttyAMA4")
	serialBaud := resolvedGNSSRuntimeValue(doc, []string{"gnss_serial_baud"}, "GNSS_SERIAL_BAUD", "921600")
	frameID := resolvedGNSSRuntimeValue(doc, []string{"gnss_frame_id"}, "GNSS_FRAME_ID", "gps_link")
	ntripEnabled := resolvedGNSSRuntimeValue(doc, []string{"gnss_ntrip_enabled", "ntrip_enabled"}, "GNSS_NTRIP_ENABLED", "true")
	ntripHost := resolvedGNSSRuntimeValue(doc, []string{"gnss_ntrip_host", "ntrip_host"}, "GNSS_NTRIP_HOST", "crtk.net")
	ntripPort := resolvedGNSSRuntimeValue(doc, []string{"gnss_ntrip_port", "ntrip_port"}, "GNSS_NTRIP_PORT", "2101")
	ntripMountpoint := resolvedGNSSRuntimeValue(doc, []string{"gnss_ntrip_mountpoint", "ntrip_mountpoint"}, "GNSS_NTRIP_MOUNTPOINT", "NEAR")
	ntripUsername := resolvedGNSSRuntimeValue(doc, []string{"gnss_ntrip_username", "ntrip_user"}, "GNSS_NTRIP_USERNAME", "")
	ntripPassword := resolvedGNSSRuntimeValue(doc, []string{"gnss_ntrip_password", "ntrip_password"}, "GNSS_NTRIP_PASSWORD", "")
	ntripGGAEnabled := resolvedGNSSRuntimeValue(doc, []string{"gnss_ntrip_gga_enabled"}, "GNSS_NTRIP_GGA_ENABLED", "false")
	ntripGGAIntervalS := resolvedGNSSRuntimeValue(doc, []string{"gnss_ntrip_gga_interval_s"}, "GNSS_NTRIP_GGA_INTERVAL_S", "10")
	configuredExists, configuredResolved := inspectGNSSSerialDevice(serialDevice.ConfiguredValue)
	activeExists, activeResolved := inspectGNSSSerialDevice(serialDevice.ActiveValue)

	return gnssRuntimeConfigResponse{
		ReceiverFamily:               receiverFamily,
		Transport:                    transport,
		SerialDevice:                 serialDevice,
		SerialBaud:                   serialBaud,
		FrameID:                      frameID,
		NTRIPEnabled:                 ntripEnabled,
		NTRIPHost:                    ntripHost,
		NTRIPPort:                    ntripPort,
		NTRIPMountpoint:              ntripMountpoint,
		NTRIPUsername:                ntripUsername,
		NTRIPPassword:                ntripPassword,
		NTRIPGGAEnabled:              ntripGGAEnabled,
		NTRIPGGAIntervalS:            ntripGGAIntervalS,
		SerialDevices:                discoverGNSSSerialDevices(),
		ConfiguredSerialDeviceExists: configuredExists,
		ActiveSerialDeviceExists:     activeExists,
		ConfiguredSerialResolvedPath: configuredResolved,
		ActiveSerialResolvedPath:     activeResolved,
	}
}

func resolvedGNSSRuntimeValue(doc gnssSettingsDocument, yamlKeys []string, envKey string, defaultValue string) gnssRuntimeValue {
	configuredValue := ""
	for _, yamlKey := range yamlKeys {
		if hasExplicitFlatValue(doc.ExplicitFlat[yamlKey]) {
			configuredValue = stringValue(doc.ExplicitFlat[yamlKey], "")
			break
		}
	}

	fallbackEnvValue := strings.TrimSpace(doc.RuntimeEnv[envKey])
	activeValue := defaultValue
	for _, yamlKey := range yamlKeys {
		value, exists := doc.Flat[yamlKey]
		if !exists {
			continue
		}
		activeValue = stringValue(value, defaultValue)
		break
	}
	source := "default"
	if configuredValue != "" {
		source = "config"
	} else if fallbackEnvValue != "" {
		source = "env"
	}

	return gnssRuntimeValue{
		ConfiguredValue:  configuredValue,
		FallbackEnvValue: fallbackEnvValue,
		ActiveValue:      activeValue,
		Source:           source,
	}
}

func inspectGNSSSerialDevice(path string) (bool, string) {
	cleanPath := strings.TrimSpace(path)
	if cleanPath == "" {
		return false, ""
	}

	if _, err := gnssPathStat(cleanPath); err != nil {
		return false, ""
	}

	resolved := cleanPath
	if target, err := gnssPathEvalLink(cleanPath); err == nil && strings.TrimSpace(target) != "" {
		resolved = target
	}
	return true, resolved
}

func discoverGNSSSerialDevices() []gnssSerialDeviceOption {
	seen := map[string]struct{}{}
	discovered := make([]gnssSerialDeviceOption, 0)

	for _, pattern := range gnssGlobPatterns {
		matches, err := gnssPathGlob(pattern)
		if err != nil {
			continue
		}
		sort.Strings(matches)
		for _, match := range matches {
			if _, exists := seen[match]; exists {
				continue
			}
			seen[match] = struct{}{}
			exists, resolved := inspectGNSSSerialDevice(match)
			discovered = append(discovered, gnssSerialDeviceOption{
				Path:         match,
				ResolvedPath: resolved,
				DisplayLabel: gnssSerialDeviceDisplayLabel(match, resolved),
				Exists:       exists,
			})
		}
	}

	return discovered
}

func gnssSerialDeviceDisplayLabel(path string, resolvedPath string) string {
	if strings.HasPrefix(path, "/dev/serial/by-id/") {
		base := filepath.Base(path)
		if resolvedPath != "" && resolvedPath != path {
			return fmt.Sprintf("%s -> %s", base, resolvedPath)
		}
		return base
	}
	if resolvedPath != "" && resolvedPath != path {
		return fmt.Sprintf("%s -> %s", path, resolvedPath)
	}
	return path
}

func validateGNSSSerialDeviceExists(device string) error {
	exists, _ := inspectGNSSSerialDevice(device)
	if exists {
		return nil
	}
	return fmt.Errorf("GNSS serial device does not exist: %s", device)
}
