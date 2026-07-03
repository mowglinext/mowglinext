package api

import (
	"encoding/json"
	"fmt"
	"log"
	"sort"
	"sync"
	"syscall"
	"time"

	"os"
	"path/filepath"
	"strconv"
	"strings"

	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"github.com/joho/godotenv"
	"gopkg.in/yaml.v3"
)

const latLonDecimalPlaces = 9

var fixedPrecisionEnvKeys = map[string]bool{
	"OM_DATUM_LAT":  true,
	"OM_DATUM_LONG": true,
	"OM_DATUM_LON":  true,
}

var fixedPrecisionYAMLKeys = map[string]bool{
	"datum_lat": true,
	"datum_lon": true,
	"dock_lat":  true,
	"dock_lon":  true,
}

type fixedPrecisionFloat float64

func (f fixedPrecisionFloat) MarshalYAML() (any, error) {
	return &yaml.Node{
		Kind:  yaml.ScalarNode,
		Tag:   "!!float",
		Value: formatLatLonDecimal(float64(f)),
	}, nil
}

// writePreservingPerms writes content to path, preserving the existing
// file's mode and uid/gid when the file already exists. When the file
// is being created for the first time, it is written owner- and
// group-writable (0664) so other processes (ROS containers) sharing the
// file's group can still update it. NOTE: 0664 is NOT world-writable, so
// the ROS-side line-splice writers (calibration service, set_docking_point,
// drive-tuning rollback) only persist if their container shares the file's
// gid; if the containers run with a different uid AND gid, those write-backs
// fail with EACCES. The previous behavior would silently rewrite the file as
// owned by the GUI process with mode 0644, which locked out those writers.
func writePreservingPerms(path string, content []byte) error {
	mode := os.FileMode(0664)
	var uid, gid int = -1, -1
	if info, err := os.Stat(path); err == nil {
		mode = info.Mode().Perm()
		if stat, ok := info.Sys().(*syscall.Stat_t); ok {
			uid = int(stat.Uid)
			gid = int(stat.Gid)
		}
	}
	if err := os.WriteFile(path, content, mode); err != nil {
		return err
	}
	// os.WriteFile only applies the mode on creation; force it after
	// every write so an externally-changed mode does not stick.
	if err := os.Chmod(path, mode); err != nil {
		return err
	}
	if uid >= 0 && gid >= 0 {
		// Best-effort: chown can fail when the GUI process is not root
		// (e.g. running directly on the host). In that case the file
		// was opened in-place so ownership is already preserved.
		_ = os.Chown(path, uid, gid)
	}
	return nil
}

func SettingsRoutes(r *gin.RouterGroup, dbProvider types.IDBProvider) {
	GetSettings(r, dbProvider)
	PostSettings(r, dbProvider)
	GetSettingsSchema(r, dbProvider)
	GetSettingsYAML(r, dbProvider)
	GetSettingsYAMLDefaults(r, dbProvider)
	PostSettingsYAML(r, dbProvider)
	GetSettingsStatus(r, dbProvider)
	PostSettingsStatus(r, dbProvider)
}

// GetSettingsStatus returns whether onboarding has been completed.
// Used by the frontend to decide whether to show the onboarding wizard.
//
// @Summary get settings onboarding status
// @Description returns whether onboarding has been completed
// @Tags settings
// @Produce json
// @Success 200 {object} SettingsStatusResponse
// @Router /settings/status [get]
func GetSettingsStatus(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/status", func(c *gin.Context) {
		val, err := dbProvider.Get("onboarding.completed")
		completed := err == nil && string(val) == "true"
		c.JSON(200, gin.H{"onboarding_completed": completed})
	})
}

// PostSettingsStatus marks onboarding as completed.
//
// @Summary mark onboarding as completed
// @Description marks onboarding as completed so the wizard is not shown again
// @Tags settings
// @Produce json
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings/status [post]
func PostSettingsStatus(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.POST("/settings/status", func(c *gin.Context) {
		if err := dbProvider.Set("onboarding.completed", []byte("true")); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, OkResponse{})
	})
}

func extractDefaults(schema map[string]any, defaults map[string]any) {
	if props, ok := schema["properties"].(map[string]any); ok {
		for key, prop := range props {
			if propMap, ok := prop.(map[string]any); ok {
				if def, hasDef := propMap["default"]; hasDef {
					defaults[key] = def
				}
				extractDefaults(propMap, defaults)
			}
		}
	}
	if allOf, ok := schema["allOf"].([]any); ok {
		for _, cond := range allOf {
			if condMap, ok := cond.(map[string]any); ok {
				if thenBlock, ok := condMap["then"].(map[string]any); ok {
					extractDefaults(thenBlock, defaults)
				}
				if elseBlock, ok := condMap["else"].(map[string]any); ok {
					extractDefaults(elseBlock, defaults)
				}
			}
		}
	}
}

func extractAllKeys(schema map[string]any, keys map[string]bool) {
	if props, ok := schema["properties"].(map[string]any); ok {
		for key, prop := range props {
			if propMap, ok := prop.(map[string]any); ok {
				propType, _ := propMap["type"].(string)
				_, hasEnvVar := propMap["x-environment-variable"]
				if hasEnvVar || (propType != "" && propType != "object") {
					keys[key] = true
				}
				extractAllKeys(propMap, keys)
			}
		}
	}
	if allOf, ok := schema["allOf"].([]any); ok {
		for _, cond := range allOf {
			if condMap, ok := cond.(map[string]any); ok {
				if thenBlock, ok := condMap["then"].(map[string]any); ok {
					extractAllKeys(thenBlock, keys)
				}
				if elseBlock, ok := condMap["else"].(map[string]any); ok {
					extractAllKeys(elseBlock, keys)
				}
			}
		}
	}
}

func extractKeyTypes(schema map[string]any, types_ map[string]string) {
	if props, ok := schema["properties"].(map[string]any); ok {
		for key, prop := range props {
			if propMap, ok := prop.(map[string]any); ok {
				propType, _ := propMap["type"].(string)
				if propType != "" && propType != "object" {
					types_[key] = propType
				}
				extractKeyTypes(propMap, types_)
			}
		}
	}
	if allOf, ok := schema["allOf"].([]any); ok {
		for _, cond := range allOf {
			if condMap, ok := cond.(map[string]any); ok {
				if thenBlock, ok := condMap["then"].(map[string]any); ok {
					extractKeyTypes(thenBlock, types_)
				}
				if elseBlock, ok := condMap["else"].(map[string]any); ok {
					extractKeyTypes(elseBlock, types_)
				}
			}
		}
	}
}

// extractNodeMappings builds a map of paramKey -> ROS2 node name from
// x-yaml-node extensions in the schema. Keys without x-yaml-node default to "mowgli".
func extractNodeMappings(schema map[string]any) map[string]string {
	mappings := map[string]string{}
	allKeys := map[string]bool{}
	extractAllKeys(schema, allKeys)

	var walk func(s map[string]any)
	walk = func(s map[string]any) {
		if props, ok := s["properties"].(map[string]any); ok {
			for key, prop := range props {
				if propMap, ok := prop.(map[string]any); ok {
					if node, ok := propMap["x-yaml-node"].(string); ok {
						mappings[key] = node
					}
					walk(propMap)
				}
			}
		}
		if allOf, ok := s["allOf"].([]any); ok {
			for _, cond := range allOf {
				if condMap, ok := cond.(map[string]any); ok {
					if thenBlock, ok := condMap["then"].(map[string]any); ok {
						walk(thenBlock)
					}
					if elseBlock, ok := condMap["else"].(map[string]any); ok {
						walk(elseBlock)
					}
				}
			}
		}
	}
	walk(schema)

	// Default unmapped keys to "mowgli"
	for key := range allKeys {
		if _, exists := mappings[key]; !exists {
			mappings[key] = "mowgli"
		}
	}
	return mappings
}

// flattenROS2YAML reads nested ROS2 YAML (node: ros__parameters: {k: v}) and
// returns a flat map of all parameter key-value pairs across all nodes. On a
// key collision across nodes the last writer wins (Go map iteration order is
// randomized, so a genuine collision with differing values resolves
// nondeterministically); callers must ensure parameter keys are unique across
// nodes.
func flattenROS2YAML(yamlData map[string]any) map[string]any {
	flat := map[string]any{}
	for _, nodeData := range yamlData {
		nodeMap, ok := nodeData.(map[string]any)
		if !ok {
			continue
		}
		rosParams, ok := nodeMap["ros__parameters"].(map[string]any)
		if !ok {
			continue
		}
		for k, v := range rosParams {
			flat[k] = v
		}
	}
	return flat
}

// nestToROS2YAML takes a flat param map and node mappings, and produces
// nested ROS2 YAML structure. It preserves any existing YAML content that
// is not covered by the schema.
func nestToROS2YAML(flat map[string]any, nodeMappings map[string]string, existingYAML map[string]any) map[string]any {
	result := map[string]any{}

	// Preserve existing top-level structure
	for nodeName, nodeData := range existingYAML {
		if nodeMap, ok := nodeData.(map[string]any); ok {
			cloned := map[string]any{}
			for k, v := range nodeMap {
				cloned[k] = v
			}
			result[nodeName] = cloned
		}
	}

	// Group flat params by node
	nodeParams := map[string]map[string]any{}
	for key, value := range flat {
		node := "mowgli" // default
		if n, ok := nodeMappings[key]; ok {
			node = n
		}
		if nodeParams[node] == nil {
			nodeParams[node] = map[string]any{}
		}
		nodeParams[node][key] = value
	}

	// Merge into result, preserving existing ros__parameters that aren't in flat
	for nodeName, params := range nodeParams {
		nodeMap, ok := result[nodeName].(map[string]any)
		if !ok {
			nodeMap = map[string]any{}
			result[nodeName] = nodeMap
		}
		rosParams, ok := nodeMap["ros__parameters"].(map[string]any)
		if !ok {
			rosParams = map[string]any{}
		}
		for k, v := range params {
			rosParams[k] = v
		}
		nodeMap["ros__parameters"] = rosParams
	}

	return result
}

// valuesEqual reports whether a config value equals a schema default,
// tolerating the numeric-type churn that YAML/JSON round-trips introduce
// (a template default of 5 may arrive as int 5, int64 5, or float64 5.0;
// booleans and strings compare directly). This is the predicate that keeps
// the installed mowgli_robot.yaml SPARSE: any key whose live value equals its
// schema default is dropped on write, so the ROS2 deep-merge falls through to
// the package template. Resetting a field to its default is therefore just
// "write the default value" — it disappears from the installed file.
func valuesEqual(a, b any) bool {
	if a == nil || b == nil {
		return a == nil && b == nil
	}
	// Numeric comparison first: JSON numbers decode to float64, YAML ints to
	// int, so a strict == would spuriously flag "5" != "5.0".
	if af, aok := asFloat64(a); aok {
		if bf, bok := asFloat64(b); bok {
			return af == bf
		}
	}
	if ab, aok := a.(bool); aok {
		if bb, bok := b.(bool); bok {
			return ab == bb
		}
	}
	return fmt.Sprintf("%v", a) == fmt.Sprintf("%v", b)
}

// pruneNestedKeys removes the given parameter keys from every node's
// ros__parameters block in a nested ROS2 YAML tree. nestToROS2YAML clones the
// pre-existing on-disk structure verbatim, so a key that was dropped from the
// flat map to keep the config sparse would otherwise survive in the clone;
// this scrubs it from the final output.
func pruneNestedKeys(nested map[string]any, keys map[string]bool) {
	if len(keys) == 0 {
		return
	}
	for _, nodeData := range nested {
		nodeMap, ok := nodeData.(map[string]any)
		if !ok {
			continue
		}
		rosParams, ok := nodeMap["ros__parameters"].(map[string]any)
		if !ok {
			continue
		}
		for key := range keys {
			delete(rosParams, key)
		}
	}
}

func coerceValue(value string, schemaType string) any {
	switch schemaType {
	case "boolean":
		return strings.EqualFold(value, "true")
	case "number":
		if f, err := strconv.ParseFloat(value, 64); err == nil {
			return f
		}
	case "integer":
		if i, err := strconv.ParseInt(value, 10, 64); err == nil {
			return i
		}
	}
	return value
}

func asFloat64(value any) (float64, bool) {
	switch v := value.(type) {
	case float64:
		return v, true
	case float32:
		return float64(v), true
	case int:
		return float64(v), true
	case int8:
		return float64(v), true
	case int16:
		return float64(v), true
	case int32:
		return float64(v), true
	case int64:
		return float64(v), true
	case uint:
		return float64(v), true
	case uint8:
		return float64(v), true
	case uint16:
		return float64(v), true
	case uint32:
		return float64(v), true
	case uint64:
		return float64(v), true
	case string:
		f, err := strconv.ParseFloat(strings.TrimSpace(v), 64)
		return f, err == nil
	default:
		return 0, false
	}
}

func formatLatLonDecimal(value float64) string {
	return strconv.FormatFloat(value, 'f', latLonDecimalPlaces, 64)
}

func serializeShellSettingValue(key string, value any) string {
	if fixedPrecisionEnvKeys[key] {
		if f, ok := asFloat64(value); ok {
			return fmt.Sprintf("%#v", formatLatLonDecimal(f))
		}
	}
	return fmt.Sprintf("%#v", value)
}

func applyFixedPrecisionGeoScalars(value any) any {
	switch typed := value.(type) {
	case map[string]any:
		out := make(map[string]any, len(typed))
		for key, child := range typed {
			if fixedPrecisionYAMLKeys[key] {
				if f, ok := asFloat64(child); ok {
					out[key] = fixedPrecisionFloat(f)
					continue
				}
			}
			out[key] = applyFixedPrecisionGeoScalars(child)
		}
		return out
	case []any:
		out := make([]any, len(typed))
		for i, child := range typed {
			out[i] = applyFixedPrecisionGeoScalars(child)
		}
		return out
	default:
		return value
	}
}

func marshalROS2YAMLWithGeoPrecision(nested map[string]any) ([]byte, error) {
	return yaml.Marshal(applyFixedPrecisionGeoScalars(nested))
}

func stringValue(value any, defaultValue string) string {
	if value == nil {
		return defaultValue
	}
	text := strings.TrimSpace(fmt.Sprintf("%v", value))
	if text == "" || text == "<nil>" {
		return defaultValue
	}
	return text
}

func boolStringValue(value any, defaultValue bool) string {
	if value == nil {
		if defaultValue {
			return "true"
		}
		return "false"
	}
	switch v := value.(type) {
	case bool:
		if v {
			return "true"
		}
		return "false"
	default:
		text := strings.ToLower(stringValue(value, ""))
		switch text {
		case "1", "true", "yes", "on":
			return "true"
		case "0", "false", "no", "off":
			return "false"
		}
	}
	if defaultValue {
		return "true"
	}
	return "false"
}

func normalizeGnssReceiverFamily(value any) string {
	switch strings.ToLower(stringValue(value, "auto")) {
	case "", "auto":
		return "auto"
	case "u-blox", "ublox":
		return "ublox"
	case "unicore":
		return "unicore"
	case "nmea":
		return "nmea"
	default:
		return strings.ToLower(stringValue(value, "auto"))
	}
}

func normalizeGnssProfile(value any) string {
	switch strings.ToLower(strings.ReplaceAll(stringValue(value, "runtime_only"), "-", "_")) {
	case "", "runtime_only", "balanced", "power_saving":
		return "runtime_only"
	case "high_precision", "survey", "rover_high_precision":
		return "rover_high_precision"
	case "debug", "rover_high_precision_debug":
		return "rover_high_precision_debug"
	case "factory_reset":
		return "factory_reset"
	default:
		return "runtime_only"
	}
}

func normalizeGnssSignalProfile(value any) string {
	switch strings.ToLower(strings.ReplaceAll(stringValue(value, "balanced"), "-", "_")) {
	case "", "balanced":
		return "balanced"
	case "minimal":
		return "minimal"
	case "ppp_optimized", "high_precision":
		return "high_precision"
	case "all_signals":
		return "all_signals"
	case "custom":
		return "custom"
	default:
		return "balanced"
	}
}

func normalizeGnssReceiverModel(value any) string {
	switch model := strings.ToUpper(strings.TrimSpace(stringValue(value, ""))); model {
	case "", "AUTO", "UNKNOWN", "UNKNOWN/AUTO", "AUTO/UNKNOWN":
		return ""
	default:
		return model
	}
}

func normalizeGnssProfileRate(value any) string {
	switch stringValue(value, "5") {
	case "1", "5", "7", "10":
		return stringValue(value, "5")
	default:
		return "5"
	}
}

func normalizeGnssSignalGroup(value any) string {
	fields := strings.Fields(stringValue(value, ""))
	return strings.Join(fields, " ")
}

func firstValue(flat map[string]any, keys ...string) any {
	for _, key := range keys {
		value, ok := flat[key]
		if !ok || value == nil {
			continue
		}
		switch typed := value.(type) {
		case string:
			if strings.TrimSpace(typed) == "" {
				continue
			}
		}
		return value
	}
	return nil
}

func gnssConnectionFromDevice(serialDevice string) string {
	switch {
	case strings.HasPrefix(serialDevice, "/dev/serial/by-id/"),
		strings.HasPrefix(serialDevice, "/dev/ttyUSB"),
		strings.HasPrefix(serialDevice, "/dev/ttyACM"):
		return "usb"
	case strings.HasPrefix(serialDevice, "/dev/ttyAMA"),
		strings.HasPrefix(serialDevice, "/dev/ttyS"),
		strings.HasPrefix(serialDevice, "/dev/ttyTHS"),
		strings.HasPrefix(serialDevice, "/dev/ttyHS"):
		return "uart"
	default:
		return "uart"
	}
}

func gnssCompatFromFlat(flat map[string]any) map[string]string {
	receiverFamily := normalizeGnssReceiverFamily(flat["gnss_receiver_family"])
	serialDevice := stringValue(flat["gnss_serial_device"], "/dev/ttyAMA4")
	serialBaud := stringValue(flat["gnss_serial_baud"], "921600")
	configBaud := stringValue(firstValue(flat, "gnss_config_baud", "gnss_serial_baud"), serialBaud)
	profile := normalizeGnssProfile(flat["gnss_profile"])
	signalProfile := normalizeGnssSignalProfile(flat["gnss_signal_profile"])
	profileRateHz := normalizeGnssProfileRate(firstValue(flat, "gnss_profile_rate_hz", "gnss_rate_hz"))
	ntripMountpoint := stringValue(flat["ntrip_mountpoint"], "NEAR")
	ntripGGAEnabled := "false"
	if strings.HasPrefix(strings.ToLower(ntripMountpoint), "near") {
		ntripGGAEnabled = "true"
	}

	// crtk.net is the public Centipede caster, whose well-known anonymous login
	// is "centipede/centipede". Only fall back to those credentials when the
	// receiver is actually pointed at that caster AND the operator has not
	// supplied (or has deliberately cleared) their own. Injecting "centipede"
	// unconditionally would (a) hand a custom caster the wrong credentials and
	// (b) silently re-enable the public caster for an operator who cleared the
	// fields to disable it.
	ntripHost := stringValue(flat["ntrip_host"], "crtk.net")
	ntripUser := stringValue(flat["ntrip_user"], "")
	ntripPassword := stringValue(flat["ntrip_password"], "")
	if strings.EqualFold(ntripHost, "crtk.net") {
		if ntripUser == "" {
			ntripUser = "centipede"
		}
		if ntripPassword == "" {
			ntripPassword = "centipede"
		}
	}

	return map[string]string{
		"GNSS_STACK":                "universal",
		"GNSS_STATUS_SOURCE":        "universal",
		"GNSS_RECEIVER_FAMILY":      receiverFamily,
		"GNSS_TRANSPORT":            "serial",
		"GNSS_SERIAL_DEVICE":        serialDevice,
		"GNSS_SERIAL_BAUD":          serialBaud,
		"GNSS_CONFIG_BAUD":          configBaud,
		"GNSS_PROFILE":              profile,
		"GNSS_SIGNAL_PROFILE":       signalProfile,
		"GNSS_PROFILE_RATE_HZ":      profileRateHz,
		"GNSS_BACKEND":              "universal",
		"GNSS_NTRIP_ENABLED":        boolStringValue(flat["ntrip_enabled"], true),
		"GNSS_NTRIP_HOST":           ntripHost,
		"GNSS_NTRIP_PORT":           stringValue(flat["ntrip_port"], "2101"),
		"GNSS_NTRIP_MOUNTPOINT":     ntripMountpoint,
		"GNSS_NTRIP_USERNAME":       ntripUser,
		"GNSS_NTRIP_PASSWORD":       ntripPassword,
		"GNSS_RTCM_FORWARDING":      "true",
		"GNSS_NTRIP_GGA_ENABLED":    ntripGGAEnabled,
		"GNSS_NTRIP_GGA_INTERVAL_S": stringValue(flat["gnss_ntrip_gga_interval_s"], "10"),
	}
}

func applyUniversalGnssCompatibility(flat map[string]any) map[string]string {
	compat := gnssCompatFromFlat(flat)

	flat["gnss_receiver_family"] = compat["GNSS_RECEIVER_FAMILY"]
	flat["gnss_serial_device"] = compat["GNSS_SERIAL_DEVICE"]
	if baud, err := strconv.Atoi(compat["GNSS_SERIAL_BAUD"]); err == nil {
		flat["gnss_serial_baud"] = baud
	} else {
		flat["gnss_serial_baud"] = compat["GNSS_SERIAL_BAUD"]
	}
	if configBaud, err := strconv.Atoi(compat["GNSS_CONFIG_BAUD"]); err == nil {
		flat["gnss_config_baud"] = configBaud
	} else {
		flat["gnss_config_baud"] = compat["GNSS_CONFIG_BAUD"]
	}
	flat["gnss_profile"] = compat["GNSS_PROFILE"]
	flat["gnss_signal_profile"] = compat["GNSS_SIGNAL_PROFILE"]
	if _, exists := flat["gnss_receiver_model"]; exists {
		flat["gnss_receiver_model"] = normalizeGnssReceiverModel(flat["gnss_receiver_model"])
	}
	if rateHz, err := strconv.Atoi(compat["GNSS_PROFILE_RATE_HZ"]); err == nil {
		flat["gnss_profile_rate_hz"] = rateHz
	} else {
		flat["gnss_profile_rate_hz"] = compat["GNSS_PROFILE_RATE_HZ"]
	}
	flat["gnss_signal_group"] = normalizeGnssSignalGroup(flat["gnss_signal_group"])
	delete(flat, "gnss_rate_hz")

	return compat
}

var legacyGnssEnvKeys = []string{
	"GPS_CONNECTION",
	"GPS_" + "RUNTIME_MODE",
	"GPS_" + "PROTOCOL",
	"GPS_" + "PORT",
	"GPS_" + "BY_ID",
	"GPS_" + "UART_DEVICE",
	"GPS_" + "BAUD",
	"GPS_" + "DEBUG_ENABLED",
	"GPS_" + "DEBUG_PORT",
	"GPS_" + "DEBUG_UART_DEVICE",
	"GPS_" + "DEBUG_BAUD",
	"GPS_" + "UART_RULE",
	"GPS_" + "DEBUG_UART_RULE",
	"UBLOX_" + "DEVICE_FAMILY",
	"UBLOX_" + "DEVICE_SERIAL_STRING",
	"UNICORE_" + "ROS_PACKAGE",
	"UNICORE_" + "ROS_EXECUTABLE",
	"UNICORE_COM_PORT",
	"UNICORE_IMAGE",
}

func writeRuntimeEnvFile(path string, updates map[string]string) error {
	var content string
	if file, err := os.ReadFile(path); err == nil {
		content = string(file)
	} else if !os.IsNotExist(err) {
		return err
	}

	lines := []string{}
	if content != "" {
		lines = strings.Split(strings.ReplaceAll(content, "\r\n", "\n"), "\n")
	} else {
		lines = []string{}
	}

	seen := map[string]bool{}
	for idx, line := range lines {
		trimmed := strings.TrimSpace(line)
		if trimmed == "" || strings.HasPrefix(trimmed, "#") {
			continue
		}
		eq := strings.IndexByte(line, '=')
		if eq <= 0 {
			continue
		}
		key := strings.TrimSpace(line[:eq])
		legacyKey := false
		for _, candidate := range legacyGnssEnvKeys {
			if key == candidate {
				lines[idx] = ""
				legacyKey = true
				break
			}
		}
		if legacyKey {
			continue
		}
		value, ok := updates[key]
		if !ok {
			continue
		}
		lines[idx] = fmt.Sprintf("%s=%s", key, value)
		seen[key] = true
	}

	keys := make([]string, 0, len(updates))
	for key := range updates {
		if !seen[key] {
			keys = append(keys, key)
		}
	}
	sort.Strings(keys)
	for _, key := range keys {
		lines = append(lines, fmt.Sprintf("%s=%s", key, updates[key]))
	}

	filtered := make([]string, 0, len(lines))
	for _, line := range lines {
		if strings.TrimSpace(line) == "" {
			continue
		}
		filtered = append(filtered, line)
	}

	out := strings.TrimRight(strings.Join(filtered, "\n"), "\n") + "\n"
	if err := os.MkdirAll(filepath.Dir(path), 0755); err != nil {
		return err
	}
	return writePreservingPerms(path, []byte(out))
}

// applyMowgliOverlay modifies the settings schema to add Mowgli-specific
// options: adds "Mowgli" to the OM_MOWER enum, moves OM_HARDWARE_VERSION and
// OM_MOWER_ESC_TYPE behind conditional allOf rules, and adds OM_NO_COMMS for
// Mowgli boards.
func applyMowgliOverlay(schema map[string]any) map[string]any {
	props, ok := schema["properties"].(map[string]any)
	if !ok {
		return schema
	}
	hw, ok := props["important_settings"].(map[string]any)
	if !ok {
		return schema
	}
	hwProps, ok := hw["properties"].(map[string]any)
	if !ok {
		return schema
	}

	// Add "Mowgli" to OM_MOWER enum if not already present.
	if omMower, ok := hwProps["OM_MOWER"].(map[string]any); ok {
		if enum, ok := omMower["enum"].([]any); ok {
			found := false
			for _, v := range enum {
				if v == "Mowgli" {
					found = true
					break
				}
			}
			if !found {
				omMower["enum"] = append(enum, "Mowgli")
			}
		}
	}

	// Extract OM_HARDWARE_VERSION and OM_MOWER_ESC_TYPE from base properties.
	hwVersion := hwProps["OM_HARDWARE_VERSION"]
	escType := hwProps["OM_MOWER_ESC_TYPE"]
	delete(hwProps, "OM_HARDWARE_VERSION")
	delete(hwProps, "OM_MOWER_ESC_TYPE")

	// Build conditional allOf rules.
	nonMowgliThenProps := map[string]any{}
	if hwVersion != nil {
		nonMowgliThenProps["OM_HARDWARE_VERSION"] = hwVersion
	}
	if escType != nil {
		nonMowgliThenProps["OM_MOWER_ESC_TYPE"] = escType
	}

	nonMowgliCond := map[string]any{
		"if": map[string]any{
			"properties": map[string]any{
				"OM_MOWER": map[string]any{
					"not": map[string]any{"const": "Mowgli"},
				},
			},
		},
		"then": map[string]any{
			"properties": nonMowgliThenProps,
		},
	}

	mowgliCond := map[string]any{
		"if": map[string]any{
			"properties": map[string]any{
				"OM_MOWER": map[string]any{"const": "Mowgli"},
			},
		},
		"then": map[string]any{
			"properties": map[string]any{
				"OM_NO_COMMS": map[string]any{
					"type":    "boolean",
					"title":   "No Comms Mode",
					"default": true,
				},
			},
		},
	}

	hw["allOf"] = []any{nonMowgliCond, mowgliCond}

	return schema
}

func getSchema(dbProvider types.IDBProvider) (map[string]any, error) {
	schemaCacheMu.RLock()
	if schemaCache != nil && time.Since(schemaCacheTime) < schemaCacheTTL {
		cached := schemaCache
		schemaCacheMu.RUnlock()
		return cached, nil
	}
	schemaCacheMu.RUnlock()

	// Load local Mowgli schema (no upstream fetch)
	var schema map[string]any
	localFile, err := os.ReadFile("asserts/mower_config.schema.json")
	if err != nil {
		return nil, fmt.Errorf("failed to read local schema: %w", err)
	}
	if err := json.Unmarshal(localFile, &schema); err != nil {
		return nil, fmt.Errorf("invalid local schema JSON: %w", err)
	}

	// Apply the Mowgli overlay (adds "Mowgli" to the OM_MOWER enum and the
	// conditional hardware fields). The base OpenMower schema does not carry it,
	// so without this the GUI would never offer "Mowgli" as a mower model. Done
	// once here, at load time, so every schema consumer sees it consistently.
	schema = applyMowgliOverlay(schema)

	schemaCacheMu.Lock()
	schemaCache = schema
	schemaCacheTime = time.Now()
	schemaCacheMu.Unlock()

	return schema, nil
}

// ============================================================================
// Legacy shell-config endpoints (kept for backward compatibility)
// ============================================================================

// PostSettings saves settings to the mower_config.sh file.
//
// @Summary saves the settings to the mower_config.sh file
// @Description saves the settings to the mower_config.sh file
// @Tags settings
// @Accept json
// @Produce json
// @Param settings body map[string]interface{} true "settings key-value map"
// @Success 200 {object} OkResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings [post]
func PostSettings(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.POST("/settings", func(c *gin.Context) {
		var settingsPayload map[string]any
		err := c.BindJSON(&settingsPayload)
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		mowerConfigFile, err := dbProvider.Get("system.mower.configFile")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		var settings = map[string]any{}
		configFileContent, err := os.ReadFile(string(mowerConfigFile))
		if err == nil {
			parse, err := godotenv.Parse(strings.NewReader(string(configFileContent)))
			if err == nil {
				for s, s2 := range parse {
					settings[s] = s2
				}
			}
		}

		schema, err := getSchema(dbProvider)
		defaults := map[string]any{}
		knownKeys := map[string]bool{}
		if err == nil {
			extractDefaults(schema, defaults)
			extractAllKeys(schema, knownKeys)
			for key, value := range defaults {
				if _, exists := settings[key]; !exists {
					settings[key] = value
				}
			}
		}

		customEnvVars := map[string]string{}
		for key, value := range settings {
			if !knownKeys[key] {
				if key != "custom_environment" {
					customEnvVars[key] = fmt.Sprintf("%v", value)
				}
				delete(settings, key)
			}
		}
		if customEnvObj, ok := settingsPayload["custom_environment"]; ok {
			customEnvVars = map[string]string{}
			if customEnv, ok := customEnvObj.(map[string]any); ok {
				for k, v := range customEnv {
					customEnvVars[k] = fmt.Sprintf("%v", v)
				}
			}
			delete(settingsPayload, "custom_environment")
		}
		for key, value := range settingsPayload {
			settings[key] = value
		}
		for k, v := range customEnvVars {
			settings[k] = v
		}

		var fileContent string
		for key, value := range settings {
			if key == "custom_environment" {
				continue
			}
			if value == true {
				value = "True"
			}
			if value == false {
				value = "False"
			}
			fileContent += "export " + key + "=" + serializeShellSettingValue(key, value) + "\n"
		}
		if err = os.MkdirAll(filepath.Dir(string(mowerConfigFile)), 0755); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		err = writePreservingPerms(string(mowerConfigFile), []byte(fileContent))
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, OkResponse{})
	})
}

// GetSettings returns a JSON object with the settings.
//
// @Summary returns a JSON object with the settings
// @Description returns a JSON object with the settings
// @Tags settings
// @Produce json
// @Success 200 {object} GetSettingsResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings [get]
func GetSettings(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings", func(c *gin.Context) {
		mowerConfigFilePath, err := dbProvider.Get("system.mower.configFile")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		file, err := os.ReadFile(string(mowerConfigFilePath))
		if err != nil {
			if os.IsNotExist(err) {
				c.JSON(200, GetSettingsResponse{Settings: map[string]any{}})
				return
			}
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		settings, err := godotenv.Parse(strings.NewReader(string(file)))
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		schema, _ := getSchema(dbProvider)
		knownKeys := map[string]bool{}
		keyTypes := map[string]string{}
		if schema != nil {
			extractAllKeys(schema, knownKeys)
			extractKeyTypes(schema, keyTypes)
		}

		finalSettings := map[string]any{}
		customEnv := map[string]string{}
		for k, v := range settings {
			if knownKeys[k] || k == "custom_environment" {
				if t, ok := keyTypes[k]; ok {
					finalSettings[k] = coerceValue(v, t)
				} else {
					finalSettings[k] = v
				}
			} else {
				customEnv[k] = v
			}
		}
		if len(customEnv) > 0 {
			finalSettings["custom_environment"] = customEnv
		}

		c.JSON(200, GetSettingsResponse{Settings: finalSettings})
	})
}

// ============================================================================
// Schema endpoint
// ============================================================================

var (
	schemaCache     map[string]any
	schemaCacheMu   sync.RWMutex
	schemaCacheTime time.Time
	schemaCacheTTL  = 1 * time.Hour
)

// GetSettingsSchema returns the mower config JSON Schema.
//
// @Summary returns the mower config JSON Schema
// @Description returns the JSON Schema for mower configuration parameters
// @Tags settings
// @Produce json
// @Success 200 {object} map[string]interface{}
// @Failure 500 {object} ErrorResponse
// @Router /settings/schema [get]
func GetSettingsSchema(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/schema", func(c *gin.Context) {
		schema, err := getSchema(dbProvider)
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		c.JSON(200, schema)
	})
}

// ============================================================================
// YAML settings endpoints — primary for Mowgli ROS2
// ============================================================================

// GetSettingsYAML reads mowgli_robot.yaml, flattens the nested ROS2 YAML,
// and returns a flat key-value JSON object for the settings form.
//
// @Summary returns the current YAML mower configuration
// @Description returns the current YAML mower configuration values as a flat key-value map
// @Tags settings
// @Produce json
// @Success 200 {object} map[string]interface{}
// @Failure 500 {object} ErrorResponse
// @Router /settings/yaml [get]
func GetSettingsYAML(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/yaml", func(c *gin.Context) {
		configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		file, err := os.ReadFile(string(configFilePath))
		if err != nil {
			if os.IsNotExist(err) {
				// Return schema defaults when file doesn't exist
				schema, schemaErr := getSchema(dbProvider)
				if schemaErr != nil {
					c.JSON(200, map[string]any{})
					return
				}
				defaults := map[string]any{}
				extractDefaults(schema, defaults)
				c.JSON(200, defaults)
				return
			}
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}

		var yamlData map[string]any
		if err := yaml.Unmarshal(file, &yamlData); err != nil {
			c.JSON(500, ErrorResponse{Error: "invalid YAML: " + err.Error()})
			return
		}

		flat := flattenROS2YAML(yamlData)

		// Merge schema defaults for missing keys
		schema, err := getSchema(dbProvider)
		if err == nil {
			defaults := map[string]any{}
			extractDefaults(schema, defaults)
			for key, value := range defaults {
				if _, exists := flat[key]; !exists {
					flat[key] = value
				}
			}
		}

		applyUniversalGnssCompatibility(flat)

		c.JSON(200, flat)
	})
}

// GetSettingsYAMLDefaults returns the flat map of schema-default values —
// the GUI's authoritative source of "default" for each parameter. Because the
// GUI backend runs in a container that does NOT ship the ROS2 package template
// (mowgli_bringup/config/mowgli_robot.yaml, the true default source consumed by
// robot_config_util's deep-merge), the JSON schema `default` values stand in
// for it. They SHOULD match the template; a divergence is a schema bug, not a
// runtime one. The frontend uses this to (a) show which values are
// operator-overridden vs at default and (b) implement per-field "reset to
// default" (writing the default value, which PostSettingsYAML then prunes to
// keep the installed config sparse).
//
// @Summary returns the schema default value for every known parameter
// @Description returns a flat key-value map of default values (the reset-to-default source)
// @Tags settings
// @Produce json
// @Success 200 {object} map[string]interface{}
// @Failure 500 {object} ErrorResponse
// @Router /settings/yaml/defaults [get]
func GetSettingsYAMLDefaults(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.GET("/settings/yaml/defaults", func(c *gin.Context) {
		schema, err := getSchema(dbProvider)
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		defaults := map[string]any{}
		extractDefaults(schema, defaults)
		c.JSON(200, defaults)
	})
}

// PostSettingsYAML receives flat key-value settings from the form,
// nests them into proper ROS2 YAML structure, and writes mowgli_robot.yaml.
//
// @Summary saves the mower configuration as YAML
// @Description saves the mower configuration as YAML to mowgli_robot.yaml
// @Tags settings
// @Accept json
// @Produce json
// @Param settings body map[string]interface{} true "flat key-value settings map"
// @Success 200 {object} OkResponse
// @Failure 400 {object} ErrorResponse
// @Failure 500 {object} ErrorResponse
// @Router /settings/yaml [post]
func PostSettingsYAML(r *gin.RouterGroup, dbProvider types.IDBProvider) gin.IRoutes {
	return r.POST("/settings/yaml", func(c *gin.Context) {
		var payload map[string]any
		if err := c.BindJSON(&payload); err != nil {
			c.JSON(400, ErrorResponse{Error: err.Error()})
			return
		}

		configFilePath, err := dbProvider.Get("system.mower.yamlConfigFile")
		if err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}

		// Read existing YAML to preserve unknown parameters
		existingYAML := map[string]any{}
		file, err := os.ReadFile(string(configFilePath))
		if err == nil {
			_ = yaml.Unmarshal(file, &existingYAML)
		}

		// Flatten existing to get current values
		existing := flattenROS2YAML(existingYAML)

		// Merge schema defaults
		schema, err := getSchema(dbProvider)
		nodeMappings := map[string]string{}
		defaults := map[string]any{}
		if err == nil {
			extractDefaults(schema, defaults)
			for key, value := range defaults {
				if _, exists := existing[key]; !exists {
					existing[key] = value
				}
			}
			nodeMappings = extractNodeMappings(schema)
		}

		// Merge payload on top. A null value is an explicit delete request
		// (from the Advanced "Remove parameter" action, or a per-field
		// "reset to default" for a key that has no schema default) — drop the
		// key so it is removed from the YAML rather than written back as
		// "key: null".
		for key, value := range payload {
			if value == nil {
				delete(existing, key)
			} else {
				existing[key] = value
			}
		}
		gnssEnvUpdates := applyUniversalGnssCompatibility(existing)

		// Keep the installed config SPARSE: any key whose value equals its
		// schema default is pruned so the ROS2 launch-time deep-merge falls
		// through to the package template (the source of defaults). This is
		// also how "reset to default" persists — the field is written with
		// its default value from the form, and pruned here. prunedKeys is
		// applied to the nested output too, because nestToROS2YAML preserves
		// pre-existing on-disk keys and would otherwise resurrect them.
		prunedKeys := map[string]bool{}
		for key, def := range defaults {
			if cur, exists := existing[key]; exists && valuesEqual(cur, def) {
				delete(existing, key)
				prunedKeys[key] = true
			}
		}

		// Nest back into ROS2 YAML structure
		nested := nestToROS2YAML(existing, nodeMappings, existingYAML)
		pruneNestedKeys(nested, prunedKeys)

		// Marshal with YAML comments header
		out, err := marshalROS2YAMLWithGeoPrecision(nested)
		if err != nil {
			c.JSON(500, ErrorResponse{Error: "failed to marshal YAML: " + err.Error()})
			return
		}

		header := "# Mowgli Robot Configuration — managed by mowglinext-gui\n" +
			"# This file is the single source of truth for robot parameters.\n" +
			"# Changes made here are picked up on container restart.\n\n"

		if err := os.MkdirAll(filepath.Dir(string(configFilePath)), 0755); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}
		if err := writePreservingPerms(string(configFilePath), []byte(header+string(out))); err != nil {
			c.JSON(500, ErrorResponse{Error: err.Error()})
			return
		}

		envFilePath, err := dbProvider.Get("system.mower.runtimeEnvFile")
		if err == nil && len(envFilePath) > 0 {
			if err := writeRuntimeEnvFile(string(envFilePath), gnssEnvUpdates); err != nil {
				c.JSON(500, ErrorResponse{Error: "failed to update runtime env: " + err.Error()})
				return
			}
		}

		log.Printf("Saved mowgli_robot.yaml (%d bytes)", len(out)+len(header))
		c.JSON(200, OkResponse{})
	})
}
