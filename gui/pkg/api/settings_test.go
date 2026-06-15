package api

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"strings"
	"testing"
	"time"

	"github.com/cedbossneo/mowglinext/pkg/types"
	"github.com/gin-gonic/gin"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

func setupSettingsRouter(dbProvider types.IDBProvider) *gin.Engine {
	gin.SetMode(gin.TestMode)
	r := gin.New()
	group := r.Group("/api")
	SettingsRoutes(group, dbProvider)
	return r
}

// chdirToGuiRoot moves the working directory to the gui module root (two levels
// up from pkg/api), where asserts/ lives, and restores it on cleanup. Used by
// tests that need the real local schema rather than a synthetic one.
func chdirToGuiRoot(t *testing.T) {
	t.Helper()
	orig, err := os.Getwd()
	if err != nil {
		t.Fatalf("getwd: %v", err)
	}
	if err := os.Chdir("../.."); err != nil {
		t.Fatalf("chdir to gui root: %v", err)
	}
	if _, err := os.Stat("asserts"); err != nil {
		t.Fatalf("expected asserts/ at gui root: %v", err)
	}
	t.Cleanup(func() { _ = os.Chdir(orig) })
}

// seedSchemaCache primes the schema cache so the schema-driven known-key filter
// recognises the given keys. getSchema otherwise loads asserts/mower_config.schema.json
// relative to the working directory, which isn't present under pkg/api during tests.
func seedSchemaCache(t *testing.T, keys ...string) {
	t.Helper()
	props := map[string]any{}
	for _, k := range keys {
		props[k] = map[string]any{"type": "string", "x-environment-variable": k}
	}
	schemaCacheMu.Lock()
	schemaCache = map[string]any{
		"type": "object",
		"properties": map[string]any{
			"important_settings": map[string]any{
				"type":       "object",
				"properties": props,
			},
		},
	}
	schemaCacheTime = time.Now()
	schemaCacheMu.Unlock()
	t.Cleanup(resetSchemaCache)
}

func TestGetSettings_Success(t *testing.T) {
	seedSchemaCache(t, "OM_DATUM_LAT", "OM_USE_NTRIP", "OM_TOOL_WIDTH")
	configFile := createTempConfigFile(t, `export OM_DATUM_LAT="48.123"
export OM_USE_NTRIP="True"
export OM_TOOL_WIDTH="0.13"
`)

	db := types.NewMockDBProvider()
	db.Set("system.mower.configFile", []byte(configFile))

	router := setupSettingsRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/settings", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	var resp GetSettingsResponse
	err := json.Unmarshal(w.Body.Bytes(), &resp)
	require.NoError(t, err)

	assert.Equal(t, "48.123", resp.Settings["OM_DATUM_LAT"])
	assert.Equal(t, "True", resp.Settings["OM_USE_NTRIP"])
	assert.Equal(t, "0.13", resp.Settings["OM_TOOL_WIDTH"])
}

func TestGetSettings_FileNotFound(t *testing.T) {
	db := types.NewMockDBProvider()
	db.Set("system.mower.configFile", []byte("/nonexistent/config.sh"))

	router := setupSettingsRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/settings", nil)
	router.ServeHTTP(w, req)

	// A configured-but-missing legacy .sh file degrades gracefully to an empty
	// settings object (the YAML flow is now primary). Only a missing config-path
	// KEY is a hard 500 — see TestGetSettings_NoConfigKey.
	assert.Equal(t, http.StatusOK, w.Code)
	var resp GetSettingsResponse
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &resp))
	assert.Empty(t, resp.Settings)
}

func TestGetSettings_NoConfigKey(t *testing.T) {
	db := types.NewMockDBProvider()
	// Don't set system.mower.configFile

	router := setupSettingsRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/settings", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusInternalServerError, w.Code)
}

func TestPostSettings_NewFile(t *testing.T) {
	configFile := createTempConfigFile(t, "")

	db := types.NewMockDBProvider()
	db.Set("system.mower.configFile", []byte(configFile))

	router := setupSettingsRouter(db)

	payload := map[string]any{
		"OM_DATUM_LAT":  "48.999",
		"OM_USE_NTRIP":  true,
		"OM_TOOL_WIDTH": 0.15,
	}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	// Verify file was written
	content, err := os.ReadFile(configFile)
	require.NoError(t, err)

	fileContent := string(content)
	assert.Contains(t, fileContent, "export OM_DATUM_LAT=")
	assert.Contains(t, fileContent, "48.999")
	assert.Contains(t, fileContent, "export OM_USE_NTRIP=")
	assert.Contains(t, fileContent, "export OM_TOOL_WIDTH=")
}

func TestPostSettings_MergesExistingSettings(t *testing.T) {
	// OM_DATUM_LAT is a schema-known key so the new value survives; OM_EXISTING_KEY
	// is unknown and is preserved as a custom-environment passthrough.
	seedSchemaCache(t, "OM_DATUM_LAT")
	configFile := createTempConfigFile(t, `export OM_DATUM_LAT="48.123"
export OM_EXISTING_KEY="keep_me"
`)

	db := types.NewMockDBProvider()
	db.Set("system.mower.configFile", []byte(configFile))

	router := setupSettingsRouter(db)

	// Send only one new setting
	payload := map[string]any{
		"OM_DATUM_LAT": "99.999",
	}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	// Verify existing settings were preserved
	content, err := os.ReadFile(configFile)
	require.NoError(t, err)

	fileContent := string(content)
	assert.Contains(t, fileContent, "OM_EXISTING_KEY")
	assert.Contains(t, fileContent, "keep_me")
	assert.Contains(t, fileContent, "99.999")
}

func TestPostSettings_BooleanConversion(t *testing.T) {
	configFile := createTempConfigFile(t, "")

	db := types.NewMockDBProvider()
	db.Set("system.mower.configFile", []byte(configFile))

	router := setupSettingsRouter(db)

	payload := map[string]any{
		"OM_ENABLE_MOWER": true,
		"OM_USE_NTRIP":    false,
	}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	content, err := os.ReadFile(configFile)
	require.NoError(t, err)

	fileContent := string(content)
	assert.Contains(t, fileContent, "True")
	assert.Contains(t, fileContent, "False")
}

func TestPostSettings_InvalidJSON(t *testing.T) {
	db := types.NewMockDBProvider()
	db.Set("system.mower.configFile", []byte("/tmp/test.sh"))

	router := setupSettingsRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings", strings.NewReader("not json"))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	// Gin's BindJSON returns 400 for malformed JSON
	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func resetSchemaCache() {
	schemaCacheMu.Lock()
	schemaCache = nil
	schemaCacheTime = time.Time{}
	schemaCacheMu.Unlock()
}

// Upstream schema fetching was removed — getSchema now loads the local
// asserts/mower_config.schema.json and applies the Mowgli overlay at load time.
// This verifies the served schema carries that overlay (the regression this
// replaced: the overlay had become dead code, so "Mowgli" silently vanished
// from the OM_MOWER enum).
func TestGetSettingsSchema_AppliesMowgliOverlay(t *testing.T) {
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	// Provide a local schema (base OpenMower shape, without the overlay).
	origDir, _ := os.Getwd()
	tmpDir := t.TempDir()
	require.NoError(t, os.MkdirAll(tmpDir+"/asserts", 0755))
	localSchema := `{"type":"object","properties":{"important_settings":{"title":"Hardware Settings","type":"object","properties":{"OM_HARDWARE_VERSION":{"type":"string","enum":["0_13_X"],"x-environment-variable":"OM_HARDWARE_VERSION"},"OM_MOWER":{"type":"string","enum":["YardForce500","CUSTOM"],"x-environment-variable":"OM_MOWER"},"OM_MOWER_ESC_TYPE":{"type":"string","enum":["xesc_mini"],"x-environment-variable":"OM_MOWER_ESC_TYPE"}}}}}`
	require.NoError(t, os.WriteFile(tmpDir+"/asserts/mower_config.schema.json", []byte(localSchema), 0644))
	require.NoError(t, os.Chdir(tmpDir))
	defer os.Chdir(origDir)

	router := setupSettingsRouter(types.NewMockDBProvider())

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/settings/schema", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	var result map[string]any
	require.NoError(t, json.Unmarshal(w.Body.Bytes(), &result))
	assert.Equal(t, "object", result["type"])

	props := result["properties"].(map[string]any)
	hw := props["important_settings"].(map[string]any)
	hwProps := hw["properties"].(map[string]any)

	// "Mowgli" added to the OM_MOWER enum by the overlay.
	omMower := hwProps["OM_MOWER"].(map[string]any)
	assert.Contains(t, omMower["enum"].([]any), "Mowgli")

	// HW version + ESC type moved out of base props into the conditional allOf.
	assert.NotContains(t, hwProps, "OM_HARDWARE_VERSION")
	assert.NotContains(t, hwProps, "OM_MOWER_ESC_TYPE")
	allOf := hw["allOf"].([]any)
	assert.GreaterOrEqual(t, len(allOf), 2)
}

func TestGetSettingsSchema_FallbackToLocal(t *testing.T) {
	resetSchemaCache()

	// Point to a bad upstream URL
	db := types.NewMockDBProvider()
	db.Set("system.mower.schemaURL", []byte("http://127.0.0.1:1/nonexistent"))

	// Provide local fallback
	origDir, _ := os.Getwd()
	tmpDir := t.TempDir()
	require.NoError(t, os.MkdirAll(tmpDir+"/asserts", 0755))
	localSchema := `{"type":"object","properties":{"important_settings":{"title":"Hardware Settings","type":"object","properties":{"OM_MOWER":{"type":"string","enum":["CUSTOM"]}}}}}`
	require.NoError(t, os.WriteFile(tmpDir+"/asserts/mower_config.schema.json", []byte(localSchema), 0644))
	require.NoError(t, os.Chdir(tmpDir))
	defer os.Chdir(origDir)

	router := setupSettingsRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/settings/schema", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	var result map[string]any
	err := json.Unmarshal(w.Body.Bytes(), &result)
	require.NoError(t, err)
	assert.Equal(t, "object", result["type"])
}

func TestGetSettingsSchema_NoUpstreamNoLocal(t *testing.T) {
	resetSchemaCache()

	db := types.NewMockDBProvider()
	db.Set("system.mower.schemaURL", []byte("http://127.0.0.1:1/nonexistent"))

	// No local fallback either
	origDir, _ := os.Getwd()
	tmpDir := t.TempDir()
	require.NoError(t, os.Chdir(tmpDir))
	defer os.Chdir(origDir)

	router := setupSettingsRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/settings/schema", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusInternalServerError, w.Code)
}

func TestApplyMowgliOverlay(t *testing.T) {
	schema := map[string]any{
		"type": "object",
		"properties": map[string]any{
			"important_settings": map[string]any{
				"title": "Hardware Settings",
				"type":  "object",
				"properties": map[string]any{
					"OM_MOWER": map[string]any{
						"type": "string",
						"enum": []any{"YardForce500", "CUSTOM"},
					},
					"OM_HARDWARE_VERSION": map[string]any{
						"type": "string",
						"enum": []any{"0_13_X"},
					},
					"OM_MOWER_ESC_TYPE": map[string]any{
						"type": "string",
						"enum": []any{"xesc_mini"},
					},
					"OM_MOWER_GAMEPAD": map[string]any{
						"type": "string",
					},
				},
			},
		},
	}

	result := applyMowgliOverlay(schema)

	hw := result["properties"].(map[string]any)["important_settings"].(map[string]any)
	hwProps := hw["properties"].(map[string]any)

	// Mowgli should be added to OM_MOWER enum
	omMower := hwProps["OM_MOWER"].(map[string]any)
	assert.Contains(t, omMower["enum"].([]any), "Mowgli")

	// OM_HARDWARE_VERSION and ESC_TYPE should be removed from base props
	assert.NotContains(t, hwProps, "OM_HARDWARE_VERSION")
	assert.NotContains(t, hwProps, "OM_MOWER_ESC_TYPE")

	// Gamepad should remain
	assert.Contains(t, hwProps, "OM_MOWER_GAMEPAD")

	// allOf should have 2 conditions
	allOf := hw["allOf"].([]any)
	require.Len(t, allOf, 2)

	// First condition: non-Mowgli shows HW version + ESC type
	nonMowgli := allOf[0].(map[string]any)
	nonMowgliThen := nonMowgli["then"].(map[string]any)
	nonMowgliProps := nonMowgliThen["properties"].(map[string]any)
	assert.Contains(t, nonMowgliProps, "OM_HARDWARE_VERSION")
	assert.Contains(t, nonMowgliProps, "OM_MOWER_ESC_TYPE")

	// Second condition: Mowgli shows OM_NO_COMMS
	mowgli := allOf[1].(map[string]any)
	mowgliThen := mowgli["then"].(map[string]any)
	mowgliProps := mowgliThen["properties"].(map[string]any)
	assert.Contains(t, mowgliProps, "OM_NO_COMMS")
	omNoComms := mowgliProps["OM_NO_COMMS"].(map[string]any)
	assert.Equal(t, true, omNoComms["default"])
}

func TestApplyMowgliOverlay_AlreadyHasMowgli(t *testing.T) {
	schema := map[string]any{
		"type": "object",
		"properties": map[string]any{
			"important_settings": map[string]any{
				"type": "object",
				"properties": map[string]any{
					"OM_MOWER": map[string]any{
						"type": "string",
						"enum": []any{"YardForce500", "Mowgli"},
					},
					"OM_HARDWARE_VERSION": map[string]any{"type": "string"},
				},
			},
		},
	}

	result := applyMowgliOverlay(schema)
	hw := result["properties"].(map[string]any)["important_settings"].(map[string]any)
	omMower := hw["properties"].(map[string]any)["OM_MOWER"].(map[string]any)

	// Should not duplicate Mowgli
	count := 0
	for _, v := range omMower["enum"].([]any) {
		if v == "Mowgli" {
			count++
		}
	}
	assert.Equal(t, 1, count)
}

func TestGetSettingsYAML_Success(t *testing.T) {
	yamlFile := createTempYAMLFile(t, `mowgli:
  ros__parameters:
    datum_lat: 48.123
    ntrip_enabled: true
    gnss_receiver_family: auto
`)

	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))

	router := setupSettingsRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/settings/yaml", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	var result map[string]any
	err := json.Unmarshal(w.Body.Bytes(), &result)
	require.NoError(t, err)
	assert.Equal(t, 48.123, result["datum_lat"])
	assert.Equal(t, true, result["ntrip_enabled"])
	assert.Equal(t, "auto", result["gnss_receiver_family"])
}

func TestGetSettingsYAML_FileNotExist_ReturnsEmpty(t *testing.T) {
	// This asserts the real schema's GNSS defaults, so run from the gui root
	// where asserts/mower_config.schema.json lives and start from a clean cache.
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte("/nonexistent/config.yaml"))

	router := setupSettingsRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/settings/yaml", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	var result map[string]any
	err := json.Unmarshal(w.Body.Bytes(), &result)
	require.NoError(t, err)
	assert.Equal(t, "auto", result["gnss_receiver_family"])
	assert.Equal(t, "runtime_only", result["gnss_profile"])
	assert.Equal(t, "balanced", result["gnss_signal_profile"])
	assert.Equal(t, float64(5), result["gnss_profile_rate_hz"])
	assert.Equal(t, float64(921600), result["gnss_config_baud"])
}

func TestGetSettingsYAML_NoConfigKey(t *testing.T) {
	db := types.NewMockDBProvider()

	router := setupSettingsRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/settings/yaml", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusInternalServerError, w.Code)
}

func TestPostSettingsYAML_NewFile(t *testing.T) {
	yamlFile := createTempYAMLFile(t, "")
	envFile := createTempConfigFile(t, "ROS_DOMAIN_ID=0\n")

	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))
	db.Set("system.mower.runtimeEnvFile", []byte(envFile))

	router := setupSettingsRouter(db)

	payload := map[string]any{
		"datum_lat":             48.999,
		"ntrip_enabled":         true,
		"gnss_receiver_family":  "unicore",
		"gnss_serial_device":    "/dev/serial/by-id/usb-gnss",
		"gnss_serial_baud":      921600,
		"gnss_config_baud":      460800,
		"gnss_profile":          "rover_high_precision",
		"gnss_signal_profile":   "all_signals",
		"gnss_profile_rate_hz":  5,
		"gnss_signal_group":     "3 6",
		"gnss_unicore_pvt_algorithm": "MULTI",
	}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/yaml", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	content, err := os.ReadFile(yamlFile)
	require.NoError(t, err)
	assert.Contains(t, string(content), "datum_lat: 48.999")
	assert.Contains(t, string(content), "ntrip_enabled: true")
	assert.Contains(t, string(content), "gnss_receiver_family: unicore")
	assert.Contains(t, string(content), "gnss_serial_device: /dev/serial/by-id/usb-gnss")
	assert.Contains(t, string(content), "gnss_serial_baud: 921600")
	assert.Contains(t, string(content), "gnss_config_baud: 460800")
	assert.Contains(t, string(content), "gnss_profile: rover_high_precision")
	assert.Contains(t, string(content), "gnss_signal_profile: all_signals")
	assert.Contains(t, string(content), "gnss_profile_rate_hz: 5")
	assert.Contains(t, string(content), "gnss_signal_group: 3 6")
	assert.Contains(t, string(content), "gnss_unicore_pvt_algorithm: MULTI")

	envContent, err := os.ReadFile(envFile)
	require.NoError(t, err)
	legacyProtocol := "GPS_" + "PROTOCOL=UBX"
	legacyByID := "GPS_" + "BY_ID=/dev/serial/by-id/usb-gnss"
	assert.Contains(t, string(envContent), "GNSS_RECEIVER_FAMILY=unicore")
	assert.Contains(t, string(envContent), "GNSS_SERIAL_DEVICE=/dev/serial/by-id/usb-gnss")
	assert.Contains(t, string(envContent), "GNSS_SERIAL_BAUD=921600")
	assert.Contains(t, string(envContent), "GNSS_CONFIG_BAUD=460800")
	assert.Contains(t, string(envContent), "GNSS_PROFILE=rover_high_precision")
	assert.Contains(t, string(envContent), "GNSS_SIGNAL_PROFILE=all_signals")
	assert.Contains(t, string(envContent), "GNSS_PROFILE_RATE_HZ=5")
	assert.Contains(t, string(envContent), "GNSS_BACKEND=universal")
	assert.Contains(t, string(envContent), "GNSS_NTRIP_ENABLED=true")
	assert.NotContains(t, string(envContent), "GNSS_SIGNAL_GROUP=3 6")
	assert.NotContains(t, string(envContent), legacyProtocol)
	assert.NotContains(t, string(envContent), legacyByID)
}

func TestPostSettingsYAML_MergesExisting(t *testing.T) {
	yamlFile := createTempYAMLFile(t, `mowgli:
  ros__parameters:
    datum_lat: 48.123
    gnss_receiver_family: auto
    extra_existing: keep_me
`)
	envFile := createTempConfigFile(t, "")

	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))
	db.Set("system.mower.runtimeEnvFile", []byte(envFile))

	router := setupSettingsRouter(db)

	payload := map[string]any{
		"datum_lat":            99.999,
		"gnss_receiver_family": "nmea",
		"gnss_serial_device":   "/dev/serial/by-id/usb-test",
		"gnss_serial_baud":     115200,
	}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/yaml", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	content, err := os.ReadFile(yamlFile)
	require.NoError(t, err)
	assert.Contains(t, string(content), "extra_existing")
	assert.Contains(t, string(content), "keep_me")
	assert.Contains(t, string(content), "99.999")
	assert.Contains(t, string(content), "gnss_receiver_family: nmea")
	assert.Contains(t, string(content), "gnss_serial_device: /dev/serial/by-id/usb-test")
	assert.Contains(t, string(content), "gnss_serial_baud: 115200")
}

func TestApplyUniversalGnssCompatibility_NormalizesProfileKeys(t *testing.T) {
	flat := map[string]any{
		"gnss_receiver_family":  "unicore",
		"gnss_serial_device":    "/dev/ttyUSB0",
		"gnss_serial_baud":      460800,
		"gnss_profile":          "debug",
		"gnss_signal_profile":   "ppp-optimized",
		"gnss_rate_hz":          7,
		"gnss_signal_group":     "  3   6  ",
		"ntrip_enabled":         true,
		"ntrip_mountpoint":      "NEAR",
	}

	compat := applyUniversalGnssCompatibility(flat)

	assert.Equal(t, "rover_high_precision_debug", compat["GNSS_PROFILE"])
	assert.Equal(t, "high_precision", compat["GNSS_SIGNAL_PROFILE"])
	assert.Equal(t, "7", compat["GNSS_PROFILE_RATE_HZ"])
	assert.Equal(t, "460800", compat["GNSS_CONFIG_BAUD"])
	assert.Equal(t, "rover_high_precision_debug", flat["gnss_profile"])
	assert.Equal(t, "high_precision", flat["gnss_signal_profile"])
	assert.Equal(t, 7, flat["gnss_profile_rate_hz"])
	assert.Equal(t, 460800, flat["gnss_config_baud"])
	assert.Equal(t, "3 6", flat["gnss_signal_group"])
	_, hasLegacyRate := flat["gnss_rate_hz"]
	assert.False(t, hasLegacyRate)
}

func TestPostSettingsYAMLPurgesLegacyRuntimeEnvKeys(t *testing.T) {
	yamlFile := createTempYAMLFile(t, "")
	legacyProtocol := "GPS_" + "PROTOCOL=UBX\n"
	legacyByID := "GPS_" + "BY_ID=/dev/serial/by-id/legacy\n"
	legacyRuntime := "UNICORE_" + "ROS_EXECUTABLE=unicore_node\n"
	envFile := createTempConfigFile(t, "ROS_DOMAIN_ID=0\n"+legacyProtocol+legacyByID+legacyRuntime)

	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))
	db.Set("system.mower.runtimeEnvFile", []byte(envFile))

	router := setupSettingsRouter(db)

	payload := map[string]any{
		"gnss_receiver_family": "ublox",
		"gnss_serial_device":   "/dev/serial/by-id/usb-test",
		"gnss_serial_baud":     921600,
	}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/yaml", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	envContent, err := os.ReadFile(envFile)
	require.NoError(t, err)
	assert.Contains(t, string(envContent), "GNSS_BACKEND=universal")
	assert.NotContains(t, string(envContent), strings.TrimSpace(legacyProtocol))
	assert.NotContains(t, string(envContent), strings.TrimSpace(legacyByID))
	assert.NotContains(t, string(envContent), strings.TrimSpace(legacyRuntime))
}

func TestPostSettingsYAML_InvalidJSON(t *testing.T) {
	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte("/tmp/test.yaml"))

	router := setupSettingsRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/yaml", strings.NewReader("not json"))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusBadRequest, w.Code)
}

func createTempYAMLFile(t *testing.T, content string) string {
	t.Helper()
	f, err := os.CreateTemp(t.TempDir(), "mower_config_*.yaml")
	require.NoError(t, err)
	_, err = f.WriteString(content)
	require.NoError(t, err)
	f.Close()
	return f.Name()
}

func createTempConfigFile(t *testing.T, content string) string {
	t.Helper()
	f, err := os.CreateTemp(t.TempDir(), "mower_config_*.sh")
	require.NoError(t, err)
	_, err = f.WriteString(content)
	require.NoError(t, err)
	f.Close()
	return f.Name()
}
