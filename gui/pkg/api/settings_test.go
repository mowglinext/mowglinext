package api

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"runtime"
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

func TestGetSettings_Success(t *testing.T) {
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

	assert.Equal(t, http.StatusInternalServerError, w.Code)
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

func TestGetSettingsSchema_FromUpstream(t *testing.T) {
	resetSchemaCache()

	// Mock upstream server serving the schema
	upstreamSchema := `{"type":"object","properties":{"important_settings":{"title":"Hardware Settings","type":"object","properties":{"OM_HARDWARE_VERSION":{"type":"string","enum":["0_13_X"],"x-environment-variable":"OM_HARDWARE_VERSION"},"OM_MOWER":{"type":"string","enum":["YardForce500","CUSTOM"],"x-environment-variable":"OM_MOWER"},"OM_MOWER_ESC_TYPE":{"type":"string","enum":["xesc_mini"],"x-environment-variable":"OM_MOWER_ESC_TYPE"}}}}}`
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		w.Write([]byte(upstreamSchema))
	}))
	defer srv.Close()

	db := types.NewMockDBProvider()
	db.Set("system.mower.schemaURL", []byte(srv.URL))

	router := setupSettingsRouter(db)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("GET", "/api/settings/schema", nil)
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	var result map[string]any
	err := json.Unmarshal(w.Body.Bytes(), &result)
	require.NoError(t, err)
	assert.Equal(t, "object", result["type"])

	// Verify Mowgli overlay was applied
	props := result["properties"].(map[string]any)
	hw := props["important_settings"].(map[string]any)
	hwProps := hw["properties"].(map[string]any)

	// OM_MOWER should have "Mowgli" added to enum
	omMower := hwProps["OM_MOWER"].(map[string]any)
	enumList := omMower["enum"].([]any)
	assert.Contains(t, enumList, "Mowgli")

	// OM_HARDWARE_VERSION and ESC_TYPE should be removed from base props
	// (moved to conditional allOf)
	assert.NotContains(t, hwProps, "OM_HARDWARE_VERSION")
	assert.NotContains(t, hwProps, "OM_MOWER_ESC_TYPE")

	// allOf should contain the conditions
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

func TestLocalSettingsSchema_AllowsUnicoreProtocol(t *testing.T) {
	resetSchemaCache()

	_, thisFile, _, ok := runtime.Caller(0)
	require.True(t, ok)

	schemaPath := filepath.Join(filepath.Dir(thisFile), "..", "..", "asserts", "mower_config.schema.json")
	content, err := os.ReadFile(schemaPath)
	require.NoError(t, err)

	var schema map[string]any
	require.NoError(t, json.Unmarshal(content, &schema))

	props := schema["properties"].(map[string]any)
	important := props["important_settings"].(map[string]any)
	importantProps := important["properties"].(map[string]any)
	gpsProtocol := importantProps["gps_protocol"].(map[string]any)
	enumValues := gpsProtocol["enum"].([]any)

	assert.Contains(t, enumValues, "UNICORE")
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
	yamlFile := createTempYAMLFile(t, "OM_DATUM_LAT: 48.123\nOM_USE_NTRIP: true\n")

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
	assert.Equal(t, 48.123, result["OM_DATUM_LAT"])
	assert.Equal(t, true, result["OM_USE_NTRIP"])
}

func TestGetSettingsYAML_FileNotExist_ReturnsEmpty(t *testing.T) {
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
	assert.Empty(t, result)
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

	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))

	router := setupSettingsRouter(db)

	payload := map[string]any{
		"OM_DATUM_LAT": 48.999,
		"OM_USE_NTRIP": true,
	}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/yaml", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	content, err := os.ReadFile(yamlFile)
	require.NoError(t, err)
	assert.Contains(t, string(content), "OM_DATUM_LAT")
	assert.Contains(t, string(content), "OM_USE_NTRIP")
}

func TestPostSettingsYAML_MergesExisting(t *testing.T) {
	yamlFile := createTempYAMLFile(t, "OM_EXISTING: keep_me\nOM_DATUM_LAT: 48.123\n")

	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))

	router := setupSettingsRouter(db)

	payload := map[string]any{
		"OM_DATUM_LAT": 99.999,
	}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/yaml", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	content, err := os.ReadFile(yamlFile)
	require.NoError(t, err)
	assert.Contains(t, string(content), "OM_EXISTING")
	assert.Contains(t, string(content), "keep_me")
	assert.Contains(t, string(content), "99.999")
}

func TestPostSettingsYAML_PreservesExistingUnicoreProtocolWhenPayloadOmitsIt(t *testing.T) {
	yamlFile := createTempYAMLFile(t, `mowgli:
  ros__parameters:
    gps_protocol: UNICORE
    gps_port: /dev/serial/by-id/usb-Unicore_UM980
    gps_baudrate: 921600
    gps_timeout_sec: 5
`)

	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))

	router := setupSettingsRouter(db)

	payload := map[string]any{
		"gps_timeout_sec": 12,
	}
	body, _ := json.Marshal(payload)

	w := httptest.NewRecorder()
	req, _ := http.NewRequest("POST", "/api/settings/yaml", bytes.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(w, req)

	assert.Equal(t, http.StatusOK, w.Code)

	content, err := os.ReadFile(yamlFile)
	require.NoError(t, err)
	assert.Contains(t, string(content), "gps_protocol: UNICORE")
	assert.Contains(t, string(content), "gps_timeout_sec: 12")
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
