package api

import (
	"bytes"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"gopkg.in/yaml.v3"
)

// Regression tests for the 2026-09-15 field incident: saving ANY setting in the
// GUI rewrote every integral float without its decimal point
// (lidar_map_tile_size_m: 5.0 -> 5), which made rclcpp's
// declare_parameter<double> throw on the next container start, aborting
// fusion_graph_node and leaving the robot with no map->odom TF and no Nav2.

// postSettingsYAML runs one save through the real HTTP handler and returns the
// file content it wrote.
func postSettingsYAML(t *testing.T, existing string, payload map[string]any) string {
	t.Helper()
	yamlFile := createTempYAMLFileAtGuiRoot(t, existing)
	envFile := createTempConfigFileAtGuiRoot(t, "")

	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))
	db.Set("system.mower.runtimeEnvFile", []byte(envFile))

	body, err := json.Marshal(payload)
	require.NoError(t, err)

	w := httptest.NewRecorder()
	req, err := http.NewRequest("POST", "/api/settings/yaml", bytes.NewReader(body))
	require.NoError(t, err)
	req.Header.Set("Content-Type", "application/json")
	setupSettingsRouter(db).ServeHTTP(w, req)
	require.Equal(t, http.StatusOK, w.Code)

	content, err := os.ReadFile(yamlFile)
	require.NoError(t, err)
	return string(content)
}

// decodeParams re-reads a written document the way ROS2 does and returns the
// flat parameter map, so a test can assert on the DECODED Go type (which is
// what rclcpp's double-vs-int check keys off) and not only on the text.
func decodeParams(t *testing.T, content string) map[string]any {
	t.Helper()
	doc := map[string]any{}
	require.NoError(t, yaml.Unmarshal([]byte(content), &doc))
	return flattenROS2YAML(doc)
}

// getThenPostSettingsYAML performs the exact operation the GUI performs when a
// user opens Settings and presses Save without changing anything: GET the flat
// settings, POST the same body straight back. It is the round trip that
// regressed while the marshal unit tests stayed green, because GET returns JSON
// and every number comes back as a float64.
func getThenPostSettingsYAML(t *testing.T, existing string) string {
	t.Helper()
	yamlFile := createTempYAMLFileAtGuiRoot(t, existing)
	envFile := createTempConfigFileAtGuiRoot(t, "")

	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))
	db.Set("system.mower.runtimeEnvFile", []byte(envFile))
	router := setupSettingsRouter(db)

	getRec := httptest.NewRecorder()
	getReq, err := http.NewRequest("GET", "/api/settings/yaml", nil)
	require.NoError(t, err)
	router.ServeHTTP(getRec, getReq)
	require.Equal(t, http.StatusOK, getRec.Code)

	var payload map[string]any
	require.NoError(t, json.Unmarshal(getRec.Body.Bytes(), &payload))

	postRec := httptest.NewRecorder()
	postReq, err := http.NewRequest("POST", "/api/settings/yaml", bytes.NewReader(getRec.Body.Bytes()))
	require.NoError(t, err)
	postReq.Header.Set("Content-Type", "application/json")
	router.ServeHTTP(postRec, postReq)
	require.Equal(t, http.StatusOK, postRec.Code)

	content, err := os.ReadFile(yamlFile)
	require.NoError(t, err)
	return string(content)
}

// TestPostSettingsYAML_SchemaNumberKeepsDecimalPoint covers rule (a) for
// "number": an integral value under a schema number key must stay a float.
func TestPostSettingsYAML_SchemaNumberKeepsDecimalPoint(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	// imu_z: schema type "number", default 0.095, so 0 is an override and is
	// not pruned. Before the fix this wrote "imu_z: 0".
	content := postSettingsYAML(t, "", map[string]any{"imu_z": 0})

	assert.Contains(t, content, "imu_z: 0.0")
	assert.IsType(t, float64(0), decodeParams(t, content)["imu_z"])
}

// TestPostSettingsYAML_SchemaIntegerStaysInt covers rule (a) for "integer": an
// integer key must NOT gain a decimal point, or a declare_parameter<int> node
// breaks the same way in the other direction.
func TestPostSettingsYAML_SchemaIntegerStaysInt(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	// dock_max_retries: schema type "integer", default 3. The form posts JSON,
	// so it arrives as float64(5).
	content := postSettingsYAML(t, "", map[string]any{"dock_max_retries": 5})

	assert.Contains(t, content, "dock_max_retries: 5\n")
	assert.NotContains(t, content, "dock_max_retries: 5.0")
	assert.IsType(t, 0, decodeParams(t, content)["dock_max_retries"])
}

// TestPostSettingsYAML_PreservesUnknownFloatOnDisk is the exact field case:
// lidar_map_tile_size_m is in neither the JSON schema nor the ROS2 template, is
// float on disk, and must survive a save that does not touch it.
func TestPostSettingsYAML_PreservesUnknownFloatOnDisk(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	existing := `mowgli:
  ros__parameters:
    lidar_map_resolution_m: 0.10
    lidar_map_tile_size_m: 5.0
    lidar_map_radius_tiles: 2
`
	// Save an unrelated key, exactly as the operator did in the field.
	content := postSettingsYAML(t, existing, map[string]any{"mowing_speed": 0.42})

	assert.Contains(t, content, "lidar_map_tile_size_m: 5.0")
	assert.Contains(t, content, "mowing_speed: 0.42")

	params := decodeParams(t, content)
	assert.IsType(t, float64(0), params["lidar_map_tile_size_m"],
		"declare_parameter<double>(\"lidar_map_tile_size_m\") throws on an int")
	assert.IsType(t, float64(0), params["lidar_map_resolution_m"])
}

// TestPostSettingsYAML_PreservesUnknownIntOnDisk covers rule (b) in the other
// direction: an int on disk under a key the schema does not know must stay an
// int (lidar_map_radius_tiles is declare_parameter<int>).
func TestPostSettingsYAML_PreservesUnknownIntOnDisk(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	existing := `mowgli:
  ros__parameters:
    lidar_map_radius_tiles: 2
`
	content := postSettingsYAML(t, existing, map[string]any{"mowing_speed": 0.42})

	assert.Contains(t, content, "lidar_map_radius_tiles: 2\n")
	assert.NotContains(t, content, "lidar_map_radius_tiles: 2.0")
	assert.IsType(t, 0, decodeParams(t, content)["lidar_map_radius_tiles"])
}

// TestPostSettingsYAML_UnknownFloatKeyEditedKeepsFloat covers the same unknown
// key when the payload DOES carry it: JSON hands the handler float64(5), and
// only the on-disk type says it must stay a float.
func TestPostSettingsYAML_UnknownFloatKeyEditedKeepsFloat(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	existing := `mowgli:
  ros__parameters:
    lidar_map_tile_size_m: 5.0
`
	content := postSettingsYAML(t, existing, map[string]any{"lidar_map_tile_size_m": 8})

	assert.Contains(t, content, "lidar_map_tile_size_m: 8.0")
	assert.IsType(t, float64(0), decodeParams(t, content)["lidar_map_tile_size_m"])
}

// TestPostSettingsYAML_KeepsGeoFixedPrecision pins that the lat/lon path still
// wins over the generic number rule.
func TestPostSettingsYAML_KeepsGeoFixedPrecision(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	content := postSettingsYAML(t, "", map[string]any{
		"datum_lat": 48.1,
		"datum_lon": 2.0,
	})

	assert.Contains(t, content, "datum_lat: 48.100000000")
	assert.Contains(t, content, "datum_lon: 2.000000000")
}

// TestPostSettingsYAML_LeavesStringsAndBoolsAlone pins that only numbers are
// retyped — a numeric-looking string must not become a number.
func TestPostSettingsYAML_LeavesStringsAndBoolsAlone(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	existing := `mowgli:
  ros__parameters:
    unknown_serial_id: "0080"
`
	// /dev/ttyUSB0 is NOT the schema default, so it survives the sparse-write
	// pruner and can be asserted on.
	content := postSettingsYAML(t, existing, map[string]any{
		"gnss_serial_device": "/dev/ttyUSB0",
		"ntrip_enabled":      true,
		"mowing_speed":       0.42,
	})

	assert.Contains(t, content, "gnss_serial_device: /dev/ttyUSB0")
	assert.Contains(t, content, "ntrip_enabled: true")
	assert.Contains(t, content, `unknown_serial_id: "0080"`)

	params := decodeParams(t, content)
	assert.IsType(t, "", params["unknown_serial_id"])
	assert.Equal(t, true, params["ntrip_enabled"])
}

// TestPostSettingsYAML_TemplateRepairsDemotedFloat is the repair case the
// on-disk fallback cannot handle: max_mps is absent from the JSON schema, the
// file ALREADY holds the demoted "1", and only the ROS2 template knows it is a
// double (template value 0.5). A consumer that does not cast such a key aborts
// on the type mismatch.
func TestPostSettingsYAML_TemplateRepairsDemotedFloat(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	existing := `mowgli:
  ros__parameters:
    max_mps: 1
    dock_pose_x: 15
    dock_pose_yaw: 5
`
	content := postSettingsYAML(t, existing, map[string]any{"mowing_speed": 0.42})

	assert.Contains(t, content, "max_mps: 1.0")
	assert.Contains(t, content, "dock_pose_x: 15.0")
	assert.Contains(t, content, "dock_pose_yaw: 5.0")

	params := decodeParams(t, content)
	assert.IsType(t, float64(0), params["max_mps"])
	assert.IsType(t, float64(0), params["dock_pose_x"])
	assert.IsType(t, float64(0), params["dock_pose_yaw"])
}

// TestPostSettingsYAML_TemplateIntStaysInt pins the other direction: a key the
// template declares as an int must not gain a decimal point.
func TestPostSettingsYAML_TemplateIntStaysInt(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	// mow_direction: template int, absent from the schema. The form posts JSON,
	// so it arrives as float64(1).
	content := postSettingsYAML(t, "", map[string]any{"mow_direction": 1})

	assert.Contains(t, content, "mow_direction: 1\n")
	assert.NotContains(t, content, "mow_direction: 1.0")
	assert.IsType(t, 0, decodeParams(t, content)["mow_direction"])
}

// TestPostSettingsYAML_TemplateUnknownKeyKeepsDiskType pins that a key the
// template does NOT know still falls through to the on-disk type. The
// lidar_map_* family lives in fusion_graph.yaml, not the template.
func TestPostSettingsYAML_TemplateUnknownKeyKeepsDiskType(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	require.NotContains(t, loadTemplateNumberKindsAtGuiRoot(t), "lidar_map_tile_size_m",
		"sanity: this test is only meaningful while the template does not declare the key")

	existing := `mowgli:
  ros__parameters:
    lidar_map_tile_size_m: 5.0
    lidar_map_radius_tiles: 2
`
	content := postSettingsYAML(t, existing, map[string]any{"mowing_speed": 0.42})

	assert.Contains(t, content, "lidar_map_tile_size_m: 5.0")
	assert.Contains(t, content, "lidar_map_radius_tiles: 2\n")
}

func loadTemplateNumberKindsAtGuiRoot(t *testing.T) map[string]yamlNumberKind {
	t.Helper()
	kinds := loadTemplateNumberKinds()
	require.NotEmpty(t, kinds, "the generated template-types asset should have loaded")
	return kinds
}

// TestPostSettingsYAML_NoChangeSaveKeepsUnknownKeyTypes is the regression guard
// for the bug this file's unit tests did NOT catch: nestToROS2YAML aliased the
// caller's ros__parameters map and merged the payload into it, so hints read
// afterwards described the payload's float64s rather than the file. Every key
// neither the schema nor the template declares came out as a float —
// lidar_map_radius_tiles: 3 became 3.0, which aborts
// declare_parameter<int>("lidar_map_radius_tiles") in fusion_graph_node exactly
// the way the original int-for-double bug aborted it.
func TestPostSettingsYAML_NoChangeSaveKeepsUnknownKeyTypes(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	// None of these four is in the JSON schema or the ROS2 template, so only
	// the on-disk type can decide how they are written back.
	existing := `mowgli:
  ros__parameters:
    lidar_map_radius_tiles: 3
    automatic_mode: 0
    gps_baudrate: 921600
    lidar_map_tile_size_m: 5.0
`
	content := getThenPostSettingsYAML(t, existing)
	params := decodeParams(t, content)

	assert.Contains(t, content, "lidar_map_radius_tiles: 3\n")
	assert.NotContains(t, content, "lidar_map_radius_tiles: 3.0")
	assert.IsType(t, 0, params["lidar_map_radius_tiles"],
		"declare_parameter<int>(\"lidar_map_radius_tiles\") aborts fusion_graph_node on a float")

	assert.Contains(t, content, "automatic_mode: 0\n")
	assert.NotContains(t, content, "automatic_mode: 0.0")
	assert.IsType(t, 0, params["automatic_mode"])

	assert.Contains(t, content, "gps_baudrate: 921600\n")
	assert.NotContains(t, content, "gps_baudrate: 921600.0")
	assert.IsType(t, 0, params["gps_baudrate"])

	// The float-on-disk direction of the same rule.
	assert.Contains(t, content, "lidar_map_tile_size_m: 5.0")
	assert.IsType(t, float64(0), params["lidar_map_tile_size_m"])
}

// TestPostSettingsYAML_NoChangeSaveKeepsKnownKeyTypes pins the declared sources
// through the same round trip: the template repairs a demoted float, a template
// int and a schema int stay ints, and lat/lon keep their fixed precision.
func TestPostSettingsYAML_NoChangeSaveKeepsKnownKeyTypes(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	existing := `mowgli:
  ros__parameters:
    max_mps: 1
    dock_pose_x: 15
    battery_full_percent: 90
    one_wheel_lift_emergency_ms: 2000
    gnss_config_baud: 460800
    datum_lat: 48.123456789
`
	content := getThenPostSettingsYAML(t, existing)
	params := decodeParams(t, content)

	// Template says double; the file held the demoted int.
	assert.Contains(t, content, "max_mps: 1.0")
	assert.Contains(t, content, "dock_pose_x: 15.0")
	assert.IsType(t, float64(0), params["max_mps"])
	// Schema says number.
	assert.Contains(t, content, "battery_full_percent: 90.0")
	// Template int and schema integer.
	assert.Contains(t, content, "one_wheel_lift_emergency_ms: 2000\n")
	assert.Contains(t, content, "gnss_config_baud: 460800\n")
	assert.IsType(t, 0, params["one_wheel_lift_emergency_ms"])
	assert.IsType(t, 0, params["gnss_config_baud"])
	// Geo precision.
	assert.Contains(t, content, "datum_lat: 48.123456789")
}

// TestNestToROS2YAML_DoesNotMutateExisting is the root-cause guard: the nester
// must not write into the document it was handed. Aliasing it is what corrupted
// the type hints, and it would corrupt anything else a caller reads afterwards.
func TestNestToROS2YAML_DoesNotMutateExisting(t *testing.T) {
	existingYAML := map[string]any{
		"mowgli": map[string]any{
			"ros__parameters": map[string]any{
				"lidar_map_radius_tiles": 3,
				"keep_me":                "original",
			},
		},
	}

	nested := nestToROS2YAML(
		map[string]any{"lidar_map_radius_tiles": 9.0, "keep_me": "payload"},
		map[string]string{},
		existingYAML,
	)

	source := existingYAML["mowgli"].(map[string]any)["ros__parameters"].(map[string]any)
	assert.Equal(t, 3, source["lidar_map_radius_tiles"], "the source document must still hold the on-disk value")
	assert.Equal(t, "original", source["keep_me"])

	written := nested["mowgli"].(map[string]any)["ros__parameters"].(map[string]any)
	assert.Equal(t, 9.0, written["lidar_map_radius_tiles"], "the result must carry the merged value")
	assert.Equal(t, "payload", written["keep_me"])
}

// TestPersistGNSSRuntimeBaud_KeepsUnknownKeyTypes and
// TestPersistRobotYamlUpdates_KeepsUnknownKeyTypes cover the other two writers
// of mowgli_robot.yaml. Both build the same nest-then-marshal pipeline, so both
// could regress the same way; the hints must come from the document as read.
func TestPersistGNSSRuntimeBaud_KeepsUnknownKeyTypes(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	yamlFile := createTempYAMLFileAtGuiRoot(t, `mowgli:
  ros__parameters:
    lidar_map_radius_tiles: 3
    lidar_map_tile_size_m: 5.0
    max_mps: 1
`)
	envFile := createTempConfigFileAtGuiRoot(t, "")
	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))
	db.Set("system.mower.runtimeEnvFile", []byte(envFile))

	require.NoError(t, persistGNSSRuntimeBaud(db, "115200"))

	content, err := os.ReadFile(yamlFile)
	require.NoError(t, err)
	assert.Contains(t, string(content), "lidar_map_radius_tiles: 3\n")
	assert.NotContains(t, string(content), "lidar_map_radius_tiles: 3.0")
	assert.Contains(t, string(content), "lidar_map_tile_size_m: 5.0")
	assert.Contains(t, string(content), "max_mps: 1.0")
}

func TestPersistRobotYamlUpdates_KeepsUnknownKeyTypes(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	yamlFile := createTempYAMLFileAtGuiRoot(t, `mowgli:
  ros__parameters:
    lidar_map_radius_tiles: 3
    lidar_map_tile_size_m: 5.0
    max_mps: 1
`)
	db := types.NewMockDBProvider()
	db.Set("system.mower.yamlConfigFile", []byte(yamlFile))

	require.NoError(t, persistRobotYamlUpdates(db, map[string]any{"wheel_pid_kp": 12.0}))

	content, err := os.ReadFile(yamlFile)
	require.NoError(t, err)
	assert.Contains(t, string(content), "lidar_map_radius_tiles: 3\n")
	assert.NotContains(t, string(content), "lidar_map_radius_tiles: 3.0")
	assert.Contains(t, string(content), "lidar_map_tile_size_m: 5.0")
	assert.Contains(t, string(content), "max_mps: 1.0")
	assert.Contains(t, string(content), "wheel_pid_kp: 12.0")
}

// --- unit level -----------------------------------------------------------

func TestFormatYAMLFloat(t *testing.T) {
	cases := map[float64]string{
		5:        "5.0",
		0:        "0.0",
		-3:       "-3.0",
		319.305:  "319.305",
		0.1:      "0.1",
		1e-7:     "0.0000001",
		282.135:  "282.135",
		10:       "10.0",
		600:      "600.0",
		2400000:  "2400000.0",
		-0.00025: "-0.00025",
	}
	for in, want := range cases {
		assert.Equal(t, want, formatYAMLFloat(in))
	}
}

func TestYAMLTypeHints_KindForPrecedence(t *testing.T) {
	// Declared sources (schema, then template) beat the on-disk observation,
	// which is what REPAIRS a file an earlier save already demoted.
	hints := yamlTypeHints{
		schema:   map[string]yamlNumberKind{"from_schema": yamlNumberFloat},
		template: map[string]yamlNumberKind{"from_schema": yamlNumberInt, "from_template": yamlNumberFloat},
		onDisk: map[string]yamlNumberKind{
			"from_schema":   yamlNumberInt,
			"from_template": yamlNumberInt,
			"from_disk":     yamlNumberFloat,
		},
	}

	assert.Equal(t, yamlNumberFloat, hints.kindFor("from_schema"), "schema outranks template and disk")
	assert.Equal(t, yamlNumberFloat, hints.kindFor("from_template"), "template outranks disk")
	assert.Equal(t, yamlNumberFloat, hints.kindFor("from_disk"), "disk is the last resort")
	assert.Equal(t, yamlNumberUnknown, hints.kindFor("nobody_knows"))
}

func TestYAMLTypeHints_LoadsAllThreeSources(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	db := types.NewMockDBProvider()
	schema, err := getSchema(db)
	require.NoError(t, err)

	onDisk := map[string]any{
		"mowgli": map[string]any{
			"ros__parameters": map[string]any{
				"lidar_map_tile_size_m":  5.0,
				"lidar_map_radius_tiles": 2,
				"mower_model":            "yardforce500",
			},
		},
	}
	hints := newYAMLTypeHints(schema, onDisk)

	// From the JSON schema.
	assert.Equal(t, yamlNumberFloat, hints.kindFor("tool_width"))
	assert.Equal(t, yamlNumberInt, hints.kindFor("led_count"))
	// From the ROS2 template only — the schema declares neither.
	assert.Equal(t, yamlNumberFloat, hints.kindFor("tick_rate"))
	assert.Equal(t, yamlNumberInt, hints.kindFor("mow_direction"))
	// From the on-disk document only — neither the schema nor the template
	// knows the lidar_map_* family (it lives in fusion_graph.yaml).
	assert.Equal(t, yamlNumberFloat, hints.kindFor("lidar_map_tile_size_m"))
	assert.Equal(t, yamlNumberInt, hints.kindFor("lidar_map_radius_tiles"))
	// Not a number anywhere.
	assert.Equal(t, yamlNumberUnknown, hints.kindFor("mower_model"))
}

func TestMarshalROS2YAML_NoHintsIsUnchangedBehaviour(t *testing.T) {
	out, err := marshalROS2YAML(map[string]any{"a": 5.0, "b": "x", "c": true}, newYAMLTypeHints(nil, nil))
	require.NoError(t, err)
	assert.Contains(t, string(out), "a: 5\n")
	assert.Contains(t, string(out), "b: x")
	assert.Contains(t, string(out), "c: true")
}

func TestMarshalROS2YAML_DoesNotRetypeSequences(t *testing.T) {
	schema := map[string]any{
		"properties": map[string]any{
			"soft_limits": map[string]any{"type": "array"},
		},
	}
	out, err := marshalROS2YAML(
		map[string]any{"soft_limits": []any{1.0, 2, "three"}},
		newYAMLTypeHints(schema, nil),
	)
	require.NoError(t, err)
	assert.Contains(t, string(out), "- 1\n")
	assert.Contains(t, string(out), "- 2\n")
	assert.Contains(t, string(out), "- three\n")
}

func TestRetypeNumber_RefusesUnsafeConversions(t *testing.T) {
	// A string never becomes a number, whatever the hint says.
	_, ok := retypeNumber("115200", yamlNumberInt)
	assert.False(t, ok)
	_, ok = retypeNumber("48.1", yamlNumberFloat)
	assert.False(t, ok)
	// A fractional value under an integer key is written through unchanged
	// rather than silently truncated.
	_, ok = retypeNumber(5.7, yamlNumberInt)
	assert.False(t, ok)
	// A bool is not a number.
	_, ok = retypeNumber(true, yamlNumberFloat)
	assert.False(t, ok)

	value, ok := retypeNumber(5.0, yamlNumberInt)
	assert.True(t, ok)
	assert.Equal(t, int64(5), value)
	value, ok = retypeNumber(5, yamlNumberFloat)
	assert.True(t, ok)
	assert.Equal(t, yamlFloatScalar(5), value)
}
