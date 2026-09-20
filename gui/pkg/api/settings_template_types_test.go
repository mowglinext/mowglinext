package api

import (
	"os"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
)

// TestTemplateTypesAssetMatchesTemplate is the anti-drift guard for the
// generated asset. The GUI image cannot ship the ROS2 template (its build
// context is ./gui), so the template's number types are baked into
// asserts/ros2_template_types.json — and this test re-derives them from the
// real template on every CI run, so the asset can never silently go stale and
// no key name is ever hand-maintained.
func TestTemplateTypesAssetMatchesTemplate(t *testing.T) {
	chdirToGuiRoot(t)

	expected, err := buildTemplateTypesFile(ros2TemplatePath)
	require.NoError(t, err, "the ROS2 package template is the source of truth for this asset")
	wantBytes, err := marshalTemplateTypesFile(expected)
	require.NoError(t, err)

	gotBytes, err := os.ReadFile(templateTypesAssetPath)
	require.NoError(t, err, "the generated asset must be checked in — the GUI image ships asserts/ and nothing else")

	assert.Equal(t, string(wantBytes), string(gotBytes),
		"asserts/ros2_template_types.json is stale; regenerate with: cd gui && go run ./cmd/gen-template-types")
}

// TestTemplateTypesAssetCoversUnschemaedFloatKeys pins the reason the asset
// exists: these keys hold an integral float default in the template, are absent
// from the JSON schema, and are consumed as C++ doubles. Without the template
// as a type source a file that already holds "tick_rate: 10" keeps it forever,
// because the on-disk type can only preserve, never repair.
func TestTemplateTypesAssetCoversUnschemaedFloatKeys(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	kinds := loadTemplateNumberKinds()
	require.NotEmpty(t, kinds)

	schemaKinds := schemaNumberKinds(mustSchema(t))

	for _, key := range []string{
		"dock_pose_x",
		"dock_pose_y",
		"dock_pose_yaw",
		"max_mps",
	} {
		assert.Equal(t, yamlNumberFloat, kinds[key], "%s must be declared a float by the template asset", key)
		_, inSchema := schemaKinds[key]
		assert.False(t, inSchema, "%s is now in the JSON schema; this test no longer covers the gap it was written for", key)
	}
}

// TestTemplateTypesAssetSkipsNonNumericKeys pins that only numbers are
// recorded: a string or bool key must not appear, or the writer could retype it.
func TestTemplateTypesAssetSkipsNonNumericKeys(t *testing.T) {
	chdirToGuiRoot(t)

	kinds := loadTemplateNumberKinds()
	require.NotEmpty(t, kinds)

	for _, key := range []string{"mower_model", "ntrip_enabled", "mowing_enabled", "gnss_serial_device"} {
		_, found := kinds[key]
		assert.False(t, found, "%s is not a number and must not be in the type asset", key)
	}
}

// TestLoadTemplateNumberKinds_MissingAssetDegrades pins that a missing asset
// cannot break a settings save: the write falls back to the on-disk types,
// which is the pre-fix behaviour.
func TestLoadTemplateNumberKinds_MissingAssetDegrades(t *testing.T) {
	orig, err := os.Getwd()
	require.NoError(t, err)
	require.NoError(t, os.Chdir(t.TempDir()))
	t.Cleanup(func() { _ = os.Chdir(orig) })

	assert.Empty(t, loadTemplateNumberKinds())
}

func mustSchema(t *testing.T) map[string]any {
	t.Helper()
	schema, err := getSchema(types.NewMockDBProvider())
	require.NoError(t, err)
	return schema
}
