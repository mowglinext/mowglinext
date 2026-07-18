package api

import (
	"fmt"
	"os"
	"strings"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"gopkg.in/yaml.v3"
)

// schemaDefaultsWithNoTemplateEntry lists JSON-schema default keys that are
// legitimately absent from the ROS2 package template
// (ros2/src/mowgli_bringup/config/mowgli_robot.yaml). Architecture
// Invariant 15 does not require every schema default to have a template
// line — a parameter's real default can instead be hardcoded in the ROS2
// node/launch file that declares it (verified below to match), or the field
// may not be a ROS2-declared parameter at all. Anything NOT on this list
// must have a template entry whose value equals the schema default, or
// TestSchemaDefaultsMatchTemplate fails — that gap is exactly what
// GetSettingsYAMLDefaults' doc comment warns about: "on divergence the
// sparse-prune makes GUI show 'at default=X' while robot runs Y."
var schemaDefaultsWithNoTemplateEntry = map[string]string{
	// Universal-GNSS receiver-profile fields are consumed entirely by
	// gui/pkg/api/gnss.go (building CLI commands for the configurator tool)
	// — never a ROS2 declare_parameter, so there is no template line to
	// compare against. Their fallback is already single-sourced through
	// this same schema (settings.go gnssSchemaDefaultString).
	"gnss_config_baud":     "Universal GNSS tool config, not a ROS2 parameter",
	"gnss_profile":         "Universal GNSS tool config, not a ROS2 parameter",
	"gnss_profile_rate_hz": "Universal GNSS tool config, not a ROS2 parameter",
	"gnss_signal_profile":  "Universal GNSS tool config, not a ROS2 parameter",

	// mowgli_bringup/launch/mowgli.launch.py reads
	// robot_params.get("imu_roll"/"imu_pitch", 0.0) directly — the launch
	// file's own hardcoded fallback matches this schema default, without a
	// template line.
	"imu_roll":  "mowgli.launch.py hardcodes 0.0 fallback (matches)",
	"imu_pitch": "mowgli.launch.py hardcodes 0.0 fallback (matches)",

	// Install-time choice (Architecture Invariant 15): lives ONLY in the
	// sparse installed config by design, never in the template.
	"lidar_enabled": "install-time choice, sparse-config-only (Invariant 15)",

	// hardware_bridge_node.cpp's declare_parameter defaults match this
	// schema default without needing a template line.
	"lift_recovery_mode":          "hardware_bridge_node.cpp declare_parameter default matches",
	"lift_blade_resume_delay_sec": "hardware_bridge_node.cpp declare_parameter default matches",

	// Documented dead controls — the template's own comments say no ROS2
	// node reads these (see the "outline_*"/"mow_angle_*" comments in
	// mowgli_robot.yaml). ros2/scripts/check_config_drift.py already
	// categorizes them as USER_OVERRIDE (no template comparison expected).
	"outline_passes":          "dead control, no ROS2 node reads it (see template comment)",
	"outline_offset":          "dead control, no ROS2 node reads it (see template comment)",
	"outline_overlap":         "dead control, no ROS2 node reads it (see template comment)",
	"mow_angle_offset_deg":    "dead control, BT hardcodes -1.0 (see template comment)",
	"mow_angle_increment_deg": "dead control, BT hardcodes 0.0 (see template comment)",

	// slam_mode / map_save_on_dock are leftovers from the slam_toolbox era
	// that ended with the FusionCore -> iSAM2 migration (see CLAUDE.md
	// Architecture Invariant 1: "slam_toolbox... removed"). Verified via
	// grep: no ROS2 node or gui/pkg code declares or reads either key. The
	// frontend already knows about this and independently hides both from
	// the Advanced settings section (web/src/hooks/useSettingsManager.ts
	// HIDDEN_FROM_ADVANCED) with the same rationale: "dead config that would
	// silently mislead anyone who edits them." Left in the schema (rather
	// than removed) because old installed YAMLs may still carry these keys
	// and the schema is also used to validate/flatten those files.
	"slam_mode":        "dead config, no consumer (slam_toolbox removed; see useSettingsManager.ts)",
	"map_save_on_dock": "dead config, no consumer (slam_toolbox removed; see useSettingsManager.ts)",
}

// findSchemaTemplateDivergence compares schemaDefaults against the
// flattened template and returns two failure classes: mismatched (a key
// present in both with a different value — the dangerous "GUI shows
// default=X, robot runs Y" case) and unaccountedMissing (a schema default
// with no template entry that isn't on the allowlist). Both empty means the
// schema and template are in parity.
func findSchemaTemplateDivergence(schemaDefaults map[string]any, tplFlat map[string]any, allowlist map[string]string) (mismatched, unaccountedMissing []string) {
	for key, schemaDefault := range schemaDefaults {
		tplValue, inTemplate := tplFlat[key]
		if !inTemplate {
			if _, allowed := allowlist[key]; !allowed {
				unaccountedMissing = append(unaccountedMissing, key)
			}
			continue
		}
		if !valuesEqual(schemaDefault, tplValue) {
			mismatched = append(mismatched, fmt.Sprintf("%s: schema=%v template=%v", key, schemaDefault, tplValue))
		}
	}
	return mismatched, unaccountedMissing
}

// TestSchemaDefaultsMatchTemplate is the CI guard for schema-vs-template
// default parity: it loads the real ROS2 package template (the source of
// truth robot_config_util's deep-merge actually falls back to) and asserts
// every JSON-schema default either matches the template's value, or is
// explicitly accounted for in schemaDefaultsWithNoTemplateEntry.
func TestSchemaDefaultsMatchTemplate(t *testing.T) {
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	db := types.NewMockDBProvider()
	schema, err := getSchema(db)
	require.NoError(t, err)

	schemaDefaults := map[string]any{}
	extractDefaults(schema, schemaDefaults)
	require.NotEmpty(t, schemaDefaults, "sanity: the schema should have yielded at least one default")

	tplBytes, err := os.ReadFile("../ros2/src/mowgli_bringup/config/mowgli_robot.yaml")
	require.NoError(t, err, "the ROS2 package template is the source of truth this guard checks against")
	var tplYAML map[string]any
	require.NoError(t, yaml.Unmarshal(tplBytes, &tplYAML))
	tplFlat := flattenROS2YAML(tplYAML)

	mismatched, unaccountedMissing := findSchemaTemplateDivergence(schemaDefaults, tplFlat, schemaDefaultsWithNoTemplateEntry)

	assert.Emptyf(t, mismatched,
		"schema default diverges from the template — the GUI would show \"at default\" while the "+
			"robot actually runs the template's value:\n%s", strings.Join(mismatched, "\n"))
	assert.Emptyf(t, unaccountedMissing,
		"schema default(s) with no template entry and not on schemaDefaultsWithNoTemplateEntry's "+
			"allowlist — either add a template line, or document why none is needed:\n%s", strings.Join(unaccountedMissing, "\n"))
}

// TestFindSchemaTemplateDivergence_DetectsMismatchAndUnaccountedMissing is a
// synthetic-data regression test proving the comparison actually catches
// both failure classes — the real-file test above currently has zero
// findings (a healthy baseline), so this is what exercises the "it works"
// path.
func TestFindSchemaTemplateDivergence_DetectsMismatchAndUnaccountedMissing(t *testing.T) {
	schemaDefaults := map[string]any{
		"tool_width":  0.18,
		"drifted_key": 5.0,
		"orphan_key":  "x",
		"allowed_key": "y",
	}
	tplFlat := map[string]any{
		"tool_width":  0.18,
		"drifted_key": 7.0, // template disagrees with the schema default
		// orphan_key and allowed_key are both absent from the template.
	}
	allowlist := map[string]string{
		"allowed_key": "documented as GUI-only, no template line needed",
	}

	mismatched, unaccountedMissing := findSchemaTemplateDivergence(schemaDefaults, tplFlat, allowlist)

	require.Len(t, mismatched, 1)
	assert.Contains(t, mismatched[0], "drifted_key")
	assert.Equal(t, []string{"orphan_key"}, unaccountedMissing)
}
