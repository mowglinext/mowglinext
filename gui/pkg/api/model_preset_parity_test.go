package api

import (
	"fmt"
	"os"
	"regexp"
	"sort"
	"strconv"
	"strings"
	"testing"

	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"gopkg.in/yaml.v3"
)

// ---------------------------------------------------------------------------
// Model-preset ↔ template parity guard.
//
// TestSchemaDefaultsMatchTemplate (schema_template_parity_test.go) pins the
// JSON schema against the ROS2 template, but NOTHING pinned
// web/src/constants/mowerModels.ts — the table the GUI's "mower model" picker
// writes into a robot's installed mowgli_robot.yaml. That gap is how imu_y sat
// at -0.195 in the preset and 0.0 in the template for months: applying the
// preset put the IMU in the right place, while a GUI "reset to default"
// (Invariant 15: reset = delete the key so it falls back to the template)
// moved it 19.5 cm sideways in the URDF. Neither surface was wrong on its own,
// so neither test caught it.
//
// The presets legitimately differ FROM EACH OTHER — a Sabo is not a YardForce.
// So the guard is scoped to the one preset that must agree: the entry whose
// `value` equals the template's own `mower_model`. That preset and the
// template describe the SAME machine, so every key the preset defines must
// equal the template's default for that key, unless it is listed with a reason
// in modelPresetDivergesFromTemplate below.
// ---------------------------------------------------------------------------

// modelPresetDivergesFromTemplate lists keys where the template's own
// mower_model preset knowingly disagrees with the template, with the reason.
// Everything not listed here must match, so a NEW divergence fails CI.
// Shrinking this map is the goal; each entry needs a physical measurement or a
// decision, not a guess, which is why they are recorded rather than "fixed".
var modelPresetDivergesFromTemplate = map[string]string{
	// Per-robot CALIBRATION OUTPUT, not a physical constant. The preset seeds a
	// pre-calibration guess; check_config_drift.py already classifies
	// ticks_per_meter as CALIBRATION_OUTPUT and expects it to differ.
	"ticks_per_meter": "calibration output (preset 300 = pre-calibration seed, template 399.0 = a calibrated robot)",

	// Unreconciled measurements, out of scope for the config-template fix that
	// added this guard. Each needs someone to measure the actual machine.
	// (chassis_length and chassis_width were reconciled on 2026-09-05 by the
	// maintainer measuring his YardForce 500: 0.60 and 0.45; both removed.)
	"imu_x":          "unreconciled: preset 0.187 vs template 0.18 (7 mm)",
	"imu_z":          "unreconciled: preset 0.0 vs template 0.095 (template says base_height/2; preset 0.0 puts the IMU on the wheel axis plane)",
	"lidar_y":        "unreconciled: preset 0.025 vs template 0.024 (1 mm)",

	// Numerically the same rotation, expressed with opposite sign: the LiDAR is
	// mounted back-to-front, and -3.1416 rad == +3.1408 rad to within 8e-4.
	// Not worth churning either surface over.
	"lidar_yaw": "equivalent rotation, opposite sign convention (-pi vs +pi, 8e-4 rad apart)",

	// Battery chemistry/pack threshold, tuned per fleet rather than measured.
	"battery_full_voltage": "unreconciled: preset 28.5 vs template 28.0 — pack threshold, not a physical dimension",
}

// mowerModelPreset is one entry of MOWER_MODELS in
// gui/web/src/constants/mowerModels.ts.
type mowerModelPreset struct {
	name     string
	defaults map[string]float64
}

var (
	// \b so a key merely ENDING in "value" (e.g. `defaultvalue: "x"`) is not
	// mistaken for a preset name.
	presetValueRe  = regexp.MustCompile(`\bvalue:\s*"([^"]+)"`)
	presetNumberRe = regexp.MustCompile(`([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(-?[0-9]+(?:\.[0-9]+)?)`)
)

// stripLineComments removes whole-line `//` comments so prose in the file (which
// legitimately mentions things like "wheel_radius: 0.1 m") is never parsed as
// data. The file contains no URLs, so there is no `://` to protect.
func stripLineComments(src string) string {
	lines := strings.Split(src, "\n")
	kept := make([]string, 0, len(lines))
	for _, ln := range lines {
		if strings.HasPrefix(strings.TrimSpace(ln), "//") {
			continue
		}
		kept = append(kept, ln)
	}
	return strings.Join(kept, "\n")
}

// parseMowerModelPresets extracts every `{ value: "X", ... defaults: { k: n, … } }`
// entry. The table is a flat literal with no nested braces inside `defaults`,
// so a scan to the next `}` is sufficient and stays readable.
func parseMowerModelPresets(src string) ([]mowerModelPreset, error) {
	src = stripLineComments(src)
	var out []mowerModelPreset
	for _, m := range presetValueRe.FindAllStringSubmatchIndex(src, -1) {
		name := src[m[2]:m[3]]
		rest := src[m[1]:]
		di := strings.Index(rest, "defaults:")
		if di < 0 {
			return nil, fmt.Errorf("preset %q: no defaults block", name)
		}
		open := strings.Index(rest[di:], "{")
		if open < 0 {
			return nil, fmt.Errorf("preset %q: defaults has no {", name)
		}
		body := rest[di+open+1:]
		end := strings.Index(body, "}")
		if end < 0 {
			return nil, fmt.Errorf("preset %q: defaults has no closing }", name)
		}
		body = body[:end]

		defaults := map[string]float64{}
		for _, kv := range presetNumberRe.FindAllStringSubmatch(body, -1) {
			v, err := strconv.ParseFloat(kv[2], 64)
			if err != nil {
				return nil, fmt.Errorf("preset %q: key %q: %w", name, kv[1], err)
			}
			defaults[kv[1]] = v
		}
		out = append(out, mowerModelPreset{name: name, defaults: defaults})
	}
	if len(out) == 0 {
		return nil, fmt.Errorf("no presets parsed")
	}
	return out, nil
}

// findPresetTemplateDivergence returns (mismatched, unaccountedMissing) for one
// preset against the flattened template, mirroring
// findSchemaTemplateDivergence's two failure classes:
//
//	mismatched         — key in both, different value (the silent-drift case)
//	unaccountedMissing — preset key with no template line at all
func findPresetTemplateDivergence(
	preset map[string]float64,
	tplFlat map[string]any,
	allowlist map[string]string,
) (mismatched, unaccountedMissing []string) {
	keys := make([]string, 0, len(preset))
	for k := range preset {
		keys = append(keys, k)
	}
	sort.Strings(keys)

	for _, k := range keys {
		if _, allowed := allowlist[k]; allowed {
			continue
		}
		tplValue, inTemplate := tplFlat[k]
		if !inTemplate {
			unaccountedMissing = append(unaccountedMissing, k)
			continue
		}
		if !valuesEqual(preset[k], tplValue) {
			mismatched = append(mismatched,
				fmt.Sprintf("%s: preset=%v template=%v", k, preset[k], tplValue))
		}
	}
	return mismatched, unaccountedMissing
}

// TestMowerModelPresetMatchesTemplate is the CI guard the imu_y divergence
// slipped past. It compares the model preset for the template's own
// mower_model against the ROS2 template's defaults.
func TestMowerModelPresetMatchesTemplate(t *testing.T) {
	chdirToGuiRoot(t)

	tplBytes, err := os.ReadFile("../ros2/src/mowgli_bringup/config/mowgli_robot.yaml")
	require.NoError(t, err, "the ROS2 package template is the source of truth this guard checks against")
	var tplYAML map[string]any
	require.NoError(t, yaml.Unmarshal(tplBytes, &tplYAML))
	tplFlat := flattenROS2YAML(tplYAML)
	require.NotEmpty(t, tplFlat, "sanity: the template should have yielded parameters")

	modelName, ok := tplFlat["mower_model"].(string)
	require.True(t, ok, "template must declare a mower_model naming the preset this guard applies to")

	src, err := os.ReadFile("web/src/constants/mowerModels.ts")
	require.NoError(t, err)
	presets, err := parseMowerModelPresets(string(src))
	require.NoError(t, err)

	var target *mowerModelPreset
	for i := range presets {
		if presets[i].name == modelName {
			target = &presets[i]
			break
		}
	}
	require.NotNilf(t, target,
		"mowerModels.ts has no preset for the template's mower_model %q — either add one or "+
			"change the template", modelName)
	require.NotEmpty(t, target.defaults, "sanity: the %q preset should define values", modelName)

	mismatched, unaccountedMissing := findPresetTemplateDivergence(
		target.defaults, tplFlat, modelPresetDivergesFromTemplate)

	assert.Emptyf(t, mismatched,
		"mowerModels.ts preset %q diverges from the ROS2 template — applying the model preset and "+
			"resetting the same field to its default would give the robot two different physical "+
			"models (this is exactly the imu_y bug):\n%s",
		modelName, strings.Join(mismatched, "\n"))
	assert.Emptyf(t, unaccountedMissing,
		"mowerModels.ts preset %q sets key(s) with NO template default, so \"reset to default\" "+
			"has nothing to fall back to — add a template line, or document the key in "+
			"modelPresetDivergesFromTemplate:\n%s",
		modelName, strings.Join(unaccountedMissing, "\n"))
}

// TestModelPresetDivergenceAllowlistIsNotStale fails when an allowlist entry no
// longer describes a real divergence, so the list shrinks as values get
// reconciled instead of accumulating dead excuses.
func TestModelPresetDivergenceAllowlistIsNotStale(t *testing.T) {
	chdirToGuiRoot(t)

	tplBytes, err := os.ReadFile("../ros2/src/mowgli_bringup/config/mowgli_robot.yaml")
	require.NoError(t, err)
	var tplYAML map[string]any
	require.NoError(t, yaml.Unmarshal(tplBytes, &tplYAML))
	tplFlat := flattenROS2YAML(tplYAML)

	modelName, _ := tplFlat["mower_model"].(string)
	src, err := os.ReadFile("web/src/constants/mowerModels.ts")
	require.NoError(t, err)
	presets, err := parseMowerModelPresets(string(src))
	require.NoError(t, err)

	var target *mowerModelPreset
	for i := range presets {
		if presets[i].name == modelName {
			target = &presets[i]
			break
		}
	}
	require.NotNil(t, target)

	var stale []string
	for k := range modelPresetDivergesFromTemplate {
		presetValue, inPreset := target.defaults[k]
		if !inPreset {
			stale = append(stale, fmt.Sprintf("%s: no longer set by the %q preset", k, modelName))
			continue
		}
		if tplValue, inTemplate := tplFlat[k]; inTemplate && valuesEqual(presetValue, tplValue) {
			stale = append(stale, fmt.Sprintf("%s: preset and template now agree (%v)", k, presetValue))
		}
	}
	sort.Strings(stale)
	assert.Emptyf(t, stale,
		"modelPresetDivergesFromTemplate has stale entries — delete them so the allowlist keeps "+
			"meaning something:\n%s", strings.Join(stale, "\n"))
}

// TestParseMowerModelPresets_ExtractsNamesAndNumbers pins the little parser the
// guards depend on, including that prose in `//` comments is not read as data.
func TestParseMowerModelPresets_ExtractsNamesAndNumbers(t *testing.T) {
	src := `
// wheel_radius: 0.999 is prose, not data.
export const MOWER_MODELS = [
    {
        value: "Alpha",
        label: "x",
        defaults: {
            wheel_radius: 0.1, imu_y: -0.195,
            chassis_width: 0.40,
        },
    },
    {
        value: "EmptyOne",
        defaults: {},
    },
];`
	presets, err := parseMowerModelPresets(src)
	require.NoError(t, err)
	require.Len(t, presets, 2)

	assert.Equal(t, "Alpha", presets[0].name)
	assert.Equal(t, map[string]float64{
		"wheel_radius":  0.1,
		"imu_y":         -0.195,
		"chassis_width": 0.40,
	}, presets[0].defaults)

	assert.Equal(t, "EmptyOne", presets[1].name)
	assert.Empty(t, presets[1].defaults)
}

// TestFindPresetTemplateDivergence_DetectsMismatchAndMissing proves the
// comparison catches both failure classes, since the real-file guard above is
// expected to be green.
func TestFindPresetTemplateDivergence_DetectsMismatchAndMissing(t *testing.T) {
	preset := map[string]float64{
		"tool_width":   0.18,
		"imu_y":        -0.195, // template disagrees — the historical bug
		"gui_only_key": 3.0,    // no template line at all
		"allowed_key":  9.0,
	}
	tplFlat := map[string]any{
		"tool_width": 0.18,
		"imu_y":      0.0,
	}
	allowlist := map[string]string{"allowed_key": "documented divergence"}

	mismatched, unaccountedMissing := findPresetTemplateDivergence(preset, tplFlat, allowlist)

	require.Len(t, mismatched, 1)
	assert.Contains(t, mismatched[0], "imu_y")
	assert.Equal(t, []string{"gui_only_key"}, unaccountedMissing)
}
