package api

import (
	"os"
	"testing"

	"github.com/mowglinext/mowglinext/pkg/types"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"gopkg.in/yaml.v3"
)

// ledSchemaDefaults returns every led_* default the JSON schema declares,
// loaded exactly the way the settings endpoints load it.
func ledSchemaDefaults(t *testing.T) map[string]any {
	t.Helper()
	chdirToGuiRoot(t)
	resetSchemaCache()
	t.Cleanup(resetSchemaCache)

	db := types.NewMockDBProvider()
	schema, err := getSchema(db)
	require.NoError(t, err)

	all := map[string]any{}
	extractDefaults(schema, all)
	require.NotEmpty(t, all, "sanity: the schema should have yielded at least one default")

	leds := map[string]any{}
	for key, value := range all {
		if len(key) > 4 && key[:4] == "led_" {
			leds[key] = value
		}
	}
	return leds
}

// TestLedEnabledSchemaDefaultIsFalseSoTurningItOnPersists is the regression
// guard for the exact shape of the bug fixed in #508 (lidar_enabled).
//
// The settings backend keeps the installed mowgli_robot.yaml SPARSE by pruning
// any value equal to its JSON-schema default (sparsifyFlat, Architecture
// Invariant 15). For a boolean that is OFF by default, that is harmless:
// "switch it ON" writes true, true != false, the key persists. Invert the
// default and the toggle becomes INERT — "switch it ON" writes the default
// value, sparsifyFlat drops it as redundant, the key never reaches the file,
// and the ROS2 deep-merge keeps falling through to the template. The GUI shows
// the switch ON while the robot runs with the ring off, forever.
//
// led_enabled MUST therefore stay false in BOTH the schema and the ROS2
// template. TestSchemaDefaultsMatchTemplate already pins the two to EACH
// OTHER; this pins them to the value that actually makes the control work, so
// flipping both together still fails.
func TestLedEnabledSchemaDefaultIsFalseSoTurningItOnPersists(t *testing.T) {
	defaults := ledSchemaDefaults(t)

	require.Contains(t, defaults, "led_enabled", "the LED ring must be configurable from the GUI")
	assert.Equal(t, false, defaults["led_enabled"],
		"led_enabled's schema default must be false: the settings backend prunes any value "+
			"equal to its default, so a default of true would make 'switch the ring ON' prune "+
			"itself away and the toggle would never persist (this is bug #508, on lidar_enabled)")

	// The prune coupling itself, exercised rather than described.
	on := map[string]any{"led_enabled": true}
	prunedOn := sparsifyFlat(on, defaults)
	assert.False(t, prunedOn["led_enabled"], "turning the ring ON must survive the sparse prune")
	assert.Equal(t, true, on["led_enabled"], "turning the ring ON must still be written to disk")

	off := map[string]any{"led_enabled": false}
	prunedOff := sparsifyFlat(off, defaults)
	assert.True(t, prunedOff["led_enabled"],
		"turning the ring OFF equals the default and SHOULD be pruned, so the installed "+
			"config stays sparse and 'reset to default' keeps working")
	assert.NotContains(t, off, "led_enabled")
}

// TestLedSchemaDefaultsMatchTemplate pins every led_* schema default to the
// ROS2 package template value the robot actually falls back to.
//
// TestSchemaDefaultsMatchTemplate covers this for the whole schema, but only
// as long as no one adds led_* to schemaDefaultsWithNoTemplateEntry's
// allowlist. These parameters have real template lines and must keep them:
// the allowlist exists for keys with no ROS2 parameter at all, which is not
// the case here (led_ring_node declares every one of them).
func TestLedSchemaDefaultsMatchTemplate(t *testing.T) {
	defaults := ledSchemaDefaults(t)

	tplBytes, err := os.ReadFile("../ros2/src/mowgli_bringup/config/mowgli_robot.yaml")
	require.NoError(t, err)
	var tplYAML map[string]any
	require.NoError(t, yaml.Unmarshal(tplBytes, &tplYAML))
	tplFlat := flattenROS2YAML(tplYAML)

	// Every parameter led_ring_node declares must be reachable from the GUI,
	// or the operator is back to SSH-ing in to hand-edit the yaml.
	expected := []string{
		"led_enabled", "led_count", "led_spi_device", "led_spi_speed_hz",
		"led_brightness", "led_idle_scale", "led_refresh_hz",
		"led_low_battery_percent", "led_charge_full_percent",
		"led_status_timeout_s", "led_keepalive_s", "led_device_retry_s",
	}
	for _, key := range expected {
		assert.Containsf(t, defaults, key, "%s has no schema default, so the GUI cannot show or reset it", key)
		tplValue, inTemplate := tplFlat[key]
		if !assert.Truef(t, inTemplate, "%s has no template line (Invariant 15: defaults live in the template)", key) {
			continue
		}
		assert.Truef(t, valuesEqual(defaults[key], tplValue),
			"%s: schema default %v diverges from template %v — the GUI would show \"at default\" "+
				"while the robot runs the template value, and the sparse prune would drop the "+
				"operator's edit", key, defaults[key], tplValue)
	}

	assert.Lenf(t, defaults, len(expected),
		"the schema declares led_* defaults not covered by this test: %v", defaults)
}

// TestLedSpiClockMatchesTheEncoderAssumption pins the SPI clock default to the
// value the WS2812 symbol table in ws2812_encoder.hpp is derived from.
//
// That encoder sends THREE SPI bits per WS2812 bit (0 -> 0b100, 1 -> 0b110).
// The symbols are only correct at a clock where three bits span the 1.25 us
// WS2812 bit period; at 2.4 MHz they do, exactly. A different default silently
// produces out-of-spec pulse widths and wrong colours on a ring nobody can
// debug from the GUI.
func TestLedSpiClockMatchesTheEncoderAssumption(t *testing.T) {
	defaults := ledSchemaDefaults(t)

	const wsBitPeriodUs = 1.25
	const spiBitsPerLedBit = 3.0

	clock, ok := asFloat64(defaults["led_spi_speed_hz"])
	require.True(t, ok, "led_spi_speed_hz must be numeric")

	bitPeriodUs := spiBitsPerLedBit / clock * 1e6
	assert.InDeltaf(t, wsBitPeriodUs, bitPeriodUs, 1e-9,
		"at %.0f Hz, 3 SPI bits span %.4f us, not the WS2812's 1.25 us — "+
			"see ros2/src/mowgli_leds/include/mowgli_leds/ws2812_encoder.hpp", clock, bitPeriodUs)
}
