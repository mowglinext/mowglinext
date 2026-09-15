package api

import (
	"math"
	"strconv"
	"strings"

	"gopkg.in/yaml.v3"
)

// Number-type preservation for the installed mowgli_robot.yaml.
//
// WHY THIS FILE EXISTS (field incident, 2026-09-15, twice: 09:13 and 23:04):
// the Settings page posts JSON, and JSON decodes EVERY number to float64, so
// the writer cannot tell 5 from 5.0 by Go type alone. gopkg.in/yaml.v3 then
// marshals float64(5) as the scalar "5", which YAML resolves as an int. On the
// next container start rclcpp's declare_parameter<double>("lidar_map_tile_size_m")
// throws InvalidParameterTypeException, fusion_graph_node aborts, nothing
// publishes map->odom, the Nav2 costmaps never activate and the robot cannot
// mow — a Play press hangs in PREFLIGHT_CHECK. Saving ANY unrelated setting in
// the GUI was enough to trigger it, because the whole document is re-marshalled
// on every save.
//
// The fix decides each numeric scalar's YAML type from, in priority order:
//  1. the lat/lon fixed-precision list (fixedPrecisionYAMLKeys) — unchanged,
//  2. the JSON schema type ("number" -> float, "integer" -> int),
//  3. the ROS2 package TEMPLATE's type, via the generated asset described in
//     settings_template_types.go. The template is the real default source
//     (Invariant 15) and covers 145 numeric parameters against the schema's
//     102, including the 19 integral-float keys the schema never declares
//     (tick_rate, imu_cal_auto_rest_sec, the loc_sigma_* family, ...).
//     Because it is a DECLARED type rather than an observation, it also
//     REPAIRS a file an earlier save already demoted,
//  4. the type the key ALREADY has in the on-disk document (yaml.v3 decodes
//     5.0 as float64 and 5 as int), which is the last resort for keys neither
//     the schema nor the template declares — the whole lidar_map_* family
//     lives in fusion_graph.yaml and is in neither,
//  5. otherwise: yaml.v3's own formatting, i.e. the previous behaviour.
//
// Booleans, strings and sequences are never retyped.

// yamlNumberKind is how a numeric scalar must be re-emitted: as a YAML float
// (5.0) or as a YAML int (5). yamlNumberUnknown means "no opinion, let yaml.v3
// format it".
type yamlNumberKind int

const (
	yamlNumberUnknown yamlNumberKind = iota
	yamlNumberFloat
	yamlNumberInt
)

// yamlFloatScalar marshals as a tagged YAML float so an integral value keeps
// its decimal point. Same mechanism as fixedPrecisionFloat, but with the
// shortest round-tripping representation instead of a fixed decimal count.
type yamlFloatScalar float64

func (f yamlFloatScalar) MarshalYAML() (any, error) {
	return &yaml.Node{
		Kind:  yaml.ScalarNode,
		Tag:   "!!float",
		Value: formatYAMLFloat(float64(f)),
	}, nil
}

// formatYAMLFloat renders v as a scalar that always reads back as a float.
// strconv 'f' with precision -1 gives the shortest representation that
// round-trips and never uses an exponent, so the only missing piece is the
// trailing ".0" on an integral value — exactly the demotion that bricked the
// localizer.
func formatYAMLFloat(v float64) string {
	s := strconv.FormatFloat(v, 'f', -1, 64)
	if !strings.ContainsAny(s, ".eE") {
		s += ".0"
	}
	return s
}

// yamlTypeHints answers "must this parameter key be written as a float or as
// an int?" for one save. Built once per write from the schema and the document
// currently on disk.
type yamlTypeHints struct {
	schema   map[string]yamlNumberKind
	template map[string]yamlNumberKind
	onDisk   map[string]yamlNumberKind
}

// newYAMLTypeHints builds the hints for a write. schema may be nil (the schema
// failed to load); existingYAML may be empty (first write of the file). With
// both empty every key falls through to yaml.v3's default formatting, which is
// the pre-fix behaviour.
func newYAMLTypeHints(schema map[string]any, existingYAML map[string]any) yamlTypeHints {
	return yamlTypeHints{
		schema:   schemaNumberKinds(schema),
		template: loadTemplateNumberKinds(),
		onDisk:   onDiskNumberKinds(existingYAML),
	}
}

// kindFor resolves one key. The two DECLARED sources (schema, then template)
// win over the on-disk observation, so a file an earlier save already demoted
// is repaired on the next write for every key either of them knows about. The
// on-disk type can only ever preserve what is already there.
func (h yamlTypeHints) kindFor(key string) yamlNumberKind {
	for _, source := range []map[string]yamlNumberKind{h.schema, h.template, h.onDisk} {
		if kind, ok := source[key]; ok {
			return kind
		}
	}
	return yamlNumberUnknown
}

// schemaNumberKinds maps every numeric key the JSON schema declares to its
// YAML kind. It reuses extractKeyTypes, the same walk that drives
// coerceValue, so the schema is read exactly one way.
func schemaNumberKinds(schema map[string]any) map[string]yamlNumberKind {
	kinds := map[string]yamlNumberKind{}
	if schema == nil {
		return kinds
	}
	schemaTypes := map[string]string{}
	extractKeyTypes(schema, schemaTypes)
	for key, schemaType := range schemaTypes {
		switch schemaType {
		case "number":
			kinds[key] = yamlNumberFloat
		case "integer":
			kinds[key] = yamlNumberInt
		}
	}
	return kinds
}

// onDiskNumberKinds records, for every numeric scalar in the parsed document,
// whether YAML decoded it as a float or an int. This is what preserves keys
// the schema does not cover (lidar_map_resolution_m, lidar_map_tile_size_m,
// lidar_map_radius_tiles, tick_rate, imu_cal_auto_rest_sec, ...).
//
// Keyed by parameter name, not by path: on a key that appears under two nodes
// with different numeric types the last writer wins, the same caveat
// flattenROS2YAML already documents. Parameter keys are unique across nodes.
func onDiskNumberKinds(doc map[string]any) map[string]yamlNumberKind {
	kinds := map[string]yamlNumberKind{}
	var walk func(value any)
	walk = func(value any) {
		switch typed := value.(type) {
		case map[string]any:
			for key, child := range typed {
				if kind := scalarNumberKind(child); kind != yamlNumberUnknown {
					kinds[key] = kind
					continue
				}
				walk(child)
			}
		case []any:
			for _, child := range typed {
				walk(child)
			}
		}
	}
	walk(doc)
	return kinds
}

// scalarNumberKind classifies a decoded YAML scalar by its Go type. A string
// that happens to hold digits is NOT numeric here: "115200" must stay a
// quoted string.
func scalarNumberKind(value any) yamlNumberKind {
	switch value.(type) {
	case float32, float64:
		return yamlNumberFloat
	case int, int8, int16, int32, int64, uint, uint8, uint16, uint32, uint64:
		return yamlNumberInt
	default:
		return yamlNumberUnknown
	}
}

// numericValue reports value as a float64 only when it is a Go numeric type.
// Deliberately narrower than asFloat64, which also parses strings: retyping a
// string would silently rewrite gnss_serial_device or a quoted baud rate.
func numericValue(value any) (float64, bool) {
	if _, isString := value.(string); isString {
		return 0, false
	}
	return asFloat64(value)
}

// retypeNumber converts value to the Go type that marshals as the wanted YAML
// kind. It returns ok=false when it must not interfere, and the caller then
// leaves the value for yaml.v3 to format.
func retypeNumber(value any, kind yamlNumberKind) (any, bool) {
	if kind == yamlNumberUnknown {
		return nil, false
	}
	number, ok := numericValue(value)
	if !ok || math.IsNaN(number) || math.IsInf(number, 0) {
		return nil, false
	}
	switch kind {
	case yamlNumberFloat:
		return yamlFloatScalar(number), true
	case yamlNumberInt:
		// A non-integral (or unrepresentable) value under an integer key is a
		// schema violation upstream. Write it through unchanged rather than
		// silently truncating the operator's number.
		if number != math.Trunc(number) || number < math.MinInt64 || number > math.MaxInt64 {
			return nil, false
		}
		return int64(number), true
	}
	return nil, false
}

// applyTypes returns a copy of the tree with every numeric scalar replaced by
// a value that marshals with the right YAML type. Maps and sequences are
// rebuilt rather than mutated (the caller's tree is left untouched).
func (h yamlTypeHints) applyTypes(value any) any {
	switch typed := value.(type) {
	case map[string]any:
		out := make(map[string]any, len(typed))
		for key, child := range typed {
			out[key] = h.applyScalarType(key, child)
		}
		return out
	case []any:
		out := make([]any, len(typed))
		for i, child := range typed {
			out[i] = h.applyTypes(child)
		}
		return out
	default:
		return value
	}
}

// applyScalarType handles one mapping entry. Lat/lon fixed precision wins over
// the generic rule so datum_lat keeps its nine decimals.
func (h yamlTypeHints) applyScalarType(key string, value any) any {
	if fixedPrecisionYAMLKeys[key] {
		if f, ok := asFloat64(value); ok {
			return fixedPrecisionFloat(f)
		}
	}
	if retyped, ok := retypeNumber(value, h.kindFor(key)); ok {
		return retyped
	}
	return h.applyTypes(value)
}

// marshalROS2YAML serialises the nested ROS2 config, preserving lat/lon
// precision and each numeric key's float-vs-int YAML type.
func marshalROS2YAML(nested map[string]any, hints yamlTypeHints) ([]byte, error) {
	return yaml.Marshal(hints.applyTypes(nested))
}
