package foxglove

import (
	"encoding/json"
	"math"
	"testing"
)

func TestSanitizeJSONValueReplacesNonFiniteFloats(t *testing.T) {
	raw := map[string]interface{}{
		"finite": 1.5,
		"nan":    math.NaN(),
		"nested": map[string]interface{}{
			"inf": math.Inf(1),
		},
		"array": []float64{2.0, math.NaN(), 4.0},
	}

	sanitized := sanitizeJSONValue(raw)
	data, err := json.Marshal(sanitized)
	if err != nil {
		t.Fatalf("json.Marshal(sanitizeJSONValue(...)) = %v", err)
	}

	var decoded map[string]interface{}
	if err := json.Unmarshal(data, &decoded); err != nil {
		t.Fatalf("json.Unmarshal = %v", err)
	}

	if decoded["nan"] != nil {
		t.Fatalf("nan field = %#v, want nil", decoded["nan"])
	}

	nested, ok := decoded["nested"].(map[string]interface{})
	if !ok {
		t.Fatalf("nested field type = %T, want map[string]interface{}", decoded["nested"])
	}
	if nested["inf"] != nil {
		t.Fatalf("nested.inf = %#v, want nil", nested["inf"])
	}

	array, ok := decoded["array"].([]interface{})
	if !ok {
		t.Fatalf("array field type = %T, want []interface{}", decoded["array"])
	}
	if len(array) != 3 {
		t.Fatalf("array len = %d, want 3", len(array))
	}
	if array[1] != nil {
		t.Fatalf("array[1] = %#v, want nil", array[1])
	}
}
