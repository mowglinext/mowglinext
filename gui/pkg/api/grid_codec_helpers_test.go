package api

import (
	"bytes"
	"encoding/json"
	"math"
	"math/rand"
	"reflect"
	"testing"

	"github.com/vmihailenco/msgpack/v5"
)

// Original generic encoding path used as the differential-test reference.
func labDevFrame(topic string, data []byte) ([]byte, error) {
	var obj any
	if err := json.Unmarshal(data, &obj); err != nil {
		return nil, err
	}
	if topic == "path" {
		compactCoveragePreview(obj)
	}
	var b bytes.Buffer
	e := msgpack.NewEncoder(&b)
	e.UseCompactInts(true)
	err := e.Encode(map[string]any{"topic": topic, "data": compactMultiplexNumbers(obj)})
	return b.Bytes(), err
}

func labGrid(n int, mode string, seed int64) []byte {
	cells := make([]int, n)
	rng := rand.New(rand.NewSource(seed))
	for i := range cells {
		switch mode {
		case "stripes":
			if i%1000 < 40 {
				cells[i] = 100
			}
		case "lidar":
			cells[i] = -1
			if i < n/3 {
				cells[i] = 0
			}
			if i%401 < 7 {
				cells[i] = 100
			}
		case "random":
			cells[i] = rng.Intn(102) - 1
		}
	}
	b, _ := json.Marshal(map[string]any{"header": map[string]any{"stamp": map[string]any{"sec": 1794567890, "nanosec": 123456789}, "frame_id": "map"}, "info": map[string]any{"width": n / 500, "height": 500, "resolution": 0.05, "map_load_time": map[string]any{"sec": 1794567000, "nanosec": 7}, "origin": map[string]any{"position": map[string]any{"x": -18.987654321, "y": 59.123456789, "z": 0}, "orientation": map[string]any{"x": 0, "y": 0, "z": 0, "w": 1}}}, "data": cells})
	return b
}

// Unlike JSON equality or reflect.DeepEqual on floats, this checks all decoded
// wire types and floating-point bits, including signed zero, recursively.
func labExactValue(t testing.TB, want, got any) {
	t.Helper()
	if !labSameValue(want, got) {
		t.Fatalf("decoded types, field values or floating-point bits differ")
	}
}

func labSameValue(want, got any) bool {
	if reflect.TypeOf(want) != reflect.TypeOf(got) {
		return false
	}
	switch w := want.(type) {
	case float64:
		return math.Float64bits(w) == math.Float64bits(got.(float64))
	case float32:
		return math.Float32bits(w) == math.Float32bits(got.(float32))
	case map[string]any:
		g := got.(map[string]any)
		if len(w) != len(g) {
			return false
		}
		for k, v := range w {
			x, ok := g[k]
			if !ok || !labSameValue(v, x) {
				return false
			}
		}
		return true
	case []any:
		g := got.([]any)
		if len(w) != len(g) {
			return false
		}
		for i := range w {
			if !labSameValue(w[i], g[i]) {
				return false
			}
		}
		return true
	case nil, int8, int16, int32, int64, uint8, uint16, uint32, uint64, string, bool:
		return want == got
	default:
		return reflect.DeepEqual(want, got)
	}
}

func labDecoded(t testing.TB, raw []byte) any {
	t.Helper()
	var obj any
	if err := msgpack.Unmarshal(raw, &obj); err != nil {
		t.Fatal(err)
	}
	return obj
}

func labCandidateFrame(topic string, data []byte) ([]byte, error) {
	obj, err := multiplexObject(topic, data)
	if err != nil {
		return nil, err
	}
	if topic == "path" {
		compactCoveragePreview(obj)
	}
	var b bytes.Buffer
	e := msgpack.NewEncoder(&b)
	e.UseCompactInts(true)
	err = e.Encode(map[string]any{"topic": topic, "data": compactMultiplexNumbers(obj)})
	return b.Bytes(), err
}
