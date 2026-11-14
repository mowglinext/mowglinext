package api

import (
	"bytes"
	"encoding/json"
	"math"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
	"github.com/vmihailenco/msgpack/v5"
)

func compareGridDecode(t testing.TB, raw []byte) {
	t.Helper()
	var want gridWire
	we := json.Unmarshal(raw, &want)
	got, parsed, ge := decodeGrid(raw)
	if (we == nil) != (ge == nil) {
		t.Fatalf("decode acceptance: %v / %v", we, ge)
	}
	if we != nil {
		if we.Error() != ge.Error() {
			t.Fatalf("error differs: %v / %v", we, ge)
		}
		return
	}
	if !bytes.Equal(want.Header, got.Header) || !bytes.Equal(want.Info.Origin, got.Info.Origin) {
		t.Fatal("raw header/origin changed")
	}
	require.Equal(t, want, got)
	require.Equal(t, math.Float64bits(want.Info.Resolution), math.Float64bits(got.Info.Resolution))
	if parsed != nil {
		wantFrame, we := labDevFrame("lidarMap", raw)
		obj, ge := parsed.object()
		if (we == nil) != (ge == nil) {
			t.Fatalf("full-frame acceptance differs: %v / %v", we, ge)
		}
		if we == nil {
			var out bytes.Buffer
			e := msgpack.NewEncoder(&out)
			e.UseCompactInts(true)
			require.NoError(t, e.Encode(map[string]any{"topic": "lidarMap", "data": compactMultiplexNumbers(obj)}))
			labExactValue(t, labDecoded(t, wantFrame), labDecoded(t, out.Bytes()))
		}
	}
}

func TestGridDecodeExactMetadataAndFallback(t *testing.T) {
	padding := `,"padding":"` + strings.Repeat("x", 4096) + `"`
	for _, doc := range []string{
		`{"data":[-128,-32,-1,0,127],"header": { "stamp" : [1,2] },"info":{"width":5,"height":1,"resolution":-0,"origin": { "x" : -0 }}}`,
		`{"data":[null,-0,0.0,1e0]}`, `{"data":[128]}`, `{"data":[-129]}`, `{"data":[]}`, `{"data":null}`,
		`{"data":[1],"info":null,"header":null}`, `{"data":[1],"info":{"width":4294967296}}`,
		`{"data":[1],"info":{"width":1,"width":2,"origin": [ 1, 2 ]}}`,
		`{"data":[1],"info":{"width":1},"INFO":{"height":2}}`,
		`{"DATA":[1]}`, `{"data":[1],"DATA":[2]}`, `{"data":[128],"data":[1]}`,
		`{"data":[1],"header":1,"HEADER":2}`, `{"d\u0061ta":[1],"data":[2]}`,
		`{"data":[1],"extra":{"x":1e400,"x":0}}`, `{"data":[1],"extra":1e400}`, `{"data":[1],"extra":1e400,"extra":0}`,
		`{"data":[1],"header":1e400}`, `{"data":[1],"info":{"origin":{"a":1e400}}}`,
		`{"data":[1],"info":{"width":1,"height":1,"resolution":5e-324,"origin":"å<>&"},"extra":{"id":18446744073709551615,"n":-0}}`,
		` { "d\u0061ta" : [ -128 , -32 , 0 , 127 ] , "header" : {"braces":"}],[\\\""}, "extra" : [null,true,{"a":"\\\\","b":"\u0000"}] } `,
	} {
		t.Run(doc, func(t *testing.T) {
			end := strings.LastIndex(doc, "}")
			large := []byte(doc[:end] + padding + doc[end:])
			before := append([]byte(nil), large...)
			compareGridDecode(t, large)
			require.Equal(t, before, large, "input must remain immutable")
		})
	}
	for _, depth := range []int{9998, 9999, 10000} {
		compareGridDecode(t, []byte(`{"data":[1],"extra":`+strings.Repeat("[", depth)+"0"+strings.Repeat("]", depth)+"}"))
	}
	for _, n := range []int{8192, 300000, 1000000} {
		compareGridDecode(t, labGrid(n, "random", 1321))
	}
}

func TestGridDecodeCanonicalBoundaries(t *testing.T) {
	for _, data := range []string{`"data":[-128,-32,0,127]`, `"d\u0061ta":[ -128 , -32 , 0 , 127 ]`} {
		raw := []byte(" \n\t{\"extra\":[null,true,{\"braces\":\"}],[\\\\\\\"\"}]," + data + `,"header": {"stamp":1},"padding":"` + strings.Repeat("x", 4096) + `" } `)
		_, parsed, err := decodeGrid(raw)
		require.NoError(t, err)
		require.NotNil(t, parsed)
		compareGridDecode(t, raw)
	}
}

func TestParsedGridCellBytesMatchGenericEncoder(t *testing.T) {
	for _, n := range []int{0, 1, 15, 16, 255, 256, 65535, 65536, 300000} {
		cells := make([]int8, n)
		generic := make([]any, n)
		for i := range cells {
			cells[i] = int8(i%256 - 128)
			generic[i] = float64(cells[i])
		}
		var expected bytes.Buffer
		e := msgpack.NewEncoder(&expected)
		e.UseCompactInts(true)
		require.NoError(t, e.Encode(compactMultiplexNumbers(generic)))
		require.Equal(t, expected.Bytes(), []byte(packInt8Cells(cells)))
		raw, err := json.Marshal(cells)
		require.NoError(t, err)
		decoded, ok := decodeGridCells(raw)
		require.True(t, ok)
		require.Equal(t, cells, decoded)
	}
}

func TestGridDeltaMatchesFrozenStateMachine(t *testing.T) {
	var reference referenceGridDelta
	var candidate gridDelta
	for i := 0; i < 70; i++ {
		raw := labGrid(8192, "lidar", 971)
		var obj map[string]any
		require.NoError(t, json.Unmarshal(raw, &obj))
		obj["header"].(map[string]any)["stamp"].(map[string]any)["sec"] = i
		obj["data"].([]any)[i%8192] = i % 101
		if i >= 35 {
			obj["info"].(map[string]any)["width"] = 999
		}
		raw, err := json.Marshal(obj)
		require.NoError(t, err)
		if i == 13 {
			raw = bytes.Replace(raw, []byte(`"data":[`), []byte(`"DATA":[`), 1)
		}
		if i == 24 {
			raw = []byte(`{"data":[1,]}`)
		}
		want, wo := reference.next(raw)
		got, goOK := candidate.next(raw)
		require.Equal(t, wo, goOK)
		parsed := got.parsed
		got.parsed = nil
		require.Equal(t, want, got)
		if parsed != nil {
			compareGridDecode(t, raw)
		}
		if i%11 == 0 {
			wf, wok := reference.resync()
			gf, gok := candidate.resync()
			require.Equal(t, wok, gok)
			if gf.parsed != nil {
				compareGridDecode(t, gf.Raw)
			}
			gf.parsed = nil
			require.Equal(t, wf, gf)
		}
	}
}

func TestParsedGridSnapshotSurvivesLaterMessages(t *testing.T) {
	var d gridDelta
	first := labGrid(8192, "random", 971)
	f, ok := d.next(first)
	require.True(t, ok)
	require.NotNil(t, f.parsed)
	original, err := f.parsed.object()
	require.NoError(t, err)
	_, ok = d.next(labGrid(8192, "random", 1321))
	require.True(t, ok)
	again, err := f.parsed.object()
	require.NoError(t, err)
	labExactValue(t, original, again)
	compareGridDecode(t, first)
}

func FuzzGridDecodeExact(f *testing.F) {
	for _, raw := range []string{`{"data":[-128,-1,0,127],"info":{"origin": { "x" : 1 }}}`, `{"data":[1],"INFO":{"width":3}}`, `{"data":[null,-0]}`, `{"data":[128],"data":[1]}`} {
		f.Add([]byte(raw))
	}
	f.Fuzz(func(t *testing.T, raw []byte) {
		if len(raw) > 32768 {
			t.Skip()
		}
		compareGridDecode(t, raw)
		// Force the fast dispatch for small generated documents without
		// discarding invalid syntax or relaxing the whole-document limit.
		if len(raw) > 1 && raw[0] == '{' && raw[len(raw)-1] == '}' {
			large := append(append([]byte(nil), raw[:len(raw)-1]...), []byte(`,"padding":"`+strings.Repeat("x", 4096)+`"}`)...)
			compareGridDecode(t, large)
		}
	})
}
