package api

import (
	"encoding/json"
	"fmt"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

func labCompareFrame(t testing.TB, topic string, raw []byte) {
	t.Helper()
	want, we := labDevFrame(topic, raw)
	got, ge := labCandidateFrame(topic, raw)
	if (we == nil) != (ge == nil) {
		t.Fatalf("acceptance differs: %v / %v", we, ge)
	}
	if we == nil {
		labExactValue(t, labDecoded(t, want), labDecoded(t, got))
	}
}

func TestMultiplexGridExactDomainAndFallback(t *testing.T) {
	for _, n := range []int{0, 1, 15, 16, 255, 256, 65535, 65536, 300000} {
		cells := make([]int, n)
		for i := range cells {
			cells[i] = i%256 - 128
		}
		raw, err := json.Marshal(cells)
		require.NoError(t, err)
		labCompareFrame(t, "mowProgress", append(append([]byte(`{"data":`), raw...), []byte(`,"header":{"stamp":{"sec":-2147483648,"nanosec":4294967295},"frame_id":"ma\u0070"},"extra":{"signed":-0,"subnormal":5e-324,"max":1.7976931348623157e308,"large":1791040000000000000,"inexact_id":9007199254740993,"uint64_max":18446744073709551615,"unicode":"å🌱","html":"<>&","binary":"AAH/","nested":[null,true,false,1.0000000000000002]}}`)...))
	}
	for _, raw := range []string{
		`{"data":[null,0]}`, `{"data":[-0,0]}`, `{"data":[0.0,1e0]}`, `{"data":[-129,128,4294967296,9007199254740992]}`,
		`{"data":null}`, `{"data":"[-1,0]"}`, `{"data":{},"Info":{"data":[1]}}`,
		`{"data":[127],"data":[-128],"\u0064ata":[-1,0,100],"info":{"x":1,"x":-0}}`,
		`{"data":[1e400],"data":[1]}`, `{"extra":1e400,"extra":0,"data":[1]}`,
		`{"d\u0061ta":[1e400],"data":[1]}`, `{"data":[1],"extra":{"x":1e400,"x":0}}`,
		`{"data":[  -128 ,\n 0,127]}`, `{"data":[1,]}`, `{"data":[01]}`, `{"data":[1e400]}`, `[]`, `null`, `{"data":[1]} trailing`,
	} {
		t.Run(raw, func(t *testing.T) {
			labCompareFrame(t, "lidarMap", []byte(raw))
			// Exercise fallback after the large-grid dispatch, too.
			if strings.HasPrefix(raw, "{") && strings.HasSuffix(raw, "}") {
				large := raw[:len(raw)-1] + `,"padding":"` + strings.Repeat("x", 5000) + `"}`
				labCompareFrame(t, "lidarMap", []byte(large))
			}
		})
	}
	for _, raw := range []string{`null`, `[-0]`, `[null]`, `[1.0]`, `[1e0]`, `[128]`, `[-129]`, `[999999999999999999999999999999]`, `[01]`, `[1,]`, `[1]x`, `[+1]`, `[--1]`, `[NaN]`, `[true]`} {
		labCompareFrame(t, "lidarMap", []byte(`{"data":`+raw+`}`))
	}
	for _, topic := range []string{"pose", "gnssStatus", "path", "diagnostics"} {
		labCompareFrame(t, topic, []byte(`{"data":[-128,0,127],"xy":[-0,0.125,1.23456789],"extra":{"f":-0}}`))
	}
}

func TestMultiplexGridHeldOut(t *testing.T) {
	for _, seed := range []int64{971, 1321, 91823} {
		for _, n := range []int{17, 4000, 17000, 470000} {
			for _, mode := range []string{"stripes", "lidar", "random"} {
				t.Run(fmt.Sprintf("%s/%d/%d", mode, n, seed), func(t *testing.T) { labCompareFrame(t, "lidarMap", labGrid(n, mode, seed)) })
			}
		}
	}
}

func TestMultiplexGridWholeDocumentDepth(t *testing.T) {
	for _, depth := range []int{9998, 9999, 10000} {
		raw := `{"data":[1],"extra":` + strings.Repeat("[", depth) + "0" + strings.Repeat("]", depth) + "}"
		labCompareFrame(t, "lidarMap", []byte(raw))
	}
}

func TestMultiplexGridProductionRouteExact(t *testing.T) {
	for _, topic := range []string{"lidarMap", "mowProgress"} {
		raw := labGrid(65536, "random", 9721)
		want, err := labDevFrame(topic, raw)
		require.NoError(t, err)
		got := compactWireFrame(t, topic, raw)
		labExactValue(t, labDecoded(t, want), labDecoded(t, got))
	}
}

func FuzzMultiplexGridExact(f *testing.F) {
	for _, s := range []string{`{"data":[-128,-32,-1,0,127],"x":-0}`, `{"data":[null,-0,0.125,1e30]}`, `{"data":[1],"data":[2]}`, `{"data":[],"extra":{"a":null}}`, `[-128,-32,-1,0,127]`, `[-0,null,0.125]`} {
		f.Add([]byte(s))
	}
	f.Fuzz(func(t *testing.T, raw []byte) {
		if len(raw) > 1<<20 {
			t.Skip()
		}
		labCompareFrame(t, "mowProgress", raw)
		// Keep the fast-path dispatch exercised even for tiny fuzz inputs.
		wrapped := append([]byte(`{"data":`), raw...)
		wrapped = append(wrapped, []byte(`,"padding":"`+strings.Repeat("x", 4096)+`"}`)...)
		labCompareFrame(t, "mowProgress", wrapped)
	})
}
