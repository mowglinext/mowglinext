package api

import (
	"fmt"
	"testing"
)

// Compare the original state machine with the shared decoder before wire
// encoding or I/O. Each benchmark invocation starts a fresh subscription.
func BenchmarkGridDeltaDecode(b *testing.B) {
	for _, cells := range []int{300000, 1000000} {
		raw := labGrid(cells, "lidar", 971)
		for _, reference := range []bool{true, false} {
			name := "shared"
			if reference {
				name = "reference"
			}
			b.Run(fmt.Sprintf("%d/%s", cells, name), func(b *testing.B) {
				var original referenceGridDelta
				var current gridDelta
				next := current.next
				if reference {
					next = original.next
				}
				full := 0
				b.ReportAllocs()
				b.ResetTimer()
				for i := 0; i < b.N; i++ {
					frame, ok := next(raw)
					if !ok {
						b.Fatal("invalid grid")
					}
					if frame.Full {
						full++
					}
				}
				b.StopTimer()
				b.ReportMetric(float64(full), "full-frames")
				b.ReportMetric(float64(b.N-full), "patch-frames")
			})
		}
	}
}

// Measure full-frame JSON→MessagePack separately from delta decoding and I/O.
func BenchmarkGridFullFrame(b *testing.B) {
	for _, mode := range []string{"stripes", "lidar", "random"} {
		raw := labGrid(300000, mode, 42)
		for _, codec := range []struct {
			name string
			fn   func(string, []byte) ([]byte, error)
		}{{"generic", labDevFrame}, {"shared", labCandidateFrame}} {
			b.Run(mode+"/"+codec.name, func(b *testing.B) {
				b.ReportAllocs()
				for i := 0; i < b.N; i++ {
					if _, err := codec.fn("lidarMap", raw); err != nil {
						b.Fatal(err)
					}
				}
			})
		}
	}
}
