package systemmetrics

import (
	"errors"
	"fmt"
	"math"
	"sync"
	"testing"
	"time"
)

func TestCPUStat(t *testing.T) {
	got, err := parseCPUStat([]byte("cpu  10 20 30 40 50 60 70 80 999 999\ncpu0 1 2 3 4 5 6 7 8\n"))
	if err != nil || got != [8]uint64{10, 20, 30, 40, 50, 60, 70, 80} {
		t.Fatalf("aggregate counters (excluding guest): %v, %v", got, err)
	}
	for _, text := range []string{"", "cpu0 1 2 3 4 5 6 7 8", "cpu 1 2", "cpu -1 2 3 4 5 6 7 8", "cpu nope 2 3 4 5 6 7 8"} {
		if _, err := parseCPUStat([]byte(text)); err == nil {
			t.Errorf("accepted malformed counters: %q", text)
		}
	}
}

func TestCPUUsage(t *testing.T) {
	for _, tc := range []struct {
		name string
		next string
		want float64
	}{
		{"idle", "cpu 100 0 0 300 0 0 0 0", 0},
		{"busy", "cpu 300 0 0 100 0 0 0 0", 100},
		{"all cores normalized", "cpu 200 0 0 400 0 0 0 0", 25},
		{"iowait not busy", "cpu 150 0 0 200 50 0 0 0", 25},
		{"guest not doubled", "cpu 150 0 0 250 0 0 0 0 50 0", 25},
	} {
		t.Run(tc.name, func(t *testing.T) {
			now := time.Unix(100, 0)
			data := "cpu 100 0 0 100 0 0 0 0"
			s := &CPUUsageSampler{now: func() time.Time { return now }, readFile: func(string) ([]byte, error) { return []byte(data), nil }}
			if s.Usage() != nil {
				t.Fatal("first sample must be unknown")
			}
			data, now = tc.next, now.Add(10*time.Second)
			if got := s.Usage(); got == nil || math.Abs(*got-tc.want) > .001 {
				t.Fatalf("usage=%v, want %v", got, tc.want)
			}
		})
	}
}

func TestCPUUsageUnavailableAndRecovery(t *testing.T) {
	now := time.Unix(100, 0)
	data := "cpu 100 0 0 100 0 0 0 0"
	var readErr error
	s := &CPUUsageSampler{now: func() time.Time { return now }, readFile: func(string) ([]byte, error) { return []byte(data), readErr }}
	check := func(next string, advance time.Duration, known bool) {
		t.Helper()
		data, now = next, now.Add(advance)
		if got := s.Usage(); (got != nil) != known {
			t.Fatalf("sample %q: known=%v, want %v", next, got != nil, known)
		}
	}
	check(data, 0, false)
	check(data, 10*time.Second, false) // no elapsed CPU ticks
	check("cpu 200 0 0 200 0 0 0 0", 10*time.Second, true)
	check("cpu 10 0 0 10 0 0 0 0", 10*time.Second, false) // reset
	check("cpu 20 0 0 20 10 0 0 0", 10*time.Second, true)
	check("cpu 30 0 0 30 9 0 0 0", 10*time.Second, false) // decreasing iowait
	check("cpu 40 0 0 40 10 0 0 0", 10*time.Second, true)
	check("cpu 50 0 0 50 10 0 0 0", 31*time.Second, false) // long-unattended average
	check("cpu 60 0 0 60 10 0 0 0", 10*time.Second, true)
	readErr = errors.New("unavailable")
	check(data, 10*time.Second, false)
	readErr = nil
	check(data, 10*time.Second, false) // recovery needs a new baseline
	check("cpu 70 0 0 70 10 0 0 0", 10*time.Second, true)
	check("bad data", 10*time.Second, false)
	check("cpu 80 0 0 80 10 0 0 0", 10*time.Second, false)
	check("cpu 90 0 0 90 10 0 0 0", 10*time.Second, true)
}

func TestCPUUsageConcurrentRequestsShareSample(t *testing.T) {
	now := time.Unix(100, 0)
	reads := 0
	s := &CPUUsageSampler{now: func() time.Time { return now }, readFile: func(path string) ([]byte, error) {
		if path != "/proc/stat" {
			t.Errorf("unexpected path: %s", path)
		}
		reads++
		return []byte(fmt.Sprintf("cpu %d 0 0 %d 0 0 0 0", reads*100, reads*100)), nil
	}}
	s.Usage()
	now = now.Add(10 * time.Second)
	var wg sync.WaitGroup
	for i := 0; i < 32; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			if got := s.Usage(); got == nil || *got != 50 {
				t.Errorf("usage=%v", got)
			}
		}()
	}
	wg.Wait()
	if reads != 2 {
		t.Fatalf("concurrent clients sampled %d times, want 2", reads)
	}
}
