// Package systemmetrics reads host health without invoking external commands.
package systemmetrics

import (
	"fmt"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"
)

// CPUUsageSampler reports aggregate Linux CPU usage on a 0–100% scale across
// all cores, not GUI-process/container usage or load average. Standard Docker
// /proc/stat exposes the host counters; no extra privileges or mounts are needed.
// Sampling is demand-driven: the diagnostics poll supplies the interval, with
// no sleeping HTTP handler or background goroutine when nobody is watching.
type CPUUsageSampler struct {
	mu       sync.Mutex
	readFile func(string) ([]byte, error)
	now      func() time.Time
	previous [8]uint64
	at       time.Time
	valid    bool
	usage    *float64
}

func NewCPUUsageSampler() *CPUUsageSampler {
	return &CPUUsageSampler{readFile: os.ReadFile, now: time.Now}
}

// Usage returns nil until two valid samples exist, and on unavailable or reset
// counters. Calls within one second share a sample; a gap over 30 seconds starts
// a new baseline rather than presenting a long unattended average as current.
func (s *CPUUsageSampler) Usage() *float64 {
	s.mu.Lock()
	defer s.mu.Unlock()
	now := s.now()
	elapsed := now.Sub(s.at)
	if !s.at.IsZero() && elapsed >= 0 && elapsed < time.Second {
		return s.usage
	}
	data, err := s.readFile("/proc/stat")
	current, parseErr := parseCPUStat(data)
	if err != nil || parseErr != nil {
		s.valid, s.usage, s.at = false, nil, now
		return nil
	}
	previous, valid := s.previous, s.valid
	s.previous, s.at, s.valid, s.usage = current, now, true, nil
	if !valid || elapsed < 0 || elapsed > 30*time.Second {
		return nil
	}
	var total, idle float64
	for i, value := range current {
		// In particular, Linux iowait can decrease. Never underflow into a
		// fabricated percentage: rebaseline for the next interval instead.
		if value < previous[i] {
			return nil
		}
		delta := float64(value - previous[i])
		total += delta
		if i == 3 || i == 4 { // idle + iowait are not busy CPU time
			idle += delta
		}
	}
	if total > 0 {
		usage := 100 * (total - idle) / total
		s.usage = &usage
	}
	return s.usage
}

func parseCPUStat(data []byte) ([8]uint64, error) {
	var counters [8]uint64
	fields := strings.Fields(strings.SplitN(string(data), "\n", 2)[0])
	if len(fields) < 9 || fields[0] != "cpu" {
		return counters, fmt.Errorf("missing aggregate CPU counters")
	}
	// user, nice, system, idle, iowait, irq, softirq, steal. guest and
	// guest_nice are already included in user/nice and must not be added twice.
	for i := range counters {
		value, err := strconv.ParseUint(fields[i+1], 10, 64)
		if err != nil {
			return counters, err
		}
		counters[i] = value
	}
	return counters, nil
}
