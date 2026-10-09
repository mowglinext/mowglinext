package blackbox

import (
	"strconv"
	"strings"
)

func parseMemoryInfo(data []byte) resourceSample {
	s := resourceSample{}
	availableFound := false
	for _, line := range strings.Split(string(data), "\n") {
		parts := strings.Fields(line)
		if len(parts) < 2 {
			continue
		}
		n, err := strconv.ParseInt(parts[1], 10, 64)
		if err != nil || n < 0 {
			continue
		}
		switch parts[0] {
		case "MemTotal:":
			s.total = n * 1024
		case "MemAvailable:":
			s.available = n * 1024
			availableFound = true
		}
	}
	// Zero is a valid pressure sample; distinguish it from a missing field.
	s.known = s.total > 0 && availableFound
	return s
}

type resourceSample struct {
	total, available int64
	known            bool
}

func initialMemoryBudget(requested int64, s resourceSample) int64 {
	if !s.known {
		return min(requested, 8<<20)
	}
	return max(4<<20, min(requested, min(s.total/32, s.available/8)))
}

func (r *Recorder) applyPressure(s resourceSample) {
	r.pressure = s.known && s.available < 32<<20
	if r.pressure {
		// Relinquish rolling history first. An existing incident remains bounded
		// by the configured capture budget, but no new optional history is kept.
		for r.buffer.count > 0 {
			r.buffer.pop()
		}
	}
}
