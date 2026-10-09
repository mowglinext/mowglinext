package blackbox

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
