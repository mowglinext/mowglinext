package api

import (
	"bytes"
	"encoding/json"
	"sort"
	"sync"
	"time"
)

// Rate limit for the diagnostics topic on the browser multiplex route (#1097).
//
// The GNSS sidecar publishes one ~10 kB DiagnosticArray of 18 statuses at its
// fix rate (10 Hz), and the robot's other producers add a few arrays per
// second more. The web client merges statuses by name and renders them once
// per second, so streaming every array to a remote client (Tailscale, mobile
// data) costs ~34 kB/s after deflate per client for nothing it can show.
//
// Each status, keyed by (name, hardware_id), is forwarded at most once per
// diagnosticsBrowserInterval, with two rules that keep the client's
// transition and episode tracking (#1017) exact:
//
//   - the first report of a key, and any report whose level or message differs
//     from the last one forwarded for that key, goes out at once;
//   - a report held back by the interval is not lost: the newest one per key is
//     kept and sent when its interval ends, even if the producer then goes
//     quiet, so the client is never more than one interval behind.
//
// A forwarded status keeps the header of the array it was published in, and a
// key's reports are forwarded in arrival order, so the client's per-key
// publication-stamp ordering (advanceDiagnosticProgress) still only sees
// stamps move forward. Anything that does not parse as a DiagnosticArray, and
// any status without a string name (the client treats those as malformed and
// invalidates the entry), is passed through unchanged.
//
// Only the browser multiplex delivery is limited: the provider's lossless
// diagnostics FIFO, the dedicated /subscribe route and every server-side
// consumer still see every array.

const diagnosticsBrowserInterval = time.Second

// diagnosticsMaxTrackedKeys bounds the per-connection state against a producer
// that invents status names; reports of further keys are forwarded unlimited.
const diagnosticsMaxTrackedKeys = 1024

type diagnosticKey struct {
	name       string
	hardwareID string
}

type diagnosticForwarded struct {
	level   json.RawMessage
	message json.RawMessage
	at      time.Time
}

type diagnosticHeld struct {
	status json.RawMessage
	level  json.RawMessage
	msg    json.RawMessage
	array  uint64 // arrival order of the array it came from
	index  int    // position inside that array
}

type diagnosticStatusWire struct {
	Name       *string         `json:"name"`
	HardwareID string          `json:"hardware_id"`
	Level      json.RawMessage `json:"level"`
	Message    json.RawMessage `json:"message"`
}

// diagnosticsRateLimiter is the per-connection state. Time is passed in so the
// tests can drive it without sleeping; diagnosticsRateGate adds the locking
// and the trailing timer.
type diagnosticsRateLimiter struct {
	interval  time.Duration
	forwarded map[diagnosticKey]diagnosticForwarded
	held      map[diagnosticKey]diagnosticHeld
	// Top-level fields (header) of each array that still has a held status.
	arrays map[uint64]map[string]json.RawMessage
	next   uint64
}

func newDiagnosticsRateLimiter(interval time.Duration) *diagnosticsRateLimiter {
	return &diagnosticsRateLimiter{
		interval:  interval,
		forwarded: map[diagnosticKey]diagnosticForwarded{},
		held:      map[diagnosticKey]diagnosticHeld{},
		arrays:    map[uint64]map[string]json.RawMessage{},
	}
}

// admit returns what to send now for one incoming array: the array reduced to
// the statuses that are due, or nothing when all of them are held back.
// Unparseable input is returned unchanged.
func (l *diagnosticsRateLimiter) admit(now time.Time, msg []byte) []byte {
	var fields map[string]json.RawMessage
	if err := json.Unmarshal(msg, &fields); err != nil {
		return msg
	}
	var statuses []json.RawMessage
	if err := json.Unmarshal(fields["status"], &statuses); err != nil || len(statuses) == 0 {
		return msg
	}
	id := l.next
	l.next++
	out := make([]json.RawMessage, 0, len(statuses))
	for i, raw := range statuses {
		var s diagnosticStatusWire
		if err := json.Unmarshal(raw, &s); err != nil || s.Name == nil {
			out = append(out, raw)
			continue
		}
		key := diagnosticKey{name: *s.Name, hardwareID: s.HardwareID}
		last, seen := l.forwarded[key]
		if !seen && len(l.forwarded) >= diagnosticsMaxTrackedKeys {
			out = append(out, raw)
			continue
		}
		// A newer report always supersedes a held one.
		l.release(key)
		changed := !seen || !bytes.Equal(last.level, s.Level) || !bytes.Equal(last.message, s.Message)
		if changed || now.Sub(last.at) >= l.interval {
			l.forwarded[key] = diagnosticForwarded{level: s.Level, message: s.Message, at: now}
			out = append(out, raw)
			continue
		}
		l.held[key] = diagnosticHeld{status: raw, level: s.Level, msg: s.Message, array: id, index: i}
		l.arrays[id] = fields
	}
	if len(out) == 0 {
		return nil
	}
	frame, ok := encodeDiagnosticArray(fields, out)
	if !ok {
		return msg
	}
	return frame
}

// release drops a key's held report, and its array once nothing of it is held.
func (l *diagnosticsRateLimiter) release(key diagnosticKey) {
	h, ok := l.held[key]
	if !ok {
		return
	}
	delete(l.held, key)
	for _, other := range l.held {
		if other.array == h.array {
			return
		}
	}
	delete(l.arrays, h.array)
}

// flush returns the held reports whose interval has ended: one frame per
// source array, in arrival order, each with that array's own header.
func (l *diagnosticsRateLimiter) flush(now time.Time) [][]byte {
	due := map[uint64][]diagnosticKey{}
	for key, h := range l.held {
		if now.Sub(l.forwarded[key].at) >= l.interval {
			due[h.array] = append(due[h.array], key)
		}
	}
	ids := make([]uint64, 0, len(due))
	for id := range due {
		ids = append(ids, id)
	}
	sort.Slice(ids, func(i, j int) bool { return ids[i] < ids[j] })
	var frames [][]byte
	for _, id := range ids {
		keys := due[id]
		sort.Slice(keys, func(i, j int) bool { return l.held[keys[i]].index < l.held[keys[j]].index })
		fields := l.arrays[id]
		statuses := make([]json.RawMessage, 0, len(keys))
		for _, key := range keys {
			h := l.held[key]
			l.forwarded[key] = diagnosticForwarded{level: h.level, message: h.msg, at: now}
			statuses = append(statuses, h.status)
			l.release(key)
		}
		if frame, ok := encodeDiagnosticArray(fields, statuses); ok {
			frames = append(frames, frame)
		}
	}
	return frames
}

// nextFlush is how long until the earliest held report is due; false when
// nothing is held.
func (l *diagnosticsRateLimiter) nextFlush(now time.Time) (time.Duration, bool) {
	var wait time.Duration
	found := false
	for key := range l.held {
		w := l.forwarded[key].at.Add(l.interval).Sub(now)
		if !found || w < wait {
			wait, found = w, true
		}
	}
	return max(wait, 0), found
}

func encodeDiagnosticArray(fields map[string]json.RawMessage, statuses []json.RawMessage) ([]byte, bool) {
	status, err := json.Marshal(statuses)
	if err != nil {
		return nil, false
	}
	out := make(map[string]json.RawMessage, len(fields))
	for k, v := range fields {
		out[k] = v
	}
	out["status"] = status
	frame, err := json.Marshal(out)
	return frame, err == nil
}

// diagnosticsRateGate serialises the limiter between the subscription callback
// and its trailing timer. It sends while holding its lock, so a held report is
// never written after a newer report of the same key.
type diagnosticsRateGate struct {
	mu       sync.Mutex
	limiter  *diagnosticsRateLimiter
	send     func([]byte)
	timer    *time.Timer
	timerGen uint64 // a fired timer whose generation is stale does nothing
	closed   bool
}

func newDiagnosticsRateGate(interval time.Duration, send func([]byte)) *diagnosticsRateGate {
	return &diagnosticsRateGate{limiter: newDiagnosticsRateLimiter(interval), send: send}
}

func (g *diagnosticsRateGate) deliver(msg []byte) {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.closed {
		return
	}
	now := time.Now()
	if frame := g.limiter.admit(now, msg); frame != nil {
		g.send(frame)
	}
	g.flushLocked(now)
}

func (g *diagnosticsRateGate) onTimer(gen uint64) {
	g.mu.Lock()
	defer g.mu.Unlock()
	if g.closed || gen != g.timerGen {
		return
	}
	g.timer = nil
	g.flushLocked(time.Now())
}

// flushLocked sends what is due and re-arms one timer while anything is held.
func (g *diagnosticsRateGate) flushLocked(now time.Time) {
	for _, frame := range g.limiter.flush(now) {
		g.send(frame)
	}
	g.stopTimerLocked()
	wait, ok := g.limiter.nextFlush(now)
	if !ok {
		return
	}
	gen := g.timerGen
	g.timer = time.AfterFunc(wait, func() { g.onTimer(gen) })
}

func (g *diagnosticsRateGate) stopTimerLocked() {
	g.timerGen++
	if g.timer != nil {
		g.timer.Stop()
		g.timer = nil
	}
}

func (g *diagnosticsRateGate) close() {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.closed = true
	g.stopTimerLocked()
}
