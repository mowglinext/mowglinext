package blackbox

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"os"
	"strings"
	"sync"
	"sync/atomic"
	"time"
)

type observation struct {
	at    time.Time
	topic string
	data  []byte
}

func (o observation) cost() int64 { return int64(len(o.data) + len(o.topic) + recordOverhead) }

type ring struct {
	slots        []observation
	start, count int
	bytes        int64
}

func (b *ring) pop() {
	if b.count == 0 {
		return
	}
	b.bytes -= b.slots[b.start].cost()
	b.slots[b.start] = observation{}
	b.start = (b.start + 1) % len(b.slots)
	b.count--
}

func (b *ring) expire(cutoff time.Time) {
	for b.count > 0 && b.slots[b.start].at.Before(cutoff) {
		b.pop()
	}
}

func (b *ring) append(o observation, limit int64) bool {
	if o.cost() > limit {
		return false
	}
	for b.count > 0 && (b.bytes+o.cost() > limit || b.count == len(b.slots)) {
		b.pop()
	}
	b.slots[(b.start+b.count)%len(b.slots)] = o
	b.bytes += o.cost()
	b.count++
	return true
}

type capture struct {
	metadata          Metadata
	trigger, deadline time.Time
	observations      []observation
	bytes, limit      int64
	drops             uint64
	startDrops        uint64
}

type writeResult struct {
	err      error
	duration time.Duration
	pruning  bool
}

// Recorder owns one collector and at most one disk writer. Ingest only copies
// a size-limited message after acquiring a queue permit; it never waits for the
// collector, storage, or configuration locks. All time retention uses Go's
// monotonic clock, independently of timestamps in ROS payloads.
type Recorder struct {
	mu                    sync.Mutex
	dir                   string
	cfg                   Config
	config                atomic.Pointer[Config]
	identifiers           map[string]string
	buffer                ring
	active                *capture
	writing               bool
	pruning               bool
	lastTrigger           time.Time
	lastError             string
	completed, coalesced  uint64
	historyEvictions      uint64
	skippedTriggerSources uint64
	lastWriteSeconds      float64
	dropped               atomic.Uint64
	queue                 chan observation
	permits               chan struct{}
	stop                  chan struct{}
	done                  chan struct{}
	result                chan writeResult
	closed                atomic.Bool
	stopOnce              sync.Once
	emergency             bool
	diagnosticErrors      map[string]bool
	effectiveMemory       int64
	pressure              bool
	now                   func() time.Time
	resources             func() resourceSample
	write                 func(*capture) error
	freeDisk              func(string) (int64, bool)
	maintain              func(Config) error
	removeCompleted       func(*os.Root, string) error
}

func New(dir string, cfg Config, identifiers map[string]string) (*Recorder, error) {
	return newRecorder(dir, cfg, identifiers, nil)
}

func newRecorder(dir string, cfg Config, identifiers map[string]string, maintain func(Config) error) (*Recorder, error) {
	if err := cfg.Validate(); err != nil {
		return nil, err
	}
	hasCompleted, err := prepareDirectory(dir)
	if err != nil {
		return nil, err
	}
	ids := make(map[string]string)
	for k, v := range identifiers {
		if len(ids) >= 16 {
			break
		}
		if len(k) <= 64 && len(v) <= 256 {
			ids[k] = v
		}
	}
	r := &Recorder{dir: dir, cfg: cfg, identifiers: ids, queue: make(chan observation, queueCapacity),
		permits: make(chan struct{}, queueCapacity), stop: make(chan struct{}), done: make(chan struct{}),
		result: make(chan writeResult, 1), diagnosticErrors: make(map[string]bool), now: time.Now, resources: sampleResources}
	sample := r.resources()
	r.effectiveMemory = initialMemoryBudget(cfg.MemoryBytes, sample)
	r.pressure = sample.known && sample.available < 32<<20
	r.allocateBuffer()
	r.config.Store(&cfg)
	r.write = r.persist
	r.freeDisk = diskFree
	r.removeCompleted = func(root *os.Root, name string) error { return root.Remove(name) }
	r.maintain = func(cfg Config) error { return r.prune(cfg, "") }
	if maintain != nil {
		r.maintain = maintain
	}
	if hasCompleted {
		r.startPrune()
	}
	go r.run()
	return r, nil
}

func (r *Recorder) allocateBuffer() {
	// Fixed slots bound record counts even for empty/high-frequency payloads.
	limit := r.historyLimit()
	n := int(limit / 512)
	if n > 32768 {
		n = 32768
	}
	if n < 1 {
		n = 1
	}
	r.buffer = ring{slots: make([]observation, n)}
}

func (r *Recorder) historyLimit() int64 {
	// Reserve a quarter for fixed slot arrays, bounded trigger state, JSON
	// encoding and writer buffers, separately from retained payload accounting.
	return (r.effectiveMemory - r.effectiveMemory/4 - 65536 - int64(queueCapacity*(maxPayload+256+recordOverhead))) / 3
}

func (r *Recorder) captureLimit() int64 { return 2 * r.historyLimit() }

func (r *Recorder) Ingest(topic string, data []byte) bool {
	cfg := r.config.Load()
	if r.closed.Load() || !cfg.Enabled {
		return false
	}
	if len(topic) == 0 || len(topic) > 256 || len(data) > cfg.MaxMessageBytes {
		r.dropped.Add(1)
		return false
	}
	select {
	case r.permits <- struct{}{}:
	default:
		r.dropped.Add(1)
		return false
	}
	o := observation{at: r.now(), topic: strings.Clone(topic), data: append([]byte(nil), data...)}
	if r.closed.Load() {
		<-r.permits
		r.dropped.Add(1)
		return false
	}
	// Queue capacity equals permits, so this send cannot block. The default
	// remains defensive if an admission check races shutdown.
	select {
	case r.queue <- o:
		return true
	default:
		<-r.permits
		r.dropped.Add(1)
		return false
	}
}

func (r *Recorder) Trigger(reason string) (string, bool) {
	r.mu.Lock()
	defer r.mu.Unlock()
	return r.trigger(reason, r.now())
}

func (r *Recorder) trigger(reason string, now time.Time) (string, bool) {
	if r.closed.Load() || !r.cfg.Enabled {
		return "", false
	}
	if len(reason) > 256 {
		reason = reason[:256]
	}
	if reason == "" {
		reason = "manual"
	}
	if r.active != nil {
		r.coalesced++
		if len(r.active.metadata.Reasons) < maxReasons {
			found := false
			for _, s := range r.active.metadata.Reasons {
				if s == reason {
					found = true
				}
			}
			if !found {
				r.active.metadata.Reasons = append(r.active.metadata.Reasons, reason)
			}
		}
		return r.active.metadata.CaptureID, false
	}
	// Maintenance must not consume incident edges. Pin one bounded RAM capture
	// immediately; its disk write waits until the maintenance worker finishes.
	if (r.writing && !r.pruning) || (!strings.HasPrefix(reason, "manual") && !r.lastTrigger.IsZero() && now.Sub(r.lastTrigger) < time.Duration(r.cfg.CooldownSeconds)*time.Second) {
		r.coalesced++
		return "", false
	}
	var random [8]byte
	if _, err := rand.Read(random[:]); err != nil {
		r.lastError = err.Error()
		return "", false
	}
	id := "blackbox-" + now.UTC().Format("20060102T150405.000000000Z") + "-" + hex.EncodeToString(random[:])
	r.buffer.expire(now.Add(-time.Duration(r.cfg.PreSeconds) * time.Second))
	capacity := min(32768, int(r.captureLimit()/512))
	if r.pressure {
		capacity = 0
	} // metadata-only incident while resources are scarce
	c := &capture{trigger: now, deadline: now.Add(time.Duration(r.cfg.PostSeconds) * time.Second), limit: r.captureLimit(), startDrops: r.dropped.Load(),
		metadata:     Metadata{Kind: "blackbox_metadata", FormatVersion: 1, CaptureID: id, TriggeredAt: now.UTC(), Reasons: []string{reason}, Identifiers: r.identifiers, Config: r.cfg},
		observations: make([]observation, 0, capacity)}
	for i := 0; i < r.buffer.count; i++ {
		r.appendCapture(c, r.buffer.slots[(r.buffer.start+i)%len(r.buffer.slots)])
	}
	r.active = c
	r.lastTrigger = now
	return id, true
}

func (r *Recorder) appendCapture(c *capture, o observation) {
	if c.bytes+o.cost() > c.limit || len(c.observations) == cap(c.observations) {
		c.drops++
		return
	}
	c.observations = append(c.observations, o)
	c.bytes += o.cost()
}

func (r *Recorder) Configure(cfg Config) error {
	return r.ConfigurePersist(cfg, nil)
}

// ConfigurePersist serializes configuration persistence with capture detection.
// A failed save leaves the old configuration and history intact. Ingest remains
// nonblocking even when the settings store is slow (its queue can drop data).
func (r *Recorder) ConfigurePersist(cfg Config, persist func() error) error {
	if err := cfg.Validate(); err != nil {
		return err
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.closed.Load() {
		return errors.New("blackbox is shut down")
	}
	if r.active != nil || r.writing {
		return errors.New("wait for the current blackbox capture to finish")
	}
	if persist != nil {
		if err := persist(); err != nil {
			return err
		}
	}
	previous := r.cfg
	r.cfg = cfg
	if cfg.MemoryBytes != previous.MemoryBytes || cfg.MaxMessageBytes != previous.MaxMessageBytes {
		old := r.buffer
		r.effectiveMemory = initialMemoryBudget(cfg.MemoryBytes, r.resources())
		r.allocateBuffer()
		for i := 0; i < old.count; i++ {
			o := old.slots[(old.start+i)%len(old.slots)]
			if len(o.data) <= cfg.MaxMessageBytes {
				r.buffer.append(o, r.historyLimit())
			}
		}
	}
	r.buffer.expire(r.now().Add(-time.Duration(cfg.PreSeconds) * time.Second))
	if !cfg.Enabled {
		for r.buffer.count > 0 {
			r.buffer.pop()
		}
	}
	// Preserve event edge state across settings edits to avoid capture storms.
	r.config.Store(&cfg)
	if cfg.MaxSnapshots != previous.MaxSnapshots || cfg.MaxDiskBytes != previous.MaxDiskBytes || r.lastError != "" {
		r.startPrune()
	}
	return nil
}

func (r *Recorder) Status() Status {
	r.mu.Lock()
	defer r.mu.Unlock()
	phase := "ready"
	if !r.cfg.Enabled {
		phase = "disabled"
	}
	if r.writing {
		phase = "writing"
	}
	if r.pruning {
		phase = "pruning"
	}
	if r.active != nil {
		phase = "capturing"
	}
	if r.closed.Load() {
		phase = "stopped"
	}
	s := Status{Config: r.cfg, Phase: phase, BufferedRecords: r.buffer.count, BufferedBytes: r.buffer.bytes,
		EffectiveMemoryBytes: r.effectiveMemory, EffectivePreSeconds: r.cfg.PreSeconds, DroppedMessages: r.dropped.Load(),
		CoalescedTriggers: r.coalesced, CompletedSnapshots: r.completed, LastError: r.lastError, MemoryPressure: r.pressure}
	s.HistoryEvictions = r.historyEvictions
	s.SkippedTriggerSources = r.skippedTriggerSources
	s.LastWriteSeconds = r.lastWriteSeconds
	if r.buffer.count > 0 {
		s.BufferedSeconds = max(0, r.now().Sub(r.buffer.slots[r.buffer.start].at).Seconds())
	}
	if r.active != nil {
		s.CaptureID = r.active.metadata.CaptureID
	}
	return s
}

func (r *Recorder) Close(ctx context.Context) error {
	r.stopOnce.Do(func() { r.closed.Store(true); close(r.stop) })
	select {
	case <-r.done:
		return nil
	case <-ctx.Done():
		return ctx.Err()
	}
}

func (r *Recorder) run() {
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()
	defer close(r.done)
	resourceTicks := 0
	for {
		select {
		case o := <-r.queue:
			r.mu.Lock()
			if len(o.data) > r.cfg.MaxMessageBytes {
				r.dropped.Add(1)
				r.mu.Unlock()
				<-r.permits
				continue
			}
			if r.cfg.Enabled && !r.pressure {
				now := r.now()
				r.buffer.expire(now.Add(-time.Duration(r.cfg.PreSeconds) * time.Second))
				if !json.Valid(o.data) {
					r.dropped.Add(1)
					r.mu.Unlock()
					<-r.permits
					continue
				}
				before := r.buffer.count
				if o.at.Before(now.Add(-time.Duration(r.cfg.PreSeconds)*time.Second)) || !r.buffer.append(o, r.historyLimit()) {
					r.dropped.Add(1)
				} else if evicted := before + 1 - r.buffer.count; evicted > 0 {
					r.historyEvictions += uint64(evicted)
				}
				if r.active != nil && !o.at.After(r.active.deadline) {
					r.appendCapture(r.active, o)
				}
				r.detect(o, now)
			} else if r.cfg.Enabled {
				r.dropped.Add(1)
				r.detect(o, r.now())
			}
			r.mu.Unlock()
			<-r.permits
		case <-ticker.C:
			r.mu.Lock()
			now := r.now()
			r.buffer.expire(now.Add(-time.Duration(r.cfg.PreSeconds) * time.Second))
			if r.active != nil && !r.writing && !r.closed.Load() && !now.Before(r.active.deadline) {
				r.finish(now, false)
			}
			resourceTicks++
			if resourceTicks >= 50 {
				resourceTicks = 0
				r.applyPressure(r.resources())
			}
			r.mu.Unlock()
		case result := <-r.result:
			r.mu.Lock()
			r.applyWriteResult(result)
			if r.active != nil && !r.closed.Load() && !r.now().Before(r.active.deadline) {
				r.finish(r.now(), false)
			}
			r.mu.Unlock()
		case <-r.stop:
			r.mu.Lock()
			draining := true
			for draining {
				select {
				case <-r.queue:
					<-r.permits
					r.dropped.Add(1)
				default:
					draining = false
				}
			}
			// A RAM incident may coexist with retention maintenance. Wait for the
			// current worker before starting its interrupted snapshot write.
			if r.writing {
				r.mu.Unlock()
				result := <-r.result
				r.mu.Lock()
				r.applyWriteResult(result)
			}
			if r.active != nil {
				r.finish(r.now(), true)
				r.mu.Unlock()
				result := <-r.result
				r.mu.Lock()
				r.applyWriteResult(result)
			}
			r.mu.Unlock()
			return
		}
	}
}

func (r *Recorder) applyWriteResult(result writeResult) {
	r.writing, r.pruning = false, false
	if !result.pruning {
		r.lastWriteSeconds = result.duration.Seconds()
	}
	var published publishedError
	if !result.pruning && (result.err == nil || errors.As(result.err, &published)) {
		r.completed++
	}
	if result.err != nil {
		r.lastError = result.err.Error()
	} else {
		r.lastError = ""
	}
}

// Called with mu held, or before New publishes the recorder. Retention shares
// the single disk worker with snapshots; settings acceptance does no disk I/O.
func (r *Recorder) startPrune() {
	if r.active != nil || r.writing || r.closed.Load() {
		return
	}
	r.writing, r.pruning = true, true
	cfg := r.cfg
	maintain := r.maintain
	go func() { r.result <- writeResult{err: maintain(cfg), pruning: true} }()
}

func (r *Recorder) finish(now time.Time, interrupted bool) {
	c := r.active
	r.active = nil
	r.writing = true
	c.metadata.Interrupted = interrupted
	c.metadata.Records = len(c.observations)
	c.metadata.DroppedMessages = r.dropped.Load()
	c.metadata.CaptureDroppedMessages = c.drops + r.dropped.Load() - c.startDrops
	c.metadata.WindowElapsedSeconds = max(0, now.Sub(c.trigger).Seconds())
	for _, o := range c.observations {
		if o.at.Before(c.trigger) {
			c.metadata.ActualPreSeconds = max(c.metadata.ActualPreSeconds, c.trigger.Sub(o.at).Seconds())
		} else {
			c.metadata.ActualPostSeconds = max(c.metadata.ActualPostSeconds, o.at.Sub(c.trigger).Seconds())
		}
	}
	go func() {
		start := time.Now()
		err := r.write(c)
		r.result <- writeResult{err: err, duration: time.Since(start)}
	}()
}

func (r *Recorder) detect(o observation, now time.Time) {
	if o.topic == "emergency" || strings.HasSuffix(o.topic, "/emergency") {
		var msg struct {
			Active  bool   `json:"active_emergency"`
			Latched bool   `json:"latched_emergency"`
			Reason  string `json:"reason"`
		}
		if json.Unmarshal(o.data, &msg) == nil {
			active := msg.Active || msg.Latched
			if active && !r.emergency {
				r.trigger(fmt.Sprintf("emergency: active=%t latched=%t: %s", msg.Active, msg.Latched, msg.Reason), now)
			}
			r.emergency = active
		}
	}
	if o.topic == "diagnostics" || o.topic == "fusionDiag" || strings.Contains(o.topic, "diagnostic") {
		var msg struct {
			Status []struct {
				Level      json.Number `json:"level"`
				Name       string      `json:"name"`
				HardwareID string      `json:"hardware_id"`
			} `json:"status"`
		}
		if json.Unmarshal(o.data, &msg) != nil {
			return
		}
		for _, s := range msg.Status {
			if len(s.Name) > 128 || len(s.HardwareID) > 128 {
				r.skippedTriggerSources++
				continue
			}
			key := o.topic + "\x00" + s.Name + "\x00" + s.HardwareID
			// An untracked source cannot provide a reliable edge. Ignore it when
			// the bounded identity table is full instead of retriggering forever.
			if _, exists := r.diagnosticErrors[key]; !exists && len(r.diagnosticErrors) >= 128 {
				r.skippedTriggerSources++
				continue
			}
			level, err := s.Level.Int64()
			if err != nil {
				continue
			}
			active := level >= 2
			if active && !r.diagnosticErrors[key] {
				r.trigger("diagnostic ERROR: "+s.Name, now)
			}
			r.diagnosticErrors[key] = active
		}
	}
}
