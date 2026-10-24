package providers

import (
	"bytes"
	"time"

	"github.com/sirupsen/logrus"
)

// Budgets per diagnostics subscriber, not a promise of N seconds of history.
// 128 arrays allow bursts without allocating an unbounded event log; the byte
// budget also bounds large arrays. At most another maxBytes-sized array can be
// in the callback. These limits do not cover provider caches or callback-owned
// copies, and do not imply any upstream delivery guarantee.
const (
	diagnosticQueueMaxMessages = 128
	diagnosticQueueMaxBytes    = 2 * 1024 * 1024
)

// diagnosticQueue is guarded by its RosSubscriber.mtx. No callback, logging or
// wait for queue space may occur while that mutex is held.
type diagnosticQueue struct {
	messages     [diagnosticQueueMaxMessages][]byte
	head         int
	count        int
	bytes        int
	dropped      uint64
	droppedBytes uint64
	reported     uint64
	canceled     uint64
	closed       bool
	done         chan struct{}
}

// DiagnosticQueueStats describes transport knowledge, never hardware severity.
// Counters last for this subscriber's lifetime, including after Close. They are
// server-side only: the existing JSON/MessagePack payloads carry no loss marker.
type DiagnosticQueueStats struct {
	PendingMessages  int
	PendingBytes     int
	DroppedMessages  uint64
	DroppedBytes     uint64
	CanceledMessages uint64
	Closed           bool
}

// DiagnosticQueueStats returns a consistent snapshot, or zero for other topics.
func (r *RosSubscriber) DiagnosticQueueStats() DiagnosticQueueStats {
	r.mtx.Lock()
	defer r.mtx.Unlock()
	q := r.diagnostics
	if q == nil {
		return DiagnosticQueueStats{}
	}
	return DiagnosticQueueStats{
		PendingMessages: q.count, PendingBytes: q.bytes,
		DroppedMessages: q.dropped, DroppedBytes: q.droppedBytes,
		CanceledMessages: q.canceled, Closed: q.closed,
	}
}

func (r *RosSubscriber) publishDiagnostic(msg []byte) {
	if msg == nil {
		return
	}
	r.mtx.Lock()
	q := r.diagnostics
	if q.closed {
		r.mtx.Unlock()
		return
	}
	if q.count == len(q.messages) || len(msg) > diagnosticQueueMaxBytes-q.bytes {
		// Drop newest: accepted arrays keep their order and contents. Even a
		// single oversized array is rejected before allocating a private copy.
		q.dropped++
		q.droppedBytes += uint64(len(msg))
	} else {
		tail := (q.head + q.count) % len(q.messages)
		q.messages[tail] = bytes.Clone(msg)
		q.count++
		q.bytes += len(msg)
	}
	r.mtx.Unlock()
	select {
	case r.wake <- struct{}{}:
	default:
	}
}

func (r *RosSubscriber) closeDiagnostics() {
	r.mtx.Lock()
	defer r.mtx.Unlock()
	q := r.diagnostics
	if q.closed {
		return
	}
	q.closed = true
	q.canceled += uint64(q.count)
	clear(q.messages[:])
	q.head, q.count, q.bytes = 0, 0, 0
	close(r.close)
}

// Logging runs on the subscriber worker, never on the shared fan-out path.
// Counts remain immediately inspectable even while a callback is blocked. A
// log is not a notification to the GUI and cannot attest resynchronization.
func (r *RosSubscriber) reportDiagnosticDrops() {
	r.mtx.Lock()
	q := r.diagnostics
	total, droppedBytes, previous := q.dropped, q.droppedBytes, q.reported
	q.reported = total
	r.mtx.Unlock()
	if total != previous {
		logrus.WithFields(logrus.Fields{
			"topic": r.Topic, "subscriber": r.Id, "transport_loss": true,
			"dropped_messages": total - previous, "dropped_total": total,
			"dropped_bytes_total": droppedBytes,
			"queue_max_messages":  diagnosticQueueMaxMessages,
			"queue_max_bytes":     diagnosticQueueMaxBytes,
		}).Warn("RosSubscriber: diagnostic transport loss; bounded queue rejected incoming messages")
	}
}

func (r *RosSubscriber) runDiagnostics() {
	defer func() {
		r.reportDiagnosticDrops()
		close(r.diagnostics.done)
	}()
	var lastDeliver time.Time
	for {
		select {
		case <-r.close:
			return
		case <-r.wake:
		}
		for {
			r.reportDiagnosticDrops()
			// Respect an explicit caller interval without coalescing its queue.
			// Current diagnostics WebSocket routes use a negative interval, so
			// no batch delay or severity-based reordering is introduced.
			if r.interval > 0 {
				if wait := r.interval - time.Since(lastDeliver); wait > 0 {
					timer := time.NewTimer(wait)
					select {
					case <-r.close:
						timer.Stop()
						return
					case <-timer.C:
					}
				}
			}
			r.mtx.Lock()
			q := r.diagnostics
			if q.closed || q.count == 0 {
				closed := q.closed
				r.mtx.Unlock()
				if closed {
					return
				}
				break
			}
			msg := q.messages[q.head]
			q.messages[q.head] = nil
			q.head = (q.head + 1) % len(q.messages)
			q.count--
			q.bytes -= len(msg)
			r.mtx.Unlock()
			r.cb(msg)
			lastDeliver = time.Now()
		}
	}
}
