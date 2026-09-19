package api

import (
	"context"
	"strconv"
	"sync"
	"time"

	"github.com/mowglinext/mowglinext/pkg/types"
)

// Snapshot identity and counters belong to the receiver, not the timer that
// republishes its status. A health-service response alone only proves that the
// node is responsive; it does not prove that the serial receiver is connected.
type receiverSnapshot struct {
	Status struct {
		SourceID    string `json:"source_id"`
		Incarnation string `json:"source_incarnation"`
	} `json:"status"`
	Diagnostics struct {
		Status []struct {
			Name       string `json:"name"`
			HardwareID string `json:"hardware_id"`
			Values     []struct {
				Key   string `json:"key"`
				Value string `json:"value"`
			} `json:"values"`
		} `json:"status"`
	} `json:"diagnostics"`
}

type receiverProgress struct {
	source, incarnation string
	count               uint64
	advancedAt          time.Time
}

func (p *receiverProgress) observe(s receiverSnapshot, now time.Time) (bool, string) {
	values := map[string]map[string]string{}
	for _, status := range s.Diagnostics.Status {
		if status.HardwareID != s.Status.SourceID {
			continue
		}
		if _, duplicate := values[status.Name]; duplicate {
			*p = receiverProgress{}
			return false, "GNSS receiver diagnostics have ambiguous ownership"
		}
		v := map[string]string{}
		for _, item := range status.Values {
			v[item.Key] = item.Value
		}
		values[status.Name] = v
	}
	health := values["universal_gnss/summary"]
	count, err := strconv.ParseUint(values["universal_gnss/parser_counters"]["runtime_observations"], 10, 64)
	if s.Status.SourceID == "" || s.Status.Incarnation == "" || err != nil {
		*p = receiverProgress{}
		return false, "GNSS receiver does not provide live observation identity"
	}
	if health["transport_healthy"] != "true" || health["parser_healthy"] != "true" {
		*p = receiverProgress{}
		return false, "GNSS receiver transport or parser is unhealthy"
	}
	if p.source != s.Status.SourceID || p.incarnation != s.Status.Incarnation || count < p.count {
		*p = receiverProgress{source: s.Status.SourceID, incarnation: s.Status.Incarnation, count: count}
		return false, "Waiting for new GNSS receiver observations"
	}
	if count > p.count {
		p.count = count
		p.advancedAt = now
	}
	// now is a local monotonic receipt time. Snapshot/position ROS timestamps
	// cannot measure how long this receiver counter has stopped advancing.
	if p.advancedAt.IsZero() || now.Sub(p.advancedAt) >= 3*time.Second {
		return false, "GNSS receiver is responding but has no new observations"
	}
	return true, ""
}

type receiverProbe struct {
	mu       sync.Mutex
	progress receiverProgress
}

func (p *receiverProbe) check(ctx context.Context, ros types.IRosProvider) (bool, string) {
	p.mu.Lock()
	defer p.mu.Unlock()
	ctx, cancel := context.WithTimeout(ctx, 2*time.Second)
	defer cancel()
	var snapshot receiverSnapshot
	if err := ros.CallService(ctx, "/universal_gnss_receiver/get_snapshot", struct{}{}, &snapshot,
		"universal_gnss_msgs/srv/GetReceiverSnapshot"); err != nil {
		p.progress = receiverProgress{}
		return false, "GNSS receiver snapshot service is unavailable"
	}
	return p.progress.observe(snapshot, time.Now())
}
