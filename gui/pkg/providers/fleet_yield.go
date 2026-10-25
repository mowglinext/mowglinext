package providers

import (
	"context"
	"fmt"
	"math"
	"time"

	"github.com/docker/distribution/uuid"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
)

const (
	fleetYieldService = "/behavior_tree_node/fleet_yield"
	fleetYieldType    = "mowgli_interfaces/srv/FleetYield"
)

const (
	fleetQuery uint8 = iota
	fleetPause
	fleetResume
	fleetCancel
)

const (
	fleetAvailable uint8 = iota
	fleetHeld
	fleetResumed
	fleetAlreadyResumed
	fleetCancelled
	fleetNotReady
	fleetIneligible
	fleetBusy
	fleetRevoked
	fleetInvalid
)

const yieldResumeHold = 3 * time.Second

// Ownership is process-local. Never persist or adopt a tuple from QUERY.
// Operation remains unchanged after a lost reply, except that cancellation
// supersedes every other intent and remains pending even across re-enable.
type yieldState struct {
	Yielded          bool
	Epoch            string
	RequestID        string
	Operation        uint8
	Pending          bool
	MayHaveResumed   bool // a RESUME transport failure may have applied
	StopHoldObserved bool // final numeric IDLE has been observed for this hold
	HoldSince        time.Time
	ClearSince       time.Time
	ResumedAt        time.Time
	NextAttempt      time.Time
	Blocked          bool // terminal result: wait for a new proximity/autonomy event
	SawInactive      bool
}

func yieldEligible(self fleetMember) bool {
	if !self.Online || !self.Autonomous {
		return false
	}
	switch self.StateName {
	case "MOWING", "PLANNING", "TRANSIT":
		return true
	default:
		return false
	}
}

func nearestPriorityPeer(self fleetMember, peers []fleetMember) float64 {
	nearest := math.Inf(1)
	if self.Pos == nil {
		return nearest
	}
	for _, p := range peers {
		if !p.Online || !p.Autonomous || p.Pos == nil || !hasPriorityOver(p.ID, self.ID) {
			continue
		}
		nearest = math.Min(nearest, math.Hypot(p.Pos.X-self.Pos.X, p.Pos.Y-self.Pos.Y))
	}
	return nearest
}

// Clearance and max-hold are checked again even for an ambiguous RESUME.
// A retry can be the first call to reach the server, so a returning peer must
// still delay it. The tuple stays intact until the outcome is reconciled.
func resumeDue(st *yieldState, self fleetMember, peers []fleetMember, s CoordinatorSettings, now time.Time) bool {
	if self.Pos == nil {
		return false
	}
	if !st.HoldSince.IsZero() && now.Sub(st.HoldSince) >= time.Duration(s.YieldMaxHoldS*float64(time.Second)) {
		return true
	}
	if nearestPriorityPeer(self, peers) < s.ResumeDistanceM {
		st.ClearSince = time.Time{}
		return false
	}
	if st.ClearSince.IsZero() {
		st.ClearSince = now
	}
	return now.Sub(st.ClearSince) >= yieldResumeHold
}

func (c *FleetCoordinator) stepYield(self fleetMember, peers []fleetMember, s CoordinatorSettings, now time.Time) {
	c.mu.Lock()
	st := c.yield
	c.mu.Unlock()
	defer func() { c.mu.Lock(); c.yield = st; c.mu.Unlock() }()

	if st.Yielded {
		// Eligible autonomy can be the pre-PAUSE cached status, or the result
		// of a lost RESUME reply. Only the server can distinguish those from
		// a newer operator START. PublishHighLevelStatus also retains numeric
		// AUTONOMOUS for the first two IDLE ticks. Tolerate that transition
		// only before observing the final stop hold, as the server does; it
		// does not establish readiness for RESUME.
		debouncingStop := !st.StopHoldObserved && self.Online && self.Autonomous && self.StateName == "IDLE"
		if self.StopHold {
			st.StopHoldObserved = true
		}
		if !s.Enabled || (!self.StopHold && !yieldEligible(self) && !debouncingStop) {
			st.Operation, st.Pending = fleetCancel, true
		}
		// Reconcile an uncertain PAUSE before attempting RESUME.
		if st.Operation != fleetCancel && !(st.Pending && st.Operation == fleetPause) {
			if !resumeDue(&st, self, peers, s, now) {
				return
			}
			st.Operation, st.Pending = fleetResume, true
		}
	} else {
		if !s.Enabled {
			return
		}
		eligible := yieldEligible(self)
		nearest := nearestPriorityPeer(self, peers)
		if st.Blocked {
			if self.Online && self.StateName != "" && !eligible {
				st.SawInactive = true
			}
			if (eligible && st.SawInactive) || (self.Pos != nil && nearest >= s.ResumeDistanceM) {
				st.Blocked, st.SawInactive = false, false
			} else {
				return
			}
		}
		if !eligible || self.Pos == nil || nearest >= s.YieldDistanceM ||
			(!st.ResumedAt.IsZero() && now.Sub(st.ResumedAt) < time.Duration(s.YieldCooldownS*float64(time.Second))) {
			return
		}
		res, ok := c.callFleetYield(mowgli.FleetYieldReq{Operation: fleetQuery})
		if !ok {
			return
		}
		if res.Result != fleetAvailable || res.Held || res.Epoch == "" {
			st.Blocked = true
			c.setError("fleet_yield: no unowned hold available; " + res.Message)
			return
		}
		st = yieldState{Yielded: true, Epoch: res.Epoch, RequestID: uuid.Generate().String(), Operation: fleetPause, Pending: true, HoldSince: now, ResumedAt: st.ResumedAt}
	}
	if now.Before(st.NextAttempt) {
		return
	}
	st.NextAttempt = now.Add(coordinatorTick)
	// Responses carry the CURRENT server epoch. It must never replace the
	// original request epoch, including after transport failure or revocation.
	req := mowgli.FleetYieldReq{Operation: st.Operation, Epoch: st.Epoch, RequestId: st.RequestID}
	c.mu.Lock()
	c.yield = st
	c.mu.Unlock()
	res, ok := c.callFleetYield(req)
	if !ok {
		if st.Operation == fleetResume {
			st.MayHaveResumed = true
		}
		return
	}
	switch {
	case st.Operation == fleetPause && res.Result == fleetHeld:
		st.Pending = false
	case st.Operation == fleetResume && (res.Result == fleetResumed || res.Result == fleetAlreadyResumed):
		st = yieldState{ResumedAt: now}
	case st.Operation == fleetResume && res.Result == fleetNotReady:
		st.MayHaveResumed = false
		c.setError("fleet_yield: waiting for STOP hold; " + res.Message)
	default:
		// All other results are terminal. In particular INELIGIBLE/BUSY do
		// not acquire a new epoch on the next stale MOWING snapshot.
		resumedAt := st.ResumedAt
		// Cancellation/revocation cannot prove an earlier lost RESUME did
		// not apply. Give that possible resume a full cooldown as well.
		if st.MayHaveResumed {
			resumedAt = now
		}
		st = yieldState{ResumedAt: resumedAt, Blocked: true}
		if req.Operation != fleetCancel || (res.Result != fleetCancelled && res.Result != fleetRevoked) {
			c.setError(fmt.Sprintf("fleet_yield: terminal result %d; %s", res.Result, res.Message))
		}
	}
}

func (c *FleetCoordinator) callFleetYield(req mowgli.FleetYieldReq) (mowgli.FleetYieldRes, bool) {
	ctx, cancel := context.WithTimeout(context.Background(), coordinatorCallTimeout)
	defer cancel()
	var res mowgli.FleetYieldRes
	if err := c.ros.CallService(ctx, fleetYieldService, &req, &res, fleetYieldType); err != nil {
		c.setError("fleet_yield unavailable or call failed (requires compatible ROS stack): " + err.Error())
		return res, false
	}
	c.mu.Lock()
	c.lastError = ""
	c.mu.Unlock()
	return res, true
}
