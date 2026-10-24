package api

import (
	"context"
	"errors"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/geometry"
	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
)

// Physical intent is distinct from the interlocked velocity. In particular a
// browser awaiting a mode response still reports a held physical deadman.
type gamepadPhysicalInput struct {
	PhysicalDeadman bool   `json:"physicalDeadman"`
	Neutral         bool   `json:"neutral"`
	Epoch           uint64 `json:"epoch"`
}

// Caller holds s.mu. Notices are coalesced snapshots, never queued permissions.
func (s *gamepadSession) setManualPhaseLocked(phase string) {
	s.manualPhase = phase
	s.manualResumed = false
	s.controlEpoch++
	s.resumeReleased = false
	s.resumeSucceeded = false
	if s.notices != nil {
		select {
		case s.notices <- struct{}{}:
		default:
		}
	}
}

func (s *gamepadSession) revokeManualLocked() {
	// STOP/status revocation also reaches an input already queued at either
	// provider's transport writer; a local phase flag alone cannot fence it.
	if s.cancelPublish != nil {
		s.cancelPublish()
	}
	s.manualBlocked = true
	s.genericManualPending = false
	s.setManualPhaseLocked("inactive")
}

func (s *gamepadSession) writeNotices() {
	for range s.notices {
		s.writeMu.Lock()
		s.mu.Lock()
		if s.ended {
			s.mu.Unlock()
			s.writeMu.Unlock()
			return
		}
		notice := struct {
			Type    string `json:"type"`
			Epoch   uint64 `json:"epoch"`
			Phase   string `json:"phase"`
			Resumed bool   `json:"resumed,omitempty"`
		}{"gamepadControl", s.controlEpoch, s.manualPhase, s.manualResumed}
		s.mu.Unlock()
		err := writeGamepadHandshake(s.conn, notice)
		s.writeMu.Unlock()
		if err != nil {
			s.terminate(gamepadServerEvent{Cause: "controlWriteFailed"})
			return
		}
	}
}

func (s *gamepadSession) observeStatus(status mowgli.HighLevelStatus) {
	c := s.coordinator
	c.actionMu.Lock()
	s.mu.Lock()
	if s.ended {
		s.mu.Unlock()
		c.actionMu.Unlock()
		return
	}
	s.state = status.State
	s.statusAt = time.Now()
	s.statusSeq++
	if !s.modeCommandOwned {
		if status.State == 4 {
			s.manualOwned = true
		} else if status.State == 3 {
			s.manualOwned = false
		}
	}
	idle := status.State == 1 && status.StateName == "IDLE" && !status.Emergency
	wasResuming := s.manualPhase == "resuming"
	if status.Emergency {
		// Also retire a successful ordinary GUI7 still awaiting its first
		// MANUAL status. A later emergency-clear callback cannot confirm it.
		s.revokeManualLocked()
	} else if s.genericManualPending {
		if status.State == 4 {
			s.genericManualPending = false
			s.manualBlocked = false
			s.setManualPhaseLocked("active")
		} else if !idle {
			s.revokeManualLocked()
		}
	}
	switch s.manualPhase {
	case "pausing":
		if s.pauseStopSucceeded && idle && s.statusSeq > s.resumeAfter {
			s.setManualPhaseLocked("paused")
		} else if status.Emergency || (status.State != 4 && !idle) {
			s.revokeManualLocked()
		}
	case "paused":
		if !idle {
			s.revokeManualLocked()
		}
	case "resuming":
		if status.Emergency || (status.State != 4 && !idle) {
			s.revokeManualLocked()
		} else if s.resumeSucceeded && status.State == 4 && s.statusSeq > s.resumeAfter {
			// Confirmation must find the same fresh, held, centered physical input.
			if s.deadman && s.neutral && time.Now().Before(s.lastValid.Add(gamepadLease)) {
				s.manualBlocked = false
				s.setManualPhaseLocked("active")
				s.manualResumed = true
			} else {
				s.revokeManualLocked()
			}
		}
	case "active":
		if status.State != 4 || status.Emergency {
			s.revokeManualLocked()
		}
	}
	// A failed confirmation or emergency cannot leave an owned start running.
	stop := (status.Emergency && s.manualOwned) || (wasResuming && s.manualPhase == "inactive")
	s.mu.Unlock()
	if stop {
		c.startStopLocked()
	}
	c.actionMu.Unlock()
}

func (s *gamepadSession) processInput(msg *geometry.TwistStamped, effective bool, input gamepadPhysicalInput) (uint64, bool) {
	frame, accepted := s.acceptFrame(input.PhysicalDeadman)
	if !accepted {
		return 0, false
	}
	c := s.coordinator
	c.mu.Lock()
	c.actionMu.Lock()
	s.mu.Lock()
	s.neutral = input.Neutral
	expiredStatus := !s.statusAt.IsZero() && time.Since(s.statusAt) >= 2500*time.Millisecond
	if s.ended || expiredStatus {
		s.mu.Unlock()
		c.actionMu.Unlock()
		c.mu.Unlock()
		if expiredStatus {
			s.terminate(gamepadServerEvent{Cause: "statusLost"})
		}
		return 0, false
	}
	phase := s.manualPhase
	epochValid := input.Epoch == s.controlEpoch
	// The physical falling edge always takes priority, even when the browser's
	// effective deadman was already false during a pending start or interlock.
	if frame.released && (s.manualOwned || phase == "resuming") {
		canPause := s.manualOwned && !s.manualBlocked && c.modeAction == nil && c.stopAction == nil && phase == "active"
		if canPause {
			s.manualBlocked = true
			s.pauseStopSucceeded = false
			s.setManualPhaseLocked("pausing")
			epoch := s.controlEpoch
			stop := c.startStopLocked()
			go s.pauseStopped(stop, epoch)
		} else {
			s.revokeManualLocked()
			c.startStopLocked()
		}
		*msg = geometry.TwistStamped{}
	} else if phase == "paused" && input.Epoch == s.controlEpoch {
		if !input.PhysicalDeadman && input.Neutral {
			s.resumeReleased = true
		}
		if !input.Neutral {
			s.resumeReleased = false
		}
	} else if phase == "inactive" && epochValid && s.state == 4 && !s.manualBlocked && effective && input.PhysicalDeadman {
		s.setManualPhaseLocked("active")
	}
	// Old epochs may renew the connection lease but cannot carry motion or intent.
	// Initial epoch zero is allowed for the poll that establishes active state.
	invalidMode := s.state != 3 && s.state != 4
	if invalidMode || !epochValid || (s.state == 4 && s.manualBlocked) || s.manualPhase == "pausing" || s.manualPhase == "paused" || s.manualPhase == "resuming" {
		*msg = geometry.TwistStamped{}
	}
	resume := phase == "paused" && input.Epoch == s.controlEpoch && frame.rising && s.resumeReleased && input.Neutral
	if phase == "paused" && frame.rising {
		s.resumeReleased = false
	}
	modeRejected := effective && epochValid && !s.manualBlocked && invalidMode && phase != "pausing" && phase != "paused" && phase != "resuming"
	processedEpoch := s.controlEpoch
	s.mu.Unlock()
	c.actionMu.Unlock()
	c.mu.Unlock()
	if modeRejected {
		s.terminate(gamepadServerEvent{Cause: "modeRejected"})
		return 0, false
	}
	if resume {
		c.resumeManual(s, input.Epoch)
	}
	return processedEpoch, true
}

func (s *gamepadSession) pauseStopped(stop *gamepadStopAction, epoch uint64) {
	<-stop.done
	s.mu.Lock()
	defer s.mu.Unlock()
	if s.ended || s.controlEpoch != epoch || s.manualPhase != "pausing" {
		return
	}
	if stop.err != nil {
		s.revokeManualLocked()
		return
	}
	s.pauseStopSucceeded = true
	// Cached or pre-response IDLE does not authorize a new manual command.
	s.resumeAfter = s.statusSeq
}

// Admission follows the same coordinator lock order and global mode/STOP fence
// as HTTP actions. No RPC or wait runs on the input reader.
func (c *gamepadCoordinator) resumeManual(s *gamepadSession, epoch uint64) {
	c.mu.Lock()
	c.actionMu.Lock()
	s.mu.Lock()
	if c.owner != s || s.ended || s.controlEpoch != epoch || s.manualPhase != "paused" || !s.deadman || !s.neutral || c.modeAction != nil || c.stopAction != nil {
		s.mu.Unlock()
		c.actionMu.Unlock()
		c.mu.Unlock()
		return
	}
	action := &gamepadModeAction{done: make(chan struct{}), changesMode: true, resume: true, command: 7}
	c.modeAction = action
	s.manualOwned, s.modeCommandOwned = true, true
	s.setManualPhaseLocked("resuming")
	action.epoch = s.controlEpoch
	s.mu.Unlock()
	c.actionMu.Unlock()
	c.mu.Unlock()
	go func() {
		ctx, cancel := context.WithTimeout(context.Background(), gamepadActionTimeout)
		defer cancel()
		var response mowgli.HighLevelControlRes
		err := c.provider.CallService(ctx, "/behavior_tree_node/high_level_control", &mowgli.HighLevelControlReq{Command: 7}, &response, "mowgli_interfaces/srv/HighLevelControl")
		if err == nil && !response.Success {
			err = errors.New("high_level_control rejected manual resume")
		}
		c.finishMode(action, s, 7, err)
	}()
}

func (c *gamepadCoordinator) revokeManualIntent() {
	c.actionMu.Lock()
	defer c.actionMu.Unlock()
	if s := c.actionOwner; s != nil {
		s.mu.Lock()
		resuming := s.manualPhase == "resuming" || (c.modeAction != nil && c.modeAction.command == 7)
		s.revokeManualLocked()
		s.mu.Unlock()
		if resuming {
			c.startStopLocked()
		}
	}
}
