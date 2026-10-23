package api

import (
	"context"
	"errors"
	"time"

	"github.com/mowglinext/mowglinext/pkg/msgs/mowgli"
)

// Internal correlation, not authentication. Never include this value in logs or
// diagnostics. A retired browser cannot dispatch a delayed action into a new session.
const gamepadActionHeader = "X-Mowgli-Gamepad-Session"
const gamepadActionTimeout = 10 * time.Second

var errGamepadActionBusy = errors.New("controller action or cleanup is still pending")
var errGamepadActionSession = errors.New("controller action session is no longer current")

type gamepadModeAction struct {
	done        chan struct{}
	changesMode bool
}
type gamepadStopAction struct {
	done chan struct{}
	err  error // published by closing done
}

// Caller holds actionMu. No path takes coordinator.mu while holding actionMu.
func (c *gamepadCoordinator) validateActionLocked(token string, command uint8) error {
	s := c.actionOwner
	if token != "" && (s == nil || s.challenge != token || (command != 3 && command != 5 && command != 6 && command != 8)) {
		return errGamepadActionSession
	}
	if s != nil {
		s.mu.Lock()
		live := !s.ended && time.Now().Before(s.lastValid.Add(gamepadLease))
		ready := s.ready
		s.mu.Unlock()
		if !live || (token != "" && !ready) {
			return errGamepadActionSession
		}
	}
	return nil
}

// Reserve before dispatch, including untagged older GUI calls. Only one mode
// request is admitted; a competing mode is rejected rather than queued. A live,
// idle controller still permits the ordinary Manual Mowing (7) button.
// The caller must finish after its bounded RPC settles. The returned ownership
// flag is captured at admission: only owned calls detach HTTP cancellation.
// Command 0 fences a non-mode operation without replacing a mode obligation.
func (c *gamepadCoordinator) beginMode(token string, command uint8) (func(error), bool, error) {
	// Serialize admission with the velocity writer, then release mu before the
	// service RPC. Frames following admission are neutralized in publish.
	c.mu.Lock()
	defer c.mu.Unlock()
	c.actionMu.Lock()
	defer c.actionMu.Unlock()
	if err := c.validateActionLocked(token, command); err != nil {
		return nil, false, err
	}
	if c.modeAction != nil || c.stopAction != nil {
		return nil, false, errGamepadActionBusy
	}
	action := &gamepadModeAction{done: make(chan struct{}), changesMode: command != 0}
	c.modeAction = action
	s := c.actionOwner
	if s != nil && command == 7 {
		// Status delivery can lag or the request can fail after dispatch. Retain the
		// same conservative manual cleanup obligation in either case.
		s.mu.Lock()
		s.manualOwned = true
		s.modeCommandOwned = true
		s.mu.Unlock()
	}
	return func(err error) {
		c.actionMu.Lock()
		// Failure is not evidence that a dispatched command had no effect. Install
		// compensation before opening admission, including for disconnected clients.
		if s != nil {
			s.mu.Lock()
			ended := s.ended
			// A successful explicit mode selection supersedes our admitted
			// Manual Mowing command. Uncorrelated status callbacks cannot do so.
			if err == nil && command >= 1 && command <= 6 {
				s.modeCommandOwned = true
				s.manualOwned = false
			}
			s.mu.Unlock()
			if action.changesMode && (err != nil || ended) && c.stopAction == nil {
				c.startStopLocked()
			}
		}
		c.modeAction = nil
		close(action.done)
		c.actionMu.Unlock()
	}, s != nil, nil
}

func (c *gamepadCoordinator) requestStop(token string) (*gamepadStopAction, error) {
	c.actionMu.Lock()
	defer c.actionMu.Unlock()
	// Untagged STOP remains an unconditional recovery path, including cleanup.
	if token != "" {
		if err := c.validateActionLocked(token, 8); err != nil {
			return nil, err
		}
	}
	return c.startStopLocked(), nil
}

// Caller holds actionMu. Coalesce STOP, failure compensation, and disconnect
// cleanup into one obligation. The first STOP bypasses the mode RPC; a second
// STOP after its settlement prevents a late start from winning, even if the
// first STOP failed. Ownership/fencing remains held through the final response.
func (c *gamepadCoordinator) startStopLocked() *gamepadStopAction {
	if c.stopAction != nil {
		return c.stopAction
	}
	stop := &gamepadStopAction{done: make(chan struct{})}
	pending := c.modeAction
	c.stopAction = stop
	go func() {
		err := c.callActionStop()
		if pending != nil {
			<-pending.done
			if pending.changesMode {
				err = c.callActionStop()
			}
		}
		c.actionMu.Lock()
		if err == nil && c.actionOwner != nil {
			s := c.actionOwner
			s.mu.Lock()
			s.modeCommandOwned = true
			s.manualOwned = false
			s.mu.Unlock()
		}
		stop.err = err
		c.stopAction = nil
		close(stop.done)
		c.actionMu.Unlock()
	}()
	return stop
}

func (c *gamepadCoordinator) callActionStop() error {
	ctx, cancel := context.WithTimeout(context.Background(), gamepadStopTimeout)
	defer cancel()
	var response mowgli.HighLevelControlRes
	err := c.provider.CallService(ctx, "/behavior_tree_node/high_level_control", &mowgli.HighLevelControlReq{Command: 8}, &response, "mowgli_interfaces/srv/HighLevelControl")
	if err == nil && !response.Success {
		err = errors.New("high_level_control rejected STOP")
	}
	return err
}

func (c *gamepadCoordinator) stopSession(s *gamepadSession) {
	c.actionMu.Lock()
	s.mu.Lock()
	manual := s.manualOwned
	s.mu.Unlock()
	// Preserve a completed recording on disconnect. Only an in-flight mode
	// change can require compensation; progress clearing cannot restart motion.
	pending := c.modeAction
	needed := manual || (pending != nil && pending.changesMode) || c.stopAction != nil
	var stop *gamepadStopAction
	if needed {
		stop = c.startStopLocked()
	}
	c.actionMu.Unlock()
	s.record(gamepadServerEvent{Event: "stopStarted", Skipped: !needed})
	event := gamepadServerEvent{Event: "stopFinished", Skipped: !needed}
	if stop != nil {
		<-stop.done
		success := stop.err == nil
		event.Success = &success
		if stop.err != nil {
			event.Error = stop.err.Error()
		}
	} else if pending != nil {
		// Still retain ownership until non-mode work settles, without discarding
		// an existing recording merely because progress clearing was in flight.
		<-pending.done
	}
	s.record(event)
}
